#include "main.h"
#include "usart.h"
#include "bsp_usart.h"
#include "usbd_cdc_if.h"
#include "auto_aim.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"

#include "usb_common.h"
#include "uproto.h"

#include "arm_math.h"

#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "referee.h"
#include "INS_task.h"
#include "shoot.h"

//#include "voltage_task.h"
#include "CRC8_CRC16.h"
#include "GRAVITY.h"
#include "tim.h"

extern uproto_context_t proto_ctx;
// [SYNC_FROM_H] Synced auto-aim control logic from H:\DM-balanceV1\User\APP

// global auto-aim state
auto_aim_t aim;

fp32 yaw = 0.0f;
fp32 CompensationAngle = 0.0f;

// ---------------- Auto-aim internal control state (MCU-side trajectory shaping) ----------------
// [SYNC_FROM_H] New trajectory shaping state replaces direct motor writes
typedef struct
{
    float err_rad;        // latest vision error (rad)
    float omega_cmd_rad;  // commanded angular velocity (rad/s)
} auto_aim_axis_ctrl_t;

typedef struct
{
    auto_aim_axis_ctrl_t yaw_axis;
    auto_aim_axis_ctrl_t pitch_axis;
    uint32_t             last_tick_ms;
} auto_aim_ctrl_t;

static auto_aim_ctrl_t s_auto_aim_ctrl = {0};

// basic tuning (can be adjusted during testing)
// Kv: position error -> target angular velocity (rad/s per rad)
#define AA_KV_YAW          (2.5f)
#define AA_KV_PITCH        (2.5f)
// Kd: derivative gain on error，用于在高 Kv 下增加阻尼
#define AA_KD_YAW          (0.5f)
#define AA_KD_PITCH        (0.5f)
// 非线性区间：误差小于该角度时自动降低 Kv、加大阻尼
#define AA_ERR_SOFT_ZONE_RAD   (3.0f * PI / 180.0f)       // ~5 deg
// D 项一阶低通滤波系数（0~1，越小越平滑）
#define AA_D_LPF_ALPHA     (0.7f)

#define AA_OMEGA_MAX_YAW   (180.0f * PI / 180.0f)         // 180 deg/s
#define AA_OMEGA_MAX_PITCH (180.0f * PI / 180.0f)
#define AA_A_MAX_YAW       (720.0f * PI / 180.0f)         // 720 deg/s^2
#define AA_A_MAX_PITCH     (720.0f * PI / 180.0f)
#define AA_EPS_TH_RAD      (0.5f * PI / 180.0f)           // ~0.5 deg
#define AA_EPS_OMEGA_RAD   (3.0f * PI / 180.0f)           // ~3 deg/s

// internal derivative state for PD-like shaping
static float s_last_err_yaw = 0.0f;
static float s_last_err_pitch = 0.0f;
static float s_err_d_yaw = 0.0f;
static float s_err_d_pitch = 0.0f;

static inline float aa_clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void auto_aim_control_tick_internal(auto_aim_t *aim_loop);

// Legacy example function (kept for compatibility, currently unused in main loop)
void auto_aim_loop(auto_aim_t* aim_loop)
{
    (void)aim_loop;
}

void auto_aim_init(auto_aim_t *aim_init)
{
    if (aim_init == NULL)
    {
        return;
    }

    aim_init->auto_aim_flag = AIM_ON;
    aim_init->last_fdb = 0;
    aim_init->online = 1;

    aim_init->shoot_delay = 0;
    aim_init->yaw_delay = 0;
    aim_init->pitch_delay = 0;

    aim_init->receive.distance = 0.0f;
    aim_init->receive.pitch = 0.0f;
    aim_init->receive.shoot_delay = 0.0f;
    aim_init->receive.yaw = 0.0f;

    aim_init->aim_rc = get_remote_control_point();
}

void auto_aim_set(auto_aim_t *aim_set)
{
    if (aim_set == NULL)
    {
        return;
    }

    static uint32_t press_r_time = 0;

#if ROBOT_MODE == release
    // timeout check: if host not updated for a while, mark offline and disable auto-aim
    if (HAL_GetTick() - aim_set->last_fdb > AUTO_AIM_TIMEOUT)
    {
        aim_set->online = 0;
        aim_set->auto_aim_flag = AIM_OFF;
        aim_set->receive.yaw = 0.0f;
        aim_set->receive.pitch = 0.0f;
        return;
    }
#endif

    // right mouse press toggles auto-aim
    if (aim_set->aim_rc && aim_set->aim_rc->mouse.press_r)
    {
        press_r_time += AUTO_AIM_TIME;
    }
    else
    {
        press_r_time = 0;
    }

    if (press_r_time > PRESS_TIME)
    {
        aim_set->auto_aim_flag = (aim_set->auto_aim_flag == AIM_OFF) ? AIM_ON : AIM_OFF;
    }
}

void auto_aim_feedback_update(auto_aim_t *aim_update)
{
    if (aim_update == NULL)
    {
        return;
    }

    if (aim_update->auto_aim_flag == AIM_OFF)
    {
        return;
    }

    uint8_t id = get_robot_id();
    (void)id;

    uproto_tick(&proto_ctx);

    // clamp raw receive values to configured bounds
    if (aim_update->receive.yaw > MAX_YAW)   aim_update->receive.yaw = MAX_YAW;
    if (aim_update->receive.yaw < MIN_YAW)   aim_update->receive.yaw = MIN_YAW;

    if (aim_update->receive.pitch > MAX_PITCH) aim_update->receive.pitch = MAX_PITCH;
    if (aim_update->receive.pitch < MIN_PITCH) aim_update->receive.pitch = MIN_PITCH;

    aim_update->shoot_delay = 0;
    aim_update->yaw_delay = 0;
    aim_update->pitch_delay = 0;
}

void auto_aim_task(void const *pvParameters)
{
    (void)pvParameters;

    osDelay(AIM_INIT_TIME);
    auto_aim_init(&aim);

    while (1)
    {
        // [SYNC_FROM_H] Task loop now drives internal control tick each cycle
        auto_aim_set(&aim);
        auto_aim_feedback_update(&aim);
        auto_aim_control_tick_internal(&aim);
        osDelay(AUTO_AIM_TIME);
    }
}

static float dyaw_rad;
static float dpitch_rad;

void auto_aim_apply_delta_udeg(int32_t dyaw_udeg,
                               int32_t dpitch_udeg,
                               uint16_t status,
                               uint64_t ts_us)
{
    (void)status;
    (void)ts_us;

    // micro-degree -> rad
    // float dyaw_rad   = ((float)dyaw_udeg)   * PI / 180000000.0f;
    // float dpitch_rad = ((float)dpitch_udeg) * PI / 180000000.0f;
	
		dyaw_rad   = ((float)dyaw_udeg)   * PI / 180000000.0f;
		dpitch_rad = ((float)dpitch_udeg) * PI / 180000000.0f;

    // Interpret host command directly as “需要转动的误差”（正误差 → 正向转动）
    // [SYNC_FROM_H] Host deltas now feed internal controller instead of direct motor commands
    s_auto_aim_ctrl.yaw_axis.err_rad   = dyaw_rad;
    s_auto_aim_ctrl.pitch_axis.err_rad = dpitch_rad;

    // keep auto-aim online / alive
    aim.last_fdb = HAL_GetTick();
}

void auto_aim_control_tick(auto_aim_t *aim_loop)
{
    auto_aim_control_tick_internal(aim_loop);
}

// Core MCU-side control: error -> limited velocity -> per-cycle angle increment (rad)
// [SYNC_FROM_H] New PD-like shaping with slew limits for smooth motion
static void auto_aim_control_tick_internal(auto_aim_t *aim_loop)
{
    if (aim_loop == NULL)
    {
        return;
    }

    uint32_t now_ms = HAL_GetTick();
    float dt_s;
    if (s_auto_aim_ctrl.last_tick_ms == 0U)
    {
        dt_s = (float)AUTO_AIM_TIME / 1000.0f;
    }
    else
    {
        uint32_t diff_ms = now_ms - s_auto_aim_ctrl.last_tick_ms;
        dt_s = (float)diff_ms / 1000.0f;
    }
    s_auto_aim_ctrl.last_tick_ms = now_ms;
    dt_s = aa_clamp(dt_s, 0.0005f, 0.02f); // 0.5–20 ms safety clamp

    // When auto-aim is off or offline, do not inject motion
    if (aim_loop->auto_aim_flag == AIM_OFF || !aim_loop->online)
    {
        s_auto_aim_ctrl.yaw_axis.omega_cmd_rad   = 0.0f;
        s_auto_aim_ctrl.pitch_axis.omega_cmd_rad = 0.0f;
        aim.receive.yaw   = 0.0f;
        aim.receive.pitch = 0.0f;
        return;
    }

    // Yaw axis: error -> omega_ref -> omega_cmd with accel limit -> angle increment
    {
        float err = s_auto_aim_ctrl.yaw_axis.err_rad;
        float e_abs = fabsf(err);

        // 非线性 Kv：远处用大增益，接近目标自动降低增益以减小冲击与振荡
        float kv = AA_KV_YAW;
        if (e_abs < AA_ERR_SOFT_ZONE_RAD)
        {
            float scale = e_abs / (AA_ERR_SOFT_ZONE_RAD + 1e-6f); // 0~1
            // Kv 在 [0.5, 1.0] * AA_KV_YAW 之间线性变化
            kv *= (0.5f + 0.5f * scale);
        }

        // 误差导数（简单 D），并做一阶低通
        float derr = (err - s_last_err_yaw) / dt_s;
        s_last_err_yaw = err;
        s_err_d_yaw = AA_D_LPF_ALPHA * derr + (1.0f - AA_D_LPF_ALPHA) * s_err_d_yaw;

        float omega_ref = kv * err + AA_KD_YAW * s_err_d_yaw;
        omega_ref = aa_clamp(omega_ref, -AA_OMEGA_MAX_YAW, AA_OMEGA_MAX_YAW);

        // 误差越小，加速度上限越小 → 目标附近更平滑
        float a_max = AA_A_MAX_YAW;
        const float soft_a_err = 10.0f * PI / 180.0f; // ~10deg
        if (e_abs < soft_a_err)
        {
            float scale = e_abs / (soft_a_err + 1e-3f); // 0~1
            a_max *= (0.3f + 0.7f * scale);             // 保留至少 30% 的加速度能力
        }
        float domega_max = a_max * dt_s;
        float domega = omega_ref - s_auto_aim_ctrl.yaw_axis.omega_cmd_rad;
        domega = aa_clamp(domega, -domega_max, domega_max);
        s_auto_aim_ctrl.yaw_axis.omega_cmd_rad += domega;

        if ((fabsf(err) < AA_EPS_TH_RAD) &&
            (fabsf(s_auto_aim_ctrl.yaw_axis.omega_cmd_rad) < AA_EPS_OMEGA_RAD))
        {
            s_auto_aim_ctrl.yaw_axis.omega_cmd_rad = 0.0f;
        }

        float dtheta = s_auto_aim_ctrl.yaw_axis.omega_cmd_rad * dt_s;
        aim.receive.yaw = dtheta;
    }

    // Pitch axis
    {
        float err = s_auto_aim_ctrl.pitch_axis.err_rad;
        float e_abs = fabsf(err);

        float kv = AA_KV_PITCH;
        if (e_abs < AA_ERR_SOFT_ZONE_RAD)
        {
            float scale = e_abs / (AA_ERR_SOFT_ZONE_RAD + 1e-6f);
            kv *= (0.5f + 0.5f * scale);
        }

        float derr = (err - s_last_err_pitch) / dt_s;
        s_last_err_pitch = err;
        s_err_d_pitch = AA_D_LPF_ALPHA * derr + (1.0f - AA_D_LPF_ALPHA) * s_err_d_pitch;

        float omega_ref = kv * err + AA_KD_PITCH * s_err_d_pitch;
        omega_ref = aa_clamp(omega_ref, -AA_OMEGA_MAX_PITCH, AA_OMEGA_MAX_PITCH);

        float a_max = AA_A_MAX_PITCH;
        const float soft_a_err = 10.0f * PI / 180.0f;
        if (e_abs < soft_a_err)
        {
            float scale = e_abs / (soft_a_err + 1e-3f);
            a_max *= (0.3f + 0.7f * scale);
        }
        float domega_max = a_max * dt_s;
        float domega = omega_ref - s_auto_aim_ctrl.pitch_axis.omega_cmd_rad;
        domega = aa_clamp(domega, -domega_max, domega_max);
        s_auto_aim_ctrl.pitch_axis.omega_cmd_rad += domega;

        if ((fabsf(err) < AA_EPS_TH_RAD) &&
            (fabsf(s_auto_aim_ctrl.pitch_axis.omega_cmd_rad) < AA_EPS_OMEGA_RAD))
        {
            s_auto_aim_ctrl.pitch_axis.omega_cmd_rad = 0.0f;
        }

        float dtheta = s_auto_aim_ctrl.pitch_axis.omega_cmd_rad * dt_s;
        aim.receive.pitch = dtheta;
    }
}
