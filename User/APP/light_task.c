#include "light_task.h"

#include <string.h>

#include "bsp_usart.h"
#include "auto_aim.h"
#include "chassis_behaviour.h"
#include "chassis_power_control.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"
#include "robot_param.h"
#include "shoot.h"

#if ROBOT_GIMBAL == multi_axis_robotic_arm
#include "arm_teach.h"
#endif

/* 灯位状态定义：0=DBUS 在线，1=底盘电机在线，2=摩擦轮在线，3=自瞄在线，4~6=底盘行为，7=云台行为，8=超级电容，9=任务心跳。 */



/* 灯板串口帧格式：帧头 0xAA 0x55，帧尾 0x55 0xAA，中间是 10 路灯的 RGB 数据。 */
#define LIGHT_FRAME_HEAD0 0xAAU
#define LIGHT_FRAME_HEAD1 0x55U
#define LIGHT_FRAME_TAIL0 0x55U
#define LIGHT_FRAME_TAIL1 0xAAU

/* 亮度档位统一集中定义，状态颜色只组合这些档位值。 */
#define LIGHT_LOW 4U
#define LIGHT_DIM LIGHT_LOW
#define LIGHT_MID LIGHT_LOW
#define LIGHT_HIGH LIGHT_LOW
#define LIGHT_HEARTBEAT_BREATH_TICKS 128U
#define LIGHT_FRIC_WORK_MIN_RPM 100.0f
#define LIGHT_FRIC_RUNNING_RPM 200.0f
#define LIGHT_FRIC_FDB_TIMEOUT 300U

/* 在线状态灯位：一个检测对象对应一颗灯。 */
#define LIGHT_DBUS_ONLINE_LED 0U
#define LIGHT_CHASSIS_ONLINE_LED 1U
#define LIGHT_FRIC_ONLINE_LED 2U
#define LIGHT_AUTO_AIM_ONLINE_LED 3U
#define LIGHT_CHASSIS_STATUS_FIRST_LED 4U
#define LIGHT_CHASSIS_STATUS_LAST_LED 6U
#define LIGHT_GIMBAL_STATUS_LED 7U
#define LIGHT_CAP_STATUS_LED 8U
#define LIGHT_HEARTBEAT_LED 9U

/* 在线状态缓存：灯光任务先汇总各模块在线结果，再统一渲染到灯板。 */
typedef struct
{
    bool dbus_online;
    bool chassis_motor_online;
    bool fric_motor_online;
    bool fric_motor_working;
    bool fric_motor_running;
    bool auto_aim_online;
    bool auto_aim_active;
} light_online_t;

/* 灯光任务控制块：保存任务句柄、工作模式、灯数组和串口帧缓存。 */
typedef struct
{
    osThreadId task_handle;
    volatile light_mode_t mode;
    light_rgb_t leds[LIGHT_LED_COUNT];
    uint8_t frame[LIGHT_UART_FRAME_BYTES];
    light_online_t online;
} light_control_t;

extern volatile gimbal_behaviour_e gimbal_behaviour;

static light_control_t light_control = {
    .task_handle = NULL,
    .mode = LIGHT_MODE_AUTO,
};

static void light_render_auto(void);
static void light_update_online_status(void);
static bool light_chassis_motor_online(void);
static bool light_fric_motor_online(void);
#if ROBOT_FRICTION != friction_none
static bool light_fric_one_motor_online(const shoot_motor_t *motor, uint32_t now);
#endif
static bool light_fric_motor_working(void);
static bool light_fric_motor_running(void);
#if ROBOT_FRICTION != friction_none
static float light_fric_average_speed_rpm(void);
static float light_abs_float(float value);
#endif
#if ROBOT_GIMBAL != multi_axis_robotic_arm
static void light_render_online_status(void);
#endif
static void light_render_arm_teach_status(uint8_t tick);
static void light_render_chassis_status(void);
static void light_render_gimbal_status(void);
static void light_render_cap_status(void);
static void light_render_heartbeat(uint8_t tick);
static void light_pack_frame(void);
static void light_send_frame(void);
static void light_fill(uint8_t first, uint8_t last, uint8_t r, uint8_t g, uint8_t b);


/* 创建灯光任务，任务只负责状态显示和串口发送，不参与控制闭环。 */
void LightTask_Init(void)
{
    osThreadDef(lightTask, light_task, osPriorityLow, 0, 256);
    light_control.task_handle = osThreadCreate(osThread(lightTask), NULL);
}


/* 灯光任务主循环：自动模式周期刷新状态灯，手动模式保持外部写入的灯值并周期发送。 */
void light_task(void const *pvParameters)
{
    (void)pvParameters;

    osDelay(LIGHT_TASK_INIT_TIME_MS);
    light_clear();
    light_send_frame();

    while (1)
    {
        if (light_control.mode == LIGHT_MODE_AUTO)
        {
            light_render_auto();
        }

        light_send_frame();
        osDelay(LIGHT_TASK_PERIOD_MS);
    }
}


/* 自动灯效入口：按在线状态、底盘、云台、电容和心跳刷新灯板。 */
static void light_render_auto(void)
{
    static uint8_t tick = 0U;

    light_update_online_status();
    light_clear();

#if ROBOT_GIMBAL == multi_axis_robotic_arm
    light_render_arm_teach_status(tick);
#else
    light_render_online_status();
#endif
    light_render_chassis_status();
    light_render_gimbal_status();
    light_render_cap_status();
    light_render_heartbeat(tick);

    tick++;
}


/* 更新灯光任务内部的在线状态缓存，渲染层只读取这个缓存。 */
static void light_update_online_status(void)
{
    const uint8_t auto_aim_online = *((volatile uint8_t *)&aim.online);
    const uint8_t auto_aim_flag = *((volatile uint8_t *)&aim.auto_aim_flag);

    light_control.online.dbus_online =
        (toe_is_error(DBUS_TOE) == 0U);
    light_control.online.chassis_motor_online =
        light_chassis_motor_online();
    light_control.online.fric_motor_online =
        light_fric_motor_online();
    light_control.online.fric_motor_working =
        light_fric_motor_working();
    light_control.online.fric_motor_running =
        light_fric_motor_running();
    light_control.online.auto_aim_online =
        (auto_aim_online != 0U);
    light_control.online.auto_aim_active =
        ((auto_aim_online != 0U) && (auto_aim_flag == AIM_ON));
}


/* 判断 8 个底盘驱动电机是否全部在线。 */
static bool light_chassis_motor_online(void)
{
    for (uint8_t toe = CHASSIS_MOTOR1_TOE; toe <= CHASSIS_MOTOR4_TOE; toe++)
    {
        if (toe_is_error(toe))
        {
            return false;
        }
    }

    return true;
}


/* 判断 3 个摩擦轮电机是否全部在线。 */
static bool light_fric_motor_online(void)
{
    const uint32_t now = HAL_GetTick();

#if ROBOT_FRICTION == friction_double
    return light_fric_one_motor_online(&shoot_control.fric1, now) &&
           light_fric_one_motor_online(&shoot_control.fric2, now) &&
           light_fric_one_motor_online(&shoot_control.fric1_, now) &&
           light_fric_one_motor_online(&shoot_control.fric2_, now);
#elif ROBOT_FRICTION == friction_3508
    return light_fric_one_motor_online(&shoot_control.fric1, now) &&
           light_fric_one_motor_online(&shoot_control.fric2, now);
#else
    (void)now;
    return true;
#endif
}


/* 判断单个摩擦轮电机是否在线。 */
#if ROBOT_FRICTION != friction_none
static bool light_fric_one_motor_online(const shoot_motor_t *motor, uint32_t now)
{
    if ((motor == NULL) || (motor->shoot_motor_measure == NULL))
    {
        return false;
    }

    if (motor->shoot_motor_measure->last_fdb_time == 0U)
    {
        return false;
    }

    return ((now - motor->shoot_motor_measure->last_fdb_time) <= LIGHT_FRIC_FDB_TIMEOUT);
}
#endif


/* 判断摩擦轮使能后是否达到工作转速。 */
static bool light_fric_motor_working(void)
{
#if ROBOT_FRICTION == friction_none
    return true;
#else
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        return true;
    }

    return (light_fric_average_speed_rpm() >= LIGHT_FRIC_WORK_MIN_RPM);
#endif
}


/* 判断摩擦轮使能后是否达到运行转速。 */
static bool light_fric_motor_running(void)
{
#if ROBOT_FRICTION == friction_none
    return false;
#else
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        return false;
    }

    return (light_fric_average_speed_rpm() > LIGHT_FRIC_RUNNING_RPM);
#endif
}


/* 计算 3 路摩擦轮平均转速的绝对值，单位 rpm。 */
#if ROBOT_FRICTION != friction_none
static float light_fric_average_speed_rpm(void)
{
    float sum = 0.0f;
    float count = 0.0f;

#if ROBOT_FRICTION == friction_double
    if (shoot_control.fric1.shoot_motor_measure != NULL)
    {
        sum += light_abs_float((float)shoot_control.fric1.shoot_motor_measure->speed_rpm);
        count += 1.0f;
    }
    if (shoot_control.fric2.shoot_motor_measure != NULL)
    {
        sum += light_abs_float((float)shoot_control.fric2.shoot_motor_measure->speed_rpm);
        count += 1.0f;
    }
    if (shoot_control.fric1_.shoot_motor_measure != NULL)
    {
        sum += light_abs_float((float)shoot_control.fric1_.shoot_motor_measure->speed_rpm);
        count += 1.0f;
    }
    if (shoot_control.fric2_.shoot_motor_measure != NULL)
    {
        sum += light_abs_float((float)shoot_control.fric2_.shoot_motor_measure->speed_rpm);
        count += 1.0f;
    }
#elif ROBOT_FRICTION == friction_3508
    if (shoot_control.fric1.shoot_motor_measure != NULL)
    {
        sum += light_abs_float((float)shoot_control.fric1.shoot_motor_measure->speed_rpm);
        count += 1.0f;
    }
    if (shoot_control.fric2.shoot_motor_measure != NULL)
    {
        sum += light_abs_float((float)shoot_control.fric2.shoot_motor_measure->speed_rpm);
        count += 1.0f;
    }
#endif

    return (count > 0.0f) ? (sum / count) : 0.0f;
}


/* 返回浮点绝对值，供阈值判断使用。 */
static float light_abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}
#endif


/* 刷新 0~3 号在线状态灯。 */
#if ROBOT_GIMBAL != multi_axis_robotic_arm
static void light_render_online_status(void)
{
    if (light_control.online.dbus_online)
    {
        light_set_pixel(LIGHT_DBUS_ONLINE_LED, 0U, LIGHT_MID, 0U);
    }
    else
    {
        light_set_pixel(LIGHT_DBUS_ONLINE_LED, LIGHT_HIGH, 0U, 0U);
    }

    if (light_control.online.chassis_motor_online)
    {
        light_set_pixel(LIGHT_CHASSIS_ONLINE_LED, 0U, LIGHT_MID, 0U);
    }
    else
    {
        light_set_pixel(LIGHT_CHASSIS_ONLINE_LED, LIGHT_HIGH, 0U, 0U);
    }

    if (light_control.online.fric_motor_online)
    {
        if (!light_control.online.fric_motor_working)
        {
            light_set_pixel(LIGHT_FRIC_ONLINE_LED, LIGHT_HIGH, LIGHT_HIGH, 0U);
        }
        else if (light_control.online.fric_motor_running)
        {
            light_set_pixel(LIGHT_FRIC_ONLINE_LED, 0U, LIGHT_MID, LIGHT_MID);
        }
        else
        {
            light_set_pixel(LIGHT_FRIC_ONLINE_LED, 0U, LIGHT_MID, 0U);
        }
    }
    else
    {
        light_set_pixel(LIGHT_FRIC_ONLINE_LED, LIGHT_HIGH, 0U, 0U);
    }

    if (light_control.online.auto_aim_active)
    {
        light_set_pixel(LIGHT_AUTO_AIM_ONLINE_LED, 0U, LIGHT_MID, 0U);
    }
    else if (light_control.online.auto_aim_online)
    {
        light_set_pixel(LIGHT_AUTO_AIM_ONLINE_LED, LIGHT_HIGH, LIGHT_HIGH, 0U);
    }
    else
    {
        light_set_pixel(LIGHT_AUTO_AIM_ONLINE_LED, LIGHT_HIGH, 0U, 0U);
    }
}
#endif


/* 刷新 4~6 号灯，显示底盘行为模式。 */
static void light_render_arm_teach_status(uint8_t tick)
{
#if ROBOT_GIMBAL == multi_axis_robotic_arm
    uint8_t selected = arm_teach_get_selected_slot();
    uint8_t blink_on = (uint8_t)(((tick >> 2) & 0x01U) != 0U);
    uint8_t breath = (uint8_t)(1U + (tick & 0x03U));

    for (uint8_t i = 0U; i < ARM_TEACH_SLOT_NUM; i++)
    {
        uint8_t level = (i == selected) ? breath : LIGHT_DIM;

        switch (arm_teach_get_light_state(i))
        {
            case ARM_TEACH_LIGHT_RECORDING:
                light_set_pixel(i, blink_on ? level : 0U, blink_on ? level : 0U, 0U);
                break;

            case ARM_TEACH_LIGHT_DONE:
                light_set_pixel(i, 0U, level, 0U);
                break;

            case ARM_TEACH_LIGHT_PLAYING:
                light_set_pixel(i, blink_on ? level : 0U, 0U, blink_on ? level : 0U);
                break;

            case ARM_TEACH_LIGHT_ERROR:
                light_set_pixel(i, blink_on ? level : 0U, 0U, 0U);
                break;

            case ARM_TEACH_LIGHT_READY:
            case ARM_TEACH_LIGHT_EMPTY:
            default:
                light_set_pixel(i, 0U, 0U, level);
                break;
        }
    }
#else
    (void)tick;
#endif
}

static void light_render_chassis_status(void)
{
    switch (chassis_behaviour_mode)
    {
        case CHASSIS_NO_MOVE:
            light_fill(LIGHT_CHASSIS_STATUS_FIRST_LED,
                       LIGHT_CHASSIS_STATUS_LAST_LED,
                       0U,
                       0U,
                       LIGHT_DIM);
            break;

        case CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW:
        case CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW:
            light_fill(LIGHT_CHASSIS_STATUS_FIRST_LED,
                       LIGHT_CHASSIS_STATUS_LAST_LED,
                       0U,
                       LIGHT_MID,
                       LIGHT_MID);
            break;

        case CHASSIS_SPIN:
            light_fill(LIGHT_CHASSIS_STATUS_FIRST_LED,
                       LIGHT_CHASSIS_STATUS_LAST_LED,
                       LIGHT_LOW,
                       0U,
                       LIGHT_MID);
            break;

        case CHASSIS_NO_FOLLOW_YAW:
        case CHASSIS_OPEN:
        default:
            light_fill(LIGHT_CHASSIS_STATUS_FIRST_LED,
                       LIGHT_CHASSIS_STATUS_LAST_LED,
                       LIGHT_MID,
                       LIGHT_MID,
                       0U);
            break;
    }
}


/* 刷新 7 号灯，显示云台行为模式。 */
static void light_render_gimbal_status(void)
{
    switch (gimbal_behaviour)
    {
        case GIMBAL_ZERO_FORCE:
        case GIMBAL_MOTIONLESS:
            light_set_pixel(LIGHT_GIMBAL_STATUS_LED, LIGHT_LOW, 0U, 0U);
            break;

        case GIMBAL_INIT:
        case GIMBAL_CALI:
            light_set_pixel(LIGHT_GIMBAL_STATUS_LED, LIGHT_MID, LIGHT_MID, 0U);
            break;

        case GIMBAL_SPIN:
            light_set_pixel(LIGHT_GIMBAL_STATUS_LED, LIGHT_LOW, 0U, LIGHT_MID);
            break;

        case GIMBAL_ABSOLUTE_ANGLE:
        case GIMBAL_RELATIVE_ANGLE:
        default:
            light_set_pixel(LIGHT_GIMBAL_STATUS_LED, 0U, LIGHT_MID, 0U);
            break;
    }
}


/* 刷新 8 号灯，显示超级电容状态。 */
static void light_render_cap_status(void)
{
    light_set_pixel(LIGHT_CAP_STATUS_LED, 0U, 0U, LIGHT_DIM);
}


/* 刷新 9 号灯作为任务心跳。 */
static void light_render_heartbeat(uint8_t tick)
{
    const uint8_t phase = tick % LIGHT_HEARTBEAT_BREATH_TICKS;
    const uint8_t ramp = (phase < (LIGHT_HEARTBEAT_BREATH_TICKS / 2U)) ?
                         phase :
                         (uint8_t)(LIGHT_HEARTBEAT_BREATH_TICKS - 1U - phase);
    const uint8_t brightness =
        (uint8_t)((uint16_t)ramp * LIGHT_LOW / ((LIGHT_HEARTBEAT_BREATH_TICKS / 2U) - 1U));

    light_set_pixel(LIGHT_HEARTBEAT_LED, brightness, 0U, brightness);
}


/* 把 light_control.leds[] 打包成灯板串口帧。 */
static void light_pack_frame(void)
{
    uint8_t frame_index = 0U;

    light_control.frame[frame_index++] = LIGHT_FRAME_HEAD0;
    light_control.frame[frame_index++] = LIGHT_FRAME_HEAD1;

    for (uint8_t i = 0U; i < LIGHT_LED_COUNT; i++)
    {
        light_control.frame[frame_index++] = light_control.leds[i].r;
        light_control.frame[frame_index++] = light_control.leds[i].g;
        light_control.frame[frame_index++] = light_control.leds[i].b;
    }

    light_control.frame[frame_index++] = LIGHT_FRAME_TAIL0;
    light_control.frame[frame_index] = LIGHT_FRAME_TAIL1;
}


/* 发送当前灯效帧。 */
static void light_send_frame(void)
{
    light_pack_frame();
    USART7_Transmit(light_control.frame, (uint16_t)sizeof(light_control.frame));
}


/* 将指定区间的灯珠设置为同一 RGB 颜色。 */
static void light_fill(uint8_t first, uint8_t last, uint8_t r, uint8_t g, uint8_t b)
{
    if (first >= LIGHT_LED_COUNT)
    {
        return;
    }

    if (last >= LIGHT_LED_COUNT)
    {
        last = LIGHT_LED_COUNT - 1U;
    }

    for (uint8_t i = first; i <= last; i++)
    {
        light_control.leds[i].r = r;
        light_control.leds[i].g = g;
        light_control.leds[i].b = b;
    }
}

/* 切换到自动灯效模式。 */
void light_set_auto_mode(void)
{
    light_control.mode = LIGHT_MODE_AUTO;
}

/* 切换到手动调灯模式，自动刷新暂停。 */
void light_set_manual_mode(void)
{
    light_control.mode = LIGHT_MODE_MANUAL;
}


/* 手动设置整板颜色。 */
void light_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    light_set_manual_mode();
    light_fill(0U, LIGHT_LED_COUNT - 1U, r, g, b);
}


/* 设置单颗灯珠颜色。 */
void light_set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= LIGHT_LED_COUNT)
    {
        return;
    }

    light_control.leds[index].r = r;
    light_control.leds[index].g = g;
    light_control.leds[index].b = b;
}

/* 清空所有灯珠颜色。 */
void light_clear(void)
{
    memset(light_control.leds, 0, sizeof(light_control.leds));
}

/* 立即按当前灯值刷新灯板。 */
void light_refresh_now(void)
{
    light_send_frame();
}
