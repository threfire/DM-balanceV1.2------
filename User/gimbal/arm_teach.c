#include "arm_teach.h"

#if ROBOT_GIMBAL == multi_axis_robotic_arm

#include <string.h>

#include "CRC8_CRC16.h"
#include "bsp_flash.h"
#include "cmsis_os.h"
#include "w25q64.h"

#define ARM_TEACH_MAGIC             0x54434841UL
#define ARM_TEACH_VERSION           1UL
#define ARM_TEACH_SECTOR_SIZE       4096UL
#define ARM_TEACH_CLEAR_LONG_MS     1000U
#define ARM_TEACH_MOTOR_TIMEOUT_MS  100U
#define ARM_TEACH_RAMP_STEP_RAD     0.008f
#define ARM_TEACH_MAX_STEP_RAD      0.20f
#define ARM_TEACH_MAX_FEEDBACK_VEL  40.0f

typedef struct
{
    float pos[ARM_JOINT_NUM];
    uint8_t claw;
    uint8_t reserved[3];
} arm_teach_point_t;

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t point_count;
    uint32_t point_size;
    uint16_t crc16;
    uint16_t header_crc16;
    uint32_t seq;
    uint32_t reserved[3];
} arm_teach_flash_header_t;

typedef struct
{
    uint32_t point_count;
    uint16_t crc16;
    uint8_t valid;
    uint8_t error;
} arm_teach_slot_t;

typedef struct
{
    uint8_t active;
    uint8_t selected_slot;
    uint8_t loaded_slot;
    uint8_t loaded_valid;
    uint8_t recording;
    uint8_t playing;
    uint8_t play_ramp;
    uint8_t clear_handled;
    volatile uint8_t save_pending;
    volatile uint8_t erase_pending;
    volatile uint8_t saving;
    uint8_t save_slot;
    uint8_t erase_slot_index;
    uint8_t erase_all_request;
    uint16_t clear_ticks;
    uint16_t last_keys;
    uint32_t save_count;
    uint32_t record_index;
    uint32_t play_index;
    uint32_t last_sample_ms;
    uint32_t seq;
    float play_target[ARM_JOINT_NUM];
    float last_play_target[ARM_JOINT_NUM];
    uint8_t play_claw;
    arm_teach_slot_t slot[ARM_TEACH_SLOT_NUM];
} arm_teach_ctrl_t;

static arm_teach_ctrl_t teach;
static arm_teach_point_t teach_points[ARM_TEACH_MAX_POINTS];
static osThreadId teach_storage_task_handle;

extern moterMapHeader motor_data;

static uint32_t arm_teach_slot_addr(uint8_t slot)
{
    return ARM_TEACH_FLASH_BASE + (uint32_t)slot * ARM_TEACH_SLOT_BYTES;
}

static uint8_t key_rising(uint16_t keys, uint16_t key)
{
    return ((keys & key) != 0U) && ((teach.last_keys & key) == 0U);
}

static uint8_t storage_busy(void)
{
    return (teach.save_pending || teach.erase_pending || teach.saving) ? 1U : 0U;
}

static float absf_local(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float clampf_local(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

static uint16_t calc_points_crc(const arm_teach_point_t *points, uint32_t count)
{
    return get_CRC16_check_sum((uint8_t *)points,
                               count * (uint32_t)sizeof(arm_teach_point_t),
                               0xFFFF);
}

static uint16_t calc_header_crc(arm_teach_flash_header_t *header)
{
    uint16_t old_crc = header->header_crc16;
    uint16_t crc;

    header->header_crc16 = 0U;
    crc = get_CRC16_check_sum((uint8_t *)header, sizeof(*header), 0xFFFF);
    header->header_crc16 = old_crc;

    return crc;
}

static uint8_t read_header(uint8_t slot, arm_teach_flash_header_t *header)
{
    if (slot >= ARM_TEACH_SLOT_NUM || header == NULL)
    {
        return 0U;
    }

    if (OSPI_W25Qxx_ReadBuffer((uint8_t *)header,
                               arm_teach_slot_addr(slot),
                               sizeof(*header)) != OSPI_W25Qxx_OK)
    {
        return 0U;
    }

    if (header->magic != ARM_TEACH_MAGIC ||
        header->version != ARM_TEACH_VERSION ||
        header->point_size != sizeof(arm_teach_point_t) ||
        header->point_count == 0U ||
        header->point_count > ARM_TEACH_MAX_POINTS)
    {
        return 0U;
    }

    return (calc_header_crc(header) == header->header_crc16) ? 1U : 0U;
}

static void load_headers(void)
{
    arm_teach_flash_header_t header;

    for (uint8_t i = 0U; i < ARM_TEACH_SLOT_NUM; i++)
    {
        if (read_header(i, &header))
        {
            teach.slot[i].point_count = header.point_count;
            teach.slot[i].crc16 = header.crc16;
            teach.slot[i].valid = 1U;
            teach.slot[i].error = 0U;
            if (header.seq > teach.seq)
            {
                teach.seq = header.seq;
            }
        }
        else
        {
            teach.slot[i].point_count = 0U;
            teach.slot[i].crc16 = 0U;
            teach.slot[i].valid = 0U;
            teach.slot[i].error = 0U;
        }
    }
}

static uint8_t load_slot_points(uint8_t slot)
{
    arm_teach_flash_header_t header;
    uint32_t data_bytes;
    uint16_t crc;

    if (slot >= ARM_TEACH_SLOT_NUM)
    {
        return 0U;
    }

    if (teach.loaded_valid && teach.loaded_slot == slot)
    {
        return 1U;
    }

    if (!read_header(slot, &header))
    {
        teach.slot[slot].error = 1U;
        return 0U;
    }

    data_bytes = header.point_count * (uint32_t)sizeof(arm_teach_point_t);
    if (OSPI_W25Qxx_ReadBuffer((uint8_t *)teach_points,
                               arm_teach_slot_addr(slot) + sizeof(header),
                               data_bytes) != OSPI_W25Qxx_OK)
    {
        teach.slot[slot].error = 1U;
        return 0U;
    }

    crc = calc_points_crc(teach_points, header.point_count);
    if (crc != header.crc16)
    {
        teach.slot[slot].error = 1U;
        return 0U;
    }

    teach.slot[slot].point_count = header.point_count;
    teach.slot[slot].crc16 = header.crc16;
    teach.slot[slot].valid = 1U;
    teach.slot[slot].error = 0U;
    teach.loaded_slot = slot;
    teach.loaded_valid = 1U;

    return 1U;
}

static uint8_t save_slot_points(uint8_t slot, uint32_t count)
{
    arm_teach_flash_header_t header;
    uint32_t addr;
    uint32_t sector_count;
    uint32_t data_bytes;

    if (slot >= ARM_TEACH_SLOT_NUM || count == 0U || count > ARM_TEACH_MAX_POINTS)
    {
        return 0U;
    }

    addr = arm_teach_slot_addr(slot);
    sector_count = ARM_TEACH_SLOT_BYTES / ARM_TEACH_SECTOR_SIZE;
    data_bytes = count * (uint32_t)sizeof(arm_teach_point_t);

    memset(&header, 0, sizeof(header));
    header.magic = ARM_TEACH_MAGIC;
    header.version = ARM_TEACH_VERSION;
    header.point_count = count;
    header.point_size = sizeof(arm_teach_point_t);
    header.crc16 = calc_points_crc(teach_points, count);
    header.seq = ++teach.seq;
    header.header_crc16 = calc_header_crc(&header);

    flash_erase_address(addr, (uint16_t)sector_count);

    if (OSPI_W25Qxx_WriteBuffer((uint8_t *)&header,
                                addr,
                                sizeof(header)) != OSPI_W25Qxx_OK)
    {
        return 0U;
    }

    if (OSPI_W25Qxx_WriteBuffer((uint8_t *)teach_points,
                                addr + sizeof(header),
                                data_bytes) != OSPI_W25Qxx_OK)
    {
        return 0U;
    }

    teach.slot[slot].point_count = count;
    teach.slot[slot].crc16 = header.crc16;
    teach.slot[slot].valid = 1U;
    teach.slot[slot].error = 0U;
    teach.loaded_slot = slot;
    teach.loaded_valid = 1U;

    return 1U;
}

static void erase_slot(uint8_t slot)
{
    if (slot >= ARM_TEACH_SLOT_NUM)
    {
        return;
    }

    flash_erase_address(arm_teach_slot_addr(slot),
                        (uint16_t)(ARM_TEACH_SLOT_BYTES / ARM_TEACH_SECTOR_SIZE));

    teach.slot[slot].point_count = 0U;
    teach.slot[slot].crc16 = 0U;
    teach.slot[slot].valid = 0U;
    teach.slot[slot].error = 0U;
    if (teach.loaded_slot == slot)
    {
        teach.loaded_valid = 0U;
    }
}

static void erase_all_slots(void)
{
    for (uint8_t i = 0U; i < ARM_TEACH_SLOT_NUM; i++)
    {
        erase_slot(i);
    }
}

static uint8_t safety_ok(gimbal_control_t *ctrl, uint8_t check_vel)
{
    uint32_t now;

    if (ctrl == NULL || ctrl->gimbal_rc_ctrl == NULL)
    {
        return 0U;
    }

    now = HAL_GetTick();
    if ((now - ctrl->gimbal_rc_ctrl->last_fdb) > ARM_TEACH_MOTOR_TIMEOUT_MS)
    {
        return 0U;
    }

    for (uint8_t i = 0U; i < ARM_JOINT_NUM; i++)
    {
        if (ctrl->joint_motor[i].motor_measure == NULL)
        {
            return 0U;
        }

        if ((now - ctrl->joint_motor[i].motor_measure->last_fdb_time) > ARM_TEACH_MOTOR_TIMEOUT_MS)
        {
            return 0U;
        }

        if (ctrl->joint_motor[i].motor_measure->pos < ctrl->joint_motor[i].min_angle - 0.10f ||
            ctrl->joint_motor[i].motor_measure->pos > ctrl->joint_motor[i].max_angle + 0.10f)
        {
            return 0U;
        }

        if (check_vel && absf_local(ctrl->joint_motor[i].motor_measure->vel) > ARM_TEACH_MAX_FEEDBACK_VEL)
        {
            return 0U;
        }
    }

    return 1U;
}

static void hold_current(gimbal_control_t *ctrl)
{
    if (ctrl == NULL)
    {
        return;
    }

    for (uint8_t i = 0U; i < ARM_JOINT_NUM; i++)
    {
        if (ctrl->joint_motor[i].motor_measure != NULL)
        {
            ctrl->multi_arm_set[i][0] = ctrl->joint_motor[i].motor_measure->pos;
            ctrl->multi_arm_set[i][1] = 0.0f;
            teach.play_target[i] = ctrl->joint_motor[i].motor_measure->pos;
            teach.last_play_target[i] = teach.play_target[i];
        }
        ctrl->joint_integrator[i].integral = 0.0f;
        ctrl->joint_integrator[i].error_prev = 0.0f;
    }
}

static void set_error(gimbal_control_t *ctrl)
{
    teach.recording = 0U;
    teach.playing = 0U;
    teach.play_ramp = 0U;
    teach.slot[teach.selected_slot].error = 1U;
    hold_current(ctrl);
}

static float claw_torque_from_flag(uint8_t claw)
{
    static uint8_t init = 0U;
    static uint8_t last = 0U;
    static float tau = 6.0f;

    if (!init)
    {
        tau = claw ? -20.0f : 6.0f;
        last = claw;
        init = 1U;
    }
    else if (claw != last)
    {
        tau = claw ? -26.0f : 20.0f;
        last = claw;
    }
    else if (claw)
    {
        tau += 0.10f * (-18.0f - tau);
    }
    else
    {
        tau += 0.001f * (6.0f - tau);
    }

    if (tau > 28.0f)
    {
        tau = 28.0f;
    }
    else if (tau < -28.0f)
    {
        tau = -28.0f;
    }

    return tau;
}

static void start_record(gimbal_control_t *ctrl)
{
    teach.recording = 1U;
    teach.playing = 0U;
    teach.play_ramp = 0U;
    teach.record_index = 0U;
    teach.last_sample_ms = HAL_GetTick() - ARM_TEACH_SAMPLE_MS;
    teach.slot[teach.selected_slot].error = 0U;
    hold_current(ctrl);
}

static void finish_record(void)
{
    uint32_t count = teach.record_index;

    teach.recording = 0U;
    if (count == 0U || storage_busy())
    {
        teach.slot[teach.selected_slot].error = 1U;
        return;
    }

    if (teach_storage_task_handle == NULL)
    {
        if (!save_slot_points(teach.selected_slot, count))
        {
            teach.slot[teach.selected_slot].error = 1U;
        }
        return;
    }

    teach.save_slot = teach.selected_slot;
    teach.save_count = count;
    teach.saving = 1U;
    teach.save_pending = 1U;
}

static void sample_record(gimbal_control_t *ctrl)
{
    arm_teach_point_t *point;
    uint32_t now = HAL_GetTick();

    if ((now - teach.last_sample_ms) < ARM_TEACH_SAMPLE_MS)
    {
        return;
    }
    teach.last_sample_ms = now;

    if (!safety_ok(ctrl, 0U))
    {
        set_error(ctrl);
        return;
    }

    if (teach.record_index >= ARM_TEACH_MAX_POINTS)
    {
        finish_record();
        return;
    }

    point = &teach_points[teach.record_index++];
    for (uint8_t i = 0U; i < ARM_JOINT_NUM; i++)
    {
        point->pos[i] = ctrl->joint_motor[i].motor_measure->pos;
    }
    point->claw = motor_data.claw ? 1U : 0U;
}

static uint8_t prepare_play_point(gimbal_control_t *ctrl)
{
    arm_teach_point_t *point;

    if (teach.play_index >= teach.slot[teach.selected_slot].point_count)
    {
        teach.play_index = 0U;
    }

    point = &teach_points[teach.play_index];
    for (uint8_t i = 0U; i < ARM_JOINT_NUM; i++)
    {
        float target = clampf_local(point->pos[i],
                                    ctrl->joint_motor[i].min_angle,
                                    ctrl->joint_motor[i].max_angle);
        if (absf_local(target - teach.last_play_target[i]) > ARM_TEACH_MAX_STEP_RAD)
        {
            return 0U;
        }
        teach.play_target[i] = target;
    }

    teach.play_claw = point->claw;
    for (uint8_t i = 0U; i < ARM_JOINT_NUM; i++)
    {
        teach.last_play_target[i] = teach.play_target[i];
    }

    teach.play_index++;
    return 1U;
}

static void start_play(gimbal_control_t *ctrl)
{
    if (!teach.slot[teach.selected_slot].valid ||
        teach.slot[teach.selected_slot].point_count == 0U ||
        !load_slot_points(teach.selected_slot))
    {
        set_error(ctrl);
        return;
    }

    teach.recording = 0U;
    teach.playing = 1U;
    teach.play_ramp = 1U;
    teach.play_index = 0U;
    teach.last_sample_ms = HAL_GetTick() - ARM_TEACH_SAMPLE_MS;
    teach.slot[teach.selected_slot].error = 0U;

    for (uint8_t i = 0U; i < ARM_JOINT_NUM; i++)
    {
        teach.play_target[i] = ctrl->joint_motor[i].motor_measure->pos;
        teach.last_play_target[i] = teach_points[0].pos[i];
    }
    teach.play_claw = teach_points[0].claw;
}

static void stop_play(gimbal_control_t *ctrl)
{
    teach.playing = 0U;
    teach.play_ramp = 0U;
    hold_current(ctrl);
}

static void update_ramp(gimbal_control_t *ctrl)
{
    uint8_t done = 1U;

    for (uint8_t i = 0U; i < ARM_JOINT_NUM; i++)
    {
        float target = clampf_local(teach_points[0].pos[i],
                                    ctrl->joint_motor[i].min_angle,
                                    ctrl->joint_motor[i].max_angle);
        float err = target - teach.play_target[i];

        if (absf_local(err) > ARM_TEACH_RAMP_STEP_RAD)
        {
            teach.play_target[i] += (err > 0.0f) ? ARM_TEACH_RAMP_STEP_RAD : -ARM_TEACH_RAMP_STEP_RAD;
            done = 0U;
        }
        else
        {
            teach.play_target[i] = target;
        }
    }

    teach.play_claw = teach_points[0].claw;
    if (done)
    {
        teach.play_ramp = 0U;
        teach.play_index = 0U;
        for (uint8_t i = 0U; i < ARM_JOINT_NUM; i++)
        {
            teach.last_play_target[i] = teach_points[0].pos[i];
        }
    }
}

static void update_play(gimbal_control_t *ctrl)
{
    uint32_t now = HAL_GetTick();

    if ((now - teach.last_sample_ms) < ARM_TEACH_SAMPLE_MS)
    {
        return;
    }
    teach.last_sample_ms = now;

    if (!safety_ok(ctrl, 1U))
    {
        set_error(ctrl);
        return;
    }

    if (teach.play_ramp)
    {
        update_ramp(ctrl);
        return;
    }

    if (!prepare_play_point(ctrl))
    {
        set_error(ctrl);
        return;
    }
}

static void handle_clear_key(uint16_t keys)
{
    if (storage_busy())
    {
        if ((keys & ARM_TEACH_CLEAR_KEY) == 0U)
        {
            teach.clear_ticks = 0U;
            teach.clear_handled = 0U;
        }
        return;
    }

    if (keys & ARM_TEACH_CLEAR_KEY)
    {
        if (teach.clear_ticks < ARM_TEACH_CLEAR_LONG_MS)
        {
            teach.clear_ticks++;
        }
        if (teach.clear_ticks >= ARM_TEACH_CLEAR_LONG_MS && !teach.clear_handled)
        {
            if (teach_storage_task_handle == NULL)
            {
                erase_all_slots();
            }
            else
            {
                teach.erase_all_request = 1U;
                teach.erase_slot_index = teach.selected_slot;
                teach.save_slot = teach.selected_slot;
                teach.saving = 1U;
                teach.erase_pending = 1U;
            }
            teach.clear_handled = 1U;
        }
    }
    else
    {
        if ((teach.last_keys & ARM_TEACH_CLEAR_KEY) && teach.clear_ticks < ARM_TEACH_CLEAR_LONG_MS)
        {
            if (teach_storage_task_handle == NULL)
            {
                erase_slot(teach.selected_slot);
            }
            else
            {
                teach.erase_all_request = 0U;
                teach.erase_slot_index = teach.selected_slot;
                teach.save_slot = teach.selected_slot;
                teach.saving = 1U;
                teach.erase_pending = 1U;
            }
        }
        teach.clear_ticks = 0U;
        teach.clear_handled = 0U;
    }
}

static void arm_teach_storage_task(void const *argument)
{
    (void)argument;

    while (1)
    {
        if (teach.save_pending)
        {
            uint8_t slot = teach.save_slot;
            uint32_t count = teach.save_count;

            teach.save_pending = 0U;
            if (!save_slot_points(slot, count))
            {
                teach.slot[slot].error = 1U;
            }
            teach.saving = 0U;
        }
        else if (teach.erase_pending)
        {
            uint8_t slot = teach.erase_slot_index;
            uint8_t erase_all = teach.erase_all_request;

            teach.erase_pending = 0U;
            if (erase_all)
            {
                erase_all_slots();
            }
            else
            {
                erase_slot(slot);
            }
            teach.saving = 0U;
        }

        osDelay(10U);
    }
}

static void start_storage_task(void)
{
    if (teach_storage_task_handle == NULL)
    {
        osThreadDef(armTeachStorageTask, arm_teach_storage_task, osPriorityLow, 0, 512);
        teach_storage_task_handle = osThreadCreate(osThread(armTeachStorageTask), NULL);
    }
}

void arm_teach_init(void)
{
    memset(&teach, 0, sizeof(teach));
    teach.loaded_slot = 0xFFU;
    load_headers();
    start_storage_task();
}

arm_joint_e arm_teach_update(gimbal_control_t *ctrl, arm_joint_e base_mode)
{
    uint16_t keys;
    arm_joint_e out_mode = base_mode;

    if (ctrl == NULL || ctrl->gimbal_rc_ctrl == NULL)
    {
        return base_mode;
    }

    keys = ctrl->gimbal_rc_ctrl->key.v;

    if (key_rising(keys, ARM_TEACH_ENABLE_KEY))
    {
        if (teach.active)
        {
            teach.active = 0U;
            teach.recording = 0U;
            teach.playing = 0U;
            teach.play_ramp = 0U;
            hold_current(ctrl);
        }
        else
        {
            teach.active = 1U;
            hold_current(ctrl);
        }
    }

    if (teach.active)
    {
        uint8_t busy = storage_busy();

        if (key_rising(keys, ARM_TEACH_SLOT_KEY) && !teach.recording && !busy)
        {
            if (teach.playing)
            {
                stop_play(ctrl);
            }
            teach.selected_slot = (uint8_t)((teach.selected_slot + 1U) % ARM_TEACH_SLOT_NUM);
            hold_current(ctrl);
        }

        if (key_rising(keys, ARM_TEACH_RECORD_KEY))
        {
            if (teach.recording)
            {
                finish_record();
                hold_current(ctrl);
            }
            else if (!teach.playing && !busy)
            {
                start_record(ctrl);
            }
        }

        if (key_rising(keys, ARM_TEACH_PLAY_KEY) && !teach.recording && !busy)
        {
            if (teach.playing)
            {
                stop_play(ctrl);
            }
            else
            {
                start_play(ctrl);
            }
        }

        handle_clear_key(keys);

        if (teach.recording)
        {
            sample_record(ctrl);
            out_mode = ARM_TEACH_RECORD;
        }
        else if (storage_busy())
        {
            out_mode = ARM_TEACH_DRAG;
        }
        else if (teach.playing)
        {
            update_play(ctrl);
            out_mode = ARM_TEACH_PLAY;
        }
        else
        {
            out_mode = ARM_TEACH_DRAG;
        }
    }

    teach.last_keys = keys;
    return out_mode;
}

void arm_teach_apply_control(gimbal_control_t *ctrl)
{
    uint8_t claw = motor_data.claw ? 1U : 0U;

    if (ctrl == NULL)
    {
        return;
    }

    if (teach.playing)
    {
        claw = teach.play_claw;
    }

    for (uint8_t i = 0U; i < ARM_JOINT_NUM; i++)
    {
        ctrl->multi_arm_set[i][1] = 0.0f;
        if (teach.playing)
        {
            ctrl->multi_arm_set[i][0] = teach.play_target[i];
        }
        else if (ctrl->joint_motor[i].motor_measure != NULL)
        {
            ctrl->multi_arm_set[i][0] = ctrl->joint_motor[i].motor_measure->pos;
        }
    }

    ctrl->multi_arm_set[6][2] = claw_torque_from_flag(claw);
}

uint8_t arm_teach_is_active(void)
{
    return teach.active;
}

uint8_t arm_teach_get_selected_slot(void)
{
    return teach.selected_slot;
}

uint16_t arm_teach_get_point_count(uint8_t slot)
{
    if (slot >= ARM_TEACH_SLOT_NUM)
    {
        return 0U;
    }

    return (uint16_t)teach.slot[slot].point_count;
}

arm_teach_light_e arm_teach_get_light_state(uint8_t slot)
{
    if (slot >= ARM_TEACH_SLOT_NUM)
    {
        return ARM_TEACH_LIGHT_EMPTY;
    }

    if (teach.slot[slot].error)
    {
        return ARM_TEACH_LIGHT_ERROR;
    }

    if (slot == teach.save_slot && storage_busy())
    {
        return ARM_TEACH_LIGHT_RECORDING;
    }

    if (teach.active && slot == teach.selected_slot)
    {
        if (teach.recording)
        {
            return ARM_TEACH_LIGHT_RECORDING;
        }
        if (teach.playing)
        {
            return ARM_TEACH_LIGHT_PLAYING;
        }
        if (teach.slot[slot].valid)
        {
            return ARM_TEACH_LIGHT_DONE;
        }
        return ARM_TEACH_LIGHT_READY;
    }

    return teach.slot[slot].valid ? ARM_TEACH_LIGHT_DONE : ARM_TEACH_LIGHT_EMPTY;
}

#endif
