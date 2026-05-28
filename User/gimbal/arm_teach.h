#ifndef ARM_TEACH_H
#define ARM_TEACH_H

#include "Multi-axis_robotic_arm.h"

#if ROBOT_GIMBAL == multi_axis_robotic_arm

#define ARM_TEACH_SLOT_NUM       4U
#define ARM_TEACH_MAX_POINTS     4000U
#define ARM_TEACH_SAMPLE_MS      1U

#ifndef ARM_TEACH_FLASH_BASE
#define ARM_TEACH_FLASH_BASE     0x010000UL
#endif

#ifndef ARM_TEACH_SLOT_BYTES
#define ARM_TEACH_SLOT_BYTES     0x24000UL
#endif

#ifndef ARM_TEACH_ENABLE_KEY
#define ARM_TEACH_ENABLE_KEY     KEY_PRESSED_OFFSET_E
#endif

#ifndef ARM_TEACH_SLOT_KEY
#define ARM_TEACH_SLOT_KEY       KEY_PRESSED_OFFSET_Q
#endif

#ifndef ARM_TEACH_RECORD_KEY
#define ARM_TEACH_RECORD_KEY     KEY_PRESSED_OFFSET_Z
#endif

#ifndef ARM_TEACH_PLAY_KEY
#define ARM_TEACH_PLAY_KEY       KEY_PRESSED_OFFSET_X
#endif

#ifndef ARM_TEACH_CLEAR_KEY
#define ARM_TEACH_CLEAR_KEY      KEY_PRESSED_OFFSET_C
#endif

typedef enum
{
    ARM_TEACH_LIGHT_EMPTY = 0,
    ARM_TEACH_LIGHT_READY,
    ARM_TEACH_LIGHT_RECORDING,
    ARM_TEACH_LIGHT_DONE,
    ARM_TEACH_LIGHT_PLAYING,
    ARM_TEACH_LIGHT_ERROR,
} arm_teach_light_e;

void arm_teach_init(void);
arm_joint_e arm_teach_update(gimbal_control_t *ctrl, arm_joint_e base_mode);
void arm_teach_apply_control(gimbal_control_t *ctrl);
uint8_t arm_teach_is_active(void);
uint8_t arm_teach_get_selected_slot(void);
uint16_t arm_teach_get_point_count(uint8_t slot);
arm_teach_light_e arm_teach_get_light_state(uint8_t slot);

#endif

#endif
