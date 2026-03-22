/**
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  * @file       robot_param.h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     3-26-2025       yibu             1. start and done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  */
#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#include "struct_typedef.h"

/*是否在线*/
#define OFFLINE 0
#define ONLINE 1

/* 电容开关 */
#define Cap_off 0
#define Cap_on  1

/* 1. 先给枚举值起同名的整型宏 */
#define Hero_robot      0x1
#define Infantry_robot  0x2
#define Balance_robot   0x3
#define Sentinel_robot  0x4
#define Engineer_robot  0x5

#define Mecanum_wheel   0
#define Omni_wheel      1
#define Steering_wheel  2
#define Balance_wheel   3

#define yaw_pitch_direct   0
#define yaw_pitch_linkage  1
#define double_yaw_pitch   2
#define multi_axis_robotic_arm 3

#define friction_3508   0
#define friction_double 1
#define friction_none   2

#define chassis_board 0x00
#define gimbal_board  0x01

#define DJI_3508 0
#define DJI_6020 1
#define DM4310   2
#define RD_motor 3

#define debug   0
#define release 1

/* ================= 机器人整体配置 ================= */

#define ROBOT_TYPE        Engineer_robot
#define ROBOT_MODE        debug
#define ROBOT_CHASSIS    	Mecanum_wheel
#define ROBOT_GIMBAL      multi_axis_robotic_arm
#define ROBOT_FRICTION    friction_none
#define ROBOT_BOARD      	gimbal_board 
#define ROBOT_CAP         Cap_off

//把底盘、云台、发射机构等通用参数藏到各个兵种的私有头文件里去
#if ROBOT_TYPE == Hero_robot 
#include "Hero.h"
#endif
#if ROBOT_TYPE == Infantry_robot
#include "Infantry_robot.h"
#endif
#if ROBOT_TYPE == Balance_robot 
#include "Balance.h"
#endif
#if ROBOT_TYPE == Sentinel_robot
#include "Sentinel.h"
#endif
#if ROBOT_TYPE == Engineer_robot
#include "Engineer.h"
#endif


//#ifndef MOTOR_ECD_TO_RAD
//#define MOTOR_ECD_TO_RAD              0.000766990394f
//#endif

//#define INIT_YAW_SET    0.0f
//#define INIT_PITCH_SET  0.0f
//#define ecd_format(ecd)               \
//    do {                              \
//        if ((ecd) > ECD_RANGE)        \
//            (ecd) -= ECD_RANGE;       \
//        else if ((ecd) < 0)           \
//            (ecd) += ECD_RANGE;       \
//    } while (0)

//#define gimbal_total_pid_clear(gimbal_clear)                                                                    \
//    do {                                                                                                        \
//        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);                    \
//        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);                    \
//        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                                     \
//        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid);                  \
//        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid);                  \
//        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                                   \
//    } while (0)
/**
  * @brief          remote control dealline solve,because the value of rocker is not zero in middle place,
  * @param          input:the raw channel value 
  * @param          output: the processed channel value
  * @param          deadline
  */
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/* ================= 外部声明 ================= */
//小陀螺转速，在底盘任务中会进行修改
extern fp32 SPIN_FACTOR;
extern fp32 SPIN_SPEED;

#endif

