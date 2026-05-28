#ifndef MULTI_AXIS_ROBOTIC_ARM_H
#define MULTI_AXIS_ROBOTIC_ARM_H
#include "struct_typedef.h"
#include "robot_param.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#if ROBOT_GIMBAL == multi_axis_robotic_arm
#include "can_bsp.h"
#include "pid.h"
#include "remote_control.h"

#include "INS_task.h"

#define testmode 1
#ifndef ARM_JOINT_NUM
#define ARM_JOINT_NUM 8
#endif
// 电机校准使用
#ifndef ARM_KP_DEFAULT
#define ARM_KP_DEFAULT (20.0f)
#endif
#ifndef ARM_KD_DEFAULT
#define ARM_KD_DEFAULT (1.5f)
#endif

//关节角度定义
//零位偏移
#define J0_ZERO_ANGLE (0.150f)
#define J1_ZERO_ANGLE (2.850f)
#define J2_ZERO_ANGLE (-4.27f)
#define J3_ZERO_ANGLE (3.14f)
#define J4_ZERO_ANGLE (2.72f)
#define J5_ZERO_ANGLE (2.64f)
#define J6_ZERO_ANGLE (2.4f)
#define J7_ZERO_ANGLE (2.114f)
//最大最小角度
#define J0_MAX_ANGLE (J0_ZERO_ANGLE+0.75f)
#define J1_MAX_ANGLE (J1_ZERO_ANGLE +1.0f)
#define J2_MAX_ANGLE (J2_ZERO_ANGLE+4.2f)
#define J3_MAX_ANGLE (J3_ZERO_ANGLE+2.1f)
#define J4_MAX_ANGLE (J4_ZERO_ANGLE+2.4f)
#define J5_MAX_ANGLE (J5_ZERO_ANGLE+2.35f)
#define J6_MAX_ANGLE (2.85f)
#define J7_MAX_ANGLE (J7_ZERO_ANGLE +1.0f)

#define J0_MIN_ANLE (J0_ZERO_ANGLE-1.0f)
#define J1_MIN_ANLE (J1_ZERO_ANGLE -1.5f)
#define J2_MIN_ANLE (J2_ZERO_ANGLE + 1.2f)
#define J3_MIN_ANLE (J3_ZERO_ANGLE-2.1f)
#define J4_MIN_ANLE (J4_ZERO_ANGLE-2.4f)
#define J5_MIN_ANLE (J5_ZERO_ANGLE-2.35f)
#define J6_MIN_ANLE (0.45f)
#define J7_MIN_ANLE (J7_ZERO_ANGLE -1.0f)
//上电角度
#define J0_INIT_ANGLE (0.15f)
#define J1_INIT_ANGLE (1.440f)
#define J2_INIT_ANGLE (-0.15f)
#define J3_INIT_ANGLE (3.14f)
#define J4_INIT_ANGLE (2.72f)
#define J5_INIT_ANGLE (2.64f)
#define J6_INIT_ANGLE ((J6_MAX_ANGLE+J6_MIN_ANLE)/2)
#define J7_INIT_ANGLE (2.114f)//大yaw

#define GAR_GAIN0 10
#define GAR_GAIN1 1.0
#define GAR_GAIN2 3.0
#define GAR_GAIN3 8
#define GAR_GAIN4 10
#define GAR_GAIN5 10
#define GAR_GAIN6 10
//电机PID参数
//注意：在电机设置的POS不为0时，KD也应不为0.否则会导致电机震荡
#ifndef J0_KP
#define J0_KP (12.0f)
#endif
#ifndef J1_KP
#define J1_KP (5.0f)
#endif
#ifndef J2_KP
#define J2_KP (12.5f)
#endif
#ifndef J3_KP
#define J3_KP (3.0f)
#endif
#ifndef J4_KP
#define J4_KP (4.4f)
#endif
#ifndef J5_KP
#define J5_KP (3.2f)
#endif
#ifndef J6_KP
#define J6_KP (0.0f)
#endif
#ifndef J7_KP
#define J7_KP (10.0f)
#endif

#ifndef J0_KD
#define J0_KD (2.0f)
#endif
#ifndef J1_KD
#define J1_KD (0.88f)
#endif
#ifndef J2_KD
#define J2_KD (3.2f)
#endif
#ifndef J3_KD
#define J3_KD (0.5f)
#endif
#ifndef J4_KD
#define J4_KD (0.6f)
#endif
#ifndef J5_KD
#define J5_KD (0.32f)
#endif
#ifndef J6_KD
#define J6_KD (0.15f)
#endif
#ifndef J7_KD
#define J7_KD (3.0f)
#endif
#define ARM_INT_KI_TABLE    { \
    0.0f,  /* J0 */ \
    20.5f,  /* J1 */ \
    50.5f,  /* J2 */ \
    30.5f,  /* J3 */ \
    30.5f,  /* J4，先重点测试 */ \
    30.5f,  /* J5 */ \
    0.0f,  /* J6 */ \
    0.0f   /* J7 */ \
}

#define ARM_INT_LIMIT_TABLE { \
    0.0f,  /* J0 */ \
    0.8f,  /* J1 */ \
    7.0f,  /* J2 */ \
    6.0f,  /* J3 */ \
    6.0f,  /* J4 */ \
    6.0f,  /* J5 */ \
    0.0f,  /* J6 */ \
    0.0f   /* J7 */ \
}
//超时时间
#define TIMEOUT 100
//机械臂电机初始化速度
#define INIT_VEL 2.0f
//机械臂电机片选控制通道
#define ARM_RC_MODE_CHANNEL         1
//机械臂电机控制通道
#define HORI_CHANNEL                2
#define VERT_CHANNEL                3

#define ARM_SEMI_AUTO_ENABLE        1
#define ARM_SEMI_AUTO_STEP        0.001f   // m，单步平移距离
#define ARM_SEMI_AUTO_STEP_ANG      0.05236f // rad，单步旋转角，约3度
#define ARM_SEMI_AUTO_TRAJ_DS       0.06f    // 每控制周期轨迹进度增量，越小越平滑
#define ARM_SEMI_AUTO_THETA_ALPHA   0.4f
#define ARM_SEMI_AUTO_MIN_TIME      0.08f
#define ARM_SEMI_AUTO_MAX_LIN_VEL   0.05f
#define ARM_SEMI_AUTO_MAX_ANG_VEL   1.2f
#define ARM_SEMI_AUTO_JOINT_VEL_LIMIT_TABLE {1.2f, 1.0f, 1.2f, 1.6f, 1.6f, 1.6f}

#define ARM_SELF_ALPHA_NORMAL      0.012f    // SELF 正常跟手滤波系数
#define ARM_SELF_ALPHA_RECONNECT   0.008f   // 切回 SELF 后前几拍更柔和
#define ARM_SELF_RECONNECT_TICKS   400U     // 回接柔化持续周期数
typedef enum
{
  ARM_NONE,
  ARM_HOLD,
  ARM_RC,
  ARM_SELF,
  ARM_SEMI_AUTO,
  ARM_TEACH_DRAG,
  ARM_TEACH_RECORD,
  ARM_TEACH_PLAY,
  ARM_INIT
}arm_joint_e;
typedef __packed struct 
{   
    uint8_t online;     //自定义控制器在线否
    uint8_t claw;   //夹爪开关（0or1）
    float j5;           //大臂末端电机
    float j4;           //肘关节电机
    float j3;            //大抬升电机
    float j2;        //PITCH电机
    float j1;         //ROLL电机
    float j0;           //YAW电机
	uint8_t res[4];        // 包序号
}moterMapHeader;               //电机映射数据		26字节

extern moterMapHeader motor_data;
extern void gimbal_init(gimbal_control_t *init);		
extern void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change);
extern void gimbal_set_mode(gimbal_control_t *set_mode);
extern void gimbal_feedback_update(gimbal_control_t *feedback_update);
extern void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change);
extern void gimbal_set_control(gimbal_control_t *set_control);
extern void gimbal_control_loop(gimbal_control_t *control_loop);
extern void gimbal_send_cmd(gimbal_control_t *control_send);
int Unpack_CustomController_Frame_Simple(const uint8_t *data, moterMapHeader *out);
/**
  * @brief          锟斤拷台校准锟斤拷锟姐，锟斤拷校准锟斤拷录锟斤拷锟斤拷值,锟斤拷锟?锟斤拷小值锟斤拷锟斤拷
  * @param[out]     yaw 锟斤拷值 指锟斤拷
  * @param[out]     pitch 锟斤拷值 指锟斤拷
  * @param[out]     yaw 锟斤拷锟斤拷锟皆角讹拷 指锟斤拷
  * @param[out]     yaw 锟斤拷小锟斤拷越嵌锟?指锟斤拷
  * @param[out]     pitch 锟斤拷锟斤拷锟皆角讹拷 指锟斤拷
  * @param[out]     pitch 锟斤拷小锟斤拷越嵌锟?指锟斤拷
  * @retval         锟斤拷锟斤拷1 锟斤拷锟斤拷锟缴癸拷校准锟斤拷希锟?锟斤拷锟斤拷0 锟斤拷锟斤拷未校准锟斤拷
  * @waring         锟斤拷锟斤拷锟斤拷锟绞癸拷玫锟絞imbal_control 锟斤拷态锟斤拷锟斤拷锟斤拷锟铰猴拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷通锟斤拷指锟诫复锟斤拷
  */
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

/**
  * @brief          gimbal cali data, set motor offset encode, max and min relative angle
  * @param[in]      yaw_offse:yaw middle place encode
  * @param[in]      pitch_offset:pitch place encode
  * @param[in]      max_yaw:yaw max relative angle
  * @param[in]      min_yaw:yaw min relative angle
  * @param[in]      max_yaw:pitch max relative angle
  * @param[in]      min_yaw:pitch min relative angle
  * @retval         none
  */
/**
  * @brief          锟斤拷台校准锟斤拷锟矫ｏ拷锟斤拷校准锟斤拷锟斤拷台锟斤拷值锟皆硷拷锟斤拷小锟斤拷锟斤拷械锟斤拷越嵌锟?
  * @param[in]      yaw_offse:yaw 锟斤拷值
  * @param[in]      pitch_offset:pitch 锟斤拷值
  * @param[in]      max_yaw:max_yaw:yaw 锟斤拷锟斤拷锟皆角讹拷
  * @param[in]      min_yaw:yaw 锟斤拷小锟斤拷越嵌锟?
  * @param[in]      max_yaw:pitch 锟斤拷锟斤拷锟皆角讹拷
  * @param[in]      min_yaw:pitch 锟斤拷小锟斤拷越嵌锟?
  * @retval         锟斤拷锟截匡拷
  * @waring         锟斤拷锟斤拷锟斤拷锟绞癸拷玫锟絞imbal_control 锟斤拷态锟斤拷锟斤拷锟斤拷锟铰猴拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷通锟斤拷指锟诫复锟斤拷
  */
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);

void gra_theta_calcu(gimbal_control_t *pos_angle, float *position_calcu);
#endif


#endif
