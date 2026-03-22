#include "Multi-axis_robotic_arm.h"
#include "cmsis_os.h"
#include "crc8_crc16.h"
#if ROBOT_GIMBAL == multi_axis_robotic_arm
/*
机械臂电机选型
                    大yaw：	RS00
					J0： RS03
                    J1： RS04
                    J2： DM4340
                    J3： RS00
                    J4： RS05
                    J5： RS05
                    末端：RS05

机械臂相关的一些方向定义
	- J1正朝向前方时，设置大yaw为0位置，零点为：-0.787，从上往下看，顺时针为负，逆时针为正，最大为正负4π，旋转180要走过9.86
    - 定义机械臂水平向前时的J0关节位置为0，，零点为0.3，从上往下看，逆时针为正方向
    - 定义机械臂竖直向上时的J1 J2关节位置为0，J1零位置（4.2），J2零位置（-3.42），从机械臂右侧看，逆时针为正方向（注：J1 J2关节的合位置为联动位置）
    - 定义机械臂J3水平(同步带位于两侧)时的J3关节位置为0(0.2)，从吸盘方向看，逆时针为正方向，一圈约6.28（2π）
    - 定义J4为末端机构右侧（上视，J3向前）电机，J5为末端机构左侧电机,爪子向前为0（3.75），从右侧看，逆时针为正，一圈约6.28（2π）

    - 定义虚拟J4关节用来衡量末端机构的pitch, 虚拟J5关节用来衡量末端机构的roll
    - 定义J4正方向为：当J3归中时，从机械臂右侧看，逆时针为正方向
    - 定义J5正方向为：当J3归中时，吸盘方向看，0位置为2.06，逆时针为负，顺时针为正，一圈为6.28（2π）
    - vj4和j4 j5的关系：vj4 = (j4 - j5)/2
	- 定义夹爪刚好卡着能力单元不施加力时为0位置（3.58），张开为正，夹紧为负；空载极限位置2.4，张开极限位置4.8

机械臂控制
    左拨杆 
          上：自定义控制器跟随模式
          中：安全模式
          下：RC模式
    右拨杆
          上：J1转动(rs04)
          中：J2转动(4340)
          下：J3转动(rs00)
*/
#include <string.h>
#include "remote_control.h"
#include "INS_task.h"
#include "can_bsp.h"
#include "arm_math.h"
#include "tim.h"
#include "kinematics.h"
// MIT command IDs (slave IDs) for each joint; adjust to wiring
#ifndef ARM_MIT_CMD_ID0
#define ARM_MIT_CMD_ID0 0x00
#define ARM_MIT_CMD_ID1 0x01
#define ARM_MIT_CMD_ID2 0x02
#define ARM_MIT_CMD_ID3 0x03
#define ARM_MIT_CMD_ID4 0x04
#define ARM_MIT_CMD_ID5 0x05
#define ARM_MIT_CMD_ID6 0x06
#define ARM_MIT_CMD_ID7 0x07
#endif

#define JOINT_TORQUE_MORE_OFFSET              ((uint8_t)1 << 0)  // 关节电机输出力矩过大偏移量
#define CUSTOM_CONTROLLER_DATA_ERROR_OFFSET   ((uint8_t)1 << 1)  // 自定义控制器数据异常偏移量
#define DBUS_ERROR_OFFSET    ((uint8_t)1 << 2)  // dbus错误偏移量
#define IMU_ERROR_OFFSET     ((uint8_t)1 << 3)  // imu错误偏移量
#define FLOATING_OFFSET      ((uint8_t)1 << 4)  // 悬空状态偏移量
#define MOTOR_OFFLINE      ((uint8_t)1 << 5)  // 电机掉线
#define RAPAM_ERROR      ((uint8_t)1 << 6)  // 设置参数错误


moterMapHeader motor_data;
arm_joint_e ARM_JOINT_BEHAVIOUR;
typedef enum
{
    ARM_MODE_IDLE = 0,
    ARM_MODE_HOLD,
    ARM_MODE_RC,
    ARM_MODE_SELF
} arm_mode_e;

static const uint16_t arm_cmd_id_table[ARM_JOINT_NUM] = {
    ARM_MIT_CMD_ID0, ARM_MIT_CMD_ID1, ARM_MIT_CMD_ID2,
    ARM_MIT_CMD_ID3, ARM_MIT_CMD_ID4, ARM_MIT_CMD_ID5,
	ARM_MIT_CMD_ID6, ARM_MIT_CMD_ID7};
static const fp32 motor_limit_table[ARM_JOINT_NUM][3] = {
    {J0_MIN_ANLE, J0_MAX_ANGLE, J0_ZERO_ANGLE},
    {J1_MIN_ANLE, J1_MAX_ANGLE, J1_ZERO_ANGLE},
    {J2_MIN_ANLE, J2_MAX_ANGLE, J2_ZERO_ANGLE},
    {J3_MIN_ANLE, J3_MAX_ANGLE, J3_ZERO_ANGLE},
    {J4_MIN_ANLE, J4_MAX_ANGLE, J4_ZERO_ANGLE},
    {J5_MIN_ANLE, J5_MAX_ANGLE, J5_ZERO_ANGLE},
    {J6_MIN_ANLE, J6_MAX_ANGLE, J6_ZERO_ANGLE},
    {J7_MIN_ANLE, J7_MAX_ANGLE, J7_ZERO_ANGLE},
};

fp32 q_angle[ARM_JOINT_NUM];
uint8_t MOTER_InitAngleright;
//舵机ccr
uint16_t pwm_ccr_set = 1700;
uint32_t last_sec_recv, total_los;
//重力补偿用变量
float position_calcu[8];
float tauqe_calculated[6];
float theta_calculated[6];//变换得到的计算用运动学角
//运动学正逆解变量（等测试好以后换成静态）
float g_theta_kin_cur[6];
float g_theta_kin_des[6];
float g_T_tool_cur[16];
float g_T_tool_goal[16];

uint8_t g_have_tool_goal = 0;
uint8_t g_semi_auto_latched = 0;
uint8_t g_semi_auto_ik_valid = 0;
float g_motor_pos_des[6];

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void kin_theta_to_motor_pos(const float theta_kin[6], float motor_pos[6])
{
    motor_pos[0] = theta_kin[0] - PI / 2.0f + J0_ZERO_ANGLE;
    motor_pos[1] = J1_ZERO_ANGLE - PI / 2.0f - theta_kin[1];
    motor_pos[2] = J2_ZERO_ANGLE - 1.4446f * (theta_kin[2] + PI / 2.0f);
    motor_pos[3] = J3_ZERO_ANGLE - theta_kin[3];
    motor_pos[4] = J4_ZERO_ANGLE - theta_kin[4];
    motor_pos[5] = J5_ZERO_ANGLE + theta_kin[5];
}

//自定义遥控器或DT7输入的数据
void arm_update_control(gimbal_control_t *add_angle);
void Initial_position_safety_check(gimbal_control_t *position);

//更新数据
__attribute__((used)) void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    (void)feedback_update;
    //从顶部开始计算重力补偿力矩
     for(int16_t i = ARM_JOINT_NUM - 1; i >= 0; i--)
    {
        //检测电机是否在线
        if(HAL_GetTick() - feedback_update->joint_motor[i].motor_measure->last_fdb_time > TIMEOUT)
        {
            feedback_update->joint_motor[i].online = OFFLINE;
            feedback_update->error_code |= MOTOR_OFFLINE;
        }
        else
        {
            feedback_update->joint_motor[i].online = ONLINE;
        }
		//计算零点偏移角度
        q_angle[i] = feedback_update->joint_motor[i].motor_measure->pos - feedback_update->joint_motor[i].zero_offset;
		if(i == 2){q_angle[i] = q_angle[i]/1.4446f;}
    }
	
	//计算当前重力补偿
	gra_theta_calcu(feedback_update, theta_calculated);
	/* 当前6轴运动学关节角 */
    for (uint8_t i = 0; i < 6; i++)
    {
        g_theta_kin_cur[i] = theta_calculated[i];
    }

    /* 正运动学：当前工具末端位姿 */
    compute_tool_pose(g_theta_kin_cur, g_T_tool_cur);
	//是否夹取能量单元判断
	if(motor_data.claw & (feedback_update->joint_motor[6].motor_measure->tor <= -15.0f)  )
	{
		feedback_update->Isgrasped = 1;
	}else
	{
		feedback_update->Isgrasped = 0;
	}
	gravity_compensation(theta_calculated,tauqe_calculated);
	//逆运动学
//	compute_desired_joint_angles(g_T_tool_cur, g_theta_kin_cur, g_theta_kin_des);			
    //遥控器掉线检测
    if((HAL_GetTick() - feedback_update->gimbal_rc_ctrl->last_fdb) > TIMEOUT)
    {
        feedback_update->error_code |= DBUS_ERROR_OFFSET;
    }
    //电机力矩过大检测
    if(feedback_update->joint_motor[0].motor_measure->tor > 15.0f || feedback_update->joint_motor[1].motor_measure->tor > 15.0f)
    {
        feedback_update->error_code |= JOINT_TORQUE_MORE_OFFSET;
    }
    //参数检测,若位置不为0，但是KD为0，会导致电机震荡
    for(uint8_t i = 0 ; i < ARM_JOINT_NUM; i++)
    {
        if(!feedback_update->joint_pid[i].Kd && !feedback_update->multi_arm_set[i][0])
        {
            feedback_update->error_code |= RAPAM_ERROR;
        }
    }
    //初始化检测，速度低于阈值之后开始计时
    if(ARM_JOINT_BEHAVIOUR == ARM_INIT)
    {
        if(fabsf(feedback_update->joint_motor[0].motor_measure->vel) < INIT_VEL * 0.6f &&
           fabsf(feedback_update->joint_motor[0].motor_measure->vel) < INIT_VEL * 0.6f  )
        {
            feedback_update->init_time++;
        }
        else
        {
            feedback_update->init_time = 0;
        }
    }
        
}
__attribute__((used)) void gimbal_init(gimbal_control_t *init)
{
    (void)init;
    osDelay(1000);
    static fp32 pid_table[ARM_JOINT_NUM][3] = 
    {
        J0_KP, 0.0f, J0_KD,
        J1_KP, 0.0f, J1_KD,
        J2_KP, 0.0f, J2_KD,
        J3_KP, 0.0f, J3_KD,
        J4_KP, 0.0f, J4_KD,
        J5_KP, 0.0f, J5_KD,
		J6_KP, 0.0f, J6_KD,
		J7_KP, 0.0f, J7_KD,
    };
	static fp32 arm_link_params[ARM_JOINT_NUM][2] = 
    {
        J0_length, J0_weight * GRAVITY, 
        J1_length, J1_weight * GRAVITY,
        J2_length, J2_weight * GRAVITY,
        J3_length, J3_weight * GRAVITY,
        J4_length, J4_weight * GRAVITY,
        J5_length, J5_weight * GRAVITY,
		J6_length, J6_weight * GRAVITY,
		J7_length, J7_weight * GRAVITY,
    };
    //获取遥控器数据
    init->gimbal_rc_ctrl = get_remote_control_point();
    //获取IMU的数据
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    init->init_flag = 0;
    init->init_time = 0;
    //电机初始化
    for(uint8_t i = 0; i <  ARM_JOINT_NUM; i++)
    {
         init->joint_motor[i].online = ONLINE;
        //获取电机结构体指针
        init->joint_motor[i].motor_measure = get_mit_motor_measure_point(i);
		init->joint_motor[i].motor_measure = get_mit_motor_measure_point(i);
		//获取电机物理参数
		init->multi_arm_params[i][0] = arm_link_params[i][0];
		init->multi_arm_params[i][1] = arm_link_params[i][1];
        //初始化电机PID,不需要使用这个PID计算，只需要发送即可
        PID_init(&init->joint_pid[i], PID_POSITION, pid_table[i], 0.0f, 0.0f);
			
		//设置电机限幅
		init->joint_motor[i].min_angle = motor_limit_table[i][0];
        init->joint_motor[i].max_angle = motor_limit_table[i][1];
		init->joint_motor[i].zero_offset = motor_limit_table[i][2];
        //init->joint_motor[i].zero_offset = (init->joint_motor[i].max_angle + init->joint_motor[i].min_angle) / 2;		//启用自校准时使用
        //电机使能
        if(init->joint_motor[i].motor_measure->id == 0x00)
        {
			Motor_enable(&GIMBAL_CAN, i);
			Motor_enable(&END_CAN, i);
//           Motor_enable(&GIMBAL_CAN, i + 1);
        }
        else
        {
			Motor_enable(&GIMBAL_CAN, init->joint_motor[i].motor_measure->id);
			Motor_enable(&END_CAN, init->joint_motor[i].motor_measure->id);
//            Motor_enable(&GIMBAL_CAN, init->joint_motor[i].motor_measure->id + 1);
        }
				//初始化set
        init->multi_arm_set[i][0] = init->multi_arm_set[i][1] = init->multi_arm_set[i][2] = 0.0f;
				init->multi_arm_cmd[i][0] = init->multi_arm_cmd[i][1] = init->multi_arm_cmd[i][2] = 0.0f;
    }
	//检测电机上电位置有无突变
	MOTER_InitAngleright = 1;
	Initial_position_safety_check(init);
	//打开舵机pwm
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    //gimbal_feedback_update(init);
    
}
//设置机械臂模式，获取控制值
__attribute__((used)) void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (!set_mode)
    {
        ARM_JOINT_BEHAVIOUR = ARM_NONE;
        return;
    }
	if(!MOTER_InitAngleright)
	{
        ARM_JOINT_BEHAVIOUR = ARM_NONE;
        return;
    }
    //开关控制 云台状态
    if (switch_is_mid(set_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) )
    {
        ARM_JOINT_BEHAVIOUR = ARM_NONE;
    }
	else if(switch_is_down(set_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
	{
		ARM_JOINT_BEHAVIOUR = ARM_SELF;
	}
    else if (switch_is_up(set_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        ARM_JOINT_BEHAVIOUR = ARM_RC;
    }
    //有错误马上关
//    if(set_mode->error_code)
//    {
//        ARM_JOINT_BEHAVIOUR = ARM_NONE;
//    }

//    //如果模式不是无力模式，初始化未完成进入初始化模式
//    if(ARM_JOINT_BEHAVIOUR != ARM_NONE && !set_mode->init_flag )
//    {
//        ARM_JOINT_BEHAVIOUR = ARM_INIT;
//    }
    //完成初始化之后，初始化标志位置1，退出初始化模式
    if(ARM_JOINT_BEHAVIOUR == ARM_INIT && set_mode->init_time > 200)
    {
        set_mode->init_flag = 1;
    }
	//半自动模式判断
	#if ARM_SEMI_AUTO_ENABLE
    {
        static uint8_t last_ctrl_key = 0;
        uint8_t ctrl_key_now = ((set_mode->gimbal_rc_ctrl->key.v & ARM_SEMI_AUTO_KEY) != 0U);

        /* 只有在 SELF 档下，CTRL 才允许切换半自动 */
        if (ARM_JOINT_BEHAVIOUR != ARM_SELF && ARM_JOINT_BEHAVIOUR != ARM_SEMI_AUTO)
        {
            g_semi_auto_latched = 0U;
        }

        /* CTRL 上升沿：翻转半自动状态 */
        if (ctrl_key_now && !last_ctrl_key)
        {
            if (ARM_JOINT_BEHAVIOUR == ARM_SELF || ARM_JOINT_BEHAVIOUR == ARM_SEMI_AUTO)
            {
                g_semi_auto_latched = !g_semi_auto_latched;
            }
        }

        last_ctrl_key = ctrl_key_now;

        /* 锁存有效时，覆盖为半自动模式 */
        if (g_semi_auto_latched)
        {
            ARM_JOINT_BEHAVIOUR = ARM_SEMI_AUTO;
        }
    }
	#endif
    //获取不同模式的控制值
    arm_update_control(set_mode);
}
//切换状态机
__attribute__((used)) void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    (void)gimbal_mode_change;
    static arm_joint_e last_ARM_JOINT_BEHAVIOUR;
    //模式切换后全部控制数据清空,防止出现奇奇怪怪的数值
    if(last_ARM_JOINT_BEHAVIOUR != ARM_NONE && ARM_JOINT_BEHAVIOUR == ARM_NONE)
    {
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            //角度设为0.0,速度也是0
            gimbal_mode_change->multi_arm_set[i][0] = 0.0f;
            gimbal_mode_change->multi_arm_set[i][1] = 0.0f;
        }
    }
    if(last_ARM_JOINT_BEHAVIOUR != ARM_RC && ARM_JOINT_BEHAVIOUR == ARM_RC)
    {
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            //角度设为0.0,速度也是0
            gimbal_mode_change->multi_arm_set[i][0] = 0.0f;
            gimbal_mode_change->multi_arm_set[i][1] = 0.0f;
        }
    }
    if(last_ARM_JOINT_BEHAVIOUR != ARM_SELF && ARM_JOINT_BEHAVIOUR == ARM_SELF)
    {
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            //角度设为0.0,速度也是0
            gimbal_mode_change->multi_arm_set[i][0] = 0.0f;
            gimbal_mode_change->multi_arm_set[i][1] = 0.0f;
        }
    }
	if(last_ARM_JOINT_BEHAVIOUR != ARM_SEMI_AUTO && ARM_JOINT_BEHAVIOUR == ARM_SEMI_AUTO)
    {
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            gimbal_mode_change->multi_arm_set[i][0] = 0.0f;
            gimbal_mode_change->multi_arm_set[i][1] = 0.0f;
        }

        /* 进入半自动时，锁存当前工具位姿为目标位姿 */
        for (uint8_t i = 0; i < 16; i++)
        {
            g_T_tool_goal[i] = g_T_tool_cur[i];
        }
        g_have_tool_goal = 1U;
        g_semi_auto_ik_valid = 0U;
    }
	if(last_ARM_JOINT_BEHAVIOUR == ARM_SEMI_AUTO && ARM_JOINT_BEHAVIOUR != ARM_SEMI_AUTO)
	{
		g_have_tool_goal = 0U;
		g_semi_auto_ik_valid = 0U;
	}
	
	last_ARM_JOINT_BEHAVIOUR = ARM_JOINT_BEHAVIOUR;
	
}

//计算控制量set_control->multi_arm_set,不操作的话直接return
__attribute__((used)) void gimbal_set_control(gimbal_control_t *set_control)
{
    (void)set_control;
    //任务执行周期
    static fp32 dt = GIMBAL_CONTROL_TIME /1000.0f;
    if(ARM_JOINT_BEHAVIOUR == ARM_NONE)
    {
        return;
    }
    else if(ARM_JOINT_BEHAVIOUR == ARM_HOLD)
    {
        return;
    }
    else if(ARM_JOINT_BEHAVIOUR == ARM_INIT)
    {
        return;
    }
    else if(ARM_JOINT_BEHAVIOUR == ARM_RC)
    {
         // for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        // {
        //     //速度为0，此时电机KD纯阻尼
        //     set_control->multi_arm_set[i][1] = 0.0f;
        // }
        for(uint8_t i = 0; i < ARM_JOINT_NUM-1; i++)
        {
			
            //限幅
            //set_control->multi_arm_set[i][0] = clampf(set_control->multi_arm_set[i][0], set_control->joint_motor[i].min_angle, set_control->joint_motor[i].max_angle);
			
			set_control->multi_arm_cmd[i][1] = set_control->multi_arm_set[i][1];
			set_control->multi_arm_cmd[i][0] = clampf(set_control->multi_arm_set[i][0] + set_control->joint_motor[i].motor_measure->pos, set_control->joint_motor[i].min_angle, set_control->joint_motor[i].max_angle);
        }
		#if GRAVITY_COMPENSATION_ENABLE
			set_control->multi_arm_cmd[1][2] = tauqe_calculated[1] * GAR_GAIN1;
			set_control->multi_arm_cmd[2][2] = tauqe_calculated[2] * GAR_GAIN2;
			set_control->multi_arm_cmd[3][2] = tauqe_calculated[3] * GAR_GAIN3;
			set_control->multi_arm_cmd[4][2] = tauqe_calculated[4] * GAR_GAIN4;	//重力补偿增益
		#endif
		set_control->multi_arm_cmd[7][1] = set_control->multi_arm_set[7][1];
		set_control->multi_arm_cmd[7][0] = clampf(set_control->multi_arm_set[7][0], set_control->joint_motor[7].min_angle, set_control->joint_motor[7].max_angle);
        
    }
    else if(ARM_JOINT_BEHAVIOUR == ARM_SELF)
    {
		if(!motor_data.online)
		{
			for(uint8_t i = 0; i < ARM_JOINT_NUM-1; i++)
			{
				
				//限幅
				//set_control->multi_arm_set[i][0] = clampf(set_control->multi_arm_set[i][0], set_control->joint_motor[i].min_angle, set_control->joint_motor[i].max_angle);
				
				set_control->multi_arm_cmd[i][1] = set_control->multi_arm_set[i][1];
				set_control->multi_arm_cmd[i][0] = clampf(set_control->joint_motor[i].motor_measure->pos, set_control->joint_motor[i].min_angle, set_control->joint_motor[i].max_angle);
			}
			#if GRAVITY_COMPENSATION_ENABLE
			//重力补偿增益
				set_control->multi_arm_cmd[1][2] = tauqe_calculated[1] * GAR_GAIN1;
				set_control->multi_arm_cmd[2][2] = tauqe_calculated[2] * GAR_GAIN2;
				set_control->multi_arm_cmd[3][2] = tauqe_calculated[3] * GAR_GAIN3;
				set_control->multi_arm_cmd[4][2] = tauqe_calculated[4] * GAR_GAIN4;
			
			#endif
			set_control->multi_arm_cmd[7][1] = set_control->multi_arm_set[7][1];
			set_control->multi_arm_cmd[7][0] = clampf(set_control->multi_arm_set[7][0], set_control->joint_motor[7].min_angle, set_control->joint_motor[7].max_angle);
		}
		else
		{
			// SELF模式：直接使用motor_data的目标位置（已经是绝对位置）
			for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
			{
				//set_control->multi_arm_cmd[i][2] = set_control->multi_arm_set[i][2];	//重力补偿
				set_control->multi_arm_cmd[i][1] = set_control->multi_arm_set[i][1];
				set_control->multi_arm_cmd[i][0] = clampf(set_control->multi_arm_set[i][0], set_control->joint_motor[i].min_angle, set_control->joint_motor[i].max_angle);
			}
			#if GRAVITY_COMPENSATION_ENABLE
			//重力补偿增益
			set_control->multi_arm_cmd[1][2] = tauqe_calculated[1] * GAR_GAIN1;
			set_control->multi_arm_cmd[2][2] = tauqe_calculated[2] * GAR_GAIN2;
			set_control->multi_arm_cmd[3][2] = tauqe_calculated[3] * GAR_GAIN3;
			set_control->multi_arm_cmd[4][2] = tauqe_calculated[4] * GAR_GAIN4;
			#endif
		}
    }
	else if(ARM_JOINT_BEHAVIOUR == ARM_SEMI_AUTO)
    {
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            set_control->multi_arm_cmd[i][1] = set_control->multi_arm_set[i][1];
            set_control->multi_arm_cmd[i][0] = clampf(set_control->multi_arm_set[i][0],
                                                      set_control->joint_motor[i].min_angle,
                                                      set_control->joint_motor[i].max_angle);
        }
		#if GRAVITY_COMPENSATION_ENABLE
        /* 重力补偿继续存在 */
        set_control->multi_arm_cmd[1][2] = tauqe_calculated[1] * GAR_GAIN1;
        set_control->multi_arm_cmd[2][2] = tauqe_calculated[2] * GAR_GAIN2;
        set_control->multi_arm_cmd[3][2] = tauqe_calculated[3] * GAR_GAIN3;
        set_control->multi_arm_cmd[4][2] = tauqe_calculated[4] * GAR_GAIN4;
		#endif
    }
	if(set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V)
	{
		pwm_ccr_set += 1;
		if(pwm_ccr_set >= 2300)pwm_ccr_set = 2300;
		if(pwm_ccr_set <= 900)pwm_ccr_set = 900;
	}
	else if(set_control->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B)
	{
		pwm_ccr_set -= 1;
		if(pwm_ccr_set >= 2300)pwm_ccr_set = 2300;
		if(pwm_ccr_set <= 900)pwm_ccr_set = 900;
	}
	else
	{
		return;
	}

}
__attribute__((used)) void gimbal_send_cmd(gimbal_control_t *control_send)
{
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwm_ccr_set);
    //电机离线或者控制模式设置
//    if(ARM_JOINT_BEHAVIOUR ==ARM_NONE)
//    {
//        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
//        {
//            CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
//        }
//    }
//    //保持机械臂姿态，方便测出限位位置
//    else if (ARM_JOINT_BEHAVIOUR == ARM_HOLD)
//    {
//        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
//        {
//            CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[i], 0.0f, 0.0f, 0.0f, 0.0f, control_send->multi_arm_set[i][2]);
//        }
//    }
//    else if(ARM_JOINT_BEHAVIOUR == ARM_SELF)
//    {
//        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
//        {
//            CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[i], control_send->multi_arm_set[i][0], control_send->multi_arm_set[i][1], control_send->joint_pid[i].Kp, control_send->joint_pid[i].Kd, control_send->multi_arm_set[i][2]);
//        }
//    }
//    else if(ARM_JOINT_BEHAVIOUR == ARM_RC)
//    {
//        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
//        {
//            CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[i], control_send->multi_arm_set[i][0], control_send->multi_arm_set[i][1], control_send->joint_pid[i].Kp, control_send->joint_pid[i].Kd, control_send->multi_arm_set[i][2]);
//        }
//    }
//    else if(ARM_JOINT_BEHAVIOUR == ARM_INIT)
//    {
//        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
//        {
//            CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[i], 0.0f, control_send->multi_arm_set[i][1], 0.0f, ARM_KD_DEFAULT, control_send->multi_arm_set[i][2]);
//        }
//    }

    if(ARM_JOINT_BEHAVIOUR ==ARM_NONE)
    {
				CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[7], 0.0f, 0.0f, 0.0f, control_send->joint_pid[7].Kd, 0.0f);
				CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[0], 0.0f, 0.0f, 0.0f, control_send->joint_pid[0].Kd, 0.0f);
				CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[1], 0.0f, 0.0f, 0.0f, 3*control_send->joint_pid[1].Kd, 0.0f);
				CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[2], 0.0f, 0.0f, 0.0f, 6*control_send->joint_pid[2].Kd, 0.0f);
				//上端4电机换can3
//				RS_MOTOR_PRE(&END_CAN, arm_cmd_id_table[3]);
//		RS_MOTOR_PRE(&END_CAN, arm_cmd_id_table[5]);
//		RS_MOTOR_PRE(&GIMBAL_CAN, arm_cmd_id_table[7]);
				CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[3], 0.0f, 0.0f, 0.0f, control_send->joint_pid[3].Kd, 0.0f);
				CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[4], 0.0f, 0.0f, 0.0f, 4*control_send->joint_pid[4].Kd, 0.0f);
				CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[5], 0.0f, 0.0f, 0.0f, control_send->joint_pid[5].Kd, 0.0f);
				CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[6], 0.0f, 0.0f, 0.0f, control_send->joint_pid[6].Kd, 0.0f);
	}

		else if(ARM_JOINT_BEHAVIOUR == ARM_RC)
    {           
			CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[7], control_send->multi_arm_cmd[7][0], control_send->multi_arm_cmd[7][1], control_send->joint_pid[7].Kp, control_send->joint_pid[7].Kd, control_send->multi_arm_cmd[7][2]);
			CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[0], control_send->multi_arm_cmd[0][0], control_send->multi_arm_cmd[0][1], control_send->joint_pid[0].Kp, control_send->joint_pid[0].Kd, control_send->multi_arm_cmd[0][2]);
			CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[1], control_send->multi_arm_cmd[1][0], control_send->multi_arm_cmd[1][1], control_send->joint_pid[1].Kp, control_send->joint_pid[1].Kd, control_send->multi_arm_cmd[1][2]);
            CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[2], control_send->multi_arm_cmd[2][0], control_send->multi_arm_cmd[2][1], control_send->joint_pid[2].Kp, control_send->joint_pid[2].Kd, control_send->multi_arm_cmd[2][2]);
			
			CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[3], control_send->multi_arm_cmd[3][0], control_send->multi_arm_cmd[3][1], control_send->joint_pid[3].Kp, control_send->joint_pid[3].Kd, control_send->multi_arm_cmd[3][2]);
			CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[4], control_send->multi_arm_cmd[4][0], control_send->multi_arm_cmd[4][1], control_send->joint_pid[4].Kp, control_send->joint_pid[4].Kd, control_send->multi_arm_cmd[4][2]);
			CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[5], control_send->multi_arm_cmd[5][0], control_send->multi_arm_cmd[5][1], control_send->joint_pid[5].Kp, control_send->joint_pid[5].Kd, control_send->multi_arm_cmd[5][2]);
			CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[6], control_send->multi_arm_cmd[6][0], control_send->multi_arm_cmd[6][1], control_send->joint_pid[6].Kp, control_send->joint_pid[6].Kd, control_send->multi_arm_cmd[6][2]);
	}
	 else if(ARM_JOINT_BEHAVIOUR == ARM_SELF)
    {
			CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[7], control_send->multi_arm_cmd[7][0], control_send->multi_arm_cmd[7][1], control_send->joint_pid[7].Kp, control_send->joint_pid[7].Kd, control_send->multi_arm_cmd[7][2]);
			CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[0], control_send->multi_arm_cmd[0][0], control_send->multi_arm_cmd[0][1], control_send->joint_pid[0].Kp, control_send->joint_pid[0].Kd, control_send->multi_arm_cmd[0][2]);
			CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[1], control_send->multi_arm_cmd[1][0], control_send->multi_arm_cmd[1][1], control_send->joint_pid[1].Kp, control_send->joint_pid[1].Kd, control_send->multi_arm_cmd[1][2]);
			CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[2], control_send->multi_arm_cmd[2][0], control_send->multi_arm_cmd[2][1], control_send->joint_pid[2].Kp, control_send->joint_pid[2].Kd, control_send->multi_arm_cmd[2][2]);
			
			CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[3], control_send->multi_arm_cmd[3][0], control_send->multi_arm_cmd[3][1], control_send->joint_pid[3].Kp, control_send->joint_pid[3].Kd, control_send->multi_arm_cmd[3][2]);
			CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[4], control_send->multi_arm_cmd[4][0], control_send->multi_arm_cmd[4][1], control_send->joint_pid[4].Kp, control_send->joint_pid[4].Kd, control_send->multi_arm_cmd[4][2]);
			CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[5], control_send->multi_arm_cmd[5][0], control_send->multi_arm_cmd[5][1], control_send->joint_pid[5].Kp, control_send->joint_pid[5].Kd, control_send->multi_arm_cmd[5][2]);
			CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[6], control_send->multi_arm_cmd[6][0], control_send->multi_arm_cmd[6][1], control_send->joint_pid[6].Kp, control_send->joint_pid[6].Kd, control_send->multi_arm_cmd[6][2]);
	}
	else if(ARM_JOINT_BEHAVIOUR == ARM_SEMI_AUTO)
	{
			CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[7], control_send->multi_arm_cmd[7][0], control_send->multi_arm_cmd[7][1], control_send->joint_pid[7].Kp, control_send->joint_pid[7].Kd, control_send->multi_arm_cmd[7][2]);
			CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[0], control_send->multi_arm_cmd[0][0], control_send->multi_arm_cmd[0][1], control_send->joint_pid[0].Kp, control_send->joint_pid[0].Kd, control_send->multi_arm_cmd[0][2]);
			CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[1], control_send->multi_arm_cmd[1][0], control_send->multi_arm_cmd[1][1], control_send->joint_pid[1].Kp, control_send->joint_pid[1].Kd, control_send->multi_arm_cmd[1][2]);
			CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[2], control_send->multi_arm_cmd[2][0], control_send->multi_arm_cmd[2][1], control_send->joint_pid[2].Kp, control_send->joint_pid[2].Kd, control_send->multi_arm_cmd[2][2]);
			
			CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[3], control_send->multi_arm_cmd[3][0], control_send->multi_arm_cmd[3][1], control_send->joint_pid[3].Kp, control_send->joint_pid[3].Kd, control_send->multi_arm_cmd[3][2]);
			CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[4], control_send->multi_arm_cmd[4][0], control_send->multi_arm_cmd[4][1], control_send->joint_pid[4].Kp, control_send->joint_pid[4].Kd, control_send->multi_arm_cmd[4][2]);
			CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[5], control_send->multi_arm_cmd[5][0], control_send->multi_arm_cmd[5][1], control_send->joint_pid[5].Kp, control_send->joint_pid[5].Kd, control_send->multi_arm_cmd[5][2]);
			CAN_cmd_MIT(&END_CAN, arm_cmd_id_table[6], control_send->multi_arm_cmd[6][0], control_send->multi_arm_cmd[6][1], control_send->joint_pid[6].Kp, control_send->joint_pid[6].Kd, control_send->multi_arm_cmd[6][2]);
	}
//				CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[1], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
//				CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[2], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

__attribute__((used)) void gimbal_control_loop(gimbal_control_t *control_loop)
{
    (void)control_loop;

}

__attribute__((used)) void set_cali_gimbal_hook(const uint16_t yaw_offset,
                                                const uint16_t pitch_offset,
                                                const fp32 max_yaw,
                                                const fp32 min_yaw,
                                                const fp32 max_pitch,
                                                const fp32 min_pitch)
{

}

__attribute__((used)) bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset,
                                                  uint16_t *pitch_offset,
                                                  fp32 *max_yaw,
                                                  fp32 *min_yaw,
                                                  fp32 *max_pitch,
                                                  fp32 *min_pitch)
{

}
void arm_update_control(gimbal_control_t *add_angle)
{
    if(ARM_JOINT_BEHAVIOUR == ARM_NONE)
    {
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            //add角度设为0.0,速度也是0
            add_angle->multi_arm_set[i][0] = 0.0f;
        }
        return;
    }
    else if(ARM_JOINT_BEHAVIOUR == ARM_RC)
    {
		add_angle->multi_arm_set[7][0] = J7_ZERO_ANGLE;  // 大yaw
			//add_angle->multi_arm_set[7][2] = 2.0f;  // 大yaw
        //设置不同电机的位置,速度在set_mode里面计算
        if(switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[ARM_RC_MODE_CHANNEL]))
        {
            add_angle->multi_arm_set[0][0] = (gimbal_control.gimbal_rc_ctrl->rc.ch[HORI_CHANNEL] * MULTI_AXIS_RC_SEN);
            add_angle->multi_arm_set[1][0] = (gimbal_control.gimbal_rc_ctrl->rc.ch[VERT_CHANNEL] * MULTI_AXIS_RC_SEN);
        }                                                                                             
        if(switch_is_mid(gimbal_control.gimbal_rc_ctrl->rc.s[ARM_RC_MODE_CHANNEL]))                   
        {                                                                                             
            add_angle->multi_arm_set[2][0] = (gimbal_control.gimbal_rc_ctrl->rc.ch[HORI_CHANNEL] * 3*MULTI_AXIS_RC_SEN);
            add_angle->multi_arm_set[3][0] = (gimbal_control.gimbal_rc_ctrl->rc.ch[VERT_CHANNEL] * MULTI_AXIS_RC_SEN);
        }                                                                                             
        if(switch_is_up(gimbal_control.gimbal_rc_ctrl->rc.s[ARM_RC_MODE_CHANNEL]))                    
        {                                                                                             
            add_angle->multi_arm_set[4][0] = (gimbal_control.gimbal_rc_ctrl->rc.ch[HORI_CHANNEL] * MULTI_AXIS_RC_SEN);
            add_angle->multi_arm_set[5][0] = (gimbal_control.gimbal_rc_ctrl->rc.ch[VERT_CHANNEL] * MULTI_AXIS_RC_SEN);
        }
				return;

    }
    else if(ARM_JOINT_BEHAVIOUR == ARM_SELF)
    {
		if(!motor_data.online){	//掉线直接返回
//			return;
		}
		// motor_data 中的数据已经是弧度值，直接赋值给对应的关节
        // 只控制 j0、j1、j2 三个关节
		float target_pos[ARM_JOINT_NUM];  // 过渡变量，存储原始目标位置

		target_pos[7] = J7_ZERO_ANGLE;  // 大yaw

		target_pos[1] = -motor_data.j1 + J1_ZERO_ANGLE;  // J1关节
		if (add_angle->joint_motor[1].motor_measure->pos > J1_MIN_ANLE + 0.1f) {
			target_pos[0] = motor_data.j0 + J0_ZERO_ANGLE;  // J0关节
		} else {
			target_pos[0] = J0_ZERO_ANGLE;
		}
		target_pos[2] = -1.4446f * motor_data.j2 + J2_ZERO_ANGLE;  // J2关节
		target_pos[3] = motor_data.j3 + J3_ZERO_ANGLE;  // J3关节
		target_pos[4] = -motor_data.j4 + J4_ZERO_ANGLE;  // J4关节
		target_pos[5] = motor_data.j5 + J5_ZERO_ANGLE;  // J5关节
		target_pos[6] = motor_data.claw ? J6_MIN_ANLE : J6_MAX_ANGLE;  // 夹爪

		// 一阶低通滤波
		static float filtered_pos[ARM_JOINT_NUM] = {0};
		static uint8_t first_run = 1;
		const float alpha = 0.8f;  // 滤波系数,越小越平滑

		if (first_run) {
			// 首次进入 SELF 模式时，直接用目标值初始化滤波器
			for (int i = 0; i < ARM_JOINT_NUM; i++) {
				filtered_pos[i] = target_pos[i];
			}
			first_run = 0;
		} else {
			// 正常滤波：新滤波值 = alpha * 新目标 + (1-alpha) * 旧滤波值
			for (int i = 0; i < ARM_JOINT_NUM; i++) {
				filtered_pos[i] = alpha * target_pos[i] + (1 - alpha) * filtered_pos[i];
			}
		}

		// 将滤波后的值写入 multi_arm_set
		for (int i = 0; i < ARM_JOINT_NUM; i++) {
			add_angle->multi_arm_set[i][0] = filtered_pos[i];
		}

		return;
	}
	else if(ARM_JOINT_BEHAVIOUR == ARM_SEMI_AUTO)
	{
		for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
		{
			add_angle->multi_arm_set[i][1] = 0.0f;   // 速度指令清零，保持阻尼
		}

		if (!g_have_tool_goal)
		{
			for (uint8_t i = 0; i < 16; i++)
			{
				g_T_tool_goal[i] = g_T_tool_cur[i];
			}
			g_have_tool_goal = 1U;
		}

		/* 按住 E：工具坐标系下沿 z 轴连续移动；松开：不再更新目标，保持最后姿态 */
		if (add_angle->gimbal_rc_ctrl->key.v & CHASSIS_TURNRIGHT_KEY)
		{
			float T_next[16];
			tool_relative_translate(g_T_tool_goal, 0.0f, 0.0f, ARM_SEMI_AUTO_Z_SPEED * GIMBAL_CONTROL_TIME / 1000.0f, T_next);

			for (uint8_t i = 0; i < 16; i++)
			{
				g_T_tool_goal[i] = T_next[i];
			}
		}

		g_semi_auto_ik_valid = compute_desired_joint_angles(g_T_tool_goal, g_theta_kin_cur, g_theta_kin_des);

		if (g_semi_auto_ik_valid)
		{
			kin_theta_to_motor_pos(g_theta_kin_des, g_motor_pos_des);

			for (uint8_t i = 0; i < 6; i++)
			{
				add_angle->multi_arm_set[i][0] = g_motor_pos_des[i];
			}
		}
		else
		{
			/* 逆解失败时，保持当前关节位置，避免乱跳 */
			for (uint8_t i = 0; i < 6; i++)
			{
				add_angle->multi_arm_set[i][0] = add_angle->joint_motor[i].motor_measure->pos;
			}
		}

		/* 其余两个附加轴保持当前 */
		add_angle->multi_arm_set[6][0] = add_angle->joint_motor[6].motor_measure->pos;
		add_angle->multi_arm_set[7][0] = J7_ZERO_ANGLE;
	}
    else if(ARM_JOINT_BEHAVIOUR == ARM_INIT)
    {
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            add_angle->multi_arm_set[i][0] = 0.0f;
            add_angle->multi_arm_set[i][1] = INIT_VEL;
        }
				return;
    }


    // for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
    //     {
    //         add_angle->multi_arm_set[i] = 0.0f;
    //     }
}
/**
 * @brief 解包自定义控制器数据帧
 * @param data 接收到的原始数据
 * @param len 数据长度
 * @param out 输出数据存储结构体
 * @return 0-成功，负值-错误码
 */
int Unpack_CustomController_Frame_Simple(const uint8_t *data, moterMapHeader *out) {
//	// 1. 检查帧头
//    if (data[0] != 0xA5) {
//        return -1; // 帧头错误
//    }

//    // 2. 提取数据长度（小端序）
    uint16_t data_length = (data[2] << 8) | data[1];
    
    // 3. 校验CRC8（前4字节）
    if (get_CRC8_check_sum((uint8_t*)data, 4, 0xFF) != data[4]) {
        return -2; // CRC8错误
    }

    // 4. 提取CmdID（小端序）
    uint16_t cmd_id = (data[6] << 8) | data[5];
    if (cmd_id != 0x0302) { // 0x302对应小端存储的0x02 0x03
        return -3; // CmdID错误
    }

    // 5. 提取电机映射数据
    memcpy(out, &data[7], sizeof(moterMapHeader));

    // 6. 计算CRC16（前33字节）
    uint16_t calc_crc = get_CRC16_check_sum((uint8_t*)data, 37, 0xFFFF);
    uint16_t recv_crc = (data[38] << 8) | data[37];
    
    if (calc_crc != recv_crc) {
        return -4; // CRC16错误
    }

    return 0; // 成功
}
fp32 abs_pos[ARM_JOINT_NUM];
void Initial_position_safety_check(gimbal_control_t *position)
{
	static fp32 moter_init_angle[ARM_JOINT_NUM] = 
    {
        J0_INIT_ANGLE,
		J1_INIT_ANGLE,
		J2_INIT_ANGLE,
		J3_INIT_ANGLE,
		J4_INIT_ANGLE,
		J5_INIT_ANGLE,
		J6_INIT_ANGLE,
		J7_INIT_ANGLE
    };
	osDelay(100);
	while(position->joint_motor[1].motor_measure->pos == 0);
	for(uint8_t i = 0; i <  ARM_JOINT_NUM; i++)
	{
//		if(position->joint_motor[i].motor_measure->pos == 0){MOTER_InitAngleright = 2;return;}
		abs_pos[i] =  fabsf(position->joint_motor[i].motor_measure->pos - moter_init_angle[i]);
		if(abs_pos[i]>0.88f)
		{
			MOTER_InitAngleright = 0;
			return;
		}
	}
	
}
void gra_theta_calcu(gimbal_control_t *pos_angle, float *position_calcu)
{
		position_calcu[0] = PI/2 + pos_angle->joint_motor[0].motor_measure->pos - J0_ZERO_ANGLE;  // J0关节
		position_calcu[1] = -(PI/2 + pos_angle->joint_motor[1].motor_measure->pos - J1_ZERO_ANGLE);  // J1关节
		position_calcu[2] = -PI/2 - (pos_angle->joint_motor[2].motor_measure->pos - J2_ZERO_ANGLE)/1.4446f;  // J2关节
		position_calcu[3] = -(pos_angle->joint_motor[3].motor_measure->pos - J3_ZERO_ANGLE);  // J3关节
		position_calcu[4] = -pos_angle->joint_motor[4].motor_measure->pos + J4_ZERO_ANGLE;  // J4关节
		position_calcu[5] = pos_angle->joint_motor[5].motor_measure->pos - J5_ZERO_ANGLE;  // J5关节
}

#endif  // ROBOT_GIMBAL == multi_axis_robotic_arm
