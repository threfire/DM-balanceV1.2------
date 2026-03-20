#include "Balance_chassis.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"

#if ROBOT_CHASSIS == Balance_wheel
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "pid.h"
#include "can_bsp.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "kalman_filter.h"
#include "observer.h"
#include "slip_control.h"
#include "robot_param.h"
#include "VMC_calc.h"
//电机挂载的can总线
#define JOINT_CAN hfdcan2
#define WHEEL_CAN hfdcan1

//腿长PID
#define LEG_PID_KP  450.0f
#define LEG_PID_KI  0.0f
#define LEG_PID_KD  3000.0f
#define LEG_PID_MAX_OUT  90.0f //90牛
#define LEG_PID_MAX_IOUT 0.0f

#define TP_PID_KP 10.0f
#define TP_PID_KI 0.0f 
#define TP_PID_KD 0.1f
#define TP_PID_MAX_OUT  2.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP 2.0f
#define TURN_PID_KI 0.0f 
#define TURN_PID_KD 0.5f
#define TURN_PID_MAX_OUT  3.0f
#define TURN_PID_MAX_IOUT 0.0f

#define ROLL_PID_KP 100.0f
#define ROLL_PID_KI 0.0f 
#define ROLL_PID_KD 0.0f
#define ROLL_PID_MAX_OUT  2.5f//轮毂电机的额定扭矩
#define ROLL_PID_MAX_IOUT 0.0f

#define Max_T 20.0f
#define Min_T -Max_T

#define MAX_LEG 0.15f
#define MIN_LEG 0.4f


//整车重量，单位KG
#define Mg 15.0f

#define V_pos 3.0f //板凳模型下关节电机的速度
//电机打滑速度，40rad/s，可以自行修改
#define Max_speed 35.0f
#define Min_speed -Max_speed

//驱动轮半径
#define WHEEL_RAD 0.055f

#define JOINT_TORQUE_MORE_OFFSET              ((uint8_t)1 << 0)  // 关节电机输出力矩过大偏移量
#define DBUS_ERROR_OFFSET    ((uint8_t)1 << 2)  // dbus错误偏移量
#define MOTOR_OFFLINE      ((uint8_t)1 << 5)  // 电机掉线
#define RAPAM_ERROR      ((uint8_t)1 << 6)  // 设置参数错误

//lqr矩阵,在matlab里面仿真，串联腿的L1 = 0
float Poly_Coefficient[12][4]={	{-67.2554, 108.1340, -86.69397, -2.8167},
																{-0.73605, 0.279703, 	-9.63876, 0.05095},
																{-4.72129, 6.298025, -2.908949, -4.2857},
																{-4.67634, 6.639103, -4.769149, -4.3895},
																{-40.8640, 65.13989, -41.52719, 15.4351},
																{-0.97457, 1.699741, -1.098356, 1.30153},
																{63.81618, -87.3989, 	43.49306, 11.5422},
																{3.798957, -4.86799, 	1.534030, 0.89065},
																{-36.4727, 	 4.1533, 	-29.9746, 6.93823},
																{-35.7735,  53.1792, 	-29.5928, 6.92360},
																{111.0142, -159.214, 	84.80462, 10.3936},
																{8.532169, -12.5131, 	6.866856, -0.2325}};	

//放置lqr的参数，初始化无所谓，主要是看上面的矩阵																
float LQR_K_R[12]={ 
-16.6271,	-1.5394,	-0.7001,	-1.4681,	4.7675,	0.4977,
19.6639,	0.7449,	0.4414,	0.8771,	29.8843,	1.2325};
//每个参数得到的值，暂存在这里方便调试
float LQR_R[6] = {0};

float LQR_K_L[12]={ 
-14.7333,	-1.1901,	-0.6917,	-1.4383,	5.8373,	0.5445,
20.0303,	0.8189,	0.65443,	1.3019,	27.8958,	1.0854};
//每个参数得到的值，暂存在这里方便调试
float LQR_L[6] = {0};

//跳跃函数声明
void jump_loop_r(chassis_move_t *chassis,vmc_leg_t *vmcr,pid_type_def *leg);
void jump_loop_l(chassis_move_t *chassis,vmc_leg_t *vmcl,pid_type_def *leg);

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
__attribute__((used))void chassis_init(chassis_move_t *chassis_move_init){
	osDelay(1000);
	//初始化腿长PID
	const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
	const static float legl_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	const static float roll[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
	
	//使能关节电机,同canid的电机是中心对称的
	Motor_enable(&JOINT_CAN, 0x06);
	osDelay(1);
	Motor_enable(&JOINT_CAN, 0x07);
	osDelay(1);
	Motor_enable(&JOINT_CAN, 0x08);
	osDelay(1);
	Motor_enable(&JOINT_CAN, 0x09);
	osDelay(1);
	//使能轮毂电机
	Motor_enable(&WHEEL_CAN,0x10);
	osDelay(1);
	Motor_enable(&WHEEL_CAN,0x11);
	osDelay(1);
	//获取IMU的六轴数据
	chassis_move_init->angle_point = get_INS_angle_point();
	chassis_move_init->acc_point = get_gyro_data_point();

	for(uint8_t i = 0; i < 4; i++){
		chassis_move_init->joint_motor[i] = get_mit_motor_measure_point(i);
	}
	for(uint8_t i = 0; i < 2; i++){
		chassis_move_init->wheel_motor[i] = get_mit_motor_measure_point(i + 4);
	}
	
	//VMC初始化,
	VMC_init(&chassis_move_init->left);//给杆长赋值
	VMC_init(&chassis_move_init->right);//给杆长赋值
	
	//初始化腿长PID
	PID_init(&chassis_move_init->LegR_Pid, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid
	PID_init(&chassis_move_init->LegL_Pid, PID_POSITION,legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid

	
	PID_init(&chassis_move_init->Tp_Pid, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
	PID_init(&chassis_move_init->Turn_Pid, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
	
	PID_init(&chassis_move_init->RollR_Pid, PID_POSITION, roll, ROLL_PID_MAX_OUT,ROLL_PID_MAX_IOUT);

	chassis_move_init -> wheel_motor_timeout = 0;
	chassis_move_init -> joint_motor_flag = 0;
	osDelay(100);
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
__attribute__((used))void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
__attribute__((used))void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //change to follow gimbal angle mode
    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //change to follow chassis yaw angle
    //切入跟随底盘角度模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //change to no follow angle
    //切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
			  //应该设置为与云台yaw轴电机相关
			  //chassis_move_transit->chassis_yaw_set = yaw_motor_angle;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}


__attribute__((used))void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f, vy_set = 0.0f, leg_set = 0.0f;
    //get three control set-point, 获取三个控制设置值
		chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_control);

		chassis_move_control->last_leg_set = chassis_move_control->leg_set;
		chassis_move_control->last_leg_left_set = chassis_move_control->leg_left_set;
		chassis_move_control->last_leg_right_set = chassis_move_control->leg_right_set;
		//修改腿长的值
		chassis_move_control->leg_set += vy_set * 0.08f;
		chassis_move_control->leg_set = fp32_constrain(chassis_move_control->leg_set, MIN_LEG, MAX_LEG);
		chassis_move_control->leg_left_set = chassis_move_control->leg_right_set  = chassis_move_control->leg_set;
		
		
		if(fabsf(chassis_move_control->last_leg_left_set - chassis_move_control->leg_left_set) > 0.0001f || fabsf(chassis_move_control->last_leg_right_set - chassis_move_control->leg_right_set) > 0.0001f)
		{//遥控器控制腿长在变化
			chassis_move_control->right.leg_flag=1;	//为1标志着腿长在主动伸缩(不包括自适应伸缩)，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判端为离地了
      		chassis_move_control->left.leg_flag=1;	 			
		}
		

    //follow gimbal mode
    //跟随云台模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //rotate chassis direction, make sure vertial direction follow gimbal 
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        //chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        //set control relative angle  set-point
        //设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(yaw_motor_relative_angle - PI /3);
        //calculate ratation speed
        //计算旋转PID角速度
        chassis_move_control->wz_set = -PID_Calc(&chassis_move_control->Turn_Pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
        //speed limit
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }
		//底盘角度控制闭环
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        fp32 delat_angle = 0.0f;
        //set chassis yaw angle set-point
        //设置底盘控制的角度
        chassis_move_control->chassis_yaw_set = rad_format(chassis_move_control->chassis_angle_set);
        delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        //calculate rotation speed
        //计算旋转的角速度
        //chassis_move_control->wz_set = PID_Calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);
				chassis_move_control->wz_set = PID_Calc(&chassis_move_control->Turn_Pid, delat_angle, 0.0f);
        //speed limit
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }
		//底盘有旋转速度控制
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //"angle_set" is rotation speed set-point
        //“angle_set” 是旋转速度控制
        chassis_move_control->wz_set = chassis_move_control->chassis_angle_set;
        //chassis_move_control->wz_set = PID_Calc(&chassis_move_control->chassis_angle_pid, yaw_motor_relative_angle, chassis_angle_set);
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
				
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //in raw mode, set-point is sent to CAN bus
        //在原始模式，设置值是发送到CAN总线
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->wz_set = chassis_move_control->chassis_angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
    }
    else if(chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW){
        chassis_move_control->wz_set = -PID_Calc(&chassis_move_control->Turn_Pid, chassis_move_control->chassis_angle_set, yaw_motor_relative_angle);
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }

}


__attribute__((used))void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
		chassis_move_update->x_set += chassis_move_update->vx_set * CHASSIS_CONTROL_TIME_MS /1000.0f; 
	
		chassis_move_update->right.phi1=PI/2.0f+chassis_move_update->joint_motor[0]->pos;
		chassis_move_update->right.phi4=PI/2.0f+chassis_move_update->joint_motor[1]->pos;
		
		chassis_move_update->right.phi1=PI/2.0f+chassis_move_update->joint_motor[2]->pos;
		chassis_move_update->right.phi4=PI/2.0f+chassis_move_update->joint_motor[3]->pos;
		//更新角度和角速度
		chassis_move_update->myPithR = chassis_move_update->angle_point[1];
		chassis_move_update->myPithGyroR = chassis_move_update->acc_point[0];
		
		chassis_move_update->myPithL=0.0f - chassis_move_update->angle_point[1];
		chassis_move_update->myPithGyroL=0.0f- chassis_move_update->acc_point[0];
		 
		chassis_move_update->total_yaw = get_YawTotalAngle();
		chassis_move_update->chassis_yaw = rad_format(chassis_move_update->total_yaw); 

		//	chassis->total_yaw=ins->YawTotalAngle;
		chassis_move_update->roll = chassis_move_update->angle_point[2];
		chassis_move_update->theta_err = 0.0f - (chassis_move_update->right.theta+chassis_move_update->left.theta);
		
		if(chassis_move_update->angle_point[1] < (3.1415926f / 6.0f)&&chassis_move_update->angle_point[1] > (-3.1415926f / 6.0f))
		{//根据pitch角度判断倒地自起是否完成
			chassis_move_update->recover_flag = 0;
		}
		
		if((HAL_GetTick() - chassis_move_update -> wheel_motor[0]->last_fdb_time) > 1000 ||\
			(HAL_GetTick() - chassis_move_update -> wheel_motor[0]->last_fdb_time) > 1000 )
		{
			chassis_move_update->wheel_motor_timeout = 1;
		}
		else
		{
			chassis_move_update->wheel_motor_timeout = 0;
		}
		for(uint8_t i = 0; i  < 4; i++)
		{
			if((HAL_GetTick() - chassis_move_update -> joint_motor[i]->last_fdb_time) > 1000 )
			{
				chassis_move_update->joint_motor_flag = 1;
			}

		}
		
}
void chassis_control_loop(chassis_move_t *chassis_move_control_loop){
	
		VMC_calc_1_right(&chassis_move_control_loop->right,&INS,((float)CHASSIS_CONTROL_TIME_MS) /1000.0f);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是0.002秒
		VMC_calc_1_left(&chassis_move_control_loop->left,&INS,((float)CHASSIS_CONTROL_TIME_MS) /1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是0.002秒
		
		for(int i=0;i<12;i++)
		{
			LQR_K_R[i]=LQR_K_calc(&Poly_Coefficient[i][0],chassis_move_control_loop->right.L0 );	
			LQR_K_L[i]=LQR_K_calc(&Poly_Coefficient[i][0],chassis_move_control_loop->left.L0 );	
		}

		//chassis_move_control_loop->turn_T=chassis_move_control_loop->Turn_Pid.Kp * (chassis_move_control_loop->turn_set - chassis_move_control_loop->total_yaw) - chassis_move_control_loop->Turn_Pid.Kd*INS.Gyro[2];//这样计算更稳一点
		chassis_move_control_loop->turn_T=chassis_move_control_loop->Turn_Pid.Kp * (chassis_move_control_loop->chassis_yaw_set - chassis_move_control_loop->total_yaw) - chassis_move_control_loop->Turn_Pid.Kd*INS.Gyro[2];//这样计算更稳一点
		chassis_move_control_loop->leg_tp=PID_Calc(&chassis_move_control_loop->Tp_Pid, chassis_move_control_loop->theta_err,0.0f);//防劈叉pid计算
		
		//theta前倾为-，后倾为+  picth前倾为+，后倾为-
		//wheel为负，右边电机逆时针转动，机体向前
		LQR_R[0] = -LQR_K_R[0]*(chassis_move_control_loop->right.theta - 0.0f);
		LQR_R[1] = -LQR_K_R[1]*(chassis_move_control_loop->right.d_theta - 0.0f);
		LQR_R[2] = -LQR_K_R[2]*(chassis_move_control_loop->x_filter - chassis_move_control_loop->x_set);
		LQR_R[3] = -LQR_K_R[3]*( chassis_move_control_loop->v_filter - chassis_move_control_loop->vx_set);
		LQR_R[4] = -LQR_K_R[4]*(chassis_move_control_loop->myPithR - 0.00f);
		LQR_R[5] = -LQR_K_R[5]*(chassis_move_control_loop->myPithGyroR - 0.0f);
		chassis_move_control_loop->wheel_motor[0]->motor_t = (
																		 LQR_R[0]
																		+LQR_R[1]
																		-LQR_R[2]
																		-LQR_R[3]
																		+LQR_R[4]
																		+LQR_R[5]
		);
		
		//右边髋关节输出力矩				
		chassis_move_control_loop->right.Tp=(LQR_K_R[6]*(chassis_move_control_loop->right.theta - 0.00f)
						+ LQR_K_R[7]*(chassis_move_control_loop->right.d_theta-0.0f)
						- LQR_K_R[8]*(chassis_move_control_loop->x_filter - chassis_move_control_loop->x_set)
						- LQR_K_R[9]*(chassis_move_control_loop->v_filter - chassis_move_control_loop->vx_set)
						//- LQR_K[9]*(chassis->v_filter - chassis->v_set)
						+ LQR_K_R[10]*(chassis_move_control_loop->myPithR - 0.00f)
						+ LQR_K_R[11]*(chassis_move_control_loop->myPithGyroR - 0.0f));
						
						
						
			//theta前倾为正，后倾为-  picth前倾为-，后倾为+
			//wheel为正，左边电机顺时针转动，机体向前
			LQR_L[0] = -LQR_K_L[0]*(chassis_move_control_loop->left.theta-0.0f) ;
			LQR_L[1] = -LQR_K_L[1]*(chassis_move_control_loop->left.d_theta-0.0f);
			LQR_L[2] = -LQR_K_L[2]*(chassis_move_control_loop->x_set - chassis_move_control_loop->x_filter);
			LQR_L[3] = -LQR_K_L[3]*(chassis_move_control_loop->vx_set - chassis_move_control_loop->v_filter);
			//	LQR_L[4] = -LQR_K[4]*(chassis->myPithL-0.0f);
			LQR_L[4] = -LQR_K_L[4]*(chassis_move_control_loop->myPithL-0.00f) ;
			LQR_L[5] = -LQR_K_L[5]*(chassis_move_control_loop->myPithGyroL-0.0f);
			chassis_move_control_loop->wheel_motor[1]->motor_t = (
																				+LQR_L[0]
																				+LQR_L[1]
																				-LQR_L[2]
																				-LQR_L[3]
																				+LQR_L[4]
																				+LQR_L[5]
			);
			
			//右边髋关节输出力矩				
			chassis_move_control_loop->left.Tp=(LQR_K_L[6]*(chassis_move_control_loop->left.theta-0.0f)
							+LQR_K_L[7]*(chassis_move_control_loop->left.d_theta-0.0f) 
							-LQR_K_L[8]*(chassis_move_control_loop->x_set - chassis_move_control_loop->x_filter)
							-LQR_K_L[9]*(chassis_move_control_loop->vx_set - chassis_move_control_loop->v_filter)
							// -LQR_K[8]*(chassis->x_set-chassis->x_filter)
							// -LQR_K[9]*(chassis->v_set-chassis->v_filter)
							+LQR_K_L[10]*(chassis_move_control_loop->myPithL+0.00f) 
							+LQR_K_L[11]*(chassis_move_control_loop->myPithGyroL-0.0f)
																											);
																											
		chassis_move_control_loop->wheel_motor[0]->motor_t = chassis_move_control_loop->wheel_motor[0]->motor_t + chassis_move_control_loop->turn_T;	//轮毂电机输出力矩
		chassis_move_control_loop->wheel_motor[1]->motor_t = chassis_move_control_loop->wheel_motor[1]->motor_t + chassis_move_control_loop->turn_T;	//轮毂电机输出力矩

		//轮毂电机限幅
		chassis_move_control_loop->wheel_motor[0]->motor_t = fp32_constrain(chassis_move_control_loop->wheel_motor[0]->motor_t, -3.5f, 3.5f);	
		chassis_move_control_loop->wheel_motor[1]->motor_t = fp32_constrain(chassis_move_control_loop->wheel_motor[1]->motor_t, -3.5f, 3.5f);	
		
		if((chassis_move_control_loop->wheel_motor[0]->vel > Max_speed || chassis_move_control_loop->wheel_motor[0]->vel < Min_speed) ||\
			 (chassis_move_control_loop->wheel_motor[1]->vel > Max_speed || chassis_move_control_loop->wheel_motor[1]->vel < Min_speed))
		{
			chassis_move_control_loop->wheel_motor[0]->motor_t = 0;
			chassis_move_control_loop->wheel_motor[1]->motor_t = 0;
		}
		//	vmcr->Tp=vmcr->Tp + chassis->leg_tp;//髋关节输出力矩
		chassis_move_control_loop->right.Tp = chassis_move_control_loop->right.Tp - chassis_move_control_loop->leg_tp;//髋关节输出力矩
		chassis_move_control_loop->left.Tp = chassis_move_control_loop->left.Tp + chassis_move_control_loop->leg_tp;//髋关节输出力矩


		chassis_move_control_loop->now_roll_set = PID_Calc(&chassis_move_control_loop->RollR_Pid, chassis_move_control_loop->roll, chassis_move_control_loop->roll_set);
		//chassis->now_roll_set = 0;

		//vmcr->F0=Mg * 10.0f / arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) - chassis->now_roll_set;//前馈+pd
		jump_loop_r(chassis_move_control_loop, &chassis_move_control_loop->right, &chassis_move_control_loop->LegR_Pid);
		jump_loop_l(chassis_move_control_loop, &chassis_move_control_loop->left, &chassis_move_control_loop->LegL_Pid);
			
		chassis_move_control_loop->right_flag = ground_detectionR(&chassis_move_control_loop->right, &INS);//右腿离地检测
		chassis_move_control_loop->left_flag = ground_detectionL(&chassis_move_control_loop->right, &INS);//左腿离地检测
		
		 if(chassis_move_control_loop->recover_flag==0)		
		 {//倒地自起不需要检测是否离地	 
			if(chassis_move_control_loop->right_flag==1 && chassis_move_control_loop->left_flag==1 && chassis_move_control_loop->right.leg_flag == 0)
			{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
					chassis_move_control_loop->wheel_motor[0]->motor_t = 0.0f;
					chassis_move_control_loop->right.Tp = LQR_K_R[6]*(chassis_move_control_loop->right.theta - 0.0f) + LQR_K_R[7] * (chassis_move_control_loop->right.d_theta - 0.0f);
					chassis_move_control_loop->x_filter = 0.0f;
					chassis_move_control_loop->x_set = chassis_move_control_loop->x_filter;
					chassis_move_control_loop->turn_set = chassis_move_control_loop->total_yaw;
					chassis_move_control_loop->right.Tp = chassis_move_control_loop->right.Tp + chassis_move_control_loop->leg_tp;		
				
				
					chassis_move_control_loop->wheel_motor[1]->motor_t = 0.0f;
					chassis_move_control_loop->left.Tp = LQR_K_L[6]*(chassis_move_control_loop->left.theta - 0.0f) + LQR_K_R[7] * (chassis_move_control_loop->left.d_theta - 0.0f);
					chassis_move_control_loop->x_set = chassis_move_control_loop->x_filter;
					chassis_move_control_loop->turn_set = chassis_move_control_loop->total_yaw;
					chassis_move_control_loop->left.Tp = chassis_move_control_loop->left.Tp + chassis_move_control_loop->leg_tp;		
				
			}
			else
			{//没有离地
				chassis_move_control_loop->right.leg_flag = 0;//置为0
				chassis_move_control_loop->left.leg_flag = 0;//置为0
			}
		 }
		 else if(chassis_move_control_loop->recover_flag == 1)
		 {
			 chassis_move_control_loop->right.Tp = 0.0f;
			 chassis_move_control_loop->left.Tp = 0.0f;
		 }	 
		 
		chassis_move_control_loop->right.F0 = fp32_constrain(chassis_move_control_loop->right.F0, -Mg , Mg);//限幅
		chassis_move_control_loop->left.F0 = fp32_constrain(chassis_move_control_loop->left.F0, -Mg , Mg);//限幅
		
		VMC_calc_2(&chassis_move_control_loop->right);//计算期望的关节输出力矩
		VMC_calc_2(&chassis_move_control_loop->left);//计算期望的关节输出力矩
		 
		//额定扭矩
		chassis_move_control_loop->right.torque_set[1] = fp32_constrain(chassis_move_control_loop->right.torque_set[1], Min_T, Max_T);	
		chassis_move_control_loop->right.torque_set[0] = fp32_constrain(chassis_move_control_loop->right.torque_set[0], Min_T, Max_T);
		chassis_move_control_loop->left.torque_set[1] = fp32_constrain(chassis_move_control_loop->left.torque_set[1], Min_T, Max_T);	
		chassis_move_control_loop->left.torque_set[0] = fp32_constrain(chassis_move_control_loop->left.torque_set[0], Min_T, Max_T);

}

__attribute__((used))void chassis_send_cmd(chassis_move_t *chassis_send_cmd)
{
/*
左                               															  右
CAN_id:9	rev:DM_M4_ID	     	开发板can				rev:DM_M1_ID  CAN_id:6
WHEEL1:rev:DM_M6_ID  CAN_id:11							WHEEL0:rev:DM_M5_ID  CAN_id:10																WHEEL0:rev:DM_M5_ID CAN_id:1
	
CAN_id:8 	rev:DM_M3_ID			   开发板IO				rev:DM_M2_ID  CAN_id:7



*/
		if(chassis_send_cmd->wheel_motor_timeout)
		{
			CAN_cmd_MIT(&JOINT_CAN, 0x06, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);  
			CAN_cmd_MIT(&JOINT_CAN, 0x07, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			osDelay(1);
			CAN_cmd_MIT(&JOINT_CAN, 0x08, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			CAN_cmd_MIT(&JOINT_CAN, 0x09, 0.0f, 0.0f,0.0f, 0.0f, 0.0f);
			osDelay(1);
			Motor_enable(&WHEEL_CAN, 0x10);
			Motor_enable(&WHEEL_CAN, 0x11);
		}
		else{
			CAN_cmd_MIT(&JOINT_CAN, 0x06, 0.0f, 0.0f,0.0f, 0.0f, chassis_send_cmd->right.torque_set[0]);  
			CAN_cmd_MIT(&JOINT_CAN, 0x07, 0.0f, 0.0f,0.0f, 0.0f, chassis_send_cmd->left.torque_set[0]);
			osDelay(1);
			CAN_cmd_MIT(&JOINT_CAN, 0x08, 0.0f, 0.0f,0.0f, 0.0f, chassis_send_cmd->right.torque_set[1]);
			CAN_cmd_MIT(&JOINT_CAN, 0x09, 0.0f, 0.0f,0.0f, 0.0f, chassis_send_cmd->left.torque_set[1]);
			osDelay(1);
			CAN_cmd_MIT(&WHEEL_CAN, 0x10, 0.0f, 0.0f,0.0f, 0.0f, chassis_send_cmd->wheel_motor[0]->motor_t);
			CAN_cmd_MIT(&WHEEL_CAN, 0x11, 0.0f, 0.0f,0.0f, 0.0f, chassis_send_cmd->wheel_motor[1]->motor_t);
		}

}

//跳跃
void jump_loop_r(chassis_move_t *chassis,vmc_leg_t *vmcr,pid_type_def *leg)
{
	if(chassis->jump_flag == 1)
	{
		if(chassis->jump_status_r == 0)
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,0.2f) ;//前馈+pd
			if(vmcr->L0<0.3f)
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 1;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 1;
			}
		}
		else if(chassis->jump_status_r == 1)
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,0.4f) ;//前馈+pd
			if(vmcr->L0>0.5f)
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>=10&&chassis->jump_time_l>=10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 2;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 2;
			}
		}
		else if(chassis->jump_status_r == 2)
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) ;//前馈+pd
			if(vmcr->L0<(chassis->leg_right_set + 0.01f))
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>=10&&chassis->jump_time_l >= 10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 3;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 3;
			}
		}
		else
		{
			vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) ;//前馈+pd
		}

		if(chassis->jump_status_r == 3&&chassis->jump_status_l == 3)
		{
			chassis->jump_flag = 0;
			chassis->jump_time_r = 0;
			chassis->jump_status_r = 0;
			chassis->jump_time_l = 0;
			chassis->jump_status_l = 0;
		}
	}
	else
	{
		vmcr->F0=Mg/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) - chassis->now_roll_set;//前馈+pd
	}
}

void jump_loop_l(chassis_move_t *chassis,vmc_leg_t *vmcl,pid_type_def *leg)
{
	if(chassis->jump_flag == 1)
	{
		if(chassis->jump_status_l == 0)
		{
			vmcl->F0= Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,0.07f) ;//前馈+pd
			if(vmcl->L0<0.3f)
			{
				chassis->jump_time_l++;
			}
		}
		else if(chassis->jump_status_l == 1)
		{
			vmcl->F0= Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,0.4f) ;//前馈+pd
			if(vmcl->L0>0.5f)
			{
				chassis->jump_time_l++;
			}
		}
		else if(chassis->jump_status_l == 2)
		{
			vmcl->F0=Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,chassis->leg_right_set) ;//前馈+pd
			if(vmcl->L0<(chassis->leg_right_set+0.01f))
			{
				chassis->jump_time_l++;
			}
		}
		else
		{
			vmcl->F0=Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,chassis->leg_left_set) ;//前馈+pd
		}

	}
	else
	{
		vmcl->F0=Mg/arm_cos_f32(vmcl->theta) + PID_Calc(leg,vmcl->L0,chassis->leg_left_set) + chassis->now_roll_set;//前馈+pd
	}
}

//平衡底盘观测速度函数
__attribute__((used))void observer(chassis_move_t *chassis_move, KalmanFilter_t *vxEstimateKF, KalmanFilter_t *vyEstimateKF){
		static float wr,wl=0.0f;
		static float vrb,vlb=0.0f;
		static float aver_v=0.0f;
		static float vel_acc[2] = {0.0f};
		wr = -chassis_move->wheel_motor[0]->vel - INS.Gyro[0] + chassis_move->right.d_alpha;//右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vrb = wr * WHEEL_RAD + chassis_move->right.L0*chassis_move->right.d_theta*arm_cos_f32(chassis_move->right.theta) + chassis_move->right.d_L0 * arm_sin_f32(chassis_move->right.theta);//机体b系的速度
		
		wl = -chassis_move->wheel_motor[1]->vel+INS.Gyro[0] + chassis_move->left.d_alpha;//左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vlb = wl * WHEEL_RAD + chassis_move->left.L0 * chassis_move->left.d_theta * arm_cos_f32(chassis_move->left.theta) + chassis_move->left.d_L0 * arm_sin_f32(chassis_move->left.theta);//机体b系的速度
		
		aver_v = (vrb - vlb)/2.0f;//取平均
    	xvEstimateKF_Update(vxEstimateKF, INS.MotionAccel_n[1], aver_v);
		
		//原地自转的过程中v_filter和x_filter应该都是为0
		chassis_move->v_filter = vel_acc[0];//得到卡尔曼滤波后的速度
		chassis_move->x_filter = chassis_move->x_filter + chassis_move->v_filter * ((float)OBSERVE_TIME / 1000.0f);
		//如果想直接用轮子速度，不做融合的话可以这样
		//chassis_move.v_filter=(chassis_move.wheel_motor[0]->para.vel-chassis_move.wheel_motor[1]->para.vel)*(-0.0603f)/2.0f;//0.0603是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
		//chassis_move.x_filter=chassis_move.x_filter+chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);
		
}

#endif
