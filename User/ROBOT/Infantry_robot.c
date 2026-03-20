#include "Infantry_robot.h"
#include "robot_param.h"
#include "gimbal_behaviour.h"

#if ROBOT_TYPE   ==  Infantry_robot
//用于发送的数组
uint8_t send[25];

void send_to_gimbal(void){
	//发送陀螺仪数据给云台
	static uint8_t send_to_gimbal[PACK_SIZE];
	uint8_t chassis_mode;
  fp32_to_bytes yaw, roll, pitch, chassis_vx, chassis_vy, chassis_wz;
  yaw.fp32 = chassis_move.chassis_yaw;
  pitch.fp32 = chassis_move.chassis_pitch;
  roll.fp32 = chassis_move.chassis_roll;
  send_to_gimbal[0] = 's';
  send_to_gimbal[1] = yaw.bytes[0];
  send_to_gimbal[2] = yaw.bytes[1];
  send_to_gimbal[3] = yaw.bytes[2];
  send_to_gimbal[4] = yaw.bytes[3];
  //
  send_to_gimbal[5] = pitch.bytes[0];
  send_to_gimbal[6] = pitch.bytes[1];
  send_to_gimbal[7] = pitch.bytes[2];
  send_to_gimbal[8] = pitch.bytes[3];
  //底盘旋转速度
  send_to_gimbal[9] = roll.bytes[0];
  send_to_gimbal[10] = roll.bytes[1];
  send_to_gimbal[11] = roll.bytes[2];
  send_to_gimbal[12] = roll.bytes[3];
	send_to_gimbal[13] = robot_id;
  send_to_gimbal[14] = 'e';
	
  USART1_Transmit_DMA(send_to_gimbal, PACK_SIZE);
}

//目前发送给底盘的数据：yaw电机由编码值转换的弧度值、imu的yaw值、pitch值、roll值、云台行为模式、射击模式
//0xFE为帧头，0xFD为帧尾
__attribute__((used))void send_data_to_chassis(gimbal_control_t *feedback_update)
{
	static fp32 motor_yaw_now;							//yaw电机由编码值转换的弧度值
  static fp32* INS_now_ptr; 							//存储INS指针, 用于发送imu的yaw值和pitch值
	static uint8_t gimbal_behaviour_now;		//云台行为模式
	static uint8_t shoot_mode_now;					//射击模式
	
  motor_yaw_now = feedback_update->gimbal_yaw_motor.radian_of_ecd;
  INS_now_ptr = get_INS_angle_point();	//0:yaw, 1:pitch, 2:roll 单位:rad
	extern gimbal_behaviour_e gimbal_behaviour;
	gimbal_behaviour_now = (uint8_t)gimbal_behaviour;
	extern shoot_control_t shoot_control; 
	shoot_mode_now = shoot_control.shoot_mode;
	
	//length = 2(帧头和帧尾) + 4 + 4*3 + 1 + 1 = 20
	const static uint8_t length = 20;
	static uint8_t data_chassis[length];		//发送给底盘的数据
	
	/*******************数据段*******************/
  data_chassis[0] = 0XFE;		//帧头
	
	// 提取每个字节
	uint32_t *ptr1 = (uint32_t*)&motor_yaw_now;
	data_chassis[1] = (*ptr1 >>  0) & 0xFF;			// 最低字节
	data_chassis[2] = (*ptr1 >>  8) & 0xFF;			// 第二个字节
	data_chassis[3] = (*ptr1 >> 16) & 0xFF;			// 第三个字节
	data_chassis[4] = (*ptr1 >> 24) & 0xFF;			// 最高字节
	
	uint32_t *ptr2 = (uint32_t*)INS_now_ptr;
	data_chassis[5] = (ptr2[0] >>  0) & 0xFF;		// 最低字节
	data_chassis[6] = (ptr2[0] >>  8) & 0xFF;		// 第二个字节
	data_chassis[7] = (ptr2[0] >> 16) & 0xFF;		// 第三个字节
	data_chassis[8] = (ptr2[0] >> 24) & 0xFF;		// 最高字节
  
	data_chassis[9]  = (ptr2[1] >>  0) & 0xFF;	// 最低字节
	data_chassis[10] = (ptr2[1] >>  8) & 0xFF;	// 第二个字节
	data_chassis[11] = (ptr2[1] >> 16) & 0xFF;	// 第三个字节
	data_chassis[12] = (ptr2[1] >> 24) & 0xFF;	// 最高字节
	
	data_chassis[13] = (ptr2[2] >> 	0) & 0xFF;	// 最低字节
	data_chassis[14] = (ptr2[2] >> 	8) & 0xFF;	// 第二个字节
	data_chassis[15] = (ptr2[2] >> 16) & 0xFF;	// 第三个字节
	data_chassis[16] = (ptr2[2] >> 24) & 0xFF;	// 最高字节
	
	data_chassis[17] = gimbal_behaviour_now;		// 单个字节
	
	data_chassis[18] = shoot_mode_now;					// 单个字节
	
  data_chassis[19] = 0XFD;  //帧尾
	/*******************************************/
	
	USART1_Transmit_DMA(data_chassis, length);	//DMA发送数据, 长度见上方 length
}

void send_to_minipc(void) {
    const gimbal_motor_t *pitch = get_pitch_motor_point();
    const gimbal_motor_t *yaw = get_yaw_motor_point();

    fp32_to_bytes pitch_data;
    fp32_to_bytes yaw_data;
    pitch_data.fp32 =  pitch->relative_angle;
    yaw_data.fp32 =  yaw->relative_angle;
	
    // 组装数据包
    send[0] = pitch_data.bytes[0];  // pitch 的第1个字节
    send[1] = pitch_data.bytes[1];  // pitch 的第2个字节
		send[2] = pitch_data.bytes[2];  // pitch 的第1个字节
    send[3] = pitch_data.bytes[3];  // pitch 的第2个字节
	
    send[4] = yaw_data.bytes[0];    // yaw 的第1个字节
    send[5] = yaw_data.bytes[1];    // yaw 的第2个字节
    send[6] = yaw_data.bytes[2];    // yaw 的第1个字节
    send[7] = yaw_data.bytes[3];    // yaw 的第2个字节
	
		send[8] = 'e'; //自瞄-大符-小符
	
		send[9] = 0x00; //云台标志位
		
		send[10] = 0x00; //反陀螺标志位
		//红3的id是3，蓝3是67
		if(robot_id <50){
			send[11] = 0X00; //对方颜色蓝色
		}
		else{
			send[11] = 0X01; //自方是蓝色，对面就是红色
		}
		send[12] = 0X00; //能量机关x轴补偿
		send[13] = 0X00;
		
		send[14] = 0X00; //能量机关y轴补偿
		send[15] = 0X10;
		
    // 发送数据包
    // usart1_tx_dma_enable(send, sizeof(send));
    USART1_Transmit_DMA(send, sizeof(send));
}

#endif
