/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-07-09 21:22:21
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-07-12 17:41:29
 * @FilePath: \smartcare:\X15\stm32\my_example\RM\H7\DM-balanceV1\User\Bsp\bsp_usart.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef BSP_USART_H
#define BSP_USART_H

#include "main.h"
#include "struct_typedef.h"
#include "fifo.h"
#include "protocol.h"
#include "remote_control.h"
#include "referee_usart_task.h"

#define BUFF_SIZE	25
#define CHASSIS_DATA_LENGTH 28

// 定义一个联合类型，用于将浮点数转换为字节
typedef union {
    float fp32;
    uint8_t bytes[4];
} fp32_to_bytes;
// 定义一个联合类型，用于将16位整数转换为字节
typedef union {
    int16_t int16;
    uint8_t bytes[2];
} int_to_bytes;

//云台收到的底盘数据,由串口发送到云台
typedef struct chassis_data
{
    //底盘速度信息
    fp32 chassis_vx;
    fp32 chassis_vy;
    fp32 chassis_wz;
    
    //底盘陀螺仪信息
    fp32 chassis_yaw;
    fp32 chassis_pitch;
    fp32 chassis_roll;

    //底盘超级电容电压
    fp32 cap_voltage;

    //底盘模式
    uint8_t chassis_mode;

    //机器人ID
    uint8_t robot_id;
}chassis_data_t;

//底盘收到的云台数据
typedef struct gimbal_data
{
    //云台板陀螺仪参数
    fp32 INS_yaw;
    fp32 INS_pitch;
    fp32 INS_roll;

    //云台电机参数（相对角度）
    fp32 motor_yaw;
    fp32 motor_pitch;

}gimbal_data_t;

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u
extern chassis_data_t chassis_data;
extern gimbal_data_t gimbal;

extern fp32 chassis_INS_yaw, chassis_INS_pitch, chassis_spin_speed;
extern uint8_t remote_buff[SBUS_RX_BUF_NUM];
extern uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];
extern uint8_t usart7_buf[ USART_RX_BUF_LENGHT ];//设置缓冲区
extern uint8_t usart10_buf[ USART10_RX_BUF_LENGHT ];//设置缓冲区

//遥控器
typedef struct
{
    uint16_t online;
		uint32_t sbus_recever_time;

    struct
    {
        int16_t ch[10];
    } rc;

    struct
    {
        /* STICK VALUE */
        int16_t left_vert;
        int16_t left_hori;
        int16_t right_vert;
        int16_t right_hori;
    } joy;
		
		struct
		{
			//l1 l2 r2 r1
			uint8_t swa;//2-Stop
			uint8_t swb;//3-Stop
			uint8_t swc;//3-Stop
			uint8_t swd;//2-Stop
		} toggle;

    struct
    {
        /* VAR VALUE */
        float a;
        float b;
    } var;

    struct
    {
        /* KEY VALUE */
        uint8_t l;
				uint8_t r;
    } key;
} remoter_t;

extern uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
extern unpack_data_t referee_unpack_obj;
extern fifo_s_t referee_fifo;

extern remoter_t remoter;
extern uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
extern unpack_data_t referee_unpack_obj;
extern fifo_s_t referee_fifo;
extern void USART1_Transmit_DMA(uint8_t *pData, uint16_t Size);
extern void USART7_Transmit(uint8_t *pData, uint16_t Size);
extern void USART10_Transmit(uint8_t *pData, uint16_t Size);
void USART1_Transmit(uint8_t *pData, uint16_t Size);
void USART1_Transmit_IT(uint8_t *pData, uint16_t Size);

#endif
