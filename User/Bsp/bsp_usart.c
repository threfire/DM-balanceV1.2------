#include "bsp_usart.h"
#include "string.h"
#include "usart.h"
#include "gimbal_task.h"
#include "Multi-axis_robotic_arm.h"
//串口5，接收遥控数据
#define SBUS_HEAD 0X0F
#define SBUS_END 0X00
#define REMOTE_RC_OFFSET 1024
#define REMOTE_TOGGLE_DUAL_VAL 1024
#define REMOTE_TOGGLE_THRE_VAL_A 600
#define REMOTE_TOGGLE_THRE_VAL_B 1400
#define DEAD_AREA	120


//串口1，接收裁判系统数据
uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];
uint8_t gimbal_receive_data[CHASSIS_DATA_LENGTH];
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
fifo_s_t referee_fifo;

uint8_t remote_buff[SBUS_RX_BUF_NUM];
remoter_t remoter;

//串口7 板间通讯
static uint8_t gimbal_data[USART_RX_BUF_LENGHT]={0};
uint8_t usart7_buf[ USART_RX_BUF_LENGHT ];//设置缓冲区
uint8_t data_send_from_pc[ USART_RX_BUF_LENGHT] = {0};
uint8_t data_send_from_chassis[ USART_RX_BUF_LENGHT] = {0};
uint8_t restart_array[10]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};//请求重新发送数据

//串口10 闲置
uint8_t usart10_buf[ USART10_RX_BUF_LENGHT ];//设置缓冲区
uint8_t prame[39];
uint8_t rx_index = 0;
uint8_t packet_complete = 0;
uint8_t test_evt ;
uint8_t test_size;
//串口1发送函数
void USART1_Transmit_DMA(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit_DMA(&huart1, pData, Size);
}

//串口1发送函数
void USART1_Transmit_IT(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit_IT(&huart1, pData, Size);
}

//串口1发送函数
void USART1_Transmit(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit(&huart1, pData, Size, 50);
}

uint8_t robot_id = 0;

gimbal_data_t gimbal;

/***** 云台发来的数据全都到这里面去了 *****/
chassis_data_t chassis_data;

//云台处理底盘发来的数据
//目前发送给云台的数据：imu的yaw值和pitch值和roll值、底盘行为模式、机器人ID、底盘速度vx、vy、wz
void chassis_to_gimbal(uint8_t *data_chassis)
{
	static fp32_to_bytes yaw_now, pitch_now, roll_now;		//imu的yaw值和pitch值和roll值
	static uint8_t chassis_behaviour_now;									//底盘行为模式
	static uint8_t Robot_ID;															//机器人ID
	static fp32_to_bytes chassis_vx_now;									//底盘速度vx
	static fp32_to_bytes chassis_vy_now;									//底盘速度vy
	static fp32_to_bytes chassis_wz_now;									//底盘速度wz

	// 解析IMU数据
	yaw_now.bytes[0] = data_chassis[1];
	yaw_now.bytes[1] = data_chassis[2];
	yaw_now.bytes[2] = data_chassis[3];
	yaw_now.bytes[3] = data_chassis[4];
	chassis_data.chassis_yaw = yaw_now.fp32;

	pitch_now.bytes[0] = data_chassis[5];
	pitch_now.bytes[1] = data_chassis[6];
	pitch_now.bytes[2] = data_chassis[7];
	pitch_now.bytes[3] = data_chassis[8];
	chassis_data.chassis_pitch = pitch_now.fp32;

	roll_now.bytes[0] = data_chassis[9];
	roll_now.bytes[1] = data_chassis[10];
	roll_now.bytes[2] = data_chassis[11];
	roll_now.bytes[3] = data_chassis[12];
	chassis_data.chassis_roll = roll_now.fp32;

	// 解析底盘行为模式
	chassis_behaviour_now = data_chassis[13];
	chassis_data.chassis_mode = chassis_behaviour_now;

	// 解析机器人ID
	Robot_ID = data_chassis[14];
	chassis_data.robot_id = Robot_ID;

	// 解析底盘速度
	chassis_vx_now.bytes[0] = data_chassis[15];
	chassis_vx_now.bytes[1] = data_chassis[16];
	chassis_vx_now.bytes[2] = data_chassis[17];
	chassis_vx_now.bytes[3] = data_chassis[18];
	chassis_data.chassis_vx = chassis_vx_now.fp32;

	chassis_vy_now.bytes[0] = data_chassis[19];
	chassis_vy_now.bytes[1] = data_chassis[20];
	chassis_vy_now.bytes[2] = data_chassis[21];
	chassis_vy_now.bytes[3] = data_chassis[22];
	chassis_data.chassis_vy = chassis_vy_now.fp32;

	chassis_wz_now.bytes[0] = data_chassis[23];
	chassis_wz_now.bytes[1] = data_chassis[24];
	chassis_wz_now.bytes[2] = data_chassis[25];
	chassis_wz_now.bytes[3] = data_chassis[26];
	chassis_data.chassis_wz = chassis_wz_now.fp32;
}

static void gimbal_memcpy_data_from_chassis(void)
{
	// 找到's'在源数组中的位置
	static int16_t start_index = -1;
	uint8_t temp[CHASSIS_DATA_LENGTH];
	
	for(uint8_t i = 0; i < CHASSIS_DATA_LENGTH; i++)
	{
		if(gimbal_receive_data[i] == 's')
		{
			start_index = i;
			break;
		}
	}
	
	// 如果找到's'，从该位置开始复制固定长度
	if (start_index != -1)
	{
		memcpy(temp, &gimbal_receive_data[start_index], CHASSIS_DATA_LENGTH);
	}
	
	chassis_to_gimbal(temp);
}

fp32 yaw_motor_relative_angle, GIMBAL_INS_yaw;
//底盘处理云台发来的数据
void gimba_to_chassis(uint8_t *gimbal_data) {
    fp32_to_bytes INS;
	INS.bytes[0] = gimbal_data[1]; 
	INS.bytes[1] = gimbal_data[2]; 
	INS.bytes[2] = gimbal_data[3]; 
	INS.bytes[3] = gimbal_data[4]; 
    GIMBAL_INS_yaw = INS.fp32;
	gimbal.INS_yaw = INS.fp32;
	
    fp32_to_bytes Motor_yaw;
	Motor_yaw.bytes[0] = gimbal_data[5]; 
	Motor_yaw.bytes[1] = gimbal_data[6]; 
	Motor_yaw.bytes[2] = gimbal_data[7]; 
	Motor_yaw.bytes[3] = gimbal_data[8]; 
	yaw_motor_relative_angle = Motor_yaw.fp32;
	gimbal.motor_yaw = Motor_yaw.fp32;
    
}
//处理图传发来的数据
#define FRAME1_HEAD1        0x52
#define FRAME1_HEAD2        0x8C
#define FRAME2_HEAD1        0xA9
#define FRAME2_HEAD2        0x53
#define DATA2_FIXED_LEN     17
#define DATA1_VAR_LEN_EMPTY 0
#define DATA1_VAR_LEN_FULL  39
#define PACKET_MIN_LEN      21
#define PACKET_MAX_LEN      60

uint32_t last_time = 0;
uint32_t now = 0;
uint8_t fps_count = 0;
uint8_t my_fps = 0;
float fps_average = 0;
uint16_t fps_sum;
uint16_t fps_num;
static uint8_t s_rem[PACKET_MAX_LEN - 1];  // 剩余字节缓冲区
static uint8_t s_rem_len = 0;
static uint16_t usart10_dma_last_pos = 0;
static void parse_data_nomal(const uint8_t *data, uint16_t len);

static void usart10_reset_rx_state(void)
{
    usart10_dma_last_pos = 0;
    s_rem_len = 0;
}

static void usart10_process_dma_rx(uint16_t pos)
{
    if (pos > USART10_RX_BUF_LENGHT) {
        usart10_reset_rx_state();
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, usart10_buf, USART10_RX_BUF_LENGHT);
        return;
    }

    if (pos == usart10_dma_last_pos) {
        return;
    }

    if (pos > usart10_dma_last_pos) {
        parse_data_nomal(&usart10_buf[usart10_dma_last_pos], pos - usart10_dma_last_pos);
    } else {
        if (usart10_dma_last_pos < USART10_RX_BUF_LENGHT) {
            parse_data_nomal(&usart10_buf[usart10_dma_last_pos], USART10_RX_BUF_LENGHT - usart10_dma_last_pos);
        }
        if (pos > 0U) {
            parse_data_nomal(usart10_buf, pos);
        }
    }

    usart10_dma_last_pos = pos;
}
static void parse_data(const uint8_t *data, uint16_t len)
{
    uint8_t buf[USART10_RX_BUF_LENGHT + PACKET_MAX_LEN];      // 创建临时拼接缓冲区
    uint16_t total = s_rem_len + len;		//计算拼接缓冲区总长
    memcpy(buf, s_rem, s_rem_len);		// 将上次未处理完的剩余数据复制到临时缓冲区开头
    memcpy(buf + s_rem_len, data, len);		// 将新接收的数据紧接着复制到临时缓冲区，形成连续的数据块

    uint16_t i = 0;
    while (i + PACKET_MIN_LEN <= total) 
		{
        // 查找帧头1
        if (buf[i] == FRAME1_HEAD1 && buf[i+1] == FRAME1_HEAD2) {
            // 检查空数据帧1情况
            if (i + 4 + DATA2_FIXED_LEN <= total && 
                buf[i+2] == FRAME2_HEAD1 && buf[i+3] == FRAME2_HEAD2) {
                // 无效包：数据帧1为空
//				PacketStats_Process(motor_data.seq);
                i += PACKET_MIN_LEN;            // 21字节
                continue;
            }
            // 检查有数据帧1情况
            uint16_t head2_pos = i + 2 + DATA1_VAR_LEN_FULL;  // i+41
            if (head2_pos + 2 + DATA2_FIXED_LEN <= total &&
                buf[head2_pos] == FRAME2_HEAD1 && buf[head2_pos+1] == FRAME2_HEAD2) {
                // 有效包：数据帧1有39字节
				memcpy(prame, &buf[i+2], 39);
				if (Unpack_CustomController_Frame_Simple(prame, &motor_data) == 0) 
				{
					fps_count ++;
					now = HAL_GetTick();
					if(now - last_time >=1000)
					{
						fps_num ++;
						my_fps = fps_count *1000/(now - last_time);
						if(fps_num > 4)
						{
							fps_sum += my_fps;
							fps_average = (float)fps_sum / (fps_num-4); 
						}
						last_time = now;
						fps_count = 0;
					}

				}
        i += PACKET_MAX_LEN;            // 60字节
        continue;
            }
        }
        // 未找到有效包，跳过当前字节继续搜索
        i++;
    }

    // 保存剩余未处理字节
    s_rem_len = total - i;
    if (s_rem_len > 0) {
        memcpy(s_rem, &buf[i], s_rem_len);
    }
}
/*平时*/
static void parse_data_nomal(const uint8_t *data, uint16_t len)
{
    uint8_t buf[USART10_RX_BUF_LENGHT + 39];      // 创建临时拼接缓冲区
    uint16_t total = s_rem_len + len;		//计算拼接缓冲区总长
    memcpy(buf, s_rem, s_rem_len);		// 将上次未处理完的剩余数据复制到临时缓冲区开头
    memcpy(buf + s_rem_len, data, len);		// 将新接收的数据紧接着复制到临时缓冲区，形成连续的数据块

    uint16_t i = 0;
    while (i + 39 <= total) 
		{
        // 查找帧头1
        if (buf[i] == 0XA5 && buf[i+1] == 0X1E)
				{  
					memcpy(prame, &buf[i], 39);
					if (Unpack_CustomController_Frame_Simple(prame, &motor_data) == 0) 
					{
						fps_count ++;
						now = HAL_GetTick();
						if(now - last_time >=1000)
						{
							fps_num ++;
							my_fps = fps_count *1000/(now - last_time);
							if(fps_num > 4)
							{
								fps_sum += my_fps;
								fps_average = (float)fps_sum / (fps_num-4); 
							}
							last_time = now;
							fps_count = 0;
						}
					}
          i += 39;            // 60字节
          continue;
        }
				// 未找到有效包，跳过当前字节继续搜索
        i++;
    }

    // 保存剩余未处理字节
    s_rem_len = total - i;
    if (s_rem_len > 0) {
        memcpy(s_rem, &buf[i], s_rem_len);
    }
}
//串口7
/**
  * @brief  通过串口7发送数据
  * @param  pData: 要发送的数据缓冲区
  * @param  Size: 要发送的数据大小
  * @retval 无
  */
void USART7_Transmit(uint8_t *pData, uint16_t Size)
{
    HAL_UART_Transmit(&huart7, pData, Size, 10);
}
void USART10_Transmit(uint8_t *pData, uint16_t Size){
	HAL_UART_Transmit(&huart10, pData, Size, HAL_MAX_DELAY);
}


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	 
//    if (huart->Instance == USART10) {
//		parse_data(usart10_buf, USART10_RX_BUF_LENGHT);
//        HAL_UART_Receive_IT(&huart10, usart10_buf, USART10_RX_BUF_LENGHT );
//    }
//}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == USART1)
	{
		/* removed: uart_dma_rx_event_handler */
//		if (Size <= USART_RX_BUF_LENGHT)
//		{
//			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_buf[0], USART_RX_BUF_LENGHT);	// 接收完毕后重启
//			fifo_s_puts(&referee_fifo, (char*)usart1_buf[0], USART_RX_BUF_LENGHT);
//		}
//		else 
//		{
//			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_buf[1], USART_RX_BUF_LENGHT); // 接收完毕后重启
//			fifo_s_puts(&referee_fifo, (char*)usart1_buf[1], USART_RX_BUF_LENGHT);					   
//		}
		gimbal_memcpy_data_from_chassis();
	}
	if(huart->Instance == UART5)
	{
		if (Size <= RC_FRAME_LENGTH)
		{
			sbus_to_rc(remote_buff, &rc_ctrl);
		}
		else if(Size > RC_FRAME_LENGTH) // 错误处理
		{	
			memset(remote_buff, 0, SBUS_RX_BUF_NUM);				   
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_buff, SBUS_RX_BUF_NUM);
	}
	if(huart->Instance == UART7){

	}

	if(huart->Instance == USART10)
	{
		 HAL_UART_RxEventTypeTypeDef evt = HAL_UARTEx_GetRxEventType(huart);
			test_evt = evt;
			test_size = Size;
		usart10_process_dma_rx(Size);
		return;
	}
	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == USART1){
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_buf[0], USART_RX_BUF_LENGHT); // 接收发生错误后重启
		memset(usart1_buf, 0,  USART_RX_BUF_LENGHT * 2);		
	}
	if(huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_buff, SBUS_RX_BUF_NUM); // 接收发生错误后重启
		memset(remote_buff, 0, SBUS_RX_BUF_NUM);							   // 清除接收缓存		
	}
	if(huart->Instance == UART7){

	}
	if(huart->Instance == USART10){
		usart10_reset_rx_state();
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, usart10_buf, USART10_RX_BUF_LENGHT);
		memset(usart10_buf, 0, USART10_RX_BUF_LENGHT);
	}
}

/* 转发 TX 完成回调 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* removed: uart_dma_tx_cplt_handler */
}

