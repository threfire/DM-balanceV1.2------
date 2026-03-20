#include "message_task.h"
#include "gimbal_task.h"
#include "chassis_power_control.h"

#include "main.h"

#include "cmsis_os.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "can_bsp.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "auto_aim.h"
#include "robot_param.h"

//具体实现在各个兵种文件里面
__weak void send_data_to_chassis(gimbal_control_t *feedback_update);
__weak void send_to_gimbal(void);
__weak void send_to_minipc(void);


void message_task(void const *pvParameters)
{

    // 初始化延迟
    vTaskDelay(MESSAGE_TASK_INIT_TIME);
    while(1) {
      #if ROBOT_BOARD == gimbal_board
        send_data_to_chassis(&gimbal_control);
				send_to_minipc();
				vTaskDelay(MESSAGE_TASK_TIME);  // 每次循环的延迟 
      #endif // DEBUG
			
      #if ROBOT_BOARD == chassis_board
        send_to_gimbal();
				vTaskDelay(MESSAGE_TASK_TIME);
      #endif // DEBUG
      
    }
}

