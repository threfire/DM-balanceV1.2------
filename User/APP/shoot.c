/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "shoot_3508.h"
#include "main.h"

#include "cmsis_os.h"

//#include "bsp_laser.h"
#include "bsp_PWM.h"
#include "arm_math.h"
#include "referee.h"

#include "can_bsp.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"

#define shoot_fric1_on(shoot) fric1_on((shoot)) //摩擦轮1宏定义
#define shoot_fric2_on(shoot) fric2_on((shoot)) //摩擦轮2宏定义
#define shoot_fric_off()    fric_off()      //关闭两个摩擦轮

//#define shoot_laser_on()    laser_on()      //激光开启宏定义
//#define shoot_laser_off()   laser_off()     //激光关闭宏定义
////微动开关IO
//#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)
__weak void fric1_on(shoot_control_t *shoot);
__weak void fric2_on(shoot_control_t *shoot);
__weak void fric_off(void);
/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
__weak void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
__weak void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
__weak void trigger_motor_turn_back(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
__weak void shoot_bullet_control(void);



shoot_control_t shoot_control;          //射击数据


/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
__weak void shoot_init(void);

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据
		if( shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_R || (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_mid(shoot_control.shoot_rc->rc.s[GIMBAL_MODE_CHANNEL])) ){
			Bombbay_on();
		}
		else if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_G || (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_mid(shoot_control.shoot_rc->rc.s[GIMBAL_MODE_CHANNEL])) ){
			Bombbay_off();
		}
		
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = CONTINUE_TRIGGER_SPEED;
    }
    else if(shoot_control.shoot_mode ==SHOOT_READY_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        shoot_control.trigger_speed_set = READY_TRIGGER_SPEED;
        trigger_motor_turn_back();
        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
         shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        
        shoot_control.given_current = 0;
				fric_off();
    }
    else
    {
        //shoot_laser_on(); //激光开启
        //计算拨弹轮电机PID
        PID_Calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
        if(shoot_control.shoot_mode == SHOOT_READY ||shoot_control.shoot_mode == SHOOT_STOP || shoot_control.shoot_mode ==  SHOOT_DONE)
        {
            shoot_control.given_current = 0;
        }
    }
		//计算摩擦轮电机的电流值
    shoot_fric1_on(&shoot_control);
    shoot_fric2_on(&shoot_control);
		//发送摩擦轮电机的电流值
		if(shoot_control.fric1.shoot_motor_measure != NULL && shoot_control.fric2.shoot_motor_measure != NULL)
		{
			//过热保护，需要启动摩擦轮才发数据，不然就是0
				if(shoot_control.fric1.shoot_motor_measure->temperate < 80.0f &&shoot_control.fric2.shoot_motor_measure->temperate < 80.0f && shoot_control.shoot_mode >  SHOOT_STOP )
				{
					#if ROBOT_FRICTION == friction_3508
						CAN_cmd_friction(shoot_control.fric1.give_current, shoot_control.fric2.give_current,0, 0);
					#endif
					#if ROBOT_FRICTION == friction_double
						CAN_cmd_friction(shoot_control.fric1.give_current, shoot_control.fric2.give_current,shoot_control.fric1_.give_current, shoot_control.fric2_.give_current);
					#endif
				}
				else
				{
						CAN_cmd_friction(0,0,0, 0);
				}
		}
		else
		{
				CAN_cmd_friction(0,0,0, 0);
		}
    return shoot_control.given_current;
}
