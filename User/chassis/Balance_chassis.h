#ifndef _BALANCE_CHASSIS_H
#define _BALANCE_CHASSIS_H
#include "chassis_task.h"
#include "chassis_behaviour.h"
#if ROBOT_CHASSIS == Balance_wheel
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "can_bsp.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
//#include "iwdg.h"
#include "slip_control.h"
#include "robot_param.h"

#define MOTOR_DISTANCE_TO_CENTER      0.22f

extern void chassis_init(chassis_move_t *chassis_move_init);
extern void chassis_set_mode(chassis_move_t *chassis_move_mode);
extern void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
extern void chassis_feedback_update(chassis_move_t *chassis_move_update);
extern void chassis_set_contorl(chassis_move_t *chassis_move_control);
extern void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
extern void chassis_send_cmd(chassis_move_t *chassis_send_cmd);
extern void observer(chassis_move_t *chassis_move, KalmanFilter_t *vxEstimateKF, KalmanFilter_t *vyEstimateKF);
#endif

#endif
