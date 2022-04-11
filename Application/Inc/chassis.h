/*
 * chassis.h
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include "main.h"
#include "remote_control.h"
#include "can_receive.h"
#include "PID.h"

#define CHASSIS_VX_RC_SEN 		2.5f			// 遥控器乘以该比例系数
#define CHASSIS_VY_RC_SEN		2.5f			// 遥控器乘以该比例系数
#define CHASSIS_WZ_RC_SEN		2.5f			// 在 CHASSIS_NO_FOLLOW_YAW 模式下 遥控器yaw遥杆乘以该比例系数
#define CHASSIS_OPEN_RC_SEN		1				// 在 CHASSIS_OPEN 模式下 遥控器乘以该比例系数
#define CHASSIS_ANGLE_Z_RC_SEN 	0.000002f		// 在 CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW 模式下，遥控器的yaw遥杆增加到车体角度的比例

#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f	//m3508 rpm转化成底盘速度（m/s）的比例

typedef enum {
	CHASSIS_ZERO_FORCE  = 0,			//底盘无力, 跟没上电那样
	CHASSIS_NO_MOVE,					//底盘保持不动
	CHASSIS_OPEN,						//遥控器的值乘以比例成电流值 直接发送到can总线上
	CHASSIS_NO_FOLLOW_YAW,				//底盘不跟随云台角度
	CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW	//正常步兵底盘跟随云台
} chassis_behaviour_e;

typedef struct {
	int16_t vx;
	int16_t vy;
	int16_t wz;
} chassis_t;

extern chassis_behaviour_e chassis_behaviour_mode, chassis_behaviour_last;

extern	void Chassis_Set_Mode(void);
extern	void Chassis_Feedback_Update(void);

#endif
