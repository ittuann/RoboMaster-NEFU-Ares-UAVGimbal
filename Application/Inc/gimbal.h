/*
 * gimbal.h
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#ifndef GIMBAL_H
#define GIMBAL_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "messageQueue.h"
#include "can_receive.h"
#include "PID.h"
#include "IMUFilter.h"
#include "chassis.h"
#include "gimbal.h"
#include "usart.h"
#include "anotc.h"

#define MOTOR_ECD_TO_RADPI	0.0219726563f	// 电机编码值转化成角度值 180 / 8192
#define MOTOR_ECD_TO_RAD2PI	0.0439453125f	// 电机编码值转化成角度值 360 / 8192

typedef struct {
	int16_t Max_Ecd;
    int16_t Min_Ecd;
	int16_t	Middle_Ecd;
	float	Max_Angle;
	float	Min_Angle;
} Gimbal_MachineTypeDef_t;

enum Gimbal_Num_e {
	Gimbal_Pitch = 0,
	Gimbal_Yaw
};

extern Gimbal_MachineTypeDef_t GimbalMachine_Pitch, GimbalMachine_Yaw;

extern float RC_Pitch_Sen, RC_Yaw_Sen;

extern void GimbalMachineTypeDef_Init(void);
extern void RC_Gimbal_SetMode(void);

#endif

