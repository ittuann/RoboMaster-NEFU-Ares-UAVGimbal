/*
 * CAN_receive.h
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#ifndef CAN_RECEIVER_H_
#define CAN_RECEIVER_H_

#include "main.h"
#include "user_lib.h"
#include "can.h"
#include "messageQueue.h"
#include "PID.h"

#define CHASSIS_CAN hcan1
#define SHOOT_CAN	hcan1
#define GIMBAL_CAN	hcan1

enum CAN_MotorID_e {
    CAN_3508_ALL_ID = 0x200,	//标识符
	CAN_3508_ETC_ID = 0x1FF,	//标识符(5-8
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	CAN_2006_M_ID = 0x206,
	CAN_2006_L_ID = 0x207,
	CAN_2006_R_ID = 0x208,
	CAN_6020_ALL_ID = 0x1FF,	//标识符
	CAN_6020_ETC_ID = 0x2FF,	//标识符(5-7
	CAN_PITCH_MOTOR_ID = 0x205,
	CAN_YAW_MOTOR_ID = 0x209,
};

typedef struct __attribute__((packed))
{
	int16_t	speed_rpm;
    int16_t	ecd;
    int16_t	torque_current;
    int16_t	temperate;
} motor_measure_t;

#if DEBUGMODE
	extern motor_measure_t motorData[4];		// motorData[MOTOR_NUM]
	extern void CAN_cmd_chassis_reset_ID(void);
#endif

extern void CAN_Cmd_C620(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, bool_t etcID);
extern void CAN_Cmd_GM6020(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, bool_t etcID);
extern void CAN_Cmd_GM3510(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3);
extern void Motor_Prevent_InitMadness(void);

#endif
