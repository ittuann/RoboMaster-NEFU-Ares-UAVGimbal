/*
 * chassis.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "chassis.h"

//static motor_measure_t MotorChassis[4];		// 底盘电机数据消息队列
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE, chassis_behaviour_last;	// 底盘行为模式
chassis_t chassis;							// 底盘整体状态

//#if DEBUGMODE
//	static Frequency_t Chassis_Freqency;
//	static float Chassis_Freq = 0.000f;		// 底盘任务运行频率
//#endif


/**
  * @brief			遥控器开关控制底盘模式
  */
void Chassis_Set_Mode(void)
{
	if (switch_is_up(rc_ctrl.rc.s[RC_SWLeft])) {
		chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
	} else if (switch_is_mid(rc_ctrl.rc.s[RC_SWLeft])) {
		chassis_behaviour_mode = CHASSIS_NO_MOVE;
	} else if (switch_is_down(rc_ctrl.rc.s[RC_SWLeft])) {
		chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
	} else {
		chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
	}
}

/**
  * @brief			底盘测量数据更新
  */
void Chassis_Feedback_Update(void)
{
//	chassis.vx = (- MotorChassis[Chassis_M1].ecd + MotorChassis[Chassis_M2].ecd + MotorChassis[Chassis_M3].ecd - MotorChassis[Chassis_M4].ecd) * 0.25f * M3508_MOTOR_RPM_TO_VECTOR;
//	chassis.vy = (- MotorChassis[Chassis_M1].ecd - MotorChassis[Chassis_M2].ecd + MotorChassis[Chassis_M3].ecd + MotorChassis[Chassis_M4].ecd) * 0.25f * M3508_MOTOR_RPM_TO_VECTOR;
//	chassis.wz = (- MotorChassis[Chassis_M1].ecd - MotorChassis[Chassis_M2].ecd - MotorChassis[Chassis_M3].ecd - MotorChassis[Chassis_M4].ecd) * 0.25f * M3508_MOTOR_RPM_TO_VECTOR;
}

/**
  * @brief			底盘控制任务
  */
void ChassisTask(void const * argument)
{
//	portTickType xLastWakeTime;
//	const portTickType xFrequency = 2 / portTICK_RATE_MS;	// 延时2ms
//
//	// 用当前tick时间初始化pxPreviousWakeTime
//	xLastWakeTime = xTaskGetTickCount();
//
//	while(1)
//	{
//		// 任务绝对延时
//		vTaskDelayUntil(&xLastWakeTime, xFrequency);
//
//		// 从消息队列中获取数据
//		xQueueReceive(messageQueue[Chassis_M1], &MotorChassis[Chassis_M1], (1 / portTICK_RATE_MS));
//		xQueueReceive(messageQueue[Chassis_M2], &MotorChassis[Chassis_M2], (1 / portTICK_RATE_MS));
//		xQueueReceive(messageQueue[Chassis_M3], &MotorChassis[Chassis_M3], (1 / portTICK_RATE_MS));
//		xQueueReceive(messageQueue[Chassis_M4], &MotorChassis[Chassis_M4], (1 / portTICK_RATE_MS));
//
//		// 遥控器设置设置底盘控制模式
//		Chassis_Set_Mode();
//
//		// 切换模式时清除和保存状态
//		if (chassis_behaviour_mode != chassis_behaviour_last) {
//			PID_Clear(&PID_Mortor_Speed[SpeedPID_Chassis_M1]);
//			PID_Clear(&PID_Mortor_Speed[SpeedPID_Chassis_M2]);
//			PID_Clear(&PID_Mortor_Speed[SpeedPID_Chassis_M3]);
//			PID_Clear(&PID_Mortor_Speed[SpeedPID_Chassis_M4]);
//		}
//
//		// 底盘不同模式下的控制
//		if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE) {
//			PID_Mortor_Speed[SpeedPID_Chassis_M1].Output = PID_Mortor_Speed[SpeedPID_Chassis_M2].Output
//			= PID_Mortor_Speed[SpeedPID_Chassis_M3].Output = PID_Mortor_Speed[SpeedPID_Chassis_M4].Output = 0;
//		} else if (chassis_behaviour_mode == CHASSIS_OPEN) {
//			RC_Chassis_Diagram();
//			PID_Mortor_Speed[SpeedPID_Chassis_M1].Output = PID_Mortor_Speed[SpeedPID_Chassis_M1].EX_Val;
//			PID_Mortor_Speed[SpeedPID_Chassis_M2].Output = PID_Mortor_Speed[SpeedPID_Chassis_M2].EX_Val;
//			PID_Mortor_Speed[SpeedPID_Chassis_M3].Output = PID_Mortor_Speed[SpeedPID_Chassis_M3].EX_Val;
//			PID_Mortor_Speed[SpeedPID_Chassis_M4].Output = PID_Mortor_Speed[SpeedPID_Chassis_M4].EX_Val;
//		} else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW || chassis_behaviour_mode == CHASSIS_NO_MOVE) {
//			if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW) {
//				RC_Chassis_Diagram();
//			} else if (chassis_behaviour_mode == CHASSIS_NO_MOVE) {
//				PID_Mortor_Speed[SpeedPID_Chassis_M1].EX_Val = PID_Mortor_Speed[SpeedPID_Chassis_M2].EX_Val = PID_Mortor_Speed[SpeedPID_Chassis_M3].EX_Val = PID_Mortor_Speed[SpeedPID_Chassis_M4].EX_Val = 0;
//			}
//
//			PID_Calc(&PID_Mortor_Speed[SpeedPID_Chassis_M1], MotorChassis[Chassis_M1].speed_rpm, PID_Mortor_Speed[Chassis_M1].EX_Val);
//			PID_Calc(&PID_Mortor_Speed[SpeedPID_Chassis_M2], MotorChassis[Chassis_M2].speed_rpm, PID_Mortor_Speed[Chassis_M2].EX_Val);
//			PID_Calc(&PID_Mortor_Speed[SpeedPID_Chassis_M3], MotorChassis[Chassis_M3].speed_rpm, PID_Mortor_Speed[Chassis_M3].EX_Val);
//			PID_Calc(&PID_Mortor_Speed[SpeedPID_Chassis_M4], MotorChassis[Chassis_M4].speed_rpm, PID_Mortor_Speed[Chassis_M4].EX_Val);
//		}
//
//		// 发送电机控制电流
//		CAN_Cmd_Chassis((int16_t)(PID_Mortor_Speed[SpeedPID_Chassis_M1].Output), (int16_t)(PID_Mortor_Speed[SpeedPID_Chassis_M2].Output), (int16_t)(PID_Mortor_Speed[SpeedPID_Chassis_M3].Output), (int16_t)(PID_Mortor_Speed[SpeedPID_Chassis_M4].Output));
//
//		// 底盘测量数据更新
//		Chassis_Feedback_Update();
//		// 记录上次控制模式
//		chassis_behaviour_last = chassis_behaviour_mode;
//
//		#if DEBUGMODE
//			Chassis_Freq = GetFrequency(&Chassis_Freqency);
//		#endif
//	}
}
