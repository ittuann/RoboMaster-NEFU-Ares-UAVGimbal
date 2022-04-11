/*
 * gimbal.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "gimbal.h"

#define NORMALIZE_ANGLE180(angle)	angle = ((angle) > 180) ? ((angle) - 360) : (((angle) < -180) ? (angle) + 360 : angle)

typedef enum {
	GIMBAL_PROTECT = 0,
	GIMBAL_RELATIVE_ANGLE,			// 云台编码值控制
	GIMBAL_ABSOLUTE_ANGLE,			// 云台陀螺仪控制
} gimbal_behaviour_e;
gimbal_behaviour_e GimbalBehaviour_Mode = GIMBAL_PROTECT, GimbalBehaviour_Last;	// 云台行为模式

static motor_measure_t MotorGimbal[2];							// 云台电机消息队列
Gimbal_MachineTypeDef_t GimbalMachine_Pitch, GimbalMachine_Yaw;	// 云台机械限位数据
static float Gimbal_Angle[3] = {0.0f};							// 云台姿态消息队列 roll翻滚角 pitch俯仰角 yaw航向角/偏航角
static float Gimbal_Gyro[3] = {0.0f};							// 云台加速度消息队列
float RC_Pitch_Sen	= -0.0000350f;								// 遥控器Pitch比例系数
float RC_Yaw_Sen	= -0.0000750f;								// 遥控器Yaw比例系数

#if DEBUGMODE
	static Frequency_t Gimbal_Freqency;
	static float Gimbal_Freq = 0.000f;							// 云台任务运行频率
#endif


/**
  * @brief		云台机械中值和限位初始化
  */
void GimbalMachineTypeDef_Init(void)
{
	GimbalMachine_Pitch.Max_Ecd = 4950;
	GimbalMachine_Pitch.Min_Ecd = 3850;
	GimbalMachine_Pitch.Middle_Ecd = 4300;
	GimbalMachine_Pitch.Min_Angle = (GimbalMachine_Pitch.Min_Ecd - GimbalMachine_Pitch.Middle_Ecd) * MOTOR_ECD_TO_RADPI;
	GimbalMachine_Pitch.Max_Angle = (GimbalMachine_Pitch.Max_Ecd - GimbalMachine_Pitch.Middle_Ecd) * MOTOR_ECD_TO_RADPI;

	GimbalMachine_Yaw.Max_Ecd = 5000;
	GimbalMachine_Yaw.Min_Ecd = 1200;
	GimbalMachine_Yaw.Middle_Ecd = 3100;
	GimbalMachine_Yaw.Min_Angle = (GimbalMachine_Yaw.Min_Ecd - GimbalMachine_Yaw.Middle_Ecd) * MOTOR_ECD_TO_RADPI;
	GimbalMachine_Yaw.Max_Angle = (GimbalMachine_Yaw.Max_Ecd - GimbalMachine_Yaw.Middle_Ecd) * MOTOR_ECD_TO_RADPI;
}

/**
  * @brief		遥控器开关控制云台模式
  */
void RC_Gimbal_SetMode(void)
{
	if (switch_is_up(rc_ctrl.rc.s[RC_SWRight])) {
		GimbalBehaviour_Mode = GIMBAL_PROTECT;
	} else if (switch_is_mid(rc_ctrl.rc.s[RC_SWRight])) {
		GimbalBehaviour_Mode = GIMBAL_RELATIVE_ANGLE;
	} else if (switch_is_down(rc_ctrl.rc.s[RC_SWRight])) {
		GimbalBehaviour_Mode = GIMBAL_ABSOLUTE_ANGLE;
	} else {
		GimbalBehaviour_Mode = GIMBAL_PROTECT;
	}
}

/**
  * @brief		云台控制任务
  */
void GimbalTask(void const * argument)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = pdMS_TO_TICKS(1UL);	// 绝对延时1ms

	PID_Init();
	GimbalMachineTypeDef_Init();

	// 用当前tick时间初始化 pxPreviousWakeTime
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		// 任务绝对延时
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		// 从消息队列中获取数据
		xQueueReceive(messageQueue[IMUANGLE], &Gimbal_Angle, (1UL / portTICK_RATE_MS));
		xQueueReceive(messageQueue[IMUGYRO], &Gimbal_Gyro, (1UL / portTICK_RATE_MS));
		xQueueReceive(messageQueue[MotorID_GimbalPitch], &MotorGimbal[Gimbal_Pitch], (1UL / portTICK_RATE_MS));
		xQueueReceive(messageQueue[MotorID_GimbalYaw], &MotorGimbal[Gimbal_Yaw], (1UL / portTICK_RATE_MS));

		// 遥控器设置设置云台控制模式
		RC_Gimbal_SetMode();

		// 切换模式时清除和保存状态
		if (GimbalBehaviour_Mode != GimbalBehaviour_Last) {
			PID_Clear(&PID_Mortor_Speed[SpeedPID_GimbalPitch]);
			PID_Clear(&PID_Mortor_Speed[SpeedPID_GimbalYaw]);
			PID_Clear(&PID_Mortor_Angle[AnglePID_GimbalPitch]);
			PID_Clear(&PID_Mortor_Angle[AnglePID_GimbalYaw]);
			PID_Clear(&PID_Mortor_Angle[AnglePID_GimbalPitch_Relative]);

			PID_Mortor_Angle[AnglePID_GimbalPitch_Relative].Now_Val = GimbalMachine_Pitch.Min_Angle;
			PID_Mortor_Angle[AnglePID_GimbalPitch].Now_Val = GimbalMachine_Pitch.Min_Angle;
			PID_Mortor_Angle[AnglePID_GimbalYaw].Now_Val = GimbalMachine_Yaw.Min_Angle;
		}

		// 云台不同模式下的控制
		if (GimbalBehaviour_Mode == GIMBAL_PROTECT) {
			PID_Mortor_Speed[SpeedPID_GimbalYaw].Output = PID_Mortor_Speed[SpeedPID_GimbalPitch].Output = 0;
			CAN_Cmd_GM3510(&GIMBAL_CAN, 0, 0, 0);
		} else if (GimbalBehaviour_Mode == GIMBAL_RELATIVE_ANGLE) {
			// 遥控器分解
			RC_Gimbal_Diagram();

			// 角度环
			PID_Calc(&PID_Mortor_Angle[AnglePID_GimbalPitch_Relative], ((MotorGimbal[Gimbal_Pitch].ecd - GimbalMachine_Pitch.Middle_Ecd) * MOTOR_ECD_TO_RADPI), PID_Mortor_Angle[AnglePID_GimbalPitch_Relative].EX_Val);
			PID_Calc(&PID_Mortor_Angle[AnglePID_GimbalYaw], ((MotorGimbal[Gimbal_Yaw].ecd - GimbalMachine_Yaw.Middle_Ecd) * MOTOR_ECD_TO_RADPI), PID_Mortor_Angle[AnglePID_GimbalYaw].EX_Val);

			// 速度环
//			PID_Calc(&PID_Mortor_Speed[SpeedPID_GimbalPitch], MotorGimbal[Gimbal_Pitch].speed_rpm, PID_Mortor_Angle[AnglePID_GimbalPitch_Relative].Output);
			PID_Calc(&PID_Mortor_Speed[SpeedPID_GimbalYaw], MotorGimbal[Gimbal_Yaw].speed_rpm, PID_Mortor_Angle[AnglePID_GimbalYaw].Output);

			// 发送电机控制电流
			CAN_Cmd_GM3510(&GIMBAL_CAN, (int16_t)PID_Mortor_Angle[AnglePID_GimbalPitch_Relative].Output, 0, 0);
		} else if (GimbalBehaviour_Mode == GIMBAL_ABSOLUTE_ANGLE) {
			// 遥控器分解
			RC_Gimbal_Diagram();

			// 云台角度环
//			PID_Calc(&PID_Mortor_Angle[AnglePID_GimbalPitch], Gimbal_Angle[1], PID_Mortor_Angle[AnglePID_GimbalPitch].EX_Val);			// pitch	俯仰角 绕Y轴旋转
//			PID_Calc(&PID_Mortor_Angle[AnglePID_GimbalYaw], Gimbal_Angle[2], PID_Mortor_Angle[AnglePID_GimbalYaw].EX_Val);				// yaw		航向角/偏航角 绕Z轴旋转
			PID_Calc(&PID_Mortor_Angle[AnglePID_GimbalPitch_Relative], -Gimbal_Angle[1], PID_Mortor_Angle[AnglePID_GimbalPitch].EX_Val);

			// 云台角速度环
//			PID_Calc(&PID_Mortor_Speed[SpeedPID_GimbalPitch], Gimbal_Gyro[0], PID_Mortor_Angle[AnglePID_GimbalPitch].Output);
//			PID_Calc(&PID_Mortor_Speed[SpeedPID_GimbalYaw], Gimbal_Gyro[2], PID_Mortor_Angle[AnglePID_GimbalYaw].Output);

			// 发送电机控制电流
//			CAN_Cmd_GM3510(&GIMBAL_CAN, (int16_t)PID_Mortor_Speed[SpeedPID_GimbalPitch].Output, 0, 0);
			CAN_Cmd_GM3510(&GIMBAL_CAN, (int16_t)PID_Mortor_Angle[AnglePID_GimbalPitch_Relative].Output, 0, 0);

		}

		// 发送电机控制电流
		CAN_Cmd_GM6020(&GIMBAL_CAN, (int16_t)(PID_Mortor_Speed[SpeedPID_GimbalYaw].Output), 0, 0, 0, 1);

		// 记录上次控制模式
		GimbalBehaviour_Last = GimbalBehaviour_Mode;

		#if DEBUGMODE
			Gimbal_Freq = GetFrequency(&Gimbal_Freqency);
		#endif
  }
}
