/*
 * shoot.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "shoot.h"

extern TIM_HandleTypeDef htim1, htim3;

#define SHOOT_L_TIM				htim1
#define SHOOT_R_TIM				htim1
#define SHOOT_LASER_TIM			htim3

#define SHOOT_L_TIM_CHANNEL		TIM_CHANNEL_2
#define SHOOT_R_TIM_CHANNEL		TIM_CHANNEL_3
#define SHOOT_LASER_TIM_CHANNEL	TIM_CHANNEL_3


typedef enum {
	DONOT_SHOOT = 0,
	SHOOT_NORMAL,		// 正常射击
	SHOOT_SLOW,			// 慢速测试射击
} shoot_behaviour_e;
shoot_behaviour_e ShootBehaviour_Mode = DONOT_SHOOT, ShootBehaviour_Last;	// 云台行为模式

static motor_measure_t MotorShoot[1];					// 发射机构电机消息队列
static Ramp_t Stir, SnailL, SnailR;						// 发射机构缓起步
static uint16_t SnailL_Set = 1000, SnailR_Set = 1000;	// 测试

#if DEBUGMODE
	static Frequency_t Shoot_Freqency;
	static float Shoot_Freq = 0.000f;					// 枪体任务运行频率
#endif

/**
  * @brief		Snail电机PWM初始化
  */
void Snail_Init(void)
{
	HAL_TIM_Base_Start(&htim1);										// 开启定时器
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);						// 使对应定时器的对应通道开始PWM输出
	HAL_TIM_PWM_Start(&SHOOT_L_TIM, SHOOT_L_TIM_CHANNEL);
	HAL_TIM_PWM_Start(&SHOOT_R_TIM, SHOOT_R_TIM_CHANNEL);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SetCompare(&SHOOT_L_TIM, SHOOT_L_TIM_CHANNEL, 1000);
	__HAL_TIM_SetCompare(&SHOOT_R_TIM, SHOOT_R_TIM_CHANNEL, 1000);
}

/**
  * @brief		激光PWM初始化
  */
void Laser_Init(void)
{
	HAL_TIM_Base_Start(&SHOOT_LASER_TIM);							// 开启定时器
	HAL_TIM_PWM_Start(&SHOOT_LASER_TIM, SHOOT_LASER_TIM_CHANNEL);	// 使激光定时器对应通道PWM输出
}

/**
  * @brief		缓慢设置Snail电机PWM
  */
void Snail_Set(void)
{
	__HAL_TIM_SetCompare(&SHOOT_L_TIM, SHOOT_L_TIM_CHANNEL, (uint16_t)(RampCalc(&SnailL, SnailL_Set, 10)));
	__HAL_TIM_SetCompare(&SHOOT_R_TIM, SHOOT_R_TIM_CHANNEL, (uint16_t)(RampCalc(&SnailR, SnailR_Set, 10)));
}

void RC_Shoot_SetMode(void)
{
	if (switch_is_up(rc_ctrl.rc.s[RC_SWLeft])) {
		ShootBehaviour_Mode = DONOT_SHOOT;
	} else if (switch_is_mid(rc_ctrl.rc.s[RC_SWLeft])) {
		ShootBehaviour_Mode = DONOT_SHOOT;
	} else if (switch_is_down(rc_ctrl.rc.s[RC_SWLeft])) {
		ShootBehaviour_Mode = SHOOT_NORMAL;
	} else {
		ShootBehaviour_Mode = DONOT_SHOOT;
	}
}

/**
  * @brief		射击控制任务
  */
void ShootTask(void const * argument)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = pdMS_TO_TICKS(5UL);	// 绝对延时5ms

	// 挂起调度器
//	vTaskSuspendAll();

	SnailL.SetVal = SnailL.NowVal = SnailL_Set;
	SnailR.SetVal = SnailR.NowVal = SnailR_Set;

	// 唤醒调度器
//	xTaskResumeAll();

	// 用当前tick时间初始化 pxPreviousWakeTime
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		// 任务绝对延时
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		// 从消息队列中获取数据
		xQueueReceive(messageQueue[MotorID_ShootM], &MotorShoot[0], (1 / portTICK_RATE_MS));

		// 遥控器设置设置云台控制模式
		RC_Shoot_SetMode();

		// 切换模式时清除和保存状态
		if (ShootBehaviour_Mode != ShootBehaviour_Last) {
			__HAL_TIM_SetCompare(&htim1, SHOOT_L_TIM_CHANNEL, 1000);
			__HAL_TIM_SetCompare(&htim1, SHOOT_R_TIM_CHANNEL, 1000);
			SnailL_Set = SnailR_Set = 1000;
			Stir.SetVal = Stir.NowVal = 0;
			SnailL.SetVal = SnailL.NowVal = SnailL_Set;
			SnailR.SetVal = SnailR.NowVal = SnailR_Set;
			PID_Clear(&PID_Mortor_Speed[SpeedPID_ShootM]);
		}

		if (ShootBehaviour_Mode == SHOOT_NORMAL) {
			// 开启摩擦轮
			SnailL_Set = SnailR_Set = 1200;
			Snail_Set();
			// 拨弹轮PID计算
			PID_Mortor_Speed[SpeedPID_ShootM].EX_Val = 1000;
			PID_Calc(&PID_Mortor_Speed[SpeedPID_ShootM], MotorShoot[0].speed_rpm, RampCalc(&Stir, PID_Mortor_Speed[SpeedPID_ShootM].EX_Val, 50));
		}

		// 发送电机控制电流 在Gimbal任务中

		// 记录上次控制模式
		ShootBehaviour_Last = ShootBehaviour_Mode;

//		CAN_Cmd_C620(&SHOOT_CAN, (int16_t)PID_Mortor_Speed[SpeedPID_ShootM].Output, 0, 0, 0, 1);

//		Wireless_Send();

		#if DEBUGMODE
			Shoot_Freq = GetFrequency(&Shoot_Freqency);
		#endif
	}
}

/**
  * @brief		测试控制任务
  */
void TestTask(void const * argument)
{
//	OLED_init();

	while(1)
	{
		osDelay(1);
	}
}
