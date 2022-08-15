/*
 * can_receive.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "can_receive.h"
#include <stdbool.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#if DEBUGMODE
	motor_measure_t motorData[MOTOR_NUM];	// 所有电机数据
#endif


/**
  * @brief          电机返回数据协议解析
  */
static void get_motor_measure(motor_measure_t *motor, uint8_t data[8])
{
	motor->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
	motor->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
	motor->torque_current = (uint16_t)((data)[4] << 8 | (data)[5]);
	motor->temperate = (data)[6];

	// 电机位置过零处理 避免出现位置数据突变的情况
//	if (motor->ecd - motor->ecd_last > 4096) {
//		motor->round--;
//	} else if (motor->ecd - motor->ecd_last < -4096) {
//		motor->round ++ ;
//	}
//	motor->position = motor->ecd + motor->round * 8192;
	// 将电机速度反馈值由无符号整型转变为有符号整型
//	if (motor->speed_rpm > 32768) {
//		motor->speed_rpm -= 65536;
//	}
}

/**
  * @brief			HAL库CAN FIFO0接受邮箱中断（Rx0）回调函数
  * @param			hcan : CAN句柄指针
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;	// 不请求上下文切换
	CAN_RxHeaderTypeDef RxHeader;							// CAN通信协议头
	uint8_t rx_data[8] = {0};								// 暂存CAN接收数据
	motor_measure_t motorDataTmp;							// 电机数据
	uint8_t i = 0;
	
	if (hcan == &hcan1)
	{
		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_data) == HAL_OK)	// 接收CAN总线上发送来的数据
		{
			// 对应电机向总线上发送的反馈的标识符和程序电机序号
			switch (RxHeader.StdId) {
				case CAN_2006_M_ID : i = MotorID_ShootM; break;
				case CAN_PITCH_MOTOR_ID : i = MotorID_GimbalPitch; break;
				case CAN_YAW_MOTOR_ID : i = MotorID_GimbalYaw; break;
				#if DEBUGMODE
					default : i = RxHeader.StdId - CAN_3508_M1_ID; break;
				#endif
			}

			// 电机返回数据协议解析
			get_motor_measure(&motorDataTmp, rx_data);
			#if DEBUGMODE
				get_motor_measure(&motorData[i], rx_data);
			#endif

			// 向消息队列中填充数据
			if (messageQueueCreateFlag) {
				xQueueOverwriteFromISR(messageQueue[i], (void *)&motorDataTmp, &xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}
		}
	}
}

/**
  * @brief          HAL库CAN FIFO1接受邮箱中断（Rx1）回调函数
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
//	CAN_RxHeaderTypeDef RxHeader;
//	uint8_t i = 0;
//	uint8_t rx_data[8] = {0};
//
//	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, rx_data) == HAL_OK)	//接收CAN总线上发送来的数据
//	{
//		i = RxHeader.StdId - CAN_3508_M1_ID;
//		get_motor_measure(&motorData[i], rx_data);
//	}
}

/**
  * @brief          发送C620电调控制电流
  * @param[in]		motorID对应的电机控制电流, 范围 [-16384, 16384]，对应电调输出的转矩电流范围 [-20A, 20A]
  * @param[in]      etcID: 控制报文标识符(电调ID)为1-4还是5-8
  */
void CAN_Cmd_C620(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, bool_t etcID)
{
	CAN_TxHeaderTypeDef TxHeader;						// CAN通信协议头
	uint8_t TxData[8] = {0};							// 发送电机指令缓存
	uint32_t TxMailboxX = CAN_TX_MAILBOX0;				// CAN发送邮箱

	if (etcID == false) {
		TxHeader.StdId = CAN_3508_ALL_ID;				// 标准格式标识符ID
	} else {
		TxHeader.StdId = CAN_3508_ETC_ID;
	}
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;							// 标准帧
	TxHeader.RTR = CAN_RTR_DATA;						// 传送帧类型为数据帧
	TxHeader.DLC = 0x08;								// 数据长度码
	TxData[0] = (uint8_t)(motor1 >> 8);
	TxData[1] = (uint8_t)motor1;
	TxData[2] = (uint8_t)(motor2 >> 8);
	TxData[3] = (uint8_t)motor2;
	TxData[4] = (uint8_t)(motor3 >> 8);
	TxData[5] = (uint8_t)motor3;
	TxData[6] = (uint8_t)(motor4 >> 8);
	TxData[7] = (uint8_t)motor4;

	//找到空的发送邮箱
	while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);	// 如果三个发送邮箱都阻塞了就等待直到其中某个邮箱空闲
	if ((hcan->Instance->TSR & CAN_TSR_TME0) != RESET) {
		// 检查发送邮箱0状态 如果邮箱0空闲就将待发送数据放入FIFO0
		TxMailboxX = CAN_TX_MAILBOX0;
	} else if ((hcan->Instance->TSR & CAN_TSR_TME1) != RESET) {
		TxMailboxX = CAN_TX_MAILBOX1;
	} else if ((hcan->Instance->TSR & CAN_TSR_TME2) != RESET) {
		TxMailboxX = CAN_TX_MAILBOX2;
	}
	// 将数据通过CAN总线发送
	#if DEBUGMODE
		if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, (uint32_t *)TxMailboxX) != HAL_OK) {
			Error_Handler();							// 如果CAN信息发送失败则进入死循环
		}
	#else
		HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, (uint32_t *)TxMailboxX);
	#endif
}

/**
  * @brief          发送GM6020电机控制电压
  * @param[in]      motorID对应的GM6020电机控制电压，范围 [-30000, 30000]
  * @param[in]      etcID: 控制报文标识符为5-8还是9-11
  */
void CAN_Cmd_GM6020(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4, bool_t etcID)
{
	CAN_TxHeaderTypeDef TxHeader;						// CAN通信协议头
	uint8_t TxData[8] = {0};							// 接收电机数据缓存
	uint32_t TxMailboxX = CAN_TX_MAILBOX0;				// CAN发送邮箱

	if (etcID == false) {
		TxHeader.StdId = CAN_6020_ALL_ID;				// 标准格式标识符ID
	} else {
		TxHeader.StdId = CAN_6020_ETC_ID;				// 标准格式标识符ID
	}
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;
	TxData[0] = (uint8_t)(motor1 >> 8);
	TxData[1] = (uint8_t)motor1;
	TxData[2] = (uint8_t)(motor2 >> 8);
	TxData[3] = (uint8_t)motor2;
	TxData[4] = (uint8_t)(motor3 >> 8);
	TxData[5] = (uint8_t)motor3;
	TxData[6] = (uint8_t)(motor4 >> 8);
	TxData[7] = (uint8_t)motor4;

	//找到空的发送邮箱 把数据发送出去
	while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);	// 如果三个发送邮箱都阻塞了就等待直到其中某个邮箱空闲
	if ((hcan->Instance->TSR & CAN_TSR_TME0) != RESET) {
		// 检查发送邮箱0状态 如果邮箱0空闲就将待发送数据放入FIFO0
		TxMailboxX = CAN_TX_MAILBOX0;
	} else if ((hcan->Instance->TSR & CAN_TSR_TME1) != RESET) {
		TxMailboxX = CAN_TX_MAILBOX1;
	} else if ((hcan->Instance->TSR & CAN_TSR_TME2) != RESET) {
		TxMailboxX = CAN_TX_MAILBOX2;
	}
	// 将数据通过CAN总线发送
	#if DEBUGMODE
		if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, (uint32_t *)TxMailboxX) != HAL_OK) {
			Error_Handler();							// 如果CAN信息发送失败则进入死循环
		}
	#else
		HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, (uint32_t *)TxMailboxX);
	#endif
}

/**
  * @brief          发送GM3510电机控制电压
  * @param[in]      motorID对应的GM6020电机控制电压，范围 [-29000, 29000]
  * @param[in]      id: 1/2/3
  */
void CAN_Cmd_GM3510(CAN_HandleTypeDef *hcan, int16_t motor1, int16_t motor2, int16_t motor3)
{
	CAN_TxHeaderTypeDef TxHeader;						// CAN通信协议头
	uint8_t TxData[8] = {0};							// 接收电机数据缓存
	uint32_t TxMailboxX = CAN_TX_MAILBOX0;				// CAN发送邮箱

	TxHeader.StdId = 0x1FF;								// 标准格式标识符ID
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;
	TxData[0] = (uint8_t)(motor1 >> 8);
	TxData[1] = (uint8_t)motor1;
	TxData[2] = (uint8_t)(motor2 >> 8);
	TxData[3] = (uint8_t)motor2;
	TxData[4] = (uint8_t)(motor3 >> 8);
	TxData[5] = (uint8_t)motor3;

	//找到空的发送邮箱 把数据发送出去
	while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);	// 如果三个发送邮箱都阻塞了就等待直到其中某个邮箱空闲
	if ((hcan->Instance->TSR & CAN_TSR_TME0) != RESET) {
		// 检查发送邮箱0状态 如果邮箱0空闲
		TxMailboxX = CAN_TX_MAILBOX0;
	} else if ((hcan->Instance->TSR & CAN_TSR_TME1) != RESET) {
		TxMailboxX = CAN_TX_MAILBOX1;
	} else if ((hcan->Instance->TSR & CAN_TSR_TME2) != RESET) {
		TxMailboxX = CAN_TX_MAILBOX2;
	}
	// 将数据通过CAN总线发送
	#if DEBUGMODE
		if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, (uint32_t *)TxMailboxX) != HAL_OK) {
			Error_Handler();							// 如果CAN信息发送失败则进入死循环
		}
	#else
		HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, (uint32_t *)TxMailboxX);
	#endif
}

/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  */
void CAN_cmd_chassis_reset_ID(void)
{
	CAN_TxHeaderTypeDef chassis_tx_header;				// CAN通信协议头
	uint8_t	chassis_TxData[8] = {0};					// 发送电机指令缓存
	chassis_tx_header.StdId = 0x700;
    chassis_tx_header.ExtId = 0;
    chassis_tx_header.IDE = CAN_ID_STD;
    chassis_tx_header.RTR = CAN_RTR_DATA;
    chassis_tx_header.DLC = 0x08;
    if (HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_header, chassis_TxData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK){
    	Error_Handler();								// 如果CAN信息发送失败则进入死循环
    }
}

/**
  * @brief          防止电机初始化疯转
  */
void Motor_Prevent_InitMadness(void)
{
	CAN_Cmd_C620(&CHASSIS_CAN, 0, 0, 0, 0, 0);
	HAL_Delay(10);
	CAN_Cmd_C620(&SHOOT_CAN, 0, 0, 0, 0, 1);
	HAL_Delay(10);
	CAN_Cmd_GM6020(&GIMBAL_CAN, 0, 0, 0, 0, 0);
	HAL_Delay(10);
	CAN_Cmd_GM6020(&GIMBAL_CAN, 0, 0, 0, 0, 1);
	HAL_Delay(10);
	CAN_Cmd_GM3510(&GIMBAL_CAN, 0, 0, 0);
}

