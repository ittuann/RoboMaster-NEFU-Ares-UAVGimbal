/*
 * messageQueue.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "messageQueue.h"
#include <stdbool.h>

QueueHandle_t messageQueue[QUEUE_NUM] = {NULL};	// 声明消息队列句柄
bool_t messageQueueCreateFlag = false;			// 消息队列创建完成标志位

/**
  * @brief	消息队列创建
  * @notice	需添加至 MX_FREERTOS_Init 中
  */
void MessageQueueCreate(void)
{
	uint8_t i = 0;
	// 电机消息队列
	for (i = 0; i < IMUANGLE; i ++ ) {
		messageQueue[i] =  xQueueCreate(1, 2 * sizeof(motor_measure_t *));	// 创建FIFO的长度为1 指向motor_measure_t结构的指针的队列
	}
	// IMU消息队列
	messageQueue[IMUANGLE] =  xQueueCreate(1, 3 * sizeof(float));
	messageQueue[IMUGYRO] =  xQueueCreate(1, 3 * sizeof(float));

	// 校验是否创建失败
	for (i = 0; i < QUEUE_NUM; i ++ ) {
		if (messageQueue[i] == NULL) {
			Error_Handler();
		}
	}

	messageQueueCreateFlag = true;
}

