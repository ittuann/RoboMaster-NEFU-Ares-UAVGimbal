/*
 * bsp_beep.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "bsp_beep.h"

extern TIM_HandleTypeDef htim4, htim5;

/**
  * @brief          初始化蜂鸣器
  */
void Beep_Init(void)
{
	HAL_TIM_Base_Start(&htim4);							    // 开启定时器
	
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);				// 开启PWM通道
	
	__HAL_TIM_PRESCALER(&htim4, 0);							// 设置定时器分频系数
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

/**
  * @brief          初始化LED
  */
void LED_Init(void)
{
	HAL_TIM_Base_Start(&htim5);							    // 开启定时器
	
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);				// 使对应定时器的对应通道开始PWM输出
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	
//	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 65535);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, 0);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 0);
}

/**
  * @brief          初始化完成
  */
void Beep_Init_Success(void)
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 10000);
	HAL_Delay(50);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
	HAL_Delay(50);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 10000);
	HAL_Delay(50);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}


/**
  * @brief          显示RGB
  * @param[in]      aRGB:0xaaRRGGBB,'aa' 是透明度,'RR'是红色,'GG'是绿色,'BB'是蓝色
  * notice			纯红色可以用8位16进制表示为0xFFFF0000
  */
void aRGB_led_show(uint32_t aRGB)
{
	uint8_t alpha = 0x00;
	uint16_t red = 0x00, green = 0x00, blue = 0x00;

    alpha = (aRGB & 0xFF000000) >> 24;
    red = ((aRGB & 0x00FF0000) >> 16) * alpha;
    green = ((aRGB & 0x0000FF00) >> 8) * alpha;
    blue = ((aRGB & 0x000000FF) >> 0) * alpha;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}
