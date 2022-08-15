/*
 * user_lib.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "user_lib.h"

/**
  * @brief			快速计算平方根的倒数
  * @param[in]		number
  * @notice			See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
  */
float invSqrt(float number)
{
	const float x2 = number * 0.5f;
	const float threehalfs = 1.5f;
	union {
		float f;
		uint32_t i;
	} conv  = { .f = number };
	conv.i = 0x5f3759df - ( conv.i >> 1 );
	conv.f *= threehalfs - ( x2 * conv.f * conv.f );
	return conv.f;
}

/**
 * @brief			一阶RC低通滤波
 * @param[out]		lpf : 滤波结构数据指针
 * @param[in]		coefficient : 一阶RC低通滤波系数(0-1)
 * @param[in]		rawData : 原始数据
 */
float LowPassFilterRC1st(LpfRC1st_t* lpf, float coefficient, float rawData)
{
	lpf->OriginData = rawData;
	return lpf->FilterData = coefficient * lpf->OriginData + (1.000f - coefficient) * lpf->FilterData;
}

/**
 * @brief			二阶IIR低通滤波器
 * @param[out]		lpf : 滤波结构数据指针
 * @param[in]		rawData : 原始数据
 * @notion			分子系数是成对出现的 可以先合并来提高运算效率
 */
void LowPassFilterIIR2nd(LpfIIR2nd_t* lpf, float rawData)
{
	const uint8_t Order = 2;
	uint8_t i = 0;
	for (i = Order; i > 0; i--) {
		lpf->OriginData[i] = lpf->OriginData[i - 1];
		lpf->FilterData[i] = lpf->FilterData[i - 1];
	}
	lpf->OriginData[0] = rawData;
	lpf->FilterData[0] = lpf->B[0] * lpf->OriginData[0];	// NUM 分子
	for (i = 1; i <= Order; i ++ ) {
		lpf->FilterData[0] = lpf->FilterData[0] + lpf->B[i] * lpf->OriginData[i] - lpf->A[i] * lpf->FilterData[i];
	}
//	lpf->FilterData[0] = (lpf->B[0] * lpf->OriginData[0] + lpf->B[1] * lpf->OriginData[1] + lpf->B[2] * lpf->OriginData[2] - lpf->A[1] * lpf->FilterData[1] - lpf->A[2] * lpf->FilterData[2]) / lpf->A[0];
}

/**
 * @brief			简单滤波
 * @param[out]		lpf : 滤波结构数据指针
 * @param[in]		rawData : 原始数据
 */
void SimpleFilter(LpfSimple_t* lpf, float rawData)
{
	uint16_t	i = 0, j = 0;
	float		FilterBuffSort[SimpleFilterDepth] = {0};	// 暂存排序值
	float		bubble_sort_temp = 0;   					// 暂存冒泡排序交换量
    uint8_t		lastSwapIndex = 0, lastSwapIndexTemp = 0;	// 冒泡排序最后一次交换的下标

    lpf->OriginData = rawData;
    // 滑动递推更新旧值
    for (i = 1; i < SimpleFilterDepth; i ++ ) {
    	lpf->FilterBuff[i] = lpf->FilterBuff[i - 1];
    }
	// 简单强制消抖 加权存入新值
    if ((fabsf(lpf->OriginData - lpf->FilterBuff[0])) < 2.0f) {
    	lpf->FilterBuff[0] = lpf->OriginData * 0.70f + lpf->FilterBuff[0] * 0.30f;
    } else {
    	lpf->FilterBuff[0] = lpf->OriginData;
    }
	// 存储待排序值
	memcpy(FilterBuffSort, lpf->FilterBuff, sizeof(lpf->FilterBuff));
	// 冒泡升序排序
	lastSwapIndexTemp = SimpleFilterDepth - 1;
	for (i = 0; i < SimpleFilterDepth - 1; i ++ ) {
	   lastSwapIndex = lastSwapIndexTemp;
	   for (j = 0; j < lastSwapIndex; j ++ ) {
		   if (FilterBuffSort[j] > FilterBuffSort[j + 1]) {
			   bubble_sort_temp = FilterBuffSort[j];
			   FilterBuffSort[j] = FilterBuffSort[j + 1];
			   FilterBuffSort[j + 1] = bubble_sort_temp;
			   lastSwapIndexTemp = j;
		   }
	   }
	   if (lastSwapIndexTemp == 0) {
		   break;
	   }
	}
	// 中值滤波
	lpf->FilterBuff[0] = 0;    // 先清零
	for (i = 2; i < (SimpleFilterDepth - 2); i ++ ) {
		lpf->FilterBuff[0] += FilterBuffSort[i];
	}
	lpf->FilterBuff[0] = lpf->FilterBuff[0] / (SimpleFilterDepth - 4);	// 去除四个极值再计算均值
}

/**
 * @brief			计算运行频率
 * @param[out]		ptr : 结构体指针
 */
float GetFrequency(Frequency_t* ptr)
{
	if (ptr->SampleCount == 0) {
		ptr->LastTime = HAL_GetTick();
	}
	ptr->SampleCount ++ ;
	if (ptr->SampleCount > 20) {
		ptr->NowTime = HAL_GetTick();
		ptr->Freq = 1000.0f / ((float)(ptr->NowTime - ptr->LastTime) / (float)(ptr->SampleCount - 1));
		ptr->SampleCount = 0;
	}
	return ptr->Freq;
}

/**
 * @brief			斜波函数计算（根据输入的值从零进行叠加
 * @param[out]		ptr : 结构体指针
 * @param[in]		set : 期望值
 * @param[in]		rate : 每次增加的数量
 */
float RampCalc(Ramp_t *ptr, int16_t set, int16_t rate)
{
	ptr->SetVal = set;
	if (ptr->SetVal != ptr->NowVal) {
		if ((ptr->SetVal - ptr->NowVal) > 0.0f) {
			ptr->NowVal += rate;
		} else {
			ptr->NowVal -= rate;
		}
	}
	return ptr->NowVal;
}

/**
  * @brief			向SWV Console输出信息
  * @param[in]		port 0~31
  * @param[in]		内容
  * @example		swvPrint(1, "Print from CHx\n")
  */
void swvPrint(uint8_t port, char *ptr)
{
    for (uint16_t i = 0; i < strlen(ptr); i ++ ) {
        while (ITM->PORT[port].u32 == 0)
        {
        }
        ITM->PORT[port].u8 = *(ptr + i);
    }
}

/**
  * @brief			printf重定向
  */
//int U1_fputc (int ch, FILE *f)
//{
//    (void)HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);
//    return ch;
//}

/**
  * @brief			us延时
  * @param[in]		us
  */
#define TimebaseSource_is_SysTick 0
#if	(!TimebaseSource_is_SysTick)
	extern TIM_HandleTypeDef htim9;		// 当使用FreeRTOS, TimebaseSource为其他定时器时
	#define Timebase_htim htim9
	#define Delay_GetCurrentValueReg()	__HAL_TIM_GetCounter(&Timebase_htim)
	#define Delay_GetReloadReg()		__HAL_TIM_GetAutoreload(&Timebase_htim)
#else
	#define Delay_GetCurrentValueReg()	(SysTick->VAL)
	#define Delay_GetReloadReg()		(SysTick->LOAD)
#endif

static uint32_t fac_us = 0;
static uint32_t fac_ms = 0;

void Delay_us_init(void)
{
	#if	(!TimebaseSource_is_SysTick)
		fac_ms = 1000000;				// 作为时基的计数器时钟频率在HAL_InitTick()中被设为了1MHz
		fac_us = fac_ms / 1000;
	#else
		fac_ms = SystemCoreClock / 1000;
		fac_us = fac_ms / 1000;
	#endif
}

void Delay_us(uint16_t us)
{
	/*** 定时器功能实现 ***/
	uint32_t ticks = us * fac_us;
	uint32_t reload = Delay_GetReloadReg();
    uint32_t told = Delay_GetCurrentValueReg();
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    while (1) {
        tnow = Delay_GetCurrentValueReg();
        if (tnow != told) {
            if (tnow < told) {
                tcnt += told - tnow;
            } else {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) {
                break;
            }
        }
    }

    /*** NOP空语句实现 ***/
//	uint32_t delay = us * fac_us / 4;
//	do {
//		__NOP();
//	}
//	while (delay --);
}
