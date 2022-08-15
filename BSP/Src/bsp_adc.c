/*
 * bsp_adc.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "bsp_adc.h"

volatile float ADCVrefintProportion = 8.0586080586080586080586080586081e-4f;

/**
  * @brief			获取ADC采样值
  * @retval			none
  * @example		adcx_get_chx_value(&hadc1, ADC_CHANNEL_1);
  */
uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
	ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ch;							// ADC转换通道
    sConfig.Rank = 1;								// ADC序列排序 即转换顺序
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;	// ADC采样时间

	HAL_ADC_ConfigChannel(ADCx, &sConfig);			// 设置ADC通道的各个属性值

    HAL_ADC_Start(ADCx);							// 开启ADC采样

    HAL_ADC_PollForConversion(ADCx, 1);				// 等待ADC转换结束

	if (HAL_IS_BIT_SET(HAL_ADC_GetState(ADCx), HAL_ADC_STATE_REG_EOC)) {	// 判断转换完成标志位是否设置
		return (uint16_t)(HAL_ADC_GetValue(ADCx));	// 获取ADC值
	} else {
		return 0;
	}
}

/**
  * @brief          ADC内部参考校准电压Vrefint初始化
  * @retval         none
  */
void ADC_Vrefint_Init(void)
{
    uint32_t total_vrefint = 0;
	
    for (uint8_t i = 0; i < 200; i ++ ) {
        total_vrefint += adcx_get_chx_value(&hadc1, ADC_CHANNEL_VREFINT);
    }

    ADCVrefintProportion = 200 * 1.200f / total_vrefint;
}

/**
  * @brief          ADC采集STM32内部温度
  * @retval         none
  */
float ADC_Get_STM32Temprate(void)
{
	float temperate = 0.000f;
    uint16_t adcx = 0;

    adcx = adcx_get_chx_value(&hadc1, ADC_CHANNEL_TEMPSENSOR);
    temperate = (float)adcx * ADCVrefintProportion;
    temperate = (temperate - 0.76f) * 400.0f + 25.0f;

    return temperate;
}

/**
  * @brief          ADC采集电池电压
  * @retval         none
  */
float ADC_Get_Voltage(void)
{
    float voltage = 0.000f;
    uint16_t adcx = 0;

    adcx = adcx_get_chx_value(&hadc3, ADC_CHANNEL_8);
    //(22K Ω + 200K Ω)  / 22K Ω = 10.090909090909090909090909090909
    voltage =  (float)adcx * ADCVrefintProportion * 10.090909090909090909090909090909f;

    return voltage;
}
