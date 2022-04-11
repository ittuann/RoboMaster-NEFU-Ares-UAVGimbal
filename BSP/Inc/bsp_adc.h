/*
 * bsp_adc.h
 *
 *  Created on: 2021Äê10ÔÂ24ÈÕ
 *      Author: LBQ
 */
#ifndef BSP_ADC_H_
#define BSP_ADC_H_

#include "main.h"
#include "adc.h"

extern volatile float ADCVrefintProportion;

extern uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch);
extern void ADC_Vrefint_Init(void);
extern float ADC_Get_STM32Temprate(void);
extern float ADC_Get_Voltage(void);


#endif /* CODE_BSP_ADC_H_ */

