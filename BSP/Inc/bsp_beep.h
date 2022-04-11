/*
 * bsp_beep.h
 *
 *  Created on: 2021Äê10ÔÂ24ÈÕ
 *      Author: LBQ
 */
#ifndef BSP_BEEP_H
#define BSP_BEEP_H

#include "main.h"
#include "tim.h"

extern void Beep_Init(void);
extern void LED_Init(void);
extern void Beep_Init_Success(void);
extern void aRGB_led_show(uint32_t aRGB);

#endif

