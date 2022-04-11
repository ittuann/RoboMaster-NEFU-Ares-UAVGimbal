/*
 * shoot.h
 *
 *  Created on: 2021Äê10ÔÂ24ÈÕ
 *      Author: LBQ
 */
#ifndef SHOOT_H
#define SHOOT_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can_receive.h"
#include "PID.h"
#include "bsp_oled.h"
#include "tim.h"
#include "anotc.h"
#include "remote_control.h"

extern void Snail_Init(void);
extern void Laser_Init(void);

#endif
