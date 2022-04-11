/*
 * IMU.h
 *
 *  Created on: 2021Äê10ÔÂ24ÈÕ
 *      Author: LBQ
 */
#ifndef IMU_H
#define IMU_H

#include "main.h"
#include "user_lib.h"
#include "messageQueue.h"
#include "tim.h"
#include "spi.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_beep.h"
#include "IMUFilter.h"
#include "PID.h"

#define SPI_DMA_GYRO_LENGHT			8
#define SPI_DMA_ACCEL_LENGHT		9
#define SPI_DMA_ACCEL_TEMP_LENGHT	4

#define IMU_DR_SHFITS		0
#define IMU_SPI_SHFITS		1
#define IMU_UPDATE_SHFITS	2
#define IMU_NOTIFY_SHFITS	3

#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

#endif
