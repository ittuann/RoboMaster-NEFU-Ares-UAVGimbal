/*
 * BMI088driver.h
 *
 *  Created on: 2021Äê10ÔÂ24ÈÕ
 *      Author: LBQ
 */
#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include "main.h"
#include "user_lib.h"
#include "spi.h"
#include "BMI088_Middleware.h"

extern	void bmi088_accel_init(void);
extern	void bmi088_gyro_init(void);

extern	void BMI088_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate);
extern	void BMI088_gyro_read_over(uint8_t *rx_buf, fp32 gyro[3]);
extern	void BMI088_accel_read_over(uint8_t *rx_buf, fp32 accel[3]);
extern	void BMI088_temperature_read_over(uint8_t *rx_buf, fp32 *temperate);

#endif
