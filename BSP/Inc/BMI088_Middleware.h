/*
 * BMI088_Middleware.h
 *
 *  Created on: 2021Äê10ÔÂ24ÈÕ
 *      Author: LBQ
 */
#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "main.h"
#include "struct_typedef.h"
#include "BMI088driver.h"

extern	uint8_t BMI088_read_write_byte(uint8_t tx_data);
extern	void BMI088_write_single_reg(uint8_t reg, uint8_t data);
extern	void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
extern	void BMI088_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
extern	void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

extern	void BMI088_init(void);

extern	float IMU_Gyro[3], IMU_Accel[3];
extern	float IMU_Temperate;

#endif
