/*
 * ist8310drive_middleware.h
 *
 *  Created on: 2021Äê10ÔÂ24ÈÕ
 *      Author: LBQ
 */
#ifndef IST8310DRIVER_MIDDLEWARE_H
#define IST8310DRIVER_MIDDLEWARE_H

#include "main.h"
#include "struct_typedef.h"
#include "ist8310driver.h"

#define IST8310_IIC_ADDRESS	0x0E	// the I2C address of IST8310

extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);

extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);

#endif
