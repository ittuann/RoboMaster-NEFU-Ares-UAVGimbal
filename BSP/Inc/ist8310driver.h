/*
 * ist8310drive.h
 *
 *  Created on: 2021Äê10ÔÂ24ÈÕ
 *      Author: LBQ
 */
#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H

#include "struct_typedef.h"
#include "user_lib.h"
#include "ist8310driver_middleware.h"

extern void ist8310_init(void);
extern void ist8310_read_over(uint8_t *status_buf, fp32 mag[3], uint8_t magStatus);
extern void ist8310_read_mag(fp32 mag[3]);

#endif
