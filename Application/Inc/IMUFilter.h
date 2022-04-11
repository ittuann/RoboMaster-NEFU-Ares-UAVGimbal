/*
 * IMUFilter.h
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#ifndef IMUFilter_H
#define IMUFilter_H

#include "main.h"
#include "user_lib.h"
#include "BMI088_Middleware.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"

#define GYRO_FILTER_NUM	3   		// 陀螺仪滤波深度
#define ACC_FILTER_NUM  3   		// 加速度计滤波深度

extern	float IMUGyroFilter[GYRO_FILTER_NUM][3], IMUAccelFilter[ACC_FILTER_NUM][3];
extern	float IMU_GyroReal[3], IMU_AccelReal[3];
extern	float INS_Speed[3];

extern	void q2eul(float *eul, uint8_t mode);
extern	void q2YawPitchRoll(float *yaw, float *pitch, float *roll);
extern	void IMUDataTransform(void);
extern	void IMUSpeedTransform(float *angleNow);
extern	void IMUDataFilter(void);

#endif
