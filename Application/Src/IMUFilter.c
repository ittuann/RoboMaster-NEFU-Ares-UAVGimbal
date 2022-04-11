/*
 * IMUFilter.c
 *
 *  Created on: 2021年10月24日
 *      Author: LBQ
 */
#include "IMUFilter.h"

//#define AcceRatio   	(32768.0f / 4.0f)
//#define GyroRatio   	(32768.0f / 2000.0f)
#define AcceRatio   	(8192.0f)
#define GyroRatio   	(16.384f)
#define MagRatio		(0.150f)
static float IMUGyroOrigin[GYRO_FILTER_NUM][3], IMUAccelOrigin[ACC_FILTER_NUM][3] = {0};
float IMUGyroFilter[GYRO_FILTER_NUM][3], IMUAccelFilter[ACC_FILTER_NUM][3] = {0};
float IMU_GyroReal[3] = {0}, IMU_AccelReal[3] = {0};
float INS_Speed[3] = {0};


/**
 * @brief		convert quat to eul angles
 * @param[in]	eul
 * @param[in]	0:ZYX: where rotation around z-axis (Yaw) takes place first, which is then followed by Pitch (around y-axis), and Roll (around x-axis).
				1:312: ENU: X:Pitch, Y:Roll Z:Yaw, YanGongMin
 * @notice		eul[0] X aixs roll; eul[1] Y aixs pith; eul[2] Z aixs yaw
 */
void q2eul(float *eul, uint8_t mode)
{
	float aSinInput = 0.0f;

    if (mode == 0) {
        aSinInput = -2.0f * (q1 * q3 - q0 * q2);
        if (aSinInput > 1) {
            aSinInput = 1;
        } else if (aSinInput < -1) {
            aSinInput = -1;
        }
        eul[2] = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / M_PI_F;
        eul[1] = asinf(aSinInput) * 180.0f / M_PI_F;
        eul[0] = atan2f(2.0f * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180.0f / M_PI_F;
    }

    if (mode == 1) {
        aSinInput = 2.0f * (q2 * q3 + q0 *q1);
        if (aSinInput > 1) {
            aSinInput = 1;
        } else if (aSinInput < -1) {
            aSinInput = -1;
        }
        eul[2] = -atan2f(2.0f * (q1 * q2 - q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * 180.0f / M_PI_F;
        eul[1] = asinf(aSinInput) * 180.0f / M_PI_F;
        eul[0] = -atan2f(2.0f * (q1 * q3 - q0 * q2), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180.0f / M_PI_F;
    }
}

/**
 * @brief		convert quat to eul angles
 * @param[in]	eul
 */
void q2YawPitchRoll(float *yaw, float *pitch, float *roll)
{
//	*yaw	= (atan2f(2.0f * (q0 * q3 + q1 * q2), 2.0f *( q0 * q0 + q1 * q1) - 1.0f)) * 57.3f;
	*yaw	= (atan2f(2.0f * (q0 * q3 + q1 * q2), 2.0f *( q0 * q0 + q1 * q1) - 1.0f)) * 180.0f / M_PI_F;
    *pitch	= (asinf(-2.0f * (q1 * q3 - q0 * q2))) * 180.0f / M_PI_F;
    *roll	= (atan2f(2.0f * (q0 * q1 + q2 * q3), 2.0f * ( q0 * q0 + q3 * q3) - 1.0f)) * 180.0f / M_PI_F;
}

/**
 * @brief	更新IMU数据
 */
void IMUDataUpdate(void)
{
	uint8_t i, j;
	for (i = 0; i < 3; i++) {
		// 更新滑动窗口数组
		for (j = GYRO_FILTER_NUM - 1; j > 0; j--) {
			IMUGyroOrigin[j][i] = IMUGyroOrigin[j - 1][i];
			IMUGyroFilter[j][i] = IMUGyroFilter[j - 1][i];
		}
		for (j = ACC_FILTER_NUM - 1; j > 0; j--) {
			IMUAccelOrigin[j][i] = IMUAccelOrigin[j - 1][i];
			IMUAccelFilter[j][i] = IMUAccelFilter[j - 1][i];
		}

		// 获取新值
		IMUGyroOrigin[0][i] = IMU_Gyro[i];
		IMUAccelOrigin[0][i] = IMU_Accel[i];
		IMUGyroFilter[0][i] = IMUGyroOrigin[0][i];
		IMUAccelFilter[0][i] = IMUAccelOrigin[0][i];
	}
}

/**
 * @brief	转化为实际物理量
 */
void IMUDataTransform(void)
{
	for (uint8_t i = 0; i < 3; i++) {
		// 单位g/s
		IMU_AccelReal[i] = IMUAccelFilter[0][i] * 8.0f / AcceRatio;
		// 陀螺仪角度转弧度
		IMU_GyroReal[i] = IMUGyroFilter[0][i] * M_PI_F / 180.0f / GyroRatio;
	}
}

/**
 * @brief	计算速度
 */
void IMUSpeedTransform(float *angleNow)
{
	static float angleLast[3] = {0};
	for (uint8_t i = 0; i < 3; i++) {
		INS_Speed[i] = (angleNow[i] - angleLast[i]) * IMUSampleFreq;
		angleLast[i] = angleNow[i];
	}
}

/**
 * @brief	Gyro阈值死区限制
 */
void GyroThresholdLimit(void)
{
	// 静止阈值 = 0.5度/秒
	const float GyroThreshold[3] = {0.008f, 0.008f, 0.008f};
	
	for (uint8_t i = 0; i < 3; i++) {
		if ((fabsf(IMUGyroOrigin[0][i] - IMUGyroOrigin[1][i]) < GyroThreshold[i])) {
			IMUGyroOrigin[0][i] = IMUGyroOrigin[1][i];
			IMUGyroFilter[0][i] = IMUGyroOrigin[0][i];
		}
	}
}

/**
 * @brief	加速度计二阶IIR低通滤波
 */
void AccLowpassIIR2Filter(void)
{
	// 使用 Matlab 工具箱 FilterDesign 计算参数
	// Fs = 1000 HZ; Fc = 50
	const uint8_t Order = 2;
    const float B[3] = {0.02008336556421123561544384017452102853f,
    					0.04016673112842247123088768034904205706f,
						0.02008336556421123561544384017452102853f};	// 二阶IIR滤波器参数
    const float A[3] = {1.00000f,
    					-1.561018075800718163392843962355982512236f,
						0.641351538057563175243558362126350402832f};

    for (uint8_t i = 0; i < 3; i++) {
    	IMUAccelFilter[0][i] = B[0] * IMUAccelOrigin[0][i];	// NUM 分子
		for (uint8_t j = 1; j <= Order; j++) {
			IMUAccelFilter[0][i] = IMUAccelFilter[0][i] + B[j] * IMUAccelOrigin[j][i] - A[j] * IMUAccelFilter[j][i];
		}
    }
}

/**
 * @brief	二阶 IIR 低通滤波
 *			APM 和 PX4 内的计算系数方式
 */
void AccLowpassIIR2Filter_E(void)
{
//	const float sample_freq = 500;
//	const float _cutoff_freq = 200;
//	
//	const float fr = sample_freq / _cutoff_freq;
//	const float ohm = tanf(M_PI_F / fr);
//	const float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
//	
//	const float _b0 = ohm * ohm / c;
//	const float _b1 = 2.0f * _b0;
//	const float _b2 = _b0;
//	const float _a1 = 2.0f * (ohm * ohm - 1.0f) / c;
//	const float _a2 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm +ohm * ohm) / c;
//	
//	for (uint8_t i = 0; i < 3; i++) {
//		IMUAccelFilter[0][i] = IMUAccelOrigin[0][i] * _b0 + IMUAccelOrigin[1][i] * _b1 + IMUAccelOrigin[2][i] * _b2 - IMUAccelFilter[1][i] * _a1 - IMUAccelFilter[2][i] * _a2;
//	}
}

/**
 * @brief	加速度计二阶FIR低通滤波
 */
void AccLowpassFIR2Filter(void)
{
	#define order 2
	
	// 窗 缩放通带 Hamming
	// Fs = 500 HZ; Fc = 200
	const float H[order + 1] = {0.018034926458344879146578065842732030433f, 0.963930147083310373545828042551875114441f, 0.018034926458344879146578065842732030433f};
	
	IMUAccelFilter[0][0] = IMUAccelFilter[0][1] = IMUAccelFilter[0][2]= 0;
	for (uint16_t i = 0; i < 3; i++) {
		for (uint16_t j = 0; j < order + 1; j++) {
			IMUAccelFilter[0][i] += H[j] * IMUAccelOrigin[j][i];
		}
	}
}

/**
 * @brief	加速度计一阶RC低通滤波
 */
void AccLowpassRC1Filter(void)
{
//	const float SampleFrq = 500;
//	const float CutFrq = 200;
//	const float RC = 1.0f / 2.0f / PI / CutFrq;
//	const float Cof1 = 1.0f / (1.0f + RC * SampleFrq);	// Cof1 * nowVal
//	const float Cof2 = (RC * SampleFrq) / (1.0f + RC * SampleFrq);
	const float Cof1 = 0.715365f;
	const float Cof2 = 1.0000f - Cof1;
	
	for (uint8_t i = 0; i < 3; i++) {
		IMUAccelFilter[0][i] = IMUAccelOrigin[0][i] * Cof1 + IMUAccelFilter[1][i] * Cof2;
	}
}

/**
 * @brief	加速度计均值滤波
 */
void AccAveFilter(void)
{
	for (uint8_t i = 0; i < 3; i++) {
		IMUAccelFilter[0][i] = (IMUAccelOrigin[0][i] + IMUAccelOrigin[1][i] + IMUAccelOrigin[2][i]) / ACC_FILTER_NUM;
	}
}

/**
 * @brief	陀螺仪二阶IIR高通滤波
 */
void GyroHighpassIIR2Filter(void)
{
	// 使用 Matlab 工具箱 FilterDesign 计算参数
	// Fs = 500 HZ; Fc = 1
	const uint8_t Order = 2;
	const float B[3] = { 0.956543225556876763882030445529380813241f,
						-1.913086451113753527764060891058761626482f,
						 0.956543225556876763882030445529380813241f};	// 二阶IIR滤波器参数
    const float A[3] = {1.0000f,
						-1.911197067426073203932901378720998764038f,
						 0.914975834801433851595220403396524488926f};

    for (uint8_t i = 0; i < 3; i++) {
		IMUGyroFilter[0][i] = B[0] * IMUGyroOrigin[0][i];	// NUM 分子
		for (uint8_t j = 1; j <= Order; j++) {
			IMUGyroFilter[0][i] = IMUGyroFilter[0][i] + B[j] * IMUGyroOrigin[j][i] - A[j] * IMUGyroFilter[j][i];
		}
    }
}

/**
 * @brief	陀螺仪一阶RC高通滤波
 */
void GyroHighpassRC1Filter(void)
{
//	const float SampleFrq = 500;
//	const float CutFrq = 1;
//	const float RC = 1.0f / 2.0f / PI / CutFrq;
//	const float Coff = RC / (RC + 1.0f / SampleFrq);
	const float Coff = 1.0f;
	
	for (uint8_t i = 0; i < 3; i++) {
		IMUGyroFilter[0][i] = (IMUGyroOrigin[0][i] - IMUGyroOrigin[1][i] + IMUGyroFilter[1][i]) * Coff;
	}
}

/**
 * @brief	陀螺仪均值滤波
 */
void GyroAveFilter(void)
{
	for (uint8_t i = 0; i < 3; i++) {
		IMUGyroFilter[0][i] = (IMUGyroOrigin[0][i] + IMUGyroOrigin[1][i] + IMUGyroOrigin[2][i]) / GYRO_FILTER_NUM;
	}
}

/**
 * @brief	地磁计二阶IIR低通滤波
 */
void MagLowpassIIR2Filter(void)
{

}

/**
 * @brief	IMU数据滤波
 */
void IMUDataFilter(void)
{
	// 更新IMU数据
	IMUDataUpdate();
	
	// 简单强制消抖
//	GyroThresholdLimit();
	
	// 滤波
//	AccLowpassIIR2Filter();
//	AccLowpassRC1Filter();
//	AccLowpassFIR2Filter();
	AccAveFilter();
	
//	GyroHighpassIIR2Filter();
//	GyroHighpassRC1Filter();
	GyroAveFilter();
}
