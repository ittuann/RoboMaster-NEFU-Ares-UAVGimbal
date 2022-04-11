//=====================================================================================================
// MahonyAHRS.h
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

#include "user_lib.h"

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

extern volatile float INS_quat[4];

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(volatile float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MahonyAHRSupdateIMU(volatile float q[4], float gx, float gy, float gz, float ax, float ay, float az);
void Get_Angle(volatile float q[4], float *yaw, float *pitch, float *roll);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
