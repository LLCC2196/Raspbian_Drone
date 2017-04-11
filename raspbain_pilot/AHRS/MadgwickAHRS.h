//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
//----------------------------------------------------------------------------------------------------
// Variable declaration
#include "app_math.h"
extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
//#define	PI	3.141592653589793238
//---------------------------------------------------------------------------------------------------
// Function declarations
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, s16 mx_i, s16 my_i, s16 mz_i, float Gt);
void MadgwickAHRS_init(void);
//void quat2eulers(float *Angle);
void quat2eulers(void);

void MadgwickAHRS_update_trig(void);
//void MadgwickAHRS_get_eular(s32 *angle);
s32 MadgwickAHRS_get_eular_roll(void);
s32 MadgwickAHRS_get_eular_pitch(void);
s32 MadgwickAHRS_get_eular_yaw(void);
float cos_roll(void);
float cos_pitch(void);
float cos_yaw(void);
float sin_roll(void);
float sin_pitch(void);
float sin_yaw(void);
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
