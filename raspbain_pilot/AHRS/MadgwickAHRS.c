//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include <math.h>
#include "my_types.h"
#include "MadgwickAHRS.h"

#define deg_to_rad 0.017453292519943295769236907684886f
//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	512.0f		// sample frequency in Hz
#define betaDef		0.5f				// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

s32 Madgwick_eular[3];
s32 Madgwick_trim[2];
float Angle[3];

float Madgwick_cos_roll = 1.0f, Madgwick_cos_pitch = 1.0f, Madgwick_cos_yaw = 1.0f;
float Madgwick_sin_roll = 0.0f, Madgwick_sin_pitch = 0.0f, Madgwick_sin_yaw = 0.0f;

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

void MadgwickAHRS_init(void)
{
	beta = betaDef;
	q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
	Madgwick_eular[0] = 0; Madgwick_eular[1] = 0; Madgwick_eular[2] = 0;
	Madgwick_trim[0] = 88;
	Madgwick_trim[1] = -44;
	Madgwick_cos_roll = 1.0f; Madgwick_cos_pitch = 1.0f; Madgwick_cos_yaw = 1.0f;
	Madgwick_sin_roll = 0.0f; Madgwick_sin_pitch = 0.0f; Madgwick_sin_yaw = 0.0f;
}

void MadgwickAHRS_update_trig()
{
	float eular[3];
//	quat2eulers(eular);
	quat2eulers();
	eular[0] = Angle[0];
	eular[1] = Angle[1];
	eular[2] = Angle[2];
	Madgwick_eular[0] = (s32)(eular[0] * 100) - Madgwick_trim[0];
	Madgwick_eular[1] = (s32)(eular[1] * 100) - Madgwick_trim[1];
	Madgwick_eular[2] = (s32)(eular[2] * 100);
	float rad[3];
	rad[0] = (float)Madgwick_eular[0]/100*deg_to_rad;
	rad[1] = (float)Madgwick_eular[1]/100*deg_to_rad;
	rad[2] = (float)Madgwick_eular[2]/100*deg_to_rad;
	Madgwick_cos_roll = cosf(rad[0]);
	Madgwick_sin_roll = sinf(rad[0]);
	Madgwick_cos_pitch = cosf(rad[1]);
	Madgwick_sin_pitch = sinf(rad[1]);
	Madgwick_cos_yaw = cosf(rad[2]);
	Madgwick_sin_yaw = sinf(rad[2]);
}

//void MadgwickAHRS_get_eular(s32 *angle)
//{
//	angle[0] = Madgwick_eular[0];
//	angle[1] = Madgwick_eular[1];
//	angle[2] = Madgwick_eular[2];
//}
s32 MadgwickAHRS_get_eular_roll(void)
{
	return Madgwick_eular[0];
}
s32 MadgwickAHRS_get_eular_pitch(void)
{
	return Madgwick_eular[1];
}
s32 MadgwickAHRS_get_eular_yaw(void)
{
	return Madgwick_eular[2];
}

float cos_roll(void){ return Madgwick_cos_roll;}
float cos_pitch(void){ return Madgwick_cos_pitch;}
float cos_yaw(void){ return Madgwick_cos_yaw;}
float sin_roll(void){ return Madgwick_sin_roll;}
float sin_pitch(void){ return Madgwick_sin_pitch;}
float sin_yaw(void){ return Madgwick_sin_yaw;}
//---------------------------------------------------------------------------------------------------
// AHRS algorithm update
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, s16 mx_i, s16 my_i, s16 mz_i, float Gt) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float mx = (float)mx_i;
	float my = (float)my_i;
	float mz = (float)mz_i;

	// avoids NaN in magnetometer normalisation
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		return;
	}
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = 2.0 * sqrtf(hx * hx + hy * hy);
		_2bz = 2.0 * (-_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3);
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = _2q2 * (-2.0 * q1q3 + _2q0q2 - ax) - _2q1 * (-2.0 * q0q1 - _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
		s1 = -_2q3 * (-2.0 * q1q3 + _2q0q2 - ax) - _2q0 * (-2.0 * q0q1 - _2q2q3 - ay) + 4.0 * q1 * (-1.0 + 2.0 * q1q1 + 2.0 * q2q2 - az) + _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
		s2 = _2q0 * (-2.0 * q1q3 + _2q0q2 - ax) - _2q3 * (-2.0 * q0q1 - _2q2q3 - ay) + 4.0 * q2 * (-1.0 + 2.0 * q1q1 + 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
		s3 = -_2q1 * (-2.0 * q1q3 + _2q0q2 - ax) - _2q2 * (-2.0 * q0q1 - _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
			
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * Gt;//(1.0f / sampleFreq);
	q1 += qDot2 * Gt;//(1.0f / sampleFreq);
	q2 += qDot3 * Gt;//(1.0f / sampleFreq);
	q3 += qDot4 * Gt;//(1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	MadgwickAHRS_update_trig();
}
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long *i_temp = (long*)&y;;
	long i = *i_temp;
	i = 0x5f3759df - (i>>1);
	float *y_temp = (float*)&i;
	y = *y_temp;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//void quat2eulers(float *Angle)
//{
//	float t[3];
//	t[0] = atan2f(2.0*(q2*q3 + q0*q1), (q0*q0-q1*q1-q2*q2+q3*q3));;	//roll
//	t[1] = asinf(-2.0*(q1*q3-q0*q2));								//pitch
//	t[2] = atan2f(2.0*(q1*q2+q0*q3), (q0*q0+q1*q1-q2*q2-q3*q3));	//yaw
//	float temp = 180.0/PI;
//	Angle[0] = t[0]*temp;
//	Angle[1] = t[1]*temp;
//	Angle[2] = t[2]*temp;
//	if(Angle[2] < 0) Angle[2] += 360.0f;
//}
void quat2eulers(void)
{
	float t[3];
	t[0] = atan2f(2.0*(q2*q3 + q0*q1), (q0*q0-q1*q1-q2*q2+q3*q3));;	//roll
	t[1] = asinf(-2.0*(q1*q3-q0*q2));								//pitch
	t[2] = atan2f(2.0*(q1*q2+q0*q3), (q0*q0+q1*q1-q2*q2-q3*q3));	//yaw
	float temp = 180.0/PI;
	Angle[0] = t[0]*temp;
	Angle[1] = t[1]*temp;
	Angle[2] = t[2]*temp;
	if(Angle[2] < 0) Angle[2] += 360.0f;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
