/********************************************************************
* File Name     : app_math.h
* Author        : 
* Version       : V1.0.0
* Date          : 
* Description   : 
********************************************************************/

#ifndef __APP_MATH_H__
#define __APP_MATH_H__
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "my_types.h"


#ifndef M_PI_F
 #define M_PI_F 3.141592653589793f
#endif
#ifndef PI
 # define PI M_PI_F
#endif
#ifndef M_PI_2
 # define M_PI_2 1.570796326794897f
#endif
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

#define RadiansToCentiDegrees(x) ((x) * 5729.5779513082320876798154814105f)

// acceleration due to gravity in m/s/s
#define GRAVITY_MSS 9.80665f

// radius of earth in meters
#define RADIUS_OF_EARTH 6378100

#define ROTATION_COMBINATION_SUPPORT 0

// convert a longitude or latitude point to meters or centimeteres.
// Note: this does not include the longitude scaling which is dependent upon location
#define LATLON_TO_M  0.01113195f
#define LATLON_TO_CM 1.113195f

#define ToRad(x) radians(x)	// *pi/180
#define ToDeg(x) degrees(x)	// *180/pi

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

s32 wrap_180_cd(s32 error);
s32 wrap_360_cd(s32 error);

float safe_asin(float v);
float safe_sqrt(float v);
float fast_atan(float v);
float constrain_float(float amt, float low, float high);
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);
uint16_t constrain_uint16(uint16_t amt,uint16_t low,uint16_t high);
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high);
uint32_t constrain_uint32(uint32_t amt, uint32_t low, uint32_t high);
float radians(float deg);
float degrees(float rad);
float sq(float v);
float pythagorous2(float a, float b);
float pythagorous3(float a, float b, float c);

float Vector3f_length(Vector3f *V);
Vector3f Vector3f_normalize(Vector3f A);
float Vector3f_angle(Vector3f A,Vector3f B);
bool Vector3f_is_inf(Vector3f *A);
bool Vector3f_is_nan(Vector3f *A);
bool Vector3f_is_zero(Vector3f *A);
float Vector2f_length(Vector2f *A);
Vector2f Vector2f_normalize(Vector2f A);
bool Vector2f_is_inf(Vector2f *A);
bool Vector2f_is_nan(Vector2f *A);
float Vector2f_angle(Vector2f *A,Vector2f *B);
#endif
