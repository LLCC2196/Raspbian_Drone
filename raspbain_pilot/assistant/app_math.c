/********************************************************************
* File Name     : app_math.c
* Author        : 
* Version       : V1.0.0
* Date          : 
* Description   : 
********************************************************************/

#include "app_math.h"

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0f) {
        return PI/2;
    }
    if (v <= -1.0f) {
        return -PI/2;
    }
    return asinf(v);
}

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v)
{
    float ret = sqrtf(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

// a faster varient of atan.  accurate to 6 decimal places for values between -1 ~ 1 but then diverges quickly
float fast_atan(float v)
{
    float v2 = v*v;
    return (v*(1.6867629106f + v2*0.4378497304f)/(1.6867633134f + v2));
}

// constrain a value
float constrain_float(float amt, float low, float high) 
{
	// the check for NaN as a float prevents propogation of
	// floating point errors through any function that uses
	// constrain_float(). The normal float semantics already handle -Inf
	// and +Inf
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int16_t value
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
//constrain a uint16_t value
uint16_t constrain_uint16(uint16_t amt,uint16_t low,uint16_t high){
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
// constrain a int32_t value
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
// constrain a uint32_t value
uint32_t constrain_uint32(uint32_t amt, uint32_t low, uint32_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
// degrees -> radians
float radians(float deg) {
	return deg * DEG_TO_RAD;
}

// radians -> degrees
float degrees(float rad) {
	return rad * RAD_TO_DEG;
}

// square
float sq(float v) {
	return v*v;
}

// 2D vector length
float pythagorous2(float a, float b) {
	return sqrtf(sq(a)+sq(b));
}

// 3D vector length
float pythagorous3(float a, float b, float c) {
	return sqrtf(sq(a)+sq(b)+sq(c));
}
float Vector3f_length(Vector3f *V){
		return pythagorous3(V->x,V->y,V->z);
}
float Vector3f_angle(Vector3f A,Vector3f B){
	return acos((A.x*B.x + A.y*B.y + A.z*B.z)/(Vector3f_length(&A)*Vector3f_length(&B)));
}
Vector3f Vector3f_normalize(Vector3f A){
	float length = Vector3f_length(&A);
	A.x /= length;A.y /= length;A.z /= length;
	return A;
}
bool Vector3f_is_nan(Vector3f *A)
{
    return isnan(A->x) || isnan(A->y) || isnan(A->z);
}
bool Vector3f_is_zero(Vector3f *A)
{ 
		return A->x==0 && A->y == 0 && A->z == 0; 
}
bool Vector3f_is_inf(Vector3f *A)
{
    return isinf(A->x) || isinf(A->y) || isinf(A->z);
}

float Vector2f_length(Vector2f *A)
{
	return pythagorous2(A->x, A->y);
}
// normalizes this vector
Vector2f Vector2f_normalize(Vector2f A)
{
	float length = Vector2f_length(&A);
	A.x /= length;A.y /= length;
	return A;
}
bool Vector2f_is_nan(Vector2f *A)
{
    return isnan(A->x) || isnan(A->y);
}
bool Vector2f_is_inf(Vector2f *A)
{
    return isinf(A->x) || isinf(A->y);
}

float Vector2f_angle(Vector2f *A,Vector2f *B)
{
		float len = Vector2f_length(A) * Vector2f_length(B);
    if (len <= 0) {
        return 0.0f;
    }
    float cosv = (A->x * B->x + A->y*B->y) / len;
    if (fabsf(cosv) >= 1) {
        return 0.0f;
    }
    return acosf(cosv);
}
/*wrap an angle in centi-degrees to 0..35999*/
s32 wrap_360_cd(s32 error)
{
    if (error > 360000 || error < -360000) {
        // for very large numbers use modulus
        error = error % 36000;
    }
    while (error >= 36000) error -= 36000;
    while (error < 0) error += 36000;
    return error;
}
/*wrap an angle in centi-degrees to -18000..18000*/
s32 wrap_180_cd(s32 error)
{
    if (error > 360000 || error < -360000) {
        // for very large numbers use modulus
        error = error % 36000;
    }
    while (error > 18000) { error -= 36000; }
    while (error < -18000) { error += 36000; }
    return error;
}
