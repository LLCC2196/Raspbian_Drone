/*******************************************************************************
* File Name     : matrix3.c
* Author        : 
* Version       : V1.0.0
* Date          : 
* Description   : matrix3����
********************************************************************************/
#include "matrix3.h"
#include "app_math.h"
#include "stdio.h"

#define HALF_SQRT_2 0.70710678118654757
// create a rotation matrix given some euler angles
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
void Matrix3f_from_euler(Matrix3f *matrix3,float roll, float pitch, float yaw){
	
    float cp = cos(pitch);
    float sp = sin(pitch);
    float sr = sin(roll);
    float cr = cos(roll);
    float sy = sin(yaw);
    float cy = cos(yaw);

    matrix3->a.x = cp * cy;
    matrix3->a.y = (sr * sp * cy) - (cr * sy);
    matrix3->a.z = (cr * sp * cy) + (sr * sy);
    matrix3->b.x = cp * sy;
    matrix3->b.y = (sr * sp * sy) + (cr * cy);
    matrix3->b.z = (cr * sp * sy) - (sr * cy);
    matrix3->c.x = -sp;
    matrix3->c.y = sr * cp;
    matrix3->c.z = cr * cp;
}
// calculate euler angles from a rotation matrix
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
void Matrix3f_to_euler(Matrix3f *matrix3,float *roll, float *pitch, float *yaw){
    if (pitch != NULL) {
        *pitch = -safe_asin(matrix3->c.x);
    }
    if (roll != NULL) {
        *roll = atan2f(matrix3->c.y, matrix3->c.z);
    }
    if (yaw != NULL) {
        *yaw = atan2f(matrix3->b.x, matrix3->a.x);
    }
}
// apply an additional rotation from a body frame gyro vector
// to a rotation matrix.
void Matrix3f_rotate(Matrix3f *matrix3,const Vector3f *g){
    Matrix3f temp_matrix;
    temp_matrix.a.x = matrix3->a.y * g->z - matrix3->a.z * g->y;
    temp_matrix.a.y = matrix3->a.z * g->x - matrix3->a.x * g->z;
    temp_matrix.a.z = matrix3->a.x * g->y - matrix3->a.y * g->x;
    temp_matrix.b.x = matrix3->b.y * g->z - matrix3->b.z * g->y;
    temp_matrix.b.y = matrix3->b.z * g->x - matrix3->b.x * g->z;
    temp_matrix.b.z = matrix3->b.x * g->y - matrix3->b.y * g->x;
    temp_matrix.c.x = matrix3->c.y * g->z - matrix3->c.z * g->y;
    temp_matrix.c.y = matrix3->c.z * g->x - matrix3->c.x * g->z;
    temp_matrix.c.z = matrix3->c.x * g->y - matrix3->c.y * g->x;

	matrix3->a.x += temp_matrix.a.x;
	matrix3->a.y += temp_matrix.a.y;
	matrix3->a.z += temp_matrix.a.z;
	
	matrix3->b.x += temp_matrix.b.x;
	matrix3->b.y += temp_matrix.b.y;
	matrix3->b.z += temp_matrix.b.z;
	
	matrix3->c.x += temp_matrix.c.x;
	matrix3->c.y += temp_matrix.c.y;
	matrix3->c.z += temp_matrix.c.z;
}

// apply an additional rotation from a body frame gyro vector
// to a rotation matrix.
void Matrix3f_rotateXY(Matrix3f *matrix3,const Vector3f *g){
    Matrix3f temp_matrix;
    temp_matrix.a.x = -matrix3->a.z * g->y;
    temp_matrix.a.y = matrix3->a.z * g->x;
    temp_matrix.a.z = matrix3->a.x * g->y - matrix3->a.y * g->x;
    temp_matrix.b.x = -matrix3->b.z * g->y;
    temp_matrix.b.y = matrix3->b.z * g->x;
    temp_matrix.b.z = matrix3->b.x * g->y - matrix3->b.y * g->x;
    temp_matrix.c.x = -matrix3->c.z * g->y;
    temp_matrix.c.y = matrix3->c.z * g->x;
    temp_matrix.c.z = matrix3->c.x * g->y - matrix3->c.y * g->x;

	matrix3->a.x += temp_matrix.a.x;
	matrix3->a.y += temp_matrix.a.y;
	matrix3->a.z += temp_matrix.a.z;
	
	matrix3->b.x += temp_matrix.b.x;
	matrix3->b.y += temp_matrix.b.y;
	matrix3->b.z += temp_matrix.b.z;
	
	matrix3->c.x += temp_matrix.c.x;
	matrix3->c.y += temp_matrix.c.y;
	matrix3->c.z += temp_matrix.c.z;
}

// apply an additional inverse rotation to a rotation matrix but 
// only use X, Y elements from rotation vector
void Matrix3f_rotateXYinv(Matrix3f *matrix3,const Vector3f *g){
    Matrix3f temp_matrix;
    temp_matrix.a.x =   matrix3->a.z * g->y;
    temp_matrix.a.y = - matrix3->a.z * g->x;
    temp_matrix.a.z = - matrix3->a.x * g->y + matrix3->a.y * g->x;
    temp_matrix.b.x =   matrix3->b.z * g->y;
    temp_matrix.b.y = - matrix3->b.z * g->x;
    temp_matrix.b.z = - matrix3->b.x * g->y + matrix3->b.y * g->x;
    temp_matrix.c.x =   matrix3->c.z * g->y;
    temp_matrix.c.y = - matrix3->c.z * g->x;
    temp_matrix.c.z = - matrix3->c.x * g->y + matrix3->c.y * g->x;

    matrix3->a.x += temp_matrix.a.x;
	matrix3->a.y += temp_matrix.a.y;
	matrix3->a.z += temp_matrix.a.z;
	
	matrix3->b.x += temp_matrix.b.x;
	matrix3->b.y += temp_matrix.b.y;
	matrix3->b.z += temp_matrix.b.z;

	matrix3->c.x += temp_matrix.c.x;
	matrix3->c.y += temp_matrix.c.y;
	matrix3->c.z += temp_matrix.c.z;
}
// multiplication by a vector
Vector3f Matrix3f_operator_mul_vector(Matrix3f *matrix3,const Vector3f *v){
	Vector3f temp;
	temp.x = matrix3->a.x * v->x + matrix3->a.y * v->y + matrix3->a.z * v->z;
	temp.y = matrix3->b.x * v->x + matrix3->b.y * v->y + matrix3->b.z * v->z;
	temp.z = matrix3->c.x * v->x + matrix3->c.y * v->y + matrix3->c.z * v->z;
	return temp;
}
// multiplication by a vector, extracting only the xy components
Vector2f Matrix3f_mulXY(Matrix3f *matrix3,const Vector3f *v){
	Vector2f temp;
	temp.x = matrix3->a.x * v->x + matrix3->a.y * v->y + matrix3->a.z * v->z;
	temp.y = matrix3->b.x * v->x + matrix3->b.y * v->y + matrix3->b.z * v->z;
	return temp;
}
// multiplication of transpose by a vector
Vector3f Matrix3f_mul_transpose(Matrix3f *matrix3,const Vector3f *v){
	Vector3f temp;
	temp.x = matrix3->a.x * v->x + matrix3->b.x * v->y + matrix3->c.x * v->z;
	temp.y = matrix3->a.y * v->x + matrix3->b.y * v->y + matrix3->c.y * v->z;
	temp.z = matrix3->a.z * v->x + matrix3->b.z * v->y + matrix3->c.z * v->z;
	return temp;
}
// multiplication by another Matrix3f
Matrix3f Matrix3f_operator_mul_matrix3(Matrix3f *matrix3,const Matrix3f *m){
    Matrix3f temp;
	temp.a.x = matrix3->a.x * m->a.x + matrix3->a.y * m->b.x + matrix3->a.z * m->c.x;
	temp.a.y = matrix3->a.x * m->a.y + matrix3->a.y * m->b.y + matrix3->a.z * m->c.y;
	temp.a.z = matrix3->a.x * m->a.z + matrix3->a.y * m->b.z + matrix3->a.z * m->c.z;

	temp.b.x = matrix3->b.x * m->a.x + matrix3->b.y * m->b.x + matrix3->b.z * m->c.x;
	temp.b.y = matrix3->b.x * m->a.y + matrix3->b.y * m->b.y + matrix3->b.z * m->c.y;
	temp.b.z = matrix3->b.x * m->a.z + matrix3->b.y * m->b.z + matrix3->b.z * m->c.z;

	temp.c.x = matrix3->c.x * m->a.x + matrix3->c.y * m->b.x + matrix3->c.z * m->c.x;
	temp.c.y = matrix3->c.x * m->a.y + matrix3->c.y * m->b.y + matrix3->c.z * m->c.y;
	temp.c.z = matrix3->c.x * m->a.z + matrix3->c.y * m->b.z + matrix3->c.z * m->c.z;
    return temp;
}

Matrix3f Matrix3f_transposed(Matrix3f *matrix3){
	Matrix3f temp;
	temp.a.x = matrix3->a.x;
	temp.a.y = matrix3->a.y;
	temp.a.z = matrix3->a.z;

	temp.b.x = matrix3->b.x;
	temp.b.y = matrix3->b.y;
	temp.b.z = matrix3->b.z;

	temp.c.x = matrix3->c.x;
	temp.c.y = matrix3->c.y;
	temp.c.z = matrix3->c.z;
	return temp;
}
void Matrix3f_zero(Matrix3f *matrix3){
    matrix3->a.x = matrix3->a.y = matrix3->a.z = 0;
    matrix3->b.x = matrix3->b.y = matrix3->b.z = 0;
    matrix3->c.x = matrix3->c.y = matrix3->c.z = 0;
}
// setup the identity matrix
void  Matrix3f_identity(Matrix3f *matrix3) {
	matrix3->a.x = matrix3->b.y = matrix3->c.z = 1;
	matrix3->a.y = matrix3->a.z = 0;
	matrix3->b.x = matrix3->b.z = 0;
	matrix3->c.x = matrix3->c.y = 0;
}
// check if any elements are NAN
bool  Matrix3f_is_nan(Matrix3f *matrix3)
{
	return Vector3f_is_nan(&matrix3->a) || Vector3f_is_nan(&matrix3->b) || Vector3f_is_nan(&matrix3->c);
}

// extract x column
Vector3f Matrix3f_colx(Matrix3f *matrix3)
{
	Vector3f temp;
	temp.x = matrix3->a.x;
	temp.y = matrix3->b.x;
	temp.z = matrix3->c.x;
	return temp;
}

// extract y column
Vector3f Matrix3f_coly(Matrix3f *matrix3)
{
	Vector3f temp;
	temp.x = matrix3->a.y;
	temp.y = matrix3->b.y;
	temp.z = matrix3->c.y;
	return temp;
}

// extract z column
Vector3f Matrix3f_colz(Matrix3f *matrix3)
{
	Vector3f temp;
	temp.x = matrix3->a.z;
	temp.y = matrix3->b.z;
	temp.z = matrix3->c.z;
	return temp;
}
