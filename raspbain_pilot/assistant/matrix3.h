/*******************************************************************************
* File Name     : matrix3.h
* Author        : 
* Version       : V1.0.0
* Date          : 
* Description   : matrix3����
********************************************************************************/
#ifndef __MATRIX3_H_
#define __MATRIX3_H_
#include "stdint.h"
#include "my_types.h"
#include "stdbool.h"
void Matrix3f_from_euler(Matrix3f *matrix3,float roll, float pitch, float yaw);
void Matrix3f_to_euler(Matrix3f *matrix3,float *roll, float *pitch, float *yaw);
void Matrix3f_rotate(Matrix3f *matrix3,const Vector3f *g);
void Matrix3f_rotateXY(Matrix3f *matrix3,const Vector3f *g);
void Matrix3f_rotateXYinv(Matrix3f *matrix3,const Vector3f *g);
Vector3f Matrix3f_operator_mul_vector(Matrix3f *matrix3,const Vector3f *v);
Vector2f Matrix3f_mulXY(Matrix3f *matrix3,const Vector3f *v);
Vector3f Matrix3f_mul_transpose(Matrix3f *matrix3,const Vector3f *v);
Matrix3f Matrix3f_operator_mul_matrix3(Matrix3f *matrix3,const Matrix3f *m);
Matrix3f Matrix3f_transposed(Matrix3f *matrix3);
void Matrix3f_zero(Matrix3f *matrix3);
void  Matrix3f_identity(Matrix3f *matrix3);
bool  Matrix3f_is_nan(Matrix3f *matrix3);
Vector3f Matrix3f_colx(Matrix3f *matrix3);
Vector3f Matrix3f_coly(Matrix3f *matrix3);
Vector3f Matrix3f_colz(Matrix3f *matrix3);
#endif //matrix3.h
