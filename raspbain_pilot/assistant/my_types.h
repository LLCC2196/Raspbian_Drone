/*
 * types.h
 *
 *  Created on: Nov 2, 2015
 *      Author: zz269
 */

#ifndef TYPES_H_
#define TYPES_H_
#include<limits.h>

typedef unsigned char           u8;
typedef unsigned short          u16;
typedef unsigned int            u32;
typedef unsigned long long      u64;
typedef signed char             s8;
typedef short                   s16;
typedef int                     s32;
typedef long long               s64;

typedef struct{
	float x;
	float y;
	float z;
}__attribute__((packed))Vector3f;

typedef struct{
	float x;
	float y;
}Vector2f;

typedef struct{
	s32 x;
	s32 y;
	s32 z;
}Vector3l;

typedef struct{
	s16 x;
	s16 y;
	s16 z;
}Vector3i;

typedef struct{
	Vector3f a;
	Vector3f b;
	Vector3f c;
}Matrix3f;

#endif /* TYPES_H_ */
