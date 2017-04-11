/*
 * hmc5883l_Interface.h
 *
 *  Created on: Nov 10, 2015
 *      Author: zz269
 */

#ifndef HMC5883L_INTERFACE_H_
#define HMC5883L_INTERFACE_H_

#include"my_types.h"
#include<stdbool.h>
#include<pthread.h>
#include<semaphore.h>

struct hmc5883l_share_object{
	Vector3f field;
	bool use_for_yaw;
	struct timeval last_update;
};

void *thread_hmc5883l_i2c_device(void *arg);

extern pthread_mutex_t mutex_hmc5883l_i2c;
extern u8 hmc5883l_i2c_sensor_inited;

int interface_hmc5883l_i2c_update(void);
bool interface_hmc5883l_i2c_use_for_yaw(void);
struct timeval interface_hmc5883l_i2c_get_last_update(void);
Vector3f interface_hmc5883l_i2c_get_field(void);
float interface_hmc5883l_i2c_get_declination(void);
float interface_hmc5883l_i2c_calculate_heading(const Matrix3f *dcm_matrix);
void  interface_hmc5883l_i2c_set_initial_location(s32 latitude, s32 longitude);


#endif /* HMC5883L_INTERFACE_H_ */
