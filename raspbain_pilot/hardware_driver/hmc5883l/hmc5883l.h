/*
 * hmc5883l.h
 *
 *  Created on: Nov 2, 2015
 *      Author: zz269
 */

#ifndef HMC5883L_H_
#define HMC5883L_H_
#include"my_types.h"
#include<stdbool.h>

int driver_hmc5883l_i2c_init(void);
int driver_hmc5883l_i2c_update(void);
int driver_hmc5883l_i2c_read_raw_data(void);
int driver_hmc5883l_i2c_reinitialise(void);
void driver_hmc5883l_i2c_release(void);
bool driver_hmc5883l_i2c_use_for_yaw(void);
Vector3f driver_hmc5883l_i2c_get_field(void);
int driver_hmc5883l_i2c_reead_raw_register(void);
#endif /* HMC5883L_H_ */
