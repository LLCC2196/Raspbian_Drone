/*
 * motor_Interface.h
 *
 *  Created on: Nov 15, 2015
 *      Author: zz269
 */

#ifndef MOTOR_INTERFACE_H_
#define MOTOR_INTERFACE_H_

#include"my_types.h"
#include<pthread.h>
#include<semaphore.h>


void *thread_send_motor_data(void *arg);
extern pthread_mutex_t mutex_motor;
extern u8 motor_inited;

int interface_motor_update(u16 *motor_value);
u32 interface_motor_send_get_last_update(void);

#endif /* MOTOR_INTERFACE_H_ */
