/*
 * rc_Interface.h
 *
 *  Created on: Nov 15, 2015
 *      Author: zz269
 */

#ifndef RC_INTERFACE_H_
#define RC_INTERFACE_H_

#include"my_types.h"
#include<pthread.h>
#include<semaphore.h>


typedef struct{
   u8 length;
   u16 RC[7];
}__attribute__((packed))RC_Receive;

void *thread_receive_rc_data(void *arg);
extern pthread_mutex_t mutex_rc;
extern u8 rc_inited;

struct timeval interface_rc_receiver_get_last_update(void);
int interface_rc_receiver_update(void);
u16 interface_rc_receiver_get_rc_value(u8 channel);

#endif /* RC_INTERFACE_H_ */
