/*
 * rc_Interface.c
 *
 *  Created on: Nov 15, 2015
 *      Author: zz269
 */
#include"rc_Interface.h"
#include"app_math.h"
#include"rc.h"

#include<sys/time.h>
#include<termio.h>
#include<fcntl.h>
#include<unistd.h>
#include<errno.h>
#include<stdio.h>
#include<string.h>

sem_t sem_rc;
pthread_mutex_t mutex_rc;
u8 rc_inited = 0;

extern pthread_mutex_t mutex_sensor_initialization;

struct rc_share_object{
	u16 RC[7];
	struct timeval last_update;
};

struct rc_share_object rc_share_data = {.last_update.tv_sec = 0,.last_update.tv_usec = 0};
s8 rc_receive_update = -1;

RC_Receive RC_Receive_Data;

//	u32 device_rc_update = 0;
//u32 device_rc_time_micros(void)
//{
//	static struct  timeval device_rc_start;
//	static u8 device_rc_record = 0;
//	struct  timeval time_now;
//
//	if(device_rc_record == 0){
//		device_rc_record = 1;
//		gettimeofday(&device_rc_start,NULL);
//	}
//	gettimeofday(&time_now,NULL);
//	if(time_now.tv_sec > device_rc_start.tv_sec)
//		return (time_now.tv_sec - device_rc_start.tv_sec)*1000000 + time_now.tv_usec  - device_rc_start.tv_usec;
//	else
//		return time_now.tv_usec  - device_rc_start.tv_usec;
//}

void *thread_receive_rc_data(void *arg)
{

	if(pthread_mutex_init(&mutex_rc, NULL) < 0){
        perror("mutex_rc initialization failed");
        pthread_exit(NULL);
    }
    if(sem_init(&sem_rc,0,0) < 0){
        perror("sem_rc initialization failed");
        pthread_exit(NULL);
    }
	memset(&rc_share_data,0,sizeof(rc_share_data));
	if(RC_Init() < 0){
		perror("failed to initialization rc device");
		pthread_exit(NULL);
	}
	pthread_mutex_lock(&mutex_sensor_initialization);
	rc_inited = 1;
	pthread_mutex_unlock(&mutex_sensor_initialization);
	printf("rc_receive thread initialization success\r\n");

	while(1){
//		if((device_rc_time_micros() - device_rc_update) > 5000){
//
//			device_rc_update = device_rc_time_micros();
				pthread_mutex_lock(&mutex_rc);
				if(RC_RECIVED){
				rc_receive_update = 0;
				RC_Receive_Data.RC[0] = RC[0];
				RC_Receive_Data.RC[1] = RC[1];
				RC_Receive_Data.RC[2] = RC[2];
				RC_Receive_Data.RC[3] = RC[3];
				RC_Receive_Data.RC[4] = RC[4];
				RC_Receive_Data.RC[5] = RC[5];
				RC_Receive_Data.RC[6] = RC[6];
				}
				pthread_mutex_unlock(&mutex_rc);
				sem_post(&sem_rc);
		}
		usleep(2000);
//		}
}


//u32 interface_rc_get_time_micros(void)
//{
//	static struct  timeval interface_rc_start;
//	static u8 interface_rc_record = 0;
//	struct  timeval time_now;
//
//	if(interface_rc_record == 0){
//		interface_rc_record = 1;
//		gettimeofday(&interface_rc_start,NULL);
//	}
//	gettimeofday(&time_now,NULL);
//	if(time_now.tv_sec > interface_rc_start.tv_sec)
//		return (time_now.tv_sec - interface_rc_start.tv_sec)*1000000 + time_now.tv_usec  - interface_rc_start.tv_usec;
//	else
//		return time_now.tv_usec  - interface_rc_start.tv_usec;
//}

#define rc_lowest 500
#define rc_highest 2000

int interface_rc_receiver_update(void)
{
	int sem_val;
	if(sem_getvalue(&sem_rc,&sem_val) == 0){
		if(sem_val > 0){

			sem_wait(&sem_rc);

			pthread_mutex_lock(&mutex_rc);
			if(rc_receive_update == 0){
				rc_receive_update = -1;
				rc_share_data.RC[0] = constrain_uint16(RC_Receive_Data.RC[0],rc_lowest,rc_highest);
				rc_share_data.RC[1] = constrain_uint16(RC_Receive_Data.RC[1],rc_lowest,rc_highest);
				rc_share_data.RC[2] = constrain_uint16(RC_Receive_Data.RC[2],rc_lowest,rc_highest);
				rc_share_data.RC[3] = constrain_uint16(RC_Receive_Data.RC[3],rc_lowest,rc_highest);
				rc_share_data.RC[4] = constrain_uint16(RC_Receive_Data.RC[4],rc_lowest,rc_highest);
				rc_share_data.RC[5] = constrain_uint16(RC_Receive_Data.RC[5],rc_lowest,rc_highest);
				rc_share_data.RC[6] = constrain_uint16(RC_Receive_Data.RC[6],rc_lowest,rc_highest);
				gettimeofday(&rc_share_data.last_update,NULL);
				pthread_mutex_unlock(&mutex_rc);
				return 0;
			}
			pthread_mutex_unlock(&mutex_rc);
		}
	}
	return -1;
}


u16 interface_rc_receiver_get_rc_value(u8 channel)
{
	if(channel >= 0 && channel <= 6){
		return rc_share_data.RC[channel];
	}else{
		perror("illegal channel request.\r\n");
		return 0;
	}
}

struct timeval interface_rc_receiver_get_last_update(void)
{
	return rc_share_data.last_update;
}

