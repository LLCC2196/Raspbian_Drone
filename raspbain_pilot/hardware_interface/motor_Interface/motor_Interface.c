/*
 * motor_Interface.c
 *
 *  Created on: Oct 2, 2016
 *      Author: LLCC2196
 */

#include"motor_Interface.h"
#include"app_math.h"

#include<sys/time.h>
#include<termio.h>
#include<fcntl.h>
#include<unistd.h>
#include<errno.h>
#include<stdio.h>
#include<string.h>

#define motor_lowest 1086
#define motor_highest 3360

sem_t sem_motor;
pthread_mutex_t mutex_motor;
u8 motor_inited = 0;
u8 motor[4] ={1,2,3,4};

extern pthread_mutex_t mutex_sensor_initialization;

typedef struct{
   u16 Motor[4];
}__attribute__((packed))Motor_Send;

struct motor_share_object{
	u16 motor[4];
	u32 last_update;
};
Motor_Send Motor_Send_Data;
struct motor_share_object motor_share_data;

//u32 device_motor_time_micros(void)
//{
//	static struct  timeval device_motor_start;
//	static u8 device_motor_record = 0;
//	struct  timeval time_now;
//
//	if(device_motor_record == 0){
//		device_motor_record = 1;
//		gettimeofday(&device_motor_start,NULL);
//	}
//	gettimeofday(&time_now,NULL);
//	if(time_now.tv_sec > device_motor_start.tv_sec)
//		return (time_now.tv_sec - device_motor_start.tv_sec)*1000000 + time_now.tv_usec  - device_motor_start.tv_usec;
//	else
//		return time_now.tv_usec  - device_motor_start.tv_usec;
//}

void *thread_send_motor_data(void *arg)
{

	if(pthread_mutex_init(&mutex_motor, NULL) < 0){
		perror("mutex_motor initialization failed");
		pthread_exit(NULL);
	}
    if(sem_init(&sem_motor,0,0) < 0){
        perror("sem_motor initialization failed");
        pthread_exit(NULL);
    }

	memset(&motor_share_data,0,sizeof(motor_share_data));

	if(PCA9685_init() < 0){
		perror("failed to initialization motor device");
		pthread_exit(NULL);
	}
	pthread_mutex_lock(&mutex_sensor_initialization);
	motor_inited = 1;
	pthread_mutex_unlock(&mutex_sensor_initialization);

	printf("motor thread initialization success\r\n");

    int sem_val;
	while(1){
//		if((device_motor_time_micros() - device_motor_update) > 5000){
//
//			device_motor_update = device_motor_time_micros();
		if(sem_getvalue(&sem_motor,&sem_val) == 0){
			if(sem_val > 0){
				sem_wait(&sem_motor);
				pthread_mutex_lock(&mutex_motor);
				Motor_Send_Data.Motor[0] = (u16)(1.679*motor_share_data.motor[0]);
				Motor_Send_Data.Motor[1] = (u16)(1.679*motor_share_data.motor[1]);
				Motor_Send_Data.Motor[2] = (u16)(1.679*motor_share_data.motor[2]);
				Motor_Send_Data.Motor[3] = (u16)(1.679*motor_share_data.motor[3]);
				pthread_mutex_unlock(&mutex_motor);
				PCA9685_setPWM(motor[0],Motor_Send_Data.Motor[0]);
				PCA9685_setPWM(motor[1],Motor_Send_Data.Motor[1]);
				PCA9685_setPWM(motor[2],Motor_Send_Data.Motor[2]);
				PCA9685_setPWM(motor[3],Motor_Send_Data.Motor[3]);
			}
		}
		usleep(4500);
//		gettimeofday(&motor_end,NULL);
//		fprintf(stderr,"time_inerval = %ld\r\n",(motor_end.tv_sec - motor_start.tv_sec)*1000000 + motor_end.tv_usec  - motor_start.tv_usec);
//		gettimeofday(&motor_start,NULL);
//		}
	}
}

u32 interface_motor_get_time_micros(void)
{
	static struct  timeval interface_motor_start;
	static u8 interface_motor_record = 0;
	struct  timeval time_now;

	if(interface_motor_record == 0){
		interface_motor_record = 1;
		gettimeofday(&interface_motor_start,NULL);
	}
	gettimeofday(&time_now,NULL);
	if(time_now.tv_sec > interface_motor_start.tv_sec)
		return (time_now.tv_sec - interface_motor_start.tv_sec)*1000000 + time_now.tv_usec  - interface_motor_start.tv_usec;
	else
		return time_now.tv_usec  - interface_motor_start.tv_usec;
}




int interface_motor_update(u16 *motor_value)
{
	pthread_mutex_lock(&mutex_motor);
	motor_share_data.motor[0] = constrain_uint16(motor_value[0],motor_lowest,motor_highest);
	motor_share_data.motor[1] = constrain_uint16(motor_value[1],motor_lowest,motor_highest);
	motor_share_data.motor[2] = constrain_uint16(motor_value[2],motor_lowest,motor_highest);
	motor_share_data.motor[3] = constrain_uint16(motor_value[3],motor_lowest,motor_highest);
	motor_share_data.last_update = interface_motor_get_time_micros();
	pthread_mutex_unlock(&mutex_motor);
	sem_post(&sem_motor);
	return 0;
}

u32 interface_motor_send_get_last_update(void)
{
	return motor_share_data.last_update;
}



