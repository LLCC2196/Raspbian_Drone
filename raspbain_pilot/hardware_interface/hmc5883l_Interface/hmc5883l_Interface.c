/*
 * hmc5883l_Interface.c
 *
 *  Created on: Nov 10, 2015
 *      Author: zz269
 */

#include"hmc5883l_Interface.h"
#include"hmc5883l.h"
#include"my_types.h"
#include"declination.h"
#include"app_math.h"
#include"matrix3.h"

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<stdbool.h>
#include<unistd.h>
#include<pthread.h>
#include<semaphore.h>
#include<sys/time.h>

sem_t sem_hmc5883l_i2c;
pthread_mutex_t mutex_hmc5883l_i2c;
extern pthread_mutex_t mutex_sensor_initialization;

struct hmc5883l_share_object hmc5883l_i2c_sensor_share_data = {.last_update.tv_sec = 0,.last_update.tv_usec = 0};;
u8 hmc5883l_i2c_sensor_inited = 0;

float hmc5883_declination = 0.0f;

//	u32 device_hmc5883l_update = 0;
//u32 device_hmc5883l_i2c_get_time_micros(void)
//{
//	static struct  timeval device_hmc5883l_i2c_start;
//	static u8 device_hmc5883l_record = 0;
//	struct  timeval time_now;
//
//	if(device_hmc5883l_record == 0){
//		device_hmc5883l_record = 1;
//		gettimeofday(&device_hmc5883l_i2c_start,NULL);
//	}
//	gettimeofday(&time_now,NULL);
//	if(time_now.tv_sec > device_hmc5883l_i2c_start.tv_sec)
//		return (time_now.tv_sec - device_hmc5883l_i2c_start.tv_sec)*1000000 + time_now.tv_usec  - device_hmc5883l_i2c_start.tv_usec;
//	else
//		return time_now.tv_usec  - device_hmc5883l_i2c_start.tv_usec;
//}


void *thread_hmc5883l_i2c_device(void *arg)
{
	int ret;

	if(pthread_mutex_init(&mutex_hmc5883l_i2c, NULL) < 0){
        perror("mutex_hmc5883l_i2c initialization failed");
        pthread_exit(NULL);
    }
    if(sem_init(&sem_hmc5883l_i2c,0,0) < 0){
        perror("sem_hmc5883l_i2c initialization failed");
        pthread_exit(NULL);
    }
	memset(&hmc5883l_i2c_sensor_share_data,0,sizeof(hmc5883l_i2c_sensor_share_data));

	if(driver_hmc5883l_i2c_init() < 0){
		perror("failed to initialization hmc5883l_i2c device");
		pthread_exit(NULL);
	}

	pthread_mutex_lock(&mutex_sensor_initialization);
	hmc5883l_i2c_sensor_inited = 1;
	pthread_mutex_unlock(&mutex_sensor_initialization);

	printf("hmc5883l_i2c initialization success!\r\n");

	while(1){
//		if((device_hmc5883l_i2c_get_time_micros() - device_hmc5883l_update) > 13333){
//
//			device_hmc5883l_update = device_hmc5883l_i2c_get_time_micros();

		pthread_mutex_lock(&mutex_hmc5883l_i2c);
		ret = driver_hmc5883l_i2c_read_raw_data();
		pthread_mutex_unlock(&mutex_hmc5883l_i2c);

		if(ret == 0)
			sem_post(&sem_hmc5883l_i2c);

		if(ret == -2){
			perror("hmc5883l initialization failed");
//			pthread_exit(NULL);
		}else if(ret == -3){
			perror("hmc5883l breakdown & cann't fixed");
//			pthread_exit(NULL);
		}else if(ret == -4){
			perror("hmc5883l read register error");
//			pthread_exit(NULL);
		}
		usleep(5000);
//		}
	}
	driver_hmc5883l_i2c_release();
	pthread_exit(NULL);
}

//u32 interface_hmc5883l_get_time_micros(void)
//{
//	static struct  timeval interface_hmc5883l_start;
//	static u8 interface_hmc5883l_record = 0;
//	struct  timeval time_now;
//
//	if(interface_hmc5883l_record == 0){
//		interface_hmc5883l_record = 1;
//		gettimeofday(&interface_hmc5883l_start,NULL);
//	}
//	gettimeofday(&time_now,NULL);
//	if(time_now.tv_sec > interface_hmc5883l_start.tv_sec)
//		return (time_now.tv_sec - interface_hmc5883l_start.tv_sec)*1000000 + time_now.tv_usec  - interface_hmc5883l_start.tv_usec;
//	else
//		return time_now.tv_usec  - interface_hmc5883l_start.tv_usec;
//}

int interface_hmc5883l_i2c_update(void)
{
	int ret,sem_val;
	if(sem_getvalue(&sem_hmc5883l_i2c,&sem_val) == 0){
		if(sem_val > 0){

			sem_wait(&sem_hmc5883l_i2c);

			pthread_mutex_lock(&mutex_hmc5883l_i2c);
			ret = driver_hmc5883l_i2c_update();
			pthread_mutex_unlock(&mutex_hmc5883l_i2c);
			if(ret == 0){
				hmc5883l_i2c_sensor_share_data.field = driver_hmc5883l_i2c_get_field();
				hmc5883l_i2c_sensor_share_data.use_for_yaw = driver_hmc5883l_i2c_use_for_yaw();
				gettimeofday(&hmc5883l_i2c_sensor_share_data.last_update,NULL);
				return 0;
			}
		}
	}
	return -1;
}

Vector3f interface_hmc5883l_i2c_get_field(void)
{
	return hmc5883l_i2c_sensor_share_data.field;
}

struct timeval interface_hmc5883l_i2c_get_last_update(void)
{
	return hmc5883l_i2c_sensor_share_data.last_update;
}

bool interface_hmc5883l_i2c_use_for_yaw(void)
{
	return hmc5883l_i2c_sensor_share_data.use_for_yaw;
}


float interface_hmc5883l_i2c_get_declination(void)
{
    return hmc5883_declination;
}

void interface_hmc5883l_i2c_set_initial_location(s32 latitude, s32 longitude)
{
	// Set the declination based on the lat/lng from GPS
	hmc5883_declination = radians(get_declination((float)latitude / 10000000,(float)longitude / 10000000));
}

float interface_hmc5883l_i2c_calculate_heading(const Matrix3f *dcm_matrix)
{
    float cos_pitch_sq = 1.0f-(dcm_matrix->c.x*dcm_matrix->c.x);
    // Tilt compensated magnetic field Y component:
    float headY = hmc5883l_i2c_sensor_share_data.field.y * dcm_matrix->c.z - hmc5883l_i2c_sensor_share_data.field.z * dcm_matrix->c.y;
    // Tilt compensated magnetic field X component:
    float headX = hmc5883l_i2c_sensor_share_data.field.x * cos_pitch_sq - dcm_matrix->c.x * (hmc5883l_i2c_sensor_share_data.field.y * dcm_matrix->c.y + hmc5883l_i2c_sensor_share_data.field.z * dcm_matrix->c.z);
    return constrain_float(atan2f(-headY,headX), -3.15f, 3.15f);
}



