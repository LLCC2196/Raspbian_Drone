/*
 * mpu9250_Interface.h
 *
 *  Created on: Nov 9, 2015
 *      Author: zz269
 */

#ifndef MPU9250_INTERFACE_H_
#define MPU9250_INTERFACE_H_
#include"my_types.h"

#include<sys/time.h>
#include<pthread.h>
#include<semaphore.h>

struct mpu9250_spi_share_object{
	Vector3f accel;
	Vector3f gyro;
	float temperature;
	float delta_time;
	struct timeval last_update;
};

extern pthread_mutex_t mutex_mpu9250_spi;
extern sem_t sem_mpu9250_spi;
extern u8 mpu9250_spi_sensor_inited;

void *thread_mpu9250_spi_device(void *arg);

Vector3f interface_mpu9250_spi_get_gyro(void);
Vector3f interface_mpu9250_spi_get_accel(void);
float interface_mpu9250_spi_get_temperature(void);
float interface_mpu9250_spi_get_delta_time(void);
float interface_mpu9250_spi_get_gyro_drift_rate(void);
int interface_mpu9250_spi_update(void);
struct timeval interface_mpu9250_spi_get_last_update(void);
int interface_mpu9250_spi_calibration(float * trim_roll,float * trim_pitch);
#endif /* MPU9250_INTERFACE_H_ */
