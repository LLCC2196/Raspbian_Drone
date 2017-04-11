/*
 * mpu9250_Interface.c
 *
 *  Created on: Nov 9, 2015
 *      Author: zz269
 */
#include"my_types.h"
#include"mpu9250.h"
#include"mpu9250_Interface.h"

#include<stdio.h>
#include<string.h>
#include<stdbool.h>
#include<pthread.h>
#include<semaphore.h>
#include<unistd.h>
#include<sys/time.h>

sem_t sem_mpu9250_spi;
pthread_mutex_t mutex_mpu9250_spi;
extern pthread_mutex_t mutex_sensor_initialization;

struct mpu9250_spi_share_object mpu9250_spi_sensor_share_data = {.last_update.tv_sec = 0,.last_update.tv_usec = 0};;
u8 mpu9250_spi_sensor_inited = 0;


float interface_mpu9250_spi_get_gyro_drift_rate(void)
{
	return driver_mpu9250_spi_get_gyro_drift_rate();
}

Vector3f interface_mpu9250_spi_get_gyro(void)
{
	return mpu9250_spi_sensor_share_data.gyro;
}

Vector3f interface_mpu9250_spi_get_accel(void)
{
	return mpu9250_spi_sensor_share_data.accel;
}

float interface_mpu9250_spi_get_temperature(void)
{
	return mpu9250_spi_sensor_share_data.temperature;
}

float interface_mpu9250_spi_get_delta_time(void)
{
	return mpu9250_spi_sensor_share_data.delta_time;
}

struct timeval interface_mpu9250_spi_get_last_update(void)
{
	return mpu9250_spi_sensor_share_data.last_update;
}

//	u32 device_mpu9250_update = 0;
//u32 device_mpu9250_spi_get_time_micros(void)
//{
//	static struct  timeval device_mpu9250_spi_start;
//	static u8 device_mpu9250_record = 0;
//	struct  timeval time_now;
//
//	if(device_mpu9250_record == 0){
//		device_mpu9250_record = 1;
//		gettimeofday(&device_mpu9250_spi_start,NULL);
//	}
//	gettimeofday(&time_now,NULL);
//	if(time_now.tv_sec > device_mpu9250_spi_start.tv_sec)
//		return (time_now.tv_sec - device_mpu9250_spi_start.tv_sec)*1000000 + time_now.tv_usec  - device_mpu9250_spi_start.tv_usec;
//	else
//		return time_now.tv_usec  - device_mpu9250_spi_start.tv_usec;
//}

void *thread_mpu9250_spi_device(void *arg)
{
	int ret;
//	struct  timeval mpu9250_start, mpu9250_end;
    if(pthread_mutex_init(&mutex_mpu9250_spi, NULL) < 0){
        perror("mutex_mpu9250_spi initialization failed");
        pthread_exit(NULL);
    }
    if(sem_init(&sem_mpu9250_spi,0,0) < 0){
        perror("sem_mpu9250_spi initialization failed");
        pthread_exit(NULL);
    }
	memset(&mpu9250_spi_sensor_share_data,0,sizeof(mpu9250_spi_sensor_share_data));
	if(driver_mpu9250_spi_init() < 0){
		perror("failed to initialization mpu9250_spi device");
		pthread_exit(NULL);
	}

	pthread_mutex_lock(&mutex_sensor_initialization);
	mpu9250_spi_sensor_inited = 1;
	pthread_mutex_unlock(&mutex_sensor_initialization);

	printf("mpu9250 initialization success!\r\n");

	while(1){
//		fprintf(stderr,"%d\r\n",device_mpu9250_spi_get_time_micros() - device_mpu9250_update);
//
//		device_mpu9250_update = device_mpu9250_spi_get_time_micros();

		pthread_mutex_lock(&mutex_mpu9250_spi);
		ret = driver_mpu9250_spi_read_raw_data();
		pthread_mutex_unlock(&mutex_mpu9250_spi);

		if(ret == 0){
			sem_post(&sem_mpu9250_spi);
//			gettimeofday(&mpu9250_end,NULL);
//			fprintf(stderr,"time_inerval = %ld\r\n",(mpu9250_end.tv_sec -mpu9250_start.tv_sec)*1000000 + mpu9250_end.tv_usec  - mpu9250_start.tv_usec);
//			gettimeofday(&mpu9250_start,NULL);
		}
		if(ret == -2){
			perror("mpu9250 thread update raw date error");
//			pthread_exit(NULL);
		}
		usleep(4500);

	}
	driver_mpu9250_spi_release();
	pthread_exit(NULL);
}

//u32 interface_mpu9250_get_time_micros(void)
//{
//	static struct  timeval interface_mpu9250_start;
//	static u8 interface_mpu9250_record = 0;
//	struct timeval time_now;
//
//	if(interface_mpu9250_record == 0){
//		interface_mpu9250_record = 1;
//		gettimeofday(&interface_mpu9250_start,NULL);
//	}
//	gettimeofday(&time_now,NULL);
//	if(time_now.tv_sec > interface_mpu9250_start.tv_sec)
//		return (time_now.tv_sec - interface_mpu9250_start.tv_sec)*1000000 + time_now.tv_usec  - interface_mpu9250_start.tv_usec;
//	else
//		return time_now.tv_usec  - interface_mpu9250_start.tv_usec;
//}


int interface_mpu9250_spi_update(void)
{
	int ret,sem_val;
	if(sem_getvalue(&sem_mpu9250_spi,&sem_val) == 0){
		if(sem_val > 0){

			sem_wait(&sem_mpu9250_spi);

			pthread_mutex_lock(&mutex_mpu9250_spi);
			ret = driver_mpu9250_spi_update();
			pthread_mutex_unlock(&mutex_mpu9250_spi);

			if(ret == 0){
				mpu9250_spi_sensor_share_data.accel = driver_mpu9250_spi_get_accel();
				mpu9250_spi_sensor_share_data.gyro = driver_mpu9250_spi_get_gyro();
				mpu9250_spi_sensor_share_data.temperature = driver_mpu9250_spi_get_temperature();
				mpu9250_spi_sensor_share_data.delta_time = driver_mpu9250_spi_get_delta_time();
				gettimeofday(&mpu9250_spi_sensor_share_data.last_update,NULL);
				return 0;
			}
		}
	}
	return -1;
}

int interface_mpu9250_spi_calibration(float * trim_roll,float * trim_pitch)
{
	int input_cal;
	u8 num_samples, i = 0;
    Vector3f samples[1][6];
    Vector3f new_offsets;
    Vector3f new_scaling;
    Vector3f temp;
	//clear accelerometer offsets and scaling
    memset(&temp,0,sizeof(temp));
    driver_mpu9250_spi_set_accel_offsets(&temp);
    temp.x = temp.y = temp.z = 1;
    driver_mpu9250_spi_set_accel_scale(&temp);

    // capture data from 6 positions
    for (i=0; i<6; i++) {
        const char *msg;
    	char msgSend[64];
        // display message to user
        switch ( i ) {
            case 0:
                msg = "level";
                break;
            case 1:
                msg = "on its LEFT side";
                break;
            case 2:
                msg = "on its RIGHT side";
                break;
            case 3:
                msg = "nose DOWN";
                break;
            case 4:
                msg = "nose UP";
                break;
            default:
            case 5:
                msg = "on its BACK";
                break;
        }
		sprintf(msgSend,"Place vehicle %s and press any key.\n", msg);
        printf("%s",msgSend);
        // wait for user input
        scanf("%d",&input_cal);
        // clear out any existing samples from ins
        interface_mpu9250_spi_update();
        // average 32 samples
		memset(&samples[0][i],0,sizeof(samples[0][i]));
        num_samples = 0;
        while (num_samples < 32) {
            // read samples from ins
        	interface_mpu9250_spi_update();
        	temp = interface_mpu9250_spi_get_accel();
            // capture sample
			samples[0][i].x += temp.x;
			samples[0][i].y += temp.y;
			samples[0][i].z += temp.z;
			usleep(10000);
            num_samples++;
        }
		samples[0][i].x /= num_samples;
		samples[0][i].y /= num_samples;
		samples[0][i].z /= num_samples;
		printf("[%d] = %f,%f,%f\r\n",i,samples[0][i].x,samples[0][i].y,samples[0][i].z);
    }
    // run the calibration routine
    if (driver_mpu9250_spi_calibrate_accel(samples[0], &new_offsets, &new_scaling) == 0) {
    	printf("Offsets: %10.8f %10.8f %10.8f\n",new_offsets.x, new_offsets.y, new_offsets.z);
    	printf("Scaling: %10.8f %10.8f %10.8f\n",new_scaling.x, new_scaling.y, new_scaling.z);
		// set and save calibration
    	driver_mpu9250_spi_set_accel_offsets(&new_offsets);
    	driver_mpu9250_spi_set_accel_scale(&new_scaling);
		// calculate the trims as well from primary accels and pass back to caller
        driver_mpu9250_spi_calculate_trim(samples[0][0], trim_roll, trim_pitch);
		temp.x = *trim_roll;
		temp.y = *trim_pitch;
		temp.z = 0;
		driver_mpu9250_spi_set_trim(temp);
        return 0;
    }
    return -1;
}


