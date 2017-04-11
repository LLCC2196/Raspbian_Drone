/*
 * hmc5883l.c
 *
 *  Created on: Nov 2, 2015
 *      Author: zz269
 */

#include<stdio.h>
#include<stdbool.h>
#include<fcntl.h>
#include<unistd.h>
#include<math.h>
#include<sys/ioctl.h>
#include<sys/time.h>

#include"hmc5883l_i2c.h"
#include"hmc5883l.h"
#include"my_types.h"
#include"rotation.h"



char *hmc5883l_dev = "/dev/i2c-1";
int hmc5883l_i2c_handler = NULL;

bool hmc5883_initialised = false;
bool hmc5883l_healthy = false;
u8 	hmc5883_base_config	= 0;
u32 hmc5883_last_accum_time = 0;
u32 hmc5883_last_read = 0;
//u32 hmc5883_last_update = 0;
s16	hmc5883_raw_x = 0;
s16	hmc5883_raw_y = 0;
s16	hmc5883_raw_z = 0;
s32	hmc5883_raw_sum_x = 0;
s32	hmc5883_raw_sum_y = 0;
s32	hmc5883_raw_sum_z = 0;
u8  hmc5883_read_count = 0;
float 	 hmc5883_calibration[3] = {0};
Vector3f hmc5883_mag_field = {0.0f,0.0f,0.0f};
Vector3f hmc5883_mag_offset	= {-54,-25,35};

u32 hmc5883l_get_time_micros(void)
{
	static struct  timeval hmc5883l_start;
	static u8 hmc5883l_record = 0;
	struct  timeval time_now;

	if(hmc5883l_record == 0){
		hmc5883l_record = 1;
		gettimeofday(&hmc5883l_start,NULL);
	}
	gettimeofday(&time_now,NULL);
	if(time_now.tv_sec > hmc5883l_start.tv_sec)
		return (time_now.tv_sec - hmc5883l_start.tv_sec)*1000000 + time_now.tv_usec  - hmc5883l_start.tv_usec;
	else
		return time_now.tv_usec  - hmc5883l_start.tv_usec;

}

int driver_hmc5883l_i2c_init(void)
{
	int numAttempts = 0, good_count = 0;
	bool success = false;
	u8 calibration_gain = 0x20;
	u16 expected_x = 715,expected_yz = 715;
	float gain_multiple = 1.0,cal[3];
	hmc5883_base_config = 0;
	if(hmc5883l_i2c_open(&hmc5883l_i2c_handler,hmc5883l_dev,HMC5883L_I2C_ADDRESS)){
		perror("failed to open hmc5883l_i2c device!");
		return -1;
	}
	if(hmc5883l_i2c_writeRegister(&hmc5883l_i2c_handler,HMC5883L_CONFIG_A,
			(SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation))){
		perror("failed to set hmc5883l_i2c device!");
		return -1;
	}
	if(hmc5883l_i2c_readRegister(&hmc5883l_i2c_handler,HMC5883L_CONFIG_A,&hmc5883_base_config)){
		perror("failed to read setting from hmc5883l_i2c device!");
		return -1;
	}else{
		calibration_gain = 0x60;
		expected_x = 766;
		expected_yz = 713;
		gain_multiple = 660.0 / 1090;
	}
	hmc5883_calibration[0] = 0.0f;
	hmc5883_calibration[1] = 0.0f;
	hmc5883_calibration[2] = 0.0f;
	while( success == 0 && numAttempts < 25 && good_count < 5){
		numAttempts++;
		// force positiveBias
		if(hmc5883l_i2c_writeRegister(&hmc5883l_i2c_handler,HMC5883L_CONFIG_A,PositiveBiasConfig))
			continue;
		// set gains
		if (hmc5883l_i2c_writeRegister(&hmc5883l_i2c_handler,HMC5883L_CONFIG_B, calibration_gain) ||
			hmc5883l_i2c_writeRegister(&hmc5883l_i2c_handler,HMC5883L_MODE, SingleConversion))
			continue;
		if(driver_hmc5883l_i2c_reead_raw_register())
		    continue;
		cal[0] = fabs(expected_x  / (float)hmc5883_raw_x);
		cal[1] = fabs(expected_yz / (float)hmc5883_raw_y);
		cal[2] = fabs(expected_yz / (float)hmc5883_raw_z);
		if (numAttempts > 2 &&
			cal[0] > 0.7f && cal[0] < 1.35f &&
			cal[1] > 0.7f && cal[1] < 1.35f &&
			cal[2] > 0.7f && cal[2] < 1.35f) {
			good_count++;
			hmc5883_calibration[0] += cal[0];
			hmc5883_calibration[1] += cal[1];
			hmc5883_calibration[2] += cal[2];
		}
		usleep(10000);
	}
	if(good_count >= 5) {
		hmc5883_calibration[0] = hmc5883_calibration[0] * gain_multiple / good_count;
		hmc5883_calibration[1] = hmc5883_calibration[1] * gain_multiple / good_count;
		hmc5883_calibration[2] = hmc5883_calibration[2] * gain_multiple / good_count;
	} else {
		hmc5883_calibration[0] = 1.0;
		hmc5883_calibration[1] = 1.0;
		hmc5883_calibration[2] = 1.0;
	}
	if(driver_hmc5883l_i2c_reinitialise())
		return -1;
	hmc5883_initialised = true;
	hmc5883l_healthy = true;
	driver_hmc5883l_i2c_update();
	return 0;
}

void driver_hmc5883l_i2c_release(void)
{
	hmc5883l_i2c_close(&hmc5883l_i2c_handler);
	hmc5883l_i2c_handler = -1;
}

int driver_hmc5883l_i2c_update(void)
{
	if(hmc5883_read_count > 0){
		hmc5883_mag_field.x = ((float)hmc5883_raw_sum_x / hmc5883_read_count) * hmc5883_calibration[0];
		hmc5883_mag_field.y = ((float)hmc5883_raw_sum_y / hmc5883_read_count) * hmc5883_calibration[1];
		hmc5883_mag_field.z = ((float)hmc5883_raw_sum_z / hmc5883_read_count) * hmc5883_calibration[2];

		hmc5883_raw_sum_x = 0;
		hmc5883_raw_sum_y = 0;
		hmc5883_raw_sum_z = 0;
		hmc5883_read_count = 0;

		// rotate to the desired orientation for HMC5883L
		rotate(ROTATION_YAW_90,&hmc5883_mag_field.x,&hmc5883_mag_field.y,&hmc5883_mag_field.z);
		rotate(ROTATION_ROLL_180,&hmc5883_mag_field.x,&hmc5883_mag_field.y,&hmc5883_mag_field.z);

		hmc5883_mag_field.x += hmc5883_mag_offset.x;
		hmc5883_mag_field.y += hmc5883_mag_offset.y;
		hmc5883_mag_field.z += hmc5883_mag_offset.z;

//		hmc5883_last_update = hmc5883l_get_time_micros();

		hmc5883l_healthy = true;
		return 0;
	}
	return -1;
}

bool driver_hmc5883l_i2c_use_for_yaw(void)
{
	return hmc5883l_healthy;
}

Vector3f driver_hmc5883l_i2c_get_field(void)
{
	return hmc5883_mag_field;
}

int driver_hmc5883l_i2c_reinitialise(void)
{
	if(hmc5883l_i2c_writeRegister(&hmc5883l_i2c_handler,HMC5883L_CONFIG_A,hmc5883_base_config) ||
		hmc5883l_i2c_writeRegister(&hmc5883l_i2c_handler,HMC5883L_CONFIG_B, magGain) ||
		hmc5883l_i2c_writeRegister(&hmc5883l_i2c_handler,HMC5883L_MODE, ContinuousConversion))
		 return -1;
	return 0;
}

int driver_hmc5883l_i2c_reead_raw_register(void)
{
	unsigned char buffer[6];
	if(hmc5883l_i2c_read_num_Registers(&hmc5883l_i2c_handler,HMC5883L_X_MSB,6,buffer)){
		perror("failed to read raw data from hmc5883l_i2c device!");
		return -1;
	}

	hmc5883_raw_x = -((((s16)buffer[0]) << 8) | buffer[1]);
	hmc5883_raw_z = -((((s16)buffer[2]) << 8) | buffer[3]);
	hmc5883_raw_y = ((((s16)buffer[4]) << 8) | buffer[5]);

	hmc5883_raw_sum_x += hmc5883_raw_x;
	hmc5883_raw_sum_y += hmc5883_raw_y;
	hmc5883_raw_sum_z += hmc5883_raw_z;
	hmc5883_read_count++;

	if(hmc5883_read_count == 20){
		hmc5883_raw_sum_x /= 2;
		hmc5883_raw_sum_y /= 2;
		hmc5883_raw_sum_z /= 2;
		hmc5883_read_count/=2;
	}
	return 0;
}
int driver_hmc5883l_i2c_read_raw_data(void)
{
	if((hmc5883l_get_time_micros() - hmc5883_last_read) < 13333)
		return -1;
	if (!hmc5883_initialised)
		return -2;
	if(!hmc5883l_healthy){
		if(driver_hmc5883l_i2c_reinitialise())
			return -3;
	}
	hmc5883_last_read = hmc5883l_get_time_micros();
	if(driver_hmc5883l_i2c_reead_raw_register() < 0)
		return -4;
	return 0;
}

