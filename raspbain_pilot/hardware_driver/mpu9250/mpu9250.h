/*
 * mpu9250.h
 *
 *  Created on: Nov 6, 2015
 *      Author: zz269
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include"my_types.h"
#include"mpu9250_spi.h"

// Gyro and Accelerometer calibration criteria
#define INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE  4.0f
#define INERTIAL_SENSOR_ACCEL_MAX_OFFSET             250.0f

int driver_mpu9250_spi_init(void);
void driver_mpu9250_spi_release(void);
int driver_mpu9250_spi_update(void);
int driver_mpu9250_spi_read_raw_data(void);
float driver_mpu9250_spi_get_temperature(void);
float driver_mpu9250_spi_get_delta_time(void);
float driver_mpu9250_spi_get_gyro_drift_rate(void);
void driver_mpu9250_spi_set_accel_offsets(const Vector3f *accel_offset);
void driver_mpu9250_spi_set_accel_scale(const Vector3f *accel_scale);
void driver_mpu9250_spi_set_gyro_offsets(const Vector3f *gyro);
Vector3f driver_mpu9250_spi_get_accel_offsets(void);
Vector3f driver_mpu9250_spi_get_accel_scale(void);
Vector3f driver_mpu9250_spi_get_accel(void);
Vector3f driver_mpu9250_spi_get_gyro_offsets(void);
Vector3f driver_mpu9250_spi_get_gyro(void);
void driver_mpu9250_spi_calculate_trim(Vector3f accel_sample, float * trim_roll, float * trim_pitch);
void driver_mpu9250_spi_set_trim(Vector3f new_trim);
void driver_mpu9250_spi_save_trim(Vector3f new_trim);
int driver_mpu9250_spi_calibrate_accel(Vector3f accel_sample[6], Vector3f * accel_offsets, Vector3f * accel_scale);

#endif /* MPU9250_H_ */
