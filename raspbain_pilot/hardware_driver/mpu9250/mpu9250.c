/*
 * mpu9250.c
 *
 *  Created on: Nov 6, 2015
 *      Author: zz269
 */

#include"mpu9250_spi.h"
#include"mpu9250.h"
#include"my_types.h"
#include"app_math.h"

#include<sys/time.h>
#include<unistd.h>
#include<stdio.h>
#include<string.h>
#include<stdbool.h>
#include<pthread.h>
#include<semaphore.h>



#define GRAVITY_MSS 9.80665f
#define MPU9250_ACCEL_SCALE_1G 0.00239420166f	//(GRAVITY_MSS / 4096.0f)

struct mpu9250_spi_property mpu9250_property;

Vector3f mpu9250_spi_accel;
Vector3f mpu9250_spi_accel_scale = {0.99885094f, 0.99767292f, 0.99176794f};
Vector3f mpu9250_spi_accel_offset = {-0.01869215f, 0.12367837f, 0.14199395f};

float 	 mpu9250_spi_gyro_scale = (0.0174532f / 16.4f);
Vector3f mpu9250_spi_gyro_offset = {0.0,0.0,0.0};
Vector3f mpu9250_spi_gyro;
s16      mpu9250_poll_data_count= 0;

u32      mpu9250_spi_last_update = 0;
float 	 mpu9250_spi_temperature = 0;
struct 	 mpu9250_spi_SensorData mpu9250_spi_raw_data;


int  driver_mpu9250_spi_poll_data(struct mpu9250_spi_SensorData *Data);
void driver_mpu9250_spi_calibrate_update_matrices(float dS[6], float JS[6][6], float beta[6], float data[3]);
void driver_mpu9250_spi_calibrate_reset_matrices(float dS[6], float JS[6][6]);
void driver_mpu9250_spi_calibrate_find_delta(float dS[6], float JS[6][6], float delta[6]);
int  driver_mpu9250_spi_init_gyro(void);
int  driver_mpu9250_spi_init_accel(void);


u32 mpu9250_get_time_micros(void)
{
	static struct  timeval mpu9250_start;
	static u8 mpu9250_record = 0;
	struct  timeval time_now;

	if(mpu9250_record == 0){
		mpu9250_record = 1;
		gettimeofday(&mpu9250_start,NULL);
	}
	gettimeofday(&time_now,NULL);
	if(time_now.tv_sec > mpu9250_start.tv_sec)
		return (time_now.tv_sec - mpu9250_start.tv_sec)*1000000 + time_now.tv_usec  - mpu9250_start.tv_usec;
	else
		return time_now.tv_usec  - mpu9250_start.tv_usec;
}


int driver_mpu9250_spi_init(void)
{
	int ret;
	mpu9250_property.dev = MPU9250_SPI_RP3;
	mpu9250_property.mode = MODE3;
	mpu9250_property.bits = 8;
	mpu9250_property.speed = 1000000;
	memset(&mpu9250_spi_raw_data,0,sizeof(mpu9250_spi_raw_data));

	ret = mpu9250_spi_open(&mpu9250_property);
	if(ret < 0){
		perror("failed to open spi");
		return -1;
	}
	usleep(50000);
	// reset chip
	ret = mpu9250_spi_writeRegister(&mpu9250_property, MPU9250_PWR_MGMT_1, MPU9250_PWRMGMT_IMU_RST);
	if(ret < 0){
		perror("failed to reset the mpu9250");
		return -1;
	}
	// give chip some time to initialize
	usleep(50000);
	// power management config
	ret = mpu9250_spi_writeRegister(&mpu9250_property, MPU9250_PWR_MGMT_1, MPU9250_PWRMGMT_INTERN_CLK);
	if(ret < 0){
		perror("failed to set the mpu9250's clk");
		return -1;
	}
	usleep(10000);
	u8 id = 0;
	ret = mpu9250_spi_readRegister(&mpu9250_property, MPU9250_WHO_AM_I, &id);
	if ((ret < 0) ||(id != MPU9250_Device_ID)){
		perror("failed to detect mpu9250_spi device");
		return -1;
	}
	//set the gyro scale
	usleep(1000);
	ret = mpu9250_spi_writeRegister(&mpu9250_property, MPU9250_GYRO_CONFIG, MPU9250_SCALE_2000_DEG);
	if(ret < 0){
		perror("failed to set the mpu9250 gyro's range");
		return -1;
	}
	//set LPF of gyro
	usleep(1000);
	ret = mpu9250_spi_writeRegister(&mpu9250_property, MPU9250_CONFIG, MPU9250_GYRO_LOWPASS_41_HZ);
	if(ret < 0){
		perror("failed to set the mpu9250 gyro's low pass filter");
		return -1;
	}
	//set the accel scale
	usleep(1000);
	ret = mpu9250_spi_writeRegister(&mpu9250_property, MPU9250_ACCEL_CONFIG, MPU9250_ACCEL_8G);
	if(ret < 0){
		perror("failed to set the mpu9250 acce's range");
		return -1;
	}
	//set LPF of accel
	usleep(1000);
	ret = mpu9250_spi_writeRegister(&mpu9250_property, MPU9250_ACCEL_CONFIG_2, MPU9250_ACCEL_LOWPASS_41_HZ);
	if(ret < 0){
		perror("failed to set the mpu9250 acce's low pass filter");
		return -1;
	}
	//set sampling Hz
	usleep(1000);
	ret = mpu9250_spi_writeRegister(&mpu9250_property, MPU9250_SMPLRT_DIV, MPUREG_SMPLRT_200HZ);
	if(ret < 0){
		perror("failed to set the mpu9250 sample rate");
		return -1;
	}
	usleep(1000);
	ret = mpu9250_spi_writeRegister(&mpu9250_property, MPU9250_PWR_MGMT_2,0X00);
	if(ret < 0){
		perror("failed to turn on  the mpu9250 gyro & acce");
		return -1;
	}
	ret = driver_mpu9250_spi_init_gyro();
	if(ret < 0){
		perror("failed to initilization gyro");
		return -1;
	}

	return 0;
}

void driver_mpu9250_spi_release(void)
{
	mpu9250_spi_close(&mpu9250_property);
}
int driver_mpu9250_spi_poll_data(struct mpu9250_spi_SensorData *Data)
{
	u8 buffer[14];
	if(mpu9250_spi_readRegisters(&mpu9250_property,sizeof(buffer),MPU9250_ACCEL_XOUT_H,buffer) < 0){
		perror("failed to read data from mpu9250_spi");
		return -1;
	}
#define int16_val(v, idx) ((s16)(((u16)v[2*idx] << 8) | v[2*idx+1]))
	Data->accel_x += int16_val(buffer,1);
	Data->accel_y += int16_val(buffer,0);
	Data->accel_z -= int16_val(buffer,2);
	Data->temp += ((float)(int16_val(buffer,3))/333.87 + 21.0);
	Data->gyro_x += int16_val(buffer,5);
	Data->gyro_y += int16_val(buffer,4);
	Data->gyro_z -= int16_val(buffer,6);
	Data->poll_data_count++;

	if(Data->poll_data_count >= 20){
		Data->accel_x /= 2;
		Data->accel_y /= 2;
		Data->accel_z /= 2;
		Data->temp /= 2;
		Data->gyro_x /= 2;
		Data->gyro_y /= 2;
		Data->gyro_z /= 2;
		Data->poll_data_count /= 2;
	}
	return 0;
}

float driver_mpu9250_spi_get_temperature(void)
{
	return mpu9250_spi_temperature;
}

Vector3f driver_mpu9250_spi_get_gyro(void)
{
	return mpu9250_spi_gyro;
}
void  driver_mpu9250_spi_set_gyro_offsets(const Vector3f *gyro)
{
	memcpy(&mpu9250_spi_gyro_offset,gyro,sizeof(mpu9250_spi_gyro_offset));
}
Vector3f driver_mpu9250_spi_get_gyro_offsets(void)
{
	return mpu9250_spi_gyro_offset;
}

Vector3f  driver_mpu9250_spi_get_accel(void)
{
	return mpu9250_spi_accel;
}
void  driver_mpu9250_spi_set_accel_scale(const Vector3f *accel_scale)
{
	memcpy(&mpu9250_spi_accel_scale,accel_scale,sizeof(mpu9250_spi_accel_scale));
}
void  driver_mpu9250_spi_set_accel_offsets(const Vector3f *accel_offset)
{
	memcpy(&mpu9250_spi_accel_offset,accel_offset,sizeof(mpu9250_spi_accel_offset));
}
Vector3f  driver_mpu9250_spi_get_accel_scale(void)
{
	return mpu9250_spi_accel_scale;
}
Vector3f  driver_mpu9250_spi_get_accel_offsets(void)
{
	return mpu9250_spi_accel_offset;
}

float driver_mpu9250_spi_get_gyro_drift_rate(void)
{
    return ToRad(0.5/60);
}
float driver_mpu9250_spi_get_delta_time(void)
{
	return 0.005*mpu9250_poll_data_count;
}

int driver_mpu9250_spi_read_raw_data(void)
{
	if((mpu9250_get_time_micros() - mpu9250_spi_last_update) < 5000)
		return -1;
	if(driver_mpu9250_spi_poll_data(&mpu9250_spi_raw_data) < 0){
		perror("failed to read raw data from mpu9250_spi");
		return -2;
	}
	mpu9250_spi_last_update = mpu9250_get_time_micros();
	return 0;
}

int driver_mpu9250_spi_update(void)
{
	if(mpu9250_spi_raw_data.poll_data_count > 0){
		mpu9250_spi_temperature = mpu9250_spi_raw_data.temp/mpu9250_spi_raw_data.poll_data_count;
		mpu9250_spi_accel.x = mpu9250_spi_raw_data.accel_x/mpu9250_spi_raw_data.poll_data_count*MPU9250_ACCEL_SCALE_1G*mpu9250_spi_accel_scale.x - mpu9250_spi_accel_offset.x;
		mpu9250_spi_accel.y = mpu9250_spi_raw_data.accel_y/mpu9250_spi_raw_data.poll_data_count*MPU9250_ACCEL_SCALE_1G*mpu9250_spi_accel_scale.y - mpu9250_spi_accel_offset.y;
		mpu9250_spi_accel.z = mpu9250_spi_raw_data.accel_z/mpu9250_spi_raw_data.poll_data_count*MPU9250_ACCEL_SCALE_1G*mpu9250_spi_accel_scale.z - mpu9250_spi_accel_offset.z;
		mpu9250_spi_gyro.x = mpu9250_spi_raw_data.gyro_x/mpu9250_spi_raw_data.poll_data_count*mpu9250_spi_gyro_scale - mpu9250_spi_gyro_offset.x;
		mpu9250_spi_gyro.y = mpu9250_spi_raw_data.gyro_y/mpu9250_spi_raw_data.poll_data_count*mpu9250_spi_gyro_scale - mpu9250_spi_gyro_offset.y;
		mpu9250_spi_gyro.z = mpu9250_spi_raw_data.gyro_z/mpu9250_spi_raw_data.poll_data_count*mpu9250_spi_gyro_scale - mpu9250_spi_gyro_offset.z;
		mpu9250_poll_data_count = mpu9250_spi_raw_data.poll_data_count;
		memset(&mpu9250_spi_raw_data,0,sizeof(mpu9250_spi_raw_data));
		return 0;
	}
	return -1;
}

// Perform cold startup initialization for just the accelerometers.
int driver_mpu9250_spi_init_accel(void){
    u8 num_accels = 1, i;
    Vector3f prev;
    Vector3f accel_offset;
    float total_change;
    float max_offset;
	Vector3f temp;
	usleep(100000);
	memset(&mpu9250_spi_accel_offset,0,sizeof(mpu9250_spi_accel_offset));
	mpu9250_spi_accel_scale.x = 1;
	mpu9250_spi_accel_scale.y = 1;
	mpu9250_spi_accel_scale.z = 1;
	// initialise accel offsets to a large value the first time
	// this will force us to calibrate accels at least twice
	mpu9250_spi_accel_offset.x = 0;
	mpu9250_spi_accel_offset.y = 0;
	mpu9250_spi_accel_offset.z = 0;
    //loop until we calculate acceptable offsets
    while (true) {
    	driver_mpu9250_spi_read_raw_data();
        //get latest accelerometer values
        driver_mpu9250_spi_update();
		//store old offsets
		prev = accel_offset;
		//get new offsets
		accel_offset = driver_mpu9250_spi_get_accel();
        //We take some readings...
        for(i = 0; i < 50; i++) {
        	driver_mpu9250_spi_read_raw_data();
            driver_mpu9250_spi_update();
            //low pass filter the offsets
			temp = driver_mpu9250_spi_get_accel();
			accel_offset.x = accel_offset.x * 0.9f + temp.x * 0.1f;
			accel_offset.y = accel_offset.y * 0.9f + temp.y * 0.1f;
			accel_offset.z = accel_offset.z * 0.9f + temp.z * 0.1f;
            usleep(20000);
        }
		//null gravity from the Z accel
		accel_offset.z += GRAVITY_MSS;
		total_change = fabsf(prev.x - accel_offset.x) + fabsf(prev.y - accel_offset.y) + fabsf(prev.z - accel_offset.z);
		max_offset = (accel_offset.x > accel_offset.y) ? accel_offset.x : accel_offset.y;
		max_offset = (max_offset > accel_offset.z) ? max_offset : accel_offset.z;
        uint8_t num_converged = 0;
		if (total_change <= INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE
				&& max_offset <= INERTIAL_SENSOR_ACCEL_MAX_OFFSET) {
			num_converged++;
		}
        if (num_converged == num_accels)
        	break;
		usleep(50000);
    }
    // set the global accel offsets
    mpu9250_spi_accel_offset = accel_offset;
    return 0;
}

int driver_mpu9250_spi_init_gyro(void){
	u8 num_gyros = 1, c, j;
    Vector3f last_average, best_avg;
    float best_diff;
    bool converged;
	// cold start
	//remove existing gyro offsets
	memset(&mpu9250_spi_gyro_offset,0,sizeof(mpu9250_spi_gyro_offset));
	memset(&last_average,0,sizeof(last_average));
	best_diff = 0;
	converged = false;
    for(c = 0; c < 5; c++) {
		usleep(5000);
		driver_mpu9250_spi_read_raw_data();
        driver_mpu9250_spi_update();
    }
    u8 num_converged = 0;
    // we try to get a good calibration estimate for up to 10 seconds
    // if the gyros are stable, we should get it in 1 second
    for (j = 0; j <= 20 && num_converged < num_gyros; j++) {
        Vector3f gyro_sum, gyro_avg, gyro_diff;
		Vector3f temp;
        float diff_norm;
        u8 i;
		memset(&gyro_sum,0,sizeof(gyro_sum));
        for (i=0; i<50; i++) {
        	driver_mpu9250_spi_read_raw_data();
			driver_mpu9250_spi_update();
			temp = driver_mpu9250_spi_get_gyro();
			gyro_sum.x += temp.x;
			gyro_sum.y += temp.y;
			gyro_sum.z += temp.z;
			usleep(5000);
        }
		gyro_avg.x = gyro_sum.x / i;
		gyro_avg.y = gyro_sum.y / i;
		gyro_avg.z = gyro_sum.z / i;

		gyro_diff.x = last_average.x - gyro_avg.x;
		gyro_diff.y = last_average.y - gyro_avg.y;
		gyro_diff.z = last_average.z - gyro_avg.z;

		diff_norm = Vector3f_length(&gyro_diff);
		if (j == 0) {
			best_diff = diff_norm;
			best_avg  = gyro_avg;
		} else if (Vector3f_length(&gyro_diff) < ToRad(0.1f)) {
			// we want the average to be within 0.1 bit, which is 0.04 degrees/s
			last_average.x = (gyro_avg.x * 0.5f) + (last_average.x * 0.5f);
			last_average.y = (gyro_avg.y * 0.5f) + (last_average.y * 0.5f);
			last_average.z = (gyro_avg.z * 0.5f) + (last_average.z * 0.5f);
			mpu9250_spi_gyro_offset.x = last_average.x;
			mpu9250_spi_gyro_offset.y = last_average.y;
			mpu9250_spi_gyro_offset.z = last_average.z;
			converged = true;
			num_converged++;
		} else if (diff_norm < best_diff) {
			best_diff = diff_norm;
			best_avg.x = (gyro_avg.x * 0.5f) + (last_average.x * 0.5f);
			best_avg.y = (gyro_avg.y * 0.5f) + (last_average.y * 0.5f);
			best_avg.z = (gyro_avg.z * 0.5f) + (last_average.z * 0.5f);
		}
		last_average.x = gyro_avg.x;
		last_average.y = gyro_avg.y;
		last_average.z = gyro_avg.z;
    }
    if (num_converged == num_gyros) {
        // all OK
        return 0;
    }
    // we've kept the user waiting long enough - use the best pair we found so far
	if (!converged) {
		printf("\ngyro did not converge: diff=%f dps\n", ToDeg(best_diff));
		mpu9250_spi_gyro_offset.x = best_avg.x;
		mpu9250_spi_gyro_offset.y = best_avg.y;
		mpu9250_spi_gyro_offset.z = best_avg.z;
	}
	return -1;
}

// Calibration routines borrowed from Rolfe Schmidt
// blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
// original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
void driver_mpu9250_spi_calibrate_update_matrices(float dS[6], float JS[6][6],float beta[6], float data[3])
{
	u8 j, k;
    float dx, b;
    float residual = 1.0;
    float jacobian[6];
    for(j=0; j<3; j++ ) {
        b = beta[3+j];
        dx = (float)data[j] - beta[j];
        residual -= b*b*dx*dx;
        jacobian[j] = 2.0f*b*b*dx;
        jacobian[3+j] = -2.0f*b*dx*dx;
    }
    for(j=0; j<6; j++ ) {
        dS[j] += jacobian[j]*residual;
        for(k=0; k<6; k++ ) {
            JS[j][k] += jacobian[j]*jacobian[k];
        }
    }
}
// driver_mpu9250_spi_calibrate_reset_matrices - clears matrices
void driver_mpu9250_spi_calibrate_reset_matrices(float dS[6], float JS[6][6])
{
	u8 j, k;
    for(j=0; j<6; j++ ) {
        dS[j] = 0.0f;
        for(k=0; k<6; k++ ) {
            JS[j][k] = 0.0f;
        }
    }
}

void driver_mpu9250_spi_calibrate_find_delta(float dS[6], float JS[6][6], float delta[6])
{
	s8 i, j, k;
    //Solve 6-d matrix equation JS*x = dS
    //first put in upper triangular form
    float mu;
    //make upper triangular
    for(i=0; i<6; i++ ) {
        //eliminate all nonzero entries below JS[i][i]
        for(j=i+1; j<6; j++ ) {
            mu = JS[i][j]/JS[i][i];
            if( mu != 0.0f ) {
                dS[j] -= mu*dS[i];
                for(k=j; k<6; k++ ) {
                    JS[k][j] -= mu*JS[k][i];
                }
            }
        }
    }
    //back-substitute
    for(i=5; i>=0; i-- ) {
        dS[i] /= JS[i][i];
        JS[i][i] = 1.0f;

        for(j=0; j<i; j++ ) {
            mu = JS[i][j];
            dS[j] -= mu*dS[i];
            JS[i][j] = 0.0f;
        }
    }
    for(i=0; i<6; i++ ) {
        delta[i] = dS[i];
    }
}
//driver_mpu9250_spi_calculate_trim  - calculates the x and y trim angles (in radians) given a raw accel sample (i.e. no scaling or offsets applied) taken when the vehicle was level
void driver_mpu9250_spi_calculate_trim(Vector3f accel_sample, float * trim_roll, float * trim_pitch)
{
    // scale sample and apply offsets
    Vector3f accel_scale,accel_offsets,scaled_accels_x,scaled_accels_y,vertical;
    accel_scale = mpu9250_spi_accel_scale;
    accel_offsets = mpu9250_spi_accel_offset;
    scaled_accels_x.x =  accel_sample.x * accel_scale.x - accel_offsets.x;
    scaled_accels_x.y =  0;
	scaled_accels_x.z =  accel_sample.z * accel_scale.z - accel_offsets.z ;
    scaled_accels_y.x =  0;
    scaled_accels_y.y =  accel_sample.y * accel_scale.y - accel_offsets.y;
    scaled_accels_y.z =  accel_sample.z * accel_scale.z - accel_offsets.z;
    // calculate x and y axis angle (i.e. roll and pitch angles)
    vertical.x = 0;
	vertical.y = 0;
	vertical.z = -1;

    *trim_roll  = Vector3f_angle(scaled_accels_y,vertical);
    *trim_pitch = Vector3f_angle(scaled_accels_x,vertical);

    // angle call doesn't return the sign so take care of it here
    if( scaled_accels_y.y > 0 ) {
        *trim_roll = -*trim_roll;
    }
    if( scaled_accels_x.x < 0 ) {
        *trim_pitch = -*trim_pitch;
    }
}
// _calibrate_model - perform low level accel calibration
// accel_sample are accelerometer samples collected in 6 different positions
// accel_offsets are output from the calibration routine
// accel_scale are output from the calibration routine
// returns true if successful
int driver_mpu9250_spi_calibrate_accel( Vector3f accel_sample[6],Vector3f * accel_offsets, Vector3f * accel_scale )
{
    s16 num_iterations = 0;
    u8 i;
    float eps = 0.000000001f;
    float change = 100.0;
    float data[3];
    float beta[6];
    float delta[6];
    float ds[6];
    float JS[6][6];
    // reset
    beta[0] = beta[1] = beta[2] = 0;
    beta[3] = beta[4] = beta[5] = 1.0f/GRAVITY_MSS;
    while(num_iterations < 20 && change > eps) {
        num_iterations++;
        driver_mpu9250_spi_calibrate_reset_matrices(ds, JS);
        for(i=0; i<6; i++ ) {
            data[0] = accel_sample[i].x;
            data[1] = accel_sample[i].y;
            data[2] = accel_sample[i].z;
            driver_mpu9250_spi_calibrate_update_matrices(ds, JS, beta, data);
        }
        driver_mpu9250_spi_calibrate_find_delta(ds, JS, delta);
        change =    delta[0]*delta[0] +
                    delta[0]*delta[0] +
                    delta[1]*delta[1] +
                    delta[2]*delta[2] +
                    delta[3]*delta[3] / (beta[3]*beta[3]) +
                    delta[4]*delta[4] / (beta[4]*beta[4]) +
                    delta[5]*delta[5] / (beta[5]*beta[5]);

        for(i=0; i<6; i++ ) {
            beta[i] -= delta[i];
        }
    }
    // copy results out
    accel_scale->x = beta[3] * GRAVITY_MSS;
    accel_scale->y = beta[4] * GRAVITY_MSS;
    accel_scale->z = beta[5] * GRAVITY_MSS;
    accel_offsets->x = beta[0] * accel_scale->x;
    accel_offsets->y = beta[1] * accel_scale->y;
    accel_offsets->z = beta[2] * accel_scale->z;
    // sanity check scale
    if( Vector3f_is_nan(accel_scale) || fabsf(accel_scale->x-1.0f) > 0.1f || fabsf(accel_scale->y-1.0f) > 0.1f || fabsf(accel_scale->z-1.0f) > 0.1f ) {
        return -1;
    }
    // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
    if( Vector3f_is_nan(accel_offsets) || fabsf(accel_offsets->x) > 3.5f || fabsf(accel_offsets->y) > 3.5f || fabsf(accel_offsets->z) > 3.5f ) {
        return -1;
    }
    //return success or failure
    return 0;
}


#define AHRS_TRIM_LIMIT 15.0f        // maximum trim angle in degrees
Vector3f  ahrs_dcm_trim = {-0.095627f,-0.013305f};
//extern Vector3f  ahrs_dcm_trim;
// driver_mpu9250_spi_save_trim
void driver_mpu9250_spi_save_trim(Vector3f new_trim)
{
	new_trim.x = constrain_float(new_trim.x, ToRad(-AHRS_TRIM_LIMIT), ToRad(AHRS_TRIM_LIMIT));
	new_trim.y = constrain_float(new_trim.y, ToRad(-AHRS_TRIM_LIMIT), ToRad(AHRS_TRIM_LIMIT));
	new_trim.z = 0;
	// save to flash
//	app_save_sensor_calibrated_data(Ahrs_DCM_trim_ID,&new_trim);
}
// driver_mpu9250_spi_set_trim
void driver_mpu9250_spi_set_trim(Vector3f new_trim)
{
    Vector3f trim;
    trim.x = constrain_float(new_trim.x, ToRad(-AHRS_TRIM_LIMIT), ToRad(AHRS_TRIM_LIMIT));
    trim.y = constrain_float(new_trim.y, ToRad(-AHRS_TRIM_LIMIT), ToRad(AHRS_TRIM_LIMIT));
	trim.z = 0;
    ahrs_dcm_trim = trim;
}



