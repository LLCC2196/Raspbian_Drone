#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<stdbool.h>
#include<unistd.h>
#include<fcntl.h>
#include<errno.h>
#include<pthread.h>
#include<semaphore.h>
#include<sys/time.h>

#include "my_types.h"
#include"MadgwickAHRS.h"
#include"app_math.h"

#include "hmc5883l.h"
#include "mpu9250.h"
#include "uart.h"
#include "PCA9685.h"
#include "isr.h"
#include "rc.h"

#include "rc_Interface.h"
#include "motor_Interface.h"
#include "mpu9250_Interface.h"
#include "hmc5883l_Interface.h"

#include"rc_channel.h"
#include"attitude_controller.h"
#include"calculate_motor_output.h"
#include"pid.h"


struct Vector3f *ml_accel;
struct Vector3f *ml_gyro;
struct Vector3i *ml_mag;

pthread_mutex_t mutex_sensor_initialization;
pthread_t thread_calibrate;
pthread_t thread_mpu9250_spi;
pthread_t thread_hmc5883l_i2c;
pthread_t thread_motor;
pthread_t thread_rc;

float g_dt = 0;
u8 sensor_inited = 0;
int uart_fd = NULL;

void sensordata_print();
void driver_init();
int thread_creat_function(void);
//int rate_loop_function(float dt);

u32 main_get_time_inerval_micros(struct timeval *time_start, struct  timeval *time_now);
//#define mpu9250_calibrate
#ifdef mpu9250_calibrate
void *thread_calibrate_mpu9250_device(void *arg);
#endif

void sensor_update(struct timeval *tnow)
{
    	struct timeval time_tmp;
    	time_tmp = interface_rc_receiver_get_last_update();
    	if(main_get_time_inerval_micros(&time_tmp,tnow) > 10000){
    		interface_rc_receiver_update();
    	}
    	time_tmp = interface_mpu9250_spi_get_last_update();
    	if(main_get_time_inerval_micros(&time_tmp,tnow) > 10000){
    		interface_mpu9250_spi_update();
    	}
    	time_tmp = interface_hmc5883l_i2c_get_last_update();
    	if(main_get_time_inerval_micros(&time_tmp,tnow) > 100000){
    		interface_hmc5883l_i2c_update();
    	}
}


int main()
{
	int channel = 0;
//	driver_init();
	struct timeval time_now;
	static struct timeval last_loop_update;
	static u16 motor_output[4];
	static u8 update_throttle = 0;
	static u32 time_inerval = 0;

    if(pthread_mutex_init(&mutex_sensor_initialization, NULL) < 0){
        perror("mutex_sensor_initialization initialization failed");
        return -1;
    }
	if(thread_creat_function() < 0)
		return -2;
	while(!sensor_inited){
		pthread_mutex_lock(&mutex_sensor_initialization);
		if(mpu9250_spi_sensor_inited && hmc5883l_i2c_sensor_inited && rc_inited && motor_inited){
			sensor_inited = 1;
			printf("initialization success\r\n");
		}
		pthread_mutex_unlock(&mutex_sensor_initialization);
		usleep(10000);
	}

	init_motor();
	pid_regular_init();
	attitude_controller_init();

	MadgwickAHRS_init();
	gettimeofday(&last_loop_update,NULL);

	while(1)
	{
		gettimeofday(&time_now,NULL);
		time_inerval = main_get_time_inerval_micros(&last_loop_update,&time_now);
		sensor_update(&time_now);
//		sensordata_print();
/*
		int i;
		for (i = 0 ; i<6 ;i++)
		{
	        printf("RC%d : %d\t",i,interface_rc_receiver_get_rc_value(i));
		}
		printf("\n");
*/
		if(time_inerval > 6000){

		//			fprintf(stderr,"%d\r\n",time_inerval);
					g_dt = ((float)time_inerval)/1000000.0f;
					gettimeofday(&last_loop_update,NULL);

					MadgwickAHRSupdate(interface_mpu9250_spi_get_gyro().x,interface_mpu9250_spi_get_gyro().y,interface_mpu9250_spi_get_gyro().z,
										interface_mpu9250_spi_get_accel().x,interface_mpu9250_spi_get_accel().y,interface_mpu9250_spi_get_accel().z,
										interface_hmc5883l_i2c_get_field().x, interface_hmc5883l_i2c_get_field().y, interface_hmc5883l_i2c_get_field().z,g_dt);
                                    	
					rc_channel_pwm_convert();
					run_rate_controller(g_dt);
/*
					int i;
					for (i = 0 ; i<4 ;i++)
					{
					printf("motor%d : %d \t ",i,motor_output[i]);
					if(i == 3)
						printf("\n");
					}
*/
					motor_output_armed(motor_output);					
					if(interface_rc_receiver_get_rc_value(6) < 1800 || interface_rc_receiver_get_rc_value(2) < 1100){						
						motor_output[0] = 1000;
						motor_output[1] = 1000;
						motor_output[2] = 1000;
						motor_output[3] = 1000;
//						printf("unarmed...\n");
					}		


					int i;
 					s32 angle[3];
					angle[0]=MadgwickAHRS_get_eular_roll();
					angle[1]=MadgwickAHRS_get_eular_pitch();
					angle[2]=MadgwickAHRS_get_eular_yaw();
					for(i = 0;i<3;i++)
					{
					printf("angle %d : %d\t",i,angle[i]);
					if(2 == i)
					printf("\n");
					}
	
					interface_motor_update(motor_output);
					update_roll_pitch_mode();
					update_yaw_mode(g_dt);
					update_rate_throttle_mode();
		}
		update_rate_controller_target();
	}
	return 0;
}
/*
int rate_loop_function(float dt)
{
	static int AHRS_init_counter = 500;

	ml_accel = interface_mpu9250_spi_get_accel();
	ml_gyro = interface_mpu9250_spi_get_gyro();
	ml_mag = interface_hmc5883l_i2c_get_field();

	MadgwickAHRSupdate(ml_gyro->x, ml_gyro->y, ml_gyro->z,
						ml_accel->x, ml_accel->y, ml_accel->z,
						ml_mag->x, ml_mag->y, ml_mag->z, dt);

	if(AHRS_init_counter > 0){
		AHRS_init_counter -= 1;
	}else {

		inf_rc_pwm_convert();
		run_rate_controller(dt);
		motor_output_armed(motor_output);

		if(inf_rc_get_rc_value(6) < 1500 || inf_rc_get_rc_value(2) < 975){
			//disarm or lost connect
			motor_output[0] = 1086;
			motor_output[1] = 1086;
			motor_output[2] = 1086;
			motor_output[3] = 1086;
		}

		inf_motor_update((void *)motor_share_addr ,motor_output);
		update_roll_pitch_mode();
		update_yaw_mode(dt);

		static u8 update_throttle = 0;
		update_throttle++;
		if(update_throttle == 2){
			update_throttle = 0;
			if(RATE_THROTTLE_MANUAL == get_rate_throttle_mode())
				update_rate_throttle_mode();
		}
		update_rate_controller_target();

	}

	return 0;
}
*/
u32 main_get_time_inerval_micros(struct timeval *time_start, struct  timeval *time_now)
{
	if(time_now->tv_sec > time_start->tv_sec)
		return (time_now->tv_sec - time_start->tv_sec)*1000000 + time_now->tv_usec  - time_start->tv_usec;
	else
		return time_now->tv_usec  - time_start->tv_usec;
}
void driver_init()
{
	uart_open(&uart_fd);
	driver_hmc5883l_i2c_init();
	driver_mpu9250_spi_init();
	driver_mpu9250_spi_init_accel();
	driver_mpu9250_spi_init_gyro();
	PCA9685_init();
	RC_Init();
	printf("all drivers inited!");
}

void sensordata_print()
{
	int i = 0,channel = 0;

	float buffer[3];
	u8 n[4] ={1,2,3,4};
	printf("magdata: x : %f		y: %f		z：%f \n",interface_hmc5883l_i2c_get_field().x, interface_hmc5883l_i2c_get_field().y, interface_hmc5883l_i2c_get_field().z);
	printf("gyro: x : %f		y: %f		z：%f \n",interface_mpu9250_spi_get_gyro().x , interface_mpu9250_spi_get_gyro().y , interface_mpu9250_spi_get_gyro().z ) ;
	printf("accel: x : %f		y: %f		z：%f \n",interface_mpu9250_spi_get_accel().x , interface_mpu9250_spi_get_accel().y , interface_mpu9250_spi_get_accel().z ) ;
	buffer[0]=interface_mpu9250_spi_get_gyro().x;
	buffer[1]=interface_mpu9250_spi_get_gyro().y;
	buffer[2]=interface_mpu9250_spi_get_gyro().z;
	uart_send_message(&uart_fd,buffer,3);


//	for(i = 0;i<4;i++)                                      //motor-control-test
//	{
//	PCA9685_setPWM(n[i],2000);
//	usleep(500000);
//	PCA9685_setPWM(n[i],1000);
//	}

	for (channel = 0 ; channel<7 ;channel++)
		printf("RC%d : %d\t",channel,interface_rc_receiver_get_rc_value(channel));
	printf("\n");


}

int thread_creat_function(void)
{
    //create mpu9250 thread to receive IMU input.
    if(pthread_create(&thread_mpu9250_spi, NULL, thread_mpu9250_spi_device, NULL) < 0){
        perror("create mpu9250 thread failed");
        return -2;
    }
     //create hmc5883l thread to receive magnetometer input.
    if(pthread_create(&thread_hmc5883l_i2c, NULL, thread_hmc5883l_i2c_device, NULL) < 0){
        perror("create hmc5883l thread failed");
        return -3;
    }
    //create rc thread to receive rc input.
    if(pthread_create(&thread_rc, NULL, thread_receive_rc_data, NULL) < 0){
        perror("create tty thread failed");
        return -6;
    }
    //create motor thread to send motor motion.
    if(pthread_create(&thread_motor, NULL, thread_send_motor_data, NULL) < 0){
        perror("create tty thread failed");
        return -6;
    }



#ifdef mpu9250_calibrate
    //create tty thread to calibrate acceleration.
    if(pthread_create(&thread_calibrate, NULL, thread_calibrate_mpu9250_device, NULL) < 0){
        perror("create tty thread failed");
        return -7;
    }
#endif
    return 0;
}
#ifdef mpu9250_calibrate
void *thread_calibrate_mpu9250_device(void *arg)
{
	int ret;
	float trim_r,trim_p;
	while(1){
		usleep(100000);
		printf("thread_calibrate_mpu9250_device\n");
		scanf("%d",&ret);
		printf("thread_calibrate_mpu9250_device\n");
		printf("%d",ret);
		if(ret == 9){
			ret = interface_mpu9250_spi_calibration(&trim_r,&trim_p);
			if(ret == 0)
				printf("trim_r = %f,trim_p = %f",trim_r,trim_p);
		}
	}
}
#endif
