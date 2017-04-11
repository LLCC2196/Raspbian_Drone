/*
 * attitude_controller.c
 *
 *  Created on: Nov 14, 2015
 *      Author: zz269
 */


/*
 * Attitude.c
 *
 *  Created on: 2015年7月27日
 *      Author: zefeng
 */
#include<sys/time.h>
#include<stdbool.h>
#include<string.h>
#include<stdio.h>
#include<stdlib.h>
#include<error.h>

#include"mpu9250_Interface.h"
#include"rc_Interface.h"
#include"app_math.h"
#include"MadgwickAHRS.h"
#include"attitude_controller.h"
#include "calculate_motor_output.h"
#include"pid.h"
#include"rc_channel.h"

s16 rc_servo_out[4];
float throttle_avg=0;
s16	throttle_cruise=626;
u8 	throttle_accel_controller_active = false;
s16 throttle_accel_target_ef;

s32 roll_rate_target_ef,pitch_rate_target_ef,yaw_rate_target_ef;
s32 roll_rate_target_bf,pitch_rate_target_bf,yaw_rate_target_bf;

u8 rate_throttle_mode = RATE_THROTTLE_MANUAL;
u8 rate_roll_pitch_mode = RATE_ROLL_PITCH_STABLE;
u8 rate_yaw_mode = RATE_YAW_HOLD;

s16 alt_controller_output;
s16 loiter_controller_output_roll;
s16 loiter_controller_output_pitch;

void attitude_controller_init(void)
{
	throttle_avg=0;
	throttle_cruise=626;

	alt_controller_output = 0;
	loiter_controller_output_roll = 0;
	loiter_controller_output_pitch = 0;

	rate_throttle_mode = RATE_THROTTLE_MANUAL;
	rate_roll_pitch_mode = RATE_ROLL_PITCH_STABLE;
	rate_yaw_mode = RATE_YAW_HOLD;

	throttle_accel_controller_active = false;

	roll_rate_target_ef = 0;pitch_rate_target_ef = 0;yaw_rate_target_ef = 0;
	roll_rate_target_bf = 0;pitch_rate_target_bf = 0;yaw_rate_target_bf = 0;
}

//update throttle target
void update_rate_throttle_mode(void)
{
	s16 pilot_throttle_scaled;
	pilot_throttle_scaled = rc_channel_get_control_in(2);
	set_throttle_out(pilot_throttle_scaled,false);
	update_throttle_cruise(pilot_throttle_scaled);
}

//更新并保存油门值，定高用
void update_throttle_cruise(s16 throttle)
{
	s32 angle[3];
//	MadgwickAHRS_get_eular(angle);
	angle[0]=MadgwickAHRS_get_eular_roll();
	angle[1]=MadgwickAHRS_get_eular_pitch();
	angle[2]=MadgwickAHRS_get_eular_yaw();

	if(throttle_avg == 0){
		throttle_avg=throttle_cruise;
	}
	if(throttle>throttle_min && labs(angle[0])<500 && labs(angle[1])<500){
		throttle_avg = throttle_avg*0.99f+(float)throttle*0.01f;
		throttle_cruise = throttle_avg;
	}

}


void throttle_accel_active(void)
{
	throttle_accel_controller_active = true;
}

//disables the accel based throttle controller
void throttle_accel_deactive(void)
{
	throttle_accel_controller_active=false;
}


s16 get_angle_boost(s16 throttle)
{
	s32 angle[3];
//	MadgwickAHRS_get_eular(angle);
	angle[0]=MadgwickAHRS_get_eular_roll();
	angle[1]=MadgwickAHRS_get_eular_pitch();
	angle[2]=MadgwickAHRS_get_eular_yaw();

	float temp = cos_pitch()*cos_roll();
	s16 throttle_out;
	temp = constrain_float(temp,0.5f,1.0f);
	//reduce throttle if we go inverted
	temp = constrain_float(9000-max(labs(angle[0]),labs(angle[1])),0,3000)/(3000*temp);
	//apply scale and constrain throttle
	throttle_out = constrain_float((float)(throttle-throttle_min)*temp+throttle_min,throttle_min,throttle_max);

	return throttle_out;
}

//转换油门
void set_throttle_out(s16 throttle_out,bool apply_angle_boost)
{
	if(apply_angle_boost){
		rc_servo_out[2] = get_angle_boost(throttle_out);//0~1000
	}else{
		rc_servo_out[2] = throttle_out;
	}
}
extern s32 roll_rate_target_bf,pitch_rate_target_bf,yaw_rate_target_bf;//期望速率
//速率环控制器
void run_rate_controller(double control_period)
{
	//	struct Vector3f rate_gyro = inf_mpu9250_get_gyro();
	//	struct AP_Motors_limit limit = get_motor_limit();
		struct AP_Motors_limit *limit = get_motor_limit();
	//	rc_servo_out[0] = pid_regular(roll_rate_target_bf,(s32)(rate_gyro.x*DEGX100),
	//			control_period, &roll_pid,limit.roll_pitch);
	//	rc_servo_out[1] = pid_regular(pitch_rate_target_bf,(s32)(rate_gyro.y*DEGX100),
	//			control_period,&pitch_pid,limit.roll_pitch);
	//	rc_servo_out[3] = pid_regular(yaw_rate_target_bf,(s32)(rate_gyro.z*DEGX100),
	//				control_period,&yaw_pid,limit.yaw);
		rc_servo_out[0] = pid_regular(roll_rate_target_bf,(s32)(interface_mpu9250_spi_get_gyro().x*DEGX100),
				control_period, &roll_pid,limit->roll_pitch);
		rc_servo_out[1] = pid_regular(pitch_rate_target_bf,(s32)(interface_mpu9250_spi_get_gyro().y*DEGX100),
				control_period,&pitch_pid,limit->roll_pitch);
		rc_servo_out[3] = pid_regular(yaw_rate_target_bf,(s32)(interface_mpu9250_spi_get_gyro().z*DEGX100),
					control_period,&yaw_pid,limit->yaw);
//		printf("%d \t%d\t%d\n",rc_servo_out[0],rc_servo_out[1],rc_servo_out[3]);
	if(throttle_accel_controller_active){
		set_throttle_out(alt_controller_output,true);
	}
}

s16 get_rc_servo_out(u8 channel)
{
	if(channel >= 0 && channel <= 3){
		return rc_servo_out[channel];
	}else{
		perror("illegal servo_out request\r\n");
		return 0;
	}

}


void set_throttle_accel_target(s16 desired_acceleration)
{
	throttle_accel_target_ef=desired_acceleration;
	throttle_accel_controller_active = true;
}

//update roll pitch target
void update_roll_pitch_mode(void)
{
	s16 control_roll,control_pitch;
	switch(rate_roll_pitch_mode){
	case RATE_ROLL_PITCH_STABLE:
		control_roll = constrain_int16(rc_channel_get_control_in(0),-ROLL_PITCH_INPUT_MAX,ROLL_PITCH_INPUT_MAX);
		control_pitch = constrain_int16(rc_channel_get_control_in(1),-ROLL_PITCH_INPUT_MAX,ROLL_PITCH_INPUT_MAX);
		get_stabilize_roll(control_roll);
		get_stabilize_pitch(control_pitch);
		break;
	case RATE_ROLL_PITCH_LOITER:
		control_roll = constrain_int16(loiter_controller_output_roll,-ROLL_PITCH_INPUT_MAX,ROLL_PITCH_INPUT_MAX);
		control_pitch = constrain_int16(loiter_controller_output_pitch,-ROLL_PITCH_INPUT_MAX,ROLL_PITCH_INPUT_MAX);
		get_stabilize_roll(control_roll);
		get_stabilize_pitch(control_pitch);
		break;
	default:
		break;
	}
}

//update yaw target
void update_yaw_mode(double control_period)
{
	s16 pilot_yaw = rc_channel_get_control_in(3);
	switch(rate_yaw_mode){
	case RATE_YAW_HOLD:
		get_stabilize_yaw(pilot_yaw,control_period);
		break;
	default:
		break;
	}
}

//get roll's target rate in earth frame
void get_stabilize_roll(s32 target_angle)
{
	s32 target_rate;
	s32 angle[3];
//	MadgwickAHRS_get_eular(angle);
	angle[0]=MadgwickAHRS_get_eular_roll();
	angle[1]=MadgwickAHRS_get_eular_pitch();
	angle[2]=MadgwickAHRS_get_eular_yaw();
	target_angle = wrap_180_cd(target_angle - angle[0]);
	target_rate=(s32)get_roll_stabilize_kp(target_angle);
	target_rate = constrain_int32(target_rate,-ANGLE_RATE_MAX,ANGLE_RATE_MAX);
	roll_rate_target_ef=target_rate;
}
//get pitch's target rate in earth frame
void get_stabilize_pitch(s32 target_angle)
{
	s32 target_rate;
	s32 angle[3];
//	MadgwickAHRS_get_eular(angle);
	angle[0]=MadgwickAHRS_get_eular_roll();
	angle[1]=MadgwickAHRS_get_eular_pitch();
	angle[2]=MadgwickAHRS_get_eular_yaw();
	target_angle = wrap_180_cd(target_angle - angle[1]);
	target_rate=(s32)get_pitch_stabilize_kp(target_angle);
	target_rate = constrain_int32(target_rate,-ANGLE_RATE_MAX,ANGLE_RATE_MAX);
	pitch_rate_target_ef=target_rate;
}
//get yaw's target rate in earth frame
void get_stabilize_yaw(s32 target_angle,double control_period)
{
	static s32 control_yaw=0;
	s32 angle_error = 0;
	s32 angle[3];
//	MadgwickAHRS_get_eular(angle);
	angle[0]=MadgwickAHRS_get_eular_roll();
	angle[1]=MadgwickAHRS_get_eular_pitch();
	angle[2]=MadgwickAHRS_get_eular_yaw();
	s32 target_rate = target_angle * 4.5f;
	control_yaw += target_rate * control_period;
	control_yaw = wrap_360_cd(control_yaw);
	angle_error = wrap_180_cd(control_yaw - angle[2]);
	angle_error = constrain_int32(angle_error,-MAX_YAW_OVERSHOOT,MAX_YAW_OVERSHOOT);
	if(get_rc_servo_out(2) == 0){
		angle_error = 0;
	}
	control_yaw = wrap_360_cd(angle_error + angle[2]);
	yaw_rate_target_ef=(s32)get_yaw_stabilize_kp(angle_error)+target_rate;
}


//alt controller call this function to deliver desire throttle output
void set_alt_controller_output(s16 output)
{
	alt_controller_output = output;
}
s16 get_throttle_cruise()
{
	return throttle_cruise;
}

//loiter controller call this function to deliver desire roll & pitch angle
void set_loiter_controller_output_roll_pitch(s32 roll, s32 pitch)
{
	loiter_controller_output_roll = (s16)roll;
	loiter_controller_output_pitch = (s16)pitch;
}

void set_rate_throttle_mode(u8 throttle_mode)
{
	rate_throttle_mode = throttle_mode;
}
u8 get_rate_throttle_mode(void)
{
	return rate_throttle_mode;
}
void set_rate_roll_pitch_mode(u8 rp_mode)
{
	rate_roll_pitch_mode = rp_mode;
}
void set_rate_yaw_mode(u8 yaw_mode)
{
	rate_yaw_mode = yaw_mode;
}
//exchange target rate into body frame
void update_rate_controller_target(void)
{
	roll_rate_target_bf = roll_rate_target_ef - sin_pitch() * yaw_rate_target_ef;
	pitch_rate_target_bf = cos_roll() * pitch_rate_target_ef + sin_roll() * cos_pitch() * yaw_rate_target_ef;
	yaw_rate_target_bf = cos_pitch() * cos_roll() * yaw_rate_target_ef-sin_roll() * pitch_rate_target_ef;
}
