/*
 * motor.c
 *
 *  Created on: Nov 14, 2015
 *      Author: zz269
 */
#include <stdio.h>
#include"app_math.h"
#include "rc_channel.h"
#include "attitude_controller.h"
#include "calculate_motor_output.h"

#define AP_MOTORS_MOT_1 0
#define AP_MOTORS_MOT_2 1
#define AP_MOTORS_MOT_3 2
#define AP_MOTORS_MOT_4 3
#define AP_MOTORS_MATRIX_YAW_FACTOR_CCW 1
#define AP_MOTORS_MATRIX_YAW_FACTOR_CW -1
#define AP_MOTORS_MAX_NUM_MOTORS 4

float _roll_factor[AP_MOTORS_MAX_NUM_MOTORS];
float _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS];
float _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];
//yaw rate limit
#define AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM 200

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

//struct AP_Motors_limit motor_limit = {1,1,1,1};
struct AP_Motors_limit motor_limit;

struct AP_Motors_limit *get_motor_limit(void){
	return &motor_limit;
}

s16 roll_pitch_yaw_servo_to_PWM_out(s16 servo_out)
{
	return servo_out*0.1;
}
s16 throttle_servo_to_PWM_out(s16 servo_out)
{
	return servo_out*(rc3_radio_max_get()-rc3_radio_min_get())/THROTTLE_RANGE + rc3_radio_min_get();
}
void calculate_pwm(s16 *rc_pwm_out)
{ 
	rc_pwm_out[0] = roll_pitch_yaw_servo_to_PWM_out(get_rc_servo_out(0));
	rc_pwm_out[1] = roll_pitch_yaw_servo_to_PWM_out(get_rc_servo_out(1));
	rc_pwm_out[2] = throttle_servo_to_PWM_out(get_rc_servo_out(2));
	rc_pwm_out[3] = roll_pitch_yaw_servo_to_PWM_out(get_rc_servo_out(3));

//	printf("%d\t%d\t%d\t%d\n",rc_pwm_out[0],rc_pwm_out[1],rc_pwm_out[2],rc_pwm_out[3]);
}
//calculate motor output
void motor_output_armed(u16 *motor_output)
{
	s16 motor_out[AP_MOTORS_MAX_NUM_MOTORS];
	s16 rc_pwm_out[AP_MOTORS_MAX_NUM_MOTORS];
	s8 i;
	s16 out_min_pwm = rc3_radio_min_get() + 117;
	s16 out_max_pwm = rc3_radio_max_get();
	s16 out_mid_pwm = (out_min_pwm + out_max_pwm)/2;

	s16 out_best_thr_pwm;
	float rpy_scale = 1.0f;

	s16 rpy_out[AP_MOTORS_MAX_NUM_MOTORS];

	s16 rpy_low = 0;
	s16 rpy_high = 0;
	s16 yaw_allowed;
	s16 thr_adj;

	motor_limit.roll_pitch = false;
	motor_limit.yaw = false;
	motor_limit.throttle_lower = false;
	motor_limit.throttle_upper = false;

	calculate_pwm(rc_pwm_out);

	//add roll & pitch controller
	for(i=0;i<AP_MOTORS_MAX_NUM_MOTORS;i++){
		rpy_out[i] = rc_pwm_out[0] * _roll_factor[i] + rc_pwm_out[1] * _pitch_factor[i];//_roll_factor:-0.707106;_pitch_factor:0.707106

		if(rpy_out[i] < rpy_low){
			rpy_low = rpy_out[i];
		}
		if(rpy_out[i] > rpy_high){
			rpy_high = rpy_out[i];
		}
	}
	s16 motor_mid = (rpy_low+rpy_high)/2;
	out_best_thr_pwm = min( out_mid_pwm-motor_mid , max( rc_pwm_out[2] , (rc_pwm_out[2]+rc3_radio_mid_get())/2 ) );

	yaw_allowed = min(out_max_pwm-out_best_thr_pwm , out_best_thr_pwm-out_min_pwm) - (rpy_high-rpy_low)/2;
	yaw_allowed = max(yaw_allowed,AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM);//200 to 390
	if(rc_pwm_out[3] >= 0){
		if(yaw_allowed >rc_pwm_out[3]){
			yaw_allowed = rc_pwm_out[3];
		}else{
			motor_limit.yaw = true;
		}
	}else{
		yaw_allowed = -yaw_allowed;
		if(yaw_allowed < rc_pwm_out[3]){
			yaw_allowed = rc_pwm_out[3];
		}else{
			motor_limit.yaw = true;
		}
	}
	rpy_low =0;
	rpy_high = 0;
	//add yaw controller
	for(i=0;i<AP_MOTORS_MAX_NUM_MOTORS;i++){
		rpy_out[i] = rpy_out[i] + yaw_allowed * _yaw_factor[i];

		if(rpy_out[i] < rpy_low){
			rpy_low = rpy_out[i];
		}
		if(rpy_out[i]>rpy_high){
			rpy_high = rpy_out[i];
		}
	}

	thr_adj = rc_pwm_out[2] - out_best_thr_pwm;//油门的调整量
	s16 thr_adj_max = max(out_max_pwm-(out_best_thr_pwm+rpy_high),0);//油门调整量的最大值
	if(thr_adj >0){
		if(thr_adj >thr_adj_max){
			thr_adj = thr_adj_max;
			motor_limit.throttle_upper = true;
		}
	}else if(thr_adj<0){
		s16 thr_adj_min = min(out_min_pwm-(out_best_thr_pwm+rpy_low),0);
		if(thr_adj>thr_adj_max){
			thr_adj=thr_adj_max;
			motor_limit.throttle_upper = true;
		}
		if(thr_adj<thr_adj_min){
			thr_adj = thr_adj_min;
			motor_limit.throttle_lower = true;
		}
	}

	if((rpy_low+out_best_thr_pwm)+thr_adj<out_min_pwm){
		rpy_scale = (float)(out_min_pwm-thr_adj-out_best_thr_pwm)/rpy_low;
		motor_limit.roll_pitch = true;
		motor_limit.yaw = true;
	}else if((rpy_high+out_best_thr_pwm)+thr_adj>out_max_pwm){
		rpy_scale = (float)(out_max_pwm-thr_adj-out_best_thr_pwm)/rpy_high;
		motor_limit.roll_pitch = true;
		motor_limit.yaw = true;
	}
	for(i=0;i<AP_MOTORS_MAX_NUM_MOTORS;i++){
		motor_out[i]=out_best_thr_pwm+thr_adj+rpy_scale*rpy_out[i];
	}

	for(i=0;i<AP_MOTORS_MAX_NUM_MOTORS;i++){
		motor_out[i]=constrain_int16(motor_out[i],out_min_pwm,out_max_pwm);
		motor_output[i] = motor_out[i];
	}
}
void add_motor_raw(s8 motor_num,float roll_fac,float pitch_fac,float yaw_fac)
{
	_roll_factor[motor_num] = roll_fac;
	_pitch_factor[motor_num] = pitch_fac;
	_yaw_factor[motor_num] = yaw_fac;
}
void add_motor(s8 motor_num,float angle_degrees,float yaw_factor)
{
	add_motor_raw(motor_num,cosf(radians(angle_degrees+90)),cosf(radians(angle_degrees)),yaw_factor);
}
void init_motor(void)
{
	motor_limit.roll_pitch = false;
	motor_limit.yaw = false;
	motor_limit.throttle_lower = false;
	motor_limit.throttle_upper = false;

	add_motor(AP_MOTORS_MOT_1,45,AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
	add_motor(AP_MOTORS_MOT_2,-135,AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
	add_motor(AP_MOTORS_MOT_3,-45,AP_MOTORS_MATRIX_YAW_FACTOR_CW);
	add_motor(AP_MOTORS_MOT_4,135,AP_MOTORS_MATRIX_YAW_FACTOR_CW);
}
