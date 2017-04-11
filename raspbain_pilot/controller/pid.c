/*
 * pid.c
 *
 *  Created on: Nov 14, 2015
 *      Author: zz269
 */
#include <stdio.h>
#include <string.h>
#include"app_math.h"
#include "pid.h"

struct PID_STRUCT roll_pid, pitch_pid, yaw_pid;

void pid_regular_init(void)
{
	memset(&roll_pid, 0 , sizeof(roll_pid));
	memset(&pitch_pid, 0 , sizeof(pitch_pid));
	memset(&yaw_pid, 0 , sizeof(yaw_pid));

	roll_pid.Kp = _roll_rate_kp;
	roll_pid.Ki = _roll_rate_ki;
	roll_pid.Kd = _roll_rate_kd;
	roll_pid.Ki_max = _roll_rate_ki_max;

	pitch_pid.Kp = _pitch_rate_kp;
	pitch_pid.Ki = _pitch_rate_ki;
	pitch_pid.Kd = _pitch_rate_kd;
	pitch_pid.Ki_max = _pitch_rate_ki_max;

	yaw_pid.Kp = _yaw_rate_kp;
	yaw_pid.Ki = _yaw_rate_ki;
	yaw_pid.Kd = _yaw_rate_kd;
	yaw_pid.Ki_max = _yaw_rate_ki_max;
}

float get_roll_stabilize_kp(s32 target_angle)
{
	return _roll_stabilize_kp*target_angle;
}
float get_pitch_stabilize_kp(s32 target_angle)
{
	return _pitch_stabilize_kp*target_angle;
}
float get_yaw_stabilize_kp(s32 target_angle)
{
	return _yaw_stabilize_kp*target_angle;
}

s16 pid_regular(s32 target,s32 fb,float control_period,
		struct PID_STRUCT *PID, u8 limit)
{
	s32 p, i, d, error, output;

	error = target - fb;
	p = error * PID->Kp;
	i = PID->integrator;

	if( !limit || ( (i > 0 && error < 0)||(i < 0 && error > 0))){
		if((PID->Ki != 0) && (control_period != 0)){
			PID->integrator += ((float)error*PID->Ki)*control_period;
			if(PID->integrator < -PID->Ki_max){
				PID->integrator = -PID->Ki_max;
			}else if(PID->integrator > PID->Ki_max){
				PID->integrator =PID->Ki_max;
			}
			i = PID->integrator;
		}else{
			i = 0;
		}
	}
	float derivative;
	if((PID->Kd != 0) && (control_period != 0)){
		if(isnan(PID->last_derivative)){
			derivative = 0;
			PID->last_derivative = 0;
		}else{
			derivative = (error - PID->last_error)/control_period;
		}
		//discrete low pass filter,cuts out the
		//high frequency noise that can drive the controller crazy
		derivative = PID->last_derivative + (control_period/(Kd_filter + control_period))*(derivative-PID->last_derivative);
		//update state
		PID->last_error = error;
		PID->last_derivative = derivative;
		d = (s32)(PID->Kd*derivative);
	}else{
		d = 0;
	}
	output = p + i + d;

	output = constrain_int32(output,-4500,4500);

	return (s16)output;
}
