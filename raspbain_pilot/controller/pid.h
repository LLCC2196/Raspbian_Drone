/*
 * pid.h
 *
 *  Created on: Nov 14, 2015
 *      Author: zz269
 */

#ifndef PID_H_
#define PID_H_

#include"my_types.h"

struct PID_STRUCT{
	float Kp;
	float Ki;
	float Kd;
	float Ki_max;
	float integrator;
	float last_derivative;
	float last_error;
};

extern struct PID_STRUCT roll_pid, pitch_pid, yaw_pid;

#define Kd_filter 0.00795774715f //1/(2*PI*_fcut)

#define throttle_min 130
#define throttle_max 1000
#define throttle_deadzone 20

#define _roll_stabilize_kp 4.5
#define _pitch_stabilize_kp 4.5
#define _yaw_stabilize_kp 4.5

////rate pid parameters
////7ms
//#define _roll_rate_kp 0.035//0.045
//#define _roll_rate_ki 0.01//0.0015
//#define _roll_rate_kd 0//0.0005
//#define _roll_rate_ki_max 500
//#define _pitch_rate_kp 0.035//0.045
//#define _pitch_rate_ki 0.01//0.0015
//#define _pitch_rate_kd 0//0.0005
//#define _pitch_rate_ki_max 500
//#define _yaw_rate_kp 0.115//0.105
//#define _yaw_rate_ki 0//0.001
//#define _yaw_rate_kd 0
//#define _yaw_rate_ki_max 8

//5ms RC offset -40 -80 -40
#define _roll_rate_kp 0.030//0.045
#define _roll_rate_ki 0.005//0.0015
#define _roll_rate_kd 0//0.0005
#define _roll_rate_ki_max 500
#define _pitch_rate_kp 0.030//0.045
#define _pitch_rate_ki 0.005//0.0015
#define _pitch_rate_kd 0//0.0005
#define _pitch_rate_ki_max 500
#define _yaw_rate_kp 0.105//0.105
#define _yaw_rate_ki 0//0.001
#define _yaw_rate_kd 0
#define _yaw_rate_ki_max 8
//filter
#define _fcut 20
//rad to deg*100
#define DEGX100 5729.57795f

void pid_regular_init(void);
//angle pid
float get_roll_stabilize_kp(s32 target_angle);
float get_pitch_stabilize_kp(s32 target_angle);
float get_yaw_stabilize_kp(s32 target_angle);
//rate pid
s16 pid_regular(s32 target,s32 fb,float control_period,
		struct PID_STRUCT *PID, u8 limit);
#endif /* PID_H_ */
