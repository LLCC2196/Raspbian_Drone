/*
 * motor.h
 *
 *  Created on: Nov 14, 2015
 *      Author: zz269
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include"my_types.h"

struct AP_Motors_limit{
	u8 roll_pitch;// we have reached roll or pitch limit
	u8 yaw       ;// we have reached yaw limit
	u8 throttle_lower;//we have reached throttle's lower limit
	u8 throttle_upper;//we have reached throttle's upper limit
};

extern struct AP_Motors_limit motor_limit;

void init_motor(void);
void motor_output_armed(u16 *motor_output);
struct AP_Motors_limit *get_motor_limit(void);

#endif /* MOTOR_H_ */
