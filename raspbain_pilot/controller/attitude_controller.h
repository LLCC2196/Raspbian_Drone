/*
 * attitude_controller.h
 *
 *  Created on: Nov 14, 2015
 *      Author: zz269
 */

#ifndef ATTITUDE_CONTROLLER_H_
#define ATTITUDE_CONTROLLER_H_

#include"my_types.h"
#include<stdbool.h>

//throttle mode
#define RATE_THROTTLE_MANUAL 0
#define RATE_THROTTLE_HOLD 1
//stabilize mode
#define RATE_ROLL_PITCH_STABLE 0
#define RATE_ROLL_PITCH_LOITER 1
//yaw mode
#define RATE_YAW_HOLD 0

void attitude_controller_init(void);
void run_rate_controller(double control_period);

s16 get_angle_boost(s16 throttle);
void throttle_accel_active(void);
void throttle_accel_deactive(void);
void update_throttle_cruise(s16 throttle);
void set_throttle_out(s16 throttle_out,bool apply_angle_boost);

s16 get_rc_servo_out(u8 channel);

void update_rate_throttle_mode(void);
void update_roll_pitch_mode(void);
void update_yaw_mode(double control_period);
void get_stabilize_roll(s32 target_angle);
void get_stabilize_pitch(s32 target_angle);
void get_stabilize_yaw(s32 target_angle,double control_period);
void update_rate_controller_target(void);

void set_alt_controller_output(s16 output);
s16 get_throttle_cruise();
void set_loiter_controller_output_roll_pitch(s32 roll, s32 pitch);

void set_rate_roll_pitch_mode(u8 rp_mode);
void set_rate_yaw_mode(u8 yaw_mode);
void set_rate_throttle_mode(u8 throttle_mode);
u8 get_rate_throttle_mode(void);

#endif /* ATTITUDE_CONTROLLER_H_ */
