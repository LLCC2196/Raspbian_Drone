/*
 * rc_channel.h
 *
 *  Created on: Nov 14, 2015
 *      Author: zz269
 */

#ifndef RC_CHANNEL_H_
#define RC_CHANNEL_H_

#include"my_types.h"

#define _dead_zone 5
//roll
#define rc1_radio_min_get() 1010
#define rc1_radio_mid 1519
#define rc1_radio_max_get() 2020
//pitch
#define rc2_radio_min_get() 1009
#define rc2_radio_mid 1518
#define rc2_radio_max_get()2020
//throttle
#define rc3_radio_min_get()990
#define rc3_radio_mid_get() 1530
#define rc3_radio_max_get() 2020
//yaw
#define rc4_radio_min_get() 1009
#define rc4_radio_mid 1511
#define rc4_radio_max_get() 2031

//角度范围
#define ROLL_PITCH_INPUT_MAX 4500
#define ROLL_PITCH_YAW_RANGE 4500
#define MAX_LEAN_ANGLE 4500
//油门范围
#define THROTTLE_RANGE 1000
//速率限幅
#define ANGLE_RATE_MAX 18000
#define MAX_YAW_OVERSHOOT 1000

typedef struct{
	u16 rc1_channel_radio;
	u16 rc2_channel_radio;
	u16 rc3_channel_radio;
	u16 rc4_channel_radio;
	u16 rc5_channel_radio;
	u16 rc6_channel_radio;
}RCCHANNEL;

void rc_channel_pwm_convert(void);
s16 rc_channel_get_control_in(u8 channel);

#endif /* RC_CHANNEL_H_ */
