/*
 * rc_channel.c
 *
 *  Created on: Nov 14, 2015
 *      Author: zz269
 */

#include "rc_channel.h"

#include"rc_Interface.h"
#include"app_math.h"
#include<error.h>
#include<stdio.h>
#include "pid.h"


s16 rc_control_in[7];

//roll target degrees (-4500 to 4500)
//pwm到degree*100的转换
s16 roll_PWM_to_angle(u16 pwm_in,s16 dead_zone){
	s16 r_in = constrain_uint16(pwm_in,rc1_radio_min_get(),rc1_radio_max_get());
	s16 roll_low_trim = rc1_radio_mid - dead_zone;
	s16 roll_high_trim = rc1_radio_mid + dead_zone;

	s16 output;
	if ((r_in > roll_low_trim) && (r_in < roll_high_trim)){
		output = 0;
	}else{
		output = (r_in-rc1_radio_min_get())*2*ROLL_PITCH_YAW_RANGE/(rc1_radio_max_get()-rc1_radio_min_get())-ROLL_PITCH_YAW_RANGE;
	}

	return output;
}
// pitch target degrees (-4500 to 4500)
//pwm到degree*100的转换
s16 pitch_PWM_to_angle(u16 pwm_in,s16 dead_zone){
	s16 r_in = constrain_uint16(pwm_in,rc2_radio_min_get(),rc2_radio_max_get());
	s16 roll_low_trim = rc2_radio_mid - dead_zone;
	s16 roll_high_trim = rc2_radio_mid + dead_zone;

	s16 output;
	if((r_in > roll_low_trim) && (r_in < roll_high_trim)){
		output = 0;
	}else{
		output = (r_in-rc2_radio_min_get())*2*ROLL_PITCH_YAW_RANGE/(rc2_radio_max_get()-rc2_radio_min_get())-ROLL_PITCH_YAW_RANGE;
	}
	return output;
}
// yaw target degrees (-4500 to 4500)
//pwm到degree*100的转换
s16 yaw_PWM_to_angle(u16 pwm_in,s16 dead_zone){
	s16 r_in = constrain_uint16(pwm_in,rc4_radio_min_get(),rc4_radio_max_get());
	s16 roll_low_trim = rc4_radio_mid - dead_zone;
	s16 roll_high_trim = rc4_radio_mid + dead_zone;

	s16 output;
	if((r_in > roll_low_trim) && (r_in < roll_high_trim)){
		output = 0;
	}else{
		output = (r_in-rc4_radio_min_get())*2*ROLL_PITCH_YAW_RANGE/(rc4_radio_max_get()-rc4_radio_min_get())-ROLL_PITCH_YAW_RANGE;
	}
	return output;
}
//throttle target (0 to 1000)
s16 throttle_PWM_to_range(u16 pwm_in)
{
	s16 r_in = constrain_uint16(pwm_in,rc3_radio_min_get(),rc3_radio_max_get());

	int output = (r_in-rc3_radio_min_get())*THROTTLE_RANGE/(rc3_radio_max_get()-rc3_radio_min_get());

	return output;
}
//遥控信号转换
void rc_channel_pwm_convert(void)
{
	rc_control_in[0] = roll_PWM_to_angle(interface_rc_receiver_get_rc_value(0),_dead_zone);
	rc_control_in[1] = pitch_PWM_to_angle(interface_rc_receiver_get_rc_value(1),_dead_zone);
	rc_control_in[2] = throttle_PWM_to_range(interface_rc_receiver_get_rc_value(2));
	rc_control_in[3] = yaw_PWM_to_angle(interface_rc_receiver_get_rc_value(3),_dead_zone);
	rc_control_in[4] = interface_rc_receiver_get_rc_value(4);
	rc_control_in[5] = interface_rc_receiver_get_rc_value(5);
	rc_control_in[6] = interface_rc_receiver_get_rc_value(6);

//	printf("%d\t%d\n",interface_rc_receiver_get_rc_value(0),rc_control_in[0]);
}

s16 rc_channel_get_control_in(u8 channel)
{
	if(channel>=0 && channel <=6){
		return rc_control_in[channel];
	}else{
		perror("illegal control_in channel request.\r\n");
		return 0;
	}

}
