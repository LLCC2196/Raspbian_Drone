/*
 * PCA9685.h
 *
 *  Created on: Aug 4, 2016
 *      Author: zz269
 */

#ifndef PCA9685_H_
#define PCA9685_H_

#include "motor_i2c.h"
#include "my_types.h"

#define MODE1 	0x00			//Mode  register  1
#define MODE2 	0x01			//Mode  register  2
#define SUBADR1 0x02			//I2C-bus subaddress 1
#define SUBADR2 0x03			//I2C-bus subaddress 2
#define SUBADR3 0x04			//I2C-bus subaddress 3
#define ALLCALLADR 	0x05     	//PWM All Call I2C-bus address
#define PWM0 		0x6			//PWM0 start register
#define PWM0_ON_L 	0x6			//PWM0 output and brightness control byte 0
#define PWM0_ON_H 	0x7			//PWM0 output and brightness control byte 1
#define PWM0_OFF_L 	0x8			//PWM0 output and brightness control byte 2
#define PWM0_OFF_H 	0x9			//PWM0 output and brightness control byte 3
#define PWM_MULTIPLYER 	4		// For the other 15 channels
#define ALLPWM_ON_L 	0xFA    	//load all the PWMn_ON registers, byte 0 (turn 0-7 channels on)
#define ALLPWM_ON_H 	0xFB		//load all the PWMn_ON registers, byte 1 (turn 8-15 channels on)
#define ALLPWM_OFF_L 	0xFC		//load all the PWMn_OFF registers, byte 0 (turn 0-7 channels off)
#define ALLPWM_OFF_H 	0xFD		//load all the PWMn_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALE 		0xFE		//prescaler for output frequency
#define CLOCK_FREQ 		25000000.0 	//25MHz default osc clock

int PCA9685_init(void);
void PCA9685_setPWM(u8 pwm, int value);

#endif /* SYS_DEV_HARDWARE_PCA9685_PCA9685_H_ */
