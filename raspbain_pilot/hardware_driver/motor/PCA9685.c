/*
 * PCA9685.c
 *
 *  Created on: Aug 4, 2016
 *      Author: zz269
 */

#include "PCA9685.h"
#include "motor_i2c.h"
#include"my_types.h"

char *PAC9685_dev = "/dev/i2c-1";
int PCA9685_i2c_handler =  0;
// Sets PCA9685 mode to 00
void PCA9685_reset() {
	motor_i2c_writeRegister(&PCA9685_i2c_handler, MODE1, 0x00); //Normal mode
	motor_i2c_writeRegister(&PCA9685_i2c_handler, MODE2, 0x04); //totem pole
}

/* Set the frequency of PWM
 *
 * freq desired frequency.
 * 40Hz to 1000Hz using internal 25MHz oscillator.
 */
void PCA9685_setPWMFreq(int freq) {
	u8 prescale_val = (CLOCK_FREQ / (4096 * freq)) + 0; //when set to 200Hz: + 2    //set to 400Hz -> + 0 -> 409.8Hz
	motor_i2c_writeRegister(&PCA9685_i2c_handler, MODE1, 0x10); //sleep
	motor_i2c_writeRegister(&PCA9685_i2c_handler, PRE_SCALE, prescale_val); // multiplyer for PWM frequency
	motor_i2c_writeRegister(&PCA9685_i2c_handler, MODE1, 0x80); //restart
	motor_i2c_writeRegister(&PCA9685_i2c_handler, MODE2, 0x04); //totem pole (default)
}

int PCA9685_init(void) {
	if(motor_i2c_open(&PCA9685_i2c_handler,PAC9685_dev,I2C2_SLAVE_ADDR)){
		perror("failed to open PAC9685_i2c device!");
		return -1;
	}
	PCA9685_reset();
	PCA9685_setPWMFreq(400);
	return 0;
}

/* PWM a single channel with custom on time
 * PWM channel to set PWM value for
 * on_value 0-4095 value to turn on the pulse
 * off_value 0-4095 value to turn off the pulse
 */
void setPWM(u8 pwm, int on_value, int off_value) {
	motor_i2c_writeRegister(&PCA9685_i2c_handler, PWM0_ON_L + PWM_MULTIPLYER * (pwm - 1), on_value & 0xFF);
	motor_i2c_writeRegister(&PCA9685_i2c_handler, PWM0_ON_H + PWM_MULTIPLYER * (pwm - 1), on_value >> 8);
	motor_i2c_writeRegister(&PCA9685_i2c_handler, PWM0_OFF_L + PWM_MULTIPLYER * (pwm - 1), off_value & 0xFF);
	motor_i2c_writeRegister(&PCA9685_i2c_handler, PWM0_OFF_H + PWM_MULTIPLYER * (pwm - 1), off_value >> 8);
}
/* PWM a single channel
 *
 * PWM channel to set PWM value for
 * value 0-4095 value for PWM
 */
void PCA9685_setPWM(u8 pwm, int value) {
	setPWM(pwm, 0, value);
}


