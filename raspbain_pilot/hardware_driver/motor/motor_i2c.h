/*
 * motor_i2c.h
 *
 *  Created on: Aug 16, 2016
 *      Author: llcc2196
 */

#ifndef MOTOR_I2C_H_
#define MOTOR_I2C_H_

#define MOTOR_I2C_RP3 "/dev/i2c-1"
/* Private Defines -----------------------------------------------------*/
#define I2C2_BASE_ADD		0x4819c000u
#define I2C2_OWN_ADDR		0xA0
#define I2C2_SLAVE_ADDR		0x40

#define I2C2_RESET_VALUE	2
#define I2C2_PSC_VALUE		0
#define I2C2_SCLL_VALUE		54
#define I2C2_SCLH_VALUE		54
#define I2C2_CON_RESET 		0x0000
#define I2C2_CON_ENABLE		0x8000
#define I2C2_CON_MS_SEND	0x8603
#define I2C2_CON_MS_RECEIVE	0x8403
#define I2C2_TIMEOUT		2000

//#define I2C_SYSC 			0x10
#define I2C2_IRQSTATUS_RAW	0x24
#define I2C2_IRQSTATUS		0x28
#define I2C2_CNT			0x98
#define	I2C2_DATA			0x9C
#define	I2C2_CON			0xA4
#define	I2C2_OA				0xA8
#define	I2C2_SA				0xAC
#define	I2C2_PSC			0xB0
#define	I2C2_SCLL			0xB4
#define	I2C2_SCLH			0xB8

int  motor_i2c_open(int *fd, char *dev, char addr);
void motor_i2c_close(int *fd);
int	 motor_i2c_write(int *fd,unsigned char value);
int  motor_i2c_readRegister(int *fd,unsigned int registerAddress,unsigned char *registerValue);
int  motor_i2c_writeRegister(int *fd,unsigned int registerAddress, unsigned char value);
int  motor_i2c_read_num_Registers(int *fd,unsigned int fromAddress, unsigned int number, unsigned char *registerValue);


#endif /* I2CDEVICE_H_ */
