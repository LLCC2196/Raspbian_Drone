/*
 * motor_i2c.c
 *
 *  Created on: Aug 16, 2016
 *      Author: llcc2196
 */

#include<fcntl.h>
#include<stdio.h>
#include<string.h>
#include<unistd.h>
#include<sys/ioctl.h>
#include<linux/i2c.h>
#include<linux/i2c-dev.h>

#include"motor_i2c.h"

/**
 * Open a i2c device.
 * @param fd   The file handler
 * @param dev  The device that need to be open.
 * @param addr The device's address
 * @return -1 on failure to open the device, 0 on success.
 */
int motor_i2c_open(int *fd, char *dev, char addr)
{
	if(!strcmp(dev,MOTOR_I2C_RP3)){
		*fd = open(dev, O_RDWR);
		if(*fd <= 0){
			perror("failed to open the I2C bus\n");
			return -1;
		}
		if(ioctl(*fd, I2C_SLAVE, addr) < 0){
			perror("failed to open the I2C Device\n");
			return -1;
		}
		return 0;
	}
	return -1;
}
/**
 * Close a hmc5883l_i2c device.
 * @param  fd The file handler
 */
void motor_i2c_close(int *fd)
{
	close(*fd);
	*fd = -1;
}
/**
 * Write a single value to the hmc5883l_i2c device.
 * Used to set up the device to read from a particular address.
 * @param fd    The file handler
 * @param value The value to write to the device
 * @return -1 on failure to write, 0 on success.
 */
int motor_i2c_write(int *fd,unsigned char value)
{
   if (write(*fd, &value, 1)!=1){
	  perror("I2C: Failed to write to the device\n");
	  return -1;
   }
   return 0;
}
/**
 * Read a single value from the hmc5883l_i2c device.
 * Used to set up the device to read from a particular address.
 * @param fd    The file handler
 * @param registerAddress The register to read from the device
 * @return the byte value at the register address.
 */
int  motor_i2c_readRegister(int *fd,unsigned int registerAddress,unsigned char *registerValue)
{
	unsigned char buffer[1];
	motor_i2c_write(fd,registerAddress);
	if(read(*fd, buffer, 1)!=1){
	  perror("I2C: Failed to read in the value.\n");
	  return -1;
	}
	*registerValue = buffer[0];
	return 0;
}
/**
 * Read a single value from the hmc5883l_i2c device.
 * Used to set up the device to read from a particular address.
 * @param fd    The file handler
 * @param number the number of registers to read
 * @param fromAddress The start register's address to read from the device
 * @return the byte value at the register address.
 */
int  motor_i2c_read_num_Registers(int *fd,unsigned int fromAddress,unsigned int number,unsigned char *registerValue)
{
	motor_i2c_write(fd,fromAddress);
	unsigned char data[number];
    if(read(*fd, data, number)!=(int)number){
       perror("IC2: Failed to read in the full buffer.\n");
	   return -1;
    }
    memcpy(registerValue,data,number);
	return 0;
}
/**
 * Write a single byte value to a single register.
 * @param registerAddress The register address
 * @param value The value to be written to the register
 * @return -1 on failure to write, 0 on success.
 */
int  motor_i2c_writeRegister(int *fd,unsigned int registerAddress, unsigned char value)
{
	unsigned char buffer[2];
	buffer[0] = registerAddress;
	buffer[1] = value;
	if(write(*fd, buffer, 2)!=2){
	  perror("I2C: Failed write to the motor device\n");//************88
	  return -1;
	}
	return 0;
}


