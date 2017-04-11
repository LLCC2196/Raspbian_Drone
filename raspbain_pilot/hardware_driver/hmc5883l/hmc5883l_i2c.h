/*
 * I2CDevice.h
 *
 *  Created on: Oct 30, 2015
 *      Author: zz269
 */

#ifndef I2CDEVICE_H_
#define I2CDEVICE_H_

#define HMC5883L_I2C_RP3 "/dev/i2c-1"
/* Private Defines -----------------------------------------------------*/
#define	HMC5883L_CONFIG_A		0x00
#define	HMC5883L_CONFIG_B		0x01
#define	HMC5883L_MODE			0x02
#define	HMC5883L_X_MSB			0x03
#define	HMC5883L_X_LSB			0x04
#define	HMC5883L_Z_MSB			0x05
#define	HMC5883L_Z_LSB			0x06
#define	HMC5883L_Y_MSB			0x07
#define	HMC5883L_Y_LSB			0x08
#define	HMC5883L_STATUS			0x09
#define	HMC5883L_IDEN_REG_A		0x10
#define	HMC5883L_IDEN_REG_B		0x11
#define	HMC5883L_IDEN_REG_C		0x12
#define HMC5883L_I2C_ADDRESS    0x1e

#define magGain              	0x20
#define NormalOperation      	0x10
#define PositiveBiasConfig   	0x11
#define NegativeBiasConfig   	0x12
#define ContinuousConversion 	0x00
#define SingleConversion     	0x01

// ConfigRegA valid sample averaging for 5883L
#define SampleAveraging_1   	0x00
#define SampleAveraging_2    	0x01
#define SampleAveraging_4    	0x02
#define SampleAveraging_8    	0x03

// ConfigRegA valid data output rates for 5883L
#define DataOutputRate_0_75HZ 	0x00
#define DataOutputRate_1_5HZ  	0x01
#define DataOutputRate_3HZ    	0x02
#define DataOutputRate_7_5HZ  	0x03
#define DataOutputRate_15HZ   	0x04
#define DataOutputRate_30HZ   	0x05
#define DataOutputRate_75HZ   	0x06

int  hmc5883l_i2c_open(int *fd, char *dev, char addr);
void hmc5883l_i2c_close(int *fd);
int	 hmc5883l_i2c_write(int *fd,unsigned char value);
int  hmc5883l_i2c_readRegister(int *fd,unsigned int registerAddress,unsigned char *registerValue);
int  hmc5883l_i2c_writeRegister(int *fd,unsigned int registerAddress, unsigned char value);
int  hmc5883l_i2c_read_num_Registers(int *fd,unsigned int fromAddress, unsigned int number, unsigned char *registerValue);


#endif /* I2CDEVICE_H_ */
