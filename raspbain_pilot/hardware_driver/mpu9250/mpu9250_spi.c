/*
 * mpu9250_spi.c
 *
 *  Created on: Nov 6, 2015
 *      Author: zz269
 */

#include"my_types.h"
#include"mpu9250_spi.h"


#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <linux/spi/spidev.h>
#include <string.h>

struct mpu9250_spi_property mpu9250_property;

int mpu9250_spi_setMode(struct mpu9250_spi_property *mpu9250_property);
int mpu9250_spi_setSpeed(struct mpu9250_spi_property *mpu9250_property);
int mpu9250_spi_setBitsPerWord(struct mpu9250_spi_property *mpu9250_property);
int mpu9250_spi_transfer(struct mpu9250_spi_property *mpu9250_property,unsigned char send[], unsigned char receive[], int length);


/**
 * This method opens the file connection to the SPI device.
 * @param device_path: The SPI Device Path
 * @return 0 on a successful open of the file
 */
int mpu9250_spi_open(struct mpu9250_spi_property *mpu9250_property){
	if((mpu9250_property->fd = open(mpu9250_property->dev,O_RDWR)) < 0 ){
		perror("SPI Error : Can't open device.");
		return -1;
	}
	//set spi mode
	if(mpu9250_spi_setMode(mpu9250_property) < 0) {
		perror("SPI Error : set mode error");
		return -1;
	}
	//set spi speed
	if(mpu9250_spi_setSpeed(mpu9250_property) < 0){
		perror("SPI Error : set speed error");
		return -1;
	}
	//set spi bits
	if(mpu9250_spi_setBitsPerWord(mpu9250_property) < 0){
		perror("SPI Error : set bits error");
		return -1;
	}
	return 0;
}

/**
 *   Close the SPI device
 *   @param device_id: The Opening SPI Device Handle
 */
void mpu9250_spi_close(struct mpu9250_spi_property *mpu9250_property){
	close(mpu9250_property->fd);
	mpu9250_property->fd = -1;
}

/**
 * Set the mode of the SPI bus
 * @param The Opening SPI Device Handle
 * @param mode the enumerated SPI mode
 * @return 0 if successful
 */
int mpu9250_spi_setMode(struct mpu9250_spi_property *mpu9250_property){
	if(ioctl(mpu9250_property->fd,SPI_IOC_WR_MODE,&mpu9250_property->mode) < 0){
		perror("SPI Error: Can't set SPI mode.");
		return -1;
	}
	if(ioctl(mpu9250_property->fd,SPI_IOC_RD_MODE,&mpu9250_property->mode) < 0){
		perror("SPI Error: Can't get SPI mode.");
		return -1;
	}
	return 0;
}

/**
 * Set the speed of the SPI bus
 * @param The Opening SPI Device handle
 * @param speed the speed in Hz
 */
int mpu9250_spi_setSpeed(struct mpu9250_spi_property *mpu9250_property){
	if(ioctl(mpu9250_property->fd,SPI_IOC_WR_MAX_SPEED_HZ,&mpu9250_property->speed) < 0){
		perror("SPI Error: Can't set SPI speed Hz.");
		return -1;
	}
	if(ioctl(mpu9250_property->fd,SPI_IOC_RD_MAX_SPEED_HZ,&mpu9250_property->speed) < 0){
		perror("SPI Error: Can't get SPI speed Hz.");
		return -1;
	}
	return 0;
}

/**
 * Set the number of bits per word of the SPI bus
 * @param The Opening SPI Device handle
 * @param bits the number of bits per word
 */
int mpu9250_spi_setBitsPerWord(struct mpu9250_spi_property *mpu9250_property){
	if(ioctl(mpu9250_property->fd,SPI_IOC_WR_BITS_PER_WORD,&mpu9250_property->bits) < 0){
		perror("SPI Error: Can't set bits per word.");
		return -1;
	}
	if(ioctl(mpu9250_property->fd,SPI_IOC_RD_BITS_PER_WORD,&mpu9250_property->bits) < 0){
		perror("SPI Error: Can't get bits per word.");
		return -1;
	}
	return 0;
}
/**
 * Generic method to transfer data to and from the SPI device. It is used by the
 * following methods to read and write registers.
 * @param send The array of data to send to the SPI device
 * @param receive The array of data to receive from the SPI device
 * @param length The length of the array to send
 * @return -1 on failure
 */
int mpu9250_spi_transfer(struct mpu9250_spi_property *mpu9250_property,unsigned char send[], unsigned char receive[], int length){
	struct spi_ioc_transfer	transfer;
	memset(&transfer, 0, sizeof (transfer));
	transfer.tx_buf = (unsigned long) send;
	transfer.rx_buf = (unsigned long) receive;
	transfer.len = length;
	transfer.speed_hz = mpu9250_property->speed;
	transfer.bits_per_word = mpu9250_property->bits;
	transfer.delay_usecs = 0;
	if (ioctl(mpu9250_property->fd, SPI_IOC_MESSAGE(1), &transfer) < 0) {
		perror("SPI: SPI_IOC_MESSAGE Failed");
		return -1;
	}
	return 0;
}
/**
 *  Writes a value to a defined register address (check the datasheet for the device)
 *  @param registerAddress the address of the register to write to
 *  @param value the value to write to the register
 *  @return returns 0 if successful
 */
int mpu9250_spi_writeRegister(struct mpu9250_spi_property *mpu9250_property,unsigned int registerAddress, unsigned char value){
	unsigned char send[2], receive[2];
	memset(receive, 0, sizeof receive);
	send[0] = (unsigned char) registerAddress;
	send[1] = value;
	if(mpu9250_spi_transfer(mpu9250_property, send, receive, 2) < 0){
		perror("failed to write mpu9250_spi register");
		return -1;
	}
	return 0;
}
/**
 * A method to read a single register at the SPI address
 * @param registerAddress the address of the register from the device datasheet
 * @return the character that is returned from the address
 */
int mpu9250_spi_readRegister(struct mpu9250_spi_property *mpu9250_property, unsigned int registerAddress, unsigned char *registerValue){
	unsigned char send[2], receive[2];
	memset(send, 0, sizeof send);
	memset(receive, 0, sizeof receive);
	send[0] = (unsigned char) (0x80 | registerAddress);
	if(mpu9250_spi_transfer(mpu9250_property, send, receive, 2) < 0){
		perror("failed to read mpu9250_spi register");
		return -1;
	}
	*registerValue = receive[1];
	return 0;
}
/**
 * A method to read a number of registers as a data array
 * @param buffer to store the read data
 * @param number the number of registers to read
 * @param fromAddress the starting address of the block of data
 */
int mpu9250_spi_readRegisters(struct mpu9250_spi_property *mpu9250_property, unsigned int number, unsigned int fromAddress, unsigned char *registerValue){
	unsigned char send[number+1], receive[number+1];
	memset(send, 0, sizeof send);
	send[0] =  (unsigned char) (0x80 | fromAddress);
	if(mpu9250_spi_transfer(mpu9250_property, send, receive, number+1) < 0){
		perror("failed to read mpu9250_spi registers");
		return -1;
	}
	memcpy(registerValue, receive+1, number);
	return 0;
}

