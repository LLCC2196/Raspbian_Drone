/*
 * gps_ublox_uart.c
 *
 *  Created on: Nov 13, 2015
 *      Author: zz269
 */

#include<stdio.h>
#include<fcntl.h>
#include<unistd.h>
#include<termios.h>
#include<string.h>
#include<sys/time.h>

#include"my_types.h"
#include"uart.h"

int uart_send_message( int *fd,void * buffer, u8 size)
{
	if(write(*fd,(u8 *)buffer, size) != size){
		perror("failed to write uart");
		return-1;
	}
	return 0;
}

int uart_read_message(int *fd,u8 * buffer, u8 size)
{
	return read(*fd, buffer, size);
}

int uart_open(int *fd)
{
	if (( *fd = open(RP3_UART, O_RDWR | O_NOCTTY | O_NDELAY)) < 0){
		perror("failed to open the ttyS0.\n");
		return -1;
	}
    if (fcntl(*fd, F_SETFL, O_NONBLOCK) < 0){
        perror("Unable set to NONBLOCK mode");
        return -2;
    }
	// Set up the communications options:
	// 38400 baud, 8-bit, enable receiver, no modem control lines
	struct termios options;
	tcgetattr(*fd, &options);
	options.c_cflag = B38400 | CS8 | CREAD | CLOCAL;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(*fd, TCIFLUSH);
	if(tcsetattr(*fd, TCSANOW, &options) < 0){
		perror("failed to setup ttyS0");
		return -3;
	}
	return 0;
}

void uart_close(int *fd)
{
	close( *fd);
	*fd = -1;
}




