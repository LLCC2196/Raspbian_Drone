/*
 * gps_ublox_uart.h
 *
 *  Created on: Nov 13, 2015
 *      Author: zz269
 */

#ifndef UART_H_
#define UART_H_

#include"my_types.h"
#define RP3_UART "/dev/ttyS0"

void uart_close(int *fd);
int  uart_open(int *fd);
int  uart_send_message(int *fd, void * buffer, u8 size);
int uart_read_message(int *fd, u8 * buffer, u8 size);

#endif /*UART_H_ */
