/*
 * serial.h
 *
 *  Created on: 18 Haz 2017
 *      Author: UGUR
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "stm32f0xx_hal.h"
#include <math.h>
#include <string.h>


typedef struct
{

	UART_HandleTypeDef *hserial;
	unsigned int BaudRate;

}serial_struct;


#define ASCII_CR		13
#define ASCII_LF		10
#define ASCII_SPACE		32


void sendData(unsigned char data);
void send_CR(void);
void send_LF(void);
void send_CRLF(void);
void send_SPACE(unsigned int number);
void send_int(int data);
void send_uint(unsigned int data);
/*void send_double(void, double data);
void send_float(void, float data);*/
void send_string(char* data);

#endif /* SERIAL_H_ */
