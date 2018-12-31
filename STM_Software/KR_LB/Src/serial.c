/*
 * serial.c
 *
 *  Created on: 18 Haz 2017
 *      Author: UGUR
 */

#include "serial.h"

extern UART_HandleTypeDef huart1;

void sendData(unsigned char data)
{
	HAL_UART_Transmit(&huart1, &data, 1, 0xFF);
}

void send_CR(void)
{
	sendData(ASCII_CR);
}

void send_LF(void)
{
	sendData(ASCII_LF);
}

void send_CRLF(void)
{
	send_CR();
	send_LF();
}

void send_SPACE(unsigned int number)
{
	unsigned int iterator;

	for(iterator = number; iterator > 0; iterator--)
		sendData(ASCII_SPACE);

}

void send_int(int data)
{
	unsigned char size,k;
	char c[8];

	sprintf(c,"%d",data);
	size = strlen(c);

	for(k=0; k<size; k++)
	{
		sendData(c[k]);
	}
}

void send_uint(unsigned int data)
{
	unsigned char size,k;
	char c[8];

	sprintf(c,"%u",data);
	size = strlen(c);

	for(k=0; k<size; k++)
	{
		sendData(c[k]);
	}
}


/*void send_double(void, double data)
{
	unsigned char size,k;
	char c[8];

	sprintf(c,"%.1f",data);
	size = strlen(c);

	for(k=0; k<size; k++)
	{
		sendData(sStruct.hserial, c[k]);
	}
}

void send_float(void, float data)
{
	send_double(sStruct, (double)data);
}*/

void send_string(char* data)
{
	unsigned char size,k;
	size = strlen(data);

	for(k=0; k<size; k++)
	{
		sendData(data[k]);
	}
}
