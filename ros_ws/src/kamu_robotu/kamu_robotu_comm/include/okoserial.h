/*
 * serial.h
 *
 *  Created on: Apr 9, 2019
 *      Author: oguz
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef OKOSERIAL_H_
#define OKOSERIAL_H_




#include "stdint.h"
#include "okoserial.h"
#include "string.h"
#include <stdbool.h>

#define BLUETOOTH_HANDLER 		&huart1
#define RASPI_SERIAL_HANDLER	&huart3

extern uint8_t Bluetooth_Data_Buffer[32];
extern uint8_t data_buffer[20];
extern volatile bool newBluetoothDataArrived;

enum bluetooth_command_types
{
	Forward_run_cmd = 	0x01,
	Backward_run_cmd = 	0x02,
	Right_run_cmd = 	0x03,
	Left_run_cmd = 		0x04,
	Encoder_cmd = 		0x05,
	LED_Toggle_cmd =	0x06,
	Kp_cmd = 			0x07,
	Kd_cmd = 			0x08,
	Ki_cmd = 			0x09,
	Step_Freq_cmd =		0x0A
};

enum message_types
{
	ping,
	laserscan,
	odometry,
	velocitycmd,
	pidparams
};
enum message_lengths
{
	laserscan_length = 128, // 64 integer
	odometry_length = 20, // 5 float each one 4 byte
	velocitycmd_length = 8, // 2 float each one 4 byte
	pidparams_length = 12, // 3 float each one 4 byte
	prelim_length = 8
};
enum error_codes
{
	none
};



enum serial_state
{
	waiting_heading,
	heading1_came,
	heading2_came,
	heading3_came,
	datatype_came,
	errorcode_came,
	extra_info_1_came,
	extra_info_2_came,
	extra_info_3_came,
	laserscan_coming,
	odometry_coming,
	velocitycmd_coming,
	pidparams_coming,
	data_came
};

typedef struct
{
	uint8_t heading[3];
	uint8_t message_type;
	uint8_t error_code;
	uint8_t extra_info_1;
	uint8_t extra_info_2;
	uint8_t extra_info_3;
	
} preliminary_message;

typedef struct
{
	preliminary_message premess;
	uint16_t data[laserscan_length];
}laserscan_message;

typedef struct
{
	preliminary_message premess;
	float x;
	float y;
	float theta;
	float vx;
	float w;
}odometry_message;

typedef struct
{
	preliminary_message premess;
	float vx; // linear velocity
	float w;  // angular velocity
}velocitycmd_message;

typedef struct
{
	preliminary_message premess;
	float kp;
	float ki;
	float kd;
}pidparams_message;

extern preliminary_message prelim_mess;
extern velocitycmd_message vel_cmd_mess;
extern pidparams_message pidparam_mess;
extern odometry_message odom_mess;
extern laserscan_message laserscan_mess;

void getSingleByte(uint8_t  byte);
void initializeOdometryMessage(odometry_message * odom);
void initializeVelocitycmdMessage(velocitycmd_message * vel_cmd_mess);
void sendVelocitycmd(float *vx, float *w);



#endif /* OKOSERIAL_H_ */

#ifdef __cplusplus
 }
#endif
