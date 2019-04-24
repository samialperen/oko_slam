/*
 * serial.c
 *
 *  Created on: Apr 9, 2019
 *      Author: oguz
 */

#include "okoserial.h"
#include <stdio.h>



uint8_t integer_buffer[2];
uint8_t float_buffer[4];
uint8_t data_buffer[20];
uint8_t state = waiting_heading;
preliminary_message prelim_mess;
velocitycmd_message vel_cmd_mess;
pidparams_message pidparam_mess;
odometry_message odom_mess;
laserscan_message laserscan_mess;


uint8_t data_index=0;
uint8_t k=0;

uint8_t Bluetooth_Data_Buffer[32];
volatile bool newBluetoothDataArrived = false;

#ifdef __cplusplus
 extern "C" {
#endif





void getSingleByte(uint8_t  byte)
{
	
	switch(state){
		case waiting_heading:
			prelim_mess.heading[0] = byte;
			if(prelim_mess.heading[0]==0xFF)
			{
				state = heading1_came;
			}
			else state = waiting_heading;
			break;
		case heading1_came:
			prelim_mess.heading[1] = byte;
			if(prelim_mess.heading[1]==0xFF)
			{
				state = heading2_came;
			}
			else state = waiting_heading;
			break;
		case heading2_came:
			prelim_mess.heading[2] = byte;
			if(prelim_mess.heading[2]==0xFD)
			{
				state = heading3_came;
			}
			else state = waiting_heading;
			break;
		case heading3_came:
			prelim_mess.message_type = byte;
			if(prelim_mess.message_type==velocitycmd)
			{
				state = datatype_came;

			}
			else if(prelim_mess.message_type==pidparams)
			{
				state = datatype_came;

			}
			
			else if(prelim_mess.message_type==laserscan)
			{
				state = datatype_came;

			}
			else if(prelim_mess.message_type==odometry)
			{
				state = datatype_came;

			}
			else if(prelim_mess.message_type==ping)
			{
				state = datatype_came;

			}

			else state = waiting_heading;
			break;
		case datatype_came: 
			prelim_mess.error_code = byte;
			if(prelim_mess.error_code==none)
			{
				state = errorcode_came;
			}
			else state = waiting_heading;
			break;
		case errorcode_came:
			prelim_mess.extra_info_1 = byte;
			if(prelim_mess.extra_info_1 == none)
			{
				state = extra_info_1_came;
			}
			else state = waiting_heading;
			break;
		case extra_info_1_came:
			prelim_mess.extra_info_2 = byte;
			if(prelim_mess.extra_info_2 == none)
			{
				state = extra_info_2_came;
			}
			else state = waiting_heading;
			break;
		case extra_info_2_came:
			prelim_mess.extra_info_3 = byte;
			if(prelim_mess.extra_info_3 == none)
			{
				if(prelim_mess.message_type==velocitycmd)
				{
					state = velocitycmd_coming;
					memcpy(&(vel_cmd_mess.premess),&prelim_mess,sizeof(prelim_mess));
				}
				else if(prelim_mess.message_type==pidparams)
				{
					state = pidparams_coming;
					memcpy(&(pidparam_mess.premess),&prelim_mess,sizeof(prelim_mess));
				}
				else if(prelim_mess.message_type==laserscan) 
				{
					state = laserscan_coming;
					memcpy(&(laserscan_mess.premess),&prelim_mess,sizeof(prelim_mess));
				}
				else if(prelim_mess.message_type==odometry)
				{
					state = odometry_coming;
					memcpy(&(odom_mess.premess),&prelim_mess,sizeof(prelim_mess));
				}
				else if(prelim_mess.message_type==ping)
				{
					state = waiting_heading;
				//	memcpy(&(ping_mess.premess),&prelim_mess,sizeof(prelim_mess));
				}
				else state = waiting_heading;
			}
			else state = waiting_heading;
			break;
		case velocitycmd_coming:
			if(data_index <3)
			{
				float_buffer[data_index++] = byte;
			}
			else if(data_index==3)
			{
				float_buffer[data_index++] = byte;
				memcpy(&(vel_cmd_mess.vx),float_buffer,4);
			}
			else if(data_index<7)
			{
				float_buffer[(data_index++)-4] = byte;
			}
			else if(data_index == 7)
			{
				float_buffer[(data_index++)-4] = byte;
				memcpy(&(vel_cmd_mess.w),float_buffer,4);
				data_index = 0;
				state = data_came;
				state = waiting_heading; // omit xor for now
			}
			break;
		case pidparams_coming:
			if(data_index <3)
			{
				float_buffer[data_index++] = byte;
			}
			else if(data_index==3)
			{
				float_buffer[data_index++] = byte;
				memcpy(&(pidparam_mess.kp),float_buffer,4);
			}
			else if(data_index<7)
			{
				float_buffer[(data_index++)-4] = byte;
			}
			else if(data_index == 7)
			{
				float_buffer[(data_index++)-4] = byte;
				memcpy(&(pidparam_mess.ki),float_buffer,4);
			}
			else if(data_index<11)
			{
				float_buffer[(data_index++)-8] = byte;
			}
			else if(data_index == 11)
			{
				float_buffer[(data_index++)-8] = byte;
				memcpy(&(pidparam_mess.kd),float_buffer,4);
				data_index = 0;
				state = data_came;
				state = waiting_heading; // omit xor for now
			}
			break;
		case odometry_coming:
		
			if(data_index <3)
			{
				float_buffer[data_index++] = byte;
			}
			else if(data_index==3)
			{
				float_buffer[data_index++] = byte;
				memcpy(&(odom_mess.x),float_buffer,4);
				printf("odom_mess.x :%f , %d \n " , odom_mess.x , state);
			}
			else if(data_index <7)
			{
				float_buffer[(data_index++)-4] = byte;
			}
			else if(data_index==7)
			{
				float_buffer[(data_index++)-4] = byte;
				memcpy(&(odom_mess.y),float_buffer,4);
				printf("odom_mess.y :%f , %d  \n" , odom_mess.y , state);
			}
			else if(data_index <11)
			{
				float_buffer[(data_index++)-8] = byte;
			}
			else if(data_index==11)
			{
				float_buffer[(data_index++)-8] = byte;
				memcpy(&(odom_mess.theta),float_buffer,4);
				printf("odom_mess.theta :%f , %d \n " , odom_mess.theta , state);
			}
			else if(data_index <15)
			{
				float_buffer[(data_index++)-12] = byte;
			}
			else if(data_index==15)
			{
				float_buffer[(data_index++)-12] = byte;
				memcpy(&(odom_mess.vx),float_buffer,4);
				printf("odom_mess.vx :%f , %d  \n" , odom_mess.vx , state);
			}
			else if(data_index<19)
			{
				float_buffer[(data_index++)-16] = byte;
			}
			else if(data_index == 19)
			{
				float_buffer[(data_index++)-16] = byte;
				memcpy(&(odom_mess.w),float_buffer,4);
				printf("odom_mess.w :%f , %d \n " , odom_mess.w , state);
				data_index = 0;
				state = data_came;
				state = waiting_heading; // omit xor for now
			
			}
			else state = waiting_heading;
			break;
		case laserscan_coming:
			if(k<laserscan_length/2)
			{
				if(data_index ==0)
				{
					integer_buffer[data_index++] = byte;
				}
				else if(data_index==1)
				{
					integer_buffer[data_index++] = byte;
					memcpy(&(laserscan_mess.data[k]),integer_buffer,2);
					printf("laserscan.%d : %d \n " , k , laserscan_mess.data[k]);
					data_index = 0;
					k++;
				}

				else state = waiting_heading;
			}
			else if(k == laserscan_length/2)
{
			k=0;
			state = waiting_heading; // omit xor for now
}
			break;
		
	
			
	}
}
void initializeOdometryMessage(odometry_message * odom_mess)
{
	odom_mess->premess.heading[0] = 0xFF;
	odom_mess->premess.heading[1] = 0xFF;
	odom_mess->premess.heading[2] = 0xFD;
	odom_mess->premess.message_type = odometry;
	odom_mess->premess.error_code = none;

}

void initializeVelocitycmdMessage(velocitycmd_message * vel_cmd_mess)
{
        vel_cmd_mess->premess.heading[0] = 0xFF;
        vel_cmd_mess->premess.heading[1] = 0xFF;
        vel_cmd_mess->premess.heading[2] = 0xFD;
        vel_cmd_mess->premess.message_type = velocitycmd;
        vel_cmd_mess->premess.error_code = none;
	vel_cmd_mess->premess.extra_info_1 = none;
	vel_cmd_mess->premess.extra_info_2 = none;
	vel_cmd_mess->premess.extra_info_3 = none;
	

}

void sendVelocitycmd(float *vx, float *w)
{
	initializeVelocitycmdMessage(&vel_cmd_mess);
	vel_cmd_mess.vx = *vx;
	vel_cmd_mess.w = *w;
	
	memcpy(data_buffer,&vel_cmd_mess,prelim_length+velocitycmd_length);
	data_buffer[16]=0x7E;
	data_buffer[17]=0x7E;
	data_buffer[18]=0x7E;
	data_buffer[19]=0x7E;
	

	
}



#ifdef __cplusplus
  }
#endif






