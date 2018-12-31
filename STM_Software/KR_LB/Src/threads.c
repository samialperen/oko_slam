/*
 * threads.c
 *
 *  Created on: 12 Ara 2018
 *      Author: UGUR
 */

#include "threads.h"


bool wait4zeroThreadEnabled = false;

uint32_t absoluteEncoderThreadCounter = 0;
bool absoluteEncoderThreadEnabled = false;
extern uint8_t absolutePosition_BinaryCoded;

uint32_t VL53L0XThreadCounter = 0;
bool VL53L0XThreadEnabled = false, VL53L0XThread_ResetIndexer = false;

uint32_t bluetoothThreadCounter = 0;
bool bluetoothThreadEnabled = false;

uint32_t RGBThreadCounter = 0;
bool RGBThreadEnabled = false;

uint32_t UART_ServiceThreadCounter = 0;
bool UART_ServiceThreadEnabled = false;

extern bool bluetoothConnection;
uint8_t bluetoothDataBuffer[260], bluetoothReceiveBuffer[3];
bool bluetoothTransmissionSuccessful = false, bluetoothIncomingDataAvaible = false;
bool LidarOperationPaused = false;

LidarStatus_Typedef Lidar_Status = Lidar_OK;
RGB_mode_Typedef RGB_mode = RGB_Mode_Initiliazition;
uint32_t LED_BlinkingDelay = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim6.Instance) // if timer6 created the interrupt 1ms
	{
		absoluteEncoderThreadCounter++;
	}
	else if(htim->Instance == htim7.Instance) //100ns
	{

	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		bluetoothTransmissionSuccessful = true;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart1.Instance)
	{
		bluetoothIncomingDataAvaible = true;
	}
}


void wait4zeroThread(void)
{
	if(wait4zeroThreadEnabled)
	{
		readAbsolutePosition_byPolling();
		convertGrayCode2BinaryCode();

		if(absolutePosition_BinaryCoded == 0)
		{
			disable_wait4zeroThread();
			enable_absoluteEncoderThread();
		}
	}
}

void enable_wait4zeroThread(void)
{
	wait4zeroThreadEnabled = true;
}

void disable_wait4zeroThread(void)
{
	wait4zeroThreadEnabled = false;
}


void absoluteEncoderThread(void) //example thread
{
	static uint8_t prevPosition = 0;

	if(absoluteEncoderThreadEnabled)
	{
		readAbsolutePosition_byPolling();
		convertGrayCode2BinaryCode();

		if(prevPosition != absolutePosition_BinaryCoded) // a better way is to use interrupts on each 7 pins
		{
			prevPosition = absolutePosition_BinaryCoded;
			enable_VL53L0XThread();
		}
	}
}


void enable_absoluteEncoderThread(void)
{
	absoluteEncoderThreadEnabled = true;
}

void disable_absoluteEncoderThread(void)
{
	absoluteEncoderThreadEnabled = false;
}



void VL53L0XThread(void)
{
	static uint16_t index = 2;
	uint16_t measurement1, measurement2, measurement3, measurement4;

	if(VL53L0XThreadEnabled)
	{
		disable_VL53L0XThread();

		if(VL53L0XThread_ResetIndexer)
		{
			VL53L0XThread_ResetIndexer = false;
			index = 2;
		}

		//read from sensors and push them to array: bluetoothDataBuffer
		/*Lidar_Status |= readVL53L0X(frontSensor_ID, &measurement1); //directions are not set yet
		Lidar_Status |= readVL53L0X(leftSensor_ID, &measurement2);
		Lidar_Status |= readVL53L0X(backSensor_ID, &measurement3);
		Lidar_Status |= readVL53L0X(rightSensor_ID, &measurement4);*/

		/*bluetoothDataBuffer[index] = 		(uint8_t)((measurement1 >> 8) & 0x00FF);
		bluetoothDataBuffer[index + 1] = 	(uint8_t)(measurement1 & 0x00FF);
		bluetoothDataBuffer[index + 64] = 	(uint8_t)((measurement2 >> 8) & 0x00FF);
		bluetoothDataBuffer[index + 65] = 	(uint8_t)(measurement2 & 0x00FF);
		bluetoothDataBuffer[index + 128] = (uint8_t)((measurement3 >> 8) & 0x00FF);
		bluetoothDataBuffer[index + 129] = (uint8_t)(measurement3 & 0x00FF);
		bluetoothDataBuffer[index + 192] = (uint8_t)((measurement4 >> 8) & 0x00FF);
		bluetoothDataBuffer[index + 193] = (uint8_t)(measurement4 & 0x00FF);*/

		bluetoothDataBuffer[index] = 		'a';
		bluetoothDataBuffer[index + 1] = 	'a';
		bluetoothDataBuffer[index + 64] = 	'b';
		bluetoothDataBuffer[index + 65] = 	'b';
		bluetoothDataBuffer[index + 128] = 	'c';
		bluetoothDataBuffer[index + 129] = 	'c';
		bluetoothDataBuffer[index + 192] = 	'd';
		bluetoothDataBuffer[index + 193] = 	'd';

		index += 2;

		if(index == 66) //32 data per sensor
		{
			index = 2;
			disable_absoluteEncoderThread();
			enable_bluetoothThread();
		}

	}
}

void enable_VL53L0XThread(void)
{
	VL53L0XThreadEnabled = true;
}

void disable_VL53L0XThread(void)
{
	VL53L0XThreadEnabled = false;
}


void bluetoothThread(bool enable_checkSum)
{
	uint8_t checkSum = 0;
	uint16_t index = 0;

	if(bluetoothThreadEnabled)
	{
		disable_bluetoothThread();
		enable_wait4zeroThread(); // wait for 0 position after completing bluetooth transmission
		//send the data through bluetooth
		if(enable_checkSum)
		{
			for(index = 2; index < 258; index++)
			{
				checkSum += bluetoothDataBuffer[index];
			}
		}

		bluetoothDataBuffer[0] = 0xAA;
		bluetoothDataBuffer[1] = 0x55;
		bluetoothDataBuffer[258] = 0xEC;
		bluetoothDataBuffer[259] = checkSum;

		if(HAL_UART_Transmit_DMA(&huart1, bluetoothDataBuffer, 260) != HAL_OK) // start transmission via DMA
			Lidar_Status = Lidar_ERROR;
	}
}

void enable_bluetoothThread(void)
{
	bluetoothThreadEnabled = true;
}

void disable_bluetoothThread(void)
{
	bluetoothThreadEnabled = false;
}



void RGBThread(void)
{
	if(RGBThreadEnabled && __HAL_TIM_GET_COUNTER(&htim7) > LED_BlinkingDelay)
	{
		__HAL_TIM_SET_COUNTER(&htim7, 0);

		if(Lidar_Status != Lidar_OK)
			RGB_mode = RGB_Mode_Error;
		else if(absoluteEncoderThreadEnabled)
			RGB_mode = RGB_Mode_CollectingData;
		else if(bluetoothThreadEnabled)
			RGB_mode = RGB_Mode_ScanCompleted;
		else if(bluetoothTransmissionSuccessful)
			RGB_mode = RGB_Mode_BluetoothTransmissionSuccesful;
		else if(LidarOperationPaused)
			RGB_mode = RGB_Mode_Paused;


		RGB_ModeFunction(RGB_mode);
	}
}

void enable_RGBThread(void)
{
	RGBThreadEnabled = true;
}

void disable_RGBThread(void)
{
	RGBThreadEnabled = false;
}

void UART_ServiceThread(void) //needs a lot of testing
{
	if(UART_ServiceThreadEnabled)
	{
		if(bluetoothIncomingDataAvaible)
		{
			bluetoothIncomingDataAvaible = false;
			if(HAL_UART_Receive_DMA(&huart1, bluetoothReceiveBuffer, 3) != HAL_OK) //needs testing
				Lidar_Status = Lidar_ERROR;

			if(bluetoothReceiveBuffer[1] == 13 && bluetoothReceiveBuffer[2] == 10) //CRLF (might be wrong in terms of values, check this
			{
				switch(bluetoothReceiveBuffer[0])
				{
					case 's': //start
						bluetoothConnection = true;
						break;

					case 'p': //pause (This is not necessary at all, only useful for battery charge when testing)
						/* In the future, disable all ToF sensors and encoder here and re-configure them when resumed */
						disable_wait4zeroThread();
						disable_absoluteEncoderThread();
						disable_VL53L0XThread();
						disable_bluetoothThread();
						//VL53L0XThread_ResetIndexer = true;
						LidarOperationPaused = true;
						break;

					case 'r': //resume
						enable_wait4zeroThread();
						disable_absoluteEncoderThread();
						disable_VL53L0XThread();
						disable_bluetoothThread();
						VL53L0XThread_ResetIndexer = true; //if there was data left before pausing
						LidarOperationPaused = false;
						break;

					default:
						break;
				}
			}
		}
	}
}

void enable_UART_ServiceThread(void)
{
	UART_ServiceThreadEnabled = true;
}

void disable_UART_ServiceThread(void)
{
	UART_ServiceThreadEnabled = false;
}

