/*
 * LidarLib.h
 *
 *  Created on: 2 Ara 2018
 *      Author: UGUR
 */

#ifndef LIDARLIB_H_
#define LIDARLIB_H_

#include "main.h"
#include "stm32f0xx_hal.h"
#include "vl53l0x_api.h"
#include "threads.h"

#include <stdio.h>
#include <stdbool.h>

#define Encoder_PORT	GPIOA
#define RGB_PORT 		GPIOB

#define frontSensor_ID	'f'
#define backSensor_ID	'b'
#define leftSensor_ID	'l'
#define rightSensor_ID	'r'


uint32_t absolutePosition_GrayCoded;
uint8_t absolutePosition_BinaryCoded;

VL53L0X_Dev_t frontSensor, backSensor, leftSensor, rightSensor;

typedef enum
{
	VOLTAGE_OFF,
	VOLTAGE_ON
}voltageStatus;

typedef enum
{
	Lidar_OK = 0,
	Lidar_ERROR = 1,
	Lidar_Timeout = 2, //not used atm
	Lidar_Bluetooth_Error = 4, //not used atm
	Lidar_VL53L0X_Error = 8 //not used atm
}LidarStatus_Typedef;

typedef enum
{
	RGB_Mode_Initiliazition,
	RGB_Mode_WaitingForBluetoothConnection,
	RGB_Mode_CollectingData,
	RGB_Mode_ScanCompleted,
	RGB_Mode_BluetoothTransmissionSuccesful,
	RGB_Mode_Paused,
	RGB_Mode_Error
}RGB_mode_Typedef;

LidarStatus_Typedef setV_Bluetooth(voltageStatus status);
LidarStatus_Typedef setV_Encoder(voltageStatus status);

LidarStatus_Typedef configureBluetooth(UART_HandleTypeDef *huart, uint32_t baudrate, bool enableInterrupt);
LidarStatus_Typedef configureVL53L0X(uint8_t id, uint32_t timingBudget_us); //add some parameters


//LidarStatus_Typedef EnableRGB_PWMs(void);
void RGB_BlueLED(bool status);
void RGB_GreenLED(bool status);
void RGB_RedLED(bool status);
void RGB_BlueLED_Toggle(void);
void RGB_GreenLED_Toggle(void);
void RGB_RedLED_Toggle(void);
void RGB_ModeFunction(RGB_mode_Typedef Mode);
//LidarStatus_Typedef setRGB_Colour(RGB_colourMap colour);

void readAbsolutePosition_byPolling(void);
void convertGrayCode2BinaryCode(void);

LidarStatus_Typedef disableAllToFSensors(void);
LidarStatus_Typedef readVL53L0X(uint8_t id, uint16_t* measurement);
LidarStatus_Typedef startVL53L0X_measuring(void);

#endif /* LIDARLIB_H_ */
