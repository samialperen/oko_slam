/*
 * kernel.c
 *
 *  Created on: 12 Ara 2018
 *      Author: UGUR
 */

#include "kernel.h"

bool bluetoothConnection = false;

extern RGB_mode_Typedef RGB_mode;

void initKernel(void) // try to increase power consumption step by step
{
	LidarStatus_Typedef Initiliazition_Status = Lidar_OK;

	/* Run RGB_ModeFunction once to light up the LEDs */
	RGB_mode = RGB_Mode_Initiliazition;
	RGB_ModeFunction(RGB_mode);
	/* */

	//HAL_TIM_Base_Start_IT(&htim6); // 1ms interrupt for general usage
	HAL_TIM_Base_Start(&htim7); // 1ms counter without interrupt
	configureBluetooth(&huart1, 115200, false); // bluetooth module settings

	setV_Encoder(VOLTAGE_OFF);
	disableAllToFSensors();
	setV_Bluetooth(VOLTAGE_ON);
	setV_Encoder(VOLTAGE_ON); //just for testing purpose
	HAL_Delay(100);

	/* after this point wait for bluetooth connection */
	RGB_mode = RGB_Mode_WaitingForBluetoothConnection;
	enable_UART_ServiceThread();
	enable_RGBThread();
	while(!bluetoothConnection)
	{
		bluetoothConnection = true; //debug
		RGBThread();
		UART_ServiceThread();
	}
	RGB_GreenLED(true);//debug
	disable_UART_ServiceThread();
	disable_RGBThread();
	/* a possible implementation: solder a wire to bluetooth yellow led(connected led) and connect it a GPIO(maybe in imu socket) */
	/* another solution is to wait until some data comes from ROS */ /* this is a better solution */
	/* */

	/* Run RGB_ModeFunction once to light up the LEDs */
	RGB_mode = RGB_Mode_Initiliazition;
	RGB_ModeFunction(RGB_mode);
	/* */

	/* Last configurations */
	Initiliazition_Status |= configureVL53L0X(frontSensor_ID, 30000);
	Initiliazition_Status |= configureVL53L0X(backSensor_ID, 30000);
	Initiliazition_Status |= configureVL53L0X(leftSensor_ID, 30000);
	Initiliazition_Status |= configureVL53L0X(rightSensor_ID, 30000);

	setV_Encoder(VOLTAGE_ON);
	Initiliazition_Status |= startVL53L0X_measuring();
	/* */

	if(Initiliazition_Status != Lidar_OK)
	{
		RGB_mode = RGB_Mode_Error;

		/* Lidar operation cannot start because error(s) occurred */
		disable_wait4zeroThread();
		disable_absoluteEncoderThread();
		disable_VL53L0XThread();
		disable_bluetoothThread();
		enable_RGBThread();
		disable_UART_ServiceThread();
		/* */
	}
	else
	{
		/* Start Lidar operation */
		enable_wait4zeroThread();
		disable_absoluteEncoderThread();
		disable_VL53L0XThread();
		disable_bluetoothThread();
		enable_RGBThread();
		disable_UART_ServiceThread();
		/* */
	}


}

void startKernel(void)
{
	while(1)
	{
		wait4zeroThread();
		absoluteEncoderThread();
		VL53L0XThread();
		bluetoothThread(false);
		RGBThread();
		UART_ServiceThread();
	}
}
