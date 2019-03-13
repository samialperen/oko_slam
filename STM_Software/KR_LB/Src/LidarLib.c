/*
 * LidarLib.c
 *
 *  Created on: 2 Ara 2018
 *      Author: UGUR
 */

#include "LidarLib.h"
#include "main.h"
#include "stm32f0xx_hal.h"

extern uint32_t LED_BlinkingDelay;
extern uint8_t bluetoothReceiveBuffer[3];

LidarStatus_Typedef setV_Bluetooth(voltageStatus status)
{
	switch(status)
	{
		case VOLTAGE_ON:	 HAL_GPIO_WritePin(Bluetooth_En_GPIO_Port, Bluetooth_En_Pin, GPIO_PIN_SET); break;
		case VOLTAGE_OFF:	 HAL_GPIO_WritePin(Bluetooth_En_GPIO_Port, Bluetooth_En_Pin, GPIO_PIN_RESET); break;
	}

	return Lidar_OK;

}


LidarStatus_Typedef configureBluetooth(UART_HandleTypeDef *huart, uint32_t baudrate, bool enableInterrupt)
{
	HAL_StatusTypeDef HAL_status;

	huart->Init.BaudRate = baudrate;
	HAL_status = HAL_UART_Init(huart);

	if(enableInterrupt == true)
	{
		__HAL_UART_ENABLE_IT(huart, UART_IT_TXE | UART_IT_RXNE);
	}
	HAL_UART_Receive_DMA(&huart1, bluetoothReceiveBuffer, 3); //start listening


	if(HAL_status == HAL_OK)
		  return Lidar_OK;
	  else
		  return Lidar_ERROR;
}

LidarStatus_Typedef configureVL53L0X(uint8_t id, uint32_t timingBudget_us) //directions are not set for now
{
	VL53L0X_Error ToF_status = VL53L0X_ERROR_NONE;

	switch(id)
	{
		case 'f':
			HAL_GPIO_WritePin(VL53L0X_Enable0_GPIO_Port, VL53L0X_Enable0_Pin, 1);
			HAL_Delay(100);

			frontSensor.I2cDevAddr = 0x52;
			frontSensor.comms_type = 1;
			frontSensor.comms_speed_khz = 400;

			ToF_status |= VL53L0X_DataInit(&frontSensor);
			ToF_status |= VL53L0X_SetDeviceAddress(&frontSensor, 0x54);
			frontSensor.I2cDevAddr = 0x54; // 53 YAZMA AMIK (OGUZ)
			ToF_status |= VL53L0X_SetDeviceMode(&frontSensor, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
			ToF_status |= VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&frontSensor, timingBudget_us);
			break;

		case 'b':
			HAL_GPIO_WritePin(VL53L0X_Enable1_GPIO_Port, VL53L0X_Enable1_Pin, 1);
			HAL_Delay(100);

			backSensor.I2cDevAddr = 0x52;
			backSensor.comms_type = 1;
			backSensor.comms_speed_khz = 400;

			ToF_status |= VL53L0X_DataInit(&backSensor);
			ToF_status |= VL53L0X_SetDeviceAddress(&backSensor, 0x51);
			backSensor.I2cDevAddr = 0x51;
			ToF_status |= VL53L0X_SetDeviceMode(&backSensor, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
			ToF_status |= VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&backSensor, timingBudget_us);
			break;

		case 'l':
			HAL_GPIO_WritePin(VL53L0X_Enable2_GPIO_Port, VL53L0X_Enable2_Pin, 1);
			HAL_Delay(100);

			leftSensor.I2cDevAddr = 0x52;
			leftSensor.comms_type = 1;
			leftSensor.comms_speed_khz = 400;

			ToF_status |= VL53L0X_DataInit(&leftSensor);
			ToF_status |= VL53L0X_SetDeviceAddress(&leftSensor, 0x55);
			leftSensor.I2cDevAddr = 0x55;
			ToF_status |= VL53L0X_SetDeviceMode(&leftSensor, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
			ToF_status |= VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&leftSensor, timingBudget_us);
			break;

		case 'r':
			HAL_GPIO_WritePin(VL53L0X_Enable3_GPIO_Port, VL53L0X_Enable3_Pin, 1);
			HAL_Delay(100);

			rightSensor.I2cDevAddr = 0x52;
			rightSensor.comms_type = 1;
			rightSensor.comms_speed_khz = 400;

			ToF_status |= VL53L0X_DataInit(&rightSensor);
			ToF_status |= VL53L0X_SetDeviceAddress(&rightSensor, 0x56);
			rightSensor.I2cDevAddr = 0x56;
			ToF_status |= VL53L0X_SetDeviceMode(&rightSensor, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
			ToF_status |= VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&rightSensor, timingBudget_us);
			break;
	}

	if(ToF_status == VL53L0X_ERROR_NONE)
		return Lidar_OK;
	else
		return Lidar_ERROR;
}

LidarStatus_Typedef readVL53L0X(uint8_t id, uint16_t* measurement) //VL53L0X_RangingMeasurementData_t
{
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
	VL53L0X_Error ToF_status = VL53L0X_ERROR_NONE;

	switch(id)
	{
		case 'f':
			ToF_status = VL53L0X_GetRangingMeasurementData(&frontSensor, &RangingMeasurementData);
			break;

		case 'b':
			ToF_status = VL53L0X_GetRangingMeasurementData(&backSensor, &RangingMeasurementData);
			break;

		case 'l':
			ToF_status = VL53L0X_GetRangingMeasurementData(&leftSensor, &RangingMeasurementData);
			break;

		case 'r':
			ToF_status = VL53L0X_GetRangingMeasurementData(&rightSensor, &RangingMeasurementData);
			break;
	}

	*measurement = RangingMeasurementData.RangeMilliMeter;

	if(ToF_status == VL53L0X_ERROR_NONE)
		return Lidar_OK;
	else
		return Lidar_ERROR;
}

LidarStatus_Typedef startVL53L0X_measuring(void)
{
	VL53L0X_Error ToF_status = VL53L0X_ERROR_NONE;

	ToF_status |= VL53L0X_StartMeasurement(&rightSensor);
	ToF_status |= VL53L0X_StartMeasurement(&backSensor);
	ToF_status |= VL53L0X_StartMeasurement(&leftSensor);
	ToF_status |= VL53L0X_StartMeasurement(&frontSensor);

	if(ToF_status == VL53L0X_ERROR_NONE)
		return Lidar_OK;
	else
		return Lidar_ERROR;
}

LidarStatus_Typedef setV_Encoder(voltageStatus status)
{

	switch(status)
	{
		case VOLTAGE_ON:	 HAL_GPIO_WritePin(V_Encoder_En_GPIO_Port, V_Encoder_En_Pin, GPIO_PIN_SET); break;
		case VOLTAGE_OFF:	 HAL_GPIO_WritePin(V_Encoder_En_GPIO_Port, V_Encoder_En_Pin, GPIO_PIN_RESET); break;
	}

		return Lidar_OK;
}


/*LidarStatus_Typedef EnableRGB_PWMs(void)
{
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2); // Blue
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); // Red

	return Lidar_OK;
}*/

void RGB_BlueLED(bool status)
{
	  HAL_GPIO_WritePin(RGB_PORT, StatusLED_Blue_Pin, status);
}

void RGB_GreenLED(bool status)
{

	HAL_GPIO_WritePin(RGB_PORT, StatusLED_Green_Pin, status);
}
void RGB_RedLED(bool status)
{
	HAL_GPIO_WritePin(RGB_PORT, StatusLED_Red_Pin, status);
}

void RGB_BlueLED_Toggle(void)
{
	HAL_GPIO_TogglePin(RGB_PORT, StatusLED_Blue_Pin);
}

void RGB_GreenLED_Toggle(void)
{
	HAL_GPIO_TogglePin(RGB_PORT, StatusLED_Green_Pin);
}

void RGB_RedLED_Toggle(void)
{
	HAL_GPIO_TogglePin(RGB_PORT, StatusLED_Red_Pin);
}

void RGB_ModeFunction(RGB_mode_Typedef Mode)
{
	if(Mode == RGB_Mode_Initiliazition) //static white
	{
		LED_BlinkingDelay = 0;

		RGB_BlueLED(true);
		RGB_GreenLED(true);
		RGB_RedLED(true);
	}
	else if(Mode == RGB_Mode_WaitingForBluetoothConnection) //blinking blue
	{
		LED_BlinkingDelay = 2000;
		RGB_BlueLED_Toggle();

		RGB_GreenLED(false);
		RGB_RedLED(false);
	}
	else if(Mode == RGB_Mode_CollectingData) //blinking green
	{
		LED_BlinkingDelay = 100;
		RGB_GreenLED_Toggle();

		RGB_BlueLED(false);
		RGB_RedLED(false);
	}
	else if(Mode == RGB_Mode_ScanCompleted) //static azure
	{
		LED_BlinkingDelay = 0;

		RGB_BlueLED(true);
		RGB_GreenLED(true);
		RGB_RedLED(false);
	}
	else if(Mode == RGB_Mode_BluetoothTransmissionSuccesful) //static purple
	{
		LED_BlinkingDelay = 0;

		RGB_BlueLED(true);
		RGB_GreenLED(false);
		RGB_RedLED(true);
	}
	else if(Mode == RGB_Mode_Paused) //blinking green
	{
		LED_BlinkingDelay = 1000;
		RGB_GreenLED_Toggle();

		RGB_BlueLED(false);
		RGB_RedLED(false);
	}
	else if(Mode == RGB_Mode_Error) //blinking red
	{
		LED_BlinkingDelay = 500;
		RGB_RedLED_Toggle();

		RGB_BlueLED(false);
		RGB_GreenLED(false);

		/* disable all threads and stop working while blinking red led */
		disable_wait4zeroThread();
		disable_absoluteEncoderThread();
		disable_VL53L0XThread();
		disable_bluetoothThread();
		disable_UART_ServiceThread(); //enabling this might be useful to get feedback about error in the future implementations
	}
}


/*LidarStatus_Typedef setRGB_Colour(RGB_colourMap colour)
{

	RGB_RedLED((	colour & 0x00FF0000) >> 16);
	RGB_GreenLED((	colour & 0x0000FF00) >> 8);
	RGB_BlueLED(	colour & 0x000000FF);

	return Lidar_OK;
}*/

void readAbsolutePosition_byPolling(void)
{
//	absolutePosition_GrayCoded = ((GPIOA->IDR & (0x0000007E))>>1) | ((GPIOA->IDR & (0x00000001))<<6);	// mask PA7 because it is not connected
	absolutePosition_GrayCoded = ((GPIOA->IDR & (0x0000007E))>>1);

}


void convertGrayCode2BinaryCode(void) // seems to be working
{
	uint32_t temp;

	temp = absolutePosition_GrayCoded;
	absolutePosition_BinaryCoded = 0; // preparation

	absolutePosition_BinaryCoded |= (temp & (0x40)); // MSB are equal 0000 0000
	absolutePosition_BinaryCoded |= ((absolutePosition_BinaryCoded & (0x40)) >> 1) ^ (temp & (0x20)); // XOR with prev. bit everytime
	absolutePosition_BinaryCoded |= ((absolutePosition_BinaryCoded & (0x20)) >> 1) ^ (temp & (0x10));
	absolutePosition_BinaryCoded |= ((absolutePosition_BinaryCoded & (0x10)) >> 1) ^ (temp & (0x08));
	absolutePosition_BinaryCoded |= ((absolutePosition_BinaryCoded & (0x08)) >> 1) ^ (temp & (0x04));
	absolutePosition_BinaryCoded |= ((absolutePosition_BinaryCoded & (0x04)) >> 1) ^ (temp & (0x02));
	absolutePosition_BinaryCoded |= ((absolutePosition_BinaryCoded & (0x02)) >> 1) ^ (temp & (0x01));
}

LidarStatus_Typedef disableAllToFSensors(void)
{
	HAL_GPIO_WritePin(VL53L0X_Enable0_GPIO_Port, VL53L0X_Enable0_Pin, 0);
	HAL_GPIO_WritePin(VL53L0X_Enable1_GPIO_Port, VL53L0X_Enable1_Pin, 0);
	HAL_GPIO_WritePin(VL53L0X_Enable2_GPIO_Port, VL53L0X_Enable2_Pin, 0);
	HAL_GPIO_WritePin(VL53L0X_Enable3_GPIO_Port, VL53L0X_Enable3_Pin, 0);

	return Lidar_OK;
}
