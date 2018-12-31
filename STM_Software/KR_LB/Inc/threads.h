/*
 * threads.h
 *
 *  Created on: 12 Ara 2018
 *      Author: UGUR
 */

#ifndef THREADS_H_
#define THREADS_H_

#include "stm32f0xx_hal.h"
#include "LidarLib.h"
#include <stdbool.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart1;


void wait4zeroThread(void);
void enable_wait4zeroThread(void);
void disable_wait4zeroThread(void);

void absoluteEncoderThread(void);
void enable_absoluteEncoderThread(void);
void disable_absoluteEncoderThread(void);

void VL53L0XThread(void);
void enable_VL53L0XThread(void);
void disable_VL53L0XThread(void);

void bluetoothThread(bool enable_checkSum);
void enable_bluetoothThread(void);
void disable_bluetoothThread(void);

void RGBThread(void);
void enable_RGBThread(void);
void disable_RGBThread(void);

void UART_ServiceThread(void);
void enable_UART_ServiceThread(void);
void disable_UART_ServiceThread(void);

#endif /* THREADS_H_ */
