/*
 * kernel.h
 *
 *  Created on: 12 Ara 2018
 *      Author: UGUR
 */

#ifndef KERNEL_H_
#define KERNEL_H_

#include "stm32f0xx_hal.h"
#include "LidarLib.h"
#include "threads.h"
#include <stdbool.h>

extern UART_HandleTypeDef huart1;

void initKernel(void);
void startKernel(void);

#endif /* KERNEL_H_ */
