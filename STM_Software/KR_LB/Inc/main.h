/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VL53L0X_Enable1_Pin GPIO_PIN_13
#define VL53L0X_Enable1_GPIO_Port GPIOC
#define VL53L0X_Enable2_Pin GPIO_PIN_14
#define VL53L0X_Enable2_GPIO_Port GPIOC
#define VL53L0X_Enable3_Pin GPIO_PIN_15
#define VL53L0X_Enable3_GPIO_Port GPIOC
#define Encoder_In0_Pin GPIO_PIN_0
#define Encoder_In0_GPIO_Port GPIOA
#define Encoder_In1_Pin GPIO_PIN_1
#define Encoder_In1_GPIO_Port GPIOA
#define Encoder_In2_Pin GPIO_PIN_2
#define Encoder_In2_GPIO_Port GPIOA
#define Encoder_In3_Pin GPIO_PIN_3
#define Encoder_In3_GPIO_Port GPIOA
#define Encoder_In4_Pin GPIO_PIN_4
#define Encoder_In4_GPIO_Port GPIOA
#define Encoder_In5_Pin GPIO_PIN_5
#define Encoder_In5_GPIO_Port GPIOA
#define Encoder_In6_Pin GPIO_PIN_6
#define Encoder_In6_GPIO_Port GPIOA
#define V_Encoder_En_Pin GPIO_PIN_0
#define V_Encoder_En_GPIO_Port GPIOB
#define IMU_Addr_Select_Pin GPIO_PIN_12
#define IMU_Addr_Select_GPIO_Port GPIOB
#define StatusLED_Green_Pin GPIO_PIN_13
#define StatusLED_Green_GPIO_Port GPIOB
#define StatusLED_Red_Pin GPIO_PIN_14
#define StatusLED_Red_GPIO_Port GPIOB
#define StatusLED_Blue_Pin GPIO_PIN_15
#define StatusLED_Blue_GPIO_Port GPIOB
#define Bluetooth_TX_Pin GPIO_PIN_9
#define Bluetooth_TX_GPIO_Port GPIOA
#define Bluetooth_RX_Pin GPIO_PIN_10
#define Bluetooth_RX_GPIO_Port GPIOA
#define Bluetooth_Key_Pin GPIO_PIN_11
#define Bluetooth_Key_GPIO_Port GPIOA
#define Bluetooth_En_Pin GPIO_PIN_12
#define Bluetooth_En_GPIO_Port GPIOA
#define VL53L0X_Interrupt2_Pin GPIO_PIN_15
#define VL53L0X_Interrupt2_GPIO_Port GPIOA
#define VL53L0X_Interrupt2_EXTI_IRQn EXTI4_15_IRQn
#define VL53L0X_Interrupt1_Pin GPIO_PIN_3
#define VL53L0X_Interrupt1_GPIO_Port GPIOB
#define VL53L0X_Interrupt1_EXTI_IRQn EXTI2_3_IRQn
#define VL53L0X_Interrupt0_Pin GPIO_PIN_4
#define VL53L0X_Interrupt0_GPIO_Port GPIOB
#define VL53L0X_Interrupt0_EXTI_IRQn EXTI4_15_IRQn
#define VL53L0X_Interrupt3_Pin GPIO_PIN_5
#define VL53L0X_Interrupt3_GPIO_Port GPIOB
#define VL53L0X_Interrupt3_EXTI_IRQn EXTI4_15_IRQn
#define VL53L0X_Enable0_Pin GPIO_PIN_9
#define VL53L0X_Enable0_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
