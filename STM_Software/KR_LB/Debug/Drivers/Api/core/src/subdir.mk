################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Api/core/src/vl53l0x_api.c \
../Drivers/Api/core/src/vl53l0x_api_calibration.c \
../Drivers/Api/core/src/vl53l0x_api_core.c \
../Drivers/Api/core/src/vl53l0x_api_ranging.c \
../Drivers/Api/core/src/vl53l0x_api_strings.c 

OBJS += \
./Drivers/Api/core/src/vl53l0x_api.o \
./Drivers/Api/core/src/vl53l0x_api_calibration.o \
./Drivers/Api/core/src/vl53l0x_api_core.o \
./Drivers/Api/core/src/vl53l0x_api_ranging.o \
./Drivers/Api/core/src/vl53l0x_api_strings.o 

C_DEPS += \
./Drivers/Api/core/src/vl53l0x_api.d \
./Drivers/Api/core/src/vl53l0x_api_calibration.d \
./Drivers/Api/core/src/vl53l0x_api_core.d \
./Drivers/Api/core/src/vl53l0x_api_ranging.d \
./Drivers/Api/core/src/vl53l0x_api_strings.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Api/core/src/%.o: ../Drivers/Api/core/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F070xB -I"D:/Ugur/STM Projects/KR_LB/Inc" -I"D:/Ugur/STM Projects/KR_LB/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/Ugur/STM Projects/KR_LB/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/Ugur/STM Projects/KR_LB/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/Ugur/STM Projects/KR_LB/Drivers/Api/core/inc" -I"D:/Ugur/STM Projects/KR_LB/Drivers/Api/platform/inc" -I"D:/Ugur/STM Projects/KR_LB/Drivers/Api/platform/src" -I"D:/Ugur/STM Projects/KR_LB/Drivers/Api/core/src" -I"D:/Ugur/STM Projects/KR_LB/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


