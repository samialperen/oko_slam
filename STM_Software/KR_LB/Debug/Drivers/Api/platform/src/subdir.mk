################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Api/platform/src/vl53l0x_i2c_platform.c \
../Drivers/Api/platform/src/vl53l0x_platform.c \
../Drivers/Api/platform/src/vl53l0x_platform_log.c 

OBJS += \
./Drivers/Api/platform/src/vl53l0x_i2c_platform.o \
./Drivers/Api/platform/src/vl53l0x_platform.o \
./Drivers/Api/platform/src/vl53l0x_platform_log.o 

C_DEPS += \
./Drivers/Api/platform/src/vl53l0x_i2c_platform.d \
./Drivers/Api/platform/src/vl53l0x_platform.d \
./Drivers/Api/platform/src/vl53l0x_platform_log.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Api/platform/src/%.o: ../Drivers/Api/platform/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F070xB -I"D:/Ugur/STM Projects/KR_LB/Inc" -I"D:/Ugur/STM Projects/KR_LB/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/Ugur/STM Projects/KR_LB/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/Ugur/STM Projects/KR_LB/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/Ugur/STM Projects/KR_LB/Drivers/Api/core/inc" -I"D:/Ugur/STM Projects/KR_LB/Drivers/Api/platform/inc" -I"D:/Ugur/STM Projects/KR_LB/Drivers/Api/platform/src" -I"D:/Ugur/STM Projects/KR_LB/Drivers/Api/core/src" -I"D:/Ugur/STM Projects/KR_LB/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


