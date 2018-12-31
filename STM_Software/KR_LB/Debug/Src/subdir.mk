################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/LidarLib.c \
../Src/kernel.c \
../Src/main.c \
../Src/serial.c \
../Src/stm32f0xx_hal_msp.c \
../Src/stm32f0xx_it.c \
../Src/system_stm32f0xx.c \
../Src/threads.c 

OBJS += \
./Src/LidarLib.o \
./Src/kernel.o \
./Src/main.o \
./Src/serial.o \
./Src/stm32f0xx_hal_msp.o \
./Src/stm32f0xx_it.o \
./Src/system_stm32f0xx.o \
./Src/threads.o 

C_DEPS += \
./Src/LidarLib.d \
./Src/kernel.d \
./Src/main.d \
./Src/serial.d \
./Src/stm32f0xx_hal_msp.d \
./Src/stm32f0xx_it.d \
./Src/system_stm32f0xx.d \
./Src/threads.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F070xB -I"D:/Ugur/STM Projects/KR_LB/Inc" -I"D:/Ugur/STM Projects/KR_LB/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/Ugur/STM Projects/KR_LB/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/Ugur/STM Projects/KR_LB/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/Ugur/STM Projects/KR_LB/Drivers/Api/core/inc" -I"D:/Ugur/STM Projects/KR_LB/Drivers/Api/platform/inc" -I"D:/Ugur/STM Projects/KR_LB/Drivers/Api/platform/src" -I"D:/Ugur/STM Projects/KR_LB/Drivers/Api/core/src" -I"D:/Ugur/STM Projects/KR_LB/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


