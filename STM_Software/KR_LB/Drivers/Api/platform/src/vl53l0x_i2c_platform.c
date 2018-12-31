/*
 * vl53l0x_i2c_platform.c
 */

#include "stm32f0xx_hal.h"
#include "vl53l0x_i2c_platform.h"

extern I2C_HandleTypeDef hi2c1;

#define VL53L0X_COMMS_I2C 1
#define VL53L0X_COMMS_SPI 0
#define speed 400

int32_t VL53L0X_comms_initialise(uint8_t  comms_type,
                                          uint16_t comms_speed_khz)
{
	return 0;
}


int32_t VL53L0X_comms_close(void)
{
	return 0;
}


int32_t VL53L0X_cycle_power(void)
{
	return 0;
}


uint32_t VL53L0X_write_multiple(uint8_t address,uint8_t index,uint8_t *pdata,int32_t count)		//dasdak
{
	if (HAL_I2C_Mem_Write(&hi2c1, address, index, 1, pdata, count, 100) == HAL_OK)
		return 0;
	else
		return 1;
}


int32_t VL53L0X_read_multi(uint8_t address,  uint8_t index, uint8_t  *pdata, int32_t count)		//fsdaşğilfa
{
	if (HAL_I2C_Mem_Read(&hi2c1, address, index, 1, pdata, count, 100) == HAL_OK)
		return 0;
	else
		return 1;
}


int32_t VL53L0X_write_byte(uint8_t address,  uint8_t index, uint8_t data)
{
	uint8_t temp = data;

	if (HAL_I2C_Mem_Write(&hi2c1, address, index, 1, &temp, 1, 100) == HAL_OK)
		return 0;
	else
		return 1;
}


int32_t VL53L0X_write_word(uint8_t address,  uint8_t index, uint16_t  data)
{
	uint8_t pdata[2];

	pdata[0] = (data >> 8) & 0x00FF;
	pdata[1] = data & 0x00FF;

	if (HAL_I2C_Mem_Write(&hi2c1, address, index, 1, pdata, 2, 100) == HAL_OK)
		return 0;
	else
		return 1;
}


int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t  data)
{
	uint8_t pdata[4];

	pdata[0] = (data >> 24) & 0x000000FF;
	pdata[1] = (data >> 16) & 0x000000FF;
	pdata[2] = (data >> 8) & 0x000000FF;
	pdata[3] = data & 0x000000FF;

	if (HAL_I2C_Mem_Write(&hi2c1, address, index, 1, pdata, 4, 100) == HAL_OK)
		return 0;
	else
		return 1;
}


int32_t VL53L0X_read_byte(uint8_t address,  uint8_t index, uint8_t  *pdata)
{

	if (HAL_I2C_Mem_Read(&hi2c1, address, index, 1, pdata, 1, 100) == HAL_OK)
			return 0;
		else
			return 1;
}


int32_t VL53L0X_read_word(uint8_t address,  uint8_t index, uint16_t *pdata)
{
	uint16_t receivedData[2];

	if (HAL_I2C_Mem_Read(&hi2c1, address, index, 1, (uint8_t*)receivedData, 2, 100) != HAL_OK)
		return 1;

	*pdata = ((receivedData[0] <<8) & 0xFF00) | receivedData[1];

	return 0;

}


int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
	uint32_t receivedData[2];

	if (HAL_I2C_Mem_Read(&hi2c1, address, index, 1, (uint8_t*)receivedData, 2, 100) != HAL_OK)
		return 1;

	*pdata = (((receivedData[0] << 8) & 0xFF000000) | ((receivedData[1] << 16) & 0x00FF0000) | ((receivedData[2] << 8) & 0x0000FF00) | (receivedData[3] & 0x000000FF));

	return 0;
}


int32_t VL53L0X_platform_wait_us(int32_t wait_us)			// implement later
{
	HAL_Delay(wait_us);
	return 0;
}


int32_t VL53L0X_wait_ms(int32_t wait_ms)
{
	HAL_Delay(wait_ms);
	return 0;
}


int32_t VL53L0X_set_gpio(uint8_t  level)
{
	return 0;
}


int32_t VL53L0X_get_gpio(uint8_t *plevel)
{
	return 0;
}


int32_t VL53L0X_release_gpio(void)
{
	return 0;
}


int32_t VL53L0X_get_timer_frequency(int32_t *ptimer_freq_hz)
{
	*ptimer_freq_hz = 0;
	return 1;
}


int32_t VL53L0X_get_timer_value(int32_t *ptimer_count)
{
	*ptimer_count = 0;
	return 1;
}


