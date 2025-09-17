/*
 * spi2Stm32Interface.c
 *
 *  Created on: Feb 8, 2025
 *      Author: NK KALAMBAY
 */

#include "spi2Stm32Interface.h"

uint8_t STM32_Read_Interface(const uint8_t addr)
{
	HAL_SPI_Transmit(SPI_HANDLE, (uint8_t *) &addr, 1, MAX_TIME_OUT);

	uint8_t returnValue;
	HAL_SPI_Receive(SPI_HANDLE, &returnValue, 1, MAX_TIME_OUT);

	return returnValue;
}

uint32_t STM32_Read_Multi_Interface(const uint8_t addr, const uint32_t u38Length, uint8_t * pData)
{
	HAL_SPI_Transmit(SPI_HANDLE, (uint8_t *) &addr, 1, MAX_TIME_OUT);

	return HAL_SPI_Receive(SPI_HANDLE, pData, u38Length, MAX_TIME_OUT) == HAL_OK ? u38Length : 0;

}

void STM32_Write_Interface(const uint8_t addr, const uint8_t data)
{
	uint8_t tempBuffer[] = {addr, data};
	HAL_SPI_Transmit(SPI_HANDLE, tempBuffer, 1, MAX_TIME_OUT);

}
void STM32_Write_Write_Interface(const uint8_t addr, const uint32_t u38Length, uint8_t * pData)
{
	HAL_SPI_Transmit(SPI_HANDLE, (uint8_t *) &addr, 1, MAX_TIME_OUT);
	HAL_SPI_Transmit(SPI_HANDLE, pData, u38Length, MAX_TIME_OUT);
}
