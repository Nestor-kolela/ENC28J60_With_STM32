/*
 * spiDriver.c
 *
 *  Created on: Feb 8, 2025
 *      Author: NK Kalambay
 */

#include "spiDriver.h"
#include "spi2Stm32Interface.h"

static void spi_slave_select(void)
{
	SPI_SLAVE_SELECT();
}
static void spi_slave_deSelect(void)
{
	SPI_SLAVE_DESELECT();
}

void spi_Init(void)
{
	//In this case do nothing.
	spi_slave_deSelect();

	//Clock

	//SPI Mode

}

void spi_write_one_byte(const uint8_t enc28j60Addr, const uint8_t u8Data)
{
	spi_slave_select();

	STM32_Write_Interface(enc28j60Addr, u8Data);

	spi_slave_deSelect();
}
void spi_write_multi_bytes(const uint8_t enc28j60Addr, const uint32_t dataLength, uint8_t * u8Data)
{
	spi_slave_select();

	SPI_WRITE_MULTI(enc28j60Addr, dataLength, u8Data);

	spi_slave_deSelect();
}

uint8_t spi_read_one_byte(const uint8_t enc28j60Addr)
{
	spi_slave_select();

	uint8_t returnValue = STM32_Read_Interface(enc28j60Addr);

	spi_slave_deSelect();

	return returnValue;
}
uint32_t spi_read_multi_bytes(const uint8_t enc28j60Addr, const uint32_t dataLength, uint8_t * u8Data)
{
	spi_slave_select();

	uint32_t returnValue = SPI_READ_MULTI(enc28j60Addr, dataLength, u8Data);

	spi_slave_deSelect();

	return returnValue;
}


