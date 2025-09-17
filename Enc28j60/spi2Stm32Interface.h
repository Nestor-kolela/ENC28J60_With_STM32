/*
 * spi2Stm32Interface.h
 *
 *  Created on: Feb 8, 2025
 *      Author: kolel
 */

#ifndef SPI2STM32INTERFACE_H_
#define SPI2STM32INTERFACE_H_

#include "main.h"

extern SPI_HandleTypeDef hspi1;

uint8_t STM32_Read_Interface(const uint8_t addr);
uint32_t STM32_Read_Multi_Interface(const uint8_t addr, const uint32_t u38Length, uint8_t * pData);

void STM32_Write_Interface(const uint8_t addr, const uint8_t data);
void STM32_Write_Write_Interface(const uint8_t addr, const uint32_t u38Length, uint8_t * pData);

#define SPI_SLAVE_SELECT()						HAL_GPIO_WritePin(Ethernet_CS_GPIO_Port, Ethernet_CS_Pin, GPIO_PIN_SET)
#define SPI_SLAVE_DESELECT()					HAL_GPIO_WritePin(Ethernet_CS_GPIO_Port, Ethernet_CS_Pin, GPIO_PIN_RESET)

#define MAX_TIME_OUT							100
#define SPI_HANDLE								&hspi1

#define SPI_READ_ONE(ADDR)						STM32_Read_Interface(ADDR)
#define SPI_READ_MULTI(ADDR,LENGTH,DATA)		STM32_Read_Multi_Interface(ADDR, LENGTH, DATA)

#define SPI_WRITE_ONE(ADDR, DATA)				STM32_Write_Interface(ADDR, DATA)
#define SPI_WRITE_MULTI(ADDR, LENGTH, DATA)		STM32_Write_Write_Interface(ADDR, LENGTH, DATA)



#endif /* SPI2STM32INTERFACE_H_ */
