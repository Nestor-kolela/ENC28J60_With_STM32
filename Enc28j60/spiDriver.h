/*
 * spiDriver.h
 *
 *  Created on: Feb 8, 2025
 *      Author: NK Kalambay
 */

#ifndef SPIDRIVER_H_
#define SPIDRIVER_H_

#include <stdint.h>
#include <stdbool.h>

void spi_Init(void);

void spi_write_one_byte(const uint8_t enc28j60Addr, const uint8_t u8Data);
void spi_write_multi_bytes(const uint8_t enc28j60Addr, const uint32_t dataLength, uint8_t * u8Data);

uint8_t spi_read_one_byte(const uint8_t enc28j60Addr);
uint32_t spi_read_multi_bytes(const uint8_t enc28j60Addr, const uint32_t dataLength, uint8_t * u8Data);

#endif /* SPIDRIVER_H_ */
