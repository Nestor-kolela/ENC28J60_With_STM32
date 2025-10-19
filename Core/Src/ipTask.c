/*
 * ipTask.c
 *
 *  Created on: Feb 13, 2025
 *      Author: NK KALAMBAY
 */
#include "ringBuffer.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "main.h"
#include "cmsis_os.h"

#include "lwip/init.h"
#include "lwip/netif.h"

#include "netif/ethernet.h"

#include "enc28j60_hwd.h"
#include "spiHwdInterface.h"

#include "FreeRTOS.h"
#include "semphr.h"


struct netif my_netif;
ip4_addr_t ipaddr, netmask, gw;

err_t ethernet_init(struct netif *netif);
void uartPrint(const char * ptr);
void uartPrintBytes(uint8_t * ptr, uint16_t length);

extern enc28j60Drv dev;
extern UART_HandleTypeDef hlpuart1;

SemaphoreHandle_t xSemaphore;

char buf[64];
uint32_t u32PacketCounter = 0;
uint8_t u8TempValue[10];
ft_ringBuffer uartQueue;
volatile bool bDMATxFlag = false;
uint32_t previous = 0;
uint32_t current = 0;
char stringUpTime[64];
uint8_t second = 0;
uint8_t minute = 0;
uint8_t hour;
uint8_t data[1024];
void ipConnectivityMainTask(void *argument)
{
	enc28j60_initDr(&dev, spi1ChipSelect, spi1ChipDeSelect, spi1Read, spi1Write, NULL, delayMsFunction);
	enc28j60_strtDr(&dev);

	// Add network interface
	netif_add_noaddr(&my_netif, NULL, ethernet_init, ethernet_input);

	// Set the interface as the default
	netif_set_default(&my_netif);

	// Bring up the interface
	netif_set_up(&my_netif);

	volatile uint8_t u8Value = 0;

	xSemaphore = xSemaphoreCreateMutex();

	tf_ringbuffer_init(&uartQueue);

	while(true)
	{
		//Print the up-time as well
		current = (xTaskGetTickCount() * 1000 / configTICK_RATE_HZ);
		if(current - previous > 1000)
		{
			UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);

			int err;
			err =  snprintf(stringUpTime, sizeof(stringUpTime), "Stack is at %lu\r\n", watermark);
			if(err > 0) tf_ringbuffer_puts(&uartQueue, stringUpTime);
			err = snprintf(stringUpTime, sizeof(stringUpTime), "Time %02u:%02u:%02u\r\n", hour, minute, second);
			if(err > 0) tf_ringbuffer_puts(&uartQueue, stringUpTime);
			previous = current;

			if(++second >= 60)
			{
				second = 0;
				if(++minute >= 60)
				{
					minute = 0;
					if(++hour >= 24)
					{
						hour = 0;
					}
				}
			}
		}

		if(tf_ringbuffer_dataIsAvailable(&uartQueue) == ft_ring_buffer_data_available)
		{
			if(bDMATxFlag == false)
			{
				uint8_t data;
				tf_ringBuffer_status stat;
				stat = tf_ringbuffer_readByte(&uartQueue, &data);
				if(stat == ft_ring_buffer_read_success)
				{
					uartPrintBytes(&data, 1);
					bDMATxFlag = true;
				}

			}
		}

		u8Value = enc28j60_readEtherReg(&dev, dev.bank0.commonRegs.EIR);
		for(uint8_t cnt = 0; cnt < 8; cnt++)
		{
			if(u8Value & (1 << cnt))
			{
				enc28j60_BitFieldClear(&dev, dev.bank0.commonRegs.EIE, 1 << 7);
				enc28j60_BitFieldSet(&dev, dev.bank0.commonRegs.EIE, 1 << 7);

				switch(cnt)
				{
				case 0:
					tf_ringbuffer_puts(&uartQueue, "1) Receive Error Interrupt Flag bit\r\n");
					break;

				case 1:
					tf_ringbuffer_puts(&uartQueue, "2) Transmit Error Interrupt Flag bit\r\n");
					break;

				case 2:
					tf_ringbuffer_puts(&uartQueue, "3) WOL Interrupt Flag bit\r\n");
					break;

				case 3:
					tf_ringbuffer_puts(&uartQueue, "4) Transmit Interrupt Flag bit\r\n");
					break;

				case 4:
					tf_ringbuffer_puts(&uartQueue, "5) Link Change Interrupt Flag bit\r\n");
					(void) enc28j60_readPhyReg(&dev, dev.phyReg.PHIR);
					break;

				case 5:
					tf_ringbuffer_puts(&uartQueue, "5) DMA Interrupt Flag bit\r\n");
					break;

				case 6:
					tf_ringbuffer_puts(&uartQueue, "6) Receive Packet Pending Interrupt Flag bit\r\n");

					bool err;
					err = enc28j60_etherReceive(&dev, u8TempValue, 0);

					if(err == true)
					{
						//Packet number
						snprintf((char *) buf, sizeof(buf), "Packet number %lu\r\n", u32PacketCounter++);
						tf_ringbuffer_puts(&uartQueue, buf);

						//Packet length
						snprintf((char *) buf, sizeof(buf), "Packet length %u\r\n", dev.rxPkt.pktLen.u16PktLen);
						tf_ringbuffer_puts(&uartQueue, buf);

						char tempBuffer[10];
						for(uint16_t i = 0; i < dev.rxPkt.pktLen.u16PktLen; i++)
						{
							uint8_t u8ReadData = dev.rxPkt.data[i];
							snprintf(tempBuffer, sizeof(tempBuffer), "0x%02X ", u8ReadData);
							if((i != 0) && ((i % 16) == 0)) tf_ringbuffer_puts(&uartQueue, "\r\n");

							tf_ringbuffer_puts(&uartQueue, tempBuffer);
						}

						tf_ringbuffer_puts(&uartQueue, "\r\n");
					}
					break;
				}

			}
		}
	}
}


void uartPrintBytes(uint8_t * ptr, uint16_t length)
{
	(void) HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *) ptr, length);
}
err_t ethernet_init(struct netif *netif)
{

	return ERR_OK;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == hlpuart1.Instance)
	{
		bDMATxFlag = false;
	}
}

