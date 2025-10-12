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

char buf[7500];
uint32_t u32PacketCounter = 0;
uint8_t u8TempValue[10];
ringBuffer uartQueue;
volatile bool bDMATxFlag = false;

void ipConnectivityMainTask(void *argument)
{
	enc28j60_initDr(&dev, spi1ChipSelect, spi1ChipDeSelect, spi1Read, spi1Write, NULL, delayMsFunction);
	enc28j60_strtDr(&dev);

	// Set IP address, netmask, and gateway
	IP4_ADDR(&ipaddr, 192, 168, 8, 100);
	IP4_ADDR(&netmask, 255, 255, 255, 0);
	IP4_ADDR(&gw, 192, 168, 8, 1);

	// Add network interface
	netif_add(&my_netif, &ipaddr, &netmask, &gw, NULL, ethernet_init, ethernet_input);

	// Set the interface as the default
	netif_set_default(&my_netif);

	// Bring up the interface
	netif_set_up(&my_netif);

	volatile uint8_t u8Value = 0;

	xSemaphore = xSemaphoreCreateMutex();

	ringBufferInit(&uartQueue);

	while(true)
	{
		//Check the queue and pop if possible
		if(ringBufferDataAvailable(&uartQueue) > 0)
		{
			if(bDMATxFlag == false)
			{
				uint8_t data;
				ringBufferReadByte(&uartQueue, &data);
				//Transmit via DMA
				uartPrintBytes(&data, 1);
				bDMATxFlag = true;
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
					ringBufferInsertString(&uartQueue, "1) Receive Error Interrupt Flag bit\r\n");
					break;

				case 1:
					ringBufferInsertString(&uartQueue, "2) Transmit Error Interrupt Flag bit\r\n");
					break;

				case 2:
					ringBufferInsertString(&uartQueue, "3) WOL Interrupt Flag bit\r\n");
					break;

				case 3:
					ringBufferInsertString(&uartQueue, "4) Transmit Interrupt Flag bit\r\n");
					break;

				case 4:
					ringBufferInsertString(&uartQueue, "5) Link Change Interrupt Flag bit\r\n");
					(void) enc28j60_readPhyReg(&dev, dev.phyReg.PHIR);
					break;

				case 5:
					ringBufferInsertString(&uartQueue, "5) DMA Interrupt Flag bit\r\n");
					break;

				case 6:
					enc28j60_etherReceive(&dev, u8TempValue, 0);

					ringBufferInsertString(&uartQueue, "6) Receive Packet Pending Interrupt Flag bit\r\n");

					//Packet number
					sprintf((char *) buf, "Packet number %lu\r\n", u32PacketCounter++);
					ringBufferInsertString(&uartQueue, buf);

					//Packet length
					sprintf((char *) buf, "Packet length %u\r\n", dev.rxPkt.pktLen.u16PktLen);
					ringBufferInsertString(&uartQueue, buf);

					char tempBuffer[10];
					for(uint16_t i = 0; i < dev.rxPkt.pktLen.u16PktLen; i++)
					{
						uint8_t u8ReadData = dev.rxPkt.data[i];
						sprintf(tempBuffer, u8ReadData <= 0xF ? "0x0%X " : "0x%2X ", u8ReadData);
						if( (i != 0) && ((i % 16) == 0)) ringBufferInsertString(&uartQueue, "\r\n");
						ringBufferInsertString(&uartQueue, tempBuffer);
					}

					ringBufferInsertString(&uartQueue, "\r\n");

					break;
				}
			}
		}
	}
}

void uartPrint(const char * ptr)
{
	if(bDMATxFlag == false)
	{
		size_t length = strlen(ptr);
		(void) HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *) ptr, length);
		bDMATxFlag = true;
		while(bDMATxFlag == true) vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

void uartPrintBytes(uint8_t * ptr, uint16_t length)
{
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	(void) HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *) ptr, length);
	xSemaphoreGive(xSemaphore);

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


