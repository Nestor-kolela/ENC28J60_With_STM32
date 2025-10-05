/*
 * ipTask.c
 *
 *  Created on: Feb 13, 2025
 *      Author: NK KALAMBAY
 */
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

extern enc28j60Drv dev;
extern UART_HandleTypeDef hlpuart1;

SemaphoreHandle_t xSemaphore;

uint8_t buf[100] = {0};
uint32_t u32PacketCounter = 0;
uint8_t u8TempValue[10];

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

	while(true)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
		u8Value = enc28j60_readEtherReg(&dev, dev.bank0.commonRegs.EIR);
		for(uint8_t cnt = 0; cnt < 8; cnt++)
		{
			if(u8Value & (1 << cnt))
			{
				enc28j60_BitFieldClear(&dev, dev.bank0.commonRegs.EIE, 1 << 7);
				enc28j60_BitFieldSet(&dev, dev.bank0.commonRegs.EIE, 1 << 7);
				vTaskDelay(10 / portTICK_PERIOD_MS);
				if(cnt == 4)
				{
					(void) enc28j60_readPhyReg(&dev, dev.phyReg.PHIR);
				}

				switch(cnt)
				{
				case 0:
					uartPrint("1) Receive Error Interrupt Flag bit\r\n");
					break;

				case 1:
					uartPrint("2) Transmit Error Interrupt Flag bit\r\n");
					break;

				case 2:
					uartPrint("3) WOL Interrupt Flag bit\r\n");
					break;

				case 3:
					uartPrint("4) Transmit Interrupt Flag bit\r\n");
					break;

				case 4:
					uartPrint("5) Link Change Interrupt Flag bit\r\n");
					break;

				case 5:
					uartPrint("5) DMA Interrupt Flag bit\r\n");
					break;

				case 6:
					uartPrint("6) Receive Packet Pending Interrupt Flag bit\r\n");
					enc28j60_etherReceive(&dev, u8TempValue, 0);
					sprintf(buf, "Packet number %u\r\n", u32PacketCounter++);
					uartPrint(buf);
					break;
				}
			}
		}
	}
}

void uartPrint(const char * ptr)
{
	xSemaphoreTake(xSemaphore, portMAX_DELAY);
	size_t length = strlen(ptr);
	(void) HAL_UART_Transmit(&hlpuart1, (uint8_t *) ptr, length, 100);
	xSemaphoreGive(xSemaphore);
}
err_t ethernet_init(struct netif *netif)
{

	return ERR_OK;
}
