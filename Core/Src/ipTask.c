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

#include "log.h"

struct netif my_netif;
ip4_addr_t ipaddr, netmask, gw;

err_t ethernet_init(struct netif *netif);

extern enc28j60Drv dev;
extern osMessageQueueId_t qDebugPrintHandle;



static uint32_t u32PacketCounter = 0;
static uint8_t u8TempValue[10];

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

	uint32_t ulEventsToProcess = 0;
	while(true)
	{
		ulEventsToProcess = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		//Print the up-time as well
		while(ulEventsToProcess > 0)
		{
			--ulEventsToProcess;
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
						dMesgPrint(DEBUG_ERROR, "1) Receive Error Interrupt Flag bit\r\n");
						break;

					case 1:
						dMesgPrint(DEBUG_ERROR, "2) Transmit Error Interrupt Flag bit\r\n");
						break;

					case 2:
						dMesgPrint(DEBUG_INFO, "3) WOL Interrupt Flag bit\r\n");
						break;

					case 3:
						dMesgPrint(DEBUG_INFO, "4) Transmit Interrupt Flag bit\r\n");
						break;

					case 4:
						dMesgPrint(DEBUG_INFO, "5) Link Change Interrupt Flag bit\r\n");
						(void) enc28j60_readPhyReg(&dev, dev.phyReg.PHIR);
						break;

					case 5:
						dMesgPrint(DEBUG_INFO, "5) DMA Interrupt Flag bit\r\n");
						break;

					case 6:
						dMesgPrint(DEBUG_INFO, "6) Receive Packet Pending Interrupt Flag bit\r\n");

						bool err;
						err = enc28j60_etherReceive(&dev, u8TempValue, 0);
						if(err == true)
						{
							//Packet number
							dMesgPrint(DEBUG_INFO, "Packet number %d\r\n", u32PacketCounter++);

							//Packet length
							dMesgPrint(DEBUG_INFO, "Packet length %d\r\n", dev.rxPkt.pktLen.u16PktLen);

							char tempBuffer[10];
							for(uint16_t i = 0; i < dev.rxPkt.pktLen.u16PktLen; i++)
							{
								uint8_t u8ReadData = dev.rxPkt.data[i];

								if((i != 0) && ((i % 16) == 0)) dMesgPrint(DEBUG_DEBUG, "\r\n");
								dMesgPrint(DEBUG_DEBUG, "%02X ", u8ReadData);
								dMesgPrint(DEBUG_DEBUG, tempBuffer);
							}

							dMesgPrint(DEBUG_DEBUG, "\r\n");

						}
						break;
					}

				}
			}

		}

	}
}

err_t ethernet_init(struct netif *netif)
{

	return ERR_OK;
}



