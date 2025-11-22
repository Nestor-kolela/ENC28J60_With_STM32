/*
 * ipTask.c
 *
 *  Created on: February 13, 2025
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
#include "lwip/dhcp.h"
#include "lwip/dns.h"

#include "enc28j60_hwd.h"
#include "spiHwdInterface.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "log.h"

struct netif my_netif;
ip4_addr_t ipaddr, netmask, gw;

err_t ethernet_init(struct netif *netif);
void ethernet_do_translation_to_pbub(enc28j60Drv * dev, struct pbuf *p);

extern enc28j60Drv dev;
extern osMessageQueueId_t qDebugPrintHandle;

static uint32_t u32PacketCounter = 0;
static uint8_t u8TempValue[10];

typedef enum _enc28j60_state
{
	enc28j60_state_init		= 0,
	enc28j60_state_running 	= 1,
	enc28j60_state_error	= 2
}enc28j60_state;

static enc28j60_state enc28j60State = enc28j60_state_init;
static uint32_t ulEventsToProcess = 0;
volatile uint8_t u8Value = 0;
volatile uint8_t u8PktCount = 0;
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

	ulEventsToProcess = 0;
	u8Value = 0;
	u8PktCount = 0;
	enc28j60State = enc28j60_state_running;

	lwip_init();

	while(true)
	{
		ulEventsToProcess = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
		while (ulEventsToProcess > 0) {

			dMesgPrint(DEBUG_INFO, "Notify value --> %u\r\n", ulEventsToProcess);
			--ulEventsToProcess;

			u8PktCount = enc28j60_readEtherReg(&dev, dev.bank1.EPKTCNT);
			dMesgPrint(DEBUG_INFO, "EPKTCNT count --> %d\r\n", u8PktCount);

			u8Value = enc28j60_readEtherReg(&dev, dev.bank0.commonRegs.EIR);
			dMesgPrint(DEBUG_INFO, "EIR REG --> %u\r\n", u8Value);

			for (uint8_t cnt = 0; cnt < 8; cnt++) {

				if (u8Value & (1 << cnt)) {
					//Allow for further interrupts to happen by making the pin go back high
					enc28j60_BitFieldClear(&dev, dev.bank0.commonRegs.EIE, 1 << 7);

					switch (cnt) {
					case 0:
						dMesgPrint(DEBUG_ERROR, "1) Receive Error Interrupt Flag bit\r\n");
						enc28j60State = enc28j60_state_error;
						break;

					case 1:
						dMesgPrint(DEBUG_ERROR, "2) Transmit Error Interrupt Flag bit\r\n");
						enc28j60State = enc28j60_state_error;
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

						enc28j60_BitFieldSet(&dev, dev.bank0.commonRegs.EIE, 1 << 7);

						break;

					case 5:
						dMesgPrint(DEBUG_INFO, "5) DMA Interrupt Flag bit\r\n");
						break;

					case 6:
						dMesgPrint(DEBUG_INFO, "6) Receive Packet Pending Interrupt Flag bit\r\n");

						bool err;
						err = enc28j60_etherReceive(&dev, u8TempValue, 0);

						//Clear the interrupt bit.
						enc28j60_BitFieldSet(&dev, dev.bank0.commonRegs.EIE, 1 << 7);

						if (err == true) {
							//Packet number
							dMesgPrint(DEBUG_INFO, "PKT number %d\r\n", u32PacketCounter++);

							//Packet length
							dMesgPrint(DEBUG_INFO, "PKT length %d\r\n", dev.rxPkt.pktLen.u16PktLen);

							//Let do the translation from array to pbuf
							uint16_t u18length = dev.rxPkt.pktLen.u16PktLen;
							struct pbuf * ethBuffer = pbuf_alloc(PBUF_LINK, u18length, PBUF_REF);
							if(ethBuffer != NULL)
							{
								ethernet_do_translation_to_pbub(&dev, ethBuffer);
								netif_input(ethBuffer, &my_netif);
							}
						}else
						{
							dMesgPrint(DEBUG_ERROR, "RX return false!\r\n");
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
	netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_IGMP | NETIF_FLAG_LINK_UP;
	return ERR_OK;
}

void ethernet_do_translation_to_pbub(enc28j60Drv * dev, struct pbuf *p)
{
	p->next = NULL;
	p->len = dev->rxPkt.pktLen.u16PktLen;
	p->payload = dev->rxPkt.data;
	memcpy((uint8_t *) p->payload, dev->rxPkt.data, dev->rxPkt.pktLen.u16PktLen);
	p->ref = 1;
}
