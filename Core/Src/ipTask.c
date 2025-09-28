/*
 * ipTask.c
 *
 *  Created on: Feb 13, 2025
 *      Author: NK KALAMBAY
 */
#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "cmsis_os.h"

#include "lwip/init.h"
#include "lwip/netif.h"

#include "netif/ethernet.h"

#include "enc28j60_hwd.h"
#include "spiHwdInterface.h"

struct netif my_netif;
ip4_addr_t ipaddr, netmask, gw;

err_t ethernet_init(struct netif *netif);

extern enc28j60Drv dev;

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

	while(true)
	{

		vTaskDelay(50 / portTICK_PERIOD_MS);
		u8Value = enc28j60_getPhyRevNumber(&dev);
		u8Value += 0;
	}
}

err_t ethernet_init(struct netif *netif)
{

	return ERR_OK;
}
