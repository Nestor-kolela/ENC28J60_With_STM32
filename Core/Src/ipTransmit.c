/*
 * ipTransmit.c
 *
 *  Created on: 23 November 2025
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

extern enc28j60Drv dev;
extern osMutexId_t enc28j60MutexHandle;
extern struct netif my_netif;
bool flag = false;

void ipTransmitTask(void *argument)
{
	//void dhcp_set_struct(struct netif *netif, struct dhcp *dhcp)
	while(true)
	{
		if(flag == false)
		{
			BaseType_t xStatus = xSemaphoreTake(enc28j60MutexHandle, portMAX_DELAY);
			if(xStatus == pdPASS)
			{
				if(ERR_OK == dhcp_start(&my_netif))
				{
					flag = true;
				}
				xSemaphoreGive(enc28j60MutexHandle);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(1000));

	}
}
