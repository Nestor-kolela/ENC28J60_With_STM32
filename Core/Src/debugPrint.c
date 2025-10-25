/*
 * debugPrint.c
 *
 *  Created on: Oct 24, 2025
 *      Author: Nestor Kalambay
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "queue.h"
#include "semphr.h"

#include "log.h"

static volatile bool bDMATxFlag = false;


extern osMutexId_t debugMsgMutexHandle;
extern osMessageQueueId_t qDebugPrintHandle;
extern UART_HandleTypeDef hlpuart1;

void dMesgPrint(uint8_t debugLevel, const char *format, ...);

static uint32_t current = 0;
static uint32_t previous = 0;

static uint8_t day = 0;
static uint8_t hour = 0;
static uint8_t minute = 0;
static uint8_t second = 0;

void debugPrintTask(void *argument)
{

	while(true)
	{
		vTaskDelay(500 / portTICK_PERIOD_MS);
		current = (xTaskGetTickCount() * 1000 / configTICK_RATE_HZ);
		if(current - previous > 1000)
		{
			dMesgPrint(DEBUG_VERBOSE, "\r\nTime: %02d day(s) %02u:%02u:%02u\r\n", day, hour, minute, second);

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
						if(++day >= 30)
						{
							day = 0;
						}
					}
				}
			}
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == hlpuart1.Instance)
	{
		bDMATxFlag = false;
	}
}
