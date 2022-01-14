/*
 * Globals.h
 *
 *  Created on: Nov 29, 2021
 *      Author: uib01493
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include "string.h"

/* Data Overhead--------------------------------------------------------------*/
/* USER CODE BEGIN Data_Overhead */
struct dataOverhead
{
	char myString[100];
	uint16_t btn1_flag;

}globalCluster;

typedef enum{
	INIT,
	IDLE,
	STATE_2,
	EXIT
}SM_STATES;

/* USER CODE END Data_Overhead */

uint16_t timer_val;

extern struct netif gnetif;

/* USER CODE BEGIN Prototypes */
void print_to_serial(char *myString);

/* USER CODE END Prototypes */

#endif /* INC_GLOBALS_H_ */
