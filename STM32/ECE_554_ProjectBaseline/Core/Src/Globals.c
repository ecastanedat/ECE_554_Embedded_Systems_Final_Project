/* USER CODE BEGIN Header */
/** * Globals.c
 *
 *  Created on: Jan 13, 2022
 *      Author: ecastanedat
**/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Globals.h"


/* Function prototypes -----------------------------------------------*/
void print_to_serial(char *myString)
{
	uint8_t buffer[100];

	strcpy((char*)buffer, myString);
	HAL_UART_Transmit(&huart3, buffer, strlen((char*)buffer), HAL_MAX_DELAY);

}
