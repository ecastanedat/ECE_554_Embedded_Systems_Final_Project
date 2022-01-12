/*
 * Globals.h
 *
 *  Created on: Nov 29, 2021
 *      Author: uib01493
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#include "main.h"

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

uint16_t timer_val;

extern struct netif gnetif;

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t myTxData[8];

#endif /* INC_GLOBALS_H_ */
