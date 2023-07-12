/** 
 * CAN header
*/

#ifndef CAN_H
#define CAN_H

#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"

enum canSpeed
{
	MBit1 = 10,
	Kbit500 = 20,
	Kbit250 = 40,
	Kbit125 = 80
};

uint8_t FDCAN_Init(FDCAN_HandleTypeDef);

#endif 