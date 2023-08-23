/** 
 * CAN header
*/

#ifndef CAN_H
#define CAN_H

#include <Arduino.h>
#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"

void FDCAN_Start(uint8_t canID);
void FDCAN_SendMessage();
void requestUniqueID(void);
void reassignID(uint8_t newID);

#endif 