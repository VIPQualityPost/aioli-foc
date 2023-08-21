/** 
 * CAN header
*/

#ifndef CAN_H
#define CAN_H

#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"

#define CAN_OK 0
#define CAN_ERROR 1

// extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);
// extern "C" void FDCAN1_IT0_IRQHandler();
// extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

void FDCAN_Start(void);
void FDCAN_SendMessage(uint8_t dataByte);

#endif 