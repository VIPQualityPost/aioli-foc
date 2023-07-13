/** 
 * CAN header
*/

#ifndef CAN_H
#define CAN_H

#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"

#define CAN_OK 0
#define CAN_ERROR 1

enum canSpeed
{
	MBit1 = 10,
	Kbit500 = 20,
	Kbit250 = 40,
	Kbit125 = 80
};

extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);
extern "C" void FDCAN1_IT0_IRQHandler();
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

uint8_t FDCAN_Init(FDCAN_HandleTypeDef);
uint8_t FDCAN_Config(FDCAN_HandleTypeDef);
uint8_t FDCAN_Write(void);


#endif 