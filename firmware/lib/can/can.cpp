#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"
#include "can.h"

extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan);
extern "C" void FDCAN1_IT0_IRQHandler();
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

uint8_t FDCAN_Init(FDCAN_HandleTypeDef hfdcan1){
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission = DISABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = DISABLE;
	// Need to check if this actually produces the right speeds- need to check clock tree
	hfdcan1.Init.NominalPrescaler = 10;
	hfdcan1.Init.NominalSyncJumpWidth = 1;
	hfdcan1.Init.NominalTimeSeg1 = 2;
	hfdcan1.Init.NominalTimeSeg2 = 2;
	hfdcan1.Init.DataPrescaler = 1;
	hfdcan1.Init.DataSyncJumpWidth = 1;
	hfdcan1.Init.DataTimeSeg1 = 1;
	hfdcan1.Init.DataTimeSeg2 = 1;
	hfdcan1.Init.StdFiltersNbr = 0;
	hfdcan1.Init.ExtFiltersNbr = 0;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
	{
		return 0;
	}

	return 1;
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
	if (hfdcan->Instance == FDCAN1)
	{
		/* USER CODE BEGIN FDCAN1_MspInit 0 */

		/* USER CODE END FDCAN1_MspInit 0 */

		/** Initializes the peripherals clocks
		 */
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
		PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		{
			return;
		}

		/* Peripheral clock enable */
		__HAL_RCC_FDCAN_CLK_ENABLE();

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**FDCAN1 GPIO Configuration
		PB8-BOOT0     ------> FDCAN1_RX
		PB9     ------> FDCAN1_TX
		*/
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
		#ifdef FDCAN1_DEF
		GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		#endif
		#ifdef FDCAN1_ALT
		GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		#endif

		/* USER CODE BEGIN FDCAN1_MspInit 1 */

		/* USER CODE END FDCAN1_MspInit 1 */
	}
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan)
{
	if (hfdcan->Instance == FDCAN1)
	{
		/* USER CODE BEGIN FDCAN1_MspDeInit 0 */

		/* USER CODE END FDCAN1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_FDCAN_CLK_DISABLE();

		/**FDCAN1 GPIO Configuration
		PB8-BOOT0     ------> FDCAN1_RX
		PB9     ------> FDCAN1_TX
		*/
		#ifdef FDCAN1_DEF
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
		#endif

		#ifdef FDCAN1_ALT
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
		#endif

		/* USER CODE BEGIN FDCAN1_MspDeInit 1 */

		/* USER CODE END FDCAN1_MspDeInit 1 */
	}
}

