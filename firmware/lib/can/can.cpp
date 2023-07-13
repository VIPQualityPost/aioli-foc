#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"
#include "can.h"

FDCAN_HandleTypeDef hfdcan1;
FDCAN_FilterTypeDef canFilterConfig;
FDCAN_RxHeaderTypeDef rxHeader;
FDCAN_TxHeaderTypeDef txHeader;

uint32_t *fullID = (uint32_t *)0x1FFF7590;
uint8_t deviceID = (fullID[2] & 0xFF);
const uint8_t canMaxMsgLen = 8;
enum canSpeed canBusSpeed = MBit1;
extern uint8_t canRxBuf[canMaxMsgLen];
extern uint8_t canTxBuf[canMaxMsgLen];

__weak void userCanRxHandler(FDCAN_RxHeaderTypeDef canRxHeader){
	// Actually implement this in the user main.c file.
}

uint8_t FDCAN_Init(FDCAN_HandleTypeDef hfdcan)
{
	hfdcan.Instance = FDCAN1;
	hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan.Init.AutoRetransmission = DISABLE;
	hfdcan.Init.TransmitPause = DISABLE;
	hfdcan.Init.ProtocolException = DISABLE;
	// Need to check if this actually produces the right speeds- need to check clock tree
	hfdcan.Init.NominalPrescaler = canBusSpeed;
	hfdcan.Init.NominalSyncJumpWidth = 1;
	hfdcan.Init.NominalTimeSeg1 = 2;
	hfdcan.Init.NominalTimeSeg2 = 2;
	hfdcan.Init.DataPrescaler = 1;
	hfdcan.Init.DataSyncJumpWidth = 1;
	hfdcan.Init.DataTimeSeg1 = 1;
	hfdcan.Init.DataTimeSeg2 = 1;
	hfdcan.Init.StdFiltersNbr = 0;
	hfdcan.Init.ExtFiltersNbr = 0;
	hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&hfdcan) != HAL_OK)
	{
		return CAN_ERROR;
	}

	return CAN_OK;
}

uint8_t FDCAN_Config(FDCAN_HandleTypeDef hfdcan)
{
	FDCAN_FilterTypeDef canFilter;
	/* Configure Rx filter */
	canFilter.IdType = FDCAN_STANDARD_ID;
	canFilter.FilterIndex = 0;
	canFilter.FilterType = FDCAN_FILTER_MASK;
	canFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	canFilter.FilterID1 = deviceID;
	if (HAL_FDCAN_ConfigFilter(&hfdcan, &canFilter) != HAL_OK)
	{
		return CAN_ERROR;
	}

	/* Configure global filter:
	   Filter all remote frames with STD and EXT ID
	   Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		return CAN_ERROR;
	}

	/* Start the FDCAN module */
	if (HAL_FDCAN_Start(&hfdcan) != HAL_OK)
	{
		return CAN_ERROR;
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		return CAN_ERROR;
	}

	/* Prepare Tx Header */
	txHeader.Identifier = deviceID;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_2;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0;

	return CAN_OK;
}

uint8_t FDCAN_Write(void){
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, canTxBuf) != HAL_OK){
		return CAN_ERROR;
	}

	return CAN_OK;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, canRxBuf);

	userCanRxHandler(rxHeader);
  }
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
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

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
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
