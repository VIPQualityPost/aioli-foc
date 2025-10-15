/**
 * @file: can.cpp
 * @author: matei jordache
*/

/**
 * The idea is that this node will ask if any other node
 * on the bus has a specific ID (CAN_ID). If it exists then
 * that node responds, we increment the ID and try again.
 * 
 * This would get run if you press the "DFU" button on a 
 * module that is not connected to the USB port. That node 
 * is always 0xFF and DFU button on that one will actually 
 * trigger the firmware update.
*/

#include <Arduino.h>
#include "aioli-board.h"
#include "stm32g4xx_hal_fdcan.h"

#define SFOC_CMD 0x01

extern uint8_t sfocCmdStr;

void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData);

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

static uint8_t configureCAN(void){
  return 0;
}

int dlcToLength(uint32_t dlc)
{
	int length = dlc >> 16;
	if (length >= 13)
	{
		return 32 + (13 - length) * 16;
	}
	else if (length == 12)
	{
		return 24;
	}
	else if (length >= 9)
	{
		return 12 + (9 - length) * 4;
	}
	return length;
}

void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData)
{
    int byte_length = dlcToLength(rxHeader.DataLength);

    #ifdef CAN_DEBUG
	Serial.print("Received packet, id=0x");
	Serial.print(rxHeader.Identifier, HEX);
	Serial.print(", length=");
	Serial.print(byte_length);
	for (int byte_index = 0; byte_index < byte_length; byte_index++)
	{
		Serial.print(" byte[" + byte_index + "]: ");
		Serial.print(rxData[byte_index] + " ");
	}
	Serial.println();
    #endif

    // Move the data back to the main SFOC loop
    sfocCmdStr = *rxData;

	digitalToggle(AIOLI_LED);
}

void writeFrame(uint8_t writeData[])
{
	TxHeader.Identifier = 0x321;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_2;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

    #ifdef CAN_DEBUG
	Serial.print("CAN: sending message ");
	Serial.println(can1.addMessageToTxFifoQ(&TxHeader, writeData) == HAL_OK ? "was ok." : "failed.");
    #endif 
}