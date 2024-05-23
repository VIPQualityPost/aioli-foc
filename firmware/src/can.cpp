// Copyright 2024 matei jordache <mateijordache@live.com>
/**
 * @file: can.c
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

#include "aioli-board.h"

#define SFOC_CMD 0x01

extern uint8_t sfocCmdStr;

static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData);
static void configureCAN();

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

static void configureCAN() {
  Serial.println(can1.init(CanSpeed::Mbit1) == HAL_OK
                     ? "CAN: initialized."
                     : "CAN: error when initializing.");
  can1.activateNotification(&can1RxHandler);
  Serial.println(can1.start() == HAL_OK ? "CAN: started."
                                        : "CAN: error starting.");
}

int dlcToLength(uint32_t dlc) {
  int length = dlc >> 16;
  if (length >= 13) {
    return 32 + (13 - length) * 16;
  } else if (length == 12) {
    return 24;
  } else if (length >= 9) {
    return 12 + (9 - length) * 4;
  }
  return length;
}

static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData) {
  int byte_length = dlcToLength(rxHeader.DataLength);

#ifdef CAN_DEBUG
  Serial.print("Received packet, id=0x");
  Serial.print(rxHeader.Identifier, HEX);
  Serial.print(", length=");
  Serial.print(byte_length);
  for (int byte_index = 0; byte_index < byte_length; byte_index++) {
    Serial.print(" byte[" + byte_index + "]: ");
    Serial.print(rxData[byte_index] + " ");
  }
  Serial.println();
#endif

  // Move the data back to the main SFOC loop
  sfocCmdStr = *rxData;

  digitalToggle(USER_LED);
}

void writeFrame(uint8_t writeData[]) {
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
  Serial.println(can1.addMessageToTxFifoQ(&TxHeader, writeData) == HAL_OK
                     ? "was ok."
                     : "failed.");
#endif
}
