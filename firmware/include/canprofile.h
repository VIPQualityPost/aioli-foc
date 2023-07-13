#ifndef CANPROFILE_H
#define CANPROFILE_H

#define CAN_REQ_DEV     0b001
#define CAN_CNF_DEV     0b101
#define CAN_REQ_HOST    0b010
#define CAN_CNF_HOST    0b110
#define CAN_CMDSTR      0b011
#define CAN_CNF_CMD     0b111
#define CAN_EOF         0b000

#define CAN_MSG_MASK    0b0111
#define CAN_TYPE_MASK   0b1000

#define CAN_DEV         0x80
#define CAN_HOST        0x00


void userCanRxHandler(FDCAN_RxHeaderTypeDef canRxHeader, uint32_t *rxData);

#endif