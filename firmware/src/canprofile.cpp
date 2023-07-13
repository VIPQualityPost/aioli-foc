/** 
 * CAN application specific profile implementation.
**/
#include "arduino.h"
#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"
#include "canprofile.h"
#include "can.h"

extern uint8_t deviceID;
uint8_t canTxBuf[8];
uint8_t canRxBuf[8];

void userCanRxHandler(FDCAN_RxHeaderTypeDef canRxHeader){
	if(canRxHeader.Identifier == deviceID){
		uint8_t canMsgID =      canRxBuf[0] & CAN_MSG_MASK;
        uint8_t canMsgType =    canRxBuf[0] & CAN_TYPE_MASK;

        if(canMsgType == CAN_DEV)
        {
            SerialUSB.write(canRxBuf, 8);
        }
        else if(canMsgType == CAN_HOST)
        {
            switch(canMsgID)
            {
                case CAN_REQ_HOST:
                    canTxBuf[0] = CAN_CNF_HOST;
                    canTxBuf[1] = deviceID;
                    canTxBuf[2] = CAN_EOF;
                    break;

                case CAN_CMDSTR:
                    canTxBuf[0] = CAN_CNF_CMD;
                    canTxBuf[1] = CAN_EOF;
                    break;

                default:
                    break;
            }

            canTxBuf[0] = canTxBuf[0] | CAN_DEV;
            FDCAN_Write();
        }
        else{
            SerialUSB.print("No valid packet received.");
        }
	}
}

void applicationInterface(void){

}