#include <Arduino.h>
#include <EEPROM.h>
#include <Spi.h>

// #include <FlashStorage_STM32.h>

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"

#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"

#include "can.h"
#include "dfu.h"
#include "aioli-board.h"

#define USBD_MANUFACTURER_STRING     	"matei repair lab"
#define USBD_PRODUCT_STRING_FS     		"knitting machine"

// Motor specific parameters.
#define POLEPAIRS 7
#define RPHASE 1.4
#define MOTORKV 1000

uint8_t useDFU = 0;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t RxData[8];

uint8_t canCounter = 0;

// simpleFOC constructors
BLDCDriver3PWM driver = BLDCDriver3PWM(U_PWM, V_PWM, W_PWM, U_EN, V_EN, W_EN);
BLDCMotor motor = BLDCMotor(POLEPAIRS, RPHASE, MOTORKV);
MagneticSensorMT6701SSI enc = MagneticSensorMT6701SSI(ENC_CS);
Commander commander = Commander(SerialUSB);

// Prototypes
void configureFOC(void);
void configureCAN(void);
void configureDFU(void);

void setup()
{
	// SCB->VTOR == 0x08000000;
	pinMode(USER_LED, OUTPUT);
	pinMode(USER_BUTTON, INPUT);

	SerialUSB.begin(115200);

	configureCAN();
	// configureDFU();
	// configureFOC();
}

void loop()
{
	// motor.loopFOC();
	// motor.move();
	// commander.run();

	// How to handle reading/writing SerialUSB from the PC with motor task?
	// if(SerialUSBUSB.available() > 0){
	// 	SerialUSBUSB.readBytes((char*)canTxBuf, 8);
	// 	FDCAN_Write();
	// }

	// if(digitalRead(USER_BUTTON) == HIGH){
	// 	// jump_to_bootloader();
	// 	FDCAN_SendMessage(0xFF);
	// 	// while(digitalRead(USER_BUTTON) == HIGH){}
	// }

	if(digitalRead(USER_BUTTON) == HIGH){
		canCounter += 1;
		FDCAN_SendMessage(canCounter);
	}

	digitalToggle(USER_LED);
	delay(1000);

	// if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
	// {
	// 	uint8_t i;
	// 	for(i=0; i<6; i++){
	// 		digitalToggle(USER_LED);
	// 		delay(10);
	// 	}
	// 	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
	// }
	// else
	// {
		// digitalToggle(USER_LED);
		// delay(1000);
	// }


	#ifdef HAS_MONITOR
	motor.monitor();
	#endif
}

void doMotor(char *cmd)
{
	commander.motor(&motor, cmd);
}

void configureFOC(void){
	commander.add('M', doMotor, "motor");
	commander.verbose = VerboseMode::machine_readable;
	
	#ifdef SIMPLEFOC_STM32_DEBUG
	SimpleFOCDebug::enable(&SerialUSB);
	#endif

	// Encoder initialization.
	// Encoder on SPI1
	enc.init();

	// Driver initialization.
	driver.pwm_frequency = 32000;
	driver.voltage_power_supply = 5;
	driver.voltage_limit = 5;
	driver.init();

	// Motor PID parameters.
	motor.PID_velocity.P = 0.2;
	motor.PID_velocity.I = 3;
	motor.PID_velocity.D = 0.002;
	motor.PID_velocity.output_ramp = 100;
	motor.LPF_velocity.Tf = 0.5;
	motor.LPF_angle.Tf = 0; // try to avoid

	// Motor initialization.
	motor.voltage_sensor_align = 2;
	motor.current_limit = 0.5;
	motor.velocity_limit = 50;
	motor.controller = MotionControlType::velocity;
	motor.foc_modulation = FOCModulationType::SinePWM;

	// Monitor initialization
	#ifdef HAS_MONITOR
	motor.useMonitoring(SerialUSB);
	motor.monitor_start_char = 'M';
	motor.monitor_end_char = 'M';
	motor.monitor_downsample = 250;
	#endif

	motor.linkSensor(&enc);
	motor.linkDriver(&driver);

	motor.target = 0;

	motor.initFOC();
	typedef struct
	{
		uint16_t signature;
		int8_t electricalDir;
		float electricalZero;
	}userData;

	userData boardData;
	EEPROM.get(0, boardData);

	uint16_t magicWord = 0xAF0C;

	if(boardData.signature != magicWord){
		// If we have not initialized the EEPROM before.
		motor.init();
		motor.initFOC();

		boardData.signature = magicWord;
		boardData.electricalZero = motor.zero_electric_angle;
		boardData.electricalDir = motor.sensor_direction;

		EEPROM.put(0, boardData);	// Signature if we have written to EEPROM before.
	}
	else{
		motor.zero_electric_angle = boardData.electricalZero;
		motor.sensor_direction = boardData.electricalDir;
		motor.init();
		motor.initFOC();
		// motor.initFOC(boardData.electricalZero, (Direction)boardData.electricalDir);
	}
}

void configureCAN(void){
	FDCAN_Start();
}

void configureDFU(void){
	// jump_to_bootloader();
	// return 1;
}
