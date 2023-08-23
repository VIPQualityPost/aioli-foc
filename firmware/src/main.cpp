#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"

#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"

#include "can.h"
#include "dfu.h"
#include "aioli-board.h"

#define USBD_MANUFACTURER_STRING     	"matei repair lab"
#define USBD_PRODUCT_STRING_FS     		"aioli-foc"

uint8_t hasUSBConnection = 0;

// board specific data
typedef struct
{
	uint16_t signature;
	int8_t electricalDir;
	float electricalZero;
	uint8_t canID;
}userData;

userData boardData;
const uint16_t magicWord = 0xAF0C;

// canbus things
typedef struct
{
	uint8_t name;
}TxPacket;

TxPacket canTxPacket;
extern uint8_t TxData[8];
extern uint8_t RxData[8];

// simpleFOC things
#define POLEPAIRS 7
#define RPHASE 1.4
#define MOTORKV 1000

BLDCDriver3PWM driver = BLDCDriver3PWM(U_PWM, V_PWM, W_PWM, U_EN, V_EN, W_EN);
BLDCMotor motor = BLDCMotor(POLEPAIRS, RPHASE, MOTORKV);
MagneticSensorMT6701SSI enc = MagneticSensorMT6701SSI(ENC_CS);
Commander commander = Commander(SerialUSB);

// Prototypes
void configureFOC(void);
void configureCAN(void);
void configureDFU(void);
void userButton_IT(void);

void setup()
{
	// SCB->VTOR == 0x08000000;
	pinMode(USER_LED, OUTPUT);
	attachInterrupt(USER_BUTTON, userButton_IT, HIGH);

	SerialUSB.begin(115200);

	EEPROM.get(0, boardData);

	configureCAN();
	configureDFU();
	// configureFOC();

	// if(boardData.signature != magicWord)
	// {
	// 	EEPROM.put(0, boardData);
	// }
}

void loop()
{
	// motor.loopFOC();
	// motor.move();
	// commander.run();
		
	#ifdef HAS_MONITOR
	motor.monitor();
	#endif

	if((USB->DADDR & 0x7F) != 0)
	{
		digitalWrite(USER_LED, HIGH);
		hasUSBConnection = 1;
	}
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

	if(boardData.signature != magicWord){
		// If we have not initialized the EEPROM before.
		motor.init();
		motor.initFOC();

		boardData.signature = magicWord;
		boardData.electricalZero = motor.zero_electric_angle;
		boardData.electricalDir = motor.sensor_direction;
	}
	else{
		motor.zero_electric_angle = boardData.electricalZero;
		motor.sensor_direction = boardData.electricalDir;
		motor.init();
		motor.initFOC();
	}
}

void configureCAN(void){
	FDCAN_Start(boardData.canID);
}

void configureDFU(void){
	// jump_to_bootloader();
	// return 1;
}

// void checkConnectionUSB(void)
// {

// }

void userButton_IT(void)
{
	FDCAN_SendMessage();
}

void canTxStructToData(void)
{
	TxData[0] = canTxPacket.name;
}

