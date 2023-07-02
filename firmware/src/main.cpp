#include <Arduino.h>
#include <Spi.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"

#include "./drv_reset.h"
#include "./aioli-board.h"
// #include "./can.c"

// Motor specific parameters.
#define POLEPAIRS 6
#define RPHASE 1.4
#define MOTORKV 1000

uint8_t pendingFrame = 0;
uint8_t sfocCmdStr;

// simpleFOC constructors
BLDCDriver3PWM driver = BLDCDriver3PWM(U_PWM, V_PWM, W_PWM, U_EN, V_EN, W_EN);
BLDCMotor motor = BLDCMotor(POLEPAIRS, RPHASE, MOTORKV);
MagneticSensorMT6701SSI enc(ENC_CS);
Commander commander = Commander(Serial);

uint8_t configureFOC(void);
uint8_t configureCAN(void);
uint8_t configureDFU(void);

void doMotor(char *cmd)
{
	commander.motor(&motor, cmd);
}

void setup()
{
	SerialUSB.begin();

	Serial.println(configureFOC() == 1 ? "SFOC successfully init." : "SFOC failed to init.");
	Serial.println(configureCAN() == 1 ? "CAN successfully init."  : "CAN failed to init.");
	Serial.println(configureDFU() == 1 ? "DFU successfully init."  : "DFU failed to init.");
}

void loop()
{
	motor.loopFOC();
	motor.move();

	if(pendingFrame){
		commander.run((char*)sfocCmdStr);
		pendingFrame = 0;
	}
	else{
		commander.run();
	}

	#ifdef HAS_COMMANDER
	motor.monitor();
	#endif
}

uint8_t configureFOC(){
	commander.add('M', doMotor, "motor");
	commander.verbose = VerboseMode::machine_readable;
	
	#ifdef SIMPLEFOC_STM_DEBUG
	SimpleFOCDebug::enable(&Serial);
	#endif

	// Encoder initialization.
	// Encoder on SPI1
	enc.init();
	motor.linkSensor(&enc);
	// Encoder initialization.
	// Encoder on SPI1
	enc.init();
	motor.linkSensor(&enc);

	// Driver initialization.
	driver.pwm_frequency = 20000;
	driver.voltage_power_supply = 5;
	driver.voltage_limit = 3;
	driver.init();
	motor.linkDriver(&driver);
	// Driver initialization.
	driver.pwm_frequency = 20000;
	driver.voltage_power_supply = 5;
	driver.voltage_limit = 3;
	driver.init();
	motor.linkDriver(&driver);

	// Motor PID parameters.
	motor.PID_velocity.P = 0.2;
	motor.PID_velocity.I = 3;
	motor.PID_velocity.D = 0.002;
	motor.PID_velocity.output_ramp = 100;
	motor.LPF_velocity.Tf = 0.5;
	motor.LPF_angle.Tf = 0; // try to avoid
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
	motor.velocity_limit = 20;
	motor.controller = MotionControlType::angle;
	motor.foc_modulation = FOCModulationType::SinePWM;
	// Motor initialization.
	motor.voltage_sensor_align = 2;
	motor.current_limit = 0.5;
	motor.velocity_limit = 20;
	motor.controller = MotionControlType::angle;
	motor.foc_modulation = FOCModulationType::SinePWM;

	// Monitor initialization
	#ifdef HAS_MONITOR
	motor.useMonitoring(Serial);
	motor.monitor_start_char = 'M';
	motor.monitor_end_char = 'M';
	motor.monitor_downsample = 500;
	#endif

	motor.init();
	return motor.initFOC();
}

uint8_t configureCAN(){

}

uint8_t configureDFU(){

}