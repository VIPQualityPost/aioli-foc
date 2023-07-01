#include <Arduino.h>
#include <Spi.h>
#include "STM32_CAN.h"
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"
#include "stm32g4xx_hal_conf.h"

#include "./drv_reset.h"
#include "./aioli-board.h"

// Motor specific parameters.
#define POLEPAIRS 6
#define RPHASE 1.4
#define MOTORKV 1000

#define CAN_ID 0x100

BLDCDriver3PWM driver = BLDCDriver3PWM(U_PWM, V_PWM, W_PWM, U_EN, V_EN, W_EN);
BLDCMotor motor = BLDCMotor(POLEPAIRS, RPHASE, MOTORKV);
MagneticSensorMT6701SSI enc(ENC_CS);
STM32_CAN Can(CAN1, ALT); 

static CAN_message_t CAN_TXmsg;

#ifdef HAS_COMMANDER
Commander commander = Commander(Serial);

void doMotor(char *cmd){
  commander.motor(&motor,cmd);
}
#endif

void setup() {
  // put your setup code here, to run once:

  #ifdef SIMPLEFOC_STM_DEBUG
  SimpleFOCDebug::enable(&Serial);
  #endif
 
  // For our jump-to-bootloader program.
  pinMode(DFU_PIN,INPUT);

  Can.begin();
  Can.setFilter(0, 0x100, 0x1FFFFFFF);
  Can.setBaudRate(115200);


  // Encoder initialization.
  // Encoder on SPI1
  enc.init();
  motor.linkSensor(&enc);

  // Driver initialization.
  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 5;
  driver.voltage_limit  = 3;
  driver.init();
  motor.linkDriver(&driver);

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

  motor.init();
  motor.initFOC();

  // Commander actions
  #ifdef HAS_COMMANDER
    SerialUSB.begin();
    motor.useMonitoring(Serial);
    motor.monitor_start_char = 'M';
    motor.monitor_end_char = 'M';
    motor.monitor_downsample = 500;
    commander.add('M',doMotor,"motor");
    commander.verbose = VerboseMode::machine_readable;
  #endif

}

void loop() {
  motor.loopFOC();
  motor.move();

  #ifdef HAS_COMMANDER
  commander.run();
  motor.monitor();
  #endif

  if(Can.read(CAN_msg)){

  }
}