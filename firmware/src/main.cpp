// Copyright 2024 matei jordache <mateijordache@live.com>
#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

#include "./aioli-board.h"
#include "./drv_reset.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
// #include "./can.cpp"

// Motor specific parameters.
#define POLEPAIRS 4
#define RPHASE 1.4
#define MOTORKV 1000

uint8_t useDFU = 0;
uint8_t pendingFrame = 0;
uint8_t sfocCmdStr;

// simpleFOC constructors
BLDCDriver3PWM driver = BLDCDriver3PWM(U_PWM, V_PWM, W_PWM, U_EN, V_EN, W_EN);
BLDCMotor motor = BLDCMotor(POLEPAIRS, RPHASE, MOTORKV);
MagneticSensorMT6701SSI enc = MagneticSensorMT6701SSI(ENC_CS);
Commander commander = Commander(SerialUSB);

uint8_t configureFOC(void);
uint8_t configureCAN(void);
uint8_t configureDFU(void);

void doMotor(char *cmd) { commander.motor(&motor, cmd); }

void setup() {
  pinMode(USER_LED, OUTPUT);
  pinMode(USER_BUTTON, INPUT);

  SerialUSB.begin();

  Serial.println(configureFOC() == 1 ? "SFOC successfully init."
                                     : "SFOC failed to init.");
  Serial.println(configureCAN() == 1 ? "CAN successfully init."
                                     : "CAN failed to init.");
  Serial.println(configureDFU() == 1 ? "DFU successfully init."
                                     : "DFU failed to init.");
}

void loop() {
  motor.loopFOC();
  motor.move();
  commander.run();

  // if(pendingFrame){
  //   commander.run((char*)sfocCmdStr);
  //   pendingFrame = 0;
  // }
  // else{
  //   commander.run();
  // }

#ifdef HAS_MONITOR
  motor.monitor();
#endif
}

uint8_t configureFOC() {
  commander.add('M', doMotor, "motor");
  commander.verbose = VerboseMode::machine_readable;

#ifdef SIMPLEFOC_STM_DEBUG
  SimpleFOCDebug::enable(&Serial);
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
  motor.LPF_angle.Tf = 0;  // try to avoid

  // Motor initialization.
  motor.voltage_sensor_align = 2;
  motor.current_limit = 0.5;
  motor.velocity_limit = 20;
  motor.controller = MotionControlType::velocity;
  motor.foc_modulation = FOCModulationType::SinePWM;

// Monitor initialization
#ifdef HAS_MONITOR
  motor.useMonitoring(Serial);
  motor.monitor_start_char = 'M';
  motor.monitor_end_char = 'M';
  motor.monitor_downsample = 250;
#endif

  motor.linkSensor(&enc);
  motor.linkDriver(&driver);
  motor.init();

  motor.target = 20;

  return motor.initFOC();
}

uint8_t configureCAN() { return 0; }

uint8_t configureDFU() { return 1; }
