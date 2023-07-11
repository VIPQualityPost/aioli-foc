#pragma once

// MOTOR DRIVER
#define U_PWM       PA8
#define U_EN        PB13
#define V_PWM       PA9
#define V_EN        PB14
#define W_PWM       PA10
#define W_EN        PB15

// MOTOR ENCODER (SPI1)
#define ENC_CIPO    PA6
#define ENC_SCK     PA5
#define ENC_CS      PA4

// CANBUS
#define CAN_TX      PB9
#define CAN_RX      PB8

// GC9AO1 TFT DISPLAY (SPI3)
#define TFT_COPI    PB5    
#define TFT_SCK     PC10
#define TFT_CS      PB3
#define TFT_DC      PC14
#define TFT_BL      PC13 // PWM TIM8CH4N

// GENERAL SPI (SPI3)
#define SPI3_CS     PB11 

// GENERAL UART
#define UART2_TX    PA2
#define UART2_RX    PA3

// GENERAL I2C
#define I2C1_SDA    PB7
#define I2C1_SCL    PA15

//MISC
#define USER_LED    PA7
#define USER_BUTTON PC4