/*
Main header file for include files and defining macros
*/

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#define DEBUG
#define PPM
//#define ESC_CALIBRATE
//#define ACC_CALIBRATE
//#define GYRO_CALIBRATE
//#define MAG_CALIBRATE
//#define LEVEL_CALIBRATE

#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4

//#define LEDPIN 13

//SPI for communication with IMU
#define SPI_PORT SPI
#define CS_PIN 10   


#include <PWMServo.h>
#include <Fusion.h>
#include "ICM_20948.h"
#include <SPI.h>
#include <math.h>
#include <PulsePosition.h>
#include <SdFat.h>
#define PPM_PIN 18

