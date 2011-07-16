#pragma once

#include "stm8s.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "imu.h"
#include "mixer.h"
#include "rc.h"
#include "sensors.h"
#include "timing.h"
#include "uart.h"

#define ADC_GYRO_ROLL	(0)
#define ADC_GYRO_PITCH	(1)
#define ADC_GYRO_YAW	(2)
#define ADC_VOLTAGE	(3)
#define ADC_ACC_PITCH	(4)
#define ADC_ACC_ROLL	(5)
#define ADC_ACC_Z	(6)

#define DIR_PITCH       (0)
#define DIR_ROLL        (1)
#define DIR_YAW         (2)
#define DIR_ACC_PITCH   (3)
#define DIR_ACC_ROLL    (4)
#define DIR_ACC_Z       (5)

// Max number of motors/servos we support
#define MAX_MOTORS      (6)

// Motor PWM outputs
#define PULSE_PERIOD    (5000)
#define PULSE_1MS       (2000)
#define PULSE_2MS       (4000)

#define MAX_THROTTLE    (1000)                     // in us
#define MIN_THROTTLE    (114)                      // in us

#define LED_OFF		GPIO_WriteHigh(GPIOD, GPIO_PIN_7);
#define LED_ON		GPIO_WriteLow(GPIOD, GPIO_PIN_7);
#define LED_TOGGLE	GPIO_WriteReverse(GPIOD, GPIO_PIN_7);

#define BUZZ_OFF	GPIO_WriteLow(GPIOF, GPIO_PIN_4);
#define BUZZ_ON		GPIO_WriteHigh(GPIOF, GPIO_PIN_4);
#define BUZZ_TOGGLE	GPIO_WriteReverse(GPIOF, GPIO_PIN_4);

#define FLAG_ISSET(var, flag) ((var) & (flag))
#define FLAG_SET(var, flag) ((var) |= (flag))
#define FLAG_CLEAR(var, flag) ((var) &= (~(flag)))

// #define UNSET_FLAG(flag) (FCFlags &= (u8)(~(flag)))
// #define SET_FLAG(flag) (FCFlags |= (u8)(flag))
// #define ISSET_FLAG(flag) (FCFlags & ((u8)flag))

enum {
    FC_FLAG_LOWVOLTAGE          = 0x01
};

#define MAX_LIPO_CELL_VOLTAGE (43)
#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

enum GyroDirection { GYRO_NORMAL = 0, GYRO_REVERSED };
enum GyroArrayIndex { ROLL = 0, PITCH, YAW };
enum MotorIndex { MOTOR1 = 0, MOTOR2, MOTOR3, MOTOR4, MOTOR5, MOTOR6 };
enum FlightControlMode { FC_MODE_ACRO = 0, FC_MODE_HOVER, FC_MODE_DONGS };

typedef struct _Config {
    u8 ChannelMapping[6];               // Individual RC channels mapping
    u8 Mixer;                           // Selected Mixer (Tri,  Quad, etc)
    u8 VoltagePerCellMin;               // LIPO Voltage per cell
    u8 RollGyroDirection;               // Direction of Roll gyro
    u8 PitchGyroDirection;              // Direction of Pitch gyro
    u8 YawGyroDirection;                // Direction of Yaw gyro
    
    u8 RollPitchGyroGain;               // Divider for pitch/roll gyros
    u8 YawGyroGain;                     // Divider for yaw gyro
    
    u8 StickP;                          // Roll/Pitch stick P-term
    u8 StickD;                          // Roll/Pitch stick D-term
    u8 YawStickP;                       // Yaw-stick P-term
    
    u8 RollPitchStickGain;              // Divider for pitch/roll stick
    u8 YawStickGain;                    // Divider for yaw stick
    u8 YawP;                            // Yaw P
    u8 YawI;                            // Yaw I
} _Config;

extern _Config Config;
extern s16 Motors[MAX_MOTORS];

void Beep(u8 Count, u16 Length, u16 Delay);
