#include "stm8s.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _MSC_VER
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef short int16_t;
typedef unsigned long uint32_t;
typedef long int32_t;
#define __inline inline
#define __near 
#define __interrupt
#else /* _MSC_VER */
#define __inline @inline
#define __near @near
#define __interrupt @interrupt
#endif

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#if defined(STM8)
#define ATAVRSBIN1
// #define ADXL345SPI              // ADXL345 on SPI
// #define ADCGYRO                 // analog gyro
#endif

//please submit any correction to this list.
#if defined(FFIMUv1)
#define ITG3200
#define BMA180
#define BMP085
#define HMC5843
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
#define BMA180_ADDRESS 0x80
#define ITG3200_ADDRESS 0XD0
#endif

#if defined(FFIMUv2)
#define ITG3200
#define BMA180
#define BMP085
#define HMC5883
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
#define BMA180_ADDRESS 0x80
#define ITG3200_ADDRESS 0XD0
#endif

#if defined(FREEIMUv1)
#define ITG3200
#define ADXL345
#define HMC5843
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  -Y; accADC[PITCH]  = X; accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X;  gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X;  magADC[PITCH]  = Y; magADC[YAW]  = Z;}
#define ADXL345_ADDRESS 0xA6
#undef INTERNAL_I2C_PULLUPS
#endif

#if defined(FREEIMUv035) || defined(FREEIMUv035_MS) || defined(FREEIMUv035_BMP)
#define ITG3200
#define BMA180
#define HMC5883
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X;  gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y;  magADC[PITCH]  = X; magADC[YAW]  = Z;}
#undef INTERNAL_I2C_PULLUPS
#if defined(FREEIMUv035_MS)
#define MS561101BA
#elif defined(FREEIMUv035_BMP)
#define BMP085
#endif
#endif

#if defined(FREEIMUv03)
#define ITG3200
#define ADXL345			// this is actually an ADXL346 but that's just the same as ADXL345
#define HMC5883
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  -Y; accADC[PITCH]  = X; accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X;  gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  -Y;  magADC[PITCH]  = X; magADC[YAW]  = Z;}
#define ADXL345_ADDRESS 0xA6
#undef INTERNAL_I2C_PULLUPS
#endif

#if defined(PIPO)
#define L3G4200D
#define ADXL345
#define HMC5883
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -Y; accADC[PITCH]  = X; accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
#define ADXL345_ADDRESS 0xA6
#endif

#if defined(QUADRINO)
#define ITG3200
#define BMA180
#define BMP085
#define HMC5883
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = Z;}
#define BMA180_ADDRESS 0x80
#define ITG3200_ADDRESS 0XD0
#endif

#if defined(ALLINONE)
#define ITG3200
#define BMA180
#define BMP085
#define HMC5883
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y; magADC[PITCH]  = X; magADC[YAW]  = Z;}
#endif

#if defined(AEROQUADSHIELDv2)	// to confirm
#define ITG3200
#define BMA180
#define BMP085
#define HMC5843
#define ACC_ORIENTATION(X, Y, Z) {accADC[ROLL] = -Y; accADC[PITCH] = X; accADC[YAW] = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] = X; gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z) {magADC[ROLL] = -Y; magADC[PITCH] = X; magADC[YAW] = Z;}
#define ITG3200_ADDRESS 0XD2
#endif

#if defined(ATAVRSBIN1)
#define ITG3200
#define BMA020			//Actually it's a BMA150, but this is a drop in replacement for the discountinued BMA020
#define AK8975
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  Y; accADC[PITCH]  = -X; accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] =  X; gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
#endif



#if defined(ADXL345) || defined(ADXL345SPI) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(ADCACC)
#define ACC 1
#else
#undef ACC
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975)
#define MAG 1
#else
#undef MAG
#endif

#if defined(ITG3200) || defined(L3G4200D) || defined(ADCGYRO)
#define GYRO 1
#else
#undef GYRO
#endif

#if defined(BMP085) || defined(MS561101BA)
#define BARO 1
#else
#undef BARO
#endif

#if defined(STM8)
#define LEDPIN_PINMODE             GPIO_Init(GPIOD, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_FAST);    // LED
#define LEDPIN_SWITCH
#define LEDPIN_OFF                 GPIO_WriteHigh(GPIOD, GPIO_PIN_7);
#define LEDPIN_ON                  GPIO_WriteLow(GPIOD, GPIO_PIN_7);
#define LEDPIN_TOGGLE              GPIO_WriteReverse(GPIOD, GPIO_PIN_7);
#define BUZZERPIN_PINMODE          GPIO_Init(GPIOF, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);
#define BUZZERPIN_ON               GPIO_WriteHigh(GPIOF, GPIO_PIN_4);
#define BUZZERPIN_OFF              GPIO_WriteLow(GPIOF, GPIO_PIN_4);
#define POWERPIN_PINMODE           ;
#define POWERPIN_ON                ;
#define POWERPIN_OFF               ;
#define I2C_PULLUPS_ENABLE         ;
#define I2C_PULLUPS_DISABLE        ;
#define PINMODE_LCD                pinMode(0, OUTPUT);
#define LCDPIN_OFF                 PORTD &= ~1;
#define LCDPIN_ON                  PORTD |= 1;
#define STABLEPIN_PINMODE          ;
#define STABLEPIN_ON               ;
#define STABLEPIN_OFF              ;
#define DIGITAL_SERVO_TRI_PINMODE  pinMode(3,OUTPUT);	//also right servo for BI COPTER
#define DIGITAL_SERVO_TRI_HIGH     PORTD |= 1<<3;
#define DIGITAL_SERVO_TRI_LOW      PORTD &= ~(1<<3);
#define DIGITAL_TILT_PITCH_PINMODE pinMode(A0,OUTPUT);
#define DIGITAL_TILT_PITCH_HIGH    PORTC |= 1<<0;
#define DIGITAL_TILT_PITCH_LOW     PORTC &= ~(1<<0);
#define DIGITAL_TILT_ROLL_PINMODE  pinMode(A1,OUTPUT);
#define DIGITAL_TILT_ROLL_HIGH     PORTC |= 1<<1;
#define DIGITAL_TILT_ROLL_LOW      PORTC &= ~(1<<1);
#define DIGITAL_BI_LEFT_PINMODE    pinMode(11,OUTPUT);
#define DIGITAL_BI_LEFT_HIGH       PORTB |= 1<<3;
#define DIGITAL_BI_LEFT_LOW        PORTB &= ~(1<<3);
#define PPM_PIN_INTERRUPT          ;
#define MOTOR_ORDER                9,10,11,3,6,5	//for a quad+: rear,right,left,front
#define DIGITAL_CAM_PINMODE        pinMode(A2,OUTPUT);
#define DIGITAL_CAM_HIGH           PORTC |= 1<<2;
#define DIGITAL_CAM_LOW            PORTC &= ~(1<<2);
//RX PIN assignment inside the port //for PORTD
#define THROTTLEPIN                2
#define ROLLPIN                    4
#define PITCHPIN                   5
#define YAWPIN                     6
#define AUX1PIN                    7
#define AUX2PIN                    7	//unused just for compatibility with MEGA
#define CAM1PIN                    7	//unused just for compatibility with MEGA
#define CAM2PIN                    7	//unused just for compatibility with MEGA
#define ISR_UART                   ISR(USART_UDRE_vect)
#define V_BATPIN                   3	// Analog PIN 3
#define PSENSORPIN                 2	// Analog PIN 2
#endif
#if defined(PROMINI)
#define LEDPIN_PINMODE             pinMode (13, OUTPUT);
#define LEDPIN_SWITCH              PINB |= 1<<5;	//switch LEDPIN state (digital PIN 13)
#define LEDPIN_OFF                 PORTB &= ~(1<<5);
#define LEDPIN_ON                  PORTB |= (1<<5);
#define BUZZERPIN_PINMODE          pinMode (8, OUTPUT);
#define BUZZERPIN_ON               PORTB |= 1;
#define BUZZERPIN_OFF              PORTB &= ~1;
#define POWERPIN_PINMODE           pinMode (12, OUTPUT);
#define POWERPIN_ON                PORTB |= 1<<4;
#define POWERPIN_OFF               PORTB &= ~(1<<4);	//switch OFF WMP, digital PIN 12
#define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;	// PIN A4&A5 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
#define PINMODE_LCD                pinMode(0, OUTPUT);
#define LCDPIN_OFF                 PORTD &= ~1;
#define LCDPIN_ON                  PORTD |= 1;
#define STABLEPIN_PINMODE          ;
#define STABLEPIN_ON               ;
#define STABLEPIN_OFF              ;
#define DIGITAL_SERVO_TRI_PINMODE  pinMode(3,OUTPUT);	//also right servo for BI COPTER
#define DIGITAL_SERVO_TRI_HIGH     PORTD |= 1<<3;
#define DIGITAL_SERVO_TRI_LOW      PORTD &= ~(1<<3);
#define DIGITAL_TILT_PITCH_PINMODE pinMode(A0,OUTPUT);
#define DIGITAL_TILT_PITCH_HIGH    PORTC |= 1<<0;
#define DIGITAL_TILT_PITCH_LOW     PORTC &= ~(1<<0);
#define DIGITAL_TILT_ROLL_PINMODE  pinMode(A1,OUTPUT);
#define DIGITAL_TILT_ROLL_HIGH     PORTC |= 1<<1;
#define DIGITAL_TILT_ROLL_LOW      PORTC &= ~(1<<1);
#define DIGITAL_BI_LEFT_PINMODE    pinMode(11,OUTPUT);
#define DIGITAL_BI_LEFT_HIGH       PORTB |= 1<<3;
#define DIGITAL_BI_LEFT_LOW        PORTB &= ~(1<<3);
#define PPM_PIN_INTERRUPT          attachInterrupt(0, rxInt, RISING);	//PIN 0
#define MOTOR_ORDER                9,10,11,3,6,5	//for a quad+: rear,right,left,front
#define DIGITAL_CAM_PINMODE        pinMode(A2,OUTPUT);
#define DIGITAL_CAM_HIGH           PORTC |= 1<<2;
#define DIGITAL_CAM_LOW            PORTC &= ~(1<<2);
  //RX PIN assignment inside the port //for PORTD
#define THROTTLEPIN                2
#define ROLLPIN                    4
#define PITCHPIN                   5
#define YAWPIN                     6
#define AUX1PIN                    7
#define AUX2PIN                    7	//unused just for compatibility with MEGA
#define CAM1PIN                    7	//unused just for compatibility with MEGA
#define CAM2PIN                    7	//unused just for compatibility with MEGA
#define ISR_UART                   ISR(USART_UDRE_vect)
#define V_BATPIN                   A3	// Analog PIN 3
#define PSENSORPIN                 A2	// Analog PIN 2
#endif
#if defined(MEGA)
#define LEDPIN_PINMODE             pinMode (13, OUTPUT);pinMode (30, OUTPUT);
#define LEDPIN_SWITCH              PINB  |= (1<<7); PINC  |= (1<<7);
#define LEDPIN_ON                  PORTB |= (1<<7); PORTC |= (1<<7);
#define LEDPIN_OFF                 PORTB &= ~(1<<7);PORTC &= ~(1<<7);
#define BUZZERPIN_PINMODE          pinMode (32, OUTPUT);
#define BUZZERPIN_ON               PORTC |= 1<<5;
#define BUZZERPIN_OFF              PORTC &= ~(1<<5);
#define POWERPIN_PINMODE           pinMode (37, OUTPUT);
#define POWERPIN_ON                PORTC |= 1<<0;
#define POWERPIN_OFF               PORTC &= ~(1<<0);
#define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;	// PIN 20&21 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
#define PINMODE_LCD                pinMode(0, OUTPUT);
#define LCDPIN_OFF                 PORTE &= ~1;	//switch OFF digital PIN 0
#define LCDPIN_ON                  PORTE |= 1;	//switch OFF digital PIN 0
#define STABLEPIN_PINMODE          pinMode (31, OUTPUT);
#define STABLEPIN_ON               PORTC |= 1<<6;
#define STABLEPIN_OFF              PORTC &= ~(1<<6);
#define DIGITAL_SERVO_TRI_PINMODE  pinMode(2,OUTPUT);	//PIN 2 //also right servo for BI COPTER
#define DIGITAL_SERVO_TRI_HIGH     PORTE |= 1<<4;
#define DIGITAL_SERVO_TRI_LOW      PORTE &= ~(1<<4);
#define DIGITAL_TILT_PITCH_PINMODE pinMode(34,OUTPUT);pinMode(44,OUTPUT);	// 34 + 44
#define DIGITAL_TILT_PITCH_HIGH    PORTC |= 1<<3;PORTL |= 1<<5;
#define DIGITAL_TILT_PITCH_LOW     PORTC &= ~(1<<3);PORTL &= ~(1<<5);
#define DIGITAL_TILT_ROLL_PINMODE  pinMode(35,OUTPUT);pinMode(45,OUTPUT);	// 35 + 45
#define DIGITAL_TILT_ROLL_HIGH     PORTC |= 1<<2;PORTL |= 1<<4;
#define DIGITAL_TILT_ROLL_LOW      PORTC &= ~(1<<2);PORTL &= ~(1<<4);
#define DIGITAL_BI_LEFT_PINMODE    pinMode(6,OUTPUT);
#define DIGITAL_BI_LEFT_HIGH       PORTH |= 1<<3;
#define DIGITAL_BI_LEFT_LOW        PORTH &= ~(1<<3);
#define PPM_PIN_INTERRUPT          attachInterrupt(4, rxInt, RISING);	//PIN 19, also used for Spektrum satellite option
#define MOTOR_ORDER                3,5,6,2,7,8,9,10	//for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
#define DIGITAL_CAM_PINMODE        pinMode(33,OUTPUT); pinMode(46,OUTPUT);	// 33 + 46
#define DIGITAL_CAM_HIGH           PORTC |= 1<<4;PORTL |= 1<<3;
#define DIGITAL_CAM_LOW            PORTC &= ~(1<<4);PORTL &= ~(1<<3);
  //RX PIN assignment inside the port //for PORTK
#define THROTTLEPIN                0	//PIN 62 =  PIN A8
#define ROLLPIN                    1	//PIN 63 =  PIN A9
#define PITCHPIN                   2	//PIN 64 =  PIN A10
#define YAWPIN                     3	//PIN 65 =  PIN A11
#define AUX1PIN                    4	//PIN 66 =  PIN A12
#define AUX2PIN                    5	//PIN 67 =  PIN A13
#define CAM1PIN                    6	//PIN 68 =  PIN A14
#define CAM2PIN                    7	//PIN 69 =  PIN A15
#define ISR_UART                   ISR(USART0_UDRE_vect)
#define V_BATPIN                   A0	// Analog PIN 3
#define PSENSORPIN                 A2	// Analog PIN 2
#endif

#if defined(POWERMETER)
#ifndef VBAT
#error "to use powermeter, you must also define and configure VBAT"
#endif
#endif
#ifdef LCD_TELEMETRY_AUTO
#ifndef LCD_TELEMETRY
#error "to use automatic telemetry, you MUST also define and configure LCD_TELEMETRY"
#endif
#endif

#if defined(TRI)
#define MULTITYPE 1
#elif defined(QUADP)
#define MULTITYPE 2
#elif defined(QUADX)
#define MULTITYPE 3
#elif defined(BI)
#define MULTITYPE 4
#elif defined(GIMBAL)
#define MULTITYPE 5
#elif defined(Y6)
#define MULTITYPE 6
#elif defined(HEX6)
#define MULTITYPE 7
#elif defined(FLYING_WING)
#define MULTITYPE 8
#elif defined(Y4)
#define MULTITYPE 9
#elif defined(HEX6X)
#define MULTITYPE 10
#elif defined(OCTOX8)
#define MULTITYPE 11
#elif defined(OCTOFLATP)
#define MULTITYPE 11		//the GUI is the same for all 8 motor configs
#elif defined(OCTOFLATX)
#define MULTITYPE 11		//the GUI is the same for all 8 motor configs
#endif
