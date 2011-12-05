#ifdef _MSC_VER
#define __CSMC__
#endif
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

#if defined(STM8) && defined(AFROV2)
#define ADXL345SPI              // ADXL345 on SPI
#define ADCGYRO                 // Analog Gyro
#endif

#if defined(STM8) && defined(AFROV3)
#define MPU6000SPI              // MPU6000 on SPI providing 6DOF + MAG
#endif

#if defined(STM8) && defined(AFROI2C)
#define ALLINONE                // CSG_EU's sensor board w/LLC
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
#define BMA180_ADDRESS 0x82
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

#if defined(SIRIUS)
#define ITG3200
#define BMA180
#define BMP085
#define HMC5883
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = -Y;  magADC[PITCH] = X; magADC[YAW]  = Z;}
#define BMA180_ADDRESS 0x80
#define ITG3200_ADDRESS 0XD0
#endif


#if defined(ADXL345) || defined(ADXL345SPI) || defined(BMA020) || defined(BMA180) || defined(NUNCHACK) || defined(ADCACC) || defined(MPU6000SPI)
#define ACC 1
#else
#define ACC 0
#endif

#if defined(HMC5883) || defined(HMC5843) || defined(AK8975) // || defined(MPU6000SPI)
#define MAG 1
#else
#define MAG 0
#endif

#if defined(ITG3200) || defined(L3G4200D) || defined(ADCGYRO) || defined(MPU6000SPI)
#define GYRO 1
#else
#define GYRO 0
#endif

#if defined(BMP085) || defined(MS561101BA) || defined(MS561101BASPI)
#define BARO 1
#else
#define BARO 0
#endif

#if defined(STM8)
#ifndef AFROI2C
#define LEDPIN_PINMODE             GPIO_Init(GPIOD, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_FAST);    // LED
#define LEDPIN_TOGGLE              GPIO_WriteReverse(GPIOD, GPIO_PIN_7);
#define LEDPIN_OFF                 GPIO_WriteHigh(GPIOD, GPIO_PIN_7);
#define LEDPIN_ON                  GPIO_WriteLow(GPIOD, GPIO_PIN_7);
#define BUZZERPIN_PINMODE          GPIO_Init(GPIOF, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);
#define BUZZERPIN_ON               GPIO_WriteHigh(GPIOF, GPIO_PIN_4);
#define BUZZERPIN_OFF              GPIO_WriteLow(GPIOF, GPIO_PIN_4);
#define POWERPIN_PINMODE           ;
#define POWERPIN_ON                ;
#define POWERPIN_OFF               ;
#define I2C_PULLUPS_ENABLE         ;
#define I2C_PULLUPS_DISABLE        ;
#define PINMODE_LCD                ;
#define LCDPIN_OFF                 ;
#define LCDPIN_ON                  ;
#define STABLEPIN_PINMODE          ;
#define STABLEPIN_ON               ;
#define STABLEPIN_OFF              ;
#define DIGITAL_SERVO_TRI_PINMODE  ;
#define DIGITAL_SERVO_TRI_HIGH     ;
#define DIGITAL_SERVO_TRI_LOW      ;
#define DIGITAL_TILT_PITCH_PINMODE ;
#define DIGITAL_TILT_PITCH_HIGH    ;
#define DIGITAL_TILT_PITCH_LOW     ;
#define DIGITAL_TILT_ROLL_PINMODE  ;
#define DIGITAL_TILT_ROLL_HIGH     ;
#define DIGITAL_TILT_ROLL_LOW      ;
#define DIGITAL_BI_LEFT_PINMODE    ;
#define DIGITAL_BI_LEFT_HIGH       ;
#define DIGITAL_BI_LEFT_LOW        ;
#define PPM_PIN_INTERRUPT          ;
#define DIGITAL_CAM_PINMODE        ;
#define DIGITAL_CAM_HIGH           ;
#define DIGITAL_CAM_LOW            ;
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
#else /* AFROI2C pinout */
#define LEDPIN_PINMODE             GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);    // LED
#define LEDPIN_TOGGLE              GPIO_WriteReverse(GPIOE, GPIO_PIN_5);
#define LEDPIN_OFF                 GPIO_WriteHigh(GPIOE, GPIO_PIN_5);
#define LEDPIN_ON                  GPIO_WriteLow(GPIOE, GPIO_PIN_5);
#define BUZZERPIN_PINMODE          ;
#define BUZZERPIN_ON               ;
#define BUZZERPIN_OFF              ;
#define POWERPIN_PINMODE           ;
#define POWERPIN_ON                ;
#define POWERPIN_OFF               ;
#define I2C_PULLUPS_ENABLE         ;
#define I2C_PULLUPS_DISABLE        ;
#define PINMODE_LCD                ;
#define LCDPIN_OFF                 ;
#define LCDPIN_ON                  ;
#define STABLEPIN_PINMODE          ;
#define STABLEPIN_ON               ;
#define STABLEPIN_OFF              ;
#define DIGITAL_SERVO_TRI_PINMODE  ;
#define DIGITAL_SERVO_TRI_HIGH     ;
#define DIGITAL_SERVO_TRI_LOW      ;
#define DIGITAL_TILT_PITCH_PINMODE ;
#define DIGITAL_TILT_PITCH_HIGH    ;
#define DIGITAL_TILT_PITCH_LOW     ;
#define DIGITAL_TILT_ROLL_PINMODE  ;
#define DIGITAL_TILT_ROLL_HIGH     ;
#define DIGITAL_TILT_ROLL_LOW      ;
#define DIGITAL_BI_LEFT_PINMODE    ;
#define DIGITAL_BI_LEFT_HIGH       ;
#define DIGITAL_BI_LEFT_LOW        ;
#define PPM_PIN_INTERRUPT          ;
#define DIGITAL_CAM_PINMODE        ;
#define DIGITAL_CAM_HIGH           ;
#define DIGITAL_CAM_LOW            ;
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
#endif /* AFROI2C */
#endif


#if defined(GPS)
#define GPSPRESENT 1
#else
#define GPSPRESENT 0
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
