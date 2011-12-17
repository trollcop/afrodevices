#pragma once

/* This header configures hardware
 * Supported platforms:
 * STM8         STMicro STM8S105 series
 *  - AFROV2    AbuseMarK "Naze" FC
 *  - AFROV3    Next-generation MPU6000 FC
 *  - AFROI2C   AbuseMarK Tokyo Drift I2C Converter as FC w/sensor board of choice

 * STM32F1      STMicro STM32F103 series
 *  - STM32_CC  OpenPilot CopterControl

 * STM32F4      STMicro STM32F40x series
 *  - No targets currently
 */

#define STM8
#ifdef STM8
#define AFROV2                  // AfroFlight rev2 (ADXL345 on SPI, Invensense Analog gyros on ADC)
// #define AFROV3                  // AfroFlight rev3 (MPU6000 on SPI, HMC5883L behind it)
// #define AFROI2C                 // AfroI2C converter + CSG_EU's sensor bob w/LLC
#endif

// #define STM32F1
#ifdef STM32F1
#define STM32_CC
#define ATAVRSBIN1
#endif


/* ======================== No user-serviceable parts below ======================== */
#ifdef STM8
/* Includes for STM8 */
#include "stm8s.h"
#endif


#ifdef STM32F1
/* Includes for STM32F1xx */
#include <stdint.h>
#include <stdlib.h>
#include <cross_studio_io.h>
#include "misc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "usb/usb_cdcacm.h"
#include "usb/usb.h"
#include <string.h>

#define digitalHi(p, i)    { p->BSRR = i; }
#define digitalLo(p, i)    { p->BRR = i; }

#endif
