#pragma once

/* This header configures hardware
 * Supported platforms:
 * STM8         STMicro STM8S105 series
 *  - AFROV2    AbuseMarK "Naze" FC
 *  - AFROV3    Next-generation MPU6000 FC
 *  - AFROI2C   AbuseMarK Tokyo Drift I2C Converter as FC w/sensor board of choice
 *  - ROME      ROME Brushed Board

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
// #define ROME_BRUSHED            // Brushed ROME
// #define ROME                    // Brushless ROME
#endif

/* ======================== No user-serviceable parts below ======================== */
#ifdef STM8
/* Includes for STM8 */
#include "stm8s.h"
#endif
