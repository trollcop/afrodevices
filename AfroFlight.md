# Introduction #

Simple, small, fun flight controller.
Two available firmware choices:

  1. Custom rate-mode (heading hold) firmware similar to kkboard - no hover/autolevel.
  1. Port of MultiWii to STM8S. All the same features of MultiWii 1.9 (and following the code for further changes).

## Hardware ##

  * STM8S105 CPU (8bit CISC, 16MHz)
  * Invensense IDG500 / ISZ500 Gyro
  * Analog ADXL345 digital SPI accelerometer
  * PPM (up to 12 channels) RC input
  * PWM (50..450Hz) motor output for up to 6 motors
  * I2C header for possible expansion or motor control
  * UART (firmware update, telemetry)
  * Battery voltage monitoring and low-voltage alarm
  * Buzzer for alarm/user notification
  * Status LED

## Status ##

It works.

## Board revisions ##

AfroFlightRev0 (development/testing version)

AfroFlightRev1 (used to make corrections for rev0, never manufactured)

AfroFlightRev2 (final production version)