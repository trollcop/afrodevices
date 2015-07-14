# Introduction #

Simple, small, fun flight controller.
Several available firmware choices:

  1. MultiWii-dev port to STM32 ("baseflight" @ github - https://github.com/multiwii/baseflight)
  1. Written from scratch "correct" firmware using various attitude estimators, Kalman filtering, etc ("baseflightplus" in SVN)
  1. Chinese 'FreeFlight' firmware. Autolevel/gyro, limited functionality. Supported on rev0 to rev3 hardware. Not supported on rev4 and above.

## Hardware ##

  * 36x36mm 2 layer pcb, 30.5mm mounting pattern
  * STM32F103CxT6 CPU (32bit ARM Cortex M3, 72MHz, 64K/128K flash)
  * Invensense MPU3050 3-axis gyro (rev0 to rev3)
  * Invensense MPU6050 3-axis gyro (rev4+)
  * Analog ADXL345 digital accelerometer (rev0 to rev3)
  * FreeScale MMA8452Q digital accelerometer (rev3 to rev4)
  * Honeywell HMC5883L digital compass
  * Bosch-Sensortec BMP085 pressure sensor (rev0 to rev3)
  * MEAS-SPEC MS5611-01BA03 pressure sensor (rev4+)
  * 6 + 8 PWM I/O can remap as input or output for RC/CPPM/Motors/Servos
  * second UART accessible for Spektrum Satellite RX or GPS
  * 16Mbit SPI flash memory (rev5)
  * Built in FrSky Telemetry converter (rev5)
  * CPPM (up to 12 channels) RC input
  * 8 channel standard PWM RC input
  * Onboard USB connector for telemetry and firmware update (through-hole Mini-B  (rev0-rev4), through-hole microUSB (rev5)
  * One 3.3V and one 5V-tolerant GPIO (rev5)
  * PWM (50..32kHz) motor output for up to 6 motors, can be remapped with other pins for 8 motors + camera stabilization. Supports direct-drive brushed motors with additional hardware (FETs).
  * Battery voltage monitoring and low-voltage alarm
  * Buzzer for alarm/user notification
  * Max 5.5V power via servo connector (rev0-4, rev5acro), Max 16V power via servo connector (rev5)
  * 2 programmable status LEDs, 1 constant power LED.

## Status ##

It works.
Discussion forum at multiwii.com 32bit section:

http://www.multiwii.com/forum/viewforum.php?f=21

Current development source code:

https://github.com/multiwii/baseflight

RCGroups thread (now defunct):

http://www.rcgroups.com/forums/showthread.php?t=1595761

Manual:

http://www.abusemark.com/downloads/naze32_rev3.pdf

## What isn't Naze32 ##

Flip32, Nanj32 and similar stuff that's been based on the published rev4 schematic is not Naze32. You can generally tell by the soldering, pcb and silk  quality:

![http://i.imgur.com/3Cse4Wj.jpg](http://i.imgur.com/3Cse4Wj.jpg)

## Board revisions ##

AfroFlight rev6 (currently unavailable, ETA unknown)

![http://i.imgur.com/SOTMUfL.jpg](http://i.imgur.com/SOTMUfL.jpg)

AfroFlight32 rev5 (current production version)

![http://i.imgur.com/tZ7uQMF.jpg](http://i.imgur.com/tZ7uQMF.jpg)

AfroFlight32 rev4

![http://i.imgur.com/m231K.jpg](http://i.imgur.com/m231K.jpg)

AfroFlight32 rev3

![http://i.imgur.com/IhJTw.jpg](http://i.imgur.com/IhJTw.jpg)

AfroFlight32 rev0 (first production prototype)

![http://i.imgur.com/7GcTd.jpg](http://i.imgur.com/7GcTd.jpg)