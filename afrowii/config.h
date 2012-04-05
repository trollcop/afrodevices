/*******************************/
/****CONFIGURABLE PARAMETERS****/
/*******************************/

#ifndef BRUSHED
/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
   This is the minimum value that allow motors to run at a idle speed  */
//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
//#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
// #define MINTHROTTLE 1180
#define MINTHROTTLE 1120
/* this is the value for the ESCs when they are not armed
   in some cases, this value must be lowered down to 900 for some specific ESCs */
#define MINCOMMAND 1000
/* this is the maximum value for the ESCs at full power
   this value can be increased up to 2000 */
#define MAXTHROTTLE 2000
// #define DIGITAL_SERVO      // If high-speed (200hz) refresh is needed on tail servo or for camera stabilization, define this. otherwise 50hz is used.

#else
/* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
   This is the minimum value that allow motors to run at a idle speed  */
//#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
//#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
//#define MINTHROTTLE 1220
#define MINTHROTTLE 100
/* this is the value for the ESCs when they are not armed
   in some cases, this value must be lowered down to 900 for some specific ESCs */
#define MINCOMMAND 0
/* this is the maximum value for the ESCs at full power
   this value can be increased up to 2000 */
#define MAXTHROTTLE 2000
// #define DIGITAL_SERVO      // If high-speed (200hz) refresh is needed on tail servo or for camera stabilization, define this. otherwise 50hz is used.
#endif

#define YAW_DIRECTION 1		// if you want to reverse the yaw correction direction
//#define YAW_DIRECTION -1

//#define I2C_SPEED 100000L	//100kHz normal mode, this value must be used for a genuine WMP
#define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones

// ******************
// rc functions
// ******************
#if 1
#define MINCHECK 1100
#define MAXCHECK 1900
#else
#define MINCHECK 1150
#define MAXCHECK 1850
#endif

//****** advanced users settings   *************

/* Pseudo-derivative conrtroller for level mode (experimental)
   Additional information: http://wbb.multiwii.com/viewtopic.php?f=8&t=503 */
//#define LEVEL_PDF

/* introduce a deadband around the stick center
   Must be greater than zero, comment if you dont want a deadband on roll, pitch and yaw */
//#define DEADBAND 6

/* Failsave settings - added by MIS
   Failsafe check pulse on THROTTLE channel. If the pulse is OFF (on only THROTTLE or on all channels) the failsafe procedure is initiated.
   After FAILSAVE_DELAY time of pulse absence, the level mode is on (if ACC or nunchuk is avaliable), PITCH, ROLL and YAW is centered
   and THROTTLE is set to FAILSAVE_THR0TTLE value. You must set this value to descending about 1m/s or so for best results. 
   This value is depended from your configuration, AUW and some other params. 
   Next, afrer FAILSAVE_OFF_DELAY the copter is disarmed, and motors is stopped.
   If RC pulse coming back before reached FAILSAVE_OFF_DELAY time, after the small quard time the RC control is returned to normal.
   If you use serial sum PPM, the sum converter must completly turn off the PPM SUM pusles for this FailSafe functionality.*/
#define FAILSAFE		// Alex: comment this line if you want to deactivate the failsafe function
#define FAILSAVE_DELAY     10	// Guard time for failsafe activation after signal lost. 1 step = 0.1sec - 1sec in example
#define FAILSAVE_OFF_DELAY 200	// Time for Landing before motors stop in 0.1sec. 1 step = 0.1sec - 20sec in example
#define FAILSAVE_THR0TTLE  (MINTHROTTLE + 200)	// Throttle level used for landing - may be relative to MINTHROTTLE - as in this case

/* EXPERIMENTAL !!
  contribution from Luis Correia
  see http://www.multiwii.com/forum/viewtopic.php?f=18&t=828
  It uses a Bluetooth Serial module as the input for controlling the device via an Android application
  As with the SPEKTRUM option, is not possible to use the configuration tool on a mini or promini. */
//#define BTSERIAL

/* The following lines apply only for a pitch/roll tilt stabilization system
   On promini board, it is not compatible with config with 6 motors or more
   Uncomment the first line to activate it 
   These constants are also used in MULTITYPE_GIMBAL, in this case no #define is needed */
// #define SERVO_TILT
#define TILT_PITCH_MIN    1020	//servo travel min, don't set it below 1020
#define TILT_PITCH_MAX    2000	//servo travel max, max value=2000
#define TILT_PITCH_MIDDLE 1500	//servo neutral value
#define TILT_ROLL_MIN     1020
#define TILT_ROLL_MAX     2000
#define TILT_ROLL_MIDDLE  1500

/* if you use a specific sensor board:
   please submit any correction to this list.
     Note from Alex: I only own some boards
                     for other boards, I'm not sure, the info was gathered via rc forums, be cautious */
//#define FFIMUv1         // first 9DOF+baro board from Jussi, with HMC5843                   <- confirmed by Alex
//#define FFIMUv2         // second version of 9DOF+baro board from Jussi, with HMC5883       <- confirmed by Alex
//#define FREEIMUv1       // v0.1 & v0.2 & v0.3 version of 9DOF board from Fabio
//#define FREEIMUv03      // FreeIMU v0.3 and v0.3.1
//#define FREEIMUv035     // FreeIMU v0.3.5 no baro
//#define FREEIMUv035_MS  // FreeIMU v0.3.5_MS                                                <- confirmed by Alex
//#define FREEIMUv035_BMP // FreeIMU v0.3.5_MS
//#define PIPO            // 9DOF board from erazz
//#define QUADRINO        // full FC board 9DOF+baro board from witespy                       <- confirmed by Alex
//#define ALLINONE        // full FC board or standalone 9DOF+baro board from CSG_EU
//#define AEROQUADSHIELDv2
//#define ATAVRSBIN1      // Atmel 9DOF (Contribution by EOSBandi). The board requires 3.3V power.
//#define SIRIUS          // Sirius Navigator IMU                                             <- confirmed by Alex


//if you use independent sensors
//leave it commented it you already checked a specific board above
/* I2C gyroscope */
//#define ITG3200
//#define L3G4200D

/* I2C accelerometer */
//#define ADXL345
//#define BMA020
//#define BMA180
//#define LIS3LV02
//#define LSM303DLx_ACC

/* I2C barometer */
//#define BMP085
//#define MS561101BA

/* I2C magnetometer */
//#define HMC5843
//#define HMC5883
//#define AK8975

/* ADC accelerometer */// for 5DOF from sparkfun, uses analog PIN A1/A2/A3
//#define ADCACC

/* ITG3200 & ITG3205 Low pass filter setting. In case you cannot eliminate all vibrations to the Gyro, you can try
   to decrease the LPF frequency, only one step per try. As soon as twitching gone, stick with that setting.
   It will not help on feedback wobbles, so change only when copter is randomly twiching and all dampening and
   balancing options ran out. Uncomment only one option!
   IMPORTANT! Change low pass filter setting changes PID behaviour, so retune your PID's after changing LPF.*/
//#define ITG3200_LPF_256HZ     // This is the default setting, no need to uncomment, just for reference
//#define ITG3200_LPF_188HZ
//#define ITG3200_LPF_98HZ
//#define ITG3200_LPF_42HZ
//#define ITG3200_LPF_20HZ
//#define ITG3200_LPF_10HZ      // Use this only in extreme cases, rather change motors and/or props

#define SERIAL_SUM_PPM

#define VBAT			// comment this line to suppress the vbat code
#define VBATFREQ 6        // to read battery voltage - keep equal to PSENSORFREQ (6) unless you know what you are doing

/* This is the speed of the serial interface. 115200 kbit/s is the best option for a USB connection.*/
#define SERIAL_COM_SPEED 115200

/* motors will not spin when the throttle command is in low position
   this is an alternative method to stop immediately the motors */
//#define MOTOR_STOP

/* some radios have not a neutral point centered on 1500. can be changed here */
#define MIDRC 1500

/* you can change the tricopter servo travel here */
#define TRI_YAW_CONSTRAINT_MIN 1020
#define TRI_YAW_CONSTRAINT_MAX 2000
#define TRI_YAW_MIDDLE 1500

/* Flying Wing: you can change change servo orientation and servo min/max values here */
/* valid for all flight modes, even passThrough mode */
/* need to setup servo directions here; no need to swap servos amongst channels at rx */ 
#define PITCH_DIRECTION_L 1 // left servo - pitch orientation
#define PITCH_DIRECTION_R -1  // right servo - pitch orientation (opposite sign to PITCH_DIRECTION_L, if servos are mounted in mirrored orientation)
#define ROLL_DIRECTION_L 1 // left servo - roll orientation
#define ROLL_DIRECTION_R 1  // right servo - roll orientation  (same sign as ROLL_DIRECTION_L, if servos are mounted in mirrored orientation)
#define WING_LEFT_MID  1500 // left servo center pos. - use this for trim
#define WING_RIGHT_MID 1500 // right servo center pos. - use this for trim
#define WING_LEFT_MIN  1020 // limit servo travel range must be inside [1020;2000]
#define WING_LEFT_MAX  2000 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MIN 1020 // limit servo travel range must be inside [1020;2000]
#define WING_RIGHT_MAX 2000 // limit servo travel range must be inside [1020;2000]
