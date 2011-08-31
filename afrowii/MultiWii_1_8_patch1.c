/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
August  2011     V1.8
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include "config.h"
#include "def.h"
#include "sysdep.h"

#define   VERSION  18

/*********** RC alias *****************/
#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5
#define CAMPITCH   6
#define CAMROLL    7

#define PIDALT     3
#define PIDVEL     4
#define PIDLEVEL   5
#define PIDMAG     6

#define BOXACC      0
#define BOXBARO     1
#define BOXMAG      2
#define BOXCAMSTAB  3
#define BOXCAMTRIG  4
#define BOXARM      5

static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint16_t cycleTime = 0;	// this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;	// the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint8_t calibratingM = 0;
static uint16_t calibratingG;
static uint8_t armed = 0;
static int16_t acc_1G;		// this is the 1G measured acceleration
static uint8_t nunchuk = 0;
static uint8_t accMode = 0;	// if level mode is a activated
static uint8_t magMode = 0;	// if compass heading hold is a activated
static uint8_t baroMode = 0;	// if altitude hold is activated
static int16_t gyroADC[3], accADC[3], magADC[3];
static int16_t accSmooth[3];	// projection of smoothed and normalized gravitation force vector on x/y/z axis, as measured by accelerometer
static int16_t accTrim[2] = { 0, 0 };
static int16_t heading, magHold;
static uint8_t calibratedACC = 0;
static uint8_t vbat;		// battery voltage in 0.1V steps
static uint8_t okToArm = 0;
static uint8_t rcOptions;
static int32_t pressure = 0;
static float BaroAlt = 0.0f;
static float EstVelocity = 0.0f;
static float EstAlt = 0.0f;

#ifdef STM8
/* 0:Pitch 1:Roll 2:Yaw 3:Battery Voltage 4:AX 5:AY 6:AZ */
static volatile s16 sensorInputs[7] = { 0, };
#endif

//for log
static uint16_t cycleTimeMax = 0;	// highest ever cycle timen
static uint16_t cycleTimeMin = 65535;	// lowest ever cycle timen
static uint16_t powerMax = 0;	// highest ever current
static uint16_t powerAvg = 0;	// last known current

// **********************
// power meter
// **********************
#define PMOTOR_SUM 8		// index into pMeter[] for sum
static uint32_t pMeter[PMOTOR_SUM + 1];	//we use [0:7] for eight motors,one extra for sum
static uint8_t pMeterV;		// dummy to satisfy the paramStruct logic in ConfigurationLoop()
static uint32_t pAlarm;		// we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
static uint8_t powerTrigger1 = 0;	// trigger for alarm based on power consumption

// **********************
// telemetry
// **********************
static uint8_t telemetry = 0;
static uint8_t telemetry_auto = 0;

// ******************
// rc functions
// ******************
#define MINCHECK 1100
#define MAXCHECK 1900

volatile int16_t failsafeCnt = 0;

static int16_t rcData[8];	// interval [1000;2000]
static int16_t rcCommand[4];	// interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 

static uint8_t rcRate8;
static uint8_t rcExpo8;
static int16_t lookupRX[7];	//  lookup table for expo & RC rate

// **************
// gyro+acc IMU
// **************
static int16_t gyroData[3] = { 0, 0, 0 };
static int16_t gyroZero[3] = { 0, 0, 0 };
static int16_t accZero[3] = { 0, 0, 0 };
static int16_t magZero[3] = { 0, 0, 0 };
static int16_t angle[2] = { 0, 0 };	// absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

// *************************
// motor and servo functions
// *************************
static int16_t axisPID[3];
static int16_t motor[8];
static int16_t servo[4] = { 1500, 1500, 1500, 1500 };

// **********************
// EEPROM & LCD functions
// **********************
static uint8_t P8[7], I8[7], D8[7];	//8 bits is much faster and the code is much shorter
static uint8_t dynP8[3], dynI8[3], dynD8[3];
static uint8_t rollPitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint8_t activate[6];

/* prototypes */
void serialCom(void);
void initOutput(void);
void initSensors(void);
void readEEPROM(void);
void checkFirstTime(void);
void configureReceiver(void);
void writeMotors(void);
void computeRC(void);
void writeServos(void);
void configurationLoop(void);
void writeParams(void);
void Mag_getADC(void);
void Baro_update(void);
void computeIMU(void);
void mixTable(void);
void annexCode(void);
uint8_t WMP_getRawADC(void);
void getEstimatedAttitude(void);
void getEstimatedAltitude(void);
void ACC_getADC(void);
void Gyro_getADC(void);
void GYRO_Common(void);
void ACC_Common(void);
void Gyro_init(void);
void Baro_init(void);
void ACC_init(void);
void Mag_init(void);
void UartSendData(void);
void WMP_init(uint8_t d);

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;
    for (r = 0; r < repeat; r++) {
	for (i = 0; i < num; i++) {
	    LEDPIN_SWITCH;	//switch LEDPIN state
	    BUZZERPIN_ON;
	    delay(wait);
	    BUZZERPIN_OFF;
	}
	delay(60);
    }
}

void annexCode()
{
    //this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
    static uint32_t serialTime = 0;
    static uint32_t buzzerTime = 0;
    static uint32_t calibratedAccTime;
    static uint8_t buzzerState = 0;
    static uint32_t vbatRaw = 0;	//used for smoothing voltage reading
    static uint8_t buzzerFreq;	//delay between buzzer ring
    uint8_t axis;
    uint8_t prop1, prop2;

    static uint32_t telemetryTime = 0;
    static uint32_t telemetryAutoTime = 0;
    uint16_t pMeterRaw, powerValue;	//used for current reading
    static uint32_t psensorTime = 0;

    //PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
    if (rcData[THROTTLE] < 1500)
	prop2 = 100;
    else if (rcData[THROTTLE] < 2000)
	prop2 = 100 - (rcData[THROTTLE] - 1500) / 5 * dynThrPID / 100;
    else
	prop2 = 100 - dynThrPID;

    for (axis = 0; axis < 2; axis++) {
	//PITCH & ROLL dynamic PID adjustemnt, depending on stick deviation
	prop1 = 100 - min(abs(rcData[axis] - 1500) / 5, 100) * rollPitchRate / 100;
	dynP8[axis] = P8[axis] * prop1 / 100 * prop2 / 100;
	dynD8[axis] = D8[axis] * prop1 / 100 * prop2 / 100;
    }

    //YAW dynamic PID adjustemnt
    prop1 = 100 - min(abs(rcData[YAW] - 1500) / 5, 100) * yawRate / 100;
    dynP8[YAW] = P8[YAW] * prop1 / 100;
    dynD8[YAW] = D8[YAW] * prop1 / 100;

#if (POWERMETER == 2)
    if (micros() > psensorTime + 19977 /*20000 */ ) {	// 50Hz, but avoid bulking of timed tasks
	pMeterRaw = analogRead(PSENSORPIN);
	powerValue = (PSENSORNULL > pMeterRaw ? PSENSORNULL - pMeterRaw : pMeterRaw - PSENSORNULL);	// do not use abs(), it would induce implicit cast to uint and overrun
#ifdef LOG_VALUES
	if (powerValue < 256) {	// only accept reasonable values. 256 is empirical
	    if (powerValue > powerMax)
		powerMax = powerValue;
	    powerAvg = powerValue;
	}
#endif
	pMeter[PMOTOR_SUM] += (uint32_t) powerValue;
	psensorTime = micros();
    }
#endif

#if defined(VBAT)
    vbatRaw = (vbatRaw * 15 + analogRead(V_BATPIN) * 16) >> 4;	// smoothing of vbat readings  
    vbat = vbatRaw / VBATSCALE;	// result is Vbatt in 0.1V steps

    if ((vbat > VBATLEVEL1_3S)
#if defined(POWERMETER)
	&& ((pMeter[PMOTOR_SUM] < pAlarm) || (pAlarm == 0))
#endif
	|| (NO_VBAT > vbat))	// ToLuSe
    {				//VBAT ok AND powermeter ok, buzzer off
	buzzerFreq = 0;
	buzzerState = 0;
	BUZZERPIN_OFF;
#if defined(POWERMETER)
    } else if (pMeter[PMOTOR_SUM] > pAlarm) {	// sound alarm for powermeter
	buzzerFreq = 4;
#endif
    } else if (vbat > VBATLEVEL2_3S)
	buzzerFreq = 1;
    else if (vbat > VBATLEVEL3_3S)
	buzzerFreq = 2;
    else
	buzzerFreq = 4;
    if (buzzerFreq) {
	if (buzzerState && (currentTime > buzzerTime + 250000)) {
	    buzzerState = 0;
	    BUZZERPIN_OFF;
	    buzzerTime = currentTime;
	} else if (!buzzerState && (currentTime > (buzzerTime + (2000000 >> buzzerFreq)))) {
	    buzzerState = 1;
	    BUZZERPIN_ON;
	    buzzerTime = currentTime;
	}
    }
#endif

    if (((calibratingA > 0 && (ACC || nunchuk)) || (calibratingG > 0))) {	// Calibration phasis
	LEDPIN_SWITCH;
    } else {
	if (calibratedACC == 1) {
	    LEDPIN_OFF;
	}
	if (armed) {
	    LEDPIN_ON;
	}
    }

    if (micros() > calibratedAccTime + 500000) {
	if (abs(angle[ROLL]) > 150 || abs(angle[PITCH]) > 150) {	//more than 15 deg detection
	    calibratedACC = 0;	//the multi uses ACC and is not calibrated or is too much inclinated
	    LEDPIN_SWITCH;
	    calibratedAccTime = micros();
	} else {
	    calibratedACC = 1;
	}
    }

    if (micros() > serialTime + 20000) {	// 50Hz
	serialCom();
	serialTime = micros();
    }
#ifdef LCD_TELEMETRY_AUTO
    if ((telemetry_auto) && (micros() > telemetryAutoTime + LCD_TELEMETRY_AUTO)) {	// every 2 seconds
	telemetry++;
	if ((telemetry < 'A') || (telemetry > 'D'))
	    telemetry = 'A';
	telemetryAutoTime = micros();	// why use micros() and not the variable currentTime ?
    }
#endif
#ifdef LCD_TELEMETRY
    if (micros() > telemetryTime + LCD_TELEMETRY) {	// 10Hz
	if (telemetry)
	    lcd_telemetry();
	telemetryTime = micros();
    }
#endif
    for (axis = 0; axis < 3; axis++) {
	uint16_t tmp = abs(rcData[axis] - MIDRC);
#if defined(DEADBAND)
	if (tmp > DEADBAND) {
	    tmp -= DEADBAND;
	} else {
	    tmp = 0;
	}
#endif
	if (axis != 2) {
	    uint16_t tmp2 = tmp / 100;
	    rcCommand[axis] = lookupRX[tmp2] + (tmp - tmp2 * 100) * (lookupRX[tmp2 + 1] - lookupRX[tmp2]) / 100;
	} else {
	    rcCommand[axis] = tmp;
	}
	if (rcData[axis] < MIDRC)
	    rcCommand[axis] = -rcCommand[axis];
    }
    rcCommand[THROTTLE] = MINTHROTTLE + (int32_t) (MAXTHROTTLE - MINTHROTTLE) * (rcData[THROTTLE] - MINCHECK) / (2000 - MINCHECK);
}


void setup()
{
    LEDPIN_PINMODE;
    POWERPIN_PINMODE;
    BUZZERPIN_PINMODE;
    STABLEPIN_PINMODE;
    POWERPIN_OFF;
    Serial_begin(SERIAL_COM_SPEED);
    initOutput();
    readEEPROM();
    checkFirstTime();
    configureReceiver();
    initSensors();
    previousTime = micros();
#if defined(GIMBAL) || defined(FLYING_WING)
    calibratingA = 400;
#endif
    calibratingG = 400;
#if defined(POWERMETER)
    for (uint8_t i = 0; i <= PMOTOR_SUM; i++)
	pMeter[i] = 0;
#endif
}

// ******** Main Loop *********
void loop()
{
    static uint8_t rcDelayCommand;	// this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    uint8_t axis, i;
    int16_t error, errorAngle;
    int16_t delta;
    int16_t PTerm, ITerm, DTerm;
    static int16_t lastGyro[3] = { 0, 0, 0 };
    static int16_t delta1[3], delta2[3];
    static int16_t errorGyroI[3] = { 0, 0, 0 };
    static int16_t errorAngleI[2] = { 0, 0 };
    static uint8_t camCycle = 0;
    static uint8_t camState = 0;
    static uint32_t camTime = 0;
    static uint32_t rcTime = 0;
    static int16_t initialThrottleHold;
    static int16_t errorAltitudeI = 0;
    int16_t AltPID = 0;
    static int16_t lastVelError = 0;
    static float AltHold = 0.0;

    if (currentTime > (rcTime + 20000)) {	// 50Hz
	rcTime = currentTime;
	computeRC();
	// Failsafe routine - added by MIS
#if defined(FAILSAFE)
	if (failsafeCnt > (5 * FAILSAVE_DELAY) && armed == 1) {	// Stabilize, and set Throttle to specified level
	    for (i = 0; i < 3; i++)
		rcData[i] = MIDRC;	// after specified guard time after RC signal is lost (in 0.1sec)
	    rcData[THROTTLE] = FAILSAVE_THR0TTLE;
	    if (failsafeCnt > 5 * (FAILSAVE_DELAY + FAILSAVE_OFF_DELAY)) {	// Turn OFF motors after specified Time (in 0.1sec)
		armed = 0;	//This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
		okToArm = 0;	//to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
	    }
	}
	failsafeCnt++;
#endif
	// end of failsave routine - next change is made with RcOptions setting
	if (rcData[THROTTLE] < MINCHECK) {
	    errorGyroI[ROLL] = 0;
	    errorGyroI[PITCH] = 0;
	    errorGyroI[YAW] = 0;
	    errorAngleI[ROLL] = 0;
	    errorAngleI[PITCH] = 0;
	    rcDelayCommand++;
	    if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK && armed == 0) {
		if (rcDelayCommand == 20)
		    calibratingG = 400;
	    } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] > MAXCHECK && armed == 0) {
		if (rcDelayCommand == 20) {
		    servo[0] = 1500;	//we center the yaw gyro in conf mode
		    writeServos();
#ifdef LCD_CONF
		    configurationLoop();	//beginning LCD configuration
#endif
		    previousTime = micros();
		}
	    } else if (activate[BOXARM] > 0) {
		if ((rcOptions & activate[BOXARM]) && okToArm)
		    armed = 1;
		else if (armed)
		    armed = 0;
		rcDelayCommand = 0;
	    } else if ((rcData[YAW] < MINCHECK || rcData[ROLL] < MINCHECK) && armed == 1) {
		if (rcDelayCommand == 20)
		    armed = 0;	// rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
	    } else if ((rcData[YAW] > MAXCHECK || rcData[ROLL] > MAXCHECK) && rcData[PITCH] < MAXCHECK && armed == 0 && calibratingG == 0 && calibratedACC == 1) {
		if (rcDelayCommand == 20)
		    armed = 1;
#ifdef LCD_TELEMETRY_AUTO
	    } else if (rcData[ROLL] < MINCHECK && rcData[PITCH] > MAXCHECK && armed == 0) {
		if (rcDelayCommand == 20) {
		    if (telemetry_auto) {
			telemetry_auto = 0;
			telemetry = 0;
		    } else
			telemetry_auto = 1;
		}
#endif
	    } else
		rcDelayCommand = 0;
	} else if (rcData[THROTTLE] > MAXCHECK && armed == 0) {
	    if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK) {
		if (rcDelayCommand == 20)
		    calibratingA = 400;
		rcDelayCommand++;
	    } else if (rcData[PITCH] > MAXCHECK) {
		accTrim[PITCH]++;
		writeParams();
	    } else if (rcData[PITCH] < MINCHECK) {
		accTrim[PITCH]--;
		writeParams();
	    } else if (rcData[ROLL] > MAXCHECK) {
		accTrim[ROLL]++;
		writeParams();
	    } else if (rcData[ROLL] < MINCHECK) {
		accTrim[ROLL]--;
		writeParams();
	    } else {
		rcDelayCommand = 0;
	    }
	}
#ifdef LOG_VALUES
	else if (armed) {	// update min and max values here, so do not get cycle time of the motor arming (which is way higher than normal)
	    if (cycleTime > cycleTimeMax)
		cycleTimeMax = cycleTime;	// remember highscore
	    if (cycleTime < cycleTimeMin)
		cycleTimeMin = cycleTime;	// remember lowscore
	}
#endif

	rcOptions = (rcData[AUX1] < 1300) + (1300 < rcData[AUX1] && rcData[AUX1] < 1700) * 2 + (rcData[AUX1] > 1700) * 4 + (rcData[AUX2] < 1300) * 8 + (1300 < rcData[AUX2] && rcData[AUX2] < 1700) * 16 + (rcData[AUX2] > 1700) * 32;

	//note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
	if (((rcOptions & activate[BOXACC]) || (failsafeCnt > 5 * FAILSAVE_DELAY)) && (ACC || nunchuk)) {
	    // bumpless transfer to Level mode
	    if (!accMode) {
		errorAngleI[ROLL] = 0;
		errorAngleI[PITCH] = 0;
		accMode = 1;
	    }
	} else
	    accMode = 0;	// modified by MIS for failsave support

	if ((rcOptions & activate[BOXARM]) == 0)
	    okToArm = 1;
	if (accMode == 1) {
	    STABLEPIN_ON;
	} else {
	    STABLEPIN_OFF;
	}

	if (BARO) {
	    if (rcOptions & activate[BOXBARO]) {
		if (baroMode == 0) {
		    baroMode = 1;
		    AltHold = EstAlt;
		    initialThrottleHold = rcCommand[THROTTLE];
		    errorAltitudeI = 0;
		    lastVelError = 0;
		    EstVelocity = 0;
		}
	    } else
		baroMode = 0;
	}
	if (MAG) {
	    if (rcOptions & activate[BOXMAG]) {
		if (magMode == 0) {
		    magMode = 1;
		    magHold = heading;
		}
	    } else
		magMode = 0;
	}
    }
    if (MAG)
	Mag_getADC();
    if (BARO)
	Baro_update();

    computeIMU();
    // Measure loop rate just afer reading the sensors
    currentTime = micros();
    cycleTime = currentTime - previousTime;
    previousTime = currentTime;

    if (MAG) {
	if (abs(rcCommand[YAW]) < 70 && magMode) {
	    int16_t dif = heading - magHold;
	    if (dif <= -180)
		dif += 360;
	    if (dif >= +180)
		dif -= 360;
	    if ((abs(angle[ROLL]) < 200) && (abs(angle[PITCH]) < 200))	//20 deg
		rcCommand[YAW] -= dif * P8[PIDMAG] / 30;
	} else
	    magHold = heading;
    }

    if (BARO) {
	if (baroMode) {
	    if (abs(rcCommand[THROTTLE] - initialThrottleHold) > 20) {
		baroMode = 0;
		errorAltitudeI = 0;
	    }
	    //**** Alt. Set Point stabilization PID ****
	    error = constrain((AltHold - EstAlt) * 10, -100, 100);	//  +/-10m,  1 decimeter accuracy
	    errorAltitudeI += error;
	    errorAltitudeI = constrain(errorAltitudeI, -5000, 5000);

	    PTerm = P8[PIDALT] * error / 100;	// 16 bits is ok here
	    ITerm = (int32_t) I8[PIDALT] * errorAltitudeI / 4000;	// 

	    AltPID = PTerm + ITerm;

	    //**** Velocity stabilization PD ****        
	    error = constrain(EstVelocity * 2000.0f, -30000.0f, 30000.0f);
	    delta = error - lastVelError;
	    lastVelError = error;

	    PTerm = (int32_t) error *P8[PIDVEL] / 800;
	    DTerm = (int32_t) delta *D8[PIDVEL] / 16;

	    rcCommand[THROTTLE] = initialThrottleHold + constrain(AltPID - (PTerm - DTerm), -100, +100);
	}
    }
    //**** PITCH & ROLL & YAW PID ****    
    for (axis = 0; axis < 3; axis++) {
	if (accMode == 1 && axis < 2) {	//LEVEL MODE
	    errorAngle = constrain(2 * rcCommand[axis], -700, +700) - angle[axis] + accTrim[axis];	//16 bits is ok here
#ifdef LEVEL_PDF
	    PTerm = -(int32_t) angle[axis] * P8[PIDLEVEL] / 100;
#else
	    PTerm = (int32_t) errorAngle *P8[PIDLEVEL] / 100;	//32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
#endif

	    errorAngleI[axis] += errorAngle;	//16 bits is ok here
	    errorAngleI[axis] = constrain(errorAngleI[axis], -10000, +10000);	//WindUp     //16 bits is ok here
	    ITerm = (int32_t) errorAngleI[axis] * I8[PIDLEVEL] / 4000;	//32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
	} else {		//ACRO MODE or YAW axis
	    error = (int32_t) rcCommand[axis] * 10 * 8 / P8[axis] - gyroData[axis];	//32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
	    PTerm = rcCommand[axis];

	    errorGyroI[axis] += error;	//16 bits is ok here
	    errorGyroI[axis] = constrain(errorGyroI[axis], -16000, +16000);	//WindUp       //16 bits is ok here
	    if (abs(gyroData[axis]) > 640)
		errorGyroI[axis] = 0;
	    ITerm = (int32_t) errorGyroI[axis] * I8[axis] / 1000 / 8;	//32 bits is needed for calculation: 16000*I8  16 bits is ok for result
	}
	PTerm -= (int32_t) gyroData[axis] * dynP8[axis] / 10 / 8;	//32 bits is needed for calculation            16 bits is ok for result

	delta = gyroData[axis] - lastGyro[axis];	//16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
	lastGyro[axis] = gyroData[axis];
	DTerm = (int32_t) (delta1[axis] + delta2[axis] + delta + 1) * dynD8[axis] / 3 / 8;	//32 bits is needed for calculation (800+800+800)*50 = 120000           16 bits is ok for result 
	delta2[axis] = delta1[axis];
	delta1[axis] = delta;

	axisPID[axis] = PTerm + ITerm - DTerm;
    }

#if defined(CAMTRIG)
    if (camCycle == 1) {
	if (camState == 0) {
	    servo[3] = CAM_SERVO_HIGH;
	    camState = 1;
	    camTime = millis();
	} else if (camState == 1) {
	    if ((millis() - camTime) > CAM_TIME_HIGH) {
		servo[3] = CAM_SERVO_LOW;
		camState = 2;
		camTime = millis();
	    }
	} else {		//camState ==2
	    if ((millis() - camTime) > CAM_TIME_LOW) {
		camState = 0;
		camCycle = 0;
	    }
	}
    }
    if (rcOptions & activate[BOXCAMTRIG])
	camCycle = 1;
#endif

    mixTable();
    writeServos();
    writeMotors();
#if defined(LOG_VALUES) || (POWERMETER == 1)
    logMotorsPower();
#endif
}



/* EEPROM --------------------------------------------------------------------- */
static uint8_t checkNewConf = 144;

typedef struct eep_entry_t {
    void *var;
    uint8_t size;
} eep_entry_t;

// ************************************************************************************************************
// EEPROM Layout definition
// ************************************************************************************************************
static eep_entry_t eep_entry[] = {
    &checkNewConf, sizeof(checkNewConf),
    &P8, sizeof(P8), 
    &I8, sizeof(I8), 
    &D8, sizeof(D8),
    &rcRate8, sizeof(rcRate8),
    &rcExpo8, sizeof(rcExpo8),
    &rollPitchRate, sizeof(rollPitchRate),
    &yawRate, sizeof(yawRate),
    &dynThrPID, sizeof(dynThrPID),
    &activate, sizeof(activate),
    &accZero, sizeof(accZero),
    &magZero, sizeof(magZero),
    &accTrim, sizeof(accTrim)
#if defined(POWERMETER)
    , &powerTrigger1, sizeof(powerTrigger1)
#endif
};
#define EEBLOCK_SIZE sizeof(eep_entry)/sizeof(eep_entry_t)
// ************************************************************************************************************

void readEEPROM()
{
    uint8_t i, _address = eep_entry[0].size;
    for (i = 1; i < EEBLOCK_SIZE; i++) {
	eeprom_read_block(eep_entry[i].var, (void *) (_address), eep_entry[i].size);
	_address += eep_entry[i].size;
    }
#if defined(POWERMETER)
    pAlarm = (uint32_t) powerTrigger1 *(uint32_t) PLEVELSCALE *(uint32_t) PLEVELDIV;	// need to cast before multiplying
#endif
    for (i = 0; i < 7; i++)
	lookupRX[i] = (2500 + rcExpo8 * (i * i - 25)) * i * (int32_t) rcRate8 / 1250;
}

void writeParams()
{
    uint8_t i, _address = 0;
    for (i = 0; i < EEBLOCK_SIZE; i++) {
	eeprom_write_block(eep_entry[i].var, (void *) (_address), eep_entry[i].size);
	_address += eep_entry[i].size;
    }
    readEEPROM();
    blinkLED(15, 20, 1);
}

void checkFirstTime()
{
    uint8_t test_val, i;
    eeprom_read_block((void *) &test_val, (void *) (0), sizeof(test_val));
    if (test_val == checkNewConf)
	return;
    P8[ROLL] = 40;
    I8[ROLL] = 30;
    D8[ROLL] = 17;
    P8[PITCH] = 40;
    I8[PITCH] = 30;
    D8[PITCH] = 17;
    P8[YAW] = 85;
    I8[YAW] = 0;
    D8[YAW] = 0;
    P8[PIDALT] = 47;
    I8[PIDALT] = 0;
    D8[PIDALT] = 0;
    P8[PIDVEL] = 0;
    I8[PIDVEL] = 0;
    D8[PIDVEL] = 0;
    P8[PIDLEVEL] = 90;
    I8[PIDLEVEL] = 45;
    P8[PIDMAG] = 40;
    rcRate8 = 45;		// = 0.9 in GUI
    rcExpo8 = 65;
    rollPitchRate = 0;
    yawRate = 0;
    dynThrPID = 0;
    for (i = 0; i < 6; i++)
	activate[i] = 0;
    accTrim[0] = 0;
    accTrim[1] = 0;
#if defined(POWERMETER)
    powerTrigger1 = 0;
#endif
    writeParams();
}

/* RX -------------------------------------------------------------------------------- */
static uint8_t pinRcChannel[8] = { ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN, AUX2PIN, CAM1PIN, CAM2PIN };
volatile uint16_t rcPinValue[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };	// interval [1000;2000]
static int16_t rcData4Values[8][4];
static int16_t rcDataMean[8];

// ***PPM SUM SIGNAL***
#ifdef SERIAL_SUM_PPM
static uint8_t rcChannel[8] = { SERIAL_SUM_PPM };
#endif
volatile uint16_t rcValue[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };	// interval [1000;2000]

// Configure each rc pin for PCINT
void configureReceiver()
{
    uint8_t chan, a;
#if !defined(SERIAL_SUM_PPM) && !defined(SPEKTRUM)
    for (chan = 0; chan < 8; chan++)
	for (a = 0; a < 4; a++)
	    rcData4Values[chan][a] = 1500;	//we initiate the default value of each channel. If there is no RC receiver connected, we will see those values
#if defined(PROMINI)
    // PCINT activated only for specific pin inside [D0-D7]  , [D2 D4 D5 D6 D7] for this multicopter
    PORTD = (1 << 2) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);	//enable internal pull ups on the PINs of PORTD (no high impedence PINs)
    PCMSK2 |= (1 << 2) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
    PCICR = 1 << 2;		// PCINT activated only for the port dealing with [D0-D7] PINs
#endif
#if defined(MEGA)
    // PCINT activated only for specific pin inside [A8-A15]
    DDRK = 0;			// defined PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
    PORTK = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);	//enable internal pull ups on the PINs of PORTK
    PCMSK2 |= (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
    PCICR = 1 << 2;		// PCINT activated only for PORTK dealing with [A8-A15] PINs
#endif
#endif
#if defined(SERIAL_SUM_PPM)
    PPM_PIN_INTERRUPT
#endif
#if defined (SPEKTRUM)
#endif
}

#if !defined(SERIAL_SUM_PPM) && !defined(SPEKTRUM) && !defined(STM8)
ISR(PCINT2_vect)
{				//this ISR is common to every receiver channel, it is call everytime a change state occurs on a digital pin [D2-D7]
    uint8_t mask;
    uint8_t pin;
    uint16_t cTime, dTime;
    static uint16_t edgeTime[8];
    static uint8_t PCintLast;

#if defined(PROMINI)
    pin = PIND;			// PIND indicates the state of each PIN for the arduino port dealing with [D0-D7] digital pins (8 bits variable)
#endif
#if defined(MEGA)
    pin = PINK;			// PINK indicates the state of each PIN for the arduino port dealing with [A8-A15] digital pins (8 bits variable)
#endif
    mask = pin ^ PCintLast;	// doing a ^ between the current interruption and the last one indicates wich pin changed
    sei();			// re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast = pin;		// we memorize the current state of all PINs [D0-D7]

    cTime = micros();		// micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits

    // mask is pins [D0-D7] that have changed // the principle is the same on the MEGA for PORTK and [A8-A15] PINs
    // chan = pin sequence of the port. chan begins at D2 and ends at D7
    if (mask & 1 << 2)		//indicates the bit 2 of the arduino port [D0-D7], that is to say digital pin 2, if 1 => this pin has just changed
	if (!(pin & 1 << 2)) {	//indicates if the bit 2 of the arduino port [D0-D7] is not at a high state (so that we match here only descending PPM pulse)
	    dTime = cTime - edgeTime[2];
	    if (900 < dTime && dTime < 2200)
		rcPinValue[2] = dTime;	// just a verification: the value must be in the range [1000;2000] + some margin
	} else
	    edgeTime[2] = cTime;	// if the bit 2 of the arduino port [D0-D7] is at a high state (ascending PPM pulse), we memorize the time
    if (mask & 1 << 4)		//same principle for other channels   // avoiding a for() is more than twice faster, and it's important to minimize execution time in ISR
	if (!(pin & 1 << 4)) {
	    dTime = cTime - edgeTime[4];
	    if (900 < dTime && dTime < 2200)
		rcPinValue[4] = dTime;
	} else
	    edgeTime[4] = cTime;
    if (mask & 1 << 5)
	if (!(pin & 1 << 5)) {
	    dTime = cTime - edgeTime[5];
	    if (900 < dTime && dTime < 2200)
		rcPinValue[5] = dTime;
	} else
	    edgeTime[5] = cTime;
    if (mask & 1 << 6)
	if (!(pin & 1 << 6)) {
	    dTime = cTime - edgeTime[6];
	    if (900 < dTime && dTime < 2200)
		rcPinValue[6] = dTime;
	} else
	    edgeTime[6] = cTime;
    if (mask & 1 << 7)
	if (!(pin & 1 << 7)) {
	    dTime = cTime - edgeTime[7];
	    if (900 < dTime && dTime < 2200)
		rcPinValue[7] = dTime;
	} else
	    edgeTime[7] = cTime;
#if defined(MEGA)
    if (mask & 1 << 0)
	if (!(pin & 1 << 0)) {
	    dTime = cTime - edgeTime[0];
	    if (900 < dTime && dTime < 2200)
		rcPinValue[0] = dTime;
	} else
	    edgeTime[0] = cTime;
    if (mask & 1 << 1)
	if (!(pin & 1 << 1)) {
	    dTime = cTime - edgeTime[1];
	    if (900 < dTime && dTime < 2200)
		rcPinValue[1] = dTime;
	} else
	    edgeTime[1] = cTime;
    if (mask & 1 << 3)
	if (!(pin & 1 << 3)) {
	    dTime = cTime - edgeTime[3];
	    if (900 < dTime && dTime < 2200)
		rcPinValue[3] = dTime;
	} else
	    edgeTime[3] = cTime;
#endif
#if defined(FAILSAFE)
    if (mask & 1 << THROTTLEPIN) {	// If pulse present on THROTTLE pin (independent from ardu version), clear FailSafe counter  - added by MIS
	if (failsafeCnt > 20)
	    failsafeCnt -= 20;
	else
	    failsafeCnt = 0;
    }
#endif
}
#endif

#if defined(SERIAL_SUM_PPM)
void rxInt()
{
    uint16_t now, diff;
    static uint16_t last = 0;
    static uint8_t chan = 0;

    now = micros();
    diff = now - last;
    last = now;
    if (diff > 3000)
	chan = 0;
    else {
	if (900 < diff && diff < 2200 && chan < 8) {	//Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
	    rcValue[chan] = diff;
#if defined(FAILSAFE)
	    if (failsafeCnt > 20)
		failsafeCnt -= 20;
	    else
		failsafeCnt = 0;	// clear FailSafe counter - added by MIS  //incompatible to quadroppm
#endif
	}
	chan++;
    }
}
#endif

#if defined(SPEKTRUM)


#endif

uint16_t readRawRC(uint8_t chan)
{
    uint16_t data;
#ifdef STM8

    data = 1500; // TODO


#else
    uint8_t oldSREG;
    oldSREG = SREG;
    cli();			// Let's disable interrupts
#ifndef SERIAL_SUM_PPM
    data = rcPinValue[pinRcChannel[chan]];	// Let's copy the data Atomically
#else
    data = rcValue[rcChannel[chan]];
#endif
    SREG = oldSREG;
    sei();			// Let's enable the interrupts
#if defined(PROMINI) && !defined(SERIAL_SUM_PPM)
    if (chan > 4)
	return 1500;
#endif
#endif /* STM8 */
    return data;		// We return the value correctly copied when the IRQ's where disabled
}

void computeRC()
{
    static uint8_t rc4ValuesIndex = 0;
    uint8_t chan, a;

    rc4ValuesIndex++;
    for (chan = 0; chan < 8; chan++) {
	rcData4Values[chan][rc4ValuesIndex % 4] = readRawRC(chan);
	rcDataMean[chan] = 0;
	for (a = 0; a < 4; a++)
	    rcDataMean[chan] += rcData4Values[chan][a];
	rcDataMean[chan] = (rcDataMean[chan] + 2) / 4;
	if (rcDataMean[chan] < rcData[chan] - 3)
	    rcData[chan] = rcDataMean[chan] + 2;
	if (rcDataMean[chan] > rcData[chan] + 3)
	    rcData[chan] = rcDataMean[chan] - 2;
    }
}

/* OUTPUT ------------------------------------------------------------------------------------- */

#if defined(BI) || defined(TRI) || defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING) || defined(CAMTRIG)
#define SERVO
#endif

#if defined(GIMBAL) || defined(FLYING_WING)
#define NUMBER_MOTOR 0
#elif defined(BI)
#define NUMBER_MOTOR 2
#elif defined(TRI)
#define NUMBER_MOTOR 3
#elif defined(QUADP) || defined(QUADX) || defined(Y4)
#define NUMBER_MOTOR 4
#elif defined(Y6) || defined(HEX6) || defined(HEX6X)
#define NUMBER_MOTOR 6
#elif defined(OCTOX8) || defined(OCTOFLATP) || defined(OCTOFLATX)
#define NUMBER_MOTOR 8
#endif

uint8_t PWM_PIN[8] = { MOTOR_ORDER };
volatile uint8_t atomicServo[4] = { 125, 125, 125, 125 };

//for HEX Y6 and HEX6/HEX6X flat and for promini
volatile uint8_t atomicPWM_PIN5_lowState;
volatile uint8_t atomicPWM_PIN5_highState;
volatile uint8_t atomicPWM_PIN6_lowState;
volatile uint8_t atomicPWM_PIN6_highState;


void writeServos()
{
#if defined(SERVO)
    atomicServo[0] = (servo[0] - 1000) / 4;
    atomicServo[1] = (servo[1] - 1000) / 4;
    atomicServo[2] = (servo[2] - 1000) / 4;
    atomicServo[3] = (servo[3] - 1000) / 4;
#endif
}

void writeMotors()
{
    uint8_t i;

    // [1000;2000] => [125;250]
#if defined(MEGA)
    for (i = 0; i < NUMBER_MOTOR; i++)
	analogWrite(PWM_PIN[i], motor[i] >> 3);
#else
    for (i = 0; i < min(NUMBER_MOTOR, 4); i++)
	analogWrite(PWM_PIN[i], motor[i] >> 3);
#if (NUMBER_MOTOR == 6)
    atomicPWM_PIN5_highState = motor[5] / 8;
    atomicPWM_PIN5_lowState = 255 - atomicPWM_PIN5_highState;
    atomicPWM_PIN6_highState = motor[4] / 8;
    atomicPWM_PIN6_lowState = 255 - atomicPWM_PIN6_highState;
#endif
#endif
}

void writeAllMotors(int16_t mc)
{      
    uint8_t i;
    // Sends commands to all motors
    for (i = 0; i < NUMBER_MOTOR; i++)
	motor[i] = mc;
    writeMotors();
}

#if defined(LOG_VALUES) || (POWERMETER == 1)
void logMotorsPower()
{
    uint32_t amp;
    uint8_t i;
    /* true cubic function; when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 1000 */
    const uint32_t amperes[16] = { 31, 246, 831, 1969, 3845, 6645, 10551, 15750, 22425, 30762, 40944, 53156, 67583, 84410, 103821, 126000 };

    if (vbat) {			// by all means - must avoid division by zero 
	for (i = 0; i < NUMBER_MOTOR; i++) {
	    amp = amperes[(motor[i] - 1000) >> 6] / vbat;	// range mapped from [1000:2000] => [0:1000]; then break that up into 16 ranges; lookup amp
#ifdef LOG_VALUES
	    pMeter[i] += amp;	// sum up over time the mapped ESC input 
#endif
#if (POWERMETER == 1)
	    pMeter[PMOTOR_SUM] += amp;	// total sum over all motors
#endif
	}
    }
}
#endif

void initOutput()
{
    uint8_t i;
    for (i = 0; i < NUMBER_MOTOR; i++)
	pinMode(PWM_PIN[i], OUTPUT);
    writeAllMotors(1000);
    delay(300);
#if defined(SERVO)
    initializeServo();
#elif (NUMBER_MOTOR == 6) && defined(PROMINI)
    initializeSoftPWM();
#endif
}

#if defined(SERVO)
void initializeServo()
{
#if defined(TRI)
    DIGITAL_SERVO_TRI_PINMODE
#endif
#if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING)
	DIGITAL_TILT_ROLL_PINMODE DIGITAL_TILT_PITCH_PINMODE
#endif
#if defined(CAMTRIG)
	DIGITAL_CAM_PINMODE
#endif
#if defined(BI)
	DIGITAL_SERVO_TRI_PINMODE DIGITAL_BI_LEFT_PINMODE
#endif
	TCCR0A = 0;		// normal counting mode
    TIMSK0 |= (1 << OCIE0A);	// Enable CTC interrupt
}

// ****servo yaw with a 50Hz refresh rate****
// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us
// algorithm strategy:
// pulse high servo 0 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 0
// pulse high servo 1 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 1
// pulse high servo 2 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 2
// pulse high servo 3 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 3
// do nothing for 14 x 1000 us
ISR(TIMER0_COMPA_vect)
{
    static uint8_t state = 0;
    static uint8_t count;
    if (state == 0) {
	//http://billgrundmann.wordpress.com/2009/03/03/to-use-or-not-use-writedigital/
#if defined(TRI) || defined (BI)
	DIGITAL_SERVO_TRI_HIGH
#endif
	    OCR0A += 250;	// 1000 us
	state++;
    } else if (state == 1) {
	OCR0A += atomicServo[0];	// 1000 + [0-1020] us
	state++;
    } else if (state == 2) {
#if defined(TRI) || defined (BI)
	DIGITAL_SERVO_TRI_LOW
#endif
#if defined(BI)
	    DIGITAL_BI_LEFT_HIGH
#endif
#if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING)
	    DIGITAL_TILT_PITCH_HIGH
#endif
	    OCR0A += 250;	// 1000 us
	state++;
    } else if (state == 3) {
	OCR0A += atomicServo[1];	// 1000 + [0-1020] us
	state++;
    } else if (state == 4) {
#if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING)
	DIGITAL_TILT_PITCH_LOW DIGITAL_TILT_ROLL_HIGH
#endif
#if defined(BI)
	 DIGITAL_BI_LEFT_LOW
#endif
	 state++;
	OCR0A += 250;		// 1000 us
    } else if (state == 5) {
	OCR0A += atomicServo[2];	// 1000 + [0-1020] us
	state++;
    } else if (state == 6) {
#if defined(SERVO_TILT) || defined(GIMBAL) || defined(FLYING_WING)
	DIGITAL_TILT_ROLL_LOW
#endif
#if defined(CAMTRIG)
	    DIGITAL_CAM_HIGH
#endif
	    state++;
	OCR0A += 250;		// 1000 us
    } else if (state == 7) {
	OCR0A += atomicServo[3];	// 1000 + [0-1020] us
	state++;
    } else if (state == 8) {
#if defined(CAMTRIG)
	DIGITAL_CAM_LOW
#endif
	    count = 10;		// 12 x 1000 us
	state++;
	OCR0A += 250;		// 1000 us
    } else if (state == 9) {
	if (count > 0)
	    count--;
	else
	    state = 0;
	OCR0A += 250;
    }
}
#endif

#if (NUMBER_MOTOR == 6) && defined(PROMINI)
void initializeSoftPWM()
{
    TCCR0A = 0;			// normal counting mode
    TIMSK0 |= (1 << OCIE0A);	// Enable CTC interrupt
    TIMSK0 |= (1 << OCIE0B);
}

ISR(TIMER0_COMPA_vect)
{
    static uint8_t state = 0;
    if (state == 0) {
	PORTD |= 1 << 5;	//digital PIN 5 high
	OCR0A += atomicPWM_PIN5_highState;	//250 x 4 microsecons = 1ms
	state = 1;
    } else if (state == 1) {
	OCR0A += atomicPWM_PIN5_highState;
	state = 2;
    } else if (state == 2) {
	PORTD &= ~(1 << 5);	//digital PIN 5 low
	OCR0A += atomicPWM_PIN5_lowState;
	state = 0;
    }
}

ISR(TIMER0_COMPB_vect)
{				//the same with digital PIN 6 and OCR0B counter
    static uint8_t state = 0;
    if (state == 0) {
	PORTD |= 1 << 6;
	OCR0B += atomicPWM_PIN6_highState;
	state = 1;
    } else if (state == 1) {
	OCR0B += atomicPWM_PIN6_highState;
	state = 2;
    } else if (state == 2) {
	PORTD &= ~(1 << 6);
	OCR0B += atomicPWM_PIN6_lowState;
	state = 0;
    }
}
#endif


void mixTable()
{
    int16_t maxMotor, a;
    uint8_t i, axis;

#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

#if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));
#endif
#ifdef BI
    motor[0] = PIDMIX(+1, 0, 0);	//LEFT
    motor[1] = PIDMIX(-1, 0, 0);	//RIGHT        
    servo[0] = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] + axisPID[PITCH]), 1020, 2000);	//LEFT
    servo[1] = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] - axisPID[PITCH]), 1020, 2000);	//RIGHT
#endif
#ifdef TRI
    motor[0] = PIDMIX(0, +4 / 3, 0);	//REAR
    motor[1] = PIDMIX(-1, -2 / 3, 0);	//RIGHT
    motor[2] = PIDMIX(+1, -2 / 3, 0);	//LEFT
    servo[0] = constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX);	//REAR
#endif
#ifdef QUADP
    motor[0] = PIDMIX(0, +1, -1);	//REAR
    motor[1] = PIDMIX(-1, 0, +1);	//RIGHT
    motor[2] = PIDMIX(+1, 0, +1);	//LEFT
    motor[3] = PIDMIX(0, -1, -1);	//FRONT
#endif
#ifdef QUADX
    motor[0] = PIDMIX(-1, +1, -1);	//REAR_R
    motor[1] = PIDMIX(-1, -1, +1);	//FRONT_R
    motor[2] = PIDMIX(+1, +1, +1);	//REAR_L
    motor[3] = PIDMIX(+1, -1, -1);	//FRONT_L
#endif
#ifdef Y4
    motor[0] = PIDMIX(+0, +1, -1);	//REAR_1 CW
    motor[1] = PIDMIX(-1, -1, 0);	//FRONT_R CCW
    motor[2] = PIDMIX(+0, +1, +1);	//REAR_2 CCW
    motor[3] = PIDMIX(+1, -1, 0);	//FRONT_L CW
#endif
#ifdef Y6
    motor[0] = PIDMIX(+0, +4 / 3, +1);	//REAR
    motor[1] = PIDMIX(-1, -2 / 3, -1);	//RIGHT
    motor[2] = PIDMIX(+1, -2 / 3, -1);	//LEFT
    motor[3] = PIDMIX(+0, +4 / 3, -1);	//UNDER_REAR
    motor[4] = PIDMIX(-1, -2 / 3, +1);	//UNDER_RIGHT
    motor[5] = PIDMIX(+1, -2 / 3, +1);	//UNDER_LEFT    
#endif
#ifdef HEX6
    motor[0] = PIDMIX(-1 / 2, +1 / 2, +1);	//REAR_R
    motor[1] = PIDMIX(-1 / 2, -1 / 2, -1);	//FRONT_R
    motor[2] = PIDMIX(+1 / 2, +1 / 2, +1);	//REAR_L
    motor[3] = PIDMIX(+1 / 2, -1 / 2, -1);	//FRONT_L
    motor[4] = PIDMIX(+0, -1, +1);	//FRONT
    motor[5] = PIDMIX(+0, +1, -1);	//REAR
#endif
#ifdef HEX6X
    motor[0] = PIDMIX(-1 / 2, +1 / 2, +1);	//REAR_R
    motor[1] = PIDMIX(-1 / 2, -1 / 2, +1);	//FRONT_R
    motor[2] = PIDMIX(+1 / 2, +1 / 2, -1);	//REAR_L
    motor[3] = PIDMIX(+1 / 2, -1 / 2, -1);	//FRONT_L
    motor[4] = PIDMIX(-1, +0, -1);	//RIGHT
    motor[5] = PIDMIX(+1, +0, +1);	//LEFT
#endif
#ifdef OCTOX8
    motor[0] = PIDMIX(-1, +1, -1);	//REAR_R
    motor[1] = PIDMIX(-1, -1, +1);	//FRONT_R
    motor[2] = PIDMIX(+1, +1, +1);	//REAR_L
    motor[3] = PIDMIX(+1, -1, -1);	//FRONT_L
    motor[4] = PIDMIX(-1, +1, +1);	//UNDER_REAR_R
    motor[5] = PIDMIX(-1, -1, -1);	//UNDER_FRONT_R
    motor[6] = PIDMIX(+1, +1, -1);	//UNDER_REAR_L
    motor[7] = PIDMIX(+1, -1, +1);	//UNDER_FRONT_L
#endif
#ifdef OCTOFLATP
    motor[0] = PIDMIX(+7 / 10, -7 / 10, +1);	//FRONT_L
    motor[1] = PIDMIX(-7 / 10, -7 / 10, +1);	//FRONT_R
    motor[2] = PIDMIX(-7 / 10, +7 / 10, +1);	//REAR_R
    motor[3] = PIDMIX(+7 / 10, +7 / 10, +1);	//REAR_L
    motor[4] = PIDMIX(+0, -1, -1);	//FRONT
    motor[5] = PIDMIX(-1, +0, -1);	//RIGHT
    motor[6] = PIDMIX(+0, +1, -1);	//REAR
    motor[7] = PIDMIX(+1, +0, -1);	//LEFT 
#endif
#ifdef OCTOFLATX
    motor[0] = PIDMIX(+1, -1 / 2, +1);	//MIDFRONT_L
    motor[1] = PIDMIX(-1 / 2, -1, +1);	//FRONT_R
    motor[2] = PIDMIX(-1, +1 / 2, +1);	//MIDREAR_R
    motor[3] = PIDMIX(+1 / 2, +1, +1);	//REAR_L
    motor[4] = PIDMIX(+1 / 2, -1, -1);	//FRONT_L
    motor[5] = PIDMIX(-1, -1 / 2, -1);	//MIDFRONT_R
    motor[6] = PIDMIX(-1 / 2, +1, -1);	//REAR_R
    motor[7] = PIDMIX(+1, +1 / 2, -1);	//MIDREAR_L 
#endif

#ifdef SERVO_TILT
    if (rcOptions & activate[BOXCAMSTAB]) {
	servo[1] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] / 16 + rcData[CAMPITCH] - 1500, TILT_PITCH_MIN, TILT_PITCH_MAX);
	servo[2] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP * angle[ROLL] / 16 + rcData[CAMROLL] - 1500, TILT_ROLL_MIN, TILT_ROLL_MAX);
    } else {
	servo[1] = constrain(TILT_PITCH_MIDDLE + rcData[CAMPITCH] - 1500, TILT_PITCH_MIN, TILT_PITCH_MAX);
	servo[2] = constrain(TILT_ROLL_MIDDLE + rcData[CAMROLL] - 1500, TILT_ROLL_MIN, TILT_ROLL_MAX);
    }
#endif
#ifdef GIMBAL
    servo[1] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] / 16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
    servo[2] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP * angle[ROLL] / 16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
#endif
#ifdef FLYING_WING
    servo[1] = constrain(1500 + axisPID[PITCH] - axisPID[ROLL], 1020, 2000);	//LEFT the direction of the 2 servo can be changed here: invert the sign before axisPID
    servo[2] = constrain(1500 + axisPID[PITCH] + axisPID[ROLL], 1020, 2000);	//RIGHT
#endif

    maxMotor = motor[0];
    for (i = 1; i < NUMBER_MOTOR; i++)
	if (motor[i] > maxMotor)
	    maxMotor = motor[i];
    for (i = 0; i < NUMBER_MOTOR; i++) {
	if (maxMotor > MAXTHROTTLE)	// this is a way to still have good gyro corrections if at least one motor reaches its max.
	    motor[i] -= maxMotor - MAXTHROTTLE;
	motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);
	if ((rcData[THROTTLE]) < MINCHECK)
#ifndef MOTOR_STOP
	    motor[i] = MINTHROTTLE;
#else
	    motor[i] = MINCOMMAND;
#endif
	if (armed == 0)
	    motor[i] = MINCOMMAND;
    }
}

/* IMU ------------------------------------------------------------------------------------------------- */

void computeIMU()
{
    uint8_t axis;
    static int16_t gyroADCprevious[3] = { 0, 0, 0 };
    int16_t gyroADCp[3];
    int16_t gyroADCinter[3];
    static int16_t lastAccADC[3] = { 0, 0, 0 };
    static uint32_t timeInterleave = 0;
    static int16_t gyroYawSmooth = 0;

    //we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
    //gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
    //gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
    if (!ACC && nunchuk) {
	annexCode();
	while ((micros() - timeInterleave) < INTERLEAVING_DELAY);	//interleaving delay between 2 consecutive reads
	timeInterleave = micros();
	WMP_getRawADC();
	getEstimatedAttitude();	// computation time must last less than one interleaving delay
#if BARO
	getEstimatedAltitude();
#endif
	while ((micros() - timeInterleave) < INTERLEAVING_DELAY);	//interleaving delay between 2 consecutive reads
	timeInterleave = micros();
	while (WMP_getRawADC() != 1);	// For this interleaving reading, we must have a gyro update at this point (less delay)

	for (axis = 0; axis < 3; axis++) {
	    // empirical, we take a weighted value of the current and the previous values
	    // /4 is to average 4 values, note: overflow is not possible for WMP gyro here
	    gyroData[axis] = (gyroADC[axis] * 3 + gyroADCprevious[axis] + 2) / 4;
	    gyroADCprevious[axis] = gyroADC[axis];
	}
    } else {
	if (ACC) {
	    ACC_getADC();
	    getEstimatedAttitude();
	    if (BARO)
		getEstimatedAltitude();
	}
	if (GYRO)
	    Gyro_getADC();
	else
	    WMP_getRawADC();
	for (axis = 0; axis < 3; axis++)
	    gyroADCp[axis] = gyroADC[axis];
	timeInterleave = micros();
	annexCode();
	while ((micros() - timeInterleave) < 650);	//empirical, interleaving delay between 2 consecutive reads
	if (GYRO)
	    Gyro_getADC();
	else
	    WMP_getRawADC();
	for (axis = 0; axis < 3; axis++) {
	    gyroADCinter[axis] = gyroADC[axis] + gyroADCp[axis];
	    // empirical, we take a weighted value of the current and the previous values
	    gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis] + 1) / 3;
	    gyroADCprevious[axis] = gyroADCinter[axis] / 2;
	    if (!ACC)
		accADC[axis] = 0;
	}
    }
#if defined(TRI)
    gyroData[YAW] = (gyroYawSmooth * 2 + gyroData[YAW] + 1) / 3;
    gyroYawSmooth = gyroData[YAW];
#endif
}


// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://wbb.multiwii.com/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// Last Modified: 19/04/2011
// Version: V1.1   
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: 8*/
#define ACC_LPF_FACTOR 8

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
/* Default WMC value: n/a*/
//#define MG_LPF_FACTOR 4

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/
/* Default WMC value: 300*/
#define GYR_CMPF_FACTOR 310.0f

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
/* Default WMC value: n/a*/
#define GYR_CMPFM_FACTOR 200.0f

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#if GYRO
// #define GYRO_SCALE ((2000.0f * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.155f)
  // +-2000/sec deg scale
#define GYRO_SCALE ((500.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)     
  // +- 200/sec deg scale
  // 1.5 is emperical, not sure what it means
  // should be in rad/sec
#else
#define GYRO_SCALE (1.0f/200e6f)
  // empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
  // !!!!should be adjusted to the rad/sec
#endif
// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

typedef struct {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

// comeon guys. endian anyone??
#define fp_is_neg(val) ((((unsigned char *)&val)[0] & 0x80) != 0)

int16_t _atan2(float y, float x)
{
    float z = y / x;
    int16_t zi = abs(((int16_t)(z * 100)));
    int8_t y_neg = fp_is_neg(y);
    if (zi < 100) {
	if (zi > 10)
	    z = z / (1.0f + 0.28f * z * z);
	if (fp_is_neg(x)) {
	    if (y_neg)
		z -= PI;
	    else
		z += PI;
	}
    } else {
	z = (PI / 2.0f) - z / (z * z + 0.28f);
	if (y_neg)
	    z -= PI;
    }
    z *= (180.0f / PI * 10);
    return z;
}

void getEstimatedAttitude()
{
    uint8_t axis;
    int16_t AccMag = 0;
    static t_fp_vector GEstG = { 0, 0, 200 };
    t_fp_vector EstG = GEstG;
    static t_fp_vector EstM = { 10, 10, 200 };
    float deltaGyroAngle;
    static uint16_t PreviousTime;
    static int16_t mgSmooth[3];	//projection of smoothed and normalized magnetic vector on x/y/z axis, as measured by magnetometer
    uint16_t CurrentTime = micros();
    float deltaTime = (CurrentTime - PreviousTime) * GYRO_SCALE;
    PreviousTime = CurrentTime;
    // Initialization
    for (axis = 0; axis < 3; axis++) {
#if defined(ACC_LPF_FACTOR)
	// LPF for ACC values
	accSmooth[axis] = (accSmooth[axis] * (ACC_LPF_FACTOR - 1) + accADC[axis]) / ACC_LPF_FACTOR;
#define ACC_VALUE accSmooth[axis]
#else
	accSmooth[axis] = accADC[axis];
#define ACC_VALUE accADC[axis]
#endif
        {
	    // AccMag += (ACC_VALUE * 10 / acc_1G) * (ACC_VALUE * 10 / acc_1G);
            short temp = (ACC_VALUE * 10 / acc_1G);
            AccMag += temp * temp;
        }
	
#if MAG
#if defined(MG_LPF_FACTOR)
	// LPF for Magnetometer values
	mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR;
#define MAG_VALUE mgSmooth[axis]
#else
#define MAG_VALUE magADC[axis]
#endif
#endif
    }
    // Rotate Estimated vector(s), ROLL
    deltaGyroAngle = gyroADC[ROLL] * deltaTime;
    EstG.V.Z = scos(deltaGyroAngle) * EstG.V.Z - ssin(deltaGyroAngle) * EstG.V.X;
    EstG.V.X = ssin(deltaGyroAngle) * EstG.V.Z + scos(deltaGyroAngle) * EstG.V.X;
#if MAG
    EstM.V.Z = scos(deltaGyroAngle) * EstM.V.Z - ssin(deltaGyroAngle) * EstM.V.X;
    EstM.V.X = ssin(deltaGyroAngle) * EstM.V.Z + scos(deltaGyroAngle) * EstM.V.X;
#endif
    // Rotate Estimated vector(s), PITCH
    deltaGyroAngle = gyroADC[PITCH] * deltaTime;
    EstG.V.Y = scos(deltaGyroAngle) * EstG.V.Y + ssin(deltaGyroAngle) * EstG.V.Z;
    EstG.V.Z = -ssin(deltaGyroAngle) * EstG.V.Y + scos(deltaGyroAngle) * EstG.V.Z;
#if MAG
    EstM.V.Y = scos(deltaGyroAngle) * EstM.V.Y + ssin(deltaGyroAngle) * EstM.V.Z;
    EstM.V.Z = -ssin(deltaGyroAngle) * EstM.V.Y + scos(deltaGyroAngle) * EstM.V.Z;
#endif
    // Rotate Estimated vector(s), YAW
    deltaGyroAngle = gyroADC[YAW] * deltaTime;
    EstG.V.X = scos(deltaGyroAngle) * EstG.V.X - ssin(deltaGyroAngle) * EstG.V.Y;
    EstG.V.Y = ssin(deltaGyroAngle) * EstG.V.X + scos(deltaGyroAngle) * EstG.V.Y;
#if MAG
    EstM.V.X = scos(deltaGyroAngle) * EstM.V.X - ssin(deltaGyroAngle) * EstM.V.Y;
    EstM.V.Y = ssin(deltaGyroAngle) * EstM.V.X + scos(deltaGyroAngle) * EstM.V.Y;
#endif
    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.4G or <0.6G => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if (!((36 > AccMag) || (AccMag > 196))) {
	for (axis = 0; axis < 3; axis++)
	    EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + ACC_VALUE) * INV_GYR_CMPF_FACTOR;
    }
    // Attitude of the estimated vector
    angle[ROLL] = _atan2(EstG.V.X, EstG.V.Z);
    angle[PITCH] = _atan2(EstG.V.Y, EstG.V.Z);
    GEstG = EstG;
#if MAG
    // Apply complimentary filter (Gyro drift correction)
    for (axis = 0; axis < 3; axis++)
	EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
    // Attitude of the cross product vector GxM
    heading = _atan2(EstG.V.Z * EstM.V.X - EstG.V.X * EstM.V.Z, EstG.V.Y * EstM.V.Z - EstG.V.Z * EstM.V.Y) / 10;
#endif
}

float InvSqrt(float x)
{
    union {
	int32_t i;
	float f;
    } conv;
    conv.f = x;
    conv.i = 0x5f3759df - (conv.i >> 1);
    return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

int32_t isq(int32_t x)
{
    return x * x;
}

#define UPDATE_INTERVAL 25000	// 40hz update rate (20hz LPF on acc)
#define INIT_DELAY      4000000	// 4 sec initialization delay
#define Kp1 0.55f		// PI observer velocity gain
#define Kp2 1.0f		// PI observer position gain
#define Ki  0.001f		// PI observer integral gain (bias cancellation)
#define dt  (UPDATE_INTERVAL / 1000000.0f)

void getEstimatedAltitude()
{
    static uint8_t inited = 0;
    static float AltErrorI = 0.0f;
    static float AccScale = 0.0f;
    static uint32_t DeadLine = INIT_DELAY;
    float AltError;
    float InstAcc = 0.0f;
    float Delta;

    if (currentTime < DeadLine)
	return;
    DeadLine = currentTime + UPDATE_INTERVAL;
    // Soft start
    if (!inited) {
	inited = 1;
	EstAlt = BaroAlt;
	EstVelocity = 0.0f;
	AltErrorI = 0.0f;
	AccScale = (9.80665f / acc_1G);
    }
    // Estimation Error
    AltError = BaroAlt - EstAlt;
    AltErrorI += AltError;
    // Gravity vector correction and projection to the local Z
    InstAcc = (accADC[YAW] * (1 - acc_1G * InvSqrt(isq(accADC[ROLL]) + isq(accADC[PITCH]) + isq(accADC[YAW])))) * AccScale + (Ki) * AltErrorI;
    // Integrators
    Delta = InstAcc * dt + (Kp1 * dt) * AltError;
    EstAlt += ((EstVelocity + Delta * 0.5f) * dt + (Kp2 * dt) * AltError);
    EstVelocity += Delta;
}

/* SENSORS ------------------------------------------------------------------------------ */
// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************
//default board orientation
#if !defined(ACC_ORIENTATION)
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#endif
#if !defined(GYRO_ORIENTATION)
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#endif
#if !defined(MAG_ORIENTATION)
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
#endif

/*** I2C address ***/
#if !defined(ADXL345_ADDRESS)
#define ADXL345_ADDRESS 0x3A
//#define ADXL345_ADDRESS 0xA6   //WARNING: Conflicts with a Wii Motion plus!
#endif

#if !defined(BMA180_ADDRESS)
#define BMA180_ADDRESS 0x80
//#define BMA180_ADDRESS 0x82
#endif

#if !defined(ITG3200_ADDRESS)
#define ITG3200_ADDRESS 0XD0
//#define ITG3200_ADDRESS 0XD2
#endif

#if !defined(MS561101BA_ADDRESS)
#define MS561101BA_ADDRESS 0xEE	//CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)
//#define MS561101BA_ADDRESS 0xEF //CBR=1 0xEF I2C address when pin CSB is connected to HIGH (VCC)
#endif

//ITG3200 and ITG3205 Gyro LPF setting
#if defined(ITG3200_LPF_256HZ) || defined(ITG3200_LPF_188HZ) || defined(ITG3200_LPF_98HZ) || defined(ITG3200_LPF_42HZ) || defined(ITG3200_LPF_20HZ) || defined(ITG3200_LPF_10HZ)
#if defined(ITG3200_LPF_256HZ)
#define ITG3200_SMPLRT_DIV 0	//8000Hz
#define ITG3200_DLPF_CFG   0
#endif
#if defined(ITG3200_LPF_188HZ)
#define ITG3200_SMPLRT_DIV 0	//1000Hz
#define ITG3200_DLPF_CFG   1
#endif
#if defined(ITG3200_LPF_98HZ)
#define ITG3200_SMPLRT_DIV 0
#define ITG3200_DLPF_CFG   2
#endif
#if defined(ITG3200_LPF_42HZ)
#define ITG3200_SMPLRT_DIV 0
#define ITG3200_DLPF_CFG   3
#endif
#if defined(ITG3200_LPF_20HZ)
#define ITG3200_SMPLRT_DIV 0
#define ITG3200_DLPF_CFG   4
#endif
#if defined(ITG3200_LPF_10HZ)
#define ITG3200_SMPLRT_DIV 0
#define ITG3200_DLPF_CFG   5
#endif
#else
//Default settings LPF 256Hz/8000Hz sample
#define ITG3200_SMPLRT_DIV 0	//8000Hz
#define ITG3200_DLPF_CFG   0
#endif

uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;

// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************

// Mask prescaler bits : only 5 bits of TWSR defines the status of each I2C request
#define TW_STATUS_MASK	(1<<TWS7) | (1<<TWS6) | (1<<TWS5) | (1<<TWS4) | (1<<TWS3)
#define TW_STATUS       (TWSR & TW_STATUS_MASK)

void i2c_init(void)
{
#ifdef STM8


#else
#if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
#else
    I2C_PULLUPS_DISABLE
#endif
	TWSR = 0;		// no prescaler => prescaler = 1
    TWBR = ((16000000L / I2C_SPEED) - 16) / 2;	// change the I2C clock rate
    TWCR = 1 << TWEN;		// enable twi module, no interrupt
#endif
}

void i2c_rep_start(uint8_t address)
{
#ifdef STM8

#else
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWSTO);	// send REPEAT START condition
    waitTransmissionI2C();	// wait until transmission completed
    checkStatusI2C();		// check value of TWI Status Register
    TWDR = address;		// send device address
    TWCR = (1 << TWINT) | (1 << TWEN);
    waitTransmissionI2C();	// wail until transmission completed
    checkStatusI2C();		// check value of TWI Status Register
#endif
}

void i2c_rep_stop(void)
{
#ifdef STM8

#else
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    waitTransmissionI2C();
    checkStatusI2C();
#endif
}

void i2c_write(uint8_t data)
{
#ifdef STM8

#else
    TWDR = data;		// send data to the previously addressed device
    TWCR = (1 << TWINT) | (1 << TWEN);
    waitTransmissionI2C();	// wait until transmission completed
    checkStatusI2C();		// check value of TWI Status Register
#endif
}

uint8_t i2c_readAck()
{
#ifdef STM8
    return 0;
#else
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    waitTransmissionI2C();
    return TWDR;
#endif
}

uint8_t i2c_readNak(void)
{
#ifdef STM8
    return 0;
#else
    TWCR = (1 << TWINT) | (1 << TWEN);
    waitTransmissionI2C();
    return TWDR;
#endif
}

void waitTransmissionI2C()
{
#ifdef STM8

#else
    uint8_t count = 255;
    while (count-- > 0 && !(TWCR & (1 << TWINT)));
    if (count < 2) {		//we are in a blocking state => we don't insist
	TWCR = 0;		//and we force a reset on TWINT register
	neutralizeTime = micros();	//we take a timestamp here to neutralize the value during a short delay after the hard reset
    }
#endif
}

void checkStatusI2C()
{
#ifdef STM8

#else
    if (TW_STATUS == 0xF8) {	//TW_NO_INFO : this I2C error status indicates a wrong I2C communication.
	// WMP does not respond anymore => we do a hard reset. I did not find another way to solve it. It takes only 13ms to reset and init to WMP or WMP+NK
	TWCR = 0;
	if (!GYRO) {
	    POWERPIN_OFF;	//switch OFF WMP
		delay(1);
	    POWERPIN_ON;	//switch ON WMP
		delay(10);
	    WMP_init(0);
	}
	neutralizeTime = micros();	//we take a timestamp here to neutralize the WMP or WMP+NK values during a short delay after the hard reset
    }
#endif
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg)
{
#ifdef STM8

#else
    i2c_rep_start(add);
    i2c_write(reg);		// Start multiple read at the reg register
    i2c_rep_start(add + 1);	// I2C read direction => I2C address + 1
    for (uint8_t i = 0; i < 5; i++) {
	rawADC[i] = i2c_readAck();
    }
    rawADC[5] = i2c_readNak();
#endif
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val)
{
    i2c_rep_start(add + 0);	// I2C write direction
    i2c_write(reg);		// register selection
    i2c_write(val);		// value to write in register
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg)
{
    i2c_rep_start(add + 0);	// I2C write direction
    i2c_write(reg);		// register selection
    i2c_rep_start(add + 1);	// I2C read direction
    return i2c_readNak();	// Read single register and return value
}

// ****************
// GYRO common part
// ****************
void GYRO_Common()
{
    static int16_t previousGyroADC[3] = { 0, 0, 0 };
    static int32_t g[3];
    uint8_t axis;

    if (calibratingG > 0) {
	for (axis = 0; axis < 3; axis++) {
	    // Reset g[axis] at start of calibration
	    if (calibratingG == 400)
		g[axis] = 0;
	    // Sum up 400 readings
	    g[axis] += gyroADC[axis];
	    // Clear global variables for next reading
	    gyroADC[axis] = 0;
	    gyroZero[axis] = 0;
	    if (calibratingG == 1) {
		gyroZero[axis] = g[axis] / 400;
		blinkLED(10, 15, 1 + 3 * nunchuk);
	    }
	}
	calibratingG--;
    }
    for (axis = 0; axis < 3; axis++) {
	gyroADC[axis] -= gyroZero[axis];
	//anti gyro glitch, limit the variation between two consecutive readings
	gyroADC[axis] = constrain(gyroADC[axis], previousGyroADC[axis] - 800, previousGyroADC[axis] + 800);
	previousGyroADC[axis] = gyroADC[axis];
    }
}

// ****************
// ACC common part
// ****************
void ACC_Common()
{
    static int32_t a[3];
    uint8_t axis;

    if (calibratingA > 0) {
	for (axis = 0; axis < 3; axis++) {
	    // Reset a[axis] at start of calibration
	    if (calibratingA == 400)
		a[axis] = 0;
	    // Sum up 400 readings
	    a[axis] += accADC[axis];
	    // Clear global variables for next reading
	    accADC[axis] = 0;
	    accZero[axis] = 0;
	}
	// Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
	if (calibratingA == 1) {
	    accZero[ROLL] = a[ROLL] / 400;
	    accZero[PITCH] = a[PITCH] / 400;
	    accZero[YAW] = a[YAW] / 400 - acc_1G;	// for nunchuk 200=1G
	    accTrim[ROLL] = 0;
	    accTrim[PITCH] = 0;
	    writeParams();	// write accZero in EEPROM
	}
	calibratingA--;
    }
    accADC[ROLL] -= accZero[ROLL];
    accADC[PITCH] -= accZero[PITCH];
    accADC[YAW] -= accZero[YAW];
}


// ************************************************************************************************************
// I2C Barometer BOSCH BMP085
// ************************************************************************************************************
// I2C adress: 0xEE (8bit)   0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)
// ************************************************************************************************************

#if defined(BMP085)
#define BMP085_ADDRESS 0xEE
static struct {
    // sensor registers from the BOSCH BMP085 datasheet
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
    union {
	uint16_t val;
	uint8_t raw[2];
    } ut;			//uncompensated T
    union {
	uint32_t val;
	uint8_t raw[4];
    } up;			//uncompensated P
    uint8_t state;
    uint32_t deadline;
} bmp085_ctx;
#define OSS 3

void i2c_BMP085_readCalibration()
{
    delay(10);
    bmp085_ctx.ac1 = i2c_BMP085_readIntRegister(0xAA);
    bmp085_ctx.ac2 = i2c_BMP085_readIntRegister(0xAC);
    bmp085_ctx.ac3 = i2c_BMP085_readIntRegister(0xAE);
    bmp085_ctx.ac4 = i2c_BMP085_readIntRegister(0xB0);
    bmp085_ctx.ac5 = i2c_BMP085_readIntRegister(0xB2);
    bmp085_ctx.ac6 = i2c_BMP085_readIntRegister(0xB4);
    bmp085_ctx.b1 = i2c_BMP085_readIntRegister(0xB6);
    bmp085_ctx.b2 = i2c_BMP085_readIntRegister(0xB8);
    bmp085_ctx.mb = i2c_BMP085_readIntRegister(0xBA);
    bmp085_ctx.mc = i2c_BMP085_readIntRegister(0xBC);
    bmp085_ctx.md = i2c_BMP085_readIntRegister(0xBE);
}

void Baro_init()
{
    delay(10);
    i2c_BMP085_readCalibration();
    i2c_BMP085_UT_Start();
    delay(5);
    i2c_BMP085_UT_Read();
}

// read a 16 bit register
int16_t i2c_BMP085_readIntRegister(uint8_t r)
{
    union {
	int16_t val;
	uint8_t raw[2];
    } data;
    i2c_rep_start(BMP085_ADDRESS + 0);
    i2c_write(r);
    i2c_rep_start(BMP085_ADDRESS + 1);	//I2C read direction => 1
    data.raw[1] = i2c_readAck();
    data.raw[0] = i2c_readNak();
    return data.val;
}

// read uncompensated temperature value: send command first
void i2c_BMP085_UT_Start()
{
    i2c_writeReg(BMP085_ADDRESS, 0xf4, 0x2e);
    i2c_rep_start(BMP085_ADDRESS + 0);
    i2c_write(0xF6);
}

// read uncompensated pressure value: send command first
void i2c_BMP085_UP_Start()
{
    i2c_writeReg(BMP085_ADDRESS, 0xf4, 0x34 + (OSS << 6));	// control register value for oversampling setting 3
    i2c_rep_start(BMP085_ADDRESS + 0);	//I2C write direction => 0
    i2c_write(0xF6);
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_BMP085_UP_Read()
{
    i2c_rep_start(BMP085_ADDRESS + 1);	//I2C read direction => 1
    bmp085_ctx.up.raw[2] = i2c_readAck();
    bmp085_ctx.up.raw[1] = i2c_readAck();
    bmp085_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_BMP085_UT_Read()
{
    i2c_rep_start(BMP085_ADDRESS + 1);	//I2C read direction => 1
    bmp085_ctx.ut.raw[1] = i2c_readAck();
    bmp085_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_BMP085_Calculate()
{
    int32_t x1, x2, x3, b3, b5, b6, p, tmp;
    uint32_t b4, b7;
    // Temperature calculations
    x1 = ((int32_t) bmp085_ctx.ut.val - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;
    x2 = ((int32_t) bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
    b5 = x1 + x2;
    // Pressure calculations
    b6 = b5 - 4000;
    x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12)) >> 11;
    x2 = bmp085_ctx.ac2 * b6 >> 11;
    x3 = x1 + x2;
    tmp = bmp085_ctx.ac1;
    tmp = (tmp * 4 + x3) << OSS;
    b3 = (tmp + 2) / 4;
    x1 = bmp085_ctx.ac3 * b6 >> 13;
    x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (bmp085_ctx.ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) (bmp085_ctx.up.val >> (8 - OSS)) - b3) * (50000 >> OSS);
    p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    pressure = p + ((x1 + x2 + 3791) >> 4);
}

void Baro_update()
{
    if (currentTime < bmp085_ctx.deadline)
	return;
    bmp085_ctx.deadline = currentTime;
    TWBR = ((16000000L / 400000L) - 16) / 2;	// change the I2C clock rate to 400kHz, BMP085 is ok with this speed
    switch (bmp085_ctx.state) {
    case 0:
	i2c_BMP085_UT_Start();
	bmp085_ctx.state++;
	bmp085_ctx.deadline += 4600;
	break;
    case 1:
	i2c_BMP085_UT_Read();
	bmp085_ctx.state++;
	break;
    case 2:
	i2c_BMP085_UP_Start();
	bmp085_ctx.state++;
	bmp085_ctx.deadline += 26000;
	break;
    case 3:
	i2c_BMP085_UP_Read();
	i2c_BMP085_Calculate();
	BaroAlt = (1.0f - pow(pressure / 101325.0f, 0.190295f)) * 44330.0f;
	bmp085_ctx.state = 0;
	bmp085_ctx.deadline += 20000;
	break;
    }
}
#endif

// ************************************************************************************************************
// I2C Barometer MS561101BA
// ************************************************************************************************************
// contribution from fabio
//
// not tested
#if defined(MS561101BA)

// registers of the device
#define MS561101BA_PRESSURE 0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

static struct {
    // sensor registers from the MS561101BA datasheet
    uint16_t c1, c2, c3, c4, c5, c6;
    union {
	uint32_t val;
	uint8_t raw[4];
    } ut;			//uncompensated T
    union {
	uint32_t val;
	uint8_t raw[4];
    } up;			//uncompensated P
    uint8_t state;
    uint32_t deadline;
} ms561101ba_ctx;
#define OSR MS561101BA_OSR_4096

void i2c_MS561101BA_reset()
{
    i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);
}

void i2c_MS561101BA_readCalibration()
{
    delay(10);
    ms561101ba_ctx.c1 = i2c_MS561101BA_readIntRegister(0xA2);
    ms561101ba_ctx.c2 = i2c_MS561101BA_readIntRegister(0xA4);
    ms561101ba_ctx.c3 = i2c_MS561101BA_readIntRegister(0xA6);
    ms561101ba_ctx.c4 = i2c_MS561101BA_readIntRegister(0xA8);
    ms561101ba_ctx.c5 = i2c_MS561101BA_readIntRegister(0xAA);
    ms561101ba_ctx.c6 = i2c_MS561101BA_readIntRegister(0xAC);
}

void Baro_init()
{
    delay(10);
    i2c_MS561101BA_reset();
    delay(10);
    i2c_MS561101BA_readCalibration();
    i2c_MS561101BA_UT_Start();
    delay(10);
    i2c_MS561101BA_UT_Read();
}

// read a 16 bit register
int16_t i2c_MS561101BA_readIntRegister(uint8_t r)
{
    union {
	int16_t val;
	uint8_t raw[2];
    } data;
    i2c_rep_start(MS561101BA_ADDRESS + 0);
    i2c_write(r);
    i2c_rep_start(MS561101BA_ADDRESS + 1);	//I2C read direction => 1
    data.raw[1] = i2c_readAck();
    data.raw[0] = i2c_readNak();
    return data.val;
}

// read uncompensated temperature value: send command first
void i2c_MS561101BA_UT_Start()
{
    i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_TEMPERATURE + OSR, 0);
    //i2c_rep_start(MS561101BA_ADDRESS + 0);
    //i2c_write(0xF6);
}

// read uncompensated pressure value: send command first
void i2c_MS561101BA_UP_Start()
{
    i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_PRESSURE + OSR, 0);	// control register value for oversampling setting 3
    //i2c_rep_start(MS561101BA_ADDRESS + 0); //I2C write direction => 0
    //i2c_write(0xF6);
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_MS561101BA_UP_Read()
{
    i2c_rep_start(MS561101BA_ADDRESS + 1);	//I2C read direction => 1
    ms561101ba_ctx.up.raw[2] = i2c_readAck();
    ms561101ba_ctx.up.raw[1] = i2c_readAck();
    ms561101ba_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_MS561101BA_UT_Read()
{
    i2c_rep_start(MS561101BA_ADDRESS + 0);
    i2c_write(0);
    i2c_rep_start(MS561101BA_ADDRESS + 1);	//I2C read direction => 1
    ms561101ba_ctx.ut.raw[2] = i2c_readAck();
    ms561101ba_ctx.ut.raw[1] = i2c_readAck();
    ms561101ba_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_MS561101BA_Calculate()
{
    // see datasheet page 7 for formulas
    int32_t dT = ms561101ba_ctx.ut.val - ms561101ba_ctx.c5 * pow(2, 8);
    int64_t off = ms561101ba_ctx.c2 * pow(2, 16) + (ms561101ba_ctx.c4 * dT) / pow(2, 7);
    int64_t sens = ms561101ba_ctx.c1 * pow(2, 15) + (ms561101ba_ctx.c3 * dT) / pow(2, 8);
    pressure = (ms561101ba_ctx.ut.val * (sens) / pow(2, 21) - off) / pow(2, 15);
}

void Baro_update()
{
    if (currentTime < ms561101ba_ctx.deadline)
	return;
    ms561101ba_ctx.deadline = currentTime;
    TWBR = ((16000000L / 400000L) - 16) / 2;	// change the I2C clock rate to 400kHz, MS5611 is ok with this speed
    switch (ms561101ba_ctx.state) {
    case 0:
	i2c_MS561101BA_UT_Start();
	ms561101ba_ctx.state++;
	ms561101ba_ctx.deadline += 10000;
	break;
    case 1:
	i2c_MS561101BA_UT_Read();
	ms561101ba_ctx.state++;
	break;
    case 2:
	i2c_MS561101BA_UP_Start();
	ms561101ba_ctx.state++;
	ms561101ba_ctx.deadline += 10000;
	break;
    case 3:
	i2c_MS561101BA_UP_Read();
	i2c_MS561101BA_Calculate();
	BaroAlt = (1.0f - pow(pressure / 101325.0f, 0.190295f)) * 44330.0f;
	ms561101ba_ctx.state = 0;
	ms561101ba_ctx.deadline += 20000;
	break;
    }
}
#endif




// ************************************************************************************************************
// I2C Accelerometer ADXL345 
// ************************************************************************************************************
// I2C adress: 0x3A (8bit)    0x1D (7bit)
// Resolution: 10bit (Full range - 14bit, but this is autoscaling 10bit ADC to the range +- 16g)
// principle:
//  1) CS PIN must be linked to VCC to select the I2C mode
//  2) SD0 PIN must be linked to VCC to select the right I2C adress
//  3) bit  b00000100 must be set on register 0x2D to read data (only once at the initialization)
//  4) bits b00001011 must be set on register 0x31 to select the data format (only once at the initialization)
// ************************************************************************************************************
#if defined(ADXL345)
void ACC_init()
{
    delay(10);
    i2c_writeReg(ADXL345_ADDRESS, 0x2D, 1 << 3);	//  register: Power CTRL  -- value: Set measure bit 3 on
    i2c_writeReg(ADXL345_ADDRESS, 0x31, 0x0B);	//  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
    i2c_writeReg(ADXL345_ADDRESS, 0x2C, 8 + 2 + 1);	// register: BW_RATE     -- value: 200Hz sampling (see table 5 of the spec)
    acc_1G = 256;
}

void ACC_getADC()
{
    TWBR = ((16000000L / 400000L) - 16) / 2;	// change the I2C clock rate to 400kHz, ADXL435 is ok with this speed
    i2c_getSixRawADC(ADXL345_ADDRESS, 0x32);

    ACC_ORIENTATION(-((rawADC[3] << 8) | rawADC[2]), ((rawADC[1] << 8) | rawADC[0]), ((rawADC[5] << 8) | rawADC[4]));
    ACC_Common();
}
#endif

#if defined(ADXL345SPI)

/* Private defines */
#define ADXL_OFF	   GPIO_WriteHigh(GPIOE, GPIO_PIN_5);
#define ADXL_ON		   GPIO_WriteLow(GPIOE, GPIO_PIN_5);

#define ADXL_READ_BIT      0x80
#define ADXL_MULTI_BIT     0x40
#define ADXL_X0_ADDR       0x32
#define ADXL_RATE_ADDR     0x2C
#define ADXL_FIFO_ADDR     0x38
#define ADXL_RATE_100      0x0A
#define ADXL_RATE_200      0x0B
#define ADXL_RATE_400      0x0C
#define ADXL_RATE_800      0x0D
#define ADXL_RATE_1600     0x0E
#define ADXL_RATE_3200     0x0F
#define ADXL_POWER_ADDR    0x2D
#define ADXL_MEASURE       0x08
#define ADXL_FORMAT_ADDR   0x31
#define ADXL_FULL_RES      0x08
#define ADXL_4WIRE         0x00
#define ADXL_RANGE_2G      0x00
#define ADXL_RANGE_4G      0x01
#define ADXL_RANGE_8G      0x02
#define ADXL_RANGE_16G     0x03
#define ADXL_FIFO_STREAM   0x80

static u8 ADXL_WriteByte(u8 Data)
{
    /* Wait until the transmit buffer is empty */
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);
    /* Send the byte */
    SPI_SendData(Data);
    /* Wait to receive a byte*/
    while(SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
    /*Return the byte read from the SPI bus */ 
    return SPI_ReceiveData();
}

static u8 ADXL_ReadByte(void)
{
    volatile u8 Data = 0;
    /* Wait until the transmit buffer is empty */
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);
    /* Send the byte */
    SPI_SendData(0xFF); // Dummy Byte
    /* Wait until a data is received */
    while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
    /* Get the received data */
    Data = SPI_ReceiveData();
    /* Return the shifted data */
    return Data;
}

static void ADXL_Init(void)
{
    // setup ADXL345 rate/range/start measuring
    
    // Rate 3200Hz
    ADXL_ON;
    ADXL_WriteByte(ADXL_RATE_ADDR);
    ADXL_WriteByte(ADXL_RATE_3200 & 0x0F);
    ADXL_OFF;

    // Range 8G
    ADXL_ON;
    ADXL_WriteByte(ADXL_FORMAT_ADDR);
    ADXL_WriteByte((ADXL_RANGE_16G & 0x03) | ADXL_FULL_RES | ADXL_4WIRE);
    ADXL_OFF;

    // Fifo depth = 16
    ADXL_ON;
    ADXL_WriteByte(ADXL_FIFO_ADDR);
    ADXL_WriteByte((16 & 0x1f) | ADXL_FIFO_STREAM);
    ADXL_OFF;

    ADXL_ON;
    ADXL_WriteByte(ADXL_POWER_ADDR);
    ADXL_WriteByte(ADXL_MEASURE);
    ADXL_OFF;
}

static u8 ADXL_GetAccelValues(void)
{
    volatile u8 i;
    
    ADXL_ON;
    ADXL_WriteByte(ADXL_X0_ADDR | ADXL_MULTI_BIT | ADXL_READ_BIT);
    for (i = 0; i < 3; i++) {
        u8 i1, i2;
        i1 = ADXL_ReadByte();
        i2 = ADXL_ReadByte();

#ifdef LOWPASS_ACC
        // new result = 0.95 * previous_result + 0.05 * current_data
        sensorInputs[i + 4] = ((sensorInputs[i + 4] * 19) / 20) + (((i1 | i2 << 8) * 5) / 100);
#else
        sensorInputs[i + 4] += (i1 | i2 << 8);
#endif
    }

    // skip over this register
    ADXL_ReadByte();
    // FIFO_STATUS register (last few bits = fifo remaining)
    i = ADXL_ReadByte();
    ADXL_OFF;

    return i & 0x7F; // return number of entires left in fifo
}

void ACC_init()
{
    // SPI
    SPI_DeInit();
    SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER, SPI_CLOCKPOLARITY_HIGH, SPI_CLOCKPHASE_2EDGE, SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 0x07);
    SPI_Cmd(ENABLE);

    // SPI ChipSelect for Accel
    GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);
    ADXL_OFF;

    delay(10);

    // Accel INT1 input tied to interrupt (TODO). Input-only for now.
    GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_IN_FL_NO_IT);

    // i2c_writeReg(ADXL345_ADDRESS, 0x2D, 1 << 3);	//  register: Power CTRL  -- value: Set measure bit 3 on
    // i2c_writeReg(ADXL345_ADDRESS, 0x31, 0x0B);	//  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
    // i2c_writeReg(ADXL345_ADDRESS, 0x2C, 8 + 2 + 1);	// register: BW_RATE     -- value: 200Hz sampling (see table 5 of the spec)
    // Initialize SPI Accelerometer
    ADXL_Init();
    acc_1G = 256;
}

void ACC_getADC()
{
    u8 count = 0;
    u8 remaining = 0;
    u8 i = 0;

    // Next up is accel fifo + avg
    do {
        count++;
        remaining = ADXL_GetAccelValues();
    } while ((count < 32) && (remaining > 0));

#ifdef LOWPASS_ACC
    // commit current values to acc[]
#else
    // accel + average
    sensorInputs[4] = sensorInputs[4] / count;
    sensorInputs[5] = sensorInputs[5] / count;
    sensorInputs[6] = sensorInputs[6] / count;
#endif
    
    // i2c_getSixRawADC(ADXL345_ADDRESS, 0x32);
    ACC_ORIENTATION( sensorInputs[4], sensorInputs[5], sensorInputs[6]);
    ACC_Common();
}
#endif

// ************************************************************************************************************
// contribution initially from opie11 (rc-groups)
// adaptation from C2po (may 2011)
// contribution from ziss_dm (June 2011)
// contribution from ToLuSe (Jully 2011)
// I2C Accelerometer BMA180
// ************************************************************************************************************
// I2C adress: 0x80 (8bit)    0x40 (7bit) (SDO connection to VCC) 
// I2C adress: 0x82 (8bit)    0x41 (7bit) (SDO connection to VDDIO)
// Resolution: 14bit
//
// Control registers:
//
// 0x20    bw_tcs:   |                                           bw<3:0> |                        tcs<3:0> |
//                   |                                             150Hz |                 !!Calibration!! |
//
// 0x35 offset_lsb1: |                                     offset_x<3:0> |        range<2:0> |    smp_skip |
//                   |                                   !!Calibration!! |                2g |     IRQ 1/T |
// ************************************************************************************************************
#if defined(BMA180)
void ACC_init()
{
    delay(10);
    //default range 2G: 1G = 4096 unit.
    i2c_writeReg(BMA180_ADDRESS, 0x0D, 1 << 4);	// register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
    uint8_t control = i2c_readReg(BMA180_ADDRESS, 0x20);
    control = control & 0x0F;	// register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 10Hz (bits value = 0000xxxx)
    delay(5);
    i2c_writeReg(BMA180_ADDRESS, 0x20, control);
    acc_1G = 512;
}

void ACC_getADC()
{
    TWBR = ((16000000L / 400000L) - 16) / 2;	// Optional line.  Sensor is good for it in the spec.
    i2c_getSixRawADC(BMA180_ADDRESS, 0x02);
    //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /8 => 11 bit resolution
    ACC_ORIENTATION(-((rawADC[1] << 8) | rawADC[0]) / 32, -((rawADC[3] << 8) | rawADC[2]) / 32, ((rawADC[5] << 8) | rawADC[4]) / 32);
    ACC_Common();
}
#endif

// ************************************************************************************************************
// contribution from Point65 and mgros (rc-groups)
// contribution from ziss_dm (June 2011)
// contribution from ToLuSe (Jully 2011)
// I2C Accelerometer BMA020
// ************************************************************************************************************
// I2C adress: 0x70 (8bit)
// Resolution: 10bit
// Control registers:
//
// Datasheet: After power on reset or soft reset it is recommended to set the SPI4-bit to the correct value.
//            0x80 = SPI four-wire = Default setting
// | 0x15: | SPI4 | enable_adv_INT | new_data_INT | latch_INT | shadow_dis | wake_up_pause<1:0> | wake_up |
// |       |    1 |              0 |            0 |         0 |          0 |                 00 |       0 |
//
// | 0x14: |                       reserved <2:0> |            range <1:0> |               bandwith <2:0> |
// |       |                      !!Calibration!! |                     2g |                         25Hz |
//
// ************************************************************************************************************
#if defined(BMA020)
void ACC_init()
{
    i2c_writeReg(0x70, 0x15, 0x80);
    uint8_t control = i2c_readReg(0x70, 0x14);
    control = control & 0xE0;
    control = control | (0x00 << 3);	//Range 2G 00
    control = control | 0x00;	//Bandwidth 25 Hz 000
    i2c_writeReg(0x70, 0x14, control);
    acc_1G = 255;
}

void ACC_getADC()
{
    TWBR = ((16000000L / 400000L) - 16) / 2;
    i2c_getSixRawADC(0x70, 0x02);
    ACC_ORIENTATION(((rawADC[1] << 8) | rawADC[0]) / 64, ((rawADC[3] << 8) | rawADC[2]) / 64, ((rawADC[5] << 8) | rawADC[4]) / 64);
    ACC_Common();
}
#endif

// ************************************************************************************************************
// standalone I2C Nunchuk
// ************************************************************************************************************
#if defined(NUNCHACK)
void ACC_init()
{
    i2c_writeReg(0xA4, 0xF0, 0x55);
    i2c_writeReg(0xA4, 0xFB, 0x00);
    delay(250);
    acc_1G = 200;
}

void ACC_getADC()
{
    TWBR = ((16000000L / I2C_SPEED) - 16) / 2;	// change the I2C clock rate. !! you must check if the nunchuk is ok with this freq
    i2c_getSixRawADC(0xA4, 0x00);

    ACC_ORIENTATION(((rawADC[3] << 2) + ((rawADC[5] >> 4) & 0x2)), -((rawADC[2] << 2) + ((rawADC[5] >> 3) & 0x2)), (((rawADC[4] & 0xFE) << 2) + ((rawADC[5] >> 5) & 0x6)));
    ACC_Common();
}
#endif

// ************************************************************************
// LIS3LV02 I2C Accelerometer
//contribution from adver (http://wbb.multiwii.com/viewtopic.php?f=8&t=451)
// ************************************************************************
#if defined(LIS3LV02)
#define LIS3A  0x3A		// I2C adress: 0x3A (8bit)

void i2c_ACC_init()
{
    i2c_writeReg(LIS3A, 0x20, 0xD7);	// CTRL_REG1   1101 0111 Pwr on, 160Hz 
    i2c_writeReg(LIS3A, 0x21, 0x50);	// CTRL_REG2   0100 0000 Littl endian, 12 Bit, Boot
    acc_1G = 256;
}

void i2c_ACC_getADC()
{
    TWBR = ((16000000L / 400000L) - 16) / 2;	// change the I2C clock rate to 400kHz
    i2c_getSixRawADC(LIS3A, 0x28 + 0x80);
    ACC_ORIENTATION((rawADC[3] << 8 | rawADC[2]) / 4, -(rawADC[1] << 8 | rawADC[0]) / 4, -(rawADC[5] << 8 | rawADC[4]) / 4);
    ACC_Common();
}
#endif



// ************************************************************************************************************
// ADC ACC
// ************************************************************************************************************
#if defined(ADCACC)
void ACC_init()
{
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    acc_1G = 75;
}

void ACC_getADC()
{
    ACC_ORIENTATION(-analogRead(A1), -analogRead(A2), analogRead(A3));
    ACC_Common();
}
#endif

// ************************************************************************************************************
// Analog Gyroscopes IDG500 + ISZ500
// ************************************************************************************************************
#ifdef STM8

static volatile u8 adcInProgress = 0;

__near __interrupt void ADC1_IRQHandler(void)
{
    u8 i = 0;

#if 0
    // clear at start of loop
    if (adcSampleCount == 0 || adcSampleCount > 30) {
        sensorInputs[0] = 0;
        sensorInputs[1] = 0;
        sensorInputs[2] = 0;
        sensorInputs[3] = 0;
        adcSampleCount = 0;
    }
    adcSampleCount++;
#endif

    // Get 4 ADC readings from buffer
    for (i = 0; i < 4; i++)
        sensorInputs[i] = ADC1_GetBufferValue(i);

    ADC1_ClearITPendingBit(ADC1_CSR_EOC);
    adcInProgress = 0;
}
#endif

#if defined(STM8) && defined(ADCGYRO)
void Gyro_init()
{
    #if 1
    // ADC1
    ADC1_DeInit();
    GPIO_Init(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_MODE_IN_FL_NO_IT);
    ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_3, ADC1_PRESSEL_FCPU_D8, ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_ALL, DISABLE);
    ADC1_DataBufferCmd(ENABLE);
    ADC1_ScanModeCmd(ENABLE);
    ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);
    #endif
}

void Gyro_getADC()
{
    #if 1
    // read out
    adcInProgress = 1;
    ADC1_StartConversion();
    while (adcInProgress) { }; // wait for conversion

    GYRO_ORIENTATION( -(sensorInputs[0] * 16), (sensorInputs[1] * 16), -(sensorInputs[2] * 16) );
    GYRO_Common();
    #endif
}
#endif

// ************************************************************************************************************
// contribution from Ciskje
// I2C Gyroscope L3G4200D 
// ************************************************************************************************************
#if defined(L3G4200D)
void Gyro_init()
{
    delay(100);
    i2c_writeReg(0XD2 + 0, 0x20, 0x8F);	// CTRL_REG1   400Hz ODR, 20hz filter, run!
    delay(5);
    i2c_writeReg(0XD2 + 0, 0x24, 0x02);	// CTRL_REG5   low pass filter enable
}

void Gyro_getADC()
{
    TWBR = ((16000000L / 400000L) - 16) / 2;	// change the I2C clock rate to 400kHz
    i2c_getSixRawADC(0XD2, 0x80 | 0x28);

    GYRO_ORIENTATION(((rawADC[1] << 8) | rawADC[0]) / 20, ((rawADC[3] << 8) | rawADC[2]) / 20, -((rawADC[5] << 8) | rawADC[4]) / 20);
    GYRO_Common();
}
#endif

// ************************************************************************************************************
// I2C Gyroscope ITG3200 
// ************************************************************************************************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)
// I2C adress: 0xD0 (8bit)   0x68 (7bit)
// principle:
// 1) VIO is connected to VDD
// 2) I2C adress is set to 0x69 (AD0 PIN connected to VDD)
// or 2) I2C adress is set to 0x68 (AD0 PIN connected to GND)
// 3) sample rate = 1000Hz ( 1kHz/(div+1) )
// ************************************************************************************************************
#if defined(ITG3200)
void Gyro_init()
{
    delay(100);
    i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x80);	//register: Power Management  --  value: reset device
    //  delay(5);
    //  i2c_writeReg(ITG3200_ADDRESS, 0x15, ITG3200_SMPLRT_DIV); //register: Sample Rate Divider  -- default value = 0: OK
    delay(5);
    i2c_writeReg(ITG3200_ADDRESS, 0x16, 0x18 + ITG3200_DLPF_CFG);	//register: DLPF_CFG - low pass filter configuration
    delay(5);
    i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x03);	//register: Power Management  --  value: PLL with Z Gyro reference
    delay(100);
}

void Gyro_getADC()
{
    TWBR = ((16000000L / 400000L) - 16) / 2;	// change the I2C clock rate to 400kHz
    i2c_getSixRawADC(ITG3200_ADDRESS, 0X1D);
    GYRO_ORIENTATION(+(((rawADC[2] << 8) | rawADC[3]) / 4),	// range: +/- 8192; +/- 2000 deg/sec
		     -(((rawADC[0] << 8) | rawADC[1]) / 4), -(((rawADC[4] << 8) | rawADC[5]) / 4));
    GYRO_Common();
}
#endif



// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************
#if MAG
void Mag_getADC()
{
    static uint32_t t, tCal = 0;
    static int16_t magZeroTempMin[3];
    static int16_t magZeroTempMax[3];
    uint8_t axis;
    if ((micros() - t) < 100000)
	return;			//each read is spaced by 100ms
    t = micros();
    TWBR = ((16000000L / 400000L) - 16) / 2;	// change the I2C clock rate to 400kHz
    Device_Mag_getADC();
    if (calibratingM == 1) {
	tCal = t;
	for (axis = 0; axis < 3; axis++) {
	    magZero[axis] = 0;
	    magZeroTempMin[axis] = 0;
	    magZeroTempMax[axis] = 0;
	}
	calibratingM = 0;
    }
    magADC[ROLL] -= magZero[ROLL];
    magADC[PITCH] -= magZero[PITCH];
    magADC[YAW] -= magZero[YAW];
    if (tCal != 0) {
	if ((t - tCal) < 30000000) {	// 30s: you have 30s to turn the multi in all directions
	    LEDPIN_SWITCH;
	    for (axis = 0; axis < 3; axis++) {
		if (magADC[axis] < magZeroTempMin[axis])
		    magZeroTempMin[axis] = magADC[axis];
		if (magADC[axis] > magZeroTempMax[axis])
		    magZeroTempMax[axis] = magADC[axis];
	    }
	} else {
	    tCal = 0;
	    for (axis = 0; axis < 3; axis++)
		magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) / 2;
	    writeParams();
	}
    }
}
#endif

// ************************************************************************************************************
// I2C Compass HMC5843 & HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************
#if defined(HMC5843) || defined(HMC5883)
void Mag_init()
{
    delay(100);
    i2c_writeReg(0X3C, 0x02, 0x00);	//register: Mode register  --  value: Continuous-Conversion Mode
}

void Device_Mag_getADC()
{
    i2c_getSixRawADC(0X3C, 0X03);
#if defined(HMC5843)
    MAG_ORIENTATION(((rawADC[0] << 8) | rawADC[1]), ((rawADC[2] << 8) | rawADC[3]), -((rawADC[4] << 8) | rawADC[5]));
#endif
#if defined (HMC5883)
    MAG_ORIENTATION(((rawADC[4] << 8) | rawADC[5]), -((rawADC[0] << 8) | rawADC[1]), -((rawADC[2] << 8) | rawADC[3]));
#endif
}
#endif

// ************************************************************************************************************
// I2C Compass AK8975 (Contribution by EOSBandi)
// ************************************************************************************************************
// I2C adress: 0x18 (8bit)   0x0C (7bit)
// ************************************************************************************************************
#if defined(AK8975)
void Mag_init()
{
    delay(100);
    i2c_writeReg(0x18, 0x0a, 0x01);	//Start the first conversion
    delay(100);
}

void Device_Mag_getADC()
{
    i2c_getSixRawADC(0x18, 0x03);
    MAG_ORIENTATION(((rawADC[3] << 8) | rawADC[2]), ((rawADC[1] << 8) | rawADC[0]), -((rawADC[5] << 8) | rawADC[4]));
    //Start another meassurement
    i2c_writeReg(0x18, 0x0a, 0x01);
}
#endif


#if !GYRO
// ************************************************************************************************************
// I2C Wii Motion Plus + optional Nunchuk
// ************************************************************************************************************
// I2C adress 1: 0xA6 (8bit)    0x53 (7bit)
// I2C adress 2: 0xA4 (8bit)    0x52 (7bit)
// ************************************************************************************************************
void WMP_init(uint8_t d)
{
    delay(d);
    i2c_writeReg(0xA6, 0xF0, 0x55);	// Initialize Extension
    delay(d);
    i2c_writeReg(0xA6, 0xFE, 0x05);	// Activate Nunchuck pass-through mode
    delay(d);
    if (d > 0) {
	// We need to set acc_1G for the Nunchuk beforehand; It's used in WMP_getRawADC() and ACC_Common()
	// If a different accelerometer is used, it will be overwritten by its ACC_init() later.
        uint8_t i;
        uint8_t numberAccRead = 0;
	acc_1G = 200;
	// Read from WMP 100 times, this should return alternating WMP and Nunchuk data
	for (i = 0; i < 100; i++) {
	    delay(4);
	    if (WMP_getRawADC() == 0)
		numberAccRead++;	// Count number of times we read from the Nunchuk extension
	}
	// If we got at least 25 Nunchuck reads, we assume the Nunchuk is present
	if (numberAccRead > 25)
	    nunchuk = 1;
	delay(10);
    }
}

uint8_t WMP_getRawADC()
{
#ifdef STM8

    // TODO
    return 0;

#else
    uint8_t axis;
    TWBR = ((16000000L / I2C_SPEED) - 16) / 2;	// change the I2C clock rate
    i2c_getSixRawADC(0xA4, 0x00);

    if (micros() < (neutralizeTime + NEUTRALIZE_DELAY)) {	//we neutralize data in case of blocking+hard reset state
	for (axis = 0; axis < 3; axis++) {
	    gyroADC[axis] = 0;
	    accADC[axis] = 0;
	}
	accADC[YAW] = acc_1G;
	return 1;
    }
    // Wii Motion Plus Data
    if ((rawADC[5] & 0x03) == 0x02) {
	// Assemble 14bit data 
	gyroADC[ROLL] = -(((rawADC[5] >> 2) << 8) | rawADC[2]);	//range: +/- 8192
	gyroADC[PITCH] = -(((rawADC[4] >> 2) << 8) | rawADC[1]);
	gyroADC[YAW] = -(((rawADC[3] >> 2) << 8) | rawADC[0]);
	GYRO_Common();
	// Check if slow bit is set and normalize to fast mode range
	gyroADC[ROLL] = (rawADC[3] & 0x01) ? gyroADC[ROLL] / 5 : gyroADC[ROLL];	//the ratio 1/5 is not exactly the IDG600 or ISZ650 specification 
	gyroADC[PITCH] = (rawADC[4] & 0x02) >> 1 ? gyroADC[PITCH] / 5 : gyroADC[PITCH];	//we detect here the slow of fast mode WMP gyros values (see wiibrew for more details)
	gyroADC[YAW] = (rawADC[3] & 0x02) >> 1 ? gyroADC[YAW] / 5 : gyroADC[YAW];	// this step must be done after zero compensation    
	return 1;
    } else if ((rawADC[5] & 0x03) == 0x00) {	// Nunchuk Data
	ACC_ORIENTATION(((rawADC[3] << 2) | ((rawADC[5] >> 4) & 0x02)), -((rawADC[2] << 2) | ((rawADC[5] >> 3) & 0x02)), (((rawADC[4] >> 1) << 3) | ((rawADC[5] >> 5) & 0x06)));
	ACC_Common();
	return 0;
    } else
	return 2;
#endif
}
#endif

void initSensors()
{
    delay(200);
    POWERPIN_ON;
    delay(100);
    i2c_init();
    delay(100);
    if (GYRO)
	Gyro_init();
    else
	WMP_init(250);
    if (BARO)
	Baro_init();
    if (ACC)
	ACC_init();
    if (MAG)
	Mag_init();
}

/* SERIAL ---------------------------------------------------------------- */
static uint8_t point;
static uint8_t s[128];
void serialize16(int16_t a)
{
    s[point++] = a;
    s[point++] = a >> 8 & 0xff;
}

void serialize8(uint8_t a)
{
    s[point++] = a;
}

// ***********************************
// Interrupt driven UART transmitter for MIS_OSD
// ***********************************
static uint8_t tx_ptr;
static uint8_t tx_busy = 0;

#ifdef STM8
__near __interrupt void UART2_TX_IRQHandler(void)
{
    UART2_SendData8(s[tx_ptr++]);
    if (tx_ptr == point) {	/* Check if all data is transmitted */
	UART2_ITConfig(UART2_IT_TXE, DISABLE); /* Disable transmitter interrupt */
	tx_busy = 0;
    }
}
#else
ISR_UART {
    UDR0 = s[tx_ptr++];		/* Transmit next byte */
    if (tx_ptr == point) {	/* Check if all data is transmitted */
	UCSR0B &= ~(1 << UDRIE0);	/* Disable transmitter UDRE interrupt */
	tx_busy = 0;
    }
}
#endif

void UartSendData()
{
#ifdef STM8
    tx_ptr = 0;
    UART2_SendData8(s[tx_ptr++]); /* Start transmission */
    UART2_ITConfig(UART2_IT_TXE, ENABLE); /* Enable TX interrupt to continue sending */
#else
    // start of the data block transmission
    cli();
    tx_ptr = 0;
    UCSR0A |= (1 << UDRE0);	/* Clear UDRE interrupt flag */
    UCSR0B |= (1 << UDRIE0);	/* Enable transmitter UDRE interrupt */
    UDR0 = s[tx_ptr++];		/* Start transmission */
    tx_busy = 1;
    sei();
#endif
}

void serialCom()
{
    int16_t a;
    uint8_t i;

    uint16_t intPowerMeterSum, intPowerTrigger1;

    if ((!tx_busy) && Serial_available()) {
	switch (Serial_read()) {
#ifdef LCD_TELEMETRY
	case 'A':		// button A press
	    if (telemetry == 'A')
		telemetry = 0;
	    else {
		telemetry = 'A';
		LCDprint(12);	/* clear screen */
	    }
	    break;
	case 'B':		// button B press
	    if (telemetry == 'B')
		telemetry = 0;
	    else {
		telemetry = 'B';
		LCDprint(12);	/* clear screen */
	    }
	    break;
	case 'C':		// button C press
	    if (telemetry == 'C')
		telemetry = 0;
	    else {
		telemetry = 'C';
		LCDprint(12);	/* clear screen */
	    }
	    break;
	case 'D':		// button D press
	    if (telemetry == 'D')
		telemetry = 0;
	    else {
		telemetry = 'D';
		LCDprint(12);	/* clear screen */
	    }
	    break;
	case 'a':		// button A release
	case 'b':		// button B release
	case 'c':		// button C release
	case 'd':		// button D release
	    break;
#endif
	case 'M':		// Multiwii @ arduino to GUI all data
	    point = 0;
	    serialize8('M');
	    serialize8(VERSION);	// MultiWii Firmware version
	    for (i = 0; i < 3; i++)
		serialize16(accSmooth[i]);
	    for (i = 0; i < 3; i++)
		serialize16(gyroData[i] / 8);
	    for (i = 0; i < 3; i++)
		serialize16(magADC[i] / 3);
	    serialize16(EstAlt * 100.0f);
	    serialize16(heading);	// compass
	    for (i = 0; i < 4; i++)
		serialize16(servo[i]);
	    for (i = 0; i < 8; i++)
		serialize16(motor[i]);
	    for (i = 0; i < 8; i++)
		serialize16(rcData[i]);
	    serialize8(nunchuk | ACC << 1 | BARO << 2 | MAG << 3);
	    serialize8(accMode | baroMode << 1 | magMode << 2);
	    serialize16(cycleTime);
	    for (i = 0; i < 2; i++)
		serialize16(angle[i] / 10);
	    serialize8(MULTITYPE);
	    for (i = 0; i < 5; i++) {
		serialize8(P8[i]);
		serialize8(I8[i]);
		serialize8(D8[i]);
	    }
	    serialize8(P8[PIDLEVEL]);
	    serialize8(I8[PIDLEVEL]);
	    serialize8(P8[PIDMAG]);
	    serialize8(rcRate8);
	    serialize8(rcExpo8);
	    serialize8(rollPitchRate);
	    serialize8(yawRate);
	    serialize8(dynThrPID);
	    for (i = 0; i < 6; i++)
		serialize8(activate[i]);
#if defined(POWERMETER)
	    intPowerMeterSum = (pMeter[PMOTOR_SUM] / PLEVELDIV);
	    intPowerTrigger1 = powerTrigger1 * PLEVELSCALE;
#endif
	    serialize16(intPowerMeterSum);
	    serialize16(intPowerTrigger1);
	    serialize8(vbat);
	    serialize16(BaroAlt * 100.0f);	// 4 variables are here for general monitoring purpose
	    serialize16(0);	// debug2
	    serialize16(0);	// debug3
	    serialize16(0);	// debug4
	    serialize8('M');
	    UartSendData();	// Serial.write(s,point);
	    break;
	case 'O':		// arduino to OSD data - contribution from MIS
	    point = 0;
	    serialize8('O');
	    for (i = 0; i < 3; i++)
		serialize16(accSmooth[i]);
	    for (i = 0; i < 3; i++)
		serialize16(gyroData[i]);
	    serialize16(EstAlt * 10.0f);
	    serialize16(heading);	// compass - 16 bytes
	    for (i = 0; i < 2; i++)
		serialize16(angle[i]);	//20
	    for (i = 0; i < 6; i++)
		serialize16(motor[i]);	//32
	    for (i = 0; i < 6; i++) {
		serialize16(rcData[i]);
	    }			//44
	    serialize8(nunchuk | ACC << 1 | BARO << 2 | MAG << 3);
	    serialize8(accMode | baroMode << 1 | magMode << 2);
	    serialize8(vbat);	// Vbatt 47
	    serialize8(VERSION);	// MultiWii Firmware version
	    serialize8('O');	//49
	    UartSendData();
	    break;
	case 'W':		//GUI write params to eeprom @ arduino
	    while (Serial_available() < 29) { }
	    for (i = 0; i < 5; i++) {
		P8[i] = Serial_read();
		I8[i] = Serial_read();
		D8[i] = Serial_read();
	    }			//9
	    P8[PIDLEVEL] = Serial_read();
	    I8[PIDLEVEL] = Serial_read();	//11
	    P8[PIDMAG] = Serial_read();
	    rcRate8 = Serial_read();
	    rcExpo8 = Serial_read();
	    rollPitchRate = Serial_read();
	    yawRate = Serial_read();	//16
	    dynThrPID = Serial_read();
	    for (i = 0; i < 6; i++)
		activate[i] = Serial_read();	//22
#if defined(POWERMETER)
	    powerTrigger1 = (Serial_read() + 256 * Serial_read()) / PLEVELSCALE;	// we rely on writeParams() to compute corresponding pAlarm value
#else
	    Serial_read();
	    Serial_read();	// so we unload the two bytes
#endif
	    writeParams();
	    break;
	case 'S':		//GUI to arduino ACC calibration request
	    calibratingA = 400;
	    break;
	case 'E':		//GUI to arduino MAG calibration request
	    calibratingM = 1;
	    break;
	}
    }
}
