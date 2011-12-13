/*
/*
MultiWiiCopter by Alexandre Dubus
www.multiwii.com
November  2011     V1.9
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 any later version. see <http://www.gnu.org/licenses/>
*/

#include "config.h"
#include "def.h"
#include "sysdep.h"

#define   VERSION  19

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
#define BOXGPSHOME  6
#define BOXGPSHOLD  7

static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;  // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint16_t calibratingA = 0;       // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
static uint8_t calibratingM = 0;
static uint16_t calibratingG;
static uint8_t armed = 0;
static uint16_t acc_1G;         // this is the 1G measured acceleration
static int16_t acc_25deg;
static uint8_t nunchuk = 0;
static uint8_t accMode = 0;     // if level mode is a activated
static uint8_t magMode = 0;     // if compass heading hold is a activated
static uint8_t baroMode = 0;    // if altitude hold is activated
static uint8_t GPSModeHome = 0; // if GPS RTH is activated
static uint8_t GPSModeHold = 0; // if GPS PH is activated
static int16_t gyroADC[3], accADC[3], magADC[3];
static int16_t accSmooth[3];    // projection of smoothed and normalized gravitation force vector on x/y/z axis, as measured by accelerometer
static int16_t accTrim[2] = { 0, 0 };
static int16_t heading, magHold;
static uint8_t calibratedACC = 0;
static uint8_t vbat;            // battery voltage in 0.1V steps
static uint8_t okToArm = 0;
static uint8_t rcOptions;
static int32_t pressure;
static int32_t BaroAlt;
static int32_t EstVelocity;
static int32_t EstAlt;          // in cm
static uint8_t buzzerState = 0;
static uint16_t i2cErrorCounter = 0;    // number of i2c errors on bus

#ifdef STM8
/* 0:Pitch 1:Roll 2:Yaw 3:Battery Voltage 4:AX 5:AY 6:AZ */
static volatile s16 sensorInputs[7] = { 0, };
#endif

//for log
static uint16_t cycleTimeMax = 0;       // highest ever cycle time
static uint16_t cycleTimeMin = 65535;   // lowest ever cycle time
static uint16_t powerMax = 0;   // highest ever current
static uint16_t powerAvg = 0;   // last known current

// **********************
// power meter
// **********************
#define PMOTOR_SUM 8            // index into pMeter[] for sum
static uint32_t pMeter[PMOTOR_SUM + 1]; //we use [0:7] for eight motors,one extra for sum
static uint8_t pMeterV;         // dummy to satisfy the paramStruct logic in ConfigurationLoop()
static uint32_t pAlarm;         // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
static uint8_t powerTrigger1 = 0;       // trigger for alarm based on power consumption

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

static int16_t failsafeEvents = 0;
static int16_t rcData[8];       // interval [1000;2000]
static int16_t rcCommand[4];    // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 

static uint8_t rcRate8;
static uint8_t rcExpo8;
static int16_t lookupRX[7];     //  lookup table for expo & RC rate
volatile uint8_t rcFrameComplete;       // for serial rc receiver Spektrum

// **************
// gyro+acc IMU
// **************
static int16_t gyroData[3] = { 0, 0, 0 };
static int16_t gyroZero[3] = { 0, 0, 0 };
static int16_t accZero[3] = { 0, 0, 0 };
static int16_t magZero[3] = { 0, 0, 0 };
static int16_t angle[2] = { 0, 0 };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
static int8_t smallAngle25 = 1;

// *************************
// motor and servo functions
// *************************
static int16_t axisPID[3];
static int16_t motor[8];
static int16_t servo[4] = { 1500, 1500, 1500, 1500 };
static uint8_t mixerConfiguration = MULTITYPE_QUADX;
static uint8_t useServo = 0;
static uint8_t numberMotor = 4;

// **********************
// EEPROM & LCD functions
// **********************
static uint8_t P8[7], I8[7], D8[7];     //8 bits is much faster and the code is much shorter
static uint8_t dynP8[3], dynI8[3], dynD8[3];
static uint8_t rollPitchRate;
static uint8_t yawRate;
static uint8_t dynThrPID;
static uint8_t activate[8];

/* prototypes */
void serialCom(void);
void initOutput(void);
void initSensors(void);
void readEEPROM(void);
void checkFirstTime(void);
void configureReceiver(void);
void initializeServo(void);
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
void Device_Mag_getADC(void);
void ACC_Common(void);
void Gyro_init(void);
#if defined(BARO)
void Baro_init(void);
#endif
void ACC_init(void);
void Mag_init(void);

/* local I2C Prototypes */
void i2c_getSixRawADC(uint8_t add, uint8_t reg);
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_readReg(uint8_t add, uint8_t reg);

void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat)
{
    uint8_t i, r;
    for (r = 0; r < repeat; r++) {
        for (i = 0; i < num; i++) {
            LEDPIN_TOGGLE;
            BUZZERPIN_ON;
            delay(wait);
            BUZZERPIN_OFF;
        }
        delay(60);
    }
}

// **********************
// GPS
// **********************
static int32_t GPS_latitude, GPS_longitude;
static int32_t GPS_latitude_home, GPS_longitude_home;
static uint8_t GPS_fix, GPS_fix_home = 0;
static uint8_t GPS_numSat;
static uint16_t GPS_distanceToHome;
static int16_t GPS_directionToHome = 0;
static uint8_t GPS_update = 0;

void annexCode(void)
{
    //this code is executed at each loop and won't interfere with control loop if it lasts less than 650 microseconds
    static uint32_t serialTime;
    static uint32_t buzzerTime, calibratedAccTime, telemetryTime, telemetryAutoTime, psensorTime;
    static uint8_t buzzerFreq;  //delay between buzzer ring
    uint8_t axis, prop1, prop2;
    uint16_t pMeterRaw, powerValue;     //used for current reading
    static uint8_t ind;
    uint16_t vbatRaw = 0;
    static uint16_t vbatRawArray[8];
    uint8_t i;

    // PITCH & ROLL only dynamic PID adjustment,  depending on throttle value
    if (rcData[THROTTLE] < 1500) {
        prop2 = 100;
    } else {
        if (rcData[THROTTLE] < 2000) {
            prop2 = 100 - (uint16_t) dynThrPID *(rcData[THROTTLE] - 1500) / 500;
        } else {
            prop2 = 100 - dynThrPID;
        }
    }

    for (axis = 0; axis < 3; axis++) {
        uint16_t tmp = min(abs(rcData[axis] - MIDRC), 500);
#if DEADBAND
        if (tmp > DEADBAND) {
            tmp -= DEADBAND;
        } else {
            tmp = 0;
        }
#endif
        if (axis != 2) {        // ROLL & PITCH
            uint16_t tmp2 = tmp / 100;
            rcCommand[axis] = lookupRX[tmp2] + (tmp - tmp2 * 100) * (lookupRX[tmp2 + 1] - lookupRX[tmp2]) / 100;
            prop1 = 100 - (uint16_t) rollPitchRate *tmp / 500;
            prop1 = (uint16_t) prop1 *prop2 / 100;
        } else {                // YAW
            rcCommand[axis] = tmp;
            prop1 = 100 - (uint16_t) yawRate *tmp / 500;
        }
        dynP8[axis] = (uint16_t) P8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t) D8[axis] * prop1 / 100;
        if (rcData[axis] < MIDRC)
            rcCommand[axis] = -rcCommand[axis];
    }

    rcCommand[THROTTLE] = MINTHROTTLE + (int32_t) (MAXTHROTTLE - MINTHROTTLE) * (rcData[THROTTLE] - MINCHECK) / (2000 - MINCHECK);

#if (POWERMETER == 2)
    if (micros() > psensorTime + 19977 /*20000 */ ) {   // 50Hz, but avoid bulking of timed tasks
        pMeterRaw = analogRead(PSENSORPIN);
        powerValue = (PSENSORNULL > pMeterRaw ? PSENSORNULL - pMeterRaw : pMeterRaw - PSENSORNULL);     // do not use abs(), it would induce implicit cast to uint and overrun
#ifdef LOG_VALUES
        if (powerValue < 333) { // only accept reasonable values. 333 is empirical
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
#ifdef STM8
    vbatRawArray[(ind++) % 8] = sensorInputs[3];        // VBAT_ADC
#else
    vbatRawArray[(ind++) % 8] = analogRead(V_BATPIN);
#endif
    for (i = 0; i < 8; i++)
        vbatRaw += vbatRawArray[i];
    vbat = (vbatRaw / (VBATSCALE / 4));   // result is Vbatt in 0.1V steps
    if (vbat > 2)
        vbat -= 2;

    if ((vbat > VBATLEVEL1_3S)
#if defined(POWERMETER)
        && ((pMeter[PMOTOR_SUM] < pAlarm) || (pAlarm == 0))
#endif
        || (NO_VBAT > vbat))    // ToLuSe
    {                           //VBAT ok AND powermeter ok, buzzer off
        buzzerFreq = 0;
        buzzerState = 0;
        BUZZERPIN_OFF;
#if defined(POWERMETER)
    } else if (pMeter[PMOTOR_SUM] > pAlarm) {   // sound alarm for powermeter
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

    if ((calibratingA > 0 && (ACC || nunchuk)) || (calibratingG > 0)) { // Calibration phasis
        LEDPIN_TOGGLE;
    } else {
        if (calibratedACC == 1) {
            LEDPIN_OFF;
        }
        if (armed) {
            LEDPIN_ON;
        }
    }

    if (currentTime > calibratedAccTime) {
        if (smallAngle25 == 0) {
            calibratedACC = 0;  //the multi uses ACC and is not calibrated or is too much inclinated
            LEDPIN_TOGGLE;
            calibratedAccTime = currentTime + 500000;
        } else {
            calibratedACC = 1;
        }
    }

    if (currentTime > serialTime) {     // 50Hz
        serialCom();
        serialTime = currentTime + 20000;
    }
#ifdef LCD_TELEMETRY_AUTO
    if ((telemetry_auto) && (micros() > telemetryAutoTime + LCD_TELEMETRY_AUTO)) {      // every 2 seconds
        telemetry++;
        if (telemetry == 'E')
            telemetry = 'Z';
        else if ((telemetry < 'A') || (telemetry > 'D'))
            telemetry = 'A';

        telemetryAutoTime = micros();   // why use micros() and not the variable currentTime ?
    }
#endif
#ifdef LCD_TELEMETRY
    if (micros() > telemetryTime + LCD_TELEMETRY) {     // 10Hz
        if (telemetry)
            lcd_telemetry();
        telemetryTime = micros();
    }
#endif
}

void setup()
{
    LEDPIN_PINMODE;
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
#if defined(GIMBAL)
    calibratingA = 400;
#endif
    calibratingG = 400;
#if defined(POWERMETER)
    for (uint8_t i = 0; i <= PMOTOR_SUM; i++)
        pMeter[i] = 0;
#endif
#if defined(GPS)
    GPS_SERIAL.begin(GPS_BAUD);
#endif
#if defined(LCD_ETPP)
    i2c_ETPP_init();
    i2c_ETPP_set_cursor(0, 0);
    LCDprintChar("MultiWii");
    i2c_ETPP_set_cursor(0, 1);
    LCDprintChar("Ready to Fly!");
#endif
}

// ******** Main Loop *********
void loop()
{
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    uint8_t axis, i;
    int16_t error, errorAngle;
    int16_t delta, deltaSum;
    int16_t PTerm, ITerm, DTerm;
    static int16_t lastGyro[3] = { 0, 0, 0 };
    static int16_t delta1[3], delta2[3];
    static int16_t errorGyroI[3] = { 0, 0, 0 };
    static int16_t errorAngleI[2] = { 0, 0 };

    static uint32_t rcTime = 0;
    static int16_t initialThrottleHold;
    static int16_t errorAltitudeI = 0;
    int16_t AltPID = 0;
    static int16_t lastVelError = 0;
    static int32_t AltHold;
    
#ifdef SPEKTRUM
    if (rcFrameComplete)
        computeRC();
#endif

    if (currentTime > rcTime) { // 50Hz
        rcTime = currentTime + 20000;
#if !(defined(SPEKTRUM) || defined(BTSERIAL))
        computeRC();
#endif
        // Failsafe routine - added by MIS
#if defined(FAILSAFE)
        if (failsafeCnt > (5 * FAILSAVE_DELAY) && armed == 1) { // Stabilize, and set Throttle to specified level
            for (i = 0; i < 3; i++)
                rcData[i] = MIDRC;      // after specified guard time after RC signal is lost (in 0.1sec)
            rcData[THROTTLE] = FAILSAVE_THR0TTLE;
            if (failsafeCnt > 5 * (FAILSAVE_DELAY + FAILSAVE_OFF_DELAY)) {      // Turn OFF motors after specified Time (in 0.1sec)
                armed = 0;      //This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
                okToArm = 0;    //to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
            }
            failsafeEvents++;
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
                    servo[0] = 1500;    //we center the yaw gyro in conf mode
                    writeServos();
#ifdef LCD_CONF
                    configurationLoop();        //beginning LCD configuration
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
                    armed = 0;  // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
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
                accTrim[PITCH] += 2;
                writeParams();
            } else if (rcData[PITCH] < MINCHECK) {
                accTrim[PITCH]--;
                writeParams();
                accTrim[PITCH] -= 2;
                writeParams();
            } else if (rcData[ROLL] > MAXCHECK) {
                accTrim[ROLL]++;
                writeParams();
                accTrim[ROLL] += 2;
                writeParams();
            } else if (rcData[ROLL] < MINCHECK) {
                accTrim[ROLL]--;
                writeParams();
                accTrim[ROLL] -= 2;
                writeParams();
            } else {
                rcDelayCommand = 0;
            }
        }
#ifdef LOG_VALUES
        else if (armed) {       // update min and max values here, so do not get cycle time of the motor arming (which is way higher than normal)
            if (cycleTime > cycleTimeMax)
                cycleTimeMax = cycleTime;       // remember highscore
            if (cycleTime < cycleTimeMin)
                cycleTimeMin = cycleTime;       // remember lowscore
        }
#endif

        rcOptions = (rcData[AUX1] < 1300) + (1300 < rcData[AUX1] && rcData[AUX1] < 1700) * 2 + (rcData[AUX1] > 1700) * 4 + (rcData[AUX2] < 1300) * 8 + (1300 < rcData[AUX2] && rcData[AUX2] < 1700) * 16 + (rcData[AUX2] > 1700) * 32;

        //note: if FAILSAFE is disable, failsafeCnt > 5 * FAILSAVE_DELAY is always false
        if (((rcOptions & activate[BOXACC]) || (failsafeCnt > 5 * FAILSAVE_DELAY)) && (ACC || nunchuk)) {
            // bumpless transfer to Level mode
            if (!accMode) {
                errorAngleI[ROLL] = 0;
                errorAngleI[PITCH] = 0;
                accMode = 1;
            }
        } else
            accMode = 0;        // modified by MIS for failsave support

        if ((rcOptions & activate[BOXARM]) == 0)
            okToArm = 1;
        if (accMode == 1) {
            STABLEPIN_ON;
        } else {
            STABLEPIN_OFF;
        }

#if BARO
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
#endif
#if MAG
        if (rcOptions & activate[BOXMAG]) {
            if (magMode == 0) {
                magMode = 1;
                magHold = heading;
            }
        } else
            magMode = 0;
#endif
    }
#if GPS
    if (rcOptions & activate[BOXGPSHOME])
        GPSModeHome = 1;
    else
        GPSModeHome = 0;
    if (rcOptions & activate[BOXGPSHOLD])
        GPSModeHold = 1;
    else
        GPSModeHold = 0;
#endif

#if MAG
    Mag_getADC();
#endif
#if BARO
    Baro_update();
#endif

    computeIMU();
    // Measure loop rate just afer reading the sensors
    currentTime = micros();
    cycleTime = currentTime - previousTime;
    previousTime = currentTime;

#if MAG
    if (abs(rcCommand[YAW]) < 70 && magMode) {
        int16_t dif = heading - magHold;
        if (dif <= -180)
            dif += 360;
        if (dif >= +180)
            dif -= 360;
        if (smallAngle25)
            rcCommand[YAW] -= dif * P8[PIDMAG] / 30;    // 18 deg
    } else
        magHold = heading;
#endif

#if BARO
    if (baroMode) {
        if (abs(rcCommand[THROTTLE] - initialThrottleHold) > 20) {
            baroMode = 0;
            errorAltitudeI = 0;
        }
        //**** Alt. Set Point stabilization PID ****
        error = constrain(AltHold - EstAlt, -1000, 1000);       //  +/-10m,  1 decimeter accuracy
        errorAltitudeI += error;
        errorAltitudeI = constrain(errorAltitudeI, -30000, 30000);

        PTerm = P8[PIDALT] * error / 100;       // 16 bits is ok here
        ITerm = (int32_t) I8[PIDALT] * errorAltitudeI / 40000;

        AltPID = PTerm + ITerm;

        //**** Velocity stabilization PD ****        
        error = constrain(EstVelocity * 2, -30000, 30000);
        delta = error - lastVelError;
        lastVelError = error;

        PTerm = (int32_t) error *P8[PIDVEL] / 800;
        DTerm = (int32_t) delta *D8[PIDVEL] / 16;

        rcCommand[THROTTLE] = initialThrottleHold + constrain(AltPID - (PTerm - DTerm), -100, +100);
    }
#endif
    //**** PITCH & ROLL & YAW PID ****    
    for (axis = 0; axis < 3; axis++) {
        if (accMode == 1 && axis < 2) { //LEVEL MODE
            // 50 degrees max inclination
            errorAngle = constrain(2 * rcCommand[axis], -500, +500) - angle[axis] + accTrim[axis];      // 16 bits is ok here
#ifdef LEVEL_PDF
            PTerm = -(int32_t) angle[axis] * P8[PIDLEVEL] / 100;
#else
            PTerm = (int32_t) errorAngle *P8[PIDLEVEL] / 100;   //32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
#endif

            errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000);      //WindUp     //16 bits is ok here
            ITerm = ((int32_t) errorAngleI[axis] * I8[PIDLEVEL]) >> 12; //32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
        } else {                //ACRO MODE or YAW axis
            if (abs(rcCommand[axis]) < 350)
                error = rcCommand[axis] * 10 * 8 / P8[axis];    // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
            else
                error = (int32_t) rcCommand[axis] * 10 * 8 / P8[axis];  //32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
            error -= gyroData[axis];

            PTerm = rcCommand[axis];
            errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000);     //WindUp       //16 bits is ok here
            if (abs(gyroData[axis]) > 640)
                errorGyroI[axis] = 0;
            ITerm = (errorGyroI[axis] / 125 * I8[axis]) >> 6;   //16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
        }

        if (abs(gyroData[axis]) < 160)
            PTerm -= gyroData[axis] * dynP8[axis] / 10 / 8;     //16 bits is needed for calculation   160*200 = 32000         16 bits is ok for result
        else
            PTerm -= (int32_t) gyroData[axis] * dynP8[axis] / 10 / 8;   //32 bits is needed for calculation   

        delta = gyroData[axis] - lastGyro[axis];        //16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastGyro[axis] = gyroData[axis];
        deltaSum = delta1[axis] + delta2[axis] + delta;
        delta2[axis] = delta1[axis];
        delta1[axis] = delta;

        if (abs(deltaSum) < 640)
            DTerm = (deltaSum * dynD8[axis]) >> 5;      //16 bits is needed for calculation 640*50 = 32000           16 bits is ok for result 
        else
            DTerm = ((int32_t) deltaSum * dynD8[axis]) >> 5;    //32 bits is needed for calculation

        axisPID[axis] = PTerm + ITerm - DTerm;
    }

    mixTable();
    writeServos();
    writeMotors();

    //GPS
#if GPS
    while (GPS_SERIAL.available()) {
        if (GPS_newFrame(GPS_SERIAL.read())) {
            if (GPS_update == 1)
                GPS_update = 0;
            else
                GPS_update = 1;
            if (GPS_fix == 1) {
                if (GPS_fix_home == 0) {
                    GPS_fix_home = 1;
                    GPS_latitude_home = GPS_latitude;
                    GPS_longitude_home = GPS_longitude;
                }
                GPS_distance(GPS_latitude_home, GPS_longitude_home, GPS_latitude, GPS_longitude, &GPS_distanceToHome, &GPS_directionToHome);
            }
        }
    }
#endif
}

/* EEPROM --------------------------------------------------------------------- */
static uint8_t checkNewConf = 147;

typedef struct eep_entry_t {
    void *var;
    uint8_t size;
} eep_entry_t;

// ************************************************************************************************************
// EEPROM Layout definition
// ************************************************************************************************************
volatile eep_entry_t eep_entry[] = {
    &checkNewConf, sizeof(checkNewConf),
    &P8, sizeof(P8),
    &I8, sizeof(I8),
    &D8, sizeof(D8),
    &rcRate8, sizeof(rcRate8),
    &rcExpo8, sizeof(rcExpo8),
    &rollPitchRate, sizeof(rollPitchRate),
    &yawRate, sizeof(yawRate),
    &dynThrPID, sizeof(dynThrPID),
    &accZero, sizeof(accZero),
    &magZero, sizeof(magZero),
    &accTrim, sizeof(accTrim),
    &activate, sizeof(activate),
    &powerTrigger1, sizeof(powerTrigger1),
    &mixerConfiguration, sizeof(mixerConfiguration),
};
#define EEBLOCK_SIZE sizeof(eep_entry)/sizeof(eep_entry_t)
// ************************************************************************************************************

void readEEPROM(void)
{
    uint8_t i;
    uint8_t _address = eep_entry[0].size;

    eeprom_open();
    for (i = 1; i < EEBLOCK_SIZE; i++) {
        eeprom_read_block(eep_entry[i].var, (void *) (_address), eep_entry[i].size);
        _address += eep_entry[i].size;
    }
    eeprom_close();

#if defined(POWERMETER)
    pAlarm = (uint32_t) powerTrigger1 *(uint32_t) PLEVELSCALE *(uint32_t) PLEVELDIV;    // need to cast before multiplying
#endif
    for (i = 0; i < 7; i++)
        lookupRX[i] = (2500 + rcExpo8 * (i * i - 25)) * i * (int32_t) rcRate8 / 1250;
}

void writeParams(void)
{
    uint8_t i;
    uint8_t _address = 0;

    eeprom_open();
    for (i = 0; i < EEBLOCK_SIZE; i++) {
        eeprom_write_block(eep_entry[i].var, (void *)(_address), eep_entry[i].size); 
        _address += eep_entry[i].size;
    }
    eeprom_close();

    readEEPROM();
    blinkLED(15, 20, 1);
}

void checkFirstTime(void)
{
    uint8_t test_val, i;

    eeprom_open();
    eeprom_read_block(&test_val, (void *)0, 1);
    eeprom_close();

    if (test_val == checkNewConf)
        return;

    P8[ROLL] = 40;
    I8[ROLL] = 30;
    D8[ROLL] = 23;
    P8[PITCH] = 40;
    I8[PITCH] = 30;
    D8[PITCH] = 23;
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
    rcRate8 = 45;               // = 0.9 in GUI
    rcExpo8 = 65;
    rollPitchRate = 0;
    yawRate = 0;
    dynThrPID = 0;
    for (i = 0; i < 8; i++)
        activate[i] = 0;
    accTrim[0] = 0;
    accTrim[1] = 0;
    powerTrigger1 = 0;
    mixerConfiguration = MULTITYPE_QUADX;
    writeParams();
}

/* RX -------------------------------------------------------------------------------- */
static uint8_t pinRcChannel[8] = { ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN, AUX2PIN, CAM1PIN, CAM2PIN };
volatile uint16_t rcPinValue[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };   // interval [1000;2000]

// ***PPM SUM SIGNAL***
#ifdef SERIAL_SUM_PPM
static uint8_t rcChannel[8] = { SERIAL_SUM_PPM };
#endif
volatile uint16_t rcValue[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };      // interval [1000;2000]

// Configure receiver pins
void configureReceiver(void)
{
    uint8_t chan, a;
#if defined(STM8)
    // Configure GPIO pin for ppm input
    GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);

    TIM3_TimeBaseInit(TIM3_PRESCALER_8, 0xFFFF);
    TIM3_ICInit(TIM3_CHANNEL_1, TIM3_ICPOLARITY_RISING, TIM3_ICSELECTION_DIRECTTI, TIM3_ICPSC_DIV1, 0x0);
    TIM3_ITConfig(TIM3_IT_CC1, ENABLE);
    TIM3_Cmd(ENABLE);
#endif
}

#if defined(SERIAL_SUM_PPM)
__near __interrupt void TIM3_CAP_COM_IRQHandler(void)
{
    uint16_t diff;
    static uint16_t now;
    static uint16_t last = 0;
    static uint8_t chan = 0;

    if (TIM3_GetITStatus(TIM3_IT_CC1) == SET) {
        last = now;
        now = TIM3_GetCapture1();
    }

    TIM3_ClearITPendingBit(TIM3_IT_CC1);

    if (now > last) {
        diff = (now - last);
    } else {
        diff = ((0xFFFF - last) + now);
    }

    if (diff > 8000) {
        chan = 0;
    } else {
        if (diff > 1500 && diff < 4500 && chan < 8) {   // div2, 750 to 2250 ms Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
            uint16_t tmp = diff >> 1;
            rcValue[chan] = tmp;

#if defined(FAILSAFE)
            if (failsafeCnt > 20)
                failsafeCnt -= 20;
            else
                failsafeCnt = 0;        // clear FailSafe counter - added by MIS  //incompatible to quadroppm
#endif
        }
        chan++;
    }
}
#endif

uint16_t readRawRC(uint8_t chan)
{
    // We return the value correctly copied when the IRQ's where disabled
    return rcValue[rcChannel[chan]];                
}

void computeRC()
{
    static int16_t rcData4Values[8][4], rcDataMean[8];
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
void writeServos()
{
    if (!useServo)
        return;

    // STM8 PWM is actually 0.5us precision, so we double it
    if (mixerConfiguration == MULTITYPE_TRI || mixerConfiguration == MULTITYPE_BI) {
        /* One servo on Motor #4 */
        pwmWrite(4, servo[0]);
        if (mixerConfiguration == MULTITYPE_BI)
            pwmWrite(5, servo[1]);
    } else {
        /* Two servos for camstab or FLYING_WING */
        pwmWrite(4, servo[1]);
        pwmWrite(5, servo[2]);
    }
}

void writeMotors(void)
{
    uint8_t i;

    for (i = 0; i < numberMotor; i++)
        pwmWrite(i, motor[i]);
}

void writeAllMotors(int16_t mc)
{
    uint8_t i;
    // Sends commands to all motors
    for (i = 0; i < numberMotor; i++)
        motor[i] = mc;
    writeMotors();
}

#if defined(LOG_VALUES) || (POWERMETER == 1)
void logMotorsPower(void)
{
    uint32_t amp;
    uint8_t i;
    /* true cubic function; when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 1000 */
    const uint32_t amperes[64] = { 0, 4, 13, 31, 60, 104, 165, 246, 350, 481, 640, 831, 1056, 1319, 1622, 1969, 2361, 2803, 3297, 3845, 4451, 5118, 5848, 6645,
        7510, 8448, 9461, 10551, 11723, 12978, 14319, 15750, 17273, 18892, 20608, 22425, 24346, 26374, 28512, 30762, 33127, 35611,
        38215, 40944, 43799, 46785, 49903, 53156, 56548, 60081, 63759, 67583, 71558, 75685, 79968, 84410, 89013, 93781, 98716, 103821,
        109099, 114553, 120186, 126000
    };

    if (vbat) {                 // by all means - must avoid division by zero 
        for (i = 0; i < numberMotor; i++) {
            amp = amperes[(motor[i] - 1000) >> 4] / vbat;       // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp

#ifdef LOG_VALUES
            pMeter[i] += amp;   // sum up over time the mapped ESC input 
#endif
#if (POWERMETER == 1)
            pMeter[PMOTOR_SUM] += amp;  // total sum over all motors
#endif
        }
    }
}
#endif

void initOutput()
{
    if (mixerConfiguration == MULTITYPE_BI || mixerConfiguration == MULTITYPE_TRI || mixerConfiguration == MULTITYPE_GIMBAL || mixerConfiguration == MULTITYPE_FLYING_WING)
        useServo = 1;

#if defined(SERVO_TILT) || defined(CAMTRIG)
    useServo = 1;
#endif

    switch (mixerConfiguration) {
        case MULTITYPE_GIMBAL:
            numberMotor = 0;
            break;
        case MULTITYPE_FLYING_WING:
            numberMotor = 1;
            break;
        case MULTITYPE_BI:
            numberMotor = 2;
            break;
        case MULTITYPE_TRI:
            numberMotor = 3;
            break;

        case MULTITYPE_QUADP:
        case MULTITYPE_QUADX:
        case MULTITYPE_Y4:
            numberMotor = 4;
            break;

        case MULTITYPE_Y6:
        case MULTITYPE_HEX6:
        case MULTITYPE_HEX6X:
            numberMotor = 6;
            break;

        case MULTITYPE_OCTOX8:
        case MULTITYPE_OCTOFLATP:
        case MULTITYPE_OCTOFLATX:
            numberMotor = 8;
            break;
    }

    // This handles motor and servo initialization in one place
    pwmInit();
    writeAllMotors(1000);
    delay(300);
}

void mixTable()
{
    int16_t maxMotor;
    uint8_t i, axis;
    static uint8_t camCycle = 0;
    static uint8_t camState = 0;
    static uint32_t camTime = 0;

#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

    if (numberMotor > 3) {
        //prevent "yaw jump" during yaw correction
        axisPID[YAW] = constrain(axisPID[YAW], -100 - abs(rcCommand[YAW]), +100 + abs(rcCommand[YAW]));
    }

    switch (mixerConfiguration) {
        case MULTITYPE_BI:
            motor[0] = PIDMIX(+1, 0, 0);        //LEFT
            motor[1] = PIDMIX(-1, 0, 0);        //RIGHT        
            servo[0] = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] + axisPID[PITCH]), 1020, 2000);   //LEFT
            servo[1] = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] - axisPID[PITCH]), 1020, 2000);   //RIGHT
            break;

        case MULTITYPE_TRI:
            motor[0] = PIDMIX(0, +4 / 3, 0);    //REAR
            motor[1] = PIDMIX(-1, -2 / 3, 0);   //RIGHT
            motor[2] = PIDMIX(+1, -2 / 3, 0);   //LEFT
            servo[0] = constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX);        //REAR
            break;

        case MULTITYPE_QUADP:
            motor[0] = PIDMIX(0, +1, -1);       //REAR
            motor[1] = PIDMIX(-1, 0, +1);       //RIGHT
            motor[2] = PIDMIX(+1, 0, +1);       //LEFT
            motor[3] = PIDMIX(0, -1, -1);       //FRONT
            break;

        case MULTITYPE_QUADX:
            motor[0] = PIDMIX(-1, +1, -1);      //REAR_R
            motor[1] = PIDMIX(-1, -1, +1);      //FRONT_R
            motor[2] = PIDMIX(+1, +1, +1);      //REAR_L
            motor[3] = PIDMIX(+1, -1, -1);      //FRONT_L
            break;

        case MULTITYPE_Y4:
            motor[0] = PIDMIX(+0, +1, -1);      //REAR_1 CW
            motor[1] = PIDMIX(-1, -1, 0);       //FRONT_R CCW
            motor[2] = PIDMIX(+0, +1, +1);      //REAR_2 CCW
            motor[3] = PIDMIX(+1, -1, 0);       //FRONT_L CW
            break;

        case MULTITYPE_Y6:
            motor[0] = PIDMIX(+0, +4 / 3, +1);  //REAR
            motor[1] = PIDMIX(-1, -2 / 3, -1);  //RIGHT
            motor[2] = PIDMIX(+1, -2 / 3, -1);  //LEFT
            motor[3] = PIDMIX(+0, +4 / 3, -1);  //UNDER_REAR
            motor[4] = PIDMIX(-1, -2 / 3, +1);  //UNDER_RIGHT
            motor[5] = PIDMIX(+1, -2 / 3, +1);  //UNDER_LEFT    
            break;

        case MULTITYPE_HEX6:
            motor[0] = PIDMIX(-1 / 2, +1 / 2, +1);      //REAR_R
            motor[1] = PIDMIX(-1 / 2, -1 / 2, -1);      //FRONT_R
            motor[2] = PIDMIX(+1 / 2, +1 / 2, +1);      //REAR_L
            motor[3] = PIDMIX(+1 / 2, -1 / 2, -1);      //FRONT_L
            motor[4] = PIDMIX(+0, -1, +1);      //FRONT
            motor[5] = PIDMIX(+0, +1, -1);      //REAR
            break;

        case MULTITYPE_HEX6X:
            motor[0] = PIDMIX(-1 / 2, +1 / 2, +1);      //REAR_R
            motor[1] = PIDMIX(-1 / 2, -1 / 2, +1);      //FRONT_R
            motor[2] = PIDMIX(+1 / 2, +1 / 2, -1);      //REAR_L
            motor[3] = PIDMIX(+1 / 2, -1 / 2, -1);      //FRONT_L
            motor[4] = PIDMIX(-1, +0, -1);      //RIGHT
            motor[5] = PIDMIX(+1, +0, +1);      //LEFT
            break;

        case MULTITYPE_OCTOX8:
            motor[0] = PIDMIX(-1, +1, -1);      //REAR_R
            motor[1] = PIDMIX(-1, -1, +1);      //FRONT_R
            motor[2] = PIDMIX(+1, +1, +1);      //REAR_L
            motor[3] = PIDMIX(+1, -1, -1);      //FRONT_L
            motor[4] = PIDMIX(-1, +1, +1);      //UNDER_REAR_R
            motor[5] = PIDMIX(-1, -1, -1);      //UNDER_FRONT_R
            motor[6] = PIDMIX(+1, +1, -1);      //UNDER_REAR_L
            motor[7] = PIDMIX(+1, -1, +1);      //UNDER_FRONT_L
            break;

        case MULTITYPE_OCTOFLATP:
            motor[0] = PIDMIX(+7 / 10, -7 / 10, +1);    //FRONT_L
            motor[1] = PIDMIX(-7 / 10, -7 / 10, +1);    //FRONT_R
            motor[2] = PIDMIX(-7 / 10, +7 / 10, +1);    //REAR_R
            motor[3] = PIDMIX(+7 / 10, +7 / 10, +1);    //REAR_L
            motor[4] = PIDMIX(+0, -1, -1);      //FRONT
            motor[5] = PIDMIX(-1, +0, -1);      //RIGHT
            motor[6] = PIDMIX(+0, +1, -1);      //REAR
            motor[7] = PIDMIX(+1, +0, -1);      //LEFT 
            break;

        case MULTITYPE_OCTOFLATX:
            motor[0] = PIDMIX(+1, -1 / 2, +1);  //MIDFRONT_L
            motor[1] = PIDMIX(-1 / 2, -1, +1);  //FRONT_R
            motor[2] = PIDMIX(-1, +1 / 2, +1);  //MIDREAR_R
            motor[3] = PIDMIX(+1 / 2, +1, +1);  //REAR_L
            motor[4] = PIDMIX(+1 / 2, -1, -1);  //FRONT_L
            motor[5] = PIDMIX(-1, -1 / 2, -1);  //MIDFRONT_R
            motor[6] = PIDMIX(-1 / 2, +1, -1);  //REAR_R
            motor[7] = PIDMIX(+1, +1 / 2, -1);  //MIDREAR_L 
            break;

        case MULTITYPE_GIMBAL:
            servo[1] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] / 16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
            servo[2] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP * angle[ROLL] / 16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
            break;
            
        case MULTITYPE_FLYING_WING:
            motor[0] = rcCommand[THROTTLE];
            //if (passthroughMode) {// use raw stick values to drive output 
            // follow aux1 as being three way switch **NOTE: better to implement via check boxes in GUI 
            if (rcData[AUX1] < 1300) {
                // passthrough
                servo[1] = constrain(WING_LEFT_MID + PITCH_DIRECTION_L * (rcData[PITCH] - MIDRC) + ROLL_DIRECTION_L * (rcData[ROLL] - MIDRC), WING_LEFT_MIN, WING_LEFT_MAX);    //LEFT
                servo[2] = constrain(WING_RIGHT_MID + PITCH_DIRECTION_R * (rcData[PITCH] - MIDRC) + ROLL_DIRECTION_R * (rcData[ROLL] - MIDRC), WING_RIGHT_MIN, WING_RIGHT_MAX); //RIGHT
            } else {                    // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
                servo[1] = constrain(WING_LEFT_MID + PITCH_DIRECTION_L * axisPID[PITCH] + ROLL_DIRECTION_L * axisPID[ROLL], WING_LEFT_MIN, WING_LEFT_MAX);      //LEFT
                servo[2] = constrain(WING_RIGHT_MID + PITCH_DIRECTION_R * axisPID[PITCH] + ROLL_DIRECTION_R * axisPID[ROLL], WING_RIGHT_MIN, WING_RIGHT_MAX);   //RIGHT
            }
            break;

    }

#ifdef SERVO_TILT
    if (rcOptions & activate[BOXCAMSTAB]) {
        servo[1] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] / 16 + rcData[CAMPITCH] - 1500, TILT_PITCH_MIN, TILT_PITCH_MAX);
        servo[2] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP * angle[ROLL] / 16 + rcData[CAMROLL] - 1500, TILT_ROLL_MIN, TILT_ROLL_MAX);
    } else {
        servo[1] = constrain(TILT_PITCH_MIDDLE + rcData[CAMPITCH] - 1500, TILT_PITCH_MIN, TILT_PITCH_MAX);
        servo[2] = constrain(TILT_ROLL_MIDDLE + rcData[CAMROLL] - 1500, TILT_ROLL_MIN, TILT_ROLL_MAX);
    }
#endif

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
        } else {                //camState ==2
            if ((millis() - camTime) > CAM_TIME_LOW) {
                camState = 0;
                camCycle = 0;
            }
        }
    }
    if (rcOptions & activate[BOXCAMTRIG])
        camCycle = 1;
#endif

    maxMotor = motor[0];
    for (i = 1; i < numberMotor; i++)
        if (motor[i] > maxMotor)
            maxMotor = motor[i];
    for (i = 0; i < numberMotor; i++) {
        if (maxMotor > MAXTHROTTLE)     // this is a way to still have good gyro corrections if at least one motor reaches its max.
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

#if defined(LOG_VALUES) || (POWERMETER == 1)
    logMotorsPower();
#endif
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
#if !ACC
    if (nunchuk) {
        annexCode();
        while ((micros() - timeInterleave) < INTERLEAVING_DELAY);       //interleaving delay between 2 consecutive reads
        timeInterleave = micros();
        WMP_getRawADC();
        getEstimatedAttitude(); // computation time must last less than one interleaving delay
#if BARO
        getEstimatedAltitude();
#endif
        while ((micros() - timeInterleave) < INTERLEAVING_DELAY);       //interleaving delay between 2 consecutive reads
        timeInterleave = micros();
        while (WMP_getRawADC() != 1);   // For this interleaving reading, we must have a gyro update at this point (less delay)

        for (axis = 0; axis < 3; axis++) {
            // empirical, we take a weighted value of the current and the previous values
            // /4 is to average 4 values, note: overflow is not possible for WMP gyro here
            gyroData[axis] = (gyroADC[axis] * 3 + gyroADCprevious[axis] + 2) / 4;
            gyroADCprevious[axis] = gyroADC[axis];
        }
    }
#else                           /* !ACC */
    ACC_getADC();
    getEstimatedAttitude();
#if BARO
    getEstimatedAltitude();
#endif                          /* BARO */
#if GYRO
    Gyro_getADC();
#else
    WMP_getRawADC();
#endif                          /* GYRO */
    for (axis = 0; axis < 3; axis++)
        gyroADCp[axis] = gyroADC[axis];
    timeInterleave = micros();
    annexCode();
    while ((micros() - timeInterleave) < 650);  //empirical, interleaving delay between 2 consecutive reads
#if GYRO
    Gyro_getADC();
#else
    WMP_getRawADC();
#endif
    for (axis = 0; axis < 3; axis++) {
        gyroADCinter[axis] = gyroADC[axis] + gyroADCp[axis];
        // empirical, we take a weighted value of the current and the previous values
        gyroData[axis] = (gyroADCinter[axis] + gyroADCprevious[axis] + 1) / 3;
        gyroADCprevious[axis] = gyroADCinter[axis] / 2;
        if (!ACC)
            accADC[axis] = 0;
    }
#endif                          /* !ACC */
#if defined(TRI)
    gyroData[YAW] = (gyroYawSmooth * 2 + gyroData[YAW] + 1) / 3;
    gyroYawSmooth = gyroData[YAW];
#endif
}


#if defined(STAB_OLD_17)
/// OLD CODE from 1.7 ////
// ************************************
// simplified IMU based on Kalman Filter
// inspired from http://starlino.com/imu_guide.html
// and http://www.starlino.com/imu_kalman_arduino.html
// for angles under 25deg, we use an approximation to speed up the angle calculation
// magnetometer addition for small angles
// ************************************
void getEstimatedAttitude()
{
    uint8_t axis;
    float R, RGyro[3];          //R obtained from last estimated value and gyro movement;
    static float REst[3] = { 0, 0, 1 }; // init acc in stable mode
    static float A[2];          //angles between projection of R on XZ/YZ plane and Z axis (in Radian)
    float wGyro = 300;          // gyro weight/smooting factor
    float invW = 1.0 / (1 + 300);
    float gyroFactor;
    static uint8_t small_angle = 1;
    static uint16_t tPrevious;
    uint16_t tCurrent, deltaTime;
    float a[2], mag[2], cos_[2];

    tCurrent = micros();
    deltaTime = tCurrent - tPrevious;
    tPrevious = tCurrent;

#if GYRO
    gyroFactor = deltaTime / 300e6;     //empirical
#else
    gyroFactor = deltaTime / 200e6;     //empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
#endif

    for (axis = 0; axis < 2; axis++)
        a[axis] = gyroADC[axis] * gyroFactor;
    for (axis = 0; axis < 3; axis++)
        accSmooth[axis] = (accSmooth[axis] * 7 + accADC[axis] + 4) / 8;

    if (accSmooth[YAW] > 0) {   //we want to be sure we are not flying inverted  
        // a very nice trigonometric approximation: under 25deg, the error of this approximation is less than 1 deg:
        //   sin(x) =~= x =~= arcsin(x)
        //   angle_axis = arcsin(ACC_axis/ACC_1G) =~= ACC_axis/ACC_1G
        // the angle calculation is much more faster in this case
        if (accSmooth[ROLL] < acc_25deg && accSmooth[ROLL] > -acc_25deg && accSmooth[PITCH] < acc_25deg && accSmooth[PITCH] > -acc_25deg) {
            for (axis = 0; axis < 2; axis++) {
                A[axis] += a[axis];
                A[axis] = ((float) accSmooth[axis] / acc_1G + A[axis] * wGyro) * invW;  // =~= sin axis
#if MAG
                cos_[axis] = 1 - A[axis] * A[axis] / 2; // cos(x) =~= 1-x^2/2
#endif
            }
            small_angle = 1;
        } else {
            //magnitude vector size
            R = sqrt(square(accSmooth[ROLL]) + square(accSmooth[PITCH]) + square(accSmooth[YAW]));
            for (axis = 0; axis < 2; axis++) {
                if (acc_1G * 3 / 5 < R && R < acc_1G * 7 / 5 && small_angle == 0)       //if accel magnitude >1.4G or <0.6G => we neutralize the effect of accelerometers in the angle estimation
                    A[axis] = atan2(REst[axis], REst[YAW]);
                A[axis] += a[axis];
                cos_[axis] = cos(A[axis]);
                RGyro[axis] = sin(A[axis]) / sqrt(1.0 + square(cos_[axis]) * square(tan(A[1 - axis]))); //reverse calculation of RwGyro from Awz angles
            }
            RGyro[YAW] = sqrt(abs(1.0 - square(RGyro[ROLL]) - square(RGyro[PITCH])));
            for (axis = 0; axis < 3; axis++)
                REst[axis] = (accADC[axis] / R + wGyro * RGyro[axis]) * invW;   //combine Accelerometer and gyro readings
            small_angle = 0;
        }
#if defined(HMC5843) || defined(HMC5883)
        mag[PITCH] = -magADC[PITCH] * cos_[PITCH] + magADC[ROLL] * A[ROLL] * A[PITCH] + magADC[YAW] * cos_[ROLL] * A[PITCH];
        mag[ROLL] = magADC[ROLL] * cos_[ROLL] - magADC[YAW] * A[ROLL];
        heading = -degrees(atan2(mag[PITCH], mag[ROLL]));
#endif
    }
    for (axis = 0; axis < 2; axis++)
        angle[axis] = A[axis] * 572.9577951;    //angle in multiple of 0.1 degree
}

#else

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
//
// code size deduction and tmp vector intermediate step for vector rotation computation: October 2011 by Alex
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
#define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f))     //should be 2279.44 but 2380 gives better result

// +-2000/sec deg scale
// #define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)
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

typedef struct fp_vector {
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
    int16_t zi = abs(((int16_t) (z * 100)));
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

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float *delta)
{
    struct fp_vector v_tmp = *v;
    v->Z -= delta[ROLL] * v_tmp.X + delta[PITCH] * v_tmp.Y;
    v->X += delta[ROLL] * v_tmp.Z - delta[YAW] * v_tmp.Y;
    v->Y += delta[PITCH] * v_tmp.Z + delta[YAW] * v_tmp.X;
}

void getEstimatedAttitude()
{
    uint8_t axis;
    int16_t accMag = 0;
    static t_fp_vector EstG, EstM;
    static int16_t mgSmooth[3], accTemp[3];     // projection of smoothed and normalized magnetic vector on x/y/z axis, as measured by magnetometer
    static uint16_t previousT;
    float scale, deltaGyroAngle[3];
    uint16_t currentT = micros();

    scale = (currentT - previousT) * GYRO_SCALE;
    previousT = currentT;

    // Initialization
    for (axis = 0; axis < 3; axis++) {
        deltaGyroAngle[axis] = gyroADC[axis] * scale;
#if defined(ACC_LPF_FACTOR)
        accTemp[axis] = (accTemp[axis] - (accTemp[axis] >> 4)) + accADC[axis];
        accSmooth[axis] = accTemp[axis] >> 4;
#define ACC_VALUE accSmooth[axis]
#else
        accSmooth[axis] = accADC[axis];
#define ACC_VALUE accADC[axis]
#endif
        accMag += (ACC_VALUE * 10 / (int16_t) acc_1G) * (ACC_VALUE * 10 / (int16_t) acc_1G);

#if MAG
#if defined(MG_LPF_FACTOR)
        mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR; // LPF for Magnetometer values
#define MAG_VALUE mgSmooth[axis]
#else
#define MAG_VALUE magADC[axis]
#endif
#endif
    }

    rotateV(&EstG.V, deltaGyroAngle);
#if MAG
    rotateV(&EstM.V, deltaGyroAngle);
#endif
    if (abs(accSmooth[ROLL]) < acc_25deg && abs(accSmooth[PITCH]) < acc_25deg && accSmooth[YAW] > 0)
        smallAngle25 = 1;
    else
        smallAngle25 = 0;

    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if ((36 < accMag && accMag < 196) || smallAngle25)
        for (axis = 0; axis < 3; axis++) {
            int16_t acc = ACC_VALUE;
#ifndef TRUSTED_ACCZ
            if (smallAngle25 && axis == YAW)
                // We consider ACCZ = acc_1G when the acc on other axis is small.
                // It's a tweak to deal with some configs where ACC_Z tends to a value < acc_1G when high throttle is applied.
                // This tweak applies only when the multi is not in inverted position
                acc = acc_1G;
#endif                          /* !TRUSTED_ACCZ */
            EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + acc) * INV_GYR_CMPF_FACTOR;
        }
#if MAG
    for (axis = 0; axis < 3; axis++)
        EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
#endif
    // Attitude of the estimated vector
    angle[ROLL] = _atan2(EstG.V.X, EstG.V.Z);
    angle[PITCH] = _atan2(EstG.V.Y, EstG.V.Z);
#if MAG
    // Attitude of the cross product vector GxM
    heading = _atan2(EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X, EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z) / 10;
#endif
}

#endif                          /* OLD_1_7_STAB_CODE */

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

#define UPDATE_INTERVAL 25000   // 40hz update rate (20hz LPF on acc)
#define INIT_DELAY      4000000 // 4 sec initialization delay
#define Kp1 0.55f               // PI observer velocity gain
#define Kp2 1.0f                // PI observer position gain
#define Ki  0.001f              // PI observer integral gain (bias cancellation)
#define dt  (UPDATE_INTERVAL / 1000000.0f)

void getEstimatedAltitude()
{
    static uint8_t inited = 0;
    static int16_t AltErrorI = 0;
    static float AccScale = 0.0f;
    static uint32_t deadLine = INIT_DELAY;
    int16_t AltError;
    int16_t InstAcc;
    int16_t Delta;

    if (currentTime < deadLine)
        return;
    deadLine = currentTime + UPDATE_INTERVAL;
    // Soft start

    if (!inited) {
        inited = 1;
        EstAlt = BaroAlt;
        EstVelocity = 0;
        AltErrorI = 0;
        AccScale = 100 * 9.80665f / acc_1G;
    }
    // Estimation Error
    AltError = BaroAlt - EstAlt;
    AltErrorI += AltError;
    AltErrorI = constrain(AltErrorI, -25000, +25000);
    // Gravity vector correction and projection to the local Z
    //InstAcc = (accADC[YAW] * (1 - acc_1G * InvSqrt(isq(accADC[ROLL]) + isq(accADC[PITCH]) + isq(accADC[YAW])))) * AccScale + (Ki) * AltErrorI;
#if defined(TRUSTED_ACCZ)
    InstAcc = (accADC[YAW] * (1 - acc_1G * InvSqrt(isq(accADC[ROLL]) + isq(accADC[PITCH]) + isq(accADC[YAW])))) * AccScale + AltErrorI / 1000;
#else
    InstAcc = AltErrorI / 1000;
#endif
    // Integrators
    Delta = InstAcc * dt + (Kp1 * dt) * AltError;
    EstAlt += (EstVelocity / 5 + Delta) * (dt / 2) + (Kp2 * dt) * AltError;
    EstVelocity += Delta * 10;
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
#define MS561101BA_ADDRESS 0xEE //CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)
//#define MS561101BA_ADDRESS 0xEF //CBR=1 0xEF I2C address when pin CSB is connected to HIGH (VCC)
#endif

//ITG3200 and ITG3205 Gyro LPF setting
#if defined(ITG3200_LPF_256HZ) || defined(ITG3200_LPF_188HZ) || defined(ITG3200_LPF_98HZ) || defined(ITG3200_LPF_42HZ) || defined(ITG3200_LPF_20HZ) || defined(ITG3200_LPF_10HZ)
#if defined(ITG3200_LPF_256HZ)
#define ITG3200_SMPLRT_DIV 0    //8000Hz
#define ITG3200_DLPF_CFG   0
#endif
#if defined(ITG3200_LPF_188HZ)
#define ITG3200_SMPLRT_DIV 0    //1000Hz
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
#define ITG3200_SMPLRT_DIV 0    //8000Hz
#define ITG3200_DLPF_CFG   0
#endif

uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;

void i2c_getSixRawADC(uint8_t add, uint8_t reg)
{
    uint8_t rv = 0;
    rv = i2c_read(rawADC, 6, add, reg);
    if (rv != 0)
        i2cErrorCounter++;
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val)
{
    uint8_t buf[3];
    uint8_t rv = 0;
    buf[0] = add;
    buf[1] = reg;
    buf[2] = val;
    rv = i2c_write(buf, 3);
    if (rv != 0)
        i2cErrorCounter++;
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg)
{
    uint8_t data[1];
    uint8_t rv = 0;
    rv = i2c_read(data, 1, add, reg);
    if (rv != 0)
        i2cErrorCounter++;
    return data[0];
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
            accZero[YAW] = a[YAW] / 400 - acc_1G;       // for nunchuk 200=1G
            accTrim[ROLL] = 0;
            accTrim[PITCH] = 0;
            writeParams();      // write accZero in EEPROM
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
#define BMP085_ADDRESS  0xEE
#define BMP085_CTRL     0xF4
#define BMP085_ADC      0xF6
#define BMP085_TEMP     0x2E
#define BMP085_DATA     0xAA

static struct {
    // sensor registers from the BOSCH BMP085 datasheet
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
    uint16_t ut;                       //uncompensated T
    uint32_t up;                       //uncompensated P
    uint8_t state;
    uint32_t deadline;
} bmp085_ctx;
#define OSS 3

// read a 16 bit register
int16_t i2c_BMP085_readIntRegister(uint8_t r)
{
    uint8_t raw[2];

    i2c_read(raw, 2, BMP085_ADDRESS, r);
    return (int16_t)raw[0] << 8 | raw[1];
}

void i2c_BMP085_readCalibration(void)
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

// read uncompensated temperature value: send command first
void i2c_BMP085_UT_Start(void)
{
    i2c_writeReg(BMP085_ADDRESS, BMP085_CTRL, BMP085_TEMP);
}

// read uncompensated pressure value: send command first
void i2c_BMP085_UP_Start(void)
{
    i2c_writeReg(BMP085_ADDRESS, BMP085_CTRL, 0x34 + (OSS << 6));      // control register value for oversampling setting 3
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_BMP085_UP_Read(void)
{
    uint8_t raw[3];
    i2c_read(raw, 3, BMP085_ADDRESS, BMP085_ADC);
    bmp085_ctx.up = ((((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | ((uint32_t)raw[2])) >> (8 - OSS));
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_BMP085_UT_Read(void)
{
    uint8_t raw[2];

    i2c_read(raw, 2, BMP085_ADDRESS, BMP085_ADC);
    bmp085_ctx.ut = (uint16_t)raw[0] << 8 | raw[1];
}

void Baro_init(void)
{
    delay(10);
    i2c_BMP085_readCalibration();
    i2c_BMP085_UT_Start();
    delay(5);
    i2c_BMP085_UT_Read();
}

void i2c_BMP085_Calculate(void)
{
    int32_t x1, x2, x3, b3, b5, b6, p, tmp;
    uint32_t b4, b7;
    // Temperature calculations
    x1 = ((int32_t) bmp085_ctx.ut - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;
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
    // b7 = ((uint32_t) (bmp085_ctx.up >> (8 - OSS)) - b3) * (50000 >> OSS);
    b7 = ((uint32_t)bmp085_ctx.up - b3) * (50000 >> OSS);
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
        BaroAlt = (1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
        bmp085_ctx.state = 0;
        bmp085_ctx.deadline += 20000;
        break;
    }
}
#endif

// ************************************************************************************************************
// I2C Barometer MS561101BA
// ************************************************************************************************************
// first contribution from Fabio
// modification from Alex (September 2011)
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12
#if defined(MS561101BA)

// registers of the device
#define MS561101BA_PRESSURE 0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET 0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096

static struct {
    // sensor registers from the MS561101BA datasheet
    uint16_t c[7];
    union {
        uint32_t val;
        uint8_t raw[4];
    } ut;                       //uncompensated T
    union {
        uint32_t val;
        uint8_t raw[4];
    } up;                       //uncompensated P
    uint8_t state;
    uint32_t deadline;
} ms561101ba_ctx;

void i2c_MS561101BA_reset(void)
{
    i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);
}

void i2c_MS561101BA_readCalibration(void)
{
    union {
        uint16_t val;
        uint8_t raw[2];
    } data;
    uint8_t i;

    delay(10);
    for (i = 0; i < 6; i++) {
        i2c_rep_start(MS561101BA_ADDRESS + 0);
        i2c_write(0xA2 + 2 * i);
        i2c_rep_start(MS561101BA_ADDRESS + 1);  //I2C read direction => 1
        data.raw[1] = i2c_readAck();    // read a 16 bit register
        data.raw[0] = i2c_readNak();
        ms561101ba_ctx.c[i + 1] = data.val;
    }
}

void Baro_init(void)
{
    delay(10);
    i2c_MS561101BA_reset();
    delay(10);
    i2c_MS561101BA_readCalibration();
}

// read uncompensated temperature value: send command first
void i2c_MS561101BA_UT_Start(void)
{
    i2c_rep_start(MS561101BA_ADDRESS + 0);      // I2C write direction
    i2c_write(MS561101BA_TEMPERATURE + OSR);    // register selection
}

// read uncompensated pressure value: send command first
void i2c_MS561101BA_UP_Start(void)
{
    i2c_rep_start(MS561101BA_ADDRESS + 0);      // I2C write direction
    i2c_write(MS561101BA_PRESSURE + OSR);       // register selection
}

// read uncompensated pressure value: read result bytes
void i2c_MS561101BA_UP_Read(void)
{
    i2c_rep_start(MS561101BA_ADDRESS + 0);
    i2c_write(0);
    i2c_rep_start(MS561101BA_ADDRESS + 1);
    ms561101ba_ctx.up.raw[2] = i2c_readAck();
    ms561101ba_ctx.up.raw[1] = i2c_readAck();
    ms561101ba_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
void i2c_MS561101BA_UT_Read(void)
{
    i2c_rep_start(MS561101BA_ADDRESS + 0);
    i2c_write(0);
    i2c_rep_start(MS561101BA_ADDRESS + 1);
    ms561101ba_ctx.ut.raw[2] = i2c_readAck();
    ms561101ba_ctx.ut.raw[1] = i2c_readAck();
    ms561101ba_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_MS561101BA_Calculate(void)
{
    int64_t dT = ms561101ba_ctx.ut.val - ((uint32_t) ms561101ba_ctx.c[5] << 8); // int32_t according to the spec, but int64_t here to avoid cast after
    int64_t off = ((uint32_t) ms561101ba_ctx.c[2] << 16) + ((dT * ms561101ba_ctx.c[4]) >> 7);
    int64_t sens = ((uint32_t) ms561101ba_ctx.c[1] << 15) + ((dT * ms561101ba_ctx.c[3]) >> 8);
    pressure = (((ms561101ba_ctx.up.val * sens) >> 21) - off) >> 15;
}

void Baro_update(void)
{
    if (currentTime < ms561101ba_ctx.deadline)
        return;
    ms561101ba_ctx.deadline = currentTime;
    TWBR = ((16000000L / 400000L) - 16) / 2;    // change the I2C clock rate to 400kHz, MS5611 is ok with this speed
    switch (ms561101ba_ctx.state) {
    case 0:
        i2c_MS561101BA_UT_Start();
        ms561101ba_ctx.state++;
        ms561101ba_ctx.deadline += 15000;       // according to the specs, the pause should be at least 8.22ms
        break;
    case 1:
        i2c_MS561101BA_UT_Read();
        ms561101ba_ctx.state++;
        break;
    case 2:
        i2c_MS561101BA_UP_Start();
        ms561101ba_ctx.state++;
        ms561101ba_ctx.deadline += 15000;       // according to the specs, the pause should be at least 8.22ms
        break;
    case 3:
        i2c_MS561101BA_UP_Read();
        i2c_MS561101BA_Calculate();
        BaroAlt = (1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
        ms561101ba_ctx.state = 0;
        ms561101ba_ctx.deadline += 30000;
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
    i2c_writeReg(ADXL345_ADDRESS, 0x2D, 1 << 3);        //  register: Power CTRL  -- value: Set measure bit 3 on
    i2c_writeReg(ADXL345_ADDRESS, 0x31, 0x0B);  //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
    i2c_writeReg(ADXL345_ADDRESS, 0x2C, 8 + 2 + 1);     // register: BW_RATE     -- value: 200Hz sampling (see table 5 of the spec)
    acc_1G = 256;
}

void ACC_getADC()
{
#ifndef STM8
    TWBR = ((16000000L / 400000L) - 16) / 2;    // change the I2C clock rate to 400kHz, ADXL435 is ok with this speed
#endif
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

static void ADXL_Init(void)
{
    // setup ADXL345 rate/range/start measuring
    // Rate 3200Hz
    ADXL_ON;
    spi_writeByte(ADXL_RATE_ADDR);
    spi_writeByte(ADXL_RATE_800 & 0x0F);
    ADXL_OFF;

    // Range 8G
    ADXL_ON;
    spi_writeByte(ADXL_FORMAT_ADDR);
    spi_writeByte((ADXL_RANGE_8G & 0x03) | ADXL_FULL_RES | ADXL_4WIRE);
    ADXL_OFF;

    // Fifo depth = 16
    ADXL_ON;
    spi_writeByte(ADXL_FIFO_ADDR);
    spi_writeByte((16 & 0x1f) | ADXL_FIFO_STREAM);
    ADXL_OFF;

    ADXL_ON;
    spi_writeByte(ADXL_POWER_ADDR);
    spi_writeByte(ADXL_MEASURE);
    ADXL_OFF;
}

static uint8_t ADXL_GetAccelValues(void)
{
    volatile uint8_t i;

    ADXL_ON;
    spi_writeByte(ADXL_X0_ADDR | ADXL_MULTI_BIT | ADXL_READ_BIT);
    for (i = 0; i < 3; i++) {
        uint8_t i1, i2;
        i1 = spi_readByte();
        i2 = spi_readByte();

#ifdef LOWPASS_ACC
        // new result = 0.95 * previous_result + 0.05 * current_data
        sensorInputs[i + 4] = ((sensorInputs[i + 4] * 19) / 20) + (((i1 | i2 << 8) * 5) / 100);
#else
        sensorInputs[i + 4] += (i1 | i2 << 8);
#endif
    }

    // skip over this register
    spi_readByte();
    // FIFO_STATUS register (last few bits = fifo remaining)
    i = spi_readByte();
    ADXL_OFF;

    return i & 0x7F;            // return number of entires left in fifo
}

void ACC_init()
{
    // SPI ChipSelect for Accel
    GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);
    ADXL_OFF;

    delay(10);

    // Accel INT1 input tied to interrupt (TODO). Input-only for now.
    GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_IN_FL_NO_IT);

    // Initialize SPI Accelerometer
    ADXL_Init();
    acc_1G = 256;
}

void ACC_getADC()
{
    uint8_t count = 0;
    uint8_t remaining = 0;
    uint8_t i = 0;

    // Next up is accel fifo + avg
    do {
        count++;
        remaining = ADXL_GetAccelValues();
    } while ((count < 32) && (remaining > 0));

    count++;

#ifdef LOWPASS_ACC
    // commit current values to acc[]
#else
    // accel + average
    sensorInputs[4] = sensorInputs[4] / count;
    sensorInputs[5] = sensorInputs[5] / count;
    sensorInputs[6] = sensorInputs[6] / count;
#endif

    ACC_ORIENTATION(sensorInputs[4], sensorInputs[5], sensorInputs[6]);
    ACC_Common();
}
#endif

#if defined(MPU6000SPI)
static uint8_t mpuInitialized = 0;
#define MPU_OFF            GPIO_WriteHigh(GPIOB, GPIO_PIN_2);
#define MPU_ON             GPIO_WriteLow(GPIOB, GPIO_PIN_2);

#define MPUREG_WHOAMI               0x75
#define MPUREG_SMPLRT_DIV           0x19
#define MPUREG_CONFIG               0x1A
#define MPUREG_GYRO_CONFIG          0x1B
#define MPUREG_ACCEL_CONFIG         0x1C
#define MPUREG_I2C_MST_CTRL         0x24
#define MPUREG_I2C_SLV0_ADDR        0x25
#define MPUREG_I2C_SLV0_REG         0x26
#define MPUREG_I2C_SLV0_CTRL        0x27
#define MPUREG_I2C_SLV4_ADDR        0x31
#define MPUREG_I2C_SLV4_REG         0x32
#define MPUREG_I2C_SLV4_DO          0x33
#define MPUREG_I2C_SLV4_CTRL        0x34
#define MPUREG_I2C_SLV4_DI          0x35
#define MPUREG_I2C_MST_STATUS       0x36
#define MPUREG_INT_PIN_CFG          0x37
#define MPUREG_INT_ENABLE           0x38 
#define MPUREG_ACCEL_XOUT_H         0x3B
#define MPUREG_ACCEL_XOUT_L         0x3C
#define MPUREG_ACCEL_YOUT_H         0x3D
#define MPUREG_ACCEL_YOUT_L         0x3E
#define MPUREG_ACCEL_ZOUT_H         0x3F
#define MPUREG_ACCEL_ZOUT_L         0x40
#define MPUREG_TEMP_OUT_H           0x41
#define MPUREG_TEMP_OUT_L           0x42
#define MPUREG_GYRO_XOUT_H          0x43
#define MPUREG_GYRO_XOUT_L          0x44
#define MPUREG_GYRO_YOUT_H          0x45
#define MPUREG_GYRO_YOUT_L          0x46
#define MPUREG_GYRO_ZOUT_H          0x47
#define MPUREG_GYRO_ZOUT_L          0x48
#define MPUREG_EXT_SENS_DATA_00     0x49 // Registers 0x49 to 0x60 - External Sensor Data
#define MPUREG_I2C_SLV0_DO          0x63 // This register holds the output data written into Slave 0 when Slave 0 is set to write mode.
#define MPUREG_I2C_MST_DELAY_CTRL   0x67 // I2C Master Delay Control
#define MPUREG_USER_CTRL            0x6A
#define MPUREG_PWR_MGMT_1           0x6B
#define MPUREG_PWR_MGMT_2           0x6C

// Configuration bits MPU 6000
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_AFS_2G                 0x00
#define BITS_AFS_4G                 0x08
#define BITS_AFS_8G                 0x10
#define BITS_AFS_16G                0x18
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_I2C_SLV0_EN             0x80

static uint8_t MPU6000_Buffer[14];   // Sensor data ACCXYZ|TEMP|GYROXYZ

static uint8_t MPU6000_ReadReg(uint8_t Address)
{
    uint8_t rv;
    MPU_ON;
    spi_writeByte(Address | 0x80); // Address with high bit set = Read operation
    rv = spi_readByte();
    MPU_OFF;
    return rv;
}

static void MPU6000_getSixRawADC(void)
{
    uint8_t i;
    MPU_ON;
    spi_writeByte(MPUREG_ACCEL_XOUT_H | 0x80); // Address with high bit set = Read operation
    // ACC X, Y, Z, TEMP, GYRO X, Y, Z
    for (i = 0; i < 14; i++)
        MPU6000_Buffer[i] = spi_readByte();
    MPU_OFF;
}

static void MPU6000_WriteReg(uint8_t Address, uint8_t Data)
{ 
    MPU_ON;
    spi_writeByte(Address); 
    spi_writeByte(Data);
    MPU_OFF;
    delay(1);
}

void MPU6000_init(void)
{
    // SPI ChipSelect for MPU-6000
    GPIO_Init(GPIOB, GPIO_PIN_2, GPIO_MODE_OUT_PP_HIGH_FAST);
    MPU_OFF;

    // MPU-6000 input tied to interrupt (TODO). Input-only for now.
    GPIO_Init(GPIOB, GPIO_PIN_3, GPIO_MODE_IN_FL_NO_IT);

    MPU6000_WriteReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
    delay(100);
    MPU6000_WriteReg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);      // Set PLL source to gyro output
    MPU6000_WriteReg(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);             // Disable I2C bus
    MPU6000_WriteReg(MPUREG_SMPLRT_DIV, 0x04);                      // Sample rate = 200Hz    Fsample = 1Khz / (4 + 1) = 200Hz   
    MPU6000_WriteReg(MPUREG_CONFIG, 0); // BITS_DLPF_CFG_42HZ);            // Fs & DLPF Fs = 1kHz, DLPF = 42Hz (low pass filter)
    MPU6000_WriteReg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);          // Gyro scale 2000º/s
    MPU6000_WriteReg(MPUREG_ACCEL_CONFIG, BITS_AFS_4G);             // Accel scale 4G
    MPU6000_WriteReg(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);            // INT: Raw data ready
    MPU6000_WriteReg(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR);     // INT: Clear on any read

    mpuInitialized = 1;
}

void ACC_init()
{
    if (!mpuInitialized)
        MPU6000_init();
        
    acc_1G = 256;
}

void ACC_getADC()
{
    MPU6000_getSixRawADC();
    ACC_ORIENTATION(-(MPU6000_Buffer[0] << 8 | MPU6000_Buffer[1]) / 16, -(MPU6000_Buffer[2] << 8 | MPU6000_Buffer[3]) / 16, (MPU6000_Buffer[4] << 8 | MPU6000_Buffer[5]) / 16);
    ACC_Common();
}

void Gyro_init(void)
{
    if (!mpuInitialized)
        MPU6000_init();
}

void Gyro_getADC(void)
{
    // range: +/- 8192; +/- 2000 deg/sec
    MPU6000_getSixRawADC();
    GYRO_ORIENTATION((((MPU6000_Buffer[10] << 8) | MPU6000_Buffer[11]) / 4), -(((MPU6000_Buffer[8] << 8) | MPU6000_Buffer[9]) / 4), -(((MPU6000_Buffer[12] << 8) | MPU6000_Buffer[13]) / 4));
    GYRO_Common();
}

#define HMC5883L_I2C_ADDRESS        0x1e
#define HMC5883L_ID_REG_A           0x0a
#define HMC5883L_ID_REG_B           0x0b
#define HMC5883L_ID_REG_C           0x0c

void Mag_init(void)
{
    volatile uint8_t i, temp;
    
    MPU6000_WriteReg(MPUREG_I2C_MST_CTRL, 0b01000000 | 13); // WAIT_FOR_ES=1, I2C Master Clock Speed 400kHz
    MPU6000_WriteReg(MPUREG_I2C_SLV4_ADDR, 0x80 | HMC5883L_I2C_ADDRESS);
    MPU6000_WriteReg(MPUREG_I2C_SLV4_REG, HMC5883L_ID_REG_A);
    MPU6000_WriteReg(MPUREG_I2C_SLV4_CTRL, 0b11000000); // I2C_SLV4_EN | I2C_SLV4_INT_EN
    delay(1);
    
    MPU6000_getSixRawADC();
    
    temp = MPU6000_ReadReg(MPUREG_I2C_MST_STATUS);
    temp = MPU6000_ReadReg(MPUREG_I2C_SLV4_DI);
    delay(1);
}

void Device_Mag_getADC(void)
{
    // i2c_getSixRawADC(0X3C, 0X03);
    // MAG_ORIENTATION(((rawADC[0] << 8) | rawADC[1]), ((rawADC[2] << 8) | rawADC[3]), -((rawADC[4] << 8) | rawADC[5]));
}
#endif /* MPU6000SPI */

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
// ************************************************************************************************************
#if defined(BMA180)
void ACC_init()
{
    uint8_t control;
    
    delay(10);
    //default range 2G: 1G = 4096 unit.
    i2c_writeReg(BMA180_ADDRESS, 0x0D, 1 << 4); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
    delay(5);
    control = i2c_readReg(BMA180_ADDRESS, 0x20);
    control = control & 0x0F;   // register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 10Hz (bits value = 0000xxxx)
    control = control | 0x00;
    i2c_writeReg(BMA180_ADDRESS, 0x20, control);
    delay(5);
    control = i2c_readReg(BMA180_ADDRESS, 0x30);
    control = control & 0xFC;
    control = control | 0x02;
    i2c_writeReg(BMA180_ADDRESS, 0x30, control);
    delay(5);
    acc_1G = 512;
}

void ACC_getADC()
{
#ifndef STM8
    TWBR = ((16000000L / 400000L) - 16) / 2;    // Optional line.  Sensor is good for it in the spec.
#endif
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
    uint8_t control;

    i2c_writeReg(0x70, 0x15, 0x80);
    control = i2c_readReg(0x70, 0x14);
    control = control & 0xE0;
    control = control | (0x00 << 3);    //Range 2G 00
    control = control | 0x00;   //Bandwidth 25 Hz 000
    i2c_writeReg(0x70, 0x14, control);
    acc_1G = 255;
}

void ACC_getADC()
{
#if !defined(STM8)
    TWBR = ((16000000L / 400000L) - 16) / 2;
#endif

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
    TWBR = ((16000000L / I2C_SPEED) - 16) / 2;  // change the I2C clock rate. !! you must check if the nunchuk is ok with this freq
    i2c_getSixRawADC(0xA4, 0x00);

    ACC_ORIENTATION(((rawADC[3] << 2) + ((rawADC[5] >> 4) & 0x2)), -((rawADC[2] << 2) + ((rawADC[5] >> 3) & 0x2)), (((rawADC[4] & 0xFE) << 2) + ((rawADC[5] >> 5) & 0x6)));
    ACC_Common();
}
#endif

// ************************************************************************
// LIS3LV02 I2C Accelerometer
//contribution from adver (http://multiwii.com/forum/viewtopic.php?f=8&t=451)
// ************************************************************************
#if defined(LIS3LV02)
#define LIS3A  0x3A             // I2C adress: 0x3A (8bit)

void i2c_ACC_init()
{
    i2c_writeReg(LIS3A, 0x20, 0xD7);    // CTRL_REG1   1101 0111 Pwr on, 160Hz 
    i2c_writeReg(LIS3A, 0x21, 0x50);    // CTRL_REG2   0100 0000 Littl endian, 12 Bit, Boot
    acc_1G = 256;
}

void i2c_ACC_getADC()
{
    TWBR = ((16000000L / 400000L) - 16) / 2;    // change the I2C clock rate to 400kHz
    i2c_getSixRawADC(LIS3A, 0x28 + 0x80);
    ACC_ORIENTATION((rawADC[3] << 8 | rawADC[2]) / 4, -(rawADC[1] << 8 | rawADC[0]) / 4, -(rawADC[5] << 8 | rawADC[4]) / 4);
    ACC_Common();
}
#endif

// ************************************************************************************************************
// I2C Accelerometer LSM303DLx
// contribution from wektorx (http://www.multiwii.com/forum/viewtopic.php?f=8&t=863)
// ************************************************************************************************************
#if defined(LSM303DLx_ACC)
void ACC_init()
{
    delay(10);
    i2c_writeReg(0x30, 0x20, 0x27);
    i2c_writeReg(0x30, 0x23, 0x30);
    i2c_writeReg(0x30, 0x21, 0x00);
    acc_1G = 256;
}

void ACC_getADC()
{
    i2c_getSixRawADC(0x30, 0xA8);

    ACC_ORIENTATION(-((rawADC[3] << 8) | rawADC[2]) / 16, ((rawADC[1] << 8) | rawADC[0]) / 16, ((rawADC[5] << 8) | rawADC[4]) / 16);
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
static volatile uint8_t adcInProgress = 0;

__near __interrupt void ADC1_IRQHandler(void)
{
    uint8_t i = 0;

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
void Gyro_init(void)
{
    // ADC1
    ADC1_DeInit();
    GPIO_Init(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_MODE_IN_FL_NO_IT);
    ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_3, ADC1_PRESSEL_FCPU_D2, ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_ALL, DISABLE);
    ADC1_DataBufferCmd(ENABLE);
    ADC1_ScanModeCmd(ENABLE);
    ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);
}

void Gyro_getADC(void)
{
    // read out
    adcInProgress = 1;
    ADC1_StartConversion();
    while (adcInProgress);      // wait for conversion

    GYRO_ORIENTATION(-(sensorInputs[0]) * 5, (sensorInputs[1]) * 5, -(sensorInputs[2]) * 5);
    GYRO_Common();
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
    i2c_writeReg(0XD2 + 0, 0x20, 0x8F); // CTRL_REG1   400Hz ODR, 20hz filter, run!
    delay(5);
    i2c_writeReg(0XD2 + 0, 0x24, 0x02); // CTRL_REG5   low pass filter enable
}

void Gyro_getADC()
{
    TWBR = ((16000000L / 400000L) - 16) / 2;    // change the I2C clock rate to 400kHz
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
    i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x80);  //register: Power Management  --  value: reset device
    //  delay(5);
    //  i2c_writeReg(ITG3200_ADDRESS, 0x15, ITG3200_SMPLRT_DIV); //register: Sample Rate Divider  -- default value = 0: OK
    delay(5);
    i2c_writeReg(ITG3200_ADDRESS, 0x16, 0x18 + ITG3200_DLPF_CFG);       //register: DLPF_CFG - low pass filter configuration
    delay(5);
    i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x03);  //register: Power Management  --  value: PLL with Z Gyro reference
    delay(100);
}

void Gyro_getADC()
{
#if !defined(STM8)
    TWBR = ((16000000L / 400000L) - 16) / 2;    // change the I2C clock rate to 400kHz
#endif
    i2c_getSixRawADC(ITG3200_ADDRESS, 0X1D);
    GYRO_ORIENTATION(+(((rawADC[2] << 8) | rawADC[3]) / 4),     // range: +/- 8192; +/- 2000 deg/sec
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

    if (currentTime < t)
        return;                 //each read is spaced by 100ms
    t = currentTime + 100000;

#if !defined(STM8)
    TWBR = ((16000000L / 400000L) - 16) / 2;    // change the I2C clock rate to 400kHz
#endif
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
        if ((t - tCal) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
            LEDPIN_TOGGLE;
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
    i2c_writeReg(0X3C, 0x02, 0x00);     //register: Mode register  --  value: Continuous-Conversion Mode
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
    i2c_writeReg(0x18, 0x0a, 0x01);     //Start the first conversion
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
    i2c_writeReg(0xA6, 0xF0, 0x55);     // Initialize Extension
    delay(d);
    i2c_writeReg(0xA6, 0xFE, 0x05);     // Activate Nunchuck pass-through mode
    delay(d);
    if (d > 0) {
        // We need to set acc_1G for the Nunchuk beforehand; It's used in WMP_getRawADC() and ACC_Common()
        // If a different accelerometer is used, it will be overwritten by its ACC_init() later.
        uint8_t i;
        uint8_t numberAccRead = 0;
        acc_1G = 200;
        acc_25deg = acc_1G * 0.423;
        // Read from WMP 100 times, this should return alternating WMP and Nunchuk data
        for (i = 0; i < 100; i++) {
            delay(4);
            if (WMP_getRawADC() == 0)
                numberAccRead++;        // Count number of times we read from the Nunchuk extension
        }
        // If we got at least 25 Nunchuck reads, we assume the Nunchuk is present
        if (numberAccRead > 25)
            nunchuk = 1;
        delay(10);
    }
}

uint8_t WMP_getRawADC()
{
    uint8_t axis;
    TWBR = ((16000000L / I2C_SPEED) - 16) / 2;  // change the I2C clock rate
    i2c_getSixRawADC(0xA4, 0x00);

    if (micros() < (neutralizeTime + NEUTRALIZE_DELAY)) {       //we neutralize data in case of blocking+hard reset state
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
        gyroADC[ROLL] = -(((rawADC[5] >> 2) << 8) | rawADC[2]); //range: +/- 8192
        gyroADC[PITCH] = -(((rawADC[4] >> 2) << 8) | rawADC[1]);
        gyroADC[YAW] = -(((rawADC[3] >> 2) << 8) | rawADC[0]);
        GYRO_Common();
        // Check if slow bit is set and normalize to fast mode range
        gyroADC[ROLL] = (rawADC[3] & 0x01) ? gyroADC[ROLL] / 5 : gyroADC[ROLL]; //the ratio 1/5 is not exactly the IDG600 or ISZ650 specification 
        gyroADC[PITCH] = (rawADC[4] & 0x02) >> 1 ? gyroADC[PITCH] / 5 : gyroADC[PITCH]; //we detect here the slow of fast mode WMP gyros values (see wiibrew for more details)
        gyroADC[YAW] = (rawADC[3] & 0x02) >> 1 ? gyroADC[YAW] / 5 : gyroADC[YAW];       // this step must be done after zero compensation    
        return 1;
    } else if ((rawADC[5] & 0x03) == 0x00) {    // Nunchuk Data
        ACC_ORIENTATION(((rawADC[3] << 2) | ((rawADC[5] >> 4) & 0x02)), -((rawADC[2] << 2) | ((rawADC[5] >> 3) & 0x02)), (((rawADC[4] >> 1) << 3) | ((rawADC[5] >> 5) & 0x06)));
        ACC_Common();
        return 0;
    } else
        return 2;
}
#endif                          /* !GYRO */

void initSensors()
{
    i2c_init();
    spi_init();
    delay(100);
#if GYRO
    Gyro_init();
#else
    WMP_init(250);
#endif
#if BARO
    Baro_init();
#endif
#if ACC
    ACC_init();
    acc_25deg = acc_1G * 0.423;
#endif
#if MAG
    Mag_init();
#endif
}

/* SERIAL ---------------------------------------------------------------- */
void serialCom()
{
    int16_t a;
    uint8_t i;

    uint16_t intPowerMeterSum, intPowerTrigger1;

    if ((!Serial_isTxBusy()) && Serial_available()) {
        switch (Serial_read()) {
#ifdef BTSERIAL
        case 'K':              //receive RC data from Bluetooth Serial adapter as a remote
            rcData[THROTTLE] = (Serial.read() * 4) + 1000;
            rcData[ROLL] = (Serial.read() * 4) + 1000;
            rcData[PITCH] = (Serial.read() * 4) + 1000;
            rcData[YAW] = (Serial.read() * 4) + 1000;
            rcData[AUX1] = (Serial.read() * 4) + 1000;
            break;
#endif
#ifdef LCD_TELEMETRY
        case 'A':              // button A press
            if (telemetry == 'A')
                telemetry = 0;
            else {
                telemetry = 'A';
                LCDprint(12);   /* clear screen */
            }
            break;
        case 'B':              // button B press
            if (telemetry == 'B')
                telemetry = 0;
            else {
                telemetry = 'B';
                LCDprint(12);   /* clear screen */
            }
            break;
        case 'C':              // button C press
            if (telemetry == 'C')
                telemetry = 0;
            else {
                telemetry = 'C';
                LCDprint(12);   /* clear screen */
            }
            break;
        case 'D':              // button D press
            if (telemetry == 'D')
                telemetry = 0;
            else {
                telemetry = 'D';
                LCDprint(12);   /* clear screen */
            }
            break;
        case 'a':              // button A release
        case 'b':              // button B release
        case 'c':              // button C release
        case 'd':              // button D release
            break;
#endif
        case 'M':              // Multiwii @ arduino to GUI all data
            Serial_reset();
            serialize8('M');
            serialize8(VERSION);        // MultiWii Firmware version
            for (i = 0; i < 3; i++)
                serialize16(accSmooth[i]);
            for (i = 0; i < 3; i++)
                serialize16(gyroData[i] / 8);
            for (i = 0; i < 3; i++)
                serialize16(magADC[i] / 3);
            serialize16(EstAlt / 10);
            serialize16(heading);       // compass
            for (i = 0; i < 4; i++)
                serialize16(servo[i]);
            for (i = 0; i < 8; i++)
                serialize16(motor[i]);
            for (i = 0; i < 8; i++)
                serialize16(rcData[i]);
            serialize8(nunchuk | ACC << 1 | BARO << 2 | MAG << 3 | GPSPRESENT << 4);
            serialize8(accMode | baroMode << 1 | magMode << 2 | (GPSModeHome | GPSModeHold) << 3);
            serialize16(cycleTime);
            for (i = 0; i < 2; i++)
                serialize16(angle[i] / 10);
            serialize8(mixerConfiguration > 10 ? 11 : mixerConfiguration); // hack for multiwiiConf GUI
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
            for (i = 0; i < 8; i++)
                serialize8(activate[i]);
            serialize16(GPS_distanceToHome);
            serialize16(GPS_directionToHome);
            serialize8(GPS_numSat);
            serialize8(GPS_fix);
            serialize8(GPS_update);
#if defined(POWERMETER)
            intPowerMeterSum = (pMeter[PMOTOR_SUM] / PLEVELDIV);
            intPowerTrigger1 = powerTrigger1 * PLEVELSCALE;
            serialize16(intPowerMeterSum);
            serialize16(intPowerTrigger1);
#else
            serialize16(0);
            serialize16(0);
#endif
            serialize8(vbat);
            serialize16(BaroAlt / 10);  // 4 variables are here for general monitoring purpose
            serialize16(i2cErrorCounter);     // debug2
            serialize16(0);     // debug3
            serialize16(0);     // debug4
            serialize8('M');
            Serial_commitBuffer();     // Serial.write(s,point);
            break;
        case 'O':              // arduino to OSD data - contribution from MIS
            Serial_reset();
            serialize8('O');
            for (i = 0; i < 3; i++)
                serialize16(accSmooth[i]);
            for (i = 0; i < 3; i++)
                serialize16(gyroData[i]);
            serialize16(EstAlt * 10.0f);
            serialize16(heading);       // compass - 16 bytes
            for (i = 0; i < 2; i++)
                serialize16(angle[i]);  //20
            for (i = 0; i < 6; i++)
                serialize16(motor[i]);  //32
            for (i = 0; i < 6; i++) {
                serialize16(rcData[i]);
            }                   //44
            serialize8(nunchuk | ACC << 1 | BARO << 2 | MAG << 3);
            serialize8(accMode | baroMode << 1 | magMode << 2);
            serialize8(vbat);   // Vbatt 47
            serialize8(VERSION);        // MultiWii Firmware version
            serialize8('O');    //49
            Serial_commitBuffer();
            break;
        case 'R':
            systemReboot();
            break;
        case 'W':              //GUI write params to eeprom @ arduino
            while (Serial_available() < 33) { }
            for (i = 0; i < 5; i++) {
                P8[i] = Serial_read();
                I8[i] = Serial_read();
                D8[i] = Serial_read();
            }                   // 15
            P8[PIDLEVEL] = Serial_read();
            I8[PIDLEVEL] = Serial_read();       // 17
            P8[PIDMAG] = Serial_read(); // 18
            rcRate8 = Serial_read();
            rcExpo8 = Serial_read();    // 20
            rollPitchRate = Serial_read();
            yawRate = Serial_read();    // 22
            dynThrPID = Serial_read();  // 23
            for (i = 0; i < 8; i++)
                activate[i] = Serial_read();    // 31
#if defined(POWERMETER)
            powerTrigger1 = (Serial_read() + 256 * Serial_read()) / PLEVELSCALE;        // we rely on writeParams() to compute corresponding pAlarm value
#else
            Serial_read();
            Serial_read();      // so we unload the two bytes
#endif
            writeParams();
            break;
        case 'S':              //GUI to arduino ACC calibration request
            calibratingA = 400;
            break;
        case 'E':              //GUI to arduino MAG calibration request
            calibratingM = 1;
            break;
        case 'X':              // GUI to change mixer type. command is X+ascii A + MULTITYPE_XXXX index. i.e. XA for tri, XB for Quad+, XC for QuadX, etc.
            while (Serial_available() < 1) { }
            i = Serial_read();
            Serial_reset();
            if (i > 64 && i < 64 + MULTITYPE_LAST) {
                serialize8('O');
                serialize8('K');
                Serial_commitBuffer();
                mixerConfiguration = i - '@'; // A..B..C.. index
                writeParams();
                systemReboot();
            }
            serialize8('N');
            serialize8('G');
            Serial_commitBuffer();
            break;
        }
    }
}
