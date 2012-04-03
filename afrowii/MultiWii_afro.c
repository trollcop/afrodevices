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

#include "board.h"
#include "def.h"
#include "config.h"
#include "sysdep.h"
#include "mw.h"

// #define LOWPASS_ACC

int16_t debug1, debug2, debug3, debug4;
uint32_t currentTime = 0;
static uint16_t previousTime = 0;
uint16_t cycleTime = 0;  // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
uint16_t calibratingA = 0;       // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint8_t calibratingM = 0;
uint16_t calibratingG = 0;
uint8_t armed = 0;
uint8_t accMode = 0;     // if level mode is a activated
uint8_t magMode = 0;     // if compass heading hold is a activated
uint8_t baroMode = 0;    // if altitude hold is activated
uint8_t headFreeMode = 0;        // if head free mode is a activated
uint8_t passThruMode = 0;        // if passthrough mode is activated
static int16_t headFreeModeHold;
int16_t accTrim[2] = { 0, 0 };
int16_t heading, magHold;
static uint8_t calibratedACC = 0;
uint8_t vbat;            // battery voltage in 0.1V steps
static uint16_t batteryWarningVoltage = 99; // annoying buzzer after this one, battery ready to be dead
static uint8_t okToArm = 0;
uint8_t rcOptions[CHECKBOXITEMS];
static int32_t pressure;
static uint8_t buzzerState = 0;
int16_t i2c_errors_count = 0;
int16_t annex650_overrun_count = 0;

/* 0:Pitch 1:Roll 2:Yaw 3:Battery Voltage 4:AX 5:AY 6:AZ */
volatile int16_t sensorInputs[7] = { 0, };

//for log
static uint16_t cycleTimeMax = 0;       // highest ever cycle time
static uint16_t cycleTimeMin = 65535;   // lowest ever cycle time

volatile int16_t failsafeCnt = 0;

static int16_t failsafeEvents = 0;
int16_t rcData[8];       // interval [1000;2000]
int16_t rcCommand[4];    // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW 
uint8_t rcRate8;
uint8_t rcExpo8;
int16_t lookupRX[7];     //  lookup table for expo & RC rate
volatile uint8_t rcFrameComplete;       // for serial rc receiver Spektrum
uint8_t useSpektrum = FALSE;     // using spektrum sat

// **************
// gyro+acc IMU
// **************
int16_t accZero[3] = { 0, 0, 0 };
int16_t magZero[3] = { 0, 0, 0 };

// *************************
// motor and servo functions
// *************************
int16_t axisPID[3];
uint8_t mixerConfiguration = MULTITYPE_QUADX;
extern int16_t servo[8];
extern int16_t motor[8];

// **********************
// EEPROM functions
// **********************
// uint8_t P8[7], I8[7], D8[7];     //8 bits is much faster and the code is much shorter
uint8_t dynP8[3], dynI8[3], dynD8[3];
uint8_t rollPitchRate;
uint8_t yawRate;
uint8_t dynThrPID;
uint8_t activate1[CHECKBOXITEMS];
uint8_t activate2[CHECKBOXITEMS];

/* prototypes */
void initOutput(void);
void initSensors(void);
void configureReceiver(void);
void initializeServo(void);
void writeMotors(void);
void computeRC(void);
void writeServos(void);
void Mag_getADC(void);
void Baro_update(void);
void computeIMU(void);
void mixTable(void);
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

uint16_t batteryAdcToVoltage(uint16_t src)
{
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 1023 = 12bit adc, 110 = 11:1 voltage divider (10k:1k) * 10 for 0.1V
    return (((src) * 3.3f) / 1023) * cfg.vbatscale;
}

void annexCode(void)
{
    // this code is executed at each loop and won't interfere with control loop if it lasts less than 650 microseconds
    static uint32_t buzzerTime, calibratedAccTime;
    static uint8_t vbatTimer = 0;
    static uint8_t buzzerFreq;  //delay between buzzer ring
    uint8_t axis, prop1, prop2;
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
            prop1 = 100 - (uint16_t) rollPitchRate * tmp / 500;
            prop1 = (uint16_t) prop1 * prop2 / 100;
        } else {                // YAW
            rcCommand[axis] = tmp;
            prop1 = 100 - (uint16_t) yawRate * tmp / 500;
        }
        dynP8[axis] = (uint16_t) cfg.P8[axis] * prop1 / 100;
        dynD8[axis] = (uint16_t) cfg.D8[axis] * prop1 / 100;
        if (rcData[axis] < MIDRC)
            rcCommand[axis] = -rcCommand[axis];
    }

    rcCommand[THROTTLE] = MINTHROTTLE + (int32_t) (MAXTHROTTLE - MINTHROTTLE) * (rcData[THROTTLE] - MINCHECK) / (2000 - MINCHECK);

    if (headFreeMode) {
        float radDiff = (heading - headFreeModeHold) * PI / 180.0f;
        float cosDiff = cos(radDiff);
        float sinDiff = sin(radDiff);
        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
        rcCommand[PITCH] = rcCommand_PITCH;
    }

#if defined(VBAT)
    if (!(++vbatTimer % VBATFREQ)) {
        vbatRawArray[(ind++) % 8] = sensorInputs[3];        // VBAT_ADC
        for (i = 0; i < 8; i++)
            vbatRaw += vbatRawArray[i];
        vbat = batteryAdcToVoltage(vbatRaw / 8);
    }

    if (rcOptions[BOXBEEPERON]) {   // unconditional beeper on via AUXn switch 
        buzzerFreq = 7;
    } else if (((vbat > batteryWarningVoltage)) || (vbat < cfg.vbatmincellvoltage)) {      // VBAT ok buzzer off
        buzzerFreq = 0;
        buzzerState = 0;
        BUZZERPIN_OFF;
    } else
        buzzerFreq = 4;     // low battery
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

    if ((calibratingA > 0 && (ACC)) || (calibratingG > 0)) { // Calibration phasis
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

    serialCom();
}

void setup(void)
{
    LEDPIN_PINMODE;
    BUZZERPIN_PINMODE;
    Serial_begin(SERIAL_COM_SPEED);
    readEEPROM();
    initOutput();
    checkFirstTime();
    configureReceiver();
    initSensors();
    previousTime = micros();
#if defined(GIMBAL)
    calibratingA = 400;
#endif
    calibratingG = 400;
}

// ******** Main Loop *********
void loop(void)
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

    if (useSpektrum) {
        if (rcFrameComplete)
            computeRC();
    }

    if (currentTime > rcTime) { // 50Hz
        rcTime = currentTime + 20000;
        if (!useSpektrum)
            computeRC();

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
            } else if ((activate1[BOXARM] > 0) || (activate2[BOXARM] > 0)) {
                if (rcOptions[BOXARM] && okToArm) {
                    armed = 1;
                    headFreeModeHold = heading;
                } else if (armed)
                    armed = 0;
                rcDelayCommand = 0;
            } else if ((rcData[YAW] < MINCHECK || rcData[ROLL] < MINCHECK) && armed == 1) {
                if (rcDelayCommand == 20)
                    armed = 0;  // rcDelayCommand = 20 => 20x20ms = 0.4s = time to wait for a specific RC command to be acknowledged
            } else if ((rcData[YAW] > MAXCHECK || rcData[ROLL] > MAXCHECK) && rcData[PITCH] < MAXCHECK && armed == 0 && calibratingG == 0 && calibratedACC == 1) {
                if (rcDelayCommand == 20) {
                    armed = 1;
                    headFreeModeHold = heading;
                }
            } else
                rcDelayCommand = 0;
        } else if (rcData[THROTTLE] > MAXCHECK && armed == 0) {
            if (rcData[YAW] < MINCHECK && rcData[PITCH] < MINCHECK) {   // throttle=max, yaw=left, pitch=min
                if (rcDelayCommand == 20)
                    calibratingA = 400;
                rcDelayCommand++;
            } else if (rcData[YAW] > MAXCHECK && rcData[PITCH] < MINCHECK) {    // throttle=max, yaw=right, pitch=min  
                if (rcDelayCommand == 20)
                    calibratingM = 1;   // MAG calibration request
                rcDelayCommand++;
            } else if (rcData[PITCH] > MAXCHECK) {
                accTrim[PITCH] += 2;
                writeParams();
            } else if (rcData[PITCH] < MINCHECK) {
                accTrim[PITCH] -= 2;
                writeParams();
            } else if (rcData[ROLL] > MAXCHECK) {
                accTrim[ROLL] += 2;
                writeParams();
            } else if (rcData[ROLL] < MINCHECK) {
                accTrim[ROLL] -= 2;
                writeParams();
            } else {
                rcDelayCommand = 0;
            }
        }

        for (i = 0; i < CHECKBOXITEMS; i++) {
            rcOptions[i] = (((rcData[AUX1] < 1300) | (1300 < rcData[AUX1] && rcData[AUX1] < 1700) << 1 | (rcData[AUX1] > 1700) << 2 | (rcData[AUX2] < 1300) << 3 | (1300 < rcData[AUX2] && rcData[AUX2] < 1700) << 4 | (rcData[AUX2] > 1700) << 5) & activate1[i]
                ) || (((rcData[AUX3] < 1300) | (1300 < rcData[AUX3] && rcData[AUX3] < 1700) << 1 | (rcData[AUX3] > 1700) << 2 | (rcData[AUX4] < 1300) << 3 | (1300 < rcData[AUX4] && rcData[AUX4] < 1700) << 4 | (rcData[AUX4] > 1700) << 5) & activate2[i]);
        }

        // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAVE_DELAY is always false
        if ((rcOptions[BOXACC] || (failsafeCnt > 5 * FAILSAVE_DELAY)) && (ACC)) {
            // bumpless transfer to Level mode
            if (!accMode) {
                errorAngleI[ROLL] = 0;
                errorAngleI[PITCH] = 0;
                accMode = 1;
            }
        } else
            accMode = 0;        // modified by MIS for failsave support

        if (rcOptions[BOXARM] == 0)
            okToArm = 1;

#if BARO
        if (rcOptions[BOXBARO]) {
            if (baroMode == 0) {
                baroMode = 1;
                AltHold = EstAlt;
                initialThrottleHold = rcCommand[THROTTLE];
                errorAltitudeI = 0;
                BaroPID = 0;
            }
        } else
            baroMode = 0;
#endif
#if MAG
        if (rcOptions[BOXMAG]) {
            if (magMode == 0) {
                magMode = 1;
                magHold = heading;
            }
        } else
            magMode = 0;
        if (rcOptions[BOXHEADFREE]) {
            if (headFreeMode == 0) {
                headFreeMode = 1;
            }
        } else
            headFreeMode = 0;
#endif
        if (rcOptions[BOXPASSTHRU]) {
            passThruMode = 1;
        } else
            passThruMode = 0;
    } else {
        
        static int8_t taskOrder = 0;    // never call all functions in the same loop, to avoid high delay spikes
        switch (taskOrder) {
        case 0:
            taskOrder++;
#if MAG
            Mag_getADC();
            break;
#endif
        case 1:
            taskOrder++;
#if BARO
            Baro_update();
            break;
#endif
        case 2:
            taskOrder++;
#if BARO
            getEstimatedAltitude();
            break;
#endif
        default:
            taskOrder = 0;
            break;
        }
    }

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
            baroMode = 0;       // so that a new althold reference is defined
        }
        rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
    }
#endif
    //**** PITCH & ROLL & YAW PID ****    
    for (axis = 0; axis < 3; axis++) {
        if (accMode == 1 && axis < 2) { //LEVEL MODE
            // 50 degrees max inclination
            errorAngle = constrain(2 * rcCommand[axis], -500, +500) - angle[axis] + accTrim[axis];      // 16 bits is ok here
#ifdef LEVEL_PDF
            PTerm = -(int32_t) angle[axis] * cfg.P8[PIDLEVEL] / 100;
#else
            PTerm = (int32_t) errorAngle * cfg.P8[PIDLEVEL] / 100;   //32 bits is needed for calculation: errorAngle*P8[PIDLEVEL] could exceed 32768   16 bits is ok for result
#endif
            PTerm = constrain(PTerm, -cfg.D8[PIDLEVEL] * 5, +cfg.D8[PIDLEVEL] * 5);

            errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000);      //WindUp     //16 bits is ok here
            ITerm = ((int32_t) errorAngleI[axis] * cfg.I8[PIDLEVEL]) >> 12; //32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
        } else {                //ACRO MODE or YAW axis
            if (abs(rcCommand[axis]) < 350)
                error = rcCommand[axis] * 10 * 8 / cfg.P8[axis];    // 16 bits is needed for calculation: 350*10*8 = 28000      16 bits is ok for result if P8>2 (P>0.2)
            else
                error = (int32_t) rcCommand[axis] * 10 * 8 / cfg.P8[axis];  //32 bits is needed for calculation: 500*5*10*8 = 200000   16 bits is ok for result if P8>2 (P>0.2)
            error -= gyroData[axis];

            PTerm = rcCommand[axis];
            errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000);     //WindUp       //16 bits is ok here
            if (abs(gyroData[axis]) > 640)
                errorGyroI[axis] = 0;
            ITerm = (errorGyroI[axis] / 125 * cfg.I8[axis]) >> 6;   //16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
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
}
