#pragma once

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

enum {
    GIMBAL_TILTONLY = 1,                        // In standalone gimbal mode, this switches PPM port into a single-channel PWM input
    GIMBAL_LEVEL_STAB = 2                       // In quadx/quadp with camstab enabled, ignore CAM1/CAM2 inputs and just stabilize level.
};

extern uint8_t P8[7], I8[7], D8[7];
extern int16_t accTrim[2];
extern int16_t lookupRX[7];
extern uint8_t mixerConfiguration;
extern uint8_t rollPitchRate;
extern uint8_t yawRate;
extern uint8_t dynThrPID;
extern uint8_t activate[8];
extern uint8_t rcRate8;
extern uint8_t rcExpo8;
extern uint8_t powerTrigger1;
extern int16_t rcData[8];

extern uint8_t gimbalFlags;
extern int8_t gimbalGainPitch;
extern int8_t gimbalGainRoll;
extern uint8_t useSpektrum;
extern volatile uint8_t rcFrameComplete;

extern int16_t accZero[3];
extern int16_t magZero[3];

extern volatile int16_t failsafeCnt;

// config.c
void readEEPROM(void);
void checkFirstTime(void);

// mw.c
void blinkLED(uint8_t num, uint8_t wait, uint8_t repeat);