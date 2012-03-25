#include "board.h"
#include "def.h"
#include "config.h"
#include "sysdep.h"
#include "mw.h"

extern int16_t axisPID[3];
extern int16_t motor[8];
extern int16_t servo[4];
extern uint8_t mixerConfiguration;
extern uint8_t useServo;
extern uint8_t numberMotor;
extern int16_t rcCommand[4];
extern int16_t angle[2];
extern uint8_t armed;

/* OUTPUT ------------------------------------------------------------------------------------- */
void writeServos(void)
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
    pwmInit(useServo);
    writeAllMotors(MINCOMMAND);

    delay(300);
}

#define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL] * X + axisPID[PITCH] * Y + YAW_DIRECTION * axisPID[YAW] * Z

void mixTable()
{
    int16_t maxMotor;
    uint8_t i, axis;
    static uint8_t camCycle = 0;
    static uint8_t camState = 0;
    static uint32_t camTime = 0;

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
            servo[1] = constrain(TILT_PITCH_MIDDLE + gimbalGainPitch * angle[PITCH] / 16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
            servo[2] = constrain(TILT_ROLL_MIDDLE + gimbalGainRoll * angle[ROLL] / 16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
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
        if (gimbalFlags & GIMBAL_LEVEL_STAB) {
            servo[1] = constrain(TILT_PITCH_MIDDLE + gimbalGainPitch * angle[PITCH] / 16, TILT_PITCH_MIN, TILT_PITCH_MAX);
            servo[2] = constrain(TILT_ROLL_MIDDLE + gimbalGainRoll * angle[ROLL] / 16, TILT_ROLL_MIN, TILT_ROLL_MAX);
        } else {
            servo[1] = constrain(TILT_PITCH_MIDDLE + gimbalGainPitch * angle[PITCH] / 16 + rcData[CAMPITCH] - 1500, TILT_PITCH_MIN, TILT_PITCH_MAX);
            servo[2] = constrain(TILT_ROLL_MIDDLE + gimbalGainRoll * angle[ROLL] / 16 + rcData[CAMROLL] - 1500, TILT_ROLL_MIN, TILT_ROLL_MAX);
        }
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

