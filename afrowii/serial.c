#include "board.h"
#include "def.h"
#include "config.h"
#include "sysdep.h"
#include "mw.h"

extern int16_t debug1, debug2, debug3, debug4;
extern uint8_t armed;
extern int16_t motor[8];
extern int16_t servo[8];
extern uint8_t rcOptions[CHECKBOXITEMS];
extern uint8_t vbat;
extern uint8_t accMode, magMode, baroMode, headFreeMode, passThruMode;
extern int16_t heading;
extern int32_t EstAlt;
extern uint16_t calibratingA;
extern uint8_t calibratingM;
extern int32_t BaroAlt;
extern int16_t i2c_errors_count;
extern uint16_t cycleTime;

/* SERIAL ---------------------------------------------------------------- */
void serialCom(void)
{
    int16_t a;
    uint8_t i;

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
        case 'M':              // Multiwii @ arduino to GUI all data
            Serial_reset();
            serialize8('M');
            serialize8(VERSION);        // MultiWii Firmware version
            for (i = 0; i < 3; i++)
                serialize16(accSmooth[i]);
            for (i = 0; i < 3; i++)
                serialize16(gyroData[i]);
            for (i = 0; i < 3; i++)
                serialize16(magADC[i]);
            serialize16(EstAlt / 10);
            serialize16(heading);       // compass
            for (i = 0; i < 8; i++)
                serialize16(servo[i]);
            for (i = 0; i < 8; i++)
                serialize16(motor[i]);
            for (i = 0; i < 8; i++)
                serialize16(rcData[i]);
            serialize8(ACC << 1 | BARO << 2 | MAG << 3);
            serialize8(accMode | baroMode << 1 | magMode << 2 | armed << 5);
#if defined(LOG_VALUES)
            serialize16(cycleTimeMax);
            cycleTimeMax = 0;
#else
            serialize16(cycleTime);
#endif
            serialize16(i2c_errors_count);
            for (i = 0; i < 2; i++)
                serialize16(angle[i]);
            serialize8(cfg.mixerConfiguration);
            for (i = 0; i < PIDITEMS; i++) {
                serialize8(cfg.P8[i]);
                serialize8(cfg.I8[i]);
                serialize8(cfg.D8[i]);
            }
            serialize8(cfg.rcRate8);
            serialize8(cfg.rcExpo8);
            serialize8(cfg.rollPitchRate);
            serialize8(cfg.yawRate);
            serialize8(cfg.dynThrPID);
            for (i = 0; i < CHECKBOXITEMS; i++) {
                serialize8(cfg.activate1[i]);
                serialize8(cfg.activate2[i] | (rcOptions[i] << 7)); // use highest bit to transport state in mwc
            }
            serialize16(0);
            serialize16(0);
            serialize8(0);
            serialize8(0);
            serialize8(0);
            serialize16(0); // POWERMETERSUM
            serialize16(0); // POWERTRIGGER
            serialize8(vbat);
            serialize16(BaroAlt / 10);      // 4 variables are here for general monitoring purpose
            serialize16(debug2);            // debug2
            serialize16(debug3);            // debug3
            serialize16(debug4);            // debug4            
            serialize8('M');
            Serial_commitBuffer();
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
            serialize8(ACC << 1 | BARO << 2 | MAG << 3);
            serialize8(accMode | baroMode << 1 | magMode << 2 | armed << 5);
            serialize8(vbat);   // Vbatt 47
            serialize8(VERSION);        // MultiWii Firmware version
            serialize8(0);
            serialize8(0);
            serialize16(0);
            serialize16(0);
            serialize16(0);
            serialize16(0);
            serialize16(0);
            serialize16(0);     // Speed for OSD
            serialize8('O');    //49
            Serial_commitBuffer();
            break;
        case 'R':
            systemReboot();
            break;
        case 'r':
            // back to default settings
            break;
        case 'W':              // GUI write params to eeprom @ arduino
            // while (Serial_available() < (7 + 3 * PIDITEMS + 2 * CHECKBOXITEMS));
            // while (Serial_available() < 33) { }
            for (i = 0; i < PIDITEMS; i++) {
                cfg.P8[i] = Serial_read();
                cfg.I8[i] = Serial_read();
                cfg.D8[i] = Serial_read();
            }                   // 15
            cfg.rcRate8 = Serial_read();
            cfg.rcExpo8 = Serial_read();    // 20
            cfg.rollPitchRate = Serial_read();
            cfg.yawRate = Serial_read();    // 22
            cfg.dynThrPID = Serial_read();  // 23
            for (i = 0; i < CHECKBOXITEMS; i++) {
                cfg.activate1[i] = Serial_read();
                cfg.activate2[i] = Serial_read();
            }
            Serial_read();      // power meter stuff, null
            Serial_read();      // power meter stuff, null
            writeParams();
            break;
        case 'S':              //GUI to arduino ACC calibration request
            calibratingA = 400;
            break;
        case 'E':              //GUI to arduino MAG calibration request
            calibratingM = 1;
            break;

        case 'G':               // GUI to multiwii - gimbal tuning parameters
            while (Serial_available() < 3) { }
            cfg.gimbal_flags = Serial_read();
            cfg.gimbal_gain_pitch = Serial_read();
            cfg.gimbal_gain_roll = Serial_read();
            writeParams();
            break;

        case 'X':              // GUI to change mixer type. command is X+ascii A + MULTITYPE_XXXX index. i.e. XA for tri, XB for Quad+, XC for QuadX, etc.
            while (Serial_available() < 1) { }
            i = Serial_read();
            Serial_reset();
            if (i > 64 && i < 64 + MULTITYPE_LAST) {
                serialize8('O');
                serialize8('K');
                Serial_commitBuffer();
                cfg.mixerConfiguration = i - '@'; // A..B..C.. index
                writeParams();
                systemReboot();
                break;
            }
            serialize8('N');
            serialize8('G');
            Serial_commitBuffer();
            break;
        }
    }
}
