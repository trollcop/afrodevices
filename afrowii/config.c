#include "board.h"
#include "def.h"
#include "config.h"
#include "sysdep.h"
#include "mw.h"

/* EEPROM --------------------------------------------------------------------- */
static uint8_t checkNewConf = 149;

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
    &gimbalFlags, sizeof(gimbalFlags),
    &gimbalGainPitch, sizeof(gimbalGainPitch),
    &gimbalGainRoll, sizeof(gimbalGainRoll)
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
    gimbalFlags = 0;
    gimbalGainPitch = 10;
    gimbalGainRoll = 10;
    writeParams();
}
