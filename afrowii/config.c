#include "board.h"
#include "def.h"
#include "config.h"
#include "sysdep.h"
#include "mw.h"

/* EEPROM --------------------------------------------------------------------- */
static uint8_t checkNewConf = 151;
config_t cfg;
const char rcChannelLetters[] = "AERT1234";

void parseRcChannels(const char *input)
{
    const char *c, *s;

    for (c = input; *c; c++) {
        s = strchr(rcChannelLetters, *c);
        if (s)                          
            cfg.rcmap[s - rcChannelLetters] = c - input;
    }
}

void readEEPROM(void)
{
    uint8_t i;

    eeprom_open();
    eeprom_read_block(&cfg, (void *)NULL, sizeof(cfg));
    eeprom_close();

    for (i = 0; i < 7; i++)
        lookupRX[i] = (2500 + cfg.rcExpo8 * (i * i - 25)) * i * (int32_t) cfg.rcRate8 / 1250;
}

void writeParams(void)
{
    eeprom_open();
    eeprom_write_block(&cfg, (void *)NULL, sizeof(cfg));
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

    cfg.version = checkNewConf;
    cfg.P8[ROLL] = 40;
    cfg.I8[ROLL] = 30;
    cfg.D8[ROLL] = 23;
    cfg.P8[PITCH] = 40;
    cfg.I8[PITCH] = 30;
    cfg.D8[PITCH] = 23;
    cfg.P8[YAW] = 85;
    cfg.I8[YAW] = 0;
    cfg.D8[YAW] = 0;
    cfg.P8[PIDALT] = 16;
    cfg.I8[PIDALT] = 15;
    cfg.D8[PIDALT] = 7;
    cfg.P8[PIDGPS] = 50;
    cfg.I8[PIDGPS] = 0;
    cfg.D8[PIDGPS] = 15;
    cfg.P8[PIDVEL] = 0;
    cfg.I8[PIDVEL] = 0;
    cfg.P8[PIDLEVEL] = 90;
    cfg.I8[PIDLEVEL] = 45;
    cfg.D8[PIDLEVEL] = 100;
    cfg.P8[PIDMAG] = 40;
    cfg.rcRate8 = 45;               // = 0.9 in GUI
    cfg.rcExpo8 = 65;
    cfg.rollPitchRate = 0;
    cfg.yawRate = 0;
    cfg.dynThrPID = 0;
    for (i = 0; i < CHECKBOXITEMS; i++) {
        cfg.activate1[i] = 0;
        cfg.activate2[i] = 0;
    }
    cfg.accTrim[0] = 0;
    cfg.accTrim[1] = 0;
    cfg.mixerConfiguration = MULTITYPE_QUADX;
    cfg.gimbal_flags = 0;
    cfg.gimbal_gain_pitch = 10;
    cfg.gimbal_gain_roll = 10;
    cfg.vbatscale = 110;
    cfg.vbatmaxcellvoltage = 43;
    cfg.vbatmincellvoltage = 33;
    parseRcChannels("AETR1234");
    writeParams();
}
