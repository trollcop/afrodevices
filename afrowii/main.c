#include "config.h"
#include "def.h"
#include "sysdep.h"

void setup(void);
void loop(void);

#ifdef STM32TEST
void loop2(void)
{
    static uint8_t usbConnected = 0;
    static uint8_t usbConfigured = 0;

    usbConnected = usbIsConnected();
    usbConfigured = usbIsConfigured();
    
    if (usbConfigured) {
        uint16_t avail = Serial_available();
        uint8_t buf[64];
        uint8_t ptr = 0;
        if (avail > 1) {
            memset(buf, 0, sizeof(buf));
            do {
                buf[ptr++] = Serial_read();
            } while (Serial_available() > 0);
            debug_printf("serial: %s\n", buf);
        }
    } 
}
    
void loop3(void)
{
    uint32_t oldms = 0;

    // test timing w/scope
    while (1) {
        uint32_t ms = micros();
        if (ms != oldms && ms % 10 == 0) {
            LEDPIN_ON;
            oldms = ms;
            LEDPIN_OFF;
        }
    }
}
#endif

int main(void)
{
    // system dependent hardware init
    hw_init();

    setup();

    for (;;)
        loop();
}
