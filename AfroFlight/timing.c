#include "main.h"

void delay_ms(u16 ms)
{
    while (ms--) {
        delay_us(1000);
    }
}
