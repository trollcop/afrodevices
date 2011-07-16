#pragma once

#define US(us) ( 16000000 / 3000000.0 * us )
#define MS(ms) US(ms * 1000) // maximum 10ms

#define _delay( loops ) \
	_asm("$N: \n decw X \n jrne $L \n nop", (u16)loops);

#define _delay_us(us) _delay(US(us))
#define delay_us(us) _delay(US(us))
#define _delay_ms(ms) delay_ms(ms)
void delay_ms(u16 ms);
