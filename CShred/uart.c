#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include "uart.h"

/*
 * Initialize the UART to 9600 Bd, tx/rx, 8N1.
 */
void uart_init(void)
{
    unsigned int ubrr = (unsigned int) ((unsigned long) F_CPU / (8 * UART_BAUD) - 1);
    // set clock divider
    UBRR0H = (uint8_t) (ubrr >> 8);
    UBRR0L = (uint8_t) ubrr;
    // enable double speed
    UCSR0A |= (1 << U2X0);

    UCSR0B = _BV(TXEN0) | _BV(RXEN0);	/* tx/rx enable */
    //asynchronous 8N1

    // UCSR0C = (1 << URSEL) | (3 << UCSZ00);
    DDRD |= (1 << DDD1);	// set TXD pin as output
    PORTD &= ~(1 << PORTD1);	// disable pullup on TXD pin
}

/*
 * Send character c down the UART Tx, wait until tx holding register
 * is empty.
 */
int uart_putchar(char c, FILE * stream)
{
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;

    return 0;
}

/*
 * Receive a character from the UART Rx.
 *
 * This features a simple line-editor that allows to delete and
 * re-edit the characters entered, until either CR or NL is entered.
 * Printable characters entered will be echoed using uart_putchar().
 *
 * Editing characters:
 *
 * . \b (BS) or \177 (DEL) delete the previous character
 * . ^u kills the entire input buffer
 * . ^w deletes the previous word
 * . ^r sends a CR, and then reprints the buffer
 * . \t will be replaced by a single space
 *
 * All other control characters will be ignored.
 *
 * The internal line buffer is RX_BUFSIZE (80) characters long, which
 * includes the terminating \n (but no terminating \0).  If the buffer
 * is full (i. e., at RX_BUFSIZE-1 characters in order to keep space for
 * the trailing \n), any further input attempts will send a \a to
 * uart_putchar() (BEL character), although line editing is still
 * allowed.
 *
 * Input errors while talking to the UART will cause an immediate
 * return of -1 (error indication).  Notably, this will be caused by a
 * framing error (e. g. serial line "break" condition), by an input
 * overrun, and by a parity error (if parity was enabled and automatic
 * parity recognition is supported by hardware).
 *
 * Successive calls to uart_getchar() will be satisfied from the
 * internal buffer until that buffer is emptied again.
 */
int uart_getchar(FILE * stream)
{
#if 1
    uint8_t c;
    loop_until_bit_is_set(UCSR0A, RXC0);
    if (UCSR0A & _BV(FE0))
	return _FDEV_EOF;
    if (UCSR0A & _BV(DOR0))
	return _FDEV_ERR;
    c = UDR0;

    return c;
#else
    uint8_t c;
    char *cp, *cp2;
    static char b[RX_BUFSIZE];
    static char *rxp;

    if (rxp == 0)
	for (cp = b;;) {
	    loop_until_bit_is_set(UCSR0A, RXC0);
	    if (UCSR0A & _BV(FE0))
		return _FDEV_EOF;
	    if (UCSR0A & _BV(DOR0))
		return _FDEV_ERR;
	    c = UDR0;
	    /* behaviour similar to Unix stty ICRNL */
	    if (c == '\r')
		c = '\n';
	    if (c == '\n') {
		*cp = c;
		uart_putchar(c, stream);
		rxp = b;
		break;
	    } else if (c == '\t')
		c = ' ';

	    if ((c >= (uint8_t) ' ' && c <= (uint8_t) '\x7e') || c >= (uint8_t) '\xa0') {
		if (cp == b + RX_BUFSIZE - 1)
		    uart_putchar('\a', stream);
		else {
		    *cp++ = c;
		    uart_putchar(c, stream);
		}
		continue;
	    }

	    switch (c) {
	    case 'c' & 0x1f:
		return -1;

	    case '\b':
	    case '\x7f':
		if (cp > b) {
		    uart_putchar('\b', stream);
		    uart_putchar(' ', stream);
		    uart_putchar('\b', stream);
		    cp--;
		}
		break;

	    case 'r' & 0x1f:
		uart_putchar('\r', stream);
		for (cp2 = b; cp2 < cp; cp2++)
		    uart_putchar(*cp2, stream);
		break;

	    case 'u' & 0x1f:
		while (cp > b) {
		    uart_putchar('\b', stream);
		    uart_putchar(' ', stream);
		    uart_putchar('\b', stream);
		    cp--;
		}
		break;

	    case 'w' & 0x1f:
		while (cp > b && cp[-1] != ' ') {
		    uart_putchar('\b', stream);
		    uart_putchar(' ', stream);
		    uart_putchar('\b', stream);
		    cp--;
		}
		break;
	    }
	}

    c = *rxp++;
    if (c == '\n')
	rxp = 0;

    return c;
#endif
}
