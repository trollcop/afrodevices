#include "config.h"
#include "def.h"
#include "sysdep.h"

/* UART */
#define RX_BUFFER_SIZE 32
typedef struct ring_buffer
{
  unsigned char buffer[RX_BUFFER_SIZE];
  int head;
  int tail;
} ring_buffer;

static ring_buffer rx_buffer  =  { { 0, }, 0, 0 };

/* __inline */ void store_char(unsigned char c, ring_buffer *rx_buffer)
{
  int i = (unsigned int)(rx_buffer->head + 1) % RX_BUFFER_SIZE;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != rx_buffer->tail) {
    rx_buffer->buffer[rx_buffer->head] = c;
    rx_buffer->head = i;
  }
}

__near __interrupt void UART2_RX_IRQHandler(void)
{
    uint8_t c;

    c = UART2_ReceiveData8();
    // UART2_ClearFlag(UART2_FLAG_RXNE);
    store_char(c, &rx_buffer);
}

void Serial_begin(uint32_t speed)
{
    UART2_DeInit();
    UART2_Init(speed, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO, UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);
    // UART2_ITConfig(UART2_IT_TXE, ENABLE);
    UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);
}

uint16_t Serial_available(void)
{
    return (uint16_t)(RX_BUFFER_SIZE + rx_buffer.head - rx_buffer.tail) % RX_BUFFER_SIZE;
}

uint8_t Serial_read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (rx_buffer.head == rx_buffer.tail) {
    return -1;
  } else {
    unsigned char c = rx_buffer.buffer[rx_buffer.tail];
    rx_buffer.tail = (unsigned int)(rx_buffer.tail + 1) % RX_BUFFER_SIZE;
    return c;
  }
}



/* TIMING */
#define F_CPU (16000000L)
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
#define microsecondsToClockCycles(a) ( ((a) * (F_CPU / 1000L)) / 1000L )
// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

__near __interrupt void TIM4_UPD_OVF_IRQHandler(void)
{
    // copy these to local variables so they can be stored in registers
    // (volatile variables must be read from memory on every access)
    unsigned long m = timer0_millis;
    unsigned char f = timer0_fract;

    // Optimize away a call() - TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
    TIM4->SR1 = (u8)(~TIM4_IT_UPDATE);

    m += MILLIS_INC;
    f += FRACT_INC;
    if (f >= FRACT_MAX) {
            f -= FRACT_MAX;
            m += 1;
    }

    timer0_fract = f;
    timer0_millis = m;
    timer0_overflow_count++;
}

uint32_t micros(void)
{
    unsigned long m;
    uint8_t t;

    m = timer0_overflow_count;
    t = TIM4_GetCounter();

    // if overflow occurred while we were here TODO
    if ((TIM4_GetFlagStatus(TIM4_IT_UPDATE) > 0) && (t < 255))
        m++;

    return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

void delay(uint16_t ms)
{
    uint16_t start = (uint16_t)micros();

    while (ms > 0) {
        if (((uint16_t)micros() - start) >= 1000) {
            ms--;
            start += 1000;
        }
    }
}

uint16_t analogRead(uint8_t channel)
{

    return 0;
}

void analogWrite(uint8_t pin, uint16_t value)
{

}

void pinMode(uint8_t pin, uint8_t mode)
{
    
}

void eeprom_read_block (void *__dst, const void *__src, size_t __n)
{
    
}

void eeprom_write_block (const void *__src, void *__dst, size_t __n)
{
    
}
