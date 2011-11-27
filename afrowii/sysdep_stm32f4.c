#include "config.h"
#include "def.h"
#include "sysdep.h"

/* HW init */
void hw_init(void)
{

}

/* UART */
static uint8_t uartPointer;
static uint8_t uartBuffer[128];
void serialize16(int16_t a)
{
    uartBuffer[uartPointer++] = a;
    uartBuffer[uartPointer++] = a >> 8 & 0xff;
}

void serialize8(uint8_t a)
{
    uartBuffer[uartPointer++] = a;
}

void Serial_commitBuffer(void)
{

}

uint8_t Serial_isTxBusy(void)
{
    return tx_busy;
}

void Serial_reset(void)
{
    uartPointer = 0;
}

#define RX_BUFFER_SIZE 48
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

void Serial_begin(uint32_t speed)
{

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

/* TIMING - TODO Systick */
uint32_t micros(void)
{

}

void delay(uint16_t ms)
{

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

void eeprom_open(void)
{

}

void eeprom_read_block (void *dst, const void *src, size_t n)
{

}

void eeprom_write_block(const void *src, void *dst, size_t n)
{

}

void eeprom_close(void)
{
    FLASH_Lock(FLASH_MEMTYPE_DATA);
}

// ************************************************************************************************************
// SPI general functions
// ************************************************************************************************************
void spi_init(void)
{

}

uint8_t spi_writeByte(uint8_t Data)
{
    return 0;
}

uint8_t spi_readByte(void)
{
    return 0;
}

// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************
void i2c_init(void)
{

}

uint8_t i2c_write(uint8_t *buf, uint8_t size)
{
    return 0;
}

uint8_t i2c_read(uint8_t *buf, uint8_t size, uint8_t address, uint8_t subaddr)
{
    return 0;
}
