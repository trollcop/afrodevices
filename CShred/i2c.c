// #define T580

//***************************************************************************
//  File Name    : i2cbus.c
//  Version      : 1.0
//  Description  : I2Cbus EEPROM AVR Microcontroller Interface
//  Author(s)    : RWB
//  Target(s)    : AVRJazz Mega168 Learning Board
//  Compiler     : AVR-GCC 4.3.0; avr-libc 1.6.2 (WinAVR 20080610)
//  IDE          : Atmel AVR Studio 4.14
//  Programmer   : AVRJazz Mega168 STK500 v2.0 Bootloader
//               : AVR Visual Studio 4.14, STK500 programmer
//  Last Updated : 28 Dec 2008
//***************************************************************************
#include <avr/io.h>
#include <util/delay.h>
#include <compat/twi.h>
#include "i2c.h"

#define MAX_TRIES 1

static uint8_t i2c_transmit(uint8_t type)
{
    uint8_t count = 0;

    switch (type) {
    case I2C_START:		// Send Start Condition
        TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	break;
    case I2C_DATA:		// Send Data
        TWCR = (1 << TWINT) | (1 << TWEN);
	break;
    case I2C_STOP:		// Send Stop Condition
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	return 0;
    }

    // Wait for TWINT flag set in TWCR Register
    while (!(TWCR & (1 << TWINT)) && count++ < 250);

    // Return TWI Status Register, mask the prescaler bits (TWPS1,TWPS0)
    return (TWSR & 0xF8);
}

void i2c_init(void)
{
    // SDA is INPUT
    DDRC &= ~(1 << DDC1);
    // SCL is output
    DDRC |= (1 << DDC0);
    // pull up SDA
    PORTC |= (1 << PORTC0) | (1 << PORTC1);

    // TWI Status Register
    // prescaler 1 (TWPS1 = 0, TWPS0 = 0)
    TWSR &= ~((1 << TWPS1) | (1 << TWPS0));
    // set TWI Bit Rate Register
    TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
}

int i2c_write(uint8_t address, uint8_t data)
{
    unsigned char n = 0;
    unsigned char twi_status;
    char r_val = -1;

    // Transmit Start Condition
    twi_status = i2c_transmit(I2C_START);

    // Check the TWI Status
    if ((twi_status != TW_START) && (twi_status != TW_REP_START))
	goto i2c_quit;

    // Send slave address (SLA_W)
    TWDR = address | TW_WRITE;

    // Transmit I2C Data
    twi_status = i2c_transmit(I2C_DATA);

    // Check the TWSR status
    if (twi_status == TW_MT_SLA_NACK)
	goto i2c_quit;
    if (twi_status != TW_MT_SLA_ACK)
	goto i2c_quit;

#ifdef T580
    // transmit dummy byte
    TWDR = 0xA2;

    // Transmit I2C Data
    twi_status = i2c_transmit(I2C_DATA);

    // Check the TWSR status
    if (twi_status != TW_MT_DATA_ACK)
	goto i2c_quit;
#endif

    // Put data into data register and start transmission
    TWDR = data;

    // Transmit I2C Data
    twi_status = i2c_transmit(I2C_DATA);

    // Check the TWSR status
    if (twi_status != TW_MT_DATA_ACK)
	goto i2c_quit;

    // TWI Transmit Ok
    r_val = 1;

  i2c_quit:
    // Transmit I2C Data
    twi_status = i2c_transmit(I2C_STOP);
    return r_val;
}

#if 0
int i2c_readbyte(unsigned int i2c_address, unsigned int dev_id, unsigned int dev_addr, char *data)
{
    unsigned char n = 0;
    unsigned char twi_status;
    char r_val = -1;

  i2c_retry:
    if (n++ >= MAX_TRIES)
	return r_val;

    // Transmit Start Condition
    twi_status = i2c_transmit(I2C_START);

    // Check the TWSR status
    if (twi_status == TW_MT_ARB_LOST)
	goto i2c_retry;
    if ((twi_status != TW_START) && (twi_status != TW_REP_START))
	goto i2c_quit;

    // Send slave address (SLA_W) 0xa0
    TWDR = (dev_id & 0xF0) | ((dev_addr << 1) & 0x0E) | TW_WRITE;

    // Transmit I2C Data
    twi_status = i2c_transmit(I2C_DATA);

    // Check the TWSR status
    if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MT_ARB_LOST))
	goto i2c_retry;
    if (twi_status != TW_MT_SLA_ACK)
	goto i2c_quit;

    // Send the Low 8-bit of I2C Address
    TWDR = i2c_address;

    // Transmit I2C Data
    twi_status = i2c_transmit(I2C_DATA);

    // Check the TWSR status
    if (twi_status != TW_MT_DATA_ACK)
	goto i2c_quit;

    // Send the High 8-bit of I2C Address
    TWDR = i2c_address >> 8;

    // Transmit I2C Data
    twi_status = i2c_transmit(I2C_DATA);

    // Check the TWSR status
    if (twi_status != TW_MT_DATA_ACK)
	goto i2c_quit;

    // Send start Condition
    twi_status = i2c_transmit(I2C_START);

    // Check the TWSR status
    if (twi_status == TW_MT_ARB_LOST)
	goto i2c_retry;
    if ((twi_status != TW_START) && (twi_status != TW_REP_START))
	goto i2c_quit;

    // Send slave address (SLA_R)
    TWDR = (dev_id & 0xF0) | ((dev_addr << 1) & 0x0E) | TW_READ;

    // Transmit I2C Data
    twi_status = i2c_transmit(I2C_DATA);

    // Check the TWSR status
    if ((twi_status == TW_MR_SLA_NACK) || (twi_status == TW_MR_ARB_LOST))
	goto i2c_retry;
    if (twi_status != TW_MR_SLA_ACK)
	goto i2c_quit;

    // Read I2C Data
    twi_status = i2c_transmit(I2C_DATA);
    if (twi_status != TW_MR_DATA_NACK)
	goto i2c_quit;

    // Get the Data
    *data = TWDR;
    r_val = 1;

  i2c_quit:
    // Send Stop Condition
    twi_status = i2c_transmit(I2C_STOP);
    return r_val;
}
#endif
