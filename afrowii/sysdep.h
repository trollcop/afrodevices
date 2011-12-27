#pragma once

#include <stdlib.h>

#define INPUT 0x0
#define OUTPUT 0x1

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

/*
 * Bit manipulation
 */

/** 1 << the bit number */
#define BIT(shift)                     (1UL << (shift))
/** Mask shifted left by 'shift' */
#define BIT_MASK_SHIFT(mask, shift)    ((mask) << (shift))
/** Bits m to n of x */
#define GET_BITS(x, m, n) ((((uint32_t)x) << (31 - (n))) >> ((31 - (n)) + (m)))

/* hardware abstraction */

/* this would setup clocks/timers/etc */
void hw_init(void);

/* SPI */
void spi_init(void);
uint8_t spi_writeByte(uint8_t Data);
uint8_t spi_readByte(void);
/* I2C */
void i2c_init(void);
uint8_t i2c_read(uint8_t *buf, uint8_t size, uint8_t address, uint8_t subaddr);
uint8_t i2c_write(uint8_t *buf, uint8_t size);

/* UART */
void serialize8(uint8_t val);
void serialize16(int16_t val);
void Serial_begin(uint32_t speed);
void Serial_reset(void);
uint16_t Serial_available(void);
uint8_t Serial_read(void);
void Serial_commitBuffer(void);
uint8_t Serial_isTxBusy(void);

/* System */
void delay(uint16_t ms);
uint32_t micros(void);
uint32_t millis(void);
uint16_t analogRead(uint8_t channel);
void analogWrite(uint8_t pin, uint16_t value);
void pinMode(uint8_t pin, uint8_t mode);
void systemReboot(void);

/* PWM */
void pwmInit(uint8_t useServo);
void pwmWrite(uint8_t channel, uint16_t value);

void eeprom_open(void);
void eeprom_read_block(void *dst, const void *src, size_t n);
void eeprom_write_block(const void *src, void *dst, size_t n);
void eeprom_close(void);

#define sei()
#define cli()
