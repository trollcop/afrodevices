#pragma once

#define INPUT 0x0
#define OUTPUT 0x1

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

void delay(uint16_t ms);
uint32_t micros(void);
uint16_t analogRead(uint8_t channel);
void analogWrite(uint8_t pin, uint16_t value);
void pinMode(uint8_t pin, uint8_t mode);
void Serial_begin(uint32_t speed);
uint16_t Serial_available(void);
uint8_t Serial_read(void);

void eeprom_read_block (void *__dst, const void *__src, size_t __n);
void eeprom_write_block (const void *__src, void *__dst, size_t __n);

#define sei()
#define cli()
