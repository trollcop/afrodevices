#ifndef I2C_H_
#define I2C_H_

#define I2C_START 0
#define I2C_DATA  1
#define I2C_STOP  2
#define SCL_CLOCK 400000L

void i2c_init(void);
int i2c_write(uint8_t address, uint8_t data);

#endif	/* I2C_H_ */
