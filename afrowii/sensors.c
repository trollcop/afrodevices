#include "board.h"
#include "def.h"
#include "config.h"
#include "sysdep.h"
#include "mw.h"

extern int16_t i2c_errors_count;
extern uint8_t calibratingM;
extern uint16_t calibratingA;
extern uint16_t calibratingG;
extern volatile int16_t sensorInputs[7];

/* SENSORS ------------------------------------------------------------------------------ */
// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************
//default board orientation
#if !defined(ACC_ORIENTATION)
#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = X; accADC[PITCH]  = Y; accADC[YAW]  = Z;}
#endif
#if !defined(GYRO_ORIENTATION)
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = X; gyroADC[PITCH] = Y; gyroADC[YAW] = Z;}
#endif
#if !defined(MAG_ORIENTATION)
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  = X; magADC[PITCH]  = Y; magADC[YAW]  = Z;}
#endif

/*** I2C address ***/
#if !defined(ADXL345_ADDRESS)
#define ADXL345_ADDRESS 0x3A
//#define ADXL345_ADDRESS 0xA6   //WARNING: Conflicts with a Wii Motion plus!
#endif

#if !defined(BMA180_ADDRESS)
#define BMA180_ADDRESS 0x80
//#define BMA180_ADDRESS 0x82
#endif

#if !defined(ITG3200_ADDRESS)
#define ITG3200_ADDRESS 0XD0
//#define ITG3200_ADDRESS 0XD2
#endif

#if !defined(MS561101BA_ADDRESS)
#define MS561101BA_ADDRESS 0xEE //CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)
//#define MS561101BA_ADDRESS 0xEF //CBR=1 0xEF I2C address when pin CSB is connected to HIGH (VCC)
#endif

//ITG3200 and ITG3205 Gyro LPF setting
#if defined(ITG3200_LPF_256HZ) || defined(ITG3200_LPF_188HZ) || defined(ITG3200_LPF_98HZ) || defined(ITG3200_LPF_42HZ) || defined(ITG3200_LPF_20HZ) || defined(ITG3200_LPF_10HZ)
#if defined(ITG3200_LPF_256HZ)
#define ITG3200_SMPLRT_DIV 0    //8000Hz
#define ITG3200_DLPF_CFG   0
#endif
#if defined(ITG3200_LPF_188HZ)
#define ITG3200_SMPLRT_DIV 0    //1000Hz
#define ITG3200_DLPF_CFG   1
#endif
#if defined(ITG3200_LPF_98HZ)
#define ITG3200_SMPLRT_DIV 0
#define ITG3200_DLPF_CFG   2
#endif
#if defined(ITG3200_LPF_42HZ)
#define ITG3200_SMPLRT_DIV 0
#define ITG3200_DLPF_CFG   3
#endif
#if defined(ITG3200_LPF_20HZ)
#define ITG3200_SMPLRT_DIV 0
#define ITG3200_DLPF_CFG   4
#endif
#if defined(ITG3200_LPF_10HZ)
#define ITG3200_SMPLRT_DIV 0
#define ITG3200_DLPF_CFG   5
#endif
#else
//Default settings LPF 256Hz/8000Hz sample
#define ITG3200_SMPLRT_DIV 0    //8000Hz
#define ITG3200_DLPF_CFG   0
#endif

uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;

void i2c_getSixRawADC(uint8_t add, uint8_t reg)
{
    uint8_t rv = 0;
    rv = i2c_read(rawADC, 6, add, reg);
    if (rv != 0)
        i2c_errors_count++;
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val)
{
    uint8_t buf[3];
    uint8_t rv = 0;
    buf[0] = add;
    buf[1] = reg;
    buf[2] = val;
    rv = i2c_write(buf, 3);
    if (rv != 0)
        i2c_errors_count++;
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg)
{
    uint8_t data[1];
    uint8_t rv = 0;
    rv = i2c_read(data, 1, add, reg);
    if (rv != 0)
        i2c_errors_count++;
    return data[0];
}

// ****************
// GYRO common part
// ****************
void GYRO_Common(void)
{
    static int16_t previousGyroADC[3] = { 0, 0, 0 };
    static int32_t g[3];
    uint8_t axis;

    if (calibratingG > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset g[axis] at start of calibration
            if (calibratingG == 400)
                g[axis] = 0;
            // Sum up 400 readings
            g[axis] += gyroADC[axis];
            // Clear global variables for next reading
            gyroADC[axis] = 0;
            gyroZero[axis] = 0;
            if (calibratingG == 1) {
                gyroZero[axis] = g[axis] / 400;
                blinkLED(10, 15, 1);
            }
        }
        calibratingG--;
    }
    for (axis = 0; axis < 3; axis++) {
        gyroADC[axis] -= gyroZero[axis];
        //anti gyro glitch, limit the variation between two consecutive readings
        gyroADC[axis] = constrain(gyroADC[axis], previousGyroADC[axis] - 800, previousGyroADC[axis] + 800);
        previousGyroADC[axis] = gyroADC[axis];
    }
}

// ****************
// ACC common part
// ****************
void ACC_Common(void)
{
    static int32_t a[3];
    uint8_t axis;

    if (calibratingA > 0) {
        for (axis = 0; axis < 3; axis++) {
            // Reset a[axis] at start of calibration
            if (calibratingA == 400)
                a[axis] = 0;
            // Sum up 400 readings
            a[axis] += accADC[axis];
            // Clear global variables for next reading
            accADC[axis] = 0;
            accZero[axis] = 0;
        }
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        if (calibratingA == 1) {
            accZero[ROLL] = a[ROLL] / 400;
            accZero[PITCH] = a[PITCH] / 400;
            accZero[YAW] = a[YAW] / 400 - acc_1G;       // for nunchuk 200=1G
            cfg.accTrim[ROLL] = 0;
            cfg.accTrim[PITCH] = 0;
            writeParams();      // write accZero in EEPROM
        }
        calibratingA--;
    }
    accADC[ROLL] -= accZero[ROLL];
    accADC[PITCH] -= accZero[PITCH];
    accADC[YAW] -= accZero[YAW];
}


// ************************************************************************************************************
// I2C Barometer BOSCH BMP085
// ************************************************************************************************************
// I2C adress: 0xEE (8bit)   0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)
// ************************************************************************************************************

#if defined(BMP085)
#define BMP085_ADDRESS  0xEE
#define BMP085_CTRL     0xF4
#define BMP085_ADC      0xF6
#define BMP085_TEMP     0x2E
#define BMP085_DATA     0xAA

static struct {
    // sensor registers from the BOSCH BMP085 datasheet
    int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
    uint16_t ac4, ac5, ac6;
    uint16_t ut;                       //uncompensated T
    uint32_t up;                       //uncompensated P
    uint8_t state;
    uint32_t deadline;
} bmp085_ctx;
#define OSS 3

// read a 16 bit register
int16_t i2c_BMP085_readIntRegister(uint8_t r)
{
    uint8_t raw[2];

    i2c_read(raw, 2, BMP085_ADDRESS, r);
    return (int16_t)raw[0] << 8 | raw[1];
}

void i2c_BMP085_readCalibration(void)
{
    delay(10);
    bmp085_ctx.ac1 = i2c_BMP085_readIntRegister(0xAA);
    bmp085_ctx.ac2 = i2c_BMP085_readIntRegister(0xAC);
    bmp085_ctx.ac3 = i2c_BMP085_readIntRegister(0xAE);
    bmp085_ctx.ac4 = i2c_BMP085_readIntRegister(0xB0);
    bmp085_ctx.ac5 = i2c_BMP085_readIntRegister(0xB2);
    bmp085_ctx.ac6 = i2c_BMP085_readIntRegister(0xB4);
    bmp085_ctx.b1 = i2c_BMP085_readIntRegister(0xB6);
    bmp085_ctx.b2 = i2c_BMP085_readIntRegister(0xB8);
    bmp085_ctx.mb = i2c_BMP085_readIntRegister(0xBA);
    bmp085_ctx.mc = i2c_BMP085_readIntRegister(0xBC);
    bmp085_ctx.md = i2c_BMP085_readIntRegister(0xBE);
}

// read uncompensated temperature value: send command first
void i2c_BMP085_UT_Start(void)
{
    i2c_writeReg(BMP085_ADDRESS, BMP085_CTRL, BMP085_TEMP);
}

// read uncompensated pressure value: send command first
void i2c_BMP085_UP_Start(void)
{
    i2c_writeReg(BMP085_ADDRESS, BMP085_CTRL, 0x34 + (OSS << 6));      // control register value for oversampling setting 3
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_BMP085_UP_Read(void)
{
    uint8_t raw[3];
    i2c_read(raw, 3, BMP085_ADDRESS, BMP085_ADC);
    bmp085_ctx.up = ((((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | ((uint32_t)raw[2])) >> (8 - OSS));
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_BMP085_UT_Read(void)
{
    uint8_t raw[2];

    i2c_read(raw, 2, BMP085_ADDRESS, BMP085_ADC);
    bmp085_ctx.ut = (uint16_t)raw[0] << 8 | raw[1];
}

void Baro_init(void)
{
    delay(10);
    i2c_BMP085_readCalibration();
    i2c_BMP085_UT_Start();
    delay(5);
    i2c_BMP085_UT_Read();
}

void i2c_BMP085_Calculate(void)
{
    int32_t x1, x2, x3, b3, b5, b6, p, tmp;
    uint32_t b4, b7;
    // Temperature calculations
    x1 = ((int32_t) bmp085_ctx.ut - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;
    x2 = ((int32_t) bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
    b5 = x1 + x2;
    // Pressure calculations
    b6 = b5 - 4000;
    x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12)) >> 11;
    x2 = bmp085_ctx.ac2 * b6 >> 11;
    x3 = x1 + x2;
    tmp = bmp085_ctx.ac1;
    tmp = (tmp * 4 + x3) << OSS;
    b3 = (tmp + 2) / 4;
    x1 = bmp085_ctx.ac3 * b6 >> 13;
    x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (bmp085_ctx.ac4 * (uint32_t) (x3 + 32768)) >> 15;
    // b7 = ((uint32_t) (bmp085_ctx.up >> (8 - OSS)) - b3) * (50000 >> OSS);
    b7 = ((uint32_t)bmp085_ctx.up - b3) * (50000 >> OSS);
    p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    pressure = p + ((x1 + x2 + 3791) >> 4);
}

void Baro_update(void)
{
    if (currentTime < bmp085_ctx.deadline)
        return;

    bmp085_ctx.deadline = currentTime;
    switch (bmp085_ctx.state) {
    case 0:
        i2c_BMP085_UT_Start();
        bmp085_ctx.state++;
        bmp085_ctx.deadline += 4600;
        break;
    case 1:
        i2c_BMP085_UT_Read();
        bmp085_ctx.state++;
        break;
    case 2:
        i2c_BMP085_UP_Start();
        bmp085_ctx.state++;
        bmp085_ctx.deadline += 26000;
        break;
    case 3:
        i2c_BMP085_UP_Read();
        i2c_BMP085_Calculate();
        BaroAlt = (1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
        bmp085_ctx.state = 0;
        bmp085_ctx.deadline += 20000;
        break;
    }
}
#endif

// ************************************************************************************************************
// I2C Barometer MS561101BA
// ************************************************************************************************************
// first contribution from Fabio
// modification from Alex (September 2011)
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12
#if defined(MS561101BA)

// registers of the device
#define MS561101BA_PRESSURE 0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET 0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096

static struct {
    // sensor registers from the MS561101BA datasheet
    uint16_t c[7];
    union {
        uint32_t val;
        uint8_t raw[4];
    } ut;                       //uncompensated T
    union {
        uint32_t val;
        uint8_t raw[4];
    } up;                       //uncompensated P
    uint8_t state;
    uint32_t deadline;
} ms561101ba_ctx;

void i2c_MS561101BA_reset(void)
{
    i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);
}

void i2c_MS561101BA_readCalibration(void)
{
    union {
        uint16_t val;
        uint8_t raw[2];
    } data;
    uint8_t i;

    delay(10);
    for (i = 0; i < 6; i++) {
        i2c_rep_start(MS561101BA_ADDRESS + 0);
        i2c_write(0xA2 + 2 * i);
        i2c_rep_start(MS561101BA_ADDRESS + 1);  //I2C read direction => 1
        data.raw[1] = i2c_readAck();    // read a 16 bit register
        data.raw[0] = i2c_readNak();
        ms561101ba_ctx.c[i + 1] = data.val;
    }
}

void Baro_init(void)
{
    delay(10);
    i2c_MS561101BA_reset();
    delay(10);
    i2c_MS561101BA_readCalibration();
}

// read uncompensated temperature value: send command first
void i2c_MS561101BA_UT_Start(void)
{
    i2c_rep_start(MS561101BA_ADDRESS + 0);      // I2C write direction
    i2c_write(MS561101BA_TEMPERATURE + OSR);    // register selection
}

// read uncompensated pressure value: send command first
void i2c_MS561101BA_UP_Start(void)
{
    i2c_rep_start(MS561101BA_ADDRESS + 0);      // I2C write direction
    i2c_write(MS561101BA_PRESSURE + OSR);       // register selection
}

// read uncompensated pressure value: read result bytes
void i2c_MS561101BA_UP_Read(void)
{
    i2c_rep_start(MS561101BA_ADDRESS + 0);
    i2c_write(0);
    i2c_rep_start(MS561101BA_ADDRESS + 1);
    ms561101ba_ctx.up.raw[2] = i2c_readAck();
    ms561101ba_ctx.up.raw[1] = i2c_readAck();
    ms561101ba_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
void i2c_MS561101BA_UT_Read(void)
{
    i2c_rep_start(MS561101BA_ADDRESS + 0);
    i2c_write(0);
    i2c_rep_start(MS561101BA_ADDRESS + 1);
    ms561101ba_ctx.ut.raw[2] = i2c_readAck();
    ms561101ba_ctx.ut.raw[1] = i2c_readAck();
    ms561101ba_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_MS561101BA_Calculate(void)
{
    int64_t dT = ms561101ba_ctx.ut.val - ((uint32_t) ms561101ba_ctx.c[5] << 8); // int32_t according to the spec, but int64_t here to avoid cast after
    int64_t off = ((uint32_t) ms561101ba_ctx.c[2] << 16) + ((dT * ms561101ba_ctx.c[4]) >> 7);
    int64_t sens = ((uint32_t) ms561101ba_ctx.c[1] << 15) + ((dT * ms561101ba_ctx.c[3]) >> 8);
    pressure = (((ms561101ba_ctx.up.val * sens) >> 21) - off) >> 15;
}

void Baro_update(void)
{
    if (currentTime < ms561101ba_ctx.deadline)
        return;
    ms561101ba_ctx.deadline = currentTime;
    TWBR = ((16000000L / 400000L) - 16) / 2;    // change the I2C clock rate to 400kHz, MS5611 is ok with this speed
    switch (ms561101ba_ctx.state) {
    case 0:
        i2c_MS561101BA_UT_Start();
        ms561101ba_ctx.state++;
        ms561101ba_ctx.deadline += 15000;       // according to the specs, the pause should be at least 8.22ms
        break;
    case 1:
        i2c_MS561101BA_UT_Read();
        ms561101ba_ctx.state++;
        break;
    case 2:
        i2c_MS561101BA_UP_Start();
        ms561101ba_ctx.state++;
        ms561101ba_ctx.deadline += 15000;       // according to the specs, the pause should be at least 8.22ms
        break;
    case 3:
        i2c_MS561101BA_UP_Read();
        i2c_MS561101BA_Calculate();
        BaroAlt = (1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f;
        ms561101ba_ctx.state = 0;
        ms561101ba_ctx.deadline += 30000;
        break;
    }
}
#endif




// ************************************************************************************************************
// I2C Accelerometer ADXL345 
// ************************************************************************************************************
// I2C adress: 0x3A (8bit)    0x1D (7bit)
// Resolution: 10bit (Full range - 14bit, but this is autoscaling 10bit ADC to the range +- 16g)
// principle:
//  1) CS PIN must be linked to VCC to select the I2C mode
//  2) SD0 PIN must be linked to VCC to select the right I2C adress
//  3) bit  b00000100 must be set on register 0x2D to read data (only once at the initialization)
//  4) bits b00001011 must be set on register 0x31 to select the data format (only once at the initialization)
// ************************************************************************************************************
#if defined(ADXL345)
void ACC_init(void)
{
    delay(10);
    i2c_writeReg(ADXL345_ADDRESS, 0x2D, 1 << 3);        //  register: Power CTRL  -- value: Set measure bit 3 on
    i2c_writeReg(ADXL345_ADDRESS, 0x31, 0x0B);  //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
    i2c_writeReg(ADXL345_ADDRESS, 0x2C, 8 + 2 + 1);     // register: BW_RATE     -- value: 200Hz sampling (see table 5 of the spec)
    acc_1G = 256;
}

void ACC_getADC(void)
{
    i2c_getSixRawADC(ADXL345_ADDRESS, 0x32);

    ACC_ORIENTATION(-((rawADC[3] << 8) | rawADC[2]), ((rawADC[1] << 8) | rawADC[0]), ((rawADC[5] << 8) | rawADC[4]));
    ACC_Common();
}
#endif

#if defined(ADXL345SPI)

/* Private defines */
#define ADXL_OFF	   GPIO_WriteHigh(GPIOE, GPIO_PIN_5);
#define ADXL_ON		   GPIO_WriteLow(GPIOE, GPIO_PIN_5);

#define ADXL_READ_BIT      0x80
#define ADXL_MULTI_BIT     0x40
#define ADXL_X0_ADDR       0x32
#define ADXL_RATE_ADDR     0x2C
#define ADXL_FIFO_ADDR     0x38
#define ADXL_RATE_100      0x0A
#define ADXL_RATE_200      0x0B
#define ADXL_RATE_400      0x0C
#define ADXL_RATE_800      0x0D
#define ADXL_RATE_1600     0x0E
#define ADXL_RATE_3200     0x0F
#define ADXL_POWER_ADDR    0x2D
#define ADXL_MEASURE       0x08
#define ADXL_FORMAT_ADDR   0x31
#define ADXL_FULL_RES      0x08
#define ADXL_4WIRE         0x00
#define ADXL_RANGE_2G      0x00
#define ADXL_RANGE_4G      0x01
#define ADXL_RANGE_8G      0x02
#define ADXL_RANGE_16G     0x03
#define ADXL_FIFO_STREAM   0x80

static void ADXL_Init(void)
{
    // setup ADXL345 rate/range/start measuring
    // Rate 3200Hz
    ADXL_ON;
    spi_writeByte(ADXL_RATE_ADDR);
    spi_writeByte(ADXL_RATE_800 & 0x0F);
    ADXL_OFF;

    // Range 8G
    ADXL_ON;
    spi_writeByte(ADXL_FORMAT_ADDR);
    spi_writeByte((ADXL_RANGE_8G & 0x03) | ADXL_FULL_RES | ADXL_4WIRE);
    ADXL_OFF;

    // Fifo depth = 16
    ADXL_ON;
    spi_writeByte(ADXL_FIFO_ADDR);
    spi_writeByte((16 & 0x1f) | ADXL_FIFO_STREAM);
    ADXL_OFF;

    ADXL_ON;
    spi_writeByte(ADXL_POWER_ADDR);
    spi_writeByte(ADXL_MEASURE);
    ADXL_OFF;
}

static uint8_t ADXL_GetAccelValues(void)
{
    volatile uint8_t i;

    ADXL_ON;
    spi_writeByte(ADXL_X0_ADDR | ADXL_MULTI_BIT | ADXL_READ_BIT);
    for (i = 0; i < 3; i++) {
        uint8_t i1, i2;
        i1 = spi_readByte();
        i2 = spi_readByte();

#ifdef LOWPASS_ACC
        // new result = 0.95 * previous_result + 0.05 * current_data
        sensorInputs[i + 4] = ((sensorInputs[i + 4] * 19) / 20) + (((i1 | (i2 << 8)) * 5) / 100);
#else
        sensorInputs[i + 4] += (i1 + (i2 << 8));
#endif
    }

    // skip over this register
    spi_readByte();
    // FIFO_STATUS register (last few bits = fifo remaining)
    i = spi_readByte();
    ADXL_OFF;

    return i & 0x7F;            // return number of entires left in fifo
}

void ACC_init()
{
    // SPI ChipSelect for Accel
    GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);
    ADXL_OFF;

    delay(10);

    // Accel INT1 input tied to interrupt (TODO). Input-only for now.
    GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_IN_FL_NO_IT);

    // Initialize SPI Accelerometer
    ADXL_Init();
    acc_1G = 256;
}

void ACC_getADC()
{
    uint8_t count = 0;
    uint8_t remaining = 0;
    uint8_t i = 0;

    // Next up is accel fifo + avg
    do {
        count++;
        remaining = ADXL_GetAccelValues();
    } while ((count < 32) && (remaining > 0));

    count++;

#ifdef LOWPASS_ACC
    // commit current values to acc[]
#else
    // accel + average
    sensorInputs[4] = sensorInputs[4] / count;
    sensorInputs[5] = sensorInputs[5] / count;
    sensorInputs[6] = sensorInputs[6] / count;
#endif

    ACC_ORIENTATION(sensorInputs[4], sensorInputs[5], sensorInputs[6]);
    ACC_Common();
}
#endif

#if defined(MPU6000SPI)
static uint8_t mpuInitialized = 0;
#define MPU_OFF            GPIO_WriteHigh(GPIOB, GPIO_PIN_2);
#define MPU_ON             GPIO_WriteLow(GPIOB, GPIO_PIN_2);

#define MPUREG_WHOAMI               0x75
#define MPUREG_SMPLRT_DIV           0x19
#define MPUREG_CONFIG               0x1A
#define MPUREG_GYRO_CONFIG          0x1B
#define MPUREG_ACCEL_CONFIG         0x1C
#define MPUREG_I2C_MST_CTRL         0x24
#define MPUREG_I2C_SLV0_ADDR        0x25
#define MPUREG_I2C_SLV0_REG         0x26
#define MPUREG_I2C_SLV0_CTRL        0x27
#define MPUREG_I2C_SLV4_ADDR        0x31
#define MPUREG_I2C_SLV4_REG         0x32
#define MPUREG_I2C_SLV4_DO          0x33
#define MPUREG_I2C_SLV4_CTRL        0x34
#define MPUREG_I2C_SLV4_DI          0x35
#define MPUREG_I2C_MST_STATUS       0x36
#define MPUREG_INT_PIN_CFG          0x37
#define MPUREG_INT_ENABLE           0x38 
#define MPUREG_ACCEL_XOUT_H         0x3B
#define MPUREG_ACCEL_XOUT_L         0x3C
#define MPUREG_ACCEL_YOUT_H         0x3D
#define MPUREG_ACCEL_YOUT_L         0x3E
#define MPUREG_ACCEL_ZOUT_H         0x3F
#define MPUREG_ACCEL_ZOUT_L         0x40
#define MPUREG_TEMP_OUT_H           0x41
#define MPUREG_TEMP_OUT_L           0x42
#define MPUREG_GYRO_XOUT_H          0x43
#define MPUREG_GYRO_XOUT_L          0x44
#define MPUREG_GYRO_YOUT_H          0x45
#define MPUREG_GYRO_YOUT_L          0x46
#define MPUREG_GYRO_ZOUT_H          0x47
#define MPUREG_GYRO_ZOUT_L          0x48
#define MPUREG_EXT_SENS_DATA_00     0x49 // Registers 0x49 to 0x60 - External Sensor Data
#define MPUREG_I2C_SLV0_DO          0x63 // This register holds the output data written into Slave 0 when Slave 0 is set to write mode.
#define MPUREG_I2C_MST_DELAY_CTRL   0x67 // I2C Master Delay Control
#define MPUREG_USER_CTRL            0x6A
#define MPUREG_PWR_MGMT_1           0x6B
#define MPUREG_PWR_MGMT_2           0x6C

// Configuration bits MPU 6000
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_AFS_2G                 0x00
#define BITS_AFS_4G                 0x08
#define BITS_AFS_8G                 0x10
#define BITS_AFS_16G                0x18
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_I2C_SLV0_EN             0x80

static uint8_t MPU6000_Buffer[14 + 6];   // Sensor data ACCXYZ|TEMP|GYROXYZ | Magnetometer data

static uint8_t MPU6000_ReadReg(uint8_t Address)
{
    uint8_t rv;
    MPU_ON;
    spi_writeByte(Address | 0x80); // Address with high bit set = Read operation
    rv = spi_readByte();
    MPU_OFF;
    return rv;
}

static void MPU6000_getSixRawADC(void)
{
    uint8_t i;
    MPU_ON;
    spi_writeByte(MPUREG_ACCEL_XOUT_H | 0x80); // Address with high bit set = Read operation
    // ACC X, Y, Z, TEMP, GYRO X, Y, Z, MagData[6]
    for (i = 0; i < 14 + 6; i++)
        MPU6000_Buffer[i] = spi_readByte();
    MPU_OFF;
}

static void MPU6000_WriteReg(uint8_t Address, uint8_t Data)
{ 
    MPU_ON;
    spi_writeByte(Address); 
    spi_writeByte(Data);
    MPU_OFF;
    delay(1);
}

void MPU6000_init(void)
{
    // SPI ChipSelect for MPU-6000
    GPIO_Init(GPIOB, GPIO_PIN_2, GPIO_MODE_OUT_PP_HIGH_FAST);
    MPU_OFF;

    // MPU-6000 input tied to interrupt (TODO). Input-only for now.
    GPIO_Init(GPIOB, GPIO_PIN_3, GPIO_MODE_IN_FL_NO_IT);

    MPU6000_WriteReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
    delay(100);
    MPU6000_WriteReg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);      // Set PLL source to gyro output
    MPU6000_WriteReg(MPUREG_USER_CTRL, 0b00110000);                 // I2C_MST_EN
    // MPU6000_WriteReg(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);             // Disable I2C bus
    MPU6000_WriteReg(MPUREG_SMPLRT_DIV, 0x04);                      // Sample rate = 200Hz    Fsample = 1Khz / (4 + 1) = 200Hz   
    MPU6000_WriteReg(MPUREG_CONFIG, 0); // BITS_DLPF_CFG_42HZ);            // Fs & DLPF Fs = 1kHz, DLPF = 42Hz (low pass filter)
    MPU6000_WriteReg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);          // Gyro scale 2000º/s
    MPU6000_WriteReg(MPUREG_ACCEL_CONFIG, BITS_AFS_4G);             // Accel scale 4G
    MPU6000_WriteReg(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);            // INT: Raw data ready
    MPU6000_WriteReg(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR);     // INT: Clear on any read

    mpuInitialized = 1;
}

void ACC_init()
{
    if (!mpuInitialized)
        MPU6000_init();
        
    acc_1G = 256;
}

void ACC_getADC()
{
    MPU6000_getSixRawADC();
    ACC_ORIENTATION(-(MPU6000_Buffer[0] << 8 | MPU6000_Buffer[1]) / 16, -(MPU6000_Buffer[2] << 8 | MPU6000_Buffer[3]) / 16, (MPU6000_Buffer[4] << 8 | MPU6000_Buffer[5]) / 16);
    ACC_Common();
}

void Gyro_init(void)
{
    if (!mpuInitialized)
        MPU6000_init();
}

void Gyro_getADC(void)
{
    // range: +/- 8192; +/- 2000 deg/sec
    MPU6000_getSixRawADC();
    GYRO_ORIENTATION((((MPU6000_Buffer[10] << 8) | MPU6000_Buffer[11]) / 4), -(((MPU6000_Buffer[8] << 8) | MPU6000_Buffer[9]) / 4), -(((MPU6000_Buffer[12] << 8) | MPU6000_Buffer[13]) / 4));
    GYRO_Common();
}

#define HMC5883L_I2C_ADDRESS        0x1e
#define HMC5883L_ID_REG_A           0x0a
#define HMC5883L_ID_REG_B           0x0b
#define HMC5883L_ID_REG_C           0x0c
#define HMC5883L_MODE_REG           0x02
#define HMC5883L_DATA_OUTPUT_X      0x03

void Mag_init(void)
{
    volatile uint8_t i, temp;
    
    // Initialize compass for continous measurement mode
    MPU6000_WriteReg(MPUREG_I2C_MST_CTRL, 0b01000000 | 13); // WAIT_FOR_ES=1, I2C Master Clock Speed 400kHz
    MPU6000_WriteReg(MPUREG_I2C_SLV4_ADDR, HMC5883L_I2C_ADDRESS); // Write to 5883
    MPU6000_WriteReg(MPUREG_I2C_SLV4_REG, HMC5883L_MODE_REG);
    MPU6000_WriteReg(MPUREG_I2C_SLV4_DO, 0x00); // Mode register  --  value: Continuous-Conversion Mode
    MPU6000_WriteReg(MPUREG_I2C_SLV4_CTRL, 0b11000000); // I2C_SLV4_EN | I2C_SLV4_INT_EN
    delay(1);

    // temp = MPU6000_ReadReg(MPUREG_I2C_MST_STATUS);
    // temp = MPU6000_ReadReg(MPUREG_I2C_SLV4_DI);

    // Prepare I2C Slave 0 for reading out mag data
    MPU6000_WriteReg(MPUREG_I2C_SLV0_ADDR, 0x80 | HMC5883L_I2C_ADDRESS); // Read from 5883
    MPU6000_WriteReg(MPUREG_I2C_SLV0_REG, HMC5883L_DATA_OUTPUT_X);
    MPU6000_WriteReg(MPUREG_I2C_SLV0_CTRL, 0b10000110); // I2C_SLV0_EN | 6 bytes from mag
    delay(1);
}

void Device_Mag_getADC(void)
{
    MAG_ORIENTATION( ((MPU6000_Buffer[18] << 8) | MPU6000_Buffer[19]),  ((MPU6000_Buffer[14] << 8) | MPU6000_Buffer[15]),   ((MPU6000_Buffer[16] << 8) | MPU6000_Buffer[17])    );
}
#endif /* MPU6000SPI */

// ************************************************************************************************************
// contribution initially from opie11 (rc-groups)
// adaptation from C2po (may 2011)
// contribution from ziss_dm (June 2011)
// contribution from ToLuSe (Jully 2011)
// I2C Accelerometer BMA180
// ************************************************************************************************************
// I2C adress: 0x80 (8bit)    0x40 (7bit) (SDO connection to VCC) 
// I2C adress: 0x82 (8bit)    0x41 (7bit) (SDO connection to VDDIO)
// Resolution: 14bit
//
// Control registers:
//
// 0x20    bw_tcs:   |                                           bw<3:0> |                        tcs<3:0> |
//                   |                                             150Hz |                 !!Calibration!! |
// ************************************************************************************************************
#if defined(BMA180)
void ACC_init()
{
    uint8_t control;
    
    delay(10);
    //default range 2G: 1G = 4096 unit.
    i2c_writeReg(BMA180_ADDRESS, 0x0D, 1 << 4); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
    delay(5);
    control = i2c_readReg(BMA180_ADDRESS, 0x20);
    control = control & 0x0F;   // register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 10Hz (bits value = 0000xxxx)
    control = control | 0x00;
    i2c_writeReg(BMA180_ADDRESS, 0x20, control);
    delay(5);
    control = i2c_readReg(BMA180_ADDRESS, 0x30);
    control = control & 0xFC;
    control = control | 0x02;
    i2c_writeReg(BMA180_ADDRESS, 0x30, control);
    delay(5);
    acc_1G = 512;
}

void ACC_getADC()
{
#ifndef STM8
    TWBR = ((16000000L / 400000L) - 16) / 2;    // Optional line.  Sensor is good for it in the spec.
#endif
    i2c_getSixRawADC(BMA180_ADDRESS, 0x02);
    //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /8 => 11 bit resolution
    ACC_ORIENTATION(-((rawADC[1] << 8) | rawADC[0]) / 32, -((rawADC[3] << 8) | rawADC[2]) / 32, ((rawADC[5] << 8) | rawADC[4]) / 32);
    ACC_Common();
}
#endif

// ************************************************************************************************************
// contribution from Point65 and mgros (rc-groups)
// contribution from ziss_dm (June 2011)
// contribution from ToLuSe (Jully 2011)
// I2C Accelerometer BMA020
// ************************************************************************************************************
// I2C adress: 0x70 (8bit)
// Resolution: 10bit
// Control registers:
//
// Datasheet: After power on reset or soft reset it is recommended to set the SPI4-bit to the correct value.
//            0x80 = SPI four-wire = Default setting
// | 0x15: | SPI4 | enable_adv_INT | new_data_INT | latch_INT | shadow_dis | wake_up_pause<1:0> | wake_up |
// |       |    1 |              0 |            0 |         0 |          0 |                 00 |       0 |
//
// | 0x14: |                       reserved <2:0> |            range <1:0> |               bandwith <2:0> |
// |       |                      !!Calibration!! |                     2g |                         25Hz |
//
// ************************************************************************************************************
#if defined(BMA020)
void ACC_init()
{
    uint8_t control;

    i2c_writeReg(0x70, 0x15, 0x80);
    control = i2c_readReg(0x70, 0x14);
    control = control & 0xE0;
    control = control | (0x00 << 3);    //Range 2G 00
    control = control | 0x00;   //Bandwidth 25 Hz 000
    i2c_writeReg(0x70, 0x14, control);
    acc_1G = 255;
}

void ACC_getADC()
{
    i2c_getSixRawADC(0x70, 0x02);
    ACC_ORIENTATION(((rawADC[1] << 8) | rawADC[0]) / 64, ((rawADC[3] << 8) | rawADC[2]) / 64, ((rawADC[5] << 8) | rawADC[4]) / 64);
    ACC_Common();
}
#endif

// ************************************************************************
// LIS3LV02 I2C Accelerometer
//contribution from adver (http://multiwii.com/forum/viewtopic.php?f=8&t=451)
// ************************************************************************
#if defined(LIS3LV02)
#define LIS3A  0x3A             // I2C adress: 0x3A (8bit)

void i2c_ACC_init(void)
{
    i2c_writeReg(LIS3A, 0x20, 0xD7);    // CTRL_REG1   1101 0111 Pwr on, 160Hz 
    i2c_writeReg(LIS3A, 0x21, 0x50);    // CTRL_REG2   0100 0000 Littl endian, 12 Bit, Boot
    acc_1G = 256;
}

void i2c_ACC_getADC(void)
{
    TWBR = ((16000000L / 400000L) - 16) / 2;    // change the I2C clock rate to 400kHz
    i2c_getSixRawADC(LIS3A, 0x28 + 0x80);
    ACC_ORIENTATION((rawADC[3] << 8 | rawADC[2]) / 4, -(rawADC[1] << 8 | rawADC[0]) / 4, -(rawADC[5] << 8 | rawADC[4]) / 4);
    ACC_Common();
}
#endif

// ************************************************************************************************************
// I2C Accelerometer LSM303DLx
// contribution from wektorx (http://www.multiwii.com/forum/viewtopic.php?f=8&t=863)
// ************************************************************************************************************
#if defined(LSM303DLx_ACC)
void ACC_init()
{
    delay(10);
    i2c_writeReg(0x30, 0x20, 0x27);
    i2c_writeReg(0x30, 0x23, 0x30);
    i2c_writeReg(0x30, 0x21, 0x00);
    acc_1G = 256;
}

void ACC_getADC()
{
    i2c_getSixRawADC(0x30, 0xA8);

    ACC_ORIENTATION(-((rawADC[3] << 8) | rawADC[2]) / 16, ((rawADC[1] << 8) | rawADC[0]) / 16, ((rawADC[5] << 8) | rawADC[4]) / 16);
    ACC_Common();
}
#endif

// ************************************************************************************************************
// ADC ACC
// ************************************************************************************************************
#if defined(ADCACC)
void ACC_init(void)
{
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    acc_1G = 75;
}

void ACC_getADC(void)
{
    ACC_ORIENTATION(-analogRead(A1), -analogRead(A2), analogRead(A3));
    ACC_Common();
}
#endif

// ************************************************************************************************************
// Analog Gyroscopes IDG500 + ISZ500
// ************************************************************************************************************
#ifdef STM8
static volatile uint8_t adcInProgress = 0;

__near __interrupt void ADC1_IRQHandler(void)
{
    uint8_t i = 0;

#if 0
    // clear at start of loop
    if (adcSampleCount == 0 || adcSampleCount > 30) {
        sensorInputs[0] = 0;
        sensorInputs[1] = 0;
        sensorInputs[2] = 0;
        sensorInputs[3] = 0;
        adcSampleCount = 0;
    }
    adcSampleCount++;
#endif

    // Get 4 ADC readings from buffer
    for (i = 0; i < 4; i++)
        sensorInputs[i] = ADC1_GetBufferValue(i);

    ADC1_ClearITPendingBit(ADC1_CSR_EOC);
    adcInProgress = 0;
}
#endif

#if defined(STM8) && defined(ADCGYRO)
void Gyro_init(void)
{
    // ADC1
    ADC1_DeInit();
    GPIO_Init(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_MODE_IN_FL_NO_IT);
    ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_3, ADC1_PRESSEL_FCPU_D2, ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_ALL, DISABLE);
    ADC1_DataBufferCmd(ENABLE);
    ADC1_ScanModeCmd(ENABLE);
    ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);
}

void Gyro_getADC(void)
{
    // read out
    adcInProgress = 1;
    ADC1_StartConversion();
    while (adcInProgress);      // wait for conversion

    GYRO_ORIENTATION(-(sensorInputs[0]) * 5, (sensorInputs[1]) * 5, -(sensorInputs[2]) * 5);
    GYRO_Common();
}
#endif

// ************************************************************************************************************
// contribution from Ciskje
// I2C Gyroscope L3G4200D 
// ************************************************************************************************************
#if defined(L3G4200D)
void Gyro_init()
{
    delay(100);
    i2c_writeReg(0XD2 + 0, 0x20, 0x8F); // CTRL_REG1   400Hz ODR, 20hz filter, run!
    delay(5);
    i2c_writeReg(0XD2 + 0, 0x24, 0x02); // CTRL_REG5   low pass filter enable
}

void Gyro_getADC()
{
    TWBR = ((16000000L / 400000L) - 16) / 2;    // change the I2C clock rate to 400kHz
    i2c_getSixRawADC(0XD2, 0x80 | 0x28);

    GYRO_ORIENTATION(((rawADC[1] << 8) | rawADC[0]) / 20, ((rawADC[3] << 8) | rawADC[2]) / 20, -((rawADC[5] << 8) | rawADC[4]) / 20);
    GYRO_Common();
}
#endif

// ************************************************************************************************************
// I2C Gyroscope ITG3200 
// ************************************************************************************************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)
// I2C adress: 0xD0 (8bit)   0x68 (7bit)
// principle:
// 1) VIO is connected to VDD
// 2) I2C adress is set to 0x69 (AD0 PIN connected to VDD)
// or 2) I2C adress is set to 0x68 (AD0 PIN connected to GND)
// 3) sample rate = 1000Hz ( 1kHz/(div+1) )
// ************************************************************************************************************
#if defined(ITG3200)
void Gyro_init(void)
{
    delay(100);
    i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x80);  //register: Power Management  --  value: reset device
    //  delay(5);
    //  i2c_writeReg(ITG3200_ADDRESS, 0x15, ITG3200_SMPLRT_DIV); //register: Sample Rate Divider  -- default value = 0: OK
    delay(5);
    i2c_writeReg(ITG3200_ADDRESS, 0x16, 0x18 + ITG3200_DLPF_CFG);       //register: DLPF_CFG - low pass filter configuration
    delay(5);
    i2c_writeReg(ITG3200_ADDRESS, 0x3E, 0x03);  //register: Power Management  --  value: PLL with Z Gyro reference
    delay(100);
}

void Gyro_getADC(void)
{
    i2c_getSixRawADC(ITG3200_ADDRESS, 0x1D);
    GYRO_ORIENTATION(+(((rawADC[2] << 8) | rawADC[3]) / 4),     // range: +/- 8192; +/- 2000 deg/sec
                     -(((rawADC[0] << 8) | rawADC[1]) / 4), -(((rawADC[4] << 8) | rawADC[5]) / 4));
    GYRO_Common();
}
#endif

#if defined(MPU3050)
// Registers
#define MPU3050_SMPLRT_DIV      0x15
#define MPU3050_DLPF_FS_SYNC    0x16
#define MPU3050_INT_CFG         0x17
#define MPU3050_TEMP_OUT        0x1B
#define MPU3050_GYRO_OUT        0x1D
#define MPU3050_USER_CTRL       0x3D
#define MPU3050_PWR_MGM         0x3E

// Bits
#define MPU3050_FS_SEL_2000DPS  0x18
#define MPU3050_DLPF_20HZ       0x04
#define MPU3050_DLPF_42HZ       0x03
#define MPU3050_DLPF_98HZ       0x02
#define MPU3050_DLPF_188HZ      0x01
#define MPU3050_DLPF_256HZ      0x00

#define MPU3050_USER_RESET      0x01
#define MPU3050_CLK_SEL_PLL_GX  0x01

void Gyro_init(void)
{
    delay(100);
    i2c_writeReg(MPU3050_ADDRESS, MPU3050_SMPLRT_DIV, 0);
    delay(5);
    i2c_writeReg(MPU3050_ADDRESS, MPU3050_DLPF_FS_SYNC, MPU3050_FS_SEL_2000DPS | MPU3050_DLPF_42HZ);
    i2c_writeReg(MPU3050_ADDRESS, MPU3050_INT_CFG, 0);
    i2c_writeReg(MPU3050_ADDRESS, MPU3050_USER_CTRL, MPU3050_USER_RESET);
    i2c_writeReg(MPU3050_ADDRESS, MPU3050_PWR_MGM, MPU3050_CLK_SEL_PLL_GX);
    delay(100);
}

void Gyro_getADC(void)
{
    i2c_getSixRawADC(MPU3050_ADDRESS, MPU3050_GYRO_OUT);
    GYRO_ORIENTATION(+(((rawADC[2] << 8) | rawADC[3]) / 4),     // range: +/- 8192; +/- 2000 deg/sec
                     -(((rawADC[0] << 8) | rawADC[1]) / 4), -(((rawADC[4] << 8) | rawADC[5]) / 4));
    GYRO_Common();
}
#endif





// ************************************************************************************************************
// I2C Compass common function
// ************************************************************************************************************
#if MAG
void Mag_getADC(void)
{
    static uint32_t t, tCal = 0;
    static int16_t magZeroTempMin[3];
    static int16_t magZeroTempMax[3];
    uint8_t axis;

    if (currentTime < t)
        return;                 //each read is spaced by 100ms
    t = currentTime + 100000;

    Device_Mag_getADC();
    if (calibratingM == 1) {
        tCal = t;
        for (axis = 0; axis < 3; axis++) {
            magZero[axis] = 0;
            magZeroTempMin[axis] = 0;
            magZeroTempMax[axis] = 0;
        }
        calibratingM = 0;
    }
    magADC[ROLL] -= magZero[ROLL];
    magADC[PITCH] -= magZero[PITCH];
    magADC[YAW] -= magZero[YAW];
    if (tCal != 0) {
        if ((t - tCal) < 30000000) {    // 30s: you have 30s to turn the multi in all directions
            LEDPIN_TOGGLE;
            for (axis = 0; axis < 3; axis++) {
                if (magADC[axis] < magZeroTempMin[axis])
                    magZeroTempMin[axis] = magADC[axis];
                if (magADC[axis] > magZeroTempMax[axis])
                    magZeroTempMax[axis] = magADC[axis];
            }
        } else {
            tCal = 0;
            for (axis = 0; axis < 3; axis++)
                magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) / 2;
            writeParams();
        }
    }
}
#endif

// ************************************************************************************************************
// I2C Compass HMC5843 & HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************
#if defined(HMC5843) || defined(HMC5883)
void Mag_init(void)
{
    delay(100);
    i2c_writeReg(0X3C, 0x02, 0x00);     //register: Mode register  --  value: Continuous-Conversion Mode
}

void Device_Mag_getADC(void)
{
    i2c_getSixRawADC(0X3C, 0X03);
#if defined(HMC5843)
    MAG_ORIENTATION(((rawADC[0] << 8) | rawADC[1]), ((rawADC[2] << 8) | rawADC[3]), -((rawADC[4] << 8) | rawADC[5]));
#endif
#if defined (HMC5883)
    MAG_ORIENTATION(((rawADC[4] << 8) | rawADC[5]), -((rawADC[0] << 8) | rawADC[1]), -((rawADC[2] << 8) | rawADC[3]));
#endif
}
#endif

// ************************************************************************************************************
// I2C Compass AK8975 (Contribution by EOSBandi)
// ************************************************************************************************************
// I2C adress: 0x18 (8bit)   0x0C (7bit)
// ************************************************************************************************************
#if defined(AK8975)
void Mag_init(void)
{
    delay(100);
    i2c_writeReg(0x18, 0x0a, 0x01);     //Start the first conversion
    delay(100);
}

void Device_Mag_getADC(void)
{
    i2c_getSixRawADC(0x18, 0x03);
    MAG_ORIENTATION(((rawADC[3] << 8) | rawADC[2]), ((rawADC[1] << 8) | rawADC[0]), -((rawADC[5] << 8) | rawADC[4]));
    //Start another meassurement
    i2c_writeReg(0x18, 0x0a, 0x01);
}
#endif

void initSensors(void)
{
    i2c_init();
    spi_init();
    delay(100);
    Gyro_init();
#if BARO
    Baro_init();
#endif
#if ACC
    ACC_init();
    acc_25deg = acc_1G * 0.423;
#endif
#if MAG
    Mag_init();
#endif
}
