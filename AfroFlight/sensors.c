#include "main.h"

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

/* Gyro Pitch/Roll/Yaw final measurements */
vs16 gyro[3] = { 512, 512, 512 };
/* Accelerometer X/Y/Z */
vs16 acc[3] = { 0, 0, 0 };
/* Battery voltage, v * 10 */
vs16 battery = 100;
/* Gyro offsets */
s16 gyroZero[3] = { 0, 0, 0 };                                // used for calibrating Gyros on ground

/* Accumulated sensor values - this is set by ADC and SPI read interrupts */
/* 0:Pitch 1:Roll 2:Yaw 3:Battery Voltage 4:AX 5:AY 6:AZ */
static vs16 sensorInputs[7] = { 0, };
/* Are we currently reading ADC data? */
static vu8 isAdcReading = FALSE;

static u16 batteryWarning;	// Battery Warning Voltage
static u16 voltageLevel = 100;	// Battery Voltage

static u8 ADXL_WriteByte(u8 Data)
{
    /* Wait until the transmit buffer is empty */
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);
    /* Send the byte */
    SPI_SendData(Data);
    /* Wait to receive a byte*/
    while(SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
    /*Return the byte read from the SPI bus */ 
    return SPI_ReceiveData();
}

static u8 ADXL_ReadByte(void)
{
    vu8 Data = 0;
    /* Wait until the transmit buffer is empty */
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);
    /* Send the byte */
    SPI_SendData(0xFF); // Dummy Byte
    /* Wait until a data is received */
    while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
    /* Get the received data */
    Data = SPI_ReceiveData();
    /* Return the shifted data */
    return Data;
}

static void ADXL_Init(void)
{
    // setup ADXL345 rate/range/start measuring
    
    // Rate
    ADXL_ON;
    ADXL_WriteByte(ADXL_RATE_ADDR);
    ADXL_WriteByte(ADXL_RATE_3200 & 0x0F);
    ADXL_OFF;

    // Range
    ADXL_ON;
    ADXL_WriteByte(ADXL_FORMAT_ADDR);
    ADXL_WriteByte((ADXL_RANGE_4G & 0x03) | ADXL_FULL_RES | ADXL_4WIRE);
    ADXL_OFF;

    // Fifo depth = 16
    ADXL_ON;
    ADXL_WriteByte(ADXL_FIFO_ADDR);
    ADXL_WriteByte((16 & 0x1f) | ADXL_FIFO_STREAM);
    ADXL_OFF;

    ADXL_ON;
    ADXL_WriteByte(ADXL_POWER_ADDR);
    ADXL_WriteByte(ADXL_MEASURE);
    ADXL_OFF;
}

static u8 ADXL_GetAccelValues(void)
{
    vu8 i;
    
    ADXL_ON;
    ADXL_WriteByte(ADXL_X0_ADDR | ADXL_MULTI_BIT | ADXL_READ_BIT);
    for (i = 0; i < 3; i++) {
        u8 i1, i2;
        i1 = ADXL_ReadByte();
        i2 = ADXL_ReadByte();
        sensorInputs[i + 4] += (i1 | i2 << 8);
    }
    // skip over this register
    ADXL_ReadByte();
    // FIFO_STATUS register (last few bits = fifo remaining)
    i = ADXL_ReadByte();
    ADXL_OFF;

    return i & 0x7F; // return number of entires left in fifo
}

/*
    AIN0        Gyro Roll
    AIN1        Gyro Pitch
    AIN2        Gyro Yaw
    AIN3        Voltage
    
    Analog IRQ handler - end of conversion
*/
__near __interrupt void ADC1_IRQHandler(void)
{
    u8 i = 0;
    static vu8 count = 0;
    
    // Get 4 ADC readings from buffer
    for (i = 0; i < 4; i++)
        sensorInputs[i] += ADC1_GetBufferValue(i);

    ADC1_ClearITPendingBit(ADC1_CSR_EOC);

    // loop 8 times
    count++;
    if (count < 8)
        ADC1_StartConversion();
    else {
        count = 0;
        isAdcReading = FALSE;
    }
}

/* Public functions */
void Sensors_Init(void)
{
    // SPI
    SPI_DeInit();
    SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER, SPI_CLOCKPOLARITY_HIGH, SPI_CLOCKPHASE_2EDGE, SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 0x07);
    SPI_Cmd(ENABLE);
    
    // SPI ChipSelect for Accel
    GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);
    ADXL_OFF;
    
    // Accel INT1 input tied to interrupt (TODO). Input-only for now.
    GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_IN_FL_NO_IT);

    //  ADC1
    ADC1_DeInit();
    GPIO_Init(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_MODE_IN_FL_NO_IT);
    ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, ADC1_CHANNEL_3, ADC1_PRESSEL_FCPU_D8, ADC1_EXTTRIG_TIM, DISABLE, ADC1_ALIGN_RIGHT, ADC1_SCHMITTTRIG_ALL, DISABLE);
    ADC1_DataBufferCmd(ENABLE);
    ADC1_ScanModeCmd(ENABLE);
    ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);

    // TODO remove
    ADC1_StartConversion();

    // Initialize SPI Accelerometer
    ADXL_Init();
}

/* 
 * Find Gyro offset, and make sure model was not moving during the operation. Will beep and flash LED if some movement is detected.
 * TODO: Check for unreasonable values (broken gyros)
*/
void Gyro_Calibrate(void)
{
    u8 i, j;
    u16 gyroCheck[8 * 3];       // gyro offset calc
    s16 gyroDiff = 0;
    
    do {
        for (i = 0; i < 3; i++)
            gyroZero[i] = 0;

	for (i = 0; i < 8; i++) {
            for (j = 0; j < 3; j++) {
                Sensors_Read();
                gyroZero[j] += gyro[j];
                gyroCheck[i + (j * 8)] = gyro[j];
            }
	    delay_ms(100);
	}
        
        for (i = 0; i < 3; i++) {
            gyroZero[i] = gyroZero[i] >> 3;
            if (gyroZero[i] > 500 || gyroZero[i] < 300) {
                while (1) {
                    Sensors_Read();
                    Beep(1, 400, 50);
                    Beep(2, 100, 25);
                    LED_TOGGLE;
                }
            }
        }
            
	LED_TOGGLE;

        // compare mean value vs individual values
        // if individual values differ from mean, then the copter was moved. Redo offset measurement.
        gyroDiff = 0;
	for (i = 0; i < 8; i++) {
            for (j = 0; j < 3; j++) {
                u16 temp = gyroZero[j] - gyroCheck[i + (j * 8)];
                gyroDiff += abs(temp);
            }
	}
        printf("GyroDiff: %d\r\n", (u16)gyroDiff);
        if (gyroDiff > 8)
            Beep(2, 50, 10);
    } while (gyroDiff > 8);

    printf("Gyro Calibrated: %d, %d, %d\r\n", gyroZero[0], gyroZero[1], gyroZero[2]);
    LED_OFF;
}

void Sensors_Read(void)
{
    u8 i = 0;
    u8 remaining = 0, count = 0;

    // Fire off 8 adc reads (Gyro, Voltage)
    isAdcReading = TRUE;
    ADC1_StartConversion();
    // Wait for the sample loop to complete
    while (isAdcReading);

    // average 8 gyro readings
    gyro[0] = (sensorInputs[0] >> 3); // - gyroZero[0];
    gyro[1] = (sensorInputs[1] >> 3); // - gyroZero[1];
    gyro[2] = (sensorInputs[2] >> 3); // - gyroZero[2];
    
    // Next up is accel fifo + avg
    do {
        count++;
        remaining = ADXL_GetAccelValues();
    } while ((count < 32) && (remaining > 0));

    // accel + average
    acc[0] = sensorInputs[4] / count;
    acc[1] = sensorInputs[5] / count;
    acc[2] = sensorInputs[6] / count;

    // 3.3V reference
    battery = (3 * battery + (11 * (sensorInputs[3] >> 3)) / 30) / 4;

    // clear for next pass
    for (i = 0; i < 8; i++)
        sensorInputs[i] = 0;
}

void Sensors_Debug(void)
{
    printf("Acc: %d %d %d Gyro: %d %d %d Volt: %d\r\n", acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], battery);
}

void Voltage_Init(void)
{
    u8 i;

    // measure a few times to start rolling average
    for (i = 0; i < 30; i++)
        Sensors_Read();

    for (i = 2; i < 6; i++) {
	if (battery < i * MAX_LIPO_CELL_VOLTAGE)
	    break;
    }
    batteryWarning = i * Config.VoltagePerCellMin;	// 3.3V per cell minimum, configurable in GUI

    // Beep (blocking) number of cells
    Beep(i, 80, 200);
}

u8 Voltage_Check(void)
{
    if (battery != 0 && battery < batteryWarning)
        return 1;
        
    return 0;
}
