#include "config.h"
#include "def.h"
#include "sysdep.h"

/* HW init */
void hw_init(void)
{
    // Clocks configuration
    CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE, DISABLE, CLK_CURRENTCLOCKSTATE_DISABLE);
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);
    CLK_ClockSecuritySystemEnable();

    TIM4_DeInit();
    TIM4_TimeBaseInit(TIM4_PRESCALER_64, 255);
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
    TIM4_ARRPreloadConfig(ENABLE);
    TIM4_Cmd(ENABLE);

    // Enable interrupts to start stuff up
    enableInterrupts();
}

void systemReboot(void)
{
    // reboot to bootloader
    WWDG_SWReset();
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

// ***********************************
// Interrupt driven UART transmitter
// ***********************************
static uint8_t tx_ptr;
static uint8_t tx_busy = 0;

__near __interrupt void UART2_TX_IRQHandler(void)
{
    UART2_SendData8(uartBuffer[tx_ptr++]);
    if (tx_ptr == uartPointer) {      /* Check if all data is transmitted */
        UART2_ITConfig(UART2_IT_TXE, DISABLE);  /* Disable transmitter interrupt */
        tx_busy = 0;
    }
}

void Serial_commitBuffer(void)
{
    tx_ptr = 0;
    UART2_SendData8(uartBuffer[tx_ptr++]);       /* Start transmission */
    UART2_ITConfig(UART2_IT_TXE, ENABLE);       /* Enable TX interrupt to continue sending */
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

__near __interrupt void UART2_RX_IRQHandler(void)
{
    uint8_t c;

    c = UART2_ReceiveData8();
    UART2_ClearFlag(UART2_FLAG_RXNE);
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

    disableInterrupts();
    m = timer0_overflow_count;
    t = TIM4->CNTR;
    enableInterrupts();

#if 0
    // if overflow occurred while we were here TODO
    if (timer0_overflow_count != m)
        m++;
#endif

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

void eeprom_open(void)
{
    FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
    FLASH_Unlock(FLASH_MEMTYPE_DATA);
}

void eeprom_read_block (void *dst, const void *src, size_t n)
{
    uint32_t _address = FLASH_DATA_START_PHYSICAL_ADDRESS + (uint32_t)src;
    memcpy(dst, (void *)_address, n);
}

void eeprom_write_block(const void *src, void *dst, size_t n)
{
    uint32_t _address = FLASH_DATA_START_PHYSICAL_ADDRESS + (uint32_t)dst;
    uint8_t j;

    uint8_t *data = (uint8_t *)src;
    for (j = 0; j < n; j++)
        FLASH_ProgramByte(_address++, *data++);
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
    SPI_DeInit();
    SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER, SPI_CLOCKPOLARITY_HIGH, SPI_CLOCKPHASE_2EDGE, SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT, 0x07);
    SPI_Cmd(ENABLE);
}

uint8_t spi_writeByte(uint8_t Data)
{
    /* Wait until the transmit buffer is empty */
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);
    /* Send the byte */
    SPI_SendData(Data);
    /* Wait to receive a byte */
    while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
    /*Return the byte read from the SPI bus */
    return SPI_ReceiveData();
}

uint8_t spi_readByte(void)
{
    volatile uint8_t data = 0;
    /* Wait until the transmit buffer is empty */
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET);
    /* Send the byte */
    SPI_SendData(0xFF);         // Dummy Byte
    /* Wait until a data is received */
    while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
    /* Get the received data */
    data = SPI_ReceiveData();
    /* Return the shifted data */
    return data;
}

// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************
void i2c_init(void)
{
    I2C_DeInit();
#if (I2C_SPEED == 100000)
    // retarded slow speed for broken boards
    I2C_Init(I2C_MAX_STANDARD_FREQ, 0xA0, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, I2C_MAX_INPUT_FREQ);
#else
    I2C_Init(I2C_MAX_FAST_FREQ, 0xA0, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, I2C_MAX_INPUT_FREQ);
#endif
    I2C_Cmd(ENABLE);
}

#define I2C_TIMEOUT 0x600

typedef enum {                  //returns I2C error/success codes
    I2C_SUCCESS = 0,            //theres only one sort of success
    I2C_START_TIMEOUT,
    I2C_RSTART_TIMEOUT,
    I2C_SACK_FAILURE,
    I2C_SACK_TIMEOUT,
    I2C_TX_TIMEOUT,
    I2C_RX_TIMEOUT
} I2C_Returntype;

uint8_t i2c_write(uint8_t *buf, uint8_t size)
{
    // Sets up an i2c device
    uint8_t n;
    uint16_t tm = 0;
    I2C_GenerateSTART(ENABLE);
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT)) {
        tm++;
        if (tm > I2C_TIMEOUT)
            return I2C_START_TIMEOUT;
    }
    tm = 0;
    I2C_Send7bitAddress(buf[0], I2C_DIRECTION_TX);      //Address write
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        tm++;
        if (tm > I2C_TIMEOUT)
            return I2C_SACK_TIMEOUT;
        if (SET == I2C_GetFlagStatus(I2C_FLAG_ACKNOWLEDGEFAILURE)) {
            I2C_ClearFlag(I2C_FLAG_ACKNOWLEDGEFAILURE);
            I2C_GenerateSTOP(ENABLE);   //Enable the STOP here - so hardware is ready again
            return I2C_SACK_FAILURE;    //Slave did not ack
        }
    }
    for (n = 1; n < size; n++) {
        tm = 0;
        I2C_SendData(buf[n]);   //Write rest of string (registers)
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
            tm++;
            if (tm > I2C_TIMEOUT)
                return I2C_TX_TIMEOUT;
        }
    }
    I2C_GenerateSTOP(ENABLE);   //Finally send the stop bit
    return I2C_SUCCESS;         //Completed ok
}

uint8_t i2c_read(uint8_t *buf, uint8_t size, uint8_t address, uint8_t subaddr)
{
    int8_t n = 0;               //0xFF as the Sub_Addr disables sub address
    uint16_t tm = 0;
    if (subaddr != 0xFF) {      //0xFF disables this - so we wont setup addr pointer
        I2C_GenerateSTART(ENABLE);
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT)) {
            tm++;
            if (tm > I2C_TIMEOUT)
                return I2C_START_TIMEOUT;
        }
        tm = 0;

        I2C_Send7bitAddress(address, I2C_DIRECTION_TX); //Address write
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
            tm++;
            if (tm > I2C_TIMEOUT)
                return I2C_SACK_TIMEOUT;        //Checks that the slave acknowledged
            if (SET == I2C_GetFlagStatus(I2C_FLAG_ACKNOWLEDGEFAILURE)) {
                I2C_ClearFlag(I2C_FLAG_ACKNOWLEDGEFAILURE);
                I2C_GenerateSTOP(ENABLE);       //Enable the STOP here - so hardware is ready again
                return I2C_SACK_FAILURE;        //Slave did not ack
            }
        }
        tm = 0;
        I2C_SendData(subaddr);  //Write sub address register
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
            tm++;
            if (tm > I2C_TIMEOUT)
                return I2C_TX_TIMEOUT;
        }
        //I2C_GenerateSTOP( I2C1, ENABLE ); //This code doesnt _seem_ to be needed
        //while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY)==SET); //Wait for bus to go inactive
    }
    tm = 0;
    I2C_GenerateSTART(ENABLE);  //Repeated start or the first start
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT)) {
        tm++;
        if (tm > I2C_TIMEOUT)
            return I2C_RSTART_TIMEOUT;  //note that if we disable sub addr, then a start error
    }                           //becomes a repeated start error
    tm = 0;
    I2C_Send7bitAddress(address | 0x01, I2C_DIRECTION_RX);      //Address to read
    while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        tm++;
        if (tm > I2C_TIMEOUT)
            return I2C_SACK_TIMEOUT;    //Checks that the slave acknowledged
        if (SET == I2C_GetFlagStatus(I2C_FLAG_ACKNOWLEDGEFAILURE)) {
            I2C_ClearFlag(I2C_FLAG_ACKNOWLEDGEFAILURE);
            I2C_GenerateSTOP(ENABLE);   //Enable the STOP here - so hardware is ready again
            return I2C_SACK_FAILURE;    //Slave did not ack
        }
    }                           //We now auto switch to rx mode
    if (size > 2) {             //More than two bytes to receive
        tm = 0;
        while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) {       //Wait for the first byte
            tm++;
            if (tm > I2C_TIMEOUT)
                return I2C_RX_TIMEOUT;
        }
        for (; n < ((int8_t) size - 3); n++) {
            tm = 0;
            buf[n] = I2C_ReceiveData();
            while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) {
                tm++;
                if (tm > I2C_TIMEOUT)
                    return I2C_RX_TIMEOUT;
            }
        }
        tm = 0;
        while (I2C_GetFlagStatus(I2C_FLAG_TRANSFERFINISHED) != SET) {   //Wait for two bytes to be received - ref man p712
            tm++;
            if (tm > I2C_TIMEOUT)
                return I2C_RX_TIMEOUT;
        }
        I2C_AcknowledgeConfig(DISABLE); //Do not ack the last byte
        buf[n++] = I2C_ReceiveData();   //Third to last byte
        I2C_GenerateSTOP(ENABLE);       //Enable the STOP here
        buf[n++] = I2C_ReceiveData();   //Read the Penultimate from buffer
        tm = 0;
        while (I2C_GetFlagStatus(I2C_FLAG_RXNOTEMPTY) != SET) { //Last byte received here with a NACK and STOP
            tm++;
            if (tm > I2C_TIMEOUT)
                return I2C_RX_TIMEOUT;
        }
    } else {
        I2C_AcknowledgeConfig(DISABLE); //Do not ack the last byte
        tm = 0;
        while (I2C_GetFlagStatus(I2C_FLAG_TRANSFERFINISHED) != SET) {   //Wait for two bytes to be received - ref man p713
            tm++;
            if (tm > I2C_TIMEOUT)
                return I2C_RX_TIMEOUT;
        }
        I2C_GenerateSTOP(ENABLE);       //Enable the STOP here
        buf[n++] = I2C_ReceiveData();   //First byte to lowest location
    }
    buf[n] = I2C_ReceiveData(); //Clear the buffer (last byte is in it)
    I2C_AcknowledgeConfig(ENABLE);      //Re-enable ACK
    return I2C_SUCCESS;         //Exit ok
}

/* Fixed lookup table for TIM1/2 Pulse Width registers */
static const struct {
    volatile uint8_t *addressH;
    volatile uint8_t *addressL;
} TimerAddress[] = { {
    &(TIM1->CCR1H), &(TIM1->CCR1L)}, {
    &(TIM1->CCR2H), &(TIM1->CCR2L)}, {
    &(TIM1->CCR3H), &(TIM1->CCR3L)}, {
    &(TIM1->CCR4H), &(TIM1->CCR4L)}, {
    &(TIM2->CCR2H), &(TIM2->CCR2L)}, {
    &(TIM2->CCR1H), &(TIM2->CCR1L)}
};

// 1ms pulse width (we have 0.5us precision)
#define PULSE_1MS       (2000)
// pulse period (400Hz)
#define PULSE_PERIOD    (5000)
// pulse period for digital servo (200Hz)
#define PULSE_PERIOD_SERVO_DIGITAL  (10000)
// pulse period for analog servo (50Hz)
#define PULSE_PERIOD_SERVO_ANALOG  (40000)

void pwmInit(uint8_t useServo)
{
    // Motor PWM timers at 400Hz
    TIM1_DeInit();
    TIM1_TimeBaseInit(7, TIM1_COUNTERMODE_UP, PULSE_PERIOD, 0);
    TIM1_OC1Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE, PULSE_1MS, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_RESET, TIM1_OCNIDLESTATE_RESET);
    TIM1_OC1PreloadConfig(ENABLE);
    TIM1_OC2Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE, PULSE_1MS, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_RESET, TIM1_OCNIDLESTATE_RESET);
    TIM1_OC2PreloadConfig(ENABLE);
    TIM1_OC3Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE, PULSE_1MS, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_RESET, TIM1_OCNIDLESTATE_RESET);
    TIM1_OC3PreloadConfig(ENABLE);
    TIM1_OC4Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, PULSE_1MS, TIM1_OCPOLARITY_LOW, TIM1_OCIDLESTATE_RESET);
    TIM1_OC4PreloadConfig(ENABLE);
    TIM1_ARRPreloadConfig(ENABLE);
    TIM1_CtrlPWMOutputs(ENABLE);

    if (!useServo) {
        // last 2 motor channels at 400Hz
        TIM2_DeInit();
        TIM2_TimeBaseInit(TIM2_PRESCALER_8, PULSE_PERIOD);
        TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, PULSE_1MS, TIM2_OCPOLARITY_LOW);
        TIM2_OC1PreloadConfig(ENABLE);
        TIM2_OC2Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, PULSE_1MS, TIM2_OCPOLARITY_LOW);
        TIM2_OC2PreloadConfig(ENABLE);
        TIM2_ARRPreloadConfig(ENABLE);
    } else {
        // Servos
        // Enable TIM2 for servo output - slower rates
        TIM2_DeInit();
#ifdef DIGITAL_SERVO
        TIM2_TimeBaseInit(TIM2_PRESCALER_8, PULSE_PERIOD_SERVO_DIGITAL);
#else
        TIM2_TimeBaseInit(TIM2_PRESCALER_8, PULSE_PERIOD_SERVO_ANALOG);
#endif
        TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, PULSE_1MS, TIM2_OCPOLARITY_LOW);
        TIM2_OC1PreloadConfig(ENABLE);
        TIM2_OC2Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, PULSE_1MS, TIM2_OCPOLARITY_LOW);
        TIM2_OC2PreloadConfig(ENABLE);
        TIM2_ARRPreloadConfig(ENABLE);
        TIM2_Cmd(ENABLE);
    }

    TIM1_Cmd(ENABLE);
    if (!useServo)
        TIM2_Cmd(ENABLE);
}

/* PWM write */
void pwmWrite(uint8_t channel, uint16_t value)
{
    uint16_t pulse = (value << 1);
    *TimerAddress[channel].addressH = (uint8_t) (pulse >> 8);
    *TimerAddress[channel].addressL = (uint8_t) (pulse);
}
