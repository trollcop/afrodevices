/* System dependent file for STM32F103CxT6 for CopterControl hardware */
#include "board.h"
#include "config.h"
#include "def.h"
#include "sysdep.h"

/* HW init */
void hw_init(void)
{
    SystemInit();

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* Configure all unused GPIO as Analog Input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    // LED
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    digitalHi(GPIOA, GPIO_Pin_6);
    digitalLo(GPIOA, GPIO_Pin_6);

    usb_cdcacm_enable();
}

/* UART */
static uint8_t uartPointer;
static uint8_t uartBuffer[128];
static uint8_t tx_ptr;
static uint8_t tx_busy = 0;

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
    if (!(usbIsConnected() && usbIsConfigured()))
        return;

    uint32_t txed = 0;
    txed += usb_cdcacm_tx((const uint8_t *)uartBuffer, uartPointer);
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
    return usb_cdcacm_data_available();
}

uint8_t Serial_read(void)
{
    uint8_t buf[4];
    uint32_t remaining = 0;
    uint32_t len = 1;

    if (!(usbIsConnected() && usbIsConfigured()))
        return -1;
    
    remaining = usb_cdcacm_rx((uint8_t *)&buf[0], len);
    return buf[0];
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

// PWM Functions
void pwmInit(uint8_t useServo)
{
    
}

void pwmWrite(uint8_t channel, uint16_t value)
{
    
    
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

void systemReboot(void)
{
    
}
