/* System dependent file for STM32F103CxT6 for CopterControl hardware */
#include "board.h"
#include "config.h"
#include "def.h"
#include "sysdep.h"

static void systick_init(void);

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
    
    // systick
    systick_init();

    usb_cdcacm_enable();

    LEDPIN_ON;
    LEDPIN_OFF;
}

/* UART */
static uint8_t uartPointer;
static uint8_t uartBuffer[256];
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

#define USB_TIMEOUT 50

void Serial_commitBuffer(void)
{
    if (!(usbIsConnected() && usbIsConfigured()))
        return;

    uint32_t txed = 0;
    uint32_t old_txed = 0;
    uint32_t start = millis();
    uint32_t len = uartPointer;

    while (txed < len && (millis() - start < USB_TIMEOUT)) {
        txed += usb_cdcacm_tx((const uint8_t*)uartBuffer + txed, len - txed);
        if (old_txed != txed) {
            start = millis();
        }
        old_txed = txed;
    }
}

uint8_t Serial_isTxBusy(void)
{
    return tx_busy;
}

void Serial_reset(void)
{
    uartPointer = 0;
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
    uint8_t buf[1];
    uint32_t remaining = 0;
    uint32_t len = 1;

    if (!(usbIsConnected() && usbIsConfigured()))
        return -1;
    
    remaining = usb_cdcacm_rx((uint8_t *)&buf[0], len);
    return buf[0];
}

uint32_t runMillis = 0;

/* TIMING */
#define CYCLES_PER_MICROSECOND  72
#define SYSTICK_RELOAD_VAL      71999 /* takes a cycle to reload */
#define US_PER_MS               1000
#define STM32_DELAY_US_MULT     12

#define SYSTICK_CSR_ENABLE              BIT(0)
#define SYSTICK_CSR_CLKSOURCE_CORE      BIT(2)
#define SYSTICK_CSR_TICKINT_PEND        BIT(1)

static void systick_init(void)
{
    volatile unsigned int *SYSTICK_CSR = (int *)0xE000E010;
    volatile unsigned int *SYSTICK_RVR = (int *)0xE000E014;
    
    *SYSTICK_CSR = (SYSTICK_CSR_CLKSOURCE_CORE | SYSTICK_CSR_ENABLE | SYSTICK_CSR_TICKINT_PEND);
    *SYSTICK_RVR = SYSTICK_RELOAD_VAL;
    NVIC_SetPriority(SysTick_IRQn, 2);	    // lower priority
}

void SysTick_Handler(void)
{
    runMillis++;
}

uint32_t millis(void)
{
    return runMillis;
}

uint32_t micros(void)
{
    volatile unsigned int *SYSTICK_CNT = (int *)0xE000E018;
    uint32_t cycle_cnt;
    uint32_t ms;
    uint32_t res;

    do {
        ms = millis();
        cycle_cnt = *SYSTICK_CNT;
    } while (ms != millis());

    res = (ms * US_PER_MS) + (SYSTICK_RELOAD_VAL + 1 - cycle_cnt) / CYCLES_PER_MICROSECOND;

    return res;
}

static inline void delay_us(uint32_t us)
{
    us *= STM32_DELAY_US_MULT;

    /* fudge for function call overhead  */
    us--;
    __asm volatile("   mov r0, %[us]          \n\t"
        "1: subs r0, #1            \n\t"
        "   bhi 1b                 \n\t"
        :
    : [us] "r" (us)
        : "r0");
}

void delay(uint16_t ms)
{
    uint32_t i;
    for (i = 0; i < ms; i++)
        delay_us(1000);
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
