#include "config.h"
#include "def.h"
#include "sysdep.h"

void init(void);
void setup(void);
void loop(void);

void init(void)
{
    // Clocks configuration
    CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE, DISABLE, DISABLE);
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);
    CLK_ClockSecuritySystemEnable();

    TIM4_DeInit();
    TIM4_TimeBaseInit(TIM4_PRESCALER_64, 255);
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
    TIM4_Cmd(ENABLE);

    // Enable interrupts to start stuff up
    enableInterrupts();
}

int main(void)
{
    init();

    setup();

    for (;;)
        loop();
}
