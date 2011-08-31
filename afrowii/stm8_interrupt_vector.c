/* BASIC INTERRUPT VECTORS TABLE FOR STM8 devices
 * Copyright (c) 2008 STMicroelectronics
 */

#include "stm8s_it.h"

extern void @near _stext(); /* startup routine */

void @near (* const _vectab[])() =
  {

    (void @near (*)())0x8200,
    _stext,                  /* RESET */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* TRAP - Software interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq0 - External Top Level interrupt (TLI) */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq1 - Auto Wake Up from Halt interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq2 - Clock Controller interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq3 - External interrupt 0 (GPIOA) */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq4 - External interrupt 1 (GPIOB) */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq5 - External interrupt 2 (GPIOC) */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq6 - External interrupt 3 (GPIOD) */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq7 - External interrupt 4 (GPIOE) */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq8 - CAN Rx interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq9 - CAN Tx/ER/SC interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq10 - SPI End of transfer interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq11 - TIM1 Update/Overflow/Trigger/Break interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq12 - TIM1 Capture/Compare interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq13 - TIM2 Update/Overflow/Break interrupt  */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq14 - TIM2 Capture/Compare interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq15 - TIM3 Update/Overflow/Break interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq16 - TIM3 Capture/Compare interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq17 - UART1 Tx complete interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq18 - UART1 Rx interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq19 - I2C interrupt */

    (void @near (*)())0x8200,
    UART2_TX_IRQHandler,   /* irq20 - UART2/UART3 Tx interrupt */

    (void @near (*)())0x8200,
    UART2_RX_IRQHandler,   /* irq21 - UART2/UART3 Rx interrupt */

    (void @near (*)())0x8200,
    ADC1_IRQHandler,   /* irq22 - ADC1/ADC2 end of conversion interrupt */

    (void @near (*)())0x8200,
    TIM4_UPD_OVF_IRQHandler,   /* irq23 - TIM4 Update/Overflow interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq24 - FLASH interrupt */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq25 - Reserved */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq26 - Reserved */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq27 - Reserved */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq28 - Reserved */

    (void @near (*)())0x8200,
    NonHandledInterrupt,   /* irq29 - Reserved */

  };

@near @interrupt void NonHandledInterrupt(void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  return;
}
