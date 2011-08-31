/**
  ******************************************************************************
  * @file I2C_IAP\stm8s_it.h
  * @brief This file contains the external declarations of the interrupt routines.
  * @author STMicroelectronics - MCD Application Team
  * @version V2.0.0
  * @date 02/04/2009
  ******************************************************************************
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2008 STMicroelectronics</center></h2>
  * @image html logo.bmp
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM8S_IT_H
#define __STM8S_IT_H

/* Includes ------------------------------------------------------------------*/
// #include "stm8s_lib.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

extern void @near NonHandledInterrupt(void);
extern void @near EEPROM_EEC_IRQHandler(void); /* EEPROM ECC CORRECTION */
extern void @near TIM4_UPD_OVF_IRQHandler(void); /* TIM4 UPD/OVF */
extern void @near ADC1_IRQHandler(void); /* ADC1 */
extern void @near ADC2_IRQHandler(void); /* ADC2 */
extern void @near UART3_RX_IRQHandler(void); /* UART3 RX */
extern void @near UART3_TX_IRQHandler(void); /* UART3 TX */
extern void @near UART2_RX_IRQHandler(void); /* UART2 RX */
extern void @near UART2_TX_IRQHandler(void); /* UART2 TX */
extern void @near I2C_IRQHandler(void); /* I2C */
extern void @near UART1_RX_IRQHandler(void); /* UART1 RX */
extern void @near UART1_TX_IRQHandler(void); /* UART1 TX */
extern void @near TIM3_CAP_COM_IRQHandler(void); /* TIM3 CAP/COM */
extern void @near TIM3_UPD_OVF_BRK_IRQHandler(void); /* TIM3 UPD/OVF/BRK */
extern void @near TIM2_CAP_COM_IRQHandler(void); /* TIM2 CAP/COM */
extern void @near TIM2_UPD_OVF_BRK_IRQHandler(void); /* TIM2 UPD/OVF/BRK */
extern void @near TIM1_CAP_COM_IRQHandler(void); /* TIM1 CAP/COM */
extern void @near TIM1_UPD_OVF_TRG_BRK_IRQHandler(void); /* TIM1 UPD/OVF/TRG/BRK */
extern void @near SPI_IRQHandler(void); /* SPI */
extern void @near CAN_TX_IRQHandler(void); /* CAN TX/SCE */
extern void @near CAN_RX_IRQHandler(void); /* CAN RX */
extern void @near EXTI_PORTE_IRQHandler(void); /* EXTI PORTE */
extern void @near EXTI_PORTD_IRQHandler(void); /* EXTI PORTD */
extern void @near EXTI_PORTC_IRQHandler(void); /* EXTI PORTC */
extern void @near EXTI_PORTB_IRQHandler(void); /* EXTI PORTB */
extern void @near EXTI_PORTA_IRQHandler(void); /* EXTI PORTA */
extern void @near CLK_IRQHandler(void); /* CLOCK */
extern void @near AWU_IRQHandler(void); /* AWU */
extern void @near TLI_IRQHandler(void); /* TLI */
extern void @near TRAP_IRQHandler(void); /* TRAP */
extern void @near _stext(void); /* RESET startup routine */

#endif /* __STM8S_IT_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
