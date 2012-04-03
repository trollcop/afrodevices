#include "board.h"
#include "def.h"
#include "config.h"
#include "sysdep.h"
#include "mw.h"

/* RX -------------------------------------------------------------------------------- */
static uint8_t rcChannel[8] = { ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,CAMPITCH,CAMROLL };
volatile uint16_t rcValue[8] = { 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };      // interval [1000;2000]

/* for single channel PWM input mode */
#ifdef SERIAL_SUM_PPM
static uint16_t riseValue = 0;
static uint16_t fallValue = 0;
static uint8_t captureState = 0;
#endif
static uint8_t usePPM = 1;

// ROME brushed, PPM input
static void configureRomePPM(void)
{
    // Configure GPIO pin for ppm input
    GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);

    TIM2_TimeBaseInit(TIM2_PRESCALER_8, 0xFFFF);
    TIM2_ICInit(TIM2_CHANNEL_1, TIM2_ICPOLARITY_RISING, TIM2_ICSELECTION_DIRECTTI, TIM2_ICPSC_DIV1, 0x0);
    TIM2_ITConfig(TIM2_IT_CC1, ENABLE);
    TIM2_Cmd(ENABLE);
}

// ROME brushed, PWM input
static void configureRomePWM(void)
{
    // Configure GPIO pins for pwm input
    GPIO_Init(GPIOD, GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_0, GPIO_MODE_IN_FL_NO_IT);

    // RC 1, 2
    TIM2_TimeBaseInit(TIM2_PRESCALER_8, 0xFFFF);
    TIM2_ICInit(TIM2_CHANNEL_1, TIM2_ICPOLARITY_RISING, TIM2_ICSELECTION_DIRECTTI, TIM2_ICPSC_DIV1, 0x0);
    TIM2_ICInit(TIM2_CHANNEL_2, TIM2_ICPOLARITY_RISING, TIM2_ICSELECTION_DIRECTTI, TIM2_ICPSC_DIV1, 0x0);
    TIM2_ITConfig(TIM2_IT_CC1 | TIM2_IT_CC2, ENABLE);
    TIM2_Cmd(ENABLE);

    // RC 3, 4
    TIM3_TimeBaseInit(TIM3_PRESCALER_8, 0xFFFF);
    TIM3_ICInit(TIM3_CHANNEL_1, TIM3_ICPOLARITY_RISING, TIM3_ICSELECTION_DIRECTTI, TIM3_ICPSC_DIV1, 0x0);
    TIM3_ICInit(TIM3_CHANNEL_2, TIM3_ICPOLARITY_RISING, TIM3_ICSELECTION_DIRECTTI, TIM3_ICPSC_DIV1, 0x0);
    TIM3_ITConfig(TIM3_IT_CC1 | TIM3_IT_CC2, ENABLE);
    TIM3_Cmd(ENABLE);
}

// Afro* boards, PPM input
static void configureAfroPPM(void)
{
    // Configure GPIO pin for ppm input
    GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);
    
    TIM3_TimeBaseInit(TIM3_PRESCALER_8, 0xFFFF);
    TIM3_ICInit(TIM3_CHANNEL_1, TIM3_ICPOLARITY_RISING, TIM3_ICSELECTION_DIRECTTI, TIM3_ICPSC_DIV1, 0x0);
    TIM3_ITConfig(TIM3_IT_CC1, ENABLE);
    TIM3_Cmd(ENABLE);
}

// Configure receiver pins
void configureReceiver(void)
{
#if defined(ROME) || defined(ROME_BRUSHED)
# ifdef SERIAL_SUM_PPM
    configureRomePPM();
# else
    configureRomePWM();
# endif

#else /* !ROME */
    configureAfroPPM();
#endif

    // when we're in camera gimbal mode, allow single PWM rx input for tilt compensation. useful? no idea.
    if ((mixerConfiguration == MULTITYPE_GIMBAL) && (gimbalFlags & GIMBAL_TILTONLY))
        usePPM = 0;
}

#ifdef SERIAL_SUM_PPM
#if (defined(ROME) || defined(ROME_BRUSHED))
 #define TIM_Channel             TIM2_CHANNEL_1
 #define TIM_GetITStatus         TIM2_GetITStatus
 #define TIM_CC_Channel          TIM2_IT_CC1
 #define TIM_GetCapture          TIM2_GetCapture1
 #define TIM_ClearITPendingBit   TIM2_ClearITPendingBit
 #define TIM_ICInit              TIM2_ICInit
 #define TIM_ICPOLARITY_FALLING  TIM2_ICPOLARITY_FALLING
 #define TIM_ICPOLARITY_RISING   TIM2_ICPOLARITY_RISING
 #define TIM_ICSELECTION_DIRECTTI        TIM2_ICSELECTION_DIRECTTI
 #define TIM_ICPSC_DIV1          TIM2_ICPSC_DIV1
 #define TIM_CAP_COM_IRQHandler  TIM2_CAP_COM_IRQHandler
#else /* various afroboards with PPM on TIM3 */
 #define TIM_Channel             TIM3_CHANNEL_1
 #define TIM_GetITStatus         TIM3_GetITStatus
 #define TIM_CC_Channel          TIM3_IT_CC1
 #define TIM_GetCapture          TIM3_GetCapture1
 #define TIM_ClearITPendingBit   TIM3_ClearITPendingBit
 #define TIM_ICInit              TIM3_ICInit
 #define TIM_ICPOLARITY_FALLING  TIM3_ICPOLARITY_FALLING
 #define TIM_ICPOLARITY_RISING   TIM3_ICPOLARITY_RISING
 #define TIM_ICSELECTION_DIRECTTI        TIM3_ICSELECTION_DIRECTTI
 #define TIM_ICPSC_DIV1          TIM3_ICPSC_DIV1
 #define TIM_CAP_COM_IRQHandler  TIM3_CAP_COM_IRQHandler
#endif

#if defined(ROME) || defined(ROME_BRUSHED)
__near __interrupt void TIM3_CAP_COM_IRQHandler(void)
{
}

__near __interrupt void TIM2_CAP_COM_IRQHandler(void)
#else
__near __interrupt void TIM2_CAP_COM_IRQHandler(void)
{
}

__near __interrupt void TIM3_CAP_COM_IRQHandler(void)
#endif
{
    uint16_t diff;
    static uint16_t now;
    static uint16_t last = 0;
    static uint8_t chan = 0;
    
    if (TIM_GetITStatus(TIM_CC_Channel) == SET) {
        last = now;
        now = TIM_GetCapture();
    }

    TIM_ClearITPendingBit(TIM_CC_Channel);

    if (!usePPM) {
        // single-channel PWM input
        if (captureState == 0)
            riseValue = now;
        else
            fallValue = now;

        if (captureState == 0) {
            // switch states
            captureState = 1;
            TIM_ICInit(TIM_Channel, TIM_ICPOLARITY_FALLING, TIM_ICSELECTION_DIRECTTI, TIM_ICPSC_DIV1, 0x0);
        } else {
            // capture compute
            if (fallValue > riseValue)
                rcValue[PITCH] = fallValue - riseValue;
            else
                rcValue[PITCH] = (0xffff - riseValue) + fallValue;

            // 0.5us resolution, so we halve it for real stuff. And it ends up in the PITCH channel (camera tilt use)
            rcValue[PITCH] >>= 1;

            // switch state
            captureState = 0;
            TIM_ICInit(TIM_Channel, TIM_ICPOLARITY_RISING, TIM_ICSELECTION_DIRECTTI, TIM_ICPSC_DIV1, 0x0);
        }
        return;
    }

    if (now > last) {
        diff = (now - last);
    } else {
        diff = ((0xFFFF - last) + now);
    }

    if (diff > 8000) {
        chan = 0;
    } else {
        if (diff > 1500 && diff < 4500 && chan < 8) {   // div2, 750 to 2250 ms Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
            uint16_t tmp = diff >> 1;
            rcValue[chan] = tmp;

#if defined(FAILSAFE)
            if (failsafeCnt > 20)
                failsafeCnt -= 20;
            else
                failsafeCnt = 0;        // clear FailSafe counter - added by MIS  //incompatible to quadroppm
#endif
        }
        chan++;
    }
}

#endif

#if (defined(ROME) || defined(ROME_BRUSHED)) && !defined(SERIAL_SUM_PPM)
uint16_t last[4] = { 0, };
uint16_t now[4] = { 0, };
uint16_t riseValue[4] = { 0, };
uint16_t fallValue[4] = { 0, };
uint8_t captureState[4] = { 0, };

// RC in 1/2
__near __interrupt void TIM2_CAP_COM_IRQHandler(void)
{
    uint8_t chan = 0;

    if (TIM2_GetITStatus(TIM2_IT_CC1) == SET) {
        last[0] = now[0];
        now[0] = TIM2_GetCapture1();
        TIM2_ClearITPendingBit(TIM2_IT_CC1);
        chan = 0;
    } else if (TIM2_GetITStatus(TIM2_IT_CC2) == SET) {
        last[1] = now[1];
        now[1] = TIM2_GetCapture2();
        TIM2_ClearITPendingBit(TIM2_IT_CC2);
        chan = 1;
    }

    // single-channel PWM input
    if (captureState[chan] == 0)
        riseValue[chan] = now[chan];
    else
        fallValue[chan] = now[chan];

    if (captureState[chan] == 0) {
        // switch states
        captureState[chan] = 1;
        if (chan == 0)
            TIM2_ICInit(TIM2_CHANNEL_1, TIM2_ICPOLARITY_FALLING, TIM2_ICSELECTION_DIRECTTI, TIM2_ICPSC_DIV1, 0x0);
        else if (chan == 1)
            TIM2_ICInit(TIM2_CHANNEL_2, TIM2_ICPOLARITY_FALLING, TIM2_ICSELECTION_DIRECTTI, TIM2_ICPSC_DIV1, 0x0);
    } else {
        // capture compute
        if (fallValue[chan] > riseValue[chan])
            rcValue[chan] = fallValue[chan] - riseValue[chan];
        else
            rcValue[chan] = (0xffff - riseValue[chan]) + fallValue[chan];
        
        // 0.5us resolution, so we halve it for real stuff. And it ends up in the PITCH channel (camera tilt use)
        rcValue[chan] >>= 1;
        
#if defined(FAILSAFE)
            if (failsafeCnt > 20)
                failsafeCnt -= 20;
            else
                failsafeCnt = 0;        // clear FailSafe counter - added by MIS  //incompatible to quadroppm
#endif


        // switch state
        captureState[chan] = 0;
        if (chan == 0)
            TIM2_ICInit(TIM2_CHANNEL_1, TIM2_ICPOLARITY_RISING, TIM2_ICSELECTION_DIRECTTI, TIM2_ICPSC_DIV1, 0x0);
        else if (chan == 1)
            TIM2_ICInit(TIM2_CHANNEL_2, TIM2_ICPOLARITY_RISING, TIM2_ICSELECTION_DIRECTTI, TIM2_ICPSC_DIV1, 0x0);
    }
}

// RC in 3/4
__near __interrupt void TIM3_CAP_COM_IRQHandler(void)
{
    uint8_t chan = 0;

    if (TIM3_GetITStatus(TIM3_IT_CC1) == SET) {
        last[2] = now[2];
        now[2] = TIM3_GetCapture1();
        TIM3_ClearITPendingBit(TIM3_IT_CC1);
        chan = 2;
    } else if (TIM3_GetITStatus(TIM3_IT_CC2) == SET) {
        last[3] = now[3];
        now[3] = TIM3_GetCapture2();
        TIM3_ClearITPendingBit(TIM3_IT_CC2);
        chan = 3;
    }

    // single-channel PWM input
    if (captureState[chan] == 0)
        riseValue[chan] = now[chan];
    else
        fallValue[chan] = now[chan];

    if (captureState[chan] == 0) {
        // switch states
        captureState[chan] = 1;
        if (chan == 2)
            TIM3_ICInit(TIM3_CHANNEL_1, TIM3_ICPOLARITY_FALLING, TIM3_ICSELECTION_DIRECTTI, TIM3_ICPSC_DIV1, 0x0);
        else if (chan == 3)
            TIM3_ICInit(TIM3_CHANNEL_2, TIM3_ICPOLARITY_FALLING, TIM3_ICSELECTION_DIRECTTI, TIM3_ICPSC_DIV1, 0x0);
    } else {
        // capture compute
        if (fallValue[chan] > riseValue[chan])
            rcValue[chan] = fallValue[chan] - riseValue[chan];
        else
            rcValue[chan] = (0xffff - riseValue[chan]) + fallValue[chan];
        
        // 0.5us resolution, so we halve it for real stuff. And it ends up in the PITCH channel (camera tilt use)
        rcValue[chan] >>= 1;

#if defined(FAILSAFE)
            if (failsafeCnt > 20)
                failsafeCnt -= 20;
            else
                failsafeCnt = 0;        // clear FailSafe counter - added by MIS  //incompatible to quadroppm
#endif

        // switch state
        captureState[chan] = 0;
        if (chan == 2)
            TIM3_ICInit(TIM3_CHANNEL_1, TIM3_ICPOLARITY_RISING, TIM3_ICSELECTION_DIRECTTI, TIM3_ICPSC_DIV1, 0x0);
        else if (chan == 3)
            TIM3_ICInit(TIM3_CHANNEL_2, TIM3_ICPOLARITY_RISING, TIM3_ICSELECTION_DIRECTTI, TIM3_ICPSC_DIV1, 0x0);
    }
}

#endif /* ROME || ROME_BRUSHED */

uint16_t readRawRC(uint8_t chan)
{
    return rcValue[rcChannel[chan]];
}

void computeRC(void)
{
    static int16_t rcData4Values[8][4], rcDataMean[8];
    static uint8_t rc4ValuesIndex = 0;
    uint8_t chan, a;

    rc4ValuesIndex++;
    for (chan = 0; chan < 8; chan++) {
        rcData4Values[chan][rc4ValuesIndex % 4] = readRawRC(chan);
        rcDataMean[chan] = 0;
        for (a = 0; a < 4; a++)
            rcDataMean[chan] += rcData4Values[chan][a];
        rcDataMean[chan] = (rcDataMean[chan] + 2) / 4;
        if (rcDataMean[chan] < rcData[chan] - 3)
            rcData[chan] = rcDataMean[chan] + 2;
        if (rcDataMean[chan] > rcData[chan] + 3)
            rcData[chan] = rcDataMean[chan] - 2;
    }
}
