#include "main.h"

/* PPM */
static u8 GoodPulses = 0;
static u8 PulseIndex;
static u16 PreviousValue;
static u16 CurrentValue;
static u16 CapturedValue;
static s16 CaptureValue[PPM_NUM_INPUTS];
static s16 PrevCaptureValue[PPM_NUM_INPUTS];

#define MAPPED_CHANNEL(Index) (CaptureValue[Config.ChannelMapping[Index]])
#define DIFF_CHANNEL(Index)   (PrevCaptureValue[Config.ChannelMapping[Index]])

/* RC Commands */
#define COMMAND_TIMER           (1500)
static u8 LastRCCommand = RC_COMMAND_NONE;
static u16 CommandTimer = 0;

@near @interrupt void TIM3_CAP_COM_IRQHandler(void)
{
    if (TIM3_GetITStatus(TIM3_IT_CC1) == SET) {
        PreviousValue = CurrentValue;
        CurrentValue = TIM3_GetCapture1();
    }

    TIM3_ClearITPendingBit(TIM3_IT_CC1);
    
    if (CurrentValue > PreviousValue) {
        CapturedValue = (CurrentValue - PreviousValue);
    } else {
        CapturedValue = ((0xFFFF - PreviousValue) + CurrentValue);
    }

    if (CapturedValue > 8000) {
        PulseIndex = 0;
    } else if (CapturedValue > 1500 && CapturedValue < 4500) { // div2, 750 to 2250 ms
        if (PulseIndex < PPM_NUM_INPUTS) {
            s16 tmp = CapturedValue >> 1;
            // save channel difference for expo calculation and trash lower few bits to reduce noise
            PrevCaptureValue[PulseIndex] = ((tmp - CaptureValue[PulseIndex]) / 3) * 3;
            CaptureValue[PulseIndex] = tmp;
            PulseIndex++;
            if (GoodPulses < 200)
                GoodPulses += 10;
        }
    }
}

void RC_Init(void)
{
    u8 i;
    PulseIndex = 0;
    PreviousValue = 0;
    CurrentValue = 0;
    CapturedValue = 0;
    GoodPulses = 0;
    
    for (i = 0; i < PPM_NUM_INPUTS; i++) {
        CaptureValue[i] = 0;
        PrevCaptureValue[i] = 0;
    }
        
    // Configure GPIO pin for ppm input
    GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);
    
    TIM3_TimeBaseInit(TIM3_PRESCALER_8, 0xFFFF);
    TIM3_ICInit(TIM3_CHANNEL_1, TIM3_ICPOLARITY_RISING, TIM3_ICSELECTION_DIRECTTI, TIM3_ICPSC_DIV1, 0x0);
    TIM3_ITConfig(TIM3_IT_CC1, ENABLE);
    TIM3_Cmd(ENABLE);
}

s16 RC_GetChannel(u8 Channel)
{
    return MAPPED_CHANNEL(Channel);
}

s16 RC_GetDiffChannel(u8 Channel)
{
    return DIFF_CHANNEL(Channel);
}

#define HYSTERESIS 32

s16 RC_Hysteresis(s16 Value)
{
    if (Value > HYSTERESIS || Value < HYSTERESIS)
        return Value;
    else
        return 0;
}

void RC_Calibrate(u8 MotorsEnable)
{
    // counts the amount of correct rx signals
    vu16 SignalGood = 0;
    
    // if motors are enabled from GUI, it is important to have stick in correct position
    if (MotorsEnable == 1) {
	do {
	    // switch and throttle stick must be zero in order to proceed
            vu16 sw, th;
            sw = RC_GetChannel(RC_SWITCH);
            th = RC_GetChannel(RC_THROTTLE);
	    if (GoodPulses < 100 || sw > 1200 || th > 1200) {
		// no RC signal or sticks not in correct position
		if (SignalGood > 0)
		    SignalGood--;
	    } else {
		SignalGood++;
	    }

	    if (SignalGood == 0) {
		LED_TOGGLE;	// give signal to pilot
                // 75 ms delay
                Beep(3, 15, 10); // beep attention
	    }
	    delay_ms(1);
	} while (SignalGood < 500);

	// turn off led
	LED_OFF;
    }
}

static u8 _RCCommand(void)
{
    if (MAPPED_CHANNEL(RC_THROTTLE) > 1700) {
        if (MAPPED_CHANNEL(RC_YAW) > 1700)
            return RC_COMMAND_GYROCAL;
        if (MAPPED_CHANNEL(RC_YAW) < 1300)
            return RC_COMMAND_ACCCAL;
        return RC_COMMAND_NONE;
    } else if (MAPPED_CHANNEL(RC_THROTTLE) < 1120) {
        if (MAPPED_CHANNEL(RC_YAW) > 1700)
            return RC_COMMAND_DISARM;
        if (MAPPED_CHANNEL(RC_YAW) < 1300)
            return RC_COMMAND_ARM;
        return RC_COMMAND_NONE;
    }

    return RC_COMMAND_NONE;
}

u8 RC_GetSwitchMode(void)
{
    s16 pos = MAPPED_CHANNEL(RC_SWITCH);
    
    if (pos < 1300)
        return FC_MODE_ACRO;
    else if (pos > 1700)
        return FC_MODE_DONGS;

    return FC_MODE_HOVER;
}

void RC_Update(void)
{
    u8 command = _RCCommand();

    // decrement  signal level
    if (GoodPulses != 0)
        GoodPulses--;

    if (LastRCCommand == command) {
      // Keep timer from overrunning.
      if (CommandTimer < COMMAND_TIMER)
        CommandTimer++;
    } else {
      // There was a change.
      LastRCCommand = command;
      CommandTimer = 0;
    }
}

u8 RC_GetCommand(void)
{
    // Stick held long enough = return command. Otherwise, nothing.
    if (CommandTimer == COMMAND_TIMER) {
        // Reset command to prevent immediate double-firing
        CommandTimer = 0;
        return LastRCCommand;
    }
        
    return RC_COMMAND_NONE;
}

u8 RC_GetQuality(void)
{
    return GoodPulses;
}
