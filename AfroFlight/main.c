#include "main.h"
#include <math.h>

/* global vars visible elsewhere */
_Config Config;                                                      // Main settings struct
s16 Motors[MAX_MOTORS] = { 0, };                                     // Global motors struct for mixer.c

/* local static vars */
static u8 Armed = FALSE;                                           // Motors armed or not :)
static u16 Loop = 0;                                                 // Loop counter used for slow timing
static vu16 Rtc2ms = 0;                                              // Semi-accurate 1sec resolution timer used for flight hours and running tasks that shouldn't have to happen too often.
static u16 Rtc1sec = 0;                                              // 1 second RTC
static u8 FlightMode = FC_MODE_ACRO;                                 // current flight mode as defined by "Switch"
static u8 OldFlightMode = FC_MODE_ACRO;                              // previous value of flight mode
static u8 FCFlags = 0;                                               // FC flags, such as low voltage etc [ In progress ]
static s16 RxInRoll;					             // Roll setpoint
static s16 RxInPitch;                                                // Pitch setpoint
static s16 RxInThrottle;                                             // Throttle setpoint
static s16 RxInYaw;                                                  // Yaw setpoint
static s32 integral[3] = { 0, };				     // PID integral term
static s32 last_error[3] = { 0, };			             // PID Last proportional error
// static s16 gyroADC[3] = { 0, 0, 0 };	                             // Holds Gyro ADCs, used for rate calculation
static s16 GainIn[3] = { 25, 25, 25 };			             // was 20 = GainInADC[3]/10

/* Main configuration struct, saved in eeprom */
static const _Config DefaultConfig = {
    { 2, 1, 0, 3, 4 },          // Default sticks for D8R [ Thr | Pitch | Roll | Yaw | Switch ]
    QUAD_COPTER,                // Type of motor mixer
    33,                         // VoltagePerCellMin
    GYRO_NORMAL,                // RollGyroDirection
    GYRO_NORMAL,                // PitchGyroDirection
    GYRO_NORMAL,                // YawGyroDirection

    2,                          // Divider for pitch/roll gyros
    2,                          // Divider for yaw gyro
    
    2,                          // StickP: Roll/Pitch stick P-term
    5,                          // StickD: Roll/Pitch stick D-term
    2,                          // YawStickP: Yaw-stick P-term

    3,                          // Divider for pitch/roll stick
    4,                          // Divider for yaw stick
    25,                         // Yaw P
    9                           // Yaw I
};

/* Index into the ADCValues[] array inside sensors.c */
static const u8 SensorIndex[] = { ADC_GYRO_ROLL, ADC_GYRO_PITCH, ADC_GYRO_YAW };

/* Fixed lookup table for TIM1/2 Pulse Width registers */
static const struct {
    u8 *addressH;
    u8 *addressL;
} TimerAddress[] = {
    { &(TIM1->CCR1H), &(TIM1->CCR1L) },
    { &(TIM1->CCR2H), &(TIM1->CCR2L) },
    { &(TIM1->CCR3H), &(TIM1->CCR3L) },
    { &(TIM1->CCR4H), &(TIM1->CCR4L) },
    { &(TIM2->CCR2H), &(TIM2->CCR2L) },
    { &(TIM2->CCR1H), &(TIM2->CCR1L) }
};

/* Sets PWM pulse width for a given Motor */
@inline void PWM_SetPulseWidth(u8 Motor, u16 Pulse)
{
    /* Set the Pulse value */
    *TimerAddress[Motor].addressH = (u8)(Pulse >> 8);
    *TimerAddress[Motor].addressL = (u8)(Pulse);
}

/* Limit and output contents of Motors[] array to hardware */
@inline void PWM_WriteMotors(void)
{
    u8 i;

    // set motor limits (0 -> 1000)
    for (i = 0; i < MAX_MOTORS; i++) {
        // Clamp final values 0..1000us
        Motors[i] = CLAMP(Motors[i], 0, 1000);
        // Our hardware PWM timer uses 0.5us precision, so to turn to PWM pulses, double it
        PWM_SetPulseWidth(i, PULSE_1MS + (Motors[i] << 1));
    }
}

/* 2ms RTC timer, to make 1s counter */
@near @interrupt void TIM4_UPD_OVF_IRQHandler(void)
{
    Rtc2ms++;
    // TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
    // Optimize away a call()
    TIM4->SR1 = (u8)(~TIM4_IT_UPDATE);
}

/* Hardware Initialization */
static void HW_Init(void)
{
    // Clocks configuration
    CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE, DISABLE, DISABLE);
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);
    CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, ENABLE);
    CLK_ClockSecuritySystemEnable();

    // Status LED, Buzzer GPIO
    GPIO_Init(GPIOD, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_FAST);    // LED
    GPIO_Init(GPIOF, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);    // Buzzer
    LED_OFF;
    BUZZ_OFF;

    // Motor PWM timers
    TIM1_DeInit();
    TIM1_TimeBaseInit(7, TIM1_COUNTERMODE_UP, PULSE_PERIOD, 0);
    TIM1_OC1Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTSTATE_DISABLE, PULSE_1MS, TIM1_OCPOLARITY_LOW, TIM1_OCPOLARITY_HIGH, TIM1_OCIDLESTATE_RESET, TIM1_OCIDLESTATE_RESET);
    TIM1_OC1PreloadConfig(ENABLE);
    TIM1_OC2Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTSTATE_DISABLE, PULSE_1MS, TIM1_OCPOLARITY_LOW, TIM1_OCPOLARITY_HIGH, TIM1_OCIDLESTATE_RESET, TIM1_OCIDLESTATE_RESET);
    TIM1_OC2PreloadConfig(ENABLE);
    TIM1_OC3Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTSTATE_DISABLE, PULSE_1MS, TIM1_OCPOLARITY_LOW, TIM1_OCPOLARITY_HIGH, TIM1_OCIDLESTATE_RESET, TIM1_OCIDLESTATE_RESET);
    TIM1_OC3PreloadConfig(ENABLE);
    TIM1_OC4Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, PULSE_1MS, TIM1_OCPOLARITY_LOW, TIM1_OCIDLESTATE_RESET);
    TIM1_OC4PreloadConfig(ENABLE);
    TIM1_ARRPreloadConfig(ENABLE);
    TIM1_CtrlPWMOutputs(ENABLE);
    TIM2_DeInit();
    TIM2_TimeBaseInit(TIM2_PRESCALER_8, PULSE_PERIOD);
    TIM2_OC1Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, PULSE_1MS, TIM2_OCPOLARITY_LOW);
    TIM2_OC1PreloadConfig(ENABLE);
    TIM2_OC2Init(TIM2_OCMODE_PWM2, TIM2_OUTPUTSTATE_ENABLE, PULSE_1MS, TIM2_OCPOLARITY_LOW);
    TIM2_OC2PreloadConfig(ENABLE);
    TIM2_ARRPreloadConfig(ENABLE);

    // system tick timer for buzzer etc
    TIM4_DeInit();
    TIM4_TimeBaseInit(TIM4_PRESCALER_128, 249); // 2ms timer for RTC-ish tasks
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);

    TIM1_Cmd(ENABLE);
    TIM2_Cmd(ENABLE);

    // I2C [ TODO ]
    I2C_DeInit();
    I2C_Init(400000, 0xA0, I2C_DUTYCYCLE_2, I2C_ACK_CURR, I2C_ADDMODE_7BIT, 16);
    I2C_Cmd(ENABLE);
}

/* 
 * Find Accelerometer "level" offset. This should be done while model is perfectly level.
 */
static void Acc_Calibration(void)
{
    // TODO
}


void loop(void)
{
    u8 i;
    s32 emax = 132000;
    s32 imax, derivative;
    s32 ppart;
    s32 ipart;
    s32 error;
    s16 RxInRollPrev, RxInPitchPrev, RxInYawPrev;

    // get RC stuff in. TODO use stick centering... or maybe allow center from GUI.
    RxInRoll = 1500 - RC_GetChannel(RC_ROLL);
    RxInRollPrev = RC_GetDiffChannel(RC_ROLL);
    RxInPitch = 1500 - RC_GetChannel(RC_PITCH);
    RxInPitchPrev = RC_GetDiffChannel(RC_PITCH);
    RxInYaw = 1500 - RC_GetChannel(RC_YAW);
    RxInYawPrev = RC_GetDiffChannel(RC_YAW);
    RxInThrottle = RC_GetChannel(RC_THROTTLE) - 1120;
    
    // prep gyro
    gyro[0] -= gyroZero[0];
    gyro[1] -= gyroZero[1];
    gyro[2] -= gyroZero[2];

    // Throttle is low, do stuff like arming etc
    if (RxInThrottle < 0 || !Armed) {
        u8 command = RC_GetCommand();

        if (command == RC_COMMAND_ARM) {
            Armed = TRUE;
            printf("Motors Armed!!\r\n");
        } else if (command == RC_COMMAND_DISARM) {
            Armed = FALSE;
            printf("Motors Disarmed!!\r\n");
        } else if (command == RC_COMMAND_GYROCAL) {
            Beep(3, 50, 50);
            Gyro_Calibrate();
        } else if (command == RC_COMMAND_ACCCAL) {
            Beep(2, 50, 50);
            Acc_Calibration();
        }

        // Clear out PID integrals
        integral[ROLL] = 0;
        integral[PITCH] = 0;
        integral[YAW] = 0;
    } else {
        // Flight Mode
        OldFlightMode = FlightMode;
        FlightMode = RC_GetSwitchMode();
    }

    // Rescale Throttle to 0..1000us
    RxInThrottle = (RxInThrottle * 10) >> 3;	// 0-800 -> 0-1000
    imax = RxInThrottle;
    if (imax < 0)
        imax = 0;
    // imax >>= 3;	/* 1000 -> 200 */

    // Limit throttle just in case
    if (RxInThrottle > MAX_THROTTLE)
        RxInThrottle = MAX_THROTTLE;
        
    // TODO STM8 has hardware division for 8 and 16bit values. There is no need to use shifts.
    // Rewrite this using proper division factors to allow finer grained control over gains.
    // Lose GainInADC stuff since there are no pots to twist.

    // ROLL
    RxInRoll = RxInRoll * Config.StickP + RxInRollPrev * Config.StickD; // RC_PRTY[CONTROL_PITCH] = RCChannel(CH_PITCH) * staticParams.StickP + RCDiff(CH_PITCH) * staticParams.StickD;
    // RxInRoll = (RxInRoll * GainIn[ROLL]) >> Config.RollPitchStickGain;
    gyro[ROLL] = (gyro[ROLL] * GainIn[ROLL]) >> Config.RollPitchGyroGain;
    if (Config.RollGyroDirection == GYRO_NORMAL)
        gyro[ROLL] = -gyro[ROLL];
    RxInRoll -= gyro[ROLL];

    // PITCH
    RxInPitch = RxInPitch * Config.StickP + RxInPitchPrev * Config.StickD;
    // RxInPitch = (RxInPitch * GainIn[PITCH]) >> Config.RollPitchStickGain;
    gyro[PITCH] = (gyro[PITCH] * GainIn[PITCH]) >> Config.RollPitchGyroGain;
    if (Config.PitchGyroDirection == GYRO_NORMAL)
            gyro[PITCH] = -gyro[PITCH];
    RxInPitch -= gyro[PITCH];

    // YAW
    error = RxInYaw - RxInYawPrev;
    ppart = (Config.YawStickP * error * abs(error)) >> 9;
    ppart += (Config.YawStickP * error) >> 2;
    RxInYaw = (s16)(ppart);
    // RxInYaw = (RxInYaw * GainIn[YAW]) >> Config.YawStickGain;
    gyro[YAW] = (gyro[YAW] * GainIn[YAW]) >> Config.YawGyroGain;
    if (Config.YawGyroDirection == GYRO_NORMAL)
            gyro[YAW] = -gyro[YAW];

#if 1
    // Yaw PID
    error = RxInYaw - gyro[YAW];
    ppart = error * Config.YawP;
    ppart = CLAMP(ppart, -emax, emax);

    integral[YAW] += error;
    ipart = integral[YAW] * Config.YawI;
    ipart = CLAMP(ipart, -imax, imax);

    derivative = (s16)(error - last_error[YAW]);
    last_error[YAW] = error;
    RxInYaw += (s16)(ppart >> 4) + (s16)(ipart >> 4) + (s16)(derivative >> 4);
#else
    RxInYaw -= gyro[YAW];
#endif

    // Multirotor mixing (mixer.c)
    Mixer(RxInThrottle, RxInRoll, RxInPitch, RxInYaw);

    // Kill off motors when not armed. Really.
    if (RxInThrottle < 0 || !Armed) {
        for (i = 0; i < MAX_MOTORS; i++)
            Motors[i] = 0;
    }

    // Output to ESCs
    PWM_WriteMotors();		// output ESC signal
}

void main(void)
{
    u8 i;
    u16 rtcOld;

    HW_Init();                  // Clocks, GPIO, PWM, I2C
    UART_Init();                // UART
    RC_Init();                  // PPM Input
    Sensors_Init();             // ADC + SPI Sensors
    
    memcpy(&Config, &DefaultConfig, sizeof(Config));

    // Enable interrupts to start stuff up
    enableInterrupts();
    
    // blink the led for lulz
    for (i = 0; i < 5; i++) {
        LED_TOGGLE;
        delay_ms(200);
    }

    // Check sticks
    RC_Calibrate(1);
    // Check gyro/motion
    Gyro_Calibrate();

    // blink the led for lulz again
    for (i = 0; i < 5; i++) {
        LED_TOGGLE;
        delay_ms(200);
    }

    LED_OFF;
    
    // 0.8 second final beep
    Beep(1, 500, 400);
    // Battery check (will beep number of cells)
    Voltage_Init();

    // RTC timer (2ms)
    TIM4_Cmd(ENABLE);

    // main loop
    while (1) {
        // LED_ON
        if (Rtc2ms > 499) {
            Rtc2ms = 0;
            Rtc1sec++;
        }
        rtcOld = Rtc2ms;
        Loop++;
 
        // Debugging only, to be removed
        #if 0
        if (Armed)
            LED_ON
        else
            LED_OFF
        #endif

        // This checks for "Commands" from RC module. Returns stuff like arm/disarm/calibrate/etc
        RC_Update();
        // Main flight loop, don't call it THAT often :)
        loop();
        // let's try: we "receive" signals often, which set flags. then transmit will happen less often. or something.
        UART_ReceiveTelemetry();

        // Low-priority loop activities
        if (Loop > 50) {
            // see if we wanna process anything
            UART_TransmitTelemetry();
            if (Voltage_Check()) {
                FLAG_SET(FCFlags, FC_FLAG_LOWVOLTAGE);
                BUZZ_ON;
            } else {
                FLAG_CLEAR(FCFlags, FC_FLAG_LOWVOLTAGE);
                BUZZ_OFF;
            }
            Loop = 0;
        }

        // artificial delay
        LED_OFF
        // update + average sensors
        Sensors_Read();
        while (rtcOld == Rtc2ms);
        LED_ON
    }
}

// Blocking Beep (not ISR-based)
void Beep(u8 Count, u16 Length, u16 Delay)
{
    u8 i;

    for (i = 0; i < Count; i++) {
        BUZZ_ON;
        delay_ms(Length);
        BUZZ_OFF;
        delay_ms(Delay);
    }
}