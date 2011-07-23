#include "main.h"
#include <math.h>

/* global vars visible elsewhere */
_Config Config;                                                      // Main settings struct
s16 Motors[MAX_MOTORS] = { 0, };                                     // Global motors struct for mixer.c

/* local static vars */
static u8 Armed = FALSE;                                             // Motors armed or not :)
static u16 Loop = 0;                                                 // Loop counter used for slow timing
static vu16 rtc200us = 0;                                            // 
static u16 Rtc1sec = 0;                                              // Semi-accurate 1sec resolution timer used for flight hours and running tasks that shouldn't have to happen too often.
static u8 FlightMode = FC_MODE_ACRO;                                 // current flight mode as defined by "Switch"
static u8 OldFlightMode = FC_MODE_ACRO;                              // previous value of flight mode
static u8 FCFlags = 0;                                               // FC flags, such as low voltage etc [ In progress ]
static s32 integral[3] = { 0, };				     // PID integral term
static s32 last_error[3] = { 0, };			             // PID Last proportional error

/* Main configuration struct, saved in eeprom */
static const _Config DefaultConfig = {
    { 2, 1, 0, 3, 4 },          // Default sticks for D8R [ Thr | Pitch | Roll | Yaw | Switch ]
    QUAD_COPTER,                // Type of motor mixer
    33,                         // VoltagePerCellMin
    GYRO_NORMAL,                // RollGyroDirection
    GYRO_NORMAL,                // PitchGyroDirection
    GYRO_NORMAL,                // YawGyroDirection

    25,                         // Gain for roll/pitch gyros
    25,                         // Gain for yaw gyro

    2,                          // Divider for pitch/roll gyros
    2,                          // Divider for yaw gyro

    2,                          // StickP: Roll/Pitch stick P-term
    5,                          // StickD: Roll/Pitch stick D-term
    8,                          // YawStickP: Yaw-stick P-term

    3,                          // Divider for pitch/roll stick
    4,                          // Divider for yaw stick
    25,                         // Yaw P
    9                           // Yaw I
};

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
__inline void PWM_SetPulseWidth(u8 Motor, u16 Pulse)
{
    /* Set the Pulse value */
    *TimerAddress[Motor].addressH = (u8)(Pulse >> 8);
    *TimerAddress[Motor].addressL = (u8)(Pulse);
}

/* Limit and output contents of Motors[] array to hardware */
__inline void PWM_WriteMotors(void)
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
__near __interrupt void TIM4_UPD_OVF_IRQHandler(void)
{
    rtc200us++;
    // Optimize away a call() - TIM4_ClearITPendingBit(TIM4_IT_UPDATE);
    TIM4->SR1 = (u8)(~TIM4_IT_UPDATE);
    // fire off gyro sensor reading (every 200us)...
    ADC1_StartConversion();
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
    TIM4_TimeBaseInit(TIM4_PRESCALER_128, 24); // 200us timer for RTC-ish tasks
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);

    TIM1_Cmd(ENABLE);
    TIM2_Cmd(ENABLE);
    // TIM3 is used by PPM input
    // RTC timer (2ms)
    TIM4_Cmd(ENABLE);

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

/* PID loop */
void pidloop(void)
{
    u8 i;
    s16 lgyro[3]; // loop gyro
    s16 Roll;					          // Roll setpoint
    s16 Pitch;                                            // Pitch setpoint
    s16 Throttle;                                         // Throttle setpoint
    s16 Yaw;                                              // Yaw setpoint

    float setpoint;
    float error;
    static float error_d[3][3] = { 0, };
    static float error_old[3][3] = { 0, };
    static float error_sum[3] = { 0, };
    static u8 index;
    float p_set;
    float i_set;
    float d_set;

    // get RC stuff in. ControlChannels[] is updated every 50hz.
    Roll = ControlChannels[RC_ROLL];
    Pitch = ControlChannels[RC_PITCH];
    Yaw = ControlChannels[RC_YAW];
    Throttle = ControlChannels[RC_THROTTLE];

    // prep gyro
    lgyro[0] = gyro[0] - gyroZero[0];
    lgyro[1] = gyro[1] - gyroZero[1];
    lgyro[2] = gyro[2] - gyroZero[2];

    // Throttle is low, do stuff like arming etc
    if (Throttle < 0 || !Armed) {
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
        error_sum[ROLL] = 0;
        error_sum[PITCH] = 0;
        error_sum[YAW] = 0;
    } else {
        // Flight Mode
        OldFlightMode = FlightMode;
        FlightMode = RC_GetSwitchMode();
    }

    // Rescale Throttle to 0..1000us
    Throttle = (Throttle * 10) >> 3;	// 0-800 -> 0-1000
    // Limit throttle just in case
    if (Throttle > MAX_THROTTLE)
        Throttle = MAX_THROTTLE;

    // D history
    index++;
    index = index % 2;

    // ROLL
    if (Config.RollGyroDirection == GYRO_NORMAL)
        lgyro[ROLL] = -lgyro[ROLL];

    setpoint = Roll * 4.0f; // roll stick position
    error = (lgyro[ROLL] * 25.0f) - setpoint; // calculate difference between angular velocity and stick position
    // this calculates the angular velocity
    error_d[ROLL][index] = error - error_old[ROLL][index];
    error_old[ROLL][index] = error;
    d_set = error_d[ROLL][index] * 0.3f; // Kd;
    error_sum[ROLL] += error; // integrate the above
    error_sum[ROLL] = CLAMP(error_sum[ROLL], -10000, 10000);
    p_set = error * 0.5f; // Kp
    i_set = error * 0.0012f; // Ki
    Roll = -(p_set + i_set + d_set);

    // PITCH
    if (Config.PitchGyroDirection == GYRO_NORMAL)
        lgyro[PITCH] = -lgyro[PITCH];

    setpoint = Pitch * 4.0f; // pitch stick position
    error = (lgyro[PITCH] * 25.0f) - setpoint; // calculate difference between angular velocity and stick position
    // this calculates the angular velocity
    error_d[PITCH][index] = error - error_old[PITCH][index];
    error_old[PITCH][index] = error;
    d_set = error_d[PITCH][index] * 0.3f; // Kd;
    error_sum[PITCH] += error; // integrate the above
    error_sum[PITCH] = CLAMP(error_sum[PITCH], -10000, 10000);
    p_set = error * 0.5f; // Kp
    i_set = error * 0.0012f; // Ki
    Pitch = -(p_set + i_set + d_set);

    // YAW
    if (Config.YawGyroDirection == GYRO_NORMAL)
            lgyro[YAW] = -lgyro[YAW];

    setpoint = Yaw * 10.0f; // pitch stick position
    error = (lgyro[YAW] * 25.0f) - setpoint; // calculate difference between angular velocity and stick position
    // this calculates the angular velocity
    error_d[YAW][index] = error - error_old[YAW][index];
    error_old[YAW][index] = error;
    d_set = error_d[YAW][index] * 0.3f; // Kd;
    error_sum[YAW] += error; // integrate the above
    error_sum[YAW] = CLAMP(error_sum[YAW], -32000, 32000);
    p_set = error * 0.4f; // Kp
    i_set = error * 0.003f; // Ki
    Yaw = -(p_set + i_set + d_set);

    // Multirotor mixing (mixer.c)
    Mixer(Throttle, Roll, Pitch, Yaw);

    // Kill off motors when not armed. Really.
    if (Throttle < 0 || !Armed) {
        for (i = 0; i < MAX_MOTORS; i++)
            Motors[i] = 0;
    }

    // Output to ESCs
    PWM_WriteMotors();		// output ESC signal
}

/* Non-pid rate loop */
void loop(void)
{
    u8 i;
    s32 emax = 132000;
    s32 imax, derivative;
    s32 ppart;
    s32 ipart;
    s32 error;
    s16 lgyro[3]; // loop gyro

    s16 Roll;					          // Roll setpoint
    s16 Pitch;                                            // Pitch setpoint
    s16 Throttle;                                         // Throttle setpoint
    s16 Yaw;                                              // Yaw setpoint

    // get RC stuff in. ControlChannels[] is updated every 50hz.
    Roll = ControlChannels[RC_ROLL];
    Pitch = ControlChannels[RC_PITCH];
    Yaw = ControlChannels[RC_YAW];
    Throttle = ControlChannels[RC_THROTTLE];

    // prep gyro
    lgyro[0] = gyroZero[0] - gyro[0];
    lgyro[1] = gyroZero[1] - gyro[1];
    lgyro[2] = gyroZero[2] - gyro[2];

    // Throttle is low, do stuff like arming etc
    if (Throttle < 0 || !Armed) {
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
    Throttle = (Throttle * 10) >> 3;	// 0-800 -> 0-1000
    imax = Throttle;
    if (imax < 0)
        imax = 0;

    // Limit throttle just in case
    if (Throttle > MAX_THROTTLE)
        Throttle = MAX_THROTTLE;
        
    // TODO STM8 has hardware division for 8 and 16bit values. There is no need to use shifts.
    // Rewrite this using proper division factors to allow finer grained control over gains.
    // Lose GainInADC stuff since there are no pots to twist.

    // ROLL
    lgyro[ROLL] = (lgyro[ROLL] * Config.RollPitchGyroGain) >> Config.RollPitchGyroDiv;
    if (Config.RollGyroDirection == GYRO_REVERSED)
        lgyro[ROLL] = -lgyro[ROLL];
    Roll -= lgyro[ROLL];

    // PITCH
    lgyro[PITCH] = (lgyro[PITCH] * Config.RollPitchGyroGain) >> Config.RollPitchGyroDiv;
    if (Config.PitchGyroDirection == GYRO_REVERSED)
            lgyro[PITCH] = -lgyro[PITCH];
    Pitch -= lgyro[PITCH];

    // YAW
    lgyro[YAW] = (lgyro[YAW] * Config.YawGyroGain) >> Config.YawGyroDiv;
    if (Config.YawGyroDirection == GYRO_REVERSED)
            lgyro[YAW] = -lgyro[YAW];

#if 1
    // Yaw PID
    error = Yaw - lgyro[YAW];
    ppart = error * Config.YawP;
    ppart = CLAMP(ppart, -emax, emax);

    integral[YAW] += error;
    ipart = integral[YAW] * Config.YawI;
    ipart = CLAMP(ipart, -imax, imax);

    derivative = (s16)(error - last_error[YAW]);
    last_error[YAW] = error;
    Yaw += (s16)(ppart >> 4) + (s16)(ipart >> 4) + (s16)(derivative >> 4);
#else
    // Yaw normal
    Yaw -= lgyro[YAW];
#endif

    // Multirotor mixing (mixer.c)
    Mixer(Throttle, Roll, Pitch, Yaw);

    // Kill off motors when not armed. Really.
    if (Throttle < 0 || !Armed) {
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
    
    // TODO eeprom
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

    // main loop
    while (1) {
        if (rtc200us > 4999) {
            rtc200us = 0;
            Rtc1sec++;
        }
        rtcOld = rtc200us;
        Loop++;
 
        // This checks for "Commands" from RC module. Returns stuff like arm/disarm/calibrate/etc
        RC_Update();

        // Main flight loop, don't call it THAT often :)
#if 0
        pidloop();
#else
        loop();
#endif

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
        // update + average sensors
        Sensors_ReadACC();
        Sensors_ReadADC();
        while (rtc200us % 10 != 0); // idle until loop reaches 500Hz
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