// #define SPEKTRUM

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include "uart.h"
#include "spektrum.h"
#include "i2c.h"

typedef uint8_t u8;
typedef int8_t s8;
typedef uint16_t u16;
typedef int16_t s16;
typedef uint32_t u32;

enum {
    FC_STATUS_RCINVALID = 0x01,
    FC_STATUS_LOWVOLTAGE = 0x02
};

enum {
    RC_MODE_PPM = 0,
    RC_MODE_SPEKTRUM = 1,
};

#define ADC_GYRO_YAW	(0)
#define ADC_GYRO_ROLL	(1)
#define ADC_GYRO_PITCH	(2)
#define ADC_PRESSURE	(3)
#define ADC_VOLTAGE	(4)
#define ADC_ACC_Z	(5)
#define ADC_ACC_ROLL	(6)
#define ADC_ACC_PITCH	(7)

#define PARAM_XACC_OFFSET   22
#define PARAM_YACC_OFFSET   23
#define PARAM_LED1_MASK	    31
#define PARAM_LED2_MASK	    32

#define MAX_LIPO_CELL_VOLTAGE (43)
#define USART_BAUD 115200ul
#define USART_UBBR_VALUE ((F_CPU / (USART_BAUD << 4)) - 1)
#define TIMER0_RELOAD_VALUE  250

#define RED_OFF   PORTB &= ~(_BV(PORTB0));
#define RED_ON    PORTB |= (_BV(PORTB0));
#define RED_FLASH PORTB ^= (_BV(PORTB0));
#define GRN_OFF   PORTB |= (_BV(PORTB1));
#define GRN_ON    PORTB &= ~(_BV(PORTB1));
#define GRN_FLASH PORTB ^= (_BV(PORTB1));

#define BUZZ_ON	  PORTC |= (_BV(PORTC7));	// Speaker an PORTC.7
#define BUZZ_OFF  PORTC &= ~(_BV(PORTC7));
#define BUZZ_FLASH  PORTC ^= (_BV(PORTC7));

#define LED1_ON          PORTC |=  (_BV(PORTC2));
#define LED1_OFF         PORTC &= ~(_BV(PORTC2));
#define LED1_TOGGLE      PORTC ^=  (_BV(PORTC2));
#define LED2_ON          PORTC |=  (_BV(PORTC3));
#define LED2_OFF         PORTC &= ~(_BV(PORTC3));
#define LED2_TOGGLE      PORTC ^=  (_BV(PORTC3));

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
// #define LIMIT_MIN_MAX(value, min, max) { if (value <= min) value = min; else if (value >= max) value = max; }

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

static float Lf;		// stick sensitivity
static float Lfdynamic_roll;	// dynamic stick sensitvity (used for flying loopings)
static float Lfdynamic_pitch;	// dynamic stick sensitvity (used for flying loopings)
static u8 Motors_on = 0;	// self explanatory, right...?!
static u8 State = 0;		// Contains selected control loop state: 0 = Motors off, 1 = Acrobatic mode, 2 = Hover mode
static u8 Old_state;		// Contains the state of the last cycle

static const u8 DefaultSettings[] = {
    1,				// [0] MotorsEnable
    1,				// [1] Roll gyro dir
    0,				// [2] Pitch gyro dir
    1,				// [3] Yaw gyro dir (1 for Inv, 0 for CRM)
    0,				// [4] Xacc dir (pitch)
    1,				// [5] YAcc dir (roll)

    130,			// [6] P_sens_acro = 130
    30,				// [7] I_sens_acro = 30

    130,			// [8] P_sens_hover = 
    190,			// [9] I_sens_hover = 
    80,				// [10] D_sens_hover = 

    100,			// [11] Yaw_p_sens_eep = 
    70,				// [12] Yaw_i_sens_eep = 

    10,				// [13] Acc Influence
    140,			// [14] XAcc Scale
    140,			// [15] YAcc Scale

    130,			// [16] Lf_acro = 
    70,				// [17] Lf_hover = 
    130,			// [18] Lf_yaw = 
    0,				// [19] Lf_boost =

    18,				// [20] Idle_up = 
    33,				// [21] Voltage_Cell_Min = 

    128,			// [22] XAcc Offset (to be removed from PC settings)
    128,			// [23] YAcc Offset (to be removed from PC settings)

#ifdef SPEKTRUM
    1,				// [24] Throttle Channel
    3,				// [25] Pitch Channel
    2,				// [26] Roll Channel
    4,				// [27] Yaw Channel
#else
    3,				// [24] Throttle Channel
    2,				// [25] Pitch Channel
    1,				// [26] Roll Channel
    4,				// [27] Yaw Channel
#endif

    15,				// [28] D_sens_acro = 15

    10,				// [29] Dd_sens = 10
    5,				// [30] Switch channel
    243,			// [31] LED1 pattern
    207,			// [32] LED2 pattern
};

// --Servo control--

// --Read Receiver (Rx)--
#define MAX_CHANNELS 12
volatile int PPM_in[26] = { 0, };
volatile int PPM_diff[26] = { 0, };
volatile unsigned char NewPpmData = 1;
volatile unsigned char RC_Quality = 0;
static u16 BeepTime = 2500;
static u16 BeepMask = 0xFFFF;
extern unsigned char SpektrumTimer;

/*
    M1 = CW
    M2 = CW
    M3 = CCW
    M4 = CCW

    Quadrotor-X config
    M1   M3
      \ /
      / \
    M4   M2
    
    Quadrotor-+ config
       M1
     M4  M3
       M2

    More mixes to be added later

*/

enum eMixerType { QUAD = 0, QUAD_X = 1 }; 
enum MotorIndex { MOTOR1 = 0, MOTOR2, MOTOR3, MOTOR4, MOTOR5, MOTOR6, MOTOR7, MOTOR8 };
#define MOTOR_START_ADDR        (0x52)          // i2c address of 1st motor, the rest are += 2
static s16 Motors[8] = { 0, };                  // motor values from mixer, for output to i2c. really only need u8 though, might fix 

// Gyros
static u16 Roll_init;		// gyro offset
static u16 Pitch_init;		// gyro offset
static u16 Yaw_init;		// gyro offset

static s16 Meas_roll;		// angular velocity reading from gyros
static float Meas_angle_roll;	// the angle of the copter including acc signal (only in Hover mode)

static s16 Meas_pitch;		// angular velocity reading from gyros
static float Meas_angle_pitch;	// the angle of the copter including acc signal (only in Hover mode)

static float P_set_roll;	// P output including scaling (gain)
static float I_set_roll;	// I output including scaling (gain)
static float D_set_roll;	// D output including scaling (gain)

static float P_set_pitch;	// P output including scaling (gain)
static float I_set_pitch;	// I output including scaling (gain)
static float D_set_pitch;	// D output including scaling (gain)

static s16 Yaw_gyro;		// angular velocity reading from gyroscope
static s16 Yaw_gyro_i;		// yaw gyro var
static float Yaw_gyro_scale;	// scaled-down values
static float Yaw_gyro_i_scale;	// scaled-down values

static float P_sens;		// roll/pitch P control loop sensitivity
static float I_sens;		// roll/pitch I control loop sensitivity
static float D_sens;		// roll/pitch D control loop sensitivity
static float Yaw_p_sens;	// yaw P sensitivity
static float Yaw_i_sens;	// yaw I sensitivity

static u8 Gyro_i_enable = 0;	// 0 or 1: don't perform gyro integration when motors off

static float Error_pitch_d[3];	// As Single
static float Error_roll_d[3];	// As Single
static float Error_pitch_old[3];// As Single
static float Error_roll_old[3];	// As Single
static u8 Looper;		// As Byte
static float D_sens_acro;	// As Single

static float Dd_sens;		// As Single
static float Dd_set_pitch;	// As Single
static float Dd_set_roll;	// As Single

//'--Acc--
static s16 Xacc;		// accelerometer pitch
static s16 Yacc;		// accelerometer roll

//'--Failsave--
static u8 Failure;		// increases when there are errors in receiver reading

//'--Mixer--
static u16 Minthrottle;		// throttle idle up

//'--Voltage Check--
static u8 Voltage_Cell_Min;	// Minimum per-cell voltage (3.3V default)
static u16 Voltage_Warn;	// Battery Warning Voltage
static u16 Voltage_Level = 100;	// Battery Voltage
static u8 Lowvoltage;		// will be 1 if voltage is low

// '--LEDs--
static u16 Ledcount = 1;	// used for bling bling
static u16 Blinker;		// used for low voltage bling bling

// '--GUI Connection--
static u8 Settings[33];		// As Byte
static u8 Motorsenable;		// As Byte
static u8 Roll_gyro_dir;	// As Byte
static u8 Pitch_gyro_dir;	// As Byte
static u8 Yaw_gyro_dir;		// As Byte
static u8 Xacc_dir;		// As Byte
static u8 Yacc_dir;		// As Byte
static float P_sens_acro;	// As Single
static float I_sens_acro;	// As Single
static float P_sens_hover;	// As Single
static float I_sens_hover;	// As Single
static float D_sens_hover;	// As Single
static float Yaw_p_sens_eep;	// As Single
static float Yaw_i_sens_eep;	// As Single
static float Acc_influence;	// As Single
static float Gyro_influence;	// As Single
static s16 Xacc_scale;		// As Integer
static s16 Yacc_scale;		// As Integer
static float Lf_acro;		// As Single
static float Lf_hover;		// As Single
static s16 Lf_yaw;		// As Integer
static u8 Lf_boost;		// As Byte
static u16 Idle_up;		// As Word
static s16 Xacc_offset;		// As Integer
static s16 Yacc_offset;		// As Integer
static u8 Throttlechannel;	// As Byte
static u8 Pitchchannel;		// As Byte
static u8 Rollchannel;		// As Byte
static u8 Yawchannel;		// As Byte
static u8 Switchchannel = 5;	// RC channel currently used for controlling flight mode
static u8 MixerType = QUAD_X;   // Type of mixer currently in use

volatile s16 CountMilliseconds = 0;

static void acc_calibration(void);
static void gyro_calibration(void);

static void Mixer();		// prepare all signals for motors and servo
static void Gyro();		// read and prepare gyro's
static void Acc();		// read and rescale accelerometer
static void Led();		// bling bling
static void Send_mots();	// check and reshape motor signals and send them via i2c
static void Voltage();		// check voltage
static void Failsave();		// safety: turn off motors if reception is lost
static void Guiconnection();

static void settings_write(void);
static void settings_load(void);

static u16 Getadc(u8 channel);

void beep(u8 count, u16 duration)
{
    return;

    if (State != 0)
	return;

    while (count--) {
	BeepTime = duration;
	while (BeepTime);
	_delay_ms(duration * 2);
    }
}

static void lipo_init()
{
    u8 i, cell = 2;

    // measure a few times to start rolling average
    for (i = 0; i < 30; i++) {
	u16 volt = Getadc(ADC_VOLTAGE);
	Voltage_Level = (3 * Voltage_Level + volt / 3) / 4;
    }

    for (cell = 2; cell < 6; cell++) {

	if (Voltage_Level < cell * MAX_LIPO_CELL_VOLTAGE)
	    break;
    }
    Voltage_Warn = cell * Voltage_Cell_Min;	// 3.3V per cell minimum, configurable in GUI
    beep(cell, 200);
}

// '===RC turned on...?===
static void rctest(void)
{
    u16 Rc_on_counter;		// counter that counts the amount of correct rx signals

    // 'if motors are enabled from GUI, it is important to have stick in correct position
    if (Motorsenable == 1) {
	do {
	    // 'switch (channel5) and throttle stick must be zero in order to proceed
	    if (RC_Quality == 0 || PPM_in[Switchchannel] > -120 || PPM_in[Throttlechannel] > -120) {
		// no RC signal or sticks not in correct position
		if (Rc_on_counter > 0)
		    Rc_on_counter--;
	    } else {
		Rc_on_counter++;
	    }

	    if (Rc_on_counter == 0) {
		// Toggle Led_1
		RED_FLASH;	// give signal to pilot
		// Toggle Led_3
		_delay_ms(75);
	    }
	    _delay_ms(1);
	} while (Rc_on_counter < 500);

	// turn off led
	RED_OFF;
    }
}

static void acc_calibration(void)
{
    int i;

    u16 Roll_check[5];
    u16 Pitch_check[5];
    s16 Checkdiff;
    u16 AccY_init = 0;
    u16 AccX_init = 0;
    u8 calibrated = 0;

    BeepTime = 500;
    BeepMask = 0xFEAF;

    while (!calibrated) {
	AccY_init = 0;
	AccX_init = 0;

	for (i = 0; i < 5; i++) {
	    AccY_init = AccY_init + Getadc(ADC_ACC_ROLL);
	    Roll_check[i] = Getadc(ADC_ACC_ROLL);
	    AccX_init = AccX_init + Getadc(ADC_ACC_PITCH);
	    Pitch_check[i] = Getadc(ADC_ACC_PITCH);
	    _delay_ms(100);
	}

	AccY_init = AccY_init / 5;
	AccX_init = AccX_init / 5;

	RED_FLASH;
	// Toggle Led_2

	for (i = 0; i < 5; i++) {
	    Checkdiff = AccY_init - Roll_check[i];	// compare mean value vs individual values
	    Checkdiff = abs(Checkdiff);
	    if (Checkdiff > 2)
		calibrated = 0;	// if individual values differ from mean, then the copter was moved. Redo offset measurement.
	    Checkdiff = AccX_init - Pitch_check[i];	// compare mean value vs individual values
	    Checkdiff = abs(Checkdiff);
	    if (Checkdiff > 2)
		calibrated = 0;	// if individual values differ from mean, then the copter was moved. Redo offset measurement.
	}
	calibrated = 1;
    }
    printf("Acc Calibrated: %d, %d\r\n", AccY_init, AccX_init);
    Settings[PARAM_XACC_OFFSET] = AccX_init - 384;
    Settings[PARAM_YACC_OFFSET] = AccY_init - 384;

    settings_write();
    settings_load();

    RED_OFF;
}

#define AUTOZERO_PORT PORTD
#define AUTOZERO_DDR DDRD
#define AUTOZERO_BIT 5

static void gyro_calibration(void)
{
    int i;

    u16 Roll_check[5];		// gyro offset
    u16 Pitch_check[5];		// gyro offset
    u16 Yaw_check[5];		// gyro offset
    s16 Checkdiff = 10;

    BeepTime = 500;
    BeepMask = 0xFEAF;

    do {
	// If port not already set to output and high, do it.
	if (!(AUTOZERO_DDR & (1 << AUTOZERO_BIT)) || !(AUTOZERO_PORT & (1 << AUTOZERO_BIT))) {
	    AUTOZERO_PORT &= ~(1 << AUTOZERO_BIT);
	    AUTOZERO_DDR |= (1 << AUTOZERO_BIT);
	    _delay_ms(100);
	}
	// Make a pulse on the auto-zero output line.
	AUTOZERO_PORT |= (1 << AUTOZERO_BIT);
	_delay_ms(3);
	AUTOZERO_PORT &= ~(1 << AUTOZERO_BIT);
	_delay_ms(100);

	Roll_init = 0;
	Pitch_init = 0;
	Yaw_init = 0;

	for (i = 0; i < 5; i++) {
	    Roll_init = Roll_init + Getadc(ADC_GYRO_ROLL);
	    Roll_check[i] = Getadc(ADC_GYRO_ROLL);
	    Pitch_init = Pitch_init + Getadc(ADC_GYRO_PITCH);
	    Pitch_check[i] = Getadc(ADC_GYRO_PITCH);
	    Yaw_init = Yaw_init + Getadc(ADC_GYRO_YAW);
	    Yaw_check[i] = Getadc(ADC_GYRO_YAW);
	    _delay_ms(100);
	}

	Roll_init = Roll_init / 5;
	Pitch_init = Pitch_init / 5;
	Yaw_init = Yaw_init / 5;

	RED_FLASH;
	// Toggle Led_2

	for (i = 0; i < 5; i++) {
	    Checkdiff = Roll_init - Roll_check[i];	// compare mean value vs individual values
	    Checkdiff = abs(Checkdiff);
	    if (Checkdiff > 2)
		continue;	                        // if individual values differ from mean, then the copter was moved. Redo offset measurement.
	    Checkdiff = Pitch_init - Pitch_check[i];	// compare mean value vs individual values
	    Checkdiff = abs(Checkdiff);
	    if (Checkdiff > 2)
		continue;	                        // if individual values differ from mean, then the copter was moved. Redo offset measurement.
	    Checkdiff = Yaw_init - Yaw_check[i];	// compare mean value vs individual values
	    Checkdiff = abs(Checkdiff);
	    if (Checkdiff > 2)
		continue;	                        // if individual values differ from mean, then the copter was moved. Redo offset measurement.
	}
    } while (Checkdiff > 2);

    printf("Gyro Calibrated: %d, %d, %d\r\n", Roll_init, Pitch_init, Yaw_init);
    RED_OFF;
}

ISR(TIMER0_OVF_vect)
{
    static u8 cnt_1ms = 1, cnt = 0;
    u8 beep_on = 0;

    if (SpektrumTimer)
	SpektrumTimer--;

    if (!cnt--) {
	cnt = 9;
	CountMilliseconds++;
	cnt_1ms++;
	cnt_1ms %= 2;

	if (BeepTime) {
	    if (BeepTime > 10)
		BeepTime -= 10;
	    else
		BeepTime = 0;

	    if (BeepTime & BeepMask)
		beep_on = 1;
	    else
		beep_on = 0;
	} else {
	    beep_on = 0;
	    BeepMask = 0xffff;
	}

	if (beep_on) {
	    BUZZ_ON;
	} else {
	    BUZZ_OFF;
	}
    }
}

/********************************************************************/
/*         Every time a positive edge is detected at PD6            */
/********************************************************************/
/*                               t-Frame
 <----------------------------------------------------------------------->
     ____   ______   _____   ________                ______    sync gap      ____
    |    | |      | |     | |        |              |      |                |
    |    | |      | |     | |        |              |      |                |
 ___|    |_|      |_|     |_|        |_.............|      |________________|
 <-----><-------><------><-------->              <------>                <---
 t0       t1      t2       t4                     tn                     t0

 The PPM-Frame length is 22.5 ms.
 Channel high pulse width range is 0.7 ms to 1.7 ms completed by an 0.3 ms low pulse.
 The mininimum time delay of two events coding a channel is ( 0.7 + 0.3) ms = 1 ms.
 The maximum time delay of two events coding a chanel is ( 1.7 + 0.3) ms = 2 ms.
 The minimum duration of all channels at minimum value is  8 * 1 ms = 8 ms.
 The maximum duration of all channels at maximum value is  8 * 2 ms = 16 ms.
 The remaining time of (22.5 - 8 ms) ms = 14.5 ms  to (22.5 - 16 ms) ms = 6.5 ms is
 the syncronization gap.
 */
ISR(TIMER1_CAPT_vect)
{				// typical rate of 1 ms to 2 ms
    int16_t signal = 0, tmp;
    static int16_t index;
    static uint16_t oldICR1 = 0;

    // 16bit Input Capture Register ICR1 contains the timer value TCNT1
    // at the time the edge was detected

    // calculate the time delay to the previous event time which is stored in oldICR1
    // calculatiing the difference of the two uint16_t and converting the result to an int16_t
    // implicit handles a timer overflow 65535 -> 0 the right way.
    signal = (uint16_t) ICR1 - oldICR1;
    oldICR1 = ICR1;

    //sync gap? (3.52 ms < signal < 25.6 ms)
    if ((signal > 1100) && (signal < 8000)) {
	// if a sync gap happens and there where at least 4 channels decoded before
	// then the NewPpmData flag is reset indicating valid data in the PPM_in[] array.
	if (index >= 4) {
	    NewPpmData = 0;	// Null means NewData for the first 4 channels
	}
	// synchronize channel index
	index = 1;
    } else {			// within the PPM frame
	if (index < MAX_CHANNELS - 1) {	// PPM24 supports 12 channels
	    // check for valid signal length (0.8 ms < signal < 2.1984 ms)
	    // signal range is from 1.0ms/3.2us = 312 to 2.0ms/3.2us = 625
	    if ((signal > 250) && (signal < 687)) {
		// shift signal to zero symmetric range  -154 to 159
		signal -= 470;	// offset of 1.4912 ms ??? (469 * 3.2µs = 1.5008 ms)
		// check for stable signal
		if (abs(signal - PPM_in[index]) < 6) {
		    if (RC_Quality < 200)
			RC_Quality += 10;
		    else
			RC_Quality = 200;
		}
		// If signal is the same as before +/- 1, just keep it there.
		if (signal >= PPM_in[index] - 1 && signal <= PPM_in[index] + 1) {
		    // In addition, if the signal is very close to 0, just set it to 0.
		    if (signal >= -1 && signal <= 1) {
			tmp = 0;
		    } else {
			tmp = PPM_in[index];
		    }
		} else
		    tmp = signal;
		// calculate signal difference on good signal level
		if (RC_Quality >= 195)
		    PPM_diff[index] = ((tmp - PPM_in[index]) / 3) * 3;	// cut off lower 3 bit for nois reduction
		else
		    PPM_diff[index] = 0;
		PPM_in[index] = CLAMP(tmp, -127, 127);	// update channel value
	    }
	    index++;		// next channel
	}
    }
}

static inline void timer_init(void)
{
    // Timer 0: 9.7kHz general-purpose timer used for blinking leds and beeping
    TCCR0B = 2;			// Prescaler /8
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | 3;	//fast PWM
    OCR0A = 0;
    OCR0B = 180;
    TCNT0 = (unsigned char) -TIMER0_RELOAD_VALUE;	// reload
    TIMSK0 |= _BV(TOIE0);

#ifndef SPEKTRUM
    // Timer 1: ICP timer for PPM input    
    TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM11) | (1 << WGM10));
    TCCR1B &= ~((1 << WGM13) | (1 << WGM12) | (1 << CS12));
    TCCR1B |= (1 << CS11) | (1 << CS10) | (1 << ICES1) | (1 << ICNC1);
    TCCR1C &= ~((1 << FOC1A) | (1 << FOC1B));
    // Set ICP to input, internal pull up
    PORTD |= _BV(PIND6);
    DDRD &= ~(_BV(PIND6));
    // Enable interrupt on input capture
    TIMSK1 |= _BV(ICIE1);
#endif
}

static inline void analog_init(void)
{
    // external ADC reference       
    ADMUX = 0;
    // ADCSRA = (1 << ADEN) | (1 << ADSC) | (0 << ADATE) | (0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // mine
    ADCSRA = (1 << ADEN) | (1 << ADSC) | (0 << ADATE) | (1 << ADPS2) | (0 << ADPS1) | (1 << ADPS0);
}

int main(void)
{
    int i;

    // I/O pins
    DDRB = 0x1B;		// LEDs and Pressure Offset
    PORTB = 0x01;		// Red LED
    DDRD = 0x3E;		// Speaker & TXD & J3 J4 J5
    PORTD = 0x47;		// Green LED
    DDRC |= (1 << DDC7);	// Speaker
    RED_OFF;
    GRN_OFF;

    analog_init();
    timer_init();
    i2c_init();
    uart_init();
#ifdef SPEKTRUM
    spektrum_init();
#endif
    stdout = stdin = &uart_str;

    _delay_ms(100);
    sei();			// Enable Interrupts

    for (i = 0; i < 4; i++) {
        Motors[i] = 0;
        i2c_write(MOTOR_START_ADDR + (i * 2), 0);
    }

    for (i = 0; i < 5; i++) {
	GRN_FLASH;
	_delay_ms(200);
    }

    settings_load();

    for (i = 0; i < 5; i++) {
	GRN_FLASH;
	_delay_ms(200);
    }

    // now we know mincell voltage, can do lipo
    lipo_init();

    // rc test
    rctest();
    _delay_ms(50);		// If we got the correct stick positions 500 times... proceed

    // Measure Gyro Offset (don't move copter now!)
    gyro_calibration();

    GRN_ON;                     // All green
    BUZZ_OFF;

    while (1) {
	Acc();
	Gyro();
	Mixer();
	Send_mots();
	Led();
	Voltage();
	// Failsave();
	Guiconnection();
    }
}

u16 Getadc(u8 channel)
{
    // choose channel
    ADMUX = channel & 0x07;
    // start the conversion
    ADCSRA |= _BV(ADSC);
    // ADSC is cleared when the conversion finishes
    while (bit_is_set(ADCSRA, ADSC));

    return ADCW;
}

static const float DynamicBoost[] = { 5.8f, 6.1f, 6.3f, 6.5f, 6.8f, 7.2f, 7.6f, 8.1f, 8.7f, 9.4f, 10.2f, 11.0f, 12.0f, 13.6f };

void Mixer(void)
{
    int i;
    static int cal_counter = 0;

    s16 throttle_stick = ((PPM_in[Throttlechannel] + 127) * (228.0f / 255.0f)) + 3;	// (3-228)
    s16 pitch_stick = PPM_in[Pitchchannel] / 4;
    s16 roll_stick = PPM_in[Rollchannel] / 4;
    s16 switch_channel = PPM_in[Switchchannel];

#if 0
    printf("Throttle: %d, State: %d Motors: %d\r\n", throttle, State, Motorsenable);
#endif

    if (State == 0) {
	// Motors off. Do various calibration things here
	if (PPM_in[Throttlechannel] > 100 && PPM_in[Yawchannel] < -100) {
	    // ACC calibration (Throttle stick up+left)
	    cal_counter++;
	    if (cal_counter == 1000) {
		RED_ON;
		BeepTime = 500;
		BeepMask = 0xFF00;
		acc_calibration();
		BeepTime = 1000;
		BeepMask = 0xFFFF;
	    }
	} else if (PPM_in[Throttlechannel] > 100 && PPM_in[Yawchannel] > 100) {
	    // Gyro calibration (Throttle stick up+right)
	    cal_counter++;
	    if (cal_counter == 1000) {
		RED_ON;
		BeepTime = 500;
		BeepMask = 0x0F0F;
		gyro_calibration();
		BeepTime = 100;
		BeepMask = 0xFFFF;
	    }
	} else {
	    cal_counter = 0;
	    RED_OFF;
	}
    }

    // Dynamic LF: Makes the copter react non-linearly to pitch/roll. I use this for flying loopings. The more I pull, the faster (exponential) the copter will pitch/roll
    if (Lf_boost == 1) {
	s16 Lookup_pos_pitch;
	s16 Lookup_pos_roll;

	if (State == 1) {	// only when in acro mode
	    Lookup_pos_pitch = abs(pitch_stick);	// make a variable that grows when stick is out of centre
	    Lookup_pos_pitch = Lookup_pos_pitch - 25;
	    Lookup_pos_pitch = CLAMP(Lookup_pos_pitch, 0, 13);
	    if (Lookup_pos_pitch != 0)
		Lfdynamic_pitch = DynamicBoost[Lookup_pos_pitch];	// TODO Lookup(lookup_pos_nick , Dta)        'look for a new sensitivity factor in a table
	    else
		Lfdynamic_pitch = Lf;

	    Lookup_pos_roll = abs(roll_stick);	//               'make a variable that grows when stick is out of centre
	    Lookup_pos_roll = Lookup_pos_roll - 26;
	    Lookup_pos_roll = CLAMP(Lookup_pos_roll, 0, 13);
	    if (Lookup_pos_roll != 0)
		Lfdynamic_roll = DynamicBoost[Lookup_pos_roll];	// TODO Lookup(lookup_pos_roll , Dta)        'look for a new sensitivity factor in a table
	    else
		Lfdynamic_roll = Lf;
	}
    } else {
	Lfdynamic_roll = Lf;
	Lfdynamic_pitch = Lf;
    }

    // ACRO MODE (angular velocity control, ACC = off)
    if (Motorsenable == 1) {	// only listen to receiver (and especially channel 5) when user enabled the motors in the GUI
	if (switch_channel > -40 && switch_channel < 40) {	// switch in middle = motors on; sempf(5) is the idle up switch
	    Minthrottle = Idle_up;	//                                        'minimum throttle
	    Gyro_i_enable = 1;	//                                        'start integrating the gyroscope signals
	    Lf = Lf_acro;	//                                             '5.8                                                  'nick and roll sensitivity
	    P_sens = P_sens_acro;	//                                    '0.45                                             'Gyro_P sensitivity roll and nick
	    I_sens = I_sens_acro;	//                                    '0.001                                            'Gyro_I Sensitivity roll and nick
	    D_sens = D_sens_acro;	//                                    'no D available in Acro mode (calculating it only yields noise)
	    Yaw_p_sens = Yaw_p_sens_eep;	//                             'Gyro_P Sensitivity yaw
	    Yaw_i_sens = Yaw_i_sens_eep;	//                             'Gyro_I Sensitivity yaw
	    Motors_on = 1;	//
	    State = 1;		//                                               'flight mode: acrobatic
	}
	// 'HOVER MODE (angle control, ACC = on)
	if (switch_channel >= 40) {	// switch top = motors on
	    Minthrottle = Idle_up;	//                                       'minimum throttle
	    Gyro_i_enable = 1;	//                                       'start integrating the gyroscope signals
	    Lf = Lf_hover;	//                                          '450                                                  'nick and roll sensitivity (much bigger, because a different control loop is used)
	    P_sens = P_sens_hover;	//                                   '0.0025                                           'Gyro_P sensitivity roll and nick (corresponds to Gyro_I in acro mode)
	    I_sens = I_sens_hover;	//                                   '0.0000015                                        'Gyro_I sensitivity roll and nick (this is a double integral, thats why the value is so low)
	    D_sens = D_sens_hover;	//                                   '0.4                                              'Gyro_D sensitivity roll and nick (corresponds to Gyro_P in acro mode)
	    Yaw_p_sens = Yaw_p_sens_eep;	//                             'Gyro_P Sensitivity yaw
	    Yaw_i_sens = Yaw_i_sens_eep;	//                             'Gyro_I Sensitivity yaw
	    Motors_on = 1;	//
	    State = 2;		//                                               'flight mode: hover
	}

	if (switch_channel < -40) {	// switch bottom = motors off
	    if (throttle_stick < 20) {	//                     'only turn motors off when throttle stick is also on bottom
		Minthrottle = 0;	//                                             'no minimum throttle = motors off
		Gyro_i_enable = 0;	//                                      'do not integrate gyros anymore
		Motors_on = 0;	//
		State = 0;	//                                              'flight mode: off
		Lf = 0;		//                                                 'don't react to stick movements
		P_sens = 0;	//                                             'don't react to gyros
		I_sens = 0;	//                                             'don't react to gyros
		D_sens = 0;	//                                             'don't react to gyros
		Yaw_p_sens = 0;	//                                         'don't react to gyros
		Yaw_i_sens = 0;	//                                         'don't react to gyros
	    }
	}
    } else {			// 'if motors were not enabled in GUI or if EEprom emty: always stay in GUI mode
	Minthrottle = 0;	//                                              'no minimum throttle = motors off
	Gyro_i_enable = 0;	//                                        'do not integrate gyros anymore
	Motors_on = 0;		//
	State = 0;		//                                                'flight mode: off
	Lf = 0;			//                                                   'don't react to stick movements
	P_sens = 0;		//                                               'don't react to gyros
	I_sens = 0;		//                                               'don't react to gyros
	D_sens = 0;		//                                               'don't react to gyros
	Yaw_p_sens = 0;		//                                           'don't react to gyros
	Yaw_i_sens = 0;		//                                           'don't react to gyros
    }

    // Mix components
    if (Motors_on == 1) {
        // only when motors are running
        s16 Roll, Pitch, Yaw;

        // Calculate roll/pitch/yaw values for mixer
        Roll = P_set_roll + I_set_roll + D_set_roll;
        Pitch = P_set_pitch + I_set_pitch + D_set_pitch;
        Yaw = Yaw_gyro_scale + Yaw_gyro_i_scale;

        if (State == 2) {
            // In hover, add Dd terms
            Roll += Dd_set_roll;
            Pitch += Dd_set_pitch;
        }

        // Init with throttle
        for (i = 0; i < 4; i++)
            Motors[i] = throttle_stick + Minthrottle;

        // Mixer
        if (MixerType == QUAD) {
            // Quad-Plus mixer
            Motors[MOTOR3] -= Roll;
            Motors[MOTOR4] += Roll;

            Motors[MOTOR1] += Pitch;
            Motors[MOTOR2] -= Pitch;

            Motors[MOTOR1] -= Yaw;
            Motors[MOTOR2] -= Yaw;
            Motors[MOTOR3] += Yaw;
            Motors[MOTOR4] += Yaw;
        } else if (MixerType == QUAD_X) {
            // Quad-X mixer
            Motors[MOTOR1] += Roll;
            Motors[MOTOR2] -= Roll;
            Motors[MOTOR3] -= Roll;
            Motors[MOTOR4] += Roll;
        
            Motors[MOTOR1] += Pitch;
            Motors[MOTOR2] -= Pitch;
            Motors[MOTOR3] += Pitch;
            Motors[MOTOR4] -= Pitch;
        
            Motors[MOTOR1] -= Yaw;
            Motors[MOTOR2] -= Yaw;
            Motors[MOTOR3] += Yaw;
            Motors[MOTOR4] += Yaw;
        }            

        for (i = 0; i < 4; i++) {
            if (Motors[i] < (s16)Minthrottle)
                Motors[i] = Minthrottle;
        }                

#if 0
	printf("Roll: %d Pitch: %d Yaw: %d Motors: %d, %d, %d, %d\r\n", Roll, Pitch, Yaw, Motors[0], Motors[1], Motors[2], Motors[3]);
#endif
    } else {
        // motors off
        for (i = 0; i < 4; i++)
            Motors[i] = 0;
    }
}

static inline void Gyro(void)
{
    int i;
    s16 yaw_stick = PPM_in[Yawchannel] / 4;
    s16 pitch_stick = PPM_in[Pitchchannel] / 4;
    s16 roll_stick = PPM_in[Rollchannel] / 4;

    float Setpoint_roll;	// Stick position
    float Error_roll;		// Stick position - current position
    float Error_roll_sum;	// Integral of the above
    float Setpoint_pitch;	// Stick position
    float Error_pitch;		// Stick position - current position
    float Error_pitch_sum;	// Integral of the above
    s16 Yaw_diff;		// yaw gyro var

    if (Old_state != State) {	// 'compare state of last cycle with current state
	Error_roll = 0;		// 'otherwise things would start mixing up...
	Error_pitch = 0;
	Meas_roll = 0;
	Meas_pitch = 0;
	Error_roll_sum = 0;
	Error_pitch_sum = 0;
	D_set_roll = 0;
	D_set_pitch = 0;
	for (i = 0; i < 3; i++) {
	    Error_roll_old[i] = 0;
	    Error_roll_d[i] = 0;
	    Error_pitch_old[i] = 0;
	    Error_pitch_d[i] = 0;
	}

	if (State == 2) {
	    Meas_angle_roll = Yacc;	            // when switching from acro mode to hover mode
	    Meas_angle_pitch = Xacc;	            // start with a "close to reality" angle
	} else {
	    Meas_angle_roll = 0;
	    Meas_angle_pitch = 0;
	}
	Yaw_gyro_i = 0;
	Yaw_diff = 0;
    }

    Old_state = State;

    if (State == 1) {		                    // ACROBATIC MODE = Angular velocity control

	if (Looper < 2) {	// for angular acceleration measurement
	    Looper++;		// acceleration will be calculated as the difference in velocity between
	} else {		// loop n and loop n+2
	    Looper = 0;
	}

	// Roll
	Meas_roll = Getadc(ADC_GYRO_ROLL);	// get gyro signal
	if (Roll_gyro_dir == 0) {	// make gyro direction reversal possible
	    Meas_roll = Roll_init - Meas_roll;	// subtract offset
	} else if (Roll_gyro_dir == 1) {
	    Meas_roll = Meas_roll - Roll_init;	// subtract offset
	}

	Setpoint_roll = roll_stick * Lfdynamic_roll;	// roll stick position
	Error_roll = Meas_roll - Setpoint_roll;	// calculate difference between angular velocity and stick position
	// this calculates the angular velocity (D-term in acro mode)
	Error_roll_d[Looper] = Error_roll - Error_roll_old[Looper];
	Error_roll_old[Looper] = Error_roll;
	D_set_roll = Error_roll_d[Looper] * D_sens;
	// clip here
	Error_roll_sum = Error_roll_sum + Error_roll;	// integrate the above
	Error_roll_sum = CLAMP(Error_roll_sum, -10000, 10000);
	P_set_roll = Error_roll * P_sens;	// multiply with gain
        // don't integrate when motors off
	if (Gyro_i_enable == 0)
	    Error_roll_sum = 0;
	I_set_roll = Error_roll_sum * I_sens;	// multiply with gain

	// Pitch
	Meas_pitch = Getadc(ADC_GYRO_PITCH);	// see above
	if (Pitch_gyro_dir == 0) {
	    Meas_pitch = Meas_pitch - Pitch_init;
	} else if (Pitch_gyro_dir == 1) {
	    Meas_pitch = Pitch_init - Meas_pitch;
	}

	Setpoint_pitch = pitch_stick * Lfdynamic_pitch;
	Error_pitch = Meas_pitch - Setpoint_pitch;
	// this calculates the angular velocity (D-term in acro mode)
	Error_pitch_d[Looper] = Error_pitch - Error_pitch_old[Looper];
	Error_pitch_old[Looper] = Error_pitch;
	D_set_pitch = Error_pitch_d[Looper] * D_sens;
	// clip here
	Error_pitch_sum = Error_pitch_sum + Error_pitch;
	Error_pitch_sum = CLAMP(Error_pitch_sum, -10000, 10000);
	P_set_pitch = Error_pitch * P_sens;
	if (Gyro_i_enable == 0)
	    Error_pitch_sum = 0;
	I_set_pitch = Error_pitch_sum * I_sens;
    }

    // HOVER MODE = Angle control
    if (State == 2 || State == 0) {
	float Accy_temp;	// used for complementary filtering
	float Accx_temp;	// used for complementary filtering

	// for angular acceleration measurement
	// acceleration will be calculated as the difference in velocity between
	// loop n and loop n+2
	if (Looper < 1) {
	    Looper++;
	} else {
	    Looper = 0;
	}

	// Roll
	Meas_roll = Getadc(ADC_GYRO_ROLL);	// get gyro signal
	if (Roll_gyro_dir == 0) {	// make gyro direction reversal possible
	    Meas_roll = Roll_init - Meas_roll;	// subtract offset
	} else if (Roll_gyro_dir == 1) {
	    Meas_roll = Meas_roll - Roll_init;
	}

	// this calculates the angular velocity (D-term in acro mode)
	Error_roll_d[Looper] = Meas_roll - Error_roll_old[Looper];
	Error_roll_old[Looper] = Meas_roll;
	Dd_set_roll = Error_roll_d[Looper] * Dd_sens;
	// integrate gyro signal
	Meas_angle_roll = Meas_angle_roll + Meas_roll;

	// this might require some further explanation:
	// The gyroscopes can only measure differences in rotational speed. Integrated over a long time (e.g. 11 minutes of flight)
	// the angle calculated from gyro's alone loses precision. Here, the acc comes into play: It always knows the angle of the
	// copter, but it reacts pretty slowly. And it contains quite some noise. The following lines of code combine the fast
	// signal of the gyroscopes and the absolute precision of the accelerometer. In the end, you get the best out of both worlds:

	Meas_angle_roll = Meas_angle_roll * Gyro_influence;	// 0.99 take 0.99 of gyro integral and 0.01 of acc...
	Accy_temp = Yacc * Acc_influence;	                // 0.01
	Meas_angle_roll = Meas_angle_roll + Accy_temp;	        // ...and put these two together (complementary filtering)

	Setpoint_roll = roll_stick * Lf;	                // roll stick position * stick sensitivity
	Error_roll = Meas_angle_roll - Setpoint_roll;	        // current angle minus desired angle (stick position)
	Error_roll_sum = Error_roll_sum + Error_roll;	        // integral of an integral
	Error_roll_sum = CLAMP(Error_roll_sum, -4000000, 4000000);	// integral clipping
	P_set_roll = Error_roll * P_sens;	                // multiply with gain
	if (Gyro_i_enable == 0)
	    Error_roll_sum = 0;
	I_set_roll = Error_roll_sum * I_sens;	                // multiply with gain
	D_set_roll = Meas_roll * D_sens;	                // multiply with gain

	// Pitch
	Meas_pitch = Getadc(ADC_GYRO_PITCH);	                // see above
	if (Pitch_gyro_dir == 0) {
	    Meas_pitch = Meas_pitch - Pitch_init;
	} else if (Pitch_gyro_dir == 1) {
	    Meas_pitch = Pitch_init - Meas_pitch;
	}

	// this calculates the angular velocity (D-term in acro mode)
	Error_pitch_d[Looper] = Meas_pitch - Error_pitch_old[Looper];
	Error_pitch_old[Looper] = Meas_pitch;
	Dd_set_pitch = Error_pitch_d[Looper] * Dd_sens;

	Meas_angle_pitch = Meas_angle_pitch + Meas_pitch;
	Meas_angle_pitch = Meas_angle_pitch * Gyro_influence;	// 0.99
	Accx_temp = Xacc * Acc_influence;	                // 0.01
	Meas_angle_pitch = Meas_angle_pitch + Accx_temp;
	Setpoint_pitch = pitch_stick * Lf;
	Error_pitch = Meas_angle_pitch - Setpoint_pitch;
	Error_pitch_sum = Error_pitch_sum + Error_pitch;
	Error_pitch_sum = CLAMP(Error_pitch_sum, -4000000, 4000000);	// integral clipping

	P_set_pitch = Error_pitch * P_sens;
	if (Gyro_i_enable == 0) {
	    Error_pitch_sum = 0;
	}
	I_set_pitch = Error_pitch_sum * I_sens;
	D_set_pitch = Meas_pitch * D_sens;
    }

    // Yaw
    Yaw_gyro = Getadc(ADC_GYRO_YAW);	                        // get yaw rate from gyro
    if (Yaw_gyro_dir == 0) {
	Yaw_gyro = Yaw_gyro - Yaw_init;	                        // subtract offset
    } else {
	Yaw_gyro = Yaw_init - Yaw_gyro;
    }

    Yaw_diff = yaw_stick * Lf_yaw;	                        // yaw stick position
    if (Yaw_gyro_dir == 1)
	Yaw_diff = -Yaw_diff;

    Yaw_diff = Yaw_diff - Yaw_gyro;	                        // stick position - current angular velocity
    Yaw_gyro_i = Yaw_gyro_i + Yaw_diff;	                        // integral of the above
    Yaw_gyro_i = CLAMP(Yaw_gyro_i, -32000, 32000);	        // protect from overflow

    Yaw_gyro_scale = Yaw_diff * Yaw_p_sens;	                // multiply with gain
    // integrate only when motors on
    if (Gyro_i_enable == 0)
	Yaw_gyro_i = 0;
    Yaw_gyro_i_scale = Yaw_gyro_i * Yaw_i_sens;	                // multiply with gain
}

void Acc()
{
    if (State != 1) {		// 'only used in HOVER MODE and when motors off
	Xacc = Getadc(ADC_ACC_PITCH);	// read pitch "angle"
	Yacc = Getadc(ADC_ACC_ROLL);	// read roll "angle"
	if (Xacc_dir == 1) {	// make xacc direction reversable
	    Xacc = Xacc_offset - Xacc;	// these values are the ACC offsets. They were determined in flight (hovering in place)
	} else if (Xacc_dir == 0) {
	    Xacc = Xacc - Xacc_offset;
	}
	if (Yacc_dir == 1) {
	    Yacc = Yacc_offset - Yacc;
	} else if (Yacc_dir == 0) {
	    Yacc = Yacc - Yacc_offset;
	}
	Yacc = Yacc * Yacc_scale;	// 120 make the amplitude if gyro_integrals and acc similar
	Xacc = Xacc * Xacc_scale;	// 120 (necessary for successful complementary filtering)
    }
}

void Led(void)
{
    static u8 l1mask;
    static u8 l2mask;
    static u8 index;

    if (Ledcount < 500) {
	Ledcount++;
	return;
    }

    Ledcount = 0;

    if (index == 0) {
	// refresh masks
	if (Lowvoltage) {
	    l1mask = 0xaa;
	    l2mask = 0xaa;
	} else {
	    l1mask = Settings[PARAM_LED1_MASK];
	    l2mask = Settings[PARAM_LED2_MASK];
	}
    }

    if (l1mask & _BV(index)) {
	LED1_ON;
    } else {
	LED1_OFF;
    }

    if (l2mask & _BV(index)) {
	LED2_ON;
    } else {
	LED2_OFF;
    }

    index++;
    index %= 8;

#if 0
    if (State > 0) {		// 'flight modes 1 & 2 get a different LED signal
	// 'flash LEDs
	if (Ledcount == 130 || Ledcount == 140 || Ledcount == 230 || Ledcount == 240) {
	    // Toggle Led_1
	}

	if (Ledcount == 100 || Ledcount == 110 || Ledcount == 200 || Ledcount == 210) {
	    // Toggle Led_2
	}

	if (Ledcount == 160 || Ledcount == 170 || Ledcount == 260 || Ledcount == 270) {
	    // Toggle Led_3
	}

	if (Ledcount == 470) {
	    // Toggle Led_grn
	}
    } else {
	// 'state = 0 ==> ready for GUI connection
	if (Ledcount == 100 || Ledcount == 120 || Ledcount == 200 || Ledcount == 220 || Ledcount == 300 || Ledcount == 320) {
	    // Toggle Led_1
	    // Toggle Led_2
	    // Toggle Led_3
	    // Toggle Led_grn
	}
    }

    // 'when battery is low, blink the LEDs in an annoying manner
    if (Lowvoltage == 1) {
	if (Blinker < 30) {
	    Blinker++;
	} else {
	    Blinker = 1;
	    // Toggle Led_1
	    // Toggle Led_2
	    // Toggle Led_3
	}
	// Reset Led_grn
    } else {
	// 'ledcount> 500
	if (State > 0) {
	    // Set Led_grn                                           'toggle green led every 500 cycles (I use this to determine the speed of the program)
	} else {
	    // Reset Led_grn
	}

	Ledcount = 1;
	// Reset Led_1
	// Reset Led_2
	// Reset Led_3
    }
#endif
}

static void Send_mots()
{
    u8 i;
    
    for (i = 0; i < 4; i++) {
        s16 Limit = (Motors_on == 1) ? Minthrottle : 0;
        if (Motors[i] < Limit)
            Motors[i] = Limit;
        if (Motors[i] > (s16)255)
            Motors[i] = 255;
    }

    // printf("Motors: %d, %d, %d, %d\r\n", Motors[0], Motors[1], Motors[2], Motors[3]);

    if (Failure < 15 && Motorsenable == 1) {	// 'if there are NO problems with the receiver and shrediquette was programmed: run motors
        for (i = 0; i < 4; i++)
            i2c_write(MOTOR_START_ADDR + (i * 2), Motors[i]);
    } else {			// 'if there are problems with the receiver: turn off motors
        for (i = 0; i < 4; i++)
            i2c_write(MOTOR_START_ADDR + (i * 2), 0);
    }
}

void Voltage()
{
    // we'll only check voltage every 500 cycles (==> about 1 Hz)
    if (Ledcount == 500) {
	u16 volt = Getadc(ADC_VOLTAGE);
	Voltage_Level = (3 * Voltage_Level + volt / 3) / 4;

	if (Voltage_Level != 0 && Voltage_Level < Voltage_Warn) {
	    Lowvoltage = 1;	// set low-voltage bit when voltage is low
	}
    }
}

void Failsave()
{
    // handle differently
}

void Guiconnection()
{
    char uartBuffer[8];		// Dim A As String * 8
    int i;
    u8 ch;
    u8 len;
    float tmp;
    u8 Sensor[13];

    if (State == 0) {		// don't listen to serial commands when motors running!
	char *ptr = uartBuffer;
	len = sizeof(uartBuffer);
	memset(uartBuffer, 0, sizeof(uartBuffer));
	if (bit_is_set(UCSR0A, RXC0)) {
	    do {
		loop_until_bit_is_set(UCSR0A, RXC0);
		// get the char
		ch = UDR0;
		if (ch == '\r' || ch == '\n') {
		    break;
		} else {
		    *ptr++ = ch;
		    len--;
		}
	    } while (bit_is_set(UCSR0A, RXC0) || len > 0);

	    if (strlen(uartBuffer) == 0)
		return;

	    if (strstr(uartBuffer, "cr!")) {
		// output all parameters
		_delay_ms(25);
		printf("s#p!\r\n");	// tell pc that data will follow
		fwrite(Settings, 1, sizeof(Settings), stdout);	// print the list of parameters that can be modified
		printf("\r\n");
	    } else if (strstr(uartBuffer, "cs!")) {
		// output realtime sensor data
		printf("ss!\r\n");	// tell pc that sensor data will follow
		// rescale the readings from the sensors to a value ranging from 0 to 255
		// and put these values in an array "sensor(X)"
		tmp = Meas_roll / 2;
		Sensor[0] = CLAMP(tmp, -127, 127) + 127;

		tmp = Meas_angle_roll / 200.0f;
		Sensor[1] = CLAMP(tmp, -127, 127) + 127;

		tmp = (float) Yacc / 200.0f;
		Sensor[2] = CLAMP(tmp, -127, 127) + 127;

		tmp = Meas_pitch / 2;
		Sensor[3] = CLAMP(tmp, -127, 127) + 127;

		tmp = Meas_angle_pitch / 200.0f;
		Sensor[4] = CLAMP(tmp, -127, 127) + 127;

		tmp = (float) Xacc / 200.0f;
		Sensor[5] = CLAMP(tmp, -127, 127) + 127;

		tmp = Yaw_gyro / 2;
		Sensor[6] = CLAMP(tmp, -127, 127) + 127;

		Sensor[7] = PPM_in[Throttlechannel] + 127;
		Sensor[8] = PPM_in[Rollchannel] + 127;
		Sensor[9] = PPM_in[Pitchchannel] + 127;
		Sensor[10] = PPM_in[Yawchannel] + 127;
		Sensor[11] = PPM_in[Switchchannel] + 127;

		Sensor[12] = Voltage_Level;

		// write out sensor data
		fwrite(Sensor, 1, sizeof(Sensor), stdout);
		// write out RC channel data too (12 channels)
		for (i = 0; i < 12; i++) {
		    u8 out = PPM_in[i] + 127;
		    fwrite(&out, 1, 1, stdout);
		}
		printf("\r\n");
	    } else if (strstr(uartBuffer, "ci!")) {
		u8 xacc_temp, yacc_temp;
		xacc_temp = Settings[PARAM_XACC_OFFSET];
		yacc_temp = Settings[PARAM_YACC_OFFSET];

		// pc wants to transfer parameters
		fread(Settings, 1, sizeof(Settings), stdin);

		Settings[PARAM_XACC_OFFSET] = xacc_temp;
		Settings[PARAM_YACC_OFFSET] = yacc_temp;

		//'Ms_h = 0 : I2csend M_h , Ms_h    'turn off the motors
		//'Ms_r = 0 : I2csend M_r , Ms_r
		//'Ms_l = 0 : I2csend M_l , Ms_l
		_delay_ms(10);
		// save parameters to eeprom
		settings_write();
		_delay_ms(50);
		settings_load();
	    } else if (strstr(uartBuffer, "reset!")) {
		_delay_ms(100);
		// TODO $3c00                                             '$3c00 -> m328p; $1c00 -> m168
	    }
	}
    }
}

static void settings_write(void)
{
    u8 i;

    for (i = 0; i < 33; i++)
	eeprom_write_byte((u8 *)i, Settings[i]);
}

static void settings_load(void)
{
    int i = 0;
    u8 blank = 1;

    // turn off motors
    for (i = 0; i < 4; i++) {
        Motors[i] = 0;
        i2c_write(MOTOR_START_ADDR + (i * 2), 0);
    }

    for (i = 0; i < 33; i++) {
	Settings[i] = eeprom_read_byte((u8 *) i);
	if (Settings[i] != 0xff)
	    blank = 0;
    }

    if (blank)
	memcpy(Settings, DefaultSettings, sizeof(Settings));

    Motorsenable = Settings[0];
    Roll_gyro_dir = Settings[1];
    Pitch_gyro_dir = Settings[2];
    Yaw_gyro_dir = Settings[3];
    Xacc_dir = Settings[4];
    Yacc_dir = Settings[5];
    P_sens_acro = Settings[6] / 255.0f;
    I_sens_acro = Settings[7] / 25500.0f;
    P_sens_hover = Settings[8] / 25500.0f;
    I_sens_hover = Settings[9] / 25500000.0f;	//                            'decrease factor to  12800000
    D_sens_hover = Settings[10] / 255.0f;
    Yaw_p_sens_eep = Settings[11] / 255.0f;
    Yaw_i_sens_eep = Settings[12] / 25500.0f;
    Acc_influence = Settings[13] / 3000.0f;
    Gyro_influence = 1 - Acc_influence;
    Xacc_scale = Settings[14];
    Yacc_scale = Settings[15];
    Lf_acro = Settings[16] / 25.5f;
    Lf_hover = Settings[17] * 4.0f;
    Lf_yaw = Settings[18] / 17;
    Lf_boost = Settings[19];
    Idle_up = Settings[20];
    Voltage_Cell_Min = Settings[21];
    Xacc_offset = Settings[PARAM_XACC_OFFSET] + 384;
    Yacc_offset = Settings[PARAM_YACC_OFFSET] + 384;
    Throttlechannel = Settings[24];
    Pitchchannel = Settings[25];
    Rollchannel = Settings[26];
    Yawchannel = Settings[27];
    D_sens_acro = Settings[28] / 50.0f;
    Dd_sens = Settings[29] / 50.0f;
    Switchchannel = Settings[30];

    P_sens_acro *= 6.0 / 2.2;
    I_sens_acro *= 6.0 / 2.2;
    D_sens_acro *= 6.0 / 2.2;

    P_sens_hover *= 6.0 / 2.2;
    I_sens_hover *= 6.0 / 2.2;
    D_sens_hover *= 6.0 / 2.2;

    Xacc_scale *= 2.2 / 6.0;
    Yacc_scale *= 2.2 / 6.0;
}
