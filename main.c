/*
 * This is a small program that uses the 3 outputs of TimerA
 * on the MSP430G2452 to drive each output of a RGB LED.
 * The "system tick" is provided by the WDT in interval mode and is
 * set to ~2ms period.
 * MCLK = SMCLK = 16MHz.
 * WDT and TA are sourced from SMCLK.
 * The RGB outputs come are:
 * P1.1 [TA0.0]
 * P1.4 [TA0.2]
 * P1.6 [TA0.1]
 *
 * The source of entropy for the 'random' modes is the LSB of the
 * internal temperature sensor.  The LSB is shifted in after every
 * ADC conversion and a new target value for the pwm is created.
 *
 * There are 4 modes of operation:
 * Mode 0: Steady on.
 * Mode 1: 'Random' mode.
 * Mode 2: 'Random fast' mode.
 * Mode 3: Up/Down mode.
 */

#include <msp430.h>
#include <stdint.h>


/* Preprocessor Defines */
#define NUM_CHANNELS	3
#define F_MCLK			16000000ul
#define F_SMCLK			16000000ul

#define MIN_TIMER_VAL	((uint16_t) 64)
#define MAX_TIMER_VAL	((uint16_t) 65472)
#define PWM_INCR		((int8_t) 16)
#define RND_FAST_MULT	((int8_t) 4)


/* Function Prototypes */
static inline void configPort1(void);
static inline void configTimerA(void);
static inline void configWDT(void);
static inline void configAdc10(void);
void initTimerVals(void);
static inline void updatePwm_mode_upDn(volatile uint16_t * ta0_ccr_reg, int8_t * incr);
static inline void updatePwm_mode_rnd( volatile uint16_t * ta0_ccr_reg,
									   int8_t * incr,
									   uint16_t * next_target_pwm,
									   uint16_t * target_data_src );
static inline void handlePwmUpdate(void);
static inline void updateTargetVals(void);
static inline void handleButtonPress(void);


/* Type Definitions */
typedef union
{
	struct
	{
		uint8_t update_pwm		: 1;
		uint8_t adc_done		: 1;
		uint8_t button_press	: 1;
		uint8_t unused			: 5;
	};
	uint8_t flags;
} sysFlags_t;

typedef union
{
	struct
	{
		uint8_t pwm_tick_count	:2;
		uint8_t operating_mode	:2;
		uint8_t button_state	:1;
		uint8_t unused			:3;
	};
	uint8_t states;
} sysStates_t;


/* Global Variables */
volatile sysFlags_t g_sys_flags;
volatile sysStates_t g_sys_status;
int8_t g_channel_incr[NUM_CHANNELS] = {PWM_INCR, PWM_INCR, PWM_INCR};
uint16_t g_channel_target_vals[NUM_CHANNELS] = {MIN_TIMER_VAL, MIN_TIMER_VAL, MIN_TIMER_VAL};
uint16_t g_channel_next_pwm_target[NUM_CHANNELS] = {MIN_TIMER_VAL, MIN_TIMER_VAL, MIN_TIMER_VAL};
uint16_t g_raw_adc10_vals[NUM_CHANNELS] = {0, 0, 0};


/*
 * The _system_pre_init() function will run before the global variables are initialized.
 * It's a good place to stop the watchdog and set the system freq, etc...
 * Just don't do anything that requires global variables yet!
 * The 'return 1' is necessary to init globals on function exit.
 */
int _system_pre_init(void)
{
    WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer

    DCOCTL = 0;
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;			// Set clock to 16MHz

	return 1;
}
int main(void)
{
    configPort1();					// Configure Port1.
    configAdc10();					// Configure the ADC10.
    configWDT();					// Configure the watchdog timer.
    __delay_cycles(16384);			// Force an offset between TA0 and WDT rollovers as they are both sourced from SMCLK.
    configTimerA();					// Configure TimerA0.

    g_sys_status.operating_mode = 1;// 'Random' mode.
    g_sys_status.button_state = 1;	// It's pulled up.

    initTimerVals();
    __enable_interrupt();
    for (;;)						// Main Loop
    {
    	if (g_sys_flags.update_pwm)
    	{
    		if (g_sys_status.operating_mode)
    			handlePwmUpdate();
    		g_sys_flags.update_pwm = 0;
    	}

    	if (g_sys_flags.adc_done)
    	{
    		updateTargetVals();
    		g_sys_flags.adc_done = 0;
    	}

    	if (g_sys_flags.button_press)
    	{
    		handleButtonPress();
    		g_sys_flags.button_press = 0;
    	}

    	if (g_sys_flags.flags == 0)
    		__bis_SR_register(LPM0_bits | GIE);
    }

    return 0;
}


/*
 * Configure Port 1:
 * - P1.1: TA0.0 output.  [LED1]
 * - P1.4: TA0.2 output.
 * - P1.6: TA0.1 output.  [LED2]
 * - P1.3: input.		  [button]
 */
static inline void configPort1(void)
{
	P1DIR = BIT6 | BIT4 | BIT1;	// P1.6, P1.4, P1.1 output.
	P1OUT = BIT3;				// Pullup resistor on P1.3.
	P1REN = BIT3;				// Enable resistor on P1.3.
	P1SEL = BIT6 | BIT4 | BIT1; // P1.6, P1.1 TA0.0, TA0.1, TA0.2(part 1) select.
	P1SEL2 = BIT4;				// P1.4 TA0.2(part 2) select.
}

/*
 * Set the WDT in:
 * - Interval Mode
 * - SMCLK source
 * - /32768 [yields ~2ms interval at SMCLK = 16MHz]
 * - WDT start
 * Enable the WDT interrupt.
 * Start the timer.
 */
static inline void configWDT(void)
{
	IFG1 &= ~WDTIFG;
	WDTCTL = WDTPW | WDTTMSEL | WDTCNTCL;
	IE1 = WDTIE;
}

/*
 * Configure TimerA0:
 * - SMCLK source, divider /1.
 * - Continuous mode.
 * - No interrupts.
 *
 * - Output mode = set. [001]
 * - TAIFG interrupt enabled [TAR overflow].
 */
static inline void configTimerA(void)
{
	TA0CTL = 0;	// Stop timerA0
	TA0CCTL0 = OUTMOD_1;
	TA0CCTL1 = OUTMOD_1;
	TA0CCTL2 = OUTMOD_1;
	TA0CTL = TASSEL_2 | MC_2 | TACLR | TAIE;
}

/*
 * Configure the ADC10:
 * - SMCLK source, divider /8 [~2MHz]
 * - Channel = Temperature sensor
 * - SW start of conversion
 *
 * - Internal Reference
 * - 64 clocks SHT [~32us/sample]
 * - REF = 1.5V
 * - ADC10 on
 * - ADC10 interrupt enable
 */
static inline void configAdc10(void)
{
	ADC10DTC0 = 0;
	ADC10DTC1 = NUM_CHANNELS;
	ADC10CTL1 = INCH_10 | ADC10DIV_7 | ADC10SSEL_3 | CONSEQ_2;
	ADC10CTL0 = SREF_1 | ADC10SHT_3 | MSC | REFON | ADC10ON | ADC10IE | ENC | ADC10SC;
	ADC10SA = (unsigned int)&g_raw_adc10_vals[0];
}


void initTimerVals(void)
{
	g_channel_incr[0] = -PWM_INCR;
	g_channel_incr[1] = -PWM_INCR;
	g_channel_incr[2] = -PWM_INCR;
	TA0CCR0 = MAX_TIMER_VAL;
	TA0CCR1 = MAX_TIMER_VAL;
	TA0CCR2 = MAX_TIMER_VAL;
}


/*
 * Update the Timer0 CCRs to shift the PWM point.
 */
static inline void updatePwm_mode_upDn(volatile uint16_t * ta0_ccr_reg, int8_t * incr)
{
	register uint16_t tmp_ccr;

	tmp_ccr = *ta0_ccr_reg;
	tmp_ccr += *incr;
	if (tmp_ccr <= MIN_TIMER_VAL)
	{
		tmp_ccr = MIN_TIMER_VAL;
		*incr = PWM_INCR;
	}
	else if (tmp_ccr >= MAX_TIMER_VAL)
	{
		tmp_ccr = MAX_TIMER_VAL;
		*incr = -PWM_INCR;
	}

	*ta0_ccr_reg = tmp_ccr;
}


static inline void updatePwm_mode_rnd(	volatile uint16_t * ta0_ccr_reg,
									int8_t * incr,
									uint16_t * next_target_pwm,
									uint16_t * target_data_src)
{
	register uint16_t tmp_ccr;

	tmp_ccr = *ta0_ccr_reg;
	tmp_ccr += *incr;

	if ( (tmp_ccr >= *next_target_pwm) && (*incr > 0) ||
		 (tmp_ccr <= *next_target_pwm) && (*incr < 0) )			// Time to change PWM target.
	{
		tmp_ccr = *next_target_pwm;
		*next_target_pwm = *target_data_src;

		// Guard against going beyond the timer bounds:
		if (*next_target_pwm >= MAX_TIMER_VAL)
			*next_target_pwm = MAX_TIMER_VAL;

		else if (*next_target_pwm <= MIN_TIMER_VAL)
			*next_target_pwm = MIN_TIMER_VAL;

		// Set the incrementer direction.
		if (*next_target_pwm < tmp_ccr)
			*incr = -PWM_INCR;
		else if (*next_target_pwm > tmp_ccr)
			*incr = PWM_INCR;
	}

	if (tmp_ccr >= MAX_TIMER_VAL)
		tmp_ccr = MAX_TIMER_VAL;

	else if (tmp_ccr <= MIN_TIMER_VAL)
		tmp_ccr = MIN_TIMER_VAL;

	*ta0_ccr_reg = tmp_ccr;
}


static inline void handlePwmUpdate(void)
{
	if (g_sys_status.operating_mode == 1 || g_sys_status.operating_mode == 2)
	{
		updatePwm_mode_rnd(&TA0CCR0,
				&g_channel_incr[0],
				&g_channel_next_pwm_target[0],
				&g_channel_target_vals[0]);

		updatePwm_mode_rnd(&TA0CCR1,
				&g_channel_incr[1],
				&g_channel_next_pwm_target[1],
				&g_channel_target_vals[1]);

		updatePwm_mode_rnd(&TA0CCR2,
				&g_channel_incr[2],
				&g_channel_next_pwm_target[2],
				&g_channel_target_vals[2]);
	}
	else if (g_sys_status.operating_mode == 3)
	{
		updatePwm_mode_upDn(&TA0CCR0, &g_channel_incr[0]);
		updatePwm_mode_upDn(&TA0CCR1, &g_channel_incr[1]);
		updatePwm_mode_upDn(&TA0CCR2, &g_channel_incr[2]);
	}
	// Otherwise do nothing.
}

/*
 * Function used to update the PWM target values.
 * It is run when the ADC10 has finished a block
 * of conversions.
 */
static inline void updateTargetVals(void)
{
	register uint8_t i;

	for (i=0; i<NUM_CHANNELS; i++)
	{
		g_channel_target_vals[i] <<= 1;
		g_channel_target_vals[i] |= (g_raw_adc10_vals[i] & 0x0001);
	}

	ADC10SA = (unsigned int)&g_raw_adc10_vals[0];			// Start the next conversions.
}


static inline void handleButtonPress(void)
{
	switch (++g_sys_status.operating_mode)
	{
		case 0:			// Full on mode.
			TA0CTL &= ~TAIE;	// Turn TA0 overflow interrupt off.
			while (TA0IV);		// Clear pending TA0 overflow interrupt flag(s).
			TA0CCTL0 = OUTMOD_0 | OUT;
			TA0CCTL1 = OUTMOD_0 | OUT;
			TA0CCTL2 = OUTMOD_0 | OUT;
			break;
		case 1:			// 'Random' mode.
			g_channel_next_pwm_target[0] = g_channel_target_vals[0];
			g_channel_next_pwm_target[1] = g_channel_target_vals[1];
			g_channel_next_pwm_target[2] = g_channel_target_vals[2];
			initTimerVals();
			TA0CCTL0 = OUTMOD_1;
			TA0CCTL1 = OUTMOD_1;
			TA0CCTL2 = OUTMOD_1;
			while (TA0IV);		// Clear pending TA0 overflow interrupt flag(s).
			TA0CTL |= TAIE;		// Turn TA0 overflow interrupt on.
			break;
		case 2:			// 'Random fast' mode.
			g_channel_next_pwm_target[0] = g_channel_target_vals[0];
			g_channel_next_pwm_target[1] = g_channel_target_vals[1];
			g_channel_next_pwm_target[2] = g_channel_target_vals[2];
			initTimerVals();
			g_channel_incr[0] *= RND_FAST_MULT;
			g_channel_incr[1] *= RND_FAST_MULT;
			g_channel_incr[2] *= RND_FAST_MULT;
			break;
		case 3:			// Up/down mode.
			initTimerVals();
			break;
		default:
			__never_executed();
	}
}

/* Interrupt Routines */
// Watchdog interrupt service routine - handles WDT overflow [interval mode].
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	const uint16_t but_down_patt = 0x8000;
	const uint16_t but_up_patt = 0x7fff;
	static uint16_t button_history = 0xffff;

	register uint8_t wake;

	// Update the button history and check the button state.
	button_history <<= 1;
	button_history |= ((P1IN & BIT3) ? 1 : 0);
	if ( (button_history == but_down_patt) && (g_sys_status.button_state) )
	{
		g_sys_status.button_state = 0;
		g_sys_flags.button_press = 1;
		wake = 1;
	}
	else if ( (button_history == but_up_patt) && (g_sys_status.button_state == 0) )
		g_sys_status.button_state = 1;

	if (++g_sys_status.pwm_tick_count == 0)
	{
		g_sys_flags.update_pwm = 1;
		wake = 1;
	}

	if (wake)
		__bic_SR_register_on_exit(LPM0_bits);
}

// Timer0_A1 interrupt service routine - handles TA0 overflow.
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{
	if (g_sys_status.operating_mode)
	{
		// First reset all of the PWM outputs.
		TA0CCTL0 = OUTMOD_0;
		TA0CCTL1 = OUTMOD_0;
		TA0CCTL2 = OUTMOD_0;

		// Then arm them again ['set' mode].
		TA0CCTL0 = OUTMOD_1;
		TA0CCTL1 = OUTMOD_1;
		TA0CCTL2 = OUTMOD_1;
	}

	// Now read TA0IV to clear the interrupt.
	TA0IV;
}

// ADC10 interrupt service routine
// Will fire once all three <g_raw_adc_vals> are filled.
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
	g_sys_flags.adc_done = 1;
	__bic_SR_register_on_exit(LPM0_bits);
}
