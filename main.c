#include <msp430.h>
#include <stdint.h>


/*
 * This is a small program that uses the 3 outputs of TimerA
 * on the MSP430G2452 to drive each output of a RGB LED.
 * The "system tick" is provided by the WDT in interval mod and is
 * set to ~2ms period.
 * MCLK = SMCLK = 16MHz.
 * WDT and TA are sourced from SMCLK.
 */

/* Preprocessor Defines */
#define NUM_CHANNELS 3
#define F_MCLK		16000000ul
#define F_SMCLK		16000000ul

#define SYSFLG_UPDATE_PWM		0x80
#define SYSFLG_ADC_DONE			0x40


/* Function Prototypes */
static inline void configPort1(void);
static inline void configTimerA(void);
static inline void configWDT(void);
static inline void configAdc10(void);
static inline void updatePwm_mode1(volatile uint16_t * ta0_ccr_reg, int8_t * incr);
static inline void updatePwm_mode0(	volatile uint16_t * ta0_ccr_reg,
									int8_t * incr,
									uint16_t * next_target_pwm,
									uint16_t * target_data_src);
static inline void updatePwm0(void);
static inline void updatePwm1(void);
static inline void updatePwm2(void);
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
		uint8_t operating_mode	:2;
		uint8_t pwm_tick_count	:2;
		uint8_t button_state	:1;
		uint8_t unused			:3;
	};
	uint8_t states;
} sysStates_t;


/* Global Variables */
const uint16_t G_MIN_TIMER_VAL = 128;
const uint16_t G_MAX_TIMER_VAL = 65408;

volatile sysFlags_t g_sys_flags;
volatile sysStates_t g_sys_status;
int8_t g_channel_incr[NUM_CHANNELS] = {8, 8, 8};
uint16_t g_channel_target_vals[NUM_CHANNELS] = {G_MIN_TIMER_VAL, G_MIN_TIMER_VAL, G_MIN_TIMER_VAL};
uint16_t g_raw_adc10_vals[NUM_CHANNELS] = {0, 0, 0};

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer

    DCOCTL = 0;
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;			// Set clock to 16MHz
	
    configPort1();					// Configure Port1.
    configWDT();					// Configure the watchdog timer.
    __delay_cycles(16384);			// Force an offset between TA0 and WDT rollovers as they are both sourced from SMCLK.
    configTimerA();					// Configure TimerA0.
    configAdc10();

   g_sys_status.operating_mode = 0;
    while (1)						// Main Loop
    {
    	if (g_sys_flags.update_pwm)
    	{
    		if (g_sys_status.operating_mode == 1)
    		{
				updatePwm_mode1(&TA0CCR0, &g_channel_incr[0]);
				updatePwm_mode1(&TA0CCR1, &g_channel_incr[1]);
				updatePwm_mode1(&TA0CCR2, &g_channel_incr[2]);
    		}
    		else
    		{
				updatePwm0();
				updatePwm1();
				updatePwm2();
    		}
    		g_sys_flags.update_pwm = 0;
    	}

    	if (g_sys_flags.adc_done)
    	{
    		updateTargetVals();
    		g_sys_flags.adc_done = 0;
    	}

    	if (g_sys_flags.button_press)
    	{
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
	P1REN = BIT3;				// Enable resistor on P1.3.
	P1OUT = BIT3;				// Pullup resistor on P1.3.
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
	ADC10SA = &g_raw_adc10_vals[0];
}


/*
 * Update the Timer0 CCRs to shift the PWM point.
 */
static inline void updatePwm_mode1(volatile uint16_t * ta0_ccr_reg, int8_t * incr)
{
	register uint16_t tmp;

	tmp = *ta0_ccr_reg;
	tmp += *incr;
	if (tmp <= G_MIN_TIMER_VAL)
	{
		tmp = G_MIN_TIMER_VAL;
		*incr *= -1;
	}
	else if (tmp >= G_MAX_TIMER_VAL)
	{
		tmp = G_MAX_TIMER_VAL;
		*incr *= -1;
	}

	*ta0_ccr_reg = tmp;
}


static inline void updatePwm_mode0(	volatile uint16_t * ta0_ccr_reg,
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
		if (*next_target_pwm >= G_MAX_TIMER_VAL)
			*next_target_pwm = G_MAX_TIMER_VAL;

		else if (*next_target_pwm <= G_MIN_TIMER_VAL)
			*next_target_pwm = G_MIN_TIMER_VAL;

		// Set the incrementer direction.
		if (*next_target_pwm < tmp_ccr)
			*incr = -8;
		else if (*next_target_pwm > tmp_ccr)
			*incr = 8;
	}

	if (tmp_ccr >= G_MAX_TIMER_VAL)
		tmp_ccr = G_MAX_TIMER_VAL;

	else if (tmp_ccr <= G_MIN_TIMER_VAL)
		tmp_ccr = G_MIN_TIMER_VAL;

	*ta0_ccr_reg = tmp_ccr;
}

static inline void updatePwm0(void)
{
	static uint16_t next_target_pwm = G_MIN_TIMER_VAL;

	updatePwm_mode0(&TA0CCR0, &g_channel_incr[0], &next_target_pwm, &g_channel_target_vals[0]);
}

static inline void updatePwm1(void)
{
	static uint16_t next_target_pwm = G_MIN_TIMER_VAL;

	updatePwm_mode0(&TA0CCR1, &g_channel_incr[1], &next_target_pwm, &g_channel_target_vals[1]);
}

static inline void updatePwm2(void)
{
	static uint16_t next_target_pwm = G_MIN_TIMER_VAL;

	updatePwm_mode0(&TA0CCR2, &g_channel_incr[2], &next_target_pwm, &g_channel_target_vals[2]);
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

	ADC10SA = &g_raw_adc10_vals[0];			// Start the next conversions.
}


static inline void handleButtonPress(void)
{
	// Update the operating mode:
	g_sys_status.operating_mode++;
	if (g_sys_status.operating_mode == 1)
	{
		register int i;
		for (i=0; i<NUM_CHANNELS; i++)
			g_channel_target_vals[i] = G_MAX_TIMER_VAL;
	}
}

/* Interrupt Routines */
// Watchdog interrupt service routine - handles WDT overflow [interval mode].
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	const uint16_t but_down_patt = 0x8000;
	const uint16_t but_up_patt - 0x7fff;
	static uint16_t button_history = 0x0000;
	register uint8_t wake;

	// Check if it's time to wake up to update the PWM.
	if (++g_sys_status.pwm_tick_count == 3)
	{
		g_sys_flags.update_pwm = 1;
		wake = 1;
	}

	// Update the button history and check the button state.
	button_history <<= ((P1IN & BIT3) ? 1 : 0);
	if ( (button_history == but_down_patt) && g_sys_status.button_state )
	{
		g_sys_status.button_state = 0;
		g_sys_flags.button_press = 1;
		wake = 1;
	}
	else if ( (button_history == but_up_patt) && (g_sys_status.button_state > 0) )
		g_sys_status.button_state = 1;

	if (wake)
		__bic_SR_register_on_exit(LPM0_bits);
}

// Timer0_A1 interrupt service routine - handles TA0 overflow.
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{
	// First reset all of the PWM outputs.
	TA0CCTL0 = OUTMOD_0;
	TA0CCTL1 = OUTMOD_0;
	TA0CCTL2 = OUTMOD_0;

	// Then arm them again ['set' mode].
	TA0CCTL0 = OUTMOD_1;
	TA0CCTL1 = OUTMOD_1;
	TA0CCTL2 = OUTMOD_1;

	// Now read TA0IV to get rid of the interrupt.
	TA0IV;
}

// ADC10 interrupt service routine
// Will fire once all three <g_raw_adc_vals> are filled.
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
	g_sys_flags.adc_done = 1;
	__bic_SR_register_on_exit(LPM0_bits);
}
