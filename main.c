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
static inline void updatePwm(volatile uint16_t * ta0_ccr_reg, int8_t * incr);
static inline void updateTargetVals(void);


/* Type Definitions */
typedef union
{
	struct
	{
		uint8_t update_pwm		: 1;
		uint8_t adc_done		: 1;
		uint8_t unused			: 6;
	};
	uint8_t flags;
} sysFlags_t;

/* Global Variables */
const uint16_t G_MIN_TIMER_VAL = 128;
const uint16_t G_MAX_TIMER_VAL = 65408;

//volatile uint8_t g_sys_flags;
volatile sysFlags_t g_sys_flags;
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

    while (1)						// Main Loop
    {
    	if (g_sys_flags.update_pwm)
    	{
    		updatePwm(&TA0CCR0, &g_channel_incr[0]);
    		updatePwm(&TA0CCR1, &g_channel_incr[1]);
    		updatePwm(&TA0CCR2, &g_channel_incr[2]);
    		g_sys_flags.update_pwm = 0;
    	}

    	if (g_sys_flags.adc_done)
    	{
    		updateTargetVals();
    		g_sys_flags.adc_done = 0;
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
 */
static inline void configPort1(void)
{
	P1DIR = BIT6 | BIT4 | BIT1;	// P1.6, P1.4, P1.1 output.
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
/*static inline void configAdc10(void)
{
	ADC10DTC0 = 0;
	ADC10DTC1 = NUM_CHANNELS;
	ADC10CTL1 = INCH_10 | ADC10DIV_7 | ADC10SSEL_3 | CONSEQ_2;
	ADC10CTL0 = SREF_1 | ADC10SHT_3 | MSC | REFON | ADC10ON | ADC10IE | ENC | ADC10SC;
	ADC10SA = &g_raw_adc10_vals[0];
}*/


/*
 * Update the Timer0 CCRs to shift the PWM point.
 */
static inline void updatePwm(volatile uint16_t * ta0_ccr_reg, int8_t * incr)
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


/*
 * Function used to update the PWM target values.
 * It is run when the ADC10 has finished a block
 * of conversions.
 * Once the next targets are ready, then shut off
 * the ADC10.
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


/* Interrupt Routines */
// Watchdog interrupt service routine - handles WDT overflow [interval mode].
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	static uint8_t wake;

	if (++wake & 0x04)							// Wake up every 4 counts; ~8.192ms.  Allows ~2 pwm cycles.
	{
		g_sys_flags.update_pwm = 1;
		__bic_SR_register_on_exit(LPM0_bits);
	}
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
