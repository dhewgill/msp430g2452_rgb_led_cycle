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

#define SYSFLG_UPDATE_PWM	0x80

/* Function Prototypes */
static inline void configTimerA(void);
static inline void configWDT(void);
static inline void updatePwm(volatile uint16_t * ta0_ccr_reg, int8_t * incr);


/* Type Definitions */


/* Global Variables */
const uint16_t gMinTimerVal = 128;
const uint16_t gMaxTimerVal = 65408;


volatile uint8_t gSysFlags;
volatile int8_t channel_incr[NUM_CHANNELS] = {8, 8, 8};


int main(void) {
    WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer

    DCOCTL = 0;
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;			// Set clock to 16MHz
	
    configWDT();					// Configure the watchdog timer.
    configTimerA();					// Configure TimerA0.

    while (1)						// Main Loop
    {
    	if (gSysFlags & SYSFLG_UPDATE_PWM)
    	{
    		updatePwm(&TA0CCR0, channel_incr[0]);
    		updatePwm(&TA0CCR1, channel_incr[1]);
    		updatePwm(&TA0CCR2, channel_incr[2]);
    		gSysFlags &= ~SYSFLG_UPDATE_PWM;
    	}

    	__bis_SR_register(LPM0_bits | GIE);
    }


    return 0;
}

/*
 * Set the WDT in:
 * - Interval Mode
 * - SMCLK source
 * - /8192 [yields ~2ms interval at SMCLK = 16MHz]
 * Start the timer.
 */
static inline void configWDT(void)
{
	WDTCTL = WDTPW | WDTTMSEL | WDTCNTCL | WDTIS0;
	WDTCTL = (WDTPW | WDTCTL) & ~WDTHOLD;
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
 * Update the Timer0 CCRs to shift the PWM point.
 */
static inline void updatePwm(volatile uint16_t * ta0_ccr_reg, int8_t * incr)
{
	register uint16_t tmp;

	tmp = *ta0_ccr_reg;
	tmp += *incr;
	if (tmp < gMinTimerVal)
	{
		tmp = gMinTimerVal;
		*incr *= -1;
	}
	else if (tmp > gMaxTimerVal)
	{
		tmp = gMaxTimerVal;
		*incr *= -1;
	}

	*ta0_ccr_reg = tmp;
}


/* Interrupt Routines */
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	uint8_t wake;


	if (wake)
		__bic_SR_register_on_exit(LPM0_bits);
}

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
