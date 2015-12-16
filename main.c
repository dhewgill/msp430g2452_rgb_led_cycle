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


/* Function Prototypes */
static inline void configTimerA(void);
static inline void configWDT(void);


/* Type Definitions */


/* Global Variables */
volatile int8_t channel_incr[NUM_CHANNELS] = {1, 1, 1};
volatile uint16_t* const ta0_pwm_regs[NUM_CHANNELS] = {&TA0CCR0, &TA0CCR1, &TA0CCR2};


int main(void) {
    WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer

    DCOCTL = 0;
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;			// Set clock to 16MHz
	
    TA0CCR0;

    while (1)						// Main Loop
    {


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

static inline void configTimerA(void)
{

}



/* Interrupt Routines */
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	uint16_t i;

	for (i=0; i<NUM_CHANNELS; i++)
	{
		if (*ta0_pwm_regs[i] == 0xffff)
			channel_incr[i] = -1;
		else if (*ta0_pwm_regs[i] == 0)
			channel_incr[i] = 1;

		*ta0_pwm_regs[i] += channel_incr[i];
	}
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{

}
