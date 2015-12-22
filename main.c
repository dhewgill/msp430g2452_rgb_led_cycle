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
static inline void configPort1(void);
static inline void configTimerA(void);
static inline void configWDT(void);
static inline void updatePwm(volatile uint16_t * ta0_ccr_reg, int8_t * incr);


/* Type Definitions */


/* Global Variables */
const uint16_t G_MIN_TIMER_VAL = 128;
const uint16_t G_MAX_TIMER_VAL = 65408;


volatile uint8_t g_sys_flags;
int8_t g_channel_incr[NUM_CHANNELS] = {8, 8, 8};


int main(void) {
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
    	if (g_sys_flags & SYSFLG_UPDATE_PWM)
    	{
    		updatePwm(&TA0CCR0, g_channel_incr[0]);
    		updatePwm(&TA0CCR1, g_channel_incr[1]);
    		updatePwm(&TA0CCR2, g_channel_incr[2]);
    		g_sys_flags &= ~SYSFLG_UPDATE_PWM;
    	}

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


/* Interrupt Routines */
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	static uint8_t wake;

	if (++wake & 0x04)							// Wake up every 4 counts; ~8.192ms.  Allows ~2 pwm cycles.
	{
		g_sys_flags |= SYSFLG_UPDATE_PWM;
		__bic_SR_register_on_exit(LPM0_bits);
	}
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
