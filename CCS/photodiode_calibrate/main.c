#include <msp430.h> 
/*
 * main.c
 */


void OA0_Config(void);	// configure OA0
void Start_OA0_Slow(void);	// start OA0 (in slow slew rate)
void Start_OA0_Fast(void);	// start OA0 (in fast slew rate)
void Stop_OA0(void);		// stop OA0 from any slew rate



void main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
	
    if( CALBC1_1MHZ == 0xFF && CALDCO_1MHZ == 0xFF )// Do not run if cal values
     {
       while(1)
       {
         __delay_cycles(65000);
       }
     }

    BCSCTL1 = CALBC1_1MHZ;                    // Set DCO = 1MHz
    DCOCTL = CALDCO_1MHZ;

    OA0_Config();	// configure OA0
    Start_OA0_Fast();	// start OA0 (in fast slew rate)

    while (1){}		// infinite loop

}



/*
 * OA_Func_Set.c
 *
 *  Created on: 16 Nov 2014
 *      Author: Ralph S-J
 * Functions for reading op-amps
 */
void OA0_Config(void)	// configure OA0 and start op-amp
{
    OA0CTL0 = OAN_1 | OAP_0;			//-ive input = OA0I1, +ive input = OA0I0, output channels O:A1,A3,A5
    ADC10AE0 = BIT0 + BIT1 + BIT2;		// route output to A1 (p2.1) and enable inputs.  Without this line the pins are not enabled.
}

void Start_OA0_Slow(void)	// start op amp (in slow slew rate)
{
	OA0CTL0 |= OAPM_1;		// start op-amp in slow slew rate mode (lowest power consumption
}

void Start_OA0_Fast(void)	// start op amp (in slow slew rate)
{
	OA0CTL0 |= OAPM_3;		// start op-amp in fast slew rate mode (highest power consumption
}

void Stop_OA0(void)
{
	OA0CTL0 &= ~OAPM_3;		// stops op-amp by clearing bits form slew rate selector (will turn op-amp off form fast. medium or slow mode)
}
