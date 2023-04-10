#include "address_map_arm.h"
extern volatile int timeout;

/* setup private timer in the ARM A9 */
void config_MPcore_private_timer() 
{
    volatile int * MPcore_private_timer_ptr =
        (int *)MPCORE_PRIV_TIMER; // timer base address

    /* set the timer period */
    int counter = 20000000; // period = 1/(200 MHz) x 40x10^6 = 0.2 sec
    *(MPcore_private_timer_ptr) = counter; // write to timer load register

    /* write to control register to start timer, with interrupts */
    *(MPcore_private_timer_ptr + 2) = 0x7; // int mask = 1, mode = 1, enable = 1
}

/*****************************************************************************
 * MPcore private timer interrupt service routine
 *                                                                          
 * Controls refresh of the VGA screen
******************************************************************************/
void MPcore_private_timer_ISR( )
{
	volatile int * MPcore_private_timer_ptr = (int *) MPCORE_PRIV_TIMER;	// private timer address

	*(MPcore_private_timer_ptr + 3) = 1;	// Write to timer interrupt status register to
														// clear the interrupt (note: not really necessary)

	timeout = 1;									// set global variable

	return;
}

