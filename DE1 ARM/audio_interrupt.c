#include "address_map_arm.h"

void set_A9_IRQ_stack(void);
void config_GIC(void);
//void config_MPcore_private_timer(void);
void config_KEYs(void);
void config_audio(void);
void enable_A9_interrupts(void);

/* these globals are written by interrupt service routines; we declare them as
 * volatile to avoid the compiler caching their values, even in registers */
extern volatile int buffer_index; // used for audio
//extern volatile int timeout; // used to synchronize with the timer

int main(void) {
    buffer_index = 0; // audio playback
    //timeout      = 0;

    set_A9_IRQ_stack();            // initialize the stack pointer for IRQ mode
    config_GIC();                  // configure the general interrupt controller
    //config_MPcore_private_timer(); // configure ARM A9 private timer
    config_KEYs(); // configure pushbutton KEYs to generate interrupts
    config_audio();

    enable_A9_interrupts(); // enable interrupts

    while (1);
}

/* setup private timer in the ARM A9 */
//void config_MPcore_private_timer() {
//    volatile int * MPcore_private_timer_ptr =
//        (int *)MPCORE_PRIV_TIMER; // timer base address
//
//    /* set the timer period */
//    int counter = 20000000; // period = 1/(200 MHz) x 40x10^6 = 0.2 sec
//    *(MPcore_private_timer_ptr) = counter; // write to timer load register
//
//    /* write to control register to start timer, with interrupts */
//    *(MPcore_private_timer_ptr + 2) = 0x7; // int mask = 1, mode = 1, enable = 1
//}

void hw_set_bit(volatile int* ptr, int bit, int value)
{
    if (value)
        *ptr |= 1 << bit;
    else
        *ptr &= ~(1 << bit);
}

void config_audio(void)
{
    volatile int* audio_ptr = (int*)AUDIO_BASE;
    *audio_ptr = 0x3; // enable read/write interrupts
}

void config_KEYs(void) 
{
    volatile int * KEY_ptr = (int *)KEY_BASE;
    *(KEY_ptr + 2) = 0xF; // enable all KEY interrupts
}
