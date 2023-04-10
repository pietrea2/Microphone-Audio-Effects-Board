#include "defines.h"

int main(void) 
{
    init_interrupts();
    config_audio_demo();
    enable_irq();

    // wait for interrupts
    while (1);
}
