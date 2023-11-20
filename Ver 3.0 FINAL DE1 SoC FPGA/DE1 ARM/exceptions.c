//
// - defines exception vectors for the A9 processor
// - provides utils + GIC initialization code
//

#include <stdint.h>
#include <assert.h>
#include "interrupt_ID.h"
#include "defines.h"
#include "address_map_arm.h"


void __attribute__((interrupt)) __cs3_reset(void)
{
    while (1);
}

void __attribute__((interrupt)) __cs3_isr_undef(void)
{
    while (1);
}

void __attribute__((interrupt)) __cs3_isr_swi(void)
{
    while (1);
}

void __attribute__((interrupt)) __cs3_isr_pabort(void)
{
    while (1);
}

void __attribute__((interrupt)) __cs3_isr_dabort(void)
{
    while (1);
}

// using_gic.pdf
void __attribute__((interrupt)) __cs3_isr_irq(void)
{
    disable_irq();

    // acknowledge interrupt
    int address = MPCORE_GIC_CPUIF + ICCIAR;
    int int_ID = *((int*)address);

    if (int_ID == AUDIO_IRQ)
        audio_ISR();
    else if (int_ID == KEYS_IRQ)
        keys_ISR();
    else
        while (1);

    // clear interrupt
    address = MPCORE_GIC_CPUIF + ICCEOIR;
    *((int*)address) = int_ID;

    enable_irq();
    return;
}

void __attribute__((interrupt)) __cs3_isr_fiq(void)
{
    while (1);
}


void enable_irq(void)
{
    int status;
    asm("mrs %[ps], cpsr" : [ps] "=r"(status) :);
    write_bits(&status, ~CPSR_IRQ_MASK, 0);
    asm("msr cpsr, %[ps]" : : [ps] "r"(status));
}

void disable_irq(void)
{
    int status;
    asm("mrs %[ps], cpsr" : [ps] "=r"(status) :);
    write_bits(&status, ~CPSR_IRQ_MASK, CPSR_IRQ_MASK);
    asm("msr cpsr, %[ps]" : : [ps] "r"(status));
}


// using_gic.pdf
void config_interrupt(int CPU_targets, int int_ID, int int_priority)
{
    int intr_bit, reg_offset, addr;

    assert(int_ID < 1024 && CPU_targets < 256 && int_priority < 256);

    // Set interrupt CPU targets (ICDIPTRn).
    // reg offset = ICPIPTR + integer_div(N, 4) * 4
    // interrupt bit = (N mod 4) * 8
    intr_bit = (int_ID & 0x3) << 3;
    reg_offset = ((int_ID >> 2) << 2);
    addr = MPCORE_GIC_DIST + ICDIPTR + reg_offset;
    write_bits((volatile int*)addr, ~(0xff << intr_bit), CPU_targets << intr_bit);

    // Set interrupt priority (ICDIPRn).
    // reg offset, interrupt bit same as ICDIPTRn
    addr = MPCORE_GIC_DIST + ICDIPR + reg_offset;
    write_bits((volatile int*)addr, ~(0xff << intr_bit), int_priority << intr_bit);

    // Set interrupt enable (ICDISERn).
    // reg offset = ICDISER + integer_div(N, 32) * 4
    // interrupt bit = N mod 32
    intr_bit = int_ID & 0x1f;
    addr = MPCORE_GIC_DIST + ICDISER + ((int_ID >> 5) << 2);
    write_bits((volatile int*)addr, ~(0x1 << intr_bit), 0x1 << intr_bit);
}

// using_gic.pdf
void init_interrupts(void)
{  
    int stack, mode, addr;

    // Initialize banked stack pointer for IRQ mode
    stack = A9_ONCHIP_END - 7;
    mode = CPSR_IRQ_DISABLE | IRQ_MODE;
    asm("msr cpsr, %[ps]" : : [ps] "r" (mode));
    asm("mov sp, %[ps]" : : [ps] "r" (stack));
    mode = CPSR_IRQ_DISABLE | SVC_MODE;
    asm("msr cpsr, %[ps]" : : [ps] "r" (mode));

    // enable interrupts for all priorities
    addr = MPCORE_GIC_CPUIF + ICCPMR;
    *((int*)addr) = 0xFFFF;

    // enable interrupt signalling from CPU interface
    addr = MPCORE_GIC_CPUIF + ICCICR;
    *((int*)addr) = ENABLE;

    // enable interrupt distributor
    addr = MPCORE_GIC_DIST + ICDDCR;
    *((int*)addr) = ENABLE;
}
