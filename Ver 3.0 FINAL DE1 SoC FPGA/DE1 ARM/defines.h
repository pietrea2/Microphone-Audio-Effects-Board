
#ifndef DEFINES_H
#define DEFINES_H

#define	EDGE_TRIGGERED          0x1
#define	LEVEL_SENSITIVE         0x0
#define	CPU0                    0x01
#define	ENABLE                  0x1

#define	USER_MODE               0b10000
#define	FIQ_MODE                0b10001
#define	IRQ_MODE                0b10010
#define	SVC_MODE                0b10011
#define	ABORT_MODE              0b10111
#define	UNDEF_MODE              0b11011
#define	SYS_MODE                0b11111

#define KEY0_MASK               0x1
#define KEY1_MASK               0x2
#define KEY2_MASK               0x4
#define KEY3_MASK               0x8

#define CPSR_IRQ_MASK           0b10000000     
#define	CPSR_IRQ_ENABLE         0b01000000
#define	CPSR_IRQ_DISABLE        0b11000000


void init_interrupts(void);
void config_interrupt(int CPU_targets, int int_ID, int int_priority);
void config_audio_demo(void);

void enable_irq(void);
void disable_irq(void);

#define write_bits(addr, mask, value) \
(*(addr) = ((mask) & *(addr)) | (value))

void audio_ISR(void);
void keys_ISR(void);

#endif