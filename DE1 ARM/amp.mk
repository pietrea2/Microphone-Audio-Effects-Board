# Created by the Intel FPGA Monitor Program
# DO NOT MODIFY

############################################
# Global Macros

DEFINE_COMMA			:= ,

############################################
# Compilation Macros

# Programs
AS		:= arm-altera-eabi-as.exe
CC		:= arm-altera-eabi-gcc.exe
LD		:= arm-altera-eabi-ld.exe
OC		:= arm-altera-eabi-objcopy.exe
RM		:= rm -f

# Flags
USERCCFLAGS	:= -g -O0
USERLDFLAGS	:= 
ARCHASFLAGS	:= -mfloat-abi=soft -march=armv7-a -mcpu=cortex-a9 --gstabs -I "$$GNU_ARM_TOOL_ROOTDIR/arm-altera-eabi/include/"
ARCHCCFLAGS	:= -mfloat-abi=soft -march=armv7-a -mtune=cortex-a9 -mcpu=cortex-a9
ARCHLDFLAGS	:= --defsym arm_program_mem=0x40 --defsym arm_available_mem_size=0x3fffffb8 --defsym __cs3_stack=0x3ffffff8 --section-start .vectors=0x0
ARCHLDSCRIPT	:= -T"C:/intelFPGA_lite/22.1std/University_Program/Monitor_Program/build/altera-socfpga-hosted-with-vectors.ld"
ASFLAGS		:= $(ARCHASFLAGS)
CCFLAGS		:= -Wall -c $(USERCCFLAGS) $(ARCHCCFLAGS)
LDFLAGS		:= $(patsubst %, -Wl$(DEFINE_COMMA)%, $(ARCHLDFLAGS)) $(ARCHLDSCRIPT) $(USERLDFLAGS)
OCFLAGS		:= -O srec

# Files
HDRS		:= defines.h interrupt_ID.h address_map_arm.h # globals.h
SRCS		:= audio_interrupt.c exceptions.c audio_ISR.c # MPcore_private_timer_ISR.c globals.c
OBJS		:= $(patsubst %, %.o, $(SRCS))

############################################
# GDB Macros

############################################
# System Macros

# Programs
SYS_PROG_QP_PROGRAMMER	:= quartus_pgm.exe
SYS_PROG_QP_HPS			:= quartus_hps.exe
SYS_PROG_SYSTEM_CONSOLE	:= system-console.exe
SYS_PROG_NII_GDB_SERVER	:= nios2-gdb-server.exe

# Flags
SYS_FLAG_CABLE			:= -c "DE-SoC [USB-1]"
SYS_FLAG_USB			:= "USB-1"
SYS_FLAG_JTAG_INST		:= --instance
SYS_FLAG_NII_HALT		:= --stop

# Files
SYS_FILE_SOF			:= "C:/intelFPGA_lite/22.1std/University_Program/Computer_Systems/DE1-SoC/DE1-SoC_Computer/verilog/DE1_SoC_Computer.sof"
SYS_SCRIPT_JTAG_ID		:= --script="C:/intelFPGA_lite/22.1std/University_Program/Monitor_Program/bin/jtag_instance_check.tcl"
SYS_FILE_ARM_PL			:= --preloader "C:/intelFPGA_lite/22.1std/University_Program/Monitor_Program/arm_tools/u-boot-spl.de1-soc.srec"
SYS_FLAG_ARM_PL_ADDR	:= --preloaderaddr 0xffff13a0

############################################
# Compilation Targets

COMPILE: audio_interrupt.srec

audio_interrupt.srec: audio_interrupt.axf
	$(RM) $@
	$(OC) $(OCFLAGS) $< $@

audio_interrupt.axf: $(OBJS)
	$(RM) $@
	$(CC) $(OBJS) $(LDFLAGS) -o $@

%.c.o: %.c $(HDRS)
	$(RM) $@
	$(CC) $(CCFLAGS) $< -o $@

%.s.o: %.s $(HDRS)
	$(RM) $@
	$(AS) $(ASFLAGS) $< -o $@

CLEAN: 
	$(RM) audio_interrupt.srec audio_interrupt.axf $(OBJS)

############################################
# System Targets

DETECT_DEVICES:
	$(SYS_PROG_QP_PROGRAMMER) $(SYS_FLAG_CABLE) --auto

ARM_RUN_PRELOADER:
	$(SYS_PROG_QP_HPS) $(SYS_FLAG_CABLE) -o GDBSERVER -gdbport0 $(SYS_ARG_GDB_PORT) $(SYS_FILE_ARM_PL) $(SYS_FLAG_ARM_PL_ADDR) 

DOWNLOAD_SYSTEM:
	$(SYS_PROG_QP_PROGRAMMER) $(SYS_FLAG_CABLE) -m jtag -o P\;$(SYS_FILE_SOF)@$(SYS_ARG_JTAG_INDEX) 

CHECK_JTAG_ID:
	$(SYS_PROG_SYSTEM_CONSOLE) $(SYS_SCRIPT_JTAG_ID) $(SYS_FLAG_USB) $(SYS_FILE_SOF) 

HALT_NII:
	$(SYS_PROG_NII_GDB_SERVER) $(SYS_FLAG_CABLE) $(SYS_FLAG_JTAG_INST) $(SYS_ARG_JTAG_INDEX) $(SYS_FLAG_NII_HALT) 

