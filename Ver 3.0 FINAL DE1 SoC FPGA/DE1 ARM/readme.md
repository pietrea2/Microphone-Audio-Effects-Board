# Version 3.0: Using the DE1 SOC FPGA Board

This final working version of the vioce effects board uses the Altera DE1 SoC FPGA to read data from the microphone, and outputs the audio through speakers using modified CODEC settings and ISR (interrupt) functions.

![FPGA Audio Effects Board](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/DE1_mic_2.jpg)

## _**DE1 ARM Folder:**_ Contains Embedded C Code for interfacing with the FPGA audio interface and interrupts
  - **Audio_ARM.amp:** Project File for Intel FPGA Monitor Program that has specification for hardware system and program source files
      - **To upload and run project onto DE1 SOC FPGA:**
      - File->Open Project->Audio_ARM.amp
  - **main.c:** interrupt setup, config and infinite loop
    - ```
      init_interrupts();
      config_audio_demo();
      enable_irq();
      ```
  - **exceptions.c:** defines exception vectors for the A9 processor, provides utils + GIC initialization code
      - Contains definitions for:
      - ```
        void enable_irq(void)
        void config_interrupt(int CPU_targets, int int_ID, int int_priority)
        void init_interrupts(void)
        ```
  - **audio_demo.c:** main code for interfacing with KEYs and audio port registers of FPGA
      - Includes Interrupt Service Routines for KEYs and AUDIO
      - ```
        ...
        enum eff
        {
            EFF_DEFAULT = 0x1,
            EFF_PITCH = 0x2,
            EFF_TREMELO = 0x4,
            EFF_DELAY = 0x8,
            EFF_LOOP = 0x10
        };
        ...
        void config_audio_demo(void)
        void keys_ISR(void)
        void audio_ISR(void)
        ```
     - Within audio_ISR() function, read and write to Audio Port FIFO Registers to read from Line In and write to Line Out
     - ![FPGA Audio Port Registers](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/de1_soc_audio_port_registers.jpg)
     - Simple Example of interfacing with audio port registers of DE1 SOC FPGA:
     - ```
       void audio_ISR(void){
	       volatile int* audio_ptr = (int *)AUDIO_BASE;  //0xFF203040

	       int fifospace;

	       if (*(audio_ptr) & 0x100){ // check read interrupt
	        
		       fifospace = *(audio_ptr + 1);

		       // read until audio-in FIFO is empty
               while ((fifospace & 0x000000FF)){
       
                 int left = *(audio_ptr + 2);
                 int right = *(audio_ptr + 3);
       
                 fifospace = *(audio_ptr + 1);
                }
             }
       
           if (*(audio_ptr) & 0x200){ // check write interrupt
	
		        fifospace = *(audio_ptr + 1);

               // write until buffer is empty or audio-out FIFO is full
              while ((fifospace & 0x00FF0000) && iaudiooutbuf < iaudioinbuf){
       
                  *(audio_ptr + 2) = left_buffer[iaudiooutbuf];
                  *(audio_ptr + 3) = right_buffer[iaudiooutbuf];
                  
                  ++iaudiooutbuf;
                  fifospace = *(audio_ptr + 1);
              }
          }
       }
       ```
  - **sin_LUT.h:** a defined Look Up Table of a sine wave (an array of int32_t) used for creating the Tremolo Effect
    - Generated from the code in the **LUTgen** folder

## _**DE1_Computer/2 Folders**_: Files for describing the DE1-SOC Computer System with NIOS® II
   - Files that are uploaded onto FPGA that describe the system to be used. This computer includes a number of interfaces to input/output devices implemented in the FPGA fabric of the chip
   - _DE1_Computer2/Computer_System/synthesis/submodules/Computer_System_AV_Config.v_
     - Within the above file, the **FPGA's audio CODEC [WM8731](https://cdn.sparkfun.com/datasheets/Dev/Arduino/Shields/WolfsonWM8731.pdf)** was modified for proper sampling rates and settings.
   - Use updated DE1_Computer2 files in Intel FPGA Monitor Program 
