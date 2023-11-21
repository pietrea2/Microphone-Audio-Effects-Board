# Version 3.0: Using the DE1 SOC FPGA Board

This final working version of the vioce effects board uses the Altera DE1 SoC FPGA to read data from the microphone, and outputs the audio through speakers using modified CODEC settings and ISR (interrupt) functions.

![FPGA Audio Effects Board](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/DE1_mic_2.jpg)

## _**DE1 ARM Folder:**_ Contains Embedded C Code for interfacing with the FPGA audio interface and interrupts
  - **Audio_ARM.amp:** Project File for Intel FPGA Monitor Program that has specification for hardware system and program source files
      - **To upload and run project onto DE1 SOC FPGA:**
      - File->Open Project->Audio_ARM.amp
      - ![Intel FPGA Program IDE](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/Intel_FPGA_Program.jpg)
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

## Audio Effects Settings
- To select next audio effect, press KEY3
- To increase/decrease audio effect (for select) press KEY0 or KEY1
- For Loop Effect: Press KEY0 to Record, Press KEY 1 to Play
- ![Default](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_1_def.jpg): Default
- ![Pitch](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_2_pitch.jpg): Pitch Effect
- ![Pitch High 1](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_3_pitch_h1.jpg): Pitch Effect High 1
- ![Pitch High 2](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_4_pitch_h2.jpg): Pitch Effect High 2
- ![Tremolo](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_5_tremelo.jpg): Tremolo Effect
- ![Tremolo High 1](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_6_tremelo_h1.jpg): Tremolo Effect High 1
- ![Tremolo High 2](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_7_tremelo_h2.jpg): Tremolo Effect High 2
- ![Tremolo Low 1](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_8_tremelo_L1.jpg): Tremolo Effect Low 1
- ![Tremolo Low 2](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_9_tremelo_L2.jpg): Tremolo Effect Low 2
- ![Echo](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_10_echo.jpg): Echo Effect
- ![Echo High](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_11_echo_HI.jpg): Echo Effect High
- ![Echo Low](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_12_echo_LO.jpg): Echo Effect Low
- ![Loop](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_13_loop.jpg): Loop Effect Start
- ![Loop Record](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_14_loop_rec.jpg): Loop Effect Record
- ![Loop Play](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/effect_15_loop_play.jpg): Loop Effect Play
