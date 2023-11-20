# UofT ECE342 Final Project: Microphone Audio Effects Board

ECE342 Computer Hardware project created by:
- Maya Warrier
- Adam Pietrewicz

The goal was to interface with 2 hardware peripherals: a microphone and speakers, using embedded systems fundamentals learned in ECE342 to read, interface and output audio data to perform different cool audio effects.
The project development included multiple designs due to the experimentation with different microphones and development boards in order to find the best fit (specifications and requirements) for an audio hardware project.

## Project Development Timeline

### [Version 3.0: Using the ALTERA ARM Cyclone V DE1 SOC FPGA Board](https://github.com/mayawarrier/ece342-final-project/tree/main/Ver%203.0%20FINAL%20DE1%20SoC%20FPGA)
The final version of the project was developed using the following hardware:
- [ARM DE1 SOC FPGA Board](https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&No=836)
- [Yanmai SF-910 Condenser Microphone](https://www.canadacomputers.com/product_info.php?cPath=35_920_922&item_id=121777&review=all)
- [Bose Companion 2 Series III Multimedia Speakers](https://www.bose.ca/en/p/speakers/bose-companion-2-series-iii-multimedia-speaker-system/C2III-SPEAKERCOMPUTR.html)

This project was developed in C for the ARM DE1 SOC FPGA to read data from the microphone and output the audio through speakers.

The **FPGA's audio CODEC [WM8731]**(https://cdn.sparkfun.com/datasheets/Dev/Arduino/Shields/WolfsonWM8731.pdf) was modified for proper sampling rates and settings.
```
Ver 3.0 FINAL DE1 SoC FPGA/DE1_Computer2/Computer_System/synthesis/submodules/Computer_System_AV_Config.v

localparam AUD_LINE_IN_LC 	= 9'h018; //9'h01A;
localparam AUD_LINE_IN_RC 	= 9'h018; //9'h01A;
localparam AUD_LINE_OUT_LC	= 9'h077; //9'h07B; 
localparam AUD_LINE_OUT_RC	= 9'h077; //9'h07B;
localparam AUD_ADC_PATH		= 9'd157; //9'h15; //d157;
localparam AUD_DAC_PATH	  	= 9'h0; // 6
localparam AUD_POWER		= 9'h000;
localparam AUD_DATA_FORMAT	= 9'd77;
localparam AUD_SAMPLE_CTRL	= 9'd0;   //9'd12
localparam AUD_SET_ACTIVE 	= 9'h001;
```

To interface with the DE1 SOC audio port registers, the **Generic Interrupt Controller (GIC)** was used, amd interrupts were modified and configured to service KEY and AUDIO interrupts:
```
Ver 3.0 FINAL DE1 SoC FPGA/DE1 ARM/audio_demo.c:

void config_audio_demo(void){
...
config_interrupt(CPU0, KEYS_IRQ, 0);
config_interrupt(CPU0, AUDIO_IRQ, 0);
...
}
```

Two **Interrupt Handlers** were created: **keys_ISR(), audio_ISR()** to service the user keys for interfacing with the audio effects board (selecting effect), for servicing microphone input data from the FIFO audio register (read data), and for outputing the audio to the speakers by writing to the FIFO audio Write registers.
```
Ver 3.0 FINAL DE1 SoC FPGA/DE1 ARM/audio_demo.c:

void config_audio_demo(void)
void keys_ISR(void)
void audio_ISR(void)
```

**The user can choose from the following Audio Effects using the KEY buttons:**
- **Default**
- **Pitch:**   Increases the pitch of your voice (2 levels)
- **Tremelo:** A trembling effect that fluctuates the volume of your voice (2 levels)
- **Delay:**   Hear an echo of your voice, you can shorten or elongate the delay effect
- **Loop:**    Press the record button to record short layers of sounds, and all layers are stacken on top of each other. Create a beat!



### [Version 1.0 & 2.0: Using the STM32 ARM NUCLEO F446ZE MCU and Digilent Pmod MIC3 Microphone](https://github.com/mayawarrier/ece342-final-project/tree/main/Ver%201.0%20%26%202.0%20NUCLEO-F446ZE)
The initial version of this project was to interface with the [**Pmod MIC3 microphone module**](https://digilent.com/reference/pmod/pmodmic3/start) using the [**Nucleo F446ZE MCU**](https://www.st.com/en/evaluation-tools/nucleo-f446ze.html) because it was the developemnt board used for the course instruction.

With the help of the university lab manager, our team found the Pmod MIC3 microphone was an easy to use module to get started with. It uses **SPI communication** to read audio data from the on-board microphone.
The module includes its own ADC. Therefore this module provides an easy interface to retrieve audio data using the Nucleo 446ZE.

#### Version 1.0: Example of reading audio data using the Nucleo 446ZE:
```
Ver 1.0 & 2.0 NUCLEO-F446ZE/Core/Src/main.c

...
HAL_GPIO_WritePin(SPI_SNSS_GPIO_Port, SPI_SNSS_Pin, GPIO_PIN_RESET); // NSS1 low
value |= get_spi_byte();
value |= get_spi_byte() << 8;
HAL_GPIO_WritePin(SPI_SNSS_GPIO_Port, SPI_SNSS_Pin, GPIO_PIN_SET);  // NSS1 low
```

- **Problem faced:** the Pmod MIC3 microphone was too low quality, voices were not audible. Most data was noise no matter the volume.

- **Solution #1:** Try to read more bits of data in parallel to see if audio data is audible. Therefore proposed idea was to read data from the microphone using SPI with the Nucleo board, then **send audio data bits in parallel to Altera DE1 SoC FPGA using GPIO pins.**

#### Version 2.0: Read audio from Pmod MIC3 using SPI on Nucleo 466ZE, then send data to DE1 SoC FPGA using GPIO pins for playback
GPIO pins on Nucleo 446ZE and DE1 SOC boards were tested. 16 audio data bits from the Nucleo were connected to the DE1 using available GPIO pins.

- **Problem faced:** Audio was still not audible, and FPGA board doesn’t have a fast enough clock speed to read from Pmod MIC3.

- **Solution #2:** Use Verilog program to read and output audio data using DE1 SoC FPGA (migrate from Nucleo 446ZE board to FPGA because FPGA has audio controller and CODEC)

