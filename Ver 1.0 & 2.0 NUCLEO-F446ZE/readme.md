# Version 1.0 & 2.0: Using the STM32 ARM NUCLEO F446ZE MCU and Pmod MIC3 Microphone

The initial microphone our team tried using for this project is:
- [**Pmod MIC3 microphone module**](https://digilent.com/reference/pmod/pmodmic3/start)
  - ![Pmod MIC3 Photo](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/Pmod_MIC3.jpg)
  - Has 12-bit A/D converter included
  - Adjust volume with on-board potentiometer
  - SPI interface to read audio data

Two different methods were tested to see if we could read and output audible audio from the Pmod MIC3:

## Code Structure
- _**FinalProject342.ioc**_: STM32CubeMX Project File that stores the configuration of the Nucleo 446ZE
  - Open with [STM32CubeMX IDE](https://www.st.com/en/development-tools/stm32cubemx.html) initialization code generator
- _**Core Folder**_: holds the main embedded C code for Nucleo 446ZE board
  - _**Core/Src/main.c**_: main code here
- _**.cproject & .project**_: Project file for whole code repo
  - Open with [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) Software
- _**DE1 Folder**_: Contains Verilog code for DE1 SOC FPGA Board to recieve audio data through GPIO pins (For Version 2.0)


## Version 1.0: Reading data from Pmod MIC3 microphone module using Nucleo 446ZE Board

1. Used **STM32CubeMX IDE** to configure Nucleo 446ZE board required pins/timers
   - ![CUBE MX IDE](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/CUBE_MX.jpg)
   - GPIO
   - USART (115200 baud rate)
   - SPI (8 bit data size)
   - Generated the main.c code

2. Implemented main.c to read 16 bits of audio data from Pmod MIC3
```
Core/Src/main.c

...
HAL_GPIO_WritePin(SPI_SNSS_GPIO_Port, SPI_SNSS_Pin, GPIO_PIN_RESET); // NSS1 low
value |= get_spi_byte();
value |= get_spi_byte() << 8;
HAL_GPIO_WritePin(SPI_SNSS_GPIO_Port, SPI_SNSS_Pin, GPIO_PIN_SET);  // NSS1 low
```

![Nucleo Microphone Setup](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/nucleo_mic.jpg)

**Problem faced:** the Pmod MIC3 microphone was too low quality, voices were not audible. Most data was noise no matter the volume.

**Solution #1:** Try to read more bits of data in parallel to see if audio data is audible. Therefore proposed idea was to read data from the microphone using SPI with the Nucleo board, then **send audio data bits in parallel to Altera DE1 SoC FPGA using GPIO pins.**




## Version 2.0: Read audio from Pmod MIC3 using SPI on Nucleo 466ZE, then send data to DE1 SoC FPGA using GPIO pins for playback
GPIO pins on Nucleo 446ZE and DE1 SOC boards were tested. 16 audio data bits from the Nucleo were connected to the DE1 using available GPIO pins.

C code for Nucleo 446ZE to read microphone data and send in parallel 16 bits through GPIO pins to FPGA
```
Core/Src/main.c

...
while(vindex <= NUM_VALS)
{
    values[vindex++] = get_spi_word();
}
...
for (int i = 0; i < NUM_VALS -4; i += 8)
{				
    HAL_GPIO_WritePin(AUDIO_WRITE_AND_ENABLE_GPIO_PORT, AUDIO_WRITE, GPIO_PIN_SET);

    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_0_3_PORT, AUDIO_GPIO_0, values[i] & 0x01 );
    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_0_3_PORT, AUDIO_GPIO_1, (values[i] & 0x02) >> 1 );
    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_0_3_PORT, AUDIO_GPIO_2, (values[i] & 0x04) >> 2);
    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_0_3_PORT, AUDIO_GPIO_3, (values[i] & 0x08) >> 3);

    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_4, (values[i] & 0x10) >> 4);
    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_5, (values[i] & 0x20) >> 5);
    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_6, (values[i] & 0x40) >> 6);
    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_7, (values[i] & 0x80) >> 7);

    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_4_8_PORT, AUDIO_GPIO_8, (values[i] & 0x100) >> 8);
    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_9_11_PORT, AUDIO_GPIO_9, (values[i] & 0x200) >> 9);
    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_9_11_PORT, AUDIO_GPIO_10, (values[i] & 0x400) >> 10);
    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_9_11_PORT, AUDIO_GPIO_11, (values[i] & 0x800) >> 11);

    HAL_GPIO_WritePin(AUDIO_GPIO_PIN_12_PORT, AUDIO_GPIO_12, (values[i] & 0x1000) >> 12);
    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_13_14_PORT, AUDIO_GPIO_13, (values[i] & 0x2000) >> 13);
    HAL_GPIO_WritePin(AUDIO_GPIO_PINS_13_14_PORT, AUDIO_GPIO_14, (values[i] & 0x4000) >> 14);
    HAL_GPIO_WritePin(AUDIO_GPIO_PIN_15_PORT, AUDIO_GPIO_15, (values[i] & 0x8000) >> 15);

    HAL_Delay(1000);
    HAL_GPIO_WritePin(AUDIO_WRITE_AND_ENABLE_GPIO_PORT, AUDIO_WRITE, GPIO_PIN_RESET);
}
```
![Nucleo and FPGA](https://github.com/mayawarrier/ece342-final-project/blob/main/Documents%2C%20Manuals%20and%20Proposal/Photos/nucleo_de1_gpio.jpg)

**Problem faced:** Audio was still not audible, and FPGA board doesn’t have a fast enough clock speed to read from Pmod MIC3.

**Solution #2:** Use Verilog program to read and output audio data using DE1 SoC FPGA (migrate from Nucleo 446ZE board to FPGA because FPGA has audio controller and CODEC)
