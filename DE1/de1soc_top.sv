module DE1
(
    // Clock pins
    input            CLOCK_50,

    // Seven Segment Displays
    output[6:0]      HEX0,
    output[6:0]      HEX1,
    output[6:0]      HEX2,
    output[6:0]      HEX3,
    output[6:0]      HEX4,
    output[6:0]      HEX5,

    // Pushbuttons
    input[3:0]       KEY,

    // LEDs
    output[9:0]      LEDR,

    // Slider Switches
    input[9:0]       SW,
	 	 
	 // DE1 audio
	 input				AUD_ADCDAT,
	 inout				AUD_BCLK,
	 inout				AUD_ADCLRCK,
	 inout				AUD_DACLRCK,
	 output				AUD_XCK,
	 output				AUD_DACDAT,
	 
	 // from STM32
	 input[15:0]      AUDIO_IN,
	 input            AUDIO_WR,
	 input            AUDIO_ENABLE,
	 
	 // to STM32
	 output           AUDIO_READY
);
	
Audio_Controller de1_audio
(
	.CLOCK_50(CLOCK_50),
	.reset(!AUDIO_ENABLE),
	
 	.AUD_ADCDAT(AUD_ADCDAT),
 	.AUD_BCLK(AUD_BCLK),
 	.AUD_ADCLRCK(AUD_ADCLRCK),
 	.AUD_DACLRCK(AUD_DACLRCK),
 	.AUD_XCK(AUD_XCK),
 	.AUD_DACDAT(AUD_DACDAT),
	
	.audio_out_allowed(AUDIO_READY),		
	.write_audio_out(AUDIO_WR),
	.left_channel_audio_out(AUDIO_IN),
	.right_channel_audio_out(AUDIO_IN)
	
);
defparam de1_audio.AUDIO_DATA_WIDTH = 16,
			de1_audio.BIT_COUNTER_INIT = 4'd15;

// debug
hex_decoder hex0(.hex_digit(AUDIO_IN[3:0]),.segments(HEX0));	
hex_decoder hex1(.hex_digit(AUDIO_IN[7:4]),.segments(HEX1));
hex_decoder hex2(.hex_digit(AUDIO_IN[11:8]),.segments(HEX2));
hex_decoder hex3(.hex_digit({3'd0, AUDIO_WR}),.segments(HEX3));
hex_decoder hex4(.hex_digit({3'd0, AUDIO_READY}),.segments(HEX4));
hex_decoder hex5(.hex_digit({3'd0, AUDIO_ENABLE}),.segments(HEX5));
	
endmodule