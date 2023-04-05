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
	 input[11:0]      AUDIO_IN,
	 input            AUDIO_WR,
	 input            AUDIO_ENABLE,
	 
	 // to STM32
	 output           AUDIO_READY
);

/*
wire				audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;

// Internal Registers

reg [18:0] delay_cnt;
wire [18:0] delay;

reg snd;


assign AUDIO_READY = audio_out_allowed;

always @(posedge CLOCK_50)
	if(delay_cnt == delay) begin
		delay_cnt <= 0;
		snd <= !snd;
	end else delay_cnt <= delay_cnt + 1;

	

assign delay = {SW[3:0], 15'd3000};

wire [31:0] sound = (SW == 0) ? 0 : snd ? 32'd10000000 : -32'd10000000;



assign left_channel_audio_out	= sound;
assign right_channel_audio_out = sound;

	
Audio_Controller de1_audio
(
	.CLOCK_50(CLOCK_50),
	.reset(~KEY[0]),
	
 	.AUD_ADCDAT(AUD_ADCDAT),
 	.AUD_BCLK(AUD_BCLK),
 	.AUD_ADCLRCK(AUD_ADCLRCK),
 	.AUD_DACLRCK(AUD_DACLRCK),
 	.AUD_XCK(AUD_XCK),
 	.AUD_DACDAT(AUD_DACDAT),
	
	.clear_audio_in_memory(),
	.clear_audio_out_memory(),
	
	.audio_out_allowed(audio_out_allowed),		
	.write_audio_out(audio_out_allowed),
	
	.left_channel_audio_out(left_channel_audio_out),
	.right_channel_audio_out(right_channel_audio_out)
	
);
//defparam de1_audio.AUDIO_DATA_WIDTH = 16,
//			de1_audio.BIT_COUNTER_INIT = 4'd15;

// debug
hex_decoder hex0(.hex_digit(AUDIO_IN[3:0]),.segments(HEX0));	
hex_decoder hex1(.hex_digit(AUDIO_IN[7:4]),.segments(HEX1));
hex_decoder hex2(.hex_digit(AUDIO_IN[11:8]),.segments(HEX2));
hex_decoder hex3(.hex_digit({3'd0, AUDIO_WR}),.segments(HEX3));
hex_decoder hex4(.hex_digit({3'd0, AUDIO_READY}),.segments(HEX4));
hex_decoder hex5(.hex_digit({3'd0, AUDIO_ENABLE}),.segments(HEX5));
*/
	
	
/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire				audio_in_available;
wire		[31:0]	left_channel_audio_in;
wire		[31:0]	right_channel_audio_in;
wire				read_audio_in;

wire				audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;
wire				write_audio_out;

// Internal Registers

reg [18:0] delay_cnt;
wire [18:0] delay;

reg snd;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/


/*****************************************************************************
 *                             Sequential Logic                              *
 *****************************************************************************/

always @(posedge CLOCK_50)
	if(delay_cnt == delay) begin
		delay_cnt <= 0;
		snd <= !snd;
	end else delay_cnt <= delay_cnt + 1;

/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/

assign delay = {SW[3:0], 15'd3000};

wire [31:0] sound = (SW == 0) ? 0 : snd ? 32'd10000000 : -32'd10000000;


assign read_audio_in			= audio_in_available & audio_out_allowed;

assign left_channel_audio_out	= left_channel_audio_in+sound;
assign right_channel_audio_out	= right_channel_audio_in+sound;
assign write_audio_out			= audio_in_available & audio_out_allowed;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Audio_Controller Audio_Controller (
	// Inputs
	.CLOCK_50						(CLOCK_50),
	.reset						(~KEY[0]),

	.clear_audio_in_memory		(),
	.read_audio_in				(read_audio_in),
	
	.clear_audio_out_memory		(),
	.left_channel_audio_out		(left_channel_audio_out),
	.right_channel_audio_out	(right_channel_audio_out),
	.write_audio_out			(write_audio_out),

	.AUD_ADCDAT					(AUD_ADCDAT),

	// Bidirectionals
	.AUD_BCLK					(AUD_BCLK),
	.AUD_ADCLRCK				(AUD_ADCLRCK),
	.AUD_DACLRCK				(AUD_DACLRCK),


	// Outputs
	.audio_in_available			(audio_in_available),
	.left_channel_audio_in		(left_channel_audio_in),
	.right_channel_audio_in		(right_channel_audio_in),

	.audio_out_allowed			(audio_out_allowed),

	.AUD_XCK					(AUD_XCK),
	.AUD_DACDAT					(AUD_DACDAT)

);
	
	
endmodule