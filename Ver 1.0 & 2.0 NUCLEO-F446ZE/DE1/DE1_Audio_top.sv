
module DE1_Audio_FinalProject342 (
// Inputs
	input				CLOCK_50,
	input		[3:0]	KEY,
	input		[3:0]	SW,
	
	output   [6:0] HEX0,
	output   [6:0] HEX1,
	output   [6:0] HEX2,
	output   [6:0] HEX3,
	output   [6:0] HEX4,
	output   [6:0] HEX5,
	
	input				AUD_ADCDAT,
	
	// Bidirectionals
	inout				AUD_BCLK,
	inout				AUD_ADCLRCK,
	inout				AUD_DACLRCK,
	
	inout				FPGA_I2C_SDAT,
	
	// Outputs
	output			AUD_XCK,
	output			AUD_DACDAT,
	
	output			FPGA_I2C_SCLK,
	
	// from STM32
	input		[15:0] GPIO_0,
	input     STM_AUDIO_WR,
	output    STM_AUDIO_READY
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/
localparam IN_AUDIO_WIDTH = 16;
localparam OUT_AUDIO_WIDTH = 32;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/


/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/

wire [OUT_AUDIO_WIDTH-1:0] left_channel_audio_out;
wire [OUT_AUDIO_WIDTH-1:0] right_channel_audio_out;
wire write_audio_out;

wire [IN_AUDIO_WIDTH-1:0] STM_AUDIO_IN;
assign STM_AUDIO_IN = GPIO_0[IN_AUDIO_WIDTH-1:0];

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
	end else delay_cnt <= delay_cnt + '1;

/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/

assign delay = {SW[3:0], 15'd3000};

wire [OUT_AUDIO_WIDTH-1:0] sound = (SW == 0) ? 0 : snd ? 32'd10000000 : -32'd10000000;


// sign extended
wire [OUT_AUDIO_WIDTH-1:0] out_audio;
assign out_audio = {{16{STM_AUDIO_IN[IN_AUDIO_WIDTH-1]}}, STM_AUDIO_IN};


assign left_channel_audio_out	= out_audio + sound;
assign right_channel_audio_out = out_audio + sound;

//reg enable_wr_audio_out;
//
//always @(posedge CLOCK_50)
//begin
//	if (write_audio_out)
//		enable_wr_audio_out <= 1'b0;
//	else
//		enable_wr_audio_out <= 1'b1;
//end

wire slow_clk;

//clock_slower c1 (.clk(CLOCK_50), .slow_clk(slow_clk));
//clock_slower c2 (.clk(slow_clk), .slow_clk(AUD_DACLRCK));
//defparam c2.CLK_SLOW_FACTOR = 16;
//assign AUD_BCLK = slow_clk;

assign write_audio_out = STM_AUDIO_READY;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Audio_Controller Audio_Controller (
	// Inputs
	.clk						(CLOCK_50),
	.CLOCK_50(CLOCK_50),
	.reset							(~KEY[0]),

	.clear_audio_in_memory		(),
	.clear_audio_out_memory		(),
	.left_channel_audio_out		(left_channel_audio_out),
	.right_channel_audio_out	(right_channel_audio_out),
	.write_audio_out				(write_audio_out),
	.audio_out_allowed 			(STM_AUDIO_READY),
	
	.AUD_ADCDAT						(AUD_ADCDAT),
	.AUD_BCLK						(AUD_BCLK),
	.AUD_ADCLRCK					(AUD_ADCLRCK),
	.AUD_DACLRCK					(AUD_DACLRCK),
	.AUD_XCK							(AUD_XCK),
	.AUD_DACDAT						(AUD_DACDAT)

);

avconf avc (
	.FPGA_I2C_SCLK					(FPGA_I2C_SCLK),
	.FPGA_I2C_SDAT					(FPGA_I2C_SDAT),
	.clk								(CLOCK_50),
	.reset							(~KEY[0])
);
// http://cdn.sparkfun.com/datasheets/Dev/Arduino/Shields/WolfsonWM8731.pdf
// should select an 8Khz sample rate?
defparam avc.AUD_SAMPLE_CTRL = 9'b000001110;

// debug
//hex_decoder hex0(.hex_digit(STM_AUDIO_IN[3:0]),.segments(HEX0));	
//hex_decoder hex1(.hex_digit(STM_AUDIO_IN[7:4]),.segments(HEX1));
//hex_decoder hex2(.hex_digit(STM_AUDIO_IN[11:8]),.segments(HEX2));
//hex_decoder hex3(.hex_digit({3'd0, STM_AUDIO_WR}),.segments(HEX3));
//hex_decoder hex4(.hex_digit({3'd0, STM_AUDIO_READY}),.segments(HEX4));
//assign HEX5 = '1;

endmodule


module clock_slower
(input clk,
 output reg slow_clk);
 
parameter CLK_SLOW_FACTOR = 5;

reg [7:0] slow_fact_cnt;
always @(posedge clk) begin
	if (slow_fact_cnt == CLK_SLOW_FACTOR) begin
		slow_fact_cnt <= 0;
		slow_clk <= !slow_clk;
	end
	else slow_fact_cnt <= slow_fact_cnt <= '1;
end
 
endmodule


