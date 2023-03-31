module control
(
	input clk,
	input reset,
	
	// Button input
	input i_enter,
	
	// Datapath
	output logic o_inc_actual,
	input i_over,
	input i_under,
	input i_equal,
	
	// LED Control: Setting this to 1 will copy the current
	// values of over/under/equal to the 3 LEDs. Setting this to 0
	// will cause the LEDs to hold their current values.
	output logic o_update_leds
);

// Declare two objects, 'state' and 'nextstate'
// that are of enum type.
enum int unsigned
{
	// idle, generating random number
	S_IDLE,
	// enter key pressed
	S_ENTER_DOWN,
	// enter key unpressed
	S_ENTER_UP
} state, next_state;

// Clocked always block for making state registers
always_ff @ (posedge clk or posedge reset) begin
	if (reset) state <= S_IDLE;
	else state <= next_state;
end

// Switch on the state to generate control signals
always_comb begin
	// default values
	next_state = state;
	o_inc_actual = 1'b0;
	o_update_leds = 1'b0;
	
	case (state)
		S_IDLE: 
		begin
			o_inc_actual = 1'b1; // keep incrementing in IDLE
			next_state = i_enter ? S_ENTER_DOWN : S_IDLE;
		end
		// wait until enter key is released
		S_ENTER_DOWN: next_state = i_enter ? S_ENTER_DOWN : S_ENTER_UP;
		// Game over, update LEDs, wait for reset
		S_ENTER_UP: o_update_leds = 1'b1; 
	endcase
end

endmodule
