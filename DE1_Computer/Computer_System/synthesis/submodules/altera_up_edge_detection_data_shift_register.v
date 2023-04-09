// (C) 2001-2021 Intel Corporation. All rights reserved.
// Your use of Intel Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files from any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Intel Program License Subscription 
// Agreement, Intel FPGA IP License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Intel and sold by 
// Intel or its authorized distributors.  Please refer to the applicable 
// agreement for further details.


// megafunction wizard: %Shift register (RAM-based)%
// GENERATION: STANDARD
// VERSION: WM1.0
// MODULE: altshift_taps 

// ============================================================
// File Name: Altera_UP_Edge_Detection_Data_Shift_Register.v
// Megafunction Name(s):
// 			altshift_taps
//
// Simulation Library Files(s):
// 			altera_mf
// ============================================================
// ************************************************************
// THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
//
// 6.1 Build 201 11/27/2006 SJ Full Version
// ************************************************************


//Copyright (C) 1991-2014 Altera Corporation
//Your use of Altera Corporation's design tools, logic functions 
//and other software and tools, and its AMPP partner logic 
//functions, and any output files from any of the foregoing 
//(including device programming or simulation files), and any 
//associated documentation or information are expressly subject 
//to the terms and conditions of the Altera Program License 
//Subscription Agreement, Altera MegaCore Function License 
//Agreement, or other applicable license agreement, including, 
//without limitation, that your use is for the sole purpose of 
//programming logic devices manufactured by Altera and sold by 
//Altera or its authorized distributors.  Please refer to the 
//applicable agreement for further details.


// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module altera_up_edge_detection_data_shift_register (
	clken,
	clock,
	shiftin,
	shiftout,
	taps);

	parameter DW	= 10;
	parameter SIZE	= 720;

	input	  clken;
	input	  clock;
	input	[DW:1]  shiftin;
	output	[DW:1]  shiftout;
	output	[DW:1]  taps;

	wire [DW:1] sub_wire0;
	wire [DW:1] sub_wire1;
	wire [DW:1] taps = sub_wire0[DW:1];
	wire [DW:1] shiftout = sub_wire1[DW:1];

	altshift_taps	altshift_taps_component (
				.clken (clken),
				.clock (clock),
				.shiftin (shiftin),
				.taps (sub_wire0),
				.shiftout (sub_wire1));
	defparam
		altshift_taps_component.lpm_type = "altshift_taps",
		altshift_taps_component.number_of_taps = 1,
		altshift_taps_component.tap_distance = SIZE,
		altshift_taps_component.width = DW;


endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: PRIVATE: CLKEN NUMERIC "1"
// Retrieval info: PRIVATE: GROUP_TAPS NUMERIC "0"
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone II"
// Retrieval info: PRIVATE: NUMBER_OF_TAPS NUMERIC "1"
// Retrieval info: PRIVATE: RAM_BLOCK_TYPE NUMERIC "0"
// Retrieval info: PRIVATE: TAP_DISTANCE NUMERIC "720"
// Retrieval info: PRIVATE: WIDTH NUMERIC "10"
// Retrieval info: CONSTANT: LPM_TYPE STRING "altshift_taps"
// Retrieval info: CONSTANT: NUMBER_OF_TAPS NUMERIC "1"
// Retrieval info: CONSTANT: TAP_DISTANCE NUMERIC "720"
// Retrieval info: CONSTANT: WIDTH NUMERIC "10"
// Retrieval info: USED_PORT: clken 0 0 0 0 INPUT VCC clken
// Retrieval info: USED_PORT: clock 0 0 0 0 INPUT NODEFVAL clock
// Retrieval info: USED_PORT: shiftin 0 0 10 0 INPUT NODEFVAL shiftin[9..0]
// Retrieval info: USED_PORT: shiftout 0 0 10 0 OUTPUT NODEFVAL shiftout[9..0]
// Retrieval info: USED_PORT: taps 0 0 10 0 OUTPUT NODEFVAL taps[9..0]
// Retrieval info: CONNECT: @shiftin 0 0 10 0 shiftin 0 0 10 0
// Retrieval info: CONNECT: shiftout 0 0 10 0 @shiftout 0 0 10 0
// Retrieval info: CONNECT: taps 0 0 10 0 @taps 0 0 10 0
// Retrieval info: CONNECT: @clock 0 0 0 0 clock 0 0 0 0
// Retrieval info: CONNECT: @clken 0 0 0 0 clken 0 0 0 0
// Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
// Retrieval info: GEN_FILE: TYPE_NORMAL Altera_UP_Edge_Detection_Data_Shift_Register.v TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL Altera_UP_Edge_Detection_Data_Shift_Register.inc FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL Altera_UP_Edge_Detection_Data_Shift_Register.cmp FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL Altera_UP_Edge_Detection_Data_Shift_Register.bsf FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL Altera_UP_Edge_Detection_Data_Shift_Register_inst.v FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL Altera_UP_Edge_Detection_Data_Shift_Register_bb.v TRUE
// Retrieval info: LIB_FILE: altera_mf
