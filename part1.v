//Project Demo
//June 22, 2023
//Aman Edward Nangia     #43754290
//Seiya Nozawa-Temchenko #34838482

//Project top module
module part1 (CLOCK_50, CLOCK2_50, KEY, SWITCH, FPGA_I2C_SCLK, FPGA_I2C_SDAT, AUD_XCK, AUD_DACLRCK, AUD_ADCLRCK, AUD_BCLK, AUD_ADCDAT, AUD_DACDAT);
	
	//Input clock and button signals
	input CLOCK_50, CLOCK2_50;
	input [0:0] KEY;
	input [3:0] SWITCH;

	//I2C Audio/Video config interface
	output FPGA_I2C_SCLK; //Serial clock line for I2C communication
	inout FPGA_I2C_SDAT; //Serial data line for I2C communication
   
	//Audio CODEC signals
	output AUD_XCK; //CODEC external clock
	input AUD_DACLRCK, AUD_ADCLRCK, AUD_BCLK; //CODEC DAC/ADC latch and bit clock signals
	input AUD_ADCDAT; //ADC data
	output AUD_DACDAT; //DAC data
	
	//Local wires
	wire read_ready, write_ready, read, write;
	wire [23:0] read_data_left, read_data_right;
	wire [23:0] write_data_left, write_data_right;
	wire [23:0] file_read_data_left, file_read_data_right;
	wire reset = ~KEY[0]; //Active when button is not pressed
	wire enable = 1'b1;

	//ROM input reg and output wire
	reg [15:0] address; 
	wire [15:0] raw_data; 
	reg [2:0] counter; //0 to 5

	//Create wires for downsampling
	wire [15:0] downsamp_data_left, downsamp_data_right;
	wire CLOCK_08, CLOCK_10, CLOCK_22, CLOCK_48;
	wire clock;

	//Instatiate audio file storage in ROM
	audio_rom ROM(.address(address), .clock(CLOCK_50), .q(raw_data));
	
	//Instantiate 10kHz clock
	clk_sampler clock_maker(.clk_50(CLOCK_50), .clk_08khz(CLOCK_08), .clk_10khz(CLOCK_10), .clk_22khz(CLOCK_22), .clk_48khz(CLOCK_48));
	
	//Assign clock speed based on switch
	/*always @(*) begin
		case ({SWITCH[3], SWITCH[2], SWITCH[1]})
			3'b001: clock <= CLOCK_10; //Switch 1 on
			3'b010: clock <= CLOCK_22; //Switch 2 on
			3'd100: clock <= CLOCK_48; //Switch 3 on
			default: clock <= 0;
		endcase
	end */
	assign clock = CLOCK_10;
	//Increment ROM storage address to match 10kHz .wav file
	always @(posedge clock) begin
		case (reset)
			1'b1: address <= 0; 
			default: address <= address + 1; //Increment address every clock cycle
		endcase
		
		//write <= write_ready
	end 
/*
	//Instatiate input selection
	input_selector select_left(.clk(CLOCK_50), .reset(reset), .select(SWITCH[0]), .mic_data(read_data_left), .file_data(raw_data_left), .output_data(downsamp_data_left));
	input_selector select_right(.clk(CLOCK_50), .reset(reset), .select(SWITCH[0]), .mic_data(read_data_right), .file_data(raw_data_right), .output_data(downsamp_data_right));

	//Instatiate downsampling to 8kHz sampling rate
	down_sampler downsamp_left(.clk(CLOCK_50), .reset(reset), .in_data(downsamp_data_left), .out_data(write_data_left));
	down_sampler downsamp_right(.clk(CLOCK_50), .reset(reset), .in_data(downsamp_data_right), .out_data(write_data_right));
*/
	//Assign read data to write data 
	assign write_data_left = raw_data;
	assign write_data_right = raw_data;

	//Assign read/write enables
	assign read = read_ready; //48kHz
	assign write = write_ready;  
	
	//Instantiate clock_generator
	clock_generator my_clock_gen(
		//Inputs
		CLOCK2_50,
		reset,

		//Outputs
		AUD_XCK //Connects external clock of the CODEC
	);

	//Instantiate audio_and_video_config
	audio_and_video_config cfg(
		//Inputs
		CLOCK_50,
		reset,

		//Bidirectionals
		FPGA_I2C_SDAT, //Serial data line for I2C communication
		FPGA_I2C_SCLK //Serial clock line for I2C communication
	);

	//Instatiate audio_codec
	audio_codec codec(
		//Inputs
		CLOCK_50,
		reset,

		read, write,
		write_data_left, write_data_right,

		AUD_ADCDAT, //ADC data

		//Bidirectionals
		AUD_BCLK, //CODEC bit clock signal
		AUD_ADCLRCK, //CODEC ADC latch signal
		AUD_DACLRCK, //CODEC DAC latch signal

		//Outputs
		read_ready, write_ready,
		read_data_left, read_data_right,
		AUD_DACDAT //DAC data
	);
endmodule

//Source
module clk_sampler(clk_50, clk_08khz, clk_10khz, clk_22khz, clk_48khz);
	input clk_50;
	output reg clk_08khz, clk_10khz, clk_22khz, clk_48khz;
	
	localparam rate_08khz = 6250; //50M / 8k
	localparam rate_10khz = 5000; //50M / 10k
	localparam rate_22khz = 2273; //50M / 22k
	localparam rate_48khz = 1042; //50M / 48k
	
	reg [13:0] count_08khz;
	reg [13:0] count_10khz; //Hold values >5000 (0 to 16383)
	reg [13:0] count_22khz; 
	reg [13:0] count_48khz;
	
	//Create a 8kHz clock
	always @(posedge clk_50) begin
		if (count_08khz == rate_08khz - 1) begin
			count_08khz <= 0; //Reset counter if 6250 has been reached
			clk_08khz <= ~clk_08khz; //Set clock to low
		end
		else begin
			count_08khz <= count_08khz + 1;
		end
	end
	
	//Create a 10kHz clock
	always @(posedge clk_50) begin
		if (count_10khz == rate_10khz - 1) begin
			count_10khz <= 0; //Reset counter if 5000 has been reached
			clk_10khz <= ~clk_10khz; //Set clock to low
		end
		else begin
			count_10khz <= count_10khz + 1;
		end
	end
	
	//Create a 22kHz clock
	always @(posedge clk_50) begin
		if (count_22khz == rate_22khz - 1) begin
			count_22khz <= 0; //Reset counter if 2273 has been reached
			clk_22khz <= ~clk_22khz; //Set clock to low
		end
		else begin
			count_22khz <= count_22khz + 1;
		end
	end
	
	//Create a 48kHz clock
	always @(posedge clk_50) begin
		if (count_48khz == rate_48khz - 1) begin
			count_48khz <= 0; //Reset counter if 1042 has been reached
			clk_48khz <= ~clk_48khz; //Set clock to low
		end
		else begin
			count_48khz <= count_48khz + 1;
		end
	end
endmodule
/*
//Input selection
module input_selector(clk, reset, select, mic_data, file_data, output_data);
	input clk, select, reset;
	input [15:0] mic_data, file_data;
	output reg [15:0] output_data;

	always @(posedge clk or posedge reset) begin
		case (reset)
			1'b1: output_data <= 16'b0;
			default: begin
				case (select)
					1'b1: output_data <= file_data; //Select audio file
					default: output_data <= mic_data; //Select mic input
				endcase
			end
		endcase
	end
endmodule

//48kHz to 8kHz downsampler
module down_sampler (clk, reset, in_data, out_data);
	input clk, reset;
	input [15:0] in_data;
	output reg [15:0] out_data;
	
	reg [2:0] count = 0;
	
	always @(posedge clk or posedge reset) begin
		case (reset)
			1'b1: begin
				out_data <= 0;
				count <= 0;
			end
			default: begin
				case (count)
					3'd5: begin
						out_data <= in_data; //Keep every 6th sample
						count <= 0; //Reset count
					end	
					default: count <= count + 1;
				endcase
			end
		endcase
	end
endmodule

//Error Coding
//Hamming(7,4) encoder
module encoder(source_bits, encoded_bits);
	input [15:0] source_bits;
	output [27:0] encoded_bits;

	//Instatiate hamming_74_encoder
	hamming_74_encoder encoder_1(source_bits[3:0], encoded_bits[6:0]);
	hamming_74_encoder encoder_2(source_bits[7:4], encoded_bits[13:7]);
	hamming_74_encoder encoder_3(source_bits[11:8], encoded_bits[20:14]);
	hamming_74_encoder encoder_4(source_bits[15:12], encoded_bits[27:21]); 
endmodule

module hamming_74_encoder(source_bits, encoded_bits);
	input [3:0] source_bits;
	output [6:0] encoded_bits;

	wire p0, p1, p2;

	//Assign parity bits
	assign p0 = source_bits[0] ^ source_bits[1] ^ source_bits[3];
	assign p1 = source_bits[0] ^ source_bits[2] ^ source_bits[3];
	assign p2 = source_bits[1] ^ source_bits[2] ^ source_bits[3];

	//Assign encoded bits
	assign encoded_bits = {source_bits[3:1], p2, source_bits[0], p1, p0};
endmodule

//Hamming(7,4) decoder 
module decoder(demod_bits, output_bits, errors);
	input [27:0] demod_bits;
	output [15:0] output_bits;

	//Instatiate hamming_74_decoder
	hamming_74_decoder decoder_1(demod_bits[6:0], output_bits[3:0]);
	hamming_74_decoder decoder_2(demod_bits[13:7], output_bits[7:4]);
	hamming_74_decoder decoder_3(demod_bits[20:14], output_bits[11:8]);
	hamming_74_decoder decoder_4(demod_bits[27:21], output_bits[15:12]);
endmodule

module hamming_74_decoder(demod_bits, output_bits);
	input [6:0] demod_bits;
	output [3:0] output_bits;

	wire s0, s1, s2;
	reg [6:0] corrected_bits;

	//Syndrome computation
	assign s0 = demod_bits[0] ^ demod_bits[2] ^ demod_bits[4] ^ demod_bits[6];
	assign s1 = demod_bits[1] ^ demod_bits[2] ^ demod_bits[5] ^ demod_bits[6];
	assign s2 = demod_bits[3] ^ demod_bits[4] ^ demod_bits[5] ^ demod_bits[6];

	//Error correction (bit flipping)
	always @(*) begin
		case ({s2, s1, s0}) 
			3'd1: corrected_bits = demod_bits & 7'b1;
			3'd2: corrected_bits = demod_bits & 7'b10;
			3'd3: corrected_bits = demod_bits & 7'b100;
			3'd4: corrected_bits = demod_bits & 7'b1000;
			3'd5: corrected_bits = demod_bits & 7'b10000;
			3'd6: corrected_bits = demod_bits & 7'b100000;
			3'd7: corrected_bits = demod_bits & 7'b1000000;
			default: corrected_bits = demod_bits;
		endcase
	end

	//Assign output bits
	assign output_bits = {demod_bits[6:4], demod_bits[2]};
endmodule

//Modulation

//Tx/Rx

//AWGN

//BER

//AWGN
module noise_generator (clk, enable, Q, in);
    input clk, enable;
	 input [15:0] in;
    output wire [15:0] Q;  
    reg [2:0] counter;
    
    always @(posedge clk) begin
        if (enable)
            counter = counter + 1'b1;
	end

    assign Q = enable ? {{10{counter[2]}}, counter, 11'd0} + in: in;
endmodule
*/