module freq_correction(clk, resetn, en, freq_mod, in_phase, quad_phase, afc_en);

input	clk, resetn;
input	en;
input	signed [7:0] in_phase, quad_phase;
input	afc_en;
output	[1:0]	freq_mod;
wire	[1:0]	freq_mod;
reg		[5:0]	freq_mod_reg;
wire	cordic_done, cordic_ready;
wire	signed	[15:0]	cordic_ang_out;
reg		cordic_start;

cordic c0(	.clk(clk), .resetn(resetn), .start(cordic_start), .x(in_phase), .y(quad_phase), 
			.angle_out(cordic_ang_out), .done(cordic_done), .ready(cordic_ready));

reg		[3:0]	cordic_counter;
reg	signed	[15:0]	phase;
wire signed	[16:0]	phase_diff = cordic_ang_out - phase;
reg	signed	[16:0]	phase_diff_reg;
wire signed [16:0]	ang_180 = 17'b00101101000000000;	// 180
wire signed [16:0]	ang_n180 = 17'b11010011000000000;// 180
wire signed [16:0]	ang_360 = 17'b01011010000000000;
wire signed [16:0]	ang_45 = 17'b00001011010000000;
wire signed [16:0]	ang_n45 = 17'b11110100110000000;
reg	signed	[16:0]	phase_comp;
reg	signed	[16:0]	phase_diff0, phase_diff1, phase_diff2;
wire signed	[17:0]	phase_diff_diff0 = phase_diff2 - phase_diff1;
wire signed	[17:0]	phase_diff_diff1 = phase_diff1 - phase_diff0;
wire [9:0] phase_diff_diff0_abs = (phase_diff_diff0[17])? (~phase_diff_diff0[16:7]) : phase_diff_diff0[16:7];
wire [9:0] phase_diff_diff1_abs = (phase_diff_diff1[17])? (~phase_diff_diff1[16:7]) : phase_diff_diff1[16:7];
wire [9:0] phase_diff_threshold = 10'b00_0001_0000;	//16
wire sign_ind = ((phase_diff0[16]&phase_diff1[16]&phase_diff2[16])) | ((~phase_diff0[16])&(~phase_diff1[16])&(~phase_diff2[16]));

reg	freq_mod_enable;
assign freq_mod = ({2{freq_mod_enable}}&freq_mod_reg[1:0]);

always @ *
begin
	if ((freq_mod_reg[5:4]==freq_mod_reg[3:2])&&(freq_mod_reg[5:4]==freq_mod_reg[1:0])&&(cordic_counter==2))
		freq_mod_enable = 1;
	else
		freq_mod_enable = 0;
end

always @ *
begin
	phase_comp = phase_diff_reg;
	if (phase_diff_reg[16]==0)
	begin	// phase diff > 180, phase_diff_reg - 360
		if (phase_diff_reg>ang_180)
			phase_comp = phase_diff_reg - ang_360;
	end
	else
	begin
		if (phase_diff_reg<ang_n180)
			phase_comp = phase_diff_reg + ang_360;
	end
end

always @ (posedge clk)
begin
	if (~resetn)
	begin
		cordic_counter <= 0;
		phase <= 0;
		phase_diff0 <= 0;
		phase_diff1 <= 0;
		phase_diff2 <= 0;
		freq_mod_reg <= 0;
		cordic_start <= 0;
		phase_diff_reg <= 0;
	end
	else
	begin
		if (en&afc_en)
		begin
			if (cordic_counter<11)
				cordic_counter <= cordic_counter + 1;
			else
				cordic_counter <= 0;

			case (cordic_counter)
				0:	// read data out
				begin
					phase <= cordic_ang_out;	
					phase_diff_reg <= phase_diff;
					cordic_start <= 0;
				end

				1:
				begin
					phase_diff0 <= phase_comp;
					phase_diff1 <= phase_diff0;
					phase_diff2 <= phase_diff1;
				end

				2:
				begin
					if ((sign_ind)&&(phase_diff_diff0_abs < phase_diff_threshold)&&(phase_diff_diff1_abs < phase_diff_threshold))
					begin
						if (phase_diff2[16])		// phase diff < 0,
						begin
							if (phase_diff2<ang_n45)
								freq_mod_reg <= {freq_mod_reg[3:0], 2'b10};		// frequency should decrease
							else
								freq_mod_reg <= {freq_mod_reg[3:0], 2'b01};		// frequency should increase
						end
						else
						begin
							if (phase_diff2>ang_45)
								freq_mod_reg <= {freq_mod_reg[3:0], 2'b01};		// frequency should increase
							else
								freq_mod_reg <= {freq_mod_reg[3:0], 2'b10};		// frequency should decrease
						end
					end
				end

				11:	// feed new data
				begin
					cordic_start <= 1;
				end

				default:
				begin
					cordic_start <= 0;
				end
			endcase
		end
		else
		begin
			freq_mod_reg <= 0;
			cordic_counter <= 0;
			cordic_start <= 0;
		end
	end
end

endmodule
