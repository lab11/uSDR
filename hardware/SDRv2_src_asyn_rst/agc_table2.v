
module agc_table2(
	input			clk, 
	input			resetn, 
	input			en,
	input	[7:0]	rssi_in, 
	input	[7:0]	pwr_in,
	output	[6:0]	gain_out
);

reg		[1:0]	lna_gain, next_lna_gain;
reg		[4:0]	vga_gain, next_vga_gain;

assign gain_out = {lna_gain, vga_gain};

parameter PWR_LOWER_BOUND = 90;
parameter PWR_UPPER_BOUND = 150;

parameter RSSI_MAX_LNA_UPPER_BOUND = 140;
parameter RSSI_MID_LNA_LOWER_BOUND = 80;

parameter MAX_LNA_GAIN = 3;
parameter MID_LNA_GAIN = 2;

reg		up, down, switch_lna_gain;

always @ *
begin
	up = 0;
	down = 0;
	switch_lna_gain = 0;
	if (lna_gain==MAX_LNA_GAIN)
	begin
		if (rssi_in > RSSI_MAX_LNA_UPPER_BOUND)
		begin
			switch_lna_gain = 1;
		end
		else
		begin
			if (pwr_in < PWR_LOWER_BOUND)
				up = 1;
			else if (pwr_in > PWR_UPPER_BOUND)
				down = 1;
		end
	end
	else	// lna_gain == MID_LNA_GAIN
	begin
		if (rssi_in < RSSI_MID_LNA_LOWER_BOUND)
		begin
			switch_lna_gain = 1;
		end
		else
		begin
			if (pwr_in < PWR_LOWER_BOUND)
				up = 1;
			else if (pwr_in > PWR_UPPER_BOUND)
				down = 1;
		end
	end
end

always @ (posedge clk or negedge resetn)
begin
	if (~resetn)
	begin
		lna_gain <= 2'b11;
		vga_gain <= 5'b11111;
	end
	else
	begin
		lna_gain <= next_lna_gain;
		vga_gain <= next_vga_gain;
	end
end

wire	[1:0]	direction = {up, down};

always @ *
begin
	next_lna_gain = lna_gain;
	next_vga_gain = vga_gain;
	if (en)
	begin
		if (switch_lna_gain)
		begin
			if (lna_gain==MAX_LNA_GAIN)
			begin
				next_lna_gain = MID_LNA_GAIN;
				next_vga_gain = 13;
			end
			else
			begin
				next_lna_gain = MAX_LNA_GAIN;
				next_vga_gain = 8;
			end
		end
		else
		begin
			case (direction)
				2'b10:
				begin
					if (vga_gain<5'b11111)
						next_vga_gain = vga_gain + 1;
				end

				2'b01:
				begin
					if (vga_gain)
						next_vga_gain = vga_gain - 1;
				end

				default:
				begin
				end
			endcase
		end
	end
end


endmodule
