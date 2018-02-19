
// amplitude levels for RGB are 4
// maximum axis are 3
// maximum indces = 4^3 = 64
module index_to_RGB(index, R_out, G_out, B_out);

input	[5:0] index;
output	[7:0] R_out, G_out, B_out;
reg		[7:0] R_out, G_out, B_out;

// define amplitude levels for each axis
parameter a0_R = 0;
parameter a1_R = 85;
parameter a2_R = 160;
parameter a3_R = 255;

parameter a0_G = 0;
parameter a1_G = 85;
parameter a2_G = 160;
parameter a3_G = 255;

parameter a0_B = 0;
parameter a1_B = 85;
parameter a2_B = 160;
parameter a3_B = 255;


always @ *
begin
	case (index[1:0])
		2'b00: begin R_out = a0_R; end
		2'b01: begin R_out = a1_R; end
		2'b10: begin R_out = a2_R; end
		2'b11: begin R_out = a3_R; end
	endcase
end

always @ *
begin
	case (index[3:2])
		2'b00: begin G_out = a0_G; end
		2'b01: begin G_out = a1_G; end
		2'b10: begin G_out = a2_G; end
		2'b11: begin G_out = a3_G; end
	endcase
end

always @ *
begin
	case (index[5:4])
		2'b00: begin B_out = a0_B; end
		2'b01: begin B_out = a1_B; end
		2'b10: begin B_out = a2_B; end
		2'b11: begin B_out = a3_B; end
	endcase
end

endmodule
