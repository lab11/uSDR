module cordic_rom (in_stage, angle_out);

input	[3:0]	in_stage;
output	[12:0]	angle_out;	//6bits exp, 7bits mantissa
reg		[12:0]	angle_out;

always @ *
begin
	case (in_stage)
		//						exp		mantissa
		0: begin angle_out = 13'b101101_0000000;	end
		1: begin angle_out = 13'b011010_1001000;	end
		2: begin angle_out = 13'b001110_0000100;	end
		3: begin angle_out = 13'b000111_0010000;	end
		4: begin angle_out = 13'b000011_1001001;	end
		5: begin angle_out = 13'b000001_1100101; 	end
		6: begin angle_out = 13'b000000_1110010;	end
		7: begin angle_out = 13'b000000_0111001;	end
		8: begin angle_out = 13'b000000_0011100;	end
		9: begin angle_out = 13'b000000_0001110;	end
		default: begin angle_out = 13'b111111_1111111;	end
	endcase
end

endmodule
