

module symbol_to_idx (symbol_in, index);

// available index
parameter NUMBER_OF_AXIS = 3;
parameter NUMBER_OF_LEVELS = 4;
//parameter NUMBER_OF_INPUT_BITS = NUMBER_OF_AXIS*(NUMBER_OF_LEVELS>>1);

input	[5:0] symbol_in;
output	[5:0] index;
wire	[5:0] index;
reg		[1:0] index_offset;
reg		[3:0] index_mult0;
reg		[5:0] index_mult1;

/* 			| #axis = 1	| #axis = 2 	| #axis = 3 |
#levels = 2	| (0,3)		| (0,3),(12,15)	| (0,3), (12,15), (48,51), (60,63)
#levels = 4 | 0 ~ 3		| 0 ~ 15		| 0 ~ 63 */

assign index = index_offset + index_mult0 + index_mult1;

always @ *
begin
	index_offset = 0;
	index_mult0 = 0;
	index_mult1 = 0;
	case (NUMBER_OF_AXIS)
		1:
		begin
			case (NUMBER_OF_LEVELS)
				2:
				begin
					case (symbol_in[0])
						1'b0: begin index_offset = 0; end
						1'b1: begin index_offset = 3; end
					endcase
				end

				4:
				begin
					case (symbol_in[1:0])
						2'b00: begin index_offset = 0; end
						2'b01: begin index_offset = 1; end
						2'b10: begin index_offset = 3; end
						2'b11: begin index_offset = 2; end
					endcase
				end
			endcase
		end

		2:
		begin
			case (NUMBER_OF_LEVELS)
				2:
				begin
					case (symbol_in[0])
						0: begin index_offset = 0; end
						1: begin index_offset = 3; end
					endcase

					case (symbol_in[1])
						0: begin index_mult0 = 0; end
						1: begin index_mult0 = 12; end
					endcase
				end

				4:
				begin
					case (symbol_in[1:0])
						2'b00: begin index_offset = 0; end
						2'b01: begin index_offset = 1; end
						2'b10: begin index_offset = 3; end
						2'b11: begin index_offset = 2; end
					endcase

					case (symbol_in[3:2])
						2'b00: begin index_mult0 = 0; end
						2'b01: begin index_mult0 = 4; end
						2'b10: begin index_mult0 = 12; end
						2'b11: begin index_mult0 = 8; end
					endcase
				end
			endcase
		end

		3:
		begin
			case (NUMBER_OF_LEVELS)
				2:
				begin
					case (symbol_in[0])
						0: begin index_offset = 0; end
						1: begin index_offset = 3; end
					endcase

					case (symbol_in[1])
						0: begin index_mult0 = 0; end
						1: begin index_mult0 = 12; end
					endcase

					case (symbol_in[2])
						0: begin index_mult1 = 0; end
						1: begin index_mult1 = 48; end
					endcase
				end

				4:
				begin
					case (symbol_in[1:0])
						2'b00: begin index_offset = 0; end
						2'b01: begin index_offset = 1; end
						2'b10: begin index_offset = 3; end
						2'b11: begin index_offset = 2; end
					endcase

					case (symbol_in[3:2])
						2'b00: begin index_mult0 = 0; end
						2'b01: begin index_mult0 = 4; end
						2'b10: begin index_mult0 = 12; end
						2'b11: begin index_mult0 = 8; end
					endcase

					case (symbol_in[5:4])
						2'b00: begin index_mult1 = 0; end
						2'b01: begin index_mult1 = 16; end
						2'b10: begin index_mult1 = 48; end
						2'b11: begin index_mult1 = 32; end
					endcase
				end
			endcase
		end
	endcase
end


endmodule
