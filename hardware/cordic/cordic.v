module cordic(clk, resetn, start, x, y, angle_out, done, ready);

input	clk, resetn;
input	start;
input signed	[7:0]	x, y;	// two's compliment, x[7], y[7] are sign bits
output signed	[15:0]	angle_out;	// two's compliment, angle[15] sign bits, 14~7 exp, 6~0 mantissa
output	done, ready;

wire	[12:0]	rom_angle;
reg	signed [9:0] x_ori, y_ori;
reg	signed [9:0] next_x_ori, next_y_ori;
wire signed	[15:0]	rom_angle_extension = {3'b0, rom_angle};
reg	signed [15:0]	angle, next_angle, angle_out, next_angle_out;
reg			state, next_state;
reg		[3:0]	stage_counter, next_stage_counter;
reg	done, next_done, ready, next_ready;
reg		[1:0]	quad, next_quad;

cordic_rom rom0(.in_stage(stage_counter), .angle_out(rom_angle));
wire signed [9:0] shift_y = (y_ori>>>stage_counter);
wire signed [9:0] shift_x = (x_ori>>>stage_counter);
wire signed [15:0] ang_comp_180 = 16'b0101101000000000;	// 180
wire signed [15:0] ang_comp_n180 = 16'b1010011000000000;// 180
wire signed [9:0] x_sign_ext = {{2{x[7]}}, x};
wire signed [9:0] y_sign_ext = {{2{y[7]}}, y};
wire signed [9:0] neg_x = -x_sign_ext;


`define	RST				0
`define CORDIC_START	1
`define MAX_STAGE		10

always @ (posedge clk)
begin
	if (~resetn)
	begin
		x_ori <= 0;
		y_ori <= 0;
		state <= `RST;
		stage_counter <= 0;
		angle <= 0;
		done <= 0;
		quad <= 0;
		ready <= 1;
		angle_out <= 0;
	end
	else
	begin
		x_ori <= next_x_ori;
		y_ori <= next_y_ori;
		state <= next_state;
		stage_counter <= next_stage_counter;
		angle <= next_angle;
		done <= next_done;
		quad <= next_quad;
		ready <= next_ready;
		angle_out <= next_angle_out;
	end
end

always @ *
begin
	next_x_ori = x_ori;
	next_y_ori = y_ori;
	next_state = state;
	next_stage_counter = stage_counter;
	next_angle = angle;
	next_done = done;
	next_quad = quad;
	next_angle_out = angle_out;
	next_ready = ready;

	case (state)
		`RST:
		begin
			case(quad)
				2: begin next_angle_out = ang_comp_180 - angle;	end
				3: begin next_angle_out = ang_comp_n180 - angle; end
				default: begin next_angle_out = angle; end
			endcase
			next_done = 1;

			if (start)
			begin
				if (x[7])	// x<0
				begin
					next_x_ori = neg_x;
					if (y[7])
						next_quad = 3;
					else				// QUADRANT III
						next_quad = 2;
				end
				else
				begin
					next_x_ori = x_sign_ext;
					next_quad = 0;
				end
				next_y_ori = y_sign_ext;
				next_state = `CORDIC_START;
				next_stage_counter = 0;
				next_angle = 0;
				next_ready = 0;
				next_done = 0;
			end
		end

		`CORDIC_START:
		begin
			if (y_ori==0)
			begin
				next_state = `RST;
				next_ready = 1;
			end
			else
			begin
				if (y_ori[9]==0)	// y > 0
				begin
					next_x_ori = x_ori + shift_y;
					next_y_ori = y_ori - shift_x;
					next_angle = angle + rom_angle_extension;
				end
				else
				begin
					next_x_ori = x_ori - shift_y;
					next_y_ori = y_ori + shift_x;
					next_angle = angle - rom_angle_extension;
				end

				if (stage_counter==`MAX_STAGE-1)
				begin
					next_state = `RST;
					next_ready = 1;
				end
				else
					next_stage_counter = stage_counter + 1;
			end
		end

	endcase

end

endmodule
