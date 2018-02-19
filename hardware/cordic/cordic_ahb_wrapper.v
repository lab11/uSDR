module cordic_ahb_wrapper(	HSEL, HADDR, HWRITE, HSIZE, HBURST, HPROT, HTRANS, 
							HMASTLOCK, HREADY, HWDATA, HRESETn, HCLK,
							HREADYOUT, HRESP, HRDATA);

input 			HSEL, HWRITE, HMASTLOCK, HREADY, HRESETn, HCLK;
input 	[31:0] 	HADDR, HWDATA;
input 	[2:0] 	HSIZE, HBURST;
input 	[3:0] 	HPROT;
input 	[1:0] 	HTRANS;

output 			HREADYOUT;
output 	[1:0] 	HRESP;
output 	[31:0] 	HRDATA;

reg				hready_reg, hwrite_reg;
reg		[15:0]	HRDATA_reg;
reg		[1:0]	fsm;

// cordic variable
reg	signed	[7:0]	cordic_x, cordic_y;
wire signed [15:0]	cordic_angle;
reg				cordic_start;
wire			cordic_done, cordic_ready;

assign HRDATA		= {HRDATA_reg, cordic_x, cordic_y};
assign HRESP		= 2'b00;
assign HREADYOUT = hready_reg&cordic_ready;

cordic c0(	.clk(HCLK), .resetn(HRESETn), .start(cordic_start), .x(cordic_x), .y(cordic_y), .angle_out(cordic_angle), 
			.done(cordic_done), .ready(cordic_ready));

always @ (posedge HCLK)
begin
	if (~HRESETn)
	begin
		fsm <= 0;
		hwrite_reg <= 0;
		hready_reg <= 1;
		cordic_x <= 0;
		cordic_y <= 0;
		cordic_start <= 0;
		HRDATA_reg <= 0;
	end
	else
	begin
		case (fsm)
			2'b00:
			begin
				if (HSEL && HREADY && HTRANS[1] && HSIZE==3'b010 )
				begin
					hwrite_reg	<= HWRITE;
					fsm			<= 2'b01;
					hready_reg	<= 0;
				end
			end

			2'b01:
			begin
				if (hwrite_reg) // write data
				begin
					cordic_x <= HWDATA[7:0];
					cordic_y <= HWDATA[15:8];
					cordic_start <= 1;
				end
				else			// read data
				begin
					HRDATA_reg <= cordic_angle;
				end
				fsm <= 2'b10;
			end

			2'b10:
			begin
				fsm <= 2'b11;
			end

			2'b11:
			begin
				cordic_start <= 0;
				fsm <= 2'b00;
				hready_reg <= 1;
			end

		endcase
	end
end

endmodule
