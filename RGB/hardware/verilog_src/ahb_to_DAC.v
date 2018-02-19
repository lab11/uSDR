
module ahb_to_DAC(  HSEL, HADDR, HWRITE, HSIZE, HBURST, HPROT, HTRANS, 
					HMASTLOCK, HREADY, HWDATA, HRESETn, HCLK,
					HREADYOUT, HRESP, HRDATA,
					DAC_CS, DAC_WR, DAC_GAIN, DAC_LDAC, DAC_CLR, DAC_PD, 
					DAC_A0, DAC_A1, DAC_DATA);

// IOs for AHB bus
input 			HSEL, HWRITE, HMASTLOCK, HREADY, HRESETn, HCLK;
input 	[31:0] 	HADDR, HWDATA;
input 	[2:0] 	HSIZE, HBURST;
input 	[3:0] 	HPROT;
input 	[1:0] 	HTRANS;

output 			HREADYOUT;
output 	[1:0] 	HRESP;
output 	[31:0] 	HRDATA;

// IOs for DAC
output			DAC_CS, DAC_WR, DAC_GAIN, DAC_LDAC, DAC_CLR, DAC_PD, DAC_A0, DAC_A1;
output	[7:0]	DAC_DATA;

reg				HREADYOUT, hwrite_reg;
reg		[5:0]	symbol, HRDATA_reg;

assign HRDATA		= {26'b0, HRDATA_reg};
assign HRESP		= 2'b00;

reg 	[2:0] 	fsm;
wire	[7:0]	Rout, Gout, Bout;
wire	[5:0]	index_out;
wire			dac_ready;
reg				dac_start;

wire			DAC_CS, DAC_WR, DAC_GAIN, DAC_LDAC, DAC_CLR, DAC_PD, DAC_A0, DAC_A1;
wire	[7:0]	DAC_DATA;

always @ (posedge HCLK)
begin
	if (~HRESETn)
	begin
		HREADYOUT <= 1;
		fsm <= 0;
		hwrite_reg <= 0;
		symbol <= 0;
		HRDATA_reg <= 0;
		dac_start <= 0;
	end
	else
	begin
		case (fsm)
			0:
			begin
				if (HSEL && HREADY && HTRANS[1] && HSIZE==3'b010 )
				begin
					hwrite_reg	<= HWRITE;
					fsm			<= 1;
					HREADYOUT <= 0;
				end
			end

			1:
			begin
				case (hwrite_reg)
					// read
					0:
					begin
						HRDATA_reg <= symbol;
					end

					// write
					1:
					begin
						symbol <= HWDATA[5:0];
					end
				endcase
				fsm <= 2;
			end

			2:
			begin
				dac_start <= 1;
				fsm <= 3;
			end

			3:
			begin
				if (~dac_ready)
				begin
					dac_start <= 0;
					fsm <= 4;
				end
			end

			4:
			begin
				if (dac_ready)
				begin
					fsm <= 0;
					HREADYOUT <= 1;
				end
			end
		endcase
	end
end

symbol_to_idx #(.NUMBER_OF_AXIS(3), .NUMBER_OF_LEVELS(2))  s0(.symbol_in(symbol), .index(index_out));
index_to_RGB i0(.index(index_out), .R_out(Rout), .G_out(Gout), .B_out(Bout));
dac_write dac0(.clk(HCLK), .resetn(HRESETn), .ch0(Rout), .ch1(Gout), .ch2(Bout), .ch3(8'b0), .start(dac_start), .pwr(1'b1), .ready(dac_ready), .clear(1'b0), .num_of_channels(2'd3),
					.CS(DAC_CS), .WR(DAC_WR), .DATA(DAC_DATA), .GAIN(DAC_GAIN), .LDAC(DAC_LDAC), .CLR(DAC_CLR), .PD(DAC_PD), .A0(DAC_A0), .A1(DAC_A1));
endmodule
