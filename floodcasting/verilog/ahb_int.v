
module ahb_int(	
	input	HCLK,
	input	HRESETn, 
	input	[31:0] HADDR, 
	input	[31:0] HWDATA, 
	input	[3:0] HPROT, 
	input	[2:0] HSIZE, 
	input	[2:0] HBURST, 
	input	[1:0] HTRANS, 
	input	HSEL, 
	input	HWRITE, 
	input 	HMASTLOCK, 
	input	HREADY, 

	output reg 	HREADYOUT, 
	output 		[1:0] HRESP, 
	output 		[31:0] HRDATA,

	output		DAC_DIN, 
	output		DAC_SCLK, 
	output		DAC_SYNC, 
	output		DAC_CLR, 
	output		DAC_LDAC
);

assign HRESP = 2'b00;
assign HRDATA = 32'hffffffff;

reg				hwrite_reg;
reg		[6:0]	haddr_reg;
reg		[2:0]	fsm;

reg		dac_start;
wire	dac_ready;
wire	[31:0] dac_data = HWDATA;
wire	[2:0] dac_command = haddr_reg[5:3];
wire	[2:0] dac_addr = haddr_reg[2:0];
wire	dac_debug = haddr_reg[6];

// any address start with 01XX_XYYY_0000 -> debug mode
// XXX -> dac commands, YYY -> dac addr
// ex: 0100_0000_0000 -> write to DAC-A register,
// See TI DAC8652 datasheet for possible commands/address

always @ (posedge HCLK)
begin
	if (~HRESETn)
	begin
		fsm	    <= 0;
		HREADYOUT <= 1;
		hwrite_reg <= 0;
		haddr_reg <= 0;
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
					haddr_reg <= HADDR[10:4];
				end
			end

			1:
			begin
				if (hwrite_reg)
				begin
					dac_start <= 1;
					fsm <= 2;
				end
				else
					fsm <= 3;
			end

			2:
			begin
				if (~dac_ready)
				begin
					fsm <= 3;
					dac_start <= 0;
				end
			end


			3:
			begin
				if (hwrite_reg)
				begin
					if (dac_ready)
					begin
						fsm <= 0;
						HREADYOUT <= 1;
					end
				end
				else
				begin
					fsm <= 0;
					HREADYOUT <= 1;
				end
			end


		endcase
	end
end

audio_dac_write dac0(
	.clk(HCLK),
	.resetn(HRESETn),
	.DATA_A(dac_data[15:0]),
	.DATA_B(dac_data[31:16]),
	.start(dac_start),

	.DEBUG(dac_debug),
	.COMMAND_IN(dac_command),
	.ADDR_IN(dac_addr),

	.ready(dac_ready),

	.LDAC(DAC_LDAC),
	.CLR(DAC_CLR),
	.DIN(DAC_DIN),
	.SCLK(DAC_SCLK),
	.SYNC(DAC_SYNC)
);

endmodule
