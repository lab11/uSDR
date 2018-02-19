
module ahb_int_fifo(	
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
	output		DAC_LDAC,

	input		audio_start,
	output		fifo_afull,
	output		fifo_aempty
);

assign HRESP = 2'b00;
assign HRDATA = 32'hffffffff;

reg				hwrite_reg;
reg		[2:0]	fsm;

`define HCLK_FREQ 60000000
`define FS 44100
parameter DEFAULT_CNT = 1357;	//  1361 = HCLK/FS, 4 additional cycle for fifo read

reg		[log2(DEFAULT_CNT-1)-1:0] audio_cnt;
reg		[2:0] audio_state;
reg		dac_start;
wire	dac_ready;

wire	[31:0] dac_data;
reg		fifo_WE, fifo_RE;
wire	fifo_full, fifo_empty;

always @ (posedge HCLK)
begin
	if (~HRESETn)
	begin
		fsm	    <= 0;
		HREADYOUT <= 1;
		hwrite_reg <= 0;
		fifo_WE <= 0;
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
				if (hwrite_reg)
				begin
					if (~fifo_full)
					begin
						fifo_WE <= 1;
						fsm <= 3;
					end
				end
				else
					fsm <= 3;
			end

			3:
			begin
				fifo_WE <= 0;
				fsm <= 0;
				HREADYOUT <= 1;
			end
		endcase
	end
end

always @ (posedge HCLK)
begin
	if (~HRESETn)
	begin
		audio_cnt <= DEFAULT_CNT;
		audio_state <= 0;
		fifo_RE <= 0;
		dac_start <= 0;
	end
	else
	begin
		case (audio_state)
			0:
			begin
				if (audio_start)
					audio_state <= 1;
			end

			1:
			begin
				audio_cnt <= DEFAULT_CNT;
				if (~fifo_empty)
				begin
					fifo_RE <= 1;
					audio_state <= 2;
				end
				else
				begin
					if (~audio_start)
						audio_state <= 0;
				end
			end

			2:
			begin
				audio_state <= 3;
				fifo_RE <= 0;
			end

			3:
			begin
				audio_state <= 4;
			end

			4:
			begin
				if (dac_ready)
				begin
					dac_start <= 1;
					audio_state <= 5;
				end
			end

			5:
			begin
				if (~dac_ready)
				begin
					dac_start <= 0;
					audio_state <= 6;
				end
			end

			6:
			begin
				if (audio_cnt)
					audio_cnt <= audio_cnt - 1'b1;
				else
					audio_state <= 1;
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

	.DEBUG(1'b0),
	.COMMAND_IN(3'b0),
	.ADDR_IN(3'b0),

	.ready(dac_ready),

	.LDAC(DAC_LDAC),
	.CLR(DAC_CLR),
	.DIN(DAC_DIN),
	.SCLK(DAC_SCLK),
	.SYNC(DAC_SYNC)
);

fifo128x32 f0(
	.DATA(HWDATA), 
	.Q(dac_data),
	.CLK(HCLK),
	.WE(fifo_WE),
	.RE(fifo_RE),
	.RESET(HRESETn),
	.FULL(fifo_full),
	.AFULL(fifo_afull),
	.EMPTY(fifo_empty),
	.AEMPTY(fifo_aempty)
);

function integer log2;
	input [31:0] value;
	for (log2=0; value>0; log2=log2+1)
	value = value>>1;
endfunction

endmodule
