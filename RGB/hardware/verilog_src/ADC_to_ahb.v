
module ADC_to_ahb(  HSEL, HADDR, HWRITE, HSIZE, HBURST, HPROT, HTRANS, 
					HMASTLOCK, HREADY, HWDATA, HRESETn, HCLK,
					HREADYOUT, HRESP, HRDATA,
					ADC_DIN, ADC_EOC, ADC_A0, ADC_A1, ADC_CONVST, ADC_PD,
					ADC_RD, ADC_CS);

// IOs for AHB bus
input 			HSEL, HWRITE, HMASTLOCK, HREADY, HRESETn, HCLK;
input 	[31:0] 	HADDR, HWDATA;
input 	[2:0] 	HSIZE, HBURST;
input 	[3:0] 	HPROT;
input 	[1:0] 	HTRANS;

output 			HREADYOUT;
output 	[1:0] 	HRESP;
output 	[31:0] 	HRDATA;

// IOs for ADC
input	[7:0]	ADC_DIN;
input			ADC_EOC;
output			ADC_A0, ADC_A1, ADC_CONVST, ADC_PD, ADC_RD, ADC_CS;

reg				adc_start;
wire			adc_ready;
wire	[7:0]	adc_ch0, adc_ch1, adc_ch2;

reg				HREADYOUT, hwrite_reg;
reg		[23:0]	HRDATA_reg;

assign HRDATA		= {8'b0, HRDATA_reg};
assign HRESP		= 2'b00;

reg 	[2:0] 	fsm;


always @ (posedge HCLK)
begin
	if (~HRESETn)
	begin
		HREADYOUT <= 1;
		fsm <= 0;
		hwrite_reg <= 0;
		HRDATA_reg <= 0;
		adc_start <= 0;
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
				// read
				if (~hwrite_reg)
					fsm <= 2;
				else
					fsm <= 5;
			end

			2:
			begin
				if (adc_ready)
				begin
					adc_start <= 1;
					fsm <= 3;
				end
			end

			3:
			begin
				if (~adc_ready)
				begin
					fsm <= 4;
					adc_start <= 0;
				end
			end

			4:
			begin
				if (adc_ready)
				begin
					HRDATA_reg <= {adc_ch0, adc_ch1, adc_ch2};
					fsm <= 5;
				end
			end

			5:
			begin
				fsm <= 0;
				HREADYOUT <= 1;
			end
		endcase
	end
end

adc_read adc0(	.clk(HCLK), .resetn(HRESETn), .pwr(1'b1), .Data_in(ADC_DIN), .EOC(ADC_EOC), .start(adc_start),
				.A0(ADC_A0), .A1(ADC_A1), .CONVST(ADC_CONVST), .PD(ADC_PD), .RD(ADC_RD), .CS(ADC_CS), .ready(adc_ready),
				.ch0_data(adc_ch0), .ch1_data(adc_ch1), .ch2_data(adc_ch2));
endmodule
