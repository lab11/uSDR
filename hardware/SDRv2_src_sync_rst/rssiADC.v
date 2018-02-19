module serialADC(clk, resetn, en, shdn, data_rdy, adc_rdy, data_out, sclk, cs, sdata);

input				clk, resetn, en, shdn;
output				data_rdy, adc_rdy;
output		[7:0]	data_out;

output				sclk, cs;
input				sdata;

reg					clk_en, next_clk_en;
reg					cs, next_cs;
reg					adc_rdy, next_adc_rdy;
reg					data_rdy, next_data_rdy;
reg			[7:0]	data_out, next_data_out;
reg			[4:0]	counter, next_counter;
reg			[1:0]	state, next_state;
reg			[1:0]	adc_state, next_adc_state;

parameter ADC_RST =		2'b00;
parameter ADC_START =	2'b01;
parameter ADC_RDY =		2'b10;
parameter ADC_PD =		2'b11;

assign sclk = (clk & clk_en);

always @ (posedge clk)
begin
	if (~resetn)
	begin
		state <= ADC_RST;
		adc_state <= 0;
		clk_en <= 0;
		counter <= 0;
		cs <= 1;
		data_out <= 0;
		adc_rdy <= 0;
		data_rdy <= 0;
	end
	else
	begin
		state <= next_state;
		adc_state <= next_adc_state;
		clk_en <= next_clk_en;
		counter <= next_counter;
		cs <= next_cs;
		data_out <= next_data_out;
		adc_rdy <= next_adc_rdy;
		data_rdy <= next_data_rdy;
	end
end

always @ *
begin
	next_state = state;
	next_adc_state = adc_state;
	next_clk_en = clk_en;
	next_counter = counter;
	next_cs = cs;
	next_data_out = data_out;
	next_adc_rdy = adc_rdy;
	next_data_rdy = data_rdy;

	case (state)
		ADC_RST:
		begin
			next_state = ADC_START;
			next_adc_state = 0;
			next_clk_en = 1;
			next_cs = 0;
		end

		ADC_START:
		begin
			next_counter = counter + 1;
			if (counter==18)
			begin
				if (adc_state==2)
					next_state = ADC_PD;
				else
				begin
					next_state = ADC_RDY;
					next_adc_rdy = 1;
				end
			end
			else if (counter>14)
			begin
				next_cs = 1;
				if (adc_state==1)
					next_data_rdy = 1;
			end
			else if ((counter>2)&&(counter<11))
				next_data_out = {data_out[6:0], sdata};
			// sleep mode
			else if ((counter==2)&&(adc_state==2))
				next_cs = 1;
		end

		ADC_RDY:
		begin
			next_adc_state = 1;
			next_counter = 0;
			if (en)
			begin
				next_adc_rdy = 0;
				next_cs = 0;
				next_data_rdy = 0;
				next_state = ADC_START;
				if (shdn)
					next_adc_state = 2;
			end
		end

		ADC_PD:
		begin
			next_counter = 0;
			next_adc_state = 0;
			next_clk_en = 0;
			if (~shdn)
			begin
				next_cs = 0;
				next_state = ADC_START;
				next_clk_en = 1;
			end
		end

	endcase
end


endmodule
