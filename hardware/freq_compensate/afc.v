module auto_freq_ctl(	clk, resetn,
						max2831_ready, data_out, MSB_LSB, direct_in, freq_tx_req, 
						freq_tx_grant, afc_en, afc_enable_in, freq_out, channels);
					
input			clk, resetn;
input			afc_enable_in;
input	[3:0]	channels;
output	[23:0]	freq_out;
reg		[1:0]	fsm;
reg				hready_reg, hwrite_reg;
reg		[3:0]	haddr_reg;
reg		[15:0]	HRDATA_reg;

input			max2831_ready;
input	[1:0]	direct_in;
output	[13:0]	data_out;
output			MSB_LSB;
output			freq_tx_req; 
input			freq_tx_grant;
output			afc_en;

reg		[2:0]	state, next_state;
reg		[23:0]	freq, next_freq;
reg		[13:0]	data_out, next_data_out;
reg				afc_en_reg, next_afc_en_reg;
reg		[9:0]	delay_en, next_delay_en;
reg				MSB_LSB, next_MSB_LSB;
reg				freq_tx_req, next_freq_tx_req;
reg				up_dn, next_up_dn;

wire	[23:0]	freq_upper_bound, freq_lower_bound, freq_center;
wire	[23:0]	freq_plus1 = freq + 1;
wire	[23:0]	freq_minus1 = freq - 1;

`define FREQ_INIT				3'b000
`define FREQ_RST				3'b001
`define DELAY					3'b010
`define FREQ_LOAD				3'b011
`define FREQ_UPDATE				3'b100
`define FREQ_TX_MSB				3'b101
`define WAIT_FOR_MSB_COMPLETE 	3'b110
`define	FREQ_TX_LSB				3'b111

assign afc_en = (afc_enable_in&afc_en_reg);
assign freq_out = freq;

frequency_table ft0(.channels(channels), .freq_upper_bound(freq_upper_bound), 
					.freq_lower_bound(freq_lower_bound), .freq_center(freq_center));

always @ (posedge clk)
begin
	if (~resetn)
	begin
		freq 		<= 0;
		state 		<= `FREQ_INIT;
		freq_tx_req <= 0;
		data_out 	<= 0;
		MSB_LSB		<= 0;
		up_dn		<= 0;
		delay_en	<= 0;
		afc_en_reg	<= 0;
	end
	else
	begin
		freq 		<= next_freq;
		state 		<= next_state;
		freq_tx_req <= next_freq_tx_req;
		data_out 	<= next_data_out;
		MSB_LSB		<= next_MSB_LSB;
		up_dn 		<= next_up_dn;
		delay_en	<= next_delay_en;
		afc_en_reg	<= next_afc_en_reg;
	end
end

always @ *
begin
	next_freq = freq;
	next_state = state;
	next_freq_tx_req = freq_tx_req;
	next_data_out = data_out;
	next_MSB_LSB = MSB_LSB;
	next_up_dn = up_dn;
	next_delay_en = delay_en;
	next_afc_en_reg = afc_en_reg;

	case (state)
		`FREQ_INIT:
		begin
			next_freq = freq_center;
			next_state = `FREQ_RST;
		end

		`FREQ_RST:
		begin
			next_delay_en = 0;
			next_afc_en_reg = 0;
			if (afc_enable_in)
				next_state = `DELAY;
		end

		`DELAY:
		begin
			next_delay_en = delay_en + 1;
			if (delay_en==10'd240)		// delay for 5us
				next_state = `FREQ_LOAD;
		end

		`FREQ_LOAD:
		begin
			next_afc_en_reg = 1;
			case (direct_in)
				2'b01:
				begin
					if (freq<freq_upper_bound)
					begin
						next_freq = freq_plus1;
						next_state = `FREQ_UPDATE;
						next_up_dn = 1;
					end
					else
						next_state = `FREQ_RST;
				end

				2'b10:
				begin
					if (freq>freq_lower_bound)
					begin
						next_freq = freq_minus1;
						next_state = `FREQ_UPDATE;
						next_up_dn = 0;
					end
					else
						next_state = `FREQ_RST;
				end

				default: begin next_state = state; end
			endcase
		end

		`FREQ_UPDATE:
		begin
			if (((freq[1:0]==0)&&(up_dn==1))||((freq[1:0]==2'b11)&&(up_dn==0)))	// both MSB and LSB need to be transmitted
			begin
				if (max2831_ready)
				begin
					next_state = `FREQ_TX_MSB;
					next_freq_tx_req = 1;
				end
				next_MSB_LSB = 1;
				next_data_out = freq[15:2];
			end
			else
			begin				// only LSB needs to be transmitted
				if (max2831_ready)
				begin
					next_state = `FREQ_TX_LSB;
					next_freq_tx_req = 1;
				end
				next_MSB_LSB = 0;
				next_data_out = {freq[1:0], 4'b0, freq[23:16]};
			end
		end

		`FREQ_TX_MSB:
		begin
			if (freq_tx_grant&~max2831_ready)
			begin
				next_freq_tx_req = 0;
				next_state = `WAIT_FOR_MSB_COMPLETE;
			end
		end

		`WAIT_FOR_MSB_COMPLETE:
		begin
			next_MSB_LSB = 0;
			if (max2831_ready)
			begin
				next_state = `FREQ_TX_LSB;
				next_freq_tx_req = 1;
				next_data_out = {freq[1:0], 4'b0, freq[23:16]};
			end
		end

		`FREQ_TX_LSB:
		begin
			if (freq_tx_grant&~max2831_ready)
			begin
				next_freq_tx_req = 0;
				if (afc_enable_in)
					next_state = `FREQ_RST;
				else
					next_state = `FREQ_INIT;
			end
		end
	endcase
end

endmodule
