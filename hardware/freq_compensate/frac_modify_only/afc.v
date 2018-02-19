module auto_freq_ctl(	HSEL, HADDR, HWRITE, HSIZE, HBURST, HPROT, HTRANS, 
						HMASTLOCK, HREADY, HWDATA, HRESETn, HCLK,
						HREADYOUT, HRESP, HRDATA, 
						max2831_ready, data_out, MSB_LSB, direct_in, freq_tx_req, freq_tx_grant, afc_en);
					
input 			HSEL, HWRITE, HMASTLOCK, HREADY, HRESETn, HCLK;
input 	[31:0] 	HADDR, HWDATA;
input 	[2:0] 	HSIZE, HBURST;
input 	[3:0] 	HPROT;
input 	[1:0] 	HTRANS;

output 			HREADYOUT;
output 	[1:0] 	HRESP;
output 	[31:0] 	HRDATA;

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
reg		[15:0]	freq, next_freq;
reg		[13:0]	data_out, next_data_out;
reg				en;
reg				MSB_LSB, next_MSB_LSB;
reg				freq_tx_req, next_freq_tx_req;
reg				up_dn, next_up_dn;

reg		[15:0]	initial_freq_frac;	// D13:D0 of R4 + D13:D12 of R3, freq_resolution = 305.2Hz
reg		[7:0]	initial_freq_intg;
wire	[15:0]	freq_upper_bound = initial_freq_frac + 63;
wire	[15:0]	freq_lower_bound = initial_freq_frac - 63;

`define FREQ_RST				3'b000
`define FREQ_LOAD				3'b001
`define FREQ_UPDATE				3'b010
`define FREQ_TX_MSB				3'b011
`define WAIT_FOR_MSB_COMPLETE 	3'b100
`define	FREQ_TX_LSB				3'b101

assign HRDATA		= {16'b0, HRDATA_reg};
assign HRESP		= 2'b00;
assign HREADYOUT = hready_reg;
assign afc_en = en;

always @ (posedge HCLK)
begin
	if (~HRESETn)
	begin
		fsm <= 2'b00;
		hready_reg <= 1;
		hwrite_reg <= 0;
		haddr_reg <= 0;
		en <= 0;
		initial_freq_frac <= 16384;
		initial_freq_intg <= 120;
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
					haddr_reg	<= HADDR[3:0];
				end
			end

			2'b01:
			begin
				fsm <= 2'b10;
				if (hwrite_reg)	// write
				begin
					case (haddr_reg[3:2])
						2'b00:
						begin
							en <= HWDATA[0];
						end

						2'b01:
						begin
							initial_freq_frac <= {HWDATA[13:0], 2'b00};
						end

						2'b10:
						begin
							initial_freq_intg <= HWDATA[7:0];
						end
					endcase
				end
				else	// read
				begin
					case (haddr_reg[3:2])
						2'b00:
						begin
							HRDATA_reg <= {15'b0, en};
						end

						2'b01:
						begin
							HRDATA_reg <= freq;
						end
					endcase
				end
			end

			2'b10:
			begin
				fsm <= 2'b11;
			end

			2'b11:
			begin
				fsm <= 2'b00;
				hready_reg <= 1;
			end

		endcase
	end
end

always @ (posedge HCLK)
begin
	if (~HRESETn)
	begin
		freq 		<= 0;
		state 		<= `FREQ_RST;
		freq_tx_req <= 0;
		data_out 	<= 0;
		MSB_LSB		<= 0;
		up_dn		<= 0;
	end
	else
	begin
		freq 		<= next_freq;
		state 		<= next_state;
		freq_tx_req <= next_freq_tx_req;
		data_out 	<= next_data_out;
		MSB_LSB		<= next_MSB_LSB;
		up_dn 		<= next_up_dn;
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

	case (state)
		`FREQ_RST:
		begin
			next_freq = initial_freq_frac;
			if (en)
				next_state = `FREQ_LOAD;
		end

		`FREQ_LOAD:
		begin
			if ((freq<freq_upper_bound)&&(direct_in==2'b01))	// increase freq;
			begin
				next_freq = freq + 1;
				next_state = `FREQ_UPDATE;
				next_up_dn = 1;
			end

			if ((freq>freq_lower_bound)&&(direct_in==2'b10))	// decrease freq;
			begin
				next_freq = freq - 1;
				next_state = `FREQ_UPDATE;
				next_up_dn = 0;
			end
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
				next_data_out = {freq[1:0], 4'b0, initial_freq_intg};
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
				next_data_out = {freq[1:0], 4'b0, initial_freq_intg};
			end
		end

		`FREQ_TX_LSB:
		begin
			if (freq_tx_grant&~max2831_ready)
			begin
				next_freq_tx_req = 0;
				if (en)
					next_state = `FREQ_LOAD;
				else
					next_state = `FREQ_RST;
			end
		end
	endcase
end

endmodule
