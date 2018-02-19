/*
 * "Copyright (c) 2010-2011 The Regents of the University of Michigan.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * @author Ye-Sheng Kuo <samkuo@eecs.umich.edu>
 */

module max2831_spi(	DATOUT, CLKOUT, ENOUT,
					HSEL, HADDR, HWRITE, HSIZE, HBURST, HPROT, HTRANS, 
					HMASTLOCK, HREADY, HWDATA, HRESETn, HCLK,
					HREADYOUT, HRESP, HRDATA, 
					rx_agc_sel, rx_gain, radio_rdy, rx_agc_grant,
					freq_tx_req, freq_tx_grant, freq_dat_in, freq_MSB_LSB_in);

//AHB interface
input HSEL, HWRITE, HMASTLOCK, HREADY, HRESETn, HCLK;
input [31:0] HADDR, HWDATA;
input [2:0] HSIZE, HBURST;
input [3:0] HPROT;
input [1:0] HTRANS;

output HREADYOUT;
output [1:0] HRESP;
output [31:0] HRDATA;

output	DATOUT, CLKOUT, ENOUT;

input	rx_agc_sel;
input	[6:0] rx_gain;
output	radio_rdy, rx_agc_grant;
reg		[6:0] rx_gain_dat;
reg		radio_rdy, rx_agc_grant;

input	freq_tx_req;
input	[13:0]	freq_dat_in;
input	freq_MSB_LSB_in;
output	freq_tx_grant;
reg		freq_tx_grant;
reg		[13:0]	freq_dat;
reg		freq_MSB_LSB;

wire	d_out, c_out, e_out; // data, clk, enable
reg		spi_start;
assign	DATOUT = d_out;
assign	CLKOUT = c_out;
assign	ENOUT = e_out;

reg		hwrite_reg, hreadyout_reg;
reg		[1:0]	fsm;
reg		[17:0]	data_reg;
reg		[3:0]	haddr_reg;


// AHB
wire	HREADYOUT	= e_out & hreadyout_reg;
assign	HRDATA		= {14'b0, data_reg};
assign	HRESP		= 2'b00;


spi_tx t0(.clk(HCLK), .resetn(HRESETn), .data_in(data_reg), .start(spi_start), .data_out(d_out), .clk_out(c_out), .en_out(e_out));

always @ (posedge HCLK)
begin
    if (~HRESETn)
    begin
		fsm			<= 2'b00;
		data_reg    <= 0;
		hwrite_reg  <= 0;
		haddr_reg	<= 0;
		hreadyout_reg <= 1;
		rx_gain_dat	<= 0;
		radio_rdy <= 1;
		rx_agc_grant <= 0;
		freq_tx_grant <= 0;
		freq_dat	<= 0;
		freq_MSB_LSB <= 0;
    end
    else
    begin
		case (fsm)
		    2'b00:
		    begin
				if (HSEL && HREADY && HTRANS[1] && HSIZE==3'b010)
				begin
					hwrite_reg		<= HWRITE;
					haddr_reg		<= HADDR[5:2];
					fsm				<= 2'b01;
					hreadyout_reg	<= 0;
				end
				else if (rx_agc_sel)
				begin
					hwrite_reg		<= 1;
					fsm				<= 2'b01;
					hreadyout_reg	<= 0;
					rx_gain_dat		<= rx_gain;
					rx_agc_grant	<= 1;
				end
				else if (freq_tx_req)
				begin
					hwrite_reg		<= 1;
					fsm				<= 2'b01;
					hreadyout_reg	<= 0;
					freq_tx_grant	<= 1;
					freq_dat		<= freq_dat_in;
					freq_MSB_LSB	<= freq_MSB_LSB_in;
				end
		    end
		    
		    2'b01:
		    begin
				radio_rdy	<= 0;
				if (hwrite_reg)
				begin
					spi_start	<= 1;
					if (rx_agc_grant)
						data_reg <= {7'b0, rx_gain_dat, 4'b1011};
					else if (freq_tx_grant)
					begin
						if (freq_MSB_LSB_in)	// MSB
							data_reg <= {freq_dat, 4'b0100};	// MAX2831 Register 4
						else
							data_reg <= {freq_dat, 4'b0011};	// MAX2831 Register 3
					end
					else
						data_reg <= {HWDATA[13:0], haddr_reg};
					fsm <= 2'b10;
				end
				else
				begin
					fsm <= 2'b11;
				end

			end

			2'b10:
			begin
				if (e_out==0)
				begin
					spi_start <= 0;
					fsm <= 2'b11;
				end
			end

			2'b11:
			begin
                if (e_out==1)
                begin
				    fsm <= 2'b00;
                    hreadyout_reg <= 1;
					radio_rdy <= 1;
					rx_agc_grant	<= 0;
					freq_tx_grant <= 0;
                end
			end
		
		endcase
    end
end

endmodule

