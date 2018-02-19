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

module max2831_spi(	
	// APB interface
	input	PCLK,
	input	PRESETn, 
	input	[31:0] PADDR, 
	input	PSELx, 
	input	PENABLE,
	input	PWRITE, 
	input	[31:0] PWDATA, 
	output	reg PREADY, 
	output	[31:0] PRDATA,
	output	PSLVERR,

	output	DATOUT, 
	output	CLKOUT, 
	output	ENOUT,
					
	input	rx_agc_sel, 
	input	[6:0] rx_gain, 
	output	reg rx_gain_rdy, 
	output	reg rx_agc_data_got
);

assign PSLVERR = 1'b0;

reg		[6:0] rx_gain_dat;

reg		spi_start;

reg		[2:0]	fsm;
reg		[17:0]	data_reg;

// AHB
assign	PRDATA		= {14'b0, data_reg};

spi_tx t0(.clk(PCLK), .resetn(PRESETn), .data_in(data_reg), .start(spi_start), .data_out(DATOUT), .clk_out(CLKOUT), .en_out(ENOUT));

always @ (posedge PCLK or negedge PRESETn)
begin
    if (~PRESETn)
    begin
		fsm			<= 0;
		data_reg    <= 0;
		PREADY		<= 0;
		rx_gain_dat	<= 0;
		rx_gain_rdy <= 1;
		rx_agc_data_got <= 0;
    end
    else
    begin
		case (fsm)
		    0:
		    begin
				if (PSELx & PENABLE)
					fsm				<= 1;
				else if (rx_agc_sel)
				begin
					fsm				<= 1;
					rx_gain_dat		<= rx_gain;
					rx_agc_data_got	<= 1;
				end
		    end
		    
		    1:
		    begin
				rx_gain_rdy	<= 0;
				if (PWRITE | rx_agc_data_got)
				begin
					spi_start	<= 1;
					if (rx_agc_data_got)
						data_reg <= {7'b0, rx_gain_dat, 4'b1011};
					else
						data_reg <= {PWDATA[13:0], PADDR[5:2]};
					fsm <= 2;
				end
				else
				begin
					fsm <= 3;
				end

			end

			2:
			begin
				if (ENOUT==0)
				begin
					spi_start <= 0;
					fsm <= 3;
				end
			end

			3:
			begin
                if (ENOUT==1)
                begin
					rx_gain_rdy <= 1;
					if (rx_agc_data_got)
					begin
						rx_agc_data_got	<= 0;
						fsm <= 0;
					end
					else
					begin
						PREADY <= 1;
						fsm <= 4;
					end
                end
			end

			4:
			begin
				if (~PSELx)
				begin
					PREADY <= 0;
					fsm <= 0;
				end
			end
		
		endcase
    end
end

endmodule

