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

module radio_ctl( 	HSEL, HADDR, HWRITE, HSIZE, HBURST, HPROT, HTRANS, 
					HMASTLOCK, HREADY, HWDATA, HRESETn, HCLK,
					HREADYOUT, HRESP, HRDATA, mode_out, ack_en, led_out, agc_latch, agc_en,
					ack_flush, afc_enable, channels, freq_in);

// IOs for AHB bus
input 			HSEL, HWRITE, HMASTLOCK, HREADY, HRESETn, HCLK;
input 	[31:0] 	HADDR, HWDATA;
input 	[2:0] 	HSIZE, HBURST;
input 	[3:0] 	HPROT;
input 	[1:0] 	HTRANS;
input			agc_latch;

output 			HREADYOUT;
output 	[1:0] 	HRESP;
output 	[31:0] 	HRDATA;

// IOs for frequency correction
input	[23:0]	freq_in;
output			afc_enable;
output	[3:0]	channels;
reg		[3:0]	channels;
reg				afc_enable;

output			mode_out;
output			ack_en;
output	[7:0]	led_out;
output			agc_en;
output			ack_flush;
reg				ack_flush;

reg				mode_out;
reg				ack_en;
reg		[7:0]	led_out;
wire			agc_en;
reg		[1:0]	agc_mode;

reg				hwrite_reg, hready_reg;
reg		[7:0]	haddr_reg;
reg		[1:0]	fsm;

`define	MODE_TX		1
`define	MODE_RX		0
assign	agc_en = (agc_mode==2'b11)? 1 : (agc_mode==0)? 0 : ~agc_latch;

reg		[15:0]	HRDATA_reg;
assign HRDATA		= {16'b0, HRDATA_reg};
assign HRESP		= 2'b00;
assign HREADYOUT = hready_reg;


always @ (posedge HCLK)
begin
	if (~HRESETn)
	begin
		fsm	    <= 2'b00;
		hwrite_reg  <= 1'b0;
		hready_reg	<= 1;
		HRDATA_reg	<= 0;
		haddr_reg	<= 0;
		mode_out	<= `MODE_RX;
		ack_en		<= 1;
		led_out		<= 8'hff;
		agc_mode	<= 0;
		ack_flush	<= 0;
		channels	<= 0;
		afc_enable	<= 0;
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
					haddr_reg	<= HADDR[7:0];
				end
			end

			2'b01:
			begin
				fsm	<= 2'b10;
				case (hwrite_reg)
					1:
					begin
						case (haddr_reg[7:4])

							4'h1:
							begin
								ack_en <= HWDATA[0];
							end

							4'h2:
							begin
								led_out <= (~HWDATA[7:0]);
							end

							4'h4:
							begin
								agc_mode <= HWDATA[1:0];
							end

							4'h5:
							begin
								ack_flush <= HWDATA[0];
							end

							4'h6:
							begin
								channels <= HWDATA[7:4];
								afc_enable <= HWDATA[0];
							end

							4'h0:
							begin
								mode_out <= HWDATA[0];	// 0000
							end
						endcase
					end
					
					0:	// read
					begin
						case (haddr_reg[7:4])
							4'h0:	// 0000
							begin
								HRDATA_reg <= {15'b0, mode_out};
							end

							4'h1:
							begin
								HRDATA_reg <= {15'b0, ack_en};
							end

							4'h2:
							begin
								HRDATA_reg <=  {8'b0, ~led_out};
							end

							4'h6:
							begin
								HRDATA_reg <= {8'b0, channels, 3'b0, afc_enable};
							end

							4'h7:
							begin
								HRDATA_reg <= freq_in[15:0];
							end

							4'h8:
							begin
								HRDATA_reg <= {8'b0, freq_in[23:16]};
							end


							4'h4:
							begin
								HRDATA_reg <= {14'b0, agc_mode};
							end


						endcase
					end
				endcase
			end

			2'b10:
			begin
				ack_flush <= 0;
				fsm <= 2'b11;
			end

			2'b11:
			begin
				hready_reg <= 1;
				fsm		<= 2'b00;
			end
		endcase
	end
end

endmodule
