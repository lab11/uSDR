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
/* Function description:
* This module takes I/Q siganl demudulate to amplitude-modulation signal.
* Also, it takes 8 samples match filter*/

module fx_demux(in_phase, quad_phase, adc_clock, resetn, bit_decode, signal_strength);

input			adc_clock, resetn;
input	[7:0]	in_phase, quad_phase;
output			bit_decode;
output	[7:0]	signal_strength;


reg		[7:0]	I_smp, Q_smp, I_pre_smp, Q_pre_smp;
// Mult IOs
wire signed	[15:0]	mult0, mult1;
wire signed	[16:0]	mult_result = mult1 - mult0;
reg	 signed	[16:0]	mult_pre_result;
parameter MSB_RES = 16;	// result is [16:0]

wire signed	[MSB_RES+1:0]	mult_result_avg = mult_result + mult_pre_result;
reg		[MSB_RES+1:0]	mult_res0, mult_res1, mult_res2, mult_res3, mult_res4, mult_res5, mult_res6, mult_res7, mult_res8;
wire	[MSB_RES+4:0]	mult_res0_extend = {{3{mult_res0[MSB_RES+1]}}, mult_res0};
wire	[MSB_RES+4:0]	mult_res8_extend = {{3{mult_res8[MSB_RES+1]}}, mult_res8};
wire	[MSB_RES+4:0]	mult_res8_extend_rev = (mult_res8_extend[MSB_RES+4])? ~(mult_res8_extend - 1) : (~mult_res8_extend + 1); //-mult_res8_extend
reg		[MSB_RES+4:0]	result_integ;

wire	[7:0]	I_smp_abs = (I_smp[7])? ~(I_smp) : I_smp;
wire	[7:0]	Q_smp_abs = (Q_smp[7])? ~(Q_smp) : Q_smp;
wire	[7:0]	signal_strength = I_smp_abs[6:0] + Q_smp_abs[6:0];
wire	bit_decode = (result_integ[MSB_RES+4])? 0 : 1;

// match filtered FM demux
always @ (posedge adc_clock)
begin
	if (~resetn)
	begin
		mult_pre_result		<= 0;
		result_integ		<= 0;
		mult_res0			<= 0;
		mult_res1			<= 0;
		mult_res2			<= 0;
		mult_res3			<= 0;
		mult_res4			<= 0;
		mult_res5			<= 0;
		mult_res6			<= 0;
		mult_res7			<= 0;
		mult_res8			<= 0;
	end
	else
	begin
		mult_pre_result		<= mult_result;
		result_integ		<= result_integ + mult_res0_extend + mult_res8_extend_rev; // mult_integ + mult_res0_extend - mult_res8_extend
		mult_res0			<= mult_result_avg;
		mult_res1			<= mult_res0;
		mult_res2			<= mult_res1;
		mult_res3			<= mult_res2;
		mult_res4			<= mult_res3;
		mult_res5			<= mult_res4;
		mult_res6			<= mult_res5;
		mult_res7			<= mult_res6;
		mult_res8			<= mult_res7;
	end
end

always @ (posedge adc_clock)
begin
	if (~resetn)
	begin
		I_smp <= 0;
		Q_smp <= 0;
		I_pre_smp <= 0;
		Q_pre_smp <= 0;
	end
	else
	begin
		I_smp <= in_phase;
		Q_smp <= quad_phase;
		I_pre_smp <= I_smp;
		Q_pre_smp <= Q_smp;
	end
end

signed_mult8x8 mult_0(
	.DataA(I_smp),
	.DataB(Q_pre_smp),
	.Mult(mult0)
);

signed_mult8x8 mult_1(
	.DataA(Q_smp),
	.DataB(I_pre_smp),
	.Mult(mult1)
);


endmodule
