module testbench();

reg		clk, resetn;
wire	[1:0]	direction;
reg	signed [7:0] I, Q;

freq_correction f0(.clk(clk), .resetn(resetn), .en(1'b1), .freq_mod(direction), .in_phase(I), .quad_phase(Q));

integer dat, cnt, ln_cnt;

`define SD 1

initial
begin
	dat = $fopen("IQ_deltaF.txt", "r");
	clk = 0;
	resetn = 0;
	@(negedge clk);
	@(negedge clk);
	@(negedge clk);
	resetn = 1;
	@(negedge clk);
	@(negedge clk);
	ln_cnt = 0;
	while (ln_cnt<8800)
	begin
		ln_cnt = ln_cnt + 1;
		cnt = $fscanf(dat, "%d %d", I, Q);
		@(negedge clk);
		@(negedge clk);
		@(negedge clk);
	end
	$fclose(dat);
	$finish;
end

always #5 clk = ~clk;


endmodule
