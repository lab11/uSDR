module testbench();

reg		[7:0]	DATA;
wire	[7:0]	Q;
reg				WCLOCK, RCLOCK, WE, RE, RESET;
wire			FULL, EMPTY;

r_fifo r0(DATA, Q, WCLOCK, RCLOCK, WE, RE, RESET, FULL, EMPTY);

initial begin
	WCLOCK = 0;
	RCLOCK = 0;
	RESET = 0;
	WE = 0;
	RE = 0;
	DATA = 0;
	@ (negedge WCLOCK)
	@ (negedge WCLOCK)
	@ (negedge WCLOCK)
	@ (negedge WCLOCK)
	@ (negedge WCLOCK)
	RESET = 1;
	# 100
	@ (negedge WCLOCK)
		DATA = 8'haa;
		WE = 1;
	
	@ (negedge WCLOCK)
		DATA = 8'hbb;
		WE = 1;

	@ (negedge WCLOCK)
		DATA = 8'hcc;
		WE = 1;

	@ (negedge WCLOCK)
		DATA = 8'hdd;
		WE = 1;

	@ (negedge WCLOCK)
		DATA = 8'hee;
		WE = 1;

	@ (negedge WCLOCK)
		WE = 0;
	
	@ (negedge RCLOCK)
		RE = 1;

	@ (negedge RCLOCK)
		RE = 1;

	@ (negedge RCLOCK)
		RE = 1;

	@ (negedge RCLOCK)
		RE = 1;

	@ (negedge RCLOCK)
		RE = 1;

	@ (negedge RCLOCK)
		RE = 1;

	@ (negedge RCLOCK)
		RE = 1;
	
	@(negedge WCLOCK)
		RESET = 0;
	@(negedge WCLOCK)
		RESET = 1;

	@ (negedge WCLOCK)
		DATA = 8'h11;
		WE = 1;
	
	@ (negedge WCLOCK)
		DATA = 8'h22;
		WE = 1;

	@ (negedge WCLOCK)
		DATA = 8'h33;
		WE = 1;

	@ (negedge WCLOCK)
		DATA = 8'h44;
		WE = 1;

	@ (negedge WCLOCK)
		DATA = 8'h55;
		WE = 1;

	@ (negedge WCLOCK)
		WE = 0;
	
	@ (negedge RCLOCK)
		RE = 1;

	@ (negedge RCLOCK)
		RE = 1;

	@ (negedge RCLOCK)
		RE = 1;

	@ (negedge RCLOCK)
		RE = 1;

	@ (negedge RCLOCK)
		RE = 1;

	@ (negedge RCLOCK)
		RE = 1;

	@ (negedge RCLOCK)
		RE = 1;
	

	$stop;
end

always #5 WCLOCK = ~WCLOCK;

always #15 RCLOCK = ~RCLOCK;


endmodule
