module testbench();

reg		[7:0]	DATA;
wire	[3:0]	Q;
reg				CLK, WE, RE, RESET;
wire			FULL, EMPTY;

fifo128x8 #(.WIDTH(8), .DEPTH(128), .DEPTH_BITS(7)) r0(DATA, Q, CLK, WE, RE, RESET, FULL, EMPTY);

initial begin
	CLK = 0;
	RESET = 0;
	WE = 0;
	RE = 0;
	DATA = 0;
	@ (negedge CLK)
	@ (negedge CLK)
	@ (negedge CLK)
	@ (negedge CLK)
	@ (negedge CLK)
	RESET = 1;
	# 100
	@ (negedge CLK)
		DATA = 8'ha1;
		WE = 1;
	
	@ (negedge CLK)
		DATA = 8'h2b;
		WE = 1;

	@ (negedge CLK)
		DATA = 8'hc3;
		WE = 1;

	@ (negedge CLK)
		DATA = 8'h4d;
		WE = 1;

	@ (negedge CLK)
		DATA = 8'hef;
		WE = 1;

	@ (negedge CLK)
		WE = 0;
	
	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;
	
	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;
	
	@ (negedge CLK)
		RE = 0;
	
	@ (negedge CLK)
		DATA = 8'ha1;
		WE = 1;
	
	@ (negedge CLK)
		DATA = 8'h2b;
		WE = 1;

	@ (negedge CLK)
		WE = 0;
		RE = 1;

	@ (negedge CLK)
		RE = 1;
	
	@ (negedge CLK)
		RE = 1;
	
	@ (negedge CLK)
		RE = 1;
	
	@ (negedge CLK)
		RE = 1;
	
	@ (negedge CLK)
		RE = 1;
	
	@ (negedge CLK)
		RE = 1;
	
	@(negedge CLK)
		RESET = 0;
		RE = 0;
	@(negedge CLK)
		RESET = 1;

	@ (negedge CLK)
		DATA = 8'h11;
		WE = 1;
	
	@ (negedge CLK)
		DATA = 8'h22;
		WE = 1;

	@ (negedge CLK)
		DATA = 8'h33;
		WE = 1;

	@ (negedge CLK)
		DATA = 8'h44;
		WE = 1;

	@ (negedge CLK)
		DATA = 8'h55;
		WE = 1;

	@ (negedge CLK)
		WE = 0;
	
	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;
	
	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;

	@ (negedge CLK)
		RE = 1;
	

	$stop;
end

always #5 CLK = ~CLK;



endmodule
