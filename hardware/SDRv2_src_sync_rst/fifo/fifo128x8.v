module fifo128x8(DATA, Q, CLK, WE, RE, RESET, FULL, EMPTY);

parameter WIDTH = 8;
parameter HALF_WIDTH = (WIDTH>>1);
parameter DEPTH = 128;
parameter DEPTH_BITS = 7; // 2^7 = 128

input	[WIDTH-1:0] DATA;
output	[HALF_WIDTH-1:0] Q;

input	CLK, RESET, WE, RE;
output	FULL, EMPTY;

reg		[WIDTH-1:0] mem [0:DEPTH-1];
reg		[DEPTH_BITS-1:0] head, tail; 
reg		half;

assign Q = (half)? mem[tail][WIDTH-1:HALF_WIDTH] : mem[tail][HALF_WIDTH-1:0];

assign EMPTY = (half)? ((head==0)&&(tail==DEPTH-1))? 1 : (head == (tail+1))? 1 : 0 : 0;
assign FULL = (head==tail)? 1:0;

integer k;

`define SD #1

/*
initial
begin
	for (k = 0; k < DEPTH - 1; k = k + 1)
    	mem[k] = 0;
end
*/

always @ (posedge CLK)
begin
	if (~RESET)
	begin
		head <= 0;
		tail <= DEPTH - 1;
		half <= 1;
		for (k = 0; k < DEPTH; k = k + 1)
    		mem[k] <= 0;
	end
	else
	begin
		if (RE & ~EMPTY)
		begin
			half <= `SD ~half;
			if (half)
				tail <= `SD tail + 1;
		end

		if (WE & ~FULL)
		begin
			mem[head] <= `SD DATA;
			head <= `SD head + 1;
		end
	end
end


endmodule
