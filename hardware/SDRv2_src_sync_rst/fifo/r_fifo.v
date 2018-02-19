module r_fifo(DATA, Q, WCLOCK, RCLOCK, WE, RE, RESET, FULL, EMPTY);

parameter WIDTH = 8;
parameter DEPTH = 512;
parameter DEPTH_BITS = 9; // 2^9 = 512

input	[WIDTH-1:0] DATA;
output	[WIDTH-1:0] Q;

input	WCLOCK, RCLOCK, RESET, WE, RE;
output	FULL, EMPTY;

reg		[WIDTH-1:0] mem [0:DEPTH-1];
reg		[DEPTH_BITS-1:0] 	head, tail; 

assign Q = mem[tail];

assign EMPTY = ((head==0)&&(tail==DEPTH-1))? 1: (head == (tail+1))? 1:0;
assign FULL = (head==tail)? 1:0;

integer k;
/*
initial
begin
	for (k = 0; k < DEPTH - 1; k = k + 1)
    	mem[k] = 0;
end
*/

always @ (posedge RCLOCK)
begin
	if (~RESET)
	begin
		tail <= DEPTH - 1;
	end
	else
	begin
		if (RE & ~EMPTY)
		begin
			tail <= tail + 1;
		end
	end
end

always @ (posedge WCLOCK)
begin
	if (~RESET)
	begin
		head <= 0;
		for (k = 0; k < DEPTH; k = k + 1)
    		mem[k] <= 0;
	end
	else
	begin
		if (WE & ~FULL)
		begin
			mem[head] <= DATA;
			head <= head + 1;
		end
	end
end


endmodule
