
module audio_dac_write(
	input clk,
	input resetn,
	input [15:0] DATA_A,
	input [15:0] DATA_B,
	input start,

	input DEBUG,
	input [2:0] COMMAND_IN,
	input [2:0] ADDR_IN,

	output reg ready,

	output LDAC,
	output CLR,
	output reg DIN,
	output reg SCLK,
	output reg SYNC
);

// no input buffer, data cannot be changed during writing

wire	[5:0] COMMAND_A = 6'b0;			// write to DAC-A input register
wire	[5:0] COMMAND_B = 6'b010_001;	// write to DAC-B input register and software LDAC

assign LDAC = 1'b1;
assign CLR = 1'b1;

parameter STATE_IDLE = 0;
parameter STATE_SYNC_HOLD = 1;
parameter STATE_START_TX = 2;
parameter STATE_COMMAND = 3;
parameter STATE_DATA = 4;
parameter STATE_EOT = 5;
parameter STATE_WAIT = 6;

reg		[2:0] state, next_state;
reg		next_ready;
reg		next_din, next_sclk, next_sync;
reg		[3:0] bit_cnt, next_bit_cnt;
reg		data_b, next_data_b;

wire	CMD_A_BIT_EXTRACT = ((COMMAND_A & (1'b1<<bit_cnt))==0)? 1'b0 : 1'b1;
wire	CMD_B_BIT_EXTRACT = ((COMMAND_B & (1'b1<<bit_cnt))==0)? 1'b0 : 1'b1;
wire	DAT_A_BIT_EXTRACT = ((DATA_A & (1'b1<<bit_cnt))==0)? 1'b0 : 1'b1;
wire	DAT_B_BIT_EXTRACT = ((DATA_B & (1'b1<<bit_cnt))==0)? 1'b0 : 1'b1;

wire	CMD_BIT_EXTRACT = (({COMMAND_IN, ADDR_IN} & (1'b1<<bit_cnt))==0)? 1'b0 : 1'b1;

always @ (posedge clk)
begin
	if (~resetn)
	begin
		state <= STATE_IDLE;
		ready <= 1;
		DIN <= 1;
		SCLK <= 1;
		SYNC <= 1;
		bit_cnt <= 5;
		data_b <= 0;
	end
	else
	begin
		state <= next_state;
		ready <= next_ready;
		DIN <= next_din;
		SCLK <= next_sclk;
		SYNC <= next_sync;
		bit_cnt <= next_bit_cnt;
		data_b <= next_data_b;
	end
end

always @ *
begin
	next_state = state;
	next_ready = ready;
	next_din = DIN;
	next_sclk = SCLK;
	next_sync = SYNC;
	next_bit_cnt = bit_cnt;
	next_data_b = data_b;

	case (state)
		STATE_IDLE:
		begin
			if (start)
			begin
				next_state = STATE_SYNC_HOLD;
				next_ready = 0;
				next_sync = 0;
				next_din = 0;
			end
			next_sclk = 1;
			next_bit_cnt = 5;
			next_data_b = 0;
		end

		STATE_SYNC_HOLD:
		begin
			next_state = STATE_START_TX;
		end

		STATE_START_TX:
		begin
			next_sclk = ~SCLK;
			if (~SCLK)
			begin
				next_state = STATE_COMMAND;
			end
		end

		STATE_COMMAND:
		begin
			next_sclk = ~SCLK;
			if (~SCLK)
			begin
				if (bit_cnt)
					next_bit_cnt = bit_cnt - 1'b1;
				else
				begin
					next_state = STATE_DATA;
					next_bit_cnt = 15;
				end

				if (DEBUG)
					next_din = CMD_BIT_EXTRACT;
				else
				begin
					if (data_b)
						next_din = CMD_B_BIT_EXTRACT;
					else
						next_din = CMD_A_BIT_EXTRACT;
				end
			end
		end

		STATE_DATA:
		begin
			next_sclk = ~SCLK;
			if (~SCLK)
			begin
				if (bit_cnt)
					next_bit_cnt = bit_cnt - 1'b1;
				else
				begin
					next_state = STATE_EOT;
					next_bit_cnt = 7;
				end

				if (data_b)
					next_din = DAT_B_BIT_EXTRACT;
				else
					next_din = DAT_A_BIT_EXTRACT;
			end
		end

		STATE_EOT:
		begin
			next_sclk = ~SCLK;
			if (~SCLK)
			begin
				next_sync = 1;
				next_state = STATE_WAIT;
			end
		end

		STATE_WAIT:
		begin
			next_din = 1;
			if (bit_cnt)
				next_bit_cnt = bit_cnt - 1'b1;
			else
			begin
				next_bit_cnt = 5;
				if (DEBUG)
				begin
					next_state = STATE_IDLE;
					next_ready = 1;
				end
				else
				begin
					next_data_b = ~data_b;
					if (data_b)
					begin
						next_state = STATE_IDLE;
						next_ready = 1;
					end
					else
					begin
						next_state = STATE_SYNC_HOLD;
						next_sync = 0;
					end
				end
			end
		end

	endcase

end

endmodule
