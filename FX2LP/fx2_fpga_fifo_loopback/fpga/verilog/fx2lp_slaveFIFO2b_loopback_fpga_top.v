module fx2lp_fifo_loopback_icesugarpro_top(
	// Reset
	rst,
	// LEDs
	led_g,
	led_r,
	led_b,
	// FX2LP IFCLK
	fx2_IFCLK,
	// FX2LP Data Bus [15:0]
	fx2_PB0, fx2_PB1, fx2_PB2, fx2_PB3, fx2_PB4, fx2_PB5, fx2_PB6, fx2_PB7,  // FD[7:0]
	fx2_PD0, fx2_PD1, fx2_PD2, fx2_PD3, fx2_PD4, fx2_PD5, fx2_PD6, fx2_PD7,  // FD[15:8]
	// FX2LP Flags
	fx2_CTL0_FLAGA,   // EP2 empty flag
	fx2_PA7_FLAGD,    // EP6 full flag
	// FX2LP FIFO Address
	fx2_PA4_FIFOADR0,
	fx2_PA5_FIFOADR1,
	// FX2LP Slave FIFO Control
	fx2_PA2_SLOE,
	fx2_RDY0_SLRD,
	fx2_RDY1_SLWR,
	fx2_PA6_PKTEND
);

// Reset (directly active-low from button, active-high in logic with ~rst)
input rst;

// LEDs
output led_g;
output led_r;
output led_b;

// FX2LP IFCLK (48 MHz from FX2LP)
input fx2_IFCLK;

// FX2LP Data Bus - individual pins for LPF mapping
inout fx2_PB0, fx2_PB1, fx2_PB2, fx2_PB3, fx2_PB4, fx2_PB5, fx2_PB6, fx2_PB7;
inout fx2_PD0, fx2_PD1, fx2_PD2, fx2_PD3, fx2_PD4, fx2_PD5, fx2_PD6, fx2_PD7;

// FX2LP Flags
input fx2_CTL0_FLAGA;   // EP2 empty (when 0, EP2 has data to read)
input fx2_PA7_FLAGD;    // EP6 full  (when 0, EP6 cannot accept more data)

// FX2LP FIFO Address
output fx2_PA4_FIFOADR0;
output fx2_PA5_FIFOADR1;

// FX2LP Slave FIFO Control (directly active-low outputs)
output fx2_PA2_SLOE;
output fx2_RDY0_SLRD;
output fx2_RDY1_SLWR;
output fx2_PA6_PKTEND;

// =-=-=-=-= Internal wires & regs =-=-=-=-=

// Combine individual data pins into a 16-bit bus
wire [15:0] fdata_in;
assign fdata_in = {fx2_PD7, fx2_PD6, fx2_PD5, fx2_PD4, fx2_PD3, fx2_PD2, fx2_PD1, fx2_PD0,
                   fx2_PB7, fx2_PB6, fx2_PB5, fx2_PB4, fx2_PB3, fx2_PB2, fx2_PB1, fx2_PB0};

reg [15:0] data_out;
wire data_out_en;

// Directly assign individual data output pins
assign fx2_PB0 = data_out_en ? data_out[0]  : 1'bz;
assign fx2_PB1 = data_out_en ? data_out[1]  : 1'bz;
assign fx2_PB2 = data_out_en ? data_out[2]  : 1'bz;
assign fx2_PB3 = data_out_en ? data_out[3]  : 1'bz;
assign fx2_PB4 = data_out_en ? data_out[4]  : 1'bz;
assign fx2_PB5 = data_out_en ? data_out[5]  : 1'bz;
assign fx2_PB6 = data_out_en ? data_out[6]  : 1'bz;
assign fx2_PB7 = data_out_en ? data_out[7]  : 1'bz;
assign fx2_PD0 = data_out_en ? data_out[8]  : 1'bz;
assign fx2_PD1 = data_out_en ? data_out[9]  : 1'bz;
assign fx2_PD2 = data_out_en ? data_out[10] : 1'bz;
assign fx2_PD3 = data_out_en ? data_out[11] : 1'bz;
assign fx2_PD4 = data_out_en ? data_out[12] : 1'bz;
assign fx2_PD5 = data_out_en ? data_out[13] : 1'bz;
assign fx2_PD6 = data_out_en ? data_out[14] : 1'bz;
assign fx2_PD7 = data_out_en ? data_out[15] : 1'bz;

// Internal flags (directly active-high for state machine logic)
wire flaga = fx2_CTL0_FLAGA;  // EP2 not empty (1 = data available)
wire flagd = fx2_PA7_FLAGD;   // EP6 not full  (1 = can write)

reg slrd_n;
reg slwr_n;
reg sloe_n;

reg [15:0] fifo_data_in;
wire [15:0] fifo_data_out;

reg [1:0] faddr_n;

// State machine states
parameter [1:0] loop_back_idle       = 2'd0;
parameter [1:0] loop_back_read       = 2'd1;
parameter [1:0] loop_back_wait_flagd = 2'd2;
parameter [1:0] loop_back_write      = 2'd3;

reg [1:0] current_loop_back_state;
reg [1:0] next_loop_back_state;

wire fifo_full;
wire fifo_empty;
wire fifo_push;
wire fifo_pop;

// Heartbeat LED counter (1Hz from 48MHz clock)
// 48MHz / 2 = 24,000,000 counts for 0.5 Hz
reg [24:0] heartbeat_cnt;
reg heartbeat_led;

// =-=-=-=-= Output assignments (directly active-low signals to FX2LP) =-=-=-=-=
assign fx2_RDY1_SLWR = slwr_n;
assign fx2_RDY0_SLRD = slrd_n;
assign fx2_PA2_SLOE  = sloe_n;
assign fx2_PA4_FIFOADR0 = faddr_n[0];
assign fx2_PA5_FIFOADR1 = faddr_n[1];
assign fx2_PA6_PKTEND = 1'b1;  // PKTEND not used (active-low, keep high)

// Data output enable: only drive bus during write
assign data_out_en = ~slwr_n;

// =-=-=-=-= LED indicators =-=-=-=-=
assign led_g = ~fifo_empty;  // Green: FIFO has data
assign led_r = fifo_full;    // Red:   FIFO is full
assign led_b = heartbeat_led;  // Blue: 1Hz heartbeat

// =-=-=-=-= Heartbeat LED =-=-=-=-=
always @(posedge fx2_IFCLK or negedge rst) begin
	if (rst == 1'b0) begin
		heartbeat_cnt <= 25'd0;
		heartbeat_led <= 1'b0;
	end else begin
		if (heartbeat_cnt >= 25'd23999999) begin  // 48MHz / 24M = 2Hz toggle = 1Hz blink
			heartbeat_cnt <= 25'd0;
			heartbeat_led <= ~heartbeat_led;
		end else begin
			heartbeat_cnt <= heartbeat_cnt + 1'b1;
		end
	end
end

// =-=-=-=-= FIFO address control =-=-=-=-=
// 00 = EP2 (read from host), 10 = EP6 (write to host)
always @(*) begin
	if ((current_loop_back_state == loop_back_idle) || (current_loop_back_state == loop_back_read)) begin
		faddr_n = 2'b00;  // Select EP2 for read
	end else begin
		faddr_n = 2'b10;  // Select EP6 for write
	end
end

// =-=-=-=-= Read control signal generation =-=-=-=-=
always @(*) begin
	if ((current_loop_back_state == loop_back_read) && (flaga == 1'b1)) begin
		slrd_n = 1'b0;
		sloe_n = 1'b0;
	end else begin
		slrd_n = 1'b1;
		sloe_n = 1'b1;
	end
end

assign fifo_push = ((slrd_n == 1'b0) && (fifo_full == 1'b0));

always @(*) begin
	if (slrd_n == 1'b0)
		fifo_data_in = fdata_in;
	else
		fifo_data_in = 16'h0000;
end

// =-=-=-=-= Write control signal generation =-=-=-=-=
always @(*) begin
	if ((current_loop_back_state == loop_back_write) && (flagd == 1'b1) && (fifo_empty == 1'b0))
		slwr_n = 1'b0;
	else
		slwr_n = 1'b1;
end

assign fifo_pop = ((slwr_n == 1'b0) && (fifo_empty == 1'b0));

// =-=-=-=-= Loopback mode state machine (sequential) =-=-=-=-=
always @(posedge fx2_IFCLK or negedge rst) begin
	if (rst == 1'b0)
		current_loop_back_state <= loop_back_idle;
	else
		current_loop_back_state <= next_loop_back_state;
end

// =-=-=-=-= Loopback mode state machine (combinational) =-=-=-=-=
always @(*) begin
	next_loop_back_state = current_loop_back_state;
	case (current_loop_back_state)
		loop_back_idle: begin
			if (flaga == 1'b1)
				next_loop_back_state = loop_back_read;
			else
				next_loop_back_state = loop_back_idle;
		end
		loop_back_read: begin
			if (flaga == 1'b0)
				next_loop_back_state = loop_back_wait_flagd;
			else
				next_loop_back_state = loop_back_read;
		end
		loop_back_wait_flagd: begin
			if (flagd == 1'b1)
				next_loop_back_state = loop_back_write;
			else
				next_loop_back_state = loop_back_wait_flagd;
		end
		loop_back_write: begin
			if ((flagd == 1'b0) || (fifo_empty == 1'b1))
				next_loop_back_state = loop_back_idle;
			else
				next_loop_back_state = loop_back_write;
		end
		default:
			next_loop_back_state = loop_back_idle;
	endcase
end

// =-=-=-=-= Data output mux =-=-=-=-=
always @(*) begin
	if (slwr_n == 1'b1)
		data_out = 16'h0000;
	else
		data_out = fifo_data_out;
end

// =-=-=-=-= FIFO instantiation for loopback =-=-=-=-=
fifo_512x8 fifo_inst (
	.din        (fifo_data_in),
	.write_busy (fifo_push),
	.fifo_full  (fifo_full),
	.dout       (fifo_data_out),
	.read_busy  (fifo_pop),
	.fifo_empty (fifo_empty),
	.fifo_clk   (fx2_IFCLK),
	.reset_     (rst),
	.fifo_flush (current_loop_back_state == loop_back_idle)
);

endmodule
