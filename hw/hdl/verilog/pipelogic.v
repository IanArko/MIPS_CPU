/*
 * Simple module which takes a string of bytes over an AXI stream, adds a
 * 32-bit value to each group of 4 bytes, and then sends them on an output
 * AXI stream.
 *
 * Ready/valid signals are used for flow control on the AXI stream.  An
 * AXI-Lite bus (via an IPIF wrapper) controls the value that gets added.
 *
 * Pieces of this file are based on the generated AXI interface templates
 * from XPS peripheral generator.
 *
 * Steven Bell <sebell@stanford.edu>
 * 31 January 2013
 */

`uselib lib=unisims_ver
`uselib lib=proc_common_v3_00_a

/*
 Registers for sobel design:
  Input stream
   - Buffer ID
   - Starting address
  Output stream
   - Buffer ID
   - Starting address
   - length (in words)
  Compute engine
   - address in input buffer
   - address in output buffer
   - length
   - command register (for start)
   - status register
*/

module pipelogic
#(
    parameter IPIF_NUM_REG = 7,
    parameter IPIF_DWIDTH = 32,
    parameter AXIS_DWIDTH = 32
)
(
    // AXI Stream interface
    input aclk,                                     // clock
    input aresetn,                                  // active low reset

    output wire axis_ready_in,                      // ready to accept data in
    input axis_valid_in,                            // data input is valid
    input [AXIS_DWIDTH-1:0] axis_data_in,           // data input
    input axis_last_in,                             // flag for last transfer in packet

    input axis_ready_out,                           // ready to receive data out
    output wire axis_valid_out,                     // data output is valid
    output wire [AXIS_DWIDTH-1:0] axis_data_out,    // data output
    output wire axis_last_out,                      // flag for last transfer in packet

    // IPIF (AXI Lite wrapper) interface
    input [IPIF_DWIDTH-1:0] ipif_data_in,           // IPIF data input
    input [IPIF_DWIDTH/8-1:0] ipif_be,              // IPIF byte enables
    input [IPIF_NUM_REG-1:0] ipif_read_ce,          // IPIF read chip enable
    input [IPIF_NUM_REG-1:0] ipif_write_ce,         // IPIF write chip enable
    output wire [IPIF_DWIDTH-1:0] ipif_data_out,    // IPIF data output
    output wire ipif_read_ack,                      // IPIF read transfer acknowledgement
    output wire ipif_write_ack,                     // IPIF write transfer acknowledgement
    output wire ipif_error                          // IPIF error response

);

localparam BUF_SIZE = 32768;
localparam ADDR_WIDTH = 15;

wire [IPIF_DWIDTH-1:0] write_buf_select, command, status, test;
wire [IPIF_DWIDTH-1:0] ul_status, ul_test;
wire [ADDR_WIDTH-1:0] start_addr_in, start_addr_out, write_addr, read_addr;
wire [ADDR_WIDTH:0] packet_size_out;
wire set_packet_size;
wire axis_write_en;

// Delay by one cycle since the packet size is also delayed by one cycle in the
// corresponding ipif register.
reg set_packet_size_d;
always @(posedge aclk)
    set_packet_size_d <= set_packet_size;

axis_addr #(ADDR_WIDTH) axis_addr (
    .aclk(aclk),
    .aresetn(aresetn),
    .ready_in(axis_ready_in),
    .valid_in(axis_valid_in),
    .last_in(axis_last_in),
    .ready_out(axis_ready_out),
    .valid_out(axis_valid_out),
    .last_out(axis_last_out),
    .read_addr(read_addr),
    .write_addr(write_addr),
    .write_en(axis_write_en),
    .start_addr_in(start_addr_in),
    .start_addr_out(start_addr_out),
    .packet_size_out(packet_size_out),
    .set_packet_size(set_packet_size_d)
);

wire [31:0] ul_read_addr_lo, ul_read_addr_hi, ul_write_addr_lo, ul_write_addr_hi;
wire [15:0] ul_read_data_lo, ul_read_data_hi, ul_write_data_lo, ul_write_data_hi;
wire [31:0] instr_addr;
wire [31:0] instr;
wire [1:0] ul_write_en_lo, ul_write_en_hi;

// Instruction Memory
wire instr_mem_we = (axis_write_en & write_buf_select[0]);
wire [13:0] instr_mem_addr = instr_mem_we ? write_addr : instr_addr;
dataram2 #(16384, 14, 32) instr_mem (
    .clk(aclk),
    .addr(instr_mem_addr),
    .we(instr_mem_we),
    .din(axis_data_in),
    .dout(instr)
);

// Input Buffer
wire input_buffer_we = axis_write_en & write_buf_select[1];
wire [ADDR_WIDTH-1:0] input_buffer_addr_lo = input_buffer_we ? write_addr : ul_read_addr_lo;
wire [ADDR_WIDTH-1:0] input_buffer_addr_hi = input_buffer_we ? write_addr : ul_read_addr_hi;

dataram2 #(BUF_SIZE, ADDR_WIDTH, AXIS_DWIDTH/2) buf_0_lo (
    .clk(aclk),
    .addr(input_buffer_addr_lo),
    .we(input_buffer_we),
    .din(axis_data_in[15:0]),
    .dout(ul_read_data_lo)
);

dataram2 #(BUF_SIZE, ADDR_WIDTH, AXIS_DWIDTH/2) buf_0_hi (
    .clk(aclk),
    .addr(input_buffer_addr_hi),
    .we(input_buffer_we),
    .din(axis_data_in[31:16]),
    .dout(ul_read_data_hi)
);

// Output Buffer
wire [1:0] output_buffer_we_lo = ul_write_en_lo;
wire [1:0] output_buffer_we_hi = ul_write_en_hi;
wire [ADDR_WIDTH-1:0] output_buffer_addr_lo = output_buffer_we_lo ? ul_write_addr_lo : read_addr;
wire [ADDR_WIDTH-1:0] output_buffer_addr_hi = output_buffer_we_hi ? ul_write_addr_hi : read_addr;

dataram3 #(.ADDR_WIDTH(ADDR_WIDTH), .COL_WIDTH(AXIS_DWIDTH/4), .N_COLS(AXIS_DWIDTH/16)) buf_1_lo (
    .clk(aclk),
    .addr(output_buffer_addr_lo),
    .we(output_buffer_we_lo),
    .din(ul_write_data_lo),
    .dout(axis_data_out[15:0])
);

dataram3 #(.ADDR_WIDTH(ADDR_WIDTH), .COL_WIDTH(AXIS_DWIDTH/4), .N_COLS(AXIS_DWIDTH/16)) buf_1_hi (
    .clk(aclk),
    .addr(output_buffer_addr_hi),
    .we(output_buffer_we_hi),
    .din(ul_write_data_hi),
    .dout(axis_data_out[31:16])
);

userlogic #(BUF_SIZE, ADDR_WIDTH, AXIS_DWIDTH) ul (
    .clk(aclk),
    .rst_n(aresetn),
    .instr(instr),
    .instr_addr(instr_addr),
    .read_data_lo(ul_read_data_lo),
    .read_data_hi(ul_read_data_hi),
    .read_addr_lo(ul_read_addr_lo),
    .read_addr_hi(ul_read_addr_hi),
    .write_data_lo(ul_write_data_lo),
    .write_data_hi(ul_write_data_hi),
    .write_addr_lo(ul_write_addr_lo),
    .write_addr_hi(ul_write_addr_hi),
    .write_en_lo(ul_write_en_lo),
    .write_en_hi(ul_write_en_hi),
    .command(command),
    .status(ul_status),
    .test(ul_test)
);




//-------------------------
// IPIF register read/write
//-------------------------

wire [IPIF_DWIDTH-1:0] slv_regs [IPIF_NUM_REG-1:0];

localparam IPIF_REG_SEL_WIDTH = 3; // should be ceil(log2(IPIF_NUM_REG))

// give the registers meaning
assign status           = slv_regs[0];  // offset 24
assign test             = slv_regs[1];  // offset 20
assign start_addr_in    = slv_regs[2];  // offset 16
assign start_addr_out   = slv_regs[3];  // offset 12
assign packet_size_out  = slv_regs[4];  // offset 8
assign write_buf_select = slv_regs[5];  // offset 4
assign command          = slv_regs[6];  // offset 0

// Note: if adding registers to this design, ensure that ipif ce index for
//       set_packet_size matches the slv_regs index for packet_size_out.
assign set_packet_size = ipif_write_ce[4];

wire [3:0] status_be = 32'hFFFF;
ipif_reg #(IPIF_DWIDTH) status_reg (
    .aclk(aclk),
    .aresetn(aresetn),
    .en(1'b1),
    .be(status_be),
    .d(ul_status),
    .q(slv_regs[0])
);

wire [3:0] test_be = 32'hFFFF;
ipif_reg #(IPIF_DWIDTH) test_reg (
    .aclk(aclk),
    .aresetn(aresetn),
    .en(1'b1),
    .be(test_be),
    .d(ul_test),
    .q(slv_regs[1])
);

generate
    genvar reg_num;
    for (reg_num = 2; reg_num < IPIF_NUM_REG; reg_num = reg_num + 1) begin : ipif_regs
        ipif_reg #(IPIF_DWIDTH) ipif_reg (
            .aclk(aclk),
            .aresetn(aresetn),
            .en(ipif_write_ce[reg_num]),
            .be(ipif_be),
            .d(ipif_data_in),
            .q(slv_regs[reg_num])
        );
    end
endgenerate

wire [IPIF_REG_SEL_WIDTH-1:0] reg_read_sel;
encoder #(IPIF_NUM_REG, IPIF_REG_SEL_WIDTH) reg_read_sel_enc (.onehot(ipif_read_ce), .binary(reg_read_sel));

assign ipif_data_out = slv_regs[reg_read_sel];
assign ipif_read_ack = |ipif_read_ce;
assign ipif_write_ack = |ipif_write_ce;
assign ipif_error = 1'b0;

endmodule
