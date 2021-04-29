module userlogic
#(
    parameter BUF_SIZE = 4096,
    parameter ADDR_WIDTH = 12,
    parameter AXIS_DWIDTH = 32
)
(
    input wire clk,
    input wire rst_n,

    // Interface to instruction memory (MIPS only)
    input  wire [31:0] instr,
    output wire [31:0] instr_addr,

    // Interface to data input buffer
    input  wire [15:0] read_data_lo,
    input  wire [15:0] read_data_hi,
    output wire [31:0] read_addr_lo,
    output wire [31:0] read_addr_hi,

    // Interface to data output buffer
    output wire [15:0] write_data_lo,
    output wire [15:0] write_data_hi,
    output wire [31:0] write_addr_lo,
    output wire [31:0] write_addr_hi,
    output wire [1:0] write_en_lo,
    output wire [1:0] write_en_hi,

    // External shared registers
    input  wire [31:0] command,
    output wire [31:0] status,
    output wire [31:0] test
);

    // 16 KiB local storage
    localparam LOCAL_ADDR_WIDTH = 12;
    localparam LOCAL_DATA_WIDTH = 32;

    wire [31:0] read_data = { read_data_hi, read_data_lo };
    wire [31:0] write_data;

    assign write_data_lo = write_data[15:0];
    assign write_data_hi = write_data[31:16];

    wire [31:0] mips_pc;
    wire [31:0] mips_addr;
    wire [3:0] mips_write_en;
    wire mips_read_en;

    wire [31:0] mips_read_data;
    wire [31:0] mips_local_read_data;

    mips_cpu cpu (
        .clk(clk),
        .rst(~command[1]),
        .en(1'b1),
        .pc(mips_pc),
        .instr(instr),
        .mem_write_en(mips_write_en),
        .mem_read_en(mips_read_en),
        .mem_addr(mips_addr),
        .mem_write_data(write_data),
        .mem_read_data(mips_read_data)
    );

    // 16384 words of instruction memory
    assign instr_addr = mips_pc[15:2];

    // Word aligned reads and writes
    assign read_addr_lo = mips_addr[ADDR_WIDTH+1:2];
    assign read_addr_hi = mips_addr[ADDR_WIDTH+1:2];
    assign write_addr_lo = mips_addr[ADDR_WIDTH+1:2];
    assign write_addr_hi = mips_addr[ADDR_WIDTH+1:2];

    assign status = status_reg;
    assign test   = test_reg;

    // Memory Mapped I/O
    reg [31:0] status_reg, test_reg;

    wire addr_status = (mips_addr == 32'h8002_0000);
    wire addr_test   = (mips_addr == 32'h8002_0004);
    wire addr_cmd    = (mips_addr == 32'h8002_0008);
    wire addr_local  = ~mips_addr[31];
    wire addr_iobuf  = ~|{addr_status, addr_test, addr_cmd, addr_local};

    always @(posedge clk) begin
        if (command[1]) begin
            status_reg <= {32{1'b0}};
            test_reg   <= {32{1'b0}};
        end
        else if (|{mips_write_en}) begin
            if (addr_status) begin
                status_reg <= write_data;
            end
            else if (addr_test) begin
                test_reg <= write_data;
            end
        end
    end

    assign write_en_hi = mips_write_en[3:2] & {2{addr_iobuf}};
    assign write_en_lo = mips_write_en[1:0] & {2{addr_iobuf}};

    reg read_cmd_d, read_iobuf_d;
    reg [31:0] cmd_d;

    always @(posedge clk) begin
        if (~rst_n) begin
            read_cmd_d   <= 1'b0;
            read_iobuf_d <= 1'b0;
        end
        else begin
            read_cmd_d   <= (mips_read_en & addr_cmd);
            read_iobuf_d <= (mips_read_en & addr_iobuf);
        end
    end

    always @(posedge clk) begin
        if (~rst_n) begin
            cmd_d <= {32{1'b0}};
        end
        else if (mips_read_en & addr_cmd) begin
            cmd_d <= command;
        end
    end

    assign mips_read_data = (read_cmd_d) ? cmd_d : ((read_iobuf_d) ? read_data : mips_local_read_data);

    dataram3 #(.ADDR_WIDTH(LOCAL_ADDR_WIDTH), .COL_WIDTH(8), .N_COLS(4)) mips_local_store (
        .clk(clk),
        .addr(mips_addr[LOCAL_ADDR_WIDTH+1:2]),
        .we((mips_write_en & {4{addr_local}})),
        .din(write_data),
        .dout(mips_local_read_data)
    );

endmodule
