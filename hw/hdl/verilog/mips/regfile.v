//=============================================================================
// EE108B Lab 2
//
// MIPS Register File Module
//=============================================================================

module regfile (
    input clk,
    input en,
    input [31:0] reg_write_data,
    input [4:0] reg_write_addr,
    input reg_we,
    input [4:0] rs_addr,
    input [4:0] rt_addr,

    output wire [31:0] rs_data,
    output wire [31:0] rt_data
);

    reg [31:0] regs [31:0];

    // internally forwarded writes
    // if we write and read the same register in the same cycle, we get
    // the latest data
    assign rs_data = rs_addr == 5'd0 ?
                        32'd0
                     : ((reg_we & (rs_addr == reg_write_addr)) ?
                        reg_write_data
                     :
                        regs[rs_addr]);

    assign rt_data = rt_addr == 5'd0 ?
                        32'd0
                     : ((reg_we & (rt_addr == reg_write_addr)) ?
                         reg_write_data
                     : regs[rt_addr]);

    always @(posedge clk)
        if (reg_we & en)
            regs[reg_write_addr] <= reg_write_data;

endmodule
