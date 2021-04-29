module dataram
#(
    parameter SIZE = 1024,
    parameter AWIDTH = 10,
    parameter DWIDTH = 32
)
(
    input clk,
    input re,
    input [AWIDTH-1:0] raddr,
    input we,
    input [AWIDTH-1:0] waddr,
    input [DWIDTH-1:0] din,
    output reg [DWIDTH-1:0] dout
);

reg [DWIDTH-1:0] mem [SIZE-1:0];

always @(negedge clk)
    if (we)
        mem[waddr] <= din;

always @(posedge clk)
    if (re)
        dout <= mem[raddr];

endmodule
