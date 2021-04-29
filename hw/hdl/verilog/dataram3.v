`timescale 1ns / 1ps
module dataram3(clk, addr, we, din, dout);
    parameter  ADDR_WIDTH = 12;
    parameter  COL_WIDTH  = 8;
    parameter  N_COLS     = 2;
    localparam DATA_WIDTH = (COL_WIDTH*N_COLS);
    localparam RAM_DEPTH  = 1 << ADDR_WIDTH;

    input  clk;
    input  [(ADDR_WIDTH-1):0] addr;
    input  [(N_COLS-1):0] we;
    input  [(DATA_WIDTH-1):0] din;
    output reg [(DATA_WIDTH-1):0] dout;

    reg [(DATA_WIDTH-1):0] ram [0:(RAM_DEPTH-1)];

/*
    integer i;
    initial begin
        for (i = 0; i < RAM_DEPTH; i = i + 1) begin
            ram[i] <= {DATA_WIDTH{1'b0}};
        end
    end
*/

    generate
    genvar c;
        for (c = 0; c < N_COLS; c = c + 1) begin: column
            always @(posedge clk) begin
                if (we[c]) begin
                    ram[addr][(c+1)*COL_WIDTH-1:c*COL_WIDTH] <= din[(c+1)*COL_WIDTH-1:c*COL_WIDTH];
                    dout[(c+1)*COL_WIDTH-1:c*COL_WIDTH] <= din[(c+1)*COL_WIDTH-1:c*COL_WIDTH];
                end
                else begin
                    dout[(c+1)*COL_WIDTH-1:c*COL_WIDTH] <= ram[addr][(c+1)*COL_WIDTH-1:c*COL_WIDTH];
                end
            end
        end
    endgenerate

endmodule

