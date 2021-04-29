module ipif_reg
#(
    parameter WIDTH = 32
)
(
    input aclk,
    input aresetn,
    input en,
    input [WIDTH/8-1:0] be,
    input [WIDTH-1:0] d,
    output reg [WIDTH-1:0] q
);

integer byte_index;
always @(posedge aclk)
    if (~aresetn)
        q <= {WIDTH{1'b0}};
    else if (en) 
        for (byte_index = 0; byte_index < (WIDTH/8); byte_index = byte_index + 1)
            if (be[byte_index])
                q[(byte_index*8) +: 8] <= d[(byte_index*8) +: 8];

endmodule
