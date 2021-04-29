module encoder
#(
    parameter OH_WIDTH = 8,
    parameter B_WIDTH = 3
)
(
    input [OH_WIDTH-1:0] onehot,
    output reg [B_WIDTH-1:0] binary
);

integer i;
always @(onehot) begin
    for (i = 0; i < OH_WIDTH; i = i + 1) begin
        if (onehot[i])
            binary = i;
    end
    if (onehot == 0)
        binary = 0;
end

endmodule
