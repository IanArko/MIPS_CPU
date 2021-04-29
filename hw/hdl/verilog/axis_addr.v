module axis_addr
#(
    parameter ADDR_WIDTH = 10
)
(
    input aclk,
    input aresetn,

    output wire ready_in,
    input valid_in,
    input last_in,

    input ready_out,
    output wire valid_out,
    output wire last_out,

    output wire [ADDR_WIDTH-1:0] read_addr,
    output wire [ADDR_WIDTH-1:0] write_addr,
    output wire write_en,

    input [ADDR_WIDTH-1:0] start_addr_in,
    input [ADDR_WIDTH-1:0] start_addr_out,

    input [ADDR_WIDTH:0] packet_size_out,
    input set_packet_size
);

reg [ADDR_WIDTH:0] input_count, output_count, output_remaining;

assign ready_in = valid_in;

assign valid_out = ready_out & |output_remaining;

assign last_out = output_remaining == {{(ADDR_WIDTH){1'b0}}, 1'd1};

assign read_addr = start_addr_out + output_count + valid_out;
assign write_addr = start_addr_in + input_count;
assign write_en = valid_in;

always @(posedge aclk) begin
    if (~aresetn | last_in)
        input_count <= {(ADDR_WIDTH+1){1'b0}};
    else if (valid_in)
        input_count <= input_count + 1'd1;
end

reg transfer_on;
always @(posedge aclk) begin
    if (~aresetn | (last_out & valid_out))
        transfer_on <= 1'b0;
    else if (set_packet_size)
        transfer_on <= 1'b1;
end

always @(posedge aclk) begin
    if (~aresetn | last_out)
        output_count <= {(ADDR_WIDTH+1){1'b0}};
    else if (set_packet_size)
        output_count <= {(ADDR_WIDTH+1){1'b0}};
    else if (ready_out & transfer_on)
        output_count <= output_count + 1'd1;
end

always @(posedge aclk) begin
    if (~aresetn | last_out)
        output_remaining <= {(ADDR_WIDTH+1){1'b0}};
    else if (set_packet_size)
        output_remaining <= packet_size_out;
    else if (ready_out & transfer_on)
        output_remaining <= output_remaining - 1'd1;
end

endmodule
