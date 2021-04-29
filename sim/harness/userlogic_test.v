`timescale 1 ns / 10 ps

module userlogic_test();

    localparam AXIS_DWIDTH = 32;
    localparam BUF_SIZE = 32768;
    localparam ADDR_WIDTH = 15;

    reg clk;
    reg rst_n;

    // User logic command, status, and test registers.
    reg  [31:0] ul_command = 2;
    wire [31:0] ul_status;
    wire [31:0] ul_test;

    // Testbench parameters.
    integer read_instr_mem;
    integer read_input_buffer;
    integer write_output_buffer;
    integer write_test_result;
    integer dump_vars;

    reg  [1024*8:1] instr_mem_filename;
    reg  [1024*8:1] input_buffer_filename;
    reg  [1024*8:1] output_buffer_filename;
    reg  [1024*8:1] test_result_filename;
    reg  [1024*8:1] dump_vars_filename;
    reg  [9:0] image_n_rows;
    reg  [9:0] image_n_columns;

    reg  [32:1] num_cycles = 32'hFFFFFFFF;
    reg  [32:1] cycle_count = 0;

    // Used to dump output buffer to file.
    integer i, outfile;
    reg  [32:0] out_start_addr = 0;
    reg  [32:0] out_end_addr = (BUF_SIZE * 4);
    reg  [ADDR_WIDTH-1:0] read_addr = 0;
    wire [AXIS_DWIDTH-1:0] data_out;

    // Initialize testbench parameters.
    integer result;
    initial begin

        read_instr_mem = $value$plusargs("instr_mem=%s", instr_mem_filename);
        read_input_buffer = $value$plusargs("in_buf=%s", input_buffer_filename);
        write_output_buffer = $value$plusargs("out_buf=%s", output_buffer_filename);
        write_test_result = $value$plusargs("test_result=%s", test_result_filename);
        dump_vars = $value$plusargs("dumpvars=%s", dump_vars_filename);

        // Sobel Filter: Extract the image width and height from the plus arguments.
        if ($value$plusargs("image_rows=%d", image_n_rows)) begin
            $display("Image rows: %d", image_n_rows);
        end
        if ($value$plusargs("image_columns=%d", image_n_columns)) begin
            $display("Image columns: %d", image_n_columns);
        end

        // Fill instruction memory and input buffer.
        if (read_instr_mem) begin
            $display("Instruction Memory: %0s", instr_mem_filename);
            $readmemh(instr_mem_filename, instr_mem.mem);
        end else begin
            $display("No instruction memory");
        end

        if (read_input_buffer) begin
            $display("Input Data Buffer: %0s", input_buffer_filename);
            $readmemh(input_buffer_filename, input_buffer.mem);
        end else begin
            $display("No input data buffer");
        end

        if (write_output_buffer) begin

            if ($test$plusargs("out_start")) begin
                result = $value$plusargs("out_start=%h", out_start_addr);
            end

            if ($test$plusargs("out_end")) begin
                result = $value$plusargs("out_end=%h", out_end_addr);
            end

            $display("Output buffer: %0s", output_buffer_filename);
            $display("  Dumping range: 0x%h - 0x%h", out_start_addr, out_end_addr);

            // Address are word-aligned in the buffer memories
            out_start_addr = out_start_addr >> 2;
            out_end_addr = out_end_addr >> 2;

        end else begin
            $display("No output buffer");
        end

        if ($test$plusargs("cycles")) begin
            result = $value$plusargs("cycles=%d", num_cycles);
        end

        $display("Running userlogic for maximum of %0d cycles", num_cycles);

    end

    initial begin

        // Initialize testbench signals
        clk = 1'b0;
        rst_n = 1'b0;
        ul_command = {{10{1'b0}}, image_n_columns, image_n_rows, 1'b1, 1'b0};

        // Create waveform dump
        if (dump_vars) begin
            $dumpfile(dump_vars_filename);
            $dumpvars(0, userlogic_test);
        end

        // Turn off reset after a few cycles
        #20;
        rst_n = 1'b1;
        ul_command[1] = 1'b0;

        // Run
        #10 ul_command[0] = 1'b1;

        cycle_count = num_cycles;
        while (cycle_count > 0 & ~ul_status) begin
            cycle_count = cycle_count - 1;
            #10;
        end

        $display("Userlogic ran for %0d cycles", num_cycles - cycle_count);
        $display("status register = %0d", ul_status);
        $display("test register = %0d", ul_test);

        ul_command[0] = 1'b0;

        // Dump the output buffer.
        if (write_output_buffer) begin
            outfile = $fopen(output_buffer_filename, "w");
            for (i = out_start_addr; i < out_end_addr; i = i + 1) begin
                read_addr = i;
                #10;
                $fwrite(outfile, "%h\n", data_out);
            end
            $fclose(outfile);
        end

        // Write the test result.
        if (write_test_result) begin
            i = $fopen(test_result_filename, "w");
            $fwrite(i, "%0d", ul_test);
            $fclose(i);
        end

        $finish;
    end

    initial forever begin
        #5 clk = ~clk;
    end

    /* Signals used to connect the instruction memory and buffers to
     * userlogic. */
    wire [31:0] ul_read_addr, ul_write_addr;
    wire [31:0] ul_read_data, ul_write_data;
    wire [15:0] ul_write_data_lo, ul_write_data_hi;
    wire [31:0] ul_instr_addr;
    wire [31:0] ul_instr;
    wire [1:0]  ul_write_en_lo, ul_write_en_hi;

    // Instruction Memory - 16,384 words = 64KiB
    dataram2 #(16384, 14, 32) instr_mem (
        .clk(clk),
        .addr(ul_instr_addr[13:0]),
        .we(1'b0),
        .din({32{1'b0}}),
        .dout(ul_instr)
    );

    // Input Buffer - read only by user logic
    wire [ADDR_WIDTH-1:0] input_buffer_addr = ul_read_addr;
    dataram2 #(BUF_SIZE, ADDR_WIDTH, AXIS_DWIDTH) input_buffer (
        .clk(clk),
        .addr(input_buffer_addr),
        .we(1'b0),
        .din(32'b0),
        .dout(ul_read_data)
    );

    // Output Buffer - write only by user logic
    wire [ADDR_WIDTH-1:0] output_buffer_addr = (|{ul_write_en_lo, ul_write_en_hi}) ? ul_write_addr : read_addr;
    assign ul_write_data = { ul_write_data_hi, ul_write_data_lo };
    dataram3 #(.ADDR_WIDTH(ADDR_WIDTH), .COL_WIDTH(AXIS_DWIDTH/4), .N_COLS(AXIS_DWIDTH/8)) output_buffer (
        .clk(clk),
        .addr(output_buffer_addr),
        .we({ul_write_en_hi, ul_write_en_lo}),
        .din(ul_write_data),
        .dout(data_out)
    );

    // User Logic
    userlogic #(BUF_SIZE, ADDR_WIDTH, AXIS_DWIDTH) ul (
        .clk(clk),
        .rst_n(rst_n),
        .instr(ul_instr),
        .instr_addr(ul_instr_addr),
        .read_data_lo(ul_read_data[15:0]),
        .read_data_hi(ul_read_data[31:16]),
        .read_addr_lo(ul_read_addr),
        .write_data_lo(ul_write_data_lo),
        .write_data_hi(ul_write_data_hi),
        .write_addr_lo(ul_write_addr),
        .write_en_lo(ul_write_en_lo),
        .write_en_hi(ul_write_en_hi),
        .command(ul_command),
        .status(ul_status),
        .test(ul_test)
    );

endmodule
