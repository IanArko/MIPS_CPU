//=============================================================================
// EE108B Lab 2
//
// MIPS CPU Module. Contains the five stages in the single-cycle MIPS CPU.
//=============================================================================

module mips_cpu (
    input clk,
    input rst,
    input en,

    output wire [3:0] mem_write_en,
    output wire mem_read_en,
    output wire [31:0] mem_addr,
    output wire [31:0] mem_write_data,
    input [31:0] mem_read_data,
    output wire [31:0] pc,
    input [31:0] instr
);

    wire [31:0] pc_if, pc_id;
    wire [31:0] instr_sav;
    wire [31:0] instr_id;
    wire jump_branch_id, jump_target_id;
    wire [4:0] rs_addr_id, rt_addr_id;
    wire [31:0] rs_data_id, rt_data_id;
    wire [31:0] mem_write_data_id, mem_write_data_ex;

    wire [31:0] jr_pc_id;
    wire [31:0] branch_pc_id;
    wire  jr_target_id;

    wire [4:0] reg_write_addr_id, reg_write_addr_ex, reg_write_addr_mem, reg_write_addr_wb;
    wire [31:0] mem_out;
    wire [31:0] reg_write_data_mem, reg_write_data_wb;
    wire reg_we_id, reg_we_cond_ex, reg_we_ex, reg_we_mem, reg_we_wb;
    wire movn_id, movn_ex, movz_id, movz_ex;
    wire [3:0] alu_opcode_id, alu_opcode_ex;
    wire [31:0] alu_op_x_id, alu_op_y_id, alu_op_x_ex, alu_op_y_ex;
    wire [31:0] sc_result, alu_sc_result_ex, alu_result_ex, alu_result_mem;
    wire alu_op_y_zero_ex;
    wire mem_we_id, mem_we_ex;
    wire mem_read_id, mem_read_ex, mem_read_mem;
    wire mem_byte_id, mem_byte_ex, mem_byte_mem;
    wire mem_signextend_id, mem_signextend_ex, mem_signextend_mem;
    wire [7:0] mem_read_data_byte_select;
    wire [31:0] mem_read_data_byte_extend;
    wire mem_sc_mask_id;
    wire mem_sc_id, mem_sc_ex;
    wire alu_overflow;
    wire stall, stall_r;
    wire en_if = ~stall & en;
    wire rst_id = stall & en;
    
    /* Pipeline Registers */
    /* ----------------------------------------------------------- */
    /* Pipeline Reg if2id */
    dffare #(32)  pc_if2id (.clk(clk), .r(rst), .en(en_if), .d(pc_if), .q(pc_id));
    dffare #(32) instr_sav_dff (.clk(clk), .r(rst), .en(en), .d(instr), .q(instr_sav));
    dffare #(1) stall_f_dff (.clk(clk), .r(rst), .en(en), .d(stall), .q(stall_r));
    
    /* Pipeline Reg id2ex */
    dffarre #(32) alu_op_x_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(alu_op_x_id), .q(alu_op_x_ex));      
    dffarre #(32) alu_op_y_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(alu_op_y_id), .q(alu_op_y_ex));      
    dffarre #(4)  alu_opcode_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(alu_opcode_id), .q(alu_opcode_ex)); 
    dffarre       movn (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(movn_id), .q(movn_ex));
    dffarre       movz (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(movz_id), .q(movz_ex));
    dffarre #(32) mem_write_data_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(mem_write_data_id), .q(mem_write_data_ex));
    dffarre       mem_we_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(mem_we_id & ~mem_sc_mask_id), .q(mem_we_ex));
    dffarre       mem_read_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(mem_read_id), .q(mem_read_ex));
    dffarre       mem_byte_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(mem_byte_id), .q(mem_byte_ex));
    dffarre       mem_signextend_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(mem_signextend_id), .q(mem_signextend_ex));
    dffarre #(5)  reg_write_addr_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(reg_write_addr_id), .q(reg_write_addr_ex));
    dffarre       reg_we_id2ex (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(reg_we_id), .q(reg_we_cond_ex));
    dffarre       sc      (.clk(clk), .ar(rst), .r(rst_id), .en(en), .d(mem_sc_id), .q(mem_sc_ex));
    
    /* Pipeline Reg ex2mem */
    dffare #(5)   reg_write_addr_ex2mem (.clk(clk), .r(rst), .en(en), .d(reg_write_addr_ex), .q(reg_write_addr_mem));
    dffare        reg_we_ex2mem (.clk(clk), .r(rst), .en(en), .d(reg_we_ex), .q(reg_we_mem));
    dffare #(32)  alu_result_ex2mem (.clk(clk), .r(rst), .en(en), .d(alu_sc_result_ex), .q(alu_result_mem));
    dffare        mem_read_ex2mem (.clk(clk), .r(rst), .en(en), .d(mem_read_ex), .q(mem_read_mem)); 
    dffare        mem_byte_ex2mem (.clk(clk), .r(rst), .en(en), .d(mem_byte_ex), .q(mem_byte_mem));
    dffare        mem_signextend_ex2mem (.clk(clk), .r(rst), .en(en), .d(mem_signextend_ex), .q(mem_signextend_mem));
    
    /* Pipeline Reg mem2wb */
    dffare #(32)  reg_write_data_mem2wb (.clk(clk), .r(rst), .en(en), .d(reg_write_data_mem), .q(reg_write_data_wb));
    dffare #(5)   reg_write_addr_mem2wb (.clk(clk), .r(rst), .en(en), .d(reg_write_addr_mem), .q(reg_write_addr_wb));
    dffare        reg_we_mem2wb (.clk(clk), .r(rst), .en(en), .d(reg_we_mem), .q(reg_we_wb));
    
    /* Instruction Fetch */
    /* ----------------------------------------------------------- */
    instruction_fetch if_stage (
        .clk            (clk),
        .rst            (rst),
        .en             (en_if),
        .jump_target    (jump_target_id),
        .pc_id          (pc_id),
        .instr_id       (instr_id[25:0]),
        .jr_pc_id        (jr_pc_id),
        .jr_target_id    (jr_target_id),
        .jump_branch_id  (jump_branch_id),
        .branch_pc_id    (branch_pc_id),

        .pc             (pc_if)
    );

    // Output pc to parent module
    assign pc = pc_if;
    
    // Saved ID instruction after a stall
    assign instr_id = (stall_r) ? instr_sav : instr;

    /* Instruction Decode */
    /* ----------------------------------------------------------- */
    decode d_stage (
        // inputs to decode
        .pc                 (pc_id),
        .instr              (instr_id),
        .rs_data_in         (rs_data_id),
        .rt_data_in         (rt_data_id),
        
        // Outputs for asychronous register file reads
        .rs_addr            (rs_addr_id),
        .rt_addr            (rt_addr_id),
        
        // Jump logic
        .jump_target        (jump_target_id),
        .jr_target          (jr_target_id),
        .jr_pc              (jr_pc_id),
        
        // Branch logic
        .jump_branch        (jump_branch_id),
        .branch_pc          (branch_pc_id),
        
        // Outputs to ALU
        .alu_opcode         (alu_opcode_id),
        .alu_op_x           (alu_op_x_id),
        .alu_op_y           (alu_op_y_id),
        .reg_write_addr     (reg_write_addr_id),
        
        // Other outputs
        .mem_we             (mem_we_id),
        .mem_write_data     (mem_write_data_id),
        .mem_read           (mem_read_id),
        .mem_byte           (mem_byte_id),
        .mem_signextend     (mem_signextend_id),
        .reg_we             (reg_we_id),
        .movn               (movn_id),
        .movz               (movz_id),
        .mem_sc_mask_id     (mem_sc_mask_id),
        .mem_sc_id          (mem_sc_id),
        .stall              (stall),

        // inputs for forwarding/stalling from X
        .reg_we_ex          (reg_we_ex),
        .reg_write_addr_ex  (reg_write_addr_ex),
        .alu_result_ex      (alu_sc_result_ex),
        .mem_read_ex        (mem_read_ex),

        // inputs for forwarding/stalling from M
        .reg_we_mem         (reg_we_mem),
        .reg_write_addr_mem (reg_write_addr_mem),
        .reg_write_data_mem (reg_write_data_mem)
    );

    /* Instruction Execute */
    /* ----------------------------------------------------------- */
    alu x_stage (
        .alu_opcode     (alu_opcode_ex),
        .alu_op_x       (alu_op_x_ex),
        .alu_op_y       (alu_op_y_ex),

        .alu_result     (alu_result_ex),
        .alu_op_y_zero  (alu_op_y_zero_ex),
        .alu_overflow   (alu_overflow) // maybe do something creative with this
    );

    /* Memory Operations */
    /* ----------------------------------------------------------- */
    assign sc_result = {{31{1'b0}},(mem_sc_ex & mem_we_ex)};
    assign alu_sc_result_ex = mem_sc_ex ? sc_result : alu_result_ex;   // If SC, write 1 or 0. Else, write alu_result_ex.

    assign reg_we_ex = reg_we_cond_ex & |{movz_ex & alu_op_y_zero_ex, movn_ex & ~alu_op_y_zero_ex, ~movz_ex & ~movn_ex};
    assign mem_read_en = mem_read_ex;
    assign mem_write_en[3] = mem_we_ex & (~mem_byte_ex | (mem_addr[1:0] == 2'b00));
    assign mem_write_en[2] = mem_we_ex & (~mem_byte_ex | (mem_addr[1:0] == 2'b01));
    assign mem_write_en[1] = mem_we_ex & (~mem_byte_ex | (mem_addr[1:0] == 2'b10));
    assign mem_write_en[0] = mem_we_ex & (~mem_byte_ex | (mem_addr[1:0] == 2'b11));
    assign mem_addr = alu_result_ex;
    assign mem_write_data = (mem_byte_ex) ? {4{mem_write_data_ex[7:0]}} : mem_write_data_ex;
    assign mem_read_data_byte_select = (alu_result_mem[1:0] == 2'b00) ? mem_read_data[31:24] :
                                       ((alu_result_mem[1:0] == 2'b01) ? mem_read_data[23:16] :
                                       ((alu_result_mem[1:0] == 2'b10) ? mem_read_data[15:8] : mem_read_data[7:0]));
    assign mem_read_data_byte_extend = {{24{mem_signextend_mem & mem_read_data_byte_select[7]}},mem_read_data_byte_select};
    assign mem_out = (mem_byte_mem) ? mem_read_data_byte_extend : mem_read_data;
    assign reg_write_data_mem = mem_read_mem ? mem_out : alu_result_mem;

    /* Write-Back (Also used for asynchronous register reads) */
    /* ----------------------------------------------------------- */
    regfile w_stage (
        .clk            (clk),
        .en             (en),
        .reg_write_data (reg_write_data_wb),
        .reg_write_addr (reg_write_addr_wb),
        .reg_we         (reg_we_wb),
        .rs_addr        (rs_addr_id),
        .rt_addr        (rt_addr_id),

        .rs_data        (rs_data_id),
        .rt_data        (rt_data_id)
    );

endmodule
