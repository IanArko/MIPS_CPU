//=============================================================================
// EE108B Lab 3
//
// Instruction fetch module. Maintains PC and updates it. Reads from the
// instruction ROM. Determines if the PC counter should change for a jump.
//=============================================================================

module instruction_fetch (
    input clk,
    input rst,
    input en,
    
    input [31:0] pc_id,
    input [25:0] instr_id,
    input jump_target,              // is j?
    input jr_target_id,             // is jr?
    input [31:0] jr_pc_id,          // what PC to jump to for a jr   
    input jump_branch_id,           // is branch?
    input [31:0] branch_pc_id,      // what PC to jump to for a branch

    output [31:0] pc
);
    
    wire [31:0] pc_id_p4 = pc_id + 3'h4;        
    wire [31:0] jump_pc_id = {pc_id_p4[31:28], instr_id[25:0], 2'b0}; // what PC we should jump to for a j
    
    wire [31:0] j_addr = jr_target_id ? jr_pc_id : (jump_branch_id ? branch_pc_id : jump_pc_id);

    wire [31:0] pc_next = (jump_target | jr_target_id | jump_branch_id) ? j_addr : (pc + 3'h4);
    dffare #(32) pc_reg (.clk(clk), .r(rst), .en(en), .d(pc_next), .q(pc));

endmodule
