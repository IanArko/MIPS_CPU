//=============================================================================
// EE108B Lab 2
//
// Decode module. Determines what to do with an instruction.
//=============================================================================

`include "mips_defines.v"

module decode (
    // inputs to decode
    input [31:0] pc,      
    input [31:0] instr,
    input [31:0] rs_data_in,
    input [31:0] rt_data_in,
    
    // Outputs for asychronous register file reads
    output wire [4:0] rs_addr,
    output wire [4:0] rt_addr,

    // Jump logic
    output wire jump_target,
    output wire jr_target,
    output wire [31:0] jr_pc,

    // Branch logic 
    output wire jump_branch,
    output wire [31:0] branch_pc,

    // Outputs to ALU 
    output reg [3:0] alu_opcode,
    output wire [31:0] alu_op_x,
    output wire [31:0] alu_op_y,
    output wire [4:0] reg_write_addr,
    
    // Other outputs
    output wire mem_we,
    output wire [31:0] mem_write_data,
    output wire mem_read,
    output wire mem_byte,
    output wire mem_signextend,
    output wire reg_we,
    output wire movn,
    output wire movz,
    output wire mem_sc_mask_id,
    output wire mem_sc_id,
    output wire stall,
    
     // inputs for forwarding/stalling from X
    input reg_we_ex,
    input [4:0] reg_write_addr_ex,
    input [31:0] alu_result_ex,
    input mem_read_ex,

    // inputs for forwarding/stalling from M
    input reg_we_mem,
    input [4:0] reg_write_addr_mem,
    input [31:0] reg_write_data_mem
);

//******************************************************************************
// instruction field
//******************************************************************************
    wire [5:0] op = instr[31:26];
    assign rs_addr = instr[25:21];
    assign rt_addr = instr[20:16];
    wire [4:0] rd_addr = instr[15:11];
    wire [4:0] shamt = instr[10:6];
    wire [5:0] funct = instr[5:0];
    wire [15:0] immediate = instr[15:0];

    wire [31:0] rs_data, rt_data;

//******************************************************************************
// branch instructions decode
//******************************************************************************

    wire isBEQ    = (op == `BEQ);
    wire isBGEZNL = (op == `BLTZ_GEZ) & (rt_addr == `BGEZ);
    wire isBGEZAL = (op == `BLTZ_GEZ) & (rt_addr == `BGEZAL);
    wire isBGTZ   = (op == `BGTZ) & (rt_addr == 5'b00000);
    wire isBLEZ   = (op == `BLEZ) & (rt_addr == 5'b00000);
    wire isBLTZNL = (op == `BLTZ_GEZ) & (rt_addr == `BLTZ);
    wire isBLTZAL = (op == `BLTZ_GEZ) & (rt_addr == `BLTZAL);
    wire isBNE    = (op == `BNE);
    wire isBranchLink = (isBGEZAL | isBLTZAL);


//******************************************************************************
// jump instructions decode
//******************************************************************************

    wire isJ    = (op == `J);
    wire isJAL  = (op == `JAL);
    wire isJALR  = ((op == `SPECIAL) & (funct == `JALR));
    wire isJR   = ((op == `SPECIAL) & (funct == `JR));
    wire link = isJAL | isJALR;

//******************************************************************************
// Branch/Jump resolution
//******************************************************************************
    wire isEqual  = rs_data == rt_data;
    wire isGEZ    = $signed(rs_data) >= 32'd0;
    wire isGZ     = $signed(rs_data) >  32'd0;
    wire isLZ     = rs_data[31];
    wire isLEZ    = isLZ || (rs_data == 32'd0);

    assign jump_branch = |{isBEQ    & isEqual,
                          isBNE     & ~isEqual,
                          isBGEZNL  & isGEZ,
                          isBGTZ    & isGZ,
                          isBLEZ    & isLEZ,
                          isBLTZNL  & isLZ};

    assign jr_pc = rs_data;
    assign jump_target = isJ || isJAL;
    assign jr_target = isJR || isJALR;

    assign branch_pc = pc + 32'd4 + (imm << 2);

//******************************************************************************
// shift instruction decode
//******************************************************************************

    wire isSLL = (op == `SPECIAL) & (funct == `SLL);
    wire isSRL = (op == `SPECIAL) & (funct == `SRL);
    wire isSLLV = (op == `SPECIAL) & (funct == `SLLV);
    wire isSRLV = (op == `SPECIAL) & (funct == `SRLV);
    wire isSRA = (op == `SPECIAL) & (funct == `SRA);
    wire isSRAV = (op == `SPECIAL) & (funct == `SRA);

    wire isShiftImm = isSLL | isSRL | isSRA;
    wire isShift = isShiftImm | isSLLV | isSRLV | isSRAV;

//******************************************************************************
// ALU instructions decode / control signal for ALU datapath
//******************************************************************************

    always @* begin
        casex({op, funct})
            {`ADDI, `DC6}:      alu_opcode = `ALU_ADD;
            {`ADDIU, `DC6}:     alu_opcode = `ALU_ADDU;
            {`SLTI, `DC6}:      alu_opcode = `ALU_SLT;
            {`SLTIU, `DC6}:     alu_opcode = `ALU_SLTU;
            {`ANDI, `DC6}:      alu_opcode = `ALU_AND;
            {`ORI, `DC6}:       alu_opcode = `ALU_OR;
            {`LB, `DC6}:        alu_opcode = `ALU_ADD;
            {`LW, `DC6}:        alu_opcode = `ALU_ADD;
            {`LBU, `DC6}:       alu_opcode = `ALU_ADD;
            {`SB, `DC6}:        alu_opcode = `ALU_ADD;
            {`SW, `DC6}:        alu_opcode = `ALU_ADD;
            {`SC, `DC6}:        alu_opcode = `ALU_ADD;
            {`BEQ, `DC6}:       alu_opcode = `ALU_PASSX;
            {`LL, `DC6}:        alu_opcode = `ALU_ADD;
            {`BNE, `DC6}:       alu_opcode = `ALU_PASSX; 
            {`SPECIAL, `ADD}:   alu_opcode = `ALU_ADD;
            {`SPECIAL, `ADDU}:  alu_opcode = `ALU_ADDU;
            {`SPECIAL, `SUB}:   alu_opcode = `ALU_SUB;
            {`SPECIAL, `SUBU}:  alu_opcode = `ALU_SUBU;
            {`SPECIAL, `AND}:   alu_opcode = `ALU_AND;
            {`SPECIAL, `OR}:    alu_opcode = `ALU_OR;
            {`SPECIAL, `MOVN}:  alu_opcode = `ALU_PASSX;
            {`SPECIAL, `MOVZ}:  alu_opcode = `ALU_PASSX;
            {`SPECIAL, `SLT}:   alu_opcode = `ALU_SLT;
            {`SPECIAL, `SLTU}:  alu_opcode = `ALU_SLTU;
            {`SPECIAL, `SLL}:   alu_opcode = `ALU_SLL;
            {`SPECIAL, `SRL}:   alu_opcode = `ALU_SRL;
            {`SPECIAL, `SLLV}:  alu_opcode = `ALU_SLL;
            {`SPECIAL, `SRLV}:  alu_opcode = `ALU_SRL;
            
            // Our Instructions
            {`SPECIAL2, `MUL}:  alu_opcode = `ALU_MUL;
            {`SPECIAL, `XOR}:   alu_opcode = `ALU_XOR;
            {`XORI, `DC6}:      alu_opcode = `ALU_XOR;
            {`SPECIAL2, `NOR}:  alu_opcode = `ALU_NOR;
            {`SPECIAL, `SRA}:  alu_opcode = `ALU_SRA;
            {`SPECIAL, `SRAV}:  alu_opcode = `ALU_SRA;

            // compare rs data to 0, only care about 1 operand
            {`BGTZ, `DC6}:      alu_opcode = `ALU_PASSX;
            {`BLEZ, `DC6}:      alu_opcode = `ALU_PASSX;
            {`BLTZ_GEZ, `DC6}: begin
                if (isBranchLink)
                    alu_opcode = `ALU_PASSY; // pass link address for mem stage
                else
                    alu_opcode = `ALU_PASSX;
            end
            // pass link address to be stored in $ra
            {`JAL, `DC6}:       alu_opcode = `ALU_PASSY;
            {`SPECIAL, `JALR}:  alu_opcode = `ALU_PASSY;
            // or immediate with 0
            {`LUI, `DC6}:       alu_opcode = `ALU_PASSY;
            default:            alu_opcode = `ALU_PASSX;
    	endcase
    end

//******************************************************************************
// Compute value for 32 bit immediate data
//******************************************************************************

    wire use_imm = &{op != `SPECIAL, op != `SPECIAL2 }; // where to get 2nd ALU operand from: 0 for RtData, 1 for Immediate

    wire [31:0] imm_sign_extend = {{16{immediate[15]}}, immediate};
    wire [31:0] imm_zero_extend = {{16{1'b0}}, immediate};
    wire [31:0] imm_upper = {immediate, 16'b0};

    wire [31:0] imm = (op == `LUI) ? 
                            imm_upper : 
                            (|{op == `ANDI, op == `ORI, op == `XORI, op == `BEQ, op == `BNE}) ?
                                imm_zero_extend : 
                                imm_sign_extend;

//******************************************************************************
// forwarding and stalling logic
//******************************************************************************

    wire forward_rs_mem = &{rs_addr == reg_write_addr_mem, rs_addr != `ZERO, reg_we_mem}; //RS_ADDRESS is about to be written to
    wire forward_rt_mem = &{rt_addr == reg_write_addr_mem, rt_addr != `ZERO, reg_we_mem}; //RT_ADDRESS is about to be written to
    wire forward_rs_ex  = &{rs_addr == reg_write_addr_ex, rs_addr != `ZERO, reg_we_ex};
    wire forward_rt_ex  = &{rt_addr == reg_write_addr_ex, rt_addr != `ZERO, reg_we_ex};

    assign rs_data = forward_rs_ex ? alu_result_ex: (forward_rs_mem ? reg_write_data_mem : rs_data_in);
    assign rt_data = forward_rt_ex ? alu_result_ex: (forward_rt_mem ? reg_write_data_mem : rt_data_in);
    
    wire rs_mem_dependency = &{rs_addr == reg_write_addr_ex, mem_read_ex, rs_addr != `ZERO};
    wire rt_mem_dependency = &{rt_addr == reg_write_addr_ex, mem_read_ex, rt_addr != `ZERO};
    
    wire isLUI = (op == `LUI);
    wire isALUImm = |{op == `ADDI, op == `ADDIU, op == `SLTI, op == `SLTIU, op == `ANDI, op == `ORI, op == `XORI};
    wire read_from_rs = ~|{isLUI, jump_target, jr_target, isShiftImm};
    wire read_from_rt = ~|{isLUI, jump_target, jr_target, isALUImm, mem_read};

    assign stall = (rs_mem_dependency & read_from_rs) || (rt_mem_dependency & read_from_rt);

    assign mem_write_data = rt_data; // Only used in storing.

//******************************************************************************
// Determine ALU inputs and register writeback address
//******************************************************************************

    // for shift operations, use either shamt field or lower 5 bits of rs
    // otherwise use rs

    wire [31:0] shift_amount = isShiftImm ? shamt : rs_data[4:0];
    assign alu_op_x = isShift ? shift_amount : rs_data;

    // for link operations, use next pc (current pc + 8)
    // for immediate operations, use Imm
    // otherwise use rt
    wire [31:0] next_pc = pc + 32'd8;
    assign alu_op_y = link? next_pc : (use_imm) ? imm : rt_data;
    assign reg_write_addr = link? `RA : (use_imm) ? rt_addr : rd_addr;

    // determine when to write back to a register (any operation that isn't an
    // unconditional store, non-linking branch, or non-linking jump)
    assign reg_we = ~|{(mem_we & (op != `SC)), isJ, isBGEZNL, isBGTZ, isBLEZ, isBLTZNL, isBNE, isBEQ};

    // determine whether a register write is conditional
    assign movn = &{op == `SPECIAL, funct == `MOVN};
    assign movz = &{op == `SPECIAL, funct == `MOVZ};

//******************************************************************************
// Memory control
//******************************************************************************
    assign mem_we = |{op == `SW, op == `SB, op == `SC};    
    assign mem_read = |{op == `LW, op == `LB, op == `LBU, op == `LL};
    assign mem_byte = |{op == `SB, op == `LB, op == `LBU};    
    assign mem_signextend = |{~(op == `LBU), op == `LB};      // sign extend sub-word memory reads

//******************************************************************************
// Load linked / Store conditional
//******************************************************************************
    assign mem_sc_id = (op == `SC);

    // 'atomic_id' is high when a load-linked has not been followed by a store.
    assign atomic_id = (op == `LL) ? 1'b1 : |{op == `SW, op == `SB} ? 1'b0 : atomic_id;

    // 'mem_sc_mask_id' is high when a store conditional should not store
    assign mem_sc_mask_id = &{mem_sc_id, ~atomic_id};

endmodule
