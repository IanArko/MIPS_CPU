// Some definitions for making MIPS instructions in irom.v or decoding them in decode.v.

`define BLTZ_GEZ 6'b000001
`define BEQ      6'b000100
`define BNE      6'b000101
`define BLEZ     6'b000110
`define BGTZ     6'b000111
`define BLTZ     5'b00000
`define BGEZ     5'b00001
`define BLTZAL   5'b10000
`define BGEZAL   5'b10001

`define SPECIAL  6'b000000   // op
`define SPECIAL2 6'b011100   // op
`define J        6'b000010   // op
`define JAL      6'b000011   // op
`define JR       6'b001000   // funct
`define JALR     6'b001001   // funct

`define SLL   6'b000000
`define SRL   6'b000010
`define SRA   6'b000011
`define SLLV  6'b000100
`define SRLV  6'b000110
`define SRAV  6'b000111

// OPCODES
`define ADDI  6'b001000
`define ADDIU 6'b001001
`define SLTI  6'b001010
`define SLTIU 6'b001011
`define ANDI  6'b001100
`define ORI   6'b001101
`define XORI  6'b001110
`define LUI   6'b001111
`define LL    6'b110000
`define LW    6'b100011
`define LB    6'b100000
`define LBU   6'b100100
`define SB    6'b101000
`define SC    6'b111000
`define SW    6'b101011

// FUNCTION CODES (more are above for shifts)
`define ADD   6'b100000
`define ADDU  6'b100001
`define SUB   6'b100010
`define SUBU  6'b100011
`define AND   6'b100100
`define OR    6'b100101
`define MOVN  6'b001011
`define MOVZ  6'b001010
`define MUL   6'b000010
`define XOR   6'b100110
`define NOR   6'b100111
`define SLT   6'b101010
`define SLTU  6'b101011
`define DC6   6'bxxxxxx

// Register names
`define ZERO  5'd0
`define AT    5'd1
`define V0    5'd2
`define V1    5'd3
`define A0    5'd4
`define A1    5'd5
`define A2    5'd6
`define A3    5'd7
`define T0    5'd8
`define T1    5'd9
`define T2    5'd10
`define T3    5'd11
`define T4    5'd12
`define T5    5'd13
`define T6    5'd14
`define T7    5'd15
`define S0    5'd16
`define S1    5'd17
`define S2    5'd18
`define S3    5'd19
`define S4    5'd20
`define S5    5'd21
`define S6    5'd22
`define S7    5'd23
`define T8    5'd24
`define T9    5'd25
`define K0    5'd26
`define K1    5'd27
`define GP    5'd28
`define SP    5'd29
`define FP    5'd30
`define RA    5'd31

`define NULL  5'd0 // same as ZERO, but indicate that it's not used, e.g., rt in a bltz

`define NOP 32'b0 // same as sll $zero, $zero, 0


// opcodes for the ALU
// ----------------------------------

// Opcodes we implement
`define ALU_XOR 4'd2
`define ALU_NOR 4'd4
`define ALU_SRA 4'd9
`define ALU_MUL 4'd15

// Already implemented
`define ALU_ADD 4'd13
`define ALU_ADDU 4'd0
`define ALU_AND 4'd1
`define ALU_OR 4'd3
`define ALU_SUB 4'd14
`define ALU_SUBU 4'd5
`define ALU_SLTU 4'd6
`define ALU_SLT 4'd7
`define ALU_SRL 4'd8
`define ALU_SLL 4'd10
`define ALU_PASSX 4'd11
`define ALU_PASSY 4'd12

