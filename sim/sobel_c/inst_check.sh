#!/bin/bash
#
# A quick check to see if a compiled MIPS executable uses unsupported instructions.
#
# Grant Ayers (ayers@cs.stanford.edu)
#

#### Disassembly file to read (e.g. 'app.lst')
FILEPATH="build/app.lst";

#### List the supported MIPS instructions below
SUPPORTED="add addi addiu addu and andi beq beqz bgez bgezal bgtz blez bltz bltzal bne bnez \
           j jal jalr jr lb lbu li lui lw move movn movz mul negu nop nor ori sb sll \
           sllv slt slti sltiu sltu sra srav srl srlv sub subu sw xor xori";

#### No need to modify below
FAILED=0
if [ ! -f $FILEPATH ]; then
    FAILED=1
fi

INSTRUCTIONS=$(grep '[0-9a-f]*.:. *[0-9a-f]\{8\}. *. *[a-z]*. *' $FILEPATH | awk '{print $3}; done' | sort | uniq)

for I in $INSTRUCTIONS; do
    if test "${SUPPORTED#*$I}" == "$SUPPORTED"; then
        echo "Unsupported instruction '$I'"
        FAILED=1
    fi
done

if [ $FAILED -eq 0 ]; then
    echo "All instructions seem to be supported."
fi
