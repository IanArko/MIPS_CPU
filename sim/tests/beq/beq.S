###############################################################################
# File         : beq.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'beq' instruction.
#
###############################################################################


    .section .boot, "x"
    .balign 4
    .set    noreorder
    .global boot
    .ent    boot
boot:
    lui     $s0, 0x8002         # Load the base address of the status/test registers
    ori     $s1, $0, 1          # Prepare the 'done' status
    ori     $t0, $0, 0xffff
    ori     $t1, $0, 0xfffc
    nop
    nop
    beq     $t0, $t1, $target   # Make sure we don't jump
    nop
    ori     $t0, $0, 0xffff
    ori     $t1, $0, 0xffff
    nop
    nop
    beq     $t0, $t1, $target
    ori     $v0, $0, 0          # The test result starts as a failure

$finish:
    nop
    nop
    sw      $v0, 4($s0)
    sw      $s1, 0($s0)

$done:
    j       $done
    nop

$target:
    j       $finish
    ori     $v0, $0, 1          # Set the result to pass

    .end boot
