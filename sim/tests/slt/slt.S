###############################################################################
# File         : slti.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'slti' instruction.
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
    lui     $s2, 0x0
    nop
    nop
    lui     $s3, 0x0fff
    nop
    nop
    ori     $s2, 0x0
    nop
    nop
    slt     $t0, $s2, $s3       # $t0 should be 1
    nop
    nop
    sw      $t0, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
