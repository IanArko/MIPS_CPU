###############################################################################
# File         : sll.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'sll' instruction.
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
    lui     $t0, 0xdeaf         # A = 0xdeafbeef
    nop
    ori     $t0, $t0, 0x0123
    nop
    nop
    sll     $t1, $t0, 4         # B = 0xdeaf0123 << 4 = 0xeaf01230
    lui     $t2, 0xeaf0         # C = 0xeaf01231
    nop
    ori     $t2, $t2, 0x1231
    nop
    subu    $t3, $t2, $t1       # D = C - B = 1
    nop
    nop
    sw      $t3, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
