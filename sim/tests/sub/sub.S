###############################################################################
# File         : sub.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'sub' instruction.
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
    lui     $t0, 0xffff         # A = 0xfffffffd (-3)
    nop
    nop
    ori     $t0, $t0, 0xfffd
    nop
    nop
    sub     $t1, $t0, $t0       # B = A - A = 0
    nop
    nop
    sub     $t2, $t1, $t0       # C = B - A = 3
    ori     $t3, $0, 2          # D = 2
    nop
    nop
    sub     $t4, $t2, $t3       # E = C - D = 1
    nop
    nop
    sw      $t4, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
