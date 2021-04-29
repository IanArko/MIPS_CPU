###############################################################################
# File         : subu.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'subu' instruction.
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
    lui     $t0, 0xffff         # A = 0xfffffffd
    nop
    ori     $t0, $t0, 0xfffd
    ori     $t1, $0, 0x4        # B = 0x4
    nop
    nop
    subu    $t2, $t0, $t1       # C = A - B = 0xfffffff9
    lui     $t3, 0xffff         # D = 0xffff8 (like -8 mod 2^32)
    nop
    ori     $t3, $t3, 0xfff8
    nop
    nop
    subu    $t4, $t2, $t3       # F = C - E = 1
    nop
    nop
    sw      $t4, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
