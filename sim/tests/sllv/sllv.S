###############################################################################
# File         : srlv.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'srlv' instruction.
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
    nop
    ori     $t0, $t0, 0xbeef
    ori     $t1, $0, 4
    nop
    nop
    sllv    $t2, $t0, $t1       # B = 0xdeafbeef << 4 = 0xeafbeef0
    lui     $t3, 0xeafb         # C = 0xeafbeeff
    nop
    nop
    ori     $t3, $t3, 0xeeff
    nop
    nop
    addiu   $t4, $t2, 0xf       # D = B + 0xf = 0xeafbeeff
    nop
    nop
    subu    $v0, $t4, $t3       # D - C = 0
    nop
    nop
    addiu   $v0, $v0, 0x1       # $v0 = 1
    nop
    nop
    sw      $v0, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
