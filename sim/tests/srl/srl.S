###############################################################################
# File         : srl.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'srl' instruction.
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
    nop
    nop
    srl     $t1, $t0, 4         # B = 0xdeafbeef >> 4 = 0x0deafbee
    lui     $t2, 0x0dea         # C = 0x0deafbee
    nop
    nop
    ori     $t2, $t2, 0xfbee
    nop
    nop
    subu    $t3, $t1, $t2       # D = B - C = 0
    nop
    nop
    sltiu   $v0, $t3, 1
    nop
    nop
    sw      $v0, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
