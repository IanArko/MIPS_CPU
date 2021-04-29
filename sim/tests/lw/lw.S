###############################################################################
# File         : lw.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'lw' instruction.
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
    lui     $t0, 0x0            # Load some valid address 0x00003210 to register $t0
    nop
    nop
    ori     $t0, $t0, 0x3210
    nop
    nop
    nop
    sw      $0, 0($t0)
    ori     $t1, $0, 1
    nop
    nop
    nop
    sw      $t1, 0($t0)
    nop
    nop
    nop
    addiu   $t2, $zero, 0x4
    nop
    nop
    lw      $t2, 0($t0)
    nop
    nop
    nop
    sw      $t2, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
