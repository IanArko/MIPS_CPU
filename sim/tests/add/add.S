###############################################################################
# File         : add.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'add' instruction.
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
    ori     $t0, $t0, 0xfffd
    ori     $t1, $0, 0x4        # B = 0x4
    nop
    nop
    add     $t2, $t0, $t1       # C = A+B = 1
    nop
    nop
    sw      $t2, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
