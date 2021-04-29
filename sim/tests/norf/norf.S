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
    lui     $t0, 0x5555         # A = 0x55555555
    ori     $t0, $t0, 0x5555
    lui     $t2, 0xAAAA         # B = 0xAAAAAAAA
    ori     $t2, $t2, 0xAAAA
    nor     $t3, $t0, $t0       # C = nor(A, A)
    and     $t4, $t2, $t3       # should get 0
    addiu   $t4, $t4, 1 
    sw      $t4, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done

    .end boot
