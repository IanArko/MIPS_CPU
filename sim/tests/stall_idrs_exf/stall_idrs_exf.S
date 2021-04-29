###############################################################################
# File         : stall_idrs_ex.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of stalling in ID from an Rs mem access in EX
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
    ori     $t0, $0, 0x3210     # Load some valid address 0x00003210 to register $t0
    ori     $t1, $0, 0xffff     # Constant for memory read
    ori     $t2, $0, 0xfffe     # Result mask
    sw      $t1, 0($t0)
    lw      $t3, 0($t0)
    xor     $t4, $t3, $t2
    sw      $t4, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
