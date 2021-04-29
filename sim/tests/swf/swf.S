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
    ori     $t0, $t0, 0x3210
    lui     $t2, 0xffff         # load 0xffffffff into $t2
    ori     $t2, $t2, 0xffff
    sw      $t2, 0($t0)          # store 0xffffffff into address 0x00003210
    lw      $t2, 0($t0)          # load 0xffffffff into $t2
    addi    $t3, $t2, 0x2       # $t3 = 0xffffffff + 2 = 1!  
    sw      $t3, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done

    .end boot
