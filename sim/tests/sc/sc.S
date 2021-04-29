###############################################################################
# File         : sc.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'sc' instruction.
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
    lui     $t2, 0xffff         # load 0xffffffff into $t2
    nop
    nop
    ori     $t2, $t2, 0xffff
    nop
    nop  
    sw      $t2, 0($t0)         # store 0xffffffff into address 0x00003210
    nop
    nop
    nop
    ll      $t3, 0($t0)         # load-link 0xffffffff into $t3
    nop
    nop
    sw      $t2, 0($t0)         # violate atomicity
    nop
    nop
    sc      $t2, 0($t0)
    nop
    nop
    nop
    addiu   $t4, $0, 0x1
    nop
    nop
    beq     $t2, $t4, $oops
    nop
    sw      $t4, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'
    j       $done
   
$oops:
    sw      $0,  4($s0)          # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
