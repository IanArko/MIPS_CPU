###############################################################################
# File         : hail.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Runs a hailstone sequence. 
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
    
    lui     $s2, 0x0000         # Load starting
    ori     $s2, 3543
    addiu   $s3, $0, 0x1        # Length begins at 1
    
$loop:
    addiu   $t0, $0, 0x1
    beq     $s2, $t0, $finish
    nop
    sll     $t1, $s2, 0x1f
    srl     $t1, $t1, 0x1f      # $t1 is 0 if even
    beq     $t1, $0, $even
    nop
$odd:
    addiu   $t2, $0, 0x3
    mul     $s2, $s2, $t2
    addiu   $s2, $s2, 0x1
    j       $increment
    nop
$even:
    srl     $s2, $s2, 1
    
$increment:
    addiu   $s3, $s3, 1
    j       $loop
    nop
    
$finish:
    lui     $s4, 0x0000
    ori     $s4, 57             # Expected
    subu    $t2, $s3, $s4
    addiu   $t2, $t2, 0x1
    sw      $t2, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
