###############################################################################
# File         : jr.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'jr' instruction.
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
    lui     $t0, 0x0
    nop
    nop
    ori     $t0, $0, $next_thing
    nop
    nop
    ori     $t1, $0, $target
    nop
    nop
    jalr    $t1                 # jump-and-link
    nop                         # 1 branch delay slot
    
$next_thing:
    nop
    nop
    nop
    nop
    nop
    nop
    
$target:
    nop
    nop
    subu    $t1, $ra, $t0
    nop
    nop
    addiu   $v0, $t1, 0x1
    nop
    nop
    
$finish:
    sw      $v0, 4($s0)
    sw      $s1, 0($s0)

$done:
    j       $done
    nop
   
    .end boot
