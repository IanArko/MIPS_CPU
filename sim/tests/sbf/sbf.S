###############################################################################
# File         : sb.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'sb' instruction.
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
    lui     $t0, 0              # Load some valid address 0x00003210 to register $t0
    ori     $t0, $t0, 0x3210
    ori     $s2, $0, 0x00ef
    ori     $s3, $0, 0x00be   
    ori     $s4, $0, 0x00af   
    ori     $s5, $0, 0x00de   
    sb      $s2, 3($t0)         
    sb      $s3, 2($t0)         
    sb      $s4, 1($t0)         
    sb      $s5, 0($t0)         
    lw      $s2, 0($t0)         # Load 0xdeafbeef into $s2 from memory
    lui     $s3, 0xdeaf         # Load 0xdeafbeef into $s3
    ori     $s3, 0xbeef
    subu    $s2, $s2, $s3
    addiu   $v0, $s2, 0x1 
    sw      $v0, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done

    .end boot
