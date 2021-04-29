###############################################################################
# File         : lbu.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'lbu' instruction.
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
    nop
    nop
    ori     $t0, $t0, 0x3210
    lui     $t1, 0xde0f
    nop
    nop
    ori     $t1, $t1, 0xbeef
    nop
    nop
    sw      $t1, 0($t0)
    lbu     $s2, 0($t0)         # 0xde
    lbu     $s3, 1($t0)         # 0x0f
    lbu     $s4, 2($t0)         # 0xbe
    lbu     $s5, 3($t0)         # 0xef
    ori     $t2, $0, 0xde
    ori     $t3, $0, 0x0f
    ori     $t4, $0, 0xbe
    ori     $t5, $0, 0xef
    subu    $v1, $t2, $s2
    nop
    nop
    sltiu   $v0, $v1, 1
    subu    $v1, $t3, $s3
    nop
    nop
    sltiu   $v1, $v1, 1
    nop
    nop
    and     $v0, $v0, $v1
    subu    $v1, $t4, $s4
    nop
    nop
    sltiu   $v1, $v1, 1
    nop
    nop
    and     $v0, $v0, $v1
    subu    $v1, $t5, $s5
    nop
    nop
    sltiu   $v1, $v1, 1
    nop
    nop
    and     $v0, $v0, $v1
    nop
    nop
    sw      $v0, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
