###############################################################################
# File         : lb.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'lb' instruction.
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
    ori     $t1, $t1, 0xbeef    #t0 = 0xde 0f 32 10
    nop
    nop
    sw      $t1, 0($t0)
    lb      $s2, 0($t0)         # 0xffffffde
    lb      $s3, 1($t0)         # 0x0000000f
    lb      $s4, 2($t0)         # 0xffffffbe
    lb      $s5, 3($t0)         # 0xffffffef
    lui     $t2, 0xffff
    nop
    nop
    ori     $t2, $t2, 0xffde
    ori     $t3, $0, 0x0f
    lui     $t4, 0xffff
    nop
    nop
    ori     $t4, $t4, 0xffbe
    lui     $t5, 0xffff
    nop
    nop
    ori     $t5, $t5, 0xffef
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
