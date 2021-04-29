###############################################################################
# File         : mul.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'mul' instruction.
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
    ori     $t0, $t0, 0xfffd
    ori     $t1, $0, 0x4        # B = 0x4
    mul     $t2, $t0, $t1       # C = A*B = -12
    mul     $t3, $t0, $t0       # D = A*A = 9
    mul     $t4, $t1, $t1       # E = B*B = 16
    addiu   $s2, $t2, 12
    sltiu   $v0, $s2, 1
    addiu   $s3, $t3, -9
    sltiu   $v1, $s3, 1
    and     $v0, $v0, $v1
    addiu   $s4, $t4, -16
    sltiu   $v1, $s4, 1
    and     $v0, $v0, $v1
    sw      $v0, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done

    .end boot
