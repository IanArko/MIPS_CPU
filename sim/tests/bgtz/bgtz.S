###############################################################################
# File         : bgez.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'bgez' instruction.
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
    ori     $t0, $0, 0x0        # Load zero
    nop
    nop
    bgtz    $t0, $target        # Should not take
    nop
    nop
    ori     $t0, $0, 0xffff     # Load positive number
    nop
    nop
    bgtz    $t0, $target
    ori     $v0, $0, 0          # The test result starts as a failure

$finish:
    nop
    nop
    sw      $v0, 4($s0)
    sw      $s1, 0($s0)

$done:
    j       $done
    nop

$target:
    j       $finish
    ori     $v0, $0, 1          # Set the result to pass

    .end boot
