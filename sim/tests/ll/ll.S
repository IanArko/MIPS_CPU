###############################################################################
# File         : ll.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'll' instruction.
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
    ori     $t2, $0, 0xbeef
    nop
    nop
    lui     $t2, 0xdead
    nop
    nop
    sw      $t2, 0($t0)         # Store 0xdeadbeef into 0x00003210
    nop
    nop
    nop
    addu    $t7, $0, $t2        # $t7 = 0xdeadbeef
    ll      $t3, 0($t0)         # Load-Link 0xdeadbeef into $t3
    nop
    nop
    nop
    subu    $t3, $t3, $t2       # $t3 = 0xdeadbeef - 0xdeadbeef = 0 (tests if ll actually loads)
    nop
    nop
    addiu   $t3, $t3, 0x1       # $t3 = 1
    nop
    nop
    nop
    sc      $t2, 0($t0)         # Conditionally store 0xdeadbeef into 0x00003210... $t2 should be 1
    nop
    nop
    nop
    lw      $t4, 0($t0)         # Load 0xdeadbeef into $t4
    nop
    nop
    nop
    sub     $t5, $t4, $t7       # $t5 = 0xdeadbeef - 0xdeadbeef = 0
    nop
    nop
    addiu   $t5, $t5, 0x1       # $t5 = 1
    nop
    nop
    and     $t6, $t5, $t3       # Should be 1
    nop
    nop
    and     $t6, $t6, $t2       # Should be 1
    nop
    nop
    sw      $t6, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
