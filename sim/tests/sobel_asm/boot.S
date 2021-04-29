###############################################################################
# File         : boot.asm
# Project      : MIPS32 Release 1
# Creator(s)   : Grant Ayers (ayers@cs.stanford.edu)
# Date         : 1 June 2014
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#  Clear bss data and jump to main.
#
###############################################################################


    .section .boot, "wx"
    .balign 4
    .global boot
    .ent    boot
    .set    noreorder
boot:
    la      $t0, _bss_start
    la      $t1, _bss_end
    la      $sp, _sp
    la      $gp, _gp

$bss_clear:
    slt     $t2, $t0, $t1
    beq     $t2, $zero, $run
    nop
    sb      $0, 0($t0)
    j       $bss_clear
    addiu   $t0, $t0, 1

$run:
    jal main
    nop

$done:
    j   $done                   # Loop forever doing nothing
    nop

    .end boot
