# Copy the stream engine input buffer to the output buffer.
#
# This program runs on the EE 180 pipelined MIPS processor and is used to test
# the functionality of the stream engine infrastructure.
#
# Memory Map
#       0x0000 0000 - 0x0000 3FFF   (16 KiB local to MIPS)
#       0x0000 4000 - 0x7FFF FFFF   (reserved)
#       0x8000 0000 - 0x8001 FFFF   (128 KiB I/O buffers)
#       0x8002 0000 - 0x8002 0003   (32-bit status  register)
#       0x8002 0004 - 0x8002 0007   (32-bit test    register)
#       0x8002 0008 - 0x8002 000B   (32-bit command register)
#       0x8002 000C - 0xFFFF FFFF   (reserved)


    .section .boot, "x"
    .balign 4
    .set    noreorder
    .global boot
    .ent    boot
boot:
    lui     $t0, 0x8000         # buffer copy start address
    lui     $t1, 0x8002         # buffer copy end address (128 KiB)
    j       loop_cond
    nop

# Copy input buffer to output buffer
#
copy_loop:
    lw      $t2, 0($t0)
    sw      $t2, 0($t0)
    addiu   $t0, $t0, 4

loop_cond:
    bne     $t0, $t1, copy_loop
    nop

# Complete test by setting the test register to 1 for success.
    lui     $t0, 0x8002
    addiu   $t0, $t0, 0x0004
    addiu   $v0, $zero, 0x0001
    sw      $v0, 0($t0)

# Set the status register to done.
    lui     $t0, 0x8002
    addiu   $t0, $t0, 0x0000    # silly assignment
    addiu   $v0, $zero, 0x0001
    sw      $v0, 0($t0)

done:
    j done
    nop

    .end boot
