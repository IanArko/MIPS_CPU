	.file	1 "sobel_mips.c"
	.section .mdebug.abi32
	.previous
	.nan	legacy
	.gnu_attribute 4, 3
	.text
	.align	2
	.globl	sobel
	.set	nomips16
	.set	nomicromips
	.ent	sobel
	.type	sobel, @function
sobel:
	.frame	$sp,8,$31		# vars= 0, regs= 1/0, args= 0, gp= 0
	.mask	0x00010000,-4
	.fmask	0x00000000,0
	.set	noreorder
	.set	nomacro
	beq	$6,$0,$L20
	nop

	addiu	$sp,$sp,-8
	addiu	$9,$4,642
	addiu	$11,$5,640
	sw	$16,4($sp)
	li	$12,2			# 0x2
$L3:
	beq	$7,$0,$L10
	addiu	$13,$9,642

	move	$10,$13
	li	$14,1			# 0x1
$L9:
	lbu	$4,-642($9)
	lbu	$25,-640($9)
	lbu	$15,0($9)
	lbu	$3,0($10)
	lbu	$5,-641($9)
	lbu	$2,2($9)
	subu	$8,$4,$25
	sll	$15,$15,1
	lbu	$24,1($10)
	addu	$15,$8,$15
	sll	$2,$2,1
	subu	$8,$4,$3
	sll	$4,$5,1
	addu	$5,$8,$4
	lbu	$16,2($10)
	subu	$2,$15,$2
	sll	$4,$24,1
	addu	$2,$3,$2
	subu	$4,$5,$4
	subu	$2,$2,$16
	addu	$4,$4,$25
	subu	$3,$4,$16
	subu	$5,$0,$2
	slt	$8,$2,0
	movn	$2,$5,$8
	subu	$4,$0,$3
	slt	$5,$3,0
	movn	$3,$4,$5
	andi	$2,$2,0xffff
	andi	$3,$3,0xffff
	sltu	$5,$2,256
	bne	$5,$0,$L6
	sltu	$4,$3,256

	li	$2,255			# 0xff
$L6:
	bne	$4,$0,$L7
	nop

	li	$3,255			# 0xff
$L7:
	addu	$2,$2,$3
	andi	$2,$2,0xffff
	addu	$4,$11,$14
	sltu	$5,$2,256
	addiu	$14,$14,1
	bne	$5,$0,$L8
	sltu	$3,$7,$14

	li	$2,255			# 0xff
$L8:
	sb	$2,-641($4)
	addiu	$9,$9,1
	beq	$3,$0,$L9
	addiu	$10,$10,1

$L10:
	sltu	$2,$6,$12
	move	$9,$13
	addiu	$12,$12,1
	beq	$2,$0,$L3
	addiu	$11,$11,640

	lw	$16,4($sp)
	addiu	$sp,$sp,8
$L20:
	j	$31
	nop

	.set	macro
	.set	reorder
	.end	sobel
	.size	sobel, .-sobel
	.align	2
	.globl	start_wait
	.set	nomips16
	.set	nomicromips
	.ent	start_wait
	.type	start_wait, @function
start_wait:
	.frame	$sp,0,$31		# vars= 0, regs= 0/0, args= 0, gp= 0
	.mask	0x00000000,0
	.fmask	0x00000000,0
	.set	noreorder
	.set	nomacro
	li	$3,-2147352576			# 0xffffffff80020000
$L23:
	lw	$2,8($3)
	andi	$2,$2,0x1
	beq	$2,$0,$L23
	nop

	lw	$2,8($3)
	srl	$3,$2,2
	srl	$2,$2,12
	andi	$3,$3,0x3ff
	andi	$2,$2,0x3ff
	sw	$3,0($4)
	j	$31
	sw	$2,0($5)

	.set	macro
	.set	reorder
	.end	start_wait
	.size	start_wait, .-start_wait
	.align	2
	.globl	done_wait
	.set	nomips16
	.set	nomicromips
	.ent	done_wait
	.type	done_wait, @function
done_wait:
	.frame	$sp,0,$31		# vars= 0, regs= 0/0, args= 0, gp= 0
	.mask	0x00000000,0
	.fmask	0x00000000,0
	.set	noreorder
	.set	nomacro
	li	$3,1			# 0x1
	li	$2,-2147352576			# 0xffffffff80020000
	sw	$3,0($2)
	li	$3,-2147352576			# 0xffffffff80020000
$L29:
	lw	$2,8($3)
	andi	$2,$2,0x1
	bne	$2,$0,$L29
	li	$2,-2			# 0xfffffffffffffffe

	sw	$2,0($3)
	j	$31
	nop

	.set	macro
	.set	reorder
	.end	done_wait
	.size	done_wait, .-done_wait
	.section	.text.startup,"ax",@progbits
	.align	2
	.globl	main
	.set	nomips16
	.set	nomicromips
	.ent	main
	.type	main, @function
main:
	.frame	$sp,40,$31		# vars= 0, regs= 5/0, args= 16, gp= 0
	.mask	0x800f0000,-4
	.fmask	0x00000000,0
	.set	noreorder
	.set	nomacro
	addiu	$sp,$sp,-40
	sw	$19,32($sp)
	sw	$18,28($sp)
	sw	$17,24($sp)
	sw	$31,36($sp)
	sw	$16,20($sp)
	li	$19,-2147483648			# 0xffffffff80000000
	li	$18,1			# 0x1
	li	$17,-2			# 0xfffffffffffffffe
$L31:
	li	$16,-2147352576			# 0xffffffff80020000
$L38:
	lw	$2,8($16)
	andi	$2,$2,0x1
	beq	$2,$0,$L31
	nop

	lw	$2,8($16)
	lw	$7,8($16)
	srl	$4,$7,24
	srl	$5,$7,25
	srl	$6,$7,2
	srl	$7,$7,12
	or	$4,$4,$19
	or	$5,$5,$19
	andi	$6,$6,0x3ff
	jal	sobel
	andi	$7,$7,0x3ff

	sw	$18,0($16)
	li	$3,-2147352576			# 0xffffffff80020000
$L39:
	lw	$2,8($3)
	andi	$2,$2,0x1
	bne	$2,$0,$L39
	nop

	sw	$17,0($3)
	j	$L38
	li	$16,-2147352576			# 0xffffffff80020000

	.set	macro
	.set	reorder
	.end	main
	.size	main, .-main
	.ident	"GCC: (GNU) 4.9.1"
