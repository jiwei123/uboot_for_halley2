/*
 * Startup Code for MIPS32 XBURST JZ4780 CPU-core
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <config.h>
#include <version.h>
#include <asm/regdef.h>
#include <asm/mipsregs.h>
#include <asm/addrspace.h>
#include <asm/cacheops.h>
#include <asm/arch/base.h>

	.set noreorder

	.globl _start
	.text
_start:
#ifdef CONFIG_SPL_MMC_SUPPORT

	/* magic value ("MSPL") */
	.word 0x4d53504c

#else /* !CONFIG_SPL_MMC_SUPPORT */

	/*
	 * NAND parameters are stored with plenty of redundancy in the
	 * first 192 bytes of the first page of the SPL image.
	 */

#ifdef CONFIG_SYS_NAND_TOGGLE
	.space 64, 0xaa
#else
	.space 64, 0x55
#endif

#if (CONFIG_SYS_NAND_ROWCYCLE == 3)
	.space 32, 0xaa
#else
	.space 32, 0x55
#endif

#if CONFIG_SYS_NAND_PAGE_SIZE == 512
	.space 96, 0x55
#elif CONFIG_SYS_NAND_PAGE_SIZE == 2048
	.space 32, 0x55
	.space 32, 0xaa
	.space 32, 0x55
#elif CONFIG_SYS_NAND_PAGE_SIZE == 4096
	.space 32, 0xaa
	.space 64, 0x55
#elif CONFIG_SYS_NAND_PAGE_SIZE == 8192
	.space 64, 0xaa
	.space 32, 0x55
#elif CONFIG_SYS_NAND_PAGE_SIZE == 16384
	.space 96, 0xaa
#endif

#endif /* !CONFIG_SPL_MMC_SUPPORT */

	/* Invalidate BTB */
	mfc0	v0, CP0_CONFIG, 7
	nop
	ori	v0, v0, 2
	mtc0	v0, CP0_CONFIG, 7
	nop

	/*
	 * CU0=UM=EXL=IE=0, BEV=ERL=1, IP2~7=1
	 */
	li	t0, 0x0040FC04
	mtc0	t0, CP0_STATUS

	/* CAUSE register */
	/* IV=1, use the specical interrupt vector (0x200) */
	li	t1, 0x00800000
	mtc0	t1, CP0_CAUSE

	/* enable bridge radical mode */
	la	t0, CPM_BASE
	lw	t1, 0x24(t0)
	ori	t1, t1, 0x22
	sw	t1, 0x24(t0)

	/* Initialize GOT pointer.
	*/
	bal     1f
	nop
/*	.word   _GLOBAL_OFFSET_TABLE_ */
	1:
	move    gp, ra
	lw      t1, 0(ra)
	move	gp, t1

#if 1
init_caches:
	la	$25, 0x80000000
        li      $2, 3                   // cacheable for kseg0 access
        mtc0    $2, $16                 // CP0_CONFIG
        nop

        li      $2, 0x20000000          // enable idx-store-data cache insn
        mtc0    $2, $26                 // CP0_ERRCTL

        ori     $2, $25, 0              // start address
        ori     $3, $2, 0x7fe0          // end address, total 32KB
        mtc0    $0, $28, 0              // CP0_TAGLO
        mtc0    $0, $28, 1              // CP0_DATALO
#endif
cache_clear_a_line:
        cache   0x8, 0($2)              // Index_Store_Tag_I
        cache   0x9, 0($2)              // Index_Store_Tag_D
        bne     $2, $3, cache_clear_a_line
        addiu   $2, $2, 32              // increment CACHE_LINE_SIZE

#if 1
        ori     $2, $25, 0              // start address
        ori     $3, $2, 0x3fe0          // end address, total 16KB spl stack space
        la      $4, 0x1ffff000          // physical address and 4KB page mask
cache_alloc_a_line:
        and     $5, $2, $4
        ori     $5, $5, 1               // V bit of the physical tag
        mtc0    $5, $28, 0              // CP0_TAGLO
        cache   0x8, 0($2)              // Index_Store_Tag_I
        cache   0x9, 0($2)              // Index_Store_Tag_D
        bne     $2, $3, cache_alloc_a_line
        addiu   $2, $2, 32              // increment CACHE_LINE_SIZE
#endif


	/* Set up stack */
	li	sp, CONFIG_SPL_STACK
	j	board_init_f
	nop

	.globl	validate_cache
	.type	validate_cache, @function
validate_cache:
#if 1
	la	$25, 0x80000000
        ori     $2, $25, 0              // start address
        ori     $3, $2, 0x3fe0          // end address, total 32(or 16)KB spl stack space
	la      $4, 0x1ffff000          // physical address and 4KB page mask
cache_alloc_a_line_2:
        and     $5, $2, $4		// V bit of the physical tag
        mtc0    $5, $28, 0              // CP0_TAGLO
        cache   0x8, 0($2)              // Index_Store_Tag_I
        cache   0x9, 0($2)              // Index_Store_Tag_D
        bne     $2, $3, cache_alloc_a_line_2
        addiu   $2, $2, 32              // increment CACHE_LINE_SIZE
	jr	$31
	nop
#endif
