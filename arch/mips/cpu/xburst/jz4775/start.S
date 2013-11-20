/*
 * Startup Code for MIPS32 XBURST JZ4775 CPU-core
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
#endif

#ifdef CONFIG_SPL_NAND_SUPPORT
	/*
	 * NAND parameters are stored with plenty of redundancy in the
	 * first 192 bytes of the first page of the SPL image.
	 */

#if (CONFIG_SYS_NAND_BW8 == 1)
	.space 64, 0x55
#else
	.space 64, 0xaa
#endif
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

#endif /* CONFIG_SPL_NAND_SUPPORT */

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

init_caches:
	li	t0, CONF_CM_CACHABLE_NONCOHERENT
	mtc0	t0, CP0_CONFIG
	nop

	/* enable idx-store-data cache insn */
	li      t0, 0x20000000
	mtc0    t0, CP0_ECC

	li	t1, KSEG0		/* Start address */
#define CACHE_ALLOC_END (CONFIG_SYS_DCACHE_SIZE - CONFIG_SYS_CACHELINE_SIZE)

	ori     t2, t1, CACHE_ALLOC_END	/* End address */
	mtc0	zero, CP0_TAGLO, 0
	mtc0	zero, CP0_TAGLO, 1
cache_clear_a_line:
	cache   INDEX_STORE_TAG_I, 0(t1)
	cache   INDEX_STORE_TAG_D, 0(t1)
	bne     t1, t2, cache_clear_a_line
	addiu   t1, t1, CONFIG_SYS_CACHELINE_SIZE

	li	t1, KSEG0		/* Start address */
	ori     t2, t1, CACHE_ALLOC_END	/* End address */
	la      t3, 0x1ffff000		/* Physical address and 4KB page mask */
cache_alloc_a_line:
	and     t4, t1, t3
	ori     t4, t4, 1		/* V bit of the physical tag */
	mtc0    t4, CP0_TAGLO, 0
	cache   INDEX_STORE_TAG_I, 0(t1)
	cache   INDEX_STORE_TAG_D, 0(t1)
	bne     t1, t2, cache_alloc_a_line
	addiu   t1, t1, CONFIG_SYS_CACHELINE_SIZE

	/* Set up stack */
#ifdef CONFIG_SPL_STACK
	li	sp, CONFIG_SPL_STACK
#endif
	j	board_init_f
	nop

	.globl	validate_cache
	.type	validate_cache, @function
validate_cache:
	li	t1, KSEG0		/* Start address */
	ori     t2, t1, CACHE_ALLOC_END	/* End address */
	la      t3, 0x1ffff000		/* Physical address and 4KB page mask */
cache_alloc_a_line2:
	and     t4, t1, t3
	mtc0    t4, CP0_TAGLO, 0
	cache   INDEX_STORE_TAG_I, 0(t1)
	cache   INDEX_STORE_TAG_D, 0(t1)
	bne     t1, t2, cache_alloc_a_line2
	addiu   t1, t1, CONFIG_SYS_CACHELINE_SIZE
	jr	ra
	nop
