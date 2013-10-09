/*
 *  Startup Code for MIPS32 XBURST CPU-core
 *
 *  Copyright (c) 2010 Xiangfu Liu <xiangfu@sharism.cc>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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

#ifdef CONFIG_SYS_NAND_ROWCYCLE_2BYTE
	.space 32, 0x55
#else
	.space 32, 0xaa
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
	mfc0	t0, CP0_CONFIG, 7
	nop
	ori	t0, 2
	mtc0	t0, CP0_CONFIG, 7
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

	/* Set up stack */
	li	sp, CONFIG_SPL_STACK

	b		board_init_f
	 nop

	.globl enable_caches
	.ent enable_caches
enable_caches:
	mtc0	zero, CP0_TAGLO
	mtc0	zero, CP0_TAGHI

	li	t0, KSEG0
	addu	t1, t0, CONFIG_SYS_DCACHE_SIZE
1:
	cache	INDEX_STORE_TAG_D, 0(t0)
	bne	t0, t1, 1b
	addiu	t0, t0, CONFIG_SYS_CACHELINE_SIZE

	li	t0, KSEG0
	addu	t1, t0, CONFIG_SYS_ICACHE_SIZE
2:
	cache	INDEX_STORE_TAG_I, 0(t0)
	bne	t0, t1, 2b
	addiu	t0, t0, CONFIG_SYS_CACHELINE_SIZE

	/* Invalidate BTB */
	mfc0	t0, CP0_CONFIG, 7
	nop
	ori	t0, 2
	mtc0	t0, CP0_CONFIG, 7
	nop

	/* Enable caches */
	li	t0, CONF_CM_CACHABLE_NONCOHERENT
	mtc0	t0, CP0_CONFIG
	nop

	jr	ra
	 nop

	.end enable_caches
