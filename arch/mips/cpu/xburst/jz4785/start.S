/*
 * Startup Code for MIPS32 XBURST JZ4785 CPU-core
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

#define RESERVED_FOR_SC(x) .space 1536, x

	.set noreorder
	.globl _start
	.text
_start:
#ifdef CONFIG_SPL_MMC_SUPPORT
	/* magic value ("MSPL") */
	.word 0x4d53504c
	.space 508, 0
	RESERVED_FOR_SC(0)
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
	.space 256, 0x55
	RESERVED_FOR_SC(0x55)
#endif /* CONFIG_SPL_NAND_SUPPORT */

#ifdef CONFIG_SPL_SPI_SUPPORT
#define SSI_GR_BOOT (CONFIG_SYS_EXTAL / (CONFIG_SYS_SPI_BOOT_FREQ * 2) - 1)
	.word 0x55020304
	.word (0x00aa55aa | (SSI_GR_BOOT << 24))
	.space 504, 0
	RESERVED_FOR_SC(0xff)
#endif /* CONFIG_SPL_SPI_SUPPORT */

#ifdef CONFIG_SPL_NOR_SUPPORT
	.word 0
#endif /* CONFIG_SPL_NOR_SUPPORT */

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

	/* Set up stack */
#ifdef CONFIG_SPL_STACK
	li	sp, CONFIG_SPL_STACK
#endif
	j	board_init_f
	nop
