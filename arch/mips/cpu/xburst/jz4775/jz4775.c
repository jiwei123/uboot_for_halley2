/*
 * JZ4775 common routines
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: Zoro <ykli@ingenic.cn>
 * Based on: arch/mips/cpu/xburst/jz4780/jz4780.c
 *           Written by Paul Burton <paul.burton@imgtec.com>
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

/* #define DEBUG */
#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <asm/arch/clk.h>
#include <asm/arch/cpm.h>
#include <spl.h>
#include <regulator.h>

#ifdef CONFIG_SPL_BUILD

/* Pointer to as well as the global data structure for SPL */
DECLARE_GLOBAL_DATA_PTR;
gd_t gdata __attribute__ ((section(".data")));

extern void pll_init(void);
extern void sdram_init(void);
extern void enable_caches(void);

void board_init_f(ulong dummy)
{
	enable_caches();

	/* Set global data pointer */
	gd = &gdata;

	gpio_init();

	/* Init uart first */
#if (CONFIG_SYS_UART_BASE == UART0_BASE)
	cpm_outl(cpm_inl(CPM_CLKGR) & ~CPM_CLKGR_UART0, CPM_CLKGR);
#endif
#if (CONFIG_SYS_UART_BASE == UART1_BASE)
	cpm_outl(cpm_inl(CPM_CLKGR) & ~CPM_CLKGR_UART1, CPM_CLKGR);
#endif
#if (CONFIG_SYS_UART_BASE == UART2_BASE)
	cpm_outl(cpm_inl(CPM_CLKGR) & ~CPM_CLKGR_UART2, CPM_CLKGR);
#endif
#if (CONFIG_SYS_UART_BASE == UART3_BASE)
	cpm_outl(cpm_inl(CPM_CLKGR) & ~CPM_CLKGR_UART3, CPM_CLKGR);
#endif
#ifdef CONFIG_SPL_SERIAL_SUPPORT
	preloader_console_init();
#endif
	debug("timer init\n");
	timer_init();

	spl_regulator_set_voltage(REGULATOR_CORE, 1300);

	debug("pll init\n");
	pll_init();

	debug("clk init\n");
	clk_init();

	debug("sdram init\n");
	sdram_init();

	/* Clear the BSS */
	memset(__bss_start, 0, (char *)&__bss_end - __bss_start);

	board_init_r(NULL, 0);
}

extern void flush_cache_all(void);

void __noreturn jump_to_image_no_args(struct spl_image_info *spl_image)
{
	typedef void __noreturn (*image_entry_noargs_t)(void);

	image_entry_noargs_t image_entry =
			(image_entry_noargs_t) spl_image->entry_point;

	flush_cache_all();

	debug("image entry point: 0x%X\n", spl_image->entry_point);
	image_entry();
}

#endif /* CONFIG_SPL_BUILD */

/*
 * U-Boot common functions
 */

void enable_interrupts(void)
{
}

int disable_interrupts(void)
{
	return 0;
}
