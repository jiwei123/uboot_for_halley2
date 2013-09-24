/*
 * JZ4780 common routines
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *7
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
#define DEBUG
#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <asm/arch/jz4775.h>
#include <spl.h>

#ifdef CONFIG_SPL_BUILD

/* Pointer to as well as the global data structure for SPL */
DECLARE_GLOBAL_DATA_PTR;
gd_t gdata __attribute__ ((section(".data")));

extern void pll_init(void);
extern void sdram_init(void);
extern void enable_caches(void);


#define CONFIG_APLL_FRQ CONFIG_SYS_MEM_SPEED
#define CONFIG_MSC_CLK 24000000
#define CONFIG_DDR_CLK 204000000
#define CGU_MSC_DIV (CONFIG_APLL_FRQ / CONFIG_MSC_CLK / 2 - 1)
#define CGU_DDR_DIV (CONFIG_APLL_FRQ / CONFIG_DDR_CLK - 1)
#define CGU_BCH_DIV 0

struct cgu {
	unsigned off:8;
	unsigned sel:8;
	unsigned sel_bit:8;
	unsigned en_bit:8;
	unsigned busy_bit:8;
	unsigned div:8;
	unsigned reserved:16;
};

struct cgu spl_cgu_clksel[] = {
	/*
	 * {offset, sel, sel_bit, en_bit, busy_bit, div}
	 */
	{CPM_MSC0CDR, 1, 30, 29, 28, CGU_MSC_DIV},
	{CPM_MSC1CDR, 1, 30, 29, 28, CGU_MSC_DIV},
	{CPM_DDRCDR, 1, 30, 29, 28, CGU_DDR_DIV},
	{CPM_BCHCDR, 2, 30, 29, 28, CGU_BCH_DIV},
};

void cgu_clks_init(struct cgu *cgu_clks, int nr_cgu_clks)
{
	int i;

	for(i = 0; i < nr_cgu_clks; i++) {
		unsigned int xcdr = (cgu_clks[i].sel << cgu_clks[i].sel_bit);
		unsigned int reg = CPM_BASE + cgu_clks[i].off;

		writel(xcdr, reg);
		if (cgu_clks[i].en_bit && cgu_clks[i].busy_bit) {
			writel(xcdr | cgu_clks[i].div | (1 << cgu_clks[i].en_bit), reg);
			while (readl(reg) & (1 << cgu_clks[i].busy_bit))
				;//printf("wait cgu %08X\n",reg);
		}
#ifdef DUMP_CGU_SELECT
		printf("0x%08X: value=0x%08X\n",
		       reg & ~(0xa << 28), readl(reg));
#endif
	}
}

void board_init_f(ulong dummy)
{
	enable_caches();
#if CONFIG_SYS_UART_BASE != UART3_BASE
	/* enable JTAG */
	writel(3 << 30, GPIO_PXINTC(0));
	writel(3 << 30, GPIO_PXMASKC(0));
	writel(3 << 30, GPIO_PXPAT1C(0));
	writel(3 << 30, GPIO_PXPAT0C(0));
#endif

	/* Set global data pointer */
	gd = &gdata;

	/* UART init */
#if CONFIG_SYS_UART_BASE == UART0_BASE
	writel(0x9, GPIO_PXINTC(5));
	writel(0x9, GPIO_PXMASKC(5));
	writel(0x9, GPIO_PXPAT1C(5));
	writel(0x9, GPIO_PXPAT0C(5));
	writel(0x9, GPIO_PXPENC(5));
	writel(readl(CPM_CLKGR0) & ~CPM_CLKGR0_UART0, CPM_CLKGR0);
#elif CONFIG_SYS_UART_BASE == UART1_BASE
#error TODO
#elif CONFIG_SYS_UART_BASE == UART2_BASE
#error TODO
#elif CONFIG_SYS_UART_BASE == UART3_BASE
	writel(1 << 12, GPIO_PXINTC(3));
	writel(1 << 12, GPIO_PXMASKS(3));
	writel(1 << 12, GPIO_PXPAT1S(3));
	writel(1 << 12, GPIO_PXPAT0C(3));
	writel(3 << 30, GPIO_PXINTC(0));
	writel(3 << 30, GPIO_PXMASKC(0));
	writel(3 << 30, GPIO_PXPAT1C(0));
	writel(1 << 30, GPIO_PXPAT0C(0));
	writel(1 << 31, GPIO_PXPAT0S(0));
	writel(readl(CPM_CLKGR0) & ~CPM_CLKGR0_UART3, CPM_CLKGR0);
#endif
#ifdef CONFIG_SPL_SERIAL_SUPPORT
	preloader_console_init();
#endif

	timer_init();
	pll_init();

	cgu_clks_init(spl_cgu_clksel, ARRAY_SIZE(spl_cgu_clksel));

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

phys_size_t initdram(int board_type)
{
	return sdram_size(0) + sdram_size(1);
}
