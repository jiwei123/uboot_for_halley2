/*
 * Copyright (C) 2013 Ingenic Semiconductor Inc.
 * Author: sonil <ztyan@ingenic.cn>
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
#include <common.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/base.h>
#include <asm/arch/cpm.h>
#include <asm/arch/clk.h>

unsigned int pll_get_rate(int pll)
{
	unsigned int base = CPM_BASE + CPM_CPAPCR;
	unsigned int *cpxpcr = (unsigned int *)(base + pll * 4);
	unsigned int m, n, od;

	m = ((*cpxpcr >> 24) & 0x7f) + 1;
	n = ((*cpxpcr >> 18) & 0x1f) + 1;
	od = ((*cpxpcr >> 16) & 0x3) == 3;
	od = (od == 3 ? 8 : (od == 0 ? 1 : (2 * od)));

	return (unsigned int)(((unsigned long)CONFIG_SYS_EXTAL) * m / n / od);
}

static unsigned int get_ddr_rate(void)
{
	unsigned int ddrcdr  = cpm_inl(CPM_DDRCDR);

	switch ((ddrcdr >> 30) & 3) {
	case 1:
		return pll_get_rate(APLL) / ((ddrcdr & 0xf) + 1);
	case 2:
		return pll_get_rate(MPLL) / ((ddrcdr & 0xf) + 1);
	}
	return 0;
}

static unsigned int get_cclk_rate(void)
{
	unsigned int cpccr  = cpm_inl(CPM_CPCCR);

	switch ((cpccr >> 28) & 3) {
	case 1:
		return pll_get_rate(APLL) / ((cpccr & 0xf) + 1);
	case 2:
		return pll_get_rate(MPLL) / ((cpccr & 0xf) + 1);
	}
	return 0;
}

static unsigned int get_msc_rate(unsigned int xcdr)
{
	unsigned int msc0cdr  = cpm_inl(CPM_MSC0CDR);
	unsigned int mscxcdr  = cpm_inl(xcdr);

	switch ((msc0cdr >> 30) & 3) {
	case 1:
		return pll_get_rate(APLL) / (((mscxcdr & 0xff) + 1) * 2);
	case 2:
		return pll_get_rate(MPLL) / (((mscxcdr & 0xff) + 1) * 2);
	}
	return 0;
}

unsigned int noinline clk_get_rate(int clk)
{
	switch (clk) {
	case DDR:
		return get_ddr_rate();
	case CPU:
		return get_cclk_rate();
	case MSC0:
		return get_msc_rate(CPM_MSC0CDR);
	case MSC1:
		return get_msc_rate(CPM_MSC1CDR);
	case MSC2:
		return get_msc_rate(CPM_MSC2CDR);
	}
	printf("clk%d is not supported\n", clk);
	return 0;
}

struct cgu __attribute__((weak)) spl_cgu_clksel[] = {
	/*
	 * {offset, sel, sel_bit, en_bit, busy_bit, div}
	 */
	{CPM_MSC0CDR, 1, 30, 29, 28, CGU_MSC_DIV},
	{CPM_MSC1CDR, 1, 30, 29, 28, CGU_MSC_DIV},
	{CPM_DDRCDR, 1, 30, 29, 28, CGU_DDR_DIV},
	{CPM_BCHCDR, 2, 30, 29, 28, CGU_BCH_DIV},
#ifdef CONFIG_VIDEO_JZ4775
	{CPM_LPCDR, 0, 31, 28, 27, CGU_LCD_DIV},
#endif
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
				printf("wait cgu %08X\n",reg);
		}
#ifdef DUMP_CGU_SELECT
		printf("0x%08X: value=0x%08X\n",
		       reg & ~(0xa << 28), readl(reg));
#endif
	}
}

void clk_init(void)
{
	unsigned int reg_clkgr = cpm_inl(CPM_CLKGR);
	unsigned int gate = 0
#if (CONFIG_SYS_UART_BASE == UART0_BASE)
		| CPM_CLKGR_UART0
#endif
#if (CONFIG_SYS_UART_BASE == UART1_BASE)
		| CPM_CLKGR_UART1
#endif 
#if (CONFIG_SYS_UART_BASE == UART2_BASE)
		| CPM_CLKGR_UART2
#endif
#if (CONFIG_SYS_UART_BASE == UART3_BASE)
		| CPM_CLKGR_UART3
#endif
#ifdef CONFIG_JZ_MMC_MSC0
		| CPM_CLKGR_MSC0
#endif
#ifdef CONFIG_JZ_MMC_MSC1
		| CPM_CLKGR_MSC1
#endif
#ifdef CONFIG_JZ_MMC_MSC2
		| CPM_CLKGR_MSC2
#endif
#ifdef CONFIG_VIDEO_JZ4775
		| CPM_CLKGR_LCD
#endif
		;

	reg_clkgr &= ~gate;

	cpm_outl(reg_clkgr,CPM_CLKGR);

	cgu_clks_init(spl_cgu_clksel, ARRAY_SIZE(spl_cgu_clksel));
}

