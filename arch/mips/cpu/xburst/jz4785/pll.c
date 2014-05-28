/*
 * JZ4785 pll configuration
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: Zoro <ykli@ingenic.cn>
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
#include <asm/arch/cpm.h>
#include <asm/arch/clk.h>

DECLARE_GLOBAL_DATA_PTR;

#define STOP_MUX_CLK		0
#define SEL_SRC_APLL		1
#define SEL_SRC_EXT		2
#define SEL_SCLK_A		1
#define SEL_MPLL		2

#ifndef CONFIG_SYS_CPCCR_SEL
/**
 * default CPCCR configure.
 * It is suggested if you are NOT sure how it works.
 */
#ifdef CONFIG_CMD_BURN
#define SEL_SRC			SEL_SRC_APLL
#define SEL_CPLL       		SEL_SCLK_A
#define SEL_H0PLL      		SEL_SCLK_A
#define SEL_H2PLL      		SEL_SCLK_A
#if (CONFIG_SYS_APLL_FREQ > 1000000000)
#define DIV_PCLK		10
#define DIV_H2			5
#else
#define DIV_PCLK		8
#define DIV_H2			4
#endif
#ifdef CONFIG_SYS_MEM_DIV
#define DIV_H0			CONFIG_SYS_MEM_DIV
#else
#define DIV_H0			gd->arch.gi->ddr_div
#endif
#define DIV_L2			2
#define DIV_CPU			1
#else /* CONFIG_CMD_BURN */
/* for jz4785, cpu and L2 use apll, AHB2 AHB0 and APB use mpll */
#define SEL_SRC			SEL_SRC_APLL
#define SEL_CPLL		SEL_SCLK_A
#define SEL_H0PLL      		SEL_SCLK_A
#define SEL_H2PLL      		SEL_SCLK_A
/**
 * divisors
 * L2 : C = 3 : 1 (apll > 200M)
 * APB : AHB2 : AHB0 = 4 : 2 : 1
 */
#define DIV_CPU			1
#define DIV_L2			3
#define DIV_H0			4
#define DIV_H2			4
#define DIV_PCLK		8
#endif /* CONFIG_CMD_BURN */

#define CPCCR_CFG		(((SEL_SRC & 0x3) << 30)		\
				 | ((SEL_CPLL & 0x3) << 28)		\
				 | ((SEL_H0PLL & 0x3) << 26)	       	\
				 | ((SEL_H2PLL & 0x3) << 24)	       	\
				 | (((DIV_PCLK - 1) & 0xf) << 16)	\
				 | (((DIV_H2 - 1) & 0xf) << 12)		\
				 | (((DIV_H0 - 1) & 0xf) << 8)		\
				 | (((DIV_L2 - 1) & 0xf) << 4)		\
				 | (((DIV_CPU - 1) & 0xf) << 0))

#else	/* CONFIG_SYS_CPCCR_SEL */
/**
 * Board CPCCR configure.
 * CONFIG_SYS_CPCCR_SEL should be define in [board].h
 */
#define CPCCR_CFG CONFIG_SYS_CPCCR_SEL
#endif

unsigned int get_pllreg_value(int pll)
{
	cpm_cpapcr_t cpapcr;
	cpm_cpmpcr_t cpmpcr;
	unsigned int ret = 0;
	unsigned int pllfreq = 100000000;
	unsigned int extal = gd->arch.gi->extal / 1000000;

	if (CONFIG_CPU_SEL_PLL == pll) {
		if (CONFIG_DDR_SEL_PLL == pll) {
			gd->arch.gi->cpufreq = gd->arch.gi->ddr_div * gd->arch.gi->ddrfreq;
		} else {
			pllfreq = gd->arch.gi->cpufreq;
		}
	} else {
#if defined(CONFIG_SYS_APLL_FREQ)
		if (pll == APLL) {
			pllfreq = CONFIG_SYS_APLL_FREQ > 0 ? CONFIG_SYS_APLL_FREQ : 100;
		}
#endif
#if defined(CONFIG_SYS_MPLL_FREQ)
		if (pll == MPLL) {
			pllfreq = CONFIG_SYS_MPLL_FREQ > 0 ? CONFIG_SYS_MPLL_FREQ : 100;
		}
#endif
		if (CONFIG_DDR_SEL_PLL == pll)
			pllfreq = (pllfreq-(pllfreq%gd->arch.gi->ddrfreq))+gd->arch.gi->ddrfreq;
	}
	pllfreq = pllfreq/1000000;

	switch (pll) {
	case APLL:
		cpapcr.d32 = 0;
		cpapcr.b.APLLN = 1;
		cpapcr.b.APLLOD0 = 1;

		if (pllfreq > 600)
			cpapcr.b.APLLM = 100;
		else
			cpapcr.b.APLLM = 25;

		cpapcr.b.APLLOD1= extal * cpapcr.b.APLLM / cpapcr.b.APLLOD0
				  / cpapcr.b.APLLN / pllfreq;
		if (cpapcr.b.APLLOD1 > 7) {
			printf("CPAPCR.APLLOD1 > 7 error!\n");
			return -1;
		}
		ret = cpapcr.d32;
		printf("cpapcr is %x\n",ret);
		break;
	case MPLL:
		cpmpcr.d32 = 0;
		cpmpcr.b.MPLLN = 1;
		cpmpcr.b.MPLLOD0 = 1;

		if (pllfreq > 600)
			cpmpcr.b.MPLLM = 100;
		else
			cpmpcr.b.MPLLM = 25;

		cpmpcr.b.MPLLOD1= extal * cpmpcr.b.MPLLM / cpmpcr.b.MPLLOD0
			/ cpmpcr.b.MPLLN / pllfreq;
		if (cpmpcr.b.MPLLOD1 > 7) {
			printf("CPMPCR.APLLOD1 > 7 error!\n");
			return -1;
		}
		ret = cpmpcr.d32;
		break;
	default:
		break;
	}

	return ret;
}

static void mpll_init(void)
{
	/* Init MPLL */
	cpm_outl(get_pllreg_value(MPLL) | (0x1 << 0), CPM_CPMPCR);
	while(!(cpm_inl(CPM_CPMPCR) & (0x1 << 3)))
		;
	debug("CPM_CPMPCR %x\n", cpm_inl(CPM_CPMPCR));
}

static void apll_init(void)
{
	/* Init APLL */
	cpm_outl(get_pllreg_value(APLL) | (0x1 << 0), CPM_CPAPCR);
	while(!(cpm_inl(CPM_CPAPCR) & (0x1 << 3)))
		;
	debug("CPM_CPAPCR %x\n", cpm_inl(CPM_CPAPCR));
}

static void cpccr_init(void)
{
	unsigned int cpccr;

	/* change div */
	cpccr = (cpm_inl(CPM_CPCCR) & (0xff << 24))
		| (CPCCR_CFG & ~(0xff << 24))
		| (7 << 20);
	cpm_outl(cpccr,CPM_CPCCR);
	while(cpm_inl(CPM_CPCSR) & 0x7)
		;

	/* change sel */
	cpccr = (CPCCR_CFG & (0xff << 24)) | (cpm_inl(CPM_CPCCR) & ~(0xff << 24));
	cpm_outl(cpccr,CPM_CPCCR);
	while(cpm_inl(CPM_CPCSR) & 0x7)
		;
}
void pll_init(void)
{
	unsigned int cpccr = 0;

	debug("pll init...");

	apll_init();
//	mpll_init();
	cpccr_init();

	debug("ok\n");
}
