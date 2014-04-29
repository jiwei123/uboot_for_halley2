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

#ifndef CONFIG_SYS_CPCCR_SEL
/**
 * default CPCCR configure.
 * It is suggested if you are NOT sure how it works.
 */
#define SEL_SCLKA		1
#define SEL_CPU			1
#define SEL_H0			1
#define SEL_H2			1
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
#define CPCCR_CFG		(((SEL_SCLKA & 0x3) << 30)		\
				 | ((SEL_CPU & 0x3) << 28)		\
				 | ((SEL_H0 & 0x3) << 26)			\
				 | ((SEL_H2 & 0x3) << 24)			\
				 | (((DIV_PCLK - 1) & 0xf) << 16)	\
				 | (((DIV_H2 - 1) & 0xf) << 12)		\
				 | (((DIV_H0 - 1) & 0xf) << 8)		\
				 | (((DIV_L2 - 1) & 0xf) << 4)		\
				 | (((DIV_CPU - 1) & 0xf) << 0))
#else
/**
 * Board CPCCR configure.
 * CONFIG_SYS_CPCCR_SEL should be define in [board].h
 */
#define CPCCR_CFG CONFIG_SYS_CPCCR_SEL
#endif

unsigned int get_pllreg_value(int pll)
{
	cpm_cpapcr_t cpapcr;
	unsigned int pll_out, ret = 0;

	switch (pll) {
	case APLL:
		cpapcr.d32 = 0;
		cpapcr.b.APLLOD0 = 1;
		cpapcr.b.APLLOD1 = 2;

		cpapcr.b.APLLN = 1;
		cpapcr.b.APLLM = (gd->arch.gi->cpufreq * cpapcr.b.APLLN * cpapcr.b.APLLOD0
				  * cpapcr.b.APLLOD1) / gd->arch.gi->extal;
		ret = cpapcr.d32;
	case MPLL:
		/* MPLL is not used */
	default:
		break;
	}

	return ret;
}

void pll_init(void)
{
	unsigned int cpccr = 0;

	debug("pll init...");
#ifdef CONFIG_BURNER
	cpccr = (0x95 << 24) | (7 << 20);
	cpm_outl(cpccr,CPM_CPCCR);
	while(cpm_inl(CPM_CPCSR) & 0x7);
#endif
	/* Only apll is init here */
	cpm_outl(get_pllreg_value(APLL) | (0x1 << 0), CPM_CPAPCR);
	while(!(cpm_inl(CPM_CPAPCR) & (0x1 << 3)));
	debug("CPM_CPAPCR %x\n", cpm_inl(CPM_CPAPCR));

	cpccr = (cpm_inl(CPM_CPCCR) & (0xff << 24))
		| (CPCCR_CFG & ~(0xff << 24))
		| (7 << 20);
	cpm_outl(cpccr,CPM_CPCCR);
	while(cpm_inl(CPM_CPCSR) & 0x7);

	cpccr = (CPCCR_CFG & (0xff << 24)) | (cpm_inl(CPM_CPCCR) & ~(0xff << 24));
	cpm_outl(cpccr,CPM_CPCCR);
	while(cpm_inl(CPM_CPCSR) & 0x7);

	debug("ok\n");
}
