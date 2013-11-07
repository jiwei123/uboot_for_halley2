/*
 * JZ4780 pll configuration
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: Justin <ptkang@ingenic.cn>
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
#define SEL_CPU			2
#define SEL_H0			2
#define SEL_H2			2
#if (CONFIG_SYS_CPU_SPEED > 1000000000)
#define DIV_PCLK		12
#define DIV_H2			6
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

#define CPCCR_CFG		(((SEL_SCLKA & 3) << 30)		\
				 | ((SEL_CPU & 3) << 28)		\
				 | ((SEL_H0 & 3) << 26)			\
				 | ((SEL_H2 & 3) << 24)			\
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

/* this get mnod is only adjust to 48M extal clk
 * so jz4780 is adjust here for its 48M extal clk.
 * here we can't set pllout as unsigned long long for the compiler
 * badly support
 */
int get_mnod(unsigned long pllout, cpm_cpxpcr_t *cpxpcr)
{
	if (pllout <=0 || cpxpcr == NULL) {
		return -1;
	}

	cpxpcr->b.PLLN = 0;
	if (pllout < 600) {
		cpxpcr->b.PLLOD = 1;
	} else {
		cpxpcr->b.PLLOD = 0;
	}
	cpxpcr->b.PLLM = (pllout / CONFIG_SYS_EXTAL) * (cpxpcr->b.PLLOD + 1) * (cpxpcr->b.PLLN + 1)  - 1;

	return 0;
}

void pll_init(void)
{
	cpm_cpxpcr_t cpxpcr;
	unsigned int cpccr = cpm_inl(CPM_CPCCR);
	unsigned int cppcr = cpm_inl(CPM_CPPCR);
	cppcr &= ~(0xfff << 8);			//BWADJ value
	cppcr |= 16 << 8 | 0xff | (0x1 << 30);	//stable delay to MAX, Fastlock mode enable
	cpm_outl(cppcr,CPM_CPPCR);

	debug("pll init...");
#ifdef CONFIG_SYS_APLL_FREQ
	cpxpcr.d32 = 0;
	get_mnod(CONFIG_SYS_APLL_FREQ, &cpxpcr);
	cpm_outl(cpxpcr.d32 | 0x1,CPM_CPAPCR);
	while(!(cpm_inl(CPM_CPAPCR) & (0x1<<4)));
	debug("CPM_CPAPCR %x\n",cpm_inl(CPM_CPAPCR));
#endif
#ifdef CONFIG_SYS_MPLL_FREQ
	cpxpcr.d32 = 0;
	get_mnod(gd->arch.gi->cpufreq, &cpxpcr);
	cpm_outl(cpxpcr.d32 | 0x1,CPM_CPMPCR);
	while(!(cpm_inl(CPM_CPMPCR) & (0x1<<4)));
	debug("2CPM_CPMPCR %x\n",cpm_inl(CPM_CPMPCR));
#endif
#ifdef CONFIG_SYS_EPLL_FREQ
	cpxpcr.d32 = 0;
	get_mnod(CONFIG_SYS_EPLL_FREQ, &cpxpcr);
	cpm_outl(cpxpcr.d32 | 0x1,CPM_CPEPCR);
	while(!(cpm_inl(CPM_CPEPCR) & (0x1<<4)));
	debug("CPM_CPEPCR %x\n",cpm_inl(CPM_CPEPCR));
#endif
#ifdef CONFIG_SYS_VPLL_FREQ
	cpxpcr.d32 = 0;
	get_mnod(CONFIG_SYS_VPLL_FREQ, &cpxpcr);
	cpm_outl(cpxpcr.d32 | 0x1,CPM_CPVPCR);
	while(!(cpm_inl(CPM_CPVPCR) & (0x1<<4)));
	debug("CPM_CPVPCR %x\n",cpm_inl(CPM_CPVPCR));
#endif
	cpccr = CPCCR_CFG | (7 << 20);
	cpm_outl(cpccr,CPM_CPCCR);
	while(cpm_inl(CPM_CPCSR) & 0x7);

	//udelay_scale(clk_get_rate(CPU));
	debug("pll init ok\n");
}
