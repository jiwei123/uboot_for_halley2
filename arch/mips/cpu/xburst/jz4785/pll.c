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

/* #define DEBUG*/
#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <asm/arch/cpm.h>
#include <asm/arch/clk.h>

DECLARE_GLOBAL_DATA_PTR;

struct pll_cfg {
	unsigned apll_freq;
	unsigned mpll_freq;
	unsigned cdiv;
	unsigned l2div;
	unsigned h0div;
	unsigned h2div;
	unsigned pdiv;
} pll_cfg;

#define SEL_SRC		0X1
#define SEL_CPLL	((CONFIG_CPU_SEL_PLL == APLL) ? 0x1 : 0x2)
#define SEL_H0CLK	((CONFIG_DDR_SEL_PLL == APLL) ? 0x1 : 0x2)
#define SEL_H2CLK	SEL_H0CLK

#define CPCCR_CFG	\
	(((SEL_SRC& 3) << 30)                \
	 | ((SEL_CPLL & 3) << 28)                \
	 | ((SEL_H0CLK & 3) << 26)                 \
	 | ((SEL_H2CLK & 3) << 24)                 \
	 | (((pll_cfg.pdiv- 1) & 0xf) << 16)       \
	 | (((pll_cfg.h2div - 1) & 0xf) << 12)         \
	 | (((pll_cfg.h0div - 1) & 0xf) << 8)          \
	 | (((pll_cfg.l2div - 1) & 0xf) << 4)          \
	 | (((pll_cfg.cdiv - 1) & 0xf) << 0))

static unsigned int get_pllreg_value(int freq)
{
	cpm_cpxpcr_t cppcr;
	unsigned int ret = 0;
	unsigned int pllfreq = freq / 1000000;
	unsigned int extal = gd->arch.gi->extal / 1000000;


	cppcr.d32 = 0;
	cppcr.b.PLLN = 1;
	cppcr.b.PLLOD0 = 1;
	
	if (!pllfreq)
		return -1;

	if (pllfreq > 600)
		cppcr.b.PLLM = 100;
	else
		cppcr.b.PLLM = 25;

	cppcr.b.PLLOD1= extal * cppcr.b.PLLM / cppcr.b.PLLOD0
		/ cppcr.b.PLLN / pllfreq;
	if (cppcr.b.PLLOD1 > 7) {
		printf("CPAPCR.APLLOD1 > 7 error!\n");
		return -1;
	}
	ret = cppcr.d32;
	printf("cppcr is %x\n",ret);

	return cppcr.d32;
}

static void pll_set(int pll,int freq)
{
	unsigned int regvalue = get_pllreg_value(freq);
	if (regvalue == -1)
		return 0;
	switch (pll) {
		case APLL:
			/* Init APLL */
			cpm_outl(regvalue | (0x1 << 0), CPM_CPAPCR);
			while(!(cpm_inl(CPM_CPAPCR) & (0x1 << 3)))
				;
			debug("CPM_CPAPCR %x\n", cpm_inl(CPM_CPAPCR));
			break;
		case MPLL:
			/* Init MPLL */
			cpm_outl(regvalue | (0x1 << 0), CPM_CPMPCR);
			while(!(cpm_inl(CPM_CPMPCR) & (0x1 << 3)))
				;
			debug("CPM_CPMPCR %x\n", cpm_inl(CPM_CPMPCR));
			break;
	}
}

static void cpccr_init(void)
{
	unsigned int cpccr;

	/* change div */
	cpccr = (cpm_inl(CPM_CPCCR) & (0xff << 24))
		| (CPCCR_CFG & ~(0xff << 24))
		| (7 << 20);
	cpm_outl(cpccr,CPM_CPCCR);
	while(cpm_inl(CPM_CPCSR) & 0x7);

	/* change sel */
	cpccr = (CPCCR_CFG & (0xff << 24)) | (cpm_inl(CPM_CPCCR) & ~(0xff << 24));
	cpm_outl(cpccr,CPM_CPCCR);

	debug("cppcr 0x%x\n",cpm_inl(CPM_CPCCR));
}

#define ALIGN_TO_CPU(freq,cpufreq) ({\
	unsigned x = -1; \
	if (!((freq)%(cpufreq)))	{ \
		pll_cfg.cdiv = (freq)/(cpufreq) ? (freq)/(cpufreq) : 1;	\
		x = (pll_cfg.cdiv * (cpufreq)); \
	} else { \
		error("pll freq is not integer times than cpu freq"); \
		while (1)	\
			asm volatile ("wait\n\t");	\
	} \
	x; \
})

#define ALIGN_TO_DDR(freq, ddrfreq) ({	\
	unsigned x = -1;	\
	if (!((freq)%(ddrfreq)))	{\
		gd->arch.gi->ddr_div = (freq)/(ddrfreq) ? (freq)/(ddrfreq) : 1;	\
		x = (gd->arch.gi->ddr_div * (ddrfreq));	\
	} else { \
		error("pll or cpu freq is not integer times than ddr freq"); \
		while (1)	\
			asm volatile ("wait\n\t");	\
	} \
	x; \
})

static int freq_correcting(void)
{
	pll_cfg.mpll_freq = CONFIG_SYS_MPLL_FREQ;
	pll_cfg.apll_freq = CONFIG_SYS_APLL_FREQ;

#define SEL_MAP(cpu,ddr) ((cpu<<16)|(ddr&0xffff))
	switch (SEL_MAP(CONFIG_CPU_SEL_PLL,CONFIG_DDR_SEL_PLL)) {
	case SEL_MAP(APLL,APLL):
		gd->arch.gi->cpufreq = ALIGN_TO_DDR(gd->arch.gi->cpufreq, gd->arch.gi->ddrfreq);
		pll_cfg.apll_freq = ALIGN_TO_CPU(pll_cfg.apll_freq, gd->arch.gi->cpufreq);
		break;
	case SEL_MAP(MPLL,MPLL):
		gd->arch.gi->cpufreq = ALIGN_TO_DDR(gd->arch.gi->cpufreq, gd->arch.gi->ddrfreq);
		//printf(" gd->arch.gi->cpufreq %d, gd->arch.gi->ddrfreq = %d\n", gd->arch.gi->cpufreq,gd->arch.gi->ddrfreq);
		pll_cfg.mpll_freq = ALIGN_TO_CPU(pll_cfg.mpll_freq, gd->arch.gi->cpufreq);
		break;
	case SEL_MAP(APLL,MPLL):
		pll_cfg.mpll_freq = ALIGN_TO_DDR(pll_cfg.mpll_freq, gd->arch.gi->ddrfreq);
		pll_cfg.apll_freq = ALIGN_TO_CPU(pll_cfg.apll_freq, gd->arch.gi->cpufreq);
		break;
	case SEL_MAP(MPLL,APLL):
		pll_cfg.apll_freq = ALIGN_TO_DDR(pll_cfg.apll_freq, gd->arch.gi->ddrfreq);
		pll_cfg.mpll_freq = ALIGN_TO_CPU(pll_cfg.mpll_freq, gd->arch.gi->cpufreq);
		break;
	}
	if (gd->arch.gi->cpufreq < 0 || pll_cfg.apll_freq < 0 || pll_cfg.mpll_freq < 0)
		return -1;

	pll_cfg.l2div = (pll_cfg.cdiv * 2);
	pll_cfg.h0div = gd->arch.gi->ddr_div;
	pll_cfg.h2div = gd->arch.gi->ddr_div;
	pll_cfg.pdiv = 2 * gd->arch.gi->ddr_div;
	printf("mpll_freq %d \napll_freq %d \nddr_freq %d \n",
			pll_cfg.mpll_freq,
			pll_cfg.apll_freq,
			gd->arch.gi->ddrfreq);
	printf("h0div : h2div : pdiv = %d:%d:%d\n cdiv : l2div = %d:%d\n",
		pll_cfg.h0div,pll_cfg.h2div,pll_cfg.pdiv,
		pll_cfg.cdiv,pll_cfg.l2div);
	return 0;
}

int pll_init(void)
{
	freq_correcting();
	pll_set(APLL,pll_cfg.apll_freq);
	pll_set(MPLL,pll_cfg.mpll_freq);
	cpccr_init();
}

