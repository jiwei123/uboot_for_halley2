/*
 * JZ4775 pll configuration
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

#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <asm/arch/cpm.h>

#include "pll.h"

#ifndef CONFIG_SYS_CPCCR_SEL
/**
 * default CPCCR configure.
 * It is suggested if you are NOT sure how it works.
 */
#define SEL_SCLKA		1
#define SEL_CPU			1
#define SEL_H0			1
#define SEL_H2			1
#define DIV_PCLK		8
#define DIV_H2			4
#define DIV_H0			4
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

void pll_init(void)
{
	unsigned int cpccr = 0;

	debug("pll init...");

#ifdef APLL_VALUE
	cpm_outl(APLL_VALUE | (0x1 << 8) | 0x20,CPM_CPAPCR);
	while(!(cpm_inl(CPM_CPAPCR) & (0x1<<10)));
	debug("CPM_CPAPCR %x\n", cpm_inl(CPM_CPAPCR));
#endif
#ifdef MPLL_VALUE
	cpm_outl(MPLL_VALUE | (0x1 << 7),CPM_CPMPCR);
	while(!(cpm_inl(CPM_CPMPCR) & (0x1)));
	debug("CPM_CPMPCR %x\n", cpm_inl(CPM_CPMPCR));
#endif

	cpccr = CPCCR_CFG | (7 << 20);
	cpm_outl(cpccr,CPM_CPCCR);
	while(cpm_inl(CPM_CPCSR) & 0x7);

	cpccr = (CPCCR_CFG  & (0xff<<24)) | (cpm_inl(CPM_CPCCR) & ~(0xff<<24));
	cpm_outl(cpccr,CPM_CPCCR);

	debug("ok\n");
}
