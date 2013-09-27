
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
	unsigned int cpccr = cpm_inl(CPM_CPCCR);

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

	cpccr = ((CPCCR_CFG | (7 << 20)) & ~(0xff<<24)) | (cpm_inl(CPM_CPCCR) & (0xff<<24));
	cpm_outl(cpccr,CPM_CPCCR);
	while(cpm_inl(CPM_CPCSR) & 0x7);

	cpccr = (CPCCR_CFG  & (0xff<<24)) | (cpm_inl(CPM_CPCCR) & ~(0xff<<24));
	cpm_outl(cpccr,CPM_CPCCR);

	debug("ok\n");
}
