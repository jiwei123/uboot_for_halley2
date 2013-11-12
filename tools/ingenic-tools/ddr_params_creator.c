/*
 * Jz4775 ddr parameters creator.
 *
 * Copyright (C) 2013 Ingenic Semiconductor Co.,Ltd
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

#ifndef CONFIG_SPL_BUILD
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define debug(fmt, ...)
#else
#include <common.h>
#include <generated/ddr_reg_values.h>
#endif
#include <config.h>
#include <ddr/ddr_common.h>

struct tck tck_g = {0, 0};

static inline int calc_nck(int x, int y)
{
	int value;

	value = x * 1000 % y == 0 ? x * 1000 / y : x * 1000 / y + 1;

	return value;
}

static void caculate_tck(struct ddr_params *params)
{
	params->tck.ps = (1000000000 / (params->freq / 1000));
	params->tck.ns = (1000000000 % params->freq == 0)
		? (1000000000 / params->freq)
		: (1000000000 / params->freq + 1);
	tck_g.ps = params->tck.ps;
	tck_g.ns = params->tck.ns;
}

static unsigned int sdram_size(int cs, struct ddr_params *p)
{
	unsigned int dw = p->dw32 ? 4 : 2;
	unsigned int banks = p->bank8 ? 8 : 4;
	unsigned int size = 0;

	switch (cs) {
	case 0:
		if (p->cs0 == 1)
			break;
		else
			return 0;
	case 1:
		if (p->cs1 == 1)
			break;
		else
			return 0;
	default:
		return 0;
	}

	size = (1 << (p->row + p->col)) * dw * banks;

	return size;
}

static void ddrc_params_creat(struct ddrc_reg *ddrc, struct ddr_params *p)
{
	unsigned int tmp = 0, mem_base0 = 0, mem_base1 = 0, mem_mask0 = 0, mem_mask1 = 0;
	unsigned int memsize_cs0, memsize_cs1, memsize;
	struct tck *tck = &p->tck;

	/* TIMING1,2,3,4,5,6 */
	ddrc->timing1.b.tRTP = calc_nck(p->tRTP, tck->ps);
	ddrc->timing1.b.tWTR = calc_nck(p->tWTR, tck->ps) + p->tWL + p->bl / 2; //??
	ddrc->timing1.b.tWR = calc_nck(p->tWR, tck->ps);
	if (ddrc->timing1.b.tWR < 5)
		ddrc->timing1.b.tWR = 5;
	if (ddrc->timing1.b.tWR > 12)
		ddrc->timing1.b.tWR = 12;
	ddrc->timing1.b.tWL = p->tWL;

	ddrc->timing2.b.tCCD = p->tCCD;
	ddrc->timing2.b.tRAS = calc_nck(p->tRAS, tck->ps);
	ddrc->timing2.b.tRCD = calc_nck(p->tRCD, tck->ps);
	ddrc->timing2.b.tRL = p->tRL;

	ddrc->timing3.b.ONUM = 4;
#if 0
	tmp = calc_nck(p->tCKSRE, tck->ps) / 8;
	if (tmp < 1) tmp = 1;
	ddrc->timing3.b.tCKSRE = tmp;
#else
	/* Set DDR_tCKSRE to max to ensafe suspend & resume */
	ddrc->timing3.b.tCKSRE = 7;
#endif
	ddrc->timing3.b.tRP = calc_nck(p->tRP, tck->ps);
	ddrc->timing3.b.tRRD = calc_nck(p->tRRD, tck->ps);
	ddrc->timing3.b.tRC = calc_nck(p->tRC, tck->ps);

	ddrc->timing4.b.tRFC = (calc_nck(p->tRFC, tck->ps) - 1) / 2;
	ddrc->timing4.b.tEXTRW = 3;/* Why?*/
	ddrc->timing4.b.tRWCOV = 3;/* Why?*/
	ddrc->timing4.b.tCKE = calc_nck(p->tCKE, tck->ps) - 1;
	tmp = p->tMINSR;
	if (tmp < 9)
		tmp = 9;
	if (tmp > 129)
		tmp = 129;
	tmp = ((tmp - 1) % 8) ? ((tmp - 1) / 8) : ((tmp - 1) / 8 - 1);
	ddrc->timing4.b.tMINSR = tmp;
	ddrc->timing4.b.tXP = p->tXP;
	ddrc->timing4.b.tMRD = p->tMRD - 1;

	ddrc->timing5.b.tCTLUPD = 0xff; /* 0xff is the default value */
	ddrc->timing5.b.tRTW = p->tRTW;
	if (p->type == LPDDR2)
		ddrc->timing5.b.tRDLAT = p->tRL;
	else
		ddrc->timing5.b.tRDLAT = p->tRDLAT;
	if (p->type == LPDDR2)
		ddrc->timing5.b.tWDLAT = p->tWL;
	else
		ddrc->timing5.b.tWDLAT = p->tWDLAT;

	ddrc->timing6.b.tXSRD = p->tXSRD / 4;
	ddrc->timing6.b.tFAW = p->tFAW; /* NOT sure */
	ddrc->timing6.b.tCFGW = 2;
	ddrc->timing6.b.tCFGR = 2;

	/* REFCNT */
	tmp = p->tREFI / tck->ns;
	tmp = tmp / (16 * (1 << p->div)) - 1;
	if (tmp < 1)
		tmp = 1;
	if (tmp > 0xff)
		tmp = 0xff;
	ddrc->refcnt = (tmp << DDRC_REFCNT_CON_BIT)
		| (p->div << DDRC_REFCNT_CLK_DIV_BIT)
		| DDRC_REFCNT_REF_EN;

	/* CFG */
	ddrc->cfg.b.ROW1 = p->row - 12;
	ddrc->cfg.b.COL1 = p->col - 8;
	ddrc->cfg.b.BA1 = p->bank8;
	ddrc->cfg.b.IMBA = 1;
	ddrc->cfg.b.BSL = (p->bl == 8) ? 1 : 0;
#ifdef CONFIG_DDR_CHIP_ODT
	ddrc->cfg.b.ODTEN = 1;
#else
	ddrc->cfg.b.ODTEN = 0;
#endif
	ddrc->cfg.b.MISPE = 1;
	ddrc->cfg.b.ROW0 = p->row - 12;
	ddrc->cfg.b.COL0 = p->col - 8;
	ddrc->cfg.b.CS1EN = p->cs1;
	ddrc->cfg.b.CS0EN = p->cs0;
	ddrc->cfg.b.CL = 0; /* NOT used in this version */
	ddrc->cfg.b.BA0 = p->bank8;
	ddrc->cfg.b.DW = p->dw32;
	switch (p->type) {
#define _CASE(D, P)			\
	case D:				\
		ddrc->cfg.b.TYPE = P;	\
		break
		_CASE(DDR3, 6);		/* DDR3:0b110 */
		_CASE(LPDDR, 3);	/* LPDDR:0b011 */
		_CASE(LPDDR2, 5);	/* LPDDR2:0b101 */
#undef _CASE
	default:
		break;
	}
	/* CTRL */
	ddrc->ctrl = DDRC_CTRL_ACTPD | DDRC_CTRL_PDT_64 | DDRC_CTRL_ACTSTP
		| DDRC_CTRL_PRET_8 | 0 << 6 | DDRC_CTRL_UNALIGN
		| DDRC_CTRL_ALH | DDRC_CTRL_RDC | DDRC_CTRL_CKE;
	/* MMAP0,1 */
	memsize_cs0 = p->size.chip0;
	memsize_cs1 = p->size.chip1;
	memsize = memsize_cs0 + memsize_cs1;

	if (memsize > 0x20000000) {
		if (memsize_cs1) {
			mem_base0 = 0x0;
			mem_mask0 = (~((memsize_cs0 >> 24) - 1) & ~(memsize >> 24))
				& DDRC_MMAP_MASK_MASK;
			mem_base1 = (memsize_cs1 >> 24) & 0xff;
			mem_mask1 = (~((memsize_cs1 >> 24) - 1) & ~(memsize >> 24))
				& DDRC_MMAP_MASK_MASK;
		} else {
			mem_base0 = 0x0;
			mem_mask0 = ~(((memsize_cs0 * 2) >> 24) - 1) & DDRC_MMAP_MASK_MASK;
			mem_mask1 = 0;
			mem_base1 = 0xff;
		}
	} else {
		mem_base0 = (DDR_MEM_PHY_BASE >> 24) & 0xff;
		mem_mask0 = ~((memsize_cs0 >> 24) - 1) & DDRC_MMAP_MASK_MASK;
		mem_base1 = ((DDR_MEM_PHY_BASE + memsize_cs0) >> 24) & 0xff;
		mem_mask1 = ~((memsize_cs1 >> 24) - 1) & DDRC_MMAP_MASK_MASK;
	}
	ddrc->mmap[0] = mem_base0 << DDRC_MMAP_BASE_BIT | mem_mask0;
	ddrc->mmap[1] = mem_base1 << DDRC_MMAP_BASE_BIT | mem_mask1;
}

static void ddrp_params_creat(struct ddrp_reg *ddrp, struct ddr_params *p)
{
	unsigned int tmp = 0;
	unsigned int dinit1 = 0;
	struct tck *tck = &p->tck;

#define BETWEEN(T, MIN, MAX) if (T < MIN) T = MIN; if (T > MAX) T = MAX
#define PNDEF(N, P, T, MIN, MAX, PS)	\
		T = calc_nck(p->P, PS);	\
		BETWEEN(T, MIN, MAX);	\
		ddrp->dtpr##N.b.P = T

	switch (p->type) {
	case DDR3:
		/* DCR register */
		ddrp->dcr = 3 | (p->bank8 << 3);

		/* MRn registers */
		tmp = calc_nck(p->tWR, tck->ps);
		if (tmp < 5)
			tmp = 5;
		if (tmp > 12)
			tmp = 12;
		if (tmp < 8)
			tmp -= 4;
		else
			tmp = (tmp + 1) / 2;
		ddrp->mr0.ddr3.WR = tmp;
		ddrp->mr0.ddr3.CL_4_6 = p->cl - 4;
		ddrp->mr0.ddr3.BL = (8 - p->bl) / 2;
		ddrp->mr1.ddr3.DIC1 = 1; /* Impedance=RZQ/7 */
#ifdef CONFIG_DDR_CHIP_ODT
		ddrp->mr1.ddr3.RTT2 = 1; /* Effective resistance of ODT RZQ/4 */
#endif
#ifdef CONFIG_DDR_DLL_OFF
		ddrp->mr1.ddr3.DE = 1; /* DLL disable */
#endif
		ddrp->mr2.ddr3.CWL = p->tCWL - 5;

		/* PTRn registers */
		ddrp->ptr0.b.tDLLSRST = calc_nck(p->tDLLSRST, tck->ps);
		ddrp->ptr0.b.tDLLLOCK = calc_nck(5120, tck->ps); /* DDR3 default 5.12us*/
		ddrp->ptr0.b.tITMSRST = 8;

		ddrp->ptr1.b.tDINIT0 = calc_nck(500000, tck->ps); /* DDR3 default 500us*/
		if (((p->tRFC + 10) * 1000) > (5 * tck->ps))  /* ddr3 only */
			dinit1 = (p->tRFC + 10) * 1000;
		else
			dinit1 = 5 * tck->ps;
		tmp = calc_nck(dinit1 / 1000, tck->ps);
		ddrp->ptr1.b.tDINIT1 = tmp;
		if (tmp > 0xff)
			tmp = 0xff;
		ddrp->ptr2.b.tDINIT2 = calc_nck(200000, tck->ps); /* DDR3 default 200us*/
		ddrp->ptr2.b.tDINIT3 = 512;

		/* DTPR0 registers */
		ddrp->dtpr0.b.tMRD = p->tMRD;
		PNDEF(0, tRTP, tmp, 2, 6, tck->ps);
		PNDEF(0, tWTR, tmp, 1, 6, tck->ps);
		PNDEF(0, tRP, tmp, 2, 11, tck->ps);
		PNDEF(0, tRCD, tmp, 2, 11, tck->ps);
		PNDEF(0, tRAS, tmp, 2, 31, tck->ps);
		PNDEF(0, tRRD, tmp, 1, 8, tck->ps);
		PNDEF(0, tRC, tmp, 2, 42, tck->ps);
		ddrp->dtpr0.b.tCCD = (p->tCCD > 4) ? 1 : 0;

		/* DTPR1 registers */
		PNDEF(1, tFAW, tmp, 2, 31, tck->ps);
		PNDEF(1, tMOD, tmp, 12, 15, tck->ps);
		ddrp->dtpr1.b.tMOD -= 12;
		PNDEF(1, tRFC, tmp, 1, 255, tck->ps);
		ddrp->dtpr1.b.tRTODT = 1;

		/* DTPR2 registers */
		tmp = (p->tXS > p->tXSDLL) ? p->tXS : p->tXSDLL;
		tmp = calc_nck(tmp, tck->ps);
		BETWEEN(tmp, 2, 1023);
		ddrp->dtpr2.b.tXS = tmp;

		tmp = (p->tXP > p->tXPDLL) ? p->tXP : p->tXPDLL;
		tmp = calc_nck(tmp, tck->ps);
		BETWEEN(tmp, 2, 31);
		ddrp->dtpr2.b.tXP = tmp;

		tmp = p->tCKE;
		BETWEEN(tmp, 2, 15);
		ddrp->dtpr2.b.tCKE = tmp;

		tmp = p->tDLLLOCK;
		BETWEEN(tmp, 2, 1023);
		ddrp->dtpr2.b.tDLLK = tmp;

		/* PGCR registers */
		ddrp->pgcr = DDRP_PGCR_DQSCFG | 7 << DDRP_PGCR_CKEN_BIT
			| 2 << DDRP_PGCR_CKDV_BIT
			| (p->cs0 | p->cs1 << 1) << DDRP_PGCR_RANKEN_BIT
			| DDRP_PGCR_ZCKSEL_32 | DDRP_PGCR_PDDISDX;
		break;
	case LPDDR:
		ddrp->dcr = 0 | (p->bank8 << 3);
		break;
	case LPDDR2:
		ddrp->dcr = 4 | (p->bank8 << 3);
		break;
	default:
		break;
	}
#undef BETWEEN
#undef PNDEF
}

void fill_in_params(struct ddr_params *params, int type)
{
#ifndef CONFIG_DDR_TYPE_VARIABLE
	memset(params, 0, sizeof(struct ddr_params));

	params->type = type;
	params->freq = CONFIG_SYS_MEM_FREQ;
#endif
	caculate_tck(params);
#ifndef CONFIG_DDR_TYPE_VARIABLE
	params->div = DDR_CLK_DIV;
	params->cs0 = CONFIG_DDR_CS0;
	params->cs1 = CONFIG_DDR_CS1;
	params->dw32 = CONFIG_DDR_DW32;
	params->cl = DDR_CL;
	params->bl = DDR_BL;
	params->col = DDR_COL;
	params->row = DDR_ROW;
	params->bank8 = DDR_BANK8;
	params->tCWL = DDR_tCWL;
	params->tRAS = DDR_tRAS;
	params->tRP = DDR_tRP;
	params->tRCD = DDR_tRCD;
	params->tRC = DDR_tRC;
	params->tWR = DDR_tWR;
	params->tRRD = DDR_tRRD;
	params->tRTP = DDR_tRTP;
	params->tWTR = DDR_tWTR;
	params->tRFC = DDR_tRFC;
	params->tMINSR = DDR_tMINSR;
	params->tXP = DDR_tXP;
	params->tMRD = DDR_tMRD;
	params->tCCD = DDR_tCCD;
	params->tFAW = DDR_tFAW;
	params->tCKE = DDR_tCKE;
	params->tRL = DDR_tRL;
	params->tWL = DDR_tWL;
	params->tRDLAT = DDR_tRDLAT;
	params->tWDLAT = DDR_tWDLAT;
	params->tRTW = DDR_tRTW;
	params->tRAS = DDR_tRAS;
	params->tCKSRE = DDR_tCKSRE;
	params->tDLLLOCK = DDR_tDLLLOCK;
	params->tXSDLL = DDR_tXSDLL;
	params->tMOD = DDR_tMOD;
	params->tXPDLL = DDR_tXPDLL;
	params->tXS = DDR_tXS;
	params->tXSRD = DDR_tXSRD;
	params->tREFI = DDR_tREFI;
	params->tDLLSRST = 50; /* default 50ns */
#endif
	params->size.chip0 = sdram_size(0, params);
	params->size.chip1 = sdram_size(1, params);
}

void ddr_params_creator(struct ddrc_reg *ddrc, struct ddrp_reg *ddrp,
			struct ddr_params *ddr_params)
{
	memset(ddrc, 0, sizeof(struct ddrc_reg));
	memset(ddrp, 0, sizeof(struct ddrp_reg));

	ddrc_params_creat(ddrc, ddr_params);
	ddrp_params_creat(ddrp, ddr_params);
}

#ifdef CONFIG_SPL_BUILD
void ddr_params_assign(struct ddrc_reg *ddrc, struct ddrp_reg *ddrp, struct ddr_params *p)
{
	/* DDRC registers assign */
	DDRC_CFG_VALUE		= ddrc->cfg.d32;
	DDRC_CTRL_VALUE		= ddrc->ctrl;
	DDRC_MMAP0_VALUE	= ddrc->mmap[0];
	DDRC_MMAP1_VALUE	= ddrc->mmap[1];
	DDRC_REFCNT_VALUE	= ddrc->refcnt;
	DDRC_TIMING1_VALUE	= ddrc->timing1.d32;
	DDRC_TIMING2_VALUE	= ddrc->timing2.d32;
	DDRC_TIMING3_VALUE	= ddrc->timing3.d32;
	DDRC_TIMING4_VALUE	= ddrc->timing4.d32;
	DDRC_TIMING5_VALUE	= ddrc->timing5.d32;
	DDRC_TIMING6_VALUE	= ddrc->timing6.d32;

	/* DDRP registers assign */
	DDRP_DCR_VALUE		= ddrp->dcr;
	DDRP_MR0_VALUE		= ddrp->mr0.d32;
	DDRP_MR1_VALUE		= ddrp->mr1.d32;
	DDRP_MR2_VALUE		= ddrp->mr2.d32;
	DDRP_PTR0_VALUE		= ddrp->ptr0.d32;
	DDRP_PTR1_VALUE		= ddrp->ptr1.d32;
	DDRP_PTR2_VALUE		= ddrp->ptr2.d32;
	DDRP_DTPR0_VALUE	= ddrp->dtpr0.d32;
	DDRP_DTPR1_VALUE	= ddrp->dtpr1.d32;
	DDRP_DTPR2_VALUE	= ddrp->dtpr2.d32;
	DDRP_PGCR_VALUE		= ddrp->pgcr;

	DDR_CHIP_0_SIZE		= p->size.chip0;
	DDR_CHIP_1_SIZE		= p->size.chip1;
}
#else /* CONFIG_SPL_BUILD */

#ifdef CONFIG_DDR_HOST_CC
static void params_print(struct ddrc_reg *ddrc, struct ddrp_reg *ddrp)
{
	/* DDRC registers print */
	printf("#define DDRC_CFG_VALUE			0x%08x\n", ddrc->cfg.d32);
	printf("#define DDRC_CTRL_VALUE			0x%08x\n", ddrc->ctrl);
	printf("#define DDRC_MMAP0_VALUE		0x%08x\n", ddrc->mmap[0]);
	printf("#define DDRC_MMAP1_VALUE		0x%08x\n", ddrc->mmap[1]);
	printf("#define DDRC_REFCNT_VALUE		0x%08x\n", ddrc->refcnt);
	printf("#define DDRC_TIMING1_VALUE		0x%08x\n", ddrc->timing1.d32);
	printf("#define DDRC_TIMING2_VALUE		0x%08x\n", ddrc->timing2.d32);
	printf("#define DDRC_TIMING3_VALUE		0x%08x\n", ddrc->timing3.d32);
	printf("#define DDRC_TIMING4_VALUE		0x%08x\n", ddrc->timing4.d32);
	printf("#define DDRC_TIMING5_VALUE		0x%08x\n", ddrc->timing5.d32);
	printf("#define DDRC_TIMING6_VALUE		0x%08x\n", ddrc->timing6.d32);

	/* DDRP registers print */
	printf("#define DDRP_DCR_VALUE			0x%08x\n", ddrp->dcr);
	printf("#define	DDRP_MR0_VALUE			0x%08x\n", ddrp->mr0.d32);
	printf("#define	DDRP_MR1_VALUE			0x%08x\n", ddrp->mr1.d32);
	printf("#define	DDRP_MR2_VALUE			0x%08x\n", ddrp->mr2.d32);
	printf("#define	DDRP_PTR0_VALUE			0x%08x\n", ddrp->ptr0.d32);
	printf("#define	DDRP_PTR1_VALUE			0x%08x\n", ddrp->ptr1.d32);
	printf("#define	DDRP_PTR2_VALUE			0x%08x\n", ddrp->ptr2.d32);
	printf("#define	DDRP_DTPR0_VALUE		0x%08x\n", ddrp->dtpr0.d32);
	printf("#define	DDRP_DTPR1_VALUE		0x%08x\n", ddrp->dtpr1.d32);
	printf("#define	DDRP_DTPR2_VALUE		0x%08x\n", ddrp->dtpr2.d32);
	printf("#define	DDRP_PGCR_VALUE			0x%08x\n", ddrp->pgcr);
}

static void sdram_size_print(struct ddr_params *p)
{
	printf("#define	DDR_CHIP_0_SIZE			%u\n", p->size.chip0);
	printf("#define	DDR_CHIP_1_SIZE			%u\n", p->size.chip1);
}
#else /* CONFIG_DDR_HOST_CC */
static void params_print(struct ddrc_reg *ddrc, struct ddrp_reg *ddrp)
{
	/* DDRC registers print */
	printf("uint32_t DDRC_CFG_VALUE;\n");
	printf("uint32_t DDRC_CTRL_VALUE;\n");
	printf("uint32_t DDRC_MMAP0_VALUE;\n");
	printf("uint32_t DDRC_MMAP1_VALUE;\n");
	printf("uint32_t DDRC_REFCNT_VALUE;\n");
	printf("uint32_t DDRC_TIMING1_VALUE;\n");
	printf("uint32_t DDRC_TIMING2_VALUE;\n");
	printf("uint32_t DDRC_TIMING3_VALUE;\n");
	printf("uint32_t DDRC_TIMING4_VALUE;\n");
	printf("uint32_t DDRC_TIMING5_VALUE;\n");
	printf("uint32_t DDRC_TIMING6_VALUE;\n");

	/* DDRP registers print */
	printf("uint32_t DDRP_DCR_VALUE;\n");
	printf("uint32_t DDRP_MR0_VALUE;\n");
	printf("uint32_t DDRP_MR1_VALUE;\n");
	printf("uint32_t DDRP_MR2_VALUE;\n");
	printf("uint32_t DDRP_PTR0_VALUE;\n");
	printf("uint32_t DDRP_PTR1_VALUE;\n");
	printf("uint32_t DDRP_PTR2_VALUE;\n");
	printf("uint32_t DDRP_DTPR0_VALUE;\n");
	printf("uint32_t DDRP_DTPR1_VALUE;\n");
	printf("uint32_t DDRP_DTPR2_VALUE;\n");
	printf("uint32_t DDRP_PGCR_VALUE;\n");
}

static void sdram_size_print(struct ddr_params *p)
{
	printf("uint32_t DDR_CHIP_0_SIZE;\n");
	printf("uint32_t DDR_CHIP_1_SIZE;\n");
}
#endif /* CONFIG_DDR_HOST_CC */

static void file_head_print(void)
{
	printf("/*\n");
	printf(" * DO NOT MODIFY.\n");
	printf(" *\n");
	printf(" * This file was generated by ddr_params_creator\n");
	printf(" *\n");
	printf(" */\n");
	printf("\n");

	printf("#ifndef __DDR_REG_VALUES_H__\n");
	printf("#define __DDR_REG_VALUES_H__\n\n");
}

static void file_end_print(void)
{
	printf("\n#endif /* __DDR_REG_VALUES_H__ */\n");
}

int main(int argc, char *argv[])
{
	struct ddrc_reg ddrc;
	struct ddrp_reg ddrp;
	struct ddr_params ddr_params;

#ifdef CONFIG_DDR_HOST_CC
#ifdef CONFIG_DDR_TYPE_DDR3
	int type = DDR3;
#elif defined(CONFIG_DDR_TYPE_LPDDR)
	int type = LPDDR;
#elif defined(CONFIG_DDR_TYPE_LPDDR2)
	int type = LPDDR2;
#elif defined(CONFIG_DDR_TYPE_VARIABLE)
	/* params were created by burn tool. */
#error "VARIABLE can NOT be created by ddr_params_creator"
#endif /* CONFIG_DDR_TYPE_DDR3 */
	fill_in_params(&ddr_params, type);
	ddr_params_creator(&ddrc, &ddrp, &ddr_params);
#endif /* CONFIG_DDR_HOST_CC */

	file_head_print();
	params_print(&ddrc, &ddrp);
	sdram_size_print(&ddr_params);
	file_end_print();

	return 0;
}
#endif /* CONFIG_SPL_BUILD */
