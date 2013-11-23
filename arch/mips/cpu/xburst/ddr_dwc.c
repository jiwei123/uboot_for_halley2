/*
 * DDR driver for Synopsys DWC DDR PHY.
 * Used by Jz4775, JZ4780...
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

/* #define DEBUG */
#include <config.h>
#include <common.h>
#include <ddr/ddr_common.h>
#include <generated/ddr_reg_values.h>

#include <asm/io.h>
#include <asm/arch/clk.h>

DECLARE_GLOBAL_DATA_PTR;
struct ddr_params *ddr_params_p = NULL;
#ifdef DUMP_DDR
static void dump_ddrc_register(void)
{
	printf("DDRC_STATUS		0x%x\n", ddr_readl(DDRC_STATUS));
	printf("DDRC_CFG		0x%x\n", ddr_readl(DDRC_CFG));
	printf("DDRC_CTRL		0x%x\n", ddr_readl(DDRC_CTRL));
	printf("DDRC_LMR		0x%x\n", ddr_readl(DDRC_LMR));
	printf("DDRC_TIMING1		0x%x\n", ddr_readl(DDRC_TIMING(1)));
	printf("DDRC_TIMING2		0x%x\n", ddr_readl(DDRC_TIMING(2)));
	printf("DDRC_TIMING3		0x%x\n", ddr_readl(DDRC_TIMING(3)));
	printf("DDRC_TIMING4		0x%x\n", ddr_readl(DDRC_TIMING(4)));
	printf("DDRC_TIMING5		0x%x\n", ddr_readl(DDRC_TIMING(5)));
	printf("DDRC_TIMING6		0x%x\n", ddr_readl(DDRC_TIMING(6)));
	printf("DDRC_REFCNT		0x%x\n", ddr_readl(DDRC_REFCNT));
	printf("DDRC_MMAP0		0x%x\n", ddr_readl(DDRC_MMAP0));
	printf("DDRC_MMAP1		0x%x\n", ddr_readl(DDRC_MMAP1));
	printf("DDRC_REMAP1		0x%x\n", ddr_readl(DDRC_REMAP(1)));
	printf("DDRC_REMAP2		0x%x\n", ddr_readl(DDRC_REMAP(2)));
	printf("DDRC_REMAP3		0x%x\n", ddr_readl(DDRC_REMAP(3)));
	printf("DDRC_REMAP4		0x%x\n", ddr_readl(DDRC_REMAP(4)));
	printf("DDRC_REMAP5		0x%x\n", ddr_readl(DDRC_REMAP(5)));
}

static void dump_ddrp_register(void)
{
	printf("DDRP_PIR		0x%x\n", ddr_readl(DDRP_PIR));
	printf("DDRP_PGCR		0x%x\n", ddr_readl(DDRP_PGCR));
	printf("DDRP_PGSR		0x%x\n", ddr_readl(DDRP_PGSR));
	printf("DDRP_PTR0		0x%x\n", ddr_readl(DDRP_PTR0));
	printf("DDRP_PTR1		0x%x\n", ddr_readl(DDRP_PTR1));
	printf("DDRP_PTR2		0x%x\n", ddr_readl(DDRP_PTR2));
	printf("DDRP_DCR		0x%x\n", ddr_readl(DDRP_DCR));
	printf("DDRP_DTPR0		0x%x\n", ddr_readl(DDRP_DTPR0));
	printf("DDRP_DTPR1		0x%x\n", ddr_readl(DDRP_DTPR1));
	printf("DDRP_DTPR2		0x%x\n", ddr_readl(DDRP_DTPR2));
	printf("DDRP_MR0		0x%x\n", ddr_readl(DDRP_MR0));
	printf("DDRP_MR1		0x%x\n", ddr_readl(DDRP_MR1));
	printf("DDRP_MR2		0x%x\n", ddr_readl(DDRP_MR2));
	printf("DDRP_MR3		0x%x\n", ddr_readl(DDRP_MR3));
	printf("DDRP_ODTCR		0x%x\n", ddr_readl(DDRP_ODTCR));
}
#endif

static void reset_dll(void)
{
	writel(3, CPM_DRCG);
	mdelay(5);
	writel(1, CPM_DRCG);
	mdelay(5);
}

static void reset_controller(void)
{
	ddr_writel(0xf << 20, DDRC_CTRL);
	mdelay(5);
	ddr_writel(0, DDRC_CTRL);
	mdelay(5);
}


static void remap_swap(int a, int b)
{
	uint32_t remmap[2], tmp[2];

	remmap[0] = ddr_readl(DDRC_REMAP(a / 4 + 1));
	remmap[1] = ddr_readl(DDRC_REMAP(b / 4 + 1));

#define BIT(bit) ((bit % 4) * 8)
#define MASK(bit) (0x1f << BIT(bit))
	tmp[0] = (remmap[0] & MASK(a)) >> BIT(a);
	tmp[1] = (remmap[1] & MASK(b)) >> BIT(b);

	remmap[0] &= ~MASK(a);
	remmap[1] &= ~MASK(b);

	ddr_writel(remmap[0] | (tmp[1] << BIT(a)), DDRC_REMAP(a / 4 + 1));
	ddr_writel(remmap[1] | (tmp[0] << BIT(b)), DDRC_REMAP(b / 4 + 1));
#undef BIT
#undef MASK
}

static void mem_remap(void)
{
	uint32_t start = 0, num = 0;
	int row, col, dw32, bank8, cs0, cs1;

#ifdef CONFIG_DDR_HOST_CC
	row = DDR_ROW;
	col = DDR_COL;
	dw32 = CONFIG_DDR_DW32;
	bank8 = DDR_BANK8;
	cs0 = CONFIG_DDR_CS0;
	cs1 = CONFIG_DDR_CS1;
#else
	row = ddr_params_p->row;
	col = ddr_params_p->col;
	dw32 = ddr_params_p->dw32;
	bank8 = ddr_params_p->bank8;
	cs0 = ddr_params_p->cs0;
	cs1 = ddr_params_p->cs1;
#endif
	start += row + col + (dw32 ? 4 : 2) / 2;
	start -= 12;

	if (bank8)
		num += 3;
	else
		num += 2;

	if (cs0 && cs1)
		num++;

	for (; num > 0; num--)
		remap_swap(0 + num - 1, start + num - 1);
}

void ddr_controller_init(void)
{
	debug("DDR Controller init\n");

	mdelay(1);
	ddr_writel(DDRC_CTRL_CKE | DDRC_CTRL_ALH, DDRC_CTRL);
	ddr_writel(0, DDRC_CTRL);

	/* DDRC CFG init*/
	ddr_writel(DDRC_CFG_VALUE, DDRC_CFG);

	/* DDRC timing init*/
	ddr_writel(DDRC_TIMING1_VALUE, DDRC_TIMING(1));
	ddr_writel(DDRC_TIMING2_VALUE, DDRC_TIMING(2));
	ddr_writel(DDRC_TIMING3_VALUE, DDRC_TIMING(3));
	ddr_writel(DDRC_TIMING4_VALUE, DDRC_TIMING(4));
	ddr_writel(DDRC_TIMING5_VALUE, DDRC_TIMING(5));
	ddr_writel(DDRC_TIMING6_VALUE, DDRC_TIMING(6));

	/* DDRC memory map configure*/
	ddr_writel(DDRC_MMAP0_VALUE, DDRC_MMAP0);
	ddr_writel(DDRC_MMAP1_VALUE, DDRC_MMAP1);

	ddr_writel(DDRC_CTRL_CKE | DDRC_CTRL_ALH, DDRC_CTRL);
	ddr_writel(DDRC_REFCNT_VALUE, DDRC_REFCNT);
	ddr_writel(DDRC_CTRL_VALUE, DDRC_CTRL);
}

void ddr_phy_init(void)
{
	unsigned int timeout = 10000, i;

	debug("DDR PHY init\n");

	/* DDR training address set*/
	ddr_writel(0x150000, DDRP_DTAR);

	ddr_writel(DDRP_DCR_VALUE, DDRP_DCR);
	ddr_writel(DDRP_MR0_VALUE, DDRP_MR0);
	ddr_writel(DDRP_MR1_VALUE, DDRP_MR1);
	ddr_writel(DDRP_MR2_VALUE, DDRP_MR2);
#ifdef CONFIG_SYS_DDR_CHIP_ODT
	ddr_writel(0, DDRP_ODTCR);
#endif
	ddr_writel(DDRP_PTR0_VALUE, DDRP_PTR0);
	ddr_writel(DDRP_PTR1_VALUE, DDRP_PTR1);
	ddr_writel(DDRP_PTR2_VALUE, DDRP_PTR2);
	ddr_writel(DDRP_DTPR0_VALUE, DDRP_DTPR0);
	ddr_writel(DDRP_DTPR1_VALUE, DDRP_DTPR1);
	ddr_writel(DDRP_DTPR2_VALUE, DDRP_DTPR2);

	for (i = 0; i < 4; i++) {
		unsigned int tmp = ddr_readl(DDRP_DXGCR(i));

		tmp &= ~(3 << 9);
#ifdef CONFIG_DDR_PHY_ODT
#ifdef CONFIG_DDR_PHY_DQ_ODT
		tmp |= 1 << 10;
#endif
#ifdef CONFIG_DDR_PHY_DQS_ODT
		tmp |= 1 << 9;
#endif
#endif
#ifndef CONFIG_DDR_HOST_CC
		if ((i > 1) && (ddr_params_p->dw32 == 0))
			tmp &= ~DDRP_DXGCR_DXEN;
#elif (CONFIG_DDR_DW32 == 0)
		if (i > 1)
			tmp &= ~DDRP_DXGCR_DXEN;
#endif
		ddr_writel(tmp, DDRP_DXGCR(i));
	}

	ddr_writel(DDRP_PGCR_VALUE, DDRP_PGCR);

	while (!(ddr_readl(DDRP_PGSR) == (DDRP_PGSR_IDONE
					  | DDRP_PGSR_DLDONE
					  | DDRP_PGSR_ZCDONE))
	       && (ddr_readl(DDRP_PGSR) != 0x1f)
	       && --timeout);
	if (timeout == 0) {
		printf("DDR PHY init timeout: PGSR=%X\n", ddr_readl(DDRP_PGSR));
		hang();
	} else {
		timeout = 10000;
	}

	debug("DDR chip init\n");
	ddr_writel(DDRP_PIR_INIT | DDRP_PIR_DRAMINT
		   | DDRP_PIR_DRAMRST | DDRP_PIR_DLLSRST, DDRP_PIR);

	while (!(ddr_readl(DDRP_PGSR) == (DDRP_PGSR_IDONE
					  | DDRP_PGSR_DLDONE
					  | DDRP_PGSR_ZCDONE
					  | DDRP_PGSR_DIDONE))
	       && (ddr_readl(DDRP_PGSR) != 0x1f)
	       && --timeout);
	if (timeout == 0) {
		printf("DDR init timeout: PGSR=%X\n", ddr_readl(DDRP_PGSR));
		hang();
	} else {
		timeout = 500000;
	}

	debug("DDR training\n");
	ddr_writel(DDRP_PIR_INIT | DDRP_PIR_QSTRN, DDRP_PIR);

	while ((ddr_readl(DDRP_PGSR) != (DDRP_PGSR_IDONE
					 | DDRP_PGSR_DLDONE
					 | DDRP_PGSR_ZCDONE
					 | DDRP_PGSR_DIDONE
					 | DDRP_PGSR_DTDONE))
	       && !(ddr_readl(DDRP_PGSR)
		   & (DDRP_PGSR_DTDONE | DDRP_PGSR_DTERR | DDRP_PGSR_DTIERR))
	       && --timeout);
	if (timeout == 0) {
		printf("DDR training timeout: PGSR=%X\n", ddr_readl(DDRP_PGSR));
		hang();
	} else if (ddr_readl(DDRP_PGSR)
		   & (DDRP_PGSR_DTERR | DDRP_PGSR_DTIERR)) {
		int i = 0;

		printf("DDR training error: PGSR=%X\n", ddr_readl(DDRP_PGSR));
		for (i = 0; i < 4; i++) {
			printf("DX%dGSR0: %x\n", i, ddr_readl(DDRP_DXGSR0(i)));
		}
		hang();
	}
#if defined(CONFIG_DDR_PHY_IMPED_PULLUP) && defined(CONFIG_DDR_PHY_IMPED_PULLDOWN)
	/**
	 * DDR3 240ohm RZQ output impedance:
	 * 	55.1ohm		0xc
	 * 	49.2ohm		0xd
	 * 	44.5ohm		0xe
	 * 	40.6ohm		0xf
	 * 	37.4ohm		0xa
	 * 	34.7ohm		0xb
	 * 	32.4ohm		0x8
	 * 	30.4ohm		0x9
	 * 	28.6ohm		0x18
	 */
	i = ddr_readl(DDRP_ZQXCR0(0)) & ~0x3ff;
	i |= DDRP_ZQXCR_ZDEN
		| ((CONFIG_DDR_PHY_IMPED_PULLUP & 0x1f) << DDRP_ZQXCR_PULLUP_IMPED_BIT)
		| ((CONFIG_DDR_PHY_IMPED_PULLDOWN & 0x1f) << DDRP_ZQXCR_PULLDOWN_IMPED_BIT);
	ddr_writel(i, DDRP_ZQXCR0(0));
#endif
	debug("DDR PHY init OK\n");
}

/* DDR sdram init */
void sdram_init(void)
{
#ifndef CONFIG_DDR_HOST_CC
	struct ddrc_reg ddrc;
	struct ddrp_reg ddrp;
	int type = VARIABLE;
  #ifndef CONFIG_DDR_TYPE_VARIABLE
	struct ddr_params ddr_params;
    #ifdef CONFIG_DDR_TYPE_DDR3
	type = DDR3;
    #elif defined(CONFIG_DDR_TYPE_LPDDR)
	type = LPDDR;
    #elif defined(CONFIG_DDR_TYPE_LPDDR2)
	type = LPDDR2;
    #endif /* CONFIG_DDR_TYPE_DDR3 */
	ddr_params_p = &ddr_params;
  #else
	ddr_params_p = &gd->arch.gi->ddr_params;
	ddr_params_p->freq = gd->arch.gi->cpufreq / gd->arch.gi->ddr_div;
  #endif
	fill_in_params(ddr_params_p, type);
	ddr_params_creator(&ddrc, &ddrp, ddr_params_p);
	ddr_params_assign(&ddrc, &ddrp, ddr_params_p);
#endif /* CONFIG_DDR_HOST_CC */

	debug("sdram init start\n");
	reset_dll();
	reset_controller();

	/* DDR PHY init*/
	ddr_phy_init();
	ddr_writel(0, DDRC_CTRL);

	/* DDR Controller init*/
	ddr_controller_init();

	/* DDRC address remap configure*/
	mem_remap();

	ddr_writel(ddr_readl(DDRC_STATUS) & ~DDRC_DSTATUS_MISS, DDRC_STATUS);

	debug("sdram init finished\n");
#ifdef DUMP_DDR
	dump_ddrc_register();
	dump_ddrp_register();
#endif
}

phys_size_t initdram(int board_type)
{
	return DDR_CHIP_0_SIZE + DDR_CHIP_1_SIZE;
}
