/*
 * Jz4780 ddr routines
 *
 *  Copyright (c) 2006
 *  Ingenic Semiconductor, <cwjia@ingenic.cn>
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
#include <ddr/ddr_chips.h>
#include <generated/ddr_params.h>

#include <asm/io.h>
#include <asm/arch/base.h>
#include <asm/arch/clk.h>
#include <asm/arch/ddr.h>

#define REMAP_BIT(bit) ((bit % 4) * 8)
#define REMAP_MASK(bit) (0x1f << REMAP_BIT(bit))

#ifdef DUMP_DDR
static void dump_ddrc_register(void)
{
	printf("#define DDRC_CFG_VALUE		0x%x\n", REG_DDRC_CFG);			/*DDR Configure Register*/
	printf("#define DDRC_CTRL_VALUE		0x%x\n", REG_DDRC_CTRL);		/*DDR Control Register*/
	printf("#define DDRC_LMR_VALUE		0x%x\n", REG_DDRC_LMR);			/*DDR Load-Mode-Register*/
	printf("#define DDRC_TIMING1_VALUE	0x%x\n", REG_DDRC_TIMING(1));		/*DDR Timing Configure Register 1*/
	printf("#define DDRC_TIMING2_VALUE	0x%x\n", REG_DDRC_TIMING(2));		/*DDR Timing Configure Register 2*/
	printf("#define DDRC_TIMING3_VALUE	0x%x\n", REG_DDRC_TIMING(3));		/*DDR Timing Configure Register 3*/
	printf("#define DDRC_TIMING4_VALUE	0x%x\n", REG_DDRC_TIMING(4));		/*DDR Timing Configure Register 4*/
	printf("#define DDRC_TIMING5_VALUE	0x%x\n", REG_DDRC_TIMING(5));		/*DDR Timing Configure Register 5*/
	printf("#define DDRC_TIMING6_VALUE	0x%x\n", REG_DDRC_TIMING(6));		/*DDR Timing Configure Register 6*/
	printf("#define DDRC_REFCNT_VALUE	0x%x\n", REG_DDRC_REFCNT);		/*Auto-Refresh Counter*/
	printf("#define DDRC_MMAP0_VALUE	0x%x\n", REG_DDRC_MMAP0);		/*DDR Memory CS0 Map Configure Register*/
	printf("#define DDRC_MMAP1_VALUE	0x%x\n", REG_DDRC_MMAP1);		/*DDR Memory CS1 Map Configure Register*/
	printf("#define DDRC_REMAP1_VALUE	0x%x\n", REG_DDRC_REMMAP(1));		/*DDR address remapping register1*/
	printf("#define DDRC_REMAP2_VALUE	0x%x\n", REG_DDRC_REMMAP(2));		/*DDR address remapping register2*/
	printf("#define DDRC_REMAP3_VALUE	0x%x\n", REG_DDRC_REMMAP(3));		/*DDR address remapping register3*/
	printf("#define DDRC_REMAP4_VALUE	0x%x\n", REG_DDRC_REMMAP(4));		/*DDR address remapping register4*/
	printf("#define DDRC_REMAP5_VALUE	0x%x\n", REG_DDRC_REMMAP(5));		/*DDR address remapping register5*/
}
static void dump_ddrp_register(void)
{
	printf("#define	DDRP_PIR_VALUE		0x%x\n", REG_DDRP_PIR);			/*PHY Initialization Register*/
	printf("#define	DDRP_PGCR_VALUE		0x%x\n", REG_DDRP_PGCR);		/*PHY General Configuration Register*/
	printf("#define	DDRP_PGSR_VALUE		0x%x\n", REG_DDRP_PGSR);		/*PHY General Status Register*/
	printf("#define	DDRP_PTR0_VALUE		0x%x\n", REG_DDRP_PTR0);		/*PHY Timing Register 0*/
	printf("#define	DDRP_PTR1_VALUE		0x%x\n", REG_DDRP_PTR1);		/*PHY Timing Register 1*/
	printf("#define	DDRP_PTR2_VALUE		0x%x\n", REG_DDRP_PTR2);		/*PHY Timing Register 2*/
	printf("#define	DDRP_DCR_VALUE		0x%x\n", REG_DDRP_DCR);			/*DRAM Configuration Register*/
	printf("#define	DDRP_DTPR0_VALUE	0x%x\n", REG_DDRP_DTPR0);		/*DRAM Timing Parameters Register 0*/
	printf("#define	DDRP_DTPR1_VALUE	0x%x\n", REG_DDRP_DTPR1);		/*DRAM Timing Parameters Register 1*/
	printf("#define	DDRP_DTPR2_VALUE	0x%x\n", REG_DDRP_DTPR2);		/*DRAM Timing Parameters Register 2*/
	printf("#define	DDRP_MR0_VALUE		0x%x\n", REG_DDRP_MR0);			/*Mode Register 0*/
	printf("#define	DDRP_MR1_VALUE		0x%x\n", REG_DDRP_MR1);			/*Mode Register 1*/
	printf("#define	DDRP_MR2_VALUE		0x%x\n", REG_DDRP_MR2);			/*Mode Register 2*/
	printf("#define	DDRP_MR3_VALUE		0x%x\n", REG_DDRP_MR3);			/*Mode Register 3*/
	printf("#define	DDRP_ODTCR_VALUE	0x%x\n", REG_DDRP_ODTCR);		/*ODT Configuration Register*/
}
#endif

struct ddrc_mmap {
	unsigned int mem_base;
	unsigned int mem_mask;
};


struct ddrc_remap_registers {
	unsigned int ddrc_remap1_reg;
	unsigned int ddrc_remap2_reg;
	unsigned int ddrc_remap3_reg;
	unsigned int ddrc_remap4_reg;
	unsigned int ddrc_remap5_reg;
};

int get_jz4775_ddrc_mmap_data(struct ddrc_mmap ddrc_mmap[2], unsigned int ddrc_cfg_reg)
{
	unsigned int ddr_cfg;
	unsigned int rows, cols, dw, banks, cs0, cs1;
	unsigned int memsize = 0, size = 0, memsize0, memsize1, mem_base0, mem_base1, mem_mask0, mem_mask1;
	ddr_cfg = ddrc_cfg_reg;
	rows = 12 + ((ddr_cfg & DDRC_CFG_ROW0_MASK) >> DDRC_CFG_ROW0_BIT);
	cols = 8 + ((ddr_cfg & DDRC_CFG_COL0_MASK) >> DDRC_CFG_COL0_BIT);

	dw = (ddr_cfg & DDRC_CFG_DW) ? 4 : 2;
	banks = (ddr_cfg & DDRC_CFG_BA0) ? 8 : 4;
	cs0 = (ddr_cfg & DDRC_CFG_CS0EN) ? 1 : 0;
	cs1 = (ddr_cfg & DDRC_CFG_CS1EN) ? 1 : 0;

	size = (1 << (rows + cols)) * dw * banks;
	memsize = size * (cs0 + cs1);

	memsize0 = memsize / (DDR_CS1EN + DDR_CS0EN);
	memsize1 = memsize - memsize0;
	debug("memsize0: 0x%08x\n", memsize0);
	debug("memsize1: 0x%08x\n", memsize1);

	if (!memsize1 && memsize0 > 0x20000000) {
		if (memsize1 || memsize > 0x40000000) {
			mem_base0 = 0x0;
			mem_mask0 = (~((memsize0 >> 24) - 1) & ~(memsize >> 24)) & DDRC_MMAP_MASK_MASK;
			mem_base1 = (memsize1 >> 24) & 0xff;
			mem_mask1 = (~((memsize1 >> 24) - 1) & ~(memsize >> 24)) & DDRC_MMAP_MASK_MASK;
		} else {
			/* 1 ddr chip, capacity 1G.
			 * when only ddr chip select 0, mmap0 and mmap1 acting on both chip select 0 in jz4775.
			 */
			mem_base0 = 0x20;
			mem_base1 = 0x40; mem_mask0 = mem_mask1 = 0xe0;
		}
	} else {
		mem_base0 = (DDR_MEM_PHY_BASE >> 24) & 0xff;
		mem_mask0 = ~((memsize0 >> 24) - 1) & DDRC_MMAP_MASK_MASK;
		mem_base1 = ((DDR_MEM_PHY_BASE + memsize0) >> 24) & 0xff;
		mem_mask1 = ~((memsize1 >> 24) - 1) & DDRC_MMAP_MASK_MASK;
	}

	ddrc_mmap[0].mem_base = mem_base0;
	ddrc_mmap[0].mem_mask = mem_mask0;
	ddrc_mmap[1].mem_base = mem_base1;
	ddrc_mmap[1].mem_mask = mem_mask1;

	debug("mem_base0 = %x, mem_mask0 = %x\n", mem_base0, mem_mask0);
	debug("mem_base1 = %x, mem_mask1 = %x\n", mem_base1, mem_mask1);

	return 0;
}

int get_ddrc_mmap_data(struct ddrc_mmap ddrc_mmap[2], unsigned int ddrc_cfg_reg)
{
	return get_jz4775_ddrc_mmap_data(ddrc_mmap, ddrc_cfg_reg);
}

int get_ddrc_remap_registers(struct ddrc_remap_registers *premap)
{
	unsigned int start = 0;
	int num = 0;
	unsigned int rows = DDR_ROW;
	unsigned int cols = DDR_COL;
	unsigned int dw = DDR_DW32 ? 4 : 2;
	unsigned int banks = DDR_BANK8 ? 8 : 4;
	unsigned int cs0 = DDR_CS0EN ? 1 : 0;
	unsigned int cs1 = DDR_CS1EN ? 1 : 0;

	premap->ddrc_remap1_reg = 0x03020100;
	premap->ddrc_remap2_reg = 0x07060504;
	premap->ddrc_remap3_reg = 0x0b0a0908;
	premap->ddrc_remap4_reg = 0x0f0e0d0c;
	premap->ddrc_remap5_reg = 0x13121110;

	start += rows + cols + dw / 2;
	start -= 12;
	if (banks == 8)
		num += 3;
	else
		num += 2;
	if (cs0 && cs1)
		num++;
	debug("start = %d, num = %d\n", start, num);

	for (; num > 0; num--) {
		if ((start + num - 1) == 17)
			continue;
		int tmp1 = 0, tmp2 = 0;
		int a = num -1, b = start + num -1;

		tmp1 = (*((&premap->ddrc_remap1_reg) + a / 4) & REMAP_MASK(a)) >> REMAP_BIT(a);
		tmp2 = (*((&premap->ddrc_remap1_reg) + b / 4) & REMAP_MASK(b)) >> REMAP_BIT(b);

		*((&premap->ddrc_remap1_reg) + a / 4) &= ~REMAP_MASK(a);
		*((&premap->ddrc_remap1_reg) + b / 4) &= ~REMAP_MASK(b);

		*((&premap->ddrc_remap1_reg) + a / 4) |= tmp2 << REMAP_BIT(a);
		*((&premap->ddrc_remap1_reg) + b / 4) |= tmp1 << REMAP_BIT(b);

		debug("%d <==> %d\n", a, b);
		debug("REG_DDRC_REMMAP(%d) = 0x%08x\n", a / 4 + 1, *((&premap->ddrc_remap1_reg) + a / 4));
		debug("REG_DDRC_REMMAP(%d) = 0x%08x\n", b / 4 + 1, *((&premap->ddrc_remap1_reg) + b / 4));
	}

	return 0;

}

/* DDR sdram init */
void sdram_init(void)
{
	int count = 0, i = 0;
	struct ddrc_mmap ddrc_mmap[2];
	struct ddrc_remap_registers ddrc_remap_registers;

	* (volatile unsigned *) 0xb00000d0 = 0x3;
	mdelay(5);
	* (volatile unsigned *) 0xb00000d0 = 0x1;
	mdelay(5);

	REG_DDRC_CTRL = 0xf0 << 16;
	mdelay(5);
	REG_DDRC_CTRL = 0x0;
	mdelay(5);

	REG_DDRP_DTAR = 0x150000;
	REG_DDRP_DCR = DDRP_DCR_VALUE;
	REG_DDRP_MR0 = DDRP_MR0_VALUE;
	REG_DDRP_MR1 = DDRP_MR1_VALUE;
	REG_DDRP_MR2 = DDRP_MR2_VALUE;
	REG_DDRP_ODTCR = DDRP_ODTCR_VALUE;
	REG_DDRP_PTR0 = DDRP_PTR0_VALUE;
	REG_DDRP_PTR1 = DDRP_PTR1_VALUE;
	REG_DDRP_PTR2 = DDRP_PTR2_VALUE;
	REG_DDRP_DTPR0 = DDRP_DTPR0_VALUE;
	REG_DDRP_DTPR1 = DDRP_DTPR1_VALUE;
	REG_DDRP_DTPR2 = DDRP_DTPR2_VALUE;
	REG_DDRP_PGCR = DDRP_PGCR_VALUE;
	while (REG_DDRP_PGSR != (DDRP_PGSR_IDONE | DDRP_PGSR_DLDONE \
				| DDRP_PGSR_ZCDONE)) {
		if (REG_DDRP_PGSR == 0x1f)
			break;
		if (count++ == 10000) {
			debug("Init PHY: PHY INIT\n");
			debug("REG_DDP_PGSR: %x\n",REG_DDRP_PGSR);
			hang();
		}
	}
	REG_DDRP_PIR = DDRP_PIR_INIT | DDRP_PIR_DRAMINT | DDRP_PIR_DRAMRST \
		       | DDRP_PIR_DLLSRST;

	count = 0;
	while (REG_DDRP_PGSR != (DDRP_PGSR_IDONE | DDRP_PGSR_DLDONE \
				| DDRP_PGSR_ZCDONE | DDRP_PGSR_DIDONE)) {
		if (REG_DDRP_PGSR == 0x1f)
			break;
		if (count++ == 20000) {
			debug("Init PHY: DDR INIT DONE\n");
			debug("REG_DDP_PGSR: %x\n",REG_DDRP_PGSR);
			hang();
		}
	}

	count = 0;
	REG_DDRP_PIR = DDRP_PIR_INIT | DDRP_PIR_QSTRN;
	while (REG_DDRP_PGSR != (DDRP_PGSR_IDONE | DDRP_PGSR_DLDONE \
				| DDRP_PGSR_ZCDONE | DDRP_PGSR_DIDONE \
				| DDRP_PGSR_DTDONE)) {
		if (count++ ==500000) {
			debug("Init PHY: DDR TRAIN DONE\n");
			debug("REG_DDP_PGSR: %x\n",REG_DDRP_PGSR);
			for (i = 0; i < 4; i++) {
				debug("REG_DDP_GXnGSR: %x\n", \
						REG_DDRP_DXGSR0(i));
			}
			if (REG_DDRP_PGSR & (DDRP_PGSR_DTDONE \
						| DDRP_PGSR_DTERR \
						| DDRP_PGSR_DTIERR)) {
				debug("REG_DDP_PGSR: %x\n",REG_DDRP_PGSR);
				while (1) ;
			}
			count = 0;
		}
	}
	REG_DDRC_CTRL = 0x0;
	REG_DDRC_CTRL = DDRC_CTRL_CKE | DDRC_CTRL_ALH;
	REG_DDRC_CTRL = 0x0;
	REG_DDRC_CFG = DDRC_CFG_VALUE;

	REG_DDRC_TIMING(1) = DDRC_TIMING1_VALUE;
	REG_DDRC_TIMING(2) = DDRC_TIMING2_VALUE;
	REG_DDRC_TIMING(3) = DDRC_TIMING3_VALUE;
	REG_DDRC_TIMING(4) = DDRC_TIMING4_VALUE;
	REG_DDRC_TIMING(5) = DDRC_TIMING5_VALUE;
	REG_DDRC_TIMING(6) = DDRC_TIMING6_VALUE;
	memset(ddrc_mmap, 0, sizeof(struct ddrc_mmap) * 2);
	get_ddrc_mmap_data(ddrc_mmap, REG_DDRC_CFG);
	REG_DDRC_MMAP0 = ddrc_mmap[0].mem_base << DDRC_MMAP_BASE_BIT \
			 | ddrc_mmap[0].mem_mask;
	REG_DDRC_MMAP1 = ddrc_mmap[1].mem_base << DDRC_MMAP_BASE_BIT \
			 | ddrc_mmap[1].mem_mask;
	REG_DDRC_CTRL = DDRC_CTRL_CKE | DDRC_CTRL_ALH;
	REG_DDRC_REFCNT = DDRC_REFCNT_VALUE;
	REG_DDRC_CTRL = 0;
	REG_DDRC_ST &= ~0x40;
	get_ddrc_remap_registers(&ddrc_remap_registers);
	REG_DDRC_REMMAP(1) = ddrc_remap_registers.ddrc_remap1_reg;
	REG_DDRC_REMMAP(2) = ddrc_remap_registers.ddrc_remap2_reg;
	REG_DDRC_REMMAP(3) = ddrc_remap_registers.ddrc_remap3_reg;
	REG_DDRC_REMMAP(4) = ddrc_remap_registers.ddrc_remap4_reg;
	REG_DDRC_REMMAP(5) = ddrc_remap_registers.ddrc_remap5_reg;
#ifdef DUMP_DDR
	dump_ddrc_register();
	dump_ddrp_register();
#endif
}

uint32_t sdram_size(int cs)
{
	uint32_t dw, banks, size = 0;

	dw = DDR_DW32 ? 4 : 2;
	banks = DDR_BANK8 ? 8 : 4;

	if ((cs == 0) && DDR_CS0EN) {
		size = (1 << (DDR_ROW + DDR_COL)) * dw * banks;
		if (DDR_CS1EN && (size > 0x20000000))
			size = 0x20000000;
	} else if ((cs == 1) && DDR_CS1EN) {
		size = (1 << (DDR_ROW + DDR_COL)) * dw * banks;
	}

	return size;
}
