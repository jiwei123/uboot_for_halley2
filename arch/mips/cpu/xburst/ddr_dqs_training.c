/*
 * DDR DQS soft training driver for Synopsys DWC DDR PHY.
 * Used by Jz4775, JZ4780...
 *
 * Copyright (C) 2014 Ingenic Semiconductor Co.,Ltd
 * Author:
 *	Uboot: Sun Jiwei <jwsun@ingenic.cn>
 *	Source code:	Jiang Tao <tjiang@ingenic.cn>
 *			Liang Bohu <lbh@ingenic.cn>
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
#include <ddr/ddr_common.h>
#include <generated/ddr_reg_values.h>

#include <asm/io.h>

#define MASK(byte) (0xff << (byte * 8))
#define EXPECT0 0x11223344
#define EXPECT1 0x55667788
#define EXPECT2 0xaabbccdd
#define EXPECT3 0xeeeeffff

union ddr_cmd {
	unsigned int word[4];
	struct {
		unsigned int data2;
		unsigned int data1;
		unsigned mask:8;
		unsigned addr:15;
		unsigned bank:3;
		unsigned rank:1;
		unsigned cmd:4;
		unsigned tag:1;
		unsigned tag_bak:1;
		unsigned dtp:5;
		unsigned rpt:5;
	}cmd;
};

static void write_dcu_cache(unsigned int *data, unsigned int len)
{
	int i;

	for (i = 0; i < len; i++) {
		debug("send: 0x%08x\n", data[i]);
		ddr_writel(data[i], DDRP_DCUDR);
	}
	debug("\n");
}

#define DCUAR_VALUE 	0x400
#define DCURR_VALUE		0x101
#define RDONE	0x1
void send_MR0(int a)
{
	union ddr_cmd ddr_send;
	int i;

	for(i = 0; i < 4; i++) {
		ddr_send.word[i] = 0;
	}

	ddr_writel(DCUAR_VALUE, DDRP_DCUAR);

	debug("PRECHARGE ALL:\n");
	ddr_send.cmd.dtp = 19;	/* tRPA */
	ddr_send.cmd.cmd = 5; 	/* RP_ALL */
	ddr_send.cmd.tag = 1;
	write_dcu_cache(ddr_send.word, 4);

	debug("LOAD MODE MR0:\n");
	ddr_send.cmd.tag = 1;
	ddr_send.cmd.dtp = 5;	/* tMOD */
	ddr_send.cmd.cmd = 1; 	/* LOAD_MODE mr0 */
	ddr_send.cmd.addr = a;	/* cmd.addr = 0x220; dll-off request, CL=6 */
	write_dcu_cache(ddr_send.word, 4);

	ddr_writel(DCURR_VALUE, DDRP_DCURR);

	while (!(ddr_readl(DDRP_DCUSR0) & RDONE));
}

static void ddr_test_write(int rank)
{
	union ddr_cmd ddr_send;
	int i;

	for(i = 0; i < 4; i++) {
		ddr_send.word[i] = 0;
	}

	ddr_writel(0x3, DDRP_DCUTPR);
	ddr_writel(0x400, DDRP_DCUAR);

	debug("PRECHARGE ALL:\n");
	ddr_send.cmd.tag = 1;
	ddr_send.cmd.dtp = 19;	//tRPA;
	ddr_send.cmd.cmd = 5; 	//RP_ALL;
	ddr_send.cmd.rank = rank;
	write_dcu_cache(ddr_send.word, 4);

	debug("ACT:\n");
	ddr_send.cmd.tag = 0;
	ddr_send.cmd.dtp = 21;	//TACT2RW;
	ddr_send.cmd.cmd = 6; 	//ACT;
	write_dcu_cache(ddr_send.word, 4);

	debug("WRITE:\n");
	ddr_send.cmd.rpt = 4;	//tBL;
	ddr_send.cmd.dtp = 25;	//TWR2RD;
	ddr_send.cmd.cmd = 8;	//WRITE;
	ddr_send.cmd.data1 = EXPECT0;
	ddr_send.cmd.data2 = EXPECT1;
	write_dcu_cache(ddr_send.word, 4);

	ddr_send.cmd.addr = 0x20;
	ddr_send.cmd.data1 = EXPECT2;
	ddr_send.cmd.data2 = EXPECT3;
	write_dcu_cache(ddr_send.word, 4);

	debug("READ:\n");
	ddr_send.cmd.rpt = 4;	//tBL;
	ddr_send.cmd.dtp = 24;	//tRD2WR;
	ddr_send.cmd.cmd = 10;	//READ;
	ddr_send.cmd.addr = 0;
	ddr_send.cmd.data1 = 0;
	ddr_send.cmd.data2 = 0;
	write_dcu_cache(ddr_send.word, 4);

	ddr_send.cmd.addr = 0x20;
	write_dcu_cache(ddr_send.word, 4);

	ddr_writel(0x301, DDRP_DCURR);

	while (!(ddr_readl(DDRP_DCUSR0) & 0x1));

	debug("dcu command stag1 over!\n");
	debug("DCUAR: 0x%08x\n", ddr_readl(DDRP_DCUAR));
	debug("DCURR: 0x%08x\n", ddr_readl(DDRP_DCURR));
	debug("DCUSR0: 0x%08x\n", ddr_readl(DDRP_DCUSR0));
	debug("DCUSR1: 0x%08x\n", ddr_readl(DDRP_DCUSR1));
}

static int ddr_test_read(int byte)
{
	unsigned int i, data, expect, res = 1;

	ddr_writel(0x00400501, DDRP_DCURR);

	while (!(ddr_readl(DDRP_DCUSR0) & 0x1));

	debug("dcu command stag2 over!\n");
	debug("DCUAR: 0x%08x\n", ddr_readl(DDRP_DCUAR));
	debug("DCURR: 0x%08x\n", ddr_readl(DDRP_DCURR));
	debug("DCUSR0: 0x%08x\n", ddr_readl(DDRP_DCUSR0));
	debug("DCUSR1: 0x%08x\n", ddr_readl(DDRP_DCUSR1));

	ddr_writel(0xe00, DDRP_DCUAR);

	for (i = 0; i < 16; i++) {
		data = ddr_readl(DDRP_DCUDR);
		if (i < 8) {
			if (i % 2)
				expect = EXPECT0;
			else
				expect = EXPECT1;
		} else {
			if (i % 2)
				expect = EXPECT2;
			else
				expect = EXPECT3;
		}
		if ((data & MASK(byte)) != (expect & MASK(byte))) {
			debug("\tERROR DCUDR: 0x%08x\n", data);
			debug("\tEXPECT DCUDR: 0x%08x\n", expect);
			debug("\tMASK: 0x%08x\n", MASK(byte));
			res = 0;
			break;
		} else {
			debug("\tOK DCUDR: 0x%08x\n", data & MASK(byte));
		}
	}
	ddr_writel(0x20011, DDRP_PIR);
	while ((ddr_readl(DDRP_PGSR) & 0xf) != 0xf);

	debug("ITM Reset over\n");
	return res;
}

static int tunning_dqs_parameter(int rank, int byte)
{
	int dgps, dgsl;
	unsigned int start, end;
	unsigned int tmp;

	start = end = 0xffff;
	for (dgsl = 0; dgsl < 8; dgsl++) {
		for (dgps = 0; dgps < 4; dgps++) {
			debug("\tdgsl: %d\n", dgsl);
			debug("\tdgps: %d\n", dgps);

			tmp = ddr_readl(DDRP_DXDQSTR(byte));
			tmp &= ~((0x7 << (rank * 3)) | (0x3 << (rank * 2 + 12)));
			ddr_writel(tmp, DDRP_DXDQSTR(byte));

			tmp = ddr_readl(DDRP_DXDQSTR(byte));
			tmp |= (dgsl << (rank * 3) | (dgps << (rank * 2 + 12)));
			ddr_writel(tmp, DDRP_DXDQSTR(byte));

			debug("ddr_test_read REG_DDRP_DXDQSTR(%d): 0x%08x\n",
					byte, ddr_readl(DDRP_DXDQSTR(byte)));

			if (ddr_test_read(byte)) {
				debug("ddr_test_read ok!!!\n");
				if (start == 0xffff)
					start = dgps + dgsl * 4;
				end = dgps + dgsl * 4;
				debug("start: %d\n", start);
				debug("end: %d\n", end);
			} else {
				debug("ddr_test_read error!!!\n");
				if (end - start > 0) {
					debug("!!!!!!found!\n");
					return (start + end + 1) / 2;
				} else
					end = start = 0xffff;
			}
		}
	}

	if (end - start > 0) {
		debug("!!!!!!found!\n");
		return (start + end + 1) / 2;
	} else {
		debug("!!!!!! NO found!\n");
		return 0xffffffff;
	}
}

int dqs_gate_train(int rank_cnt, int byte_cnt)
{
	unsigned int rank, byte, middle = 0, res = 0;
	u32 tmp, i;

	tmp = ddr_readl(DDRP_PGCR);
	tmp &= ~DDRP_PGCR_DFTCMP;
	ddr_writel(tmp, DDRP_PGCR);

	for (rank = 0; rank < rank_cnt; rank++ ) {
		debug("########RANK: %d\n", rank);
		ddr_test_write(rank);
		for (byte = 0; byte < byte_cnt; byte++) {
			debug("########BYTE: %d\n", byte);
			for (i = 0; i < 4; i++) {
				tmp = ddr_readl(DDRP_DXGCR(i));
				tmp &= ~DDRP_DXGCR_DXEN;
				ddr_writel(tmp, DDRP_DXGCR(i));
			}

			tmp = ddr_readl(DDRP_DXGCR(byte));
			tmp |= DDRP_DXGCR_DXEN;
			ddr_writel(tmp, DDRP_DXGCR(byte));

			middle = tunning_dqs_parameter(rank, byte);
			if (middle != 0xffffffff) {
				tmp = ddr_readl(DDRP_DXDQSTR(byte));
				tmp &= ~((0x7 << (rank * 3)) | (0x3 << (rank * 2 + 12)));
				ddr_writel(tmp, DDRP_DXDQSTR(byte));

				tmp = ddr_readl(DDRP_DXDQSTR(byte));
				tmp |= ((middle / 4) << (rank * 3) | ((middle % 4) << (rank * 2 + 12)));
				ddr_writel(tmp, DDRP_DXDQSTR(byte));

				debug("Middle: %d\n", middle);
				debug("Rank:%d\n", rank);
				debug("Byte:%d\n", byte);
				debug("@pas:DXDQSTR= 0x%x\n", ddr_readl(DDRP_DXDQSTR(byte)));
			} else {
				debug("Rank:%d\n", rank);
				printf("Rank:%d\n", rank);
				debug("No pass at byte:%d\n", byte);
				printf("No pass at byte:%d\n", byte);
				res = 1;
			}
		}
	}

	debug("Dummy read: ");
	ddr_test_read(0);
	for (i = 0; i < 4; i++) {
		tmp = ddr_readl(DDRP_DXGCR(i));
		tmp |= DDRP_DXGCR_DXEN;
		ddr_writel(tmp, DDRP_DXGCR(i));
	}
	tmp = ddr_readl(DDRP_PGCR);
	tmp |= DDRP_PGCR_DFTCMP;
	ddr_writel(tmp, DDRP_PGCR);

	debug("soft training over\n");
	return res;
}
