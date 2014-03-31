/*
 * Copyright (C) 2007 Ingenic Semiconductor Inc.
 * Author: Regen Huang <lhhuang@ingenic.cn>
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


//#include <ingenic_nand_mgr/nandflash.h>
#include <mipsregs.h>
#include <ingenic_nand_mgr/jz4775.h>
#include <ingenic_nand_mgr/nand_param.h>
#include "nandflash.h"
#include "jz4780misc.h"
#include "hand.h"
#include "convert_img.h"
#include "nand_chip.h"

#define USE_BCH 1
#define pn_enable() \
do { \
	REG_NEMC_PNCR = 0x3; \
} while (0)
#define pn_disable() \
do { \
	REG_NEMC_PNCR = 0x0; \
} while (0)

/*
 * NAND flash definitions
 */
#define NAND_DATAPORT	0xBB000000
#define NAND_ADDRPORT   0xBB800000
#define NAND_COMMPORT   0xBB400000

#define MARK_ERASE_OOB	64  /* in nand_mark_erase_4780 for format u-disk fs zone; */
#define ECC_BLOCK	1024
#define ECC_BLOCK_512 512
#define MAX_FREE_SIZE	1024
/* for spl */
#define SPL_SIZE	(16 * 1024)
#define ECC_BIT_SPL	64
#define ECC_BLOCK_SPL   256
#define ECC_PAR_SPL	(64 * 14 / 8)

#define CMD_READA	0x00
#define CMD_READB	0x01
#define CMD_READC	0x50
#define CMD_ERASE_SETUP	0x60
#define CMD_ERASE	0xD0
#define CMD_READ_STATUS 0x70
#define CMD_CONFIRM	0x30
#define CMD_SEQIN	0x80
#define CMD_PGPROG	0x10
#define CMD_READID	0x90
#define CMD_RESET	0xFF

#define __nand_cmd(n)		(REG8(NAND_COMMPORT) = (n))
#define __nand_addr(n)		(REG8(NAND_ADDRPORT) = (n))
#define __nand_data8()		REG8(NAND_DATAPORT)
#define __nand_data16()		REG16(NAND_DATAPORT)

//#define DEBUG
#ifdef DEBUG
#define dprint1(n,x...) printf(n, ##x)
#else
#define dprint1(n,x...)
#endif
//#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
#define dprint2(n,x...) printf(n, ##x)
#else
#define dprint2(n,x...)
#endif

static int bus = 8;
static int eccpos = 0;
static int ppb = 1024;
static int burn_mode = ZONE_MANGER_BURN ;
static int flash_type = TYPE_COMMON; /* indicate either common_nand or toggle_nand  */

static int tprobe_done;
static int nm_init = 1; /*add for make sure nandmanager init onece*/

#define __raw_readb(addr) (*(volatile unsigned char *)(addr))
#define __raw_readw(addr) (*(volatile unsigned short *)(addr))
#define __raw_readl(addr) (*(volatile unsigned int *)(addr))
#define readb(addr) __raw_readb((addr))
#define readw(addr) __ioswab16(__raw_readw((addr)))
#define readl(addr) __ioswab32(__raw_readl((addr)))

#define __raw_writeb(b, addr) (*(volatile unsigned char *)(addr)) = (b)
#define __raw_writew(b, addr) (*(volatile unsigned short *)(addr)) = (b)
#define __raw_writel(b, addr) (*(volatile unsigned int *)(addr)) = (b)
#define writeb(b, addr) __raw_writeb((b), (addr))
#define writew(b, addr) __raw_writew(__ioswab16(b), (addr))
#define writel(b, addr) __raw_writel(__ioswab32(b), (addr))

#define NEMC_NFCSR_NFCES(n)  (1 << (((n) << 1) + 1))  /* NAND Flash CSn Enable, 0 ~ 5 */
#define NEMC_NFCSR_NFES(n)   (1 << ((n) << 1))    /* NAND Flash CSn FCE# Assertion Enable, 0 ~ 5 */
#define NEMC_NFCSR_NFCECC(n) (0 << (((n) << 1) + 1))  /* NAND Flash CSn Enable, 0 ~ 5 */
#define NEMC_NFCSR_NFECC(n)  (0 << ((n) << 1))    /* NAND Flash CSn FCE# Assertion Enable, 0 ~ 5 */



/*
 * External routines
 */
extern void *memset(void *s, int c, size_t count);
extern void *memcpy(void *, const void *, size_t);
extern void flush_cache_all(void);
extern int serial_init(void);
extern void sdram_init(void);
extern void pll_init(void);
extern void serial_putc (const char c);
extern void serial_puts(const char *s);
extern void printf(char *fmt, ...);

extern int isnotread; /*add for system partition reread*/

volatile u32 JZ_EXTAL = 24000000;

static void nand_enable(int cs)
{

	udelay(1); 
	REG32(NEMC_NFCSR) = (NEMC_NFCSR_NFES(0) | NEMC_NFCSR_NFCES(0));
	udelay(1); 

}
static void nand_reset()
{
	__nand_cmd(CMD_RESET);
}
static void nand_send_addr(int addr, unsigned int cycle, unsigned int delay)
{   
	while(cycle--){
		__nand_addr((addr & 0xff));
		addr = addr >> 0x08;
	}
	udelay(delay);
}       
static void nand_disable(int cs)
{
	udelay(1); // 300ns
	REG32(NEMC_NFCSR) = (NEMC_NFCSR_NFECC(cs) | NEMC_NFCSR_NFCECC(cs));
}
static void wait_rb(int rb)
{
	int cnt = 100000;
	int gpio = rb / 32;
	int gpio_bit = rb % 32;
	int mask_rb = (1 << gpio_bit);//papin

	while(((REG32(GPIO_BASE + gpio * 0x100) & mask_rb) == mask_rb) && cnt--);
	if(cnt < 0){
		printf("^^^^^^^  wait rb timeout !!! \n");
	}
	while((REG32(GPIO_BASE + gpio * 0x100 ) & mask_rb) != mask_rb);

}
static int try_to_get_nand_id(int rb_gpio,nand_flash_id *fid)
{
	unsigned char nand_id[6];
	unsigned int i;

	nand_enable(1);

	nand_reset();
	wait_rb(rb_gpio);
	mdelay(1);

	__nand_cmd(CMD_READID);
	udelay(1);

	nand_send_addr(0x00, 1, 1 * 1000);

	for(i=0; i < 5; i++){
		nand_id[i] = __nand_data8();
	}
	nand_disable(0);

	fid->id = ((nand_id[0] << 8) | nand_id[1]);
	fid->extid = ((nand_id[4] << 16) | (nand_id[3] << 8) | nand_id[2]);

}
void dump_ptinfo(PartitionInfo *ptinfo)
{
	int i,part_num;
	printf("%s dump rbinfo rbcount[%d]----->\n",__func__,ptinfo->rbcount);
	for(i=0; i<ptinfo->rbcount; i++){
		printf("rb[%d] = %d\n",i,ptinfo->rb_gpio[i]);
		printf("rb_pulldown_strength[%d] = %d\n",i,ptinfo->rb_pulldown_strength[i]);
	}
	printf("dump ptinfo ptcount[%d]----->\n",ptinfo->ptcount);
	for(i=0; i<ptinfo->ptcount; i++){
		part_num = 0;
		printf("pt[%s]:\n",(ptinfo->ndppt + i)->name);
		printf("\t offset[%dM]:\n",(ptinfo->ndppt + i)->offset);
		printf("\t size[%dM]:\n",(ptinfo->ndppt + i)->size);
		printf("\t managermode[%d]:\n",(ptinfo->ndppt + i)->managermode);
		printf("\t cache[%d]:\n",(ptinfo->ndppt + i)->cache);
		while((ptinfo->ndppt + i)->ui_ex_partition[part_num].size != 0){
			printf("\tpt[%s]:\n",(ptinfo->ndppt + i)->ui_ex_partition[part_num].name);
			printf("\toffset[%dM]:\n",(ptinfo->ndppt + i)->ui_ex_partition[part_num].offset);
			printf("\tsize[%dM]:\n",(ptinfo->ndppt + i)->ui_ex_partition[part_num].size);
			part_num++;
		}
	}
	printf("wp_gpio= %d\n",ptinfo->gpio_wp);
	printf("nand_driver_strength= %d\n",ptinfo->nand_driver_strength);
	printf("%s end ============>\n",__func__);

}
#define EXTID_MARK 0x00ffffff
static nand_flash *get_nand_info_from_table(nand_flash_id *fid,nand_flash *nand_info_table,int total_nand,nand_flash *nand_info)
{
	int index;

	for (index = 0; index < total_nand; index++) {
		if ((fid->id == nand_info_table[index].id) && ((fid->extid & EXTID_MARK) == (nand_info_table[index].extid & EXTID_MARK)))
		{
			memcpy(nand_info,&nand_info_table[index],sizeof(nand_flash));
			break;
		}
	}

}
int nand_init_4775(PartitionInfo *pt_info,nand_flash *total_nand_info,int total_nand,int nand_erase_mod)
{
	nand_flash_id fid;
	nand_flash nand_info;

	if (bus != 8 && bus != 16) {
		tprobe_done = 0;
		bus = 8;
	}

	/* Initialize NAND Flash Pins */
	if (bus == 8) {
		//REG_NEMC_SMCR1 = 0x13444400;
		if(burn_mode != ZONE_MANGER_BURN)
			REG_NEMC_SMCR1 = 0x0FFF7700;
		__gpio_as_nand_8bit(1);
		if(burn_mode == ZONE_MANGER_BURN)
			__gpio_as_nand_rbintc();
		else
			__gpio_as_nand_rbinput();
		__gpio_as_nand_cs(3);
	} else {
		if(burn_mode != ZONE_MANGER_BURN)
			REG_NEMC_SMCR1 = 0x0FFF7700 | 0x40;
		__gpio_as_nand_16bit(1);
		if(burn_mode == ZONE_MANGER_BURN)
			__gpio_as_nand_rbintc();
		else
			__gpio_as_nand_rbinput();
	}
	
	try_to_get_nand_id(pt_info->rb_gpio[0],&fid);

	get_nand_info_from_table(&fid,total_nand_info,total_nand,&nand_info);
	if(nand_info.buswidth != 8)
	{
		if(burn_mode != ZONE_MANGER_BURN)
			REG_NEMC_SMCR1 = 0x0FFF7700 | 0x40;
		__gpio_as_nand_16bit(1);
		if(burn_mode == ZONE_MANGER_BURN)
			__gpio_as_nand_rbintc();
		else
			__gpio_as_nand_rbinput();

	}
	
	/* Initialize Toggle NAND DQS */
	if (flash_type == TYPE_TOGGLE && tprobe_done == 0) {
		serial_puts("\nToggle NAND DQS Init ...");
		REG_NEMC_TGCR1 = 0x13333407;
		__gpio_as_nand_toggle();
		__tnand_dqsdelay_probe();

		if (__tnand_dqsdelay_checkerr())
			serial_puts(" failed.\n");
		else
			serial_puts(" OK.\n");

		serial_puts("NEMC_TGDR: ");
		//serial_put_hex(REG_NEMC_TGDR);
		tprobe_done = 1;
	}

	/* If ECCPOS isn't configured in config file, the initial value is 0 */
	if (eccpos == 0) {
		eccpos = 3;
	}

	if(burn_mode == ZONE_MANGER_BURN){
		if(ppb!=0 && ppb != 512 && nm_init){
			extern void fill_nand_flash_info(nand_flash *nand_info);
			fill_nand_flash_info(&nand_info);
			burn_nandmanager_init(pt_info,nand_erase_mod);
			nm_init = 0;
		}
	}

#ifdef USE_BCH
	serial_puts("Use bch.\n");
#else
	serial_puts("Not use bch.\n");
#endif
	if (burn_mode == ZONE_MANGER_BURN)
		serial_puts("Use PN.\n");
	else
		serial_puts("Not use PN.\n");

	printf("%s ok!!!!!\n",__func__);
	return 0;
}


