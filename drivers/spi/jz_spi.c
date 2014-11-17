/*
 * Ingenic JZ SPI driver
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: Tiger <lbh@ingenic.cn>
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
#include <spi.h>
#include <spi_flash.h>
#include <malloc.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/cpm.h>
#include <asm/arch/spi.h>
#include <asm/arch/clk.h>
#include <asm/arch/base.h>

#define SSI_BASE CONFIG_SSI_BASE

static uint32_t jz_spi_readl(unsigned int offset)
{
	return readl(SSI_BASE + offset);
}

static void jz_spi_writel(unsigned int value, unsigned int offset)
{
	writel(value, SSI_BASE + offset);
}

static void jz_spi_flush(void )
{
	jz_spi_writel(jz_spi_readl(SSI_CR0) | SSI_CR0_TFLUSH | SSI_CR0_RFLUSH, SSI_CR0);
}

static unsigned int spi_rxfifo_empty(void )
{
	return (jz_spi_readl(SSI_SR) & SSI_SR_RFE);
}

static unsigned int spi_txfifo_full(void )
{
	return (jz_spi_readl(SSI_SR) & SSI_SR_TFF);
}

static unsigned int spi_get_rxfifo_count(void )
{
	return ((jz_spi_readl(SSI_SR) & SSI_SR_RFIFONUM_MASK) >> SSI_SR_RFIFONUM_BIT);
}

#ifdef CONFIG_SOFT_SPI
int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	gpio_direction_input(GPIO_PE(20));	//spi-dr
	return 1;
}

void spi_cs_activate(struct spi_slave *slave)
{
	gpio_direction_output(GPIO_PE(29), 0); //cs
}

void spi_cs_deactivate(struct spi_slave *slave)
{
	gpio_direction_output(GPIO_PE(29), 1); //cs
}
#endif

void spi_init(void )
{
#if DEBUG
	unsigned int errorpc;
		__asm__ __volatile__ (
				"mfc0  %0, $30,  0   \n\t"
				"nop                  \n\t"
				:"=r"(errorpc)
				:);

		printf("RESET ERROR PC:%x\n",errorpc);
#endif
	unsigned int ssi_rate = 24000000;
	clk_set_rate(SSI, ssi_rate);
	jz_spi_writel(~SSI_CR0_SSIE & jz_spi_readl(SSI_CR0), SSI_CR0);
	jz_spi_writel(0, SSI_GR);
	jz_spi_writel(SSI_CR0_EACLRUN | SSI_CR0_RFLUSH | SSI_CR0_TFLUSH, SSI_CR0);
	jz_spi_writel(SSI_FRMHL_CE0_LOW_CE1_LOW | SSI_GPCMD | SSI_GPCHL_HIGH | SSI_CR1_TFVCK_3 | SSI_CR1_TCKFI_3 | SSI_CR1_FLEN_8BIT | SSI_CR1_PHA | SSI_CR1_POL, SSI_CR1);
	jz_spi_writel(SSI_CR0_SSIE | jz_spi_readl(SSI_CR0), SSI_CR0);
}

void spi_send_cmd(unsigned char *cmd, unsigned int count)
{
	unsigned int sum = count;
	jz_spi_flush();
	while(!spi_rxfifo_empty());
	while(count) {
		jz_spi_writel(*cmd, SSI_DR);
		while (spi_txfifo_full());
		cmd++;
		count--;
	}
	while (spi_get_rxfifo_count() != sum);
}

void spi_recv_cmd(unsigned char *read_buf, unsigned int count)
{
	unsigned int offset = 0;
	jz_spi_flush();
	while(!spi_rxfifo_empty());
	while (count) {
		jz_spi_writel(0, SSI_DR);
		while (spi_rxfifo_empty())
			;
		writeb(jz_spi_readl(SSI_DR), read_buf + offset);
		offset++;
		count--;
	}
}

void spi_load(unsigned int src_addr, unsigned int count, unsigned int dst_addr)
{
	unsigned char cmd;
	cmd = CMD_READ;
	src_addr = ((src_addr & 0xFF) << 16) | (src_addr & 0x0000FF00) | ((src_addr >> 16) & 0xFF);
	spi_init();
	jz_spi_writel(jz_spi_readl(SSI_CR1) | SSI_CR1_UNFIN, SSI_CR1);
	spi_send_cmd(&cmd, 1);
	spi_send_cmd((unsigned char *)&src_addr, 3);
	spi_recv_cmd((unsigned char *)dst_addr, count);
	jz_spi_writel(jz_spi_readl(SSI_CR1) & (~SSI_CR1_UNFIN), SSI_CR1);
}

struct jz_spi_slave {
	struct spi_slave slave;
	unsigned int mode;
	unsigned int max_hz;
};

static inline struct jz_spi_slave *to_jz_spi(struct spi_slave *slave)
{
	return container_of(slave, struct jz_spi_slave, slave);
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int mode)
{
	spi_init();
	struct jz_spi_slave *ss;

	ss = spi_alloc_slave(struct jz_spi_slave, bus, cs);
	if (!ss)
		return NULL;

	ss->mode = mode;
	ss->max_hz = max_hz;

	return &ss->slave;
}

void spi_free_slave(struct spi_slave *slave)
{
	struct soft_spi_slave *ss = to_jz_spi(slave);

	free(ss);
}

int spi_claim_bus(struct spi_slave *slave)
{
	jz_spi_writel(jz_spi_readl(SSI_CR1) | SSI_CR1_UNFIN, SSI_CR1);
	return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
	jz_spi_writel(jz_spi_readl(SSI_CR1) & (~SSI_CR1_UNFIN), SSI_CR1);
	jz_spi_writel(jz_spi_readl(SSI_SR) & (~SSI_SR_UNDR) , SSI_SR);
}

int  spi_xfer(struct spi_slave *slave, unsigned int bitlen,
		const void *dout, void *din, unsigned long flags)
{
	unsigned int count = bitlen / 8;
	unsigned char *cmd = dout;
	unsigned char *addr = din;
	unsigned int fifo_size = 64;

	if(dout != NULL) {
		while(count) {
			if(count > fifo_size) {
				spi_send_cmd(cmd, fifo_size);
				cmd += fifo_size;
				count -= fifo_size;
			} else {
				spi_send_cmd(cmd, count);
				break;
			}
		}
	}

	if(din != NULL) {
		spi_recv_cmd((unsigned char *)addr, count);
	}

	return 0;
}

static void jz_cs_reversal(void )
{
	spi_release_bus(NULL);

	udelay(1000);

	spi_claim_bus(NULL);

	return ;
}

int jz_read(struct spi_flash *flash, u32 offset, size_t len, void *data)
{
	unsigned char cmd[5];
	unsigned long remain_len, read_len;

	cmd[0] = CMD_FAST_READ;
	cmd[1] = offset >> 16;
	cmd[2] = offset >> 8;
	cmd[3] = offset >> 0;
	cmd[4] = 0x00;

	read_len = flash->size - offset;

	if(len < read_len)
		read_len = len;

	jz_cs_reversal();
	spi_send_cmd(&cmd[0], 5);
	spi_recv_cmd(data, read_len);

	return 0;
}

int jz_write(struct spi_flash *flash, u32 offset, size_t len, const void *buf)
{
	unsigned char cmd[6], tmp;
	int chunk_len, actual, i;
	unsigned long byte_addr, page_size;

	page_size = flash->page_size;

	cmd[0] = CMD_WREN;

	cmd[1] = CMD_PP;

	cmd[5] = CMD_RDSR;

	for (actual = 0; actual < len; actual += chunk_len) {
		byte_addr = offset % page_size;
		chunk_len = min(len - actual, page_size - byte_addr);

		cmd[2] = offset >> 16;
		cmd[3] = offset >> 8;
		cmd[4] = offset >> 0;

		jz_cs_reversal();
		spi_send_cmd(&cmd[0], 1);

		jz_cs_reversal();
		spi_send_cmd(&cmd[1], 4);
		for(i = 0; i < chunk_len; i += 100) {
			if((chunk_len - i) < 100)
				spi_send_cmd((buf + actual + i), (chunk_len - i));
			else
				spi_send_cmd((buf + actual + i), 100);

		}

		jz_cs_reversal();
		spi_send_cmd(&cmd[5], 1);
		spi_recv_cmd(&tmp, 1);
		while(tmp & CMD_SR_WIP) {
			spi_recv_cmd(&tmp, 1);
		}

		offset += chunk_len;
	}

	return 0;
}

int jz_erase(struct spi_flash *flash, u32 offset, size_t len)
{
	unsigned long erase_size;
	unsigned char cmd[6], buf;

	erase_size = flash->sector_size;
	if (offset % erase_size || len % erase_size) {
		printf("Erase offset/length not multiple of erase size\n");
		return -1;
	}

	cmd[0] = CMD_WREN;

	switch(erase_size) {
	case 0x1000 :
		cmd[1] = CMD_ERASE_4K;
		break;
	case 0x8000 :
		cmd[1] = CMD_ERASE_32K;
		break;
	case 0x10000 :
		cmd[1] = CMD_ERASE_64K;
		break;
	default:
		printf("unknown erase size !\n");
		return -1;
	}

	cmd[5] = CMD_RDSR;

	while(len) {
		cmd[2] = offset >> 16;
		cmd[3] = offset >> 8;
		cmd[4] = offset >> 0;

		printf("erase %x %x %x %x %x %x %x \n", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], offset);

		jz_cs_reversal();
		spi_send_cmd(&cmd[0], 1);

		jz_cs_reversal();
		spi_send_cmd(&cmd[1], 4);

		jz_cs_reversal();
		spi_send_cmd(&cmd[5], 1);
		spi_recv_cmd(&buf, 1);
		while(buf & CMD_SR_WIP) {
			spi_recv_cmd(&buf, 1);
		}

		offset += erase_size;
		len -= erase_size;
	}

	return 0;
}
#ifdef CONFIG_SPI_FLASH_INGENIC
#define SIZEOF_NAME	32

struct jz_spi_support {
	u8 id;
	char name[SIZEOF_NAME];
	int page_size;
	int sector_size;
	int size;
};

static struct jz_spi_support jz_spi_support_table[] = {
	{
		.id = 0xc2,
		.name = "MX25L12835F",
		.page_size = 256,
		.sector_size = 4 * 1024,
		.size = 16 * 1024 * 1024,
	},
};

struct spi_flash *spi_flash_probe_ingenic(struct spi_slave *spi, u8 *idcode)
{
	int i;
	struct spi_flash *flash;
	struct jz_spi_support *params;

	for (i = 0; i < ARRAY_SIZE(jz_spi_support_table); i++) {
		params = &jz_spi_support_table[i];
		if (params->id == idcode[0])
			break;
	}

	if (i == ARRAY_SIZE(jz_spi_support_table)) {
		printf("ingenic: Unsupported ID %04x\n", idcode[0]);
		return NULL;
	}

	flash = spi_flash_alloc_base(spi, params->name);
	if (!flash) {
		printf("ingenic: Failed to allocate memory\n");
		return NULL;
	}
	flash->erase = jz_erase;
	flash->write = jz_write;
	flash->read  = jz_read;

	flash->page_size = params->page_size;
	flash->sector_size = params->sector_size;
	flash->size = params->size;

	return flash;
}
#endif
#ifdef CONFIG_SPL_SPI_SUPPORT
void spl_spi_load_image(void)
{
	struct image_header *header;

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE);

	spl_parse_image_header(header);

	spi_load(CONFIG_UBOOT_OFFSET, CONFIG_SYS_MONITOR_LEN, CONFIG_SYS_TEXT_BASE);
}
#endif
