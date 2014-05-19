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

static void jz_spi_flush()
{
	jz_spi_writel(jz_spi_readl(SSI_CR0) | SSI_CR0_TFLUSH | SSI_CR0_RFLUSH, SSI_CR0);
}

static unsigned int spi_rxfifo_empty()
{
	return (jz_spi_readl(SSI_SR) & SSI_SR_RFE);
}

static unsigned int spi_txfifo_full()
{
	return (jz_spi_readl(SSI_SR) & SSI_SR_TFF);
}

static unsigned int spi_get_rxfifo_count()
{
	return ((jz_spi_readl(SSI_SR) & SSI_SR_RFIFONUM_MASK) >> SSI_SR_RFIFONUM_BIT);
}


void spi_init()
{
	jz_spi_writel(~SSI_CR0_SSIE & jz_spi_readl(SSI_CR0), SSI_CR0);
	jz_spi_writel(0, SSI_GR);
	jz_spi_writel(SSI_CR0_EACLRUN | SSI_CR0_RFLUSH | SSI_CR0_TFLUSH, SSI_CR0);
	jz_spi_writel(SSI_CR1_TFVCK_3 | SSI_CR1_TCKFI_3 | SSI_CR1_FLEN_8BIT | SSI_CR1_PHA | SSI_CR1_POL, SSI_CR1);
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
		while (spi_rxfifo_empty());
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

	if (!spi_cs_is_valid(bus, cs))
		return NULL;

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

struct spi_flash *spi_flash_probe_ingenic(struct spi_slave *spi, u8 *idcode)
{
	struct spi_flash *flash;

	flash = spi_flash_alloc_base(spi, "ingenic");
	if (!flash) {
		printf("%s Failed to allocate memory\n", __FILE__);
		return NULL;
	}

	flash->page_size = 256;
	flash->sector_size = 4 * 1024;
	flash->size = 32 * 1024 * 1024;

	return flash;
}

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
	return bus == 0 && cs == 0;
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
