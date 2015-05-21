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

#ifndef __JZ_SPI_H__
#define __JZ_SPI_H__

#define SSI_BASE CONFIG_SSI_BASE
#define COMMAND_MAX_LENGTH		8
#define SIZEOF_NAME			32
#define FIFI_THRESHOLD			64
#define SPI_WRITE_CHECK_TIMES		50

struct jz_spi_support {
	u8 id_manufactory;
	u8 id_device;
	char name[SIZEOF_NAME];
	int page_size;
	int sector_size;
	int block_size;
	int size;
	int page_num;
	unsigned int *page_list;
};

struct jz_spi_slave {
	struct spi_slave slave;
	unsigned int mode;
	unsigned int max_hz;
};

static struct jz_spi_support jz_spi_nand_support_table[] = {
	{
		.id_manufactory = 0xc8,
		.id_device = 0xf1,
		.name = "GD5F1GQ4U",
		.page_size = 2 * 1024,
		.block_size = 128 * 1024,
		.size = 128 * 1024 * 1024,
	},
	{
		.id_manufactory = 0xc8,
		.id_device = 0xf2,
		.name = "GD5F2GQ4U",
		.page_size = 2 * 1024,
		.block_size = 128 * 1024,
		.size = 256 * 1024 * 1024,
	},
	{
		.id_manufactory = 0xc8,
		.id_device = 0xf4,
		.name = "GD5F4GQ4U",
		.page_size = 2 * 1024,
		.block_size = 128 * 1024,
		.size = 512 * 1024 * 1024,
	}
};

static struct jz_spi_support jz_spi_support_table[] = {
	{
		.id_manufactory = 0xc2,
		.name = "MX25L12835F",
		.page_size = 256,
		.sector_size = 4 * 1024,
		.size = 16 * 1024 * 1024,
	},
	{
		.id_manufactory = 0xc8,
		.name = "GD25LQ64C",
		.page_size = 256,
		.sector_size = 4 * 1024,
		.size = 8 * 1024 * 1024,
	}
};

#endif /* __JZ_SPI_H__ */
