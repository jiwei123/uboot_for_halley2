/*
 * JZ4775 common routines
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: Huddy <hyli@ingenic.cn>
 * Based on: xboot/boot/lcd/jz4775_android_lcd.h
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

#ifndef __ECX336AF_H__
#define __ECX336AF_H__

#include <regulator.h>

/*
 * @gpio_xclr    : reset : 0 active
 * @gpio_spi_cs  : spi chip_select 0:selected 1:not selected
 * @gpio_spi_clk : spi clock
 * @gpio_spi_mosi: spi master out, slave in
 * @gpio_spi_miso: spi master in, slave out
 */
struct ecx336af_data {
	unsigned int gpio_reset;
	unsigned int gpio_spi_cs;
	unsigned int gpio_spi_clk;
	unsigned int gpio_spi_sdi;
	unsigned int gpio_spi_sdo;
	struct regulator *power_1v8;
};

extern struct ecx336af_data ecx336af_pdata;

#endif /* __ECX336AF_H__ */
