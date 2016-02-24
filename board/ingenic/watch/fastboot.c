/*
 * Ingenic mensa setup code
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
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

#include <common.h>
#include <fastboot.h>

struct partition_info partition_info[PARTITION_NUM] = {
	[0] = {.pname = "ndxboot",     .offset = 0*1024*1024,    .size = 3*1024*1024},
	[1] = {.pname = "ndboot",      .offset = 3*1024*1024,    .size = 8*1024*1024},
	[2] = {.pname = "ndrecovery",  .offset = 12*1024*1024,   .size = 16*1024*1024},
	[3] = {.pname = "ndmisc",      .offset = 28*1024*1024,   .size = 16*1024*1024},
	[4] = {.pname = "ndreserved",  .offset = 44*1024*1024,   .size = 52*1024*1024},
	[5] = {.pname = "ndoeminfo",   .offset = 96*1024*1024,   .size = 4*1024*1024},
	[6] = {.pname = "ndcache",     .offset = 100*1024*1024,  .size = 100*1024*1024},
#ifdef CONFIG_MMC8GP2
	[7] = {.pname = "ndsystem",    .offset = 200*1024*1024,  .size = 824*1024*1024},
	[8] = {.pname = "nddata",      .offset = 1024*1024*1024, .size = 6144*1024*1024},
#elif CONFIG_MMC8GP1
	[7] = {.pname = "ndsystem",    .offset = 200*1024*1024,  .size = 512*1024*1024},
	[8] = {.pname = "nddata",      .offset = 712*1024*1024,  .size = 6456*1024*1024},
#elif CONFIG_MMC4GP2
	[7] = {.pname = "ndsystem",    .offset = 200*1024*1024,  .size = 824*1024*1024},
	[8] = {.pname = "nddata",      .offset = 1024*1024*1024, .size = 2560*1024*1024},
#else
	[7] = {.pname = "ndsystem",    .offset = 200*1024*1024,  .size = 400*1024*1024},
	[8] = {.pname = "nddata",      .offset = 600*1024*1024,  .size = 2984*1024*1024},
#endif
};
