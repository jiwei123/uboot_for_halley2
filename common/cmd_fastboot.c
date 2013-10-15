/*
 * Ingenic mensa boot android system command
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Martin <czhu@ingenic.cn>
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

#include <stdarg.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <errno.h>
#include <div64.h>
#include <common.h>
#include <command.h>
#include <configs/mensa.h>

static int do_fastboot(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
/*fastboot*/
	printf("Now,we are in fast boot mode.\n");
	return 0;
}

U_BOOT_CMD(
	fastboot, 1, 1, do_fastboot,
	"enter fastboot mode",
	"enter fastboot mode"
);
