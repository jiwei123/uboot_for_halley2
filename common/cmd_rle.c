/*
 * (C) Copyright 2002
 * Detlev Zundel, DENX Software Engineering, dzu@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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

/*
 * BMP handling routines
 */

#include <common.h>
#include <lcd.h>
#include <bmp_layout.h>
#include <command.h>
#include <asm/byteorder.h>
#include <malloc.h>
#include <splash.h>
#include <video.h>

#include <jz_logo_file.h>
#define RLE_LOGO_BASE_ADDR   (0x00000000)	// need to fixed!
/* print the logo base info including width, height, size*/
static int do_rle_info(cmd_tbl_t * cmdtp, int flag, int argc,
		       char *const argv[])
{
	/*fixed */

	return 0;
}

/* display the logo on lcd panel */
static int do_rle_display(cmd_tbl_t * cmdtp, int flag, int argc,
			  char *const argv[])
{
	ulong addr;
	int console_en;

	switch (argc) {
/*	case 1:		// fixed! display a default logo if logo address offset is not set!   
		addr = RLE_LOGO_BASE_ADDR;
		break;
*/
	case 2:		/* use argument */
		addr = simple_strtoul(argv[1], NULL, 16);
		break;
	default:
		return CMD_RET_USAGE;
	}

	return (rle_display(addr));
}

static int do_rle_console(cmd_tbl_t * cmdtp, int flag, int argc,
			  char *const argv[])
{
	/*fixed */
	int val;
	if (argc < 2)
		val = 1;
	else
		val = simple_strtoul(argv[1], NULL, 0);

	lcd_console_enable(val);
	return 0;
}

static cmd_tbl_t cmd_logo_sub[] = {
	U_BOOT_CMD_MKENT(info, 2, 0, do_rle_info, "", ""),
	U_BOOT_CMD_MKENT(display, 2, 0, do_rle_display, "", ""),
	U_BOOT_CMD_MKENT(console, 2, 0, do_rle_console, "", ""),
};

#ifdef CONFIG_NEEDS_MANUAL_RELOC
void logo_reloc(void)
{
	fixup_cmdtable(cmd_logo_sub, ARRAY_SIZE(cmd_logo_sub));
}
#endif

/*
 * Subroutine:  do_logo
 *
 * Description: Handler for 'logo' command..
 *
 * Inputs:	argv[1] contains the subcommand
 *
 * Return:      None
 *
 */
static int do_logo(cmd_tbl_t * cmdtp, int flag, int argc, char *const argv[])
{
	cmd_tbl_t *c;

	/* Strip off leading 'logo' command argument */
	argc--;
	argv++;

	c = find_cmd_tbl(argv[0], &cmd_logo_sub[0], ARRAY_SIZE(cmd_logo_sub));

	if (c)
		return c->cmd(cmdtp, flag, argc, argv);
	else
		return CMD_RET_USAGE;

}

U_BOOT_CMD(rle, 4, 1, do_logo,
	   "manipulate rle image data",
	   "\tinfo <rle_AddrOffset>      - print logo info\n"
	   "\tdisplay  <rle_AddrOffset>  - display logo on display panel\n"
	   "\tconsole  <is_enabled> - display logo on display panel\n");

/*
 * Subroutine:  rle_display
 *
 * Description: Display rle file located in memory
 *
 * Inputs:	addr address of the rle file
 *
 * Return:      None
 *
 */

int rle_display(unsigned int addr_offset)
{
	int ret;

#if defined(CONFIG_LCD)
	unsigned short *logo_addr =
	    (unsigned short *)(addr_offset + RLE_LOGO_BASE_ADDR);
	//ret = lcd_display_rle(logo_addr);
	ret = lcd_display_rle(jz_logo_file);
#elif defined(CONFIG_VIDEO)
/*	ret = video_display_rle(addr, x, y);*/
#else
# error rle_display() requires CONFIG_LCD or CONFIG_VIDEO
#endif

	return ret;
}
