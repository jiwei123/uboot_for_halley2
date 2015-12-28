/*
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
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
 *
 * Author :   Kznan <Derrick.kznan@ingenic.com>
 * Description :   used for fastly detect hardware BUG
 *
 */

#include <common.h>
#include <serial.h>

static int detect_ddr(void)
{
	unsigned int addr_start = 0;
	unsigned int addr_end = 0;
	unsigned int i = 0;
	unsigned int value = 0;
	
	addr_start = KSEG1 + 0x100000;
	addr_end = addr_start + 0x40000;

	for (i = addr_start; i <= addr_end; i++)
		*(volatile unsigned char *)i = 0xaa;

	for (i = addr_start; i <= addr_end; i++) {
		value = *(volatile unsigned char *)i;
		if (value != 0xaa) {
			printf("[ Error ] ADDR(0x%x) Expect(0x%x) REAL(0x%x)\n", i, 0xaa, value);
			return 1;
		}
	}

	for (i = addr_start; i <= addr_end; i++)
		*(volatile unsigned char *)i = (unsigned char)i;

	for (i = addr_start; i <= addr_end; i++) {
		value = *(volatile unsigned char *)i;
		if (value != (unsigned char)i) {
			printf("[ Error ] ADDR(0x%x) Expect(0x%x) REAL(0x%x)\n", i, 0xaa, value);
			return 1;
		}
	}

	// FIXME : do we need to test the dataline and addrline ?

	return 0;
}

#ifdef CONFIG_REGULATOR
extern int regulator_init(void);
extern int pmu_selfdet(void);
#else
#error "Please config regulator for detect!!!"
#endif

static int detect_pmu(void)
{
	int ret = 0;

	ret = pmu_selfdet();

	return ret;
}

extern int lcd_selfdet(void);
static int detect_lcd(void)
{
	int ret = 0;

	regulator_init();
	board_set_lcd_power_on();
	panel_pin_init();

	ret = lcd_selfdet();

	return ret;
}

static int do_hardware_detect(struct cmd_tbl_s *cmdtp, int flag,
			      int argc, char * const argv[])
{
	int ret = 0;
	puts("\n");
	puts("Hardware Detect Start\n\n");

	ret = detect_ddr();
	if (ret)
		puts("Detect DDR ...  ERR\n");
	else
		puts("Detect DDR ...  OK\n");

	ret = detect_pmu();
	if (ret)
		puts("Detect PMU ...  ERR\n");
	else
		puts("Detect PMU ...  OK\n");

	ret = detect_lcd();
	if (ret)
		puts("Detect LCD ...  ERR\n");
	else
		puts("Detect LCD ...  OK\n");

	puts("\n");

	return 0;
}

U_BOOT_CMD(hardwaredet, 1, 1, do_hardware_detect,
	   "Detect hardware and report the result", NULL);
