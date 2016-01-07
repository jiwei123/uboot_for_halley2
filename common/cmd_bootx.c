/*
 *Ingenic mensa boot android system command
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
#include <config.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <errno.h>
#include <div64.h>
#include <common.h>
#include <command.h>
#include <config.h>
#include <mmc.h>
#include <boot_img.h>
#include <asm/gpio.h>
#include <fs.h>
#include <fat.h>
extern void flush_cache_all(void);


/*boot.img has been in memory alreaxdy. just call init_boot_linux() and jump to kernel.*/
static void bootx_jump_kernel(unsigned long mem_address)
{
	static u32 *param_addr = NULL;
	typedef void (*image_entry_arg_t)(int, char **, void *)
		__attribute__ ((noreturn));
	unsigned int update_flag;
	update_flag = get_update_flag();

	image_entry_arg_t image_entry =
		(image_entry_arg_t) mem_address;

	printf("Prepare kernel parameters ...\n");
	param_addr = (u32 *)CONFIG_PARAM_BASE;
	param_addr[0] = 0;
	if((update_flag & 0x3) != 0x3)
		param_addr[1] = CONFIG_SPL_BOOTARGS;
	else
		param_addr[1] = CONFIG_BOOTX_BOOTARGS;
	printf("param_addr[1] is %s\n",param_addr[1]);
	flush_cache_all();
	image_entry(2, (char **)param_addr, NULL);
	printf("We should not come here ... \n");
}

/* boot the android system form the memory directly.*/
static int mem_bootx(unsigned int mem_address)
{
	printf("Enter mem_boot routine ...\n");
	bootx_jump_kernel(mem_address);
	return 0;
}

#ifdef CONFIG_JZ_SPI
static void spi_boot(unsigned int mem_address,unsigned int spi_addr)
{
	struct image_header *header;
	unsigned int header_size;
	unsigned int entry_point, load_addr, size;

	printf("Enter SPI_boot routine ...\n");
	header_size = sizeof(struct image_header);
	spi_load(spi_addr, header_size, CONFIG_SYS_TEXT_BASE);
	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE);

	entry_point = image_get_load(header);
	/* Load including the header */
	load_addr = entry_point - header_size;
	size = image_get_data_size(header) + header_size;

	spi_load(spi_addr, size, load_addr);

	bootx_jump_kernel(mem_address);
}
#endif

#ifdef CONFIG_JZ_SFC
int bat_cap = 0;
int first = 0;
static void sfc_boot(unsigned int mem_address,unsigned int sfc_addr)
{
	struct image_header *header;
	unsigned int header_size;
	unsigned int entry_point, load_addr, size;
	gpio_port_direction_input(1,31);
	gpio_port_direction_input(1,8);
	unsigned int update_flag;
	update_flag = get_update_flag();
	if((update_flag & 0x03) != 0x03){
		while(gpio_get_value(63) && (!(gpio_get_value(40)))){
	#if 1	
		if(!first){
				first = 1;
				bat_cap = get_battery_current_cpt();
			}else{
				if(bat_cap != get_battery_current_cpt()){
					bat_cap = get_battery_current_cpt();
					lcd_display_bat_cap_first(bat_cap);
				}
			}
	#else
					
			//test battery capacity test
			bat_cap = (bat_cap + 1) % 101;
			lcd_display_bat_cap_first(bat_cap);
			mdelay(500);
	#endif		
		
		}
		if(gpio_get_value(40)){
		
			printf("usb have remove ,power off!!!\n");
			//call axp173 power off 
			jz_hibernate();		
		}
	
	}
	
	printf("Enter SFC_boot routine ...\n");
	header_size = sizeof(struct image_header);
	sfc_nor_read(sfc_addr, header_size, CONFIG_SYS_TEXT_BASE);
	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE);

	entry_point = image_get_load(header);
	/* Load including the header */
	load_addr = entry_point - header_size;
	size = image_get_data_size(header) + header_size;

	sfc_nor_read(sfc_addr, size, load_addr);
	//gpio_set_value(88,0);
	panel_power_off();
	bootx_jump_kernel(mem_address);
}
#endif
static int do_bootx(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned long mem_address,sfc_addr, size;
	unsigned int update_flag;
        argc--; argv++;
	update_flag = get_update_flag();
	if((update_flag & 0x3) != 0x3)	{
		strcpy(argv[0],"sfc");
		strcpy(argv[1],argv[3]);
	}
	printf("argv[0]: %s,argv[1]:%s\n",argv[0],argv[1]);
	/* Consume 'boota' */
	if (argc < 2)
		return CMD_RET_USAGE;

	if (!strcmp("mem",argv[0])) {
		mem_address=simple_strtoul(argv[1], NULL, 16);
		printf("mem boot start\n");
		mem_bootx(mem_address);
		printf("mem boot error\n");
	} else if (!strcmp("sfc",argv[0])) {
		mem_address = simple_strtoul(argv[1], NULL, 16);
	//	if((update_flag & 0x03) != 0x03)
			sfc_addr = 0x100000;
	//	else
	//		sfc_addr = simple_strtoul(argv[2], NULL, 16);
		printf("===>sfc_addr is 0x%x,mem_address is 0x%x\n",sfc_addr,mem_address);
		printf("SFC boot start\n");
#ifdef CONFIG_JZ_SFC
		sfc_boot(mem_address, sfc_addr);
#endif
		printf("SFC boot error\n");
		return 0;
	} else if (!strcmp("spi",argv[0])) {
		mem_address = simple_strtoul(argv[1], NULL, 16);
		sfc_addr = simple_strtoul(argv[2], NULL, 16);
		printf("SPI boot start\n");
#ifdef CONFIG_JZ_SPI
		spi_boot(mem_address, sfc_addr);
#endif
		printf("SPI boot error\n");
		return 0;
	    } else if (!strcmp("fat", argv[0])) {
                unsigned long time;
                int len_read;
                /* Get fat block device */
                if (fs_set_blk_dev(argv[1], (argc >= 3) ? argv[2] : NULL, FS_TYPE_FAT)) {
                        printf("Fat no %s device\n", argv[1]);
                        return CMD_RET_FAILURE;
                }

                mem_address = simple_strtoul(argv[3], NULL, 16);

                /* fat DOS filesystem read */
                time = get_timer(0);
                len_read = fs_read(argv[4], mem_address, sizeof(struct image_header), 0);
                time = get_timer(time);
                if (len_read <= 0) {
                        printf("FAT read failed\n");
                        return CMD_RET_FAILURE;
                }

                printf("%d bytes read in %lu ms", len_read, time);
                if (time > 0) {
                        puts(" (");
			print_size(len_read / time * 1000, "/s");
                        puts(")");
                }
                puts("\n");

                setenv_hex("filesize", len_read);

                printf("FAT boot start\n");
                bootx_jump_kernel(mem_address);
                printf("FAT boot error\n");
	}else {
		printf("%s boot unsupport\n", argv[0]);
                return CMD_RET_USAGE;
	}
	return 0;
}

#ifdef CONFIG_SYS_LONGHELP
static char bootx_help_text[] =
        "[[way],[mem_address],[offset]]\n"
        "- boot Android system....\n"
        "\tThe argument [way] means the way of booting boot.img.[way]='mem'/'sfc'.\n"
        "\tThe argument [mem_address] means the start position of xImage in memory.\n"
        "\tThe argument [offset] means the position of xImage in sfc-nor.\n"
        "";
#endif

U_BOOT_CMD(
        bootx, 6, 1, do_bootx,
        "boot xImage ",bootx_help_text
);


