/*
 * ReliableIPTV urboard configuration
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
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

#ifndef __CONFIG_MENSA_H__
#define __CONFIG_MENSA_H__

#define CONFIG_MIPS32		/* MIPS32 CPU core */
#define CONFIG_SYS_LITTLE_ENDIAN
#define CONFIG_JZ4775		/* Jz4775 SoC */

#define CONFIG_SYS_APLL_FREQ		816000000
#define CONFIG_SYS_MPLL_FREQ		-1

#define CONFIG_SYS_EXTAL		24000000	/* EXTAL freq: 48 MHz */
#define CONFIG_SYS_HZ			(CONFIG_SYS_EXTAL / 16) /* incrementer freq */
#define CONFIG_SYS_MIPS_TIMER_FREQ	CONFIG_SYS_CPU_SPEED

#define CONFIG_SYS_CPU_FREQ		CONFIG_SYS_APLL_FREQ	/* CPU clock: 1.2 GHz */
#define CONFIG_SYS_MEM_FREQ		204000000

#define CONFIG_DDR_PARAMS_CREATOR
#define CONFIG_DDR3_H5TQ1G83DFR_H9C

#define CONFIG_SYS_AUDIO_SPEED (768 * 1000000)

#define CONFIG_SYS_UART_BASE UART3_BASE
#define CONFIG_BAUDRATE 115200

#define CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_FLASH_BASE	0 /* init flash_base as 0 */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_MISC_INIT_R 1

#define CONFIG_BOOTP_MASK	(CONFIG_BOOTP_DEFAUL)

#define CONFIG_BOOTDELAY 1
#define BOOTARGS_COMMON "console=ttyS3,115200 mem=256M@0x0 mem=256M@0x30000000"

#ifdef CONFIG_MBR_CREATOR
#define CONFIG_MBR_P0_OFF	64mb
#define CONFIG_MBR_P0_END	556mb
#define CONFIG_MBR_P0_TYPE 	linux

#define CONFIG_MBR_P1_OFF	580mb
#define CONFIG_MBR_P1_END 	1604mb
#define CONFIG_MBR_P1_TYPE 	linux

#define CONFIG_MBR_P2_OFF	28mb
#define CONFIG_MBR_P2_END	58mb
#define CONFIG_MBR_P2_TYPE 	linux

#define CONFIG_MBR_P3_OFF	1609mb
#define CONFIG_MBR_P3_END	7800mb
#define CONFIG_MBR_P3_TYPE 	fat
#endif

#ifdef CONFIG_SPL_MMC_SUPPORT

/* SD/MMC card defaults */

#define CONFIG_BOOTARGS \
    BOOTARGS_COMMON " root=/dev/mmcblk0p1"
#define CONFIG_BOOTCOMMAND \
    "fatload mmc 1:1 0x88000000 vmlinux.ub; bootm 0x88000000"

#else /* !CONFIG_SPL_MMC_SUPPORT */

/* NAND defaults */

#define CONFIG_BOOTARGS \
    BOOTARGS_COMMON " ubi.mtd=1 root=ubi0:root rootfstype=ubifs rw"
#define CONFIG_BOOTCOMMAND \
    "mtdparts default; ubi part system; ubifsmount ubi:boot; " \
    "ubifsload 0x88000000 vmlinux.ub; bootm 0x88000000"


#endif /* !CONFIG_SPL_MMC_SUPPORT */

#define CONFIG_JZ_GPIO

/* NAND */
#define CONFIG_NAND			1
#define CONFIG_NAND_JZ4780		1
#define CONFIG_SYS_NAND_BASE		0xbb000000
#define CONFIG_SYS_NAND_ONFI_DETECTION	1
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_PAGE_SIZE	4096
#define CONFIG_SYS_NAND_BLOCK_SIZE	(1024 << 10)
#define CONFIG_SYS_NAND_OOBSIZE		224
#define CONFIG_SYS_NAND_ECCSIZE		1024
#define CONFIG_SYS_NAND_ECCSTRENGTH	24
#define CONFIG_SYS_NAND_ECCBYTES	((CONFIG_SYS_NAND_ECCSTRENGTH * 14) / 8)
#define CONFIG_SYS_NAND_ECC_POS		56
#define CONFIG_SYS_NAND_ECCPOS { \
	56, 57, 58, 59, 60, 61, 62, 63, \
	64, 65, 66, 67, 68, 69, 70, 71, \
	72, 73, 74, 75, 76, 77, 78, 79, \
	80, 81, 82, 83, 84, 85, 86, 87, \
	88, 89, 90, 91, 92, 93, 94, 95, \
	96, 97, 98, 99, 100, 101, 102, 103, \
	104, 105, 106, 107, 108, 109, 110, 111, \
	112, 113, 114, 115, 116, 117, 118, 119, \
	120, 121, 122, 123, 124, 125, 126, 127, \
	128, 129, 130, 131, 132, 133, 134, 135, \
	136, 137, 138, 139, 140, 141, 142, 143, \
	144, 145, 146, 147, 148, 149, 150, 151, \
	152, 153, 154, 155, 156, 157, 158, 159, \
	160, 161, 162, 163, 164, 165, 166, 167, \
	168, 169, 170, 171, 172, 173, 174, 175, \
	176, 177, 178, 179, 180, 181, 182, 183, \
	184, 185, 186, 187, 188, 189, 190, 191, \
	192, 193, 194, 195, 196, 197, 198, 199, \
	200, 201, 202, 203, 204, 205, 206, 207, \
	208, 209, 210, 211, 212, 213, 214, 215, \
	216, 217, 218, 219, 220, 221, 222, 223 }
#define CONFIG_SYS_NAND_HW_ECC_OOBFIRST	1
#define CONFIG_SYS_NAND_5_ADDR_CYCLE	1
#define CONFIG_SYS_NAND_PAGE_COUNT      (CONFIG_SYS_NAND_BLOCK_SIZE / CONFIG_SYS_NAND_PAGE_SIZE)
#define CONFIG_SYS_NAND_BAD_BLOCK_POS	0
#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#define MTDIDS_DEFAULT			"nand0=nand"
#define MTDPARTS_DEFAULT		"mtdparts=nand:4m(uboot-spl),1m(uboot),1m(uboot-env),2m(skip),-(system)"

/*
 * MMC
 */

#define CONFIG_GENERIC_MMC		1
#define CONFIG_MMC			1
#define CONFIG_JZ_MMC 1
#define CONFIG_JZ_MMC_MSC0 1
#define CONFIG_JZ_MMC_MSC0_PA_4BIT 1
#define CONFIG_JZ_MMC_MSC1 1
#define CONFIG_JZ_MMC_MSC1_PE_4BIT 1
#define CONFIG_JZ_MMC_SPLMSC 0

/*
 * I2C
 */
#define CONFIG_SOFT_I2C
#define CONFIG_SYS_I2C_SPEED 50     /* the function is not implemented */
#define CONFIG_SYS_I2C_SLAVE 0x00   /* the function is not implemented */
#define CONFIG_SOFT_I2C_GPIO_SCL    (32 * 4 + 3) /* GPE3 ok*/
#define CONFIG_SOFT_I2C_GPIO_SDA    (32 * 4 + 0) /* GPE0 ok*/

/* Ethernet:  davicom DM9000 */
#define CONFIG_DRIVER_DM9000		1
#define CONFIG_DM9000_BASE		0xb6000000
#define DM9000_IO			CONFIG_DM9000_BASE
#define DM9000_DATA			(CONFIG_DM9000_BASE + 2)

/*
 * Command line configuration.
 */
#define CONFIG_CMD_BOOTD	/* bootd			*/
#define CONFIG_CMD_CONSOLE	/* coninfo			*/
#define CONFIG_CMD_DHCP 	/* DHCP support			*/
#define CONFIG_CMD_ECHO		/* echo arguments		*/
#define CONFIG_CMD_EXT4 	/* ext4 support			*/
#define CONFIG_CMD_FAT		/* FAT support			*/
#define CONFIG_CMD_LOADB	/* loadb			*/
#define CONFIG_CMD_LOADS	/* loads			*/
#define CONFIG_CMD_MEMORY	/* md mm nm mw cp cmp crc base loop mtest */
#define CONFIG_CMD_MISC		/* Misc functions like sleep etc*/
#define CONFIG_CMD_MMC 	    /* MMC/SD support			*/
#define CONFIG_CMD_MTDPARTS
#define CONFIG_CMD_NET 	    /* networking support			*/
#define CONFIG_CMD_PING
#define CONFIG_CMD_RUN		/* run command in env variable	*/
#define CONFIG_CMD_SAVEENV	/* saveenv			*/
#define CONFIG_CMD_SETGETDCR	/* DCR support on 4xx		*/
#define CONFIG_CMD_SOURCE	/* "source" command support	*/
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS

#define CONFIG_DOS_PARTITION

#define CONFIG_LZO
#define CONFIG_RBTREE

/*
 * Serial download configuration
 */
#define CONFIG_LOADS_ECHO	1	/* echo on for serial download */

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_MAXARGS 16
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT "mensa# "
#define CONFIG_SYS_CBSIZE 1024 /* Console I/O Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

#define CONFIG_SYS_MONITOR_LEN		(512 * 1024)
#define CONFIG_SYS_MALLOC_LEN		(64 * 1024 * 1024)
#define CONFIG_SYS_BOOTPARAMS_LEN	(128 * 1024)

#define CONFIG_SYS_SDRAM_BASE		0x80000000 /* cached (KSEG0) address */
#define CONFIG_SYS_SDRAM_MAX_TOP	0x90000000 /* don't run into IO space */
#define CONFIG_SYS_INIT_SP_OFFSET	0x400000
#define CONFIG_SYS_LOAD_ADDR		0x88000000
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		0x88000000

#define CONFIG_SYS_TEXT_BASE		0x80100000
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_TEXT_BASE

/*
 * Environment
 */
#ifdef CONFIG_ENV_IS_IN_MMC

#define CONFIG_SYS_MMC_ENV_DEV		0
#define CONFIG_ENV_SIZE			(32 << 10)
#define CONFIG_ENV_OFFSET		((16 + 512) << 10)

#else

#define CONFIG_ENV_IS_IN_NAND
#define CONFIG_ENV_SIZE			(32 << 10)
#define CONFIG_ENV_OFFSET		(CONFIG_SYS_NAND_BLOCK_SIZE * 5)

#endif

/*
 * SDRAM Info.
 */
#define CONFIG_NR_DRAM_BANKS	1

/*
 * Cache Configuration
 */
#define CONFIG_SYS_DCACHE_SIZE		16384
#define CONFIG_SYS_ICACHE_SIZE		16384
#define CONFIG_SYS_CACHELINE_SIZE	32

/* SPL */
#define CONFIG_SPL
#define CONFIG_SPL_FRAMEWORK
#define CONFIG_SPL_STACK		0x80004000 /* only max. 2KB spare! */

#define CONFIG_SPL_BSS_START_ADDR	0x8f000000
#define CONFIG_SPL_BSS_MAX_SIZE		0x00080000 /* 512KB, arbitrary */

#define CONFIG_SPL_NO_CPU_SUPPORT_CODE
#define CONFIG_SPL_START_S_PATH		"$(CPUDIR)/$(SOC)"
#define CONFIG_SPL_LDSCRIPT		"$(CPUDIR)/$(SOC)/u-boot-spl.lds"
#define CONFIG_SPL_PAD_TO		15872 /* u-boot start addr - mbr size(512) */

#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	0x20 /* 16KB offset */
#define CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS	0x400 /* 512 KB */
#define CONFIG_SYS_NAND_U_BOOT_OFFS	(CONFIG_SYS_NAND_BLOCK_SIZE * 4)
#define CONFIG_SYS_NAND_U_BOOT_DST	CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_NAND_U_BOOT_START	CONFIG_SYS_NAND_U_BOOT_DST
#define CONFIG_SYS_NAND_U_BOOT_SIZE	(512 * 1024)

#define CONFIG_SPL_BOARD_INIT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_GPIO_SUPPORT
#define CONFIG_SPL_I2C_SUPPORT

#ifdef CONFIG_SPL_MMC_SUPPORT

#define CONFIG_SPL_TEXT_BASE		0xf4000a00
#define CONFIG_SPL_MAX_SIZE		((16 * 1024) - 0xa00)

#define CONFIG_SPL_SERIAL_SUPPORT

#else /* !CONFIG_SPL_MMC_SUPPORT */

#define CONFIG_SPL_NAND_SUPPORT
#define CONFIG_SPL_NAND_BASE
#define CONFIG_SPL_NAND_DRIVERS
#define CONFIG_SPL_NAND_SIMPLE
#define CONFIG_SPL_NAND_LOAD

#define CONFIG_SPL_TEXT_BASE		0xf4000800
#define CONFIG_SPL_MAX_SIZE		((16 * 1024) - 0x800)

/* the NAND SPL is small enough to enable serial */
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SPL_LIBCOMMON_SUPPORT

#endif /* !CONFIG_SPL_MMC_SUPPORT */

#endif /* __CONFIG_MENSA_H__ */
