/*
 * Ingenic watch configuration
 *
 * Copyright (c) 2014 Ingenic Semiconductor Co.,Ltd
 * Author: Zoro <ykli@ingenic.cn>
 * Based on: include/configs/urboard.h
 *           Written by Paul Burton <paul.burton@imgtec.com>
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

#ifndef __CONFIG_WATCH_H__
#define __CONFIG_WATCH_H__

/**
 * Basic configuration(SOC, Cache, UART, DDR).
 */

#define CONFIG_MIPS32		/* MIPS32 CPU core */
#define CONFIG_SYS_LITTLE_ENDIAN
#define CONFIG_M200		/* M200 SoC */
#define CONFIG_DDR_AUTO_SELF_REFRESH
#define CONFIG_SPL_DDR_SOFT_TRAINING

#define EMC_LOW_SDRAM_SPACE_SIZE (250 * 0x100000)

/*#define CONFIG_26M*/

#if defined(CONFIG_IWOP)
#define CONFIG_SYS_APLL_FREQ        984000000	/*If APLL not use mast be set 0*/
#define CONFIG_SYS_CPU_FREQ         984000000  /*If APLL not use mast be set 0*/
#define CONFIG_SYS_MEM_FREQ         270000000  /*If APLL not use mast be set 0*/
#define CONFIG_SYS_MPLL_FREQ        810000000   /*If MPLL not use mast be set 0*/
#else
#define CONFIG_SYS_APLL_FREQ        984000000   /*If APLL not use mast be set 0*/
#define CONFIG_SYS_CPU_FREQ         984000000   /*If APLL not use mast be set 0*/
#define CONFIG_SYS_MEM_FREQ         200000000   /*If APLL not use mast be set 0*/
#define CONFIG_SYS_MPLL_FREQ        600000000   /*If MPLL not use mast be set 0*/
#endif

#define CONFIG_CPU_SEL_PLL          APLL
#define CONFIG_DDR_SEL_PLL          MPLL
#define CONFIG_SYS_EXTAL            24000000    /* EXTAL freq: 24 MHz */
#define CONFIG_SYS_HZ               1000        /* incrementer freq */


#define CONFIG_SYS_DCACHE_SIZE		32768
#define CONFIG_SYS_ICACHE_SIZE		32768
#define CONFIG_SYS_CACHELINE_SIZE	32

#define CONFIG_SYS_UART_INDEX		3
#define CONFIG_BAUDRATE			57600

/*#define CONFIG_DDR_TEST_CPU
#define CONFIG_DDR_TEST*/
#define CONFIG_DDR_PARAMS_CREATOR
#define CONFIG_DDR_HOST_CC
#define CONFIG_DDR_FORCE_SELECT_CS1

#if defined(CONFIG_MEM1GB) && !defined(CONFIG_MEM_RANKS)		/* MEM:1GB default RANKS:1 */
	#define CONFIG_MEM_RANKS		1
#elif !defined(CONFIG_MEM_RANKS)
	#define CONFIG_MEM_RANKS		0
#endif

#define CONFIG_DDR_TYPE_LPDDR2
#define CONFIG_DDR_CS0				1			/* 1-connected, 0-disconnected */
#define CONFIG_DDR_CS1				CONFIG_MEM_RANKS	/* 1-connected, 0-disconnected */
#define CONFIG_DDR_DW32				1			/* 1-32bit-width, 0-16bit-width */

#ifdef  CONFIG_SOLAR
#define CONFIG_PMU_SM5007
#define CONFIG_CMD_PMU
#define CONFIG_MCP_H9TU32A4GDMCLR_KGM_LPDDR2
#else
#define CONFIG_PMU_RICOH6x
#define CONFIG_MCP_H9TP32A8JDMC_PRKGM_LPDDR2
#endif
/*#define CONFIG_MCP_SAMSUNG_KMN5X000ZM_LPDDR2*/

/* #define CONFIG_DDR_DLL_OFF */
/*
 * #define CONFIG_DDR_CHIP_ODT
 * #define CONFIG_DDR_PHY_ODT
 * #define CONFIG_DDR_PHY_DQ_ODT
 * #define CONFIG_DDR_PHY_DQS_ODT
 * #define CONFIG_DDR_PHY_IMPED_PULLUP		0xe
 * #define CONFIG_DDR_PHY_IMPED_PULLDOWN	0xe
 */

/**
 * Boot arguments definitions.
 */
/* console */
#ifdef CONFIG_KERNEL_CONSOLE
	#define BOOTARGS_CONSOLE " console="CONFIG_KERNEL_CONSOLE
#else
	#define BOOTARGS_CONSOLE " console=ttyS3,57600n8 "
#endif

/* DDR Capacity */
#ifdef CONFIG_MEM1GB
	#define BOOTARGS_MEM " mem=250M@0x0 mem=768M@0x30000000"
#else
	#define BOOTARGS_MEM " mem=250M@0x0 mem=256M@0x30000000"
#endif

#define BOOTARGS_COMMON BOOTARGS_CONSOLE BOOTARGS_MEM

#ifdef CONFIG_BOOT_ANDROID
  #define CONFIG_BOOTARGS BOOTARGS_COMMON " ip=off root=/dev/ram0 rw rdinit=/init"
#else
  #ifdef CONFIG_SPL_MMC_SUPPORT
/*    #define CONFIG_BOOTARGS BOOTARGS_COMMON " ip=192.168.10.205:192.168.10.1:192.168.10.1:255.255.255.0 nfsroot=192.168.8.3:/home/nfsroot/bliu/buildroot rw" */
/*	#define CONFIG_BOOTARGS BOOTARGS_COMMON " ip=off root=/dev/ram0 rw rdinit=/linuxrc" */
	#define CONFIG_BOOTARGS BOOTARGS_COMMON " rootdelay=2 init=/linuxrc root=/dev/mmcblk0p7 rw"
  #else
    #define CONFIG_BOOTARGS BOOTARGS_COMMON " ubi.mtd=1 root=ubi0:root rootfstype=ubifs rw"
  #endif
#endif

/**
 * Boot command definitions.
 */
#define CONFIG_BOOTDELAY 0

#ifdef CONFIG_BOOT_ANDROID
  #ifdef CONFIG_SPL_MMC_SUPPORT
    #define CONFIG_BOOTCOMMAND	\
	  "cls; batterydet; battery_for_kernel; lcd_logo on; boota mmc 0 0x80f00000 6144"
    #define CONFIG_NORMAL_BOOT CONFIG_BOOTCOMMAND
    #define CONFIG_RECOVERY_BOOT "cls; lcd_logo on; boota mmc 0 0x80f00000 24576"
  #else
    #define CONFIG_BOOTCOMMAND "cls; lcd_logo on; boota nand 0 0x80f00000 6144"
    #define CONFIG_NORMAL_BOOT CONFIG_BOOTCOMMAND
    #define CONFIG_RECOVERY_BOOT "cls; lcd_logo on; boota nand 0 0x80f00000 24576"
  #endif
#else  /* CONFIG_BOOT_ANDROID */
  #ifdef CONFIG_SPL_MMC_SUPPORT
/*    #define CONFIG_BOOTCOMMAND "tftpboot 0x80600000 bliu/85/uImage.new; bootm" */
	#define CONFIG_BOOTCOMMAND "cls; lcd_logo on; mmc read 0x80600000 0x1800 0x3000; bootm 0x80600000"
  #else
    #define CONFIG_BOOTCOMMAND						\
	"cls; lcd_logo on;" \
	"mtdparts default; ubi part system; ubifsmount ubi:boot; "	\
	"ubifsload 0x80f00000 vmlinux.ub; bootm 0x80f00000"
  #endif
#endif /* CONFIG_BOOT_ANDROID */

/**
 * Drivers configuration.
 */
#define CONFIG_LCD
#ifdef CONFIG_LCD
#define LCD_BPP				5
#define CONFIG_GPIO_LCD_PWM	 	GPIO_PE(1)
#define CONFIG_LCD_LOGO
#define CONFIG_RLE_LCD_LOGO
/*#define CONFIG_LCD_INFO_BELOW_LOGO*/     /*display the console info on lcd panel for debugg */
#define CONFIG_SYS_WHITE_ON_BLACK
#define CONFIG_SYS_PWM_PERIOD		10000 /* Pwm period in ns */
#define CONFIG_SYS_PWM_CHN		1  /* Pwm channel ok*/
#define CONFIG_SYS_PWM_FULL		256
#define CONFIG_SYS_BACKLIGHT_LEVEL	80 /* Backlight brightness is (80 / 256) */
#define CONFIG_VIDEO_M200
#define CONFIG_JZ_PWM
#ifdef CONFIG_RLE_LCD_LOGO
#define CONFIG_CMD_BATTERYDET   	/* detect battery and show charge logo */
#define CONFIG_CMD_LOGO_RLE	/*display the logo using rle command*/
#endif

#define CONFIG_SYS_BACKLIGHT_LEVELS   256

/* #define CONFIG_EASYSCALE_BACKLIGHT */
#ifdef CONFIG_EASYSCALE_BACKLIGHT
#define CONFIG_GPIO_EASYSCALE_CTRL   GPIO_PE(1)
#define CONFIG_GPIO_EASYSCALE_PWR_EN GPIO_PC(23) /* board x3 is pc23 */
#define CONFIG_EASYSCALE_CTRL_STEPS  32		     /* TPS61165 is 32 */
#endif

/*#define CONFIG_VIDEO_TRULY_TFT240240_2_E*/
/*#define CONFIG_VIDEO_TRULY_TFT320320*/
/*CONFIG_VIDEO_X163*/
/*CONFIG_VIDEO_SAMSUNG*/

#ifdef  CONFIG_VIDEO_X163
#define CONFIG_JZ_MIPI_DSI
#endif

#ifdef  CONFIG_VIDEO_ST7796S
#define CONFIG_JZ_MIPI_DSI
#endif

#ifdef  CONFIG_VIDEO_BYD_9177AA
#define CONFIG_JZ_MIPI_DSI
#endif

#ifdef  CONFIG_VIDEO_SAMSUNG
#define CONFIG_JZ_MIPI_DSI
#endif

#ifdef CONFIG_VIDEO_ORISE_OTM3201A
#define CONFIG_JZ_MIPI_DSI
#define CONFIG_LCD_FORMAT_X8B8G8R8
#endif

#ifdef CONFIG_VIDEO_BOE_TFT320320
#define CONFIG_JZ_MIPI_DSI
#endif

#ifdef CONFIG_VIDEO_H160_TFT320320
#define CONFIG_JZ_MIPI_DSI
#endif

#ifdef CONFIG_VIDEO_EDO_E1392AM1
#define CONFIG_JZ_MIPI_DSI
#endif

#ifdef CONFIG_VIDEO_AUO_H139BLN01
#define CONFIG_JZ_MIPI_DSI
#endif

#ifdef CONFIG_VIDEO_ARS_NT35350
#define CONFIG_JZ_MIPI_DSI
#endif

#ifdef  CONFIG_VIDEO_BM347WV_F_8991FTGF
#define CONFIG_LCD_GPIO_FUNC2_SLCD
#endif

#ifdef  CONFIG_VIDEO_TRULY_TFT240240_2_E
#define CONFIG_LCD_GPIO_FUNC2_SLCD
#endif

#ifdef  CONFIG_VIDEO_BOE_HSX0154B24B
#define CONFIG_LCD_GPIO_FUNC2_SLCD
#endif

#ifdef  CONFIG_VIDEO_TRULY_TFT320320
#define CONFIG_LCD_GPIO_FUNC2_SLCD
#endif

#ifdef  CONFIG_LCD_GPIO_FUNC0_24BIT
/* ... */
#endif

#endif /* CONFIG_LCD */

/* MMC */
#define CONFIG_GENERIC_MMC		1
#define CONFIG_MMC			1
#define CONFIG_JZ_MMC 1

#ifdef CONFIG_JZ_MMC_MSC0
#define CONFIG_JZ_MMC_SPLMSC 0
#define CONFIG_JZ_MMC_MSC0_PA_8BIT 1
#define CONFIG_JZ_MMC_MSC0_INTERNAL_PULL 1
#endif
#ifdef CONFIG_JZ_MMC_MSC1
#define CONFIG_JZ_MMC_SPLMSC 1
#define CONFIG_JZ_MMC_MSC1_PE 1
#define CONFIG_JZ_MMC_MSC1_INTERNAL_PULL 0
#endif

/* I2C */
#define CONFIG_MUTIPLE_I2C_BUS 	   /* mutiple i2c bus enable */
#define CONFIG_SOFT_I2C
#define CONFIG_SYS_I2C_SPEED		50     /* the function is not implemented */
#define CONFIG_SYS_I2C_SLAVE		0x00   /* the function is not implemented */

#ifdef CONFIG_SOFT_I2C0 /* the first (the default) i2c bus */
#define CONFIG_SOFT_I2C_GPIO_SCL	GPIO_PD(31)
#define CONFIG_SOFT_I2C_GPIO_SDA	GPIO_PD(30)
#endif

#ifndef CONFIG_SOFT_I2C0
#define CONFIG_SOFT_I2C_GPIO_SCL	GPIO_PE(31)
#define CONFIG_SOFT_I2C_GPIO_SDA	GPIO_PE(30)
#endif

#if defined(CONFIG_MUTIPLE_I2C_BUS) /* the second i2c bus if mutiple i2c bus config */

#define CONFIG_SOFT_I2C_GPIO_SCL0	CONFIG_SOFT_I2C_GPIO_SCL
#define CONFIG_SOFT_I2C_GPIO_SDA0	CONFIG_SOFT_I2C_GPIO_SDA

#if defined(CONFIG_VIBRATE_DRV2605)
#define CONFIG_SOFT_I2C_GPIO_SCL1	GPIO_PB(8)
#define CONFIG_SOFT_I2C_GPIO_SDA1	GPIO_PB(7)
#endif
#endif

#define CONFIG_SOFT_I2C_READ_REPEATED_START


#define CONFIG_REGULATOR


/* DEBUG ETHERNET */
/*
#define CONFIG_SERVERIP		192.168.8.3
#define CONFIG_IPADDR		192.168.10.206
#define CONFIG_GATEWAYIP        192.168.10.1
#define CONFIG_NETMASK          255.255.255.0
#define CONFIG_ETHADDR          00:11:22:33:44:55
*/
/* GPIO */
#define CONFIG_JZ_GPIO
#define CONFIG_CMD_GPIO

/**
 * Command configuration.
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
#define CONFIG_CMD_MMC		/* MMC/SD support			*/
#define CONFIG_CMD_NET		/* networking support			*/
#define CONFIG_CMD_PING
#define CONFIG_CMD_RUN		/* run command in env variable	*/
#define CONFIG_CMD_SETGETDCR	/* DCR support on 4xx		*/
#define CONFIG_CMD_SOURCE	/* "source" command support	*/
#define CONFIG_CMD_GETTIME
#define CONFIG_CMD_EEPROM
#define CONFIG_CMD_SAVEENV	/* saveenv			*/
#define CONFIG_MD5          /* MD5 */
/*#define CONFIG_CMD_I2C*/

/* Console configure */
#define CONFIG_AUTO_COMPLETE
#define CONFIG_CMDLINE_EDITING

/*eeprom*/
#ifdef CONFIG_CMD_EEPROM
#define CONFIG_SYS_I2C_EEPROM_ADDR  0x50
/*#define CONFIG_ENV_EEPROM_IS_ON_I2C*/
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN	1
#endif


/* USB */
/*#define CONFIG_CMD_FASTBOOT*/
#define CONFIG_USB_GADGET
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_JZ_DWC2_UDC_V1_1
#define CONFIG_FASTBOOT_GADGET
#define CONFIG_FASTBOOT_FUNCTION
#define CONFIG_G_FASTBOOT_VENDOR_NUM	(0x18d1)
#define CONFIG_G_FASTBOOT_PRODUCT_NUM	(0xdddd)
#define CONFIG_USB_GADGET_VBUS_DRAW 500

/**
 * Serial download configuration
 */
#define CONFIG_LOADS_ECHO	1	/* echo on for serial download */

/**
 * Miscellaneous configurable options
 */
#define CONFIG_DOS_PARTITION

#define CONFIG_LZO
#define CONFIG_RBTREE

#define CONFIG_SKIP_LOWLEVEL_INIT
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_GPIO_EARLY_INIT
#define CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_FLASH_BASE	0 /* init flash_base as 0 */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_MISC_INIT_R 1

#define CONFIG_BOOTP_MASK	(CONFIG_BOOTP_DEFAUL)

#define CONFIG_SYS_MAXARGS 16
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_PROMPT CONFIG_SYS_BOARD "# "
#define CONFIG_SYS_CBSIZE 1024 /* Console I/O Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)

#if defined(CONFIG_F1)
#define CONFIG_SYS_MONITOR_LEN      (1768 * 1024)
#else
#define CONFIG_SYS_MONITOR_LEN		(768 * 1024)
#endif
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

/**
 * Environment
 */
#ifdef CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		0
#define CONFIG_ENV_SIZE			(32 << 10)
#define CONFIG_ENV_OFFSET		(CONFIG_SYS_MONITOR_LEN + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR * 512)
#else
/*
#define CONFIG_ENV_IS_IN_NAND
*/
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_SIZE			(32 << 10)
#define CONFIG_ENV_OFFSET		(CONFIG_SYS_NAND_BLOCK_SIZE * 5)
#endif

/**
 * SPL configuration
 */
#define CONFIG_SPL
#define CONFIG_SPL_FRAMEWORK

#define CONFIG_SPL_NO_CPU_SUPPORT_CODE
#define CONFIG_SPL_START_S_PATH		"$(CPUDIR)/$(SOC)"
#ifdef CONFIG_SPL_NOR_SUPPORT
#define CONFIG_SPL_LDSCRIPT		"$(CPUDIR)/$(SOC)/u-boot-nor-spl.lds"
#else
#define CONFIG_SPL_LDSCRIPT		"$(CPUDIR)/$(SOC)/u-boot-spl.lds"
#endif
#define CONFIG_SPL_PAD_TO		26624 /* equal to spl max size in M200 */


#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	86//0x5A //wli changed 0x20 /* 16KB offset */
#define CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS	0x400 /* 512 KB */
#define CONFIG_SYS_NAND_U_BOOT_OFFS	(CONFIG_SYS_NAND_BLOCK_SIZE * 4)
#define CONFIG_SYS_NAND_U_BOOT_DST	CONFIG_SYS_TEXT_BASE
#define CONFIG_SYS_NAND_U_BOOT_START	CONFIG_SYS_NAND_U_BOOT_DST
#define CONFIG_SYS_NAND_U_BOOT_SIZE	(512 * 1024)

#define CONFIG_SPL_BOARD_INIT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_GPIO_SUPPORT
#define CONFIG_SPL_I2C_SUPPORT
#define CONFIG_SPL_REGULATOR_SUPPORT

#define CONFIG_SPL_CORE_VOLTAGE		1100

#ifdef CONFIG_SPL_NOR_SUPPORT
#define CONFIG_SPL_TEXT_BASE		0xba000000
#else
#define CONFIG_SPL_TEXT_BASE		0x80001000
#endif	/*CONFIG_SPL_NOR_SUPPORT*/
#define CONFIG_SPL_MAX_SIZE		(26 * 1024)

#ifdef CONFIG_SPL_MMC_SUPPORT
#define CONFIG_SPL_SERIAL_SUPPORT

#endif /* CONFIG_SPL_MMC_SUPPORT */

#ifdef CONFIG_SPL_NAND_SUPPORT


/* the NAND SPL is small enough to enable serial */
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SPL_LIBCOMMON_SUPPORT

#endif /* CONFIG_SPL_NAND_SUPPORT */

#ifdef CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SPI_SPL_CHECK
#define CONFIG_SYS_SPI_BOOT_FREQ	1000000
#endif

#ifdef CONFIG_SPL_NOR_SUPPORT
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SYS_UBOOT_BASE		(CONFIG_SPL_TEXT_BASE + CONFIG_SPL_PAD_TO - 0x40)	//0x40 = sizeof (image_header)
#define CONFIG_SYS_OS_BASE		0
#define CONFIG_SYS_SPL_ARGS_ADDR	0
#define CONFIG_SYS_FDT_BASE		0
#endif

/**
 * GPT configuration
 */
#ifdef CONFIG_GPT_CREATOR
#define CONFIG_GPT_TABLE_FILE   "$(TOPDIR)/board/$(BOARDDIR)/partitions.tab"

#ifdef CONFIG_MMC4GP2
#undef CONFIG_GPT_TABLE_FILE
#define CONFIG_GPT_TABLE_FILE   "$(TOPDIR)/board/$(BOARDDIR)/mmc4gp2.tab"
#endif

#ifdef CONFIG_MMC8GP1
#undef CONFIG_GPT_TABLE_FILE
#define CONFIG_GPT_TABLE_FILE   "$(TOPDIR)/board/$(BOARDDIR)/mmc8gp1.tab"
#endif

#ifdef CONFIG_MMC8GP2
#undef CONFIG_GPT_TABLE_FILE
#define CONFIG_GPT_TABLE_FILE   "$(TOPDIR)/board/$(BOARDDIR)/mmc8gp2.tab"
#endif

#else
/* USE MBR + zero-GPT-table instead if no gpt table defined*/
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

#define CONFIG_MSC_U_BOOT
#ifdef  CONFIG_MSC_U_BOOT
#define PTN_MISC_OFFSET         0x1C00000
#endif
/**
 * Keys.
 */
#if defined(CONFIG_ACRAB)
#define CONFIG_GPIO_USB_DETECT		GPIO_PA(29)
#define CONFIG_GPIO_USB_DETECT_ENLEVEL	0
#elif defined(CONFIG_AW808) || defined(CONFIG_SWIMBOT) || defined(CONFIG_X3) || defined(CONFIG_IN901) || defined(CONFIG_F1)
#define CONFIG_GPIO_USB_DETECT		GPIO_PA(1)
#define CONFIG_GPIO_PRE_TEST		GPIO_PE(10)
#define CONFIG_GPIO_USB_DETECT_ENLEVEL	0
#elif defined(CONFIG_NEWTON2)
#define CONFIG_GPIO_USB_DETECT		GPIO_PE(10)
#define CONFIG_GPIO_USB_DETECT_ENLEVEL	0
#else
#undef CONFIG_GPIO_USB_DETECT
#undef CONFIG_GPIO_USB_DETECT_ENLEVEL
#endif

/* Pretest keys. */
#if  defined(CONFIG_X3) || defined(CONFIG_AW808) || defined(CONFIG_SWIMBOT)
#define CONFIG_GPIO_RECOVERY		GPIO_PE(10)      	/* pretest key */
#define CONFIG_GPIO_RECOVERY_ENLEVEL	1
#else
#define CONFIG_GPIO_RECOVERY		GPIO_PC(22)
#define CONFIG_GPIO_RECOVERY_ENLEVEL	 0
#endif

/* Wrong keys. */
#define CONFIG_GPIO_FASTBOOT		(-1)
#define CONFIG_GPIO_FASTBOOT_ENLEVEL	 0

/*
#define CONFIG_GPIO_MENU		CONFIG_GPIO_FASTBOOT
#define CONFIG_GPIO_MENU_ENLEVEL	CONFIG_GPIO_FASTBOOT_ENLEVEL
*/

/*#define CONFIG_GPIO_VOL_SUB		GPIO_PD(17)*/	/* SW9 */
/*#define CONFIG_GPIO_VOL_SUB_ENLEVEL	1

#define CONFIG_GPIO_VOL_ADD		GPIO_PD(18)*/	/* SW8 */
/*#define CONFIG_GPIO_VOL_ADD_ENLEVEL	1

#define CONFIG_GPIO_BACK		GPIO_PD(19)	*//* SW7 */
/*#define CONFIG_GPIO_BACK_ENLEVEL	0*/

#define CONFIG_GPIO_PWR_WAKE		GPIO_PA(30)
#define CONFIG_GPIO_PWR_WAKE_ENLEVEL	0

/*#define CONFIG_GPIO_DC_DETECT         GPIO_PB(1)
#define CONFIG_GPIO_DC_DETECT_ENLEVEL   0
*/
/* TEST
#define CONFIG_GPIO_DC_DETECT           GPIO_PG(10)
#define CONFIG_GPIO_DC_DETECT_ENLEVEL   1

#define CONFIG_GPIO_CHARGE_DETECT               GPIO_PG(12)
#define CONFIG_GPIO_CHARGE_DETECT_ENLEVEL       0
*/
#if defined(CONFIG_SENSORS_PIXART_PAH8001)
#if defined(CONFIG_IN901)
#define GPIO_PAH8001_INT        GPIO_PA(10)
#define GPIO_PAH8001_RESET      GPIO_PA(11)
#endif
#endif

/* ***************************GPIO VIBRATOR ***************************** */
#if defined(CONFIG_VIBRATE_GPIO)
#define	VIBRATOR_EN	GPIO_PE(2)
#define	ACTIVE_LEVEL	0
#endif
/* ***************************GPIO VIBRATOR ***************************** */

/* ***************************DRV2605 VIBRATOR START ******************** */
#if defined(CONFIG_VIBRATE_DRV2605)
#define DRV2605_ENABLE          GPIO_PE(2)
#define DRV2605_ACTIVE_LEVEL    1
#define DRV2605_I2C_ADDR        0x5a
#endif
/* ***************************DRV2605 VIBRATOR END ********************** */

#endif /* __CONFIG_WATCH_H__ */