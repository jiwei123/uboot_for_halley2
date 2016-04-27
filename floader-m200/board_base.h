/*
 * Copyright (C) 2016 Ingenic Semiconductor
 *
 * SunWenZhong(Fighter) <wenzhong.sun@ingenic.com, wanmyqawdr@126.com>
 *
 * For project-5
 *
 * Release under GPLv2
 *
 */


#ifndef BOARD_BASE_H
#define BOARD_BASE_H

/*
 * Config function of floader
 *
 * 0 --- For bootloader of Android system
 * 1 --- For bootloader of U-boot
 * 2 --- For bootloader of Burner
 */
#define CONFIG_FLOADER_FUNCTION                             1

/*
 * When CONFIG_FLOADER_FUNCTION=1, following configuration will be consider
 */
#define CONFIG_UBOOT_IMG_LOAD_ADDRESS_FROM_LPDDR2           0x80100000
#define CONFIG_UBOOT_IMG_LOAD_ADDRESS_FROM_EMMC             0x12c00     // 75KBytes
#define CONFIG_UBOOT_IMG_SIZE                               0x100000    // 1MBytes
#define CONFIG_UBOOT_IMG_ENTRY_ADDRESS_FROM_LPDDR2          0x80100000

/*
 * APLL freq configuration
 */
#define CONFIG_APLL_FREQ                                    1032

/*
 * MPLL freq configuration
 */
#define CONFIG_MPLL_FREQ                                    816

/*
 * CPU and L2 cache clock source configuration
 *
 * <APLL or MPLL>
 */
#define CONFIG_CPU_AND_L2_CLOCK_SOURCE_SEL                  APLL

/*
 * LPDDR2 and SoC clock source configuration
 *
 * <APLL or MPLL>
 */
#define CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL              MPLL

/*
 * L2cache and SoC clk div configuration
 * To explain:
 *
 * if (CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL == MPLL) {
 *     clk_ahb0 = CONFIG_MPLL_FREQ / CONFIG_AHB_CLK_DIV;
 *     clk_ahb2 = clk_ahb0
 *
 *     if (CONFIG_AHB_CLK_IS_TWO_TIMES_OF_APB_CLK == 1)
 *         clk_apb  = clk_ahb2 / 2;
 *     else
 *         clk_apb = clk_ahb2;
 * } else {
 *     clk_ahb0 = CONFIG_APLL_FREQ / CONFIG_AHB_CLK_DIV;
 *     clk_ahb2 = clk_ahb0
 *
 *     if (CONFIG_AHB_CLK_IS_TWO_TIMES_OF_APB_CLK == 1)
 *         clk_apb  = clk_ahb2 / 2;
 *     else
 *         clk_apb = clk_ahb2;
 * }
 *
 * clk_l2cache = CONFIG_CPU_AND_L2_CLOCK_SOURCE_SEL
 *                                      / CONFIG_L2_CACHE_CLK_DIV
 *
 * Note 1: clk_l2cache must be about 1.5 times of clk_ahb0
 */
#define CONFIG_L2_CACHE_CLK_DIV                             3
#define CONFIG_AHB_CLK_DIV                                  4
#define CONFIG_AHB_CLK_IS_TWO_TIMES_OF_APB_CLK              1

/*
 * LPDDR2 freq divider configuration
 * To explain:
 *
 * if (CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL == MPLL) {
 *     clk_lpddr2 = CONFIG_MPLL_FREQ / CONFIG_DDR_FREQ_DIV;
 * } else {
 *     clk_lpddr2 = CONFIG_APLL_FREQ / CONFIG_DDR_FREQ_DIV;
 * }
 */
#define CONFIG_DDR_FREQ_DIV                                 3

/*
 * EMMC MSC0 bus width
 * Selection of <1bit, 4bit, 8bit>
 */
#define CONFIG_EMMC_MSC0_BUS_WIDTH                          8

/*
 * EMMC MSC0 bus freq MHz
 */
#define CONFIG_EMMC_MSC0_FREQ_MHZ                           40

/*
 * EMMC block size(counted by byte)
 */
#define CONFIG_EMMC_MSCO_BLOCK_SIZE                         512

/*
 * Boot image load address from EMMC(counted by byte)
 */
#define CONFIG_BOOT_IMG_LOAD_ADDRESS_FROM_EMMC              0x300000

/*
 * Boot image load address of LPDDR2
 */
#define CONFIG_BOOT_IMG_LOAD_ADDRESS_FROM_LPDDR2            0x80f00000

/*
 * Boot image entry address of LPDDR2
 */
#define CONFIG_BOOT_IMG_ENTRY_ADDRESS_FROM_LPDDR2           0x80f00000

/*
 * Core & LPDDR2 voltage(mV) configuration
 */
#define CONFIG_CORE_VOLTAGE_MV                              1150
#define CONFIG_DDR_VDD2_VOLTAGE_MV                          1150

/*
 * Config sleep voltage
 */
#define CONFIG_SLEEP_VOLTAGE_ADJUST                         1
#define CONFIG_CORE_SLEEP_VOLTAGE_MV                        850
#define CONFIG_DDR_VDD2_SLEEP_VOLTAGE_MV                    1000

/*
 * Console UART port
 */
#define CONFIG_CONSOLE_UART_NO                              3

/*
 * Boot arguments
 */
#define CONFIG_KERNEL_ARG_CONSOLE   "console=ttyS3"
#define CONFIG_KERNEL_ARG_MEM       "mem=250M@0x0 mem=256M@0x30000000"
#define CONFIG_KERNEL_ARG_MEM_FOR_1024M "mem=250M@0x0 mem=768M@0x30000000"
#define CONFIG_KERNEL_ARG_ROOTFS    "root=/dev/ram0 rw rdinit=/init"
#define CONFIG_KERNEL_MISC          "androidboot.hardware=watch ip=off"

#endif
