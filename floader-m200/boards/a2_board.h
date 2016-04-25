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


#ifndef A2_BOARD_H
#define A2_BOARD_H

/*
 * ===================================================================
 * || !!! IMPORTANT !!!                                             ||
 * ===================================================================
 * Xtal freq fix set at 24MHz
 * ===================================================================
 */

/*
 *  Debug build configuration
 */
//#define DEBUG

/*
 * Config function of floader
 *
 * 0 --- For bootloader of Android system
 * 1 --- For bootloader of U-boot
 * 2 --- For bootloader of Burner
 */
#define CONFIG_FLOADER_FUNCTION                             0

/*
 * EMCP selection here
 */
#include "../emcp-lpddr2/emcp-4-4.h"

/*
 * EMMC MSC0 bus enable internal pull-up configuration
 */
#define CONFIG_EMMC_MSC0_PORT_PA_ENABLE_PULL_UP             0


/*
 * Console UART port
 */
#define CONFIG_CONSOLE_UART_NO                              3

/*
 * Boot arguments
 */
#define CONFIG_KERNEL_ARG_CONSOLE_BAUD_RATE                 2000000
#define CONFIG_KERNEL_ARG_CONSOLE   "console=ttyS3"
#define CONFIG_KERNEL_ARG_MEM       "mem=250M@0x0 mem=256M@0x30000000"
#define CONFIG_KERNEL_ARG_ROOTFS    "root=/dev/ram0 rw rdinit=/init"
#define CONFIG_KERNEL_MISC          "androidboot.hardware=watch ip=off"

/*
 * PMU I2C GPIO location
 */
#define CONFIG_PMU_GPIO_I2C_SCL                             GPIO_PD(31)
#define CONFIG_PMU_GPIO_I2C_SDA                             GPIO_PD(30)

/*
 * PMU Sleep pin
 *
 * undef for disable the function
 */
#define CONFIG_PMU_GPIO_SLEEP                               GPIO_PA(1)


/*
 * Config vibrate GPIO & pin assert level
 *
 * undef for disable ths function
 */
#define CONFIG_VIBRATE_GPIO                                 GPIO_PE(2)
#define CONFIG_VIBRATE_GPIO_ASSERT_LEVEL                    1


#endif

