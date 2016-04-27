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


#include "./floader_m200.h"

static int pmu_sleep_low_power_mode_enabled = 0;
static int core_poweroff_mode_enabled = 0;

extern u8 sleep_lib_entry[];

__attribute__ ((__aligned__(4)))
struct sleep_context *context =
                    (struct sleep_context *)(0xafff8000 + 1024 * 16);

void enable_set_pmu_suspend_mode_voltage(
        int enable, unsigned int core_volt_mV, unsigned int ddr_vdd2_volt_mV)
{
    debug("Sleep lib: enable: %d for"
            " pmu suspend mode at voltage: %dmV for core,"
            " %dmV for DDR VDD2.\n", enable, core_volt_mV, ddr_vdd2_volt_mV);

#ifdef CONFIG_PMU_GPIO_SLEEP
    gpio_request(CONFIG_PMU_GPIO_SLEEP, "PMU sleep");
    gpio_direction_output(CONFIG_PMU_GPIO_SLEEP, 0);
#endif

    if (get_pmu() == SM5007) {
        pmu_sm5007_set_core_sleep_voltage(core_volt_mV);

    } else {
        pmu_5t619_set_core_sleep_voltage(core_volt_mV);
        pmu_5t619_set_ddr_vdd2_sleep_voltage(ddr_vdd2_volt_mV);
    }

    pmu_sleep_low_power_mode_enabled = enable;
}

void enable_sleep_poweroff_mode(int enable)
{
    debug("Sleep lib: enable: %d for core sleep poweroff.\n", enable);

    core_poweroff_mode_enabled = enable;
}

void restore_context(void)
{
    extern void restore_context_goto(void);
    restore_context_goto();
}

static unsigned int cpm_barrier(void)
{
    volatile unsigned int temp;
    temp = cpm_readl(CPM_OPCR);

    return temp;
}

static void resume(void)
{
    /*
     * Restore PMU status if settings enabled
     */
#ifdef CONFIG_PMU_GPIO_SLEEP
    if (pmu_sleep_low_power_mode_enabled) {
        gpio_set_value(CONFIG_PMU_GPIO_SLEEP, 0);

        if (get_pmu() == SM5007) {
            /*
             * Real delay is almost 350us
             */
            udelay(5);
        } else {
            /*
             * Real delay is almost 350us
             */
            udelay(5);
        }
    }
#endif

    debug("Wakeup Step 1 done.\n");

    /*
     * Restore clock
     */
    init_clk();

    /*
     * Enable all clock
     */
    cpm_writel(0, CPM_CLKGR);
    cpm_writel(0, CPM_CLKGR1);
    udelay(1);
    
    debug("Wakeup Step 2 done.\n");

    /*
     * Restore lpddr2
     */
    restore_lpddr2();

    debug("Wakeup Step 3 done.\n");

    /*
     * Restore CPM
     */

    cpm_writel(context->cpm_lcr, CPM_LCR);
    cpm_writel(context->cpm_opcr, CPM_OPCR);
    cpm_writel(context->cpm_pgr, CPM_PGR);
    cpm_writel(context->cpm_spcr0, CPM_SPCR0);
    cpm_writel(context->cpm_usbpcr, CPM_USBPCR);

    cpm_writel(context->cpm_clkgr, CPM_CLKGR);
    cpm_writel(context->cpm_clkgr1, CPM_CLKGR1);

    cpm_barrier();

    debug("Wakeup Step 4 done.\n");

    /*
     * Like nothing happened...
     */
    restore_context();
}

static void resume_goto()
{
    __asm__ volatile(
        ".set mips32\n\t"
        "move $29, %0\n\t"
        ".set mips32\n\t"
        :
        :"r" (sleep_lib_entry + 16 * 1024 - 4)
        :);

    __asm__ volatile(".set mips32\n\t"
             "jr %0\n\t"
             "nop\n\t"
             ".set mips32 \n\t" :: "r" (resume));
}

static void sleep(void)
{
    debug("Prepare sleep configuration...\n");

    /*
     * Step 1.
     ************************
     */
    disable_fpu();

    debug("Sleep step 1 done\n");

    /*
     * Step 2.
     ************************
     * For CPM LCR
     */
    volatile unsigned int temp;

    /*
     * Save open all clock
     */
    context->cpm_clkgr = cpm_readl(CPM_CLKGR);
    context->cpm_clkgr1 = cpm_readl(CPM_CLKGR1);
    cpm_writel(0x0, CPM_CLKGR);
    cpm_writel(0x0, CPM_CLKGR1);
    udelay(1);

    temp = cpm_readl(CPM_LCR);
    context->cpm_lcr = temp;
    /*
     * Enter sleep status, shut down P1, L2, VPU, GPU, ISP, H2D
     */
    temp = (0x1f << 26) | (0xf << 8) | (0x1 << 7) | 0x1;
    cpm_writel(temp, CPM_LCR);

    /*
     * For CPM USBPCR
     */
    context->cpm_usbpcr = cpm_readl(CPM_USBPCR);
    temp = context->cpm_usbpcr;
    temp |= (1 << 25) | (1 << 20);
    cpm_writel(temp, CPM_USBPCR);

    debug("Sleep step 2 done\n");

    /*
     * Step 3.
     ************************
     * For CPM OPCR and Core resume address
     */
    temp = cpm_readl(CPM_OPCR);
    context->cpm_opcr = temp;
    /*
     * Disable otg, uhc, extoscillator
     */
    temp &= ~((1 << 7) | (1 << 6) | (1 << 4) | (3 << 25));
    /*
     * extoscillator state wait time,
     * mask interrupt,
     * use RTC clock,
     * L2 power down
     * RTC clock enable
     */
    temp |= (0xf << 8) | (1 << 30) | (1 << 2) | (1 << 27) | (1 << 23);
    /*
     * Core power down
     */
    unsigned int cpu_no = __read_32bit_c0_register($15, 1) & 0x1;
    temp |= ((cpu_no + 1) << 25);
    cpm_writel(temp, CPM_OPCR);
    /*
     * Set core resume address
     */
    temp = get_smp_ctrl();
    temp &= ~(0x7 << 8);
    temp |= (1 << (cpu_no + 8));
    set_smp_ctrl(temp);

    temp = get_smp_reim();
    temp &= ~(0xffff << 16);
    /*
     * High half address
     */
    set_smp_reim((unsigned int)resume_goto & (0xffff << 16));
    /*
     * Low half address
     */
    __write_32bit_c0_register($12, 7, (unsigned int)resume_goto & 0xffff);

    debug("Sleep step 3 done\n");

    /*
     * Step 4.
     ************************
     * For CPM SPCR0
     */
    context->cpm_spcr0 = cpm_readl(CPM_SPCR0);
    temp = (0x7f << 21)         /* NFI,AES,SS1,PCM,SSI2,I2S,TCU     */
            | (0x1f << 16)      /* UART4~0                          */
            | (0x1f << 10)      /* UHC,MSC2,MSC1,OTG0,LCD           */
            | (0x7f << 3)       /* EPD,DSI,DES,SADC,SSI0,MSC0,BCH   */
            | (0x1 << 1);       /* NEMC                             */
    /*
     * Enable UART3 if DEBUG
     */
#ifdef DEBUG
    temp &= ~(1 << 19);                           /* UART3 */
#endif

    cpm_writel(temp, CPM_SPCR0);

    debug("Sleep step 4 done\n");

    /*
     * Step 5.
     ************************
     * For CPM PGR
     */
    context->cpm_pgr = cpm_readl(CPM_PGR) & 0xffff;
    temp = 0;
    temp |= (0x3 << 9)          /* BCH,HASH */
                | (0x3 << 3);   /* USB,UHC */
    cpm_writel(temp, CPM_PGR);

    debug("Sleep step 5 done\n");

    /*
     * Step 6.
     ************************
     * For CPM CLKGR0 & CLKGR1
     */

    /*
     * CLKGR
     */
    temp = (0x1 << 30)     /* TCU                                   */
            | (0x7f << 22) /* DES,PCM,DSI,CSI,LCD,ISP,UHC           */
            | (0x3 << 19)  /* SSI2,SSI1                             */
            | (0x1f << 14) /* UART4~0                               */
            | 0x3fff;      /* SADC,MSC2,AIC,SMB3~0,SSI0,MSC1~0,OTG,BCH,NEMC,NFI */
    /*
     * Enable UART3 if DEBUG
     */
#ifdef DEBUG
    temp &= ~(0x1 << 17);                         /* UART3 */
#endif

    cpm_writel(temp, CPM_CLKGR);

    /*
     * CLKGR1
     */
    temp = (0x7 << 11)      /* DLINE,TCU_EXCLK,SYS_OST              */
            | (0x1ff << 0); /* P1,DMIC,HASH,AES,EPD,AHB_MON,IPU,GPU,VPU */
    cpm_writel(temp, CPM_CLKGR1);

    debug("Sleep step 6 done\n");

    /*
     * Step 7.
     ************************
     * Clear CPM RSR
     */
    cpm_writel(0, CPM_RSR);

    debug("Sleep step 7 done\n");

    /*
     * Step 8.
     ************************
     */
    prepare_lpddr2_for_sleep();

    debug("Sleep step 8 done\n");

    /*
     * Step 9.
     ************************
     */
    reset_clk_tree();

    /*
     * Here gating clk of APB0
     */
    cpm_writel(cpm_readl(CPM_CLKGR1) | (0x1 << 14), CPM_CLKGR1);

    debug("Sleep step 9 done\n");

    /*
     * Step 10.
     ************************
     * Set PMU in sleep status if settings enabled
     */
#ifdef CONFIG_PMU_GPIO_SLEEP
    if (pmu_sleep_low_power_mode_enabled) {
        gpio_set_value(CONFIG_PMU_GPIO_SLEEP, 1);
    }
#endif

    /*
     * Barriers all all all...
     */
    cpm_barrier();

    debug("Sleep done.\n");

    /*
     * The End
     ************************
     * We are going to sleep...
     */
    __asm__ volatile(".set mips32\n\t"
                     "wait\n\t");
}

static void sleep_goto()
{
    __asm__ volatile(
        ".set mips32\n\t"
        "move $29, %0\n\t"
        ".set mips32\n\t"
        :
        :"r" (sleep_lib_entry + 16 * 1024 - 4)
        :);

    __asm__ volatile(".set mips32\n\t"
             "jr %0\n\t"
             "nop\n\t"
             ".set mips32 \n\t" :: "r" (sleep));
}

int enter_sleep(int state)
{
    flush_cache_all();
    flush_scache_all();

    debug("Go to sleep...\n");

    extern void save_context_goto(void *address);
    save_context_goto(sleep_goto);

    flush_cache_all();

#ifdef DEBUG
    static u32 sleep_times = 0;
    debug("Sleep back, times: %u\n", ++sleep_times);
#endif

    return 0;
}

