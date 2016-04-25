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

static void dump_register()
{
    debug("=====================\n");
    debug("Dump CPM:\n");
    debug("=====================\n");
    debug("CPM_LCR     = 0x%08x\n", cpm_readl(CPM_LCR));
    debug("CPM_PGR     = 0x%08x\n", cpm_readl(CPM_PGR));
    debug("CPM_CLKGR   = 0x%08x\n", cpm_readl(CPM_CLKGR));
    debug("CPM_OPCR    = 0x%08x\n", cpm_readl(CPM_OPCR));
    debug("CPM_SPCR0   = 0x%08x\n", cpm_readl(CPM_SPCR0));
    debug("=====================\n");
}

static unsigned int apll_freq =
        (CONFIG_CPU_AND_L2_CLOCK_SOURCE_SEL == MPLL
                && CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL == MPLL) ?
                                                    600 : CONFIG_APLL_FREQ;

static unsigned int mpll_freq =
        (CONFIG_CPU_AND_L2_CLOCK_SOURCE_SEL == APLL
                && CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL == APLL) ?
                                                    0 : CONFIG_MPLL_FREQ;

static unsigned char ddr_freq_div = CONFIG_DDR_FREQ_DIV;

unsigned int get_ddr_freq()
{
    u32 freq = CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL == MPLL ?
                    mpll_freq / ddr_freq_div : apll_freq / ddr_freq_div;

    return freq * 1000 * 1000;
}

static void wait_pll_stable()
{
    debug("CPM_CPPCR = 0x%x\n", cpm_readl(CPM_CPPCR));

    /*
     * Wait APLL
     */
    while(!(cpm_readl(CPM_CPAPCR) & (0x1 << 3)))
        continue;

    debug("CPM_CPAPCR = 0x%x\n", cpm_readl(CPM_CPAPCR));

    /*
     * Wait MPLL
     */
    while(!(cpm_readl(CPM_CPMPCR) & (0x1 << 3)))
        continue;

    debug("CPM_CPMPCR = 0x%x\n", cpm_readl(CPM_CPMPCR));
}

void reset_clk_tree()
{
    unsigned int reg;
    unsigned int cpccr_cfg = 0x95000000;

    cpccr_cfg |= ((1 - 1) << 16)    /* PDIV                   */
                | ((1 - 1) << 12)   /* H2DIV                  */
                | ((1 - 1) << 8)    /* H0DIV                  */
                | ((1 - 1) << 4)    /* L2DIV                  */
                | (1 - 1);          /* CDIV                   */

    /* Change sel */
    reg = (cpccr_cfg & (0xff << 24)) | (cpm_readl(CPM_CPCCR) & ~(0xff << 24));
    cpm_writel(reg, CPM_CPCCR);

    /* Change div */
    reg = (cpm_readl(CPM_CPCCR) & (0xff << 24))
                                        | (cpccr_cfg & ~(0xff << 24))
                                        | (7 << 20);
    cpm_writel(reg, CPM_CPCCR);
    while(cpm_readl(CPM_CPCSR) & 0x7)
        continue;
    /*
     * xxdelay can use now
     */
    extern unsigned int __cpu_freq__;
    __cpu_freq__ = 24;
}

static void init_clk_tree()
{
    unsigned int pdiv, h0div, h2div, l2div, cdiv;

    l2div = CONFIG_L2_CACHE_CLK_DIV;
    h0div = CONFIG_AHB_CLK_DIV;
    h2div = h0div;

    pdiv = h2div;
    if (CONFIG_AHB_CLK_IS_TWO_TIMES_OF_APB_CLK == 1)
        pdiv = h2div * 2;

    cdiv = 1;

    u8 soc_clk_sel = CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL == MPLL ? 0x2 : 0x1;
    u8 cpu_clk_sel = CONFIG_CPU_AND_L2_CLOCK_SOURCE_SEL == MPLL ? 0x2 : 0x1;

    unsigned int cpccr_cfg = (0x1 << 30)            /* SCLK_A: APLL           */
                    | (cpu_clk_sel << 28)           /* Core & L2: cpu_clk_sel */
                    | (soc_clk_sel << 26)           /* H0: soc_clk_sel        */
                    | (soc_clk_sel << 24)           /* H2: soc_clk_sel        */
                    | ((pdiv - 1) << 16)            /* PDIV                   */
                    | ((h2div - 1) << 12)           /* H2DIV                  */
                    | ((h0div - 1) << 8)            /* H0DIV                  */
                    | ((l2div - 1) << 4)            /* L2DIV                  */
                    | (cdiv - 1)                    /* CDIV                   */
                    | (0x7 << 20);

    /* Change div */
    unsigned int reg = (cpm_readl(CPM_CPCCR) & (0xff << 24))
                                        | (cpccr_cfg & ~(0xff << 24))
                                        | (7 << 20);
    cpm_writel(reg, CPM_CPCCR);
    while(cpm_readl(CPM_CPCSR) & 0x7)
        continue;

    /* Change sel */
    reg = (cpccr_cfg & (0xff << 24)) | (cpm_readl(CPM_CPCCR) & ~(0xff << 24));
    cpm_writel(reg, CPM_CPCCR);

    debug("CPM_CPCCR = 0x%x\n",cpm_readl(CPM_CPCCR));
}

static void try_stop_sclka()
{
    if(CONFIG_CPU_AND_L2_CLOCK_SOURCE_SEL == MPLL
            && CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL == MPLL) {

        cpm_writel((cpm_readl(CPM_CPCCR) & ~(0x3 << 30)) | (1 << 23), CPM_CPCCR);
    } else if (CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL == MPLL) {

        cpm_writel(cpm_readl(CPM_CPCCR) | (1 << 23), CPM_CPCCR);
    }
}

void init_clk()
{
    dump_register();

    wait_pll_stable();
    init_clk_tree();
    try_stop_sclka();
}

