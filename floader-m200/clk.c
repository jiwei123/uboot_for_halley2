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


#define SRC_EOF -1

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


static struct {
    unsigned int freq;
    unsigned int m;
    unsigned int n;
    unsigned int od0;
    unsigned int od1;
} apll_cfg, mpll_cfg;

static unsigned int cpu_freq =
        CONFIG_CPU_AND_L2_CLOCK_SOURCE_SEL == MPLL ? CONFIG_MPLL_FREQ : CONFIG_APLL_FREQ;

static unsigned int apll_freq =
        (CONFIG_CPU_AND_L2_CLOCK_SOURCE_SEL == MPLL
                && CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL == MPLL) ?
                                                    600 : CONFIG_APLL_FREQ;

static unsigned int mpll_freq =
        (CONFIG_CPU_AND_L2_CLOCK_SOURCE_SEL == APLL
                && CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL == APLL) ?
                                                    0 : CONFIG_MPLL_FREQ;

static unsigned char ddr_freq_div = CONFIG_DDR_FREQ_DIV;

struct cgu {
    u32 en:8;
    u32 off:8;
    u32 sel_bit:8;
    u32 sel_src:8;
    u8  sel[4];
    u32 ce;
    u32 busy;
    u32 stop;
    s32 clkgr_no;
    u32 clkgr_bit;
};

#if (CONFIG_FLOADER_FUNCTION == 0)

    struct cgu cgu_clk_sel[CGU_CNT] = {
        [DDR] =  {1, CPM_DDRCDR,  30, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {0,     APLL, MPLL, -1},      29, 28, 27, -1, -1},
        [MSC] =  {1, CPM_MSC0CDR, 31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,   -1},      29, 28, 27, 0, 4},
        [BCH] =  {0, CPM_BCHCDR,  31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,   -1},      29, 28, 27, 0, 2},
        [VPU] =  {0, CPM_VPUCDR,  31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,   -1},      29, 28, 27, 1, 0},
        [OTG] =  {0, CPM_USBCDR,  30, EXCLK,                                  {EXCLK, EXCLK, APLL, MPLL},   29, 28, 27, 0, 3},
        [I2S] =  {0, CPM_I2SCDR,  30, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL,  EXCLK,-1},     29, 28, 27, -1, -1},
        [LCD] =  {0, CPM_LPCDR,   31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,    -1},     28, 27, 26, 0, 24},
        [MSC1] = {0, CPM_MSC1CDR, 0,  0,                                      {-1,    -1,   -1,    -1},     29, 28, 27, 0, 5},
        [MSC2] = {0, CPM_MSC2CDR, 0,  0,                                      {-1,    -1,   -1,    -1},     29, 28, 27, 0, 4},
        [UHC] =  {0, CPM_UHCCDR,  30, OTG,                                    {APLL,  MPLL, OTG,   -1},     29, 28, 27, 0, 22},
        [SSI] =  {0, CPM_SSICDR,  30, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, EXCLK, -1},     29, 28, 27, 0, 6},
        [CIM] =  {0, CPM_CIMCDR,  31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,    -1},     30, 29, 28, -1, -1},
        [PCM] =  {0, CPM_PCMCDR,  30, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, EXCLK, -1},     28, 27, 26, 0, 27},
        [GPU] =  {0, CPM_GPUCDR,  31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,    -1},     29, 28, 27, 1, 1},
        [ISP] =  {0, CPM_ISPCDR,  31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,    -1},     29, 28, 27, 0, 23},
    };

#else

    struct cgu cgu_clk_sel[CGU_CNT] = {
        [DDR] =  {1, CPM_DDRCDR,  30, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {0,     APLL, MPLL, -1},      29, 28, 27, -1, -1},
        [MSC] =  {1, CPM_MSC0CDR, 31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,   -1},      29, 28, 27, 0, 4},
        [BCH] =  {0, CPM_BCHCDR,  31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,   -1},      29, 28, 27, 0, 2},
        [VPU] =  {0, CPM_VPUCDR,  31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,   -1},      29, 28, 27, 1, 0},
        [OTG] =  {1, CPM_USBCDR,  30, EXCLK,                                  {EXCLK, EXCLK, APLL, MPLL},   29, 28, 27, 0, 3},
        [I2S] =  {0, CPM_I2SCDR,  30, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL,  EXCLK,-1},     29, 28, 27, -1, -1},
        [LCD] =  {1, CPM_LPCDR,   31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,    -1},     28, 27, 26, 0, 24},
        [MSC1] = {0, CPM_MSC1CDR, 0,  0,                                      {-1,    -1,   -1,    -1},     29, 28, 27, 0, 5},
        [MSC2] = {0, CPM_MSC2CDR, 0,  0,                                      {-1,    -1,   -1,    -1},     29, 28, 27, 0, 4},
        [UHC] =  {0, CPM_UHCCDR,  30, OTG,                                    {APLL,  MPLL, OTG,   -1},     29, 28, 27, 0, 22},
        [SSI] =  {0, CPM_SSICDR,  30, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, EXCLK, -1},     29, 28, 27, 0, 6},
        [CIM] =  {0, CPM_CIMCDR,  31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,    -1},     30, 29, 28, -1, -1},
        [PCM] =  {0, CPM_PCMCDR,  30, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, EXCLK, -1},     28, 27, 26, 0, 27},
        [GPU] =  {0, CPM_GPUCDR,  31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,    -1},     29, 28, 27, 1, 1},
        [ISP] =  {0, CPM_ISPCDR,  31, CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL, {APLL,  MPLL, -1,    -1},     29, 28, 27, 0, 23},
    };

#endif

static void stop_clock()
{
    for (int id = 0; id < CGU_CNT; id++) {
        struct cgu *cgu = cgu_clk_sel + id;

        unsigned int reg = CPM_BASE + cgu->off;
        unsigned int regval = readl(reg);
        /*
         * Set div max
         */
        regval |= 0xff | (1 << cgu->ce);
        while (readl(reg) & (1 << cgu->busy))
            continue;

        writel(regval, reg);

        while (readl(reg) & (1 << cgu->busy))
            continue;

        /*
         * Stop clock
         */
        regval = readl(reg);
        regval |= ((1 << cgu->stop) | (1 << cgu->ce));
        writel(regval, reg);

        while (readl(reg) & (1 << cgu->busy))
            continue;

        /*
         * Clear ce bit
         */
        regval = readl(reg);
        regval &= ~(1 << cgu->ce);
        writel(regval, reg);

        /*
         * Disable clk gate
         */
        if (cgu->clkgr_no != -1) {
            if (cgu->clkgr_no == 0) {
                cpm_writel(
                    cpm_readl(CPM_CLKGR) | (1 << cgu->clkgr_bit),
                    CPM_CLKGR);
            } else {
                cpm_writel(
                    cpm_readl(CPM_CLKGR1) | (1 << cgu->clkgr_bit),
                    CPM_CLKGR1);
            }
        }
    }
}

static void start_clock()
{
    /*
     * Set devices clock mxu
     */
    int size = sizeof(cgu_clk_sel) / sizeof(struct cgu);
    for(int i = 0; i < size; i++) {
        for (int j = 0; j < 4; j++) {
            if (cgu_clk_sel[i].sel_src == cgu_clk_sel[i].sel[j]) {

                /*
                 * Enable clk gate
                 */
                if (cgu_clk_sel[i].en
                        && cgu_clk_sel[i].clkgr_no != -1) {
                    if (cgu_clk_sel[i].clkgr_no == 0) {
                        cpm_writel(
                            cpm_readl(CPM_CLKGR) & ~(1 << cgu_clk_sel[i].clkgr_bit),
                            CPM_CLKGR);
                    } else {
                        cpm_writel(
                            cpm_readl(CPM_CLKGR1) & ~(1 << cgu_clk_sel[i].clkgr_bit),
                            CPM_CLKGR1);
                    }

                    if (i == LCD) {
                        /*
                         * Also enable DSI, CSI
                         */
                        cpm_writel(
                            cpm_readl(CPM_CLKGR) & ~(0x3 << 25),
                            CPM_CLKGR);
                    }
                }

                /*
                 * Enable clk cgu
                 */
                unsigned int addr = CPM_BASE + cgu_clk_sel[i].off;
                unsigned int reg = readl(addr);

                reg &= ~(3 << 30);
                reg |= j << cgu_clk_sel[i].sel_bit;
                reg |= 1 << cgu_clk_sel[i].ce;
                reg |= !cgu_clk_sel[i].en << cgu_clk_sel[i].stop;

                debug("CGU%d = 0x%x\n", i, reg);

                writel(reg, addr);
                while (readl(addr) & (1 << cgu_clk_sel[i].busy))
                    continue;

                break;
            }
        }
    }

    /* Enable PDMA */
    unsigned int reg = cpm_readl(CPM_SPCR0);
    reg &= ~((1 << 31) | (1 << 15));
    cpm_writel(reg, CPM_SPCR0);


    reg = cpm_readl(CPM_CLKGR);
    reg &= ~(0x1 << 21);
    cpm_writel(reg, CPM_CLKGR);
}

static void init_pll_cfg()
{
    /*
     * set APLL
     * set Xtal 24MHz
     */
    apll_cfg.freq = apll_freq;
    apll_cfg.m = apll_freq / 24;
    apll_cfg.n = 1;
    apll_cfg.od0 = 1;
    apll_cfg.od1 = 1;

    /*
     * set MPLL
     * set Xtal 24MHz
     */
    mpll_cfg.freq = mpll_freq;
    mpll_cfg.m = mpll_freq / 24;
    mpll_cfg.n = 1;
    mpll_cfg.od0 = 1;
    mpll_cfg.od1 = 1;
}

static void init_wait_pll_stable()
{
    /*
     * Set PLL stabilize time to 0
     */
    cpm_writel(cpm_readl(CPM_CPPCR) & ~0xff, CPM_CPPCR);

    debug("CPM_CPPCR = 0x%x\n", cpm_readl(CPM_CPPCR));

    /*
     * Set APLL
     */
    unsigned int reg;

    reg = (apll_cfg.m << 20)
                | (apll_cfg.n << 14)
                | (apll_cfg.od0 << 8)
                | (apll_cfg.od1 << 11)
                | 0x1;

    cpm_writel(reg, CPM_CPAPCR);
    while(!(cpm_readl(CPM_CPAPCR) & (0x1 << 3)))
        continue;

    debug("CPM_CPAPCR = 0x%x\n", cpm_readl(CPM_CPAPCR));

    /*
     * Set MPLL
     */
    if (mpll_freq) {
        reg = (mpll_cfg.m << 20)
                    | (mpll_cfg.n << 14)
                    | (mpll_cfg.od0 << 8)
                    | (mpll_cfg.od1 << 11)
                    | 0x1;
    } else {
        reg = cpm_readl(CPM_CPMPCR) | 0x1;
    }

    cpm_writel(reg, CPM_CPMPCR);
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
    unsigned int reg;

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
    reg = (cpm_readl(CPM_CPCCR) & (0xff << 24))
                                    | (cpccr_cfg & ~(0xff << 24))
                                    | (7 << 20);
    cpm_writel(reg, CPM_CPCCR);
    while(cpm_readl(CPM_CPCSR) & 0x7)
        continue;

    /* Change sel */
    reg = (cpccr_cfg & (0xff << 24)) | (cpm_readl(CPM_CPCCR) & ~(0xff << 24));
    cpm_writel(reg, CPM_CPCCR);

    debug("CPM_CPCCR = 0x%x\n",cpm_readl(CPM_CPCCR));

    /* Set CPM_LCR */
    reg = cpm_readl(CPM_LCR);
    reg &= ~0x7;
    reg |= (0xff << 8);
    reg |= (0x1 << 2);
    cpm_writel(reg, CPM_LCR);

    debug("CPM_LCR = 0x%x\n",cpm_readl(CPM_LCR));

    /* Set CPM_OPCR */
    reg = cpm_readl(CPM_OPCR);
    reg &= ~(0x2 << 25);
    reg |= (0xff << 8);
    cpm_writel(reg, CPM_OPCR);

    debug("CPM_OPCR = 0x%x\n",cpm_readl(CPM_OPCR));
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

static void set_ddr_clock()
{
    /*
     * Set DDR freq at pll / ddr_freq_div
     */
    unsigned int div = ddr_freq_div - 1;

    unsigned int reg = cpm_readl(CPM_DDRCDR);
    reg &= ~(0xf | (0x3f << 24));
    reg |= (1 << 29) | div;

    cpm_writel(reg, CPM_DDRCDR);

    while (cpm_readl(CPM_DDRCDR) & (1 << 28))
        continue;

    debug("CPM_DDRCDR = 0x%x\n", cpm_readl(CPM_DDRCDR));
}

unsigned int get_ddr_freq()
{
    u32 freq = CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL == MPLL ?
                    mpll_freq / ddr_freq_div : apll_freq / ddr_freq_div;

    return freq * 1000 * 1000;
}

static void init_pll(void)
{
    init_pll_cfg();
    reset_clk_tree();
    init_wait_pll_stable();

    init_clk_tree();

    /*
     * xxdelay can use now
     */
    extern unsigned int __cpu_freq__;
    __cpu_freq__ = apll_cfg.freq;
}

void set_msc0_freq(unsigned int freq)
{
    unsigned int div = 0xff;

    if (freq) {
        div = ((mpll_freq ? mpll_freq : cpu_freq)
                    * 1000 * 1000 + freq - 1) / freq / 2 - 1;
    }

    if (div >= 0xff)
        div = 0xff;

    unsigned int reg = cpm_readl(CPM_MSC0CDR);

    reg = cpm_readl(CPM_MSC0CDR);
    reg &= ~((0x3 << 27) | 0xff);
    reg |= (1 << 29) | div;

    cpm_writel(reg, CPM_MSC0CDR);
    while (cpm_readl(CPM_MSC0CDR) & (1 << 28))
        continue;

    debug("CPM_MSC0CDR = 0x%x\n", cpm_readl(CPM_MSC0CDR));
}

unsigned int get_msc0_freq()
{
    unsigned int div = ((cpm_readl(CPM_MSC0CDR) & 0xff) + 1) * 2;

    return (mpll_freq ? mpll_freq : cpu_freq) / div * 1000 * 1000;
}

void enable_clock_uart3()
{
    unsigned int reg = cpm_readl(CPM_CLKGR) & ~CPM_CLKGR_UART3;
    cpm_writel(reg, CPM_CLKGR);
}

void init_clk()
{
    stop_clock();
    init_pll();
    start_clock();
    set_ddr_clock();
    try_stop_sclka();

    dump_register();
}

