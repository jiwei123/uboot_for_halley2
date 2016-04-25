/*
 * Copyright (C) 2016 Ingenic Semiconductor
 *
 * SunWenZhong(Fighter) <wenzhong.sun@ingenic.com, wanmyqawdr@126.com>
 *
 * MaWeiBin <weibin.ma@ingenic.com>
 *
 * Copyright (C) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: Zoro <ykli@ingenic.cn>
 *
 * For project-5
 *
 * Release under GPLv2
 *
 */


#include "./floader_m200.h"

extern int open (const char *__file, int __oflag, ...);
extern int close(int fd);
extern size_t write(int fd, const void *buf, size_t count);
#define O_RDWR          02
#define O_CREAT       0100



#undef CONFIG_APLL_FREQ
#undef CONFIG_MPLL_FREQ
#undef CONFIG_CPU_AND_L2_CLOCK_SOURCE_SEL
#undef CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL

/*
 * APLL freq configuration
 *
 */
#define CONFIG_APLL_FREQ                                    816

/*
 * MPLL freq configuration
 *
 */
#define CONFIG_MPLL_FREQ                                    600

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
#define CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL              APLL


static struct {
    unsigned int freq;
    unsigned int m;
    unsigned int n;
    unsigned int od0;
    unsigned int od1;
} apll_cfg, mpll_cfg;

struct desc {
    unsigned set_addr :16;
    unsigned poll_addr :16;
    unsigned value :32;
    unsigned poll_h_mask :32;
    unsigned poll_l_mask :32;
};

typedef union reg_cpccr {
    /** raw register data */
    uint32_t d32;
    /** register bits */
    struct {
        unsigned CDIV :4;
        unsigned L2CDIV :4;
        unsigned H0DIV :4;
        unsigned H2DIV :4;
        unsigned PDIV :4;
        unsigned CE_AHB2 :1;
        unsigned CE_AHB0 :1;
        unsigned CE_CPU :1;
        unsigned GATE_SCLKA :1;
        unsigned SEL_H2PLL :2;
        unsigned SEL_H0PLL :2;
        unsigned SEL_CPLL :2;
        unsigned SEL_SRC :2;
    } b;
} reg_cpccr_t;

typedef union nand_timing {
    /** raw register data */
    uint32_t nand_timing[4];
    /** register bits */
    struct {
        unsigned set_rw :8;
        unsigned wait_rw :8;
        unsigned hold_rw :8;
        unsigned set_cs :8;
        unsigned wait_cs :8;
        unsigned trr :8;
        unsigned tedo :8;
        unsigned trpre :8;
        unsigned twpre :8;
        unsigned tds :8;
        unsigned tdh :8;
        unsigned twpst :8;
        unsigned tdqsre :8;
        unsigned trhw :8;
        unsigned t1 :8;
        unsigned t2 :8;
    } b;
} nand_timing_t;

struct params {
    unsigned int id;
    unsigned int length;
    unsigned int pll_freq;
    reg_cpccr_t cpccr;
    nand_timing_t nand_timing;
    struct desc cpm_desc[0];
};

struct desc descriptors[14] = {
            /*
             * saddr,   paddr,      value,      poll_h_mask,    poll_l_mask
             */
#if 0
            { 0x20,     0xffff,     0x1fffffb4, 0,          0 },        /* gate clk */
            { 0x10,     0x10,       0x2904901,  0x8,        0 },        /* conf APLL */
            { 0x14,     0x14,       0x1904901,  0x8,        0 },        /* conf MPLL */
            { 0,        0xd4,       0x95752230, 0,          0x7 },      /* conf DIV */
            { 0,        0xffff,     0x5a752230, 0,          0 },        /* conf select */
            { 0x68,     0x68,       0xa0000006, 0,          0x10000000 }, /* conf MSC0CDR */
            { 0x20,     0xffff,     0x1fffff80, 0,          0 },        /* ungate clk */
            { 0xffff,   0xffff,     0,          0,          0 },
#endif
};

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
static int desc_tail = 0;

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

static int desc_append(uint32_t saddr, uint32_t paddr, uint32_t value,
                                uint32_t poll_h_mask,  uint32_t poll_l_mask) {
    if (desc_tail <= sizeof(descriptors) / sizeof(struct desc)) {
        descriptors[desc_tail].set_addr = saddr;
        descriptors[desc_tail].poll_addr = paddr;
        descriptors[desc_tail].value = value;
        descriptors[desc_tail].poll_h_mask = poll_h_mask;
        descriptors[desc_tail].poll_l_mask = poll_l_mask;
        desc_tail++;
        return 0;
    }
    else if(!((saddr == 0xffff) && (paddr == 0xffff))){
        printf("descriptors append error!!!\n");
        return -1;
    }
    return 0;
}

static void init_wait_pll_stable()
{
    unsigned int reg;

    reg = (apll_cfg.m << 20)
                | (apll_cfg.n << 14)
                | (apll_cfg.od0 << 8)
                | (apll_cfg.od1 << 11)
                | 0x1;
    desc_append(CPM_CPAPCR, CPM_CPAPCR, reg, 0x8, 0);
}

static void init_clk_tree()
{
    unsigned int pdiv, h0div, h2div, l2div, cdiv;

    pdiv = 8;
    h0div = 4;
    h2div = 4;
    cdiv = 1;
    l2div = 3;

    unsigned char soc_clk_sel = CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL == MPLL ? 0x2 : 0x1;
    unsigned char cpu_clk_sel = CONFIG_CPU_AND_L2_CLOCK_SOURCE_SEL == MPLL ? 0x2 : 0x1;

    unsigned int cpccr_cfg = (0x1 << 30)            /* SCLK_A: APLL      */
                    | (cpu_clk_sel << 28)           /* Core & L2: SCLK_A */
                    | (soc_clk_sel << 26)           /* H0: soc_clk_sel   */
                    | (soc_clk_sel << 24)           /* H2: soc_clk_sel   */
                    | ((pdiv - 1) << 16)            /* PDIV              */
                    | ((h2div - 1) << 12)           /* H2DIV             */
                    | ((h0div - 1) << 8)            /* H0DIV             */
                    | ((l2div - 1) << 4)            /* L2DIV             */
                    | (cdiv - 1)                    /* CDIV              */
                    | (0x7 << 20);

    /* Change div */
    unsigned int reg = (0x95 << 24) | (cpccr_cfg & ~(0xff << 24)) | (7 << 20);
    desc_append(CPM_CPCCR, CPM_CPCSR, reg, 0, 0x07);

    /* Change sel */
    reg = cpccr_cfg;
    desc_append(CPM_CPCCR, 0xffff, reg, 0, 0);
}

void set_msc0_freq(unsigned int freq)
{
    unsigned int div = 0xff;

    div = ((mpll_freq ? mpll_freq : cpu_freq)
                + freq - 1) / freq / 2 - 1;

    if (div >= 0xff)
        div = 0xff;

    unsigned int reg;
    reg  = (mpll_freq ? (1 << 31) : (0 << 31)) | (1 << 29) | div;

    desc_append(CPM_MSC0CDR, CPM_MSC0CDR, reg, 0, (0x1 << 28));
}

static void init_desc() {
    desc_tail = 0;

    desc_append(CPM_CLKGR, 0xffff, 0x1fffffb4, 0, 0);   /* gate clk */
    init_pll_cfg();
    init_wait_pll_stable();
    init_clk_tree();
    set_msc0_freq(CONFIG_EMMC_MSC0_FREQ_MHZ);
    desc_append(CPM_CLKGR, 0xffff, 0x1fffff80, 0, 0);   /* ungate clk */
    desc_append(0xffff, 0xffff, 0, 0, 0);               /* end of desc */
}

static void dump_params(struct params *p) {
    int i;

    printf("SPL Params Fixer:\n");
    printf("id:\t\t0x%08X (%c%c%c%c)\n", p->id,
            ((char *) (&p->id))[0],
            ((char *) (&p->id))[1],
            ((char *) (&p->id))[2],
            ((char *) (&p->id))[3]);
    printf("length:\t\t%u\n", p->length);
    printf("pll_freq:\t%u\n", p->pll_freq);
    printf("CPM_CPCCR:\t0x%08X\n", p->cpccr.d32);

    for (i = 0; i < 4; i++)
        printf("nand_timing[%d]:\t0x%08X\n", i, p->nand_timing.nand_timing[i]);

    printf("descriptors:\n");
    for (i = 0; i < 14; i++) {
        struct desc *desc = &p->cpm_desc[i];

        if ((desc->set_addr == 0xffff) && (desc->poll_addr = 0xffff))
            break;

        printf("NO.%d:\n", i);
        printf("\tsaddr = 0x%04X\n", desc->set_addr);
        printf("\tsaddr = 0x%04X\n", desc->poll_addr);
        printf("\tpaddr = 0x%08X\n", desc->value);
        printf("\tpoll_h_mask = 0x%08X\n", desc->poll_h_mask);
        printf("\tpoll_l_mask = 0x%08X\n", desc->poll_l_mask);
    }
}

int main(void) {
    char buffer[256];

    int fd, i;
    char *spl_path, *fix_file;
    unsigned int spl_length = 0;
    struct params *params;
    char valid_id[4] = { 'I', 'N', 'G', 'E' };
    struct desc *desc;
    unsigned int *p;

    memset(buffer, 0, 256);
    params = (struct params *) buffer;
    p = (unsigned int *) valid_id;
    params->id = *p;

    init_desc();

    params->length = 26 * 1024;

    params->pll_freq = 0;
    desc = params->cpm_desc;
    for (i = 0; i < 14; i++) {
        memcpy(&desc[i], &descriptors[i], sizeof(struct desc));
    }

    dump_params(params);

    fd = open("./spl_params.bin", O_RDWR | O_CREAT);
    if (fd < 0) {
        printf("open %s Error\n", fix_file);
        return -1;
    }

    if (write(fd, params, 256) != 256) {
        printf("write %s Error\n", spl_path);
        return -1;
    }

    close(fd);

    return 0;
}
