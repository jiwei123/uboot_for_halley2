/*
 * Copyright (C) 2016 Ingenic Semiconductor
 *
 * SunWenZhong(Fighter) <wenzhong.sun@ingenic.com, wanmyqawdr@126.com>
 *
 * Copyright (C) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: Zoro <ykli@ingenic.cn>
 *
 * For project-5
 *
 * Release under GPLv2
 *
 */


#include "../floader_m200.h"

#if (CONFIG_DDR_CS1 == 1)
    #ifndef DDR_ROW1
        #error "please define DDR_ROW1"
    #endif

    #ifndef DDR_COL1
        #error "please define DDR_COL1"
    #endif
#endif

#define BETWEEN(T, MIN, MAX) if (T < MIN) T = MIN; if (T > MAX) T = MAX
struct tck tck_g = {0, 0};

struct RL_LPDDR2 rl_LPDDR2[] = {
    {100000000, 3},/*memclk xxM, RL*/
    {150000000, 3},
    {200000000, 4},
    {300000000, 5},
    {400000000, 6},
    {450000000, 7},
    {500000000, 8},
};

struct WL_LPDDR2 wl_LPDDR2[]= {
    {100000000, 1},/*memclk xxM, WL*/
    {150000000, 1},
    {200000000, 2},
    {300000000, 2},
    {400000000, 3},
    {450000000, 4},
    {500000000, 4},
};

unsigned int get_ddr_freq()
{
    u32 freq = CONFIG_LPDDR2_AND_SOC_CLOCK_SOURCE_SEL == MPLL ?
                            CONFIG_MPLL_FREQ / CONFIG_DDR_FREQ_DIV
                                : CONFIG_APLL_FREQ / CONFIG_DDR_FREQ_DIV;
    return freq * 1000 * 1000;
}

static int calc_nck(int x, int y)
{
    int value;

    value = x * 1000 % y == 0 ? x * 1000 / y : x * 1000 / y + 1;

    return value;
}

static void caculate_tck(struct ddr_params *params)
{
    params->tck.ps = (1000000000 / (params->freq / 1000));
    params->tck.ns = (1000000000 % params->freq == 0)
        ? (1000000000 / params->freq)
        : (1000000000 / params->freq + 1);
    tck_g.ps = params->tck.ps;
    tck_g.ns = params->tck.ns;
}

unsigned int sdram_size(int cs, struct ddr_params *p)
{
    unsigned int dw;
    unsigned int banks;
    unsigned int size = 0;
    unsigned int row, col;

    switch (cs) {
    case 0:
        if (p->cs0 == 1) {
            row = p->row;
            col = p->col;
            break;
        } else
            return 0;
    case 1:
        if (p->cs1 == 1) {
            row = p->row1;
            col = p->col1;
            break;
        } else
            return 0;
    default:
        return 0;
    }

    banks = p->bank8 ? 8 : 4;
    dw = p->dw32 ? 4 : 2;
    size = (1 << (row + col)) * dw * banks;

    return size;
}

void fill_lpddr2_params(struct ddr_params *ddr_params)
{
    struct lpddr2_params *params = &ddr_params->lpddr2_params;

    ddr_params->type = LPDDR2;
    ddr_params->freq = get_ddr_freq();

    caculate_tck(ddr_params);

    ddr_params->div = DDR_CLK_DIV;
    ddr_params->cs0 = CONFIG_DDR_CS0;
    ddr_params->cs1 = CONFIG_DDR_CS1;
    ddr_params->dw32 = CONFIG_DDR_DW32;
    ddr_params->cl = DDR_CL;
    ddr_params->bl = DDR_BL;
    ddr_params->col = DDR_COL;
    ddr_params->row = DDR_ROW;

#ifdef DDR_COL1
    ddr_params->col1 = DDR_COL1;
#endif

#ifdef DDR_ROW1
    ddr_params->row1 = DDR_ROW1;
#endif

    ddr_params->bank8 = DDR_BANK8;
    params->tRP = DDR_tRP;
    params->tRCD = DDR_tRCD;
    params->tRC = DDR_tRC;
    params->tWR = DDR_tWR;
    params->tRRD = DDR_tRRD;
    params->tRTP = DDR_tRTP;
    params->tWTR = DDR_tWTR;
    params->tRFC = DDR_tRFC;
    params->tMINSR = DDR_tMINSR;
    params->tXP = DDR_tXP;
    params->tMRD = DDR_tMRD;
    params->tCCD = DDR_tCCD;
    params->tFAW = DDR_tFAW;
    params->tCKE = DDR_tCKE;
    params->tRL = DDR_tRL;
    params->tWL = DDR_tWL;
    params->tRTW = DDR_tRTW;
    params->tRAS = DDR_tRAS;
    params->tCKSRE = DDR_tCKSRE;

    params->tDQSCK = DDR_tDQSCK;
    params->tDQSCKmax = DDR_tDQSCKMAX;
    params->tDLLLOCK = DDR_tDLLLOCK;

    params->tXS = DDR_tXS;
    params->tXSRD = DDR_tXSRD;
    params->tREFI = DDR_tREFI;
    params->tDLLSRST = 50; /* default 50ns */

    ddr_params->size.chip0 = sdram_size(0, ddr_params);
    ddr_params->size.chip1 = sdram_size(1, ddr_params);
}

static void remap_swap(int a, int b, struct ddrc_reg *ddrc)
{
    unsigned int remmap[2];
    remmap[0] = ddrc->remap[a / 4];
    remmap[1] = ddrc->remap[b / 4];

#define BIT(bit) ((bit % 4) * 8)
#define MASK(bit) (0x1f << BIT(bit))

    unsigned int temp[2];
    temp[0] = (remmap[0] & MASK(a)) >> BIT(a);
    temp[1] = (remmap[1] & MASK(b)) >> BIT(b);

    remmap[0] &= ~MASK(a);
    remmap[1] &= ~MASK(b);

    ddrc->remap[a / 4] = remmap[0] | (temp[1] << BIT(a));
    ddrc->remap[b / 4] = remmap[1] | (temp[0] << BIT(b));

#undef BIT
#undef MASK
}

static void mem_remap(struct ddrc_reg *ddrc, struct ddr_params *ddr_params)
{
    unsigned int start = 0, num = 0;
    int row, col, dw32, bank8, cs0, cs1;
    unsigned int size0 = 0, size1 = 0;

    size0 = ddr_params->size.chip0;
    size1 = ddr_params->size.chip1;
    if (size0 && size1) {
        if (size1 <= size0) {
            row = ddr_params->row1;
            col = ddr_params->col1;
            dw32 = ddr_params->dw32;
            bank8 = ddr_params->bank8;
        } else {
            row = ddr_params->row;
            col = ddr_params->col;
            dw32 = ddr_params->dw32;
            bank8 = ddr_params->bank8;
        }
    } else if (size0) {
        row = ddr_params->row;
        col = ddr_params->col;
        dw32 = ddr_params->dw32;
        bank8 = ddr_params->bank8;
    } else if (size1) {
        row = ddr_params->row1;
        col = ddr_params->col1;
        dw32 = ddr_params->dw32;
        bank8 = ddr_params->bank8;
    }

    /*
     * Preload remap registers
     */
    ddrc->remap[0] = 0x03020100;
    ddrc->remap[1] = 0x07060504;
    ddrc->remap[2] = 0x0b0a0908;
    ddrc->remap[3] = 0x0f0e0d0c;
    ddrc->remap[4] = 0x13121110;

    cs0 = ddr_params->cs0;
    cs1 = ddr_params->cs1;

    start += row + col + (dw32 ? 4 : 2) / 2;
    start -= 12;

    if (bank8)
        num += 3;
    else
        num += 2;

    if (cs0 && cs1)
        num++;

    for (; num > 0; num--)
        remap_swap(num - 1, start + num - 1, ddrc);
}

void create_lpddr2_ddrc_params(struct ddrc_reg *ddrc,
                                        struct ddr_params *ddr_params)
{
    unsigned int tmp = 0;
    unsigned int mem_base0 = 0, mem_base1 = 0;
    unsigned int mem_mask0 = 0, mem_mask1 = 0;
    unsigned int memsize_cs0, memsize_cs1, memsize;

    struct tck *tck = &ddr_params->tck;

    struct lpddr2_params *params = &ddr_params->lpddr2_params;

    /*
     * TIMING1,2,3,4,5,6
     */
    ddrc->timing1.b.tRTP =
        calc_nck(params->tRTP, tck->ps);
    ddrc->timing1.b.tWTR =
        calc_nck(params->tWTR, tck->ps) +
        params->tWL + ddr_params->bl / 2 + 1;
    ddrc->timing1.b.tWR =
        calc_nck(params->tWR, tck->ps);

    if (ddrc->timing1.b.tWR < 5)
        ddrc->timing1.b.tWR = 5;

    if (ddrc->timing1.b.tWR > 12)
        ddrc->timing1.b.tWR = 12;

    ddrc->timing1.b.tWL = params->tWL;

    ddrc->timing2.b.tCCD = params->tCCD;
    ddrc->timing2.b.tRAS =
        calc_nck(params->tRAS, tck->ps);
    ddrc->timing2.b.tRCD =
        calc_nck(params->tRCD, tck->ps);
    ddrc->timing2.b.tRL = params->tRL;

    ddrc->timing3.b.ONUM = 4;

    /* Set DDR_tCKSRE to max to ensafe suspend & resume */
    ddrc->timing3.b.tCKSRE = 7;

    ddrc->timing3.b.tRP =
        calc_nck(params->tRP, tck->ps);
    ddrc->timing3.b.tRRD =
        calc_nck(params->tRRD, tck->ps);
    ddrc->timing3.b.tRC =
        calc_nck(params->tRC, tck->ps);

    /*
     * TODO
     */
    ddrc->timing4.b.tRFC = calc_nck(params->tRFC, tck->ps);
    ddrc->timing4.b.tEXTRW = 3;/* internal use, don't care */
    ddrc->timing4.b.tRWCOV = 3;/* interanl use, don't care */

#ifdef CONFIG_LPDDR2_PDSR_PARAMS_OVERRIDE

    ddrc->timing4.b.tCKE = params->tCKE - 1;

    tmp = params->tMINSR;
    tmp = calc_nck(tmp, tck->ps);
    tmp = ((tmp - 1) % 8) ? ((tmp - 1) / 8) : ((tmp - 1) / 8 + 1);
    ddrc->timing4.b.tMINSR = tmp + 1;

    tmp = params->tXP;
    tmp = calc_nck(tmp, tck->ps);
    BETWEEN(tmp, 0, 7);
    ddrc->timing4.b.tXP = tmp;

#else
    /*
     * Safe configuration
     */
    ddrc->timing4.b.tCKE = 7;
    ddrc->timing4.b.tMINSR = 15;
    ddrc->timing4.b.tXP = 7;

#endif

    ddrc->timing4.b.tMRD = 0; /* LPDDR2 not use. don't care */

    ddrc->timing5.b.tCTLUPD = 0x0; /* 0xff is the default value, unsupport feature. don't care */

    ddrc->timing5.b.tRTW = params->tRTW;

    /*
     * TODO: fix me, should be RL in MR2 + tDQSCK - 2
     *       refer publ doc
     */
    ddrc->timing5.b.tRDLAT = params->tRL - 1;
    ddrc->timing5.b.tWDLAT = params->tWL;

    tmp = params->tXSRD;
    tmp = calc_nck(tmp, tck->ps);
    tmp = (tmp + 4 - 1) / 4;
    BETWEEN(tmp, 0, 255);
    ddrc->timing6.b.tXSRD = tmp;

    tmp = calc_nck(params->tFAW, tck->ps);
    BETWEEN(tmp, 0, 31);
    ddrc->timing6.b.tFAW = tmp;

    ddrc->timing6.b.tCFGW = 2; /* internal use. don't care */
    ddrc->timing6.b.tCFGR = 2; /* internal use. don't care */

    tmp = params->tREFI / tck->ns;

    tmp = tmp / (16 * (1 << ddr_params->div)) - 1;
    if (tmp < 1)
        tmp = 1;
    if (tmp > 0xff)
        tmp = 0xff;
    ddrc->refcnt = (tmp << DDRC_REFCNT_CON_BIT)
        | (ddr_params->div << DDRC_REFCNT_CLK_DIV_BIT)
        | DDRC_REFCNT_REF_EN;

    /* CFG */
    ddrc->cfg.b.ROW1 = ddr_params->row1 - 12;
    ddrc->cfg.b.COL1 = ddr_params->col1 - 8;
    ddrc->cfg.b.BA1 = ddr_params->bank8;
    ddrc->cfg.b.IMBA = 1;
    ddrc->cfg.b.BSL = (ddr_params->bl == 8) ? 1 : 0;

    ddrc->cfg.b.ODTEN = 0;

    ddrc->cfg.b.MISPE = 1;
    ddrc->cfg.b.ROW0 = ddr_params->row - 12;
    ddrc->cfg.b.COL0 = ddr_params->col - 8;

    /*
     * TODO: DDRC bug, here must enable rank1
     */
    ddrc->cfg.b.CS1EN = 1;

    ddrc->cfg.b.CS0EN = ddr_params->cs0;

    tmp = ddr_params->cl - 1; /* NOT used in this version */
    if (tmp < 0)
        tmp = 0;
    if (tmp > 4)
        tmp = 4;
    ddrc->cfg.b.CL = tmp | 0x8; /* NOT used in this version */

    ddrc->cfg.b.BA0 = ddr_params->bank8;
    ddrc->cfg.b.DW = ddr_params->dw32;

    ddrc->cfg.b.TYPE = 5;   /* LPDDR2 */

    /* CTRL */
    ddrc->ctrl = DDRC_CTRL_ACTPD | DDRC_CTRL_PDT_64 | DDRC_CTRL_ACTSTP
        | DDRC_CTRL_PRET_8 | 0 << 6 | DDRC_CTRL_UNALIGN
        | DDRC_CTRL_ALH | DDRC_CTRL_RDC | DDRC_CTRL_CKE;

    /* MMAP0,1 */
    memsize_cs0 = ddr_params->size.chip0;
    memsize_cs1 = ddr_params->size.chip1;
    memsize = memsize_cs0 + memsize_cs1;

    if (memsize > 0x20000000) {
        if (memsize_cs1) {
            mem_base0 = 0x0;
            mem_mask0 = (~((memsize_cs0 >> 24) - 1) & ~(memsize >> 24))
                & DDRC_MMAP_MASK_MASK;
            mem_base1 = (memsize_cs1 >> 24) & 0xff;
            mem_mask1 = (~((memsize_cs1 >> 24) - 1) & ~(memsize >> 24))
                & DDRC_MMAP_MASK_MASK;
        } else {
            mem_base0 = 0x0;
            mem_mask0 = ~(((memsize_cs0 * 2) >> 24) - 1) & DDRC_MMAP_MASK_MASK;
            mem_mask1 = 0;
            mem_base1 = 0xff;
        }
    } else {
        mem_base0 = (DDR_MEM_PHY_BASE >> 24) & 0xff;
        mem_mask0 = ~((memsize_cs0 >> 24) - 1) & DDRC_MMAP_MASK_MASK;
        mem_base1 = ((DDR_MEM_PHY_BASE + memsize_cs0) >> 24) & 0xff;
        mem_mask1 = ~((memsize_cs1 >> 24) - 1) & DDRC_MMAP_MASK_MASK;
    }

    ddrc->mmap[0] = mem_base0 << DDRC_MMAP_BASE_BIT | mem_mask0;
    ddrc->mmap[1] = mem_base1 << DDRC_MMAP_BASE_BIT | mem_mask1;

    mem_remap(ddrc, ddr_params);
}

void create_lpddr2_ddrp_params(struct ddrp_reg *ddrp,
                                        struct ddr_params *ddr_params)
{
    unsigned int tmp = 0;
    unsigned int  count = 0;
    struct tck *tck = &ddr_params->tck;

    struct lpddr2_params *params = &ddr_params->lpddr2_params;

    /*
     * ==============================================================
     */
#define PNDEF(N, P, T, MIN, MAX, PS)  \
        T = calc_nck(params->P, PS); \
        BETWEEN(T, MIN, MAX);   \
        ddrp->dtpr##N.b.P = T
    /*
     * ==============================================================
     */

    /* DCR register */
    ddrp->dcr = 4 | (ddr_params->bank8 << 3);

    /* MRn registers */
    ddrp->mr0.d32 = 0x852;

    tmp = calc_nck(params->tWR, tck->ps);
    if (tmp < 3)
        tmp = 3;

    ddrp->mr1.lpddr2.nWR = tmp - 2;

    tmp = ddr_params->bl;

    while (tmp >>= 1)
        count++;

    ddrp->mr1.lpddr2.BL = count;

    ddrp->mr2.lpddr2.RL_WL = params->tRL - 2;

    ddrp->mr3.lpddr2.DS = 2;

    /* PTRn registers */
    ddrp->ptr0.b.tDLLSRST = calc_nck(params->tDLLSRST, tck->ps);
    ddrp->ptr0.b.tDLLLOCK = calc_nck(5120, tck->ps); /* DDR3 default 5.12us*/
    ddrp->ptr0.b.tITMSRST = 8;

    ddrp->ptr1.b.tDINIT0 = calc_nck(200000, tck->ps); /* LPDDR default 200us*/
    tmp = calc_nck(100, tck->ps);
    ddrp->ptr1.b.tDINIT1 = tmp;

    ddrp->ptr2.b.tDINIT2 = calc_nck(11000, tck->ps); /* DDR3 default 200us*/
    ddrp->ptr2.b.tDINIT3 = calc_nck(1000, tck->ps);

    /* DTPR0 registers */
    ddrp->dtpr0.b.tMRD = 0; /* LPDDR2 no use, don't care */
    PNDEF(0, tRTP, tmp, 2, 6, tck->ps);
    PNDEF(0, tWTR, tmp, 1, 6, tck->ps);
    PNDEF(0, tRP, tmp, 2, 11, tck->ps);
    PNDEF(0, tRCD, tmp, 2, 11, tck->ps);
    PNDEF(0, tRAS, tmp, 2, 31, tck->ps);
    PNDEF(0, tRRD, tmp, 1, 8, tck->ps);
    PNDEF(0, tRC, tmp, 2, 42, tck->ps);
    ddrp->dtpr0.b.tCCD = (params->tCCD > (ddr_params->bl / 2)) ? 1 : 0;

    /* DTPR1 registers */
    PNDEF(1, tFAW, tmp, 2, 31, tck->ps);
    PNDEF(1, tRFC, tmp, 1, 255, tck->ps);
    PNDEF(1, tDQSCK, tmp, 1, 7, tck->ps);
    PNDEF(1, tDQSCKmax, tmp, 1, 7, tck->ps);

    /* DTPR2 registers */
    tmp = calc_nck(params->tXS, tck->ps);
    BETWEEN(tmp, 2, 1023);
    ddrp->dtpr2.b.tXS = tmp;

#ifdef CONFIG_LPDDR2_PDSR_PARAMS_OVERRIDE

    tmp = calc_nck(params->tXP, tck->ps);
    BETWEEN(tmp, 2, 31);
    ddrp->dtpr2.b.tXP = tmp;

    tmp = calc_nck(params->tMINSR, tck->ps);
    BETWEEN(tmp, 2, 15);
    ddrp->dtpr2.b.tCKE = tmp;

#else

    ddrp->dtpr2.b.tXP = 31;
    ddrp->dtpr2.b.tCKE = 15;

#endif

    tmp = params->tDLLLOCK;
    BETWEEN(tmp, 2, 1023);
    ddrp->dtpr2.b.tDLLK = tmp;
    /* PGCR registers */
    ddrp->pgcr = DDRP_PGCR_DQSCFG | 7 << DDRP_PGCR_CKEN_BIT
        | 2 << DDRP_PGCR_CKDV_BIT
        | (ddr_params->cs0 | ddr_params->cs1 << 1) << DDRP_PGCR_RANKEN_BIT
        | DDRP_PGCR_ZCKSEL_32 | DDRP_PGCR_PDDISDX;

#undef BETWEEN
#undef PNDEF
}

void create_lpddr2_params(struct ddrc_reg *ddrc,
                struct ddrp_reg *ddrp, struct ddr_params *ddr_params)
{
    create_lpddr2_ddrc_params(ddrc, ddr_params);
    create_lpddr2_ddrp_params(ddrp, ddr_params);
}

static void print_result(struct ddrc_reg *ddrc,
                struct ddrp_reg *ddrp, struct ddr_params *ddr_params)
{
    printf("#ifndef LPDDR2_PARAMS_H\n");
    printf("#define LPDDR2_PARAMS_H\n");

    /* DDRC registers print */
    printf("static const unsigned int DDRC_CFG_VALUE     = 0x%08x;\n", ddrc->cfg.d32);
    printf("static const unsigned int DDRC_CTRL_VALUE    = 0x%08x;\n", ddrc->ctrl);
    printf("static const unsigned int DDRC_MMAP0_VALUE   = 0x%08x;\n", ddrc->mmap[0]);
    printf("static const unsigned int DDRC_MMAP1_VALUE   = 0x%08x;\n", ddrc->mmap[1]);
    printf("static const unsigned int DDRC_REFCNT_VALUE  = 0x%08x;\n", ddrc->refcnt);
    printf("static const unsigned int DDRC_TIMING1_VALUE = 0x%08x;\n", ddrc->timing1.d32);
    printf("static const unsigned int DDRC_TIMING2_VALUE = 0x%08x;\n", ddrc->timing2.d32);
    printf("static const unsigned int DDRC_TIMING3_VALUE = 0x%08x;\n", ddrc->timing3.d32);
    printf("static const unsigned int DDRC_TIMING4_VALUE = 0x%08x;\n", ddrc->timing4.d32);
    printf("static const unsigned int DDRC_TIMING5_VALUE = 0x%08x;\n", ddrc->timing5.d32);
    printf("static const unsigned int DDRC_TIMING6_VALUE = 0x%08x;\n", ddrc->timing6.d32);
    printf("static const unsigned int DDRC_DREMAP1_VALUE = 0x%08x;\n", ddrc->remap[0]);
    printf("static const unsigned int DDRC_DREMAP2_VALUE = 0x%08x;\n", ddrc->remap[1]);
    printf("static const unsigned int DDRC_DREMAP3_VALUE = 0x%08x;\n", ddrc->remap[2]);
    printf("static const unsigned int DDRC_DREMAP4_VALUE = 0x%08x;\n", ddrc->remap[3]);
    printf("static const unsigned int DDRC_DREMAP5_VALUE = 0x%08x;\n", ddrc->remap[4]);

    /* DDRP registers print */
    printf("static const unsigned int DDRP_DCR_VALUE     = 0x%08x;\n", ddrp->dcr);
    printf("static const unsigned int DDRP_MR0_VALUE     = 0x%08x;\n", ddrp->mr0.d32);
    printf("static const unsigned int DDRP_MR1_VALUE     = 0x%08x;\n", ddrp->mr1.d32);
    printf("static const unsigned int DDRP_MR2_VALUE     = 0x%08x;\n", ddrp->mr2.d32);
    printf("static const unsigned int DDRP_MR3_VALUE     = 0x%08x;\n", ddrp->mr3.d32);
    printf("static const unsigned int DDRP_PTR0_VALUE    = 0x%08x;\n", ddrp->ptr0.d32);
    printf("static const unsigned int DDRP_PTR1_VALUE    = 0x%08x;\n", ddrp->ptr1.d32);
    printf("static const unsigned int DDRP_PTR2_VALUE    = 0x%08x;\n", ddrp->ptr2.d32);
    printf("static const unsigned int DDRP_DTPR0_VALUE   = 0x%08x;\n", ddrp->dtpr0.d32);
    printf("static const unsigned int DDRP_DTPR1_VALUE   = 0x%08x;\n", ddrp->dtpr1.d32);
    printf("static const unsigned int DDRP_DTPR2_VALUE   = 0x%08x;\n", ddrp->dtpr2.d32);
    printf("static const unsigned int DDRP_PGCR_VALUE    = 0x%08x;\n", ddrp->pgcr);

    /* DDR size print */
    printf("static const unsigned int DDR_CHIP_0_SIZE    = %u;\n", ddr_params->size.chip0);
    printf("static const unsigned int DDR_CHIP_1_SIZE    = %u;\n", ddr_params->size.chip1);

    printf("#endif\n");
}

int main(int argc, char *argv[])
{
    struct ddrc_reg ddrc;
    struct ddrp_reg ddrp;
    struct ddr_params ddr_params;

    memset(&ddrc, 0, sizeof(struct ddrc_reg));
    memset(&ddrp, 0, sizeof(struct ddrp_reg));
    memset(&ddr_params, 0, sizeof(struct ddr_params));

    fill_lpddr2_params(&ddr_params);

    create_lpddr2_params(&ddrc, &ddrp, &ddr_params);

    print_result(&ddrc, &ddrp, &ddr_params);

    return 0;
}
