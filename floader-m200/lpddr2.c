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

#include "./lpddr2_params.h"

union ZQnSR0 {
    u32 reg;
    struct {
        u32 pulldown_oz:5;
        u32 pullup_oz:5;
        u32 pulldown_odtz:5;
        u32 pullup_odtz:5;
        u32 reserved:10;
        u32 zerr:1;
        u32 zdone:1;
    } ZQnSR0;
};

static void dump_register(void)
{
    debug("=====================\n");
    debug("Dump DDRC & PUBL:\n");
    debug("=====================\n");
    debug("DDRC_STATUS     0x%x\n", ddr_readl(DDRC_STATUS));
    debug("DDRC_CFG        0x%x\n", ddr_readl(DDRC_CFG));
    debug("DDRC_CTRL       0x%x\n", ddr_readl(DDRC_CTRL));
    debug("DDRC_LMR        0x%x\n", ddr_readl(DDRC_LMR));
    debug("DDRC_TIMING1    0x%x\n", ddr_readl(DDRC_TIMING(1)));
    debug("DDRC_TIMING2    0x%x\n", ddr_readl(DDRC_TIMING(2)));
    debug("DDRC_TIMING3    0x%x\n", ddr_readl(DDRC_TIMING(3)));
    debug("DDRC_TIMING4    0x%x\n", ddr_readl(DDRC_TIMING(4)));
    debug("DDRC_TIMING5    0x%x\n", ddr_readl(DDRC_TIMING(5)));
    debug("DDRC_TIMING6    0x%x\n", ddr_readl(DDRC_TIMING(6)));
    debug("DDRC_REFCNT     0x%x\n", ddr_readl(DDRC_REFCNT));
    debug("DDRC_MMAP0      0x%x\n", ddr_readl(DDRC_MMAP0));
    debug("DDRC_MMAP1      0x%x\n", ddr_readl(DDRC_MMAP1));
    debug("DDRC_REMAP1     0x%x\n", ddr_readl(DDRC_REMAP(1)));
    debug("DDRC_REMAP2     0x%x\n", ddr_readl(DDRC_REMAP(2)));
    debug("DDRC_REMAP3     0x%x\n", ddr_readl(DDRC_REMAP(3)));
    debug("DDRC_REMAP4     0x%x\n", ddr_readl(DDRC_REMAP(4)));
    debug("DDRC_REMAP5     0x%x\n", ddr_readl(DDRC_REMAP(5)));

    debug("DDRP_PIR        0x%x\n", ddr_readl(DDRP_PIR));
    debug("DDRP_PGCR       0x%x\n", ddr_readl(DDRP_PGCR));
    debug("DDRP_PGSR       0x%x\n", ddr_readl(DDRP_PGSR));
    debug("DDRP_PTR0       0x%x\n", ddr_readl(DDRP_PTR0));
    debug("DDRP_PTR1       0x%x\n", ddr_readl(DDRP_PTR1));
    debug("DDRP_PTR2       0x%x\n", ddr_readl(DDRP_PTR2));
    debug("DDRP_DCR        0x%x\n", ddr_readl(DDRP_DCR));
    debug("DDRP_DTPR0      0x%x\n", ddr_readl(DDRP_DTPR0));
    debug("DDRP_DTPR1      0x%x\n", ddr_readl(DDRP_DTPR1));
    debug("DDRP_DTPR2      0x%x\n", ddr_readl(DDRP_DTPR2));
    debug("DDRP_MR0        0x%x\n", ddr_readl(DDRP_MR0));
    debug("DDRP_MR1        0x%x\n", ddr_readl(DDRP_MR1));
    debug("DDRP_MR2        0x%x\n", ddr_readl(DDRP_MR2));
    debug("DDRP_MR3        0x%x\n", ddr_readl(DDRP_MR3));
    debug("DDRP_ACDLLCR    0x%x\n", ddr_readl(DDRP_ACDLLCR));
    debug("DDRP_DSGCR      0x%x\n", ddr_readl(DDRP_DSGCR));
    debug("DDRP_DLLGCR     0x%x\n", ddr_readl(DDRP_DLLGCR));
    debug("DDRP_ODTCR      0x%x\n", ddr_readl(DDRP_ODTCR));

    for(int i = 0; i < 4;i++) {
        debug("DX%dGSR0         0x%x\n", i, ddr_readl(DDRP_DXGSR0(i)));
        debug("DX%dDQSTR        0x%x\n", i,ddr_readl(DDRP_DXDQSTR(i)));
    }

    for (int i = 0; i < 4; i++) {
        debug("DX%dGCR          0x%x\n", i, ddr_readl(DDRP_DXGCR(i)));
    }

    for(int i = 0; i < 4;i++) {
#ifdef DEBUG
        union ZQnSR0 sr0;
        sr0.reg = ddr_readl(DDRP_ZQXSR0(i));
#endif

        debug("ZQ%dSR0          0x%x\n", i, sr0.reg);
        debug("pullup_oz      0x%02x\n", sr0.ZQnSR0.pullup_oz);
        debug("pulldown_oz    0x%02x\n", sr0.ZQnSR0.pulldown_oz);
        debug("pullup_odtz    0x%02x\n", sr0.ZQnSR0.pullup_odtz);
        debug("pulldown_odtz  0x%02x\n", sr0.ZQnSR0.pulldown_odtz);

        debug("ZQ%dSR1          0x%x\n", i,ddr_readl(DDRP_ZQXSR1(i)));
    }
    debug("=====================\n");
}

static void disable_external_cke_gate()
{
    /*
     * TODO
     */
}

static void enable_external_cke_gate()
{
    /*
     * TODO
     */
}


static void reset_all()
{
    disable_external_cke_gate();

    /*
     * Don't care what is it, it's magic...
     */
    writel(3 | (1 << 6), CPM_BASE + CPM_DRCG);
    mdelay(5);
    writel(0x7d | (1 << 6), CPM_BASE + CPM_DRCG);
    mdelay(5);

    ddr_writel(0xf << 20, DDRC_CTRL);
    mdelay(5);
    ddr_writel(0, DDRC_CTRL);
    mdelay(5);
}

static void wait_phy_init(
        unsigned int reg_pir, unsigned int done_mask, unsigned int timeout)
{
    unsigned int count = 0;
    unsigned int pgsr = 0;

    debug("PHY init command reg_pir = 0x%x\n", reg_pir);
    debug("PHY init command done_mask = 0x%x\n", done_mask);

    ddr_writel(reg_pir, DDRP_PIR);

    do {
        if(count++ > timeout) {
            dump_register();
            stop(STOP_ERROR_PUBL_PGSR_TIMEOUT);
        }

        pgsr = ddr_readl(DDRP_PGSR);
    } while((pgsr & done_mask) != done_mask);

    debug("PHY init PGSR: 0x%x\n", pgsr);
    debug("PHY init wait count: %d\n", count);

    if (pgsr & (7 << 5)) {
        dump_register();
        stop(STOP_ERROR_PUBL_PGSR_TRAINING_FAILED);
    }
}

void lpddr2_enter_self_refresh()
{
    if (ddr_readl(DDRC_STATUS) & DDRC_ST_SREF)
        stop(STOP_ERROR_DRAM_WRONG_STATE);

    /*
     * Wait for enter SR
     */
    ddr_writel(ddr_readl(DDRC_CTRL) | DDRC_CTRL_SR, DDRC_CTRL);
    while (!(ddr_readl(DDRC_STATUS) & DDRC_ST_SREF))
        continue;
}

void lpddr2_exit_self_refresh(
                        unsigned int training_pir,
                        unsigned int done_mask,
                        unsigned int training_timeout)
{
    if (!(ddr_readl(DDRC_STATUS) & DDRC_ST_SREF))
        stop(STOP_ERROR_DRAM_WRONG_STATE);

    /*
     * Wait for exit SR
     */
    ddr_writel(ddr_readl(DDRC_CTRL) & ~DDRC_CTRL_SR, DDRC_CTRL);
    while (ddr_readl(DDRC_STATUS) & DDRC_ST_SREF)
        continue;

    /*
     * Training PUBL
     */
    wait_phy_init(training_pir, done_mask, training_timeout);
}

void wakeup_lpddr2(
                unsigned int training_pir,
                unsigned int done_mask,
                unsigned int training_timeout)
{
    lpddr2_enter_self_refresh();

    debug("Wakeup LPDDR2 step 1 done.\n");

    enable_external_cke_gate();

    debug("Wakeup LPDDR2 step 2 done.\n");

    lpddr2_exit_self_refresh(training_pir, done_mask, training_timeout);

    debug("Wakeup LPDDR2 step 3 done.\n");
}

int has_two_lpddr2_chips()
{
    return (((DDRP_PGCR_VALUE >> DDRP_PGCR_RANKEN_BIT) & 0x3) == 0x3);
}

static void init_dwc_ddr_publ(int is_from_powerup)
{
    debug("Init PUBL start...\n");

    /*
     * Disable auto enter SR status
     */
    ddr_writel(0x0, DDRC_AUTOSR_EN);

    /*
     * Configure DDRC
     */
    ddr_writel(DDRC_CFG_VALUE, DDRC_CFG);

    /*
     * Force enable CKE
     */
    ddr_writel(DDRC_CTRL_CKE, DDRC_CTRL);

    /*
     * Configure DRAM type to LPDDR2
     */
    ddr_writel(DDRP_DCR_VALUE, DDRP_DCR);

    /*
     * BL, BT, WC, N Twr
     */
    ddr_writel(DDRP_MR1_VALUE, DDRP_MR1);

    /*
     * RL/WL
     */
    ddr_writel(DDRP_MR2_VALUE, DDRP_MR2);

    /*
     * Driver strength
     */
    ddr_writel(DDRP_MR3_VALUE, DDRP_MR3);

    /*
     * PHY timing configuration
     */
    ddr_writel(DDRP_PTR0_VALUE, DDRP_PTR0);
    ddr_writel(DDRP_PTR1_VALUE, DDRP_PTR1);
    ddr_writel(DDRP_PTR2_VALUE, DDRP_PTR2);

    /*
     * DRAM timing configuration
     */
    ddr_writel(DDRP_DTPR0_VALUE, DDRP_DTPR0);
    ddr_writel(DDRP_DTPR1_VALUE, DDRP_DTPR1);
    ddr_writel(DDRP_DTPR2_VALUE, DDRP_DTPR2);


    /*
     * Data lane(8 bits) configuration
     */
    for (int i = 0; i < 4; i++) {
        unsigned int dxgcr = ddr_readl(DDRP_DXGCR(i));

        /*
         * Disable ODT (Do not supported by LPDDR2)
         */
        dxgcr &= ~(3 << 9);

        if (!(DDRC_CFG_VALUE & 0x1)) {
            /*
             * We should disable DATX8(2) & DATX8(3)
             * if it is 16 bits DRAM
             */
            if (i == 2 || i == 3) {
                dxgcr &= ~DDRP_DXGCR_DXEN;
            }
        }

        ddr_writel(dxgcr, DDRP_DXGCR(i));
    }

    /*
     * PHY general configuration
     */
    ddr_writel(DDRP_PGCR_VALUE, DDRP_PGCR);

    /*
     * DQS pull-down/up configuration
     */
    ddr_writel(0x910, DDRP_DXCCR);

    /*
     * Execute PUBL and DRAM initialization
     */
    unsigned int pir_val = DDRP_PIR_INIT
                                | DDRP_PIR_DRAMINT
                                | DDRP_PIR_ITMSRST
                                | DDRP_PIR_DLLSRST
                                | DDRP_PIR_DLLLOCK
                                | DDRP_PIR_ZCAL;

    unsigned int done_mask = DDRP_PGSR_IDONE
                                | DDRP_PGSR_DIDONE
                                | DDRP_PGSR_DLDONE
                                | DDRP_PGSR_ZCDONE;
    /*
     * DLL bypassed mode
     */
    if (get_ddr_freq() <= CONFIG_DDR_PUBL_DLL_BYPASS_FREQ) {
        debug("Publ running at DLL bypass mode.\n");

        pir_val |= DDRP_PIR_DLLBYP | (1 << 29);
        pir_val &= ~(DDRP_PIR_DLLSRST | DDRP_PIR_DLLLOCK);

        /*
         * DLL disabled
         */
        ddr_writel(0x1 << 31, DDRP_ACDLLCR);
        ddr_writel(ddr_readl(DDRP_DLLGCR) | (1 << 23), DDRP_DLLGCR);
        ddr_writel(ddr_readl(DDRP_DSGCR) & ~(1 << 4), DDRP_DSGCR);

        done_mask &= ~DDRP_PGSR_DLDONE;
    }

    if (is_from_powerup)
        enable_external_cke_gate();

    /*
     * Initialize PUBL
     */
    wait_phy_init(pir_val, done_mask, 5000000);

    /*
     * Set PHY training address
     */
    ddr_writel(0x0, DDRP_DTAR);

    pir_val &= ~DDRP_PIR_DRAMINT;
    pir_val |= DDRP_PIR_QSTRN;
    done_mask &= ~DDRP_PGSR_DIDONE;
    done_mask |= DDRP_PGSR_DTDONE;

    if (is_from_powerup) {
        /*
         * Training PUBL
         */
        wait_phy_init(pir_val, done_mask, 5000000);
    } else {
        debug("Start wakeup LPDDR2...\n");

        wakeup_lpddr2(pir_val, done_mask, 5000000);

        debug("Wakeup LPDDR2 done.\n");
    }

    debug("Init PUBL finished.\n");
}

static void init_ddr_controller()
{
    debug("Init DDRC start...\n");

    ddr_writel(DDRC_CTRL_CKE | DDRC_CTRL_ALH, DDRC_CTRL);
    ddr_writel(0, DDRC_CTRL);

    ddr_writel(DDRC_CFG_VALUE, DDRC_CFG);

    ddr_writel(DDRC_TIMING1_VALUE, DDRC_TIMING(1));
    ddr_writel(DDRC_TIMING2_VALUE, DDRC_TIMING(2));
    ddr_writel(DDRC_TIMING3_VALUE, DDRC_TIMING(3));
    ddr_writel(DDRC_TIMING4_VALUE, DDRC_TIMING(4));
    ddr_writel(DDRC_TIMING5_VALUE, DDRC_TIMING(5));
    ddr_writel(DDRC_TIMING6_VALUE, DDRC_TIMING(6));

    ddr_writel(DDRC_MMAP0_VALUE, DDRC_MMAP0);
    ddr_writel(DDRC_MMAP1_VALUE, DDRC_MMAP1);

    ddr_writel(DDRC_CTRL_CKE | DDRC_CTRL_ALH, DDRC_CTRL);
    ddr_writel(DDRC_REFCNT_VALUE, DDRC_REFCNT);
    ddr_writel(DDRC_CTRL_VALUE, DDRC_CTRL);

    ddr_writel(DDRC_DREMAP1_VALUE, DDRC_REMAP(1));
    ddr_writel(DDRC_DREMAP2_VALUE, DDRC_REMAP(2));
    ddr_writel(DDRC_DREMAP3_VALUE, DDRC_REMAP(3));
    ddr_writel(DDRC_DREMAP4_VALUE, DDRC_REMAP(4));
    ddr_writel(DDRC_DREMAP5_VALUE, DDRC_REMAP(5));

    ddr_writel(ddr_readl(DDRC_STATUS) & ~DDRC_DSTATUS_MISS, DDRC_STATUS);

    if (get_ddr_freq() > CONFIG_DDR_PUBL_DLL_BYPASS_FREQ) {
        ddr_writel(0, DDRC_DLP);
    } else {
        /*
         * Stop clock when LPDDR2 in SR state
         */
        cpm_writel(cpm_readl(CPM_DDRCDR) | (0x1 << 26), CPM_DDRCDR);
        ddr_apb_writel(
                ddr_apb_readl(DDRC_APB_CLKSTP_CFG) | (0x9 << 28),
                DDRC_APB_CLKSTP_CFG);
        ddr_apb_writel(
                ddr_apb_readl(DDRC_APB_CKC_CFG1) | (0x4 << 16),
                DDRC_APB_CKC_CFG1);
    }

    /*
     * Enable auto enter SR status
     */
    ddr_writel(0x1, DDRC_AUTOSR_EN);

    debug("Init DDRC finished.\n");
}

void init_lpddr2(int is_from_powerup)
{
    debug("Init LPDDR2 from %s start...\n",
                is_from_powerup ? "powerup" : "SR state");

    reset_all();
    init_dwc_ddr_publ(is_from_powerup);
    init_ddr_controller();

    dump_register();

#if (CONFIG_FLOADER_FUNCTION == 2)
    struct global_info *gi = (void *)0x80001000;
    gi->ddr_params.size.chip0 = DDR_CHIP_0_SIZE;
    gi->ddr_params.size.chip1 = DDR_CHIP_1_SIZE;
#endif

    debug("Init LPDDR2 finished.\n");
}
