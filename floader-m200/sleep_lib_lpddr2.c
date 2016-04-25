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

static u32 backup_training_buffer[16];

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
        union ZQnSR0 sr0;
        sr0.reg = ddr_readl(DDRP_ZQXSR0(i));
        debug("ZQ%dSR0          0x%x\n", i, sr0.reg);
        debug("pullup_oz      0x%02x\n", sr0.ZQnSR0.pullup_oz);
        debug("pulldown_oz    0x%02x\n", sr0.ZQnSR0.pulldown_oz);
        debug("pullup_odtz    0x%02x\n", sr0.ZQnSR0.pullup_odtz);
        debug("pulldown_odtz  0x%02x\n", sr0.ZQnSR0.pulldown_odtz);

        debug("ZQ%dSR1          0x%x\n", i,ddr_readl(DDRP_ZQXSR1(i)));
    }

    debug("=====================\n");
}

static void reset_all()
{
    /*
     * Don't care what is it, it's magic...
     */
    writel(0x53 | (1 << 6), CPM_BASE + CPM_DRCG);
    udelay(100);
    writel(0x7d | (1 << 6), CPM_BASE + CPM_DRCG);
    udelay(100);
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

static void training_lpddr2_publ()
{
    debug("Training PUBL start...\n");

    /*
     * Un-gating clk of DDRC
     */
    cpm_writel(cpm_readl(CPM_CLKGR) & ~(0x1 << 31), CPM_CLKGR);
    cpm_writel(cpm_readl(CPM_SPCR0) & ~0x1, CPM_SPCR0);
    udelay(5);

    /*
     * Execute PUBL initialization
     */
    unsigned int pir_val = DDRP_PIR_INIT
                                | DDRP_PIR_ITMSRST
                                | DDRP_PIR_DLLSRST
                                | DDRP_PIR_DLLLOCK
                                | DDRP_PIR_ZCAL
                                | DDRP_PIR_QSTRN;

    unsigned int done_mask = DDRP_PGSR_IDONE
                                | DDRP_PGSR_DLDONE
                                | DDRP_PGSR_ZCDONE
                                | DDRP_PGSR_DTDONE;
    /*
     * DLL bypassed mode
     */
    if (get_ddr_freq() <= CONFIG_DDR_PUBL_DLL_BYPASS_FREQ) {
        debug("Publ running at DLL bypass mode.\n");

        pir_val |= DDRP_PIR_DLLBYP | (1 << 29);
        pir_val &= ~(DDRP_PIR_DLLSRST | DDRP_PIR_DLLLOCK);

        done_mask &= ~DDRP_PGSR_DLDONE;
    } else {
        /*
         * Keep PHY exit power down mode
         */
        ddr_writel(0x0, DDRC_DLP);
        ddr_writel(ddr_readl(DDRP_DSGCR) & ~(0x1 << 4), DDRP_DSGCR);

        udelay(500);
    }

    /*
     * Exit SR status
     */
    ddr_writel(ddr_readl(DDRC_CTRL) & ~(DDRC_CTRL_SR | DDRC_CTRL_KEEPSR), DDRC_CTRL);
    while (ddr_readl(DDRC_STATUS) & DDRC_ST_SREF)
        continue;

    wait_phy_init(pir_val, done_mask, 5000000);

    /*
     * Restore
     */
    ddr_writel(DDRC_CTRL_VALUE, DDRC_CTRL);

    debug("Training PUBL finished.\n");
}

void prepare_lpddr2_for_sleep()
{
    /*
     * Backup training address
     */
    memcpy(backup_training_buffer,
                (void *)0xa0000000, sizeof(backup_training_buffer));

    /*
     * Disable LPDDR2 auto enter SR status
     */
    ddr_writel(0, DDRC_AUTOSR_EN);
    while (ddr_readl(DDRC_STATUS) & DDRC_ST_SREF)
        continue;

    /*
     * Disable active power down
     */
    ddr_writel(ddr_readl(DDRC_CTRL) & ~(0x1f << 11), DDRC_CTRL);

    /*
     * PHY enter power down if not DLL bypass mode
     */
    if (get_ddr_freq() > CONFIG_DDR_PUBL_DLL_BYPASS_FREQ) {
        ddr_writel(ddr_readl(DDRP_DSGCR) | (0x1 << 4), DDRP_DSGCR);
        ddr_writel(0xf003, DDRC_DLP);

        udelay(500);
    }

    /*
     * Wait for enter SR
     */
    ddr_writel(ddr_readl(DDRC_CTRL) | DDRC_CTRL_SR | DDRC_CTRL_KEEPSR, DDRC_CTRL);
    while (!(ddr_readl(DDRC_STATUS) & DDRC_ST_SREF))
        continue;

    /*
     * Gating clk of DDRC
     */
    cpm_writel(cpm_readl(CPM_SPCR0) | 0x1, CPM_SPCR0);
    cpm_writel(cpm_readl(CPM_CLKGR) | (0x1 << 31), CPM_CLKGR);
}

void restore_lpddr2()
{
    reset_all();
    training_lpddr2_publ();

    /*
     * Enable auto enter SR status
     */
    ddr_writel(0x1, DDRC_AUTOSR_EN);

    dump_register();

    /*
     * Restore training buffer
     */
    memcpy((void *)0xa0000000,
            backup_training_buffer, sizeof(backup_training_buffer));
}
