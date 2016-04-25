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

int main(void)
{
    /*
     * Amazing travel start...
     */
    init_uart();

    printf("Floader_m200: %s, %s\n", __DATE__, __TIME__);

    /*
     * Power configuration
     */
    enum pmu_t pmu = SM5007;
    if (!probe_sm5007()) {
        pmu_sm5007_set_core_voltage(CONFIG_CORE_VOLTAGE_MV);
        pmu = SM5007;

    } else if (!probe_5t619()) {
        pmu_5t619_set_core_voltage(CONFIG_CORE_VOLTAGE_MV);
        pmu_5t619_set_ddr_vdd2_voltage(CONFIG_DDR_VDD2_VOLTAGE_MV);
        pmu = RICOH_5T619;

    } else {
        stop(STOP_ERROR_DETECT_PMU_FAILED);
    }

    select_pmu(pmu);

    init_clk();

    int is_from_powerup = is_boot_from_powerup();

    init_lpddr2(is_from_powerup);

    /*
     * Write back myself
     */
    flush_cache_all();

    if (is_from_powerup) {
        init_mmc_host();

#if (CONFIG_FLOADER_FUNCTION == 2)
        return 0;
#endif

        init_sleep_lib(pmu);

#if 0
        while (1)
            enter_sleep_with_powerkey_wake();
#endif

        boot_from_emmc();
    } else {
        boot_from_ddr();
    }

    /*
     * All here.
     */
    return 0;
}
