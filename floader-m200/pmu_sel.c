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

static enum pmu_t g_pmu = SM5007;

void select_pmu(enum pmu_t pmu)
{
    debug("PMU selection: select pmu: %s\n",
                    pmu == SM5007 ? "SM5007" : "RICOH_5T619");
    g_pmu = pmu;
}

enum pmu_t get_pmu()
{
    return g_pmu;
}
