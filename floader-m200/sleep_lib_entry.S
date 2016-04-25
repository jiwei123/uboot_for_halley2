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

    .extern enable_set_pmu_suspend_mode_voltage
    .extern enable_sleep_poweroff_mode
    .extern restore_context
    .extern enter_sleep
    .extern select_pmu

    .extern context

    .global sleep_lib_entry

    .text
sleep_lib_entry:
    .word enable_set_pmu_suspend_mode_voltage
    .word enable_sleep_poweroff_mode
    .word restore_context
    .word enter_sleep
    .word select_pmu

    .global save_context_goto
    .align  4
save_context_goto:
    la  k0, context
    lw  k0, 0(k0)

    /* s0-s7 */
    sw  s0, 0*4(k0)
    sw  s1, 1*4(k0)
    sw  s2, 2*4(k0)
    sw  s3, 3*4(k0)
    sw  s4, 4*4(k0)
    sw  s5, 5*4(k0)
    sw  s6, 6*4(k0)
    sw  s7, 7*4(k0)

    /* gp,sp,fp,ra */
    sw  gp, 8*4(k0)
    sw  sp, 9*4(k0)
    sw  fp, 10*4(k0)
    sw  ra, 11*4(k0)

    mfc0    k1, CP0_STATUS, 0
    sw  k1, 12*4(k0)

    mfc0    k1, CP0_EBASE, 1
    sw  k1, 13*4(k0)

    mfc0    k1, CP0_RANDOM, 0
    sw  k1, 14*4(k0)

    mfc0    k1, CP0_PAGEMASK, 0
    sw  k1, 15*4(k0)

    mfc0    k1, CP0_WIRED, 0
    sw  k1, 16*4(k0)

    mfc0    k1, CP0_ENTRYHI, 0
    sw  k1, 17*4(k0)

    mfc0    k1, CP0_CONFIG, 0
    sw  k1, 18*4(k0)

    mfc0    k1, CP0_CONFIG7, 7
    sw  k1, 19*4(k0)

    mfc0    k1, CP0_WATCHLO, 0
    sw  k1, 20*4(k0)

    mfc0    k1, CP0_WATCHHI, 0
    sw  k1, 21*4(k0)

    mfc0    k1, CP0_DESAVE, 0
    sw  k1, 22*4(k0)

    mfc0    k1, PMON_CSR
    sw  k1, 23*4(k0)

    mfc0    k1, PMON_HIGH
    sw  k1, 24*4(k0)

    mfc0    k1, PMON_LC
    sw  k1, 25*4(k0)

    mfc0    k1, PMON_RC
    sw  k1, 26*4(k0)

    mfhi    k1
    sw  k1, 27*4(k0)
    mflo    k1
    sw  k1, 28*4(k0)

    mfc0    k1, CORE_CTRL
    sw  k1, 29*4(k0)

    mfc0    k1, CORE_REIM
    sw  k1, 30*4(k0)

    mfc0    k1, CP0_ECC, 0
    sw  k1, 31*4(k0)

    mfc0    k1, CORE_REIM_LOW
    sw  k1, 32*4(k0)

    jr  a0

    nop

    .global restore_context_goto
    .align  4
restore_context_goto:
    la  k1, context
    lw  k1, 0(k1)

    lw  k0, 29*4(k1)
    mtc0    k0, CORE_CTRL

    lw  k0, 30*4(k1)
    mtc0    k0, CORE_REIM

    lw  k0, 15*4(k1)
    mtc0    k0, CP0_PAGEMASK, 0

    /* restroe the regs */
    lw  k0, 17*4(k1)
    mtc0    k0, CP0_ENTRYHI, 0

    lw  k0, 16*4(k1)
    mtc0    k0, CP0_WIRED, 0

    /* 15 is loaded before the tlb write */
    lw  k0, 14*4(k1)
    mtc0    k0, CP0_RANDOM, 0

    lw  k0, 13*4(k1)
    mtc0    k0, CP0_EBASE, 1

    lw  k0, 12*4(k1)
    mtc0    k0, CP0_STATUS, 0

    lw  k0, 18*4(k1)
    mtc0    k0, CP0_CONFIG, 0

    lw  k0, 19*4(k1)
    mtc0    k0, CP0_CONFIG7, 7

    lw  k0, 20*4(k1)
    mtc0    k0, CP0_WATCHLO, 0

    lw  k0, 21*4(k1)
    mtc0    k0, CP0_WATCHHI, 0

    lw  k0, 22*4(k1)
    mtc0    k0, CP0_DESAVE, 0

    lw  k0, 23*4(k1)
    mtc0    k0, PMON_CSR

    lw  k0, 24*4(k1)
    mtc0    k0, PMON_HIGH

    lw  k0, 25*4(k1)
    mtc0    k0, PMON_LC

    lw  k0, 26*4(k1)
    mtc0    k0, PMON_RC

    lw  k0, 27*4(k1)
    mthi    k0

    lw  k0, 28*4(k1)
    mtlo    k0

    lw  k0, 29*4(k1)
    mtc0    k0, CORE_CTRL

    lw  k0, 30*4(k1)
    mtc0    k0, CORE_REIM

    lw  k0, 31*4(k1)
    mtc0    k0, CP0_ECC, 0

    lw  k0, 32*4(k1)
    mtc0    k0, CORE_REIM_LOW

    /* restore regs */
    lw  s0, 0*4(k1)
    lw  s1, 1*4(k1)
    lw  s2, 2*4(k1)
    lw  s3, 3*4(k1)
    lw  s4, 4*4(k1)
    lw  s5, 5*4(k1)
    lw  s6, 6*4(k1)
    lw  s7, 7*4(k1)

    lw  gp, 8*4(k1)
    lw  sp, 9*4(k1)
    lw  fp, 10*4(k1)
    lw  ra, 11*4(k1)

    jr  ra
    nop

