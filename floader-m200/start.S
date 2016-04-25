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

.extern main
.global floader_header

.text
.set noreorder

floader_header:

#if (CONFIG_FLOADER_FUNCTION == 2)

    .org 0x800

#else

    /* magic value ("MSPL") */
    .word 0x4d53504c
    .space 1024 * 2 - 4, 0

#endif

    /* Invalidate BTB */
    mfc0    v0, CP0_CONFIG, 7
    ori v0, v0, 2
    mtc0    v0, CP0_CONFIG, 7
    nop
    nop
    nop
    nop
    nop

    mfc0    v0, $12, 2
    li  v1, 0x7fffffff
    and v0, v0, v1          /* L2C write-through */
    li v1, (0x1 << 29)      /* Disable L2C prefetch from DDR */
    or v0, v0, v1
    mtc0    v0, $12, 2
    nop
    nop
    nop
    nop
    nop

    /*
     * CU0=UM=EXL=IE=0, BEV=ERL=1, IP2~7=1
     */
    li  t0, 0x0040FC04
    mtc0    t0, CP0_STATUS
    nop
    nop
    nop
    nop
    nop

#if (CONFIG_FLOADER_FUNCTION == 2)
    /*
     * CU0=UM=EXL=IE=0, BEV=ERL=1, IP2~7=1
     */
    li  t0, 0x0040FC00
    mtc0    t0, CP0_STATUS
#else
    /*
     * CU0=UM=EXL=IE=ERL=0, BEV=1, IP2~7=1
     */
    li  t0, 0x0040FC04
    mtc0    t0, CP0_STATUS

#endif
    nop
    nop
    nop
    nop
    nop


    /* CAUSE register */
    /* IV=1, use the specical interrupt vector (0x200) */
    li  t1, 0x00800000
    mtc0    t1, CP0_CAUSE
    nop
    nop
    nop
    nop
    nop

    /*
     * Clear BSS
     */
    la  t1, __bss_start     # t1 <-- __bss_start
    la  t2, __bss_end       # t2 <-- __bss_end

1:
    sw  zero, 0(t1)
    blt t1, t2, 1b
    addi   t1, 4

    /* Set up stack */
#ifdef CONFIG_SPL_STACK
    li  sp, CONFIG_SPL_STACK
#endif

    j main
    nop



