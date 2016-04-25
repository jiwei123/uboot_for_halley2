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

OUTPUT_FORMAT("elf32-tradlittlemips", "elf32-tradlittlemips", "elf32-tradlittlemips")
OUTPUT_ARCH(mips)
ENTRY(sleep_lib_entry)

SECTIONS
{
    .text : { *(.text*) }

    . = ALIGN(4);
    .rodata : { *(.rodata*) }

    . = ALIGN(4);
    .sdata : { *(.sdata*) }

    . = ALIGN(4);
    .data : { *(.data*) *(.scommon*) *(.reginfo*) }

    . = ALIGN(4);
    __bss_start = .;
    .sbss : { *(.sbss*) }
    .bss : { *(.bss*) }
    __bss_end = .;
    . = ALIGN (4);
}