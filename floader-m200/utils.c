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

#include <stdarg.h>

static inline int isdigit(int ch)
{
    return (ch >= '0') && (ch <= '9');
}

static int strnlen(const char *s, int count)
{
    const char *sc;

    for (sc = s; count-- && *sc != '\0'; ++sc)
        /* nothing */;
    return sc - s;
}

static int skip_atoi(const char **s)
{
    int i = 0;

    while (isdigit(**s))
        i = i * 10 + *((*s)++) - '0';
    return i;
}

#define ZEROPAD 1       /* pad with zero */
#define SIGN    2       /* unsigned/signed long */
#define PLUS    4       /* show plus */
#define SPACE   8       /* space if plus */
#define LEFT    16      /* left justified */
#define SMALL   32      /* Must be 32 == 0x20 */
#define SPECIAL 64      /* 0x */

#define __do_div(n, base) ({ \
int __res; \
__res = ((unsigned long) n) % (unsigned) base; \
n = ((unsigned long) n) / (unsigned) base; \
__res; })

static char *number(char *str, long num, int base, int size, int precision,
            int type)
{
    /* we are called with base 8, 10 or 16, only, thus don't need "G..."  */
    static const char digits[16] = "0123456789ABCDEF"; /* "GHIJKLMNOPQRSTUVWXYZ"; */

    char tmp[66];
    char c, sign, locase;
    int i;

    /* locase = 0 or 0x20. ORing digits or letters with 'locase'
     * produces same digits or (maybe lowercased) letters */
    locase = (type & SMALL);
    if (type & LEFT)
        type &= ~ZEROPAD;
    if (base < 2 || base > 36)
        return NULL;
    c = (type & ZEROPAD) ? '0' : ' ';
    sign = 0;
    if (type & SIGN) {
        if (num < 0) {
            sign = '-';
            num = -num;
            size--;
        } else if (type & PLUS) {
            sign = '+';
            size--;
        } else if (type & SPACE) {
            sign = ' ';
            size--;
        }
    }
    if (type & SPECIAL) {
        if (base == 16)
            size -= 2;
        else if (base == 8)
            size--;
    }
    i = 0;
    if (num == 0)
        tmp[i++] = '0';
    else
        while (num != 0)
            tmp[i++] = (digits[__do_div(num, base)] | locase);
    if (i > precision)
        precision = i;
    size -= precision;
    if (!(type & (ZEROPAD + LEFT)))
        while (size-- > 0)
            *str++ = ' ';
    if (sign)
        *str++ = sign;
    if (type & SPECIAL) {
        if (base == 8)
            *str++ = '0';
        else if (base == 16) {
            *str++ = '0';
            *str++ = ('X' | locase);
        }
    }
    if (!(type & LEFT))
        while (size-- > 0)
            *str++ = c;
    while (i < precision--)
        *str++ = '0';
    while (i-- > 0)
        *str++ = tmp[i];
    while (size-- > 0)
        *str++ = ' ';
    return str;
}

int vsprintf(char *buf, const char *fmt, va_list args)
{
    int len;
    unsigned long num;
    int i, base;
    char *str;
    const char *s;

    int flags;      /* flags to number() */

    int field_width;    /* width of output field */
    int precision;      /* min. # of digits for integers; max
                   number of chars for from string */
    int qualifier;      /* 'h', 'l', or 'L' for integer fields */

    for (str = buf; *fmt; ++fmt) {
        if (*fmt != '%') {
            *str++ = *fmt;
            continue;
        }

        /* process flags */
        flags = 0;
          repeat:
        ++fmt;      /* this also skips first '%' */
        switch (*fmt) {
        case '-':
            flags |= LEFT;
            goto repeat;
        case '+':
            flags |= PLUS;
            goto repeat;
        case ' ':
            flags |= SPACE;
            goto repeat;
        case '#':
            flags |= SPECIAL;
            goto repeat;
        case '0':
            flags |= ZEROPAD;
            goto repeat;
        }

        /* get field width */
        field_width = -1;
        if (isdigit(*fmt))
            field_width = skip_atoi(&fmt);
        else if (*fmt == '*') {
            ++fmt;
            /* it's the next argument */
            field_width = va_arg(args, int);
            if (field_width < 0) {
                field_width = -field_width;
                flags |= LEFT;
            }
        }

        /* get the precision */
        precision = -1;
        if (*fmt == '.') {
            ++fmt;
            if (isdigit(*fmt))
                precision = skip_atoi(&fmt);
            else if (*fmt == '*') {
                ++fmt;
                /* it's the next argument */
                precision = va_arg(args, int);
            }
            if (precision < 0)
                precision = 0;
        }

        /* get the conversion qualifier */
        qualifier = -1;
        if (*fmt == 'h' || *fmt == 'l' || *fmt == 'L') {
            qualifier = *fmt;
            ++fmt;
        }

        /* default base */
        base = 10;

        switch (*fmt) {
        case 'c':
            if (!(flags & LEFT))
                while (--field_width > 0)
                    *str++ = ' ';
            *str++ = (unsigned char)va_arg(args, int);
            while (--field_width > 0)
                *str++ = ' ';
            continue;

        case 's':
            s = va_arg(args, char *);
            len = strnlen(s, precision);

            if (!(flags & LEFT))
                while (len < field_width--)
                    *str++ = ' ';
            for (i = 0; i < len; ++i)
                *str++ = *s++;
            while (len < field_width--)
                *str++ = ' ';
            continue;

        case 'p':
            if (field_width == -1) {
                field_width = 2 * sizeof(void *);
                flags |= ZEROPAD;
            }
            str = number(str,
                     (unsigned long)va_arg(args, void *), 16,
                     field_width, precision, flags);
            continue;

        case 'n':
            if (qualifier == 'l') {
                long *ip = va_arg(args, long *);
                *ip = (str - buf);
            } else {
                int *ip = va_arg(args, int *);
                *ip = (str - buf);
            }
            continue;

        case '%':
            *str++ = '%';
            continue;

            /* integer number formats - set up the flags and "break" */
        case 'o':
            base = 8;
            break;

        case 'x':
            flags |= SMALL;
        case 'X':
            base = 16;
            break;

        case 'd':
        case 'i':
            flags |= SIGN;
        case 'u':
            break;

        default:
            *str++ = '%';
            if (*fmt)
                *str++ = *fmt;
            else
                --fmt;
            continue;
        }
        if (qualifier == 'l')
            num = va_arg(args, unsigned long);
        else if (qualifier == 'h') {
            num = (unsigned short)va_arg(args, int);
            if (flags & SIGN)
                num = (short)num;
        } else if (flags & SIGN)
            num = va_arg(args, int);
        else
            num = va_arg(args, unsigned int);
        str = number(str, num, base, field_width, precision, flags);
    }
    *str = '\0';
    return str - buf;
}

int sprintf(char *buf, const char *fmt, ...)
{
    va_list args;
    int i;

    va_start(args, fmt);
    i = vsprintf(buf, fmt, args);
    va_end(args);

    return i;
}

int printf(const char *fmt, ...)
{
    char printf_buf[256];
    va_list args;
    int printed;

    va_start(args, fmt);
    printed = vsprintf(printf_buf, fmt, args);
    va_end(args);

    puts(printf_buf);

    return printed;
}

void stop(int stop_reason)
{
    printf("==========================================\n");
    printf("        Stop reason: %d\n", stop_reason);
    printf("==========================================\n");

#ifdef CONFIG_VIBRATE_GPIO
    unsigned long delay_ms = 5000;
    unsigned long delay_ms_vib = 200;
    unsigned long delay_ms_stp = 1000;

    int in_tcsm = (u32)&mdelay > 0xb0000000;

    if (get_pmu() == RICOH_5T619) {
        /*
         * Enable buck3 for vibrate
         *
         * TODO: should set by board config
         */
        pmu_5t619_set_buck5_for_vibrate(3000);
    } else {
        /*
         * TODO
         */
    }

    while (1) {
        int short_times = stop_reason;

        while (short_times--) {
            gpio_direction_output(CONFIG_VIBRATE_GPIO,
                                    CONFIG_VIBRATE_GPIO_ASSERT_LEVEL);
            if (in_tcsm)
                mdelay(delay_ms_vib / 3);
            else
                mdelay(delay_ms_vib);

            gpio_direction_output(CONFIG_VIBRATE_GPIO,
                                    !CONFIG_VIBRATE_GPIO_ASSERT_LEVEL);
            if (in_tcsm)
                mdelay(delay_ms_stp / 3);
            else
                mdelay(delay_ms_stp);
        }

        if (in_tcsm)
            mdelay(delay_ms / 3);
        else
            mdelay(delay_ms);
    }

#else

    while (1) {

    }

#endif
}

unsigned int __cpu_freq__ = 24;
void udelay(unsigned long usec)
{
    unsigned long loops = usec * (__cpu_freq__ / 2);

    __asm__ __volatile__ (
        ".set noreorder \n"
        ".align 3 \n"
        "1:bnez %0, 1b \n"
        "subu %0, 1\n"
        ".set reorder \n"
        : "=r" (loops)
        : "0" (loops)
    );
}

void mdelay(unsigned long msec)
{
    while (msec--)
        udelay(1000);
}

char *strcpy(char *dest,const char *src)
{
    char *tmp = dest;

    while ((*dest++ = *src++) != '\0')
        /* nothing */;
    return tmp;
}

void *memcpy(void *dest, const void *src, size_t count)
{
    char *d8, *s8;

    if (src == dest)
        return dest;

    d8 = (char *)dest;
    s8 = (char *)src;

    while (count--)
        *d8++ = *s8++;

    return dest;
}

void *memset(void *s,int c, size_t count)
{
    char *p = s;

    while (count--)
        *p++ = c;

    return s;
}

int memcmp(const void *cs,const void *ct,size_t count)
{
    const unsigned char *su1, *su2;
    int res = 0;

    for( su1 = cs, su2 = ct; 0 < count; ++su1, ++su2, count--)
        if ((res = *su1 - *su2) != 0)
            break;

    return res;
}

void flush_icache_all(void)
{
    u32 addr, t = 0;

    __asm__ __volatile__("mtc0 $0, $28"); /* Clear Taglo */
    __asm__ __volatile__("mtc0 $0, $29"); /* Clear TagHi */

    for (addr = CKSEG0; addr < CKSEG0 + CONFIG_SYS_ICACHE_SIZE;
         addr += CONFIG_SYS_CACHELINE_SIZE) {
        cache_op(INDEX_STORE_TAG_I, addr);
    }

    /* invalidate btb */
    __asm__ __volatile__(
        ".set mips32\n\t"
        "mfc0 %0, $16, 7\n\t"
        "nop\n\t"
        "ori %0,2\n\t"
        "mtc0 %0, $16, 7\n\t"
        ".set mips2\n\t"
        :
        : "r" (t));
}

void flush_dcache_all(void)
{
    u32 addr;

    for (addr = CKSEG0; addr < CKSEG0 + CONFIG_SYS_DCACHE_SIZE;
         addr += CONFIG_SYS_CACHELINE_SIZE) {
        cache_op(INDEX_WRITEBACK_INV_D, addr);
    }

    iob();
}

void flush_cache_all(void)
{
    flush_dcache_all();
    flush_icache_all();
}

void init_cache(void)
{
    flush_icache_all();
}

void flush_scache_all(void)
{
    unsigned long start = 0x80000000;
    unsigned long end = 0x80010000;
    unsigned long ws_inc = 0x10000;
    unsigned long ws_end = 0x80000;
    unsigned long ws, addr;

    for (ws = 0; ws < ws_end; ws += ws_inc)
        for (addr = start; addr < end; addr += 32 * 32)
            __asm__ __volatile__(
    "   .set push                   \n"
    "   .set noreorder                  \n"
    "   .set mips3                  \n"
    "   cache %1, 0x000(%0); cache %1, 0x020(%0)    \n"
    "   cache %1, 0x040(%0); cache %1, 0x060(%0)    \n"
    "   cache %1, 0x080(%0); cache %1, 0x0a0(%0)    \n"
    "   cache %1, 0x0c0(%0); cache %1, 0x0e0(%0)    \n"
    "   cache %1, 0x100(%0); cache %1, 0x120(%0)    \n"
    "   cache %1, 0x140(%0); cache %1, 0x160(%0)    \n"
    "   cache %1, 0x180(%0); cache %1, 0x1a0(%0)    \n"
    "   cache %1, 0x1c0(%0); cache %1, 0x1e0(%0)    \n"
    "   cache %1, 0x200(%0); cache %1, 0x220(%0)    \n"
    "   cache %1, 0x240(%0); cache %1, 0x260(%0)    \n"
    "   cache %1, 0x280(%0); cache %1, 0x2a0(%0)    \n"
    "   cache %1, 0x2c0(%0); cache %1, 0x2e0(%0)    \n"
    "   cache %1, 0x300(%0); cache %1, 0x320(%0)    \n"
    "   cache %1, 0x340(%0); cache %1, 0x360(%0)    \n"
    "   cache %1, 0x380(%0); cache %1, 0x3a0(%0)    \n"
    "   cache %1, 0x3c0(%0); cache %1, 0x3e0(%0)    \n"
    "   .set pop                    \n"
        :
        : "r" (addr|ws),
          "i" (0x03));;
}


#define PAGE_SHIFT  12
#define UNIQUE_ENTRYHI(idx) (CKSEG0 + ((idx) << (PAGE_SHIFT + 1)))

void local_flush_tlb_all(void)
{
    unsigned long old_ctx;
    int entry;

    /* Save old context and create impossible VPN2 value */
    old_ctx = read_c0_entryhi();
    write_c0_entrylo0(0);
    write_c0_entrylo1(0);

    entry = read_c0_wired();

    /* Blast 'em all away. */
    while (entry < 32) {
        /* Make sure all entries differ. */
        write_c0_entryhi(UNIQUE_ENTRYHI(entry));
        write_c0_index(entry);
        mtc0_tlbw_hazard();
        tlb_write_indexed();
        entry++;
    }
    tlbw_use_hazard();
    write_c0_entryhi(old_ctx);
}

void enter_sleep_with_powerkey_wake()
{
    gpio_enable_pa30_as_fall_edge_irq();

    struct m200_sleep_lib_entry *entry =
                        (struct m200_sleep_lib_entry *)SLEEP_LIB_TCSM;
    entry->enter_sleep(0);

    gpio_disable_ack_pa30_irq();
}


#if 0

void dump_mem_u8(const void *src, unsigned int size_by_byte)
{
    volatile const u8 *from = (const u8 *)src;
    const u8 *end = (const u8 *)(src + size_by_byte);

    for (; from < end; from++) {
        printf("[0x%08x] = 0x%08x\n", from, *from);
    }
}

void dump_mem_u32(const void *src, unsigned int size_by_byte)
{
    volatile const u32 *from = (const u32 *)src;
    const u32 *end = (const u32 *)(src + size_by_byte);

    for (; from < end; from++) {
        printf("[0x%08x] = 0x%08x\n", from, *from);
    }
}

#endif
