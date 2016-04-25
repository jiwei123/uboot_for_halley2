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

#include <stdlib.h>
#include <stdio.h>


/*
 * Calc baud_rate rate register of UART
 */

unsigned int uartclk = 24000000;

# define do_div(n,base) ({                  \
    unsigned long long __base = (base);               \
    unsigned long long __rem;                     \
    __rem = ((unsigned long long)(n)) % __base;           \
    (n) = ((unsigned long long)(n)) / __base;             \
    __rem;                          \
 })

void calc_baud_rate_divisor(unsigned short *quot1, unsigned int baud_rate)
{
    int err, sum, i, j;
    int a[12], b[12];
    unsigned short div, umr, uacr;
    unsigned short umr_best, div_best, uacr_best;
    unsigned long long tt0, tt1, tt2, tt3;

    sum = 0;
    umr_best = div_best = uacr_best = 0;
    div = 1;

    if ((uartclk % (16 * baud_rate)) == 0) {
        quot1[0] = uartclk / (16 * baud_rate);
        quot1[1] = 16;
        quot1[2] = 0;

        return;
    }

    while (1) {
        umr = uartclk / (baud_rate * div);
        if (umr > 32) {
            div++;
            continue;
        }
        if (umr < 4) {
            break;
        }
        for (i = 0; i < 12; i++) {
            a[i] = umr;
            b[i] = 0;
            sum = 0;
            for (j = 0; j <= i; j++) {
                sum += a[j];
            }

            /* the precision could be 1/2^(36) due to the value of t0 */
            tt0 = 0x1000000000LL;
            tt1 = (i + 1) * tt0;
            tt2 = (sum * div) * tt0;
            tt3 = div * tt0;
            do_div(tt1, baud_rate);
            do_div(tt2, uartclk);
            do_div(tt3, (2 * uartclk));
            err = tt1 - tt2 - tt3;

            if (err > 0) {
                a[i] += 1;
                b[i] = 1;
            }
        }

        uacr = 0;
        for (i = 0; i < 12; i++) {
            if (b[i] == 1) {
                uacr |= 1 << i;
            }
        }
        if (div_best == 0) {
            div_best = div;
            umr_best = umr;
            uacr_best = uacr;
        }

        /* the best value of umr should be near 16, and the value of uacr should better be smaller */
        if (abs(umr - 16) < abs(umr_best - 16)
                || (abs(umr - 16) == abs(umr_best - 16) && uacr_best > uacr)) {
            div_best = div;
            umr_best = umr;
            uacr_best = uacr;
        }
        div++;
    }

    quot1[0] = div_best;
    quot1[1] = umr_best;
    quot1[2] = uacr_best;
}

int main(void)
{
    unsigned short quot[3];
    calc_baud_rate_divisor(quot, CONFIG_KERNEL_ARG_CONSOLE_BAUD_RATE);

    printf("#ifndef LPDDR2_PARAMS_H\n");
    printf("#define LPDDR2_PARAMS_H\n");

    printf("static const unsigned int UART_BAUD_DIV_BEST  = 0x%08x;\n",
            quot[0]);
    printf("static const unsigned int UART_BAUD_UMR_BEST  = 0x%08x;\n",
            quot[1]);
    printf("static const unsigned int UART_BAUD_UACR_BEST = 0x%08x;\n",
            quot[2]);

    printf("#endif\n");

    return 0;
}
