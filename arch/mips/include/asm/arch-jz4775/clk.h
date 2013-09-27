#ifndef __INCLUDE__CLK__
#define __INCLUDE__CLK__

#include <asm/arch/base.h>

enum {
	APLL,
	MPLL,
	EPLL,
	VPLL,
};

enum {
	DDR = 1,
	CPU,
	BCH,
	MSC0,
	MSC1,
	MSC2,
};

struct cgu {
	unsigned off:8;
	unsigned sel:8;
	unsigned sel_bit:8;
	unsigned en_bit:8;
	unsigned busy_bit:8;
	unsigned div:8;
	unsigned reserved:16;
};

#define CGU_MSC_FREQ 24000000
#define CGU_MSC_DIV (CONFIG_SYS_APLL_FREQ / CGU_MSC_FREQ / 2 - 1)
#define CGU_DDR_DIV (CONFIG_SYS_APLL_FREQ / CONFIG_SYS_MEM_FREQ - 1)
#define CGU_BCH_DIV 0
#define CGU_LCD_DIV (CONFIG_SYS_APLL_FREQ / CONFIG_SYS_PCLK_FREQ - 1)

unsigned int clk_get_rate(int clk);
void cgu_clks_init(struct cgu *cgu_sel, int nr_cgu_clks);

#endif

