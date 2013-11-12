#include <asm/addrspace.h>
#include <asm/cacheops.h>

#define CFG_DCACHE_SIZE         16384
#define CFG_ICACHE_SIZE         16384
#define CFG_CACHELINE_SIZE      32

#define cache_unroll(base,op)	        	\
	__asm__ __volatile__("	         	\
		.set noreorder;		        \
		.set mips3;		        \
		cache %1, (%0);	                \
		.set mips0;			\
		.set reorder"			\
		:				\
		: "r" (base),			\
		  "i" (op));

static inline void jz_sync(void)
{
	__asm__ volatile ("sync");
}
#define REG32(addr)     *((volatile u32 *)(addr))
#define CPM_CLKGR0_MAC          (1 << 21)
#define CPM_CLKGR0              (CPM_BASE+0x20)
#define REG_CPM_CLKGR0  REG32(CPM_CLKGR0)
#define __cpm_start_mac()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_MAC)

#define __gpio_clear_pin          __gpio_as_output0
#define __gpio_as_output0(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_as_output0(p, o);		\
} while (0)

#define __gpio_port_as_output0(p, o)		\
do {						\
    REG_GPIO_PXINTC(p) = (1 << (o));		\
    REG_GPIO_PXMASKS(p) = (1 << (o));		\
    REG_GPIO_PXPAT1C(p) = (1 << (o));		\
    REG_GPIO_PXPAT0C(p) = (1 << (o));		\
} while (0)
#define REG_GPIO_PXINTS(n)	REG32(GPIO_PXINTS((n)))
#define REG_GPIO_PXINTC(n)	REG32(GPIO_PXINTC((n)))
#define REG_GPIO_PXMASK(n)	REG32(GPIO_PXMASK((n)))   /* 1: mask pin interrupt */
#define REG_GPIO_PXMASKS(n)	REG32(GPIO_PXMASKS((n)))
#define REG_GPIO_PXMASKC(n)	REG32(GPIO_PXMASKC((n)))
#define REG_GPIO_PXPAT1(n)	REG32(GPIO_PXPAT1((n)))   /* 1: disable pull up/down */
#define REG_GPIO_PXPAT1S(n)	REG32(GPIO_PXPAT1S((n)))
#define REG_GPIO_PXPAT1C(n)	REG32(GPIO_PXPAT1C((n)))
#define REG_GPIO_PXPAT0(n)	REG32(GPIO_PXPAT0((n)))  /* 0:GPIO/INTR, 1:FUNC */
#define REG_GPIO_PXPAT0S(n)	REG32(GPIO_PXPAT0S((n)))
#define REG_GPIO_PXPAT0C(n)	REG32(GPIO_PXPAT0C((n)))
#define REG_GPIO_PXFLG(n)	REG32(GPIO_PXFLG((n))) /* 0:GPIO/Fun0,1:intr/fun1*/
#define REG_GPIO_PXFLGC(n)	REG32(GPIO_PXFLGC((n)))
#define REG_GPIO_PXOEN(n)	REG32(GPIO_PXOEN((n)))
#define REG_GPIO_PXOENS(n)	REG32(GPIO_PXOENS((n))) /* 0:input/low-level-trig/falling-edge-trig, 1:output/high-level-trig/rising-edge-trig */
#define REG_GPIO_PXOENC(n)	REG32(GPIO_PXOENC((n)))
#define REG_GPIO_PXPEN(n)	REG32(GPIO_PXPEN((n)))
#define REG_GPIO_PXPENS(n)	REG32(GPIO_PXPENS((n))) /* 0:Level-trigger/Fun0, 1:Edge-trigger/Fun1 */
#define REG_GPIO_PXPENC(n)	REG32(GPIO_PXPENC((n)))
#define REG_GPIO_PXDS(n)	REG32(GPIO_PXDS((n)))
#define REG_GPIO_PXDSS(n)	REG32(GPIO_PXDSS((n))) /* interrupt flag */
#define GPIO_PXMASKS(n)	(GPIO_BASE + (0x24 + (n)*0x100)) /* Port Interrupt Mask Set Reg */
#define GPIO_PXMASKC(n)	(GPIO_BASE + (0x28 + (n)*0x100)) /* Port Interrupt Mask Clear Reg */
#define __gpio_as_output1(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_as_output1(p, o);		\
} while (0)

#define __gpio_port_as_output1(p, o)		\
do {						\
    REG_GPIO_PXINTC(p) = (1 << (o));		\
    REG_GPIO_PXMASKS(p) = (1 << (o));		\
    REG_GPIO_PXPAT1C(p) = (1 << (o));		\
    REG_GPIO_PXPAT0S(p) = (1 << (o));		\
} while (0)
