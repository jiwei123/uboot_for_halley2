/*
 * Include file for Ingenic Semiconductor's JZ4780 CPU.
 */
#ifndef __JZ4780_H__
#define __JZ4780_H__

#ifndef __ASSEMBLY__

#if 1 /* if 0, for spl program */
#define UCOS_CSP 0
#include <asm/types.h>

#if UCOS_CSP
#define __KERNEL__
#include <bsp.h>

#include <sysdefs.h>
#include <cacheops.h>
//#define KSEG0 KSEG0BASE
#else
//#include <asm/addrspace.h>
//#include <asm/cacheops.h>
#endif
//#define u32 unsigned int
//#define u8 unsigned char
//#define u16 unsigned short
//#define ulong unsigned long
#define KSEG0			0x80000000
#define CFG_DCACHE_SIZE		16384
#define CFG_ICACHE_SIZE		16384
#define CFG_CACHELINE_SIZE	32

//typedef unsigned int  size_t;

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

#if 0
static inline void jz_flush_dcache(void)
{
	unsigned long start;
	unsigned long end;

	start = KSEG0;
	end = start + CFG_DCACHE_SIZE;
	while (start < end) {
		cache_unroll(start,Index_Writeback_Inv_D);
		start += CFG_CACHELINE_SIZE;
	}
	__asm__ volatile ("sync");
}

static inline void jz_flush_icache(void)
{
	unsigned long start;
	unsigned long end;

	start = KSEG0;
	end = start + CFG_ICACHE_SIZE;
	while(start < end) {
		cache_unroll(start,Index_Invalidate_I);
		start += CFG_CACHELINE_SIZE;
	}
}
#endif

/* cpu pipeline flush */
static inline void jz_sync(void)
{
	__asm__ volatile ("sync");
}

static inline void jz_writeb(u32 address, u8 value)
{
	*((volatile u8 *)address) = value;
}

static inline void jz_writew(u32 address, u16 value)
{
	*((volatile u16 *)address) = value;
}

static inline void jz_writel(u32 address, u32 value)
{
	*((volatile u32 *)address) = value;
}

static inline u8 jz_readb(u32 address)
{
	return *((volatile u8 *)address);
}

static inline u16 jz_readw(u32 address)
{
	return *((volatile u16 *)address);
}

static inline u32 jz_readl(u32 address)
{
	return *((volatile u32 *)address);
}

static inline void udelay(unsigned int usec)
{
    //unsigned int i = usec * (336000000 / 2000000);
	unsigned int i = usec  << 5;
    __asm__ __volatile__ (
        "\t.set noreorder\n"
        "1:\n\t"
        "bne\t%0, $0, 1b\n\t"
        "addi\t%0, %0, -1\n\t"
        ".set reorder\n"
        : "=r" (i)
        : "0" (i)
    );

}
#endif

#define REG8(addr)	*((volatile u8 *)(addr))
#define REG16(addr)	*((volatile u16 *)(addr))
#define REG32(addr)	*((volatile u32 *)(addr))

#else

#define REG8(addr)	(addr)
#define REG16(addr)	(addr)
#define REG32(addr)	(addr)

#endif /* !ASSEMBLY */

//----------------------------------------------------------------------
// Boot ROM Specification
//

/* NOR Boot config */
#define JZ4760_NORBOOT_8BIT	0x00000000	/* 8-bit data bus flash */
#define JZ4760_NORBOOT_16BIT	0x10101010	/* 16-bit data bus flash */
#define JZ4760_NORBOOT_32BIT	0x20202020	/* 32-bit data bus flash */

/* NAND Boot config */
#define JZ4760_NANDBOOT_B8R3	0xffffffff	/* 8-bit bus & 3 row cycles */
#define JZ4760_NANDBOOT_B8R2	0xf0f0f0f0	/* 8-bit bus & 2 row cycles */
#define JZ4760_NANDBOOT_B16R3	0x0f0f0f0f	/* 16-bit bus & 3 row cycles */
#define JZ4760_NANDBOOT_B16R2	0x00000000	/* 16-bit bus & 2 row cycles */


//----------------------------------------------------------------------
// Register Definitions
//
/* AHB0 BUS Devices Base */
#define HARB0_BASE	0xB3000000
//#define	EMC_BASE	0xB3010000
#define	DDRC_BASE	0xB3010000
#define	MDMAC_BASE	0xB3420000
#define	LCD_BASE	0xB3050000
#define	TVE_BASE	0xB3050000
#define	SLCD_BASE	0xB3050000
#define	CIM_BASE	0xB3060000
#define	IPU_BASE	0xB3080000
/* AHB1 BUS Devices Base */
#define HARB1_BASE	0xB3200000
#define	DMAGP0_BASE	0xB3210000
#define	DMAGP1_BASE	0xB3220000
#define	DMAGP2_BASE	0xB3230000
#define	MC_BASE		0xB3250000
#define	ME_BASE		0xB3260000
#define	DEBLK_BASE	0xB3270000
#define	IDCT_BASE	0xB3280000
#define	CABAC_BASE	0xB3290000
#define	TCSM0_BASE	0xB32B0000
#define	TCSM1_BASE	0xB32C0000
#define	SRAM_BASE	0xB32D0000
/* AHB2 BUS Devices Base */
#define HARB2_BASE	0xB3400000
#define NEMC_BASE	0xB3410000
#define DMAC_BASE	0xB3420000
#define UHC_BASE	0xB3430000
#define UDC_BASE	0xB3440000
#define GPS_BASE	0xB3480000
#define ETHC_BASE	0xB34B0000
#define BCH_BASE	0xB34D0000
/* APB BUS Devices Base */
#define	CPM_BASE	0xB0000000
#define	INTC_BASE	0xB0001000
#define	TCU_BASE	0xB0002000
#define	OST_BASE	0xB0002000
#define	WDT_BASE	0xB0002000
//#define	RTC_BASE	0xB0003000
#define	GPIO_BASE	0xB0010000
#define	AIC_BASE	0xB0020000
#define	ICDC_BASE	0xB0020000
#define	MSC0_BASE	0xB3450000
#define	MSC1_BASE	0xB3460000
#define	MSC2_BASE	0xB3470000
#define	UART0_BASE	0xB0030000
#define	UART1_BASE	0xB0031000
#define	UART2_BASE	0xB0032000
#define	UART3_BASE	0xB0033000
#define	SCC_BASE	0xB0040000
#define	SSI0_BASE	0xB0043000
#define	SSI1_BASE	0xB0044000
#define	SSI2_BASE	0xB0045000
#define	I2C0_BASE	0xB0050000
#define	I2C1_BASE	0xB0051000
#define	PS2_BASE	0xB0060000
#define	SADC_BASE	0xB0070000
#define	OWI_BASE	0xB0072000
#define	TSSI_BASE	0xB0073000

/*************************************************************************
 * INTC (Interrupt Controller)
 *************************************************************************/
#define INTC_ISR(n)	(INTC_BASE + 0x00 + (n) * 0x20)
#define INTC_IMR(n)	(INTC_BASE + 0x04 + (n) * 0x20)
#define INTC_IMSR(n)	(INTC_BASE + 0x08 + (n) * 0x20)
#define INTC_IMCR(n)	(INTC_BASE + 0x0c + (n) * 0x20)
#define INTC_IPR(n)	(INTC_BASE + 0x10 + (n) * 0x20)

#define REG_INTC_ISR(n)		REG32(INTC_ISR((n)))
#define REG_INTC_IMR(n)		REG32(INTC_IMR((n)))
#define REG_INTC_IMSR(n)	REG32(INTC_IMSR((n)))
#define REG_INTC_IMCR(n)	REG32(INTC_IMCR((n)))
#define REG_INTC_IPR(n)		REG32(INTC_IPR((n)))


#define INTC_GOS                (0x20)
#define INTC_ICMSR(n)   (INTC_BASE + (n) * INTC_GOS + (0x08))



// 1st-level interrupts
#define IRQ_I2C1	0
#define IRQ_I2C0	1
#define IRQ_UART3	2
#define IRQ_UART2	3
#define IRQ_UART1	4
#define IRQ_UART0	5
#define IRQ_SSI2   	6
#define IRQ_SSI1   	7
#define IRQ_SSI0   	8
#define IRQ_TSSI	9
#define IRQ_BDMA	10
#define IRQ_KBC		11
#define IRQ_GPIO5	12
#define IRQ_GPIO4	13
#define IRQ_GPIO3	14
#define IRQ_GPIO2	15
#define IRQ_GPIO1	16
#define IRQ_GPIO0	17
#define IRQ_SADC	18
#define IRQ_ETH		19
#define IRQ_UHC		20
#define IRQ_OTG		21
#define IRQ_MDMA	22
#define IRQ_DMAC1	23
#define IRQ_DMAC0	24
#define IRQ_TCU2	25
#define IRQ_TCU1	26
#define IRQ_TCU0	27
#define IRQ_GPS		28
#define IRQ_IPU		29
#define IRQ_CIM		30
#define IRQ_LCD		31

#define IRQ_RTC		32
#define IRQ_OWI		33
#define IRQ_AIC 	34
#define IRQ_MSC2	35
#define IRQ_MSC1	36
#define IRQ_MSC0	37
#define IRQ_SCC		38
#define IRQ_BCH		39
#define IRQ_PCM		40

// 2nd-level interrupts
#define IRQ_DMA_0	64  /* 64 ~ 75 for DMAC0 channel 0 ~ 5 & DMAC1 channel 0 ~ 5 */
#define IRQ_DMA_1	(IRQ_DMA_0 + HALF_DMA_NUM)  /* 64 ~ 75 for DMAC0 channel 0 ~ 5 & DMAC1 channel 0 ~ 5 */
#define IRQ_MDMA_0	(IRQ_DMA_0 + MAX_DMA_NUM) /* 64 ~ 66 for MDMAC channel 0 ~ 2 */

#define IRQ_GPIO_0	96  /* 96 to 287 for GPIO pin 0 to 127 */

#define NUM_INTC	41
#define NUM_DMA         MAX_DMA_NUM	/* 12 */
#define NUM_MDMA        MAX_MDMA_NUM	/* 3 */
#define NUM_GPIO        MAX_GPIO_NUM	/* GPIO NUM: 192, Jz4760 real num GPIO 178 */

/*************************************************************************
 * RTC (defined in chip-rtc.h)
 *************************************************************************/
#include "chip-rtc.h"

/*************************************************************************
 * CPM (Clock reset and Power control Management)
 *************************************************************************/
#define CPM_CPCCR		(CPM_BASE+0x00) /* Clock control register		*/
#define CPM_CPCSR		(CPM_BASE+0xd4) /* Clock Status register		*/
#define CPM_CPPCR		(CPM_BASE+0x0c) /* PLL control register 		*/
#define CPM_CPAPCR		(CPM_BASE+0x10) /* APLL control Register	*/
#define CPM_CPMPCR		(CPM_BASE+0x14) /* MPLL control Register	*/
#define CPM_CPEPCR		(CPM_BASE+0x18) /* EPLL control Register	*/
#define CPM_CPVPCR		(CPM_BASE+0x1c) /* VPLL control Register	*/
#define CPM_DDCDR		(CPM_BASE+0x2c) /* DDR clock divider register	*/
#define CPM_VPUCDR		(CPM_BASE+0x30) /* VPU clock divider register	*/
#define CPM_CPSPR		(CPM_BASE+0x34) /* CPM scratch pad register		*/
#define CPM_CPSPPR		(CPM_BASE+0x38) /* CPM scratch protected register	*/
#define CPM_USBPCR		(CPM_BASE+0x3c) /* USB parameter control register	*/
#define CPM_USBRDT		(CPM_BASE+0x40) /* USB reset detect timer register	*/
#define CPM_USBVBFIL	(CPM_BASE+0x44) /* USB jitter filter register		*/
#define CPM_USBPCR1		(CPM_BASE+0x48) /* USB parameter control register 1	*/
#define CPM_USBCDR		(CPM_BASE+0x50) /* USB OTG PHY clock divider register	*/
#define CPM_I2SCDR		(CPM_BASE+0x60) /* I2S device clock divider register	*/
#define CPM_I2S1CDR		(CPM_BASE+0xa0) /* I2S1 device clock divider register	*/
#define CPM_LPCDR		(CPM_BASE+0x64) /* LCD pix clock divider register	*/
#define CPM_MSCCDR		(CPM_BASE+0x68) /* MSC clock divider register		*/
#define CPM_UHCCDR		(CPM_BASE+0x6C) /* UHC 48M clock divider register	*/
#define CPM_SSICDR		(CPM_BASE+0x74) /* SSI clock divider register		*/
#define CPM_CIMCDR		(CPM_BASE+0x7c) /* CIM MCLK clock divider register	*/
#define CPM_PCMCDR		(CPM_BASE+0x84) /* PCM device clock divider register	*/
#define CPM_GPUCDR		(CPM_BASE+0x88) /* GPU clock divider register		*/
#define CPM_BCHCDR		(CPM_BASE+0xAC) /* BCH clock divider register		*/

#define CPM_LCR			(CPM_BASE+0x04)
#define CPM_SPCR0		(CPM_BASE+0xb8) /* SRAM Power Control Register0 */
#define CPM_SPCR1		(CPM_BASE+0xbc) /* SRAM Power Control Register1 */
#define CPM_PSWCST(n)		(CPM_BASE+0x4*(n)+0x90)
#define CPM_CLKGR0		(CPM_BASE+0x20) /* Clock Gate Register0 */
#define CPM_CLKGR1		(CPM_BASE+0x28) /* Clock Gate Register1 */
#define CPM_OPCR		(CPM_BASE+0x24) /* Oscillator and Power Control Register */

#define CPM_RSR			(CPM_BASE+0x08)

/* Register */
#define REG_CPM_CPCCR		REG32(CPM_CPCCR)
#define REG_CPM_CPCSR		REG32(CPM_CPCSR)
#define REG_CPM_CPPCR		REG32(CPM_CPPCR)
#define REG_CPM_CPAPCR		REG32(CPM_CPAPCR)
#define REG_CPM_CPMPCR		REG32(CPM_CPMPCR)
#define REG_CPM_CPEPCR		REG32(CPM_CPEPCR)
#define REG_CPM_CPVPCR		REG32(CPM_CPVPCR)
#define REG_CPM_DDCDR		REG32(CPM_DDCDR)
#define REG_CPM_VPUCDR		REG32(CPM_VPUCDR)
#define REG_CPM_CPSPR		REG32(CPM_CPSPR)
#define REG_CPM_CPSPPR		REG32(CPM_CPSPPR)
#define REG_CPM_USBPCR		REG32(CPM_USBPCR)
#define REG_CPM_USBRDT		REG32(CPM_USBRDT)
#define REG_CPM_USBVBFIL	REG32(CPM_USBVBFIL)
#define REG_CPM_USBPCR1		REG32(CPM_USBPCR1)
#define REG_CPM_USBCDR		REG32(CPM_USBCDR)
#define REG_CPM_I2SCDR		REG32(CPM_I2SCDR)
#define REG_CPM_I2S1CDR		REG32(CPM_I2S1CDR)
#define REG_CPM_LPCDR		REG32(CPM_LPCDR)
#define REG_CPM_MSCCDR		REG32(CPM_MSCCDR)
#define REG_CPM_UHCCDR		REG32(CPM_UHCCDR)
#define REG_CPM_SSICDR		REG32(CPM_SSICDR)
#define REG_CPM_CIMCDR		REG32(CPM_CIMCDR)
#define REG_CPM_PCMCDR		REG32(CPM_PCMCDR)
#define REG_CPM_GPUCDR		REG32(CPM_GPUCDR)
#define REG_CPM_BCHCDR		REG32(CPM_BCHCDR)

/* BCH clock divider register */
#define CPM_BCHCDR_BPCS			(1 << 31)
#define CPM_BCHCDR_BCHM			(1 << 30) //0: hardware as default, 1: software(must change)
#define CPM_BCHCDR_BCHDIV_BIT		0
#define CPM_BCHCDR_BCHDIV_MASK		(0x7 << CPM_BCHCDR_BCHDIV_BIT)

#define __cpm_get_bchdiv(n) \
	((REG_CPM_BCHCDR(n) & CPM_BCHCDR_BCHDIV_MASK) >> CPM_BCHCDR_BCHDIV_BIT)
#define __cpm_set_bchdiv(v) \
	(REG_CPM_BCHCDR = (REG_CPM_BCHCDR & ~CPM_BCHCDR_BCHDIV_MASK) | ((v) << (CPM_BCHCDR_BCHDIV_BIT)))
#define __cpm_sw_bchm()			\
        (REG_CPM_BCHCDR |= CPM_BCHCDR_BCHM)
#define __cpm_hw_bchm()			\
        (REG_CPM_BCHCDR &= ~CPM_BCHCDR_BCHM)


#define REG_CPM_LCR	REG32(CPM_LCR)
#define REG_CPM_SPCR0	REG32(CPM_SPCR0)
#define REG_CPM_SPCR1	REG32(CPM_SPCR1)
#define REG_CPM_CLKGR0	REG32(CPM_CLKGR0)
#define REG_CPM_CLKGR1	REG32(CPM_CLKGR1)
#define REG_CPM_OPCR	REG32(CPM_OPCR)

#define REG_CPM_RSR	REG32(CPM_RSR)
/* Clock control register */
#define CPM_CPCCR_SEL_SRC_BIT		30
#define CPM_CPCCR_SEL_SRC_MASK		(0x3 << CPM_CPCCR_SEL_SRC_BIT)
 #define CPM_SRC_SEL_STOP 0	
  #define CPM_SRC_SEL_APLL  1	
  #define CPM_SRC_SEL_EXCLK 2
  #define CPM_SRC_SEL_RTCLK 3
#define CPM_CPCCR_SEL_CPLL_BIT		28
#define CPM_CPCCR_SEL_CPLL_MASK		(0x3 << CPM_CPCCR_SEL_CPLL_BIT)
#define CPM_CPCCR_SEL_H0PLL_BIT		26
#define CPM_CPCCR_SEL_H0PLL_MASK		(0x3 << CPM_CPCCR_SEL_H0PL_BIT)
#define CPM_CPCCR_SEL_H2PLL_BIT		24
#define CPM_CPCCR_SEL_H2PLL_MASK		(0x3 << CPM_CPCCR_SEL_H2PL_BIT)
  #define CPM_PLL_SEL_STOP  0
  #define CPM_PLL_SEL_SRC   1	
  #define CPM_PLL_SEL_MPLL  2
  #define CPM_PLL_SEL_EPLL  3
#define CPM_CPCCR_CE_CPU		(0x1 << 22)
#define CPM_CPCCR_CE_AHB0		(0x1 << 21)
#define CPM_CPCCR_CE_AHB2		(0x1 << 20)
#define CPM_CPCCR_PDIV_BIT		16
#define CPM_CPCCR_PDIV_MASK		(0xf << CPM_CPCCR_PDIV_BIT)
#define CPM_CPCCR_H2DIV_BIT		12
#define CPM_CPCCR_H2DIV_MASK		(0xf << CPM_CPCCR_H2DIV_BIT)
#define CPM_CPCCR_H0DIV_BIT		8
#define CPM_CPCCR_H0DIV_MASK		(0x0f << CPM_CPCCR_H0DIV_BIT)
#define CPM_CPCCR_L2DIV_BIT		4
#define CPM_CPCCR_L2DIV_MASK		(0x0f << CPM_CPCCR_L2DIV_BIT)
#define CPM_CPCCR_CDIV_BIT		0
#define CPM_CPCCR_CDIV_MASK		(0x0f << CPM_CPCCR_CDIV_BIT)

/* Clock Status register */
#define CPM_CPCSR_H2DIV_BUSY		(1 << 2)
#define CPM_CPCSR_H0DIV_BUSY		(1 << 1)
#define CPM_CPCSR_CDIV_BUSY		(1 << 0)

/* PLL control register */
#define CPM_CPPCR_PLLST_BIT		0
#define CPM_CPPCR_PLLST_MASK		(0xff << CPM_CPPCR_PLLST_BIT)

/* XPLL control register */
#define CPM_CPXPCR_XBS				(1 << 31)
#define CPM_CPXPCR_XPLLM_BIT		24
#define CPM_CPXPCR_XPLLM_MASK		(0x7f << CPM_CPXPCR_XPLLM_BIT)
#define CPM_CPXPCR_XPLLN_BIT		18
#define CPM_CPXPCR_XPLLN_MASK		(0x1f << CPM_CPXPCR_XPLLN_BIT)
#define CPM_CPXPCR_XPLLOD_BIT		16
#define CPM_CPXPCR_XPLLOD_MASK		(0x3 << CPM_CPXPCR_XPLLOD_BIT)
/* APLL*/
#define CPM_CPAPCR_XLOCK		(1 << 15)
#define CPM_CPAPCR_XPLL_ON		(1 << 10)
#define CPM_CPAPCR_XPLLBP		(1 << 9)
#define CPM_CPAPCR_XPLLEN		(1 << 8)
#define CPM_CPAPCR_XPPLST_BIT	0
#define CPM_CPAPCR_XPPLST_MASK	(0xff << CPM_CPAPCR_XPPLST_BIT)
/* MPLL */
#define CPM_CPMPCR_XPLLEN		(1 << 7)
#define CPM_CPMPCR_XPLLBP		(1 << 6)
#define CPM_CPMPCR_XLOCK		(1 << 1)
#define CPM_CPMPCR_XPLL_ON		(1 << 0)

/* CPM scratch protected register */
#define CPM_CPSPPR_BIT			0
#define CPM_CPSPPR_MASK			(0xffff << CPM_CPSPPR_BIT)

/* USB parameter control register */
#define CPM_USBPCR_USB_MODE		(1 << 31)  /* 1: OTG, 0: UDC*/
#define CPM_USBPCR_AVLD_REG		(1 << 30)
#define CPM_USBPCR_IDPULLUP_MASK_BIT	28
#define CPM_USBPCR_IDPULLUP_MASK_MASK	(0x02 << IDPULLUP_MASK_BIT)
#define CPM_USBPCR_INCR_MASK		(1 << 27)
#define CPM_USBPCR_CLK12_EN		(1 << 26)
#define CPM_USBPCR_COMMONONN		(1 << 25)
#define CPM_USBPCR_VBUSVLDEXT		(1 << 24)
#define CPM_USBPCR_VBUSVLDEXTSEL	(1 << 23)
#define CPM_USBPCR_POR			(1 << 22)
#define CPM_USBPCR_SIDDQ		(1 << 21)
#define CPM_USBPCR_OTG_DISABLE		(1 << 20)
#define CPM_USBPCR_COMPDISTUNE_BIT	17
#define CPM_USBPCR_COMPDISTUNE_MASK	(0x07 << COMPDISTUNE_BIT)
#define CPM_USBPCR_OTGTUNE_BIT		14
#define CPM_USBPCR_OTGTUNE_MASK		(0x07 << OTGTUNE_BIT)
#define CPM_USBPCR_SQRXTUNE_BIT		11
#define CPM_USBPCR_SQRXTUNE_MASK	(0x7x << SQRXTUNE_BIT)
#define CPM_USBPCR_TXFSLSTUNE_BIT	7
#define CPM_USBPCR_TXFSLSTUNE_MASK	(0x0f << TXFSLSTUNE_BIT)
#define CPM_USBPCR_TXPREEMPHTUNE	(1 << 6)
#define CPM_USBPCR_TXRISETUNE_BIT	4
#define CPM_USBPCR_TXRISETUNE_MASK	(0x03 << TXRISETUNE_BIT)
#define CPM_USBPCR_TXVREFTUNE_BIT	0
#define CPM_USBPCR_TXVREFTUNE_MASK	(0x0f << TXVREFTUNE_BIT)

/* DDR memory clock divider register */
#define CPM_DDRCDR_DCS_BIT		30
#define CPM_DDRCDR_DCS_MASK		(0x3 << CPM_DDRCDR_DCS_BIT)
  #define CPM_DDRCDR_DCS_STOP		(0x0 << CPM_DDRCDR_DCS_BIT)
  #define CPM_DDRCDR_DCS_SRC		(0x1 << CPM_DDRCDR_DCS_BIT)
  #define CPM_DDRCDR_DCS_MPLL		(0x2 << CPM_DDRCDR_DCS_BIT)
#define CPM_DDRCDR_CE_DDR		(1 << 29)
#define CPM_DDRCDR_DDR_BUSY		(1 << 28)
#define CPM_DDRCDR_DDR_STOP		(1 << 27)
#define CPM_DDRCDR_DDRDIV_BIT		0
#define CPM_DDRCDR_DDRDIV_MASK		(0xf << CPM_DDRCDR_DDRDIV_BIT)

/* USB reset detect timer register */
#define CPM_USBRDT_VBFIL_LD_EN		(1 << 25)
#define CPM_USBRDT_IDDIG_EN		(1 << 24)
#define CPM_USBRDT_IDDIG_REG		(1 << 23)
#define CPM_USBRDT_USBRDT_BIT		0
#define CPM_USBRDT_USBRDT_MASK		(0x7fffff << CPM_USBRDT_USBRDT_BIT)

/* USB OTG PHY clock divider register */
#define CPM_USBCDR_UCS			(1 << 31)
#define CPM_USBCDR_UPCS			(1 << 30)
#define CPM_USBCDR_OTGDIV_BIT		0
#define CPM_USBCDR_OTGDIV_MASK		(0x3f << CPM_USBCDR_OTGDIV_BIT)

/* I2S device clock divider register */
#define CPM_I2SCDR_I2CS			(1 << 31)
#define CPM_I2SCDR_I2PCS		(1 << 30)
#define CPM_I2SCDR_I2SDIV_BIT		0
#define CPM_I2SCDR_I2SDIV_MASK		(0x1ff << CPM_I2SCDR_I2SDIV_BIT)

/* LCD pix clock divider register */
#define CPM_LPCDR_LSCS			(1 << 31)
#define CPM_LPCDR_LTCS			(1 << 30)
#define CPM_LPCDR_LPCS			(1 << 29)
#define CPM_LPCDR_PIXDIV_BIT		0
#define CPM_LPCDR_PIXDIV_MASK		(0x7ff << CPM_LPCDR_PIXDIV_BIT)

/* MSC clock divider register */
#define CPM_MSCCDR_MPCS_BIT		30
#define CPM_MSCCDR_MPCS_MASK		(3 << CPM_MSCCDR_MPCS_BIT)
  #define CPM_MSCCDR_MPCS_STOP		(0x0 << CPM_MSCCDR_MPCS_BIT)
  #define CPM_MSCCDR_MPCS_SRC		(0x1 << CPM_MSCCDR_MPCS_BIT)
  #define CPM_MSCCDR_MPCS_MPLL		(0x2 << CPM_MSCCDR_MPCS_BIT)
#define CPM_MSCCDR_CE			(1 << 29)
#define CPM_MSCCDR_MSC_BUSY		(1 << 28)
#define CPM_MSCCDR_MSC_STOP		(1 << 27)
#define CPM_MSCCDR_MSC_CLK0_SEL		(1 << 15)
#define CPM_MSCCDR_MSCDIV_BIT		0
#define CPM_MSCCDR_MSCDIV_MASK		(0xff << CPM_MSCCDR_MSCDIV_BIT)

/* UHC 48M clock divider register */
#define CPM_UHCCDR_UHCS_BIT		30
#define CPM_UHCCDR_UHCS_MASK		(0x3 << CPM_UHCCDR_UHCS_BIT)
  #define CPM_UHCCDR_UHCS_SRC		(0x0 << CPM_UHCCDR_UHCS_BIT)
  #define CPM_UHCCDR_UHCS_MPLL		(0x1 << CPM_UHCCDR_UHCS_BIT)
  #define CPM_UHCCDR_UHCS_EPLL		(0x2 << CPM_UHCCDR_UHCS_BIT)
  #define CPM_UHCCDR_UHCS_OTG		(0x3 << CPM_UHCCDR_UHCS_BIT)
#define CPM_UHCCDR_CE_UHC		(1 << 29)
#define CPM_UHCCDR_UHC_BUSY		(1 << 28)
#define CPM_UHCCDR_UHC_STOP		(1 << 27)
#define CPM_UHCCDR_UHCDIV_BIT		0
#define CPM_UHCCDR_UHCDIV_MASK		(0xff << CPM_UHCCDR_UHCDIV_BIT)

/* SSI clock divider register */
#define CPM_SSICDR_SCS			(1 << 31)
#define CPM_SSICDR_SSIDIV_BIT		0
#define CPM_SSICDR_SSIDIV_MASK		(0x3f << CPM_SSICDR_SSIDIV_BIT)

/* CIM MCLK clock divider register */
#define CPM_CIMCDR_CIMDIV_BIT		0
#define CPM_CIMCDR_CIMDIV_MASK		(0xff << CPM_CIMCDR_CIMDIV_BIT)

/* GPS clock divider register */
#define CPM_GPSCDR_GPCS			(1 << 31)
#define CPM_GPSCDR_GPSDIV_BIT		0
#define CPM_GSPCDR_GPSDIV_MASK		(0xf << CPM_GPSCDR_GPSDIV_BIT)

/* PCM device clock divider register */
#define CPM_PCMCDR_PCMS			(1 << 31)
#define CPM_PCMCDR_PCMPCS		(1 << 30)
#define CPM_PCMCDR_PCMDIV_BIT		0
#define CPM_PCMCDR_PCMDIV_MASK		(0x1ff << CPM_PCMCDR_PCMDIV_BIT)

/* GPU clock divider register */
#define CPM_GPUCDR_GPCS			(1 << 31)
#define CPM_GPUCDR_GPUDIV_BIT		0
#define CPM_GPUCDR_GPUDIV_MASK		(0x7 << CPM_GPUCDR_GPUDIV_BIT)

/* BCH clock divider register */
#define CPM_BCHCDR_BPCS_BIT		30
#define CPM_BCHCDR_BPCS_MASK		(0x3 << CPM_BCHCDR_BPCS_BIT)
  #define CPM_BCHCDR_BPCS_STOP		(0X0 << CPM_BCHCDR_BPCS_BIT)
  #define CPM_BCHCDR_BPCS_SRC_CLK	(0x1 << CPM_BCHCDR_BPCS_BIT)
  #define CPM_BCHCDR_BPCS_MPLL		(0x2 << CPM_BCHCDR_BPCS_BIT)
  #define CPM_BCHCDR_BPCS_EPLL		(0x3 << CPM_BCHCDR_BPCS_BIT)
#define CPM_BCHCDR_CE_BCH		(1 << 29)
#define CPM_BCHCDR_BCH_BUSY		(1 << 28)
#define CPM_BCHCDR_BCH_STOP		(1 << 27)
#define CPM_BCHCDR_BCHCDR_BIT		0
#define CPM_BCHCDR_BCHCDR_MASK		(0x7 << CPM_BCHCDR_BCHCDR_BIT)

/* Low Power Control Register */
#define CPM_LCR_PD_SCPU		(1 << 31)
#define CPM_LCR_PD_VPU		(1 << 30)
#define CPM_LCR_PD_GPU		(1 << 29)
#define CPM_LCR_PD_GPS		(1 << 28)
#define CPM_LCR_SCPUS		(1 << 27)
#define CPM_LCR_VPUS		(1 << 26)
#define CPM_LCR_GPUS		(1 << 25)
#define CPM_LCR_GPSS		(1 << 24)
#define CPM_LCR_GPU_IDLE	(1 << 20)
#define CPM_LCR_PST_BIT 	8
#define CPM_LCR_PST_MASK 	(0xfff << CPM_LCR_PST_BIT)
#define CPM_LCR_DOZE_DUTY_BIT 	3
#define CPM_LCR_DOZE_DUTY_MASK 	(0x1f << CPM_LCR_DOZE_DUTY_BIT)
#define CPM_LCR_DOZE_ON		(1 << 2)
#define CPM_LCR_LPM_BIT		0
#define CPM_LCR_LPM_MASK	(0x3 << CPM_LCR_LPM_BIT)
  #define CPM_LCR_LPM_IDLE	(0x0 << CPM_LCR_LPM_BIT)
  #define CPM_LCR_LPM_SLEEP	(0x1 << CPM_LCR_LPM_BIT)

/* Clock Gate Register0 */
#define CPM_CLKGR0_DDR1		(1 << 31)
#define CPM_CLKGR0_DDR0 	(1 << 30)
#define CPM_CLKGR0_IPU    	(1 << 29)
#define CPM_CLKGR0_LCD		(1 << 28)
#define CPM_CLKGR0_TVE  	(1 << 27)
#define CPM_CLKGR0_CIM    	(1 << 26)
#define CPM_CLKGR0_I2C2    	(1 << 25)
#define CPM_CLKGR0_UHC    	(1 << 24)
#define CPM_CLKGR0_MAC    	(1 << 23)
#define CPM_CLKGR0_GPS    	(1 << 22)
#define CPM_CLKGR0_PDMAC    	(1 << 21)
#define CPM_CLKGR0_SSI2    	(1 << 20)
#define CPM_CLKGR0_SSI1    	(1 << 19)
#define CPM_CLKGR0_UART3    	(1 << 18)
#define CPM_CLKGR0_UART2    	(1 << 17)
#define CPM_CLKGR0_UART1    	(1 << 16)
#define CPM_CLKGR0_UART0	(1 << 15)
#define CPM_CLKGR0_SADC		(1 << 14)
#define CPM_CLKGR0_KBC		(1 << 13)
#define CPM_CLKGR0_MSC2		(1 << 12)
#define CPM_CLKGR0_MSC1		(1 << 11)
#define CPM_CLKGR0_OWI		(1 << 10)
#define CPM_CLKGR0_TSSI		(1 << 9)
#define CPM_CLKGR0_AIC		(1 << 8)
#define CPM_CLKGR0_SCC		(1 << 7)
#define CPM_CLKGR0_I2C1		(1 << 6)
#define CPM_CLKGR0_I2C0		(1 << 5)
#define CPM_CLKGR0_SSI0		(1 << 4)
#define CPM_CLKGR0_MSC0		(1 << 3)
#define CPM_CLKGR0_OTG		(1 << 2)
#define CPM_CLKGR0_BCH		(1 << 1)
#define CPM_CLKGR0_NEMC		(1 << 0)

/* Clock Gate Register1 */
#define CPM_CLKGR1_P1		(1 << 15)
#define CPM_CLKGR1_X2D		(1 << 14)
#define CPM_CLKGR1_DES		(1 << 13)
#define CPM_CLKGR1_I2C4		(1 << 12)
#define CPM_CLKGR1_AHB		(1 << 11)
#define CPM_CLKGR1_UART4	(1 << 10)
#define CPM_CLKGR1_HDMI		(1 << 9)
#define CPM_CLKGR1_OTG1		(1 << 8)
#define CPM_CLKGR1_GPVLC	(1 << 7)
#define CPM_CLKGR1_AIC1 	(1 << 6)
#define CPM_CLKGR1_COMPRES	(1 << 5)
#define CPM_CLKGR1_GPU		(1 << 4)
#define CPM_CLKGR1_PCM		(1 << 3)
#define CPM_CLKGR1_VPU		(1 << 2)
#define CPM_CLKGR1_TSSI1	(1 << 1)
#define CPM_CLKGR1_I2C3		(1 << 0)

/* Oscillator and Power Control Register */
#define CPM_OPCR_O1ST_BIT	8
#define CPM_OPCR_O1ST_MASK	(0xff << CPM_OPCR_O1ST_BIT)
#define CPM_OPCR_SPENDN		(1 << 7)
#define CPM_OPCR_GPSEN		(1 << 6)
#define CPM_OPCR_SPENDH		(1 << 5)
#define CPM_OPCR_O1SE		(1 << 4) /* */
#define CPM_OPCR_ERCS           (1 << 2) /* 0: select EXCLK/512 clock, 1: RTCLK clock */
#define CPM_OPCR_USBM           (1 << 0) /* 0: select EXCLK/512 clock, 1: RTCLK clock */


/* Reset Status Register */
#define CPM_RSR_P0R		(1 << 2)
#define CPM_RSR_WR		(1 << 1)
#define CPM_RSR_PR		(1 << 0)


/*************************************************************************
 * MDMAC (MEM Copy DMA Controller)
 *************************************************************************/

#define MAX_MDMA_NUM	3  /* max 3 channels */

/* m is the DMA controller index (0, 1), n is the DMA channel index (0 - 11) */

#define MDMAC_DSAR(n)		(MDMAC_BASE + (0x00 + (n) * 0x20)) /* DMA source address */
#define MDMAC_DTAR(n)  		(MDMAC_BASE + (0x04 + (n) * 0x20)) /* DMA target address */
#define MDMAC_DTCR(n)  		(MDMAC_BASE + (0x08 + (n) * 0x20)) /* DMA transfer count */
#define MDMAC_DRSR(n)  		(MDMAC_BASE + (0x0c + (n) * 0x20)) /* DMA request source */
#define MDMAC_DCCSR(n) 		(MDMAC_BASE + (0x10 + (n) * 0x20)) /* DMA control/status */
#define MDMAC_DCMD(n)  		(MDMAC_BASE + (0x14 + (n) * 0x20)) /* DMA command */
#define MDMAC_DDA(n)   		(MDMAC_BASE + (0x18 + (n) * 0x20)) /* DMA descriptor address */
#define MDMAC_DSD(n)   		(MDMAC_BASE + (0xc0 + (n) * 0x04)) /* DMA Stride Address */

#define MDMAC_DMACR		(MDMAC_BASE + 0x0300) /* DMA control register */
#define MDMAC_DMAIPR		(MDMAC_BASE + 0x0304) /* DMA interrupt pending */
#define MDMAC_DMADBR		(MDMAC_BASE + 0x0308) /* DMA doorbell */
#define MDMAC_DMADBSR		(MDMAC_BASE + 0x030C) /* DMA doorbell set */
#define MDMAC_DMACKE  		(MDMAC_BASE + 0x0310)
#define MDMAC_DMACKES  		(MDMAC_BASE + 0x0314)
#define MDMAC_DMACKEC  		(MDMAC_BASE + 0x0318)

#define REG_MDMAC_DSAR(n)	REG32(MDMAC_DSAR((n)))
#define REG_MDMAC_DTAR(n)	REG32(MDMAC_DTAR((n)))
#define REG_MDMAC_DTCR(n)	REG32(MDMAC_DTCR((n)))
#define REG_MDMAC_DRSR(n)	REG32(MDMAC_DRSR((n)))
#define REG_MDMAC_DCCSR(n)	REG32(MDMAC_DCCSR((n)))
#define REG_MDMAC_DCMD(n)	REG32(MDMAC_DCMD((n)))
#define REG_MDMAC_DDA(n)	REG32(MDMAC_DDA((n)))
#define REG_MDMAC_DSD(n)        REG32(MDMAC_DSD(n))
#define REG_MDMAC_DMACR		REG32(MDMAC_DMACR)
#define REG_MDMAC_DMAIPR	REG32(MDMAC_DMAIPR)
#define REG_MDMAC_DMADBR	REG32(MDMAC_DMADBR)
#define REG_MDMAC_DMADBSR	REG32(MDMAC_DMADBSR)
#define REG_MDMAC_DMACKE     	REG32(MDMAC_DMACKE)
#define REG_MDMAC_DMACKES     	REG32(MDMAC_DMACKES)
#define REG_MDMAC_DMACKEC     	REG32(MDMAC_DMACKEC)

/*************************************************************************
 * DMAC (DMA Controller)
 *************************************************************************/

#define MAX_DMA_NUM	32  /* max 32 channels */
#define HALF_DMA_NUM	16   /* the number of one dma controller's channels */

/* m is the DMA controller index (0, 1), n is the DMA channel index (0 - 11) */

#define DMAC_DSAR(n)  (DMAC_BASE + 0x00 + (n * 0x20)) /* DMA source address */
#define DMAC_DTAR(n)  (DMAC_BASE + 0x04 + (n * 0x20)) /* DMA target address */
#define DMAC_DTCR(n)  (DMAC_BASE + 0x08 + (n * 0x20)) /* DMA transfer count */
#define DMAC_DRSR(n)  (DMAC_BASE + 0x0c + (n * 0x20)) /* DMA request source */
#define DMAC_DCCSR(n) (DMAC_BASE + 0x10 + (n * 0x20)) /* DMA control/status */
#define DMAC_DCMD(n)  (DMAC_BASE + 0x14 + (n * 0x20)) /* DMA command */
#define DMAC_DDA(n)   (DMAC_BASE + 0x18 + (n * 0x20)) /* DMA descriptor address */
#define DMAC_DSD(n)   (DMAC_BASE + 0x1c + (n * 0x20)) /* DMA Stride Difference */

#define DMAC_DMACR	(DMAC_BASE + 0x1000)    /* DMA control register */
#define DMAC_DMAIPR	(DMAC_BASE + 0x1004)    /* DMA interrupt pending */
#define DMAC_DMADBR	(DMAC_BASE + 0x1008)    /* DMA doorbell */
#define DMAC_DMADBSR	(DMAC_BASE + 0x100c)	/* DMA doorbell set */
#define DMAC_DMACPR	(DMAC_BASE + 0x101c)	/* DMA channel programmable */	  
#define DMAC_DMASIPR	(DMAC_BASE + 0x1020)	/* DMA Soft IRQ Pending */
#define DMAC_DMASIMR	(DMAC_BASE + 0x1024)	/* DMA Soft IRQ Mask */
#define DMAC_DMAIPR_MCU	(DMAC_BASE + 0x1028)	/* DMA channel IRQ Pending to MCU */
#define DMAC_DMAIMR_MCU	(DMAC_BASE + 0x1028)	/* DMA channel IRQ to MCU Mask */


#define REG_DMAC_DSAR(n)	REG32(DMAC_DSAR((n)))
#define REG_DMAC_DTAR(n)	REG32(DMAC_DTAR((n)))
#define REG_DMAC_DTCR(n)	REG32(DMAC_DTCR((n)))
#define REG_DMAC_DRSR(n)	REG32(DMAC_DRSR((n)))
#define REG_DMAC_DCCSR(n)	REG32(DMAC_DCCSR((n)))
#define REG_DMAC_DCMD(n)	REG32(DMAC_DCMD((n)))
#define REG_DMAC_DDA(n)		REG32(DMAC_DDA((n)))
#define REG_DMAC_DSD(n)         REG32(DMAC_DSD(n))

#define REG_DMAC_DMACR(m)	REG32(DMAC_DMACR)
#define REG_DMAC_DMAIPR(m)	REG32(DMAC_DMAIPR)
#define REG_DMAC_DMADBR(m)	REG32(DMAC_DMADBR)
#define REG_DMAC_DMADBSR(m)	REG32(DMAC_DMADBSR)
#define REG_DMAC_DMACPR(m)	REG32(DMAC_DMACPR)
#define REG_DMAC_DMASIPR(m)	REG32(DMAC_DMASIPR)
#define REG_DMAC_DMASIMR(m)	REG32(DMAC_DMASIMR)
#define REG_DMAC_DMAIPR_MCU(m)	REG32(DMAC_DMAIPR_MCU)
#define REG_DMAC_DMAIMR_MCU(m)	REG32(DMAC_DMAIMR_MCU)

// DMA request source register
#define DMAC_DRSR_RS_BIT	0
#define DMAC_DRSR_RS_MASK	(0x1f << DMAC_DRSR_RS_BIT)
  //#define DMAC_DRSR_RS_EXT	(0 << DMAC_DRSR_RS_BIT)
  //#define DMAC_DRSR_RS_NAND	(1 << DMAC_DRSR_RS_BIT)
  //#define DMAC_DRSR_RS_BCH_ENC(2 << DMAC_DRSR_RS_BIT)
  //#define DMAC_DRSR_RS_BCH_DEC(3 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2S1OUT	(4 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2S1IN	(5 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2S0OUT	(6 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2S0IN	(7 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_AUTO	(8 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_SADCIN	(9 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART4OUT	(12 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART4IN	(13 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART3OUT	(14 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART3IN	(15 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART2OUT	(16 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART2IN	(17 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART1OUT	(18 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART1IN	(19 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART0OUT	(20 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_UART0IN	(21 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_SSI0OUT	(22 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_SSI0IN	(23 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_SSI1OUT	(24 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_SSI1IN	(25 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_MSC0OUT	(26 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_MSC0IN	(27 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_MSC1OUT	(28 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_MSC1IN	(29 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_MSC2OUT	(30 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_MSC2IN	(31 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_PCM0OUT	(32 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_PCM0IN	(33 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_PCM1OUT	(34 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_PCM1IN	(35 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2C0OUT	(36 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2C0IN	(37 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2C1OUT	(38 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2C1IN	(39 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2C2OUT	(40 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2C2IN	(41 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2C3OUT	(42 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2C3IN	(43 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2C4OUT	(44 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_I2C4IN	(45 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_DESOUT	(46 << DMAC_DRSR_RS_BIT)
  #define DMAC_DRSR_RS_DESIN	(47 << DMAC_DRSR_RS_BIT)

// DMA channel control/status register
#define DMAC_DCCSR_NDES		(1 << 31) /* descriptor (0) or not (1) ? */
#define DMAC_DCCSR_DES8    	(1 << 30) /* Descriptor 8 Word */
#define DMAC_DCCSR_DES4    	(0 << 30) /* Descriptor 4 Word */
#define DMAC_TOC_BIT		16	  /* Time out counter */
#define DMAC_TOC_MASK		(0x3fff << DMAC_TOC_BIT)
#define DMAC_DCCSR_CDOA_BIT	16        /* copy of DMA offset address */
#define DMAC_DCCSR_CDOA_MASK	(0xff << DMAC_DCCSR_CDOA_BIT)
#define DMAC_DCCSR_BERR		(1 << 7)  /* BCH error within this transfer, Only for channel 0 */
#define DMAC_DCCSR_INV		(1 << 6)  /* descriptor invalid */
#define DMAC_DCCSR_AR		(1 << 4)  /* address error */
#define DMAC_DCCSR_TT		(1 << 3)  /* transfer terminated */
#define DMAC_DCCSR_HLT		(1 << 2)  /* DMA halted */
#define DMAC_DCCSR_TOE		(1 << 1)  /* time out enable for transaction of a data unit */
#define DMAC_DCCSR_EN		(1 << 0)  /* channel enable bit */

// DMA channel command register
#if 0
#define DMAC_DCMD_EACKS_LOW  	(1 << 31) /* External DACK Output Level Select, active low */
#define DMAC_DCMD_EACKS_HIGH  	(0 << 31) /* External DACK Output Level Select, active high */
#define DMAC_DCMD_EACKM_WRITE 	(1 << 30) /* External DACK Output Mode Select, output in write cycle */
#define DMAC_DCMD_EACKM_READ 	(0 << 30) /* External DACK Output Mode Select, output in read cycle */
#define DMAC_DCMD_ERDM_BIT      28        /* External DREQ Detection Mode Select */
#define DMAC_DCMD_ERDM_MASK     (0x03 << DMAC_DCMD_ERDM_BIT)
  #define DMAC_DCMD_ERDM_LOW    (0 << DMAC_DCMD_ERDM_BIT)
  #define DMAC_DCMD_ERDM_FALL   (1 << DMAC_DCMD_ERDM_BIT)
  #define DMAC_DCMD_ERDM_HIGH   (2 << DMAC_DCMD_ERDM_BIT)
  #define DMAC_DCMD_ERDM_RISE   (3 << DMAC_DCMD_ERDM_BIT)
#endif
#define DMAC_DCMD_SID_BIT       26        /* Source identification, only of channel 0/1 */
  #define DMAC_DCMD_SID_MASK	(0x03 << DMAC_DCMD_SID_BIT)
  #define DMAC_DCMD_SID_TCSM	(0 << DMAC_DCMD_SID_BIT)
  #define DMAC_DCMD_SID_BCH	(1 << DMAC_DCMD_SID_BIT)
  #define DMAC_DCMD_SID_DDR	(2 << DMAC_DCMD_SID_BIT)
#define DMAC_DCMD_DID_BIT       24        /* Destination identification, only of channel 0/1 */
  #define DMAC_DCMD_DID_MASK	(0x03 << DMAC_DCMD_DID_BIT)
  #define DMAC_DCMD_DID_TCSM	(0 << DMAC_DCMD_DID_BIT)
  #define DMAC_DCMD_DID_BCH	(1 << DMAC_DCMD_DID_BIT)
  #define DMAC_DCMD_DID_DDR	(2 << DMAC_DCMD_DID_BIT)
#define DMAC_DCMD_SAI		(1 << 23) /* source address increment */
#define DMAC_DCMD_DAI		(1 << 22) /* dest address increment */
#define DMAC_DCMD_RDIL_BIT	16        /* request detection interval length */
#define DMAC_DCMD_RDIL_MASK	(0x0f << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_IGN	(0 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_1	(1 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_2	(2 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_3	(3 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_4	(4 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_8	(5 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_16	(6 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_32	(7 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_64	(8 << DMAC_DCMD_RDIL_BIT)
  #define DMAC_DCMD_RDIL_128	(9 << DMAC_DCMD_RDIL_BIT)
#define DMAC_DCMD_SWDH_BIT	14  /* source port width */
#define DMAC_DCMD_SWDH_MASK	(0x03 << DMAC_DCMD_SWDH_BIT)
  #define DMAC_DCMD_SWDH_32	(0 << DMAC_DCMD_SWDH_BIT)
  #define DMAC_DCMD_SWDH_8	(1 << DMAC_DCMD_SWDH_BIT)
  #define DMAC_DCMD_SWDH_16	(2 << DMAC_DCMD_SWDH_BIT)
#define DMAC_DCMD_DWDH_BIT	12  /* dest port width */
#define DMAC_DCMD_DWDH_MASK	(0x03 << DMAC_DCMD_DWDH_BIT)
  #define DMAC_DCMD_DWDH_32	(0 << DMAC_DCMD_DWDH_BIT)
  #define DMAC_DCMD_DWDH_8	(1 << DMAC_DCMD_DWDH_BIT)
  #define DMAC_DCMD_DWDH_16	(2 << DMAC_DCMD_DWDH_BIT)
#define DMAC_DCMD_DS_BIT	8  /* transfer data size of a data unit */
#define DMAC_DCMD_DS_MASK	(0x07 << DMAC_DCMD_DS_BIT)
  #define DMAC_DCMD_DS_32BIT	(0 << DMAC_DCMD_DS_BIT)
  #define DMAC_DCMD_DS_8BIT	(1 << DMAC_DCMD_DS_BIT)
  #define DMAC_DCMD_DS_16BIT	(2 << DMAC_DCMD_DS_BIT)
  #define DMAC_DCMD_DS_16BYTE	(3 << DMAC_DCMD_DS_BIT)
  #define DMAC_DCMD_DS_32BYTE	(4 << DMAC_DCMD_DS_BIT)
  #define DMAC_DCMD_DS_64BYTE	(5 << DMAC_DCMD_DS_BIT)
  #define DMAC_DCMD_DS_128BYTE	(6 << DMAC_DCMD_DS_BIT)
  #define DMAC_DCMD_DS_ATUO	(7 << DMAC_DCMD_DS_BIT)
#define DMAC_DCMD_STDE   	(1 << 2) /* Stride Disable/Enable */
#define DMAC_DCMD_TIE		(1 << 1)  /* DMA transfer interrupt enable */
#define DMAC_DCMD_LINK		(1 << 0)  /* descriptor link enable */

// DMA descriptor address register
#define DMAC_DDA_BASE_BIT	12  /* descriptor base address */
#define DMAC_DDA_BASE_MASK	(0x0fffff << DMAC_DDA_BASE_BIT)
#define DMAC_DDA_OFFSET_BIT	4   /* descriptor offset address */
#define DMAC_DDA_OFFSET_MASK	(0x0ff << DMAC_DDA_OFFSET_BIT)

// DMA stride address register
#define DMAC_DSD_TSD_BIT        16  /* target stride address */
#define DMAC_DSD_TSD_MASK      	(0xffff << DMAC_DSD_TSD_BIT)
#define DMAC_DSD_SSD_BIT        0  /* source stride address */
#define DMAC_DSD_SSD_MASK      	(0xffff << DMAC_DSD_SSD_BIT)

// DMA control register
#define DMAC_DMACR_FMSC		(1 << 31)  /* MSC Fast DMA mode */
#define DMAC_DMACR_FSSI		(1 << 30)  /* SSI Fast DMA mode */
#define DMAC_DMACR_FTSSI	(1 << 29)  /* TSSI Fast DMA mode */
#define DMAC_DMACR_FUART	(1 << 28)  /* UART Fast DMA mode */
#define DMAC_DMACR_FAIC		(1 << 27)  /* AIC Fast DMA mode */
#define DMAC_DMACR_INTCC_BIT	17	/* Which channel is bound with INTC_IRQ */
  #define DMAC_DMACR_INTCC_MASK	(0x1f << DMAC_DMACR_INTCC_BIT)
#define DMAC_DMACR_INTCE	16	/* Enable INTCC */
#define DMAC_DMACR_HLT		(1 << 3)  /* DMA halt flag */
#define DMAC_DMACR_AR		(1 << 2)  /* address error flag */
#define DMAC_DMACR_CH01		(1 << 1)  /* Special channel 0 and channel 1 enable */
#define DMAC_DMACR_DMAE		(1 << 0)  /* DMA enable bit */

// DMA doorbell register
#define DMAC_DMADBR_DB5		(1 << 5)  /* doorbell for channel 5 */
#define DMAC_DMADBR_DB4		(1 << 4)  /* doorbell for channel 4 */
#define DMAC_DMADBR_DB3		(1 << 3)  /* doorbell for channel 3 */
#define DMAC_DMADBR_DB2		(1 << 2)  /* doorbell for channel 2 */
#define DMAC_DMADBR_DB1		(1 << 1)  /* doorbell for channel 1 */
#define DMAC_DMADBR_DB0		(1 << 0)  /* doorbell for channel 0 */

// DMA doorbell set register
#define DMAC_DMADBSR_DBS5	(1 << 5)  /* enable doorbell for channel 5 */
#define DMAC_DMADBSR_DBS4	(1 << 4)  /* enable doorbell for channel 4 */
#define DMAC_DMADBSR_DBS3	(1 << 3)  /* enable doorbell for channel 3 */
#define DMAC_DMADBSR_DBS2	(1 << 2)  /* enable doorbell for channel 2 */
#define DMAC_DMADBSR_DBS1	(1 << 1)  /* enable doorbell for channel 1 */
#define DMAC_DMADBSR_DBS0	(1 << 0)  /* enable doorbell for channel 0 */

// DMA interrupt pending register
#define DMAC_DMAIPR_CIRQ5	(1 << 5)  /* irq pending status for channel 5 */
#define DMAC_DMAIPR_CIRQ4	(1 << 4)  /* irq pending status for channel 4 */
#define DMAC_DMAIPR_CIRQ3	(1 << 3)  /* irq pending status for channel 3 */
#define DMAC_DMAIPR_CIRQ2	(1 << 2)  /* irq pending status for channel 2 */
#define DMAC_DMAIPR_CIRQ1	(1 << 1)  /* irq pending status for channel 1 */
#define DMAC_DMAIPR_CIRQ0	(1 << 0)  /* irq pending status for channel 0 */


/*************************************************************************
 * GPIO (General-Purpose I/O Ports)
 *************************************************************************/
#define MAX_GPIO_NUM	192

//n = 0,1,2,3,4,5
#define GPIO_PXPIN(n)	(GPIO_BASE + (0x00 + (n)*0x100)) /* PIN Level Register */
#define GPIO_PXINT(n)	(GPIO_BASE + (0x10 + (n)*0x100)) /* Port Interrupt Register */
#define GPIO_PXINTS(n)	(GPIO_BASE + (0x14 + (n)*0x100)) /* Port Interrupt Set Register */
#define GPIO_PXINTC(n)	(GPIO_BASE + (0x18 + (n)*0x100)) /* Port Interrupt Clear Register */
#define GPIO_PXMASK(n)	(GPIO_BASE + (0x20 + (n)*0x100)) /* Port Interrupt Mask Register */
#define GPIO_PXMASKS(n)	(GPIO_BASE + (0x24 + (n)*0x100)) /* Port Interrupt Mask Set Reg */
#define GPIO_PXMASKC(n)	(GPIO_BASE + (0x28 + (n)*0x100)) /* Port Interrupt Mask Clear Reg */
#define GPIO_PXPAT1(n)	(GPIO_BASE + (0x30 + (n)*0x100)) /* Port Pattern 1 Register */
#define GPIO_PXPAT1S(n)	(GPIO_BASE + (0x34 + (n)*0x100)) /* Port Pattern 1 Set Reg. */
#define GPIO_PXPAT1C(n)	(GPIO_BASE + (0x38 + (n)*0x100)) /* Port Pattern 1 Clear Reg. */
#define GPIO_PXPAT0(n)	(GPIO_BASE + (0x40 + (n)*0x100)) /* Port Pattern 0 Register */
#define GPIO_PXPAT0S(n)	(GPIO_BASE + (0x44 + (n)*0x100)) /* Port Pattern 0 Set Register */
#define GPIO_PXPAT0C(n)	(GPIO_BASE + (0x48 + (n)*0x100)) /* Port Pattern 0 Clear Register */
#define GPIO_PXFLG(n)	(GPIO_BASE + (0x50 + (n)*0x100)) /* Port Flag Register */
#define GPIO_PXFLGC(n)	(GPIO_BASE + (0x54 + (n)*0x100)) /* Port Flag clear Register */
#define GPIO_PXOEN(n)	(GPIO_BASE + (0x60 + (n)*0x100)) /* Port Output Disable Register */
#define GPIO_PXOENS(n)	(GPIO_BASE + (0x64 + (n)*0x100)) /* Port Output Disable Set Register */
#define GPIO_PXOENC(n)	(GPIO_BASE + (0x68 + (n)*0x100)) /* Port Output Disable Clear Register */
#define GPIO_PXPEN(n)	(GPIO_BASE + (0x70 + (n)*0x100)) /* Port Pull Disable Register */
#define GPIO_PXPENS(n)	(GPIO_BASE + (0x74 + (n)*0x100)) /* Port Pull Disable Set Register */
#define GPIO_PXPENC(n)	(GPIO_BASE + (0x78 + (n)*0x100)) /* Port Pull Disable Clear Register */
#define GPIO_PXDS(n)	(GPIO_BASE + (0x80 + (n)*0x100)) /* Port Drive Strength Register */
#define GPIO_PXDSS(n)	(GPIO_BASE + (0x84 + (n)*0x100)) /* Port Drive Strength set Register */
#define GPIO_PXDSC(n)	(GPIO_BASE + (0x88 + (n)*0x100)) /* Port Drive Strength clear Register */

#define REG_GPIO_PXPIN(n)	REG32(GPIO_PXPIN((n)))  /* PIN level */
#define REG_GPIO_PXINT(n)	REG32(GPIO_PXINT((n)))  /* 1: interrupt pending */
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
#define REG_GPIO_PXDSC(n)	REG32(GPIO_PXDSC((n))) /* interrupt flag */



/*************************************************************************
 * MSC
 *************************************************************************/
#define	MSC_STRPCL		(MSC0_BASE + 0x000)
#define	MSC_STAT		(MSC0_BASE + 0x004)
#define	MSC_CLKRT		(MSC0_BASE + 0x008)
#define	MSC_CMDAT		(MSC0_BASE + 0x00C)
#define	MSC_RESTO		(MSC0_BASE + 0x010)
#define	MSC_RDTO		(MSC0_BASE + 0x014)
#define	MSC_BLKLEN		(MSC0_BASE + 0x018)
#define	MSC_NOB			(MSC0_BASE + 0x01C)
#define	MSC_SNOB		(MSC0_BASE + 0x020)
#define	MSC_IMASK		(MSC0_BASE + 0x024)
#define	MSC_IREG		(MSC0_BASE + 0x028)
#define	MSC_CMD			(MSC0_BASE + 0x02C)
#define	MSC_ARG			(MSC0_BASE + 0x030)
#define	MSC_RES			(MSC0_BASE + 0x034)
#define	MSC_RXFIFO		(MSC0_BASE + 0x038)
#define	MSC_TXFIFO		(MSC0_BASE + 0x03C)
#define	MSC_LPM 		(MSC0_BASE + 0x040)
#define MSC_DBG                 (MSC0_BASE + 0x0fc)
#define MSC_DMAC                (MSC0_BASE + 0x044)
#define MSC_DMANDA              (MSC0_BASE + 0x048)
#define MSC_DMADA               (MSC0_BASE + 0x04c)
#define MSC_DMALEN              (MSC0_BASE + 0x050)
#define MSC_DMACMD              (MSC0_BASE + 0x054)
#define MSC_CTRL2               (MSC0_BASE + 0x058)
#define MSC_RTCNT               (MSC0_BASE + 0x05c)

#define	REG_MSC_STRPCL		REG16(MSC_STRPCL)
#define	REG_MSC_STAT		REG32(MSC_STAT)
#define	REG_MSC_CLKRT		REG16(MSC_CLKRT)
#define	REG_MSC_CMDAT		REG32(MSC_CMDAT)
#define	REG_MSC_RESTO		REG16(MSC_RESTO)
#define	REG_MSC_RDTO		REG32(MSC_RDTO)
#define	REG_MSC_BLKLEN		REG16(MSC_BLKLEN)
#define	REG_MSC_NOB		REG16(MSC_NOB)
#define	REG_MSC_SNOB		REG16(MSC_SNOB)
#define	REG_MSC_IMASK		REG32(MSC_IMASK)
#define	REG_MSC_IREG		REG32(MSC_IREG)
#define	REG_MSC_CMD		REG8(MSC_CMD)
#define	REG_MSC_ARG		REG32(MSC_ARG)
#define	REG_MSC_RES		REG16(MSC_RES)
#define	REG_MSC_RXFIFO		REG32(MSC_RXFIFO)
#define	REG_MSC_TXFIFO		REG32(MSC_TXFIFO)
#define	REG_MSC_LPM		REG32(MSC_LPM)
#define REG_MSC_DBG             REG32(MSC_DBG)
#define REG_MSC_DMAC            REG32(MSC_DMAC)
#define REG_MSC_DMANDA          REG32(MSC_DMANDA)
#define REG_MSC_DMADA           REG32(MSC_DMADA)
#define REG_MSC_DMALEN          REG32(MSC_DMALEN)
#define REG_MSC_DMACMD          REG32(MSC_DMACMD)
#define REG_MSC_CTRL2           REG32(MSC_CTRL2)  
#define REG_MSC_RTCNT           REG32(MSC_RTCNT)

/* MSC Clock and Control Register (MSC_STRPCL) */

#define MSC_STRPCL_EXIT_MULTIPLE	(1 << 7)
#define MSC_STRPCL_EXIT_TRANSFER	(1 << 6)
#define MSC_STRPCL_START_READWAIT	(1 << 5)
#define MSC_STRPCL_STOP_READWAIT	(1 << 4)
#define MSC_STRPCL_RESET		(1 << 3)
#define MSC_STRPCL_START_OP		(1 << 2)
#define MSC_STRPCL_CLOCK_CONTROL_BIT	0
#define MSC_STRPCL_CLOCK_CONTROL_MASK	(0x3 << MSC_STRPCL_CLOCK_CONTROL_BIT)
  #define MSC_STRPCL_CLOCK_CONTROL_STOP	  (0x1 << MSC_STRPCL_CLOCK_CONTROL_BIT) /* Stop MMC/SD clock */
  #define MSC_STRPCL_CLOCK_CONTROL_START  (0x2 << MSC_STRPCL_CLOCK_CONTROL_BIT) /* Start MMC/SD clock */

/* MSC Status Register (MSC_STAT) */

#define MSC_STAT_AUTO_CMD_DONE         (1 << 31)
#define MSC_STAT_IS_RESETTING		(1 << 15)
#define MSC_STAT_SDIO_INT_ACTIVE	(1 << 14)
#define MSC_STAT_PRG_DONE		(1 << 13)
#define MSC_STAT_DATA_TRAN_DONE		(1 << 12)
#define MSC_STAT_END_CMD_RES		(1 << 11)
#define MSC_STAT_DATA_FIFO_AFULL	(1 << 10)
#define MSC_STAT_IS_READWAIT		(1 << 9)
#define MSC_STAT_CLK_EN			(1 << 8)
#define MSC_STAT_DATA_FIFO_FULL		(1 << 7)
#define MSC_STAT_DATA_FIFO_EMPTY	(1 << 6)
#define MSC_STAT_CRC_RES_ERR		(1 << 5)
#define MSC_STAT_CRC_READ_ERROR		(1 << 4)
#define MSC_STAT_CRC_WRITE_ERROR_BIT	2
#define MSC_STAT_CRC_WRITE_ERROR_MASK	(0x3 << MSC_STAT_CRC_WRITE_ERROR_BIT)
  #define MSC_STAT_CRC_WRITE_ERROR_NO		(0 << MSC_STAT_CRC_WRITE_ERROR_BIT) /* No error on transmission of data */
  #define MSC_STAT_CRC_WRITE_ERROR		(1 << MSC_STAT_CRC_WRITE_ERROR_BIT) /* Card observed erroneous transmission of data */
  #define MSC_STAT_CRC_WRITE_ERROR_NOSTS	(2 << MSC_STAT_CRC_WRITE_ERROR_BIT) /* No CRC status is sent back */
#define MSC_STAT_TIME_OUT_RES		(1 << 1)
#define MSC_STAT_TIME_OUT_READ		(1 << 0)

/* MSC Bus Clock Control Register (MSC_CLKRT) */

#define	MSC_CLKRT_CLK_RATE_BIT		0
#define	MSC_CLKRT_CLK_RATE_MASK		(0x7 << MSC_CLKRT_CLK_RATE_BIT)
  #define MSC_CLKRT_CLK_RATE_DIV_1	  (0x0 << MSC_CLKRT_CLK_RATE_BIT) /* CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_2	  (0x1 << MSC_CLKRT_CLK_RATE_BIT) /* 1/2 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_4	  (0x2 << MSC_CLKRT_CLK_RATE_BIT) /* 1/4 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_8	  (0x3 << MSC_CLKRT_CLK_RATE_BIT) /* 1/8 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_16	  (0x4 << MSC_CLKRT_CLK_RATE_BIT) /* 1/16 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_32	  (0x5 << MSC_CLKRT_CLK_RATE_BIT) /* 1/32 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_64	  (0x6 << MSC_CLKRT_CLK_RATE_BIT) /* 1/64 of CLK_SRC */
  #define MSC_CLKRT_CLK_RATE_DIV_128	  (0x7 << MSC_CLKRT_CLK_RATE_BIT) /* 1/128 of CLK_SRC */

/* MSC Command Sequence Control Register (MSC_CMDAT) */

#define	MSC_CMDAT_IO_ABORT		(1 << 11)
#define	MSC_CMDAT_BUS_WIDTH_BIT		9
#define	MSC_CMDAT_BUS_WIDTH_MASK	(0x3 << MSC_CMDAT_BUS_WIDTH_BIT)
  #define MSC_CMDAT_BUS_WIDTH_1BIT	  (0x0 << MSC_CMDAT_BUS_WIDTH_BIT) /* 1-bit data bus */
  #define MSC_CMDAT_BUS_WIDTH_4BIT	  (0x2 << MSC_CMDAT_BUS_WIDTH_BIT) /* 4-bit data bus */
  #define CMDAT_BUS_WIDTH1	  (0x0 << MSC_CMDAT_BUS_WIDTH_BIT)
  #define CMDAT_BUS_WIDTH4	  (0x2 << MSC_CMDAT_BUS_WIDTH_BIT)
#define	MSC_CMDAT_DMA_EN		(1 << 8)
#define	MSC_CMDAT_INIT			(1 << 7)
#define	MSC_CMDAT_BUSY			(1 << 6)
#define	MSC_CMDAT_STREAM_BLOCK		(1 << 5)
#define	MSC_CMDAT_WRITE			(1 << 4)
#define	MSC_CMDAT_READ			(0 << 4)
#define	MSC_CMDAT_DATA_EN		(1 << 3)
#define	MSC_CMDAT_RESPONSE_BIT	0
#define	MSC_CMDAT_RESPONSE_MASK	(0x7 << MSC_CMDAT_RESPONSE_BIT)
  #define MSC_CMDAT_RESPONSE_NONE  (0x0 << MSC_CMDAT_RESPONSE_BIT) /* No response */
  #define MSC_CMDAT_RESPONSE_R1	  (0x1 << MSC_CMDAT_RESPONSE_BIT) /* Format R1 and R1b */
  #define MSC_CMDAT_RESPONSE_R2	  (0x2 << MSC_CMDAT_RESPONSE_BIT) /* Format R2 */
  #define MSC_CMDAT_RESPONSE_R3	  (0x3 << MSC_CMDAT_RESPONSE_BIT) /* Format R3 */
  #define MSC_CMDAT_RESPONSE_R4	  (0x4 << MSC_CMDAT_RESPONSE_BIT) /* Format R4 */
  #define MSC_CMDAT_RESPONSE_R5	  (0x5 << MSC_CMDAT_RESPONSE_BIT) /* Format R5 */
  #define MSC_CMDAT_RESPONSE_R6	  (0x6 << MSC_CMDAT_RESPONSE_BIT) /* Format R6 */

#define	CMDAT_DMA_EN	(1 << 8)
#define	CMDAT_INIT	(1 << 7)
#define	CMDAT_BUSY	(1 << 6)
#define	CMDAT_STREAM	(1 << 5)
#define	CMDAT_WRITE	(1 << 4)
#define	CMDAT_DATA_EN	(1 << 3)

/* MSC Interrupts Mask Register (MSC_IMASK) */

#define	MSC_IMASK_SDIO			(1 << 7)
#define	MSC_IMASK_TXFIFO_WR_REQ		(1 << 6)
#define	MSC_IMASK_RXFIFO_RD_REQ		(1 << 5)
#define	MSC_IMASK_END_CMD_RES		(1 << 2)
#define	MSC_IMASK_PRG_DONE		(1 << 1)
#define	MSC_IMASK_DATA_TRAN_DONE	(1 << 0)


/* MSC Interrupts Status Register (MSC_IREG) */

#define	MSC_IREG_SDIO			(1 << 7)
#define	MSC_IREG_TXFIFO_WR_REQ		(1 << 6)
#define	MSC_IREG_RXFIFO_RD_REQ		(1 << 5)
#define	MSC_IREG_END_CMD_RES		(1 << 2)
#define	MSC_IREG_PRG_DONE		(1 << 1)
#define	MSC_IREG_DATA_TRAN_DONE		(1 << 0)

/*************************************************************************
 * NEMC (External Normal Memory Controller)
 *************************************************************************/
#define NEMC_SMCR1	(NEMC_BASE + 0x14)  /* Static Memory Control Register 1 */
#define NEMC_SMCR2	(NEMC_BASE + 0x18)  /* Static Memory Control Register 2 */
#define NEMC_SMCR3	(NEMC_BASE + 0x1c)  /* Static Memory Control Register 3 */
#define NEMC_SMCR4	(NEMC_BASE + 0x20)  /* Static Memory Control Register 4 */
#define NEMC_SMCR5	(NEMC_BASE + 0x24)  /* Static Memory Control Register 5 */
#define NEMC_SMCR6	(NEMC_BASE + 0x28)  /* Static Memory Control Register 6 */
#define NEMC_SACR1	(NEMC_BASE + 0x34)  /* Static Memory Bank 1 Addr Config Reg */
#define NEMC_SACR2	(NEMC_BASE + 0x38)  /* Static Memory Bank 2 Addr Config Reg */
#define NEMC_SACR3	(NEMC_BASE + 0x3c)  /* Static Memory Bank 3 Addr Config Reg */
#define NEMC_SACR4	(NEMC_BASE + 0x40)  /* Static Memory Bank 4 Addr Config Reg */
#define NEMC_SACR5	(NEMC_BASE + 0x44)  /* Static Memory Bank 5 Addr Config Reg */
#define NEMC_SACR6	(NEMC_BASE + 0x48)  /* Static Memory Bank 6 Addr Config Reg */

#define NEMC_NFCSR	(NEMC_BASE + 0x050) /* NAND Flash Control/Status Register */
#define NEMC_PNCR	(NEMC_BASE + 0x100)
#define NEMC_PNDR	(NEMC_BASE + 0x104)
#define NEMC_BITCNT	(NEMC_BASE + 0x108)

/* NEMC for TOGGLE NAND */
#define NEMC_TGWE	(NEMC_BASE + 0x10C) /* Toggle NAND Data Write Access */
#define NEMC_TGCR1	(NEMC_BASE + 0x110) /* Toggle NAND Control Register 1 */
#define NEMC_TGCR2	(NEMC_BASE + 0x114)
#define NEMC_TGCR3	(NEMC_BASE + 0x118)
#define NEMC_TGCR4	(NEMC_BASE + 0x11C)
#define NEMC_TGCR5	(NEMC_BASE + 0x120)
#define NEMC_TGCR6	(NEMC_BASE + 0x124)
#define NEMC_TGSR	(NEMC_BASE + 0x128) /* Toggle NAND RD# to DQS and DQ delay Register */
#define NEMC_TGFL	(NEMC_BASE + 0x12C) /* Toggle NAND ALE Fall to DQS Rise (bank 1/2/3 TGFL) */
#define NEMC_TGFH	(NEMC_BASE + 0x130) /* Toggle NAND ALE Fall to DQS Rise (bank 4/5/6 TGFH) */
#define NEMC_TGCL	(NEMC_BASE + 0x134) /* Toggle NAND CLE to RD# Low (bank 1/2/3 TGCL) */
#define NEMC_TGCH	(NEMC_BASE + 0x138) /* Toggle NAND CLE to RD# low (bank 4/5/6 TGCH) */
#define NEMC_TGPD	(NEMC_BASE + 0x13C) /* Toggle NAND Data Postamble Hold Time Done */
#define NEMC_TGSL	(NEMC_BASE + 0x140) /* Toggle NAND DQS Setup Time for Data Input Start (bank 1/2/3 TGSL) */
#define NEMC_TGSH	(NEMC_BASE + 0x144) /* Toggle NAND DQS Setup Time for Data Input Start (bank 4/5/6 TGSH) */
#define NEMC_TGRR	(NEMC_BASE + 0x148) /* Toggle NAND Timer for Random Data Input and Register Read Out */
#define NEMC_TGDR	(NEMC_BASE + 0x14C) /* Toggle NAND DQS Delay Control Register */

#define REG_NEMC_SMCR1	REG32(NEMC_SMCR1)
#define REG_NEMC_SMCR2	REG32(NEMC_SMCR2)
#define REG_NEMC_SMCR3	REG32(NEMC_SMCR3)
#define REG_NEMC_SMCR4	REG32(NEMC_SMCR4)
#define REG_NEMC_SMCR5	REG32(NEMC_SMCR5)
#define REG_NEMC_SMCR6	REG32(NEMC_SMCR6)
#define REG_NEMC_SACR1	REG32(NEMC_SACR1)
#define REG_NEMC_SACR2	REG32(NEMC_SACR2)
#define REG_NEMC_SACR3	REG32(NEMC_SACR3)
#define REG_NEMC_SACR4	REG32(NEMC_SACR4)
#define REG_NEMC_SACR5	REG32(NEMC_SACR5)
#define REG_NEMC_SACR6	REG32(NEMC_SACR6)

#define REG_NEMC_NFCSR	REG32(NEMC_NFCSR)
#define REG_NEMC_PNCR	REG32(NEMC_PNCR)
#define REG_NEMC_PNDR	REG32(NEMC_PNDR)
#define REG_NEMC_BITCNT	REG32(NEMC_BITCNT)

#define REG_NEMC_TGWE	REG32(NEMC_TGWE)
#define REG_NEMC_TGCR1	REG32(NEMC_TGCR1)
#define REG_NEMC_TGCR2	REG32(NEMC_TGCR2)
#define REG_NEMC_TGCR3	REG32(NEMC_TGCR3)
#define REG_NEMC_TGCR4	REG32(NEMC_TGCR4)
#define REG_NEMC_TGCR5	REG32(NEMC_TGCR5)
#define REG_NEMC_TGCR6	REG32(NEMC_TGCR6)
#define REG_NEMC_TGSR	REG32(NEMC_TGSR)
#define REG_NEMC_TGFL	REG32(NEMC_TGFL)
#define REG_NEMC_TGFH	REG32(NEMC_TGFH)
#define REG_NEMC_TGCL	REG32(NEMC_TGCL)
#define REG_NEMC_TGCH	REG32(NEMC_TGCH)
#define REG_NEMC_TGPD	REG32(NEMC_TGPD)
#define REG_NEMC_TGSL	REG32(NEMC_TGSL)
#define REG_NEMC_TGSH	REG32(NEMC_TGSH)
#define REG_NEMC_TGRR	REG32(NEMC_TGRR)
#define REG_NEMC_TGDR	REG32(NEMC_TGDR)

/* Static Memory Control Register */
#define NEMC_SMCR_STRV_BIT	24
#define NEMC_SMCR_STRV_MASK	(0x0f << NEMC_SMCR_STRV_BIT)
#define NEMC_SMCR_TAW_BIT	20
#define NEMC_SMCR_TAW_MASK	(0x0f << NEMC_SMCR_TAW_BIT)
#define NEMC_SMCR_TBP_BIT	16
#define NEMC_SMCR_TBP_MASK	(0x0f << NEMC_SMCR_TBP_BIT)
#define NEMC_SMCR_TAH_BIT	12
#define NEMC_SMCR_TAH_MASK	(0x07 << NEMC_SMCR_TAH_BIT)
#define NEMC_SMCR_TAS_BIT	8
#define NEMC_SMCR_TAS_MASK	(0x07 << NEMC_SMCR_TAS_BIT)
#define NEMC_SMCR_BW_BIT	6
#define NEMC_SMCR_BW_MASK	(0x03 << NEMC_SMCR_BW_BIT)
  #define NEMC_SMCR_BW_8BIT		(0 << NEMC_SMCR_BW_BIT)
  #define NEMC_SMCR_BW_16BIT		(1 << NEMC_SMCR_BW_BIT)
  #define NEMC_SMCR_BW_32BIT		(2 << NEMC_SMCR_BW_BIT)
#define NEMC_SMCR_BCM		(1 << 3)
#define NEMC_SMCR_BL_BIT	1
#define NEMC_SMCR_BL_MASK	(0x03 << NEMC_SMCR_BL_BIT)
  #define NEMC_SMCR_BL_4		(0 << NEMC_SMCR_BL_BIT)
  #define NEMC_SMCR_BL_8		(1 << NEMC_SMCR_BL_BIT)
  #define NEMC_SMCR_BL_16		(2 << NEMC_SMCR_BL_BIT)
  #define NEMC_SMCR_BL_32		(3 << NEMC_SMCR_BL_BIT)
#define NEMC_SMCR_SMT		(1 << 0)

/* Static Memory Bank Addr Config Reg */
#define NEMC_SACR_BASE_BIT	8
#define NEMC_SACR_BASE_MASK	(0xff << NEMC_SACR_BASE_BIT)
#define NEMC_SACR_MASK_BIT	0
#define NEMC_SACR_MASK_MASK	(0xff << NEMC_SACR_MASK_BIT)

/* NAND Flash Control/Status Register */
#define NEMC_NFCSR_DAEC		(1 << 31) /* Toggle NAND Data Access Enabel Clear */
#define NEMC_NFCSR_TNFE(n)	(1 << ((n) + 15))
#define NEMC_NFCSR_TNFE6	(1 << 21) /* Toggle NAND Flash Enable */
#define NEMC_NFCSR_TNFE5	(1 << 20)
#define NEMC_NFCSR_TNFE4	(1 << 19) /* Toggle NAND Flash Enable */
#define NEMC_NFCSR_TNFE3	(1 << 18)
#define NEMC_NFCSR_TNFE2	(1 << 17)
#define NEMC_NFCSR_TNFE1	(1 << 16)
#define NEMC_NFCSR_NFCE(n)	(1 << ((((n) - 1) << 1) + 1))
#define NEMC_NFCSR_NFE(n)	(1 << (((n) -1) << 1))
#define NEMC_NFCSR_NFCE6	(1 << 11) /* NAND Flash Enable */
#define NEMC_NFCSR_NFE6		(1 << 10) /* NAND Flash FCE# Assertion Enable */
#define NEMC_NFCSR_NFCE5	(1 << 9)
#define NEMC_NFCSR_NFE5		(1 << 8)
#define NEMC_NFCSR_NFCE4	(1 << 7) /* NAND Flash Enable */
#define NEMC_NFCSR_NFE4		(1 << 6) /* NAND Flash FCE# Assertion Enable */
#define NEMC_NFCSR_NFCE3	(1 << 5)
#define NEMC_NFCSR_NFE3		(1 << 4)
#define NEMC_NFCSR_NFCE2	(1 << 3)
#define NEMC_NFCSR_NFE2		(1 << 2)
#define NEMC_NFCSR_NFCE1	(1 << 1)
#define NEMC_NFCSR_NFE1		(1 << 0)

/* NAND PN Control Register */
// PN(bit 0):0-disable, 1-enable
// PN(bit 1):0-no reset, 1-reset
// (bit 2):Reserved
// BITCNT(bit 3):0-disable, 1-enable
// BITCNT(bit 4):0-calculate, 1's number, 1-calculate 0's number
// BITCNT(bit 5):0-no reset, 1-reset bitcnt
#define NEMC_PNCR_PNRST		(1 << 1)
#define NEMC_PNCR_PNEN		(1 << 0)

/* Toggle NAND Data Write Access */
#define NEMC_TGWE_DAE		(1 << 31)
#define NEMC_TGWE_WCD		(1 << 16) /* DQS Setup Time for data input start Done */
#define NEMC_TGWE_SDE(n)	(1 << ((n) - 1))
#define NEMC_TGWE_SDE6		(1 << 5) /* Set DQS output enable bank6 */
#define NEMC_TGWE_SDE5		(1 << 4) 
#define NEMC_TGWE_SDE4		(1 << 3)
#define NEMC_TGWE_SDE3		(1 << 2)
#define NEMC_TGWE_SDE2		(1 << 1)
#define NEMC_TGWE_SDE1		(1 << 0)

/* Toggle NAND RD# to DQS and DQ delay Register */
#define NEMC_TGSR_DQSRE6_BIT	20 /* Toggle NAND Flash RD# to DQS and DQ delay bank6 */
#define NEMC_TGSR_DQSRE5_BIT	16
#define NEMC_TGSR_DQSRE4_BIT	12
#define NEMC_TGSR_DQSRE3_BIT	8
#define NEMC_TGSR_DQSRE2_BIT	4
#define NEMC_TGSR_DQSRE1_BIT	0

/* Toggle NAND ALE Fall to DQS Rise (bank 3/2/1 TGFL) */
#define NEMC_TGFL_FDA3_BIT	16 /* Toggle NAND Flash ALE Fall to DQS Rise Bank3 */
#define NEMC_TGFL_FDA2_BIT	8
#define NEMC_TGFL_FDA1_BIT	0
/* Toggle NAND ALE Fall to DQS Rise (bank 4/5/6 TGFH) */
#define NEMC_TGFH_FDA6_BIT	16 /* Toggle NAND Flash First ALE Fall to DQS Rise Bank6 */
#define NEMC_TGFH_FDA5_BIT	8
#define NEMC_TGFH_FDA4_BIT	0

/* Toggle NAND CLE to RD# Low (bank 1/2/3 TGCL) */
#define NEMC_TGCL_CLR3_BIT	16 /* Toggle NAND Flash CLE to RE_n Low Bank3 */
#define NEMC_TGCL_CLR2_BIT	8
#define NEMC_TGCL_CLR1_BIT	0
/* Toggle NAND CLE to RD# low (bank 4/5/6 TGCH) */
#define NEMC_TGCH_CLR6_BIT	16 /* Toggle NAND Flash CLE to RE_n Low Bank6 */
#define NEMC_TGCH_CLR5_BIT	8
#define NEMC_TGCH_CLR4_BIT	0

/* Toggle NAND Data Postamble Hold Time Done */
#define NEMC_TGPD_DPHTD(n)	(1 << ((n) - 1))
#define NEMC_TGPD_DPHTD6	(1 << 5) /* Toggle NAND Flash Data Postamble Hold Time Done Bank6 */
#define NEMC_TGPD_DPHTD5	(1 << 4)
#define NEMC_TGPD_DPHTD4	(1 << 3)
#define NEMC_TGPD_DPHTD3	(1 << 2)
#define NEMC_TGPD_DPHTD2	(1 << 1)
#define NEMC_TGPD_DPHTD1	(1 << 0)

/* Toggle NAND DQS Setup Time for Data Input Start (bank 1/2/3 TGSL) */
#define NEMC_TGSL_CQDSS3_BIT	16 /* DQS Setup Time for data input start (bank3) */
#define NEMC_TGSL_CQDSS2_BIT	8
#define NEMC_TGSL_CQDSS1_BIT	0
/* Toggle NAND DQS Setup Time for Data Input Start (bank 4/5/6 TGSH) */
#define NEMC_TGSH_CQDSS6_BIT	16 /* DQS Setup Time for data input start (bank6) */
#define NEMC_TGSH_CQDSS5_BIT	8
#define NEMC_TGSH_CQDSS4_BIT	0

/* Toggle NAND Timer for Random Data Input and Register Read Out */
#define NEMC_TGRR_TD_MASK	(1 << 16) /* Timer Done */
#define NEMC_TGRR_CWAW_MASK	0xFF /* Command Write Cycle to Address Write Cycle Time */

/* Toggle NAND Data Access Enabel Clear */
#define NEMC_TGDR_ERR		(1 << 29)
#define NEMC_TGDR_DONE		(1 << 28)
#define NEMC_TGDR_DET		(1 << 23)
#define NEMC_TGDR_AUTO		(1 << 22)
#define NEMC_TGDR_RDQS_BIT	0
#define NEMC_TGDR_RDQS_MASK	(0x1f << NEMC_TGDR_RDQS_BIT)

/*************************************************************************
 * BCH
 *************************************************************************/
#define BCH_CR		(BCH_BASE + 0x00) /* BCH Control register */
#define BCH_CRS		(BCH_BASE + 0x04) /* BCH Control Set register */
#define BCH_CRC		(BCH_BASE + 0x08) /* BCH Control Clear register */
#define BCH_CNT		(BCH_BASE + 0x0C) /* BCH ENC/DEC Count register */
#define BCH_DR		(BCH_BASE + 0x10) /* BCH data register */
#define BCH_PAR0	(BCH_BASE + 0x14) /* BCH Parity 0 register */
#define BCH_PAR(n)	(BCH_BASE + 0x14 + ((n) << 2)) /* BCH Parity n register */
#define BCH_ERR0	(BCH_BASE + 0x84) /* BCH Error Report 0 register */
#define BCH_ERR(n)	(BCH_BASE + 0x84 + ((n) << 2)) /* BCH Error Report n register */
#define BCH_INTS	(BCH_BASE + 0x184) /* BCH Interrupt Status register */
#define BCH_INTES	(BCH_BASE + 0x188) /* BCH Interrupt Set register */
#define BCH_INTEC	(BCH_BASE + 0x18C) /* BCH Interrupt Clear register */
#define BCH_INTE	(BCH_BASE + 0x190) /* BCH Interrupt Enable register */
#define BCH_TO		(BCH_BASE + 0x194) /* BCH User Tag Output register */

#define REG_BCH_CR	REG32(BCH_CR)
#define REG_BCH_CRS	REG32(BCH_CRS)
#define REG_BCH_CRC	REG32(BCH_CRC)
#define REG_BCH_CNT	REG32(BCH_CNT)
#define REG_BCH_DR	REG8(BCH_DR)
#define REG_BCH_DR8	REG8(BCH_DR)
#define REG_BCH_DR16	REG16(BCH_DR)
#define REG_BCH_DR32	REG32(BCH_DR)
#define REG_BCH_PAR0	REG32(BCH_PAR0)
#define REG_BCH_PAR(n)	REG32(BCH_PAR(n))
#define REG_BCH_ERR0	REG32(BCH_ERR0)
#define REG_BCH_ERR(n)	REG32(BCH_ERR(n))
#define REG_BCH_INTS	REG32(BCH_INTS)
#define REG_BCH_INTES	REG32(BCH_INTES)
#define REG_BCH_INTEC	REG32(BCH_INTEC)
#define REG_BCH_INTE	REG32(BCH_INTE)
#define REG_BCH_TO	REG32(BCH_TO)

/* BCH Control Register*/
#define BCH_CR_TAG_BIT		16
#define BCH_CR_TAG_MASK		(0xffff << BCH_CR_TAG_BIT)
#define BCH_CR_TAG(n)		((n) << 16)  /* BCH User-provided TAG */
#define BCH_CR_MZSB_BIT		13
#define BCH_CR_MZSB_MASK	(0x7 << BCH_CR_MZSB_BIT)
#define BCH_CR_MZEB(n)		((n) << 13)  /* BCH MAX Zero Bit in Erased Block */
#define BCH_CR_BPS		(1 << 12)  /* BCH Decoder Bypass */
#define BCH_CR_BSEL_BIT		4
#define BCH_CR_BSEL_MASK	(0x7f << BCH_CR_BSEL_BIT)
#define BCH_CR_BSEL(n)		((n) << BCH_CR_BSEL_BIT)  /* n Bit BCH Select */
  #define BCH_CR_BSEL_4		(0x4 << BCH_CR_BSEL_BIT)  /* 4 Bit BCH Select */
  #define BCH_CR_BSEL_8		(0x8 << BCH_CR_BSEL_BIT)  /* 8 Bit BCH Select */
  #define BCH_CR_BSEL_24	(0x18 << BCH_CR_BSEL_BIT)  /* 24 Bit BCH Select */
  #define BCH_CR_BSEL_40	(0x28 << BCH_CR_BSEL_BIT)  /* 40 Bit BCH Select */
  #define BCH_CR_BSEL_60	(0x3C << BCH_CR_BSEL_BIT)  /* 40 Bit BCH Select */
  #define BCH_CR_BSEL_64	(0x40 << BCH_CR_BSEL_BIT)  /* 64 Bit BCH Select */
#define BCH_CR_ENCE		(1 << 2)  /* BCH Encoding Select */
#define BCH_CR_DECE		(0 << 2)  /* BCH Decoding Select */
#define BCH_CR_INIT		(1 << 1)  /* BCH Reset */
#define BCH_CR_BCHE		(1 << 0)  /* BCH Enable */

/* BCH ENC/DEC Count Register */
#define BCH_CNT_PARITY_BIT	16
#define BCH_CNT_PARITY_MASK	(0x7f << BCH_CNT_PARITY_BIT)
#define BCH_CNT_BLOCK_BIT	0
#define BCH_CNT_BLOCK_MASK	(0x7ff << BCH_CNT_BLOCK_BIT)

/* BCH Error Report Register */
#define BCH_ERR_MASK_BIT	16
#define BCH_ERR_MASK_MASK	(0xffff << BCH_ERR_MASK_BIT)
#define BCH_ERR_INDEX_BIT	0
#define BCH_ERR_INDEX_MASK	(0x7ff << BCH_ERR_INDEX_BIT)

/* BCH Interrupt Status Register */
#define BCH_INTS_ERRC_BIT	24
#define BCH_INTS_ERRC_MASK	(0x7f << BCH_INTS_ERRC_BIT)
#define BCH_INTS_BPSO		(1 << 23)
#define BCH_INTS_TERRC_BIT	16
#define BCH_INTS_TERRC_MASK	(0x7f << BCH_INTS_TERRC_BIT)
#define BCH_INTS_SDMF		(1 << 5)
#define BCH_INTS_ALLf		(1 << 4)
#define BCH_INTS_DECF		(1 << 3)
#define BCH_INTS_ENCF		(1 << 2)
#define BCH_INTS_UNCOR		(1 << 1)
#define BCH_INTS_ERR		(1 << 0)

/* BCH Interrupt Enable Register */
#define BCH_INTE_SDMFE		(1 << 5)
#define BCH_INTE_ALL_FE		(1 << 4)
#define BCH_INTE_DECFE		(1 << 3)
#define BCH_INTE_ENCFE		(1 << 2)
#define BCH_INTE_UNCORE		(1 << 1)
#define BCH_INTE_ERRE		(1 << 0)

/* BCH User TAG OUTPUT Register */
#define BCH_BHTO_TAGO_BIT	0
#define BCH_BHTO_TAGO_MASK	(0xffff << BCH_TAGO_BIT)

//----------------------------------------------------------------------
//
// Module Operation Definitions
//
//----------------------------------------------------------------------
#ifndef __ASSEMBLY__

#define is_share_mode() (1)

/***************************************************************************
 * GPIO
 ***************************************************************************/

//------------------------------------------------------
// GPIO Pins Description
//
// PORT 0:
//
// PIN/BIT N	FUNC0		FUNC1		FUNC2         NOTE
//	0	SD0		-		-
//	1	SD1		-		-
//	2	SD2		-		-
//	3	SD3		-		-
//	4	SD4		-		-
//	5	SD5		-		-
//	6	SD6		-		-
//	7	SD7		-		-
//	8	SD8		-		-
//	9	SD9		-		-
//	10	SD10		-		-
//	11	SD11		-		-
//	12	SD12		-		-
//	13	SD13		-		-
//	14	SD14		-		-
//	15	SD15		-		-
//      16      RD_             -		-
//      17      WE_             -		-
//      18      FRE_            MSC0_CLK        SSI0_CLK
//      19      FWE_            MSC0_CMD        SSI0_CE0_
//      20      MSC0_D0         SSI0_DR		-	       1
//      21      CS1_            MSC0_D1         SSI0_DT
//      22      CS2_            MSC0_D2		-
//      23      CS3_		-		-
//      24      CS4_ 		-		-
//      25      CS5_		-		-
//      26      CS6_		-		-
//      27      WAIT_		-		-
//      28      DREQ0		-		-
//      29      DACK0           OWI		-
//      30	-		-		-	     6
//      31	-		-		-	     7

//Note1. PA20: GPIO group A bit 20. If NAND flash is used, this pin must be used as NAND FRB. (NAND flash ready/busy)
//Note6. PA30: GPIO group A bit 30 can only be used as input and interrupt, no pull-up and pull-down.
//Note7. PA31: GPIO group A bit 31. No corresponding pin exists for this GPIO. It is only used to select the function between UART and JTAG, which share the same set of pins, by using register PASEL [31]
//       When PASEL [31]=0, select JTAG function.
//       When PASEL [31]=1, select UART function

//------------------------------------------------------
// PORT 1:
//
// PIN/BIT N	FUNC0		FUNC1	       FUNC2         NOTE
//	0	SA0		-              -	     8
//	1	SA1		-              -             9
//	2	SA2		-              -             CL
//	3	SA3		-              -             AL
//	4	SA4		-              -
//	5	SA5		-              -
//	6	CIM_PCLK 	TSCLK	       -
//	7	CIM_HSYN  	TSFRM          -
//	8	CIM_VSYN 	TSSTR          -
//	9	CIM_MCLK 	TSFAIL         -
//	10	CIM_D0 	 	TSDI0          -
//	11	CIM_D1 	 	TSDI1          -
//	12	CIM_D2 	 	TSDI2          -
//	13	CIM_D3 		TSDI3          -
//	14	CIM_D4 		TSDI4          -
//	15	CIM_D5		TSDI5          -
//	16 	CIM_D6 		TSDI6          -
//	17 	CIM_D7 		TSDI7          -
//	18	-               -	       -
//	19	-               -	       -
//	20 	MSC2_D0 	SSI2_DR        TSDI0
//	21 	MSC2_D1 	SSI2_DT        TSDI1
//	22 	TSDI2		-              -
//	23 	TSDI3		-              -
//	24 	TSDI4		-              -
//	25 	TSDI5		-              -
//	26 	TSDI6		-              -
//	27 	TSDI7		-              -
//	28 	MSC2_CLK        SSI2_CLK       TSCLK
//	29 	MSC2_CMD        SSI2_CE0_      TSSTR
//	30 	MSC2_D2         SSI2_GPC       TSFAIL
//	31 	MSC2_D3         SSI2_CE1_      TSFRM
//Note1. PB0: GPIO group B bit 0. If NAND flash is used, this pin must be used as NAND CLE.
//Note1. PB1: GPIO group B bit 1. If NAND flash is used, this pin must be used as NAND ALE.

//------------------------------------------------------
// PORT 2:
// PIN/BIT N	FUNC0		FUNC1		FUNC2 		FUNC3		NOTE
//	0	LCD_B0 (O)	LCD_REV (O)	-               -
//	1	LCD_B1 (O)	LCD_PS (O)	-               -
//	2	LCD_B2 (O)	-               -	        -
//	3	LCD_B3 (O)	-               -	        -
//	4	LCD_B4 (O)	-               -	        -
//	5	LCD_B5 (O)	-               -	        -
//	6	LCD_B6 (O)	-               -	        -
//	7	LCD_B7 (O)	-               -	        -
//	8	LCD_PCLK (O)	-               -	        -
//	9	LCD_DE (O)	-               -	        -
//	10	LCD_G0 (O)	LCD_SPL (O)	-               -
//	11	LCD_G1 (O)	-               -	        -
//	12	LCD_G2 (O)	-               -	        -
//	13	LCD_G3 (O)	-               -	        -
//	14	LCD_G4 (O)	-               -	        -
//	15	LCD_G5 (O)	-               -	        -
//	16	LCD_G6 (O)	-               -	        -
//	17	LCD_G7 (O)	-               -	        -
//	18	LCD_HSYN (IO)	-               -	        -
//	19	LCD_VSYN (IO)	-               -	        -
//	20	LCD_R0 (O)	LCD_CLS (O)	-               -
//	21	LCD_R1 (O)	-               -	        -
//	22	LCD_R2 (O)	-               -	        -
//	23	LCD_R3 (O)	-               -	        -
//	24	LCD_R4 (O)	-               -	        -
//	25	LCD_R5 (O)	-               -	        -
//	26	LCD_R6 (O)	-               -	        -
//	27	LCD_R7 (O)	-               -	        -
//	28	UART2_RxD (I)	-               -	        -
//	29	UART2_CTS_ (I)	-               -	        -
//	30	UART2_TxD (O)	-               -	        -
//	31	UART2_RTS_ (O)	-               -	        -

//------------------------------------------------------
// PORT 3:
//
// PIN/BIT N	FUNC0		FUNC1		FUNC2 		FUNC3		NOTE
//	0 	MII_TXD0 	-     		-  		-
//	1 	MII_TXD1	-     		-  		-
//	2 	MII_TXD2	-     		-  		-
//	3 	MII_TXD3	- 		-  		-
//	4 	MII_TXEN	-		-		-
//	5  MII_TXCLK(RMII_CLK)	-		-  		-
//	6 	MII_COL   	-		-  		-
//	7 	MII_RXER	-		-  		-
//	8 	MII_RXDV	-		-  		-
//	9 	MII_RXCLK	-		-		-
//	10 	MII_RXD0	-		-  		-
//	11 	MII_RXD1	-		-  		-
//	12 	MII_RXD2	-		-  		-
//	13 	MII_RXD3	-		-  		-
//	14 	MII_CRS		-		-  		-
//	15 	MII_MDC		-		-  		-
//	16 	MII_MDIO	-		-  		-
//	17 	BOOT_SEL0	-		-  		-		Note2,5
//	18 	BOOT_SEL1	-		-  		- 		Note3,5
//	19 	BOOT_SEL2	-		-  		- 		Note4,5
//	20 	MSC1_D0 	SSI1_DR 	-  		-
//	21	MSC1_D1 	SSI1_DT 	-  		-
//	22 	MSC1_D2  	SSI1_GPC 	-  		-
//	23 	MSC1_D3  	SSI1_CE1_ 	-  		-
//	24 	MSC1_CLK  	SSI1_CLK 	-  		-
//	25 	MSC1_CMD  	SSI1_CE0_ 	-  		-
//	26 	UART1_RxD 	-		-  		-
//	27 	UART1_CTS_ 	-		-  		-
//	28 	UART1_TxD 	-		-  		-
//	29 	UART1_RTS_ 	-		-  		-
//	30 	I2C0_SDA 	-		-  		-
//	31 	I2C0_SCK 	-		-  		-
//
// Note2. PD17: GPIO group D bit 17 is used as BOOT_SEL0 input during boot.
// Note3. PD18: GPIO group D bit 18 is used as BOOT_SEL1 input during boot.
// Note4. PD19: GPIO group D bit 19 is used as BOOT_SEL2 input during boot.
// Note5. BOOT_SEL2, BOOT_SEL1, BOOT_SEL0 are used to select boot source and function during the processor boot.
//
//------------------------------------------------------
// PORT 4:
//
// PIN/BIT N	FUNC0		FUNC1	       FUNC2         FUNC3         NOTE
//	0  	PWM0		- 		- 		-
//	1  	PWM1		- 		- 		-
//	2  	PWM2 		SYNC 		- 		-
//	3  	PWM3 		UART3_RxD 	BCLK 		-
//	4  	PWM4 		- 		- 		-
//	5  	PWM5 		UART3_TxD 	SCLK_RSTN 	-
//	6  	SDATI		- 		- 		-
//	7  	SDATO 		- 		- 		-
//	8  	UART3_CTS_ 	- 		- 		-
//	9  	UART3_RTS_ 	- 		- 		-
//	10  	- 		- 		- 		-
//	11  	SDATO1 		- 		- 		-
//	12  	SDATO2 		- 		- 		-
//	13  	SDATO3 		-		-		-
//	14  	SSI0_DR 	SSI1_DR 	SSI2_DR 	-
//	15  	SSI0_CLK 	SI1_CLK 	SSI2_CLK 	-
//	16  	SSI0_CE0_ 	SI1_CE0_ 	SSI2_CE0_ 	-
//	17  	SSI0_DT 	SSI1_DT 	SSI2_DT 	-
//	18  	SSI0_CE1_ 	SSI1_CE1_ 	SSI2_CE1_ 	-
//	19  	SSI0_GPC 	SSI1_GPC 	SSI2_GPC 	-
//	20  	MSC0_D0 	MSC1_D0 	MSC2_D0 	-
//	21  	MSC0_D1 	MSC1_D1 	MSC2_D1 	-
//	22  	MSC0_D2 	MSC1_D2 	MSC2_D2 	-
//	23  	MSC0_D3 	MSC1_D3 	MSC2_D3 	-
//	24  	MSC0_CLK 	MSC1_CLK 	MSC2_CLK 	-
//	25  	MSC0_CMD 	MSC1_CMD 	MSC2_CMD 	-
//	26  	MSC0_D4 	MSC0_D4 	MSC0_D4 	PS2_MCLK
//	27  	MSC0_D5 	MSC0_D5 	MSC0_D5 	PS2_MDATA
//	28  	MSC0_D6 	MSC0_D6 	MSC0_D6 	PS2_KCLK
//	29  	MSC0_D7 	MSC0_D7 	MSC0_D7 	PS2_KDATA
//	30  	I2C1_SDA 	SCC_DATA 	- 		-
//	31  	I2C1_SCK 	SCC_CLK 	- 		-
//
//------------------------------------------------------
// PORT 5:
//
// PIN/BIT N	FUNC0		FUNC1		FUNC2		FUNC3		NOTE
//	0   	UART0_RxD 	GPS_CLK 	- 		-
//	1   	UART0_CTS_ 	GPS_MAG 	- 		-
//	2   	UART0_TxD 	GPS_SIG 	- 		-
//	3   	UART0_RTS_ 	- 		-		-
//
//////////////////////////////////////////////////////////

/*----------------------------------------------------------------
 * p is the port number (0,1,2,3,4,5)
 * o is the pin offset (0-31) inside the port
 * n is the absolute number of a pin (0-127), regardless of the port
 */

//----------------------------------------------------------------
// Function Pins Mode

#define __gpio_as_func0(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXFUNS(p) = (1 << o);		\
	REG_GPIO_PXTRGC(p) = (1 << o);		\
	REG_GPIO_PXSELC(p) = (1 << o);		\
} while (0)

#define __gpio_as_func1(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXFUNS(p) = (1 << o);		\
	REG_GPIO_PXTRGC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
} while (0)

#define __gpio_as_func2(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXFUNS(p) = (1 << o);		\
	REG_GPIO_PXTRGS(p) = (1 << o);		\
	REG_GPIO_PXSELC(p) = (1 << o);		\
} while (0)

#define __gpio_as_func3(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXFUNS(p) = (1 << o);		\
	REG_GPIO_PXTRGS(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
} while (0)


/*
 * MII_TXD0- D3 MII_TXEN MII_TXCLK MII_COL
 * MII_RXER MII_RXDV MII_RXCLK MII_RXD0 - D3
 * MII_CRS MII_MDC MII_MDIO
 */

#define __gpio_as_eth()				\
do {						\
	REG_GPIO_PXINTC(1) =  0x00000010;	\
	REG_GPIO_PXMASKC(1) = 0x00000010;	\
	REG_GPIO_PXPAT1S(1) = 0x00000010;	\
	REG_GPIO_PXPAT0C(1) = 0x00000010;	\
	REG_GPIO_PXINTC(3) =  0x3c000000;	\
	REG_GPIO_PXMASKC(3) = 0x3c000000;	\
	REG_GPIO_PXPAT1C(3) = 0x3c000000;	\
	REG_GPIO_PXPAT0S(3) = 0x3c000000;	\
	REG_GPIO_PXINTC(5) =  0xfff0;		\
	REG_GPIO_PXMASKC(5) = 0xfff0;		\
	REG_GPIO_PXPAT1C(5) = 0xfff0;		\
	REG_GPIO_PXPAT0C(5) = 0xfff0;		\
} while (0)


/*
 * UART0_TxD, UART0_RxD
 */
#define __gpio_as_uart0()			\
do {						\
	REG_GPIO_PXINTC(5) = 0x00000009;	\
	REG_GPIO_PXMASKC(5) = 0x00000009;	\
	REG_GPIO_PXPAT1C(5) = 0x00000009;	\
	REG_GPIO_PXPAT0C(5) = 0x00000009;	\
} while (0)


/*
 * UART0_TxD, UART0_RxD, UART0_CTS, UART0_RTS
 */
#define __gpio_as_uart0_ctsrts()		\
do {						\
	REG_GPIO_PXFUNS(5) = 0x0000000f;	\
	REG_GPIO_PXTRGC(5) = 0x0000000f;	\
	REG_GPIO_PXSELC(5) = 0x0000000f;	\
	REG_GPIO_PXPES(5) = 0x0000000f;		\
} while (0)
/*
 * GPS_CLK GPS_MAG GPS_SIG
 */
#define __gpio_as_gps()			\
do {						\
	REG_GPIO_PXFUNS(5) = 0x00000007;	\
	REG_GPIO_PXTRGC(5) = 0x00000007;	\
	REG_GPIO_PXSELS(5) = 0x00000007;	\
	REG_GPIO_PXPES(5) = 0x00000007;		\
} while (0)

/*
 * UART1_TxD, UART1_RxD
 */
#define __gpio_as_uart1()			\
do {						\
	REG_GPIO_PXINTC(3)  = 0x14000000;	\
	REG_GPIO_PXMASKC(3) = 0x14000000;	\
	REG_GPIO_PXPAT1C(3) = 0x14000000;	\
	REG_GPIO_PXPAT0C(3) = 0x14000000;	\
} while (0)

/*
 * UART1_TxD, UART1_RxD, UART1_CTS, UART1_RTS
 */
#define __gpio_as_uart1_ctsrts()		\
do {						\
	REG_GPIO_PXFUNS(3) = 0x3c000000;	\
	REG_GPIO_PXTRGC(3) = 0x3c000000;	\
	REG_GPIO_PXSELC(3) = 0x3c000000;	\
	REG_GPIO_PXPES(3)  = 0x3c000000;	\
} while (0)

/*
 * UART2_TxD, UART2_RxD
 */
#define __gpio_as_uart2()			\
do {						\
	REG_GPIO_PXINTC(2) = 0x50000000;	\
	REG_GPIO_PXMASKC(2) = 0x50000000;	\
	REG_GPIO_PXPAT1C(2) = 0x50000000;	\
	REG_GPIO_PXPAT0C(2)  = 0x50000000;	\
} while (0)

/*
 * UART2_TxD, UART2_RxD, UART2_CTS, UART2_RTS
 */
#define __gpio_as_uart2_ctsrts()		\
do {						\
	REG_GPIO_PXFUNS(2) = 0xf0000000;	\
	REG_GPIO_PXTRGC(2) = 0xf0000000;	\
	REG_GPIO_PXSELC(2) = 0xf0000000;	\
	REG_GPIO_PXPES(2)  = 0xf0000000;	\
} while (0)

/*
 * UART3_TxD, UART3_RxD
 */
#define __gpio_as_uart3()                       \
	do {                                    \
	REG_GPIO_PXINTC(4)  = (0x01<<5);        \
	REG_GPIO_PXMASKC(4)  = (0x01<<5);        \
	REG_GPIO_PXPAT1C(4) = (0x01<<5);        \
  	REG_GPIO_PXPAT0S(4) = (0x01<<5);        \
	REG_GPIO_PXINTC(3)  = (0x01<<12);       \
	REG_GPIO_PXMASKC(3)  = (0x01<<12);       \
	REG_GPIO_PXPAT1C(3) = (0x01<<12);       \
	REG_GPIO_PXPAT0C(3) = (0x01<<12);       \
	REG_GPIO_PXINTC(0)  = (0x03<<30);       \
	REG_GPIO_PXMASKC(0)  = (0x03<<30);       \
	REG_GPIO_PXPAT1C(0) = (0x03<<30);       \
	REG_GPIO_PXPAT0C(0) = (0x01<<30);       \
	REG_GPIO_PXPAT0S(0) = (0x01<<31);       \
	} while (0)
/*
 * UART3_TxD, UART3_RxD, UART3_CTS, UART3_RTS
 */
#define __gpio_as_uart3_ctsrts()		\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00000028;	\
	REG_GPIO_PXTRGC(4) = 0x00000028;	\
	REG_GPIO_PXSELS(4) = 0x00000028;	\
	REG_GPIO_PXFUNS(4) = 0x00000300;	\
	REG_GPIO_PXTRGC(4) = 0x00000300;	\
	REG_GPIO_PXSELC(4) = 0x00000300;	\
	REG_GPIO_PXPES(4)  = 0x00000328;	\
}

#define __gpio_as_uart4()                       \
	do {                                    \
 	REG_GPIO_PXINTC(2)  = 0x00100400;    \
	REG_GPIO_PXMASKC(2)  = 0x00100400;      \
	REG_GPIO_PXPAT1S(2) = 0x00100400;       \
	REG_GPIO_PXPAT0C(2) = 0x00100400;       \
	} while (0)


/*
 * SD0 ~ SD7, CS1#, CLE, ALE, FRE#, FWE#, FRB#
 * @n: chip select number(1 ~ 6)
 */
#define __gpio_as_nand_8bit(n)						\
	do {								\
		REG_GPIO_PXINTC(0) = 0x000c00ff; /* SD0 ~ SD7, FRE#, FWE# */ \
		REG_GPIO_PXMASKC(0) = 0x000c00ff;			\
		REG_GPIO_PXPAT1C(0) = 0x000c00ff;			\
		REG_GPIO_PXPAT0C(0) = 0x000c00ff;			\
		REG_GPIO_PXPENS(0) = 0x000c00ff;			\
									\
		REG_GPIO_PXINTC(1) = 0x00000003; /* CLE(SA2), ALE(SA3) */ \
		REG_GPIO_PXMASKC(1) = 0x00000003;			\
		REG_GPIO_PXPAT1C(1) = 0x00000003;			\
		REG_GPIO_PXPAT0C(1) = 0x00000003;			\
		REG_GPIO_PXPENS(1) = 0x00000003;			\
									\
		REG_GPIO_PXINTC(0) = 0x00200000 << ((n)-1); /* CSn */	\
		REG_GPIO_PXMASKC(0) = 0x00200000 << ((n)-1);            \
		REG_GPIO_PXPAT1C(0) = 0x00200000 << ((n)-1);            \
		REG_GPIO_PXPAT0C(0) = 0x00200000 << ((n)-1);            \
		REG_GPIO_PXPENS(0) = 0x00200000 << ((n)-1);		\
	} while (0)

#define __gpio_as_nand_rbinput()					\
	do {								\
		REG_GPIO_PXINTC(0) = 0x08100000; /* FRB0-1#(input) */	\
		REG_GPIO_PXMASKS(0) = 0x08100000;			\
		REG_GPIO_PXPAT1S(0) = 0x08100000;			\
		REG_GPIO_PXPENS(0) = 0x08100000;			\
	} while (0)

#define __gpio_as_nand_rbintc()						\
	do {								\
		REG_GPIO_PXINTS(0) = 0x08100000; /* FRB0-1#(rising) */	\
		REG_GPIO_PXMASKC(0) = 0x08100000;			\
		REG_GPIO_PXPAT1S(0) = 0x08100000;			\
		REG_GPIO_PXPAT0S(0) = 0x08100000;		\
	} while (0)

#define __gpio_as_nand_cs(n)						\
	do{								\
		REG_GPIO_PXINTC(0) = 0x00200000 << ((n)-1); /* CSn */	\
		REG_GPIO_PXMASKC(0) = 0x00200000 << ((n)-1);            \
		REG_GPIO_PXPAT1C(0) = 0x00200000 << ((n)-1);            \
		REG_GPIO_PXPAT0C(0) = 0x00200000 << ((n)-1);            \
		REG_GPIO_PXPENS(0) = 0x00200000 << ((n)-1);		\
	}while(0)

#define __gpio_as_nand_16bit(n)						\
do {		              						\
	REG_GPIO_PXINTC(0) = 0x000c00ff; /* SD0 ~ SD7, FRE#, FWE# */   \
	REG_GPIO_PXMASKC(0) = 0x000c00ff;				\
	REG_GPIO_PXPAT1C(0) = 0x000c00ff;				\
	REG_GPIO_PXPAT0C(0) = 0x000c00ff;				\
	REG_GPIO_PXPENS(0) = 0x000c00ff;				\
									\
	REG_GPIO_PXINTC(6) = 0x0003fc00; /* SD8 ~ SD15 */		\
	REG_GPIO_PXMASKC(6) = 0x0003fc00;				\
	REG_GPIO_PXPAT1C(6) = 0x0003fc00;				\
	REG_GPIO_PXPAT0S(6) = 0x0003fc00;				\
	REG_GPIO_PXPENS(6) = 0x0003fc00;				\
			\
									\
	REG_GPIO_PXINTC(1) = 0x00000003; /* CLE(SA2), ALE(SA3) */	\
	REG_GPIO_PXMASKC(1) = 0x00000003;				\
	REG_GPIO_PXPAT1C(1) = 0x00000003;				\
	REG_GPIO_PXPAT0C(1) = 0x00000003;				\
	REG_GPIO_PXPENS(1) = 0x00000003;				\
									\
	REG_GPIO_PXINTC(0) = 0x00200000 << ((n)-1); /* CSn */		\
	REG_GPIO_PXMASKC(0) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPAT1C(0) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPAT0C(0) = 0x00200000 << ((n)-1);			\
	REG_GPIO_PXPENS(0) = 0x00200000 << ((n)-1);			\
} while (0)

/*
 * Toggle nDQS
 */
#define __gpio_as_nand_toggle()						\
do {									\
	REG_GPIO_PXINTC(0) = 0x20000000;				\
	REG_GPIO_PXMASKC(0) = 0x20000000;				\
	REG_GPIO_PXPAT1C(0) = 0x20000000;				\
	REG_GPIO_PXPAT0C(0) = 0x20000000;				\
	REG_GPIO_PXPENC(0) = 0x20000000;				\
} while (0)

/*
 * SD0 ~ SD7, SA0 ~ SA5, CS2#, RD#, WR#, WAIT#
 */
#define __gpio_as_nor()							\
do {								        \
	/* SD0 ~ SD7, RD#, WR#, CS2#, WAIT# */				\
	REG_GPIO_PXINTC(0) = 0x084300ff;				\
	REG_GPIO_PXMASKC(0) = 0x084300ff;				\
	REG_GPIO_PXPAT1C(0) = 0x084300ff;				\
	REG_GPIO_PXPAT0C(0) = 0x084300ff;				\
	REG_GPIO_PXPENS(0) = 0x084300ff;				\
	/* SA0 ~ SA5 */							\
	REG_GPIO_PXINTC(1) = 0x0000003f;				\
	REG_GPIO_PXMASKC(1) = 0x0000003f;				\
	REG_GPIO_PXPAT1C(1) = 0x0000003f;				\
	REG_GPIO_PXPAT0C(1) = 0x0000003f;				\
	REG_GPIO_PXPENS(1) = 0x0000003f;				\
} while (0)


/*
 * LCD_D0~LCD_D7, LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_8bit()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x000c03ff;	\
	REG_GPIO_PXTRGC(2) = 0x000c03ff;	\
	REG_GPIO_PXSELC(2) = 0x000c03ff;	\
	REG_GPIO_PXPES(2) = 0x000c03ff;		\
} while (0)

/*
 * LCD_R3~LCD_R7, LCD_G2~LCD_G7, LCD_B3~LCD_B7,
 * LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_16bit()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x0f8ff3f8;	\
	REG_GPIO_PXTRGC(2) = 0x0f8ff3f8;	\
	REG_GPIO_PXSELC(2) = 0x0f8ff3f8;	\
	REG_GPIO_PXPES(2) = 0x0f8ff3f8;		\
} while (0)

/*
 * LCD_R2~LCD_R7, LCD_G2~LCD_G7, LCD_B2~LCD_B7,
 * LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_18bit()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x0fcff3fc;	\
	REG_GPIO_PXTRGC(2) = 0x0fcff3fc;	\
	REG_GPIO_PXSELC(2) = 0x0fcff3fc;	\
	REG_GPIO_PXPES(2) = 0x0fcff3fc;		\
} while (0)

/*
 * LCD_R0~LCD_R7, LCD_G0~LCD_G7, LCD_B0~LCD_B7,
 * LCD_PCLK, LCD_HSYNC, LCD_VSYNC, LCD_DE
 */
#define __gpio_as_lcd_24bit()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x0fffffff;	\
	REG_GPIO_PXTRGC(2) = 0x0fffffff;	\
	REG_GPIO_PXSELC(2) = 0x0fffffff;	\
	REG_GPIO_PXPES(2) = 0x0fffffff;		\
} while (0)

/*
 *  LCD_CLS, LCD_SPL, LCD_PS, LCD_REV
 */
#define __gpio_as_lcd_special()			\
do {						\
	REG_GPIO_PXFUNS(2) = 0x0fffffff;	\
	REG_GPIO_PXTRGC(2) = 0x0fffffff;	\
	REG_GPIO_PXSELC(2) = 0x0feffbfc;	\
	REG_GPIO_PXSELS(2) = 0x00100403;	\
	REG_GPIO_PXPES(2) = 0x0fffffff;		\
} while (0)

/*
 * CIM_D0~CIM_D7, CIM_MCLK, CIM_PCLK, CIM_VSYNC, CIM_HSYNC
 */
#define __gpio_as_cim()				\
do {						\
	REG_GPIO_PXFUNS(1) = 0x0003ffc0;	\
	REG_GPIO_PXTRGC(1) = 0x0003ffc0;	\
	REG_GPIO_PXSELC(1) = 0x0003ffc0;	\
	REG_GPIO_PXPES(1)  = 0x0003ffc0;	\
} while (0)

/*
 * SDATO, SDATI, BCLK, SYNC, SCLK_RSTN(gpio sepc) or
 * SDATA_OUT, SDATA_IN, BIT_CLK, SYNC, SCLK_RESET(aic spec)
 */
#define __gpio_as_aic()				\
do {						\
	REG_GPIO_PXFUNS(4) = 0x16c00000;	\
	REG_GPIO_PXTRGC(4) = 0x02c00000;	\
	REG_GPIO_PXTRGS(4) = 0x14000000;	\
	REG_GPIO_PXSELC(4) = 0x14c00000;	\
	REG_GPIO_PXSELS(4) = 0x02000000;	\
	REG_GPIO_PXPES(4)  = 0x16c00000;	\
} while (0)

/*
 * MSC0_CMD, MSC0_CLK, MSC0_D0 ~ MSC0_D3
 */
#define __gpio_a_as_msc0_4bit()			\
do {						\
	REG_GPIO_PXINTC(0)  = 0x00fc0000;	\
	REG_GPIO_PXMASKC(0) = 0x00fc0000;	\
	REG_GPIO_PXPAT1C(0) = 0x00fc0000;	\
	REG_GPIO_PXPAT0S(0) = 0x00fc0000;	\
} while (0)

#define __gpio_e_as_msc0_4bit()			\
do {						\
	REG_GPIO_PXINTC(4)  = 0x30f00000;	\
	REG_GPIO_PXMASKC(4) = 0x30f00000;	\
	REG_GPIO_PXPAT1C(4) = 0x30f00000;	\
	REG_GPIO_PXPAT0C(4) = 0x30f00000;	\
} while (0)


#define __gpio_b_as_msc2_4bit()			\
do {						\
	REG_GPIO_PXINTC(1)  = 0xf0300000;	\
	REG_GPIO_PXMASKC(1) = 0xf0300000;	\
	REG_GPIO_PXPAT1C(1) = 0xf0300000;	\
	REG_GPIO_PXPAT0C(1) = 0xf0300000;	\
} while (0)

#define __gpio_as_msc 	__gpio_e_as_msc0_4bit /* default as msc0 4bit */

/*
 * TSCLK, TSSTR, TSFRM, TSFAIL, TSDI0~7
 */
#define __gpio_as_tssi_1()			\
do {						\
	REG_GPIO_PXFUNS(1) = 0x0003ffc0;	\
	REG_GPIO_PXTRGC(1) = 0x0003ffc0;	\
	REG_GPIO_PXSELS(1) = 0x0003ffc0;	\
	REG_GPIO_PXPES(1)  = 0x0003ffc0;	\
} while (0)

/*
 * TSCLK, TSSTR, TSFRM, TSFAIL, TSDI0~7
 */
#define __gpio_as_tssi_2()			\
do {						\
	REG_GPIO_PXFUNS(1) = 0xfff00000;	\
	REG_GPIO_PXTRGC(1) = 0x0fc00000;	\
	REG_GPIO_PXTRGS(1) = 0xf0300000;	\
	REG_GPIO_PXSELC(1) = 0xfff00000;	\
	REG_GPIO_PXPES(1)  = 0xfff00000;	\
} while (0)

/*
 * SSI_CE0, SSI_CE1, SSI_GPC, SSI_CLK, SSI_DT, SSI_DR
 */
#define __gpio_as_ssi()				\
do {						\
	REG_GPIO_PXFUNS(0) = 0x002c0000; /* SSI0_CE0, SSI0_CLK, SSI0_DT	*/ \
	REG_GPIO_PXTRGS(0) = 0x002c0000;	\
	REG_GPIO_PXSELC(0) = 0x002c0000;	\
	REG_GPIO_PXPES(0)  = 0x002c0000;	\
						\
	REG_GPIO_PXFUNS(0) = 0x00100000; /* SSI0_DR */	\
	REG_GPIO_PXTRGC(0) = 0x00100000;	\
	REG_GPIO_PXSELS(0) = 0x00100000;	\
	REG_GPIO_PXPES(0)  = 0x00100000;	\
} while (0)

/*
 * SSI_CE0, SSI_CE2, SSI_GPC, SSI_CLK, SSI_DT, SSI1_DR
 */
#define __gpio_as_ssi_1()			\
do {						\
	REG_GPIO_PXFUNS(5) = 0x0000fc00;	\
	REG_GPIO_PXTRGC(5) = 0x0000fc00;	\
	REG_GPIO_PXSELC(5) = 0x0000fc00;	\
	REG_GPIO_PXPES(5)  = 0x0000fc00;	\
} while (0)

/* Port B
 * SSI2_CE0, SSI2_CE2, SSI2_GPC, SSI2_CLK, SSI2_DT, SSI12_DR
 */
#define __gpio_as_ssi2_1()			\
do {						\
	REG_GPIO_PXFUNS(5) = 0xf0300000;	\
	REG_GPIO_PXTRGC(5) = 0xf0300000;	\
	REG_GPIO_PXSELS(5) = 0xf0300000;	\
	REG_GPIO_PXPES(5)  = 0xf0300000;	\
} while (0)

/*
 * I2C_SCK, I2C_SDA
 */
#define __gpio_as_i2c()				\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00003000;	\
	REG_GPIO_PXSELC(4) = 0x00003000;	\
	REG_GPIO_PXPES(4)  = 0x00003000;	\
} while (0)

/*
 * PWM0
 */
#define __gpio_as_pwm0()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00100000;	\
	REG_GPIO_PXSELC(4) = 0x00100000;	\
	REG_GPIO_PXPES(4) = 0x00100000;		\
} while (0)

/*
 * PWM1
 */
#define __gpio_as_pwm1()			\
do {						\
	REG_GPIO_PXFUNS(5) = 0x00000800;	\
	REG_GPIO_PXSELC(5) = 0x00000800;	\
	REG_GPIO_PXPES(5) = 0x00000800;		\
} while (0)

/*
 * PWM2
 */
#define __gpio_as_pwm2()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00400000;	\
	REG_GPIO_PXSELC(4) = 0x00400000;	\
	REG_GPIO_PXPES(4) = 0x00400000;		\
} while (0)

/*
 * PWM3
 */
#define __gpio_as_pwm3()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x00800000;	\
	REG_GPIO_PXSELC(4) = 0x00800000;	\
	REG_GPIO_PXPES(4) = 0x00800000;		\
} while (0)

/*
 * PWM4
 */
#define __gpio_as_pwm4()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x01000000;	\
	REG_GPIO_PXSELC(4) = 0x01000000;	\
	REG_GPIO_PXPES(4) = 0x01000000;		\
} while (0)

/*
 * PWM5
 */
#define __gpio_as_pwm5()			\
do {						\
	REG_GPIO_PXFUNS(4) = 0x02000000;	\
	REG_GPIO_PXSELC(4) = 0x02000000;	\
	REG_GPIO_PXPES(4) = 0x02000000;		\
} while (0)

/*
 * n = 0 ~ 5
 */
#define __gpio_as_pwm(n)	__gpio_as_pwm##n()


//-------------------------------------------
// GPIO or Interrupt Mode

#define __gpio_get_port(p)	(REG_GPIO_PXPIN(p))

#define __gpio_port_as_output0(p, o)		\
do {						\
    REG_GPIO_PXINTC(p) = (1 << (o));		\
    REG_GPIO_PXMASKS(p) = (1 << (o));		\
    REG_GPIO_PXPAT1C(p) = (1 << (o));		\
    REG_GPIO_PXPAT0C(p) = (1 << (o));		\
} while (0)

#define __gpio_port_as_output1(p, o)		\
do {						\
    REG_GPIO_PXINTC(p) = (1 << (o));		\
    REG_GPIO_PXMASKS(p) = (1 << (o));		\
    REG_GPIO_PXPAT1C(p) = (1 << (o));		\
    REG_GPIO_PXPAT0S(p) = (1 << (o));		\
} while (0)

#define __gpio_port_as_input(p, o)		\
do {						\
    REG_GPIO_PXINTC(p) = (1 << (o));		\
    REG_GPIO_PXMASKS(p) = (1 << (o));		\
    REG_GPIO_PXPAT1S(p) = (1 << (o));		\
    REG_GPIO_PXPAT0C(p) = (1 << (o));		\
} while (0)

#define __gpio_as_output0(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_as_output0(p, o);		\
} while (0)

#define __gpio_as_output1(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_as_output1(p, o);		\
} while (0)

#define __gpio_as_input(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	__gpio_port_as_input(p, o);		\
} while (0)

#define __gpio_get_pin(n)			\
({						\
	unsigned int p, o, v;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	if (__gpio_get_port(p) & (1 << o))	\
		v = 1;				\
	else					\
		v = 0;				\
	v;					\
})

#define __gpio_set_pin            __gpio_as_output1           
#define __gpio_clear_pin          __gpio_as_output0           


#define __gpio_as_irq_high_level(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
	REG_GPIO_PXTRGC(p) = (1 << o);		\
	REG_GPIO_PXFUNC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
	REG_GPIO_PXDIRS(p) = (1 << o);		\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_as_irq_low_level(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
	REG_GPIO_PXTRGC(p) = (1 << o);		\
	REG_GPIO_PXFUNC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
	REG_GPIO_PXDIRC(p) = (1 << o);		\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_as_irq_rise_edge(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
	REG_GPIO_PXTRGS(p) = (1 << o);		\
	REG_GPIO_PXFUNC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
	REG_GPIO_PXDIRS(p) = (1 << o);		\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_as_irq_fall_edge(n)		\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
	REG_GPIO_PXTRGS(p) = (1 << o);		\
	REG_GPIO_PXFUNC(p) = (1 << o);		\
	REG_GPIO_PXSELS(p) = (1 << o);		\
	REG_GPIO_PXDIRC(p) = (1 << o);		\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_mask_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMS(p) = (1 << o);		\
} while (0)

#define __gpio_unmask_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXIMC(p) = (1 << o);		\
} while (0)

#define __gpio_ack_irq(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXFLGC(p) = (1 << o);		\
} while (0)

#define __gpio_get_irq()			\
({						\
	unsigned int p, i, tmp, v = 0;		\
	for (p = 3; p >= 0; p--) {		\
		tmp = REG_GPIO_PXFLG(p);	\
		for (i = 0; i < 32; i++)	\
			if (tmp & (1 << i))	\
				v = (32*p + i);	\
	}					\
	v;					\
})

#define __gpio_group_irq(n)			\
({						\
	register int tmp, i;			\
	tmp = REG_GPIO_PXFLG((n));		\
	for (i=31;i>=0;i--)			\
		if (tmp & (1 << i))		\
			break;			\
	i;					\
})

#define __gpio_enable_pull(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXPEC(p) = (1 << o);		\
} while (0)

#define __gpio_disable_pull(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXPES(p) = (1 << o);		\
} while (0)


/***************************************************************************
 * CPM
 ***************************************************************************/
#define __cpm_get_pllm(cpxpcr) \
	((cpxpcr & CPM_CPXPCR_XPLLM_MASK) >> CPM_CPXPCR_XPLLM_BIT)
#define __cpm_get_plln(cpxpcr) \
	((cpxpcr & CPM_CPXPCR_XPLLN_MASK) >> CPM_CPXPCR_XPLLN_BIT)
#define __cpm_get_pllod(cpxpcr) \
	((cpxpcr & CPM_CPXPCR_XPLLOD_MASK) >> CPM_CPXPCR_XPLLOD_BIT)

#define __cpm_get_cdiv() \
	((REG_CPM_CPCCR & CPM_CPCCR_CDIV_MASK) >> CPM_CPCCR_CDIV_BIT)
#define __cpm_get_hdiv() \
	((REG_CPM_CPCCR & CPM_CPCCR_H0DIV_MASK) >> CPM_CPCCR_H0DIV_BIT)
#define __cpm_get_h2div() \
	((REG_CPM_CPCCR & CPM_CPCCR_H2DIV_MASK) >> CPM_CPCCR_H2DIV_BIT)
#define __cpm_get_pdiv() \
	((REG_CPM_CPCCR & CPM_CPCCR_PDIV_MASK) >> CPM_CPCCR_PDIV_BIT)

#define __cpm_get_sdiv() \
	((REG_CPM_CPCCR & CPM_CPCCR_SDIV_MASK) >> CPM_CPCCR_SDIV_BIT)
#define __cpm_get_i2sdiv() \
	((REG_CPM_I2SCDR & CPM_I2SCDR_I2SDIV_MASK) >> CPM_I2SCDR_I2SDIV_BIT)
#define __cpm_get_pixdiv() \
	((REG_CPM_LPCDR & CPM_LPCDR_PIXDIV_MASK) >> CPM_LPCDR_PIXDIV_BIT)
#define __cpm_get_mscdiv(n) \
	((REG_CPM_MSCCDR(n) & CPM_MSCCDR_MSCDIV_MASK) >> CPM_MSCCDR_MSCDIV_BIT)
#define __cpm_get_ssidiv() \
	((REG_CPM_SSICCDR & CPM_SSICDR_SSICDIV_MASK) >> CPM_SSICDR_SSIDIV_BIT)
#define __cpm_get_pcmdiv() \
	((REG_CPM_PCMCDR & CPM_PCMCDR_PCMCD_MASK) >> CPM_PCMCDR_PCMCD_BIT)

#define __cpm_set_cdiv(v) \
	(REG_CPM_CPCCR = (REG_CPM_CPCCR & ~CPM_CPCCR_CDIV_MASK) | ((v) << (CPM_CPCCR_CDIV_BIT)))
#define __cpm_set_hdiv(v) \
	(REG_CPM_CPCCR = (REG_CPM_CPCCR & ~CPM_CPCCR_HDIV_MASK) | ((v) << (CPM_CPCCR_HDIV_BIT)))
#define __cpm_set_pdiv(v) \
	(REG_CPM_CPCCR = (REG_CPM_CPCCR & ~CPM_CPCCR_PDIV_MASK) | ((v) << (CPM_CPCCR_PDIV_BIT)))
#define __cpm_set_mdiv(v) \
	(REG_CPM_CPCCR = (REG_CPM_CPCCR & ~CPM_CPCCR_MDIV_MASK) | ((v) << (CPM_CPCCR_MDIV_BIT)))
#define __cpm_set_h1div(v) \
	(REG_CPM_CPCCR = (REG_CPM_CPCCR & ~CPM_CPCCR_H1DIV_MASK) | ((v) << (CPM_CPCCR_H1DIV_BIT)))
#define __cpm_set_udiv(v) \
	(REG_CPM_CPCCR = (REG_CPM_CPCCR & ~CPM_CPCCR_UDIV_MASK) | ((v) << (CPM_CPCCR_UDIV_BIT)))
#define __cpm_set_i2sdiv(v) \
	(REG_CPM_I2SCDR = (REG_CPM_I2SCDR & ~CPM_I2SCDR_I2SDIV_MASK) | ((v) << (CPM_I2SCDR_I2SDIV_BIT)))
#define __cpm_set_pixdiv(v) \
	(REG_CPM_LPCDR = (REG_CPM_LPCDR & ~CPM_LPCDR_PIXDIV_MASK) | ((v) << (CPM_LPCDR_PIXDIV_BIT)))
#define __cpm_set_mscdiv(v) \
	(REG_CPM_MSCCDR = (REG_CPM_MSCCDR & ~CPM_MSCCDR_MSCDIV_MASK) | ((v) << (CPM_MSCCDR_MSCDIV_BIT)))
#define __cpm_set_ssidiv(v) \
	(REG_CPM_SSICDR = (REG_CPM_SSICDR & ~CPM_SSICDR_SSIDIV_MASK) | ((v) << (CPM_SSICDR_SSIDIV_BIT)))
#define __cpm_set_pcmdiv(v) \
	(REG_CPM_PCMCDR = (REG_CPM_PCMCDR & ~CPM_PCMCDR_PCMCD_MASK) | ((v) << (CPM_PCMCDR_PCMCD_BIT)))

#define __cpm_select_i2sclk_pll1() 	(REG_CPM_I2SCDR |= CPM_I2SCDR_I2PCS)
#define __cpm_select_i2sclk_pll0()	(REG_CPM_I2SCDR &= ~CPM_I2SCDR_I2PCS)
#define __cpm_select_otgclk_pll1() 	(REG_CPM_USBCDR |= CPM_USBCDR_UPCS)
#define __cpm_select_otgclk_pll0()	(REG_CPM_USBCDR &= ~CPM_USBCDR_UPCS)
#define __cpm_select_lcdpclk_pll1() 	(REG_CPM_LPCDR |= CPM_LPCDR_LPCS)
#define __cpm_select_lcdpclk_pll0()	(REG_CPM_LPCDR &= ~CPM_LPCDR_LPCS)
#define __cpm_select_uhcclk_pll1() 	(REG_CPM_UHCCDR |= CPM_UHCCDR_UHPCS)
#define __cpm_select_uhcclk_pll0()	(REG_CPM_UHCCDR &= ~CPM_UHCCDR_UHPCS)
#define __cpm_select_gpsclk_pll1() 	(REG_CPM_GPSCDR |= CPM_GPSCDR_GPCS)
#define __cpm_select_gpsclk_pll0()	(REG_CPM_GPSCDR &= ~CPM_GPSCDR_GPCS)
#define __cpm_select_pcmclk_pll1() 	(REG_CPM_PCMCDR |= CPM_PCMCDR_PCMPCS)
#define __cpm_select_pcmclk_pll0()	(REG_CPM_PCMCDR &= ~CPM_PCMCDR_PCMPCS)
#define __cpm_select_gpuclk_pll1() 	(REG_CPM_GPUCDR |= CPM_GPUCDR_GPCS)
#define __cpm_select_gpuclk_pll0()	(REG_CPM_GPUCDR &= ~CPM_GPUCDR_GPCS)

#define __cpm_select_pcmclk_pll() 	(REG_CPM_PCMCDR |= CPM_PCMCDR_PCMS)
#define __cpm_select_pcmclk_exclk() 	(REG_CPM_PCMCDR &= ~CPM_PCMCDR_PCMS)
#define __cpm_select_pixclk_ext()	(REG_CPM_LPCDR |= CPM_LPCDR_LPCS)
#define __cpm_select_pixclk_pll()	(REG_CPM_LPCDR &= ~CPM_LPCDR_LPCS)
#define __cpm_select_tveclk_exclk()	(REG_CPM_LPCDR |= CPM_CPCCR_LSCS)
#define __cpm_select_tveclk_pll()	(REG_CPM_LPCDR &= ~CPM_LPCDR_LSCS)
#define __cpm_select_pixclk_lcd()	(REG_CPM_LPCDR &= ~CPM_LPCDR_LTCS)
#define __cpm_select_pixclk_tve()	(REG_CPM_LPCDR |= CPM_LPCDR_LTCS)
#define __cpm_select_i2sclk_exclk()	(REG_CPM_I2SCDR &= ~CPM_I2SCDR_I2CS)
#define __cpm_select_i2sclk_pll()	(REG_CPM_I2SCDR |= CPM_I2SCDR_I2CS)
//#define __cpm_select_usbclk_exclk()	(REG_CPM_CPCCR &= ~CPM_CPCCR_UCS)
//#define __cpm_select_usbclk_pll()	(REG_CPM_CPCCR |= CPM_CPCCR_UCS)

#define __cpm_get_cclk_doze_duty() \
	((REG_CPM_LCR & CPM_LCR_DOZE_DUTY_MASK) >> CPM_LCR_DOZE_DUTY_BIT)
#define __cpm_set_cclk_doze_duty(v) \
	(REG_CPM_LCR = (REG_CPM_LCR & ~CPM_LCR_DOZE_DUTY_MASK) | ((v) << (CPM_LCR_DOZE_DUTY_BIT)))

#define __cpm_doze_mode()		(REG_CPM_LCR |= CPM_LCR_DOZE_ON)
#define __cpm_idle_mode() \
	(REG_CPM_LCR = (REG_CPM_LCR & ~CPM_LCR_LPM_MASK) | CPM_LCR_LPM_IDLE)
#define __cpm_sleep_mode() \
	(REG_CPM_LCR = (REG_CPM_LCR & ~CPM_LCR_LPM_MASK) | CPM_LCR_LPM_SLEEP)

#define __cpm_stop_all() 	\
	do {\
		(REG_CPM_CLKGR0 = 0xffffffff);\
		(REG_CPM_CLKGR1 = 0x3ff);\
	}while(0)
#define __cpm_stop_ddr1()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_DDR1)
#define __cpm_stop_ddr0()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_DDR0)
#define __cpm_stop_ipu()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_IPU)
#define __cpm_stop_lcd()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_LCD)
#define __cpm_stop_tve()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_TVE)
#define __cpm_stop_cim()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_CIM)
#define __cpm_stop_i2c2()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_I2C2)
#define __cpm_stop_uhc()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_UHC)
#define __cpm_stop_mac()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_MAC)
#define __cpm_stop_gps()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_GPS)
#define __cpm_stop_pdmac()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_PDMAC)
#define __cpm_stop_ssi2()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_SSI2)
#define __cpm_stop_ssi1()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_SSI1)
#define __cpm_stop_uart3()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_UART3)
#define __cpm_stop_uart2()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_UART2)
#define __cpm_stop_uart1()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_UART1)
#define __cpm_stop_uart0()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_UART0)
#define __cpm_stop_sadc()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_SADC)
#define __cpm_stop_kbc()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_KBC)
#define __cpm_stop_msc2()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_MSC2)
#define __cpm_stop_msc1()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_MSC1)
#define __cpm_stop_owi()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_OWI)
#define __cpm_stop_tssi()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_TSSI)
#define __cpm_stop_aic()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_AIC)
#define __cpm_stop_scc()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_SCC)
#define __cpm_stop_i2c1()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_I2C1)
#define __cpm_stop_i2c0()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_I2C0)
#define __cpm_stop_ssi0()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_SSI0)
#define __cpm_stop_msc0()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_MSC0)
#define __cpm_stop_otg()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_OTG)
//#define __cpm_stop_bch()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_BCH)
#define __cpm_stop_nemc()	(REG_CPM_CLKGR0 |= CPM_CLKGR0_NEMC)
#define __cpm_stop_p1()		(REG_CPM_CLKGR0 |= CPM_CLKGR1_P1)
#define __cpm_stop_x2d()	(REG_CPM_CLKGR0 |= CPM_CLKGR1_X2D)
#define __cpm_stop_des()	(REG_CPM_CLKGR0 |= CPM_CLKGR1_DES)
#define __cpm_stop_i2c4()	(REG_CPM_CLKGR0 |= CPM_CLKGR1_I2C4)
#define __cpm_stop_ahb()	(REG_CPM_CLKGR1 |= CPM_CLKGR1_AHB)
#define __cpm_stop_uart4()	(REG_CPM_CLKGR1 |= CPM_CLKGR1_UART4)
#define __cpm_stop_hdmi()	(REG_CPM_CLKGR1 |= CPM_CLKGR1_HDMI)
#define __cpm_stop_otg1()	(REG_CPM_CLKGR1 |= CPM_CLKGR1_OTG1)
#define __cpm_stop_gpvlc()	(REG_CPM_CLKGR1 |= CPM_CLKGR1_GPVLC)
#define __cpm_stop_aic1()	(REG_CPM_CLKGR1 |= CPM_CLKGR1_AIC1)
#define __cpm_stop_compres()	(REG_CPM_CLKGR1 |= CPM_CLKGR1_COMPRES)
#define __cpm_stop_gpu()	(REG_CPM_CLKGR1 |= CPM_CLKGR1_GPU)
#define __cpm_stop_pcm()	(REG_CPM_CLKGR1 |= CPM_CLKGR1_PCM)
#define __cpm_stop_vpu()	(REG_CPM_CLKGR1 |= CPM_CLKGR1_VPU)
#define __cpm_stop_tssi1()	(REG_CPM_CLKGR1 |= CPM_CLKGR1_TSSI1)
#define __cpm_stop_i2c3()	(REG_CPM_CLKGR0 |= CPM_CLKGR1_I2C3)

#define __cpm_start_all() 	\
	do {\
		REG_CPM_CLKGR0 = 0x0;\
		REG_CPM_CLKGR1 = 0x0;\
	} while(0)
#define __cpm_start_ddr1()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_DDR1)
#define __cpm_start_ddr0()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_DDR0)
#define __cpm_start_ipu()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_IPU)
#define __cpm_start_lcd()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_LCD)
#define __cpm_start_tve()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_TVE)
#define __cpm_start_cim()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_CIM)
#define __cpm_start_i2c2()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_I2C2)
#define __cpm_start_uhc()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_UHC)
#define __cpm_start_mac()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_MAC)
#define __cpm_start_gps()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_GPS)
#define __cpm_start_pdmac()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_PDMAC)
#define __cpm_start_ssi2()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_SSI2)
#define __cpm_start_ssi1()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_SSI1)
#define __cpm_start_uart3()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_UART3)
#define __cpm_start_uart2()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_UART2)
#define __cpm_start_uart1()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_UART1)
#define __cpm_start_uart0()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_UART0)
#define __cpm_start_sadc()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_SADC)
#define __cpm_start_kbc()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_KBC)
#define __cpm_start_msc2()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_MSC2)
#define __cpm_start_msc1()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_MSC1)
#define __cpm_start_owi()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_OWI)
#define __cpm_start_tssi()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_TSSI)
#define __cpm_start_aic()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_AIC)
#define __cpm_start_scc()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_SCC)
#define __cpm_start_i2c1()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_I2C1)
#define __cpm_start_i2c0()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_I2C0)
#define __cpm_start_ssi0()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_SSI0)
#define __cpm_start_msc0()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_MSC0)
#define __cpm_start_otg()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_OTG)
//#define __cpm_start_bch()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_BCH)
#define __cpm_start_nemc()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR0_NEMC)
#define __cpm_start_p1()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR1_P1)
#define __cpm_start_x2d()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR1_X2D)
#define __cpm_start_des()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR1_DES)
#define __cpm_start_i2c4()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR1_I2C4)
#define __cpm_start_ahb()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR1_AHB)
#define __cpm_start_uart4()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR1_UART4)
#define __cpm_start_hdmi()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR1_HDMI)
#define __cpm_start_otg1()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR1_OTG1)
#define __cpm_start_gpvlc()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR1_GPVLC)
#define __cpm_start_aic1()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR1_AIC1)
#define __cpm_start_compres()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR1_COMPRES)
#define __cpm_start_gpu()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR0_GPU)
#define __cpm_start_pcm()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR0_PCM)
#define __cpm_start_vpu()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR1_VPU)
#define __cpm_start_tssi1()	(REG_CPM_CLKGR1 &= ~CPM_CLKGR1_TSSI1)
#define __cpm_start_i2c3()	(REG_CPM_CLKGR0 &= ~CPM_CLKGR1_I2C3)


#define __cpm_get_o1st() \
	((REG_CPM_OPCR & CPM_OPCR_O1ST_MASK) >> CPM_OPCR_O1ST_BIT)
#define __cpm_set_o1st(v) \
	(REG_CPM_OPCR = (REG_CPM_OPCR & ~CPM_OPCR_O1ST_MASK) | ((v) << (CPM_OPCR_O1ST_BIT)))
#define __cpm_suspend_udcphy()		(REG_CPM_OPCR &= ~CPM_OPCR_UDCPHY_ENABLE)
#define __cpm_enable_osc_in_sleep()	(REG_CPM_OPCR |= CPM_OPCR_OSC_ENABLE)
#define __cpm_select_rtcclk_rtc()	(REG_CPM_OPCR |= CPM_OPCR_ERCS)
#define __cpm_select_rtcclk_exclk()	(REG_CPM_OPCR &= ~CPM_OPCR_ERCS)


/* CPM scratch pad protected register(CPSPPR) */
#define CPSPPR_CPSPR_WRITABLE   (0x00005a5a)
#define RECOVERY_SIGNATURE      0x52454359      /* means "RECY" */
#define RECOVERY_SIGNATURE_SEC  0x800      /* means "RECY" */

#define __cpm_get_scrpad()	REG_CPM_CPSPR
#define __cpm_set_scrpad(data)				\
do {							\
	REG_CPM_CPSPPR = CPSPPR_CPSPR_WRITABLE;		\
	REG_CPM_CPSPR = data;				\
	REG_CPM_CPSPPR = ~CPSPPR_CPSPR_WRITABLE;	\
} while (0)

extern volatile u32 JZ_EXTAL;
extern volatile u32 CONFIG_BAUDRATE;
extern volatile u32 UART_BASE;
#define JZ_EXTAL2		32768 /* RTC clock */

typedef enum {
	SCLK_APLL = 0,
	SCLK_MPLL,
	SCLK_EPLL,
	SCLK_VPLL,
} sclk;

/* xPLL output frequency */
void serial_puts (const char *s);
void serial_printf(char *fmt, ...);
void serial_put_hex(unsigned int  d);

static __inline__ unsigned int __cpm_get_xpllout(sclk sclk_name)
{
	unsigned long m, n, od, pllout = JZ_EXTAL;
	unsigned long cpxpcr;
	unsigned long xpllen = 0;
	unsigned long xpllbp = 0;

	switch (sclk_name) {
	default:
		serial_puts("WARNING unknow clk name use SCLK_APLL\n");
	case SCLK_APLL:
		cpxpcr = REG_CPM_CPAPCR;
		xpllen = CPM_CPAPCR_XPLLEN;
		xpllbp = CPM_CPAPCR_XPLLBP;
		break;
	case SCLK_MPLL:
		cpxpcr = REG_CPM_CPMPCR;
		xpllen = CPM_CPMPCR_XPLLEN;
		xpllbp = CPM_CPMPCR_XPLLBP;
		break;
	}
	
	serial_printf("REG_CPM_CPAPCR = %x\n",REG_CPM_CPAPCR);
	if ((cpxpcr & xpllen) && (!(cpxpcr & xpllbp))) {
		m = __cpm_get_pllm(cpxpcr) + 1;
		n = __cpm_get_plln(cpxpcr) + 1;
		od = __cpm_get_pllod(cpxpcr) + 1;
		serial_printf("m = %d,n = %d,od = %d JZ_EXTAL = %d\n",m , n, od,JZ_EXTAL);
		pllout = ((JZ_EXTAL) * m / (n * od));
	}

	return pllout;
}

/* CPU core clock */
static __inline__ unsigned int __cpm_get_cclk(void)
{
	return __cpm_get_xpllout(SCLK_APLL) / (__cpm_get_cdiv() + 1);
}

/* Memory bus clock */
static __inline__ unsigned int __cpm_get_mclk(void)
{
	return __cpm_get_xpllout(SCLK_MPLL);
}

/* AHB0 system bus clock */
static __inline__ unsigned int __cpm_get_hclk(void)
{
	return __cpm_get_xpllout(SCLK_APLL) / (__cpm_get_hdiv() + 1);
}

/* AHB2 module clock */
static __inline__ unsigned int __cpm_get_h2clk(void)
{
	return __cpm_get_xpllout(SCLK_MPLL) / (__cpm_get_h2div() + 1);
}

/* APB peripheral bus clock */
static __inline__ unsigned int __cpm_get_pclk(void)
{
	return __cpm_get_xpllout(SCLK_APLL) / (__cpm_get_pdiv() + 1);
}

/* PLL output frequency for MSC/I2S/LCD/USB */
static __inline__ unsigned int __cpm_get_pllout2(void)
{
	return __cpm_get_xpllout(SCLK_APLL);
}

/* LCD pixel clock */
static __inline__ unsigned int __cpm_get_pixclk(void)
{
	return __cpm_get_pllout2() / (__cpm_get_pixdiv() + 1);
}

/* I2S clock */
static __inline__ unsigned int __cpm_get_i2sclk(void)
{
	if (REG_CPM_I2SCDR & CPM_I2SCDR_I2CS) {
		return __cpm_get_pllout2() / (__cpm_get_i2sdiv() + 1);
	}
	else {
		return JZ_EXTAL;
	}
}

/* USB clock */
/*
static __inline__ unsigned int __cpm_get_usbclk(void)
{
	if (REG_CPM_CPCCR & CPM_CPCCR_UCS) {
		return __cpm_get_pllout2() / (__cpm_get_udiv() + 1);
	}
	else {
		return JZ_EXTAL;
	}
}
*/
/* EXTAL clock for UART,I2C,SSI,TCU,USB-PHY */
static __inline__ unsigned int __cpm_get_extalclk(void)
{
	return JZ_EXTAL;
}

/* RTC clock for CPM,INTC,RTC,TCU,WDT */
static __inline__ unsigned int __cpm_get_rtcclk(void)
{
	return JZ_EXTAL2;
}

/*
 * Output 24MHz for SD and 16MHz for MMC.
 */
#if 0
static inline void __cpm_select_msc_clk(int n, int sd)
{
	unsigned int pllout2 = __cpm_get_pllout2();
	unsigned int div = 0;

	if (sd) {
		div = pllout2 / 24000000;
	}
	else {
		div = pllout2 / 16000000;
	}

	REG_CPM_MSCCDR = div - 1;
	REG_CPM_CPCCR |= CPM_CPCCR_CE;
}
#endif

/***************************************************************************
 * Mem Copy DMAC
 ***************************************************************************/

/* n is the DMA channel index (0 - 3) */

#define __mdmac_enable_module \
	( REG_MDMAC_DMACR |= DMAC_MDMACR_DMAE | DMAC_MDMACR_PR_012345 )
#define __mdmac_disable_module \
	( REG_MDMAC_DMACR &= ~DMAC_MDMACR_DMAE )

/* p=0,1,2,3 */
#define __mdmac_set_priority(p)			\
do {							\
	REG_MDMAC_DMACR &= ~DMAC_DMACR_PR_MASK;	\
	REG_MDMAC_DMACR |= ((p) << DMAC_DMACR_PR_BIT);	\
} while (0)

#define __mdmac_test_halt_error ( REG_MDMAC_DMACR & DMAC_MDMACR_HLT )
#define __mdmac_test_addr_error ( REG_MDMAC_DMACR & DMAC_MDMACR_AR )

#define __mdmac_channel_enable_clk \
	REG_MDMAC_DMACKE |= 1 << (n);

#define __mdmac_enable_descriptor(n) \
  ( REG_MDMAC_DCCSR((n)) &= ~DMAC_DCCSR_NDES )
#define __mdmac_disable_descriptor(n) \
  ( REG_MDMAC_DCCSR((n)) |= DMAC_DCCSR_NDES )

#define __mdmac_enable_channel(n)                 \
do {                                             \
	REG_MDMAC_DCCSR((n)) |= DMAC_DCCSR_EN;    \
} while (0)
#define __mdmac_disable_channel(n)                \
do {                                             \
	REG_MDMAC_DCCSR((n)) &= ~DMAC_DCCSR_EN;   \
} while (0)
#define __mdmac_channel_enabled(n) \
  ( REG_MDMAC_DCCSR((n)) & DMAC_DCCSR_EN )

#define __mdmac_channel_enable_irq(n) \
  ( REG_MDMAC_DCMD((n)) |= DMAC_DCMD_TIE )
#define __mdmac_channel_disable_irq(n) \
  ( REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_TIE )

#define __mdmac_channel_transmit_halt_detected(n) \
  (  REG_MDMAC_DCCSR((n)) & DMAC_DCCSR_HLT )
#define __mdmac_channel_transmit_end_detected(n) \
  (  REG_MDMAC_DCCSR((n)) & DMAC_DCCSR_TT )
#define __mdmac_channel_address_error_detected(n) \
  (  REG_DMAC_DCCSR((n)) & DMAC_DCCSR_AR )
#define __mdmac_channel_count_terminated_detected(n) \
  (  REG_MDMAC_DCCSR((n)) & DMAC_DCCSR_CT )
#define __mdmac_channel_descriptor_invalid_detected(n) \
  (  REG_MDMAC_DCCSR((n)) & DMAC_DCCSR_INV )

#define __mdmac_channel_clear_transmit_halt(n)				\
	do {								\
		/* clear both channel halt error and globle halt error */ \
		REG_MDMAC_DCCSR(n) &= ~DMAC_DCCSR_HLT;			\
		REG_MDMAC_DMACR &= ~DMAC_DMACR_HLT;	\
	} while (0)
#define __mdmac_channel_clear_transmit_end(n) \
  (  REG_MDMAC_DCCSR(n) &= ~DMAC_DCCSR_TT )
#define __mdmac_channel_clear_address_error(n)				\
	do {								\
		REG_MDMAC_DDA(n) = 0; /* clear descriptor address register */ \
		REG_MDMAC_DSAR(n) = 0; /* clear source address register */ \
		REG_MDMAC_DTAR(n) = 0; /* clear target address register */ \
		/* clear both channel addr error and globle address error */ \
		REG_MDMAC_DCCSR(n) &= ~DMAC_DCCSR_AR;			\
		REG_MDMAC_DMACR &= ~DMAC_DMACR_AR;	\
	} while (0)
#define __mdmac_channel_clear_count_terminated(n) \
  (  REG_MDMAC_DCCSR((n)) &= ~DMAC_DCCSR_CT )
#define __mdmac_channel_clear_descriptor_invalid(n) \
  (  REG_MDMAC_DCCSR((n)) &= ~DMAC_DCCSR_INV )

#define __mdmac_channel_set_transfer_unit_32bit(n)	\
do {							\
	REG_MDMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_MDMAC_DCMD((n)) |= DMAC_DCMD_DS_32BIT;	\
} while (0)

#define __mdmac_channel_set_transfer_unit_16bit(n)	\
do {							\
	REG_MDMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_MDMAC_DCMD((n)) |= DMAC_DCMD_DS_16BIT;	\
} while (0)

#define __mdmac_channel_set_transfer_unit_8bit(n)	\
do {							\
	REG_MDMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_MDMAC_DCMD((n)) |= DMAC_DCMD_DS_8BIT;	\
} while (0)

#define __mdmac_channel_set_transfer_unit_16byte(n)	\
do {							\
	REG_MDMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_MDMAC_DCMD((n)) |= DMAC_DCMD_DS_16BYTE;	\
} while (0)

#define __mdmac_channel_set_transfer_unit_32byte(n)	\
do {							\
	REG_MDMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_MDMAC_DCMD((n)) |= DMAC_DCMD_DS_32BYTE;	\
} while (0)

/* w=8,16,32 */
#define __mdmac_channel_set_dest_port_width(n,w)		\
do {							\
	REG_MDMAC_DCMD((n)) &= ~DMAC_DCMD_DWDH_MASK;	\
	REG_MDMAC_DCMD((n)) |= DMAC_DCMD_DWDH_##w;	\
} while (0)

/* w=8,16,32 */
#define __mdmac_channel_set_src_port_width(n,w)		\
do {							\
	REG_MDMAC_DCMD((n)) &= ~DMAC_DCMD_SWDH_MASK;	\
	REG_MDMAC_DCMD((n)) |= DMAC_DCMD_SWDH_##w;	\
} while (0)

/* v=0-15 */
#define __mdmac_channel_set_rdil(n,v)				\
do {								\
	REG_MDMAC_DCMD((n)) &= ~DMAC_DCMD_RDIL_MASK;		\
	REG_MDMAC_DCMD((n) |= ((v) << DMAC_DCMD_RDIL_BIT);	\
} while (0)

#define __mdmac_channel_dest_addr_fixed(n) \
	(REG_MDMAC_DCMD((n)) &= ~DMAC_DCMD_DAI)
#define __mdmac_channel_dest_addr_increment(n) \
	(REG_MDMAC_DCMD((n)) |= DMAC_DCMD_DAI)

#define __mdmac_channel_src_addr_fixed(n) \
	(REG_MDMAC_DCMD((n)) &= ~DMAC_DCMD_SAI)
#define __mdmac_channel_src_addr_increment(n) \
	(REG_MDMAC_DCMD((n)) |= DMAC_DCMD_SAI)

#define __mdmac_channel_set_doorbell(n)	\
	(REG_MDMAC_DMADBSR = (1 << (n)))

#define __mdmac_channel_irq_detected(n)  (REG_MDMAC_DMAIPR & (1 << (n)))
#define __mdmac_channel_ack_irq(n)       (REG_MDMAC_DMAIPR &= ~(1 <<(n)))

static __inline__ int __mdmac_get_irq(void)
{
	int i;
	for (i = 0; i < MAX_MDMA_NUM; i++)
		if (__mdmac_channel_irq_detected(i))
			return i;
	return -1;
}



/***************************************************************************
 * DMAC
 ***************************************************************************/

/* m is the DMA controller index (0, 1), n is the DMA channel index (0 - 11) */

#define __dmac_enable_module(m) \
	( REG_DMAC_DMACR(m) |= DMAC_DMACR_DMAE | DMAC_DMACR_PR_012345 )
#define __dmac_disable_module(m) \
	( REG_DMAC_DMACR(m) &= ~DMAC_DMACR_DMAE )

/* p=0,1,2,3 */
#define __dmac_set_priority(m,p)			\
do {							\
	REG_DMAC_DMACR(m) &= ~DMAC_DMACR_PR_MASK;	\
	REG_DMAC_DMACR(m) |= ((p) << DMAC_DMACR_PR_BIT);	\
} while (0)

#define __dmac_test_halt_error(m) ( REG_DMAC_DMACR(m) & DMAC_DMACR_HLT )
#define __dmac_test_addr_error(m) ( REG_DMAC_DMACR(m) & DMAC_DMACR_AR )

#define __dmac_enable_descriptor(n) \
  ( REG_DMAC_DCCSR((n)) &= ~DMAC_DCCSR_NDES )
#define __dmac_disable_descriptor(n) \
  ( REG_DMAC_DCCSR((n)) |= DMAC_DCCSR_NDES )

#define __dmac_enable_channel(n) \
  ( REG_DMAC_DCCSR((n)) |= DMAC_DCCSR_EN )
#define __dmac_disable_channel(n) \
  ( REG_DMAC_DCCSR((n)) &= ~DMAC_DCCSR_EN )
#define __dmac_channel_enabled(n) \
  ( REG_DMAC_DCCSR((n)) & DMAC_DCCSR_EN )

#define __dmac_channel_enable_irq(n) \
  ( REG_DMAC_DCMD((n)) |= DMAC_DCMD_TIE )
#define __dmac_channel_disable_irq(n) \
  ( REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_TIE )

#define __dmac_channel_transmit_halt_detected(n) \
  (  REG_DMAC_DCCSR((n)) & DMAC_DCCSR_HLT )
#define __dmac_channel_transmit_end_detected(n) \
  (  REG_DMAC_DCCSR((n)) & DMAC_DCCSR_TT )
#define __dmac_channel_address_error_detected(n) \
  (  REG_DMAC_DCCSR((n)) & DMAC_DCCSR_AR )
#define __dmac_channel_count_terminated_detected(n) \
  (  REG_DMAC_DCCSR((n)) & DMAC_DCCSR_CT )
#define __dmac_channel_descriptor_invalid_detected(n) \
  (  REG_DMAC_DCCSR((n)) & DMAC_DCCSR_INV )

#define __dmac_channel_clear_transmit_halt(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_HLT )
#define __dmac_channel_clear_transmit_end(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_TT )
#define __dmac_channel_clear_address_error(n) \
  (  REG_DMAC_DCCSR(n) &= ~DMAC_DCCSR_AR )
#define __dmac_channel_clear_count_terminated(n) \
  (  REG_DMAC_DCCSR((n)) &= ~DMAC_DCCSR_CT )
#define __dmac_channel_clear_descriptor_invalid(n) \
  (  REG_DMAC_DCCSR((n)) &= ~DMAC_DCCSR_INV )

#define __dmac_channel_set_transfer_unit_32bit(n)	\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_DS_32BIT;	\
} while (0)

#define __dmac_channel_set_transfer_unit_16bit(n)	\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_DS_16BIT;	\
} while (0)

#define __dmac_channel_set_transfer_unit_8bit(n)	\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_DS_8BIT;	\
} while (0)

#define __dmac_channel_set_transfer_unit_16byte(n)	\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_DS_16BYTE;	\
} while (0)

#define __dmac_channel_set_transfer_unit_32byte(n)	\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DS_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_DS_32BYTE;	\
} while (0)

/* w=8,16,32 */
#define __dmac_channel_set_dest_port_width(n,w)		\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DWDH_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_DWDH_##w;	\
} while (0)

/* w=8,16,32 */
#define __dmac_channel_set_src_port_width(n,w)		\
do {							\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_SWDH_MASK;	\
	REG_DMAC_DCMD((n)) |= DMAC_DCMD_SWDH_##w;	\
} while (0)

/* v=0-15 */
#define __dmac_channel_set_rdil(n,v)				\
do {								\
	REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_RDIL_MASK;		\
	REG_DMAC_DCMD((n) |= ((v) << DMAC_DCMD_RDIL_BIT);	\
} while (0)

#define __dmac_channel_dest_addr_fixed(n) \
  (  REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_DAI )
#define __dmac_channel_dest_addr_increment(n) \
  (  REG_DMAC_DCMD((n)) |= DMAC_DCMD_DAI )

#define __dmac_channel_src_addr_fixed(n) \
  (  REG_DMAC_DCMD((n)) &= ~DMAC_DCMD_SAI )
#define __dmac_channel_src_addr_increment(n) \
  (  REG_DMAC_DCMD((n)) |= DMAC_DCMD_SAI )

#define __dmac_channel_set_doorbell(m,n)	\
	(  REG_DMAC_DMADBSR(m) = (1 << (n)) )

#define __dmac_channel_irq_detected(m,n)  ( REG_DMAC_DMAIPR(m) & (1 << (n)) )
#define __dmac_channel_ack_irq(m,n)       ( REG_DMAC_DMAIPR(m) &= ~(1 << (n)) )

static __inline__ int __dmac_get_irq(void)
{
	int i;
	for (i = 0; i < MAX_DMA_NUM; i++)
		if (__dmac_channel_irq_detected(i/HALF_DMA_NUM, i-i/HALF_DMA_NUM*HALF_DMA_NUM))
			return i;
	return -1;
}


/***************************************************************************
 * MSC
 ***************************************************************************/

#define __msc_start_op() \
  ( REG_MSC_STRPCL = MSC_STRPCL_START_OP | MSC_STRPCL_CLOCK_CONTROL_START )

#define __msc_set_resto(to) 	( REG_MSC_RESTO = to )
#define __msc_set_rdto(to) 	( REG_MSC_RDTO = to )
#define __msc_set_cmd(cmd) 	( REG_MSC_CMD = cmd )
#define __msc_set_arg(arg) 	( REG_MSC_ARG = arg )
#define __msc_set_nob(nob) 	( REG_MSC_NOB = nob )
#define __msc_get_nob() 	( REG_MSC_NOB )
#define __msc_set_blklen(len) 	( REG_MSC_BLKLEN = len )
#define __msc_set_cmdat(cmdat) 	( REG_MSC_CMDAT = cmdat )
#define __msc_set_cmdat_ioabort() 	( REG_MSC_CMDAT |= MSC_CMDAT_IO_ABORT )
#define __msc_clear_cmdat_ioabort() 	( REG_MSC_CMDAT &= ~MSC_CMDAT_IO_ABORT )

#define __msc_set_cmdat_bus_width1() 			\
do { 							\
	REG_MSC_CMDAT &= ~MSC_CMDAT_BUS_WIDTH_MASK; 	\
	REG_MSC_CMDAT |= MSC_CMDAT_BUS_WIDTH_1BIT; 	\
} while(0)

#define __msc_set_cmdat_bus_width4() 			\
do { 							\
	REG_MSC_CMDAT &= ~MSC_CMDAT_BUS_WIDTH_MASK; 	\
	REG_MSC_CMDAT |= MSC_CMDAT_BUS_WIDTH_4BIT; 	\
} while(0)

#define __msc_set_cmdat_dma_en() ( REG_MSC_CMDAT |= MSC_CMDAT_DMA_EN )
#define __msc_set_cmdat_init() 	( REG_MSC_CMDAT |= MSC_CMDAT_INIT )
#define __msc_set_cmdat_busy() 	( REG_MSC_CMDAT |= MSC_CMDAT_BUSY )
#define __msc_set_cmdat_stream() ( REG_MSC_CMDAT |= MSC_CMDAT_STREAM_BLOCK )
#define __msc_set_cmdat_block() ( REG_MSC_CMDAT &= ~MSC_CMDAT_STREAM_BLOCK )
#define __msc_set_cmdat_read() 	( REG_MSC_CMDAT &= ~MSC_CMDAT_WRITE_READ )
#define __msc_set_cmdat_write() ( REG_MSC_CMDAT |= MSC_CMDAT_WRITE_READ )
#define __msc_set_cmdat_data_en() ( REG_MSC_CMDAT |= MSC_CMDAT_DATA_EN )

/* r is MSC_CMDAT_RESPONSE_FORMAT_Rx or MSC_CMDAT_RESPONSE_FORMAT_NONE */
#define __msc_set_cmdat_res_format(r) 				\
do { 								\
	REG_MSC_CMDAT &= ~MSC_CMDAT_RESPONSE_FORMAT_MASK; 	\
	REG_MSC_CMDAT |= (r); 					\
} while(0)

#define __msc_clear_cmdat() \
  REG_MSC_CMDAT &= ~( MSC_CMDAT_IO_ABORT | MSC_CMDAT_DMA_EN | MSC_CMDAT_INIT| \
  MSC_CMDAT_BUSY | MSC_CMDAT_STREAM_BLOCK | MSC_CMDAT_WRITE_READ | \
  MSC_CMDAT_DATA_EN | MSC_CMDAT_RESPONSE_FORMAT_MASK )

#define __msc_get_imask() 		( REG_MSC_IMASK )
#define __msc_mask_all_intrs() 		( REG_MSC_IMASK = 0xff )
#define __msc_unmask_all_intrs() 	( REG_MSC_IMASK = 0x00 )
#define __msc_mask_rd() 		( REG_MSC_IMASK |= MSC_IMASK_RXFIFO_RD_REQ )
#define __msc_unmask_rd() 		( REG_MSC_IMASK &= ~MSC_IMASK_RXFIFO_RD_REQ )
#define __msc_mask_wr() 		( REG_MSC_IMASK |= MSC_IMASK_TXFIFO_WR_REQ )
#define __msc_unmask_wr() 		( REG_MSC_IMASK &= ~MSC_IMASK_TXFIFO_WR_REQ )
#define __msc_mask_endcmdres() 		( REG_MSC_IMASK |= MSC_IMASK_END_CMD_RES )
#define __msc_unmask_endcmdres() 	( REG_MSC_IMASK &= ~MSC_IMASK_END_CMD_RES )
#define __msc_mask_datatrandone() 	( REG_MSC_IMASK |= MSC_IMASK_DATA_TRAN_DONE )
#define __msc_unmask_datatrandone() 	( REG_MSC_IMASK &= ~MSC_IMASK_DATA_TRAN_DONE )
#define __msc_mask_prgdone() 		( REG_MSC_IMASK |= MSC_IMASK_PRG_DONE )
#define __msc_unmask_prgdone() 		( REG_MSC_IMASK &= ~MSC_IMASK_PRG_DONE )

/* n=0,1,2,3,4,5,6,7 */
#define __msc_set_clkrt(n) 	\
do { 				\
	REG_MSC_CLKRT = n;	\
} while(0)

#define __msc_get_ireg() 		( REG_MSC_IREG )
#define __msc_ireg_rd() 		( REG_MSC_IREG & MSC_IREG_RXFIFO_RD_REQ )
#define __msc_ireg_wr() 		( REG_MSC_IREG & MSC_IREG_TXFIFO_WR_REQ )
#define __msc_ireg_end_cmd_res() 	( REG_MSC_IREG & MSC_IREG_END_CMD_RES )
#define __msc_ireg_data_tran_done() 	( REG_MSC_IREG & MSC_IREG_DATA_TRAN_DONE )
#define __msc_ireg_prg_done() 		( REG_MSC_IREG & MSC_IREG_PRG_DONE )
#define __msc_ireg_clear_end_cmd_res() 	( REG_MSC_IREG = MSC_IREG_END_CMD_RES )
#define __msc_ireg_clear_data_tran_done() ( REG_MSC_IREG = MSC_IREG_DATA_TRAN_DONE )
#define __msc_ireg_clear_prg_done() 	( REG_MSC_IREG = MSC_IREG_PRG_DONE )

#define __msc_get_stat() 		( REG_MSC_STAT )
#define __msc_stat_not_end_cmd_res() 	( (REG_MSC_STAT & MSC_STAT_END_CMD_RES) == 0)
#define __msc_stat_crc_err() \
  ( REG_MSC_STAT & (MSC_STAT_CRC_RES_ERR | MSC_STAT_CRC_READ_ERROR | MSC_STAT_CRC_WRITE_ERROR_YES) )
#define __msc_stat_res_crc_err() 	( REG_MSC_STAT & MSC_STAT_CRC_RES_ERR )
#define __msc_stat_rd_crc_err() 	( REG_MSC_STAT & MSC_STAT_CRC_READ_ERROR )
#define __msc_stat_wr_crc_err() 	( REG_MSC_STAT & MSC_STAT_CRC_WRITE_ERROR_YES )
#define __msc_stat_resto_err() 		( REG_MSC_STAT & MSC_STAT_TIME_OUT_RES )
#define __msc_stat_rdto_err() 		( REG_MSC_STAT & MSC_STAT_TIME_OUT_READ )

#define __msc_rd_resfifo() 		( REG_MSC_RES )
#define __msc_rd_rxfifo()  		( REG_MSC_RXFIFO )
#define __msc_wr_txfifo(v)  		( REG_MSC_TXFIFO = v )

#define __msc_reset() 						\
do { 								\
	REG_MSC_STRPCL = MSC_STRPCL_RESET;			\
 	while (REG_MSC_STAT & MSC_STAT_IS_RESETTING);		\
} while (0)

#define __msc_start_clk() 					\
do { 								\
	REG_MSC_STRPCL = MSC_STRPCL_CLOCK_CONTROL_START;	\
} while (0)

#define __msc_stop_clk() 					\
do { 								\
	REG_MSC_STRPCL = MSC_STRPCL_CLOCK_CONTROL_STOP;	\
} while (0)

#define MMC_CLK 19169200
#define SD_CLK  24576000

/* msc_clk should little than pclk and little than clk retrieve from card */
#define __msc_calc_clk_divisor(type,dev_clk,msc_clk,lv)		\
do {								\
	unsigned int rate, pclk, i;				\
	pclk = dev_clk;						\
	rate = type?SD_CLK:MMC_CLK;				\
  	if (msc_clk && msc_clk < pclk)				\
    		pclk = msc_clk;					\
	i = 0;							\
  	while (pclk < rate)					\
    	{							\
      		i ++;						\
      		rate >>= 1;					\
    	}							\
  	lv = i;							\
} while(0)

/* divide rate to little than or equal to 400kHz */
#define __msc_calc_slow_clk_divisor(type, lv)			\
do {								\
	unsigned int rate, i;					\
	rate = (type?SD_CLK:MMC_CLK)/1000/400;			\
	i = 0;							\
	while (rate > 0)					\
    	{							\
      		rate >>= 1;					\
      		i ++;						\
    	}							\
  	lv = i;							\
} while(0)



/*************************************************************************
 * NEMC
**************************************************************************/
#define __pn_enable()		(REG_NEMC_PNCR = NEMC_PNCR_PNRST | NEMC_PNCR_PNEN)
#define __pn_disable()		(REG_NEMC_PNCR = 0x0)

#define __tnand_dae_sync()	while (!(REG_NEMC_TGWE & NEMC_TGWE_DAE))
#define __tnand_dae_clr()	while (REG_NEMC_TGWE & NEMC_TGWE_DAE)
#define __tnand_wcd_sync()	while (!(REG_NEMC_TGWE & NEMC_TGWE_WCD))
#define __tnand_dphtd_sync(n)	while (!(REG_NEMC_TGPD & NEMC_TGPD_DPHTD(n)))
#define __tnand_delay_sync()	while (!(REG_NEMC_TGDR & NEMC_TGDR_DONE))

#define __tnand_fce_set(n) \
do { \
	__tnand_dphtd_sync(n); \
	REG_NEMC_NFCSR |= NEMC_NFCSR_NFCE(n) | NEMC_NFCSR_DAEC; \
} while (0)
 
#define __tnand_fce_clear(n) \
do { \
	REG_NEMC_NFCSR &= ~NEMC_NFCSR_NFCE(n); \
	__tnand_dphtd_sync(n); \
} while (0)

#define __tnand_datard_perform() \
do { \
	REG_NEMC_TGWE |= NEMC_TGWE_DAE; \
	__tnand_dae_sync(); \
} while (0)

#define __tnand_datawr_perform(n) \
do { \
	REG_NEMC_TGWE |= NEMC_TGWE_DAE | NEMC_TGWE_SDE(n); \
	__tnand_dae_sync(); \
	__tnand_wcd_sync(); \
} while (0)

#define __tnand_dqsdelay_checkerr() \
	(REG_NEMC_TGDR & NEMC_TGDR_ERR)

#define __tnand_dqsdelay_probe() \
do { \
	REG_NEMC_TGDR |= NEMC_TGDR_DET | NEMC_TGDR_AUTO; \
	__tnand_delay_sync(); \
} while (0)

/*************************************************************************
 * BCH	n = BCH_BIT
 *************************************************************************/
#define __bch_encoding(n)                                               \
do {                                                                    \
        REG_BCH_CRS = BCH_CR_BSEL(n) | BCH_CR_ENCE | BCH_CR_BCHE;       \
        REG_BCH_CRC = ~(BCH_CR_BSEL(n) | BCH_CR_ENCE | BCH_CR_BCHE);    \
        REG_BCH_CRS = BCH_CR_INIT;                                      \
} while(0)

#define __bch_decoding(n)                                               \
do {                                                                    \
        REG_BCH_CRS = BCH_CR_BSEL(n) | BCH_CR_DECE | BCH_CR_BCHE;       \
        REG_BCH_CRC = ~(BCH_CR_BSEL(n) | BCH_CR_DECE | BCH_CR_BCHE);    \
        REG_BCH_CRS = BCH_CR_INIT;                                     \
} while(0)

#define __bch_encoding_4bit() \
	(REG_BCH_CR = BCH_CR_BSEL_4 | BCH_CR_ENCE | BCH_CR_INIT | BCH_CR_BCHE)
#define __bch_decoding_4bit() \
	(REG_BCH_CR = BCH_CR_BSEL_4 | BCH_CR_DECE | BCH_CR_INIT | BCH_CR_BCHE)
#define __bch_encoding_8bit() \
	(REG_BCH_CR = BCH_CR_BSEL_8 | BCH_CR_ENCE | BCH_CR_INIT | BCH_CR_BCHE)
#define __bch_decoding_8bit() \
	(REG_BCH_CR = BCH_CR_BSEL_8 | BCH_CR_DECE | BCH_CR_INIT | BCH_CR_BCHE)
#define __bch_encoding_24bit() \
	(REG_BCH_CR = BCH_CR_BSEL_24 | BCH_CR_ENCE | BCH_CR_INIT | BCH_CR_BCHE)
#define __bch_decoding_24bit() \
	(REG_BCH_CR = BCH_CR_BSEL_24 | BCH_CR_DECE | BCH_CR_INIT | BCH_CR_BCHE)
#define __bch_encoding_40bit() \
	(REG_BCH_CR = BCH_CR_BSEL_40 | BCH_CR_ENCE | BCH_CR_INIT | BCH_CR_BCHE)
#define __bch_decoding_40bit() \
	(REG_BCH_CR = BCH_CR_BSEL_40 | BCH_CR_DECE | BCH_CR_INIT | BCH_CR_BCHE)
#define __bch_encoding_60bit() \
	(REG_BCH_CR = BCH_CR_BSEL_60 | BCH_CR_ENCE | BCH_CR_INIT | BCH_CR_BCHE)
#define __bch_decoding_60bit() \
	(REG_BCH_CR = BCH_CR_BSEL_60 | BCH_CR_DECE | BCH_CR_INIT | BCH_CR_BCHE)
#define __bch_encoding_64bit() \
	(REG_BCH_CR = BCH_CR_BSEL_64 | BCH_CR_ENCE | BCH_CR_INIT | BCH_CR_BCHE)
#define __bch_decoding_64bit() \
	(REG_BCH_CR = BCH_CR_BSEL_64 | BCH_CR_DECE | BCH_CR_INIT | BCH_CR_BCHE)

#define __bch_disable()		(REG_BCH_CRC = BCH_CR_BCHE)

#define __bch_encode_sync()	while (!(REG_BCH_INTS & BCH_INTS_ENCF))
#define __bch_decode_sync()	while (!(REG_BCH_INTS & BCH_INTS_DECF))
#define __bch_decode_sdmf()	while (!(REG_BCH_INTS & BCH_INTS_SDMF))

#define __bch_encints_clean()	(REG_BCH_INTS |= BCH_INTS_ENCF)
#define __bch_decints_clean()	(REG_BCH_INTS |= BCH_INTS_DECF)

/* blk = ECC_BLOCK_SIZE, par = ECC_PARITY_SIZE */
#define __bch_cnt_set(blk, par) \
do { \
	REG_BCH_CNT &= ~(BCH_CNT_PARITY_MASK | BCH_CNT_BLOCK_MASK); \
	REG_BCH_CNT |= ((par) << BCH_CNT_PARITY_BIT | (blk) << BCH_CNT_BLOCK_BIT); \
} while(0)

#endif /* !__ASSEMBLY__ */

#endif /* __JZ4780_H__ */
