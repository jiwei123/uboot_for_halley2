/*
 * chip-rtc.h
 * JZ4760 RTC register definition
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 *
 * Author: cjwang@ingenic.cn
 */

#ifndef __CHIP_RTC_H__
#define __CHIP_RTC_H__

#define BIT0            (1 << 0)
#define BIT1            (1 << 1)
#define BIT2            (1 << 2)
#define BIT3            (1 << 3)
#define BIT4            (1 << 4)
#define BIT5            (1 << 5)
#define BIT6            (1 << 6)
#define BIT7            (1 << 7)
#define BIT8            (1 << 8)
#define BIT9            (1 << 9)
#define BIT10           (1 << 10)
#define BIT11           (1 << 11)
#define BIT12 	        (1 << 12)
#define BIT13 	        (1 << 13)
#define BIT14 	        (1 << 14)
#define BIT15 	        (1 << 15)
#define BIT16 	        (1 << 16)
#define BIT17 	        (1 << 17)
#define BIT18 	        (1 << 18)
#define BIT19 	        (1 << 19)
#define BIT20 	        (1 << 20)
#define BIT21 	        (1 << 21)
#define BIT22 	        (1 << 22)
#define BIT23 	        (1 << 23)
#define BIT24 	        (1 << 24)
#define BIT25 	        (1 << 25)
#define BIT26 	        (1 << 26)
#define BIT27 	        (1 << 27)
#define BIT28 	        (1 << 28)
#define BIT29 	        (1 << 29)
#define BIT30 	        (1 << 30)
#define BIT31 	        (1 << 31)
#define BITS_H2L(msb, lsb)  ((0xFFFFFFFF >> (32-((msb)-(lsb)+1))) << (lsb))

#define INREG32(x)              ((unsigned int)(*(volatile unsigned int *)(x)))
#define OUTREG32(x, y)          *(volatile unsigned int *)(x) = (y)

/*
 * Real time clock module(RTC) address definition
 */
#define	RTC_BASE	0xb0003000


/*
 * RTC registers offset address definition
 */
#define RTC_RTCCR_OFFSET	(0x00)	/* rw, 32, 0x00000081 */
#define RTC_RTCSR_OFFSET	(0x04)	/* rw, 32, 0x???????? */
#define RTC_RTCSAR_OFFSET	(0x08)	/* rw, 32, 0x???????? */
#define RTC_RTCGR_OFFSET	(0x0c)	/* rw, 32, 0x0??????? */

#define RTC_HCR_OFFSET		(0x20)  /* rw, 32, 0x00000000 */
#define RTC_HWFCR_OFFSET	(0x24)  /* rw, 32, 0x0000???0 */
#define RTC_HRCR_OFFSET		(0x28)  /* rw, 32, 0x00000??0 */
#define RTC_HWCR_OFFSET		(0x2c)  /* rw, 32, 0x00000008 */
#define RTC_HWRSR_OFFSET	(0x30)  /* rw, 32, 0x00000000 */
#define RTC_HSPR_OFFSET		(0x34)  /* rw, 32, 0x???????? */
#define RTC_WENR_OFFSET		(0x3c)  /* rw, 32, 0x00000000 */


/*
 * RTC registers address definition
 */
#define RTC_RTCCR	(RTC_BASE + RTC_RTCCR_OFFSET)
#define RTC_RTCSR	(RTC_BASE + RTC_RTCSR_OFFSET)
#define RTC_RTCSAR	(RTC_BASE + RTC_RTCSAR_OFFSET)
#define RTC_RTCGR	(RTC_BASE + RTC_RTCGR_OFFSET)

#define RTC_HCR		(RTC_BASE + RTC_HCR_OFFSET)
#define RTC_HWFCR	(RTC_BASE + RTC_HWFCR_OFFSET)
#define RTC_HRCR	(RTC_BASE + RTC_HRCR_OFFSET)
#define RTC_HWCR	(RTC_BASE + RTC_HWCR_OFFSET)
#define RTC_HWRSR	(RTC_BASE + RTC_HWRSR_OFFSET)
#define RTC_HSPR	(RTC_BASE + RTC_HSPR_OFFSET)
#define RTC_WENR	(RTC_BASE + RTC_WENR_OFFSET)


/*
 * RTC registers common define
 */

/* RTC control register(RTCCR) */
#define RTCCR_WRDY		BIT7
#define RTCCR_1HZ		BIT6
#define RTCCR_1HZIE		BIT5
#define RTCCR_AF		BIT4
#define RTCCR_AIE		BIT3
#define RTCCR_AE		BIT2
#define RTCCR_SELEXC		BIT1
#define RTCCR_RTCE		BIT0

/* RTC regulator register(RTCGR) */
#define RTCGR_LOCK		BIT31

#define RTCGR_ADJC_LSB		16
#define RTCGR_ADJC_MASK		BITS_H2L(25, RTCGR_ADJC_LSB)

#define RTCGR_NC1HZ_LSB		0
#define RTCGR_NC1HZ_MASK	BITS_H2L(15, RTCGR_NC1HZ_LSB)

/* Hibernate control register(HCR) */
#define HCR_PD			BIT0

/* Hibernate wakeup filter counter register(HWFCR) */
#define HWFCR_LSB		5
#define HWFCR_MASK		BITS_H2L(15, HWFCR_LSB)
#define HWFCR_WAIT_TIME(ms)	(((ms) << HWFCR_LSB) > HWFCR_MASK ? HWFCR_MASK : ((ms) << HWFCR_LSB))

/* Hibernate reset counter register(HRCR) */
#define HRCR_LSB		5
#define HRCR_MASK		BITS_H2L(11, HRCR_LSB)
#define HRCR_WAIT_TIME(ms)     (((ms) << HRCR_LSB) > HRCR_MASK ? HRCR_MASK : ((ms) << HRCR_LSB))

/* Hibernate wakeup control register(HWCR) */
#define HWCR_EPDET		BIT3
#define HWCR_WKUPVL		BIT2
#define HWCR_EALM		BIT0

/* Hibernate wakeup status register(HWRSR) */
#define HWRSR_APD		BIT8
#define HWRSR_HR		BIT5
#define HWRSR_PPR		BIT4
#define HWRSR_PIN		BIT1
#define HWRSR_ALM		BIT0

/* write enable pattern register(WENR) */
#define WENR_WEN		BIT31

#define WENR_WENPAT_LSB		0
#define WENR_WENPAT_MASK	BITS_H2L(15, WENR_WENPAT_LSB)
#define WENR_WENPAT_WRITABLE	(0xa55a)

/* Hibernate scratch pattern register(HSPR) */
#define HSPR_RTCV               0x52544356      /* The value is 'RTCV', means rtc is valid */ 


#ifndef __MIPS_ASSEMBLER

/* Waiting for the RTC register writing finish */
#define __wait_write_ready()						\
do {									\
	int timeout = 0x1000;						\
	while (!(rtc_read_reg(RTC_RTCCR) & RTCCR_WRDY) && timeout--);	\
}while(0);

/* Waiting for the RTC register writable */
#define __wait_writable()						\
do {									\
	int timeout = 0x1000;						\
	__wait_write_ready();						\
	OUTREG32(RTC_WENR, WENR_WENPAT_WRITABLE);			\
	__wait_write_ready();						\
	while (!(rtc_read_reg(RTC_WENR) & WENR_WEN) && timeout--);	\
}while(0);

/* Basic RTC ops */
#define rtc_read_reg(reg)				\
({							\
	unsigned int data, timeout = 0x10000;		\
	do {						\
		data = INREG32(reg);			\
	} while (INREG32(reg) != data && timeout--);	\
	data;						\
})

#define rtc_write_reg(reg, data)			\
do {							\
	__wait_writable();				\
	OUTREG32(reg, data);				\
	__wait_write_ready();				\
}while(0);

#define rtc_set_reg(reg, data)	rtc_write_reg(reg, rtc_read_reg(reg) | (data))
#define rtc_clr_reg(reg, data)	rtc_write_reg(reg, rtc_read_reg(reg) & ~(data))

typedef volatile struct
{
	unsigned int RTCCR;
	unsigned int RTCSR;
	unsigned int RTCSAR;
	unsigned int RTCGR;

	unsigned int RTCRSV[(RTC_HCR_OFFSET - RTC_RTCGR_OFFSET)/4];

	unsigned int HCR;
	unsigned int HWFCR;
	unsigned int HRCR;
	unsigned int HWCR;
	unsigned int HWRSR;
	unsigned int HSPR;
	unsigned int WENR;
} JZ4760_RTC, *PJZ4760_RTC;

#define OPCR_ERCS               BIT2

#endif /* __MIPS_ASSEMBLER */

#endif /* __CHIP_RTC_H__ */
