/*
 * JZSOC CPM register definition.
 *
 * CPM (Clock reset and Power control Management)
 *
 * Copyright (C) 2010 Ingenic Semiconductor Co., Ltd.
 */

#ifndef __JZSOC_CPM_H__
#define __JZSOC_CPM_H__

#define CPM_CPCCR	(0x00)
#define CPM_CPCSR	(0xd4)

#define CPM_DDRCDR	(0x2c)
#define CPM_VPUCDR	(0x30)
#define CPM_I2SCDR	(0x60)
#define CPM_LPCDR	(0x64)
#define CPM_MSC0CDR	(0x68)
#define CPM_MSC1CDR	(0xa4)
#define CPM_MSC2CDR	(0xa8)
#define CPM_USBCDR	(0x50)
#define CPM_UHCCDR	(0x6c)
#define CPM_SSICDR	(0x74)
#define CPM_CIMCDR	(0x7c)
#define CPM_CIM1CDR	(0x80)
#define CPM_PCMCDR	(0x84)
#define CPM_BCHCDR	(0xac)

#define CPM_MPHYCR	(0xe0)

#define CPM_CPAPCR	(0x10)
#define CPM_CPMPCR	(0x14)

#define CPM_INTR	(0xb0)
#define CPM_INTRE	(0xb4)
#define CPM_CPSPPR	(0x38)
#define CPM_CPPSR	(0x34)

#define CPM_USBPCR	(0x3c)
#define CPM_USBRDT	(0x40)
#define CPM_USBVBFIL	(0x44)
#define CPM_USBPCR1	(0x48)

#define CPM_LCR		(0x04)
#define CPM_PSWC0ST     (0x90)
#define CPM_PSWC1ST     (0x94)
#define CPM_PSWC2ST     (0x98)
#define CPM_PSWC3ST     (0x9c)
#define CPM_CLKGR	(0x20)
#define CPM_SRBC	(0xc4)
#define CPM_SLBC	(0xc8)
#define CPM_SLPC	(0xcc)
#define CPM_OPCR	(0x24)

#define CPM_RSR		(0x08)

#define LCR_LPM_MASK		(0x3)
#define LCR_LPM_SLEEP		(0x1)

#define CPM_LCR_PD_X2D		(0x1<<31)
#define CPM_LCR_PD_VPU		(0x1<<30)
#define CPM_LCR_PD_IPU		(0x1<<29)
#define CPM_LCR_PD_EPD		(0x1<<28)
#define CPM_LCR_PD_MASK		(0x7<<28)
#define CPM_LCR_X2DS 		(0x1<<27)
#define CPM_LCR_VPUS		(0x1<<26)
#define CPM_LCR_IPUS		(0x1<<25)
#define CPM_LCR_EPDS 		(0x1<<24)
#define CPM_LCR_STATUS_MASK 	(0xf<<24)

#define OPCR_ERCS		(0x1<<2)
#define OPCR_PD			(0x1<<3)
#define OPCR_IDLE		(0x1<<31)

#define CLKGR_VPU              (0x1<<19)

#define cpm_inl(off)		*(volatile unsigned int *)(CPM_IOBASE + (off))
#define cpm_outl(val,off)	*((volatile unsigned int *)(CPM_IOBASE + (off))) = (val)
#define cpm_test_bit(bit,off)	(cpm_inl(off) & 0x1<<(bit))
#define cpm_set_bit(bit,off)	(cpm_outl((cpm_inl(off) | 0x1<<(bit)),off))
#define cpm_clear_bit(bit,off)	(cpm_outl(cpm_inl(off) & ~(0x1 << bit), off))

#define arch_suspend_usb()	cpm_clear_bit(7,CPM_OPCR)
#define arch_enable_usb()	cpm_set_bit(7,CPM_OPCR)

#define arch_select_utmi8bit()		\
do{					\
	cpm_clear_bit(18,CPM_USBPCR1);	\
	cpm_clear_bit(19,CPM_USBPCR1);	\
}while(0)

#define arch_select_utmi16bit()		\
do{					\
	cpm_set_bit(18,CPM_USBPCR1);	\
	cpm_set_bit(19,CPM_USBPCR1);	\
}while(0)

#define arch_otg_init() 		\
do{					\
	cpm_set_bit(28,CPM_USBPCR1);	\
	cpm_clear_bit(31,CPM_USBPCR);	\
	cpm_set_bit(24,CPM_USBPCR);	\
	cpm_set_bit(22,CPM_USBPCR);	\
	udelay(30);			\
	cpm_clear_bit(22,CPM_USBPCR);	\
	udelay(300);			\
	arch_enable_usb();		\
}while(0)

#endif
