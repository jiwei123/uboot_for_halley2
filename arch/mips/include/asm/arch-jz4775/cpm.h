/*
 * JZ4775 cpm definitions
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CPM_H__
#define __CPM_H__

#include <asm/arch/base.h>

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

#define CPM_CLKGR_DDR		(1 << 31)
#define CPM_CLKGR_EPDE		(1 << 27)
#define CPM_CLKGR_EPDC		(1 << 26)
#define CPM_CLKGR_LCD		(1 << 25)
#define CPM_CLKGR_CIM1		(1 << 24)
#define CPM_CLKGR_CIM0		(1 << 23)
#define CPM_CLKGR_UHC		(1 << 22)
#define CPM_CLKGR_GMAC		(1 << 21)
#define CPM_CLKGR_PDMA		(1 << 20)
#define CPM_CLKGR_VPU		(1 << 19)
#define CPM_CLKGR_UART3		(1 << 18)
#define CPM_CLKGR_UART2		(1 << 17)
#define CPM_CLKGR_UART1		(1 << 16)
#define CPM_CLKGR_UART0		(1 << 15)
#define CPM_CLKGR_SADC		(1 << 14)
#define CPM_CLKGR_PCM		(1 << 13)
#define CPM_CLKGR_MSC2		(1 << 12)
#define CPM_CLKGR_MSC1		(1 << 11)
#define CPM_CLKGR_AHB_MON	(1 << 10)
#define CPM_CLKGR_X2D		(1 << 9)
#define CPM_CLKGR_AIC		(1 << 8)
#define CPM_CLKGR_I2C2		(1 << 7)
#define CPM_CLKGR_I2C1		(1 << 6)
#define CPM_CLKGR_I2C0		(1 << 5)
#define CPM_CLKGR_SSI0		(1 << 4)
#define CPM_CLKGR_MSC0		(1 << 3)
#define CPM_CLKGR_OTG		(1 << 2)
#define CPM_CLKGR_BCH		(1 << 1)
#define CPM_CLKGR_NEMC		(1 << 0)

#define OPCR_ERCS		(0x1<<2)
#define OPCR_PD			(0x1<<3)
#define OPCR_IDLE		(0x1<<31)

#define CLKGR_VPU              (0x1<<19)

#define cpm_inl(off)		readl(CPM_BASE + (off))
#define cpm_outl(val,off)	writel(val, CPM_BASE + (off))
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

#endif /* __CPM_H__ */
