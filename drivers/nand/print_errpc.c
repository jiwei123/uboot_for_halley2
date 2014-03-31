/*
 * Copyright (C) 2007 Ingenic Semiconductor Inc.
 * Author: Regen Huang <lhhuang@ingenic.cn>
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


//#include <ingenic_nand_mgr/nandflash.h>
#include <mipsregs.h>
#include <ingenic_nand_mgr/jz4775.h>


#define __raw_readb(addr) (*(volatile unsigned char *)(addr))
#define __raw_readw(addr) (*(volatile unsigned short *)(addr))
#define __raw_readl(addr) (*(volatile unsigned int *)(addr))
#define readb(addr) __raw_readb((addr))
#define readw(addr) __ioswab16(__raw_readw((addr)))
#define readl(addr) __ioswab32(__raw_readl((addr)))

#define __raw_writeb(b, addr) (*(volatile unsigned char *)(addr)) = (b)
#define __raw_writew(b, addr) (*(volatile unsigned short *)(addr)) = (b)
#define __raw_writel(b, addr) (*(volatile unsigned int *)(addr)) = (b)
#define writeb(b, addr) __raw_writeb((b), (addr))
#define writew(b, addr) __raw_writew(__ioswab16(b), (addr))
#define writel(b, addr) __raw_writel(__ioswab32(b), (addr))

extern void flush_cache_all(void);

int my_print_errpc()
{
	unsigned int errpc;

	__asm__ __volatile__(
			"mfc0 %0,$30,0 \n\t"
			"nop"
			: "=r" (errpc)
			:
			);
	printf("^^^^^^^^^^^^^^^^^^^^^^^ ERR PC 0x%x \n",errpc);
}
typedef unsigned char uint8_t ;

#define UART_BASE (0xB0030000 + 3 * 0x1000)
#define OFF_LSR     (0x14)  /* R  8b H'00 */
#define OFF_TDR     (0x00)  /* W  8b H'xx */
#define UART_LSR_TDRQ   (1 << 5)    /* 1: transmit FIFO half "empty" */
#define UART_LSR_TEMT   (1 << 6)    /* 1: transmit FIFO and shift registers empty */

static  void jz_serial_putc(const char c)
{
	volatile uint8_t * uart_lsr = (volatile uint8_t *)(UART_BASE + OFF_LSR);
	volatile uint8_t * uart_tdr = (volatile uint8_t *)(UART_BASE + OFF_TDR);

	if(c == '\n') {
		    while (!((readb(uart_lsr) & (UART_LSR_TDRQ | UART_LSR_TEMT)) == 0x60))
				        ;
			  writeb((uint8_t) '\r', uart_tdr);
	}


	/* Wait for fifo to shift out some bytes */
	while (!((readb(uart_lsr) & (UART_LSR_TDRQ | UART_LSR_TEMT)) == 0x60))
		;

	writeb((uint8_t) c, uart_tdr);
}

static inline void jz_serial_puts (const char *s)
{
	while (*s) {
		jz_serial_putc (*s++);
	}
}

static inline void jz_serial_put_hex(unsigned int d)
{
	uint8_t c[12];
	uint8_t i;

	for(i = 0; i < 8; i++) {
		c[i] = (d >> ((7 - i) * 4)) & 0xf;
		if(c[i] < 10)
			c[i] += 0x30;
		else
			c[i] += (0x41 - 10);
	}
	c[8] = '\n';
	c[9] = 0;
	jz_serial_puts((const char *)c);

}


void __attribute__ ( (noreturn) ) my_print_eepc(unsigned int a,unsigned int b,unsigned int c) {
	int i;
	for(i = 0; i<16;i++) {
		jz_serial_put_hex(*(unsigned int *)((b | 0xa0000000) - 8*4  + i*4));
	}

	jz_serial_put_hex(a);
	jz_serial_put_hex(b);
	jz_serial_put_hex(c);

	while(1);
}
static unsigned int *g_errpc = 0x80000004;
void my_print_epc(){
#if 0
	__asm__ __volatile__(
			"mfc0 %0,$14,0 \n\t"
			"nop \n\t"
			"addu %0,%0,4 \n\t"
			"mtc0 %0,$14,0 \n\t"
			"nop \n\t"
			"nop \n\t"
			"nop \n\t"
			"eret \n\t"
			: "=r" (errpc)
			:
			);
#endif
	unsigned int errpc = *(unsigned int *)(0x80000004);
	__asm__ __volatile__(
			"mfc0 $5,$14,0 \n\t"
			"move $6,$29 \n\t"
			"move $4,$25 \n\t"
			"jr %0 \n\t"
			"nop \n\t"
			"aaa:  b aaa \n\t"
			"nop"
			:: "r" (errpc)
			);

     
}
void test_print_errpc()
{
	int tmp;

	*g_errpc = (unsigned int)my_print_eepc;
    //jz_serial_put_hex(0x12345678);
	//while(1);
#if 1
	//my_print_errpc();
	memcpy((unsigned int *)0x80000180,my_print_epc,16*4);
	memcpy((unsigned int *)0x80000200,my_print_epc,16*4);
	flush_cache_all();
    write_c0_cause(0x0 | 0x1<<23);
    write_c0_status(0x1000ff01);

#if 0
	tmp = (unsigned int )memcpy & (~7);
	write_c0_watchhi0((1 << 30));
	write_c0_watchlo0((tmp | 1));
	//printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>1 \n");
	//*(volatile unsigned int *)0 = 19999;
	printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>2 \n");
	*(volatile unsigned int *)memcpy = 0x10101010;

	while(1);
#endif

#endif
}

