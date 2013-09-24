/*
 * Include file for Ingenic Semiconductor's GPIO CPU.
 */
#ifndef __GPIO_H__
#define __GPIO_H__

enum gpio_function {
	GPIO_FUNC_0     = 0x00,  //0000, GPIO as function 0 / device 0
	GPIO_FUNC_1     = 0x01,  //0001, GPIO as function 1 / device 1
	GPIO_FUNC_2     = 0x02,  //0010, GPIO as function 2 / device 2
	GPIO_FUNC_3     = 0x03,  //0011, GPIO as function 3 / device 3
	GPIO_INPUT	= 0x06,	 //0110, GPIO as input 
};

enum gpio_port {
	GPIO_PORT_A, GPIO_PORT_B,
	GPIO_PORT_C, GPIO_PORT_D,
	GPIO_PORT_E, GPIO_PORT_F,
	GPIO_PORT_G,
	/* this must be last */
	GPIO_NR_PORTS,
};

struct jz_gpio_func_def {
	int port;
	int func;
	unsigned long pins;
};

#ifndef GPIO_PG
#define GPIO_PG(n)      (5*32 + 23 + n)
#endif

/*************************************************************************
 * GPIO (General-Purpose I/O Ports)
 *************************************************************************/
#define MAX_GPIO_NUM	192

//n = 0,1,2,3,4,5
#define GPIO_PXPIN(n)	(GPIO_IOBASE+ (0x00 + (n)*0x100)) /* PIN Level Register */
#define GPIO_PXINT(n)	(GPIO_IOBASE + (0x10 + (n)*0x100)) /* Port Interrupt Register */
#define GPIO_PXINTS(n)	(GPIO_IOBASE + (0x14 + (n)*0x100)) /* Port Interrupt Set Register */
#define GPIO_PXINTC(n)	(GPIO_IOBASE + (0x18 + (n)*0x100)) /* Port Interrupt Clear Register */
#define GPIO_PXMASK(n)	(GPIO_IOBASE + (0x20 + (n)*0x100)) /* Port Interrupt Mask Register */
#define GPIO_PXMASKS(n)	(GPIO_IOBASE + (0x24 + (n)*0x100)) /* Port Interrupt Mask Set Reg */
#define GPIO_PXMASKC(n)	(GPIO_IOBASE + (0x28 + (n)*0x100)) /* Port Interrupt Mask Clear Reg */
#define GPIO_PXPAT1(n)	(GPIO_IOBASE + (0x30 + (n)*0x100)) /* Port Pattern 1 Register */
#define GPIO_PXPAT1S(n)	(GPIO_IOBASE + (0x34 + (n)*0x100)) /* Port Pattern 1 Set Reg. */
#define GPIO_PXPAT1C(n)	(GPIO_IOBASE + (0x38 + (n)*0x100)) /* Port Pattern 1 Clear Reg. */
#define GPIO_PXPAT0(n)	(GPIO_IOBASE + (0x40 + (n)*0x100)) /* Port Pattern 0 Register */
#define GPIO_PXPAT0S(n)	(GPIO_IOBASE + (0x44 + (n)*0x100)) /* Port Pattern 0 Set Register */
#define GPIO_PXPAT0C(n)	(GPIO_IOBASE + (0x48 + (n)*0x100)) /* Port Pattern 0 Clear Register */
#define GPIO_PXFLG(n)	(GPIO_IOBASE + (0x50 + (n)*0x100)) /* Port Flag Register */
#define GPIO_PXFLGC(n)	(GPIO_IOBASE + (0x54 + (n)*0x100)) /* Port Flag clear Register */
#define GPIO_PXOEN(n)	(GPIO_IOBASE + (0x60 + (n)*0x100)) /* Port Output Disable Register */
#define GPIO_PXOENS(n)	(GPIO_IOBASE + (0x64 + (n)*0x100)) /* Port Output Disable Set Register */
#define GPIO_PXOENC(n)	(GPIO_IOBASE + (0x68 + (n)*0x100)) /* Port Output Disable Clear Register */
#define GPIO_PXPEN(n)	(GPIO_IOBASE + (0x70 + (n)*0x100)) /* Port Pull Disable Register */
#define GPIO_PXPENS(n)	(GPIO_IOBASE + (0x74 + (n)*0x100)) /* Port Pull Disable Set Register */
#define GPIO_PXPENC(n)	(GPIO_IOBASE + (0x78 + (n)*0x100)) /* Port Pull Disable Clear Register */
#define GPIO_PXDS(n)	(GPIO_IOBASE + (0x80 + (n)*0x100)) /* Port Drive Strength Register */
#define GPIO_PXDSS(n)	(GPIO_IOBASE + (0x84 + (n)*0x100)) /* Port Drive Strength set Register */
#define GPIO_PXDSC(n)	(GPIO_IOBASE + (0x88 + (n)*0x100)) /* Port Drive Strength clear Register */

#define REG_GPIO_PXPIN(n)	*(volatile unsigned int *)(GPIO_PXPIN((n)))  /* PIN level */
#define REG_GPIO_PXINT(n)	*(volatile unsigned int *)(GPIO_PXINT((n)))  /* 1: interrupt pending */
#define REG_GPIO_PXINTS(n)	*(volatile unsigned int *)(GPIO_PXINTS((n)))
#define REG_GPIO_PXINTC(n)	*(volatile unsigned int *)(GPIO_PXINTC((n)))
#define REG_GPIO_PXMASK(n)	*(volatile unsigned int *)(GPIO_PXMASK((n)))   /* 1: mask pin interrupt */
#define REG_GPIO_PXMASKS(n)	*(volatile unsigned int *)(GPIO_PXMASKS((n)))
#define REG_GPIO_PXMASKC(n)	*(volatile unsigned int *)(GPIO_PXMASKC((n)))
#define REG_GPIO_PXPAT1(n)	*(volatile unsigned int *)(GPIO_PXPAT1((n)))   /* 1: disable pull up/down */
#define REG_GPIO_PXPAT1S(n)	*(volatile unsigned int *)(GPIO_PXPAT1S((n)))
#define REG_GPIO_PXPAT1C(n)	*(volatile unsigned int *)(GPIO_PXPAT1C((n)))
#define REG_GPIO_PXPAT0(n)	*(volatile unsigned int *)(GPIO_PXPAT0((n)))  /* 0:GPIO/INTR, 1:FUNC */
#define REG_GPIO_PXPAT0S(n)	*(volatile unsigned int *)(GPIO_PXPAT0S((n)))
#define REG_GPIO_PXPAT0C(n)	*(volatile unsigned int *)(GPIO_PXPAT0C((n)))
#define REG_GPIO_PXFLG(n)	*(volatile unsigned int *)(GPIO_PXFLG((n))) /* 0:GPIO/Fun0,1:intr/fun1*/
#define REG_GPIO_PXFLGC(n)	*(volatile unsigned int *)(GPIO_PXFLGC((n)))
#define REG_GPIO_PXOEN(n)	*(volatile unsigned int *)(GPIO_PXOEN((n)))
#define REG_GPIO_PXOENS(n)	*(volatile unsigned int *)(GPIO_PXOENS((n))) /* 0:input/low-level-trig/falling-edge-trig, 1:output/high-level-trig/rising-edge-trig */
#define REG_GPIO_PXOENC(n)	*(volatile unsigned int *)(GPIO_PXOENC((n)))
#define REG_GPIO_PXPEN(n)	*(volatile unsigned int *)(GPIO_PXPEN((n)))
#define REG_GPIO_PXPENS(n)	*(volatile unsigned int *)(GPIO_PXPENS((n))) /* 0:Level-trigger/Fun0, 1:Edge-trigger/Fun1 */
#define REG_GPIO_PXPENC(n)	*(volatile unsigned int *)(GPIO_PXPENC((n)))
#define REG_GPIO_PXDS(n)	*(volatile unsigned int *)(GPIO_PXDS((n)))
#define REG_GPIO_PXDSS(n)	*(volatile unsigned int *)(GPIO_PXDSS((n))) /* interrupt flag */
#define REG_GPIO_PXDSC(n)	*(volatile unsigned int *)(GPIO_PXDSC((n))) /* interrupt flag */

#endif /* __GPIO_H__ */
