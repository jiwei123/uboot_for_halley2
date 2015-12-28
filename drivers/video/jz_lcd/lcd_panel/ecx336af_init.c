
#include <common.h>
#include <config.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <serial.h>
#include <jz_lcd/ecx336af.h>

#define DEBUG_WRITE  0
#define DEBUG_READ   0
#define DEBUG_REG    0

extern struct ecx336af_data ecx336af_pdata;

// Control
#define RESET(n)     gpio_direction_output(ecx336af_pdata.gpio_reset, n)

// SPI
#define CS(n)     gpio_direction_output(ecx336af_pdata.gpio_spi_cs, n)
#define SCK(n)    gpio_direction_output(ecx336af_pdata.gpio_spi_clk, n)
#define SDO(n)    gpio_direction_output(ecx336af_pdata.gpio_spi_sdi, n)
#define SDI()     gpio_get_value(ecx336af_pdata.gpio_spi_sdo)

// SPI clock rate : 500KHz
#define SPI_DELAY  udelay(1)

/* 
 * SPI Write Func, LSB first
 */
void SPI_WRITE_REG(unsigned char addr, unsigned char *value, int len)
{
	int i = 0;
	int byte = 0;
	unsigned char temp = 0;

#if DEBUG_WRITE
	{
		int i = 0;
		printf("Write %02x reg value is", addr);
		for (i = 0; i < len; i++)
			printf(" %02x", value[i]);
		printf("\n");
	}
#endif

	CS(0);
	SCK(0);
	SDO(0);
	SPI_DELAY;
	// Addr
	temp = addr;
	for(i = 0; i < 8; i++) {
		SCK(0);
		SDO((temp & 0x1));
		temp = temp >> 1;
		SPI_DELAY;
		SCK(1);
		SPI_DELAY;
	}
	// Value
	while (byte < len) {
		temp = value[byte++];
		for(i = 0; i < 8; i++) {
			SCK(0);
			SDO((temp & 0x1));
			temp = temp >> 1;
			SPI_DELAY;
			SCK(1);
			SPI_DELAY;
		}
	}
	SPI_DELAY;
	CS(1);
	SCK(1);
	SDO(1);
	SPI_DELAY;
	SPI_DELAY;
}

void SPI_GET_VALUE(unsigned char *value, int len)
{
	int i = 0;
	int byte = 0;
	unsigned char temp = 0;

	CS(0);
	SCK(0);
	SDO(0);
	SPI_DELAY;
	// Addr
	temp = 0x81;
	for(i = 0; i < 8; i++) {
		SCK(0);
		SDO((temp & 0x1));
		temp = temp >> 1;
		SPI_DELAY;
		SCK(1);
		SPI_DELAY;
	}
	// Value
	while (byte < len) {
		temp = 0;
		for(i = 0; i < 8; i++) {
			SCK(0);
			temp = temp >> 1;
			temp |= (SDI() << 7);
			SPI_DELAY;
			SCK(1);
			SPI_DELAY;
		}
		value[byte++] = temp;
	}
	SPI_DELAY;
	CS(1);
	SCK(1);
	SDO(1);
	SPI_DELAY;
	SPI_DELAY;
}
 
/* 
 * SPI Read Func, need Open RD_ON in reg 0x80
 */
void SPI_READ_REG(unsigned char addr, unsigned char *value, int len)
{
	unsigned char RD_ON;
	RD_ON = 0x01;
	SPI_WRITE_REG(0x80, &RD_ON, 1);
	SPI_WRITE_REG(0x81, &addr, 1);
	SPI_GET_VALUE(value, len);
#if DEBUG_READ
	{
		int i = 0;
		printf("Read %02x reg value is", addr);
		for (i = 0; i < len; i++)
			printf(" %02x", value[i]);
		printf("\n");
	}
#endif
}

void ecx336af_write_reg(unsigned char addr, unsigned char value)
{
	SPI_WRITE_REG(addr, &value, 1);
}

unsigned char ecx336af_read_reg(unsigned char addr)
{
	unsigned char value = 0;
	SPI_READ_REG(addr, &value, 1);
	return value;
}

/* 
 * Power Func, defined Power On/Off sequence
 */
void ECX336AF_PS_ON(void)
{
	unsigned char ps0;
	SPI_READ_REG(0x00, &ps0, 1);
	ps0 |= 0x1;
	SPI_WRITE_REG(0x00, &ps0, 1);
	udelay(30000);
}

void ECX336AF_PS_OFF(void)
{
	unsigned char ps0;
	SPI_READ_REG(0x00, &ps0, 1);
	ps0 &= (~0x1);
	SPI_WRITE_REG(0x00, &ps0, 1);
	udelay(30000);
}

void ECX336AF_POWER_ON(void)
{
	if (ecx336af_pdata.power_1v8)
		regulator_disable(ecx336af_pdata.power_1v8);
	RESET(0);
	mdelay(5);
	if (ecx336af_pdata.power_1v8) {
		regulator_set_voltage(ecx336af_pdata.power_1v8, 1800000, 1800000);
		regulator_enable(ecx336af_pdata.power_1v8);
	}
	mdelay(5);
	RESET(1);
	mdelay(40);

	ECX336AF_PS_ON();
}

void ECX336AF_POWER_OFF(void)
{
	ECX336AF_PS_OFF();
	udelay(10000);
	RESET(0);
	udelay(2000);
	if (ecx336af_pdata.power_1v8)
		regulator_disable(ecx336af_pdata.power_1v8);
}

void Initial_IC(void)
{
        ecx336af_write_reg(0x00, 0x0e);//03 0e
	ecx336af_write_reg(0x01, 0x00);
	ecx336af_write_reg(0x02, 0x00);
	ecx336af_write_reg(0x03, 0x00);
	ecx336af_write_reg(0x04, 0x3F);
	ecx336af_write_reg(0x05, 0xC0);
	ecx336af_write_reg(0x06, 0x00);
	ecx336af_write_reg(0x07, 0x40);
	ecx336af_write_reg(0x08, 0x0F);
	ecx336af_write_reg(0x09, 0x00);
	ecx336af_write_reg(0x0A, 0x10);
	ecx336af_write_reg(0x0B, 0x00);
	ecx336af_write_reg(0x0C, 0x00);
	ecx336af_write_reg(0x0D, 0x00);
	ecx336af_write_reg(0x0E, 0x00);
	ecx336af_write_reg(0x0F, 0x56);

	ecx336af_write_reg(0x10, 0x00);
	ecx336af_write_reg(0x11, 0x00);
	ecx336af_write_reg(0x12, 0x00);
	ecx336af_write_reg(0x13, 0x00);
	ecx336af_write_reg(0x14, 0x00);
	ecx336af_write_reg(0x15, 0x00);
	ecx336af_write_reg(0x16, 0x00);
	ecx336af_write_reg(0x17, 0x00);
	ecx336af_write_reg(0x18, 0x00);
	ecx336af_write_reg(0x19, 0x1F);
	ecx336af_write_reg(0x1A, 0x00);
	ecx336af_write_reg(0x1B, 0x00);
	ecx336af_write_reg(0x1C, 0x00);
	ecx336af_write_reg(0x1D, 0x00);
	ecx336af_write_reg(0x1E, 0x00);
	ecx336af_write_reg(0x1F, 0x00);

	ecx336af_write_reg(0x20, 0x00);
	ecx336af_write_reg(0x21, 0xE0);
	ecx336af_write_reg(0x22, 0x4A);
	ecx336af_write_reg(0x23, 0x40);
	ecx336af_write_reg(0x24, 0x3D);
	ecx336af_write_reg(0x25, 0x80);//B0 80
	ecx336af_write_reg(0x26, 0x40);
	ecx336af_write_reg(0x27, 0x40);
	ecx336af_write_reg(0x28, 0x50);
	ecx336af_write_reg(0x29, 0x0B);
	ecx336af_write_reg(0x2A, 0xDA);
	ecx336af_write_reg(0x2B, 0x46);
	ecx336af_write_reg(0x2C, 0x02);
	ecx336af_write_reg(0x2D, 0x7A);
	ecx336af_write_reg(0x2E, 0x02);
	ecx336af_write_reg(0x2F, 0xFA);

	ecx336af_write_reg(0x30, 0x26);
	ecx336af_write_reg(0x31, 0x01);
	ecx336af_write_reg(0x32, 0xB6);
	ecx336af_write_reg(0x33, 0x00);
	ecx336af_write_reg(0x34, 0x03);
	ecx336af_write_reg(0x35, 0x5A);
	ecx336af_write_reg(0x36, 0x00);
	ecx336af_write_reg(0x37, 0x76);
	ecx336af_write_reg(0x38, 0x02);
	ecx336af_write_reg(0x39, 0xFE);
	ecx336af_write_reg(0x3A, 0x02);
	ecx336af_write_reg(0x3B, 0x0D);
	ecx336af_write_reg(0x3C, 0x00);
	ecx336af_write_reg(0x3D, 0x1B);
	ecx336af_write_reg(0x3E, 0x00);
	ecx336af_write_reg(0x3F, 0x1C);

	ecx336af_write_reg(0x40, 0x01);
	ecx336af_write_reg(0x41, 0xF3);
	ecx336af_write_reg(0x42, 0x01);
	ecx336af_write_reg(0x43, 0xF4);
	ecx336af_write_reg(0x44, 0x80);
	ecx336af_write_reg(0x45, 0x00);
	ecx336af_write_reg(0x46, 0x00);
	ecx336af_write_reg(0x47, 0x41);
	ecx336af_write_reg(0x48, 0x08);
	ecx336af_write_reg(0x49, 0x02);
	ecx336af_write_reg(0x4A, 0xFC);
	ecx336af_write_reg(0x4B, 0x08);
	ecx336af_write_reg(0x4C, 0x16);
	ecx336af_write_reg(0x4D, 0x08);
	ecx336af_write_reg(0x4E, 0x00);
	ecx336af_write_reg(0x4F, 0x4E);

	ecx336af_write_reg(0x50, 0x02);
	ecx336af_write_reg(0x51, 0xC2);
	ecx336af_write_reg(0x52, 0x01);
	ecx336af_write_reg(0x53, 0x2D);
	ecx336af_write_reg(0x54, 0x01);
	ecx336af_write_reg(0x55, 0x2B);
	ecx336af_write_reg(0x56, 0x00);
	ecx336af_write_reg(0x57, 0x2B);
	ecx336af_write_reg(0x58, 0x23);
	ecx336af_write_reg(0x59, 0x02);
	ecx336af_write_reg(0x5A, 0x25);
	ecx336af_write_reg(0x5B, 0x02);
	ecx336af_write_reg(0x5C, 0x25);
	ecx336af_write_reg(0x5D, 0x02);
	ecx336af_write_reg(0x5E, 0x1D);
	ecx336af_write_reg(0x5F, 0x00);

	ecx336af_write_reg(0x60, 0x23);
	ecx336af_write_reg(0x61, 0x02);
	ecx336af_write_reg(0x62, 0x1D);
	ecx336af_write_reg(0x63, 0x00);
	ecx336af_write_reg(0x64, 0x1A);
	ecx336af_write_reg(0x65, 0x03);
	ecx336af_write_reg(0x66, 0x0A);
	ecx336af_write_reg(0x67, 0xF0);
	ecx336af_write_reg(0x68, 0x00);
	ecx336af_write_reg(0x69, 0xF0);
	ecx336af_write_reg(0x6A, 0x00);
	ecx336af_write_reg(0x6B, 0x00);
	ecx336af_write_reg(0x6C, 0x00);
	ecx336af_write_reg(0x6D, 0xF0);
	ecx336af_write_reg(0x6E, 0x00);
	ecx336af_write_reg(0x6F, 0x60);

	ecx336af_write_reg(0x70, 0x00);
	ecx336af_write_reg(0x71, 0x00);
	ecx336af_write_reg(0x72, 0x00);
	ecx336af_write_reg(0x73, 0x00);
	ecx336af_write_reg(0x74, 0x00);
	ecx336af_write_reg(0x75, 0x00);
	ecx336af_write_reg(0x76, 0x00);
	ecx336af_write_reg(0x77, 0x00);
	ecx336af_write_reg(0x78, 0x00);
	ecx336af_write_reg(0x79, 0x68);
	ecx336af_write_reg(0x7A, 0x00);
	ecx336af_write_reg(0x7B, 0x00);
	ecx336af_write_reg(0x7C, 0x00);
	ecx336af_write_reg(0x7D, 0x00);
	ecx336af_write_reg(0x7E, 0x00);
	ecx336af_write_reg(0x7F, 0x00);

	ecx336af_write_reg(0x00, 0x0F);//03 0e
#if DEBUG_REG
	{
		int i = 0;
		for (i = 0; i < 0x80; i++)
			ecx336af_read_reg(i);

	}
#endif
}

#ifdef CONFIG_HARDWARE_DETECT_MODE
int lcd_selfdet(void)
{
	ecx336af_write_reg(0x00, 0x03);

	if ( ecx336af_read_reg(0x00) != 0x03 )
		return 1;

	return 0;
}
#endif
