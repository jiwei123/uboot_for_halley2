#include <common.h>
#include <asm/io.h>
#include <serial.h>

#define CONFIG_HIMAX7033_I2C0_GPIO_SCL     GPIO_PC(18)
#define CONGIS_HIMAX7033_I2C0_GPIO_SDA     GPIO_PC(19)

# ifndef I2C_GPIO_SYNC
#  define I2C_GPIO_SYNC
# endif

# ifndef I2C_ACTIVE
#  define I2C_ACTIVE do { } while (0)
# endif

# ifndef I2C_TRISTATE
#  define I2C_TRISTATE do { } while (0)
# endif

# ifndef I2C_READ
#  define I2C_READ gpio_get_value(CONGIS_HIMAX7033_I2C0_GPIO_SDA)
# endif

# ifndef I2C_SDA
#  define I2C_SDA(bit)							\
	do {								\
	        gpio_direction_output(CONGIS_HIMAX7033_I2C0_GPIO_SDA, bit); \
		I2C_GPIO_SYNC;						\
	} while (0)
# endif

# ifndef I2C_SCL
#  define I2C_SCL(bit)							\
	do {								\
		gpio_direction_output(CONFIG_HIMAX7033_I2C0_GPIO_SCL, bit); \
		I2C_GPIO_SYNC;						\
	} while (0)
# endif

# ifndef I2C_DELAY
#  define I2C_DELAY udelay(5)	/* 1/4 I2C clock duration */
# endif

/*-----------------------------------------------------------------------
 * Definitions
 */

#define I2C_ACK		0		/* PD_SDA level to ack a byte */
#define I2C_NOACK	1		/* PD_SDA level to noack a byte */

static void  send_start	(void);
static void  send_stop	(void);
static void  send_ack	(int);
static int   write_byte	(unsigned char byte);
static unsigned char  read_byte	(int);


/*-----------------------------------------------------------------------
 * START: High -> Low on SDA while SCL is High
 */
static void send_start(void)
{
	I2C_DELAY;
	I2C_SDA(1);
	I2C_ACTIVE;
	I2C_DELAY;
	I2C_SCL(1);
	I2C_DELAY;
	I2C_SDA(0);
	I2C_DELAY;
}

/*-----------------------------------------------------------------------
 * STOP: Low -> High on SDA while SCL is High
 */
static void send_stop(void)
{
	I2C_SCL(0);
	I2C_DELAY;
	I2C_SDA(0);
	I2C_ACTIVE;
	I2C_DELAY;
	I2C_SCL(1);
	I2C_DELAY;
	I2C_SDA(1);
	I2C_DELAY;
	I2C_TRISTATE;
}

/*-----------------------------------------------------------------------
 * ack should be I2C_ACK or I2C_NOACK
 */
static void send_ack(int ack)
{
	I2C_SCL(0);
	I2C_DELAY;
	I2C_ACTIVE;
	I2C_SDA(ack);
	I2C_DELAY;
	I2C_SCL(1);
	I2C_DELAY;
	I2C_DELAY;
	I2C_SCL(0);
	I2C_DELAY;
}

/*-----------------------------------------------------------------------
 * Send 8 bits and look for an acknowledgement.
 */
static int write_byte(unsigned char  data)
{

	int j;
	int nack;

	I2C_ACTIVE;
	for(j = 0; j < 8; j++) {
		I2C_SCL(0);
		I2C_DELAY;
		I2C_SDA(data & 0x80);
		I2C_DELAY;
		I2C_SCL(1);
		I2C_DELAY;
		I2C_DELAY;

		data <<= 1;
	}

	/*
	 * Look for an <ACK>(negative logic) and return it.
	 */
	I2C_SCL(0);
	I2C_DELAY;
	I2C_SDA(1);
	I2C_TRISTATE;
	I2C_DELAY;
	I2C_SCL(1);
	I2C_DELAY;
	I2C_DELAY;
	gpio_direction_input(CONGIS_HIMAX7033_I2C0_GPIO_SDA);
	nack = I2C_READ;
	I2C_SCL(0);
	I2C_DELAY;
	I2C_ACTIVE;

	return nack;	/* not a nack is an ack */
}

/*-----------------------------------------------------------------------
 * if ack == I2C_ACK, ACK the byte so can continue reading, else
 * send I2C_NOACK to end the read.
 */
static unsigned char  read_byte(int ack)
{

	int  data;
	int  j;

	/*
	 * Read 8 bits, MSB first.
	 */
	I2C_TRISTATE;
	I2C_SDA(1);
	gpio_direction_input(CONGIS_HIMAX7033_I2C0_GPIO_SDA);
	data = 0;
	for(j = 0; j < 8; j++) {
		I2C_SCL(0);
		I2C_DELAY;
		I2C_SCL(1);
		I2C_DELAY;
		data <<= 1;
		data |= I2C_READ;
		I2C_DELAY;
	}
	send_ack(ack);

	return data;
}

int  himax7033_i2c_read(unsigned char  chip, uint addr, int alen, unsigned char *buffer, int len)
{
	int shift;

	send_start();
	if(alen > 0) {
		if(write_byte(chip << 1)) {	/* write cycle */
			send_stop();
			printf("i2c_read : chip no responded\n");
			return 1;
		}
		shift = (alen-1) * 8;
		while(alen-- > 0) {
			if(write_byte(addr >> shift)) {
				printf("i2c_read : address no ACK\n");
				return 1;
			}
			shift -= 8;
		}

	       
		send_stop();
		send_start();
	}
      
	write_byte((chip << 1) | 1);	/* read cycle */
	while(len-- > 0)
		*buffer++ = read_byte(I2C_ACK);

	send_stop();

	return 0;
}

/*-----------------------------------------------------------------------
 * Write bytes
 */
int  himax7033_i2c_write(unsigned char  chip, uint addr, int alen, unsigned char  *buffer, int len)
{
	int shift, failures = 0;

	send_start();
	if(write_byte(chip << 1)) {	/* write cycle */
		send_stop();
		printf("i2c_write : chip no responded\n");
		return 1;
	}
	shift = (alen-1) * 8;
	while(alen-- > 0) {
		if(write_byte(addr >> shift)) {
			printf("i2c_write : chip no ACK\n");
			return 1;
		}
		shift -= 8;
	}

	while(len-- > 0)
		if(write_byte(*buffer++))
			failures++;

	send_stop();

	return failures;
}

#define HIMAX7033_I2C_ADDR  0x48

int himax7033_read_reg( int reg )
{
	unsigned char  data;
	int retval = -1;

	retval = himax7033_i2c_read(HIMAX7033_I2C_ADDR, reg, 1, &data, 1);

	if (retval)
		printf("Read 0x%x register error!\n", reg);

	return data;
}

int himax7033_write_reg(int reg, unsigned char data)
{
	int retval = -1;
	unsigned char buff = data;

	retval = himax7033_i2c_write(HIMAX7033_I2C_ADDR, reg, 1, &buff, 1);

	if (retval)
		printf("Write 0x%02x 0x%02x error : %d\n", reg, buff, retval);

	return retval;
}

void himax7033_init(struct platform_himax7033_lcd_data *pdata)
{    
	himax7033_write_reg( 0x00, 0x96 );
	himax7033_write_reg( 0x01, 0x00 );
	himax7033_write_reg( 0x02, 0x00 );
	himax7033_write_reg( 0x03, 0x50 );
	himax7033_write_reg( 0x04, 0x84 );
	himax7033_write_reg( 0x05, 0xf5 );
	himax7033_write_reg( 0x06, 0x55 );
	himax7033_write_reg( 0x07, 0x55 );

	himax7033_write_reg( 0x08, 0x55 );
	himax7033_write_reg( 0x09, 0x55 );
	himax7033_write_reg( 0x0A, 0xf6 );
	himax7033_write_reg( 0x0B, 0x80 );
	himax7033_write_reg( 0x0C, 0x00 );
	himax7033_write_reg( 0x0D, 0x4c );
	himax7033_write_reg( 0x0E, 0x6b );
	himax7033_write_reg( 0x0F, 0x8a );

	himax7033_write_reg( 0x10, 0xb5 );
	himax7033_write_reg( 0x11, 0xea );
	himax7033_write_reg( 0x12, 0x15 );
	himax7033_write_reg( 0x13, 0x4a );
	himax7033_write_reg( 0x14, 0x75 );
	himax7033_write_reg( 0x15, 0x94 );
	himax7033_write_reg( 0x16, 0xb2 );
	himax7033_write_reg( 0x17, 0xff );

	himax7033_write_reg( 0x18, 0x00 );
	himax7033_write_reg( 0x19, 0x4c );
	himax7033_write_reg( 0x1A, 0x6b );
	himax7033_write_reg( 0x1B, 0x8a );
	himax7033_write_reg( 0x1C, 0xb5 );
	himax7033_write_reg( 0x1D, 0xea );
	himax7033_write_reg( 0x1E, 0x15 );
	himax7033_write_reg( 0x1F, 0x4a );

	himax7033_write_reg( 0x20, 0x75 );
	himax7033_write_reg( 0x21, 0x94 );
	himax7033_write_reg( 0x22, 0xb2 );
	himax7033_write_reg( 0x23, 0xff );
	himax7033_write_reg( 0x24, 0x00 );
	himax7033_write_reg( 0x25, 0x4c );
	himax7033_write_reg( 0x26, 0x6b );
	himax7033_write_reg( 0x27, 0x8a );

	himax7033_write_reg( 0x28, 0xb5 );
	himax7033_write_reg( 0x29, 0xea );
	himax7033_write_reg( 0x2A, 0x15 );
	himax7033_write_reg( 0x2B, 0x4a );
	himax7033_write_reg( 0x2C, 0x75 );
	himax7033_write_reg( 0x2D, 0x94 );
	himax7033_write_reg( 0x2E, 0xb2 );
	himax7033_write_reg( 0x2F, 0xff );

	himax7033_write_reg( 0x30, 0x00 );
	himax7033_write_reg( 0x31, 0xff );
	himax7033_write_reg( 0x32, 0x50 );
	himax7033_write_reg( 0x33, 0x78 );
	himax7033_write_reg( 0x34, 0xa0 );
	himax7033_write_reg( 0x35, 0xc8 );
	himax7033_write_reg( 0x36, 0x04 );
	himax7033_write_reg( 0x37, 0x22 );

	himax7033_write_reg( 0x38, 0x88 );
	himax7033_write_reg( 0x39, 0x00 );
	himax7033_write_reg( 0x3A, 0x00 );
	himax7033_write_reg( 0x3B, 0x00 );
	himax7033_write_reg( 0x3C, 0x00 );
	himax7033_write_reg( 0x3D, 0x00 );
	himax7033_write_reg( 0x3E, 0x00 );
	himax7033_write_reg( 0x3F, 0x00 );
}

#ifdef CONFIG_HARDWARE_DETECT_MODE
int lcd_selfdet(void)
{
	if ( himax7033_write_reg( 0x00, 0x96 ) < 0 ||
	     himax7033_read_reg( 0x00 ) != 0x96 )
		return 1;

	return 0;
}
#endif
