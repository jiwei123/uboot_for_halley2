#include <common.h>
#include <asm/io.h>

#define CONFIG_HIMAX7097_I2C0_GPIO_SCL     GPIO_PC(18)
#define CONFIG_HIMAX7097_I2C0_GPIO_SDA     GPIO_PC(19)

# ifndef I2C_GPIO_SYNC
#  define I2C_GPIO_SYNC
# endif

# ifndef I2C_INIT
#  define I2C_INIT							\
	do {								\
		gpio_request(CONFIG_HIMAX7097_I2C0_GPIO_SCL, "himax7097_i2c"); \
		gpio_request(CONFIG_HIMAX7097_I2C0_GPIO_SDA, "himax7097_i2c"); \
	} while (0)
# endif

# ifndef I2C_ACTIVE
#  define I2C_ACTIVE do { } while (0)
# endif

# ifndef I2C_TRISTATE
#  define I2C_TRISTATE do { } while (0)
# endif

# ifndef I2C_READ
#  define I2C_READ gpio_get_value(CONFIG_HIMAX7097_I2C0_GPIO_SDA)
# endif

/* gpio_direction_input(CONFIG_HIMAX7097_I2C0_GPIO_SDA); */
# ifndef I2C_SDA
#  define I2C_SDA(bit)							\
	do {								\
	        gpio_direction_output(CONFIG_HIMAX7097_I2C0_GPIO_SDA, bit); \
		I2C_GPIO_SYNC;						\
	} while (0)
# endif

# ifndef I2C_SCL
#  define I2C_SCL(bit)							\
	do {								\
		gpio_direction_output(CONFIG_HIMAX7097_I2C0_GPIO_SCL, bit); \
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
static int   write_byte	(uchar byte);
static uchar read_byte	(int);

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
static int write_byte(uchar data)
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
	nack = I2C_READ;
	I2C_SCL(0);
	I2C_DELAY;
	I2C_ACTIVE;

	return(nack);	/* not a nack is an ack */
}

/*-----------------------------------------------------------------------
 * if ack == I2C_ACK, ACK the byte so can continue reading, else
 * send I2C_NOACK to end the read.
 */
static uchar read_byte(int ack)
{

	int  data;
	int  j;

	/*
	 * Read 8 bits, MSB first.
	 */
	I2C_TRISTATE;
	I2C_SDA(1);
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

int  himax7097_i2c_read(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	int shift;

	send_start();

	if(alen > 0) {
		if(write_byte(chip << 1)) {	/* write cycle */
			send_stop();
			printf("i2c_read, no chip responded %02X\n", chip);
                        return 1;
		}
		shift = (alen-1) * 8;
		while(alen-- > 0) {
			if(write_byte(addr >> shift)) {
				printf("i2c_read, address not <ACK>ed\n");
				return 1;
			}
			shift -= 8;
		}

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
int  himax7097_i2c_write(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	int shift, failures = 0;

	send_start();
	if(write_byte(chip << 1)) {	/* write cycle */
		send_stop();
		printf("i2c_write, no chip responded %02X\n", chip);
                return 1;
	}
	shift = (alen-1) * 8;
	while(alen-- > 0) {
		if(write_byte(addr >> shift)) {
			printf("i2c_write, address not <ACK>ed\n");
                        return 1;
		}
		shift -= 8;
	}

	while(len-- > 0) {
		if(write_byte(*buffer++))
			failures++;
	}
	send_stop();
	return(failures);
}

#define HIMAX7097_I2C_ADDR  0x48

typedef unsigned char uchar;

unsigned char himax7097_read_reg( int reg )
{
	unsigned char  data;
	int retval = -1;

	retval = himax7097_i2c_read(HIMAX7097_I2C_ADDR, reg, 1, &data, 1);

	if (retval)
		printf("Read 0x%x register error!\n", reg);

	return data;
}

int himax7097_write_reg(int reg, unsigned char data )
{
	int retval = -1;
	uchar buff = data;

	retval = himax7097_i2c_write(HIMAX7097_I2C_ADDR, reg, 1, &buff, 1);

	if (retval)
		printf("Write 0x%02x 0x%02x error : %d\n", reg, buff, retval);

	return retval;
}

void lcd_panel_init(void)
{
	himax7097_write_reg( 0x00, 0x04 );
#if (defined(CONFIG_VIDEO_HIMAX7097_UPSIDE_DOWN) && defined(CONFIG_VIDEO_HIMAX7097_HORIZONTAL_EXCHANGE))
	himax7097_write_reg( 0x01, 0x02 );
#elif defined(CONFIG_VIDEO_HIMAX7097_HORIZONTAL_EXCHANGE)
	himax7097_write_reg( 0x01, 0x10 );
#elif defined(CONFIG_VIDEO_HIMAX7097_UPSIDE_DOWN)
	himax7097_write_reg( 0x01, 0x13 );
#else
	himax7097_write_reg( 0x01, 0x11 );
#endif
	himax7097_write_reg( 0x02, 0x04 );
	himax7097_write_reg( 0x03, 0x01 );
	himax7097_write_reg( 0x04, 0x02 );
	//himax7097_write_reg( 0x05, 0xf5 );
	//himax7097_write_reg( 0x06, 0x55 );
	//himax7097_write_reg( 0x07, 0x55 );
	//himax7097_write_reg( 0x08, 0x55 );
	himax7097_write_reg( 0x09, 0xFF );
	himax7097_write_reg( 0x0A, 0xA4 );
	himax7097_write_reg( 0x0B, 0x6C );
	himax7097_write_reg( 0x0C, 0x3F );
	himax7097_write_reg( 0x0D, 0x10 );
	himax7097_write_reg( 0x0E, 0x0A );
	himax7097_write_reg( 0x0F, 0x00 );

	himax7097_write_reg( 0x10, 0x5B );
	himax7097_write_reg( 0x11, 0x93 );
	himax7097_write_reg( 0x12, 0xC0 );
	himax7097_write_reg( 0x13, 0xEF );
	himax7097_write_reg( 0x14, 0xFF );
	himax7097_write_reg( 0x15, 0xFF );
	himax7097_write_reg( 0x16, 0xA4 );
	himax7097_write_reg( 0x17, 0x6C );

	himax7097_write_reg( 0x18, 0x3F );
	himax7097_write_reg( 0x19, 0x10 );
	himax7097_write_reg( 0x1A, 0x0A );
	himax7097_write_reg( 0x1B, 0x00 );
	himax7097_write_reg( 0x1C, 0x5B );
	himax7097_write_reg( 0x1D, 0x93 );
	himax7097_write_reg( 0x1E, 0xC0 );
	himax7097_write_reg( 0x1F, 0xEF );

	himax7097_write_reg( 0x20, 0xFF );
	himax7097_write_reg( 0x21, 0xFF );
	himax7097_write_reg( 0x22, 0xA4 );
	himax7097_write_reg( 0x23, 0x6C );
	himax7097_write_reg( 0x24, 0x3F );
	himax7097_write_reg( 0x25, 0x10 );
	himax7097_write_reg( 0x26, 0x0A );
	himax7097_write_reg( 0x27, 0x00 );

	himax7097_write_reg( 0x28, 0x5B );
	himax7097_write_reg( 0x29, 0x93 );
	himax7097_write_reg( 0x2A, 0xC0 );
	himax7097_write_reg( 0x2B, 0xEF );
	himax7097_write_reg( 0x2C, 0xFF );
}

#ifdef CONFIG_HARDWARE_DETECT_MODE
int lcd_selfdet(void)
{
	if ( himax7097_write_reg( 0x00, 0x04 ) < 0 ||
	     himax7097_read_reg( 0x00 ) != 0x04 )
		return 1;

	return 0;
}
#endif
