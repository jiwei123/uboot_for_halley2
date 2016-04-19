#include <common.h>
#include <asm/io.h>

#define CONFIG_HIMAX7097_I2C0_GPIO_SCL2     GPIO_PD(31)
#define CONFIG_HIMAX7097_I2C0_GPIO_SDA2     GPIO_PD(30)

# ifndef I2C_GPIO_SYNC2
#  define I2C_GPIO_SYNC2
# endif

# ifndef I2C_INIT2
#  define I2C_INIT2							\
	do {								\
		gpio_request(CONFIG_HIMAX7097_I2C0_GPIO_SCL2, "himax7097_i2c2"); \
		gpio_request(CONFIG_HIMAX7097_I2C0_GPIO_SDA2, "himax7097_i2c2"); \
	} while (0)
# endif

# ifndef I2C_ACTIVE2
#  define I2C_ACTIVE2 do { } while (0)
# endif

# ifndef I2C_TRISTATE2
#  define I2C_TRISTATE2 do { } while (0)
# endif

# ifndef I2C_READ2
#  define I2C_READ2 gpio_get_value(CONFIG_HIMAX7097_I2C0_GPIO_SDA2)
# endif

/* gpio_direction_input(CONFIG_HIMAX7097_I2C0_GPIO_SDA2); */
# ifndef I2C_SDA2
#  define I2C_SDA2(bit)							\
	do {								\
	        gpio_direction_output(CONFIG_HIMAX7097_I2C0_GPIO_SDA2, bit); \
		I2C_GPIO_SYNC2;						\
	} while (0)
# endif

# ifndef I2C_SCL2
#  define I2C_SCL2(bit)							\
	do {								\
		gpio_direction_output(CONFIG_HIMAX7097_I2C0_GPIO_SCL2, bit); \
		I2C_GPIO_SYNC2;						\
	} while (0)
# endif

# ifndef I2C_DELAY2
#  define I2C_DELAY2 udelay(5)	/* 1/4 I2C clock duration */
# endif

/*-----------------------------------------------------------------------
 * Definitions
 */

#define I2C_ACK2		0		/* PD_SDA level to ack a byte */
#define I2C_NOACK2	1		/* PD_SDA level to noack a byte */

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
	I2C_DELAY2;
	I2C_SDA2(1);
	I2C_ACTIVE2;
	I2C_DELAY2;
	I2C_SCL2(1);
	I2C_DELAY2;
	I2C_SDA2(0);
	I2C_DELAY2;
}

/*-----------------------------------------------------------------------
 * STOP: Low -> High on SDA while SCL is High
 */
static void send_stop(void)
{
	I2C_SCL2(0);
	I2C_DELAY2;
	I2C_SDA2(0);
	I2C_ACTIVE2;
	I2C_DELAY2;
	I2C_SCL2(1);
	I2C_DELAY2;
	I2C_SDA2(1);
	I2C_DELAY2;
	I2C_TRISTATE2;
}

/*-----------------------------------------------------------------------
 * ack should be I2C_ACK2 or I2C_NOACK2
 */
static void send_ack(int ack)
{
	I2C_SCL2(0);
	I2C_DELAY2;
	I2C_ACTIVE2;
	I2C_SDA2(ack);
	I2C_DELAY2;
	I2C_SCL2(1);
	I2C_DELAY2;
	I2C_DELAY2;
	I2C_SCL2(0);
	I2C_DELAY2;
}

/*-----------------------------------------------------------------------
 * Send 8 bits and look for an acknowledgement.
 */
static int write_byte(uchar data)
{

	int j;
	int nack;

	I2C_ACTIVE2;
	for(j = 0; j < 8; j++) {
		I2C_SCL2(0);
		I2C_DELAY2;
		I2C_SDA2(data & 0x80);
		I2C_DELAY2;
		I2C_SCL2(1);
		I2C_DELAY2;
		I2C_DELAY2;

		data <<= 1;
	}

	/*
	 * Look for an <ACK>(negative logic) and return it.
	 */
	I2C_SCL2(0);
	I2C_DELAY2;
	I2C_SDA2(1);
	I2C_TRISTATE2;
	I2C_DELAY2;
	I2C_SCL2(1);
	I2C_DELAY2;
	I2C_DELAY2;
	nack = I2C_READ2;
	I2C_SCL2(0);
	I2C_DELAY2;
	I2C_ACTIVE2;

	return(nack);	/* not a nack is an ack */
}

/*-----------------------------------------------------------------------
 * if ack == I2C_ACK2, ACK the byte so can continue reading, else
 * send I2C_NOACK2 to end the read.
 */
static uchar read_byte(int ack)
{

	int  data;
	int  j;

	/*
	 * Read 8 bits, MSB first.
	 */
	I2C_TRISTATE2;
	I2C_SDA2(1);
	data = 0;
	for(j = 0; j < 8; j++) {
		I2C_SCL2(0);
		I2C_DELAY2;
		I2C_SCL2(1);
		I2C_DELAY2;
		data <<= 1;
		data |= I2C_READ2;
		I2C_DELAY2;
	}
	send_ack(ack);

	return data;
}

int  himax7097_i2c_read2(uchar chip, uint addr, int alen, uchar *buffer, int len)
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
		*buffer++ = read_byte(I2C_ACK2);

	send_stop();

	return 0;
}

/*-----------------------------------------------------------------------
 * Write bytes
 */
int  himax7097_i2c_write2(uchar chip, uint addr, int alen, uchar *buffer, int len)
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

unsigned char himax7097_read_reg2( int reg )
{
	unsigned char  data;
	int retval = -1;

	retval = himax7097_i2c_read2(HIMAX7097_I2C_ADDR, reg, 1, &data, 1);

	if (retval)
		printf("Read 0x%x register error!\n", reg);

	return data;
}

int himax7097_write_reg2(int reg, unsigned char data )
{
	int retval = -1;
	uchar buff = data;

	retval = himax7097_i2c_write2(HIMAX7097_I2C_ADDR, reg, 1, &buff, 1);

	if (retval)
		printf("Write 0x%02x 0x%02x error : %d\n", reg, buff, retval);

	return retval;
}

void lcd_second_panel_init(void)
{
	himax7097_write_reg2( 0x00, 0x04 );
#if (defined(CONFIG_VIDEO_HIMAX7097_SECOND_UPSIDE_DOWN) && defined(CONFIG_VIDEO_HIMAX7097_SECOND_HORIZONTAL_EXCHANGE))
	himax7097_write_reg2( 0x01, 0x02 );
#elif defined(CONFIG_VIDEO_HIMAX7097_SECOND_HORIZONTAL_EXCHANGE)
	himax7097_write_reg2( 0x01, 0x10 );
#elif defined(CONFIG_VIDEO_HIMAX7097_SECOND_UPSIDE_DOWN)
	himax7097_write_reg2( 0x01, 0x13 );
#else
	himax7097_write_reg2( 0x01, 0x11 );
#endif
	himax7097_write_reg2( 0x02, 0x04 );
	himax7097_write_reg2( 0x03, 0x01 );
	himax7097_write_reg2( 0x04, 0x02 );
	//himax7097_write_reg2( 0x05, 0xf5 );
	//himax7097_write_reg2( 0x06, 0x55 );
	//himax7097_write_reg2( 0x07, 0x55 );
	//himax7097_write_reg2( 0x08, 0x55 );
	himax7097_write_reg2( 0x09, 0xFF );
	himax7097_write_reg2( 0x0A, 0xA4 );
	himax7097_write_reg2( 0x0B, 0x6C );
	himax7097_write_reg2( 0x0C, 0x3F );
	himax7097_write_reg2( 0x0D, 0x10 );
	himax7097_write_reg2( 0x0E, 0x0A );
	himax7097_write_reg2( 0x0F, 0x00 );

	himax7097_write_reg2( 0x10, 0x5B );
	himax7097_write_reg2( 0x11, 0x93 );
	himax7097_write_reg2( 0x12, 0xC0 );
	himax7097_write_reg2( 0x13, 0xEF );
	himax7097_write_reg2( 0x14, 0xFF );
	himax7097_write_reg2( 0x15, 0xFF );
	himax7097_write_reg2( 0x16, 0xA4 );
	himax7097_write_reg2( 0x17, 0x6C );

	himax7097_write_reg2( 0x18, 0x3F );
	himax7097_write_reg2( 0x19, 0x10 );
	himax7097_write_reg2( 0x1A, 0x0A );
	himax7097_write_reg2( 0x1B, 0x00 );
	himax7097_write_reg2( 0x1C, 0x5B );
	himax7097_write_reg2( 0x1D, 0x93 );
	himax7097_write_reg2( 0x1E, 0xC0 );
	himax7097_write_reg2( 0x1F, 0xEF );

	himax7097_write_reg2( 0x20, 0xFF );
	himax7097_write_reg2( 0x21, 0xFF );
	himax7097_write_reg2( 0x22, 0xA4 );
	himax7097_write_reg2( 0x23, 0x6C );
	himax7097_write_reg2( 0x24, 0x3F );
	himax7097_write_reg2( 0x25, 0x10 );
	himax7097_write_reg2( 0x26, 0x0A );
	himax7097_write_reg2( 0x27, 0x00 );

	himax7097_write_reg2( 0x28, 0x5B );
	himax7097_write_reg2( 0x29, 0x93 );
	himax7097_write_reg2( 0x2A, 0xC0 );
	himax7097_write_reg2( 0x2B, 0xEF );
	himax7097_write_reg2( 0x2C, 0xFF );
}

#ifdef CONFIG_HARDWARE_DETECT_MODE
int lcd_selfdet(void)
{
	if ( himax7097_write_reg2( 0x00, 0x04 ) < 0 ||
	     himax7097_read_reg2( 0x00 ) != 0x04 )
		return 1;

	return 0;
}
#endif
