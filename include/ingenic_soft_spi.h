
#ifndef __INCLUDE__GPIO__SPI__
#define __INCLUDE__GPIO__SPI__

struct spi {
	unsigned int clk ;
	unsigned int data_in ;
	unsigned int data_out ;
	unsigned int enable  ;
};

void spi_init_jz(struct spi *spi);

#endif
