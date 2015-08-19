/*
 * Ingenic JZ SFC driver
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: Tiger <xyfu@ingenic.cn>
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

#include <config.h>
#include <common.h>
#include <spi.h>
#include <spi_flash.h>
#include <malloc.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/cpm.h>
#include <asm/arch/spi.h>
#include <asm/arch/sfc.h>
#include <asm/arch/clk.h>
#include <asm/arch/base.h>
#include <malloc.h>

#include "jz_spi.h"

static struct jz_spi_support *gparams;
#ifdef CONFIG_SPI_QUAD
struct spi_quad_mode *quad_mode = NULL;
#endif
/* wait time before read status (us) for spi nand */
//static int t_reset = 500;
int mode = 0;
int flag = 0;


struct jz_sfc {
	unsigned int  addr;
	unsigned int  len;
	unsigned int  cmd;
	unsigned int  addr_plus;
	unsigned int  sfc_mode;
	unsigned char daten;
	unsigned char addr_len;
	unsigned char pollen;
	unsigned char phase;
	unsigned char dummy_byte;
};


static uint32_t jz_sfc_readl(unsigned int offset)
{
	return readl(SFC_BASE + offset);
}

static void jz_sfc_writel(unsigned int value, unsigned int offset)
{
	writel(value, SFC_BASE + offset);
}

void dump_sfc_reg()
{
	int i = 0;
	printf("SFC_GLB			:%x\n", jz_sfc_readl(SFC_GLB ));
	printf("SFC_DEV_CONF	:%x\n", jz_sfc_readl(SFC_DEV_CONF ));
	printf("SFC_DEV_STA_RT	:%x\n", jz_sfc_readl(SFC_DEV_STA_RT ));
	printf("SFC_DEV_STA_MSK	:%x\n", jz_sfc_readl(SFC_DEV_STA_MSK ));
	printf("SFC_TRAN_LEN		:%x\n", jz_sfc_readl(SFC_TRAN_LEN ));

	for(i = 0; i < 6; i++)
		printf("SFC_TRAN_CONF(%d)	:%x\n", i,jz_sfc_readl(SFC_TRAN_CONF(i)));

	for(i = 0; i < 6; i++)
		printf("SFC_DEV_ADDR(%d)	:%x\n", i,jz_sfc_readl(SFC_DEV_ADDR(i)));

	printf("SFC_MEM_ADDR :%x\n", jz_sfc_readl(SFC_MEM_ADDR));
	printf("SFC_TRIG	 :%x\n", jz_sfc_readl(SFC_TRIG));
	printf("SFC_SR		 :%x\n", jz_sfc_readl(SFC_SR));
	printf("SFC_SCR		 :%x\n", jz_sfc_readl(SFC_SCR));
	printf("SFC_INTC	 :%x\n", jz_sfc_readl(SFC_INTC));
	printf("SFC_FSM		 :%x\n", jz_sfc_readl(SFC_FSM ));
	printf("SFC_CGE		 :%x\n", jz_sfc_readl(SFC_CGE ));

}

unsigned int sfc_fifo_num()
{
	unsigned int tmp;
	tmp = jz_sfc_readl(SFC_SR);
	tmp &= (0x7f << 16);
	tmp = tmp >> 16;
	return tmp;
}


void sfc_set_mode(int channel, int value)
{
	unsigned int tmp;
	tmp = jz_sfc_readl(SFC_TRAN_CONF(channel));
	tmp &= ~(TRAN_MODE_MSK);
	tmp |= (value << TRAN_MODE_OFFSET);
	jz_sfc_writel(tmp,SFC_TRAN_CONF(channel));
}


void sfc_dev_addr_dummy_bytes(int channel, unsigned int value)
{
	unsigned int tmp;
	tmp = jz_sfc_readl(SFC_TRAN_CONF(channel));
	tmp &= ~TRAN_CONF_DMYBITS_MSK;
	tmp |= (value << TRAN_CONF_DMYBITS_OFFSET);
	jz_sfc_writel(tmp,SFC_TRAN_CONF(channel));
}

void sfc_transfer_direction(int value)
{
	if(value == 0) {
		unsigned int tmp;
		tmp = jz_sfc_readl(SFC_GLB);
		tmp &= ~TRAN_DIR;
		jz_sfc_writel(tmp,SFC_GLB);
	} else {
		unsigned int tmp;
		tmp = jz_sfc_readl(SFC_GLB);
		tmp |= TRAN_DIR;
		jz_sfc_writel(tmp,SFC_GLB);
	}
}

void sfc_set_length(int value)
{
	jz_sfc_writel(value,SFC_TRAN_LEN);
}

void sfc_set_addr_length(int channel, unsigned int value)
{
	unsigned int tmp;
	tmp = jz_sfc_readl(SFC_TRAN_CONF(channel));
	tmp &= ~(ADDR_WIDTH_MSK);
	tmp |= (value << ADDR_WIDTH_OFFSET);
	jz_sfc_writel(tmp,SFC_TRAN_CONF(channel));
}

void sfc_cmd_en(int channel, unsigned int value)
{
	if(value == 1) {
		unsigned int tmp;
		tmp = jz_sfc_readl(SFC_TRAN_CONF(channel));
		tmp |= CMDEN;
		jz_sfc_writel(tmp,SFC_TRAN_CONF(channel));
	} else {
		unsigned int tmp;
		tmp = jz_sfc_readl(SFC_TRAN_CONF(channel));
		tmp &= ~CMDEN;
		jz_sfc_writel(tmp,SFC_TRAN_CONF(channel));
	}
}

void sfc_data_en(int channel, unsigned int value)
{
	if(value == 1) {
		unsigned int tmp;
		tmp = jz_sfc_readl(SFC_TRAN_CONF(channel));
		tmp |= DATEEN;
		jz_sfc_writel(tmp,SFC_TRAN_CONF(channel));
	} else {
		unsigned int tmp;
		tmp = jz_sfc_readl(SFC_TRAN_CONF(channel));
		tmp &= ~DATEEN;
		jz_sfc_writel(tmp,SFC_TRAN_CONF(channel));
	}
}

void sfc_write_cmd(int channel, unsigned int value)
{
	unsigned int tmp;
	tmp = jz_sfc_readl(SFC_TRAN_CONF(channel));
	tmp &= ~CMD_MSK;
	tmp |= value;
	jz_sfc_writel(tmp,SFC_TRAN_CONF(channel));
}

void sfc_dev_addr(int channel, unsigned int value)
{
	jz_sfc_writel(value, SFC_DEV_ADDR(channel));
}

void sfc_dev_addr_plus(int channel, unsigned int value)
{
	jz_sfc_writel(value,SFC_DEV_ADDR_PLUS(channel));
}

void sfc_set_transfer(struct jz_sfc *hw,int dir)
{
	if(dir == 1)
		sfc_transfer_direction(GLB_TRAN_DIR_WRITE);
	else
		sfc_transfer_direction(GLB_TRAN_DIR_READ);
	sfc_set_length(hw->len);
	sfc_set_addr_length(0, hw->addr_len);
	sfc_cmd_en(0, 0x1);
	sfc_data_en(0, hw->daten);
	sfc_write_cmd(0, hw->cmd);
	sfc_dev_addr(0, hw->addr);
	sfc_dev_addr_plus(0, hw->addr_plus);
	sfc_dev_addr_dummy_bytes(0,hw->dummy_byte);
	sfc_set_mode(0,hw->sfc_mode);

}

static int sfc_read_data(unsigned int *data, unsigned int length)
{
	unsigned int tmp_len = 0;
	unsigned int fifo_num = 0;
	unsigned int i;
	unsigned int reg_tmp = 0;
	unsigned int  len = (length + 3) / 4 ;
	unsigned int time_out = 1000;

	while(1){
		reg_tmp = jz_sfc_readl(SFC_SR);
		if (reg_tmp & RECE_REQ) {
			jz_sfc_writel(CLR_RREQ,SFC_SCR);
			if ((len - tmp_len) > THRESHOLD)
				fifo_num = THRESHOLD;
			else {
				fifo_num = len - tmp_len;
			}

			for (i = 0; i < fifo_num; i++) {
				*data = jz_sfc_readl(SFC_DR);
				data++;
				tmp_len++;
			}
		}
		if (tmp_len == len)
			break;
	}

	reg_tmp = jz_sfc_readl(SFC_SR);
	while (!(reg_tmp & END)){
		reg_tmp = jz_sfc_readl(SFC_SR);
	}

	if ((jz_sfc_readl(SFC_SR)) & END)
		jz_sfc_writel(CLR_END,SFC_SCR);


	return 0;
}


static int sfc_write_data(unsigned int *data, unsigned int length)
{
	unsigned int tmp_len = 0;
	unsigned int fifo_num = 0;
	unsigned int i;
	unsigned int reg_tmp = 0;
	unsigned int  len = (length + 3) / 4 ;
	unsigned int time_out = 10000;

	while(1){
		reg_tmp = jz_sfc_readl(SFC_SR);
		if (reg_tmp & TRAN_REQ) {
			jz_sfc_writel(CLR_TREQ,SFC_SCR);
			if ((len - tmp_len) > THRESHOLD)
				fifo_num = THRESHOLD;
			else {
				fifo_num = len - tmp_len;
			}

			for (i = 0; i < fifo_num; i++) {
				jz_sfc_writel(*data,SFC_DR);
				data++;
				tmp_len++;
			}
		}
		if (tmp_len == len)
			break;
	}

	reg_tmp = jz_sfc_readl(SFC_SR);
	while (!(reg_tmp & END)){
		reg_tmp = jz_sfc_readl(SFC_SR);
	}

	if ((jz_sfc_readl(SFC_SR)) & END)
		jz_sfc_writel(CLR_END,SFC_SCR);


	return 0;
}
/*this code is same as the spl  in common/spl*/
#if 0
static void sfc_set_read_reg(unsigned int cmd, unsigned int addr,
		unsigned int addr_plus, unsigned int addr_len, unsigned int data_en)
{
	volatile unsigned int tmp;
	unsigned int timeout = 0xffff;

	tmp = jz_sfc_readl(SFC_GLB);
	tmp &= ~PHASE_NUM_MSK;
	tmp |= (0x1 << PHASE_NUM_OFFSET);
	jz_sfc_writel(tmp,SFC_GLB);

	tmp = jz_sfc_readl(SFC_TRAN_CONF(0));
	tmp &= ~(ADDR_WIDTH_MSK | DMYBITS_MSK | CMD_MSK | FMAT | DATEEN);
	if (data_en) {
		tmp |= (addr_len << ADDR_WIDTH_OFFSET) | CMDEN |
			DATEEN | (cmd << CMD_OFFSET);
	} else {
		tmp |= (addr_len << ADDR_WIDTH_OFFSET) | CMDEN |
			(cmd << CMD_OFFSET);
	}
	jz_sfc_writel(tmp,SFC_TRAN_CONF(0));

	jz_sfc_writel(addr,SFC_DEV_ADDR(0));
	jz_sfc_writel(addr_plus,SFC_DEV_ADDR_PLUS(0));


#if defined (CONFIG_SPI_QUAD)
	sfc_dev_addr_dummy_bytes(0,8);
	sfc_set_mode(0,TRAN_SPI_QUAD);
#elif defined  (CONFIG_SPI_DUAL)
	sfc_dev_addr_dummy_bytes(0,8);
	sfc_set_mode(0,TRAN_SPI_DUAL);
#else
   sfc_dev_addr_dummy_bytes(0,0);
   sfc_set_mode(0,0);
#endif

	jz_sfc_writel(START,SFC_TRIG);
}



void sfc_write_enable()
{
	struct jz_sfc sfc;
	sfc.cmd = CMD_WREN;
	sfc.addr_len = 0;
	sfc.addr = 0;
	sfc.addr_plus = 0;
	sfc.dummy_byte = 0;
	sfc.daten = 0;
	sfc.len = 0;
	sfc.sfc_mode = 0;

	sfc_set_transfer(&sfc,0);
	jz_sfc_writel(START,SFC_TRIG);
	if ((jz_sfc_readl(SFC_SR)) & END)
		jz_sfc_writel(CLR_END,SFC_SCR);
}


static unsigned int jz_nor_read_status(int regnum)
{
	struct jz_sfc sfc;
	unsigned char cmd_rdsr[3] = {0x05, 0x35, 0x15}; /* reg0, reg1, reg2 */
	unsigned int status_reg = 0;
	int err;

	printf("coc -5\n");
	sfc.cmd = cmd_rdsr[regnum];
	sfc.addr_len = 0;
	sfc.addr = 0;
	sfc.addr_plus = 0;
	sfc.dummy_byte = 0;
	sfc.daten = 1;
	sfc.len = 1;
	sfc.sfc_mode = 0;
	sfc_set_transfer(&sfc,1);
	jz_sfc_writel(START,SFC_TRIG);

	err = sfc_read_data(&status_reg, 1);
	return status_reg;
}

void sfc_set_status_reg(unsigned regnum)
{
	struct jz_sfc sfc;
	volatile unsigned int tmp;
	unsigned int  data;
	unsigned char cmd_wdsr[3] = {0x01, 0x31, 0x11};

	printf("coc -1\n");
	sfc_write_enable();

	printf("coc -2\n");
	sfc.cmd = cmd_wdsr[regnum];
	sfc.addr_len = 0;
	sfc.addr = 0;
	sfc.addr_plus = 0;
	sfc.dummy_byte = 0;
	sfc.daten = 1;
//	sfc.len = 1;
	sfc.len = 2;
	sfc.sfc_mode = 0;
	data = 0x0200;
//	data = 0x2;
	sfc_set_transfer(&sfc,0);
	printf("coc -3\n");
	jz_sfc_writel(START,SFC_TRIG);
	sfc_write_data(&data, 1);



	while (!(jz_nor_read_status(1) & 0x2/*STATUS_WIP*/)) {

	}

}

static int sfc_read(unsigned int addr, unsigned int addr_plus,
		unsigned int addr_len, unsigned int *data, unsigned int len)
{
	int ret;

	unsigned char cmd = 0,mode = 0;

#if defined (CONFIG_SPI_QUAD)
	cmd  = CMD_QUAD_READ;
	mode = TRAN_SPI_QUAD;
#elif defined (CONFIG_SPI_DUAL)
	cmd  = CMD_DUAL_READ;
	mode = TRAN_SPI_DUAL;
#else
	cmd  = CMD_READ;
	mode = TRAN_SPI_STANDARD;
#endif

	//sfc_set_status_reg(0);

	jz_sfc_writel((len * 4), SFC_TRAN_LEN);

	sfc_set_read_reg(cmd, addr, addr_plus, addr_len, 1);

	dump_sfc_reg();

	ret = sfc_read_data(data, len*4);
	if (ret)
		return ret;
	else
		return 0;
}
#endif

void sfc_init(void )
{
	unsigned int i;
	volatile unsigned int tmp;

	tmp = jz_sfc_readl(SFC_GLB);
	tmp &= ~(TRAN_DIR | OP_MODE );
	tmp |= WP_EN;
	jz_sfc_writel(tmp,SFC_GLB);

	tmp = jz_sfc_readl(SFC_DEV_CONF);
	tmp &= ~(CMD_TYPE | CPHA | CPOL | SMP_DELAY_MSK |
			THOLD_MSK | TSETUP_MSK | TSH_MSK);
	tmp |= (CEDL | HOLDDL | WPDL);
	jz_sfc_writel(tmp,SFC_DEV_CONF);

	for (i = 0; i < 6; i++) {
		jz_sfc_writel((jz_sfc_readl(SFC_TRAN_CONF(i))& (~(TRAN_MODE_MSK | FMAT))),SFC_TRAN_CONF(i));
	}

	jz_sfc_writel((CLR_END | CLR_TREQ | CLR_RREQ | CLR_OVER | CLR_UNDER),SFC_INTC);

	jz_sfc_writel(0,SFC_CGE);

	tmp = jz_sfc_readl(SFC_GLB);
	tmp &= ~(THRESHOLD_MSK);
	tmp |= (THRESHOLD << THRESHOLD_OFFSET);
	jz_sfc_writel(tmp,SFC_GLB);

}

void sfc_send_cmd(unsigned char *cmd,unsigned int len,unsigned char *addr ,unsigned addr_len,unsigned dummy_byte,unsigned int daten,unsigned char dir)
{
	struct jz_sfc sfc;
	unsigned int reg_tmp = 0;

	sfc.cmd = *cmd;
	sfc.addr_len = addr_len;
	sfc.addr = ((*addr << 16) &0x00ff0000) | ((*(addr + 1) << 8)&0x0000ff00) |((*(addr + 2))&0xff);
	sfc.addr_plus = 0;
	sfc.dummy_byte = dummy_byte;
	sfc.daten = daten;
	sfc.len = len;

	if((daten == 1)&&(addr_len != 0)){
		sfc.sfc_mode = mode;
	}else{
		sfc.sfc_mode = 0;
	}
	sfc_set_transfer(&sfc,dir);
	jz_sfc_writel(1 << 2,SFC_TRIG);
	jz_sfc_writel(START,SFC_TRIG);

	/*this must judge the end status*/
	if((daten == 0)){
		reg_tmp = jz_sfc_readl(SFC_SR);
		while (!(reg_tmp & END)){
			reg_tmp = jz_sfc_readl(SFC_SR);
		}

		if ((jz_sfc_readl(SFC_SR)) & END)
			jz_sfc_writel(CLR_END,SFC_SCR);
	}

}

int jz_read(struct spi_flash *flash, u32 offset, size_t len, void *data)
{
	unsigned char cmd[5];
	unsigned long read_len;
	unsigned int words_of_len = 0;


#if defined (CONFIG_SPI_QUAD)
	cmd[0]  = quad_mode->cmd_read;
	mode = quad_mode->sfc_mode;
#elif defined (CONFIG_SPI_DUAL)
	cmd[0]  = CMD_DUAL_READ;
	mode = TRAN_SPI_DUAL;
#else
	cmd[0]  = CMD_READ;
	mode = TRAN_SPI_STANDARD;
#endif
	cmd[1] = offset >> 16;
	cmd[2] = offset >> 8;
	cmd[3] = offset >> 0;

	read_len = flash->size - offset;

	if(len < read_len)
		read_len = len;
	/* the paraterms is
		 * cmd , len, addr,addr_len
		 * dummy_byte, daten
		 * dir
		 *
		 * */

#if defined(CONFIG_SPI_DUAL) || defined(CONFIG_SPI_QUAD)
	sfc_send_cmd(&cmd[0],read_len,&cmd[1],3,quad_mode->dummy_byte,1,0);
#else
	sfc_send_cmd(&cmd[0],read_len,&cmd[1],3,0,1,0);
#endif
//	dump_sfc_reg();
	sfc_read_data(data, len);

	return 0;
}

int jz_write(struct spi_flash *flash, u32 offset, size_t length, const void *buf)
{
	unsigned char cmd[6];
	unsigned tmp = 0;
	int chunk_len, actual, i;
	unsigned long byte_addr;
	unsigned char *send_buf = (unsigned char *)buf;
	unsigned int pagelen = 0,len = 0,retlen = 0;

	if (offset + length > flash->size) {
		printf("Data write overflow this chip\n");
		return -1;
	}

	cmd[0] = CMD_WREN;

#if defined (CONFIG_SPI_QUAD)
	cmd[1]  = CMD_QPP;
    mode = TRAN_SPI_QUAD;
#else
	cmd[1]  = CMD_PP;
	mode = TRAN_SPI_STANDARD;
#endif


//	cmd[1] = CMD_PP;

	cmd[5] = CMD_RDSR;


	while (length) {
		if (length >= flash->page_size)
			pagelen = 0;
		else
			pagelen = length % flash->page_size;

		/* the paraterms is
		 * cmd , len, addr,addr_len
		 * dummy_byte, daten
		 * dir
		 *
		 * */
		sfc_send_cmd(&cmd[0],0,0,0,0,0,1);

		if (!pagelen || pagelen > flash->page_size)
			len = flash->page_size;
		else
			len = pagelen;

		if (offset % flash->page_size + len > flash->page_size)
			len -= offset % flash->page_size + len - flash->page_size;

		cmd[2] = offset >> 16;
		cmd[3] = offset >> 8;
		cmd[4] = offset >> 0;

		sfc_send_cmd(&cmd[1], len,&cmd[2],3,0,1,1);

	//	dump_sfc_reg();

		sfc_write_data(send_buf ,len);

		retlen = len;

		/*polling*/
		sfc_send_cmd(&cmd[5],1,0,0,0,1,0);
		sfc_read_data(&tmp, 1);
		while(tmp & CMD_SR_WIP) {
			sfc_send_cmd(&cmd[5],1,0,0,0,1,0);
			sfc_read_data(&tmp, 1);
		}

		if (!retlen) {
			printf("spi nor write failed\n");
			return -1;
		}

		offset += retlen;
		send_buf += retlen;
		length -= retlen;
	}

	return 0;
}



int jz_erase(struct spi_flash *flash, u32 offset, size_t len)
{
	unsigned long erase_size;
	unsigned char cmd[6];
	unsigned int  buf = 0;


	if(len >= 0x8000){
		erase_size = 0x8000;
	}else{
		erase_size = 0x1000;
	}

	if(len % erase_size != 0){
		len = len - (len % erase_size) + erase_size;
	}
//
//	if (len % erase_size) {
//		printf("Erase offset/length not multiple of erase size\n");
//		return -1;
//	}

	cmd[0] = CMD_WREN;

	switch(erase_size) {
	case 0x1000 :
		cmd[1] = CMD_ERASE_4K;
		break;
	case 0x8000 :
		cmd[1] = CMD_ERASE_32K;
		break;
	case 0x10000 :
		cmd[1] = CMD_ERASE_64K;
		break;
	default:
		printf("unknown erase size !\n");
		return -1;
	}

	cmd[5] = CMD_RDSR;

	while(len > 0) {
		cmd[2] = offset >> 16;
		cmd[3] = offset >> 8;
		cmd[4] = offset >> 0;

	//	printf("erase %x %x %x %x %x %x %x \n", cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], offset);

		/* the paraterms is
		 * cmd , len, addr,addr_len
		 * dummy_byte, daten
		 * dir
		 *
		 * */
		sfc_send_cmd(&cmd[0],0,0,0,0,0,1);

		sfc_send_cmd(&cmd[1],0,&cmd[2],3,0,0,1);

		sfc_send_cmd(&cmd[5], 1,0,0,0,1,0);
		sfc_read_data(&buf, 1);
		while(buf & CMD_SR_WIP) {
			sfc_send_cmd(&cmd[5], 1,0,0,0,1,0);
			sfc_read_data(&buf, 1);
		}

		offset += erase_size;
		len -= erase_size;
	}

	return 0;
}


struct spi_flash *sfc_flash_probe_ingenic(struct spi_slave *spi, u8 *idcode)
{
	int i;
	struct spi_flash *flash;
	struct jz_spi_support *params;

	for (i = 0; i < ARRAY_SIZE(jz_spi_support_table); i++) {
		params = &jz_spi_support_table[i];
		if (params->id_manufactory == idcode[0])
			break;
	}

	if (i == ARRAY_SIZE(jz_spi_support_table)) {
#ifdef CONFIG_BURNER
		if (idcode[0] != 0){
			printf("unsupport ID is %04x if the id not be 0x00,the flash is ok for burner\n",idcode[0]);
			params = &jz_spi_support_table[1];
		}else{
			printf("ingenic: Unsupported ID %04x\n", idcode[0]);
			return NULL;

		}
#else
			printf("ingenic: Unsupported ID %04x\n", idcode[0]);
			return NULL;
#endif
	}

	flash = spi_flash_alloc_base(spi, params->name);
	if (!flash) {
		printf("ingenic: Failed to allocate memory\n");
		return NULL;
	}

	flash->erase = jz_erase;
	flash->write = jz_write;
	flash->read  = jz_read;

	flash->page_size = params->page_size;
	flash->sector_size = params->sector_size;
	flash->size = params->size;

	return flash;
}

#ifdef CONFIG_SPI_QUAD
void sfc_set_quad_mode()
{
	/* the paraterms is
	 * cmd , len, addr,addr_len
	 * dummy_byte, daten
	 * dir
	 *
	 * */
	unsigned char cmd[5];
	unsigned int buf = 0;
	unsigned int tmp = 0;
	int i = 10;

	if(quad_mode != NULL){
		cmd[0] = CMD_WREN;
		cmd[1] = quad_mode->WRSR_CMD;
		cmd[2] = quad_mode->RDSR_CMD;
		cmd[3] = CMD_RDSR;

		sfc_send_cmd(&cmd[0],0,0,0,0,0,1);

		sfc_send_cmd(&cmd[1],1,0,0,0,1,1);
		sfc_write_data(&quad_mode->WRSR_DATE,1);

		sfc_send_cmd(&cmd[3],1,0,0,0,1,0);
		sfc_read_data(&tmp, 1);

		while(tmp & CMD_SR_WIP) {
			sfc_send_cmd(&cmd[3],1,0,0,0,1,0);
			sfc_read_data(&tmp, 1);
		}

		sfc_send_cmd(&cmd[2], 1,0,0,0,1,0);
		sfc_read_data(&buf, 1);
		while(!(buf & quad_mode->RDSR_DATE)&&((i--) > 0)) {
			sfc_send_cmd(&cmd[2], 1,0,0,0,1,0);
			sfc_read_data(&buf, 1);
		}

		printf("set quad mode is enable.the buf = %x\n",buf);
	}else{

		printf("the quad_mode is NULL,the nor flash id we not support\n");
	}
}
#endif

void sfc_nor_RDID(unsigned char *idcode)
{
	/* the paraterms is
	 * cmd , len, addr,addr_len
	 * dummy_byte, daten
	 * dir
	 *
	 * */
	unsigned char cmd[1];
	unsigned char chip_id[4];
	cmd[0] = CMD_RDID;
	sfc_send_cmd(&cmd[0],3,0,0,0,1,0);
	sfc_read_data(chip_id, 1);
	*idcode = chip_id[0];
}

int sfc_nor_init()
{
	unsigned char idcode;
	struct jz_spi_support *params;
	int i = 0;
	sfc_nor_RDID(&idcode);

	for (i = 0; i < ARRAY_SIZE(jz_spi_support_table); i++) {
		params = &jz_spi_support_table[i];
		if (params->id_manufactory == idcode){
			printf("the id code = %x, the flash name is %s\n",idcode,params->name);
#ifdef CONFIG_SPI_QUAD
			quad_mode = &jz_spi_support_table[i].quad_mode;
#endif
			break;
		}
	}

	if (i == ARRAY_SIZE(jz_spi_support_table)) {
		if ((idcode != 0)&&(idcode != 0xff)){
			printf("the id code = %x, the flash name is %s\n",idcode,params->name);
			printf("unsupport ID is %04x if the id not be 0x00,the flash is ok for burner\n");
#ifdef CONFIG_SPI_QUAD
			quad_mode = &jz_spi_support_table[1].quad_mode;
#endif
		}else{
			printf("ingenic: Unsupported ID %04x\n", idcode);
			return -1;

		}
	}
	return 0;

}

void sfc_nor_read(unsigned int src_addr, unsigned int count,unsigned int dst_addr)
{

	int i,j;
	unsigned char *data;
	unsigned int temp;
//	int sfc_mode = 0;
	int ret = 0,err = 0;
	unsigned int spl_len = 0,words_of_spl;
	int addr_len = 3;
	struct spi_flash flash;

	flag = 0;
	flash.page_size = 256;
	flash.sector_size = 4 *1024;
	flash.size = 16*1024*1024;
//	ret = sfc_read(src_addr, 0x0, addr_len, (unsigned int *)(dst_addr), words_of_spl);

#ifdef CONFIG_SPI_QUAD
	if(quad_mode == NULL){
		err = sfc_nor_init();
		if(err < 0){
			printf("the sfc quad mode err,check your soft code\n");
			return;
		}
	}
	sfc_set_quad_mode();
#endif

	jz_sfc_writel(1 << 2,SFC_TRIG);

	jz_read(&flash,src_addr,count,dst_addr);
	if (ret) {
		printf("sfc read error\n");
	}

	return ;
}

void sfc_nor_write(unsigned int src_addr, unsigned int count,unsigned int dst_addr,unsigned int erase_en)
{

	int i,j;
	unsigned char *data;
	int ret = 0,err = 0;
	struct spi_flash flash;

	flash.page_size = 256;
	flash.sector_size = 4 *1024;
	flash.size = 16*1024*1024;

	if(erase_en == 1){
		jz_erase(&flash,src_addr,count);
		printf("sfc erase ok\n");
	}
#ifdef CONFIG_SPI_QUAD
	if(quad_mode == NULL){
		err = sfc_nor_init();
		if(err < 0){
			printf("the sfc quad mode err,check your soft code\n");
			return;
		}
	}
	sfc_set_quad_mode();
#endif

	jz_sfc_writel(1 << 2,SFC_TRIG);
	jz_write(&flash,src_addr,count,dst_addr);
	if (ret) {
		printf("sfc write error\n");
	}

	return ;
}

void sfc_nor_erase(unsigned int src_addr, unsigned int count)
{

	int i,j;
	unsigned char *data;
	unsigned int temp;
	int sfc_mode = 0;
	int ret = 0;
	int err = 0;
	unsigned int spl_len = 0,words_of_spl;
	int addr_len = 3;
	struct spi_flash flash;

	flash.page_size = 256;
	flash.sector_size = 4 *1024;
	flash.size = 16*1024*1024;

#ifdef CONFIG_SPI_QUAD
	if(quad_mode == NULL){
		err = sfc_nor_init();
		if(err < 0){
			printf("the sfc quad mode err,check your soft code\n");
			return;
		}
	}
	sfc_set_quad_mode();
#endif
	jz_sfc_writel(1 << 2,SFC_TRIG);
	jz_erase(&flash,src_addr,count);
	if (ret) {
		printf("sfc erase error\n");
	}

	return ;
}

