#include <common.h>
#include <config.h>
#include <spl.h>
#include <spi.h>
#include <asm/io.h>
#include <nand.h>
#include <asm/arch/sfc.h>
#include <asm/arch/spi.h>
#include <asm/arch/clk.h>

static uint32_t jz_sfc_readl(unsigned int offset)
{
	return readl(SFC_BASE + offset);
}

static void jz_sfc_writel(unsigned int value, unsigned int offset)
{
	writel(value, SFC_BASE + offset);
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

#if defined (CONFIG_SPI_QUAD) || defined(CONFIG_SPI_DUAL)
	sfc_dev_addr_dummy_bytes(0,8);
	sfc_set_mode(0,TRAN_SPI_DUAL);
#else
	sfc_dev_addr_dummy_bytes(0,0);
	sfc_set_mode(0,0);
#endif

	jz_sfc_writel(START,SFC_TRIG);
}

static int sfc_read_data(unsigned int *data, unsigned int len)
{
	unsigned int tmp_len = 0;
	unsigned int fifo_num = 0;
	unsigned int i;
	unsigned int reg_tmp;

	while (1) {
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

	if ((jz_sfc_readl(SFC_SR)) & END)
		jz_sfc_writel(CLR_END,SFC_SCR);

	return 0;
}

static int sfc_read(unsigned int addr, unsigned int addr_plus,
		unsigned int addr_len, unsigned int *data, unsigned int len)
{
	int ret;
	int cmd = 0;

#if defined (CONFIG_SPI_QUAD) || defined (CONFIG_SPI_DUAL)
	cmd  = CMD_DUAL_READ;
#else
	cmd  = CMD_READ;
#endif

	jz_sfc_writel((len * 4), SFC_TRAN_LEN);

	sfc_set_read_reg(cmd, addr, addr_plus, addr_len, 1);

	ret = sfc_read_data(data, len);
	if (ret)
		return ret;
	else
		return 0;
}

void sfc_init()
{
	unsigned int i;
	volatile unsigned int tmp;


	tmp = jz_sfc_readl(SFC_GLB);
	tmp &= ~(TRAN_DIR | OP_MODE);
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


void sfc_nor_load(unsigned int src_addr, unsigned int count,unsigned int dst_addr)
{

	int i,j;
	unsigned char *data;
	unsigned int temp;
	int sfc_mode = 0;
	int ret = 0;
	unsigned int spl_len = 0,words_of_spl = 0;
	int addr_len = 3;

	if ((count % 4) == 0) {
		words_of_spl = count / 4;
	} else {
		words_of_spl = count / 4 + 1;
	}

	jz_sfc_writel(1 << 2,SFC_TRIG);

	ret = sfc_read(src_addr, 0x0, addr_len, (unsigned int *)(dst_addr), words_of_spl);
	if (ret) {
		printf("sfc read error\n");
	}

	return ;
}
#define NV_AREA_START (288 * 1024)
static void nv_map_area(unsigned int *base_addr)
{
	unsigned int buf[3][2];
	unsigned int tmp_buf[4];
	unsigned int nv_num = 0, nv_count = 0;
	unsigned int addr, i;

	for(i = 0; i < 3; i++) {
		addr = NV_AREA_START + i * 32 * 1024;
		sfc_nor_load(addr, 4, buf[i]);
		if(buf[i][0] == 0x5a5a5a5a) {
			sfc_nor_load(addr + 1 *1024,  16, tmp_buf);
			addr += 32 * 1024 - 8;
			sfc_nor_load(addr, 8, buf[i]);
			if(buf[i][1] == 0xa5a5a5a5) {
				if(nv_count < buf[i][0]) {
					nv_count = buf[i][0];
					nv_num = i;
				}
			}
		}
	}
	*base_addr = NV_AREA_START + nv_num * 32 * 1024 + 1024;
}

void spl_sfc_nor_load_image(void)
{
	struct image_header *header;

#ifdef CONFIG_SPL_OS_BOOT
	unsigned nv_buf[4];
	int count = 16;
	unsigned int src_addr, updata_flag;
#endif
	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE);

	/*the sfc clk is 1/2 ssi clk */
	clk_set_rate(SSI,70000000);

	jz_sfc_writel(0,SFC_TRIG);
	jz_sfc_writel(1 << 2,SFC_TRIG);

	sfc_init();
#ifdef CONFIG_SPL_OS_BOOT
	nv_map_area((unsigned int)&src_addr);
	sfc_nor_load(src_addr, count, nv_buf);
	updata_flag = nv_buf[3];
	if((updata_flag & 0x3) != 0x3) {
		sfc_nor_load(CONFIG_SPL_OS_OFFSET, sizeof(struct image_header), CONFIG_SYS_TEXT_BASE);
		spl_parse_image_header(header);
		sfc_nor_load(CONFIG_SPL_OS_OFFSET, spl_image.size, spl_image.load_addr);
	} else
#endif
	{
		spl_parse_image_header(header);
		sfc_nor_load(CONFIG_UBOOT_OFFSET, CONFIG_SYS_MONITOR_LEN,CONFIG_SYS_TEXT_BASE);
	}
	return ;

}
