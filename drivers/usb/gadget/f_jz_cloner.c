/*
 * Ingenic burner function Code (Vendor Private Protocol)
 *
 * Copyright (c) 2013 cli <cli@ingenic.cn>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
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

#include <errno.h>
#include <common.h>
#include <malloc.h>
#include <mmc.h>
#include <rtc.h>
#include <part.h>
#include <spi.h>
#include <spi_flash.h>
#include <efuse.h>
#include <ingenic_soft_i2c.h>
#include <ingenic_soft_spi.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/compiler.h>
#include <linux/usb/composite.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/mtd.h>
#include <nand.h>
#include <ingenic_nand_mgr/nand_param.h>
#include "cloner.h"
#include "cloner_nand.c"


int i2c_program(struct cloner *cloner)
{
	int i = 0;
	struct i2c_args *i2c_arg = (struct i2c_args *)cloner->write_req->buf;
	struct i2c i2c;
	i2c.scl = i2c_arg->clk;
	i2c.sda = i2c_arg->data;
	i2c_init(&i2c);

	for(i=0;i<i2c_arg->value_count;i++) {
		char reg = i2c_arg->value[i] >> 16;
		unsigned char value = i2c_arg->value[i] & 0xff;
		i2c_write(&i2c,i2c_arg->device,reg,1,&value,1);
	}
	return 0;
}

#define MMC_BYTE_PER_BLOCK 512
extern int get_mmc_csd_perm_w_protect(void);
extern ulong mmc_erase_t(struct mmc *mmc, ulong start, lbaint_t blkcnt);
static int mmc_erase(struct cloner *cloner)
{
	int curr_device = 0;
	struct mmc *mmc = find_mmc_device(0);
	uint32_t blk, blk_end, blk_cnt;
	uint32_t erase_cnt = 0;
	int timeout = 30000;
	int i;
	int ret;

	if (!mmc) {
		printf("no mmc device at slot %x\n", curr_device);
		return -ENODEV;
	}

	mmc_init(mmc);
	if(get_mmc_csd_perm_w_protect()){
		printf("ERROR: MMC Init error ,can not be erase !!!!!!!!!\n");
		return -EPERM;
	}

	if (mmc_getwp(mmc) == 1) {
		printf("Error: card is write protected!\n");
		return -EPERM;
	}
	if (cloner->args->mmc_erase == MMC_ERASE_ALL) {
		blk = 0;
		blk_cnt = mmc->capacity / MMC_BYTE_PER_BLOCK;

		printf("MMC erase: dev # %d, start block # %d, count %u ... \n",
				curr_device, blk, blk_cnt);

		ret = mmc_erase_t(mmc, blk, blk_cnt);
		if (ret) {
			printf("mmc erase error\n");
			return ret;
		}
		ret = mmc_send_status(mmc, timeout);
		if(ret){
			printf("mmc erase error\n");
			return ret;
		}

		printf("mmc all erase ok, blocks %d\n", blk_cnt);
		return 0;
	} else if (cloner->args->mmc_erase != MMC_ERASE_PART) {
		return -EINVAL;
	}

	/*mmc part erase */
	erase_cnt = (cloner->args->mmc_erase_range_count >MMC_ERASE_CNT_MAX) ?
		MMC_ERASE_CNT_MAX : cloner->args->mmc_erase_range_count;

	for (i = 0; erase_cnt > 0; i++, erase_cnt--) {
		blk = cloner->args->mmc_erase_range[i].start / MMC_BYTE_PER_BLOCK;
		blk_end = cloner->args->mmc_erase_range[i].end / MMC_BYTE_PER_BLOCK;
		blk_cnt = blk_end - blk + 1;

		printf("MMC erase: dev # %d, start block # 0x%x, count 0x%x ... \n",
				curr_device, blk, blk_cnt);

		if ((blk % mmc->erase_grp_size) || (blk_cnt % mmc->erase_grp_size)) {
			printf("\n\nCaution! Your devices Erase group is 0x%x\n"
					"The erase block range would be change to "
					"0x" LBAF "~0x" LBAF "\n\n",
					mmc->erase_grp_size, (unsigned long)(blk & ~(mmc->erase_grp_size - 1)),
					(unsigned long)((blk + blk_cnt + mmc->erase_grp_size)
					 & ~(mmc->erase_grp_size - 1)) - 1);
		}

		ret = mmc_erase_t(mmc, blk, blk_cnt);
		if (ret) {
			printf("mmc erase error\n");
			return ret;
		}
		ret = mmc_send_status(mmc, timeout);
		if(ret){
			printf("mmc erase error\n");
			return ret;
		}

		printf("mmc part erase, part %d ok\n", i);
	}
	printf("mmc erase ok\n");
	return 0;
}

extern int mtd_nand_probe_burner(PartitionInfo *pinfo, nand_flash_param *nand_params,
		int nr_nand_args, int eraseall, void ** title_fill, int *size);
int cloner_init(struct cloner *cloner)
{
	if(cloner->args->use_nand_mgr) {
#ifdef CONFIG_JZ_NAND_MGR
		nand_probe_burner(&(cloner->args->PartInfo),
				&(cloner->args->nand_params[0]),
				cloner->args->nr_nand_args,
				cloner->args->nand_erase,cloner->args->offsets,cloner->args->nand_erase_count);
#endif	/*CONFIG_JZ_NAND_MGR*/
	}

	if(cloner->args->use_nand_mtd) {
#ifdef CONFIG_CMD_UBI
		mtd_nand_probe_burner(&cloner->args->MTDPartInfo,
				&cloner->args->nand_params,
				cloner->args->nr_nand_args,
				cloner->args->nand_erase,
				&cloner->spl_title,
				&cloner->spl_title_sz);
#endif	/*CONFIG_CMD_UBI*/
	}

	if (cloner->args->use_mmc) {
		if (cloner->args->mmc_erase) {
			mmc_erase(cloner);
		}
	}

	if(cloner->args->use_spi){
		printf("cloner->args->spi_args.rate:%d\n",cloner->args->spi_args.rate);
	}
	return 0;
}

int mmc_program(struct cloner *cloner,int mmc_index)
{
#define MMC_BYTE_PER_BLOCK 512
	int curr_device = 0;
	struct mmc *mmc = find_mmc_device(mmc_index);
	u32 blk = (cloner->cmd->write.partation + cloner->cmd->write.offset)/MMC_BYTE_PER_BLOCK;
	u32 cnt = (cloner->cmd->write.length + MMC_BYTE_PER_BLOCK - 1)/MMC_BYTE_PER_BLOCK;
	void *addr = (void *)cloner->write_req->buf;
	u32 n;

	if (!mmc) {
		printf("no mmc device at slot %x\n", curr_device);
		return -ENODEV;
	}

	//debug_cond(BURNNER_DEBUG,"\nMMC write: dev # %d, block # %d, count %d ... ",
	printf("MMC write: dev # %d, block # %d, count %d ... ",
			curr_device, blk, cnt);

	mmc_init(mmc);

	if (mmc_getwp(mmc) == 1) {
		printf("Error: card is write protected!\n");
		return -EPERM;
	}

	n = mmc->block_dev.block_write(curr_device, blk,
			cnt, addr);
	//debug_cond(BURNNER_DEBUG,"%d blocks write: %s\n",n, (n == cnt) ? "OK" : "ERROR");
	printf("%d blocks write: %s\n",n, (n == cnt) ? "OK" : "ERROR");

	if (n != cnt)
		return -EIO;

	if (cloner->args->write_back_chk) {
		mmc->block_dev.block_read(curr_device, blk,
				cnt, addr);
		debug_cond(BURNNER_DEBUG,"%d blocks read: %s\n",n, (n == cnt) ? "OK" : "ERROR");
		if (n != cnt)
			return -EIO;

		uint32_t tmp_crc = local_crc32(0xffffffff,addr,cloner->cmd->write.length);
		debug_cond(BURNNER_DEBUG,"%d blocks check: %s\n",n,(cloner->cmd->write.crc == tmp_crc) ? "OK" : "ERROR");
		if (cloner->cmd->write.crc != tmp_crc) {
			printf("src_crc32 = %08x , dst_crc32 = %08x\n",cloner->cmd->write.crc,tmp_crc);
			return -EIO;
		}
	}
	return 0;
}

int efuse_program(struct cloner *cloner)
{
	static int enabled = 0;
	if(!enabled) {
		efuse_init(cloner->args->efuse_gpio);
		enabled = 1;
	}
	u32 partation = cloner->cmd->write.partation;
	u32 length = cloner->cmd->write.length;
	void *addr = (void *)cloner->write_req->buf;
	u32 r = 0;

	if (!!(r = efuse_write(addr, length, partation))) {
		printf("efuse write error\n");
		return r;
	}
	return r;
}

extern unsigned int ssi_rate;
int spi_program(struct cloner *cloner)
{
	unsigned int bus = CONFIG_SF_DEFAULT_BUS;
	unsigned int cs = CONFIG_SF_DEFAULT_CS;
	unsigned int speed = CONFIG_SF_DEFAULT_SPEED;
	unsigned int mode = CONFIG_SF_DEFAULT_MODE;
	u32 offset = cloner->cmd->write.partation + cloner->cmd->write.offset;
	u32 length = cloner->cmd->write.length;
	int blk_size = cloner->args->spi_erase_block_siz;
	void *addr = (void *)cloner->write_req->buf;
	struct spi_args *spi_arg = &cloner->args->spi_args;
	unsigned int ret;
	int len = 0;
	struct spi_flash *flash;
	spi.enable = spi_arg->enable;
	spi.clk   = spi_arg->clk;
	spi.data_in  = spi_arg->data_in;
	spi.data_out  = spi_arg->data_out;
	spi.rate  = spi_arg->rate ;
	ssi_rate = spi.rate;

#ifdef CONFIG_JZ_SPI
	spi_init();
#endif
#ifdef CONFIG_INGENIC_SOFT_SPI
	spi_init_jz(&spi);
#endif


	if(flash == NULL){
		flash = spi_flash_probe(bus, cs, spi.rate, mode);
		if (!flash) {
			printf("Failed to initialize SPI flash at %u:%u\n", bus, cs);
			return 1;
		}
	}

	debug("the offset = %x\n",offset);
	debug("the length = %x\n",length);


	if (length%blk_size == 0){
		len = length;
		printf("the length = %x\n",length);
	}
	else{
		printf("the length = %x, is no enough %x\n",length,blk_size);
		len = (length/blk_size)*blk_size + blk_size;
	}

	ret = spi_flash_erase(flash, offset, len);
	printf("SF: %zu bytes @ %#x Erased: %s\n", (size_t)len, (u32)offset,
			ret ? "ERROR" : "OK");
	ret = spi_flash_write(flash, offset, len, addr);
	printf("SF: %zu bytes @ %#x write: %s\n", (size_t)len, (u32)offset,
			ret ? "ERROR" : "OK");


	if (cloner->args->write_back_chk) {
		spi_flash_read(flash, offset,len, addr);

		uint32_t tmp_crc = local_crc32(0xffffffff,addr,cloner->cmd->write.length);
		debug_cond(BURNNER_DEBUG,"%d blocks check: %s\n",len,(cloner->cmd->write.crc == tmp_crc) ? "OK" : "ERROR");
		if (cloner->cmd->write.crc != tmp_crc) {
			printf("src_crc32 = %08x , dst_crc32 = %08x\n",cloner->cmd->write.crc,tmp_crc);
			return -EIO;
		}
	}

#if debug
	int buf_debug[8*1024*1024];
	if (spi_flash_read(flash, 1024, /*len*/2048, buf_debug)) {
		printf("read failed\n");
		return -1;
	}
	int i = 0;
	for(i=0;i<4096;i++){
		printf("the debug[%d] = %x\n",i,buf_debug[i]);
	}

#endif
	return 0;
}

void handle_read(struct usb_ep *ep,struct usb_request *req)
{
}

void handle_write(struct usb_ep *ep,struct usb_request *req)
{
	struct cloner *cloner = req->context;

	if(req->status == -ECONNRESET) {
		cloner->ack = -ECONNRESET;
		return;
	}

	if (req->actual != req->length) {
		printf("write transfer length is errr,actual=%08x,length=%08x\n",req->actual,req->length);
		cloner->ack = -EIO;
		return;
	}

	if(cloner->cmd_type == VR_UPDATE_CFG) {
		cloner->ack = 0;
		printf("nand_erase:%d\n",cloner->args->nand_erase);
		printf("mmc_erase:%d\n",cloner->args->mmc_erase);
		printf("mmc_open_card:%d\n",cloner->args->mmc_open_card);
		return;
	}

	if (cloner->args->transfer_data_chk) {
		uint32_t tmp_crc = local_crc32(0xffffffff,req->buf,req->actual);
		if (cloner->cmd->write.crc != tmp_crc) {
			printf("crc is errr! src crc=%08x crc=%08x\n",cloner->cmd->write.crc,tmp_crc);
			cloner->ack = -EINVAL;
			return;
		}
	}
#define OPS(x,y) ((x<<16)|(y&0xffff))
	switch(cloner->cmd->write.ops) {
		case OPS(I2C,RAW):
			cloner->ack = i2c_program(cloner);
			break;
		case OPS(NAND,IMAGE):
			cloner->ack = nand_program(cloner);
			break;
		case OPS(NAND, MTD_RAW):
			cloner->ack = nand_mtd_raw_program(cloner);
			break;
		case OPS(NAND, MTD_UBI):
			cloner->ack = nand_mtd_ubi_program(cloner);
			break;
		case OPS(MMC,0):
		case OPS(MMC,1):
		case OPS(MMC,2):
			cloner->ack = mmc_program(cloner,cloner->cmd->write.ops & 0xffff);
			break;
		case OPS(MEMORY,RAW):
			cloner->ack = 0;
			break;
		case OPS(EFUSE,RAW):
			cloner->ack = efuse_program(cloner);
			break;
		case OPS(REGISTER,RAW):
			{
				volatile unsigned int *tmp = (void *)cloner->cmd->write.partation;
				if((unsigned)tmp > 0xb0000000 && (unsigned)tmp < 0xb8000000) {
					*tmp = *((int*)cloner->write_req->buf);
					cloner->ack = 0;
				} else {
					printf("OPS(REGISTER,RAW): not supported address.");
					cloner->ack = -ENODEV;
				}
			}
			break;
		case OPS(SPI,RAW):
			cloner->ack = spi_program(cloner);
			break;
		default:
			printf("ops %08x not support yet.\n",cloner->cmd->write.ops);
	}
#undef OPS
}

#ifdef CONFIG_FPGA
extern int do_udc_reset(void);
#endif

extern void burner_set_reset_tag(void);
void handle_cmd(struct usb_ep *ep,struct usb_request *req)
{
	struct cloner *cloner = req->context;
	if(req->status == -ECONNRESET) {
		cloner->ack = -ECONNRESET;
		return;
	}

	if (req->actual != req->length) {
		printf("cmd transfer length is err req->actual = %d, req->length = %d\n",
				req->actual,req->length);
		cloner->ack = -EIO;
		return;
	}

	union cmd *cmd = req->buf;
	debug_cond(BURNNER_DEBUG,"handle_cmd type=%x\n",cloner->cmd_type);
	switch(cloner->cmd_type) {
		case VR_UPDATE_CFG:
			cloner->args_req->length = cmd->update.length;
			usb_ep_queue(cloner->ep_out, cloner->args_req, 0);
			break;
		case VR_WRITE:
			if(cloner->buf_size < cmd->write.length) {
				cloner->buf_size = cmd->write.length;
				cloner->write_req->buf = realloc(cloner->write_req->buf,cloner->buf_size);
			}
			cloner->write_req->length = cmd->write.length;
			usb_ep_queue(cloner->ep_out, cloner->write_req, 0);
			break;
		case VR_INIT:
			if(!cloner->inited) {
				cloner->ack = -EBUSY;
				cloner_init(cloner);
				cloner->inited = 1;
				cloner->ack = 0;
			}
		case VR_READ:
			break;
		case VR_SYNC_TIME:
			cloner->ack = rtc_set(&cloner->cmd->rtc);
			break;
		case VR_GET_ACK:
		case VR_GET_CPU_INFO:
		case VR_SET_DATA_ADDR:
		case VR_SET_DATA_LEN:
			break;
		case VR_REBOOT:
#ifdef CONFIG_FPGA
			mdelay(1000);
			do_udc_reset();
			mdelay(10000);
#endif
			do_reset(NULL,0,0,NULL);
			break;
		case VR_POWEROFF:
			burner_set_reset_tag();
			do_reset(NULL,0,0,NULL);
			break;
	}
}

int f_cloner_setup_handle(struct usb_function *f,
		const struct usb_ctrlrequest *ctlreq)
{
	struct cloner *cloner = f->config->cdev->req->context;
	struct usb_request *req = cloner->ep0req;

	debug_cond(BURNNER_DEBUG,"vendor bRequestType %x,bRequest %x wLength %d\n",
			ctlreq->bRequestType,
			ctlreq->bRequest,
			ctlreq->wLength);

	if ((ctlreq->bRequestType & USB_TYPE_MASK) != USB_TYPE_VENDOR) {
		printf("Unkown RequestType 0x%x \n",ctlreq->bRequestType);
		cloner->ack = -ENOSYS;
		return -ENOSYS;
	}

	usb_ep_dequeue(cloner->ep0, cloner->ep0req);
	usb_ep_dequeue(cloner->ep_in, cloner->read_req);
	usb_ep_dequeue(cloner->ep_out, cloner->write_req);

	cloner->cmd_type = ctlreq->bRequest;
	req->length = ctlreq->wLength;
	req->complete = handle_cmd;

	switch (ctlreq->bRequest) {
		case VR_GET_CPU_INFO:
			strcpy(cloner->ep0req->buf,"BOOT47XX");
			break;
		case VR_GET_ACK:
			memcpy(cloner->ep0req->buf,&cloner->ack,sizeof(int));
			break;
		case VR_INIT:
			break;
		case VR_UPDATE_CFG:
		case VR_WRITE:
			cloner->ack = -EBUSY;
			break;
		case VR_SET_DATA_ADDR:
		case VR_SET_DATA_LEN:
			cloner->full_size = ctlreq->wIndex | ctlreq->wValue << 16;
			cloner->full_size_remainder = cloner->full_size;
			printf("cloner->full_size = %x\n", cloner->full_size);
			break;
	}

	return usb_ep_queue(cloner->ep0, cloner->ep0req, 0);
}

int f_cloner_bind(struct usb_configuration *c,
		struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct cloner *cloner = func_to_cloner(f);

	debug_cond(BURNNER_DEBUG,"f_cloner_bind\n");

	intf_desc.bInterfaceNumber = usb_interface_id(c, f);
	if(intf_desc.bInterfaceNumber < 0 )
		return intf_desc.bInterfaceNumber;

	cloner->ep0 = cdev->gadget->ep0;
	cloner->ep0req = cdev->req;
	cloner->gadget = cdev->gadget;
	cloner->ack = 0;
	cloner->cdev = cdev;

	cloner->cmd = (union cmd *)cloner->ep0req->buf;

	if (gadget_is_dualspeed(cdev->gadget)) {
		hs_bulk_in_desc.bEndpointAddress =
			fs_bulk_in_desc.bEndpointAddress;
		hs_bulk_out_desc.bEndpointAddress =
			fs_bulk_out_desc.bEndpointAddress;
	}

	cdev->req->context = cloner;

	cloner->ep_in = usb_ep_autoconfig(cdev->gadget, &fs_bulk_in_desc);
	cloner->ep_out = usb_ep_autoconfig(cdev->gadget, &fs_bulk_out_desc);

	cloner->write_req = usb_ep_alloc_request(cloner->ep_out,0);
	cloner->args_req = usb_ep_alloc_request(cloner->ep_out,0);
	cloner->read_req = usb_ep_alloc_request(cloner->ep_in,0);

	cloner->buf_size = 1024*1024;
	cloner->write_req->complete = handle_write;
	cloner->write_req->buf = malloc(1024*1024);
	cloner->write_req->length = 1024*1024;
	cloner->write_req->context = cloner;

	cloner->args_req->complete = handle_write;
	cloner->args_req->buf = cloner->args;
	cloner->args_req->length = ARGS_LEN;
	cloner->args_req->context = cloner;

	cloner->read_req->complete = handle_read;
	cloner->read_req->buf = malloc(1024*1024);
	cloner->read_req->length = 1024*1024;
	cloner->read_req->context = cloner;

	return 0;
}

int f_cloner_set_alt(struct usb_function *f,
		unsigned interface, unsigned alt)
{
	struct cloner *cloner = func_to_cloner(f);
	const struct usb_endpoint_descriptor *epin_desc,*epout_desc;
	int status = 0;

	debug_cond(BURNNER_DEBUG,"set interface %d alt %d\n",interface,alt);
	epin_desc = ep_choose(cloner->gadget,&hs_bulk_in_desc,&fs_bulk_in_desc);
	epout_desc = ep_choose(cloner->gadget,&hs_bulk_out_desc,&fs_bulk_out_desc);

	status += usb_ep_enable(cloner->ep_in,epin_desc);
	status += usb_ep_enable(cloner->ep_out,epout_desc);

	if (status < 0) {
		printf("usb enable ep in failed\n");
		goto failed;
	}

	cloner->ep_in->driver_data = cloner;
	cloner->ep_out->driver_data = cloner;
failed:
	return status;
}

void f_cloner_unbind(struct usb_configuration *c,struct usb_function *f)
{
}

void f_cloner_disable(struct usb_function *f)
{
	struct cloner *cloner = func_to_cloner(f);
	int status = 0;
	status += usb_ep_disable(cloner->ep_in);
	status += usb_ep_disable(cloner->ep_out);
	if (status < 0)
		printf("usb disable ep failed");
	return;
}

int cloner_function_bind_config(struct usb_configuration *c)
{
	int status = 0;
	struct cloner *cloner = calloc(sizeof(struct cloner),1);

	if (!cloner)
		return -ENOMEM;

	cloner->usb_function.name = "vendor burnner interface";
	cloner->usb_function.bind = f_cloner_bind;
	cloner->usb_function.hs_descriptors = hs_intf_descs;
	cloner->usb_function.descriptors = fs_intf_descs;
	cloner->usb_function.set_alt = f_cloner_set_alt;
	cloner->usb_function.setup = f_cloner_setup_handle;
	cloner->usb_function.strings= burn_intf_string_tab;
	cloner->usb_function.disable = f_cloner_disable;
	cloner->usb_function.unbind = f_cloner_unbind;

	cloner->args = malloc(ARGS_LEN);
	cloner->args->transfer_data_chk = 1;
	cloner->args->write_back_chk = 1;

	cloner->inited = 0;

	INIT_LIST_HEAD(&cloner->usb_function.list);
	bitmap_zero(cloner->usb_function.endpoints,32);

	status =  usb_add_function(c,&cloner->usb_function);
	if (status)
		free(cloner);
	return status;
}

int jz_cloner_add(struct usb_configuration *c)
{
	int id;

	id = usb_string_id(c->cdev);
	if (id < 0)
		return id;
	burner_intf_string_defs[0].id = id;
	intf_desc.iInterface = id;

	debug_cond(BURNNER_DEBUG,"%s: cdev: 0x%p gadget:0x%p gadget->ep0: 0x%p\n", __func__,
			c->cdev, c->cdev->gadget, c->cdev->gadget->ep0);

	return cloner_function_bind_config(c);
}

