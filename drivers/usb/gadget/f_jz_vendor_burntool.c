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
#include <part.h>
#include <crc.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/compiler.h>
#include <linux/usb/composite.h>

#define BURNNER_DEBUG 1

/*bootrom stage request*/
#define VEN_GET_CPU_INFO	0x00
#define VEN_SET_DATA_ADDR	0x01
#define VEN_SET_DATA_LEN	0x02
#define VEN_FLUSH_CACHE		0x03
#define VEN_PROG_STAGE1		0x04
#define VEN_PROG_STAGE2		0x05
/*firmware stage request*/
#define VEN_GET_ACK		0x10
#define VEN_CTL			0x11
#define VEN_WRITE		0x12
#define VEN_READ		0x13

enum medium_type {
	MEMORY = 0,
	NAND,
	MMC,
	NOR,
};

enum data_type {
	RAW = 0,
	OOB,
	IMAGE,
};

struct burner_pipe {
	struct usb_ep		*ep;
	struct usb_request	*epreq;
	void			*buf;
	unsigned int		buf_length;
	unsigned int		crc;
};

struct burner_trans_args {
	unsigned int addr;
	unsigned int length;
	unsigned int crc;
};

struct jz_burner {
	struct usb_function     usb_function;
	struct usb_composite_dev *cdev;		/*Copy of config->cdev*/
	struct usb_gadget	*gadget;	/*Copy of cdev->gadget*/
	struct usb_ep		*ep0;		/*Copy of gadget->ep0*/
	struct usb_request	*ep0req;	/*Copy of cdev->req*/
	struct burner_pipe		*bulk_in;
	struct burner_pipe		*bulk_out;

	/*BURN INFO*/
	unsigned int		has_crc:1;
	void			*args_buf;
	enum	medium_type	medium_type;
	enum	data_type	data_type;
	unsigned int		request_type;
	unsigned long long	offset_base;
	unsigned int		offset;
	unsigned int		length;

	/*BURN STATE*/
	unsigned int		ack_status;

	/*EP INFO*/
	unsigned int		bulk_in_enabled:1;
	unsigned int		bulk_out_enabled:1;
	int (*vir_enable_ep)(struct usb_ep *usb_ep,
			const struct usb_endpoint_descriptor *desc);
};

static const char burntool_name[] = "INGENIC VENDOR BURNNER";

static struct usb_string burner_intf_string_defs[] = {
	[0].s = burntool_name,
	{}
};

static struct usb_gadget_strings  burn_intf_string = {
	.language = 0x0409, /* en-us */
	.strings = burner_intf_string_defs,
};

static struct usb_gadget_strings  *burn_intf_string_tab[] = {
	&burn_intf_string,
	NULL,
};

static struct usb_interface_descriptor intf_desc = {
	.bLength =              sizeof(intf_desc),
	.bDescriptorType =      USB_DT_INTERFACE,
	.bNumEndpoints =        2,
};

static struct usb_endpoint_descriptor fs_bulk_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN|0x1,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_bulk_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT|0x1,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor hs_bulk_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN|0x1,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_bulk_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT|0x1,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_descriptor_header *fs_intf_descs[] = {
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &fs_bulk_out_desc,
	(struct usb_descriptor_header *) &fs_bulk_in_desc,
	NULL,
};

static struct usb_descriptor_header *hs_intf_descs[] = {
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &hs_bulk_out_desc,
	(struct usb_descriptor_header *) &hs_bulk_in_desc,
	NULL,
};

extern int f_ep_vir_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc);

static inline struct jz_burner *func_to_jz_burner(struct usb_function *f)
{
	return container_of(f, struct jz_burner, usb_function);
}

static int burner_get_cpu_info(struct jz_burner *jz_burner,
		u16 wValue,u16 wIndex,u16 wLength)
{
	struct usb_request *req = jz_burner->ep0req;
#ifdef CONFIG_BURNER_CPU_INFO
	static char *buf = CONFIG_BURNER_CPU_INFO;
#else
	static char *buf = "BOOT47XX";
#endif
	printf("burner_get_cpu_info\n");
	if (wLength) {
		req->length = strlen(buf);
		if (req->length > wLength)
			req->length = wLength;
		memcpy(req->buf,buf,req->length);
		return req->length;
	}

	jz_burner->ack_status = -EINVAL;
	return -EINVAL;
}

static int burner_get_ack(struct jz_burner *jz_burner,
		u16 wValue,u16 wIndex,u16 wLength)
{
	struct usb_request *req = jz_burner->ep0req;
	int length = wLength > sizeof(jz_burner->ack_status) ?
		sizeof(jz_burner->ack_status) : wLength;

	if (length) {
		memcpy(req->buf,&jz_burner->ack_status,length);
		req->length = length;
		return length;
	}

	jz_burner->ack_status = -EINVAL;
	return -EINVAL;
}

enum ctl_type {
	BOARD_REQ = 0,
	NAND_REQ,
	MMC_REQ,
};

static void handle_default_complete(struct usb_ep *ep,
				struct usb_request *req)
{
	return;
}

/*FIXME*/
static int burner_control(struct jz_burner *jz_burner,
		u16 wValue,u16 wIndex,u16 wLength)
{
	struct usb_request *req = jz_burner->ep0req;
	int length = 0;

	req->length = 0;
	req->buf = (void *)~(0);
	req->complete = handle_default_complete;

	switch (wValue) {
	case BOARD_REQ:
		/*Board control : reboot,board cfg, flush cache and so on*/
		break;
	case NAND_REQ:
		/*Nand control : nand query,init,erase and so on*/
		break;
	case MMC_REQ:
		/*MMC control : sd open card and so on*/
		break;
	default:
		printf("Unkown bunner control command :%d\n",wValue);
		return -EINVAL;
	}

	return length;
}

int nand_program(struct jz_burner *jz_burner,enum data_type) __attribute__((weak,
			alias("nand_program_default")));
int mmc_program(struct jz_burner *jz_burner,enum data_type) __attribute__((weak,
			alias("mmc_program_default")));
int nor_program(struct jz_burner *jz_burner,enum data_type) __attribute__((weak,
			alias("nor_program_default")));

int nand_program_default(struct jz_burner *jz_burner,
		enum data_type data_type)
{
	return 0; //0 success or errno
}

int curr_device = 0;
#define MMC_BYTE_PER_BLOCK 512
int mmc_program_default(struct jz_burner *jz_burner,
		enum data_type data_type)
{
	printf("start burn mmc\n");

	struct mmc *mmc = find_mmc_device(0);
	u32 blk = (jz_burner->offset_base + jz_burner->offset)/MMC_BYTE_PER_BLOCK;
	u32 cnt = jz_burner->length/MMC_BYTE_PER_BLOCK;
	void *addr = (void *)jz_burner->bulk_out->buf;
	u32 n;

	if (!mmc) {
		printf("no mmc device at slot %x\n", curr_device);
		return -ENODEV;
	}

	printf("\nMMC write: dev # %d, block # %d, count %d ... ",
			curr_device, blk, cnt);

	mmc_init(mmc);

	if (mmc_getwp(mmc) == 1) {
		printf("Error: card is write protected!\n");
		return -EPERM;
	}

	n = mmc->block_dev.block_write(curr_device, blk,
			cnt, addr);
	printf("%d blocks write: %s\n",
			n, (n == cnt) ? "OK" : "ERROR");
	if (n != cnt)
		return -EIO;
	return 0;
}

int nor_program_default(struct jz_burner *jz_burner,
		enum data_type data_type)
{
	return 0;
}

static void handle_write(struct usb_ep *ep,
		struct usb_request *req)
{
	struct jz_burner *jz_burner = req->context;
	struct burner_pipe *bulk_out = jz_burner->bulk_out;
	int ret;

	debug_cond(BURNNER_DEBUG,"handle write\n");
	if (jz_burner->has_crc) {
		if (bulk_out->crc != crc32(0,req->buf,req->actual)) {
			printf("crc is errr\n");
			jz_burner->ack_status = -EIO;
			return;
		}
	} else if (req->actual != req->length) {
		printf("transfer length is errr\n");
		jz_burner->ack_status = -EIO;
		return;
	}

	switch (jz_burner->medium_type) {
	case NAND:
		ret = nand_program(jz_burner,jz_burner->data_type);
		break;
	case MMC:
		ret = mmc_program(jz_burner,jz_burner->data_type);
		break;
	case NOR:
		ret = nor_program(jz_burner,jz_burner->data_type);
		break;
	default:
		ret = -EMEDIUMTYPE;
		printf("Unkown bunner program command :%d\n",jz_burner->medium_type);
	}

	jz_burner->ack_status = ret;
	return;
}

static void parse_write_args(struct usb_ep *ep,
		struct usb_request *req)
{
	struct jz_burner *jz_burner = req->context;
	struct burner_pipe *bulk_out = jz_burner->bulk_out;
	struct usb_request *w_req = bulk_out->epreq;
	struct usb_ep	*w_ep = bulk_out->ep;
	int ret;
	unsigned int foffset_l,offset,length,crc32;
	unsigned int foffset_b;

	if (req->actual < req->length) {
		printf("burnner write args transfer error,actual %d,length %d\n",
				req->actual,
				req->length);
		jz_burner->ack_status = req->status;
		return;
	}

	foffset_l = ((int*)req->buf)[0];
	foffset_b = ((int*)req->buf)[1];
	offset = ((int *)req->buf)[2];
	length = ((int *)req->buf)[3];
	crc32 = ((int *)req->buf)[4];

	foffset_b = le32_to_cpu(foffset_b);
	foffset_l = le32_to_cpu(foffset_l);
	length = le32_to_cpu(length);
	offset = le32_to_cpu(offset);
	crc32 = le32_to_cpu(crc32);

	jz_burner->offset_base = ((foffset_b << 32)|foffset_l);
	jz_burner->length = length;
	jz_burner->offset = offset;
	bulk_out->crc = crc32;
	jz_burner->ack_status = -EBUSY;

	debug_cond(BURNNER_DEBUG,
			"file offset %lld,medium offset %d,length %d,crc32 %x\n",
			jz_burner->offset_base,
			jz_burner->offset,
			jz_burner->length,
			bulk_out->crc);

	if (bulk_out->buf) {
		if (bulk_out->buf_length < length) {
			bulk_out->buf = realloc(bulk_out->buf,length);
			bulk_out->buf_length = length;
		}
	} else {
		bulk_out->buf = malloc(length);
		bulk_out->buf_length = length;
	}

	if (!bulk_out->buf) {
		printf("%s no mem\n",__func__);
		bulk_out->buf_length = 0;
		jz_burner->ack_status = -ENOMEM;
		return;
	}

	w_req->buf = bulk_out->buf;
	w_req->length = jz_burner->length;
	w_req->complete = handle_write;
	w_req->context = jz_burner;
	ret = usb_ep_queue(w_ep,w_req,0);
	if (ret) {
		printf("Usb queue bulk out failed\n");
		jz_burner->ack_status = ret;
		return;
	}
	return;
}

static int write_args_trans_prepare(struct jz_burner *jz_burner,
		u16 wValue,u16 wIndex,u16 wLength)
{
	struct usb_request *req = jz_burner->ep0req;

	debug_cond(BURNNER_DEBUG,"Write medium_type %d,data_type %d length %d\n",
			wValue,wIndex,wLength);
	jz_burner->medium_type = wValue;
	jz_burner->data_type = wIndex;
	jz_burner->request_type = VEN_WRITE;
	req->length = wLength;
	req->complete = parse_write_args;
	req->context = jz_burner;
	jz_burner->ack_status = -EBUSY;

	return wLength;
}

int burner_nand_read(struct jz_burner *jz_burner, void *buf, int length,
		unsigned long long offset) __attribute__((weak,alias("nand_read_default")));
int burner_mmc_read(struct jz_burner *jz_burner, void *buf, int length,
		unsigned long long offset) __attribute__((weak,alias("mmc_read_default")));
int burner_nor_read(struct jz_burner *jz_burner, void *buf, int length,
		unsigned long long offset) __attribute__((weak,alias("nor_read_default")));

int nand_read_default(struct jz_burner *jz_burner,
		void *buf, int length,unsigned long long offset)
{
	return 0;	//length success or errno
}
int mmc_read_default(struct jz_burner *jz_burner,
		void *buf, int length,unsigned long long offset)
{
	return 0;
}
int nor_read_default(struct jz_burner *jz_burner,
		void *buf, int length,unsigned long long offset)
{
	return 0;
}

static void bunner_read_complete(struct usb_ep *ep,
		struct usb_request *req)
{
	struct jz_burner *jz_burner = req->context;
	jz_burner->ack_status = 0;
	return;
}

static void handle_read(struct usb_ep *ep,
		struct usb_request *req)
{
	struct jz_burner *jz_burner = req->context;
	struct burner_pipe *bulk_in = jz_burner->bulk_in;
	int offset_l = ((int *)req->buf)[0];
	unsigned long long offset_b = ((int *)req->buf)[1];
	int length = ((int *)req->buf)[2];
	int ret = 0;

	offset_b = le32_to_cpu(offset_b);
	offset_l = le32_to_cpu(offset_l);
	length = le32_to_cpu(length);
	jz_burner->offset_base = ((offset_b << 32)|offset_l);
	jz_burner->length = length;
	printf("Read medium %d frome %lld, length %d\n",
			jz_burner->medium_type,
			jz_burner->offset_base,
			jz_burner->length);
	if (bulk_in->buf) {
		if (bulk_in->buf_length < length) {
			bulk_in->buf = realloc(bulk_in->buf,length);
			bulk_in->buf_length = length;
		}
	} else {
		bulk_in->buf = malloc(length);
		bulk_in->buf_length = length;
	}

	if (!bulk_in->buf) {
		printf("%s no mem\n",__func__);
		bulk_in->buf_length = 0;
		jz_burner->ack_status = -ENOMEM;
		return;
	}

	switch (jz_burner->medium_type) {
	case NAND:
		length = burner_nand_read(jz_burner,bulk_in->buf,
				jz_burner->length,jz_burner->offset_base);
		break;
	case MMC:
		length = burner_mmc_read(jz_burner,bulk_in->buf,
				jz_burner->length,jz_burner->offset_base);
		break;
	case NOR:
		length = burner_nor_read(jz_burner,bulk_in->buf,
				jz_burner->length,jz_burner->offset_base);
		break;
	default:
		length = -EMEDIUMTYPE;
		printf("Cannooooooooot happen %d\n",__LINE__);
	}

	if (length < 0) {
		jz_burner->ack_status = length;
		return;
	}

	bulk_in->epreq->buf = bulk_in->buf;
	bulk_in->epreq->length = length;
	bulk_in->epreq->context = jz_burner;
	bulk_in->epreq->complete = bunner_read_complete;
	ret= usb_ep_queue(bulk_in->ep,bulk_in->epreq,0);
	if (ret) {
		debug("Usb queue bulk in failed\n");
		jz_burner->ack_status = ret;
		bulk_in->epreq->status = 0;
	}
	return;
}

static int read_args_trans_prepare(struct jz_burner *jz_burner,
		u16 wValue,u16 wIndex,u16 wLength)
{
	struct usb_request *req = jz_burner->ep0req;
	jz_burner->medium_type = wValue;
	jz_burner->request_type = VEN_READ;
	req->length = wLength;
	req->buf = jz_burner->args_buf;
	req->complete = handle_read;
	req->context = jz_burner;
	jz_burner->ack_status = -EBUSY;

	return wLength;
}

static int f_vendor_burner_setup_handle(struct usb_function *f,
		const struct usb_ctrlrequest *ctlreq)
{
	struct usb_gadget *gadget = f->config->cdev->gadget;
	struct jz_burner *jz_burner = f->config->cdev->req->context;
	struct usb_request *req = jz_burner->ep0req;
	u16 wLength = le16_to_cpu(ctlreq->wLength);
	u16 wValue = le16_to_cpu(ctlreq->wValue);
	u16 wIndex = le16_to_cpu(ctlreq->wIndex);
	int length = 0;

	req->length = 0;
	debug_cond(BURNNER_DEBUG,"vendor bRequestType %x,bRequest %x wLength %d\n",
			ctlreq->bRequestType,
			ctlreq->bRequest,
			ctlreq->wLength);
	if ((ctlreq->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		switch (ctlreq->bRequest) {
		case VEN_GET_CPU_INFO:
			length = burner_get_cpu_info(jz_burner,wValue,
					wIndex,wLength);
			break;
		case VEN_SET_DATA_ADDR:
		case VEN_SET_DATA_LEN:
		case VEN_FLUSH_CACHE:
		case VEN_PROG_STAGE1:
		case VEN_PROG_STAGE2:
			break;
		case VEN_GET_ACK:
			length = burner_get_ack(jz_burner,wValue,
					wIndex,wLength);
			break;
		case VEN_CTL:
			length = burner_control(jz_burner,wValue,
					wIndex,wLength);
			break;
		case VEN_WRITE:
			length = write_args_trans_prepare(jz_burner,wValue,
					wIndex,wLength);
			break;
		case VEN_READ:
			length = read_args_trans_prepare(jz_burner,wValue,
					wIndex,wLength);
			break;
		default:
			printf("Unkown Vendor Request 0x%x\n",ctlreq->bRequest);
			return -ENOSYS;
		}
	} else {
		printf("Unkown RequestType 0x%x \n",
				ctlreq->bRequestType);
		jz_burner->ack_status = -ENOSYS;
		return -ENOSYS;
	}

	if (length >= 0) {
		length = usb_ep_queue(gadget->ep0, req, 0);
		if (length) {
			printf("ep_queue error--> %x\n", length);
			jz_burner->ack_status = length;
			req->status = 0;
		}
	}
	return length;
}

static int alloc_bulk_endpoints(struct jz_burner *jz_burner,
		struct usb_endpoint_descriptor *in,
		struct usb_endpoint_descriptor *out)
{
	struct usb_composite_dev *cdev = jz_burner->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int ret;

	jz_burner->bulk_in = calloc(sizeof(struct burner_pipe),1);
	jz_burner->bulk_out = calloc(sizeof(struct burner_pipe),1);
	if (!jz_burner->bulk_in || !jz_burner->bulk_out) {
		printf("%s:%d :nomem\n",__func__,__LINE__);
		ret = -ENOMEM;
		goto bulk_alloc_err;
	}

	ep = usb_ep_autoconfig(cdev->gadget, in);
	if (!ep) {
		printf("usb_ep_autoconfig for ep_in failed\n");
		ret = -ENODEV;
		goto in_ep_err;
	}
	jz_burner->bulk_in->ep = ep;
	debug("usb_ep_autoconfig for bulk_in got %s\n", ep->name);

	ep = usb_ep_autoconfig(cdev->gadget, out);
	if (!ep) {
		printf("usb_ep_autoconfig for ep_out failed\n");
		ret = -ENODEV;
		goto out_ep_err;
	}
	jz_burner->bulk_out->ep = ep;
	debug("usb_ep_autoconfig for bulk_out got %s\n", ep->name);

	req =  usb_ep_alloc_request(jz_burner->bulk_in->ep,0);
	if (!req) {
		printf("usb_ep_alloc_request for ep_in failed\n");
		ret = -ENOMEM;
		goto in_req_err;
	}
	req->complete = handle_default_complete;
	req->buf = NULL;
	jz_burner->bulk_in->epreq = req;

	req =  usb_ep_alloc_request(jz_burner->bulk_out->ep,0);
	if (!req) {
		printf("usb_ep_alloc_request for ep_out failed\n");
		ret = -ENOMEM;
		goto out_req_err;
	}
	req->complete = handle_default_complete;
	req->buf = NULL;
	jz_burner->bulk_out->epreq = req;

	jz_burner->bulk_in->buf = NULL;
	jz_burner->bulk_in->buf_length = 0;
	jz_burner->bulk_out->buf = NULL;
	jz_burner->bulk_out->buf_length = 0;

	return 0;

out_req_err:
	usb_ep_free_request(jz_burner->bulk_out->ep,
			jz_burner->bulk_in->epreq);
in_req_err:
out_ep_err:
	usb_ep_autoconfig_reset(cdev->gadget);
in_ep_err:
bulk_alloc_err:
	free(jz_burner->bulk_in);
	free(jz_burner->bulk_out);

	return ret;
}

static int f_vendor_burner_bind(struct usb_configuration *c,
		struct usb_function *f)
{
	int id,ret;
	struct usb_composite_dev *cdev = c->cdev;
	struct jz_burner *jz_burner = func_to_jz_burner(f);

	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	intf_desc.bInterfaceNumber = id;

	jz_burner->ep0 = cdev->gadget->ep0;
	jz_burner->ep0req = cdev->req;
	jz_burner->gadget = cdev->gadget;
	jz_burner->bulk_out_enabled = 0;
	jz_burner->bulk_in_enabled = 0;
	jz_burner->ack_status = 0;
	jz_burner->cdev = cdev;
	jz_burner->args_buf = malloc(5*sizeof(int));

	ret = alloc_bulk_endpoints(jz_burner,
			&fs_bulk_in_desc,&fs_bulk_out_desc);
	if (ret)
		return ret;

	if (gadget_is_dualspeed(cdev->gadget)) {
		 hs_bulk_in_desc.bEndpointAddress =
			 fs_bulk_in_desc.bEndpointAddress;
		 hs_bulk_out_desc.bEndpointAddress =
			 fs_bulk_out_desc.bEndpointAddress;
	}
	cdev->req->context = jz_burner;

	return 0;
}

void free_bulk_endpoints(struct usb_configuration *c,
		 struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct jz_burner *jz_burner = func_to_jz_burner(f);

	free(f->hs_descriptors);
	free(f->descriptors);
	usb_ep_free_request(jz_burner->bulk_out->ep,
			jz_burner->bulk_out->epreq);
	usb_ep_free_request(jz_burner->bulk_in->ep,
			jz_burner->bulk_in->epreq);
	usb_ep_autoconfig_reset(cdev->gadget);

	free(jz_burner->bulk_in);
	free(jz_burner->bulk_out);
	return;
}


static void f_vendor_burner_unbind(struct usb_configuration *c,
		struct usb_function *f)
{
	free_bulk_endpoints(c,f);
	return;
}

static int f_vendor_burner_set_alt(struct usb_function *f,
		unsigned interface, unsigned alt)
{
	struct jz_burner *jz_burner = func_to_jz_burner(f);
	struct usb_ep *in_ep = jz_burner->bulk_in->ep;
	struct usb_ep *out_ep = jz_burner->bulk_out->ep;
	const struct usb_endpoint_descriptor *epdesc = NULL;
	int status = 0;

	debug("set interface %d alt %d\n",interface,alt);

	/*Bulk in endpoint set alt*/
	if (in_ep->driver_data) {
		debug("set alt when ep in enable\n");
		status = usb_ep_disable(in_ep);
		if (status) {
			printf("usb disable ep in failed\n");
			goto failed;
		}
		in_ep->driver_data = NULL;
	}
	epdesc = ep_choose(jz_burner->gadget,&hs_bulk_in_desc,
			&fs_bulk_in_desc);
	if (jz_burner->vir_enable_ep)
		status = jz_burner->vir_enable_ep(in_ep,epdesc);
	else {
		status = usb_ep_enable(in_ep,epdesc);
	}
	if (status < 0) {
		printf("usb enable ep in failed\n");
		goto failed;
	}
	in_ep->driver_data = jz_burner;

	/*Bulk out endpoint set alt*/
	if (out_ep->driver_data) {
		debug("set alt when ep out enable\n");
		status = usb_ep_disable(out_ep);
		if (status) {
			printf("usb disable ep out failed\n");
			goto failed;
		}
		out_ep->driver_data = NULL;
	}
	epdesc = ep_choose(jz_burner->gadget,&hs_bulk_out_desc,
			&fs_bulk_out_desc);
	if (jz_burner->vir_enable_ep)
		status = jz_burner->vir_enable_ep(out_ep,epdesc);
	else {
		status = usb_ep_enable(out_ep, epdesc);
	}
	if (status < 0) {
		printf("usb enable ep out failed\n");
		goto failed;
	}
	out_ep->driver_data = jz_burner;
failed:
	jz_burner->vir_enable_ep = NULL;
	return status;
}

static void f_vendor_burner_disable(struct usb_function *f)
{
	debug("dump function\n");
}

static int burnfunction_bind_config(struct usb_configuration *c)
{
	struct jz_burner *jz_burner;
	int status = 0;

	jz_burner = calloc(sizeof(struct jz_burner),1);
	if (!jz_burner)
		return -ENOMEM;

	jz_burner->usb_function.name = "vendor burnner interface";
	jz_burner->usb_function.bind = f_vendor_burner_bind;
	jz_burner->usb_function.hs_descriptors = hs_intf_descs;
	jz_burner->usb_function.descriptors = fs_intf_descs;
	jz_burner->usb_function.set_alt = f_vendor_burner_set_alt;
	jz_burner->usb_function.disable = f_vendor_burner_disable;
	jz_burner->usb_function.unbind = f_vendor_burner_unbind;
	jz_burner->usb_function.setup = f_vendor_burner_setup_handle;
	jz_burner->usb_function.strings= burn_intf_string_tab;
	jz_burner->vir_enable_ep = f_ep_vir_enable;
	INIT_LIST_HEAD(&jz_burner->usb_function.list);
	bitmap_zero(jz_burner->usb_function.endpoints,32);

	status =  usb_add_function(c,&jz_burner->usb_function);
	if (status)
		free(jz_burner);
	return status;
}

int jz_vendor_burner_add(struct usb_configuration *c)
{
	int id;

	id = usb_string_id(c->cdev);
	if (id < 0)
		return id;
	burner_intf_string_defs[0].id = id;
	intf_desc.iInterface = id;

	debug("%s: cdev: 0x%p gadget:0x%p gadget->ep0: 0x%p\n", __func__,
			c->cdev, c->cdev->gadget, c->cdev->gadget->ep0);

	return burnfunction_bind_config(c);
}
