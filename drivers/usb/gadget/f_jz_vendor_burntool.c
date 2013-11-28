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

/*bootrom stage request*/
#define VEN_GET_CPU_INFO	0x00
#define VEN_SET_DATA_ADDR	0x01
#define VEN_SET_DATA_LEN	0x02
#define VEN_FLUSH_CACHE		0x03
#define VEN_PROG_STAGE1		0x04
#define VEN_PROG_STAGE2		0x05
/*firmware stage request*/
#define VEN_GET_ACK		0x10
#define VEN_SET_TRANS_ARGS	0x11
#define VEN_CTL			0x12
#define VEN_WRITE		0x13
#define VEN_READ		0x14


enum transfer_state {
	TRANSFER_IDLE = 0,
	TRANSFER_PREPARE,
	TRANSFER_BUSY,
	TRANSFER_SUCCESSS,
	TRANSFER_ERROR,
};

enum medium_type {
	NAND = 0,
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
	unsigned int	*buf_addr;
	unsigned int		buf_length;
	unsigned int		crc;
	enum transfer_state	transfer_state;
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
	unsigned int		has_crc:1;
	enum	medium_type	pg_medium_type;
	enum	data_type	pg_data_type;
	enum	medium_type	rd_medium_type;
	unsigned long long	pg_offset;
	unsigned int		pg_length;
	unsigned long long	rd_offset;
	unsigned int		rd_length;
	unsigned int		bulk_in_enabled:1;
	unsigned int		bulk_out_enabled:1;
	unsigned int		ack_status;
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
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_bulk_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor hs_bulk_in_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_IN,
	.bmAttributes           = USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize         = __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_bulk_out_desc = {
	.bLength                = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType        = USB_DT_ENDPOINT,
	.bEndpointAddress       = USB_DIR_OUT,
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

	if (wLength) {
		memcpy(req->buf,buf,wLength);
		req->length = wLength;
		return wLength;
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

static void bunner_crc_calculate(struct usb_ep *ep,
		struct usb_request *req) {
	struct jz_burner *jz_burner = req->context;
	struct burner_pipe *bulk_out = jz_burner->bulk_out;

	if (bulk_out->transfer_state != TRANSFER_BUSY) {
		bulk_out->transfer_state = TRANSFER_ERROR;
		jz_burner->ack_status = -EBUSY;
		return;
	}
	bulk_out->transfer_state = TRANSFER_SUCCESSS;

	if (req->actual != bulk_out->buf_length)	//FIXME
		debug("Bulk out transfer less than request\n");
	if (jz_burner->has_crc) {	/*bulk data have crc code*/
		/* crc polynomial  x^16+x^12+x^5+1 */
		if (bulk_out->crc != cyg_crc16(req->buf,req->actual)) {
			jz_burner->ack_status = 0;
		} else {
			bulk_out->transfer_state = TRANSFER_ERROR;
			jz_burner->ack_status = -EIO;
		}
	} else {
		jz_burner->ack_status = 0;	//success
	}
	return;
}


#define IS_ALIGNED(x, a)                (((x) & ((typeof(x))(a) - 1)) == 0)
static int addr_is_invalid(unsigned int *addr, unsigned int length)
{
	if ((unsigned)addr < 0x8000000)
		return -EFAULT;
	if ((unsigned)addr > (0x90000000 - length) && (unsigned)addr < 0xa0000000)
		return -EFAULT;
	if ((unsigned)addr > (0xb000000 - length))
		return -EFAULT;
	if (!IS_ALIGNED((unsigned)addr, 4))
		return -EADDRNOTAVAIL;

	return 0;
}

/*Complete function for bunner_set_trans_args data phase*/
static void bunner_start_transfer(struct usb_ep *ep,
		struct usb_request *req)
{
	struct jz_burner *jz_burner = req->context;
	struct burner_pipe *bulk_out = jz_burner->bulk_out;
	struct burner_trans_args *trans_args = req->buf;
	int error;

	if (trans_args) {
		/*Store transer args*/
		bulk_out->buf_length = le32_to_cpu(trans_args->length);
		bulk_out->buf_addr = (unsigned *)le32_to_cpu(trans_args->addr);
		bulk_out->crc = le32_to_cpu(trans_args->crc);

		if (bulk_out->transfer_state != TRANSFER_PREPARE) {
			bulk_out->transfer_state = TRANSFER_ERROR;
			jz_burner->ack_status = -EBUSY;
			return;
		}

		bulk_out->transfer_state = TRANSFER_BUSY;
		if (!bulk_out->buf_length) {
			jz_burner->ack_status = -EINVAL;
			return;
		}
		if ((error = addr_is_invalid(bulk_out->buf_addr,
					bulk_out->buf_length)) != 0) {
			jz_burner->ack_status = error;
			return;
		}

		/*Init one bulk out transfer*/
		bulk_out->epreq->complete = bunner_crc_calculate;
		bulk_out->epreq->context = jz_burner;
		bulk_out->epreq->length = bulk_out->buf_length;
		bulk_out->epreq->buf = bulk_out->buf_addr;
		bulk_out->epreq->status = 0;
		error = usb_ep_queue(bulk_out->ep,bulk_out->epreq,0);
		if (error) {
			debug("Bulk out queue request failed\n");
			bulk_out->transfer_state = TRANSFER_PREPARE;
			jz_burner->ack_status = -EIO;
		}
		return;
	}

	jz_burner->ack_status = -EINVAL;
	return;
}

static int burner_set_trans_args(struct jz_burner *jz_burner,
		u16 wValue,u16 wIndex,u16 wLength)
{
	struct usb_request *req = jz_burner->ep0req;
	struct burner_pipe *bulk_out = jz_burner->bulk_out;

	if (!wLength) {
		jz_burner->ack_status = -EINVAL;
		return -EINVAL;
	}

	if (bulk_out->transfer_state == TRANSFER_IDLE ||
			bulk_out->transfer_state == TRANSFER_ERROR ||
			bulk_out->transfer_state == TRANSFER_SUCCESSS) {
		bulk_out->transfer_state = TRANSFER_PREPARE;
		req->length = wLength;
		req->status = 0;
		req->complete= bunner_start_transfer;
		req->context = jz_burner;
		return wLength;
	}

	bulk_out->transfer_state = TRANSFER_ERROR;
	jz_burner->ack_status = -EBUSY;
	return -EBUSY;
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
int mmc_program_default(struct jz_burner *jz_burner,
		enum data_type data_type)
{
	return 0;
}
int nor_program_default(struct jz_burner *jz_burner,
		enum data_type data_type)
{
	return 0;
}


static void burner_start_program(struct usb_ep *ep,
		struct usb_request *req)
{
	struct jz_burner *jz_burner = req->context;
	int ret = 0;
	int offset_l = ((int *)req->buf)[0];
	unsigned long long offset_b = ((int*)req->buf)[1];
	int length = ((int *)req->buf)[2];

	offset_b = le32_to_cpu(offset_b);
	offset_l = le32_to_cpu(offset_l);
	length = le32_to_cpu(length);
	jz_burner->rd_offset = ((offset_b << 32)|offset_l);
	jz_burner->rd_length = length;
	printf("Program medium %d frome %lld, length %d\n",
			jz_burner->rd_medium_type,
			jz_burner->rd_offset,
			jz_burner->rd_length);

	switch (jz_burner->pg_medium_type) {
	case NAND:
		ret = nand_program(jz_burner,jz_burner->pg_data_type);
		break;
	case MMC:
		ret = mmc_program(jz_burner,jz_burner->pg_data_type);
		break;
	case NOR:
		ret = nor_program(jz_burner,jz_burner->pg_data_type);
		break;
	default:
		printf("Unkown bunner program command :%d\n",jz_burner->pg_medium_type);
	}

	jz_burner->ack_status = ret;
	return;
}

static int burner_program(struct jz_burner *jz_burner,
		u16 wValue,u16 wIndex,u16 wLength)
{
	struct usb_request *req = jz_burner->ep0req;

	switch (wValue) {
	case NAND:
	case MMC:
	case NOR:
	default:
		printf("Unkown bunner program medium type :%d\n",wValue);
		return -EINVAL;
	}

	jz_burner->pg_medium_type = wValue;
	jz_burner->pg_data_type = wIndex;
	req->length = wLength;
	req->complete = burner_start_program;
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
	/*Now do noting*/
	return;
}

static void burner_start_read(struct usb_ep *ep,
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
	jz_burner->rd_offset = ((offset_b << 32)|offset_l);
	jz_burner->rd_length = length;
	printf("Read medium %d frome %lld, length %d\n",
			jz_burner->rd_medium_type,
			jz_burner->rd_offset,
			jz_burner->rd_length);
	if (bulk_in->buf_length < length) {
		bulk_in->buf_length = length;
		free(bulk_in->buf_addr);
		bulk_in->buf_addr = calloc(bulk_in->buf_length,1);
	}

	switch (jz_burner->rd_medium_type) {
	case NAND:
		length = burner_nand_read(jz_burner,bulk_in->buf_addr,
				jz_burner->rd_length,jz_burner->rd_offset);
		break;
	case MMC:
		length = burner_mmc_read(jz_burner,bulk_in->buf_addr,
				jz_burner->rd_length,jz_burner->rd_offset);
		break;
	case NOR:
		length = burner_nor_read(jz_burner,bulk_in->buf_addr,
				jz_burner->rd_length,jz_burner->rd_offset);
		break;
	default:
		printf("Cannooooooooot happen %d\n",__LINE__);
	}

	if (length < 0) {
		jz_burner->ack_status = length;
		return;
	}

	bulk_in->epreq->buf = bulk_in->buf_addr;
	bulk_in->epreq->length = length;
	bulk_in->epreq->status = 0;
	bulk_in->epreq->context = jz_burner;
	bulk_in->epreq->complete = bunner_read_complete;
	ret= usb_ep_queue(bulk_in->ep,bulk_in->epreq,0);
	if (ret) {
		debug("Usb queue bulk in failed\n");
		jz_burner->ack_status = ret;
		bulk_in->epreq->status = 0;
	}
	jz_burner->ack_status = 0;
	return;
}

static int burner_read(struct jz_burner *jz_burner,
		u16 wValue,u16 wIndex,u16 wLength)
{
	struct usb_request *req = jz_burner->ep0req;

	switch (wValue) {
	case NAND:
	case MMC:
	case NOR:
	default:
		printf("Unkown bunner read medium type :%d\n",wValue);
		return -EINVAL;
	}

	jz_burner->rd_medium_type = wValue;
	req->length = wLength;
	req->complete = burner_start_read;
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
	if ((ctlreq->bRequestType & USB_TYPE_MASK) == USB_TYPE_VENDOR) {
		switch (ctlreq->bRequest) {
		case VEN_GET_CPU_INFO:
			printf("%wLength is %d\n", wLength);
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
		case VEN_SET_TRANS_ARGS:
			length = burner_set_trans_args(jz_burner,wValue,
					wIndex,wLength);
			break;
		case VEN_CTL:
			length = burner_control(jz_burner,wValue,
					wIndex,wLength);
			break;
		case VEN_WRITE:
			length = burner_program(jz_burner,wValue,
					wIndex,wLength);
			break;
		case VEN_READ:
			length = burner_read(jz_burner,wValue,
					wIndex,wLength);
			break;
		default:
			printf("Unkown Vendor Request 0x%x\n",ctlreq->bRequest);
			return -ENOSYS;
		}
	} else {
		printf("Unkown RequestType 0x%x \n",
				ctlreq->bRequestType);
		return -ENOSYS;
	}

	if (length >= 0) {
		length = usb_ep_queue(gadget->ep0, req, 0);
		if (length) {
			debug("ep_queue --> %d\n", length);
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
		debug("usb_ep_autoconfig for ep_in failed\n");
		ret = -ENODEV;
		goto in_ep_err;
	}
	jz_burner->bulk_in->ep = ep;
	debug("usb_ep_autoconfig for bulk_in got %s\n", ep->name);

	ep = usb_ep_autoconfig(cdev->gadget, out);
	if (!ep) {
		debug("usb_ep_autoconfig for ep_out failed\n");
		ret = -ENODEV;
		goto out_ep_err;
	}
	jz_burner->bulk_out->ep = ep;
	debug("usb_ep_autoconfig for bulk_out got %s\n", ep->name);

	req =  usb_ep_alloc_request(jz_burner->bulk_in->ep,0);
	if (!req) {
		debug("usb_ep_alloc_request for ep_in failed\n");
		ret = -ENOMEM;
		goto in_req_err;
	}
	req->complete = handle_default_complete;
	req->buf = NULL;
	jz_burner->bulk_in->epreq = req;

	req =  usb_ep_alloc_request(jz_burner->bulk_out->ep,0);
	if (!req) {
		debug("usb_ep_alloc_request for ep_out failed\n");
		ret = -ENOMEM;
		goto out_req_err;
	}
	req->complete = handle_default_complete;
	req->buf = NULL;
	jz_burner->bulk_out->epreq = req;

	jz_burner->bulk_in->buf_addr = NULL;
	jz_burner->bulk_in->buf_length = 0;
	jz_burner->bulk_in->transfer_state = TRANSFER_IDLE;
	jz_burner->bulk_out->buf_addr = NULL;
	jz_burner->bulk_out->buf_length = 0;
	jz_burner->bulk_out->transfer_state = TRANSFER_IDLE;

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

	ret = alloc_bulk_endpoints(jz_burner,
			&fs_bulk_in_desc,&fs_bulk_out_desc);
	if (ret)
		return ret;

	/*f->descriptors = usb_copy_descriptors(fs_intf_descs);
	if (unlikely(!f->descriptors))
		return -ENOMEM;*/

	if (gadget_is_dualspeed(cdev->gadget)) {
		 hs_bulk_in_desc.bEndpointAddress =
			 fs_bulk_in_desc.bEndpointAddress;
		 hs_bulk_out_desc.bEndpointAddress =
			 fs_bulk_out_desc.bEndpointAddress;
		/*f->hs_descriptors = usb_copy_descriptors(hs_intf_descs);
		if (unlikely(!f->hs_descriptors)) {
			free(f->descriptors);
			return -ENOMEM;
		}*/
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
	status = usb_ep_enable(in_ep,epdesc);
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
	status = usb_ep_enable(out_ep, epdesc);
	if (status < 0) {
		printf("usb enable ep out failed\n");
		goto failed;
	}
	out_ep->driver_data = jz_burner;
failed:
	return status;
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
	jz_burner->usb_function.unbind = f_vendor_burner_unbind;
	jz_burner->usb_function.setup = f_vendor_burner_setup_handle;
	jz_burner->usb_function.strings= burn_intf_string_tab;
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
