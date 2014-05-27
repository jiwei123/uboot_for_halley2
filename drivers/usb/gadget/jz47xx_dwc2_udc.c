/* Ingenic JZ DWC2 OTG Controller Driver
 *
 *  Copyright (C) 2013 Ingenic Semiconductor Co., LTD.
 *  Sun Jiwei <jwsun@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#define DWC2_DEBUG	0
#define DEBUG_RXFIFO	0x1	//0 : off 1 : epnum 0 2: epnum 1 3:ep_num 0,1

#include <common.h>
#include <malloc.h>
#include <asm/errno.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <asm/io.h>
#include <linux/list.h>
#include <asm/arch/clk.h>
#include <usb/lin_gadget_compat.h>
#include <usb/jz47xx_dwc2_udc.h>
#include "jz47xx_dwc2_regs.h"

#define DWC2_EP0_MTS_LIMIT	64
#define DWC2_HEP_MTS_LIMIT	(1023 * 512)
#define DWC2_FEP_MTS_LIMIT	(1023 * 64)

struct dwc2_udc	*the_controller;
LIST_HEAD(request_list);

static void dwc_otg_core_reset(void)
{
        u32 cnt = 0;
	/* Core Soft Reset */
	udc_set_reg(0,RSTCTL_CORE_RST, GRST_CTL);
        while (udc_test_reg(RSTCTL_CORE_RST,GRST_CTL)) {
                if (cnt++ > 10000) {
			pr_err("HANG! GRESET wait core reset timeout.\n");
                        return;
                }
                udelay(1);
        }
	cnt = 0;
	/* Wait for AHB master IDLE state. */
        while (!udc_test_reg(RSTCTL_AHB_IDLE,GRST_CTL)) {
                if (cnt++ > 100000) {
			pr_err("HANG! GRESET wait AHB IDLE timeout.\n");
                        return;
                }
                udelay(10);
        }
        /* wait for 3 phy clocks */
        udelay(100);
}

static void dwc_otg_core_init(void)
{
	u32 gusbcfg;
	u32 reset = 0;
	/*HB config Slave mode ,Unmask globle inter*/
	udc_write_reg(AHBCFG_GLOBLE_INTRMASK, GAHB_CFG);
	/*Mask RxfvlMsk Intr*/
	udc_set_reg(GINTSTS_RXFIFO_NEMPTY,0,GINT_MASK);
	/*HNP SRP not support , usb2.0 , utmi+, 16bit phy*/
	gusbcfg = udc_read_reg(GUSB_CFG);
	if (!(gusbcfg | USBCFG_16BIT_PHY) ||
			(gusbcfg | USBCFG_PHY_INF_UPLI))
		reset = 1;
	udc_set_reg(USBCFG_HNP_EN|USBCFG_SRP_EN|USBCFG_PHY_SEL_USB1|
			USBCFG_TRDTIME_MASK|USBCFG_PHY_INF_UPLI,
			USBCFG_16BIT_PHY|USBCFG_TRDTIME(5),GUSB_CFG);
	if (reset) {
		dwc_otg_core_reset();
		udc_write_reg(AHBCFG_GLOBLE_INTRMASK, GAHB_CFG);
	}
	/*Umask otg intr and mode mismatch*/
	udc_write_reg(GINTSTS_MODE_MISMATCH|GINTSTS_OTG_INTR, GINT_MASK);
	/*Clear sof intrrupt after core reset*/
	udc_write_reg(GINTSTS_START_FRAM,GINT_STS);
}

static void dwc2_otg_flush_tx_fifo(unsigned char txf_num)
{
	int timeout = 0x10000;
	pr_info("flush tx fifo fifo num %d\n",txf_num);
	/*Set globle nak*/
	if (udc_test_reg(GINTSTS_GINNAK_EFF,GINT_STS))
	{
		udc_set_reg(0,GINTSTS_GINNAK_EFF,OTG_DCTL);
		while(!(udc_read_reg(GINT_STS) & GINTSTS_GINNAK_EFF) && --timeout)
			udelay(1);
		if (!timeout) pr_warn("flush fifo globle in nak set timeout\n");
	}
	/*Check AHB is idle*/
	timeout = 100000;
	while(!(udc_test_reg(RSTCTL_AHB_IDLE,GRST_CTL)) && --timeout);
	if (!timeout) pr_warn("flush fifo ahb idle timeout\n");
	/*Check fifo is not in flushing*/
	timeout = 100000;
	while(!(udc_test_reg(RSTCTL_TXFIFO_FLUSH,GRST_CTL)) && --timeout);
	/*Flush fifo*/
	udc_set_reg(0,(txf_num << 6),GRST_CTL);
	udc_set_reg(0,RSTCTL_TXFIFO_FLUSH ,GRST_CTL);
	timeout = 100000;
	while (udc_test_reg(RSTCTL_TXFIFO_FLUSH,GRST_CTL) && --timeout);
	if (!timeout) pr_warn("flush fifo timeout\n");
	/*Clear globle nak*/
	udc_set_reg(0,DCTL_CLR_GNPINNAK,OTG_DCTL);
}

void handle_rxfifo_nempty(struct dwc2_udc *dwc);
void dwc2_otg_flush_rx_fifo(void)
{
	pr_warn_start();
	pr_info("dwc flush rx fifo\n");
	udc_write_reg(DCTL_SET_GONAK,OTG_DCTL);
	while(!(udc_read_reg(GINT_STS) & GINTSTS_GOUTNAK_EFF)) {
		if ((udc_read_reg(GINT_STS) & GINTSTS_RXFIFO_NEMPTY)) {
			handle_rxfifo_nempty(the_controller);
		}
		udelay(1);
	};
	udc_write_reg(RSTCTL_RXFIFO_FLUSH, GRST_CTL);
        while (udc_read_reg(GRST_CTL) & RSTCTL_RXFIFO_FLUSH);
	mdelay(8);
	udc_write_reg(DCTL_CLR_GONAK,OTG_DCTL);
	pr_warn_end();
}

static void dwc_fifo_allocate(void)
{
	u16 start_addr = 0;
	u16 gdfifocfg;
	/*rx fifo size*/
	udc_write_reg(RXFIFO_SIZE,GRXFIFO_SIZE);
	/* txfifo0 size */
	start_addr += RXFIFO_SIZE;
	udc_write_reg((NPTXFIFO_SIZE << 16)|start_addr, GNPTXFIFO_SIZE);
	/* txfifo1 size */
	start_addr += NPTXFIFO_SIZE;
	udc_write_reg((DTXFIFO1_SIZE << 16) | start_addr, DIEPTXF(1));
	/* txfifo2 size */
	start_addr += DTXFIFO1_SIZE;
	udc_write_reg((DTXFIFO2_SIZE << 16) | start_addr, DIEPTXF(2));
	/*ep info size*/
	start_addr += DTXFIFO2_SIZE;
	gdfifocfg = (udc_read_reg(GHW_CFG3) >> 16) | (start_addr << 16);
	udc_write_reg(gdfifocfg,GDFIFO_CFG);

	dwc2_otg_flush_tx_fifo(0x10);
	dwc2_otg_flush_rx_fifo();
}

static void dwc_otg_device_init(void)
{
	/* dma disable ,High speed , stall no zero handshack*/
	udc_write_reg(DCFG_HANDSHAKE_STALL_ERR_STATUS, OTG_DCFG);
	/* Soft Disconnect connect*/
	udc_set_reg(DCTL_SOFT_DISCONN, DCTL_NAK_ON_BBLE, OTG_DCTL);
	/* Unmask suspend earlysuspend reset enumdone sof intr*/
	udc_set_reg(0, GINTSTS_USB_SUSPEND|GINTSTS_USB_RESET|
			GINTSTS_ENUM_DONE|GINTSTS_USB_EARLYSUSPEND,
			GINT_MASK);
	return;
}

static int dwc_udc_init(struct dwc2_udc *dev)
{
	dwc_otg_core_reset();

	otg_phy_init(DEVICE_ONLY_MODE,CONFIG_SYS_EXTAL);

	dwc_otg_core_init();

	dwc_fifo_allocate();

	dwc_otg_device_init();

	return 0;
}

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	return 0;
}

static int jz_ep_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct dwc2_ep *dep = to_dwc2_ep(ep);
	int epnum = ep_num(dep);
	u32 reg_tmp;

	if (!ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		pr_err("%s: bad ep or descriptor\n", __func__);
		return -EINVAL;
	}

	if (!dep->desc) {
		dep->desc = desc;
		dep->bmAttributes = (desc->bmAttributes&USB_ENDPOINT_XFERTYPE_MASK);
	}

	if (dep->flags & DWC2_EP_ACTIVE)
		return 0;

	dep->flags = DWC2_EP_ACTIVE;
	dep->ep.maxpacket = le16_to_cpu(get_unaligned(&desc->wMaxPacketSize));
	reg_tmp = (dep->ep.maxpacket << DEPCTL_MPS_BIT)|
			DEPCTL_USBACTEP | DEPCTL_SETD0PID |
			((dep->bmAttributes)<< DEPCTL_TYPE_BIT);
	if (ep_is_in(dep)) {
		reg_tmp &= ~DIEPCTL_TX_FIFO_NUM_MASK;
		reg_tmp |= DIEPCTL_TX_FIFO_NUM(epnum);
		udc_write_reg(reg_tmp, DIEP_CTL(epnum));
		udc_set_reg(0, (1<<epnum),DAINT_MASK);
	} else {
		udc_write_reg(reg_tmp, DOEP_CTL(epnum));
		udc_set_reg(0, (1<<(epnum+DAINT_OUT_BIT)),DAINT_MASK);
	}
	return 0;
}

static void dwc2_giveback_urb(struct dwc2_ep *dep,
		struct dwc2_request *request, int status)
{
	pr_info("giveback ep%d%s\n",ep_num(dep),ep_is_in(dep)?"in":"out");
	list_del_init(&request->queue);
	request->req.status = status;
	request->req.complete(&dep->ep, &request->req);
}

static void __dwc2_set_globle_out_nak(int epnum) {
	udc_write_reg(DCTL_SET_GONAK,OTG_DCTL);
	while(!(udc_read_reg(GINT_STS) & GINTSTS_GOUTNAK_EFF)) {
		if ((udc_read_reg(GINT_STS) & GINTSTS_RXFIFO_NEMPTY)) {
			handle_rxfifo_nempty(the_controller);
		}
		udelay(1);
	}
}

static void __dwc2_clear_globle_out_nak(int epnum) {
	udc_write_reg(DCTL_CLR_GONAK,OTG_DCTL);
}

static void __dwc2_disable_out_endpoint(int epnum) {

	int timeout = 5000;
	udc_set_reg(0, (DEPCTL_EPDIS|DEPCTL_SNAK), DOEP_CTL(epnum));
	do {
		udelay(1);
	} while ( (!(udc_read_reg(DOEP_INT(epnum)) & DEP_EPDIS_INT)) && (--timeout > 0));
	udc_write_reg(DEP_EPDIS_INT,DOEP_INT(epnum));
	if (udc_read_reg(DOEP_CTL(epnum)) & DEPCTL_EPENA)
		pr_warn("disable out endpoint %d failed.\n",epnum);
}

int __dwc2_stop_out_transfer(struct dwc2_ep *dep)
{
	int xfer_size_left = 0;
	int epnum = ep_num(dep);
	unsigned int doep_ctl = udc_read_reg(DOEP_CTL(epnum));

	if (!(doep_ctl & DEPCTL_EPENA))
		return 0;

	/*Step 1: set globle nak*/
	__dwc2_set_globle_out_nak(epnum);

	/*Step 2: disable endponit*/
	__dwc2_disable_out_endpoint(epnum);

	/*Step3: clear globle nak*/
	__dwc2_clear_globle_out_nak(epnum);

	/*Step4: return trans size left*/
	xfer_size_left = udc_read_reg(DOEP_SIZE(epnum));
	xfer_size_left &=  ((1 << 19) - 1);

	return xfer_size_left;
}

int dwc2_disable_out_endpoint(struct dwc2_ep *dep)
{
	return __dwc2_stop_out_transfer(dep);
}

static void __dwc2_set_in_nak(int epnum)
{
	int  timeout = 5000;

	udc_write_reg(DEPCTL_SNAK,DIEP_CTL(epnum));
	do
	{
		udelay(1);
		if (timeout < 2) {
			pr_info("dwc set in nak timeout\n");
		}
	} while ( (!(udc_read_reg(DIEP_INT(epnum)) & DEP_INEP_NAKEFF)) && (--timeout > 0));

	udc_write_reg(DEP_INEP_NAKEFF,DIEP_INT(epnum));
}

static void __dwc2_disable_in_ep(int epnum)
{
	int  timeout = 100000;
	unsigned int diep_ctl = udc_read_reg(DIEP_CTL(epnum));

	udc_write_reg(diep_ctl | DEPCTL_EPDIS,DIEP_CTL(epnum));
	do
	{
		udelay(1);
		if (timeout < 2) {
			pr_info("dwc disable in ep timeout\n");
		}
	} while ( (!(udc_read_reg(DIEP_INT(epnum)) & DEP_EPDIS_INT)) && (--timeout > 0));

	udc_write_reg(DEP_EPDIS_INT,DIEP_INT(epnum));

}

static int __dwc2_stop_in_transfer(struct dwc2_ep *dep)
{
	int xfer_size_left = 0;
	int epnum = ep_num(dep);
	unsigned int diep_ctl = udc_read_reg(DIEP_CTL(epnum));

	if (diep_ctl & DEPCTL_EPENA) {
		printf("stop intransfer epnum %d\n",epnum);
		/*step 1: set in nak*/
		__dwc2_set_in_nak(epnum);
		dep->flags &= ~DWC2_EP_BUSY;

		/*step 2: disable in endpoint*/
		__dwc2_disable_in_ep(epnum);

		/*step 3: return trans size left*/
		xfer_size_left = udc_read_reg(DIEP_SIZE(epnum));
		xfer_size_left &=  ((1 << 19) - 1);
	}

	/*step 4: flush fifo*/
	dwc2_otg_flush_tx_fifo(epnum);

	return xfer_size_left;
}

static int dwc2_disable_in_endpoint(struct dwc2_ep *dep)
{
	return __dwc2_stop_in_transfer(dep);
}

static int dwc2_stop_transfer(struct dwc2_ep *dep)
{
	if (ep_is_in(dep))
		return __dwc2_stop_in_transfer(dep);
	else
		return __dwc2_stop_out_transfer(dep);
}

static void dwc2_deactive_endpoint(struct dwc2_ep *dep)
{
	int epnum = ep_num(dep);
	struct dwc2_request *request = NULL;
	int left_size = 0;

	if (!(dep->flags & DWC2_EP_ACTIVE))
		return;

	if (ep_is_in(dep)) {
		left_size = dwc2_disable_in_endpoint(dep);
		udc_set_reg(DEPCTL_USBACTEP, 0, DIEP_CTL(epnum));
		udc_set_reg((1<<epnum),0,DAINT_MASK);
	} else {
		left_size = dwc2_disable_out_endpoint(dep);
		udc_set_reg(DEPCTL_USBACTEP, 0, DOEP_CTL(epnum));
		udc_set_reg((1<<(epnum+16)),0 ,DAINT_MASK);
	}

	while ((request = next_request(&dep->urb_list))) {
		request->req.actual += (request->xfersize - left_size);
		left_size = 0;
		dwc2_giveback_urb(dep,request,-ESHUTDOWN);
	}

	dep->flags &= ~DWC2_EP_ACTIVE;
	return;
}

static int jz_ep_disable(struct usb_ep *ep)
{
	struct dwc2_ep *dep = to_dwc2_ep(ep);
	dwc2_deactive_endpoint(dep);
	dep->desc = 0;
	return 0;
}

static struct usb_request *jz_alloc_request(struct usb_ep *ep,
		gfp_t gfp_flags)
{
	struct dwc2_request *request = NULL;
	if (!list_empty(&request_list)) {
		request = list_first_entry(&request_list,
				struct dwc2_request, queue);
		list_del_init(&request->queue);
	} else {
		request = kzalloc(sizeof(*request), gfp_flags);
		if (!request)
			return NULL;
		INIT_LIST_HEAD(&request->queue);
	}
	return &request->req;
}

static void jz_free_request(struct usb_ep *ep, struct usb_request *req)
{
	struct dwc2_request *request = to_dwc2_request(req);
	list_add_tail(&request->queue,&request_list);
}

int calculate_xfer_pktcnt(struct dwc2_ep *dep, struct dwc2_request *request)
{
	struct usb_request *req = &request->req;
	struct usb_ep *ep = &dep->ep;
	int pktcnt = 0;

	request->xfersize = req->length - req->actual;
	if (request->xfersize > dep->max_xfer_once) {
		request->xfersize = dep->max_xfer_once -
			(dep->max_xfer_once%ep->maxpacket);
	}

	if (request->xfersize == 0) {
		pktcnt = 1;
	} else {
		pktcnt = (request->xfersize + ep->maxpacket - 1)/ep->maxpacket;
	}
	return pktcnt;
}

int in_transfer_error_detect(struct dwc2_ep *dep);
static void __dwc2_start_in_transfer(struct dwc2_ep *dep,struct dwc2_request *request)
{
	u32 pktcnt;
	u32 epnum = ep_num(dep);
	pktcnt = calculate_xfer_pktcnt(dep,request);
	udc_write_reg(pktcnt << 19 | request->xfersize, DIEP_SIZE(epnum));
	udc_set_reg(0,(DEPCTL_EPENA|DEPCTL_CNAK),DIEP_CTL(epnum));
	udc_set_reg(0,(1 << epnum),DIEP_EMPMSK);
	if (request->xfersize == 0) {
		dep->wait_inxfer_complete = 1;
		in_transfer_error_detect(dep);
	}
	pr_info("epnum in %d is transfer %d\n",epnum, request->xfersize);
	return;
}

static void __dwc2_start_out_transfer(struct dwc2_ep *dep, struct dwc2_request *request)
{
	u32 pktcnt;
	u32 epnum = ep_num(dep);
	pktcnt = calculate_xfer_pktcnt(dep,request);
	udc_write_reg(pktcnt << 19 | request->xfersize, DOEP_SIZE(epnum));
	udc_set_reg(0,(DEPCTL_EPENA|DEPCTL_CNAK),DOEP_CTL(epnum));
	pr_info("epnum out %d is transfer %d\n",epnum,request->xfersize);
	return;
}

static void dwc2_start_transfer(struct dwc2_ep *dep)
{
	struct dwc2_request *request = next_request(&dep->urb_list);

	if (request) {
		dep->flags |= DWC2_EP_BUSY;
		if (ep_is_in(dep))
			__dwc2_start_in_transfer(dep ,request);
		else
			__dwc2_start_out_transfer(dep, request);
	}

	return;
}

static void dwc2_restart_transfer(struct dwc2_ep *dep)
{
	struct dwc2_request *request = NULL;
	struct dwc2_udc *dev = the_controller;
	int xfer_size_left = 0;
	int epnum = ep_num(dep);

	xfer_size_left = dwc2_stop_transfer(dep);
	if ((request = next_request(&dep->urb_list))) {
		request->req.actual += request->xfersize - xfer_size_left;
		dwc2_giveback_urb(dep,request,-ECONNRESET);
	}
	dwc2_start_transfer(dep);
	if (!epnum)
		dev->ep0state = SETUP_STAGE;
	return;
}

static int jz_queue(struct usb_ep *ep, struct usb_request *req, gfp_t gfp_flags)
{
	struct dwc2_request *request = to_dwc2_request(req);
	struct dwc2_ep *dep = to_dwc2_ep(ep);
	int epnum = ep_num(dep);
	int transfer_idle;

	printf("epnum %d queue\n",epnum);
	if (unlikely(!list_empty(&request->queue))) {
		printf("epnum %d is busy\n",epnum);
		return -EBUSY;
	}

	if (!(dep->flags & DWC2_EP_ACTIVE)) {
		printf("epnum %d is closed\n",epnum);
		return -ESHUTDOWN;
	}

	transfer_idle = list_empty(&dep->urb_list);

	req->status = -EINPROGRESS;
	req->actual = 0;

	request->zlp_transfered = false;
	request->dep = dep;
	list_add_tail(&request->queue,&dep->urb_list);

	if (transfer_idle)
		dwc2_start_transfer(dep);
	return 0;
}

static int jz_dequeue(struct usb_ep *ep, struct usb_request *req)
{
	struct dwc2_ep *dep = to_dwc2_ep(ep);
	struct dwc2_request *request = to_dwc2_request(req);
	struct dwc2_request *r = NULL;

	list_for_each_entry(request,&dep->urb_list,queue) {
		if (r == request)
			break;
	}

	if (r != request) {
		return -EINVAL;
	} else {
		r = next_request(&dep->urb_list);
		if (r == request)
			dwc2_stop_transfer(dep);
	}
	dwc2_giveback_urb(dep, request, -ECONNRESET);
	return 0;
}

static void jz_fifo_flush(struct usb_ep *ep)
{
	struct dwc2_ep *dep = to_dwc2_ep(ep);

	if (ep_is_in(dep))
		dwc2_otg_flush_tx_fifo(ep_num(dep));
	else
		dwc2_otg_flush_rx_fifo();
	return;
}

static struct usb_ep_ops dwc2_ep_ops = {
	.enable = jz_ep_enable,
	.disable = jz_ep_disable,
	.alloc_request = jz_alloc_request,
	.free_request = jz_free_request,
	.queue = jz_queue,
	.dequeue = jz_dequeue,
	.fifo_flush = jz_fifo_flush,
};

static void dwc2_init_endpoint(struct dwc2_udc *dev, int is_in)
{
	int step = 0, end = DWC2_MAX_OUT_ENDPOINTS, i;

	if (is_in) {
		step += DWC2_MAX_OUT_ENDPOINTS;
		end += DWC2_MAX_IN_ENDPOINTS;
	}
	for (i = 0; (i + step) < end; i++) {
		struct dwc2_ep *dep = &dev->ep_attr[i + step];
		if (i == 0 && is_in)
			continue;

		dep->dev = dev;
		dep->ep.ops = &dwc2_ep_ops;
		dep->desc = NULL;
		dep->flags = 0;
		INIT_LIST_HEAD(&dep->ep.ep_list);
		INIT_LIST_HEAD(&dep->urb_list);
		if (i == 0) {
			dep->bEndpointAddress = (i&USB_ENDPOINT_NUMBER_MASK);
			dep->bmAttributes = USB_ENDPOINT_XFER_CONTROL;
			dev->gadget.ep0 = &dep->ep;
			dev->ep_out_attr[0] = dep;
			dev->ep_in_attr[0] = dep;
			snprintf(dep->name, sizeof(dep->name), "ep0-control");
			dep->ep.maxpacket = 64;
			dep->max_xfer_once = DWC2_EP0_MTS_LIMIT;
		} else {
			if (is_in) {
				dep->bEndpointAddress = (i&USB_ENDPOINT_NUMBER_MASK)|0x80;
				dev->ep_in_attr[i] = dep;
				snprintf(dep->name, sizeof(dep->name), "ep%din", i);
			} else {
				dep->bEndpointAddress = (i&USB_ENDPOINT_NUMBER_MASK);
				dev->ep_out_attr[i] = dep;
				snprintf(dep->name, sizeof(dep->name), "ep%dout", i);
			}
			dep->bmAttributes = USB_ENDPOINT_XFER_BULK;
			list_add_tail(&dep->ep.ep_list, &dev->gadget.ep_list);
			dep->ep.maxpacket = 512;
			dep->max_xfer_once = DWC2_HEP_MTS_LIMIT;
		}
		dep->ep.name = dep->name;
	}
}

static const struct usb_gadget_ops jz_udc_ops;
int jz_udc_probe(void)
{
	the_controller = (struct dwc2_udc *)kzalloc(sizeof(struct dwc2_udc), 0);
	if (!the_controller)
		return -ENOMEM;

	INIT_LIST_HEAD(&the_controller->gadget.ep_list);
	the_controller->ep0state = SETUP_STAGE;
	snprintf(the_controller->name, sizeof(the_controller->name),
			"jz_dwc2_udc_v1.1");
	printf("jz_dwc2_udc_v1.1\n");
	the_controller->gadget.is_dualspeed = 1;
	the_controller->gadget.ops = &jz_udc_ops;
	the_controller->gadget.name = the_controller->name;
	dwc2_init_endpoint(the_controller, 0);
	dwc2_init_endpoint(the_controller, 1);

	return 0;
}

static int udc_setup_status(int is_in)
{
	pr_ep0("**enable** %s status stage\n", is_in ? "in" : "out");
	if (is_in) {
		struct dwc2_ep *dep = the_controller->ep_in_attr[0];
		udc_write_reg(1 << 19, DIEP_SIZE(0));
		udc_set_reg(0, DEPCTL_EPENA|DEPCTL_CNAK, DIEP_CTL(0));
		udc_set_reg(0, 1 ,DAINT_MASK);
		dep->wait_inxfer_complete = 1;
		return in_transfer_error_detect(dep);
	} else {
		udc_write_reg((1 << 19),DOEP_SIZE(0));
		udc_set_reg(0, DEPCTL_EPENA|DEPCTL_CNAK, DOEP_CTL(0));
		pr_debug("DOEP_CTL(0) %x\n",udc_read_reg(DOEP_CTL(0)));
		return 0;
	}
}

void udc_start_new_setup(void)
{
	pr_ep0("**enable** setup stage\n");
	udc_write_reg(DOEPSIZE0_SUPCNT_3|DOEPSIZE0_PKTCNT|8*3, DOEP_SIZE(0));
	udc_set_reg(0, DEPCTL_EPENA|DEPCTL_CNAK, DOEP_CTL(0));
}

static void handle_early_suspend_intr(struct dwc2_udc *dev)
{
	pr_info("Handle early suspend intr, mask EARLYSUSPEND bit\n");
	udc_write_reg(GINTSTS_USB_EARLYSUSPEND, GINT_STS);
}

static void handle_reset_intr(struct dwc2_udc *dev)
{
	int i;
	pr_info("Handle reset intr\n");
	/* Step 1: SET NAK for all OUT ep */
	for (i = 0; i < 2; i++)
		udc_set_reg(0, DEPCTL_SNAK, DOEP_CTL(i));

	/* Step 2: unmask intr. */
	udc_set_reg(0, (1<<DAINT_IN_BIT)|(1<<DAINT_OUT_BIT),
			DAINT_MASK);
	udc_set_reg(0, DEPMSK_XFERCOMLMSK|DEPMSK_SETUPMSK|DEPMSK_B2BSETUPMSK,
			DOEP_MASK);
	udc_set_reg(0, DEPMSK_XFERCOMLMSK|DEPMSK_TXFIFOEMTMSK|DEPMSK_TIMEOUTMSK,
			DIEP_MASK);
	/* Step 3: device init nothing to do */
	/* Step 4: dfifo dynamic allocated */
	/* Step 5: Reset Device Address */
	udc_set_reg(DCFG_DEV_ADDR_MASK,0,OTG_DCFG);

	/*Step 6: setup EP0 to receive SETUP packets*/
	udc_write_reg(DOEPSIZE0_SUPCNT_3|DOEPSIZE0_PKTCNT|8*3, DOEP_SIZE(0));
	udc_write_reg(GINTSTS_USB_RESET, GINT_STS);
}

int enum_done_speed_detect(struct dwc2_udc *dev)
{
	u32 dsts = udc_read_reg(OTG_DSTS);
	u32 ep_fifo_size = 0;
	u32 ep_mts = 0;
	int i;

	switch(dsts & DSTS_ENUM_SPEED_MASK) {
	case DSTS_ENUM_SPEED_HIGH:
		pr_info("High Speed.\n");
		ep_fifo_size = 512;
		ep_mts = DWC2_HEP_MTS_LIMIT;
		dev->gadget.speed = USB_SPEED_HIGH;
		break;
	case DSTS_ENUM_SPEED_FULL_30OR60:
	case DSTS_ENUM_SPEED_FULL_48:
		pr_info("Full Speed.\n");
		ep_fifo_size = 64;
		ep_mts = DWC2_FEP_MTS_LIMIT;
		dev->gadget.speed = USB_SPEED_FULL;
		break;
	case DSTS_ENUM_SPEED_LOW:
	default:
		pr_err("Low Speed is not support\n");
		return -1;
	}

	for (i = 1; i < DWC2_MAX_ENDPOINTS; i++) {
		dev->ep_attr[i].ep.maxpacket = ep_fifo_size;
		dev->ep_attr[i].max_xfer_once = ep_mts;
	}
	return 0;
}

void handle_enum_done_intr(struct dwc2_udc *dev)
{
	pr_info("Handle enum done intr.\n");

	if (enum_done_speed_detect(dev))
		return;

	udc_set_reg(0, DCTL_CLR_GNPINNAK, OTG_DCTL);
	udc_set_reg(0, GINTSTS_RXFIFO_NEMPTY|GINTSTS_IEP_INTR|GINTSTS_OEP_INTR,
			GINT_MASK);
	udc_set_reg(0, DEPCTL_EP0_MPS_64|DIEPCTL_TX_FIFO_NUM(0), DIEP_CTL(0));
	udc_set_reg(0, DEPCTL_EPENA|DEPCTL_CNAK|DEPCTL_EP0_MPS_64, DOEP_CTL(0));
	udc_write_reg(GINTSTS_ENUM_DONE, GINT_STS);
}

static void parse_setup(struct dwc2_ep *dep)
{
	struct dwc2_udc *dev = the_controller;
	int ret = 0;

	pr_ep0("setup done requesttype %x request %x\n",dev->crq.bRequestType,
			dev->crq.bRequest);

	if (dev->crq.wLength)
		dev->ep0state = DATA_STAGE;
	else
		dev->ep0state = STATUS_STAGE;

	if (!(dev->crq.bRequestType & USB_DIR_IN))
		dev->ctl_trans_is_out = 1;
	else
		dev->ctl_trans_is_out = 0;

	if ((dev->crq.bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		switch (dev->crq.bRequest) {
		case USB_REQ_SET_ADDRESS:
			pr_info("set device address 0x%x\n",dev->crq.wValue);
			udc_set_reg(DCFG_DEV_ADDR_MASK,
					(dev->crq.wValue << DCFG_DEV_ADDR_BIT),
					OTG_DCFG);
			udc_setup_status(1);
			break;
		default:
			ret = dev->driver->setup(&dev->gadget, &dev->crq);
			break;
		}
	} else {
		ret = dev->driver->setup(&dev->gadget, &dev->crq);
	}

	if (ret) {
		//usb_stall_ep0(dep);
		return;
	}
#if 0
	if (dev->ep0state == DATA_STAGE && !dev->ctl_trans_is_out) {
		pr_ep0("control in(3) transfer setup stage parse ok\n");
		udc_setup_status(0);
	} else if (dev->ep0state == STATUS_STAGE) {
		pr_ep0("control out(2) transfer setup stage parse ok\n");
		udc_start_new_setup();
	} else {
		pr_ep0("control out(3) transfer setup stage parse ok\n");
	}
#endif
}

static void udc_fetch_data_packet(struct dwc2_ep *dep, int fifo_count)
{
	struct dwc2_request *request = next_request(&dep->urb_list);
	int dwords = 0;
	int epnum = ep_num(dep);
	int i,dat;

	if (unlikely(!request))
		return;
	dwords = (fifo_count + 3) / 4;
	for (i = 0; i < dwords; i++) {
		dat = udc_read_reg(EP_FIFO(epnum));
		if (request->xfersize > 0) {
			*((u8 *)(request->req.buf + request->req.actual + 0)) = dat & 0xff;
			*((u8 *)(request->req.buf + request->req.actual + 1)) = (dat >> 8) & 0xff;
			*((u8 *)(request->req.buf + request->req.actual + 2)) = (dat >> 16) & 0xff;
			*((u8 *)(request->req.buf + request->req.actual + 3)) = (dat >> 24) & 0xff;
			request->xfersize -=4;
			request->req.actual += 4;
		}
	}
	return;
}

static void udc_fetch_setup_packet(struct dwc2_ep *dep, int fifo_count)
{
	struct dwc2_udc *dev = the_controller;
	unsigned int *buf = (unsigned int *)(&dev->crq);
	int epnum = ep_num(dep);
	BUG_ON(fifo_count != 8);
	buf[0] = udc_read_reg(EP_FIFO(epnum));
	buf[1] = udc_read_reg(EP_FIFO(epnum));
	pr_info("setup recv requesttype %x request %x\n",dev->crq.bRequestType,
			dev->crq.bRequest);
	return;
}

void handle_rxfifo_nempty(struct dwc2_udc *dev)
{
	unsigned volatile rxsts_pop = udc_read_reg(GRXSTS_POP);
	int	epnum = (rxsts_pop&0xf);
	int fifo_count = (rxsts_pop&GRXSTSP_BYTE_CNT_MASK) >> GRXSTSP_BYTE_CNT_BIT;
	struct dwc2_ep *dep = dev->ep_out_attr[epnum];

	debug_cond((DEBUG_RXFIFO&(1 << epnum)),
			"%s: GRXSTS_POP is 0x%x\n", __func__, rxsts_pop);
	switch(rxsts_pop & GRXSTSP_PKSTS_MASK) {
	case GRXSTSP_PKSTS_GOUT_NAK:
		debug_cond((DEBUG_RXFIFO&(1 << epnum)),
				"%s: OUT NAK\n", __func__);
		break;
	case GRXSTSP_PKSTS_GOUT_RECV:
		debug_cond((DEBUG_RXFIFO&(1 << epnum)),
				"%s: GRXSTSP_PKSTS_GOUT_RECV epnum %d\n",__func__,epnum);
		if (epnum == 0 && fifo_count == NO_OUT_ZREO_PKT)
			dep->zero_pkt_recv = OUT_ZERO_PKT_RECV;
		udc_fetch_data_packet(dep, fifo_count);
		break;
	case GRXSTSP_PKSTS_TX_COMP:
		debug_cond((DEBUG_RXFIFO&(1 << epnum)),
				"%s: TX complete epnum %d\n", __func__,epnum);
		if (epnum == 0 && dep->zero_pkt_recv == OUT_ZERO_PKT_RECV)
			dep->zero_pkt_recv = OUT_ZERO_PKT_COMP;
		break;
        case GRXSTSP_PKSTS_SETUP_COMP:
		debug_cond((DEBUG_RXFIFO&(1 << epnum)),
				"%s: SETUP complete\n", __func__);
                break;
        case GRXSTSP_PKSTS_SETUP_RECV:
		debug_cond((DEBUG_RXFIFO&(1 << epnum)),
				"%s: SETUP receive\n", __func__);
		udc_fetch_setup_packet(dep,fifo_count);
                break;
        default:
		debug_cond((DEBUG_RXFIFO&(1 << epnum)),
				"%s: Warring, have not intr\n", __func__);
                break;
        }
	udc_write_reg(GINTSTS_RXFIFO_NEMPTY, GINT_STS);
}

void inep0_transfer_complete (struct dwc2_ep *dep)
{
	struct dwc2_udc *dev = the_controller;
	struct dwc2_request *request = next_request(&dep->urb_list);
	int is_last = 0;
	static int first_in_filter = 1;

	if (dev->ep0state == STATUS_STAGE && dev->ctl_trans_is_out) {
		pr_ep0("==in== control %s transfer status stage complete\n", request ? "out(2)" : "out(3)");
		dev->ep0state = SETUP_STAGE;
		udc_start_new_setup();
		is_last = 1;
	} else if (dev->ep0state == DATA_STAGE && !dev->ctl_trans_is_out) {
		pr_ep0("==in== control in(3) transfer data stage complete\n");
		BUG_ON(!request);
		if (request->req.actual >= request->req.length) {
			dev->ep0state = STATUS_STAGE;
			is_last = 1;
			udc_setup_status(0);
		}
	} else if (!first_in_filter) {
		pr_err("[ep0]:==in== unkown control transfer in stage complete\n");
		return;
	}
	first_in_filter = 0;

	if (likely(is_last)) {
		udc_set_reg(1,0,DIEP_EMPMSK);
		if (!request) return;
		dwc2_giveback_urb(dep, request, 0);
	} else {
		pr_info("unlikely happen %d\n",__LINE__);
		dwc2_start_transfer(dep);
	}
	return;
}

void inepx_transfer_complete(struct dwc2_ep *dep)
{
	struct dwc2_request	*request = next_request(&dep->urb_list);
	int is_last = 0;
	int epnum = ep_num(dep);

	if (unlikely(!request)) {
		pr_warn("in xfercomplete happen but does not have urb\n");
		return;
	}

	if (request->req.actual >= request->req.length)
		is_last = 1;

	if (is_last && need_send_zlp(request)) {
		request->zlp_transfered = true;
	} else if (is_last) {
		udc_set_reg((1 << epnum),0,DIEP_EMPMSK);
		dwc2_giveback_urb(dep,request,0);
	}
	dwc2_start_transfer(dep);
}

void dwc2_fill_tx_fifo(struct dwc2_ep *dep)
{
	struct dwc2_request *request = next_request(&dep->urb_list);
	int epnum = ep_num(dep);

	if (!request)
		return;

	while (1) {
		int i = 0 ,xfersize = 0 ,xferdwords = 0;
		int fifo_status = 0;
		int *buf = (int*)request->req.buf + (request->req.actual/4);

		fifo_status = udc_read_reg(DIEP_TXFSTS(epnum));
		if (request->xfersize > ep_maxpacket(dep))
			xfersize = ep_maxpacket(dep);
		else
			xfersize = request->xfersize;
		xferdwords = (xfersize + 3) / 4;

		/*transfer complete or fifo no space*/
		if (!xferdwords || fifo_status < xferdwords)
			break;

		pr_info("xferdwords %d\n",xferdwords);
		for (;i < xferdwords; i++) {
			udc_write_reg(buf[i],EP_FIFO(epnum));
		}
		request->req.actual += xfersize;
		request->xfersize -= xfersize;
	}

	if (!request->xfersize)
		dep->wait_inxfer_complete = 1;
	return;
}

int in_transfer_error_detect(struct dwc2_ep *dep)
{
	/*can we just stall the endpoint\n*/
	if (dep->wait_inxfer_complete) {
		unsigned int timeout = 0x7ffff;
		int epnum = ep_num(dep);
		dep->wait_inxfer_complete = 0;
		pr_info("in_transfer_error_detect\n");
		while (!(udc_read_reg(DIEP_INT(epnum))&DEP_XFER_COMP)
				&& --timeout);
		if (!timeout) {
			pr_err("handle inep timeout\n");
			dwc2_restart_transfer(dep);
			udc_set_reg((1 << epnum),0,DIEP_EMPMSK);
			return -1;
		}
	}
	return 0;
}

void handle_inep_intr(struct dwc2_udc *dev)
{
        u32 ep_intr, intr;
	u32 ep_msk;
	u32 ep_pending;
        int epnum;
	struct dwc2_ep *dep = NULL;
	for (epnum = 0, intr = udc_read_reg(OTG_DAINT) & DAINT_IN_MASK;
			intr != 0 && epnum <= DWC2_MAX_IN_ENDPOINTS;
			intr &= ~(0x1 << epnum), epnum++) {

		if (!(intr & (0x1 << epnum)))
			continue;
		else
			dep = dev->ep_in_attr[epnum];

		ep_intr = udc_read_reg(DIEP_INT(epnum));
		pr_info("epnum %d in intr %x aint%x\n",epnum,ep_intr, intr);
		ep_msk = udc_read_reg(DIEP_MASK);
		ep_pending = (ep_intr&ep_msk);

		if (ep_pending & DEP_XFER_COMP) {
			dep->wait_inxfer_complete = 0;
			if (ep_num(dep))
				inepx_transfer_complete(dep);
			else
				inep0_transfer_complete(dep);
			udc_write_reg(DEP_XFER_COMP, DIEP_INT(epnum));
		}

		if (ep_pending & DEP_INEP_NAKEFF)
			udc_write_reg(DEP_INEP_NAKEFF, DIEP_INT(epnum));

		if (ep_pending & DEP_NAK_INT)
			udc_write_reg(DEP_NAK_INT, DIEP_INT(epnum));

		if (ep_pending & DEP_TIME_OUT) {
			pr_info("DEP_TIME_OUT %d\n",epnum);
			udc_write_reg(DEP_TIME_OUT, DIEP_INT(epnum));
		}

		if (ep_pending & DEP_TXFIFO_EMPTY) {
			pr_info("DEP_TXFIFO_EMPTY %d\n",epnum);
			if ((udc_read_reg(DIEP_EMPMSK) & (1 << epnum))) {
				pr_info("fill epnum %d\n",epnum);
				dwc2_fill_tx_fifo(dep);
				in_transfer_error_detect(dep);
			}
			udc_write_reg(DEP_TXFIFO_EMPTY, DIEP_INT(epnum));
		}
	}
}

void outep0_transfer_complete(struct dwc2_ep *dep)
{
	struct dwc2_udc *dev = the_controller;
	struct dwc2_request *request = next_request(&dep->urb_list);
	int is_last = 0;
	static int out_data_cover = 1;

	if (dev->ep0state == DATA_STAGE && dev->ctl_trans_is_out) {
		BUG_ON(!request);
		pr_ep0("==out== control out(3) transfer data stage complete\n");
		if (out_data_cover) {
			out_data_cover = 0;
			return;
		}
		out_data_cover = 1;
		is_last = !!udc_read_reg(DOEP_SIZE(0));
		if (request->req.actual >= request->req.length)
			is_last = 1;
		if (is_last) {
			dev->ep0state  = STATUS_STAGE;
#if 1
			while (udc_read_reg(GINT_STS) & GINTSTS_RXFIFO_NEMPTY) {
				int stat = udc_read_reg(GRXSTS_READ);
				if ((stat&GRXSTSP_PKSTS_MASK)==GRXSTSP_PKSTS_TX_COMP) {
					handle_rxfifo_nempty(the_controller);
					udelay(10);
					//dwc2_otg_flush_rx_fifo();
				} else {
					break;
				}
			}
#endif
			udc_write_reg(DEP_STATUS_PHASE_RECV, DOEP_INT(0));
			if (!udc_setup_status(1)) {
				dev->ep0state  = DATA_STAGE;
				dwc2_giveback_urb(dep, request, 0);
				dev->ep0state  = STATUS_STAGE;
			}
		} else {
			dwc2_start_transfer(dep);
		}
	} else if (dev->ep0state == STATUS_STAGE && !dev->ctl_trans_is_out) {
		pr_ep0("==out== control in(3) transfer status stage complete\n");
		if (dep->zero_pkt_recv != OUT_ZERO_PKT_COMP) {
			pr_warn_start();
			while((udc_read_reg(GINT_STS) & GINTSTS_RXFIFO_NEMPTY)) {
				handle_rxfifo_nempty(dev);
				udelay(1);
			}
			pr_warn_end();
		}

		if (dep->zero_pkt_recv != OUT_ZERO_PKT_COMP &&
				(udc_read_reg(DOEP_INT(0)) & DEP_SETUP_PHASE_DONE)) {
			pr_err("out status uncomming\n");
			udc_write_reg(DEP_SETUP_PHASE_DONE, DOEP_INT(0));
		}
		dep->zero_pkt_recv = NO_OUT_ZREO_PKT;
		dev->ep0state = SETUP_STAGE;
		udc_start_new_setup();
	} else if (dev->ep0state == SETUP_STAGE) {
		pr_ep0("==out== control setup stage complete\n");
	} else {
		pr_err("[ep0]:==out== unkown control out stage complete\n");
	}

	if (udc_read_reg(DOEP_INT(0))&DEP_STATUS_PHASE_RECV)
		pr_err("DEP_STATUS_PHASE_RECV\n");
	return;
}

void outepx_transfer_complete(struct dwc2_ep *dep)
{
	struct dwc2_request	*request = next_request(&dep->urb_list);
	int epnum = ep_num(dep);
	int is_last = 0;

	if (unlikely(!request)) {
		pr_warn("out xfercomplete happen but does not have urb\n");
		return;
	}

	is_last = !!udc_read_reg(DOEP_SIZE(epnum));
	if (request->req.actual >= request->req.length)
		is_last = 1;

	if (is_last)
		dwc2_giveback_urb(dep, request, 0);
	else
		dwc2_start_transfer(dep);
	return;
}

int handle_outep_intr(struct dwc2_udc *dev)
{
        u32 ep_intr, intr;
	u32 ep_msk;
	u32 ep_pending;
        int epnum;
	struct dwc2_ep *dep = NULL;

	for (epnum = 0, intr = (udc_read_reg(OTG_DAINT)& DAINT_OUT_MASK)>>DAINT_OUT_BIT;
			intr != 0 && epnum <= DWC2_MAX_OUT_ENDPOINTS;
			intr &= ~(0x1 << epnum), epnum++) {

		if (!(intr & (0x1 << epnum)))
			continue;
		else
			dep = dev->ep_out_attr[epnum];

		ep_intr = udc_read_reg(DOEP_INT(epnum));
		pr_info("===== epnum %d out intr %x ======\n",epnum,ep_intr);
		ep_msk = udc_read_reg(DOEP_MASK);
		ep_pending = (ep_intr&ep_msk);

		if (ep_pending & DEP_XFER_COMP) {
			if (!epnum) {
				outep0_transfer_complete(dep);
			} else {
				outepx_transfer_complete(dep);
			}
			udc_write_reg(DEP_XFER_COMP, DOEP_INT(epnum));
		}
		if (ep_pending & DEP_SETUP_PHASE_DONE) {
			udc_write_reg(DEP_SETUP_PHASE_DONE, DOEP_INT(epnum));
			if (DEP_B2B_SETUP_RECV & ep_intr) {
				pr_err("back to back received \n");
				udc_write_reg(DEP_B2B_SETUP_RECV, DOEP_INT(epnum));
			}
			parse_setup(dep);
		}
	}
	return 0;
}

int udc_irq(void)
{
	struct dwc2_udc *dev = the_controller;
	u32 intsts = udc_read_reg(GINT_STS);
	u32 gintmsk = udc_read_reg(GINT_MASK);
	u32 pending = intsts & gintmsk;

	if (pending & GINTSTS_USB_EARLYSUSPEND)
		handle_early_suspend_intr(dev);

	if (pending & GINTSTS_USB_RESET)
		handle_reset_intr(dev);

	if (pending & GINTSTS_ENUM_DONE)
		handle_enum_done_intr(dev);

	if (pending & GINTSTS_IEP_INTR)
		handle_inep_intr(dev);

	if (pending & GINTSTS_OEP_INTR)
		handle_outep_intr(dev);

	if (pending & GINTSTS_RXFIFO_NEMPTY)
		handle_rxfifo_nempty(dev);

	return IRQ_HANDLED;
}

static int dwc_udc_init_burnner(struct dwc2_udc *dev)
{
	int i = 0;

	if (enum_done_speed_detect(dev))
		return -1;

	udc_set_reg(0, DCTL_NAK_ON_BBLE, OTG_DCTL);
	udc_set_reg(0, DCFG_HANDSHAKE_STALL_ERR_STATUS, OTG_DCFG);
	udc_set_reg(0, (0x3<<DAINT_OUT_BIT)|(0x3<<DAINT_IN_BIT), DAINT_MASK);
	udc_set_reg(0, DEPMSK_XFERCOMLMSK|DEPMSK_SETUPMSK|DEPMSK_B2BSETUPMSK,
			DOEP_MASK);
	udc_set_reg(0, DEPMSK_XFERCOMLMSK|DEPMSK_TXFIFOEMTMSK|DEPMSK_TIMEOUTMSK,
			DIEP_MASK);

	for (i = 0; i < DWC2_MAX_ENDPOINTS; i++)
	{
		struct dwc2_ep *dep = &dev->ep_attr[i];
		dep->flags |= DWC2_EP_ACTIVE;
	}
	udc_set_reg(DIEPCTL_TX_FIFO_NUM_MASK, DIEPCTL_TX_FIFO_NUM(1), DIEP_CTL(1));

}
/*
  Register entry point for the peripheral controller driver.
*/
static int usb_poll_active = false;
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct dwc2_udc *dev = the_controller;
	int retval = -ENODEV;

	printf("%p",&driver->bind);
	if (driver->bind)
		retval = driver->bind(&dev->gadget);
	if (retval)
		return retval;
	dev->driver = driver;

#ifndef CONFIG_BURNER
	dwc_udc_init(dev);
#else
	dwc_udc_init_burnner(dev);
#endif /* CONFIG_BURNER */

	usb_poll_active = true;

	return 0;
}

int usb_gadget_handle_interrupts(void)
{
	if (usb_poll_active == true)
		udc_irq();
	return 0;
}
