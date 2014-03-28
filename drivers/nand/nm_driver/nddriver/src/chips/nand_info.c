/**
 * nand_info.c
 **/

#include <os_clib.h>
#include <ingenic_nand_mgr/nand_param.h>
#include "nand_info.h"
#include "nand_debug.h"
#include "ndcommand.h"
#include "nand_io.h"

extern int (*__try_wait_rb) (rb_item *, int);

static void dump_retry_parms(retry_parms *retryparms)
{
	int i;
	unsigned char *data = retryparms->data;

	ndd_debug("\ndump retry parms, recycle = %d, regcnt = %d",
		  retryparms->cycle, retryparms->regcnt);
	for (i = 0; i < retryparms->cycle * retryparms->regcnt; i++) {
		if (!(i % retryparms->regcnt))
			ndd_debug("\n %d:", i / retryparms->regcnt);
		ndd_debug(" %02x", data[i]);
	}
	ndd_debug("\n\n");
}

#define TRY_RB_DELAY 1 // 1ms
static rb_item* wait_unknown_rb_timeout(rb_info *rbinfo, int timeout)
{
	int i, wait_times = 0;

	for (i = 0; wait_times < timeout; i++)
	{
		if (__try_wait_rb(&(rbinfo->rbinfo_table[i % rbinfo->totalrbs]), TRY_RB_DELAY))
			break;
		wait_times += TRY_RB_DELAY;
	}

	if (wait_times >= timeout)
		return NULL;
	else
		return &(rbinfo->rbinfo_table[i % rbinfo->totalrbs]);
}

rb_item* get_rbitem(nfi_base *base, unsigned int cs_id, rb_info *rbinfo)
{
	int context, ret;
	rb_item *rbitem = NULL;

	context = nand_io_open(base, NULL);
	if (!context)
		RETURN_ERR(NULL, "nand io open error");

	ret = nand_io_chip_select(context, cs_id);
	if (ret)
		RETURN_ERR(NULL, "nand io chip select error, cs = [%d]", cs_id);

	ret = nand_io_send_cmd(context, CMD_RESET_1ST, 300);
	if (ret)
		RETURN_ERR(NULL, "nand io sent cmd error, cs = [%d], cmd = [%d]", cs_id, CMD_RESET_1ST);

	rbitem = wait_unknown_rb_timeout(rbinfo, 8);
	ret = nand_io_chip_deselect(context, cs_id);
	if (ret)
		RETURN_ERR(NULL, "nand io chip deselect error, cs = [%d]", cs_id);

	nand_io_close(context);

	return rbitem;
}

/**
 * get_nand_id: get nand id and extid from device
 *
 * @cid: contain nand id and extid
 * @return: 0: success, !0: fail
 **/
int get_nand_id(nfi_base *base, unsigned int cs_id, rb_info *rbinfo, nand_flash_id *fid)
{
	int context, ret;
	unsigned char nand_id[6];

	context = nand_io_open(base, NULL);
	if (!context)
		RETURN_ERR(ENAND, "nand io open error");

	ret = nand_io_chip_select(context, cs_id);
	if (ret)
		RETURN_ERR(ENAND, "nand io chip select error, cs = [%d]", cs_id);

	ret = nand_io_send_cmd(context, CMD_RESET_1ST, 300);
	if (ret)
		RETURN_ERR(ret, "nand io sent cmd error, cs = [%d], cmd = [%d]", cs_id, CMD_RESET_1ST);

	if ((wait_unknown_rb_timeout(rbinfo, 8)) == NULL)
		RETURN_ERR(TIMEOUT, "nand io wait rb timeout, cs = [%d]", cs_id);

	ret = nand_io_send_cmd(context, CMD_READ_ID_1ST, 300);
	if (ret)
		RETURN_ERR(ret, "nand io sent cmd error, cs = [%d], cmd = [%d]", cs_id, CMD_READ_ID_1ST);

	nand_io_send_spec_addr(context, 0x00, 1, 1 * 1000 * 1000);

	ret = nand_io_receive_data(context, nand_id, sizeof(nand_id));
	if (ret)
		RETURN_ERR(ret, "nand io receive data error, cs = [%d]", cs_id);

	ret = nand_io_chip_deselect(context, cs_id);
	if (ret)
		RETURN_ERR(ret, "nand io chip deselect error, cs = [%d]", cs_id);

	nand_io_close(context);

	fid->id = ((nand_id[0] << 8) | nand_id[1]);
	fid->extid = ((nand_id[4] << 16) | (nand_id[3] << 8) | nand_id[2]);

	ndd_debug("get nand io chip[%d] id: id = [%04x], extid = [%08x]!\n", cs_id, fid->id, fid->extid);

	return 0;
}

struct hy_rr_msg {
	unsigned int offset;
        unsigned char addr1[2];
        unsigned char wdata[2];
        unsigned char addr2[5];
};

struct hy_rr_msg hy_rr_f20_64g_a = {
	2,
	{0xff, 0xcc},
	{0x40, 0x4c},
	{0x00, 0x00, 0x00, 0x02, 0x00},
};

struct hy_rr_msg hy_rr_f20_64g_b = {
	2,
	{0xae, 0xb0},
	{0x00, 0x4d},
	{0x00, 0x00, 0x00, 0x02, 0x00},
};

struct hy_rr_msg hy_rr_f20_32g = {
	2,
	{0xae, 0xb0},
	{0x40, 0x4d},
	{0x00, 0x00, 0x00, 0x02, 0x00},
};

struct hy_rr_msg hy_rr_f1y_64g = {
	16,
	/**
	 * 0xff is not legal cmd in this area,
	 * because 0xff is RESET CMD, so here we
	 * use to indicate invalid value
	 **/
	{0x38, -1},
	{0x52, -1},
	{0x00, 0x00, 0x00, 0x021, 0x00},
};

int get_retry_parms(nfi_base *base, unsigned int cs_id, rb_info *rbinfo, retry_parms *retryparms)
{
	int i, ret, context, datasize;
	int retry_flag, retrycnt = 0;
	struct hy_rr_msg *msg = NULL;
	unsigned char inversed_val;
	unsigned char *buf = retryparms->data;
        unsigned char cmd[4] = {0x16, 0x17, 0x04, 0x19};

	switch (retryparms->mode) {
	case HY_RR_F26_32G_MLC:
		retryparms->cycle = 6;
		retryparms->regcnt = 4;
		return 0;
	case HY_RR_F20_64G_MLC_A:
		retryparms->cycle = 8;
		retryparms->regcnt = 8;
		msg = &hy_rr_f20_64g_a;
		break;
	case HY_RR_F20_64G_MLC_B:
		retryparms->cycle = 8;
		retryparms->regcnt = 8;
		msg = &hy_rr_f20_64g_b;
		break;
	case HY_RR_F20_32G_MLC_C:
		retryparms->cycle = 8;
		retryparms->regcnt = 8;
		msg = &hy_rr_f20_32g;
		break;
	case HY_RR_F1Y_64G_MLC:
		retryparms->cycle = 8;
		retryparms->regcnt = 4;
		msg = &hy_rr_f1y_64g;
		break;
	default:
		RETURN_ERR(ENAND, "unknown retry mode");
	}

	/* io open */
	context = nand_io_open(base, NULL);
	if (!context)
		RETURN_ERR(ENAND, "nand io open error");

	/* chip select */
	ret = nand_io_chip_select(context, cs_id);
	if (ret)
		GOTO_ERR(chip_select);

	/* reset */
	nand_io_send_cmd(context, CMD_RESET_1ST, 300);
	if ((wait_unknown_rb_timeout(rbinfo, 8)) == NULL) {
		ret = TIMEOUT;
		GOTO_ERR(get_data);
	}

	/* send read otp cmd and data */
	nand_io_send_cmd(context, 0x36, 300);
	for (i = 0; i < 2; i++) {
		if (msg->addr1[i] != -1) {
			nand_io_send_spec_addr(context, msg->addr1[i], 1, 300);
			nand_io_send_data(context, &(msg->wdata[i]), 1);
		}
	}
        for (i = 0; i < 4; i++)
                nand_io_send_cmd(context, cmd[i], 300);

	/* sent read cmd */
	nand_io_send_cmd(context, CMD_PAGE_READ_1ST, 300);
	for (i = 0; i < 5; i++)
		nand_io_send_spec_addr(context, msg->addr2[i], 1, 300);
	nand_io_send_cmd(context, CMD_PAGE_READ_2ND, 300);
	if ((wait_unknown_rb_timeout(rbinfo, 8)) == NULL) {
		ret = TIMEOUT;
		GOTO_ERR(get_data);
	}

	/* ignore recycle and regcnt data */
	nand_io_receive_data(context, buf, (msg->offset - 0));

	/* calc datasize */
	datasize = retryparms->cycle * retryparms->regcnt;

	/* receive and check data */
retry:
	retry_flag = 0;
	nand_io_receive_data(context, buf, datasize);

	for (i = 0; i < datasize; i++) {
		nand_io_receive_data(context, &inversed_val, 1);
		if (buf[i] != (unsigned char)(~(inversed_val))) {
			ndd_debug("buf[%d] = %02x, ~(inversed_val) = %02x\n", i, buf[i], ~(inversed_val));
			retry_flag = 1;
		}
	}

	if (retry_flag) {
		if (retrycnt++ < 8) {
			goto retry;
		} else {
			ret = IO_ERROR;
			GOTO_ERR(get_data);
		}
	}

	/* reset */
	nand_io_send_cmd(context, CMD_RESET_1ST, 300);
	if ((wait_unknown_rb_timeout(rbinfo, 8)) == NULL) {
		ret = TIMEOUT;
		GOTO_ERR(get_data);
	}

	if (retryparms->mode == HY_RR_F1Y_64G_MLC) {
		unsigned char tmp = 0x00;
		nand_io_send_cmd(context, 0x36, 300);
		nand_io_send_spec_addr(context, 0x38, 1, 300);
		nand_io_send_data(context, &tmp, 1);
		nand_io_send_cmd(context, 0x16, 300);
		/* read any page */
		nand_io_send_cmd(context, CMD_PAGE_READ_1ST, 300);
		for (i = 0; i < 5; i++)
			nand_io_send_spec_addr(context, msg->addr2[i], 1, 300);
		nand_io_send_cmd(context, CMD_PAGE_READ_2ND, 300);
	} else
		nand_io_send_cmd(context, 0x38, 300);

	if ((wait_unknown_rb_timeout(rbinfo, 8)) == NULL) {
		ret = TIMEOUT;
		GOTO_ERR(get_data);
	}

	ret = SUCCESS;
	dump_retry_parms(retryparms);

ERR_LABLE(get_data):
	nand_io_chip_deselect(context, cs_id);
ERR_LABLE(chip_select):
	nand_io_close(context);

	return ret;
}

int __set_features(int io_context, rb_info *rbinfo, const nand_timing *timing,
		   unsigned char addr, unsigned char *data, int len)
{
	nand_io_send_cmd(io_context, CMD_SET_FEATURES, 0);
	nand_io_send_spec_addr(io_context, addr, 1, timing->tADL);
	nand_io_send_data(io_context, data, len);
	ndelay(timing->tWB);
	if ((wait_unknown_rb_timeout(rbinfo, 8)) == NULL)
		RETURN_ERR(TIMEOUT, "nand io wait rb timeout");

	return 0;
}

int __get_features(int io_context, rb_info *rbinfo, const nand_timing *timing,
		   unsigned char addr, unsigned char *data, int len)
{
	nand_io_send_cmd(io_context, CMD_GET_FEATURES, 0);
	nand_io_send_spec_addr(io_context, addr, 1, timing->tWB);
	if ((wait_unknown_rb_timeout(rbinfo, 8)) == NULL)
		RETURN_ERR(TIMEOUT, "nand io wait rb timeout");
	ndelay(timing->tRR);
	nand_io_receive_data(io_context, data, len);

	return 0;
}

static int __nand_set_features(int io_context, rb_info *rbinfo, const nand_timing *timing,
			unsigned char addr, unsigned char *data, int len)
{
	int i, ret;
	unsigned char rdata[4] = {0x00};

	ret = __set_features(io_context, rbinfo, timing, addr, data, len);
	if (ret)
		RETURN_ERR(ENAND, "set Nand feature faild, addr = 0x%02x", addr);

	ret = __get_features(io_context, rbinfo, timing, addr, rdata, 4);
	if (ret)
		RETURN_ERR(ENAND, "get Nand feature faild, addr = 0x%02x", addr);

	for (i = 0; i < len; i++) {
		if (data[i] != rdata[i])
			RETURN_ERR(ENAND, "set Nand feature error, addr = 0x%02x,"
				   " data[%d] = 0x%02x, rata[%d] = 0x%02x", i, data[i], i, rdata[i]);
	}

	return 0;
}

int nand_set_features(nfi_base *base, unsigned int cs_id, rb_info *rbinfo, chip_info *cinfo)
{
	int io_context, ret;
	unsigned char data[4] = {0x00};
	const nand_timing *timing = cinfo->timing;

	io_context = nand_io_open(base, NULL);
	if (!io_context)
		RETURN_ERR(ENAND, "nand io open error");

	ret = nand_io_chip_select(io_context, cs_id);
	if (ret)
		RETURN_ERR(ENAND, "nand io chip select error, cs = [%d]", cs_id);

	/* set timimg mode */
	if (SUPPROT_TIMING_MODE(cinfo)) {
		data[0] = TIMING_MODE(cinfo);
		data[1] = TIMING_MODE(cinfo) >> 8;
		data[2] = TIMING_MODE(cinfo) >> 16;
		data[3] = TIMING_MODE(cinfo) >> 24;
		ret = __nand_set_features(io_context, rbinfo, timing, 0x01, data, 4);
		if (ret)
			RETURN_ERR(ENAND, "set Nand flash timing mode faild!");
	}

	/* set driver strength */
	if (SUPPROT_DRIVER_STRENGTH(cinfo)) {
		if (cinfo->drv_strength != DRV_STRENGTH_DEFAULT) {
			if (cinfo->manuf == NAND_MFR_MICRON) {
				if (cinfo->drv_strength == DRV_STRENGTH_LEVEL0)
					data[0] = MR_DRIVER_STRENGTH_UNDER;
				else if (cinfo->drv_strength == DRV_STRENGTH_LEVEL1)
					data[0] = MR_DRIVER_STRENGTH_NORMAL;
				else if (cinfo->drv_strength == DRV_STRENGTH_LEVEL2)
					data[0] = MR_DRIVER_STRENGTH_OVER1;
				else if (cinfo->drv_strength == DRV_STRENGTH_LEVEL3)
					data[0] = MR_DRIVER_STRENGTH_OVER2;
				else
					RETURN_ERR(ENAND, "unsupport driver strength!");

				data[1] = 0x00 >> 8;
				data[2] = 0x00 >> 16;
				data[3] = 0x00 >> 24;
				ret = __nand_set_features(io_context, rbinfo, timing, 0x10, data, 4);
				if (ret)
					RETURN_ERR(ENAND, "set Nand flash driver strength faild!");
			}
		}
	}

	/* set rb pull_down strength */
	if (SUPPROT_RB_PULL_DOWN_STRENGTH(cinfo)) {
		data[0] = cinfo->rb_pulldown;
		data[1] = 0x00 >> 8;
		data[2] = 0x00 >> 16;
		data[3] = 0x00 >> 24;
		ret = __nand_set_features(io_context, rbinfo, timing, 0x81, data, 4);
		if (ret)
			RETURN_ERR(ENAND, "set Nand flash pull_down strength faild!");
	}

	ret = nand_io_chip_deselect(io_context, cs_id);
	if (ret)
		RETURN_ERR(ret, "nand io chip deselect error, cs = [%d]", cs_id);

	nand_io_close(io_context);

	return 0;
}
