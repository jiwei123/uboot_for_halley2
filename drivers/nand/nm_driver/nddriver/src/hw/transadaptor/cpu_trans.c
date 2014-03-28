#include <os_clib.h>
#include <nand_debug.h>
#include "cpu_trans.h"
#include "transadaptor.h"

static void src_add_to_dst_8bit(unsigned char *src, unsigned char *dst, unsigned int len)
{
	unsigned int i;

	for(i=0; i<len; i++)
		*(volatile unsigned char *)dst = src[i];
}

static void src_add_to_dst_16bit(unsigned char *src, unsigned char *dst, unsigned int len)
{
	unsigned int i;
	unsigned int u16_len = (len + 1) >> 1;
	unsigned short *u16_src = (unsigned short *)src;

	for(i=0; i<u16_len; i++)
		*(volatile unsigned short *)dst = u16_src[i];
}

static void src_to_dst_add_8bit(unsigned char *src, unsigned char *dst, unsigned int len)
{
	unsigned int i;

	for(i=0; i<len; i++)
		dst[i] = *(volatile unsigned char *)src;
}

static void src_to_dst_add_16bit(unsigned char *src, unsigned char *dst, unsigned int len)
{
	unsigned int i;
	unsigned int u16_len = (len + 1) >> 1;
	unsigned short *u16_dst = (unsigned short *)dst;

	for(i=0; i<u16_len; i++)
		u16_dst[i] = *(volatile unsigned short *)src;
}

static void src_to_dst_both_add_8bit(unsigned char *src, unsigned char *dst, unsigned int len)
{
	unsigned int i;

	for(i=0; i<len; i++)
		dst[i] = *(volatile unsigned char *)src++;
}

static void src_to_dst_both_add_16bit(unsigned char *src, unsigned char *dst, unsigned int len)
{
	unsigned int i;
	unsigned int u16_len = (len + 1) >> 1;
	volatile unsigned short *u16_src = (volatile unsigned short *)src;
	unsigned short *u16_dst = (unsigned short *)dst;

	for(i = 0; i < u16_len; i++)
		u16_dst[i] = u16_src[i];
}

int cpu_prepare_memcpy(int context, unsigned char *src, unsigned char *dst, unsigned int len, unsigned short flag)
{
	cpu_copy_info *info = (cpu_copy_info *)context;

	if(src == NULL || dst == NULL || len == 0)
		RETURN_ERR(ENAND, "memcpy failed, src = %p dst = %p len = %d", src, dst, len);

	info->src = src;
	info->dst = dst;
	info->len = len;

	if(flag == SRCADD){
		info->copy_data = info->src_add_to_dst;
	}else if(flag == DSTADD){
		info->copy_data = info->src_to_dst_add;
	}else if(flag == SRC_AND_DST_ADD){
		info->copy_data = info->src_to_dst_both_add;
	}else
		RETURN_ERR(ENAND, "memcpy flag is invalid, flag = %d", flag);

	return 0;
}

int cpu_finish_memcpy(int context)
{
	cpu_copy_info *info = (cpu_copy_info *)context;

	info->copy_data(info->src,info->dst,info->len);
	return 0;
}

int cpu_move_init(transadaptor *trans, unsigned int burstlen)
{
	cpu_copy_info *copy_info;

	copy_info = ndd_alloc(sizeof(cpu_copy_info));
	if(!copy_info)
		RETURN_ERR(ENAND, "alloc memory error");

	if (burstlen == 8) {
		copy_info->src_add_to_dst = src_add_to_dst_8bit;
		copy_info->src_to_dst_add = src_to_dst_add_8bit;
		copy_info->src_to_dst_both_add = src_to_dst_both_add_8bit;
	} else if (burstlen == 16) {
		copy_info->src_add_to_dst = src_add_to_dst_16bit;
		copy_info->src_to_dst_add = src_to_dst_add_16bit;
		copy_info->src_to_dst_both_add = src_to_dst_both_add_16bit;
	} else
		RETURN_ERR(ENAND, "cpu trans burstlen not support, burstlen = %d", burstlen);

	trans->prepare_memcpy = cpu_prepare_memcpy;
	trans->finish_memcpy = cpu_finish_memcpy;
	return (int)copy_info;
}

void cpu_move_deinit(int context, transadaptor *trans)
{
	cpu_copy_info *copy_info = (cpu_copy_info *)context;

	ndd_free(copy_info);
	trans->prepare_memcpy = NULL;
	trans->finish_memcpy = NULL;
}
