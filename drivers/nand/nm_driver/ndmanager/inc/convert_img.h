#ifndef __CONVERT_IMG_H_
#define __CONVERT_IMG_H_

#include <ingenic_nand_mgr/nand_param.h>
#include "nandmanagerinterface.h"
#include "uperrorpartition.h"
#include "pagelist.h"
#include "blocklist.h"
#include "ppartition.h"
#include "nandinterface.h"
#include "vnandinfo.h"
#include "memorydealer.h"
#include "singlelist.h"
#include "pmanager.h"
#include "os/clib.h"

#define NM_DATA_SIZE  4*1024*1024

#define NORMAL_MTD_BURN  0
#define SD_CARD_BURN     1
#define ZONE_MANGER_BURN 2

int burn_nandmanager_init(PartitionInfo *pinfo, int eraseall);
unsigned int convert_img_to_bin(unsigned int startaddr, void *Bulk_out_buf, unsigned int ops_length);
int NM_erase_and_updataerrpt(int blocknum, int startblockid);
unsigned int strlen(const char *s);
char * strncpy(char *dest,const char *src, unsigned int count);

#endif
