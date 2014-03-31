#include <malloc.h>
#include "nand_chip.h"
#include "convert_img.h"
#include "jz4775.h"

#define SECTOR_SIZE 512
#define MAX_PARTITION_NUM 20
#define ZM_MEMORY_SIZE  (8*1024*1024)

extern nand_flash nand_flash_info;
extern int nd_raw_boundary;

struct Ghandle{
	int pagesize;
	int pageperblock;
	int nd_rbcnt;
	unsigned int  sectorid;
	int zm_handle;
	int  pphandle[MAX_PARTITION_NUM];
	char  pphandle_index;
	char  curpt_index;
	char  uperrorpt;
	char eraseall;
	PPartition *m_ppt;
	LPartition* lp;	
}g_handle = {
	.sectorid = 0,
	.pphandle_index = 0,
	.curpt_index = 0,
	.uperrorpt = 1,
};

/*for one rb*/
//Nandppt ndppt[5]={{"ndxboot",0,8,0,0,{{0},{0},{0},{0}},},{"ndboot",8,16,1,0,{{0},{0},{0},{0}},},{"ndrecovery",24,16,1,0,{{0},{0},{0},{0}},},{"ndsystem",64,512,2,0,{{0},{0},{0},{0}},},{"ndextern",576,-1,2,0,{{"ndcache",0,128,},{"nddata",128,1024,},{"ndmisc",(128 + 1024),-1,}},},};

//Nandppt ndppt[7]={{"ndxboot",0,8,0,0},{"ndboot",8,16,1,0},{"ndrecovery",24,16,1,0},{"ndsystem",64,512,2,0},{"nddata",(576+128),1024,2,0},{"ndcache",576,128,2,0},{"ndmisc",(576+128+1024),/*794624*/-1,2,0}};

/*for two rb --> 16G*/
//Nandppt ndppt[7]={{"ndxboot",0,1024*2,0,0},{"ndboot",1024*2,2048*2,1,0},{"ndrecovery",3072*2,2048*2,1,0},{"ndsystem",8192*2,65536,2,0},{"nddata",81920,131072,2,0},{"ndcache",212992,16384,2,0},{"ndmisc",229376,1884160,2,0}};
//Nandppt ndppt[7]={{"ndxboot",0,1024,0,0},{"ndboot",1024,2048,1,0},{"ndrecovery",3072,2048,1,0},{"ndsystem",8192,65536,2,0},{"nddata",73728,131072,2,0},{"ndcache",204800,16384,2,0},{"ndmisc",221184,/*1884160*/-1,2,0}};
int erase_partition_fill_pphandle(unsigned int startpage, int pt_index);
extern int nand_probe(PartitionInfo *pinfo);

static int jz_strcmp(char *s1, char *s2)
{
  while (*s1 != '\0' && *s1 == *s2)
    {
      s1++;
      s2++;
    }

  return (*(unsigned char *) s1) - (*(unsigned char *) s2);
}

static void start(int handle)
{
	g_handle.m_ppt = ((PManager*)(g_handle.zm_handle))->vnand->pt->ppt;
	switch (g_handle.eraseall) {
                /*if force eraseall,then skip erase each partition before write*/
		g_handle.uperrorpt = 0;
	case 1:
		serial_printf("NANDMANAGER: Nand Manager normal erase all the flash!\n");
		NandManger_Ioctrl(g_handle.zm_handle, NANDMANAGER_NORMAL_ERASE_FLASH, 0);
		break;
	case 2:
		serial_printf("NANDMANAGER: Nand Manager force erase all the flash!\n");
		NandManger_Ioctrl(g_handle.zm_handle, NANDMANAGER_FORCE_ERASE_FLASH, 0);
		break;
	case 3:
		serial_printf("NANDMANAGER: Nand Manager prepare new flash!\n");
		NandManger_Ioctrl(g_handle.zm_handle, NANDMANAGER_PREPARE_NEW_FLASH, 0);
		break;
	default:
		break;
	}

	serial_printf("NANDMANAGER: Nand Manager scan bad blocks!\n");
	NandManger_Ioctrl(g_handle.zm_handle, NANDMANAGER_SCAN_BADBLOCKS,0);
	/*update all the partition with CPU_OPS for read/write/erase*/
	NandManger_getPartition(g_handle.zm_handle,&g_handle.lp);
}

unsigned int jz_strlen(const char *s)
{
    int len = 0;
    while(*s++) len++;
    return len;
}

char * jz_strncpy(char *dest,const char *src, unsigned int count)
{
	while(count){
		if((*dest = *src) != 0)
			src++;
		dest++;
		count--;
	}
	return dest;
}

int burn_nandmanager_init(PartitionInfo *pinfo, int eraseall)
{
	void *heap = (void*)malloc(ZM_MEMORY_SIZE);
	if(!heap){
		serial_printf("%s %d malloc heap error!\n",__func__,__LINE__);
		return -1;
	}
	/* init global structure g_handle*/
	g_handle.zm_handle = NandManger_Init(heap,ZM_MEMORY_SIZE,0);
	g_handle.eraseall = eraseall;
	g_handle.nd_rbcnt = pinfo->rbcount;
	g_handle.pagesize = nand_flash_info.pagesize;
	g_handle.pageperblock = nand_flash_info.blocksize / nand_flash_info.pagesize;
	memset(g_handle.pphandle, 0xff, MAX_PARTITION_NUM);

	NandManger_startNotify(g_handle.zm_handle, start, g_handle.zm_handle);

	nand_probe(pinfo);

	return g_handle.zm_handle;
}
//#define DEBUG_PTWRITE
unsigned int do_nand_request(unsigned int startaddr, void *Bulk_out_buf, unsigned int ops_length,unsigned int offset)
{
	int pHandle;
	void *databuf = Bulk_out_buf;
#ifdef DEBUG_PTWRITE
	char readbuf[512*1024]={0};
#endif
	int totalbytes = ops_length;
	unsigned int wlen = 0;
	SectorList *sl;
	unsigned int rsectorid;
	int bl;
	int pt_index;
	int ret = startaddr + ops_length;
	int pt_startpage,pt_endpage;

	startaddr = startaddr/g_handle.pagesize;
	g_handle.sectorid = offset/512;

	/*handle two rb,find the correct pt_index*/
	if(g_handle.nd_rbcnt > 1 && startaddr != 0){
		if(startaddr*g_handle.pagesize < 64*1024*1024)
			startaddr *= 2;
		else
			startaddr += (nd_raw_boundary * g_handle.pageperblock);
	}
	for(pt_index = 0; pt_index < MAX_PARTITION_NUM; pt_index++){
		pt_startpage = g_handle.m_ppt[pt_index].startPage;
		pt_endpage = pt_startpage + g_handle.m_ppt[pt_index].totalblocks * g_handle.m_ppt[pt_index].pageperblock;
		//serial_printf("$$$$ pt_index=%d startpage=%d endpage=%d startaddr=%d  nd_raw_boundary=%d $$$$$\n",pt_index,pt_startpage,pt_endpage,startaddr,nd_raw_boundary);
		if( pt_startpage <=startaddr && (startaddr + ops_length / g_handle.pagesize) < pt_endpage){
			break;
		}
	}
	if(pt_index == MAX_PARTITION_NUM)
		serial_printf("%s ERROR !!!!!!!!!!!!!!!!!!\n",__func__);
	//serial_printf("pt[%d]:%s ops_length = %d pagesize=%d totalbytes=%d\n",pt_index,__func__,ops_length,g_handle.pagesize,totalbytes);
	erase_partition_fill_pphandle(startaddr, pt_index);
	pHandle = g_handle.pphandle[pt_index];
	if(g_handle.curpt_index != pt_index){
		g_handle.sectorid = 0;
		g_handle.curpt_index = pt_index;
	}

	if(pHandle == -1){
		serial_printf("the partition is not open,so the handle is -1,checkout the index!\n");
		return -1;
	}
	bl = BuffListManager_BuffList_Init();
	if(bl == 0){
		serial_printf("BuffListManager Init failed!\n");
		return -1;
	}
	while(totalbytes){
		if(totalbytes >= 256 *512)
			wlen = 256 * 512;
		else{
			wlen = totalbytes;
			if(wlen % 512 != 0)
				memset(databuf+wlen, 0xff, 512-wlen%512);
		}
		sl = BuffListManager_getTopNode(bl,sizeof(SectorList));
		if(sl == 0){
			serial_printf("Bufferlist request sectorlist failed!\n");
			return -1;
		}
		rsectorid = g_handle.sectorid;
		sl->startSector = g_handle.sectorid;
		sl->pData = (void*)databuf;
		sl->sectorCount = (wlen + 511)/ 512;
		g_handle.sectorid += (wlen + 511)/ 512;
rewrite:
		if(NandManger_ptWrite(pHandle,sl) < 0){
			serial_printf("NandManger_ptWrite failed, now rewrite!\n");
			goto rewrite;
		}
		if(sl->startSector == 0){
			sl->sectorCount = (wlen + 511)/ 512;
		}
#ifdef DEBUG_PTWRITE
		sl->startSector = rsectorid;
		sl->pData = (void*)readbuf;
		if(NandManger_ptRead(pHandle,sl) < 0){
			serial_printf("NandManger_ptRead failed, now rewrite!\n");
			goto rewrite;
		}
		if(strncmp(readbuf,databuf,sl->sectorCount * 512))
		{
			serial_printf("NandManger_ptRead failed !!!!!!! \n");
			while(1);
		}
#endif
		databuf+=wlen;
		totalbytes-=wlen;
		BuffListManager_freeAllList(bl,(void **)&sl,sizeof(SectorList));
	}
	BuffListManager_BuffList_DeInit(bl);

	return ret;
}

static int NM_PtDirecterase(PPartition *pt)
{
	int blockid,markcnt;
	int ret;
	BlockList bl;

	serial_printf("%s info: pt[%s] totalblocks= %d\n",__func__,pt->name,pt->totalblocks);
	for(blockid = 0; blockid < pt->totalblocks; blockid++){
		bl.startBlock = blockid;
		bl.BlockCount = 1;
		bl.head.next = NULL;
/*		if(NandManger_DirectIsBadBlock(0, pt, &bl)){
			serial_printf("xxxxxxxxxxx WARNING xxxxxxxxxx\n");
			serial_printf("%s pt[%s]:blockid=%d is badblock!\n",__func__,pt->name,blockid);
			serial_printf("xxxxxxxxxxx ENDING xxxxxxxxxx\n");
			continue;
		}
*/
		ret = NandManger_DirectErase(0, pt, &bl);
		if(ret){
			ret = NandManger_DirectMarkBadBlock(0, pt, blockid);
			if(ret){
				serial_printf("============== ERROR ATENTION ================\n");
				serial_printf("%s DirectMarkBadBlock Fail. pt[%s],blockid=%d,ret=%d\n",__func__,pt->name,blockid,ret);
				serial_printf("=============== MARK FAILED ==================\n");
			}
			markcnt++;
		}
	}
	return markcnt;
}

int NM_erase_and_updataerrpt(int blocknum, int startblockid)
{
	LPartition *lpentry;
	struct singlelist *it;
	int pt_index,ret = 0;
	PPartition *pt;
	if(g_handle.nd_rbcnt > 1 && startblockid != 0){
		if(startblockid * g_handle.pageperblock * g_handle.pagesize < 64*1024*1024)
			startblockid *= 2;
		else
			startblockid += nd_raw_boundary;
	}

	//serial_printf("[%s]-------> startblockid=%d nd_raw_boundary=%d\n",__func__,startblockid,nd_raw_boundary);
	for(pt_index = 0; pt_index < MAX_PARTITION_NUM; pt_index++){
		if(g_handle.m_ppt[pt_index].startblockID == startblockid){
			if(g_handle.m_ppt[pt_index].totalblocks != blocknum)
				serial_printf("pt[%s] erase totalblocks [%d] change to [%d]\n",
					      g_handle.m_ppt[pt_index].name,blocknum,g_handle.m_ppt[pt_index].totalblocks);
			break;
		}

	}
	if(pt_index == MAX_PARTITION_NUM){
		serial_printf("\nERROR !\n"
			      "---------------------------------ATTENTION--------------------------------\n"
			      "the file startpage[%d] isn't match the partitioninfo you had filled in UI.\n"
			      "please check the file startpage in UI and then refill\n!",startblockid*g_handle.pageperblock);
		while(1);
	}
	pt = &(g_handle.m_ppt[pt_index]);
        /*splmanager and simplemanager will erase and update its partition*/
	if(pt_index < 2)
		goto exit;
	if(g_handle.uperrorpt){
		NM_PtDirecterase(pt);
		ret = vNand_UpdateErrorPartition(((PManager*)g_handle.zm_handle)->vnand, pt);
	}
exit:
	singlelist_for_each(it,&(g_handle.lp->head)){
		lpentry = singlelist_entry(it,LPartition,head);
		if(jz_strcmp(lpentry->name,pt->name) == 0){
			g_handle.pphandle[g_handle.pphandle_index] = NandManger_ptOpen(g_handle.zm_handle,lpentry->name,lpentry->mode);
			g_handle.pphandle_index = 0;
			break;
		}
		g_handle.pphandle_index++;
	}

	return ret;
}

int erase_partition_fill_pphandle(unsigned int startpage, int pt_index)
{
	PPartition *pt = &(g_handle.m_ppt[pt_index]);
	LPartition *lpentry;
	struct singlelist *it;
	int ret = 0;
//	serial_printf("===========> startpage = %d pt_startblock[%d] pt_pageperblock[%d] pt_startpage[%d]\n",startpage,pt->startblockID,pt->pageperblock,pt->startPage);
	if(startpage == pt->startPage){
		if(g_handle.eraseall == 0){
			serial_printf("%s %d erase and update PPartition!\n",__func__,__LINE__);
			NM_PtDirecterase(pt);
			ret = vNand_UpdateErrorPartition(((PManager*)g_handle.zm_handle)->vnand, pt);
		}
		//serial_printf("\nlp=%x\n",(int)(g_handle.lp));
		singlelist_for_each(it,&(g_handle.lp->head)){
			lpentry = singlelist_entry(it,LPartition,head);
			if(jz_strcmp(lpentry->name,pt->name) == 0){
				g_handle.pphandle[g_handle.pphandle_index] = 
					NandManger_ptOpen(g_handle.zm_handle,lpentry->name,lpentry->mode);
				//serial_printf("====================>  lpentry->name = %s pt->name = %s g_handle.pphandle_index = %d pphandle = 0x%08x\n",lpentry->name,pt->name,g_handle.pphandle_index,g_handle.pphandle[g_handle.pphandle_index]);
				g_handle.pphandle_index = 0;
				break;
			}
			g_handle.pphandle_index++;
		}
		serial_printf("%s %d erase and open pt[%s] ok!\n",__func__,__LINE__,pt->name);
	}

	return ret;
}
