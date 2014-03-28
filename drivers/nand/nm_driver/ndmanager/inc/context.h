#ifndef __CONTEXT_H__
#define __CONTEXT_H__

#include "vnandinfo.h"
#include "cacheinfo.h"
#include "sigzoneinfo.h"
#include "bufflistmanager.h"
#include "l1info.h"
#include "zonemanager.h"
#include "cachemanager.h"
#include "recycle.h"
#include "junkzone.h"
#include "timerdebug.h"

#define SECTOR_SIZE 512
#define INTERNAL_TIME  100000000
#define RECHECK_VALIDPAGE

/* flags */
#define NM_NO_ERROR	0x01

typedef struct _Context Context;

struct _Context {
	ZoneManager *zonep;
	int blm;
	CacheManager *cachemanager;
	Recycle *rep;
	VNandInfo vnand;
	CacheInfo *cacheinfo;
	SigZoneInfo *top;
	L1Info *l1info;
	int thandle;
	int junkzone; //l2p recycle
	long long t_startrecycle;
	unsigned int L2InfoLen;
	unsigned int L3InfoLen;
	unsigned int L4InfoLen;
	int l2pid;
	int fs_totalsector;
	unsigned int pageinfo_filldata;
	unsigned int blocksperzone;
	int writecache;
#ifdef STATISTICS_DEBUG
	TimeByte *timebyte;
#endif
	int flags;
};

#define CONTEXT_VNAND(context) 	((Context *)context)->vnand
#endif
