#ifndef _MEMDEALER_H_
#define _MEMDEALER_H_

#include "bilist.h"

#define MEMDEALER_DEBUG

struct MemoryDealer
{
	void *heap;
	unsigned int heapsize;
	int zmid;
	int memadpt;
	struct bilist_head top;
};
#define NO_INIT 0
int InitContinueMemory(void *h,int size);
void DeinitContinueMemory(int mid);

void *Allocate(int mid,int size);
void *PageAllocate(int mid,int size);
void Deallocate(int mid,void *v);

void* Nand_MemoryInit(void *h,int size,int type);
void Nand_MemoryDeinit(void);

void * __Nand_VirtualAlloc(int size);
void __Nand_VirtualFree(void *val);
void * __Nand_ContinueAlloc(int size);
void __Nand_ContinueFree(void *val);

#ifdef MEMDEALER_DEBUG
extern void mm_dbg_valloc(void *heap, int line, const char *fun_name);
extern void mm_dbg_vfree(void *heap, int line, const char *fun_name);
extern void print_vmem_history(void);
extern void mm_dbg_calloc(void *heap, int line, const char *fun_name);
extern void mm_dbg_cfree(void *heap, int line, const char *fun_name);
extern void print_cmem_history(void);
#define Nand_VirtualAlloc(size)					\
	({							\
		void *__heap;					\
		__heap = __Nand_VirtualAlloc(size);		\
		mm_dbg_valloc(__heap, __LINE__, __FUNCTION__);	\
		__heap;						\
	})
#define Nand_VirtualFree(val)					\
	do {							\
		mm_dbg_vfree(val, __LINE__, __FUNCTION__);	\
		__Nand_VirtualFree(val);			\
	} while (0)
#define Nand_VirtualPrintHistory()	print_vmem_history()
#define Nand_ContinueAlloc(size)				\
	({							\
		void *__heap;					\
		__heap = __Nand_ContinueAlloc(size);		\
		mm_dbg_calloc(__heap, __LINE__, __FUNCTION__);	\
		__heap;						\
	})
#define Nand_ContinueFree(val)					\
	do {							\
		mm_dbg_cfree(val, __LINE__, __FUNCTION__);	\
		__Nand_ContinueFree(val);			\
	} while (0)
#define Nand_ContinuePrintHistory()	print_cmem_history()
#else
#define Nand_VirtualAlloc(size)		__Nand_VirtualAlloc(size)
#define Nand_VirtualFree(val)		__Nand_VirtualFree(val)
#define Nand_VirtualPrintHistory()
#define Nand_ContinueAlloc(size)	__Nand_ContinueAlloc(size)
#define Nand_ContinueFree(val)		__Nand_ContinueFree(val)
#define Nand_ContinuePrintHistory()
#endif

#endif /* _MEMDEALER_H_ */
