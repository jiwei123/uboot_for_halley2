#define MAX_SEM_COUNT 100
typedef int NandSemaphore;
typedef int NandMutex;
struct semphoretype
{
	unsigned short lock;
	unsigned short alloc;
};
volatile struct semphoretype semphore[MAX_SEM_COUNT] = {0};
#if 0
static void *alloc_sem(int val) {
	int i;
	for(i = 0;i < MAX_SEM_COUNT;i++) {
		if(semphore[i].alloc == 0) {
			break;
		}
	}
	if(i >= MAX_SEM_COUNT) {
		//nd_print(3,"sem too many!");
		//nd_print("sem too many!");
		while(1);
	}
	semphore[i].alloc = 1;
	semphore[i].lock = val;
	return &semphore[i];
}

static void free_sem(void *sem) {
	struct semphoretype *d = (struct semphoretype *)sem;
	d->alloc = 0;
}
#endif
//val 1 is unLock val 0 is Lock
void __InitSemaphore(NandSemaphore *sem,int val)
{
#if 0
	void *__sem = alloc_sem(val);
	*sem = (NandSemaphore)__sem;
#endif
}

void __DeinitSemaphore(NandSemaphore *sem)
{
#if 0
	free_sem((void *)sem);
#endif
}

void __Semaphore_wait(NandSemaphore *sem)
{
#if 0
	struct semphoretype *d = (struct semphoretype *)sem;
	while(d->lock);
#endif
}

//timeout return < 0
int __Semaphore_waittimeout(NandSemaphore *sem,long jiffies)
{
	return 0;
#if 0
	int j = 0;
	struct semphoretype *d = (struct semphoretype *)sem;
	while(d->lock) {
		j++;
		if(j > 0xfffff *jiffies)
			break;
	}
	return j / 0xfffff;
#endif
}

void __Semaphore_signal(NandSemaphore *sem)
{
#if 0
	struct semphoretype *d = (struct semphoretype *)sem;
	d->lock++;
#endif
}

//#define DEBUG_NDMUTEX
void __InitNandMutex(NandMutex *mutex)
{
#if 0
	InitSemaphore((NandSemaphore *)mutex, 1);
#endif
}

void __DeinitNandMutex(NandMutex *mutex)
{
#if 0
	DeinitSemaphore((NandSemaphore *)mutex);
#endif
}

void __NandMutex_Lock(NandMutex *mutex)
{
#if 0
	Semaphore_wait((NandSemaphore *)mutex);
#endif
}

void __NandMutex_Unlock(NandMutex* mutex)
{
#if 0
	Semaphore_signal((NandSemaphore *)mutex);
#endif
}

int __NandMutex_TryLock(NandMutex *mutex)
{
	return 0;
#if 0
	return pthread_mutex_trylock((pthread_mutex_t *)(*mutex));
#endif
}
