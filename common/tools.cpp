/******************************************************************************
 *
 *  Copyright (C) 2015 NXP Semiconductors
 *
 *  Licensed under the Apache License, Version 2.0 (the "License")
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "tools.h"


void* framework_AllocMem(size_t size);
void  framework_FreeMem(void *ptr);

extern int printf_d(const char* fmt, ...);


/****************** Thread ****************************/


typedef struct tLinuxThread
{
	pthread_t thread;
	void* ctx;
	void* (*threadedFunc)(void *);
	void* mutexCanDelete;
}tLinuxThread_t;

void* thread_object_func(void* obj)
{
	tLinuxThread_t *linuxThread = (tLinuxThread_t *)obj;
	void *res = NULL;
	framework_LockMutex(linuxThread->mutexCanDelete);
	res = linuxThread->threadedFunc(linuxThread->ctx);
	framework_UnlockMutex(linuxThread->mutexCanDelete);

	return res;
}


eResult framework_CreateThread(void** threadHandle, void * (* threadedFunc)(void *) , void * ctx)
{
	tLinuxThread_t *linuxThread = (tLinuxThread_t *)framework_AllocMem(sizeof(tLinuxThread_t));
	
	linuxThread->ctx = ctx;
	linuxThread->threadedFunc = threadedFunc;
	framework_CreateMutex(&(linuxThread->mutexCanDelete));
	
	if (pthread_create(&(linuxThread->thread), NULL, thread_object_func, linuxThread))
	{
		printf("Cannot create Thread\n");
		framework_DeleteMutex(linuxThread->mutexCanDelete);
		framework_FreeMem(linuxThread);
		
		return FRAMEWORK_FAILED;
	}
	pthread_detach(linuxThread->thread);
	
	*threadHandle = linuxThread;
	
	return FRAMEWORK_SUCCESS;
}

void framework_JoinThread(void * threadHandle)
{
	tLinuxThread_t *linuxThread = (tLinuxThread_t*)threadHandle;
	if (pthread_self() != linuxThread->thread)
	{
		// Will cause block if thread still running !!!
		framework_LockMutex(linuxThread->mutexCanDelete);
		framework_UnlockMutex(linuxThread->mutexCanDelete);
		// Thread now just ends up !
	}
}


void framework_DeleteThread(void * threadHandle)
{
	tLinuxThread_t *linuxThread = (tLinuxThread_t*)threadHandle;
	framework_DeleteMutex(linuxThread->mutexCanDelete);
	framework_FreeMem(linuxThread);
}

void * framework_GetCurrentThreadId()
{
	return (void*)pthread_self();
}

void * framework_GetThreadId(void * threadHandle)
{
	tLinuxThread_t *linuxThread = (tLinuxThread_t*)threadHandle;
	return (void*)linuxThread->thread;
}

void framework_MilliSleep(uint32_t ms)
{
	usleep(1000*ms);
}

/****************** Mutex ****************************/

typedef struct tLinuxMutex
{
	pthread_mutex_t *lock;
	pthread_cond_t  *cond;
}tLinuxMutex_t;

eResult framework_CreateMutex(void ** mutexHandle)
{
	tLinuxMutex_t *mutex = (tLinuxMutex_t *)framework_AllocMem(sizeof(tLinuxMutex_t));
	
	mutex->lock = (pthread_mutex_t*)framework_AllocMem(sizeof(pthread_mutex_t));
	mutex->cond = (pthread_cond_t*)framework_AllocMem(sizeof(pthread_cond_t));
	
	pthread_mutex_init(mutex->lock,NULL);
	pthread_cond_init(mutex->cond,NULL);
	
	*mutexHandle = mutex;
	
	return FRAMEWORK_SUCCESS;
}

void framework_LockMutex(void * mutexHandle)
{
	//printf_d("Lock %p", mutexHandle);
	tLinuxMutex_t *mutex = (tLinuxMutex_t*)mutexHandle;
	
	int res = pthread_mutex_lock(mutex->lock);
	if (res)
	{
		printf("lock() failed errno %s\n", strerror(errno));
	}
}

void framework_UnlockMutex(void * mutexHandle)
{
	//printf_d("Unlock %p", mutexHandle);
	tLinuxMutex_t *mutex = (tLinuxMutex_t*)mutexHandle;
	int res = pthread_mutex_unlock(mutex->lock);
	if (res)
	{
		printf("unlock() failed %s\n", strerror(errno));
	}
}

void framework_WaitMutex(void * mutexHandle, uint8_t needLock)
{
	tLinuxMutex_t *mutex = (tLinuxMutex_t*)mutexHandle;
	
	if (needLock)
	{
		framework_LockMutex(mutexHandle);
	}

	pthread_cond_wait(mutex->cond,mutex->lock);
	
	if (needLock)
	{
		framework_UnlockMutex(mutexHandle);
	}
	
}

int framework_TimedWaitMutex(void * mutexHandle, uint8_t needLock, uint32_t waitMs)
{
	tLinuxMutex_t *mutex = (tLinuxMutex_t*)mutexHandle;
	
	if (needLock)
	{
		framework_LockMutex(mutexHandle);
	}

	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec += (waitMs / 1000);
	ts.tv_nsec += ((waitMs % 1000) * 1e6);
	if (ts.tv_nsec > 1e9)
	{
		ts.tv_sec += (ts.tv_nsec / (long)1e9);
		ts.tv_nsec = ts.tv_nsec % (long)1e9;
	}

	int res = pthread_cond_timedwait(mutex->cond,mutex->lock, &ts);
	
	if (needLock)
	{
		framework_UnlockMutex(mutexHandle);
	}
	
	return (res == ETIMEDOUT ? 1 : 0);
}

void framework_NotifyMutex(void * mutexHandle, uint8_t needLock)
{
	tLinuxMutex_t *mutex = (tLinuxMutex_t*)mutexHandle;
	
	if (needLock)
	{
		framework_LockMutex(mutexHandle);
	}

	pthread_cond_broadcast(mutex->cond);
	
	if (needLock)
	{
		framework_UnlockMutex(mutexHandle);
	}
}

void framework_DeleteMutex(void * mutexHandle)
{
	tLinuxMutex_t *mutex = (tLinuxMutex_t*)mutexHandle;
	
	pthread_mutex_destroy(mutex->lock);
	pthread_cond_destroy(mutex->cond);
	
	framework_FreeMem(mutex);
}

/****************** Memory Mgmt ****************************/


typedef struct sMemInfo
{
	uint32_t magic;
	size_t size;
} sMemInfo_t;

typedef struct sMemInfoEnd
{
	uint32_t magicEnd;
} sMemInfoEnd_t;


void* framework_AllocMem(size_t size)
{
	sMemInfo_t *info = NULL;
	sMemInfoEnd_t *infoEnd = NULL;
	uint8_t * pMem = (uint8_t *) malloc(size+sizeof(sMemInfo_t)+sizeof(sMemInfoEnd_t));
	info = (sMemInfo_t*)pMem;
	
	info->magic = 0xDEADC0DE;
	info->size  = size;

	pMem = pMem+sizeof(sMemInfo_t);

	memset(pMem,0xAB,size);

	infoEnd = (sMemInfoEnd_t*)(pMem+size);

	infoEnd->magicEnd = 0xDEADC0DE;
	return pMem;
}

void framework_FreeMem(void *ptr)
{
	if(NULL !=  ptr)
	{
		sMemInfoEnd_t *infoEnd = NULL;
		uint8_t *memInfo = (uint8_t*)ptr;
		sMemInfo_t *info = (sMemInfo_t*)(memInfo - sizeof(sMemInfo_t));

		infoEnd = (sMemInfoEnd_t*)(memInfo+info->size);

		if ((info->magic != 0xDEADC0DE)||(infoEnd->magicEnd != 0xDEADC0DE))
		{
			// Call Debugger
			*(int *)(uintptr_t)0xbbadbeef = 0;
		}else
		{
			memset(info,0x14,info->size+sizeof(sMemInfo_t)+sizeof(sMemInfoEnd_t));
		}
			
		free(info);
	}
}




static uint16_t crc16_tab[256] = {
	0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
	0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
	0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
	0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
	0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
	0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
	0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
	0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
	0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
	0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
	0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
	0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
	0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
	0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
	0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
	0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
	0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
	0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
	0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
	0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
	0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
	0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
	0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
	0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
	0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
	0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
	0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
	0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
	0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
	0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
	0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
	0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
};

const uint32_t crc32_tab[256] =
{
	0x00000000, 0x04c11db7, 0x09823b6e, 0x0d4326d9,
	0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
	0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61,
	0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,
	0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9,
	0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
	0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011,
	0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd,
	0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039,
	0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5,
	0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81,
	0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
	0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49,
	0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95,
	0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1,
	0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d,
	0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae,
	0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
	0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16,
	0x018aeb13, 0x054bf6a4, 0x0808d07d, 0x0cc9cdca,
	0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde,
	0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02,
	0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1, 0x53dc6066,
	0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
	0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e,
	0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692,
	0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6,
	0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a,
	0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e,
	0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
	0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686,
	0xd5b88683, 0xd1799b34, 0xdc3abded, 0xd8fba05a,
	0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637,
	0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb,
	0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f,
	0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
	0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47,
	0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b,
	0x0315d626, 0x07d4cb91, 0x0a97ed48, 0x0e56f0ff,
	0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623,
	0xf12f560e, 0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7,
	0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
	0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f,
	0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3,
	0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7,
	0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b,
	0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f,
	0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
	0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640,
	0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c,
	0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8,
	0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24,
	0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30,
	0x029f3d35, 0x065e2082, 0x0b1d065b, 0x0fdc1bec,
	0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088,
	0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654,
	0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0,
	0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c,
	0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18,
	0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
	0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0,
	0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c,
	0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668,
	0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4
};


uint16_t crc16(uint16_t start_crc, const uint8_t *input_str, uint32_t num_bytes)
{
	uint16_t crc;
	const uint8_t *ptr;
	uint32_t a;

	crc = start_crc;
	ptr = input_str;

	if (ptr != NULL) {
		for (a = 0; a < num_bytes; a++) {

			crc = (crc >> 8) ^ crc16_tab[(crc ^ (uint16_t)*ptr++) & 0x00FF];
		}
	}

	return crc;
}
