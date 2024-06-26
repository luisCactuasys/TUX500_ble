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

#ifndef TOOLS_H
#define TOOLS_H

#include <stdint.h>

typedef enum _eResult
{
	FRAMEWORK_SUCCESS,
	FRAMEWORK_FAILED
}eResult;


/**
 * Create a thread using the given function in parameters
 * @param threadHandle Obfuscated thread handle allocated by this function.
 * @param threadedFunc function to start in new thread. ( void* myFunc(void* ctx) )
 * @param ctx Parameters given to threadedFunc.
 * @return SUCCESS if no error.
 */
eResult framework_CreateThread(void** threadHandle,void *(*threadedFunc)(void*),void *ctx);
/**
 * Wait until the given thread finish to run.
 * @param threadHandle Obfuscated thread handle allocated by framework_CreateThread()
 */
void framework_JoinThread(void* threadHandle);
/**
 * Delete the given thread. NOTE : framework_JoinThread() will be called before this function. So
 * the Thread is already stopped.
 * @param threadHandle Obfuscated thread handle allocated by framework_CreateThread()
 */
void framework_DeleteThread(void* threadHandle);
/**
 * Return the calling thread ID.
 * @return thread id.
 */
void* framework_GetCurrentThreadId();
/**
 * Get the thread id of the given thread handle.
 * @param threadHandle Obfuscated thread handle allocated by framework_CreateThread()
 * @return thread id.
 */
void* framework_GetThreadId(void* threadHandle);


/**
 * Create a mutex object. To gain performances, do not implement this function using interprocess
 * lock mechanism such as Semaphore.
 * @param mutexHandle Obfuscated mutex handle allocated by this function.
 * @return SUCCESS if no error.
 */
eResult framework_CreateMutex(void** mutexHandle);
/**
 * Lock the mutex.
 * @param mutexHandle Obfuscated mutex handle allocated by framework_CreateMutex().
 */
void framework_LockMutex(void* mutexHandle);
/**
 * Unlock the mutex
 * @param mutexHandle Obfuscated mutex handle allocated by framework_CreateMutex().
 */
void framework_UnlockMutex(void* mutexHandle);
/**
 * Block the current thread until wake up by framework_NotifyMutex(). 
 * The mutex need to be locked before blocking the thread. (needLock parameter can be used)
 * @param mutexHandle Obfuscated mutex handle allocated by framework_CreateMutex().
 * @param needLock Indicate if the mutex need to be locked internaly or not. This avoid to call lock();wait();unlock();
 */
void framework_WaitMutex(void* mutexHandle,uint8_t needLock);
int framework_TimedWaitMutex(void * mutexHandle, uint8_t needLock, uint32_t waitMs);
/**
 * Wake up a thread blocked by the mutex. The mutex must be locked before waking up another thread.
 * The mutex need to be locked before waking up a thread. (needLock parameter can be used)
 * @param mutexHandle Obfuscated mutex handle allocated by framework_CreateMutex().
 * @param needLock Indicate if the mutex need to be locked internaly or not. This avoid to call lock();wait();unlock();
 */
void framework_NotifyMutex(void* mutexHandle,uint8_t needLock);
/**
 * Delete the mutex. If the mutex is locked, any locked thread will be unlocked.
 * @param mutexHandle Obfuscated mutex handle allocated by framework_CreateMutex().
 */
void framework_DeleteMutex(void* mutexHandle);

/**
 * Cause the calling thread to sleep until ms milliseconds elapsed.
 * @param ms Milliseconds to wait until wakeup.
 */
void framework_MilliSleep(uint32_t ms);


uint16_t crc16(uint16_t start_crc, const uint8_t *input_str, uint32_t num_bytes);


#endif
