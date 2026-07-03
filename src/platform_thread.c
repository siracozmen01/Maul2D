// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "platform_thread.h"

#include "maul2d/base.h"

#include <stdlib.h>

#define M2_MAX_WORKERS 8

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <process.h>

typedef SRWLOCK m2Mutex;
typedef CONDITION_VARIABLE m2Cond;
typedef HANDLE m2Thread;

static void MutexInit(m2Mutex* m)
{
    InitializeSRWLock(m);
}
static void MutexDestroy(m2Mutex* m)
{
    (void)m;
}
static void MutexLock(m2Mutex* m)
{
    AcquireSRWLockExclusive(m);
}
static void MutexUnlock(m2Mutex* m)
{
    ReleaseSRWLockExclusive(m);
}
static void CondInit(m2Cond* c)
{
    InitializeConditionVariable(c);
}
static void CondDestroy(m2Cond* c)
{
    (void)c;
}
static void CondWait(m2Cond* c, m2Mutex* m)
{
    SleepConditionVariableSRW(c, m, INFINITE, 0);
}
static void CondBroadcast(m2Cond* c)
{
    WakeAllConditionVariable(c);
}
static void CondSignal(m2Cond* c)
{
    WakeConditionVariable(c);
}

#else

#include <pthread.h>

typedef pthread_mutex_t m2Mutex;
typedef pthread_cond_t m2Cond;
typedef pthread_t m2Thread;

static void MutexInit(m2Mutex* m)
{
    pthread_mutex_init(m, NULL);
}
static void MutexDestroy(m2Mutex* m)
{
    pthread_mutex_destroy(m);
}
static void MutexLock(m2Mutex* m)
{
    pthread_mutex_lock(m);
}
static void MutexUnlock(m2Mutex* m)
{
    pthread_mutex_unlock(m);
}
static void CondInit(m2Cond* c)
{
    pthread_cond_init(c, NULL);
}
static void CondDestroy(m2Cond* c)
{
    pthread_cond_destroy(c);
}
static void CondWait(m2Cond* c, m2Mutex* m)
{
    pthread_cond_wait(c, m);
}
static void CondBroadcast(m2Cond* c)
{
    pthread_cond_broadcast(c);
}
static void CondSignal(m2Cond* c)
{
    pthread_cond_signal(c);
}

#endif

typedef struct m2Worker
{
    struct m2ThreadPool* pool;
    int32_t slot; // 1..threadCount; the caller is slot 0
    m2Thread thread;
} m2Worker;

struct m2ThreadPool
{
    int32_t workerCount; // including the calling thread
    int32_t threadCount; // spawned threads (workerCount - 1)
    m2Worker workers[M2_MAX_WORKERS];

    m2Mutex mutex;
    m2Cond wake;
    m2Cond done;
    uint32_t generation;
    int32_t finished;
    int32_t shutdown;

    m2ParallelFn* fn;
    void* ctx;
    int32_t itemCount;
};

static void RunRange(m2ThreadPool* pool, m2ParallelFn* fn, void* ctx, int32_t itemCount,
                     int32_t slot)
{
    int32_t begin = (int32_t)((int64_t)itemCount * slot / pool->workerCount);
    int32_t end = (int32_t)((int64_t)itemCount * (slot + 1) / pool->workerCount);
    if (begin < end)
    {
        fn(begin, end, ctx);
    }
}

#ifdef _WIN32
static unsigned __stdcall WorkerMain(void* arg)
#else
static void* WorkerMain(void* arg)
#endif
{
    m2Worker* worker = (m2Worker*)arg;
    m2ThreadPool* pool = worker->pool;
    uint32_t seen = 0;

    MutexLock(&pool->mutex);
    for (;;)
    {
        while (pool->shutdown == 0 && pool->generation == seen)
        {
            CondWait(&pool->wake, &pool->mutex);
        }
        if (pool->shutdown != 0)
        {
            break;
        }
        seen = pool->generation;
        m2ParallelFn* fn = pool->fn;
        void* ctx = pool->ctx;
        int32_t itemCount = pool->itemCount;
        MutexUnlock(&pool->mutex);

        RunRange(pool, fn, ctx, itemCount, worker->slot);

        MutexLock(&pool->mutex);
        pool->finished += 1;
        if (pool->finished == pool->threadCount)
        {
            CondSignal(&pool->done);
        }
    }
    MutexUnlock(&pool->mutex);
#ifdef _WIN32
    return 0;
#else
    return NULL;
#endif
}

m2ThreadPool* m2ThreadPoolCreate(int32_t workerCount)
{
    if (workerCount > M2_MAX_WORKERS)
    {
        workerCount = M2_MAX_WORKERS;
    }
    if (workerCount <= 1)
    {
        return NULL;
    }
    m2ThreadPool* pool = (m2ThreadPool*)calloc(1, sizeof(m2ThreadPool));
    if (pool == NULL)
    {
        return NULL;
    }
    pool->workerCount = workerCount;
    pool->threadCount = workerCount - 1;
    MutexInit(&pool->mutex);
    CondInit(&pool->wake);
    CondInit(&pool->done);

    for (int32_t i = 0; i < pool->threadCount; ++i)
    {
        m2Worker* worker = &pool->workers[i];
        worker->pool = pool;
        worker->slot = i + 1;
#ifdef _WIN32
        worker->thread = (HANDLE)_beginthreadex(NULL, 0, WorkerMain, worker, 0, NULL);
        M2_ASSERT(worker->thread != NULL);
#else
        int rc = pthread_create(&worker->thread, NULL, WorkerMain, worker);
        M2_ASSERT(rc == 0);
        (void)rc;
#endif
    }
    return pool;
}

void m2ThreadPoolDestroy(m2ThreadPool* pool)
{
    if (pool == NULL)
    {
        return;
    }
    MutexLock(&pool->mutex);
    pool->shutdown = 1;
    CondBroadcast(&pool->wake);
    MutexUnlock(&pool->mutex);
    for (int32_t i = 0; i < pool->threadCount; ++i)
    {
#ifdef _WIN32
        WaitForSingleObject(pool->workers[i].thread, INFINITE);
        CloseHandle(pool->workers[i].thread);
#else
        pthread_join(pool->workers[i].thread, NULL);
#endif
    }
    MutexDestroy(&pool->mutex);
    CondDestroy(&pool->wake);
    CondDestroy(&pool->done);
    free(pool);
}

void m2ThreadPoolRun(m2ThreadPool* pool, m2ParallelFn* fn, void* ctx, int32_t itemCount)
{
    if (itemCount <= 0)
    {
        return;
    }
    if (pool == NULL || itemCount < pool->workerCount)
    {
        fn(0, itemCount, ctx);
        return;
    }

    MutexLock(&pool->mutex);
    pool->fn = fn;
    pool->ctx = ctx;
    pool->itemCount = itemCount;
    pool->finished = 0;
    pool->generation += 1;
    CondBroadcast(&pool->wake);
    MutexUnlock(&pool->mutex);

    RunRange(pool, fn, ctx, itemCount, 0);

    MutexLock(&pool->mutex);
    while (pool->finished < pool->threadCount)
    {
        CondWait(&pool->done, &pool->mutex);
    }
    MutexUnlock(&pool->mutex);
}
