// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The HOST-side task pool the worker-count invariance tests ride
// since the engine stopped opening threads (integration audit A1).
// Same shape as Maul3D's proven test pool: split into ranges, run
// range 0 on the caller, spawn the rest, join in finish.
#ifndef MAUL2D_TEST_TASK_POOL_H
#define MAUL2D_TEST_TASK_POOL_H

#include "maul2d/world.h"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <pthread.h>
#endif

#define TP_MAX_WORKERS 8

typedef struct TpRange
{
    m2TaskFn* task;
    void* taskContext;
    int32_t start;
    int32_t end;
} TpRange;

typedef struct TpTask
{
    TpRange ranges[TP_MAX_WORKERS];
    int32_t rangeCount;
#if defined(_WIN32)
    HANDLE threads[TP_MAX_WORKERS];
#else
    pthread_t threads[TP_MAX_WORKERS];
#endif
    int32_t threadCount;
} TpTask;

// The pool's worker count rides the user context as an int pointer.
#if defined(_WIN32)
static DWORD WINAPI TpMain(LPVOID arg)
{
    TpRange* r = (TpRange*)arg;
    r->task(r->start, r->end, r->taskContext);
    return 0;
}
#else
static void* TpMain(void* arg)
{
    TpRange* r = (TpRange*)arg;
    r->task(r->start, r->end, r->taskContext);
    return NULL;
}
#endif

static void* TpEnqueue(m2TaskFn* task, int32_t itemCount, int32_t minRange, void* taskContext,
                       void* userContext)
{
    static TpTask taskSlot; // one in-flight task: the library's contract
    TpTask* t = &taskSlot;
    int32_t workers = userContext != NULL ? *(int32_t*)userContext : 2;
    workers = workers < 1 ? 1 : (workers > TP_MAX_WORKERS ? TP_MAX_WORKERS : workers);
    int32_t grain = (itemCount + workers - 1) / workers;
    grain = grain < minRange ? minRange : grain;
    t->rangeCount = 0;
    t->threadCount = 0;
    for (int32_t start = 0; start < itemCount; start += grain)
    {
        int32_t end = start + grain < itemCount ? start + grain : itemCount;
        TpRange* r = &t->ranges[t->rangeCount++];
        r->task = task;
        r->taskContext = taskContext;
        r->start = start;
        r->end = end;
        if (t->rangeCount == TP_MAX_WORKERS)
        {
            r->end = itemCount;
            break;
        }
    }
    for (int32_t k = 1; k < t->rangeCount; ++k)
    {
#if defined(_WIN32)
        t->threads[t->threadCount] = CreateThread(NULL, 0, TpMain, &t->ranges[k], 0, NULL);
#else
        pthread_create(&t->threads[t->threadCount], NULL, TpMain, &t->ranges[k]);
#endif
        t->threadCount += 1;
    }
    t->ranges[0].task(t->ranges[0].start, t->ranges[0].end, taskContext);
    return t;
}

static void TpFinish(void* userTask, void* userContext)
{
    (void)userContext;
    TpTask* t = (TpTask*)userTask;
    for (int32_t k = 0; k < t->threadCount; ++k)
    {
#if defined(_WIN32)
        WaitForSingleObject(t->threads[k], INFINITE);
        CloseHandle(t->threads[k]);
#else
        pthread_join(t->threads[k], NULL);
#endif
    }
}

#endif
