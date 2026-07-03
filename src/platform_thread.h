// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Minimal fork-join worker pool (topic-08). Work items are handed out
// as STATIC ranges - worker w always gets the same slice of the same
// job - so parallel execution is a pure scheduling change and can
// never reorder arithmetic. Determinism comes from the caller only
// submitting jobs whose items touch disjoint data (graph colors,
// per-pair manifolds); the pool adds no ordering of its own.

#ifndef MAUL2D_PLATFORM_THREAD_H
#define MAUL2D_PLATFORM_THREAD_H

#include <stdint.h>

typedef struct m2ThreadPool m2ThreadPool;

// fn(begin, end, ctx): process items [begin, end).
typedef void m2ParallelFn(int32_t begin, int32_t end, void* ctx);

// workerCount counts the calling thread too; <= 1 returns NULL (serial).
m2ThreadPool* m2ThreadPoolCreate(int32_t workerCount);
void m2ThreadPoolDestroy(m2ThreadPool* pool);

// Runs fn over [0, itemCount) split into workerCount static ranges;
// the calling thread takes range 0 and the call returns only when
// every range is done. pool == NULL runs everything inline.
void m2ThreadPoolRun(m2ThreadPool* pool, m2ParallelFn* fn, void* ctx, int32_t itemCount);

// Monotonic wall clock for diagnostics only - profile numbers are
// never simulation state, never snapshot, never hashed.
uint64_t m2TimeNowNs(void);

#endif // MAUL2D_PLATFORM_THREAD_H
