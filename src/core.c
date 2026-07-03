// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "maul2d/base.h"

#include <stdio.h>
#include <stdlib.h>

#include <stdlib.h>

static void* DefaultAllocZeroed(size_t bytes)
{
    return calloc(1, bytes);
}

static void DefaultFree(void* memory)
{
    free(memory);
}

static m2AllocZeroedFn* s_alloc = DefaultAllocZeroed;
static m2FreeFn* s_free = DefaultFree;

void m2SetAllocator(m2AllocZeroedFn* allocZeroed, m2FreeFn* freeFn)
{
    s_alloc = allocZeroed != NULL ? allocZeroed : DefaultAllocZeroed;
    s_free = freeFn != NULL ? freeFn : DefaultFree;
}

// Internal faces (world_internal.h).
void* m2AllocZeroed(size_t bytes)
{
    return s_alloc(bytes);
}

void m2Free(void* memory)
{
    s_free(memory);
}

int32_t m2GetVersion(void)
{
    return M2_VERSION_MAJOR * 10000 + M2_VERSION_MINOR * 100 + M2_VERSION_PATCH;
}

uint64_t m2Hash64(uint64_t seed, const void* data, int32_t byteCount)
{
    // FNV-1a, 64-bit. The constants are frozen: this hash feeds the
    // determinism gates and its output is compared across platforms.
    uint64_t hash = seed;
    const uint8_t* bytes = (const uint8_t*)data;
    for (int32_t i = 0; i < byteCount; ++i)
    {
        hash = (hash ^ bytes[i]) * 1099511628211ULL;
    }
    return hash;
}

void m2AssertFail(const char* condition, const char* file, int line)
{
    fprintf(stderr, "maul2d assertion failed: %s (%s:%d)\n", condition, file, line);
    abort();
}
