// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL2D_BASE_H
#define MAUL2D_BASE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define M2_VERSION_MAJOR 1
#define M2_VERSION_MINOR 6
#define M2_VERSION_PATCH 0

    /// Library version, encoded as major * 10000 + minor * 100 + patch.
    /// Thread class: reader (callable from any thread, no world required).
    int32_t m2GetVersion(void);

    /// Routes every internal allocation through your hooks. Set BEFORE
    /// creating any world and never change it while worlds exist. The
    /// zeroFn contract: returned memory must be zero-initialized.
    typedef void* m2AllocZeroedFn(size_t bytes);
    typedef void m2FreeFn(void* memory);
    void m2SetAllocator(m2AllocZeroedFn* allocZeroed, m2FreeFn* freeFn);

    /// FNV-1a 64-bit hash over a byte range. This is the hash used by the
    /// determinism gates; its constants are frozen and will never change.
    /// Thread class: reader (pure function).
    uint64_t m2Hash64(uint64_t seed, const void* data, int32_t byteCount);

/// Seed value for m2Hash64 chains (FNV-1a offset basis).
#define M2_HASH_INIT 14695981039346656037ULL

    /// Internal assertion failure sink (debug builds only). Prints and traps.
    void m2AssertFail(const char* condition, const char* file, int line);

#if defined(NDEBUG)
#define M2_ASSERT(cond) ((void)0)
#else
#define M2_ASSERT(cond) ((cond) ? (void)0 : m2AssertFail(#cond, __FILE__, __LINE__))
#endif

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_BASE_H
