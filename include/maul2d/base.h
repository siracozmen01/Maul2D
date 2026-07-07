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
#define M2_VERSION_MINOR 9
#define M2_VERSION_PATCH 0

    /// Library version, encoded as major * 10000 + minor * 100 + patch.
    /// Thread class: reader (callable from any thread, no world required).
    int32_t m2GetVersion(void);

    /// The SIMD backend this library was COMPILED against: "avx2", "neon"
    /// or "scalar". It is a compile-time choice, and every backend produces
    /// bit-identical results by contract; this only reports which kernels
    /// the binary carries. Thread class: reader.
    const char* m2GetSimdBackend(void);

    /// Whether the CPU running this call actually supports the compiled
    /// backend: 1 if it can run, 0 if not. An "avx2" binary needs AVX2 and
    /// FMA3 with OS wide-register support; "neon" is architectural on
    /// arm64 and "scalar" runs anywhere, so both return 1. Creating a world
    /// on a CPU that returns 0 aborts loudly rather than trapping on an
    /// illegal instruction; check this first for a graceful path, or build
    /// with -DMAUL2D_SIMD=scalar for a portable binary. Thread class: reader.
    int32_t m2CpuSupportsBackend(void);

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
