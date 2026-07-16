// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "maul2d/base.h"

#include "simd.h" // the M2_SIMD_* backend selection, single source of truth

#if defined(M2_SIMD_AVX2) && defined(_MSC_VER)
#include <intrin.h> // __cpuid, __cpuidex, _xgetbv for the capability check
#endif

#include <stdio.h>
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

const char* m2GetSimdBackend(void)
{
#if defined(M2_SIMD_AVX2)
    return "avx2";
#elif defined(M2_SIMD_NEON)
    return "neon";
#else
    return "scalar";
#endif
}

int32_t m2CpuSupportsBackend(void)
{
#if defined(M2_SIMD_AVX2)
    // The AVX2 kernels need AVX2 and FMA3, and the OS must have enabled the
    // wide registers (XGETBV). On GCC and Clang one builtin checks all of
    // it; MSVC needs cpuid and xgetbv spelled out. immintrin.h (pulled in
    // by simd.h under this backend) provides the MSVC intrinsics.
#if defined(_MSC_VER)
    int regs[4];
    __cpuid(regs, 0);
    if (regs[0] < 7)
    {
        return 0; // no leaf 7, so no AVX2
    }
    __cpuidex(regs, 1, 0);
    int osxsave = (regs[2] >> 27) & 1;
    int fma = (regs[2] >> 12) & 1;
    int avx = (regs[2] >> 28) & 1;
    if (!(osxsave && avx && fma))
    {
        return 0;
    }
    unsigned long long xcr = _xgetbv(0);
    if ((xcr & 0x6) != 0x6)
    {
        return 0; // OS has not enabled XMM and YMM state
    }
    __cpuidex(regs, 7, 0);
    return ((regs[1] >> 5) & 1) ? 1 : 0; // leaf 7 EBX bit 5 = AVX2
#else
    return (__builtin_cpu_supports("avx2") && __builtin_cpu_supports("fma")) ? 1 : 0;
#endif
#else
    // NEON is architectural on arm64, scalar needs nothing, and wasm has a
    // single path: each runs wherever it was built.
    return 1;
#endif
}

// Internal (world_internal.h): a one-time guard so an AVX2 binary on a
// pre-Haswell CPU aborts with a clear, actionable message instead of
// trapping on the first wide instruction deep in the solver.
static m2AssertFn* s_assertHandler = NULL;
static void* s_assertContext = NULL;

void m2SetAssertHandler(m2AssertFn* handler, void* context)
{
    s_assertHandler = handler;
    s_assertContext = context;
}

int m2VerifyCpuBackend(void)
{
    // B1 (integration audit): the library never aborts in release.
    // An unsupported CPU routes through the assert hook and refuses
    // the world with a null id; the printed reason remains for the
    // hookless case, and the host that skipped the public probe
    // (m2CpuSupportsBackend) still gets a loud, typed refusal.
    static int32_t checked = 0;
    static int32_t supported = 1;
    if (checked == 0)
    {
        checked = 1;
        supported = m2CpuSupportsBackend() != 0;
        if (!supported)
        {
            const char* message = "CPU does not support the built SIMD backend; "
                                  "rebuild with -DMAUL2D_SIMD=scalar";
            if (s_assertHandler == NULL ||
                s_assertHandler(message, "m2CreateWorld", 0, s_assertContext) == 0)
            {
                fprintf(stderr,
                        "maul2d: built for the '%s' SIMD backend, but this CPU "
                        "does not support it. Rebuild with -DMAUL2D_SIMD=scalar "
                        "for a portable binary.\n",
                        m2GetSimdBackend());
            }
        }
    }
    return supported;
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
    if (s_assertHandler != NULL && s_assertHandler(condition, file, line, s_assertContext) != 0)
    {
        return; // handled by the host (A2)
    }
    fprintf(stderr, "maul2d assertion failed: %s (%s:%d)\n", condition, file, line);
    abort();
}
