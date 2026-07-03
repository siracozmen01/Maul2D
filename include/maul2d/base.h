// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL2D_BASE_H
#define MAUL2D_BASE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define M2_VERSION_MAJOR 0
#define M2_VERSION_MINOR 0
#define M2_VERSION_PATCH 1

    /// Library version, encoded as major * 10000 + minor * 100 + patch.
    /// Thread class: reader (callable from any thread, no world required).
    int32_t m2GetVersion(void);

    /// FNV-1a 64-bit hash over a byte range. This is the hash used by the
    /// determinism gates; its constants are frozen and will never change.
    /// Thread class: reader (pure function).
    uint64_t m2Hash64(uint64_t seed, const void* data, int32_t byteCount);

/// Seed value for m2Hash64 chains (FNV-1a offset basis).
#define M2_HASH_INIT 14695981039346656037ULL

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_BASE_H
