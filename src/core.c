// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#include "maul2d/base.h"

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
