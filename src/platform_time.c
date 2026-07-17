// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The monotonic clock behind the step profile. This is the only
// platform code left in the library since the worker pool moved to
// the host's executor (integration audit A1): observer machinery,
// never a hash input.
#if !defined(_WIN32)
#define _POSIX_C_SOURCE 199309L // clock_gettime
#endif
#include "world_internal.h"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
uint64_t m2TimeNowNs(void)
{
    static LARGE_INTEGER frequency;
    if (frequency.QuadPart == 0)
    {
        QueryPerformanceFrequency(&frequency);
    }
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return (uint64_t)((double)now.QuadPart * 1.0e9 / (double)frequency.QuadPart);
}
#else
#include <time.h>
uint64_t m2TimeNowNs(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}
#endif
