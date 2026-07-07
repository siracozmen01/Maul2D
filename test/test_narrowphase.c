// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Narrowphase batching gate (queue #2, phase 5 groundwork). The scalar
// SAT (m2FindMaxSeparation) is the largest scalar cost left in the step;
// the plan is to run it eight pairs at a time in m2f8. Before that
// kernel is allowed anywhere near the hot path, this suite proves it is
// BIT-IDENTICAL to the scalar kernel it will replace: for many random
// convex polygon pairs, batched of eight and scalar one at a time must
// return the exact same separation bits and the exact same edge index,
// across mixed vertex counts, partial batches, rounded radii and null
// padding. A single differing bit is a hard failure, because the whole
// determinism contract (the sixteen gated hashes, rollback, replay,
// cross-platform bit identity) rides on manifolds staying bit-exact.
// White-box (the SAT kernels are internal); no gated hash of its own.

#include "world_internal.h"

#include "maul2d/base.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int s_failures = 0;

#define CHECK(cond, msg)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);                                 \
            s_failures += 1;                                                                       \
        }                                                                                          \
    } while (0)

// Deterministic PRNG (SplitMix32), so the fuzz is reproducible bit for
// bit on every machine, like everything else in the engine.
static uint32_t s_state = 0x9e3779b9u;
static uint32_t NextU32(void)
{
    s_state += 0x9e3779b9u;
    uint32_t z = s_state;
    z = (z ^ (z >> 16)) * 0x85ebca6bu;
    z = (z ^ (z >> 13)) * 0xc2b2ae35u;
    return z ^ (z >> 16);
}

// A float in [lo, hi).
static float NextF(float lo, float hi)
{
    float u = (float)(NextU32() >> 8) * (1.0f / 16777216.0f); // 24-bit mantissa
    return lo + (hi - lo) * u;
}

// A random convex polygon: 3..8 scattered points run through the hull
// builder (so normals and counts are the real ones the narrowphase
// sees), placed near a random center, with an occasional round radius.
static m2Polygon RandomPolygon(void)
{
    int32_t n = 3 + (int32_t)(NextU32() % 6); // 3..8 raw points
    m2Vec2 pts[8];
    float cx = NextF(-40.0f, 40.0f);
    float cy = NextF(-40.0f, 40.0f);
    for (int32_t i = 0; i < n; ++i)
    {
        pts[i].x = cx + NextF(-3.0f, 3.0f);
        pts[i].y = cy + NextF(-3.0f, 3.0f);
    }
    float radius = (NextU32() & 3u) == 0u ? NextF(0.0f, 0.2f) : 0.0f;
    m2Polygon p = m2ComputeHull(pts, n, radius);
    if (p.count < 3)
    {
        // Degenerate scatter collapsed the hull; fall back to a box so
        // every lane always holds a valid convex polygon.
        p = m2MakeBox(NextF(0.3f, 2.0f), NextF(0.3f, 2.0f));
    }
    return p;
}

// One batch of `count` (1..8) pairs: batched result must equal scalar
// bit for bit. `withNulls` pads the unused tail lanes with null pointers
// to exercise the kernel's null guard.
static void CheckBatch(int32_t count, int withNulls)
{
    m2Polygon a[8];
    m2Polygon b[8];
    const m2Polygon* pa[8];
    const m2Polygon* pb[8];
    for (int32_t k = 0; k < 8; ++k)
    {
        a[k] = RandomPolygon();
        b[k] = RandomPolygon();
        if (k < count)
        {
            pa[k] = &a[k];
            pb[k] = &b[k];
        }
        else
        {
            pa[k] = withNulls ? NULL : &a[k];
            pb[k] = withNulls ? NULL : &b[k];
        }
    }

    float batchSep[8];
    int32_t batchEdge[8];
    m2FindMaxSeparationBatch8(pa, pb, count, batchSep, batchEdge);

    for (int32_t k = 0; k < count; ++k)
    {
        int32_t scalarEdge = 0;
        float scalarSep = m2FindMaxSeparation(&scalarEdge, &a[k], &b[k]);
        // Compare the raw bits: a NaN or a signed zero must also match.
        uint32_t sb;
        uint32_t bb;
        memcpy(&sb, &scalarSep, 4);
        memcpy(&bb, &batchSep[k], 4);
        CHECK(sb == bb, "batched separation bits differ from scalar");
        CHECK(scalarEdge == batchEdge[k], "batched edge index differs from scalar");
    }
}

// A worst case for the masking: one lane is a 3-gon, one an 8-gon, the
// rest between, so maxC1 and maxC2 run past most lanes' real counts and
// the padding is exercised on nearly every lane.
static void CheckMixedCounts(void)
{
    m2Polygon a[8];
    m2Polygon b[8];
    const m2Polygon* pa[8];
    const m2Polygon* pb[8];
    m2Vec2 tri[3] = {{0.0f, 0.0f}, {2.0f, 0.0f}, {1.0f, 2.0f}};
    for (int32_t k = 0; k < 8; ++k)
    {
        if (k == 0)
        {
            a[k] = m2ComputeHull(tri, 3, 0.0f);
        }
        else
        {
            a[k] = RandomPolygon();
        }
        b[k] = (k == 7) ? RandomPolygon() : m2ComputeHull(tri, 3, 0.0f);
        pa[k] = &a[k];
        pb[k] = &b[k];
    }
    float batchSep[8];
    int32_t batchEdge[8];
    m2FindMaxSeparationBatch8(pa, pb, 8, batchSep, batchEdge);
    for (int32_t k = 0; k < 8; ++k)
    {
        int32_t scalarEdge = 0;
        float scalarSep = m2FindMaxSeparation(&scalarEdge, &a[k], &b[k]);
        uint32_t sb;
        uint32_t bb;
        memcpy(&sb, &scalarSep, 4);
        memcpy(&bb, &batchSep[k], 4);
        CHECK(sb == bb, "mixed-count batched separation differs");
        CHECK(scalarEdge == batchEdge[k], "mixed-count batched edge differs");
    }
}

int main(void)
{
    // Full batches of eight over a large random population.
    for (int32_t iter = 0; iter < 4000; ++iter)
    {
        CheckBatch(8, 0);
    }
    // Partial batches (1..7) and the null-padded tail.
    for (int32_t iter = 0; iter < 500; ++iter)
    {
        for (int32_t count = 1; count <= 7; ++count)
        {
            CheckBatch(count, 0);
            CheckBatch(count, 1);
        }
    }
    // The masking worst case.
    for (int32_t iter = 0; iter < 2000; ++iter)
    {
        CheckMixedCounts();
    }

    if (s_failures == 0)
    {
        printf("test_narrowphase: all batched SAT results bit-identical to scalar\n");
    }
    return s_failures == 0 ? 0 : 1;
}
