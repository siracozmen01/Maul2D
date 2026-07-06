// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Eight-lane f32 vectors under a bit law: every operation is a
// single-rounding IEEE op with IDENTICAL semantics on every backend.
// The solver kernels are written once against this API; AVX2, NEON
// and the scalar fallback must be indistinguishable in their bits,
// and CI holds that as a hard gate (the sanitize cell runs the scalar
// backend and its hashes join the cross-platform comparison).
//
// Rules that keep the law:
// - min/max are compare+select, NEVER the native instructions: SSE
//   and NEON disagree about NaN propagation and signed zero (the
//   MSVC-arm64 ternary lesson, slice 24).
// - fused multiply-add is EXPLICIT and used identically everywhere:
//   _mm256_fmadd_ps, vfmaq_f32 and fmaf are all correctly rounded, so
//   they agree bit for bit. Implicit contraction stays forbidden by
//   the compiler flags; only this header spells fma.
// - negation is a sign flip (exact), so composed forms like
//   a*b - c == fma(a, b, -c) hold exactly on all three backends.

#ifndef MAUL2D_SIMD_H
#define MAUL2D_SIMD_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#if defined(MAUL2D_SIMD_FORCE_SCALAR)
#define M2_SIMD_SCALAR 1
#elif defined(__AVX2__) && (defined(__FMA__) || defined(_MSC_VER))
// MSVC defines __AVX2__ under /arch:AVX2 but never __FMA__; the AVX2
// baseline (Haswell+) always ships FMA3.
#define M2_SIMD_AVX2 1
#elif defined(__aarch64__) || defined(_M_ARM64)
#define M2_SIMD_NEON 1
#else
#define M2_SIMD_SCALAR 1
#endif

#if defined(M2_SIMD_AVX2)

#include <immintrin.h>

typedef __m256 m2f8;

static inline m2f8 m2F8Load(const float* p)
{
    return _mm256_loadu_ps(p);
}
static inline void m2F8Store(float* p, m2f8 v)
{
    _mm256_storeu_ps(p, v);
}
static inline m2f8 m2F8Set1(float x)
{
    return _mm256_set1_ps(x);
}
static inline m2f8 m2F8Zero(void)
{
    return _mm256_setzero_ps();
}
static inline m2f8 m2F8Add(m2f8 a, m2f8 b)
{
    return _mm256_add_ps(a, b);
}
static inline m2f8 m2F8Sub(m2f8 a, m2f8 b)
{
    return _mm256_sub_ps(a, b);
}
static inline m2f8 m2F8Mul(m2f8 a, m2f8 b)
{
    return _mm256_mul_ps(a, b);
}
static inline m2f8 m2F8Neg(m2f8 a)
{
    return _mm256_xor_ps(a, _mm256_set1_ps(-0.0f));
}
// a * b + c, one rounding.
static inline m2f8 m2F8MulAdd(m2f8 a, m2f8 b, m2f8 c)
{
    return _mm256_fmadd_ps(a, b, c);
}
// c - a * b, one rounding.
static inline m2f8 m2F8NegMulAdd(m2f8 a, m2f8 b, m2f8 c)
{
    return _mm256_fnmadd_ps(a, b, c);
}
// mask lanes are all-ones or all-zeros; mask ? a : b.
static inline m2f8 m2F8Select(m2f8 mask, m2f8 a, m2f8 b)
{
    return _mm256_blendv_ps(b, a, mask);
}
static inline m2f8 m2F8GT(m2f8 a, m2f8 b)
{
    return _mm256_cmp_ps(a, b, _CMP_GT_OQ);
}
static inline m2f8 m2F8LT(m2f8 a, m2f8 b)
{
    return _mm256_cmp_ps(a, b, _CMP_LT_OQ);
}

#elif defined(M2_SIMD_NEON)

#include <arm_neon.h>

typedef struct m2f8
{
    float32x4_t lo;
    float32x4_t hi;
} m2f8;

static inline m2f8 m2F8Load(const float* p)
{
    m2f8 r = {vld1q_f32(p), vld1q_f32(p + 4)};
    return r;
}
static inline void m2F8Store(float* p, m2f8 v)
{
    vst1q_f32(p, v.lo);
    vst1q_f32(p + 4, v.hi);
}
static inline m2f8 m2F8Set1(float x)
{
    m2f8 r = {vdupq_n_f32(x), vdupq_n_f32(x)};
    return r;
}
static inline m2f8 m2F8Zero(void)
{
    return m2F8Set1(0.0f);
}
static inline m2f8 m2F8Add(m2f8 a, m2f8 b)
{
    m2f8 r = {vaddq_f32(a.lo, b.lo), vaddq_f32(a.hi, b.hi)};
    return r;
}
static inline m2f8 m2F8Sub(m2f8 a, m2f8 b)
{
    m2f8 r = {vsubq_f32(a.lo, b.lo), vsubq_f32(a.hi, b.hi)};
    return r;
}
static inline m2f8 m2F8Mul(m2f8 a, m2f8 b)
{
    m2f8 r = {vmulq_f32(a.lo, b.lo), vmulq_f32(a.hi, b.hi)};
    return r;
}
static inline m2f8 m2F8Neg(m2f8 a)
{
    m2f8 r = {vnegq_f32(a.lo), vnegq_f32(a.hi)};
    return r;
}
static inline m2f8 m2F8MulAdd(m2f8 a, m2f8 b, m2f8 c)
{
    // vfmaq(c, a, b) = c + a * b, one rounding.
    m2f8 r = {vfmaq_f32(c.lo, a.lo, b.lo), vfmaq_f32(c.hi, a.hi, b.hi)};
    return r;
}
static inline m2f8 m2F8NegMulAdd(m2f8 a, m2f8 b, m2f8 c)
{
    // vfmsq(c, a, b) = c - a * b, one rounding.
    m2f8 r = {vfmsq_f32(c.lo, a.lo, b.lo), vfmsq_f32(c.hi, a.hi, b.hi)};
    return r;
}
static inline m2f8 m2F8Select(m2f8 mask, m2f8 a, m2f8 b)
{
    m2f8 r = {vbslq_f32(vreinterpretq_u32_f32(mask.lo), a.lo, b.lo),
              vbslq_f32(vreinterpretq_u32_f32(mask.hi), a.hi, b.hi)};
    return r;
}
static inline m2f8 m2F8GT(m2f8 a, m2f8 b)
{
    m2f8 r = {vreinterpretq_f32_u32(vcgtq_f32(a.lo, b.lo)),
              vreinterpretq_f32_u32(vcgtq_f32(a.hi, b.hi))};
    return r;
}
static inline m2f8 m2F8LT(m2f8 a, m2f8 b)
{
    m2f8 r = {vreinterpretq_f32_u32(vcltq_f32(a.lo, b.lo)),
              vreinterpretq_f32_u32(vcltq_f32(a.hi, b.hi))};
    return r;
}

#else // M2_SIMD_SCALAR

typedef struct m2f8
{
    float v[8];
} m2f8;

static inline m2f8 m2F8Load(const float* p)
{
    m2f8 r;
    memcpy(r.v, p, sizeof(r.v));
    return r;
}
static inline void m2F8Store(float* p, m2f8 v)
{
    memcpy(p, v.v, sizeof(v.v));
}
static inline m2f8 m2F8Set1(float x)
{
    m2f8 r;
    for (int32_t i = 0; i < 8; ++i)
    {
        r.v[i] = x;
    }
    return r;
}
static inline m2f8 m2F8Zero(void)
{
    return m2F8Set1(0.0f);
}
static inline m2f8 m2F8Add(m2f8 a, m2f8 b)
{
    m2f8 r;
    for (int32_t i = 0; i < 8; ++i)
    {
        r.v[i] = a.v[i] + b.v[i];
    }
    return r;
}
static inline m2f8 m2F8Sub(m2f8 a, m2f8 b)
{
    m2f8 r;
    for (int32_t i = 0; i < 8; ++i)
    {
        r.v[i] = a.v[i] - b.v[i];
    }
    return r;
}
static inline m2f8 m2F8Mul(m2f8 a, m2f8 b)
{
    m2f8 r;
    for (int32_t i = 0; i < 8; ++i)
    {
        r.v[i] = a.v[i] * b.v[i];
    }
    return r;
}
static inline m2f8 m2F8Neg(m2f8 a)
{
    m2f8 r;
    for (int32_t i = 0; i < 8; ++i)
    {
        r.v[i] = -a.v[i];
    }
    return r;
}
static inline m2f8 m2F8MulAdd(m2f8 a, m2f8 b, m2f8 c)
{
    m2f8 r;
    for (int32_t i = 0; i < 8; ++i)
    {
        r.v[i] = fmaf(a.v[i], b.v[i], c.v[i]);
    }
    return r;
}
static inline m2f8 m2F8NegMulAdd(m2f8 a, m2f8 b, m2f8 c)
{
    m2f8 r;
    for (int32_t i = 0; i < 8; ++i)
    {
        r.v[i] = fmaf(-a.v[i], b.v[i], c.v[i]);
    }
    return r;
}
static inline m2f8 m2F8Select(m2f8 mask, m2f8 a, m2f8 b)
{
    m2f8 r;
    for (int32_t i = 0; i < 8; ++i)
    {
        uint32_t m;
        uint32_t ua;
        uint32_t ub;
        memcpy(&m, &mask.v[i], 4);
        memcpy(&ua, &a.v[i], 4);
        memcpy(&ub, &b.v[i], 4);
        uint32_t out = (ua & m) | (ub & ~m);
        memcpy(&r.v[i], &out, 4);
    }
    return r;
}
static inline m2f8 m2F8Cmp(int gt, m2f8 a, m2f8 b)
{
    m2f8 r;
    for (int32_t i = 0; i < 8; ++i)
    {
        uint32_t m = (gt ? (a.v[i] > b.v[i]) : (a.v[i] < b.v[i])) ? 0xFFFFFFFFu : 0u;
        memcpy(&r.v[i], &m, 4);
    }
    return r;
}
static inline m2f8 m2F8GT(m2f8 a, m2f8 b)
{
    return m2F8Cmp(1, a, b);
}
static inline m2f8 m2F8LT(m2f8 a, m2f8 b)
{
    return m2F8Cmp(0, a, b);
}

#endif

// Ternary-law min/max: a>b?a:b and a<b?a:b, identical on every
// backend by construction (compare + select, no native min/max).
static inline m2f8 m2F8Max(m2f8 a, m2f8 b)
{
    return m2F8Select(m2F8GT(a, b), a, b);
}
static inline m2f8 m2F8Min(m2f8 a, m2f8 b)
{
    return m2F8Select(m2F8LT(a, b), a, b);
}

#endif // MAUL2D_SIMD_H
