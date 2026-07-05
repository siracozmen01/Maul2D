// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL2D_MATH_H
#define MAUL2D_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define M2_PI 3.14159265359f

    /// Local-space / velocity-space vector. World positions use m2Pos2.
    typedef struct m2Vec2
    {
        float x, y;
    } m2Vec2;

    /// World-space position. Positions are double precision so that
    /// simulation quality does not degrade far from the origin.
    typedef struct m2Pos2
    {
        double x, y;
    } m2Pos2;

    /// A rotation stored as a unit complex number. No angles are stored;
    /// angles exist only at the API rim.
    typedef struct m2Rot
    {
        float c, s;
    } m2Rot;

    /// A body pose: double-precision position plus rotation.
    typedef struct m2Transform
    {
        m2Pos2 p;
        m2Rot q;
    } m2Transform;

    // Snapshot-visible structs must have no hidden padding: byte-exact
    // snapshots depend on it. sizeof must equal the sum of the members.
    _Static_assert(sizeof(m2Vec2) == 8, "m2Vec2 must be 8 bytes");
    _Static_assert(sizeof(m2Pos2) == 16, "m2Pos2 must be 16 bytes");
    _Static_assert(sizeof(m2Rot) == 8, "m2Rot must be 8 bytes");
    _Static_assert(sizeof(m2Transform) == 24, "m2Transform must be 24 bytes, no padding");
    _Static_assert(_Alignof(m2Vec2) == 4 && _Alignof(m2Rot) == 4, "float pair alignment");
    _Static_assert(_Alignof(m2Pos2) == 8 && _Alignof(m2Transform) == 8, "double alignment");

#if defined(_MSC_VER) && defined(_M_X64)
#include <xmmintrin.h>
#elif defined(_MSC_VER) && defined(_M_ARM64)
#include <arm_neon.h>
#endif

    /// Pinned minimum: exactly (a < b ? a : b), in this operand order -
    /// including signed zero and NaN behavior. MSVC folds the plain ternary
    /// under value numbering (+0/-0 compare equal), so on MSVC x64 this is
    /// implemented with scalar MINSS, whose ISA-defined semantics are exactly
    /// the pinned form. NEON FMIN has different semantics and must never be
    /// used here; GCC/Clang compile the ternary faithfully (CI-verified).
    static inline float m2MinF(float a, float b)
    {
#if defined(_MSC_VER) && defined(_M_X64)
        return _mm_cvtss_f32(_mm_min_ss(_mm_set_ss(a), _mm_set_ss(b)));
#elif defined(_MSC_VER) && defined(_M_ARM64)
    // The plain ternary got folded away from the pinned zero-sign
    // semantics (the arm64 cell caught it on its first run), so the
    // select is explicit NEON: compare into a mask, bit-select the
    // lanes - never fminnm.
    float32x2_t va = vdup_n_f32(a);
    float32x2_t vb = vdup_n_f32(b);
    return vget_lane_f32(vbsl_f32(vclt_f32(va, vb), va, vb), 0);
#elif defined(_MSC_VER)
#error                                                                                             \
    "MSVC on this target: pinned min/max semantics unproven; implement with verified intrinsics first"
#else
    return a < b ? a : b;
#endif
    }

    /// Pinned maximum: exactly (a > b ? a : b), in this operand order.
    /// See m2MinF for the MSVC note (scalar MAXSS matches the pinned form).
    static inline float m2MaxF(float a, float b)
    {
#if defined(_MSC_VER) && defined(_M_X64)
        return _mm_cvtss_f32(_mm_max_ss(_mm_set_ss(a), _mm_set_ss(b)));
#elif defined(_MSC_VER) && defined(_M_ARM64)
    // See m2MinF: explicit NEON compare + bit-select.
    float32x2_t va = vdup_n_f32(a);
    float32x2_t vb = vdup_n_f32(b);
    return vget_lane_f32(vbsl_f32(vcgt_f32(va, vb), va, vb), 0);
#elif defined(_MSC_VER)
#error                                                                                             \
    "MSVC on this target: pinned min/max semantics unproven; implement with verified intrinsics first"
#else
    return a > b ? a : b;
#endif
    }

    /// Absolute value, pinned to IEEE |x| semantics: the sign bit is
    /// cleared, so m2AbsF(-0.0f) == +0.0f on every platform. Implemented
    /// as a bit mask because a ternary abs is compiler-foldable into
    /// divergent ±0 behavior (MSVC emits a sign-mask, GCC/Clang keep the
    /// branch - a real cross-cell hash break, caught by the gate).
    static inline float m2AbsF(float a)
    {
        union
        {
            float f;
            uint32_t u;
        } bits;
        bits.f = a;
        bits.u &= 0x7FFFFFFFu;
        return bits.f;
    }

    /// Clamp to [lo, hi] using the pinned min/max.
    static inline float m2ClampF(float a, float lo, float hi)
    {
        return m2MaxF(lo, m2MinF(a, hi));
    }

    /// Map an angle to [-pi, pi] (boundary within one rounding step of pi).
    /// Deterministic on every platform (uses only +, -, *, / and floorf).
    /// Valid input range is |radians| <= 1.0e6f (asserted in debug); beyond
    /// float precision limits an angle is meaningless anyway, and the
    /// deterministic fallback is the clamped boundary, never NaN.
    float m2UnwindAngle(float radians);

    /// Build a rotation from an angle. Deterministic across platforms:
    /// does not call libm; never returns NaN (out-of-range input falls
    /// back deterministically, see m2UnwindAngle). Accuracy is a
    /// documented approximation, identical bits everywhere.
    m2Rot m2MakeRot(float radians);

    /// Deterministic atan2 replacement. Returns 0 for (0, 0) instead of NaN.
    float m2Atan2(float y, float x);

    /// Renormalize a rotation. Every rotation composition site must call
    /// this immediately (drift control is part of the determinism contract).
    /// A degenerate input (zero or non-finite magnitude) returns the
    /// identity rotation - never NaN, never a non-unit result.
    m2Rot m2NormalizeRot(m2Rot q);

    /// Compose two rotations (q followed by r), renormalized.
    m2Rot m2MulRot(m2Rot q, m2Rot r);

    /// True if the rotation is unit length within tolerance.
    int m2IsNormalizedRot(m2Rot q);

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_MATH_H
