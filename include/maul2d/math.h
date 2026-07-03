// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen

#ifndef MAUL2D_MATH_H
#define MAUL2D_MATH_H

#include <assert.h>
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
static_assert(sizeof(m2Vec2) == 8, "m2Vec2 must be 8 bytes");
static_assert(sizeof(m2Pos2) == 16, "m2Pos2 must be 16 bytes");
static_assert(sizeof(m2Rot) == 8, "m2Rot must be 8 bytes");
static_assert(sizeof(m2Transform) == 24, "m2Transform must be 24 bytes, no padding");

/// Pinned minimum: exactly (a < b ? a : b), in this operand order.
/// Native SIMD min instructions must not replace this: SSE and NEON
/// disagree on signed zero and NaN. See m2MaxF for the mirror rule.
static inline float m2MinF(float a, float b)
{
	return a < b ? a : b;
}

/// Pinned maximum: exactly (a > b ? a : b), in this operand order.
static inline float m2MaxF(float a, float b)
{
	return a > b ? a : b;
}

/// Absolute value without libm.
static inline float m2AbsF(float a)
{
	return a < 0.0f ? -a : a;
}

/// Clamp to [lo, hi] using the pinned min/max.
static inline float m2ClampF(float a, float lo, float hi)
{
	return m2MaxF(lo, m2MinF(a, hi));
}

/// Map an angle in radians to [-pi, pi). Deterministic on every
/// platform (uses only +, -, *, / and floorf). Intended for API-rim
/// angles of ordinary magnitude; extreme inputs lose precision like
/// any float angle does.
float m2UnwindAngle(float radians);

/// Build a rotation from an angle. Deterministic across platforms:
/// does not call libm. Accuracy is a documented approximation
/// (see src/math_functions.c), identical bits everywhere.
m2Rot m2MakeRot(float radians);

/// Deterministic atan2 replacement. Returns 0 for (0, 0) instead of NaN.
float m2Atan2(float y, float x);

/// Renormalize a rotation. Every rotation composition site must call
/// this immediately (drift control is part of the determinism contract).
m2Rot m2NormalizeRot(m2Rot q);

/// Compose two rotations (q followed by r), renormalized.
m2Rot m2MulRot(m2Rot q, m2Rot r);

/// True if the rotation is unit length within tolerance.
int m2IsNormalizedRot(m2Rot q);

#ifdef __cplusplus
}
#endif

#endif // MAUL2D_MATH_H
