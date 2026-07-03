// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// The cosine/sine (Bhaskara I) and atan2 (minimax polynomial) approximations
// are adapted from Box2D v3 (https://github.com/erincatto/box2d),
// SPDX-FileCopyrightText: 2023 Erin Catto, SPDX-License-Identifier: MIT.
// They are chosen for bit-identical results on every platform and for
// behavioral parity with the reference implementation.

#include "maul2d/math.h"

#include <math.h> // floorf, sqrtf only: both are IEEE-exact operations

float m2UnwindAngle(float radians)
{
    // Map to [-pi, pi) using only *, +, - and floorf.
    float twoPi = 2.0f * M2_PI;
    float k = floorf((radians + M2_PI) / twoPi);
    return radians - twoPi * k;
}

m2Rot m2MakeRot(float radians)
{
    float x = m2UnwindAngle(radians);
    float pi2 = M2_PI * M2_PI;

    // Cosine needs the angle in [-pi/2, pi/2].
    float c;
    if (x < -0.5f * M2_PI)
    {
        float y = x + M2_PI;
        float y2 = y * y;
        c = -(pi2 - 4.0f * y2) / (pi2 + y2);
    }
    else if (x > 0.5f * M2_PI)
    {
        float y = x - M2_PI;
        float y2 = y * y;
        c = -(pi2 - 4.0f * y2) / (pi2 + y2);
    }
    else
    {
        float y2 = x * x;
        c = (pi2 - 4.0f * y2) / (pi2 + y2);
    }

    // Sine needs the angle in [0, pi].
    float s;
    if (x < 0.0f)
    {
        float y = x + M2_PI;
        s = -16.0f * y * (M2_PI - y) / (5.0f * pi2 - 4.0f * y * (M2_PI - y));
    }
    else
    {
        s = 16.0f * x * (M2_PI - x) / (5.0f * pi2 - 4.0f * x * (M2_PI - x));
    }

    float mag = sqrtf(s * s + c * c);
    float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
    m2Rot q = {c * invMag, s * invMag};
    return q;
}

float m2Atan2(float y, float x)
{
    // (0, 0) returns 0 instead of NaN: the sim path is NaN-free by contract.
    if (x == 0.0f && y == 0.0f)
    {
        return 0.0f;
    }

    float ax = m2AbsF(x);
    float ay = m2AbsF(y);
    float mx = m2MaxF(ay, ax);
    float mn = m2MinF(ay, ax);
    float a = mn / mx;

    // Minimax polynomial approximation to atan(a) on [0, 1].
    float s = a * a;
    float c = s * a;
    float q = s * s;
    float r = 0.024840285f * q + 0.18681418f;
    float t = -0.094097948f * q - 0.33213072f;
    r = r * s + t;
    r = r * c + a;

    // Map to the full circle.
    if (ay > ax)
    {
        r = 1.57079637f - r;
    }
    if (x < 0.0f)
    {
        r = 3.14159274f - r;
    }
    if (y < 0.0f)
    {
        r = -r;
    }

    return r;
}

m2Rot m2NormalizeRot(m2Rot q)
{
    float mag = sqrtf(q.c * q.c + q.s * q.s);
    float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
    m2Rot result = {q.c * invMag, q.s * invMag};
    return result;
}

m2Rot m2MulRot(m2Rot q, m2Rot r)
{
    // Complex multiply, then renormalize: every composition site
    // renormalizes immediately (drift control is part of the contract).
    m2Rot qr;
    qr.c = q.c * r.c - q.s * r.s;
    qr.s = q.s * r.c + q.c * r.s;
    return m2NormalizeRot(qr);
}

int m2IsNormalizedRot(m2Rot q)
{
    float mag2 = q.c * q.c + q.s * q.s;
    float tolerance = 4.0f * 1.19209290e-7f; // 4 * FLT_EPSILON
    return mag2 > 1.0f - tolerance && mag2 < 1.0f + tolerance;
}
