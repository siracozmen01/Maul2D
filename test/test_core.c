// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Core determinism gate. This test does three jobs:
//   1. Correctness: the approximations stay within their documented error
//      bounds against libm references (libm is allowed in tests only).
//   2. Environment canaries: fail loudly if the compiler contracted FMA
//      or if min/max semantics drift from the pinned form.
//   3. Determinism hash: a dense sweep of every deterministic function is
//      hashed; CI compares the printed hash across every platform cell.
//      Different bits anywhere = red build.

#include "maul2d/maul2d.h"

#include <math.h>
#include <stdio.h>
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

static uint32_t FloatBits(float x)
{
	uint32_t u;
	memcpy(&u, &x, sizeof(u));
	return u;
}

// --- 1. Accuracy against libm (tolerances documented in topic-01) ---------

static void TestAccuracy(void)
{
	double maxSinErr = 0.0;
	double maxCosErr = 0.0;
	double maxAtanErr = 0.0;

	for (int i = 0; i <= 200000; ++i)
	{
		float angle = -10.0f + 20.0f * (float)i / 200000.0f;
		m2Rot q = m2MakeRot(angle);
		double c = cos((double)angle);
		double s = sin((double)angle);
		double cosErr = fabs((double)q.c - c);
		double sinErr = fabs((double)q.s - s);
		if (cosErr > maxCosErr)
		{
			maxCosErr = cosErr;
		}
		if (sinErr > maxSinErr)
		{
			maxSinErr = sinErr;
		}
		CHECK(m2IsNormalizedRot(q), "m2MakeRot must return a unit rotation");
	}

	for (int iy = -50; iy <= 50; ++iy)
	{
		for (int ix = -50; ix <= 50; ++ix)
		{
			if (ix == 0 && iy == 0)
			{
				continue;
			}
			float y = 0.13f * (float)iy;
			float x = 0.17f * (float)ix;
			double err = fabs((double)m2Atan2(y, x) - atan2((double)y, (double)x));
			if (err > maxAtanErr)
			{
				maxAtanErr = err;
			}
		}
	}

	// Bhaskara I: ~2e-3 absolute worst case; atan2 minimax: ~1e-4.
	CHECK(maxSinErr < 3.0e-3, "m2MakeRot sine outside error bound");
	CHECK(maxCosErr < 3.0e-3, "m2MakeRot cosine outside error bound");
	CHECK(maxAtanErr < 1.0e-3, "m2Atan2 outside error bound");
	CHECK(m2Atan2(0.0f, 0.0f) == 0.0f, "m2Atan2(0,0) must be 0, not NaN");

	printf("accuracy: sin %.2e cos %.2e atan2 %.2e\n", maxSinErr, maxCosErr, maxAtanErr);
}

// --- 2a. FMA canary (RT1-DET-1) -------------------------------------------
// float: a = 1 + 2^-13. Exact a*a = 1 + 2^-12 + 2^-26. The 2^-26 term is
// below half an ulp of 1, so the separately-rounded product is exactly
// 1 + 2^-12 and r == 0. A contracted fused multiply-add keeps the low
// term, so r == 2^-26 != 0. The double case mirrors it at 2^-27 (the
// low term 2^-54 sits below half an ulp of 1, which is 2^-53).

static volatile float s_canaryA = 1.0f + 0x1p-13f;
static volatile float s_canaryC = 1.0f + 0x1p-12f;
static volatile double s_canaryDA = 1.0 + 0x1p-27;
static volatile double s_canaryDC = 1.0 + 0x1p-26;

static void TestFmaCanary(void)
{
	float a = s_canaryA;
	float c = s_canaryC;
	float r = a * a - c;
	CHECK(r == 0.0f, "float FMA contraction detected: build flags are wrong");

	double da = s_canaryDA;
	double dc = s_canaryDC;
	double dr = da * da - dc;
	CHECK(dr == 0.0, "double FMA contraction detected: build flags are wrong");
}

// --- 2b. Pinned min/max semantics (RT1-DET-2) ------------------------------

static void TestMinMaxSemantics(void)
{
	float pz = 0.0f;
	float nz = -0.0f;

	// a < b ? a : b, exactly. (+0 < -0) is false, so b comes back.
	CHECK(FloatBits(m2MinF(pz, nz)) == FloatBits(nz), "m2MinF(+0,-0) must return -0");
	CHECK(FloatBits(m2MinF(nz, pz)) == FloatBits(pz), "m2MinF(-0,+0) must return +0");
	CHECK(FloatBits(m2MaxF(pz, nz)) == FloatBits(nz), "m2MaxF(+0,-0) must return -0");
	CHECK(FloatBits(m2MaxF(nz, pz)) == FloatBits(pz), "m2MaxF(-0,+0) must return +0");

	// NaN never satisfies a comparison, so b comes back, verbatim.
	uint32_t nanBits = 0x7FC00000u;
	float qnan;
	memcpy(&qnan, &nanBits, sizeof(qnan));
	CHECK(FloatBits(m2MinF(qnan, 1.0f)) == FloatBits(1.0f), "m2MinF(NaN,1) must return 1");
	CHECK(FloatBits(m2MinF(1.0f, qnan)) == nanBits, "m2MinF(1,NaN) must return the NaN");
}

// --- 3. Determinism hash ----------------------------------------------------

static uint64_t HashSweep(void)
{
	uint64_t h = M2_HASH_INIT;

	// Trig sweep: hash the raw output bits of every function.
	for (int i = 0; i <= 100000; ++i)
	{
		float angle = -12.0f + 24.0f * (float)i / 100000.0f;
		m2Rot q = m2MakeRot(angle);
		float u = m2UnwindAngle(angle);
		h = m2Hash64(h, &q, sizeof(q));
		h = m2Hash64(h, &u, sizeof(u));
	}

	for (int iy = -40; iy <= 40; ++iy)
	{
		for (int ix = -40; ix <= 40; ++ix)
		{
			float a = m2Atan2(0.31f * (float)iy, 0.27f * (float)ix);
			h = m2Hash64(h, &a, sizeof(a));
		}
	}

	// Rotation composition chain: exercises mul + renormalize drift control.
	m2Rot q = m2MakeRot(0.001f);
	m2Rot r = m2MakeRot(0.7f);
	for (int i = 0; i < 10000; ++i)
	{
		r = m2MulRot(r, q);
		h = m2Hash64(h, &r, sizeof(r));
	}

	// Double-precision position arithmetic (the f64 side of hybrid precision).
	double px = 1.0e6;
	double py = -2.5e6;
	for (int i = 0; i < 10000; ++i)
	{
		px += 0.016 * (double)((i % 7) - 3) * 1.25;
		py += 0.016 * (double)((i % 5) - 2) * 0.75;
		double mix = px * 3.0 + py / 7.0;
		h = m2Hash64(h, &mix, sizeof(mix));
	}

	// Pinned min/max outcomes are part of the contract, so hash them too.
	float pz = 0.0f;
	float nz = -0.0f;
	float mm[4] = { m2MinF(pz, nz), m2MinF(nz, pz), m2MaxF(pz, nz), m2MaxF(nz, pz) };
	h = m2Hash64(h, mm, sizeof(mm));

	return h;
}

int main(void)
{
	printf("maul2d version %d\n", m2GetVersion());

	TestAccuracy();
	TestFmaCanary();
	TestMinMaxSemantics();

	uint64_t hash = HashSweep();
	// CI extracts this line from every platform cell and requires equality.
	printf("M2_DET_HASH=%016llx\n", (unsigned long long)hash);

	if (s_failures > 0)
	{
		printf("%d failure(s)\n", s_failures);
		return 1;
	}
	printf("all checks passed\n");
	return 0;
}
