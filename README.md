# Maul2D

[![ci](https://github.com/siracozmen01/Maul2D/actions/workflows/ci.yml/badge.svg)](https://github.com/siracozmen01/Maul2D/actions/workflows/ci.yml)

A deterministic 2D physics engine for games. Written in C17 with a
pure C API, zero dependencies, MIT licensed.

**[Play the testbed in your browser](https://siracozmen01.github.io/Maul2D/)**:
the same engine, compiled to WebAssembly, producing the same bits
it produces everywhere else (the wasm cell in CI proves that on
every push). Hold R and time runs backward.

Maul2D's difference is a contract, not a feature flag: the same
inputs produce the same bits on every supported platform, and the
engine is built around that promise end to end. Snapshots restore
worlds bit-exactly, a command journal records sessions and replays
them byte for byte, and CI fails on a single differing bit across
sixteen gated hash lines and ten platform cells spanning x64 AVX2,
arm64 NEON, portable scalar and WebAssembly. Rollback netcode,
deterministic lockstep, kill-cam replays and server-verified
simulation stop being research projects and become four lines of
code.

```c
int32_t size = m2World_SnapshotSize(world);
m2World_Snapshot(world, buffer, size);
/* ... mispredicted steps ... */
m2World_Restore(world, buffer, size); // bit-exact resimulation from here
```

## What is in the box

- **Rigid bodies**: circles, capsules, polygons (rounded too),
  segments and one-way chain terrain with ghost corners; static,
  kinematic, dynamic; forces, damping, dominance groups, conveyor
  surfaces, explosions.
- **A soft-step solver** with speculative contacts, warm starting
  and graph-colored SIMD solving. Worker count is non-semantic: 1,
  2 or 8 threads produce bit-identical trajectories and a CI gate
  proves it every commit.
- **Eleven joint types**: distance (with hard range), revolute
  (with angular spring), prismatic, weld, wheel, motor, mouse,
  filter, gear, pulley and ratchet; motors, limits, runtime tuning,
  and built-in breaking with events.
- **Particle fluids**: an opt-in, deterministic particle system
  with water, viscosity, surface tension, powder and jelly (spring
  and elastic nets), two-way coupled with rigid bodies, living
  inside the same snapshot, journal and hash contract as everything
  else.
- **Continuous collision** for bullets, island-coupled sleeping
  with a hibernation short-circuit, contact and sensor events with
  strict begin/end bookending.
- **Queries**: rays, shape sweeps, overlaps and all-hits variants,
  a character mover kit (collide, solve planes, clip), and particle
  region queries; all canonically ordered and exact hundreds of
  kilometers from the origin (positions are f64).
- **Geometry tools**: convex hull from a point cloud and convex
  decomposition from a concave outline, the road from a sprite
  silhouette to a destructible body.
- **Integration surface**: 267 frozen 1.x functions, full readback
  (a mirror test rebuilds a world from getters alone and matches
  its hash), debug draw, deterministic counters and profile,
  allocator hooks.

## Quick start

Prebuilt static libraries for Linux, Windows and macOS are attached
to every [release](https://github.com/siracozmen01/Maul2D/releases),
alongside a playable Windows testbed. From source:

```
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

No dependencies. A C17 compiler is required (MSVC needs VS2022 or
newer: older versions cannot disable FP contraction, which the
determinism contract requires). x64 builds target AVX2+FMA (Haswell
2013+); configure with `-DMAUL2D_SIMD=scalar` for a portable
fallback that produces bit-identical results, and CI enforces that
equality.

`cmake --install` ships the library, headers, a CMake package and a
pkg-config file; `find_package(maul2d)` and `pkg-config maul2d`
both work from any prefix. Three samples show the shape of the API:

- `samples/minimal`: a standalone `find_package` consumer that
  stacks a tower, snapshots it, wrecks it and rolls the wreck back
  out of existence, hashes on screen as proof.
- `sample_car`: a motorized, sprung two-wheeler drives itself and
  prints telemetry; it ends on the same world hash on every
  supported platform.
- `sample_replay`: records a session into the journal, replays it
  into a fresh world, and compares end hashes.

The interactive testbed (raylib, viewer only, outside the engine's
dependency surface) runs [in the browser](https://siracozmen01.github.io/Maul2D/)
or builds natively with `-DMAUL2D_TESTBED=ON`: twelve
scenes including a playable platformer on the character mover, a
machinery hall, particle goo, and a rewind ring; hold R and time
runs backward, bit for bit.

## Determinism and performance

Determinism here means: pinned IEEE arithmetic (no fast math, no FP
contraction), explicit compare-and-select min/max, one bit law
across AVX2, NEON and scalar kernels, canonical ordering on every
path, and scheduling-independent threading. A hostile-integrator
fuzzer drives random sessions through four promises on every seed
(journal replay, rollback identity, unjournaled twin, threaded
twin), an API mirror test guards the readback surface, eight
red-team rounds attack the engine as code, and a weekly scheduled
CI run plus a 300k-step soak watch for toolchain drift.

That contract costs less than folklore says: a dual-backend harness
runs the same scenes through Maul2D and Box2D v3 side by side, and
Maul2D holds joint parity, sleeps for free, and stays within
roughly 1.1 to 1.2 times Box2D's AVX solve times on dense active
stacks while carrying guarantees no engine in its class ships.

## Status and stability

Current release: 1.8. The 1.x API surface is frozen: functions and
defs may be added in minor releases, but existing signatures,
semantics and id layouts do not change until a 2.0. Defs are
cookie-guarded, so a stale compiled caller fails loudly instead of
subtly. Snapshots and journal tapes are versioned artifacts of a
single library version: see "Versions and formats" in the guide.

## Learn more

- [The guide](docs/guide.md): the contract, rollback netcode,
  joints, water, queries, the character mover, and what Maul2D
  leaves out on purpose.
- [CONTRIBUTING.md](CONTRIBUTING.md) for contributions.
- [THIRD_PARTY.md](THIRD_PARTY.md) for adapted-code licenses.

## Acknowledgments

Maul2D stands on the shoulders of [Box2D](https://github.com/erincatto/box2d)
by Erin Catto (MIT): several kernels (the polygon clipper, the
soft-step stage structure, joint formulations, ray casts, the
trigonometric approximations) are adapted from Box2D v3 sources,
with the adaptation noted in each file. The particle fluid recipe
descends from LiquidFun, and the ratchet joint from Chipmunk2D;
their licenses live in [THIRD_PARTY.md](THIRD_PARTY.md). The
determinism architecture (hybrid f64 positions, snapshot rollback,
the command journal, cross-platform hash gating) is Maul's own.
