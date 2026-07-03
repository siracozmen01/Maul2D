# Maul2D

The 2D companion to [Maul3D](https://github.com/siracozmen01/Maul3D): same solver core, same C API style, two dimensions. This is where the solver, the determinism pipeline, and the rollback system get validated before the 3D engine takes them over.

Written in C17. MIT licensed.

## Status

Early. The engine is being built determinism-first: what exists today is the
core math substrate (deterministic trig, pinned min/max semantics, the state
hash) and the cross-platform determinism gate in CI. Every commit must produce
bit-identical results on Linux/Windows/macOS, x64 and arm64, GCC/Clang/MSVC —
the gate compares raw output hashes across all cells and fails the build on a
single differing bit. The FMA canary and the min/max semantics tests catch
toolchain drift at the function level before it can smear into simulation
noise.

Solver, broadphase, and the rollback/snapshot system land next, in that
architecture. Contributions: see [CONTRIBUTING.md](CONTRIBUTING.md).

## Building

```
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/test_core
```

No dependencies. C17 compiler required (MSVC ≥ VS2022 17.0 — older versions
cannot disable FP contraction, which this engine requires).
