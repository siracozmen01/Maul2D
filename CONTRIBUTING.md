# Contributing to Maul2D

Contributions are welcome. A few ground rules keep the project sane:

- **The maintainer has final say on every merge.** Expect design questions on
  anything that changes simulation behavior.
- **All CI gates must be green.** The determinism gates are non-negotiable:
  the whole point of this engine is bit-identical simulation across
  platforms, compilers, and thread counts. A PR that trades determinism for
  speed or convenience will not merge, however fast it is.
- **Sign your commits with DCO** (`git commit -s`). Contributions are MIT,
  inbound = outbound. No CLA.
- **If you deviate from how the reference engines (Box2D, Box3D, Jolt) solve
  a problem, argue the deviation in the PR body.** "It also works" is not an
  argument; these codebases carry twenty years of hard-won lessons.
- Formatting is enforced by the checked-in `.clang-format` — run it, don't
  debate it. Commit messages: imperative subject, body explains why, no
  trailers.
- Bug reports with a reproducing scene are gold. Determinism bug reports
  should include the platform pair and the `M2_DET_HASH` outputs.
