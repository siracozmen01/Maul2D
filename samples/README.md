# Samples

## minimal: ten minutes to a rollback

Install Maul2D somewhere, then point the sample at it:

    cmake -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build
    cmake --install build --prefix /tmp/maul2d-prefix

    cd samples/minimal
    cmake -B build -DCMAKE_PREFIX_PATH=/tmp/maul2d-prefix
    cmake --build build
    ./build/minimal

It stacks a tower, snapshots it, wrecks it, and rolls the wreck
back out of existence; the hashes on screen are the proof. From
here the guide (docs/guide.md) covers everything else: joints,
water, journals, queries, the character mover.
