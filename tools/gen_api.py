#!/usr/bin/env python3
# Generate docs/api.md from the public headers: every function
# signature with the doc comment that precedes it, grouped by header
# and by the m2Type_ family the name announces. Pure text extraction,
# no compiler, so it stays honest to what the headers actually say.
#
# Usage: python3 tools/gen_api.py  (run from the repo root)

import glob
import os
import re

HEADERS = [
    "base.h", "math.h", "world.h", "body.h", "shape.h",
    "joint.h", "events.h", "particle.h",
]

HEADER_TITLES = {
    "base.h": "Base: versions, allocator, ids",
    "math.h": "Math: vectors, rotations, deterministic trig",
    "world.h": "World: lifecycle, stepping, snapshots, queries, diagnostics",
    "body.h": "Bodies: creation, dynamics, mass, readback",
    "shape.h": "Shapes: geometry, filters, chains, hulls, decomposition",
    "joint.h": "Joints: eleven types, motors, limits, springs, breaking",
    "events.h": "Events: contact, sensor and joint streams",
    "particle.h": "Fluids: particles, behaviors, lifetime, fills, buoyancy",
}


def collect(path):
    """Return a list of (comment_lines, signature) for the header."""
    lines = open(path).read().split("\n")
    entries = []
    comment = []
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        if stripped.startswith("///"):
            comment.append(stripped[3:].strip())
            i += 1
            continue
        # A function declaration: a return type, an m2 name, an open
        # paren. Typedefs and macros are skipped.
        m = re.match(r"^\s*[A-Za-z_][A-Za-z0-9_ \*]*\bm2[A-Za-z0-9_]+\s*\(", line)
        starts_decl = (m is not None and "(" in line and
                       not stripped.startswith("typedef") and
                       not stripped.startswith("#") and
                       "static const" not in stripped)
        if starts_decl:
            # Gather the whole signature to its terminating semicolon.
            sig = stripped
            j = i
            while not sig.rstrip().endswith(";") and j < len(lines) - 1:
                j += 1
                sig += " " + lines[j].strip()
            sig = re.sub(r"\s+", " ", sig).strip()
            if sig.endswith(";") and "(" in sig and ")" in sig:
                entries.append((comment, sig))
            comment = []
            i = j + 1
            continue
        if stripped and not stripped.startswith("//"):
            comment = []
        i += 1
    return entries


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = []
    out.append("# Maul2D API reference")
    out.append("")
    out.append("Generated from the public headers by tools/gen_api.py; the")
    out.append("headers are the source of truth and this file mirrors them.")
    out.append("For the why and how, read the [guide](guide.md).")
    out.append("")

    total = 0
    for h in HEADERS:
        path = os.path.join(root, "include", "maul2d", h)
        if not os.path.exists(path):
            continue
        entries = collect(path)
        fns = [e for e in entries if re.search(r"\bm2[A-Za-z0-9_]+\s*\(", e[1])]
        if not fns:
            continue
        out.append("## " + HEADER_TITLES.get(h, h))
        out.append("")
        for comment, sig in fns:
            out.append("```c")
            out.append(sig)
            out.append("```")
            if comment:
                out.append(" ".join(comment))
            out.append("")
            total += 1

    out.append("---")
    out.append("")
    out.append(str(total) + " functions across "
               + str(len([h for h in HEADERS
                          if os.path.exists(os.path.join(root, "include", "maul2d", h))]))
               + " headers.")
    out.append("")

    dest = os.path.join(root, "docs", "api.md")
    open(dest, "w").write("\n".join(out))
    print("wrote " + dest + " (" + str(total) + " functions)")


if __name__ == "__main__":
    main()
