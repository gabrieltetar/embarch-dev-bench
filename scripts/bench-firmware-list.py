#!/usr/bin/env python3
"""Write release/bench-firmware.json, the list of bench firmware a release ships.

  bench-firmware-list.py [--version V] [--commit C] [--out FILE] <release> <image>...

An <image> is `app:board:path` (a .hex, .elf, or a bin as `app:board:path@base`,
base decimal or 0x hex): its app and board are named here, its version defaults
to <release> and its commit to `git rev-parse HEAD` of this repo (both are the
release's, so --version and --commit override them once for every image).
Each image's identity and segments come from embarch-study-designer's
`image_segments` example, run through cargo in the sibling checkout
(../embarch-study-designer, or --study-designer): the segment reduction is not
reimplemented here. Entries are sorted by (app, board); a repeated (app, board)
is an error. The output has the shape of `BenchFirmwareList`
(embarch-study-designer interfaces/hil-image.md), schema 1, and is committed with
the release it names. Default output: release/bench-firmware.json at the repo root.

Exit codes: 0 written, 2 error. Standard library only, plus cargo.
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SCHEMA = 1


def parse_image(spec: str):
    """Split `app:board:path[@base]` into (app, board, path, base or None)."""
    parts = spec.split(":", 2)
    if len(parts) != 3 or not all(parts):
        raise SystemExit(f"{spec}: want app:board:path[@base]")
    app, board, path = parts
    base = None
    if "@" in path:
        path, base = path.rsplit("@", 1)
    return app, board, Path(path).resolve(), base


def describe(designer: Path, path: Path, base):
    """Run the shared crate's example on one image; return its parsed JSON."""
    cmd = ["cargo", "run", "-q", "--features", "hil-image", "--example", "image_segments", "--", str(path)]
    if base is not None:
        cmd.append(base)
    try:
        r = subprocess.run(cmd, cwd=designer, capture_output=True, text=True)
    except OSError as exc:
        raise SystemExit(f"cannot run cargo in {designer}: {exc}")
    if r.returncode != 0:
        raise SystemExit(f"{path}: image_segments failed: {(r.stderr or r.stdout).strip()}")
    return json.loads(r.stdout)


def head_commit() -> str:
    r = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit("no --commit given and git rev-parse HEAD failed")
    return r.stdout.strip()


def build(release, specs, designer, version=None, commit=None):
    commit = commit or head_commit()
    images = []
    for spec in specs:
        app, board, path, base = parse_image(spec)
        d = describe(designer, path, base)
        images.append({"app": app, "board": board, "version": version or release, "commit": commit,
                       "identity": d["identity"], "segments": d["segments"]})
    images.sort(key=lambda i: (i["app"], i["board"]))
    for a, b in zip(images, images[1:]):
        if (a["app"], a["board"]) == (b["app"], b["board"]):
            raise SystemExit(f"two images for {a['app']} on {a['board']}")
    return {"schema": SCHEMA, "release": release, "images": images}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("release")
    ap.add_argument("image", nargs="+")
    ap.add_argument("--version")
    ap.add_argument("--commit")
    ap.add_argument("--out", type=Path, default=ROOT / "release" / "bench-firmware.json")
    ap.add_argument("--study-designer", type=Path, default=ROOT.parent / "embarch-study-designer")
    a = ap.parse_args(argv)
    if not (a.study_designer / "Cargo.toml").is_file():
        print(f"error: no embarch-study-designer checkout at {a.study_designer}", file=sys.stderr)
        return 2
    try:
        doc = build(a.release, a.image, a.study_designer, a.version, a.commit)
    except (SystemExit, ValueError, KeyError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {a.out}: {len(doc['images'])} image(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
