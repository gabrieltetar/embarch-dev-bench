#!/usr/bin/env python3
"""Check or refresh the sha256sum fields of a west patches.yml.

  patch-sums.py --check  <patches.yml>   list stale entries, exit 1 if any
  patch-sums.py --update <patches.yml>   rewrite only the stale sha256sum values

Runs from the repo alone: no west workspace, no network, standard library only.

Path resolution is west patch's own (zephyr scripts/west_commands/patch.py):
patches.yml lives at <manifest_dir>/zephyr/patches.yml and each entry's `path`
is joined to its sibling `patches/` directory, the default patch base
(<manifest_dir>/zephyr/patches). So `path: zephyr/local/x.patch` in
.../manifest/zephyr/patches.yml names .../manifest/zephyr/patches/zephyr/local/x.patch.
The sum is west's too: the file is read as UTF-8 text with universal newlines,
so CRLF and LF hash alike.

--update edits each stale value on its own line; every other byte of the file
(fields, order, comments, quoting, line endings) is left as it was.
Exit codes: 0 clean (or updated), 1 stale entries on --check, 2 error.
"""
import argparse
import hashlib
import re
import sys
from pathlib import Path

ITEM = re.compile(r"^(\s*)-\s+(.*)$")
KEY = re.compile(r"^(\s*)(?:-\s+)?([A-Za-z0-9_-]+):\s*(.*?)\s*(?:#.*)?$")


def west_sum(path: Path) -> str:
    with open(path, encoding="utf-8", newline=None) as fp:
        return hashlib.sha256(fp.read().encode("utf-8")).hexdigest()


def unquote(v: str) -> str:
    return v[1:-1] if len(v) >= 2 and v[0] == v[-1] and v[0] in "'\"" else v


def entries(lines):
    """Yield (path, sha_line_index, sha_value, sha_quote) per patch entry."""
    cur = None  # dict: key_indent, path, idx, sha
    item_indent = None
    for i, raw in enumerate(lines):
        line = raw.rstrip("\r\n")
        m = ITEM.match(line)
        if m and (item_indent is None or len(m.group(1)) == item_indent):
            if cur:
                yield cur
            item_indent = len(m.group(1))
            cur = {"path": None, "idx": None, "sha": None}
        if cur is None:
            continue
        k = KEY.match(line)
        if not k:
            continue
        indent = len(k.group(1)) + (2 if ITEM.match(line) else 0)
        if indent != item_indent + 2:
            continue
        if k.group(2) == "path":
            cur["path"] = unquote(k.group(3))
        elif k.group(2) == "sha256sum":
            cur["idx"], cur["sha"] = i, unquote(k.group(3))
    if cur:
        yield cur


def scan(yml: Path):
    """Return (lines, stale) where stale is [(entry, actual_sum)]."""
    with open(yml, encoding="utf-8", newline="") as fp:
        lines = fp.read().splitlines(keepends=True)
    base = yml.parent / "patches"
    stale = []
    for e in entries(lines):
        if e["path"] is None or e["idx"] is None:
            raise SystemExit(f"{yml}: entry without path or sha256sum near line {(e['idx'] or 0) + 1}")
        target = base / e["path"]
        if not target.is_file():
            raise SystemExit(f"{yml}: {e['path']}: no such file {target}")
        actual = west_sum(target)
        if actual != e["sha"]:
            stale.append((e, actual))
    return lines, stale


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--update", action="store_true")
    ap.add_argument("patches_yml", type=Path)
    a = ap.parse_args(argv)
    try:
        lines, stale = scan(a.patches_yml)
    except (OSError, UnicodeDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    for e, actual in stale:
        print(f"{a.patches_yml}:{e['idx'] + 1}: {e['path']}: sha256sum {e['sha']} != {actual}")
    if a.check:
        if stale:
            print(f"{len(stale)} stale sha256sum value(s); run --update", file=sys.stderr)
        return 1 if stale else 0
    for e, actual in stale:
        old = lines[e["idx"]]
        lines[e["idx"]] = old.replace(e["sha"], actual, 1)
    if stale:
        with open(a.patches_yml, "w", encoding="utf-8", newline="") as fp:
            fp.write("".join(lines))
        print(f"updated {len(stale)} sha256sum value(s)")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit as exc:
        if isinstance(exc.code, str):
            print(f"error: {exc.code}", file=sys.stderr)
            sys.exit(2)
        raise
