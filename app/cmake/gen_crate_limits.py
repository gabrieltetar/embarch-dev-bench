#!/usr/bin/env python3
"""Write crate_limits.h: embarch-study-designer's limits and `.eap` wire enums, for C.

Run at configure time by crate_limits.cmake (decision 56). Every constant
dev-bench's headers used to type by hand with a `/* mirrors limits::X */`
comment comes from here instead, and so does every `.eap` enum whose
declaration order is its postcard wire value. A header that still disagrees
with the crate then fails to compile rather than decoding with an old table.

What it reads, and nothing else:

  - src/limits.rs: every `pub const NAME: usize = <integer or another such const>;`
    becomes `#define CRATE_NAME <value>`.
  - The enums in ENUMS below: each variant becomes
    `#define CRATE_<ENUM>_<VARIANT> <declaration index>`, plus
    `CRATE_<ENUM>_COUNT`. postcard writes a variant as its declaration index,
    whatever discriminant the source gives it.

Anything it cannot read exactly is an error, never a guess: a missing file, a
const whose value is an expression, an enum it cannot find. The header is
rewritten only when its text changes, so an unchanged crate triggers no
rebuild.
"""

import argparse
import re
import sys
from pathlib import Path

LIMITS_SRC = "src/limits.rs"

# (crate source file, Rust enum name, C macro prefix)
ENUMS = (
    ("src/decoder.rs", "ScalarType", "CRATE_SCALAR_TYPE"),
    ("src/eap.rs", "Operand", "CRATE_OPERAND"),
    ("src/eap.rs", "Expr", "CRATE_EXPR"),
    ("src/eap.rs", "CompareOp", "CRATE_COMPARE_OP"),
    ("src/eap.rs", "StateKind", "CRATE_STATE_KIND"),
    ("src/eap.rs", "TerminalOutcome", "CRATE_TERMINAL_OUTCOME"),
)

CONST_RE = re.compile(r"^pub const ([A-Z][A-Z0-9_]*): usize = ([A-Za-z0-9_]+);", re.M)
VARIANT_RE = re.compile(r"^([A-Z][A-Za-z0-9]*)\b")
OPEN = "([{<"
CLOSE = ")]}>"


class CrateReadError(Exception):
    pass


def read_consts(text):
    raw = dict(CONST_RE.findall(text))
    if not raw:
        raise CrateReadError(f"no `pub const NAME: usize = ...;` found in {LIMITS_SRC}")
    resolved = {}

    def resolve(name, seen):
        if name in resolved:
            return resolved[name]
        if name in seen:
            raise CrateReadError(f"{LIMITS_SRC}: {name} is defined in terms of itself")
        value = raw[name]
        if re.fullmatch(r"[0-9][0-9_]*", value):
            number = int(value.replace("_", ""))
        elif value in raw:
            number = resolve(value, seen | {name})
        else:
            raise CrateReadError(
                f"{LIMITS_SRC}: {name} = {value}, which is neither an integer nor another "
                "usize const in that file"
            )
        resolved[name] = number
        return number

    for name in raw:
        resolve(name, frozenset())
    return resolved


def strip_comments(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def enum_body(text, enum, path):
    match = re.search(rf"\bpub enum {enum}\s*\{{", text)
    if match is None:
        raise CrateReadError(f"{path}: no `pub enum {enum} {{`")
    depth = 1
    start = match.end()
    for pos in range(start, len(text)):
        char = text[pos]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[start:pos]
    raise CrateReadError(f"{path}: `pub enum {enum}` is never closed")


def split_top_level(body):
    """Split an enum body on the commas between variants, not inside them."""
    items, depth, current = [], 0, []
    for char in body:
        if char in OPEN:
            depth += 1
        elif char in CLOSE:
            depth -= 1
        if char == "," and depth == 0:
            items.append("".join(current))
            current = []
        else:
            current.append(char)
    items.append("".join(current))
    return items


def read_variants(text, enum, path):
    body = re.sub(r"#\[[^\]]*\]", "", strip_comments(enum_body(text, enum, path)))
    variants = []
    for item in split_top_level(body):
        item = item.strip()
        if not item:
            continue
        match = VARIANT_RE.match(item)
        if match is None:
            raise CrateReadError(f"{path}: cannot read a variant name in `{enum}` from {item!r}")
        variants.append(match.group(1))
    if not variants:
        raise CrateReadError(f"{path}: `{enum}` has no variants")
    return variants


def macro_suffix(variant):
    """`U16Le` -> `U16_LE`, `SpanLen` -> `SPAN_LEN`."""
    return re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", variant).upper()


def render(consts, enums):
    lines = [
        "/* Generated at configure time by app/cmake/gen_crate_limits.py from",
        " * embarch-study-designer's own source (decision 56). Do not edit: change the",
        " * crate, and the next build regenerates this. */",
        "#ifndef EMBARCH_DEV_BENCH_CRATE_LIMITS_H_",
        "#define EMBARCH_DEV_BENCH_CRATE_LIMITS_H_",
        "",
        f"/* {LIMITS_SRC} */",
    ]
    for name, value in consts.items():
        lines.append(f"#define CRATE_{name} {value}")
    for path, enum, prefix, variants in enums:
        lines += ["", f"/* `{enum}` ({path}): postcard's wire value is the declaration index. */"]
        for index, variant in enumerate(variants):
            lines.append(f"#define {prefix}_{macro_suffix(variant)} {index}")
        lines.append(f"#define {prefix}_COUNT {len(variants)}")
    lines += ["", "#endif /* EMBARCH_DEV_BENCH_CRATE_LIMITS_H_ */", ""]
    return "\n".join(lines)


def read_text(crate, relative):
    path = crate / relative
    try:
        return path.read_text(encoding="utf-8")
    except OSError as err:
        raise CrateReadError(f"cannot read {path}: {err}") from err


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--crate", required=True, type=Path,
                        help="the embarch-study-designer checkout")
    parser.add_argument("--out", required=True, type=Path, help="the header to write")
    args = parser.parse_args(argv)

    try:
        consts = read_consts(read_text(args.crate, LIMITS_SRC))
        sources = {}
        enums = []
        for path, enum, prefix in ENUMS:
            if path not in sources:
                sources[path] = read_text(args.crate, path)
            enums.append((path, enum, prefix, read_variants(sources[path], enum, path)))
    except CrateReadError as err:
        print(f"gen_crate_limits: {err}", file=sys.stderr)
        return 1

    text = render(consts, enums)
    if args.out.exists() and args.out.read_text(encoding="utf-8") == text:
        return 0
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
