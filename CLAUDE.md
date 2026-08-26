# embarch-dev-bench

## Docs

Design doc: [../embarch-doc/embarch-dev-bench/design.md](../embarch-doc/embarch-dev-bench/design.md) — source of truth for this project's architecture/design.
Update it proactively per [../embarch-doc/DOC-PROTOCOL.md](../embarch-doc/DOC-PROTOCOL.md) whenever a notable design decision, feature, or status change happens here.

## Building

Never build firmware or flash devices autonomously — always ask before running `west build`/`west flash`, and share the output for review rather than assuming success.

## Git

**Work directly on `main` — no feature branches, no PRs (2026-08-25).** Commit and push straight to `main` once the change builds and its tests and `clippy --all-targets -- -D warnings` are clean. This **overrides** the general "if you're on the default branch, branch first" default, for this suite only. It ends when the repo owner explicitly says it does, and on no other condition — not on an agent's read of whether the project has outgrown it. Reasoning, the sequencing rules that keep it safe, and the one case that still warrants a branch: [../embarch-doc/embarch-dev-workflow.md](../embarch-doc/embarch-dev-workflow.md) §6.
