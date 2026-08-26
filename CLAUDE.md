# embarch-dev-bench

## Docs

Design doc: [../embarch-doc/embarch-dev-bench/design.md](../embarch-doc/embarch-dev-bench/design.md) — source of truth for this project's architecture/design.
Update it proactively per [../embarch-doc/DOC-PROTOCOL.md](../embarch-doc/DOC-PROTOCOL.md) whenever a notable design decision, feature, or status change happens here.

## Building

**Build and flash this repo's firmware autonomously — no need to ask (2026-08-25).** This is EmbArch's own test rig, not a DUT: a bad build here breaks the bench, not a customer's board, and the whole point of the suite is that bench work is automatable. Share the output rather than assuming success.

The ask-first rule this replaces was written for **DUT** firmware and had been over-applied to dev-bench itself. It still holds for a DUT: never build or flash a device under test without asking.

`west` is not on bare `PATH` here and none of `workspaces/*` carries its own `.venv` — pass an absolute path to a `west` that works, the same way `embarch-api`'s config declares `west_binary` for every project.

## Git

**Work directly on `main` — no feature branches, no PRs (2026-08-25).** Commit and push straight to `main` once the change builds and its tests and `clippy --all-targets -- -D warnings` are clean. This **overrides** the general "if you're on the default branch, branch first" default, for this suite only. It ends when the repo owner explicitly says it does, and on no other condition — not on an agent's read of whether the project has outgrown it. Reasoning, the sequencing rules that keep it safe, and the one case that still warrants a branch: [../embarch-doc/embarch-dev-workflow.md](../embarch-doc/embarch-dev-workflow.md) §6.
