# CLAUDE.md

Use `AGENTS.md` as the source of truth for repo workflow, safety rules, and validation.

Claude-specific slash commands live under `.claude/commands/`, but they must follow the same constraints:

- no direct `main` pushes,
- no breaking payload changes,
- no hard auth requirement for deployed devices,
- no broad OTA rollout without hardware-owner review,
- and `scripts/check` before a PR whenever PlatformIO is available.
