# McGill AeroHacks 2026

## Git hooks

This repo keeps its commit hooks in `.githooks/`.

Enable them once per clone:

```bash
git config core.hooksPath .githooks
```

The `pre-commit` hook runs `uv run ruff format` in both `drone-hover/` and `rtm/`, then re-stages any files that were already part of the commit.