# MioLink Agent Rules

## Scope

These shared rules apply to the whole repository and are intended for Codex, Claude Code, Cursor, and other coding agents. Tool-specific wrappers should import or reference this file instead of duplicating the rules.

MioLink contains firmware, hardware, documentation, and test-board assets. Do not apply firmware assumptions to hardware or documentation work unless a task explicitly crosses those boundaries.

For files under `firmware/`, also read and follow `firmware/AGENTS.md`.

## Repository Map

- `firmware/`: RP2040 firmware for the MioLink Black Magic Probe port.
- `hardware/`: KiCad and generated hardware artifacts.
- `docs/`: project documentation.
- `test_board/`: local test-board assets, ignored by git.
- `build/`, `firmware/build/`, `firmware/cmake-build-*`: generated build trees.

## Working Tree Rules

- Start substantial work by checking `git status --short`.
- The worktree may contain user changes. Never revert, rewrite, or clean changes you did not make unless explicitly asked.
- If unrelated files are dirty, leave them alone.
- If dirty files overlap with the requested task, inspect them and work with the current contents.
- Do not run destructive commands such as `git reset --hard`, `git checkout --`, or broad `rm` commands unless the user explicitly requests them.
- Do not create commits, branches, tags, or pull requests unless asked.

## Search And Editing

- Prefer `rg` and `rg --files` for text/file search.
- Keep edits scoped to the requested behavior and the local architecture.
- Use existing project patterns before introducing new abstractions.
- Do not reformat unrelated code or churn generated metadata.
- Preserve existing license headers and attribution.
- Use ASCII for new text unless the existing file or requested content needs non-ASCII.

## Generated And Vendored Content

- Do not edit generated build directories.
- Do not edit vendored dependencies unless the task is explicitly about dependency integration or a vendored patch.
- Do not update submodules, fetch dependencies, or regenerate large vendor outputs unless asked.
- Do not edit IDE caches/settings (`.idea/`, `.vscode/`, `.cache/`) unless the task is specifically about those files.

## Validation

- Run the smallest relevant checks for the change.
- For documentation-only rule changes, no firmware build is required unless the user asks for one.
- For code changes, report what was run and what could not be verified.
- If a change affects hardware behavior but hardware is unavailable, state the exact unverified hardware path.

## Cross-Tool Layout

- `AGENTS.md` is the canonical shared rule file.
- `firmware/AGENTS.md` contains firmware-scoped rules.
- `CLAUDE.md` is a thin Claude Code adapter that imports shared rules.
- `.cursor/rules/*.mdc` files are Cursor adapters that reference the same shared rules.
