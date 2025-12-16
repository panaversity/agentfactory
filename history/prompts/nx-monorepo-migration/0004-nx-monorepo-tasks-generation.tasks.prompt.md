---
id: "0004"
title: "Nx Monorepo Task Generation"
stage: tasks
feature: nx-monorepo-migration
date: "2025-12-15"
tags: [monorepo, nx, tasks, migration, planning]
---

# Nx Monorepo Task Generation

## Prompt

perfect now run /sp.tasks to plan the tasks breakdown and make it possible. THis required deep thinking to carefully plan and ensure nothing breakings especailly book

## Response

Generated comprehensive tasks.md with **95 tasks** organized by migration phase:

**Task Distribution by Phase**:
- Phase 0: Nx Configuration (T001-T018) - 18 tasks
- Phase 1: Directory Structure Migration (T019-T050) - 32 tasks
- Phase 2: CI/CD Migration (T051-T063) - 13 tasks
- Phase 3: Production Cutover (T064-T079) - 16 tasks
- Phase 4: Optimization (T080-T095) - 16 tasks

**User Story Mapping**:
| Story | Priority | Description | Validates In |
|-------|----------|-------------|--------------|
| US1 | P1 | Developer Setup | Phase 0-1 |
| US2 | P1 | Book Writer Workflow | Phase 1 |
| US3 | P2 | CI Affected Detection | Phase 2 |
| US4 | P2 | Project Commands | Phase 1-2 |
| US5 | P3 | Rollback Capability | Phase 3 |

**Critical Book Writer Protection Tasks**:
- T033: Create `book-source → apps/learn-app` symlink
- T034: Verify symlink resolution for all 61+ path references
- T045: **CRITICAL TEST** - Manual book writer workflow validation
- T046: Test context-gathering protocol from CLAUDE.md
- T071: Final book writer workflow validation before symlink removal
- T090: Document book writer path conventions

**Rollback Checkpoints** (safety gates at each phase):
- T018: Phase 0 checkpoint before any file moves
- T049: Phase 1 checkpoint after structure complete
- T063: Phase 2 checkpoint after CI migration
- T079: Phase 3 checkpoint after production cutover

**Key Technical Decisions**:
1. `nx:run-commands` executor with `cwd` option (not custom executor)
2. Symlink strategy for backward compatibility during transition
3. Parallel workflow validation before removing old pipelines
4. Plugin extraction to `libs/docusaurus/` namespace
5. Python integration via Makefile wrapper preserving existing `uv` tooling

**Implementation Strategy**: MVP-first approach
1. Complete Phase 0 + minimal Phase 1 (learn-app only)
2. Validate book writer workflows work
3. Add plugins as libs one-by-one
4. Then CI/CD migration
5. Production cutover only after validation

## Artifacts

- `specs/034-nx-monorepo-migration/tasks.md` (664 lines, 95 tasks)

## Next Step

Begin implementation with Phase 0, Task T001: `pnpm add -D nx@latest @nx/js @nx/workspace`
