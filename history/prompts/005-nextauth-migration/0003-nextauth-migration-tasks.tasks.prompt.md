---
id: 005-003
title: NextAuth to Better Auth Migration Tasks
stage: tasks
date: 2024-12-05
surface: claude-code
model: claude-opus-4-5
feature: 005-nextauth-migration
branch: 005-nextauth-migration
user: mjs
command: /sp.tasks
labels: [migration, tasks, batch-processing, conflict-resolution]
links:
  spec: specs/005-nextauth-migration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/005-nextauth-migration/tasks.md
    action: created
    description: Complete task breakdown with 59 tasks organized by user story
tests:
  - name: task-dependency-validation
    status: passed
    description: All task dependencies correctly ordered
---

## Prompt

Generate actionable tasks from plan.md for NextAuth to Better Auth migration. Organize by user story with proper dependencies and MVP scope identification.

## Response snapshot

### Task Breakdown Created

Created `specs/005-nextauth-migration/tasks.md` with 59 tasks organized into 9 phases:

**Phase 1: Setup (5 tasks)**
- T001-T005: Script scaffold, TypeScript interfaces, CLI parser, config loader, DB connections

**Phase 2: Foundational (5 tasks)**
- T006-T010: Logger utility, source query, target lookup, transaction wrapper, banner output
- CRITICAL: Blocks all user story implementation

**Phase 3: User Story 1 - Dry Run (5 tasks)**
- T011-T015: Dry-run flag handling, logging for INSERT/MERGE operations, summary output

**Phase 4: User Story 2 - Batch Processing (6 tasks)**
- T016-T021: Batch fetching, transaction-per-batch, rollback, progress tracking, timing

**Phase 5: User Story 5 - Conflict Resolution (13 tasks)**
- T022-T034: Merge function, FK updates for 8 tables, user.id update, fillNullFields, credential upsert

**Phase 6: User Story 3 - Country Normalization (5 tasks)**
- T035-T039: Country map, normalizeCountry function, warning logging, integration

**Phase 7: User Story 4 - Username Recreation (6 tasks)**
- T040-T045: Email prefix extraction, random suffix, collision detection, integration

**Phase 8: User Story 6 - Resume Capability (4 tasks)**
- T046-T049: Offset flag handling, progress adjustment

**Phase 9: Polish (10 tasks)**
- T050-T059: INSERT function, hash validation, error handling, summary, verification queries

### MVP Scope

MVP = 34 tasks (T001-T034): Setup + Foundational + P1 stories (Dry Run, Batch, Conflict)

## Outcome

- ✅ Impact: High - Complete task breakdown for 14,821 user migration with clear dependencies
- 🧪 Tests: 1/1 passed (dependency validation)
- 📁 Files: 1 created (tasks.md)
- 🔁 Next prompts: /sp.implement 005-nextauth-migration
- 🧠 Reflection: Single-file implementation simplifies coordination but limits parallelism

## Evaluation notes (flywheel)

- Failure modes observed: None - task structure follows plan architecture
- Graders run and results (PASS/FAIL): Dependency validation PASS
- Prompt variant (if applicable): SDD-RI orchestrated workflow
- Next experiment (smallest change to try): Proceed to implementation phase
