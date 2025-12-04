---
id: "0003"
title: chapter-reorganization-tasks
stage: tasks
date: 2025-11-26
surface: claude-code
model: claude-opus-4-5-20251101
feature: 031-chapter-reorganization
branch: 031-chapter-reorganization
user: mjs
command: /sp.tasks 031-chapter-reorganization
labels: [tasks, file-reorganization, infrastructure]
links:
  spec: specs/031-chapter-reorganization/spec.md
  ticket: ""
  adr: ""
  pr: ""
files:
  - specs/031-chapter-reorganization/tasks.md
tests:
  - validation: format-check PASS
---

## Prompt

/sp.tasks 031-chapter-reorganization

## Response snapshot

Generated comprehensive task list for book chapter reorganization:
- 104 total tasks across 10 phases
- 43 tasks parallelizable (marked [P])
- Phase 1: Pre-validation (4 tasks)
- Phase 2: Delete Chapter 15 (1 task)
- Phase 3: Rename directories in reverse order (18 tasks - sequential)
- Phase 4: Update README frontmatter (18 tasks - parallelizable)
- Phase 5: Rename slide PDFs in reverse order (18 tasks - sequential)
- Phase 6: Rename image directories (18 tasks, 15 actual - sequential)
- Phase 7: Update image references (15 tasks - parallelizable)
- Phase 8: Update Part READMEs (2 tasks)
- Phase 9: Validation scripts (8 tasks)
- Phase 10: Atomic commit (2 tasks)

Key design decisions:
- Reverse-order renaming (33→32 first) to prevent conflicts
- Skip tasks for non-existent image directories (sparse)
- All validation scripts map to success criteria from spec

## Outcome

- Impact: Complete executable task list ready for implementation
- Tests: Format validation PASS (all tasks follow [ID] [P?] format with file paths)
- Files: tasks.md created with 104 tasks
- Next prompts: Execute tasks (implementation phase)
- Reflection: Infrastructure task - user stories not applicable, organized by operation phase instead

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Format check PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
