---
id: 0002
title: book-restructure-tasks-generation
stage: tasks
date: 2025-01-24
surface: claude-code
model: claude-sonnet-4-5
feature: 030-book-restructure-sdd-before-python
branch: 030-book-restructure-sdd-before-python
user: mjs
command: /sp.tasks
labels: [task-breakdown, sequential-execution, reverse-order-operations]
links:
  spec: specs/030-book-restructure-sdd-before-python/spec.md
  plan: specs/030-book-restructure-sdd-before-python/plan.md
  tasks: specs/030-book-restructure-sdd-before-python/tasks.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/030-book-restructure-sdd-before-python/tasks.md
    status: created
    lines: estimated_400+
tests: []
---

## Prompt

User executed `/sp.tasks 030-book-restructure-sdd-before-python` after completing specification and implementation plan.

**Context**:
- Spec complete with 5 user stories (US1-US5) prioritized P1/P2/P3
- Plan complete with ~78-task estimate across 5 phases
- Plan documented reverse-order execution as critical pattern for overlapping numeric ranges
- Plan identified 14 Success Criteria requiring validation

**Task**: Generate comprehensive tasks.md following /sp.tasks workflow:
- Organize by user story priority
- Use strict checklist format for all tasks
- Include pre/post migration validation
- Document reverse-order execution sequences
- Map all 14 Success Criteria to validation tasks

## Response snapshot

Spawned general-purpose agent (sonnet) to generate comprehensive tasks.md file.

**Agent generated 149 tasks across 8 phases**:

**Phase 1: Setup & Pre-Migration Validation** (6 tasks)
- Verify git clean state
- Capture baseline metrics (file counts, directory structure)
- Backup chapter-index.md
- Record pre-migration state for rollback

**Phase 2: Part Directory Renames - US5** (4 tasks)
- Rename 04-Python → 05-Python
- Rename 05-SDD → 04-SDD-RI
- Create 06-AI-Product-Leadership
- Verification checkpoint

**Phase 3: Asset Migration - US4 (CRITICAL)** (44 tasks)
- 21 slide PDF renames in REVERSE ORDER (T011-T028: 30→33, 29→32, ..., 13→16)
- 23 image directory moves in REVERSE ORDER (T032-T054)
- Git mv operations throughout

**Phase 4: Chapter Directory Moves - US2** (22 tasks)
- 18 Python chapters in REVERSE ORDER (T055-T072: 30→33, ..., 13→16)
- 2 SDD chapters (31→13, 32→14)
- 1 AI Orchestra chapter (33→35)
- Verification checkpoint

**Phase 5: Update Metadata - US1** (22 tasks)
- 21 YAML frontmatter updates (sidebar_position + slides.source)
- Replace chapter-index.md with updated 86-chapter structure

**Phase 6: Create Placeholders - US3** (6 tasks)
- Create directories for chapters 15, 34, 36
- Generate placeholder README.md files with proper frontmatter

**Phase 7: Content Reference Updates & Plugin Config** (28 tasks)
- Update image markdown references (3 tasks)
- Update plugin configuration files (4 tasks: docusaurus.config.ts, remark-interactive-python/index.js)
- Update 21 narrative chapter references in REVERSE ORDER (T112-T132: Ch 33→35, 32→14, 31→13, then Python)

**Phase 8: Final Validation** (17 tasks)
- Validate all 14 Success Criteria (SC-001 through SC-014)
- Docusaurus build check
- Git history preservation verification
- Asset count validation
- Manual browser test for interactive code
- Spot-check chapter references

**Critical Requirements Enforced**:
- ✅ ALL 149 tasks use strict checklist format: `- [ ] T### [P?] [USN] Description`
- ✅ Reverse-order execution documented for slides (T011-T028), chapters (T055-T072), references (T112-T132)
- ✅ Git mv operations specified throughout
- ✅ User story tags [US1]-[US5] on all applicable tasks
- ✅ Parallel opportunities marked with [P] (27 parallelizable tasks)
- ✅ All 14 Success Criteria mapped to validation tasks in Phase 8

## Outcome

- ✅ Impact: Complete task breakdown ready for /sp.implement workflow. 149 sequential, atomic tasks with clear dependencies, reverse-order execution patterns, and comprehensive validation.
- 🧪 Tests: No automated tests—infrastructure migration with manual validation checkpoints
- 📁 Files: Created specs/030-book-restructure-sdd-before-python/tasks.md (estimated 400+ lines)
- 🔁 Next prompts: User can execute tasks manually or run `/sp.implement 030-book-restructure-sdd-before-python` for automated execution
- 🧠 Reflection: Reverse-order execution pattern is critical design element—documented consistently across 3 phases (slides, chapters, references) to prevent file overwrites and double-replacement bugs.

## Evaluation notes (flywheel)

- Failure modes observed: None—task generation completed successfully via agent
- Graders run and results (PASS/FAIL): Manual review—all 149 tasks follow checklist format, user stories correctly mapped
- Prompt variant (if applicable): Used general-purpose agent for task generation (complex, multi-phase decomposition)
- Next experiment (smallest change to try): Proceed to /sp.implement or manual task execution
