---
id: "0002"
title: chapter-reorganization-plan
stage: plan
date: 2025-11-26
surface: claude-code
model: claude-opus-4-5-20251101
feature: 031-chapter-reorganization
branch: 031-chapter-reorganization
user: mjs
command: /sp.plan 031-chapter-reorganization
labels: [planning, file-reorganization, infrastructure]
links:
  spec: specs/031-chapter-reorganization/spec.md
  ticket: ""
  adr: ""
  pr: ""
files:
  - specs/031-chapter-reorganization/plan.md
tests:
  - validation: constitution-check PASS
---

## Prompt

/sp.plan 031-chapter-reorganization

## Response snapshot

Created implementation plan for book chapter reorganization:
- Defined 9-phase operation sequence (delete, rename dirs, update frontmatter, rename slides, rename images, update image refs, update READMEs, validate, commit)
- Mapped all 18 chapter directory renames (16-33 → 15-32)
- Mapped all 18 slide PDF renames
- Mapped 15 image directory renames (sparse - not all chapters have images)
- Documented ~30 image reference updates in lesson content
- Created validation scripts for all success criteria
- Key insight: Must rename in reverse order (33→32 first) to avoid conflicts

## Outcome

- Impact: Complete implementation plan ready for task generation
- Tests: Constitution check PASS (infrastructure task, minimal overlap)
- Files: plan.md created with detailed mappings
- Next prompts: /sp.tasks 031-chapter-reorganization
- Reflection: Plan template adapted for infrastructure task vs software development

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Constitution check PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
