---
id: "0001"
title: chapter-reorganization-spec
stage: spec
date: 2025-11-26
surface: claude-code
model: claude-opus-4-5-20251101
feature: 031-chapter-reorganization
branch: 031-chapter-reorganization
user: mjs
command: /sp.specify 031-chapter-reorganization
labels: [specification, book-reorganization, chapter-renumbering]
links:
  spec: specs/031-chapter-reorganization/spec.md
  ticket: ""
  adr: ""
  pr: ""
files:
  - specs/031-chapter-reorganization/spec.md
  - specs/031-chapter-reorganization/checklists/requirements.md
tests:
  - validation: spec-architect
---

## Prompt

/sp.loopflow.v2 Reorganize book chapters as a single commit:
1. Delete Part 6 (AI Product Leadership)
2. Renumber Parts 7-13 → 6-12
3. Delete Chapter 15 (Capstone) from Part 4
4. Renumber Part 5 Python chapters 16-33 → 15-32
5. Update all assets (slides, images, README frontmatter)

We have to ensure every chapter title, sidebar title, number, assets and references everywhere are updated using @specs/book/chapter-index.md

Use SDD Loop from spec to plan and tasks before implementation

## Response snapshot

Created specification for book chapter reorganization:
- Analyzed current state (Part 4 with Ch 13-15, Part 5 with Ch 16-33)
- Mapped target state from chapter-index.md (Ch 15 deleted, Python chapters renumbered 15-32)
- Defined 14 functional requirements covering directory operations, frontmatter updates, slide renames
- Established 9 measurable success criteria (all grep-verifiable)
- Identified edge cases (missing slides, cross-references, quiz files)
- Set explicit constraints (single commit, no content changes, preserve git history)

spec-architect validation: READY (9.5/10)
- Testability: 10/10
- Completeness: 10/10
- Ambiguity: 9/10
- Traceability: 9/10

## Outcome

- Impact: Specification complete for chapter reorganization
- Tests: spec-architect validation PASS
- Files: spec.md, checklists/requirements.md created
- Next prompts: /sp.plan 031-chapter-reorganization
- Reflection: Well-scoped infrastructure task with clear before/after states

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): spec-architect PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
