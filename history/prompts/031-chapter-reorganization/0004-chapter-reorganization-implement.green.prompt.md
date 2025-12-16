---
id: "0004"
title: chapter-reorganization-implement
stage: green
date: 2025-11-26
surface: claude-code
model: claude-opus-4-5-20251101
feature: 031-chapter-reorganization
branch: 031-chapter-reorganization
user: mjs
command: (continue from tasks)
labels: [implementation, file-reorganization, infrastructure]
links:
  spec: specs/031-chapter-reorganization/spec.md
  ticket: ""
  adr: ""
  pr: ""
files:
  - apps/learn-app/docs/04-SDD-RI-Fundamentals/ (1 chapter deleted)
  - apps/learn-app/docs/05-Python-Fundamentals/ (18 chapters renamed)
  - book-source/static/slides/ (18 PDFs renamed)
  - book-source/static/img/part-5/ (15 directories renamed)
tests:
  - validation: SC-003 PASS (18 directories)
  - validation: SC-004 PASS (18 slides)
  - validation: SC-005 PASS (0 old chapter refs)
  - validation: SC-006 PASS (0 old slide refs)
  - validation: SC-007 PASS (2 Part 4 chapters)
  - validation: SC-010 PASS (15 image dirs)
  - validation: SC-011 PASS (0 old image refs)
---

## Prompt

yes (continue to implement chapter reorganization from tasks.md)

## Response snapshot

Implemented complete book chapter reorganization:

**Phase 2**: Deleted Chapter 15 (AI Product & Business Intelligence Capstone) from Part 4

- Removed 9 files using git rm -rf

**Phase 3**: Renamed Part 5 directories (18 operations)

- Used reverse-order renaming (33→32 first) to avoid conflicts
- All directories now numbered 15-32

**Phase 4**: Updated README frontmatter (18 files)

- Fixed sidebar_position, title, slides.source, slides.title
- Initial replacement cascade caused all to become "Chapter 15"
- Fixed by iterating directories and setting correct number per directory

**Phase 5**: Renamed slide PDFs (18 operations)

- Used temp file strategy to avoid git mv conflicts
- All slides now numbered chapter-15 through chapter-32

**Phase 6**: Renamed image directories (15 operations)

- Sparse - only 15 of 18 chapters have image directories
- Used same temp file strategy

**Phase 7**: Updated image references (~20 files)

- Find-replace /img/part-5/chapter-N/ paths in lesson content

**Phase 8**: Updated Part READMEs

- Part 4: Removed Chapter 15 references, updated to "two chapters (13-14)"
- Part 5: Updated chapter range to "15-32"

**Total**: 309 files changed, single atomic commit created

## Outcome

- Impact: Complete chapter reorganization matching chapter-index.md
- Tests: All 7 validation checks PASS
- Files: 309 files changed (1,183 insertions, 4,064 deletions)
- Next prompts: Run Docusaurus build to verify site builds correctly
- Reflection: Reverse-order renaming critical for avoiding conflicts; temp file strategy needed for git mv with overlapping names

## Evaluation notes (flywheel)

- Failure modes observed: Replacement cascade (16→15, then 17→16→15, etc.) - fixed by directory-based iteration
- Graders run and results (PASS/FAIL): All SC checks PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
