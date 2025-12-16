# Tasks: Book Chapter Reorganization

**Input**: Design documents from `/specs/031-chapter-reorganization/`
**Prerequisites**: plan.md (required), spec.md (required)

**Tests**: Not requested - validation scripts provided in plan.md
**Organization**: Tasks grouped by operation phase (infrastructure task, not user-story based)

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions

## Path Conventions

- **Book source**: `apps/learn-app/docs/` for chapter directories
- **Slides**: `book-source/static/slides/` for PDF files
- **Images**: `book-source/static/img/part-5/` for image directories

---

## Phase 1: Pre-Implementation Validation

**Purpose**: Verify current state matches expectations before making changes

- [ ] T001 Verify Part 4 contains chapters 13, 14, 15 in `apps/learn-app/docs/04-SDD-RI-Fundamentals/`
- [ ] T002 [P] Verify Part 5 contains 18 chapter directories (16-33) in `apps/learn-app/docs/05-Python-Fundamentals/`
- [ ] T003 [P] Verify slide PDFs exist for chapters 16-33 in `book-source/static/slides/`
- [ ] T004 [P] Verify image directories exist in `book-source/static/img/part-5/` (15 dirs: chapter-16 through chapter-33, sparse)

**Checkpoint**: Current state verified - safe to proceed with reorganization

---

## Phase 2: Delete Chapter 15 (Part 4)

**Purpose**: Remove the AI Product & Business Intelligence Capstone chapter (FR-001)

- [ ] T005 Delete directory `apps/learn-app/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/` using git rm -rf

**Checkpoint**: Part 4 now contains only chapters 13 and 14 (SC-007)

---

## Phase 3: Rename Part 5 Chapter Directories

**Purpose**: Renumber chapter directories 16-33 → 15-32 (FR-002)

**⚠️ CRITICAL**: Must rename in REVERSE order (33→32 first) to avoid conflicts

- [ ] T006 Rename `apps/learn-app/docs/05-Python-Fundamentals/33-cpython-gil/` → `32-cpython-gil/` using git mv
- [ ] T007 Rename `apps/learn-app/docs/05-Python-Fundamentals/32-asyncio/` → `31-asyncio/` using git mv
- [ ] T008 Rename `apps/learn-app/docs/05-Python-Fundamentals/31-pydantic-generics/` → `30-pydantic-generics/` using git mv
- [ ] T009 Rename `apps/learn-app/docs/05-Python-Fundamentals/30-metaclasses-dataclasses/` → `29-metaclasses-dataclasses/` using git mv
- [ ] T010 Rename `apps/learn-app/docs/05-Python-Fundamentals/29-oop-part-2/` → `28-oop-part-2/` using git mv
- [ ] T011 Rename `apps/learn-app/docs/05-Python-Fundamentals/28-oop-part-1/` → `27-oop-part-1/` using git mv
- [ ] T012 Rename `apps/learn-app/docs/05-Python-Fundamentals/27-math-datetime-calendar/` → `26-math-datetime-calendar/` using git mv
- [ ] T013 Rename `apps/learn-app/docs/05-Python-Fundamentals/26-io-file-handling/` → `25-io-file-handling/` using git mv
- [ ] T014 Rename `apps/learn-app/docs/05-Python-Fundamentals/25-exception-handling/` → `24-exception-handling/` using git mv
- [ ] T015 Rename `apps/learn-app/docs/05-Python-Fundamentals/24-module-functions/` → `23-module-functions/` using git mv
- [ ] T016 Rename `apps/learn-app/docs/05-Python-Fundamentals/23-set-frozenset-gc/` → `22-set-frozenset-gc/` using git mv
- [ ] T017 Rename `apps/learn-app/docs/05-Python-Fundamentals/22-lists-tuples-dictionary/` → `21-lists-tuples-dictionary/` using git mv
- [ ] T018 Rename `apps/learn-app/docs/05-Python-Fundamentals/21-control-flow-loops/` → `20-control-flow-loops/` using git mv
- [ ] T019 Rename `apps/learn-app/docs/05-Python-Fundamentals/20-strings-type-casting/` → `19-strings-type-casting/` using git mv
- [ ] T020 Rename `apps/learn-app/docs/05-Python-Fundamentals/19-operators-keywords-variables/` → `18-operators-keywords-variables/` using git mv
- [ ] T021 Rename `apps/learn-app/docs/05-Python-Fundamentals/18-data-types/` → `17-data-types/` using git mv
- [ ] T022 Rename `apps/learn-app/docs/05-Python-Fundamentals/17-introduction-to-python/` → `16-introduction-to-python/` using git mv
- [ ] T023 Rename `apps/learn-app/docs/05-Python-Fundamentals/16-python-uv-package-manager/` → `15-python-uv-package-manager/` using git mv

**Checkpoint**: Part 5 directories now numbered 15-32 (SC-003)

---

## Phase 4: Update README Frontmatter (18 files)

**Purpose**: Update sidebar_position, title, and slides references (FR-004, FR-005, FR-006, FR-007)

**Note**: Tasks are parallelizable - each edits a different file

- [ ] T024 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/15-python-uv-package-manager/README.md`: sidebar_position 16→15, "Chapter 16:" → "Chapter 15:", chapter-16-slides → chapter-15-slides
- [ ] T025 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/16-introduction-to-python/README.md`: sidebar_position 17→16, "Chapter 17:" → "Chapter 16:", chapter-17-slides → chapter-16-slides
- [ ] T026 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/17-data-types/README.md`: sidebar_position 18→17, "Chapter 18:" → "Chapter 17:", chapter-18-slides → chapter-17-slides
- [ ] T027 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/18-operators-keywords-variables/README.md`: sidebar_position 19→18, "Chapter 19:" → "Chapter 18:", chapter-19-slides → chapter-18-slides
- [ ] T028 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/19-strings-type-casting/README.md`: sidebar_position 20→19, "Chapter 20:" → "Chapter 19:", chapter-20-slides → chapter-19-slides
- [ ] T029 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/20-control-flow-loops/README.md`: sidebar_position 21→20, "Chapter 21:" → "Chapter 20:", chapter-21-slides → chapter-20-slides
- [ ] T030 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/21-lists-tuples-dictionary/README.md`: sidebar_position 22→21, "Chapter 22:" → "Chapter 21:", chapter-22-slides → chapter-21-slides
- [ ] T031 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/22-set-frozenset-gc/README.md`: sidebar_position 23→22, "Chapter 23:" → "Chapter 22:", chapter-23-slides → chapter-22-slides
- [ ] T032 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/23-module-functions/README.md`: sidebar_position 24→23, "Chapter 24:" → "Chapter 23:", chapter-24-slides → chapter-23-slides
- [ ] T033 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/24-exception-handling/README.md`: sidebar_position 25→24, "Chapter 25:" → "Chapter 24:", chapter-25-slides → chapter-24-slides
- [ ] T034 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/25-io-file-handling/README.md`: sidebar_position 26→25, "Chapter 26:" → "Chapter 25:", chapter-26-slides → chapter-25-slides
- [ ] T035 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/26-math-datetime-calendar/README.md`: sidebar_position 27→26, "Chapter 27:" → "Chapter 26:", chapter-27-slides → chapter-26-slides
- [ ] T036 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/27-oop-part-1/README.md`: sidebar_position 28→27, "Chapter 28:" → "Chapter 27:", chapter-28-slides → chapter-27-slides
- [ ] T037 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/28-oop-part-2/README.md`: sidebar_position 29→28, "Chapter 29:" → "Chapter 28:", chapter-29-slides → chapter-28-slides
- [ ] T038 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/29-metaclasses-dataclasses/README.md`: sidebar_position 30→29, "Chapter 30:" → "Chapter 29:", chapter-30-slides → chapter-29-slides
- [ ] T039 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/30-pydantic-generics/README.md`: sidebar_position 31→30, "Chapter 31:" → "Chapter 30:", chapter-31-slides → chapter-30-slides
- [ ] T040 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/31-asyncio/README.md`: sidebar_position 32→31, "Chapter 32:" → "Chapter 31:", chapter-32-slides → chapter-31-slides
- [ ] T041 [P] Update frontmatter in `apps/learn-app/docs/05-Python-Fundamentals/32-cpython-gil/README.md`: sidebar_position 33→32, "Chapter 33:" → "Chapter 32:", chapter-33-slides → chapter-32-slides

**Checkpoint**: All frontmatter updated (SC-005, SC-006)

---

## Phase 5: Rename Slide PDFs (18 files)

**Purpose**: Renumber slide PDFs 16-33 → 15-32 (FR-008)

**⚠️ CRITICAL**: Must rename in REVERSE order (33→32 first) to avoid conflicts

- [ ] T042 Rename `book-source/static/slides/chapter-33-slides.pdf` → `chapter-32-slides.pdf` using git mv
- [ ] T043 Rename `book-source/static/slides/chapter-32-slides.pdf` → `chapter-31-slides.pdf` using git mv
- [ ] T044 Rename `book-source/static/slides/chapter-31-slides.pdf` → `chapter-30-slides.pdf` using git mv
- [ ] T045 Rename `book-source/static/slides/chapter-30-slides.pdf` → `chapter-29-slides.pdf` using git mv
- [ ] T046 Rename `book-source/static/slides/chapter-29-slides.pdf` → `chapter-28-slides.pdf` using git mv
- [ ] T047 Rename `book-source/static/slides/chapter-28-slides.pdf` → `chapter-27-slides.pdf` using git mv
- [ ] T048 Rename `book-source/static/slides/chapter-27-slides.pdf` → `chapter-26-slides.pdf` using git mv
- [ ] T049 Rename `book-source/static/slides/chapter-26-slides.pdf` → `chapter-25-slides.pdf` using git mv
- [ ] T050 Rename `book-source/static/slides/chapter-25-slides.pdf` → `chapter-24-slides.pdf` using git mv
- [ ] T051 Rename `book-source/static/slides/chapter-24-slides.pdf` → `chapter-23-slides.pdf` using git mv
- [ ] T052 Rename `book-source/static/slides/chapter-23-slides.pdf` → `chapter-22-slides.pdf` using git mv
- [ ] T053 Rename `book-source/static/slides/chapter-22-slides.pdf` → `chapter-21-slides.pdf` using git mv
- [ ] T054 Rename `book-source/static/slides/chapter-21-slides.pdf` → `chapter-20-slides.pdf` using git mv
- [ ] T055 Rename `book-source/static/slides/chapter-20-slides.pdf` → `chapter-19-slides.pdf` using git mv
- [ ] T056 Rename `book-source/static/slides/chapter-19-slides.pdf` → `chapter-18-slides.pdf` using git mv
- [ ] T057 Rename `book-source/static/slides/chapter-18-slides.pdf` → `chapter-17-slides.pdf` using git mv
- [ ] T058 Rename `book-source/static/slides/chapter-17-slides.pdf` → `chapter-16-slides.pdf` using git mv
- [ ] T059 Rename `book-source/static/slides/chapter-16-slides.pdf` → `chapter-15-slides.pdf` using git mv

**Checkpoint**: All slides renamed (SC-004)

---

## Phase 6: Rename Image Directories (15 dirs - sparse)

**Purpose**: Renumber image directories for chapters that have them (FR-015)

**⚠️ CRITICAL**: Must rename in REVERSE order to avoid conflicts. Only rename directories that exist.

- [ ] T060 Rename `book-source/static/img/part-5/chapter-33/` → `chapter-32/` using git mv
- [ ] T061 Rename `book-source/static/img/part-5/chapter-32/` → `chapter-31/` using git mv
- [ ] T062 Rename `book-source/static/img/part-5/chapter-31/` → `chapter-30/` using git mv
- [ ] T063 Rename `book-source/static/img/part-5/chapter-30/` → `chapter-29/` using git mv
- [ ] T064 Rename `book-source/static/img/part-5/chapter-29/` → `chapter-28/` using git mv
- [ ] T065 Rename `book-source/static/img/part-5/chapter-28/` → `chapter-27/` using git mv
- [ ] T066 Rename `book-source/static/img/part-5/chapter-27/` → `chapter-26/` using git mv
- [ ] T067 Rename `book-source/static/img/part-5/chapter-26/` → `chapter-25/` using git mv
- [ ] T068 Skip - `book-source/static/img/part-5/chapter-25/` does not exist (chapter 25 has no images)
- [ ] T069 Rename `book-source/static/img/part-5/chapter-24/` → `chapter-23/` using git mv
- [ ] T070 Rename `book-source/static/img/part-5/chapter-23/` → `chapter-22/` using git mv
- [ ] T071 Rename `book-source/static/img/part-5/chapter-22/` → `chapter-21/` using git mv
- [ ] T072 Rename `book-source/static/img/part-5/chapter-21/` → `chapter-20/` using git mv
- [ ] T073 Skip - `book-source/static/img/part-5/chapter-20/` does not exist (chapter 20 has no images)
- [ ] T074 Rename `book-source/static/img/part-5/chapter-19/` → `chapter-18/` using git mv
- [ ] T075 Rename `book-source/static/img/part-5/chapter-18/` → `chapter-17/` using git mv
- [ ] T076 Skip - `book-source/static/img/part-5/chapter-17/` does not exist (chapter 17 has no images)
- [ ] T077 Rename `book-source/static/img/part-5/chapter-16/` → `chapter-15/` using git mv

**Checkpoint**: All image directories renamed (SC-010)

---

## Phase 7: Update Image References in Lesson Content

**Purpose**: Find-replace image paths in all Part 5 lesson markdown files (FR-016)

**Note**: Must process from highest to lowest chapter number to avoid double-replacement

- [ ] T078 Update all `/img/part-5/chapter-33/` → `/img/part-5/chapter-32/` in `apps/learn-app/docs/05-Python-Fundamentals/32-cpython-gil/**/*.md`
- [ ] T079 [P] Update all `/img/part-5/chapter-32/` → `/img/part-5/chapter-31/` in `apps/learn-app/docs/05-Python-Fundamentals/31-asyncio/**/*.md`
- [ ] T080 [P] Update all `/img/part-5/chapter-31/` → `/img/part-5/chapter-30/` in `apps/learn-app/docs/05-Python-Fundamentals/30-pydantic-generics/**/*.md`
- [ ] T081 [P] Update all `/img/part-5/chapter-30/` → `/img/part-5/chapter-29/` in `apps/learn-app/docs/05-Python-Fundamentals/29-metaclasses-dataclasses/**/*.md`
- [ ] T082 [P] Update all `/img/part-5/chapter-29/` → `/img/part-5/chapter-28/` in `apps/learn-app/docs/05-Python-Fundamentals/28-oop-part-2/**/*.md`
- [ ] T083 [P] Update all `/img/part-5/chapter-28/` → `/img/part-5/chapter-27/` in `apps/learn-app/docs/05-Python-Fundamentals/27-oop-part-1/**/*.md`
- [ ] T084 [P] Update all `/img/part-5/chapter-27/` → `/img/part-5/chapter-26/` in `apps/learn-app/docs/05-Python-Fundamentals/26-math-datetime-calendar/**/*.md`
- [ ] T085 [P] Update all `/img/part-5/chapter-26/` → `/img/part-5/chapter-25/` in `apps/learn-app/docs/05-Python-Fundamentals/25-io-file-handling/**/*.md`
- [ ] T086 [P] Update all `/img/part-5/chapter-24/` → `/img/part-5/chapter-23/` in `apps/learn-app/docs/05-Python-Fundamentals/23-module-functions/**/*.md`
- [ ] T087 [P] Update all `/img/part-5/chapter-23/` → `/img/part-5/chapter-22/` in `apps/learn-app/docs/05-Python-Fundamentals/22-set-frozenset-gc/**/*.md`
- [ ] T088 [P] Update all `/img/part-5/chapter-22/` → `/img/part-5/chapter-21/` in `apps/learn-app/docs/05-Python-Fundamentals/21-lists-tuples-dictionary/**/*.md`
- [ ] T089 [P] Update all `/img/part-5/chapter-21/` → `/img/part-5/chapter-20/` in `apps/learn-app/docs/05-Python-Fundamentals/20-control-flow-loops/**/*.md`
- [ ] T090 [P] Update all `/img/part-5/chapter-19/` → `/img/part-5/chapter-18/` in `apps/learn-app/docs/05-Python-Fundamentals/18-operators-keywords-variables/**/*.md`
- [ ] T091 [P] Update all `/img/part-5/chapter-18/` → `/img/part-5/chapter-17/` in `apps/learn-app/docs/05-Python-Fundamentals/17-data-types/**/*.md`
- [ ] T092 [P] Update all `/img/part-5/chapter-16/` → `/img/part-5/chapter-15/` in `apps/learn-app/docs/05-Python-Fundamentals/15-python-uv-package-manager/**/*.md`

**Checkpoint**: All image references updated (SC-011)

---

## Phase 8: Update Part READMEs

**Purpose**: Update Part 4 and Part 5 READMEs if they contain chapter number references (FR-010, FR-011)

- [ ] T093 Update `apps/learn-app/docs/04-SDD-RI-Fundamentals/README.md`: Remove references to Chapter 15 (now only Chapters 13-14)
- [ ] T094 [P] Update `apps/learn-app/docs/05-Python-Fundamentals/README.md`: Update chapter range to 15-32 if mentioned

**Checkpoint**: Part READMEs updated

---

## Phase 9: Validation

**Purpose**: Verify all changes meet success criteria (FR-012, FR-013)

- [ ] T095 Validate SC-003: Run `ls apps/learn-app/docs/05-Python-Fundamentals/ | grep "^1[5-9]-\|^2[0-9]-\|^3[0-2]-" | wc -l` — expect 18
- [ ] T096 [P] Validate SC-004: Check all 18 slide PDFs exist (chapter-15 through chapter-32)
- [ ] T097 [P] Validate SC-005: Run `grep -r "Chapter 1[6-9]\|Chapter 2[0-9]\|Chapter 3[0-3]" apps/learn-app/docs/05-Python-Fundamentals/*/README.md` — expect 0 matches
- [ ] T098 [P] Validate SC-006: Run `grep -r "chapter-1[6-9]-slides\|chapter-2[0-9]-slides\|chapter-3[0-3]-slides" apps/learn-app/docs/05-Python-Fundamentals/` — expect 0 matches
- [ ] T099 [P] Validate SC-007: Run `ls apps/learn-app/docs/04-SDD-RI-Fundamentals/ | grep "^1[34]-" | wc -l` — expect 2
- [ ] T100 [P] Validate SC-010: Run `ls book-source/static/img/part-5/ | grep "chapter-" | wc -l` — expect 15
- [ ] T101 [P] Validate SC-011: Run `grep -r "/img/part-5/chapter-1[6-9]\|/img/part-5/chapter-2[0-9]\|/img/part-5/chapter-3[0-3]" apps/learn-app/docs/05-Python-Fundamentals/` — expect 0 matches
- [ ] T102 Validate SC-001: Run `cd book-source && npm run build` — expect success

**Checkpoint**: All success criteria validated

---

## Phase 10: Commit

**Purpose**: Create single atomic git commit (FR-014, SC-008)

- [ ] T103 Stage all changes with `git add -A`
- [ ] T104 Create atomic commit: "refactor(book): Reorganize chapters - delete Ch15, renumber Part 5 (16-33→15-32)"

**Checkpoint**: All changes committed (SC-008)

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Validation) ─────────────────────────────────────┐
                                                          │
Phase 2 (Delete Ch15) ────────────────────────────────────┤
                                                          │
Phase 3 (Rename Dirs) ─── MUST be sequential (33→32→...→16→15) ──┤
                                                          │
Phase 4 (Update Frontmatter) ─── All [P] parallelizable ──┤
                                                          │
Phase 5 (Rename Slides) ─── MUST be sequential (33→32→...→16→15) ─┤
                                                          │
Phase 6 (Rename Image Dirs) ─── MUST be sequential ───────┤
                                                          │
Phase 7 (Update Image Refs) ─── All [P] parallelizable ───┤
                                                          │
Phase 8 (Update Part READMEs) ─── Both [P] parallelizable ┤
                                                          │
Phase 9 (Validation) ─────────────────────────────────────┤
                                                          │
Phase 10 (Commit) ────────────────────────────────────────┘
```

### Critical Ordering Rules

1. **Phase 3 (Rename Dirs)**: T006-T023 MUST execute in reverse order (33→32 first) to avoid name conflicts
2. **Phase 5 (Rename Slides)**: T042-T059 MUST execute in reverse order
3. **Phase 6 (Rename Image Dirs)**: T060-T077 MUST execute in reverse order, skip non-existent dirs
4. **Phase 4 & 7**: Can parallelize all tasks since each edits different files

### Parallel Opportunities

**Phase 1**: T002, T003, T004 can run in parallel
**Phase 4**: ALL tasks (T024-T041) can run in parallel — 18 concurrent file edits
**Phase 7**: ALL tasks (T078-T092) can run in parallel — ~15 concurrent file edits
**Phase 8**: T093, T094 can run in parallel
**Phase 9**: T096-T101 can run in parallel

---

## Implementation Strategy

### MVP First (Minimum Viable Change)

1. Complete Phase 1: Pre-validation
2. Complete Phase 2: Delete Chapter 15
3. Complete Phase 3: Rename directories (core operation)
4. Complete Phase 4: Update frontmatter
5. **STOP and VALIDATE**: Run T95, T97 to check directory and frontmatter status
6. Continue with slides, images, and final validation

### Rollback Strategy

If any phase fails:

1. Run `git checkout .` to restore all files
2. Or revert the commit if already committed

### Total Task Count

| Phase                       | Task Count     | Parallelizable |
| --------------------------- | -------------- | -------------- |
| Phase 1: Pre-validation     | 4              | 3              |
| Phase 2: Delete Ch15        | 1              | 0              |
| Phase 3: Rename Dirs        | 18             | 0              |
| Phase 4: Update Frontmatter | 18             | 18             |
| Phase 5: Rename Slides      | 18             | 0              |
| Phase 6: Rename Image Dirs  | 18 (15 actual) | 0              |
| Phase 7: Update Image Refs  | 15             | 14             |
| Phase 8: Update READMEs     | 2              | 2              |
| Phase 9: Validation         | 8              | 6              |
| Phase 10: Commit            | 2              | 0              |
| **TOTAL**                   | **104**        | **43**         |

---

## Notes

- [P] tasks = different files, no dependencies
- Reverse-order renames prevent directory/file name collisions
- Skip tasks for non-existent image directories (sparse - only 15 of 18 chapters have images)
- Commit after all changes complete — single atomic commit required
- Validation scripts from plan.md can be used for manual verification
