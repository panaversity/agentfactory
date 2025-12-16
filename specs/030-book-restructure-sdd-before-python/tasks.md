# Tasks: Book Restructure - SDD-RI Before Python

**Input**: Design documents from `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/specs/030-book-restructure-sdd-before-python/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Organization**: Tasks are grouped by migration phase to enable sequential execution with validation checkpoints. Each phase follows the migration sequence defined in plan.md and spec.md.

## Format: `- [ ] T### [P] [USN] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[USN]**: Which user story this task belongs to (US1-US5)
- Include exact file paths in descriptions
- **CRITICAL**: All operations use `git mv` to preserve history

## User Story Priority Reference

- **US1**: Update Book Structure Metadata (P1) - Foundation for all work
- **US2**: Move Existing Chapter Directories (P2) - File system consistency
- **US3**: Create Placeholder READMEs (P3) - Complete structure
- **US4**: Rename All Slide PDFs and Images (P1) - CRITICAL infrastructure
- **US5**: Update Part Folder Names (P2) - Directory semantic clarity

---

## Phase 1: Setup & Pre-Migration Validation

**Purpose**: Ensure clean state and capture baseline metrics for rollback

- [ ] T001 [US5] Verify git repository is in clean state (no uncommitted changes): `git status`
- [ ] T002 [US1] Create backup of current chapter-index.md: `cp specs/book/chapter-index.md specs/book/chapter-index.md.backup`
- [ ] T003 [US5] Count all markdown files (baseline): `find book-source/docs -name '*.md' | tee /tmp/md-files-before.txt | wc -l` (record count)
- [ ] T004 [US4] Count all slide PDFs (baseline): `ls -1 book-source/static/slides/*.pdf | tee /tmp/slides-before.txt | wc -l` (record count: expect 33)
- [ ] T005 [US4] Count all image directories (baseline): `find book-source/static/img -type d -name "chapter-*" | tee /tmp/images-before.txt | wc -l` (record count: expect ~30)
- [ ] T006 [US5] Verify Docusaurus build succeeds before migration: `cd book-source && npm run build`

**Checkpoint**: Baseline metrics captured - can proceed with migration

---

## Phase 2: Foundational - Part Directory Renames (US5 - P2)

**Purpose**: Rename part directories to avoid conflicts during chapter moves

**⚠️ CRITICAL**: Execute in this exact order to avoid naming conflicts

- [ ] T007 [US5] Rename Part 4 (Python) to Part 5: `git mv apps/learn-app/docs/04-Python-Fundamentals apps/learn-app/docs/05-Python-Fundamentals`
- [ ] T008 [US5] Rename Part 5 (SDD) to Part 4: `git mv apps/learn-app/docs/05-Spec-Driven-Development apps/learn-app/docs/04-SDD-RI-Fundamentals`
- [ ] T009 [US5] Create Part 6 directory: `mkdir -p apps/learn-app/docs/06-AI-Product-Leadership`
- [ ] T010 [US5] Verify part directory structure: `ls -1d apps/learn-app/docs/0*` (should show 01-06 parts)

**Checkpoint**: Part directories renamed - ready for chapter moves

---

## Phase 3: User Story 4 - Asset Migration (US4 - P1, CRITICAL)

**Purpose**: Rename slides and move images to match new chapter numbers BEFORE moving chapter directories

### Slide PDF Renaming (REVERSE ORDER)

**⚠️ CRITICAL**: Execute in REVERSE ORDER to avoid overwriting files

#### Python Slides (Chapters 13-30 → 16-33)

- [ ] T011 [US4] Rename slide: `git mv book-source/static/slides/chapter-30-slides.pdf book-source/static/slides/chapter-33-slides.pdf`
- [ ] T012 [US4] Rename slide: `git mv book-source/static/slides/chapter-29-slides.pdf book-source/static/slides/chapter-32-slides.pdf`
- [ ] T013 [US4] Rename slide: `git mv book-source/static/slides/chapter-28-slides.pdf book-source/static/slides/chapter-31-slides.pdf`
- [ ] T014 [US4] Rename slide: `git mv book-source/static/slides/chapter-27-slides.pdf book-source/static/slides/chapter-30-slides.pdf`
- [ ] T015 [US4] Rename slide: `git mv book-source/static/slides/chapter-26-slides.pdf book-source/static/slides/chapter-29-slides.pdf`
- [ ] T016 [US4] Rename slide: `git mv book-source/static/slides/chapter-25-slides.pdf book-source/static/slides/chapter-28-slides.pdf`
- [ ] T017 [US4] Rename slide: `git mv book-source/static/slides/chapter-24-slides.pdf book-source/static/slides/chapter-27-slides.pdf`
- [ ] T018 [US4] Rename slide: `git mv book-source/static/slides/chapter-23-slides.pdf book-source/static/slides/chapter-26-slides.pdf`
- [ ] T019 [US4] Rename slide: `git mv book-source/static/slides/chapter-22-slides.pdf book-source/static/slides/chapter-25-slides.pdf`
- [ ] T020 [US4] Rename slide: `git mv book-source/static/slides/chapter-21-slides.pdf book-source/static/slides/chapter-24-slides.pdf`
- [ ] T021 [US4] Rename slide: `git mv book-source/static/slides/chapter-20-slides.pdf book-source/static/slides/chapter-23-slides.pdf`
- [ ] T022 [US4] Rename slide: `git mv book-source/static/slides/chapter-19-slides.pdf book-source/static/slides/chapter-22-slides.pdf`
- [ ] T023 [US4] Rename slide: `git mv book-source/static/slides/chapter-18-slides.pdf book-source/static/slides/chapter-21-slides.pdf`
- [ ] T024 [US4] Rename slide: `git mv book-source/static/slides/chapter-17-slides.pdf book-source/static/slides/chapter-20-slides.pdf`
- [ ] T025 [US4] Rename slide: `git mv book-source/static/slides/chapter-16-slides.pdf book-source/static/slides/chapter-19-slides.pdf`
- [ ] T026 [US4] Rename slide: `git mv book-source/static/slides/chapter-15-slides.pdf book-source/static/slides/chapter-18-slides.pdf`
- [ ] T027 [US4] Rename slide: `git mv book-source/static/slides/chapter-14-slides.pdf book-source/static/slides/chapter-17-slides.pdf`
- [ ] T028 [US4] Rename slide: `git mv book-source/static/slides/chapter-13-slides.pdf book-source/static/slides/chapter-16-slides.pdf`

#### SDD Slides (Chapters 31-32 → 13-14)

- [ ] T029 [US4] Rename slide: `git mv book-source/static/slides/chapter-32-slides.pdf book-source/static/slides/chapter-14-slides.pdf`
- [ ] T030 [US4] Rename slide: `git mv book-source/static/slides/chapter-31-slides.pdf book-source/static/slides/chapter-13-slides.pdf`

#### AI Orchestra Slide (Chapter 33 → 35)

- [ ] T031 [US4] Rename slide: `git mv book-source/static/slides/chapter-33-slides.pdf book-source/static/slides/chapter-35-slides.pdf`

### Image Directory Migration (REVERSE ORDER)

**⚠️ CRITICAL**: Execute in REVERSE ORDER to avoid overwriting directories

#### Python Images (part-4/chapter-13-30 → part-5/chapter-16-33)

- [ ] T032 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-30 book-source/static/img/part-5/chapter-33` (if exists)
- [ ] T033 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-29 book-source/static/img/part-5/chapter-32` (if exists)
- [ ] T034 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-28 book-source/static/img/part-5/chapter-31` (if exists)
- [ ] T035 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-27 book-source/static/img/part-5/chapter-30` (if exists)
- [ ] T036 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-26 book-source/static/img/part-5/chapter-29` (if exists)
- [ ] T037 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-25 book-source/static/img/part-5/chapter-28` (if exists)
- [ ] T038 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-24 book-source/static/img/part-5/chapter-27` (if exists)
- [ ] T039 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-23 book-source/static/img/part-5/chapter-26` (if exists)
- [ ] T040 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-22 book-source/static/img/part-5/chapter-25` (if exists)
- [ ] T041 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-21 book-source/static/img/part-5/chapter-24` (if exists)
- [ ] T042 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-20 book-source/static/img/part-5/chapter-23` (if exists)
- [ ] T043 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-19 book-source/static/img/part-5/chapter-22` (if exists)
- [ ] T044 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-18 book-source/static/img/part-5/chapter-21` (if exists)
- [ ] T045 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-17 book-source/static/img/part-5/chapter-20` (if exists)
- [ ] T046 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-16 book-source/static/img/part-5/chapter-19` (if exists)
- [ ] T047 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-15 book-source/static/img/part-5/chapter-18` (if exists)
- [ ] T048 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-14 book-source/static/img/part-5/chapter-17` (if exists)
- [ ] T049 [US4] Move image dir: `git mv book-source/static/img/part-4/chapter-13 book-source/static/img/part-5/chapter-16` (if exists)

#### SDD Images (part-5/chapter-31-32 → part-4/chapter-13-14)

- [ ] T050 [US4] Create part-4 image directory: `mkdir -p book-source/static/img/part-4`
- [ ] T051 [US4] Move image dir: `git mv book-source/static/img/part-5/chapter-32 book-source/static/img/part-4/chapter-14` (if exists)
- [ ] T052 [US4] Move image dir: `git mv book-source/static/img/part-5/chapter-31 book-source/static/img/part-4/chapter-13` (if exists)

#### AI Orchestra Images (part-5/chapter-33 → part-6/chapter-35)

- [ ] T053 [US4] Create part-6 image directory: `mkdir -p book-source/static/img/part-6`
- [ ] T054 [US4] Move image dir: `git mv book-source/static/img/part-5/chapter-33 book-source/static/img/part-6/chapter-35` (if exists)

**Checkpoint**: All assets renamed/moved - ready for chapter directory moves

---

## Phase 4: User Story 2 - Chapter Directory Moves (US2 - P2)

**Purpose**: Move chapter directories to match new numbering

**⚠️ CRITICAL**: Execute in REVERSE ORDER to avoid overwriting directories

### Python Chapters (13-30 → 16-33, REVERSE ORDER)

- [ ] T055 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/30-python-package-managers-comparison apps/learn-app/docs/05-Python-Fundamentals/33-python-package-managers-comparison`
- [ ] T056 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/29-async-and-concurrency apps/learn-app/docs/05-Python-Fundamentals/32-async-and-concurrency`
- [ ] T057 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/28-python-docstrings-for-genai apps/learn-app/docs/05-Python-Fundamentals/31-python-docstrings-for-genai`
- [ ] T058 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/27-reading-and-writing-files apps/learn-app/docs/05-Python-Fundamentals/30-reading-and-writing-files`
- [ ] T059 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/26-error-handling-and-exceptions apps/learn-app/docs/05-Python-Fundamentals/29-error-handling-and-exceptions`
- [ ] T060 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/25-modules-packages-imports apps/learn-app/docs/05-Python-Fundamentals/28-modules-packages-imports`
- [ ] T061 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/24-dictionaries apps/learn-app/docs/05-Python-Fundamentals/27-dictionaries`
- [ ] T062 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/23-lists-and-tuples apps/learn-app/docs/05-Python-Fundamentals/26-lists-and-tuples`
- [ ] T063 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/22-loops apps/learn-app/docs/05-Python-Fundamentals/25-loops`
- [ ] T064 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/21-conditionals apps/learn-app/docs/05-Python-Fundamentals/24-conditionals`
- [ ] T065 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/20-functions apps/learn-app/docs/05-Python-Fundamentals/23-functions`
- [ ] T066 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/19-operators-and-expressions apps/learn-app/docs/05-Python-Fundamentals/22-operators-and-expressions`
- [ ] T067 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/18-variables-data-types apps/learn-app/docs/05-Python-Fundamentals/21-variables-data-types`
- [ ] T068 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/17-control-flow-overview apps/learn-app/docs/05-Python-Fundamentals/20-control-flow-overview`
- [ ] T069 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/16-python-syntax-basics apps/learn-app/docs/05-Python-Fundamentals/19-python-syntax-basics`
- [ ] T070 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/15-interactive-python-in-claude-ai apps/learn-app/docs/05-Python-Fundamentals/18-interactive-python-in-claude-ai`
- [ ] T071 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/14-introduction-to-python apps/learn-app/docs/05-Python-Fundamentals/17-introduction-to-python`
- [ ] T072 [US2] Move chapter: `git mv apps/learn-app/docs/05-Python-Fundamentals/13-python-uv-package-manager apps/learn-app/docs/05-Python-Fundamentals/16-python-uv-package-manager`

### SDD Chapters (31-32 → 13-14)

- [ ] T073 [US2] Move chapter: `git mv apps/learn-app/docs/04-SDD-RI-Fundamentals/32-spec-kit-plus-hands-on apps/learn-app/docs/04-SDD-RI-Fundamentals/14-youtube-content-workflow-gemini`
- [ ] T074 [US2] Move chapter: `git mv apps/learn-app/docs/04-SDD-RI-Fundamentals/31-specification-driven-development-fundamentals apps/learn-app/docs/04-SDD-RI-Fundamentals/13-specification-driven-development-fundamentals`

### AI Orchestra Chapter (33 → 35)

- [ ] T075 [US2] Move chapter: `git mv apps/learn-app/docs/04-SDD-RI-Fundamentals/33-ai-orchestra-agent-teams-manager apps/learn-app/docs/06-AI-Product-Leadership/35-ai-orchestra-agent-teams-manager`

- [ ] T076 [US2] Verify chapter directory structure: `find book-source/docs -type d -name "[0-9]*-*" | sort` (should show correct numbering)

**Checkpoint**: All chapters moved to correct locations

---

## Phase 5: User Story 1 - Update Metadata (US1 - P1)

**Purpose**: Update YAML frontmatter and replace chapter-index.md

### YAML Frontmatter Updates

**⚠️ NOTE**: Tasks T077-T097 update `sidebar_position` and `slides.source` in chapter README.md files

#### Python Chapters (16-33) - YAML Updates

- [ ] T077 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/16-python-uv-package-manager/README.md (sidebar_position: 16, slides.source: /slides/chapter-16-slides.pdf)
- [ ] T078 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/17-introduction-to-python/README.md (sidebar_position: 17, slides.source: /slides/chapter-17-slides.pdf)
- [ ] T079 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/18-interactive-python-in-claude-ai/README.md (sidebar_position: 18, slides.source: /slides/chapter-18-slides.pdf)
- [ ] T080 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/19-python-syntax-basics/README.md (sidebar_position: 19, slides.source: /slides/chapter-19-slides.pdf)
- [ ] T081 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/20-control-flow-overview/README.md (sidebar_position: 20, slides.source: /slides/chapter-20-slides.pdf)
- [ ] T082 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/21-variables-data-types/README.md (sidebar_position: 21, slides.source: /slides/chapter-21-slides.pdf)
- [ ] T083 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/22-operators-and-expressions/README.md (sidebar_position: 22, slides.source: /slides/chapter-22-slides.pdf)
- [ ] T084 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/23-functions/README.md (sidebar_position: 23, slides.source: /slides/chapter-23-slides.pdf)
- [ ] T085 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/24-conditionals/README.md (sidebar_position: 24, slides.source: /slides/chapter-24-slides.pdf)
- [ ] T086 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/25-loops/README.md (sidebar_position: 25, slides.source: /slides/chapter-25-slides.pdf)
- [ ] T087 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/26-lists-and-tuples/README.md (sidebar_position: 26, slides.source: /slides/chapter-26-slides.pdf)
- [ ] T088 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/27-dictionaries/README.md (sidebar_position: 27, slides.source: /slides/chapter-27-slides.pdf)
- [ ] T089 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/28-modules-packages-imports/README.md (sidebar_position: 28, slides.source: /slides/chapter-28-slides.pdf)
- [ ] T090 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/29-error-handling-and-exceptions/README.md (sidebar_position: 29, slides.source: /slides/chapter-29-slides.pdf)
- [ ] T091 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/30-reading-and-writing-files/README.md (sidebar_position: 30, slides.source: /slides/chapter-30-slides.pdf)
- [ ] T092 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/31-python-docstrings-for-genai/README.md (sidebar_position: 31, slides.source: /slides/chapter-31-slides.pdf)
- [ ] T093 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/32-async-and-concurrency/README.md (sidebar_position: 32, slides.source: /slides/chapter-32-slides.pdf)
- [ ] T094 [P] [US1] Update YAML in apps/learn-app/docs/05-Python-Fundamentals/33-python-package-managers-comparison/README.md (sidebar_position: 33, slides.source: /slides/chapter-33-slides.pdf)

#### SDD Chapters (13-14) - YAML Updates

- [ ] T095 [P] [US1] Update YAML in apps/learn-app/docs/04-SDD-RI-Fundamentals/13-specification-driven-development-fundamentals/README.md (sidebar_position: 13, slides.source: /slides/chapter-13-slides.pdf)
- [ ] T096 [P] [US1] Update YAML in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-youtube-content-workflow-gemini/README.md (sidebar_position: 14, slides.source: /slides/chapter-14-slides.pdf)

#### AI Orchestra Chapter (35) - YAML Update

- [ ] T097 [P] [US1] Update YAML in apps/learn-app/docs/06-AI-Product-Leadership/35-ai-orchestra-agent-teams-manager/README.md (sidebar_position: 35, slides.source: /slides/chapter-35-slides.pdf)

### Chapter Index Replacement

- [ ] T098 [US1] Replace chapter-index.md: `cp specs/book/chapter-index-NEW.md specs/book/chapter-index.md` (updates 84→86 chapters, all parts correctly numbered)

**Checkpoint**: All metadata updated to reflect new structure

---

## Phase 6: User Story 3 - Create Placeholders (US3 - P3)

**Purpose**: Create placeholder README.md files for new chapters

- [ ] T099 [P] [US3] Create Chapter 15 directory: `mkdir -p apps/learn-app/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone`
- [ ] T100 [P] [US3] Create Chapter 15 placeholder README.md at apps/learn-app/docs/04-SDD-RI-Fundamentals/15-ai-product-business-intelligence-capstone/README.md (YAML: sidebar_position: 15, title, slides metadata; content: chapter overview)
- [ ] T101 [P] [US3] Create Chapter 34 directory: `mkdir -p apps/learn-app/docs/06-AI-Product-Leadership/34-ai-product-development-evaluation-first`
- [ ] T102 [P] [US3] Create Chapter 34 placeholder README.md at apps/learn-app/docs/06-AI-Product-Leadership/34-ai-product-development-evaluation-first/README.md (YAML: sidebar_position: 34, title, slides metadata; content: chapter overview)
- [ ] T103 [P] [US3] Create Chapter 36 directory: `mkdir -p apps/learn-app/docs/06-AI-Product-Leadership/36-product-engineering-leadership-ai`
- [ ] T104 [P] [US3] Create Chapter 36 placeholder README.md at apps/learn-app/docs/06-AI-Product-Leadership/36-product-engineering-leadership-ai/README.md (YAML: sidebar_position: 36, title, slides metadata; content: chapter overview)

**Checkpoint**: All placeholder chapters created

---

## Phase 7: Content Reference Updates & Plugin Config

**Purpose**: Update content references and plugin configurations to match new structure

### Image Reference Updates

- [ ] T105 [US4] Update image markdown references in Python lesson files (grep `/img/part-4/chapter-` and replace with `/img/part-5/chapter-` using +3 shift): `find apps/learn-app/docs/05-Python-Fundamentals -name "*.md" -type f` (automated search-and-replace)
- [ ] T106 [US4] Update image markdown references in SDD lesson files (grep `/img/part-5/chapter-31` → `/img/part-4/chapter-13`, etc.): `find apps/learn-app/docs/04-SDD-RI-Fundamentals -name "*.md" -type f`
- [ ] T107 [US4] Update image markdown references in AI Orchestra lesson files (grep `/img/part-5/chapter-33` → `/img/part-6/chapter-35`): `find apps/learn-app/docs/06-AI-Product-Leadership -name "*.md" -type f`

### Plugin Configuration Updates

- [ ] T108 [US4] Update docusaurus.config.ts: Replace `includePaths: ['/04-Python-Fundamentals/']` with `includePaths: ['/05-Python-Fundamentals/']` in book-source/docusaurus.config.ts
- [ ] T109 [US4] Update remark-interactive-python plugin: Replace `includePaths = ['/04-Python-Fundamentals/']` with `includePaths = ['/05-Python-Fundamentals/']` in book-source/plugins/remark-interactive-python/index.js
- [ ] T110 [US4] Investigate og-image-generator plugin for chapter number parsing: Read book-source/plugins/docusaurus-plugin-og-image-generator/ source (verify no regex like `/(\d+)-/`)
- [ ] T111 [US4] Investigate structured-data plugin for chapter number parsing: Read book-source/plugins/docusaurus-plugin-structured-data/ source (verify uses frontmatter not directory names)

### Narrative Chapter Reference Updates (REVERSE ORDER)

**⚠️ CRITICAL**: Execute in REVERSE ORDER to prevent double-replacement

- [ ] T112 [US1] Update "Chapter 33" → "Chapter 35" in all lesson content: `grep -rl "Chapter 33" apps/learn-app/docs/` (automated replace, affects AI Orchestra mentions)
- [ ] T113 [US1] Update "Chapter 32" → "Chapter 14" in all lesson content: `grep -rl "Chapter 32" apps/learn-app/docs/` (automated replace)
- [ ] T114 [US1] Update "Chapter 31" → "Chapter 13" in all lesson content: `grep -rl "Chapter 31" apps/learn-app/docs/` (automated replace)
- [ ] T115 [US1] Update "Chapter 30" → "Chapter 33" in all lesson content: `grep -rl "Chapter 30" apps/learn-app/docs/` (automated replace, Python chapters start here)
- [ ] T116 [US1] Update "Chapter 29" → "Chapter 32" in all lesson content: `grep -rl "Chapter 29" apps/learn-app/docs/`
- [ ] T117 [US1] Update "Chapter 28" → "Chapter 31" in all lesson content: `grep -rl "Chapter 28" apps/learn-app/docs/`
- [ ] T118 [US1] Update "Chapter 27" → "Chapter 30" in all lesson content: `grep -rl "Chapter 27" apps/learn-app/docs/`
- [ ] T119 [US1] Update "Chapter 26" → "Chapter 29" in all lesson content: `grep -rl "Chapter 26" apps/learn-app/docs/`
- [ ] T120 [US1] Update "Chapter 25" → "Chapter 28" in all lesson content: `grep -rl "Chapter 25" apps/learn-app/docs/`
- [ ] T121 [US1] Update "Chapter 24" → "Chapter 27" in all lesson content: `grep -rl "Chapter 24" apps/learn-app/docs/`
- [ ] T122 [US1] Update "Chapter 23" → "Chapter 26" in all lesson content: `grep -rl "Chapter 23" apps/learn-app/docs/`
- [ ] T123 [US1] Update "Chapter 22" → "Chapter 25" in all lesson content: `grep -rl "Chapter 22" apps/learn-app/docs/`
- [ ] T124 [US1] Update "Chapter 21" → "Chapter 24" in all lesson content: `grep -rl "Chapter 21" apps/learn-app/docs/`
- [ ] T125 [US1] Update "Chapter 20" → "Chapter 23" in all lesson content: `grep -rl "Chapter 20" apps/learn-app/docs/`
- [ ] T126 [US1] Update "Chapter 19" → "Chapter 22" in all lesson content: `grep -rl "Chapter 19" apps/learn-app/docs/`
- [ ] T127 [US1] Update "Chapter 18" → "Chapter 21" in all lesson content: `grep -rl "Chapter 18" apps/learn-app/docs/`
- [ ] T128 [US1] Update "Chapter 17" → "Chapter 20" in all lesson content: `grep -rl "Chapter 17" apps/learn-app/docs/`
- [ ] T129 [US1] Update "Chapter 16" → "Chapter 19" in all lesson content: `grep -rl "Chapter 16" apps/learn-app/docs/`
- [ ] T130 [US1] Update "Chapter 15" → "Chapter 18" in all lesson content: `grep -rl "Chapter 15" apps/learn-app/docs/`
- [ ] T131 [US1] Update "Chapter 14" → "Chapter 17" in all lesson content: `grep -rl "Chapter 14" apps/learn-app/docs/`
- [ ] T132 [US1] Update "Chapter 13" → "Chapter 16" in all lesson content: `grep -rl "Chapter 13" apps/learn-app/docs/`

**Checkpoint**: All content references and plugin configs updated

---

## Phase 8: Final Validation

**Purpose**: Validate all 14 Success Criteria from spec.md

### Git History & File Integrity Validation

- [ ] T133 [US2] SC-001: Verify all 3 chapters moved with git history: `git log --follow --oneline apps/learn-app/docs/04-SDD-RI-Fundamentals/13-specification-driven-development-fundamentals | head -5` (expect commit history), repeat for chapters 14, 35
- [ ] T134 [US3] SC-002: Verify all 3 placeholder READMEs exist with valid YAML: `grep "sidebar_position:" apps/learn-app/docs/04-SDD-RI-Fundamentals/15-*/README.md apps/learn-app/docs/06-AI-Product-Leadership/34-*/README.md apps/learn-app/docs/06-AI-Product-Leadership/36-*/README.md`
- [ ] T135 [US1] SC-003: Verify chapter-index.md shows 86 chapters correctly: `grep "^##" specs/book/chapter-index.md | wc -l` (expect 86 chapters across 12 parts)
- [ ] T136 [US5] SC-004: Verify directory structure (Part 4: 3 chapters, Part 5: 18, Part 6: 3): `ls -1d apps/learn-app/docs/04-SDD-RI-Fundamentals/*/ | wc -l` (expect 3), `ls -1d apps/learn-app/docs/05-Python-Fundamentals/*/ | wc -l` (expect 18), `ls -1d apps/learn-app/docs/06-AI-Product-Leadership/*/ | wc -l` (expect 3)
- [ ] T137 [US5] SC-005: Verify zero content files lost: `find book-source/docs -name '*.md' | wc -l` (compare with T003 baseline, must match)

### Docusaurus Build Validation

- [ ] T138 [US5] SC-006: Run Docusaurus build and verify success: `cd book-source && npm run build` (expect exit code 0, no errors)

### YAML & Asset Validation

- [ ] T139 [US1] SC-007: Verify all moved chapter READMEs have correct sidebar_position: `grep "sidebar_position:" apps/learn-app/docs/04-SDD-RI-Fundamentals/13-*/README.md` (expect 13), repeat for all 21 affected chapters
- [ ] T140 [US4] SC-008: Verify all 21 slide PDFs renamed with git history: `git log --follow --oneline book-source/static/slides/chapter-13-slides.pdf | head -3` (expect history), spot-check 5 random slide PDFs
- [ ] T141 [US4] SC-009: Verify all ~18 image directories moved with git history: `git log --follow --oneline book-source/static/img/part-5/chapter-16 | head -3` (if exists), spot-check 3 random image dirs
- [ ] T142 [US4] SC-010: Verify all YAML slides.source references updated: `grep "slides.source:" apps/learn-app/docs/04-SDD-RI-Fundamentals/13-*/README.md` (expect /slides/chapter-13-slides.pdf), spot-check 5 random chapters
- [ ] T143 [US4] SC-011: Verify all image markdown references updated (no old part- paths): `grep -r "/img/part-4/chapter-" apps/learn-app/docs/05-Python-Fundamentals` (expect zero matches EXCEPT chapters 13-15 which weren't renumbered), `grep -r "/img/part-5/chapter-31\|chapter-32\|chapter-33" book-source/docs` (expect zero matches)

### Plugin & Content Validation

- [ ] T144 [US4] SC-012: Verify interactive Python plugin paths updated: `grep -r "includePaths.*04-Python" book-source/docusaurus.config.ts book-source/plugins/remark-interactive-python/` (expect zero matches)
- [ ] T145 [US4] SC-013: Manual browser test - verify Chapter 16 interactive code blocks: Start dev server `cd book-source && npm start`, navigate to Chapter 16, verify code is interactive (manual test)
- [ ] T146 [US1] SC-014: Spot-check narrative chapter references: `grep -n "Chapter 16" apps/learn-app/docs/05-Python-Fundamentals/17-*/README.md` (verify mentions of Python UV chapter use new number 16 not 13), repeat for 4 more random chapters

### Asset Count Validation

- [ ] T147 [US4] Verify slide PDF count unchanged: `ls -1 book-source/static/slides/*.pdf | wc -l` (expect 33, same as T004 baseline)
- [ ] T148 [US4] Verify image directory count unchanged: `find book-source/static/img -type d -name "chapter-*" | wc -l` (expect ~30, same as T005 baseline)
- [ ] T149 [US5] Verify no orphaned files in old locations: `ls apps/learn-app/docs/04-Python-Fundamentals 2>/dev/null` (expect "No such file or directory"), `ls apps/learn-app/docs/05-Spec-Driven-Development 2>/dev/null` (expect "No such file or directory")

**Checkpoint**: All 14 Success Criteria validated - migration complete

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - can start immediately
- **Phase 2 (Part Renames)**: Depends on Phase 1 validation - BLOCKS all subsequent phases
- **Phase 3 (Asset Migration)**: Depends on Phase 2 completion - must happen BEFORE chapter moves
- **Phase 4 (Chapter Moves)**: Depends on Phase 2 & 3 completion - file system operations
- **Phase 5 (Metadata Updates)**: Depends on Phase 4 completion - updates chapter frontmatter
- **Phase 6 (Placeholders)**: Can happen anytime after Phase 2, but logically after Phase 5
- **Phase 7 (Content Updates)**: Depends on Phase 4 & 5 completion - updates references
- **Phase 8 (Validation)**: Depends on ALL previous phases - final verification

### Critical Sequencing

1. **Part renames BEFORE chapter moves** (Phase 2 → Phase 4): Avoids naming conflicts
2. **Asset migrations BEFORE chapter moves** (Phase 3 → Phase 4): Assets match chapter numbers
3. **Chapter moves BEFORE YAML updates** (Phase 4 → Phase 5): Directories exist before metadata updates
4. **YAML updates BEFORE content reference updates** (Phase 5 → Phase 7): Frontmatter accurate before narrative updates
5. **All migrations BEFORE validation** (Phases 1-7 → Phase 8): Validate complete state

### Parallel Opportunities

- **Phase 5 (T077-T097)**: All YAML updates marked [P] can run in parallel (21 different files)
- **Phase 6 (T099-T104)**: All placeholder creation tasks marked [P] can run in parallel (3 different directories)
- **Phase 8**: Some validation tasks can run in parallel (different verification types)

### Within Each Phase

- **Phase 3 (Assets)**: MUST execute in REVERSE ORDER (T011→T031 for slides, T032→T054 for images)
- **Phase 4 (Chapters)**: MUST execute in REVERSE ORDER (T055→T075)
- **Phase 7 (Content Refs)**: MUST execute in REVERSE ORDER (T112→T132)

---

## Rollback Procedure

**If migration fails at any point**:

```bash
# Undo all uncommitted changes
git reset --hard HEAD

# If commit was made but not pushed
git reset --hard HEAD~1

# Restore backup
cp specs/book/chapter-index.md.backup specs/book/chapter-index.md

# Verify restoration
ls -la apps/learn-app/docs/04-Python-Fundamentals/  # Should exist
ls -la apps/learn-app/docs/05-Spec-Driven-Development/  # Should exist
find book-source/docs -name "*.md" | wc -l  # Should match T003 baseline
```

---

## Notes

- **[P]** tasks = different files, no dependencies, can run in parallel
- **[USN]** label maps task to specific user story for traceability
- **CRITICAL**: All `git mv` operations preserve history automatically
- **REVERSE ORDER**: Essential for overlapping numeric ranges (prevents overwriting)
- Commit after each phase or logical group for easy rollback
- Stop at any checkpoint to validate progress
- Estimated total time: 2-3 hours (mostly automated operations)
