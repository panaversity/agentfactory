# Implementation Tasks (Refined): Chapter 7 Bash Essentials Redesign

**Feature**: Chapter 7 Bash Essentials for AI-Driven Development
**Branch**: `feature/chapter-7-bash-essentials`
**Created**: 2025-11-07
**Status**: Ready for Implementation

---

## Executive Summary: Existing vs. New Lesson Mapping

This refined task list specifically addresses: **What existing lessons to update, what to rewrite, what to create from scratch, and what to archive.**

### Content Reuse Strategy Matrix

| Existing Lesson | New Plan Lesson | Action Required | Reuse % | Effort | Rationale |
|-----------------|-----------------|-----------------|---------|--------|-----------|
| **01-introducing-ai-workspace.md** | **Lesson 1: Finding Your Way** | **Major Update** | 40% | 6-8h | Keep: foundational navigation concepts, file system basics<br>Add: natural language framing, location awareness, CEFR A1 metadata<br>Rewrite: 60% new content (AI-native approach) |
| **02-safety-first-pattern.md** | **Lesson 2: The Safety Pattern** | **Minor Update** | 70% | 3-4h | Keep: excellent 5-step pattern structure, most dialogue examples<br>Add: real data loss story opening, red flag section, more checkpoints<br>Update: 30% enhancement (already strong) |
| **03-understanding-navigation.md** | *(merge into Lesson 1)* | **Archive** | 100% | 1h | Merge navigation content into redesigned Lesson 1<br>Move file to `.archive/` directory |
| **04-understanding-file-operations.md** | **Lesson 4: File Operations via Natural Language** | **Major Update** | 50% | 5-7h | Keep: file operation fundamentals (copy, move, organize)<br>Add: safety pattern integration throughout, backup thinking, validation techniques<br>Rewrite: 50% new content (safety emphasis) |
| **05-configuration-secrets.md** | **Lesson 5: Configuration and Secrets** | **Minor Update** | 65% | 4-5h | Keep: `.env` file coverage, secrets management basics<br>Add: `.gitignore` integration, security framing, dependency install (from merged 06)<br>Update: 35% enhancement |
| **06-packages-dependencies.md** | *(merge into Lesson 5)* | **Archive** | 100% | 1h | Merge dependency installation content into Lesson 5<br>Move file to `.archive/` directory |
| **07-pipes-complex-commands.md** | **Lesson 3: Reading Bash Commands** | **Complete Rewrite** | 20% | 8-10h | Keep: basic pipe examples<br>Rewrite: EVERYTHING ELSE — reorder before file ops, supervision framing, command anatomy, no memorization<br>80% new content (fundamental reframing) |
| **08-real-project-troubleshooting.md** | **Lessons 6-8 (split)** | **Complete Rewrite** | 10% | 18-24h | Keep: some troubleshooting examples<br>Rewrite: Split into 3 separate lessons (exploration, scripts, orchestration)<br>90% new content (split + tier 3 orchestration) |
| *(none)* | **Lesson 6: Exploring File System Safely** | **Create New** | 0% | 8-10h | Completely new lesson: AI-orchestrated file exploration, find command, pattern matching, validation |
| *(none)* | **Lesson 7: Building and Running Scripts** | **Create New** | 0% | 8-10h | Completely new lesson: directing AI to write scripts, testing safely, debugging |
| *(none)* | **Lesson 8: Orchestrating Complex Workflows** | **Create New** | 0% | 10-12h | Completely new capstone lesson: decomposing complexity, batch operations, multi-step workflows |
| **README.md** | **Chapter README** | **Complete Rewrite** | 30% | 2-3h | Keep: basic chapter positioning<br>Rewrite: AI-native philosophy, new lesson titles/descriptions, updated learning outcomes<br>70% new content |

### Summary Statistics

- **Keep as-is**: 0 lessons (none meet new standards)
- **Minor updates (30-35% new)**: 2 lessons (Lessons 2, 5)
- **Major updates (40-60% new)**: 2 lessons (Lessons 1, 4)
- **Complete rewrites (70-90% new)**: 2 lessons (Lessons 3, and split of old Lesson 8)
- **Create from scratch**: 3 lessons (Lessons 6, 7, 8)
- **Archive/merge**: 2 lessons (old 03, 06)
- **README rewrite**: 1 file (70% new)

**Total Effort**: 75-100 hours (9-13 story points)

---

## Implementation Tasks

### Phase 1: Setup & Content Audit (Week 1, Day 1) — 3-4 hours

**Goal**: Prepare project structure, analyze existing content, archive outdated files.

- [ ] T001 Create content reuse analysis spreadsheet documenting keep/update/rewrite/create decisions for each lesson
- [ ] T002 Create archive directory `/book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/.archive/` and move lessons 03 and 06 with timestamp
- [ ] T003 Backup existing README.md to `.archive/README-old-2025-11-07.md`

**Acceptance Criteria**:
- [ ] Content reuse matrix exists (spreadsheet or markdown table)
- [ ] Old lessons 03 and 06 archived with clear timestamp
- [ ] Original README backed up before rewrite
- [ ] All active lesson files (01, 02, 04, 05, 07, 08) remain in place for updating

---

### Phase 2: Foundation Lessons (US1, US2 | Week 1, Days 2-5) — 12-17 hours

**User Story 1**: Natural Language Navigation (P1)
**User Story 2**: Safety-First File Operations (P1)

#### Lesson 1: Finding Your Way (Major Update — 40% reuse, 60% new)

- [ ] T004 [US1] Read existing 01-introducing-ai-workspace.md and identify sections to keep vs. rewrite
- [ ] T005 [US1] Merge navigation content from archived 03-understanding-navigation.md into Lesson 1
- [ ] T006 [US1] Rewrite Section A "What Is Terminal" with AI-native framing (NOT command-memorization)
- [ ] T007 [P] [US1] Update Section B "Working Directory" to emphasize location awareness as supervision prerequisite
- [ ] T008 [US1] Rewrite Section C "Asking Right Questions" with natural language patterns ("Where am I?" not "run pwd")
- [ ] T009 [P] [US1] Add 2 real dialogue transcripts (navigation conversation, file listing with explanation)
- [ ] T010 [P] [US1] Create 3 checkpoint exercises requiring AI conversations, not bash typing
- [ ] T011 [P] [US1] Add YAML frontmatter with skills proficiency metadata (CEFR A1, Bloom's Understand, DigComp)
- [ ] T012 [US1] Update "Try With AI" section with 3 natural language prompts and expected outcomes

**Acceptance Criteria (Lesson 1)**:
- [ ] Natural language first approach demonstrated throughout
- [ ] Location awareness framed as foundation for all operations
- [ ] 2 authentic dialogue transcripts included
- [ ] CEFR A1 proficiency validated (max 5 concepts)
- [ ] No command syntax memorization required
- [ ] Estimated time: 6-8 hours

---

#### Lesson 2: The Safety Pattern (Minor Update — 70% reuse, 30% new)

- [ ] T013 [US2] Read existing 02-safety-first-pattern.md and identify sections needing enhancement
- [ ] T014 [US2] Add real data loss story as opening (developer deleted 6 months of work in 5 seconds)
- [ ] T015 [P] [US2] Enhance 5-step pattern sections with 1-2 additional real dialogue examples
- [ ] T016 [P] [US2] Add NEW Section C "Red Flags That Require Extra Caution" (`rm -rf`, `sudo`, `chmod 777`)
- [ ] T017 [P] [US2] Add NEW Section D "Common Mistakes and How to Avoid Them"
- [ ] T018 [P] [US2] Add 1-2 more checkpoint exercises (identify missing steps, write clarifying questions)
- [ ] T019 [P] [US2] Add safety pattern flowchart visual aid (decision tree)
- [ ] T020 [US2] Update YAML frontmatter with A1 proficiency and safety emphasis

**Acceptance Criteria (Lesson 2)**:
- [ ] Real consequences story hooks reader immediately
- [ ] Red flag commands explicitly identified with cautionary language
- [ ] 4 total real dialogue transcripts (existing + new)
- [ ] Practice exercises require students to identify missing safety steps
- [ ] CEFR A1 proficiency validated
- [ ] Estimated time: 3-4 hours

---

**Phase 2 Independent Test (US1)**: Student conducts 3 AI conversations about file system without typing commands.

**Phase 2 Independent Test (US2)**: Student directs AI to delete folder with all 5 safety steps demonstrated.

**Phase 2 Parallel Opportunities**: T007, T009, T010, T011 (Lesson 1) | T015, T016, T017, T018, T019 (Lesson 2)

---

### Phase 3: Command Comprehension (US5 | Week 2, Days 1-3) — 8-10 hours

**User Story 5**: Reading and Understanding AI-Generated Commands (P2)

#### Lesson 3: Reading Bash Commands (Complete Rewrite — 20% reuse, 80% new)

- [ ] T021 [US5] Read existing 07-pipes-complex-commands.md and extract reusable pipe examples (20% salvage)
- [ ] T022 [US5] **COMPLETELY REWRITE** lesson as 03-reading-bash-commands.md with supervision framing (not syntax learning)
- [ ] T023 [P] [US5] Write NEW Section A "Command Anatomy" (command/args/flags structure breakdown)
- [ ] T024 [P] [US5] Write NEW Section B "Common Flags You'll See" (practical examples, NOT exhaustive reference)
- [ ] T025 [P] [US5] Rewrite Section C "Pipes and Redirects" with plain English explanations and supervision strategy
- [ ] T026 [US5] Write NEW Section D "Asking Clarifying Questions" (how to supervise command execution)
- [ ] T027 [P] [US5] Add 5-6 command breakdown examples (ls -lah, find, grep, cat | sort, mkdir -p)
- [ ] T028 [P] [US5] Add 3 real dialogue transcripts (command explanation, pipe breakdown, flag clarification)
- [ ] T029 [P] [US5] Create 3 checkpoint exercises (explain command in plain English, identify flags, pipe comprehension)
- [ ] T030 [US5] Add YAML frontmatter with A2 proficiency and comprehension focus

**Acceptance Criteria (Lesson 3)**:
- [ ] Command structure taught as anatomy, NOT syntax to memorize
- [ ] Supervision strategy emphasized (ask "What does this do?" before execution)
- [ ] Pipes presented as connecting commands (output → input)
- [ ] 3 authentic dialogue transcripts showing command explanation
- [ ] CEFR A2 proficiency validated (max 7 concepts)
- [ ] **CRITICAL**: Reorder BEFORE file operations (was Lesson 7, now Lesson 3)
- [ ] Estimated time: 8-10 hours

---

**Phase 3 Independent Test (US5)**: Student reads `find . -name "*.py" | xargs grep "import os"` and explains: "Finds all Python files and searches them for 'import os'."

**Phase 3 Parallel Opportunities**: T023, T024, T027, T028, T029 (different sections after T022 framework complete)

---

### Phase 4: File Operations & Configuration (US2, US3 | Week 2, Days 4-5 + Week 3, Days 1-2) — 14-19 hours

**User Story 2**: Safety-First File Operations (P1) — continued
**User Story 3**: Configuration and Environment Setup (P2)

#### Lesson 4: File Operations via Natural Language (Major Update — 50% reuse, 50% new)

- [ ] T031 [US2] Read existing 04-understanding-file-operations.md and identify sections to keep/enhance
- [ ] T032 [US2] Rename file to 04-file-operations-natural-language.md
- [ ] T033 [P] [US2] Rewrite Section A "Copy Operations" with backup thinking and safety pattern integration
- [ ] T034 [P] [US2] Rewrite Section B "Move Operations" with extra caution framing (removes originals)
- [ ] T035 [P] [US2] Update Section C "Organizing Files" with hierarchical directory thinking
- [ ] T036 [US2] Add NEW Section D "Validation Techniques" (how to verify operations succeeded)
- [ ] T037 [P] [US2] Add 3 real dialogue transcripts (backup before operation, copy with safety, move with verification)
- [ ] T038 [P] [US2] Update 3 checkpoint exercises to require safety pattern demonstration
- [ ] T039 [US2] Update YAML frontmatter with A2 proficiency and operation validation focus

**Acceptance Criteria (Lesson 4)**:
- [ ] Every file operation demonstrates 5-step safety pattern
- [ ] Backup thinking emphasized (copy before move/delete)
- [ ] Validation techniques taught (how to confirm success)
- [ ] 3 authentic dialogue transcripts with full safety steps
- [ ] CEFR A2 proficiency validated
- [ ] Estimated time: 5-7 hours

---

#### Lesson 5: Configuration and Secrets (Minor Update — 65% reuse, 35% new)

- [ ] T040 [US3] Read existing 05-configuration-secrets.md and identify sections needing enhancement
- [ ] T041 [US3] Merge dependency installation content from archived 06-packages-dependencies.md into Section D
- [ ] T042 [P] [US3] Update Section A "Environment Variables" with security framing
- [ ] T043 [P] [US3] Enhance Section B ".env Files" with real API key example and why-not-commit explanation
- [ ] T044 [P] [US3] Add NEW Section C ".gitignore Integration" (protecting secrets from version control)
- [ ] T045 [US3] Write NEW Section D "Installing Dependencies" (merged from old 06)
- [ ] T046 [P] [US3] Add 3 real dialogue transcripts (API key setup, .gitignore, dependency install with error)
- [ ] T047 [P] [US3] Update checkpoint exercises to include `.gitignore` configuration
- [ ] T048 [US3] Update YAML frontmatter with A2 proficiency and security focus

**Acceptance Criteria (Lesson 5)**:
- [ ] `.env` + `.gitignore` presented as integrated security workflow
- [ ] Security framing: why secrets must NEVER be committed
- [ ] Dependency installation included (from merged 06-packages)
- [ ] 3 authentic dialogue transcripts showing configuration tasks
- [ ] CEFR A2 proficiency validated
- [ ] Estimated time: 4-5 hours

---

**Phase 4 Independent Test (US2)**: Student directs AI to organize 20 files, demonstrates safety pattern, validates success.

**Phase 4 Independent Test (US3)**: Student creates `.env`, adds API key, configures `.gitignore`—all via conversation.

**Phase 4 Parallel Opportunities**: T033, T034, T035, T037, T038 (Lesson 4) | T042, T043, T046, T047 (Lesson 5)

---

### Phase 5: Orchestration — AI Automates Multi-Step Workflows (US4 | Week 3, Days 3-5 + Week 4) — 30-40 hours

**User Story 4**: Complex Multi-Step Workflows (P3)

**Note**: Old Lesson 08 (real-project-troubleshooting) will be completely rewritten and split into 3 separate orchestration lessons.

#### Lesson 6: Exploring File System Safely (Create New — 0% reuse, 100% new)

- [ ] T049 [US4] Analyze old 08-real-project-troubleshooting.md for any salvageable troubleshooting examples (estimate <10%)
- [ ] T050 [US4] Create NEW file 06-exploring-file-system-safely.md with orchestration framing (Tier 3)
- [ ] T051 [P] [US4] Write Section A "Finding Files with AI" (delegate `find` command to AI, student supervises)
- [ ] T052 [P] [US4] Write Section B "Pattern Matching" (wildcards, regex basics with AI help)
- [ ] T053 [P] [US4] Write Section C "Validation Before Action" (preview results before destructive operations)
- [ ] T054 [US4] Write Section D "Combining Search with Operations" (find + act workflow)
- [ ] T055 [P] [US4] Add 3 real dialogue transcripts (find files, pattern search, preview-then-act)
- [ ] T056 [P] [US4] Create 3 checkpoint exercises (search task, pattern matching, safe batch operation)
- [ ] T057 [US4] Add YAML frontmatter with A2 proficiency and orchestration tier

**Acceptance Criteria (Lesson 6)**:
- [ ] AI orchestrates file searches; student supervises and validates
- [ ] Pattern matching taught as strategic thinking (not regex memorization)
- [ ] Preview-then-act workflow emphasized throughout
- [ ] 3 authentic dialogue transcripts showing orchestration
- [ ] CEFR A2 proficiency validated (7 concepts)
- [ ] Estimated time: 8-10 hours

---

#### Lesson 7: Building and Running Scripts (Create New — 0% reuse, 100% new)

- [ ] T058 [US4] Create NEW file 07-building-running-scripts.md with script orchestration framing (Tier 3)
- [ ] T059 [P] [US4] Write Section A "Directing AI to Write Scripts" (specify intent, not syntax)
- [ ] T060 [P] [US4] Write Section B "Understanding Script Structure" (shebang, permissions, execution basics)
- [ ] T061 [P] [US4] Write Section C "Testing Scripts Safely" (dry run, small test set, full execution)
- [ ] T062 [US4] Write Section D "Debugging When Scripts Fail" (reading error output, asking AI to fix)
- [ ] T063 [P] [US4] Add 3 real dialogue transcripts (script creation, testing workflow, debugging error)
- [ ] T064 [P] [US4] Create 3 checkpoint exercises (specify script, test safely, debug error)
- [ ] T065 [US4] Add YAML frontmatter with A2/B1 proficiency and script orchestration

**Acceptance Criteria (Lesson 7)**:
- [ ] Students direct AI to write scripts, NOT write scripts themselves
- [ ] Testing workflow emphasized (dry run → small test → full execution)
- [ ] Debugging framed as collaboration (student reads error, AI proposes fix)
- [ ] 3 authentic dialogue transcripts showing script lifecycle
- [ ] CEFR A2/B1 boundary proficiency validated (7 concepts)
- [ ] Estimated time: 8-10 hours

---

#### Lesson 8: Orchestrating Complex Workflows (Create New — 0% reuse, 100% new) — **CAPSTONE**

- [ ] T066 [US4] Create NEW file 08-orchestrating-complex-workflows.md as capstone lesson (Tier 3)
- [ ] T067 [P] [US4] Write Section A "Decomposing Complexity" (breaking big tasks into manageable steps)
- [ ] T068 [P] [US4] Write Section B "Batch Operations" (operating on 10+ items at scale)
- [ ] T069 [P] [US4] Write Section C "Multi-Step Workflows" (chaining operations with validation points)
- [ ] T070 [US4] Write Section D "When to Stop and Ask" (recognizing limits, manual intervention)
- [ ] T071 [P] [US4] Add 3 real dialogue transcripts (git worktrees, batch processing, pipeline debugging)
- [ ] T072 [P] [US4] Create 3 checkpoint exercises (decompose task, orchestrate workflow, validate batch)
- [ ] T073 [US4] Add YAML frontmatter with B1 proficiency and capstone emphasis

**Acceptance Criteria (Lesson 8)**:
- [ ] Decomposition taught as strategic skill (not command syntax)
- [ ] Batch operations demonstrated with real workflows (10+ items)
- [ ] Validation checkpoints emphasized throughout multi-step processes
- [ ] 3 authentic dialogue transcripts showing complex orchestration
- [ ] CEFR B1 proficiency validated (10 concepts maximum)
- [ ] Estimated time: 10-12 hours

---

**Phase 5 Independent Test (US4)**: Student specifies "Create 10 git worktrees for features 1-10, set up virtual environments, validate all complete"—supervises AI, validates results.

**Phase 5 Parallel Opportunities**: T051, T052, T053, T055, T056 (Lesson 6) | T059, T060, T061, T063, T064 (Lesson 7) | T067, T068, T069, T071, T072 (Lesson 8) — all can be written in parallel after framework tasks (T050, T058, T066)

---

### Phase 6: Integration & Polish (Week 4-5) — 8-12 hours

**Goal**: Complete chapter README rewrite, validate cross-references, final review.

- [ ] T074 **COMPLETELY REWRITE** README.md with AI-native philosophy (salvage 30%, rewrite 70%) in `/book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/README.md`
- [ ] T075 [P] Validate all internal cross-references (Lesson 2 refs 1, Lesson 3 refs 1-2, etc.)
- [ ] T076 [P] Update all "Try With AI" sections to specify tool choice (Claude Code, Gemini CLI, ChatGPT) consistently
- [ ] T077 [P] Validate YAML frontmatter consistency (skills proficiency, durations, CEFR levels) across all 8 lessons
- [ ] T078 Run Docusaurus build and fix broken links, formatting errors, or render issues
- [ ] T079 Final proofread for voice, tone, Grade 7-8 reading level across all lessons
- [ ] T080 Accessibility validation (alt text for images, heading hierarchy, screen reader compatibility)

**Acceptance Criteria (Phase 6)**:
- [ ] Chapter README clearly articulates AI-native philosophy vs. traditional bash tutorials
- [ ] All cross-references validated and working
- [ ] Docusaurus build succeeds with zero errors
- [ ] Reading level validated at Grade 7-8 (readability tool)
- [ ] Voice and tone consistent across all 8 lessons
- [ ] Accessibility standards met (WCAG 2.1 AA minimum)
- [ ] Estimated time: 8-12 hours

---

**Phase 6 Parallel Opportunities**: T075, T076, T077, T080 (independent validation tasks)

---

## Summary Metrics

### Total Task Count: 80 tasks

| Phase | Tasks | Est. Hours | Story Points | Week |
|-------|-------|-----------|--------------|------|
| Phase 1 (Setup) | 3 | 3-4h | 0.5 | Week 1, Day 1 |
| Phase 2 (Lessons 1-2) | 17 | 12-17h | 2-3 | Week 1, Days 2-5 |
| Phase 3 (Lesson 3) | 10 | 8-10h | 1-2 | Week 2, Days 1-3 |
| Phase 4 (Lessons 4-5) | 18 | 14-19h | 2-3 | Week 2-3 |
| Phase 5 (Lessons 6-8) | 25 | 30-40h | 4-5 | Week 3-4 |
| Phase 6 (Integration) | 7 | 8-12h | 1-2 | Week 4-5 |
| **TOTAL** | **80** | **75-102h** | **11-16 SP** | **4-5 weeks** |

### Effort by Action Type

| Action | Lessons | Tasks | Est. Hours | % of Total |
|--------|---------|-------|-----------|-----------|
| Minor Update (30-35% new) | 2 (L2, L5) | 13 | 7-9h | 9% |
| Major Update (40-60% new) | 2 (L1, L4) | 15 | 11-15h | 15% |
| Complete Rewrite (70-90% new) | 1 (L3) | 10 | 8-10h | 10% |
| Create from Scratch | 3 (L6, L7, L8) | 25 | 26-36h | 38% |
| Integration & Polish | 1 (README + validation) | 10 | 11-16h | 17% |
| Setup & Archive | 2 files | 3 | 3-4h | 4% |
| **TOTAL** | **8 lessons + README** | **80** | **75-102h** | **100%** |

### Independent Test Criteria (by User Story)

- **US1 (Navigation)**: Student conducts 3 AI conversations about file system without typing commands
- **US2 (Safety + File Ops)**: Student directs AI to delete folder with all 5 safety steps; organizes 20 files safely
- **US3 (Configuration)**: Student creates `.env`, adds API key, configures `.gitignore` via conversation
- **US4 (Orchestration)**: Student directs AI to create 10 git worktrees, supervises, validates all complete
- **US5 (Comprehension)**: Student reads complex command and explains in plain English within 2 minutes

---

## Implementation Strategy

**Week 1**: Setup (Phase 1) + Foundation Lessons 1-2 (Phase 2)
**Week 2**: Command Comprehension Lesson 3 (Phase 3) + Start File Ops/Config (Phase 4)
**Week 3**: Complete File Ops/Config Lessons 4-5 (Phase 4) + Start Orchestration (Phase 5)
**Week 4**: Complete Orchestration Lessons 6-8 (Phase 5) + Integration (Phase 6)
**Week 5**: Final polish, README rewrite, Docusaurus validation, publication

**Suggested MVP Scope** (if time-constrained):
- **MVP = Lessons 1-5 only** (Navigation, Safety, Comprehension, File Ops, Config)
- Defer Lessons 6-8 (Orchestration) to Phase 2 release
- Rationale: First 5 lessons cover P1 and P2 user stories. Orchestration (P3) is advanced and can follow.

---

## Definition of Done

**Lesson-Level**:
- [ ] All sections written per plan.md
- [ ] Real dialogue transcripts collected (2-3 minimum)
- [ ] Checkpoint exercises created with validation criteria
- [ ] "Try With AI" section (2-4 prompts)
- [ ] YAML frontmatter complete (skills, CEFR, duration, Bloom's)
- [ ] Cognitive load validated (≤5 A1, ≤7 A2, ≤10 B1)
- [ ] Reading level Grade 7-8
- [ ] Cross-references working
- [ ] Docusaurus renders correctly
- [ ] Peer review complete

**Chapter-Level**:
- [ ] All 8 lessons meet lesson-level DoD
- [ ] README.md rewritten with AI-native philosophy
- [ ] All cross-references validated
- [ ] Docusaurus full build succeeds
- [ ] Final proofread (voice, tone, grammar)
- [ ] Accessibility validated
- [ ] US1-US5 acceptance tests passed
- [ ] Technical + pedagogical sign-off

---

## Revision History

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| 2025-11-07 | 1.0 | Refined task breakdown with existing vs. new lesson mapping | Claude Code (Main Orchestrator) |
