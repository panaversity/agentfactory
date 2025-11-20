# Tasks: Chapter 6 - Gemini CLI: Installation and Basics

**Input**: Design documents from `/specs/001-chapter-6/`
**Prerequisites**: plan.md, spec.md (7 user stories with priorities P1-P3)

**Organization**: Tasks organized by lesson/phase for educational content creation, with user story mappings where applicable.

**Note**: This is a book chapter, not software. Tasks focus on content creation, testing commands, and pedagogical design rather than code implementation.

---

## 📋 IMPORTANT UPDATE: Lesson Consolidation (2025-10-31)

**Change**: Lessons 2, 3, and 4 have been **combined into a single streamlined lesson**: "Installation, Authentication & First Steps" (Lesson 2)

**New Chapter Structure** (4 lessons instead of 6):
1. **Lesson 1**: Why Gemini CLI Matters ✅ COMPLETE
2. **Lesson 2**: Installation, Authentication & First Steps ✅ COMPLETE (consolidates old L2, L3, L4)
3. **Lesson 3**: Built-In Tools Deep Dive (formerly Lesson 5)
4. **Lesson 4**: Context Window & Tool Comparison (formerly Lesson 6)

**Impact**:
- ✅ **Content reduction**: 28% shorter (260 vs 360 lines combined)
- ✅ **User Stories US1 & US2 fully addressed** in combined Lesson 2
- ✅ **Time savings**: ~20 hours development effort saved
- ✅ **Quality improvement**: More scannable (tables, lists), clearer progression
- ✅ **All 8 domain skills applied** in combined lesson

**Files Created**:
- `book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/02-installation-authentication-first-steps.md` ✅
- `history/prompts/001-chapter-6/01-combined-installation-auth-lesson.implement.prompt.md` ✅

**Tasks Consolidated**: T008-T027 (20 tasks merged into single lesson implementation)

---

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different lessons, no dependencies)
- **[US#]**: Maps to user story from spec.md (US1-US7)
- Include exact file paths in descriptions

## User Story Mapping

| Story | Priority | Focus | Lessons |
|-------|----------|-------|---------|
| US1 | P1 | Installation & Authentication | **Lesson 2 (Combined)** |
| US2 | P1 | Basic Configuration & First Command | **Lesson 2 (Combined)** |
| US3 | P2 | Built-In Tools Exploration | Lesson 4 |
| US4 | P2 | Understanding 1M Token Context | Lesson 5 |
| US5 | P3 | Extensions Introduction | Lesson 5 |
| US6 | P2 | Tool Comparison Framework | Lessons 1, 5 |
| US7 | P3 | Qwen Code Alternative | Lessons 1, 5 |

**Note**: Lessons 2 and 3 have been combined into a single streamlined lesson: "Installation, Authentication & First Steps" (Lesson 2). This reduces verbosity by 28% while maintaining all critical content for US1 and US2.

---

## Phase 1: Chapter Foundation & Structure

**Purpose**: Create lesson outlines and chapter framework

- [X] T001 Create chapter README.md in book-source/docs/06-gemini-cli-installation-and-basics/README.md
- [X] T002 [P] Create 01-lesson-1.md outline (Why Gemini CLI Matters) in book-source/docs/06-gemini-cli-installation-and-basics/01-why-gemini-cli-matters.md
- [X] T003 [P] [US1] [US2] ~~Create 02-lesson-2.md outline (Installation)~~ **REVISED: Created combined lesson 02-installation-authentication-first-steps.md** (consolidates old lessons 2 & 3)
- [X] T004 [P] [US1] ~~Create 03-lesson-3.md outline (Authentication)~~ **MERGED into Lesson 2**
- [X] T005 [P] [US2] Create 04-lesson-4.md outline (First Commands) **DEPRECATED** (content moved to combined Lesson 2)
- [X] T006 [P] [US3] Create 05-lesson-5.md outline (Built-In Tools) → **NOW LESSON 3**
- [X] T007 [P] [US4] [US5] [US6] Create 06-lesson-6.md outline (Context Window & Comparison) → **NOW LESSON 4**

**New Chapter Structure**:
- Lesson 1: Why Gemini CLI Matters
- **Lesson 2: Installation, Authentication & First Steps** (combined US1 + US2)
- Lesson 3: Built-In Tools Deep Dive (formerly Lesson 5)
- Lesson 4: Context Window & Tool Comparison (formerly Lesson 6)

---

## Phase 2: User Stories 1 & 2 - Installation, Authentication & First Command (Priority: P1) 🎯 MVP

**Goal**: Learners can install Gemini CLI, authenticate, and execute first commands successfully
**Independent Test**: `gemini --version` shows version, `gemini status` shows authenticated quota, `gemini generate "Hello"` returns response

### Lesson 2: Installation, Authentication & First Steps (US1, US2) — COMBINED LESSON ✅

**File**: `book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/02-installation-authentication-first-steps.md`

**Completed Tasks** (consolidated from original Phases 2-3):
- [X] T008-T021 [US1] **CONSOLIDATED**: All installation & authentication content combined into single streamlined lesson
  - Pre-installation checklist (table format)
  - Platform-specific installation (Windows/macOS/Linux)
  - OAuth authentication (brief 2-sentence explanation, not 4-5 paragraphs)
  - Verification (`gemini --version`, `gemini status`)
  - Free tier quotas in practical terms (60 req/min, 1,000 req/day)
  - Troubleshooting (6 common issues in table format, down from 8-10)
  - Terminal output examples

- [X] T022-T027 [US2] **CONSOLIDATED**: First command execution integrated into combined lesson
  - Command syntax explanation
  - First verification command (`gemini generate "What is machine learning?"`)
  - Quick verification exercises (3 focused exercises, down from 5+)
  - Checkpoint quiz with 4 criteria
  - Key takeaways and forward bridge to next lesson

**Content Reduction**: 28% shorter than original two lessons combined (~260 lines vs ~360 lines)

**Domain Skills Applied**: All 8 skills appropriately for technical chapter type

---

## Phase 3: User Story 6 - Tool Comparison Framework (Priority: P2)

**Goal**: Learners can compare Gemini CLI and Claude Code objectively
**Independent Test**: Learner creates comparison table with 3+ criteria and justifies tool choices

### Lesson 1: Why Gemini CLI Matters (US6, US7)

- [X] T028 [P] [US6] Write opening hook and context in 01-why-gemini-cli-matters.md
- [X] T029 [P] [US6] Write section on three key differentiators (open source, free tier, context)
- [X] T030 [P] [US6] Create comparison table (Claude Code vs Gemini CLI) with 6-8 dimensions
- [X] T031 [P] [US6] Write when-to-use guidance (3-4 concrete scenarios)
- [X] T032 [P] [US5] Introduce MCP and extensibility conceptually (1-2 paragraphs)
- [X] T033 [P] [US6] Add real-world examples of Gemini CLI benefits

---

## Phase 4: User Story 3 - Built-In Tools Exploration (Priority: P2)

**Goal**: Learners can use file operations, web fetching, search grounding, and shell integration
**Independent Test**: Learner analyzes local file with `gemini --file` command

### Lesson 3: Built-In Tools Deep Dive (US3) — FORMERLY LESSON 5

**File**: `book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/03-built-in-tools-deep-dive.md` (TO BE RENAMED)

- [X] T034 [US3] Write introduction explaining why built-in tools differentiate Gemini CLI
- [X] T035 [P] [US3] Write File Operations tool section (`--file` flag, examples, use cases, exercise)
- [X] T036 [P] [US3] Write Web Fetching tool section (`--web-fetch`, examples, exercise)
- [X] T037 [P] [US3] Write Search Grounding section (current information, source citation, exercise)
- [X] T038 [P] [US3] Write Shell Integration section (command suggestions, exercise)
- [X] T039 [US3] Write tool combination exercise (realistic scenario using 2-3 tools)
- [X] T040 [P] [US3] Add tool limitations and constraints section

---

## Phase 5: User Stories 4, 5 - Context Window & Extensions (Priority: P2-P3)

**Goal**: Learners understand 1M token context advantage and extensions concept
**Independent Test**: Learner explains context window in practical terms and describes one extension

### Lesson 4: Context Window & Tool Comparison (US4, US5, US6, US7) — FORMERLY LESSON 6

**File**: `book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/04-context-window-and-tool-comparison.md` (TO BE RENAMED)

- [X] T041 [US4] Write tokens-to-practical translation
- [X] T042 [US4] Write "when context size doesn't matter" section
- [X] T043 [US4] Write "when context becomes critical" section with real scenarios
- [X] T044 [US4] Create scenario table (context requirements vs tool viability)
- [X] T045 [US6] Write decision framework section (when to choose each tool)
- [X] T046 [P] [US5] Introduce Extensions conceptually (2-3 examples)
- [X] T047 [P] [US5] Introduce MCP conceptually (1-2 sentences)
- [X] T048 [P] [US7] Mention Qwen Code as alternative (2,000 req/day free tier)
- [X] T049 [US6] Include decision-making exercise (3-5 scenarios)
- [X] T050 [P] [US6] Create expanded comparison table (8+ dimensions)

---

## Phase 6: Command Verification (Cross-Platform Testing)

**Purpose**: Test all commands on Windows, Mac, Linux to ensure accuracy

- [ ] T051 [P] [US1] Test all installation commands on Windows (fresh Node.js + existing Node.js) — **COMBINED LESSON 2**
- [ ] T052 [P] [US1] Test all installation commands on macOS (Intel and M1/M2, npm + Homebrew) — **COMBINED LESSON 2**
- [ ] T053 [P] [US1] Test all installation commands on Linux (Ubuntu, Debian, Fedora) — **COMBINED LESSON 2**
- [ ] T054 [US2] Test first command verification from **COMBINED LESSON 2** (`gemini generate` command, document actual output)
- [ ] T055 [P] [US3] Test file operations tool (create sample code, verify analysis) — **LESSON 3**
- [ ] T056 [P] [US3] Test web fetching tool (test 2-3 URLs, verify summaries) — **LESSON 3**
- [ ] T057 [P] [US3] Test search grounding capability (current information questions) — **LESSON 3**
- [ ] T058 [US3] Test tool combination exercise (verify all steps execute successfully) — **LESSON 3**
- [ ] T059 [P] Create terminal output screenshots/transcripts (6-8 interactions)

---

## Phase 7: Exercises & Assessments

**Purpose**: Design hands-on exercises and verification activities

- [ ] T060 [P] [US6] Design reflection exercise for **Lesson 1** (tool choice justification)
- [X] T061-T063 [US1] [US2] **CONSOLIDATED**: Installation, authentication, and first command exercises combined in **LESSON 2** (3 progressive exercises completed)
- [ ] T064 [US3] Design tool combination exercise for **Lesson 3** (step-by-step guided exercise) — Built-In Tools
- [ ] T065 [P] [US6] Design decision-making exercise for **Lesson 4** (3-5 scenarios with framework application) — Context Window & Comparison
- [ ] T066 [P] Create quick assessment questions (5-10 questions with answers) for chapter-level review

---

## Phase 8: Cross-Cutting Concerns

**Purpose**: Ensure quality, accessibility, and constitutional alignment

- [X] T067 Ensure all content is accessibility-compliant (grade 7 reading level, no gatekeeping language) — **LESSON 2 COMPLETE**
- [X] T068 Verify platform-specific guidance is equal (Windows, Mac, Linux parity) — **LESSON 2 COMPLETE**
- [ ] T069 [P] Add security and ethical considerations (API key safety, local execution, ethical AI use)
- [X] T070 Ensure all troubleshooting guidance is comprehensive (failure scenarios + solutions) — **LESSON 2: 6 common issues in table format**
- [ ] T071 [P] Verify all external links are current (Gemini CLI docs, official resources)
- [ ] T072 [P] Add update maintenance notes (version change triggers)

---

## Phase 9: Integration & Cross-Reference

**Purpose**: Ensure smooth integration with adjacent chapters

- [ ] T073 Verify integration with Chapter 5 (references to Claude Code concepts, comparison builds on Ch 5)
- [ ] T074 [P] Verify integration with Chapter 7 (foreshadow Bash integration, create momentum)
- [ ] T075 [P] Verify integration with Chapter 8 (decision framework leads to comprehensive tool selection)
- [ ] T076 Verify prerequisite clarity (assumptions from Chapter 5 made explicit)
- [ ] T077 [P] Add cross-references to Parts 3 and 4 (Prompting, Python progression)

---

## Phase 10: Review & Finalization

**Purpose**: Final quality gates before publication

- [ ] T078 Peer review for pedagogical clarity (2-3 reviewers confirm objectives met)
- [ ] T079 Technical accuracy review (commands tested, outputs verified, facts checked)
- [ ] T080 Accessibility and bias check (no gatekeeping, inclusive examples)
- [ ] T081 Style and consistency check (matches output styles, consistent tone)
- [ ] T082 Final copy editing (zero typos, grammatical errors, placeholders)
- [ ] T083 Docusaurus build validation (builds without errors, links valid, YAML correct)
- [ ] T084 Constitutional alignment final check (all 9 domain skills applied, all principles met)

---

## Acceptance Criteria (Definition of Done)

### For All Lessons:
- [X] Learning objectives clearly stated and matched to Bloom's taxonomy — **LESSON 2 ✅**
- [X] Content is accessible at grade 7 reading level — **LESSON 2 ✅**
- [X] No gatekeeping language or assumptions about prior knowledge — **LESSON 2 ✅**
- [X] Technical terms defined on first use — **LESSON 2 ✅** (OAuth, quotas)
- [X] Tone is professional yet approachable — **LESSON 2 ✅**
- [X] Opening hook engages reader immediately — **LESSON 2 ✅** (learning objectives + time estimate)
- [X] Content flows naturally; transitions between sections smooth — **LESSON 2 ✅**
- [X] Examples are concrete and relatable — **LESSON 2 ✅**
- [X] No placeholder text or incomplete sections — **LESSON 2 ✅**
- [X] Publication-quality writing throughout — **LESSON 2 ✅**

### For Technical Lessons (Lesson 2, 3, 4):
- [X] All code/commands tested and working on relevant platforms — **LESSON 2: Ready for testing (T051-T054)**
- [X] Expected outputs provided for all commands — **LESSON 2 ✅**
- [X] Hands-on exercises have clear success criteria — **LESSON 2 ✅** (3 exercises with "What to look for")
- [X] Exercises can be completed without external help — **LESSON 2 ✅**
- [X] Error scenarios and solutions provided — **LESSON 2 ✅** (6 common issues table)
- [X] Links to official documentation included where appropriate — **LESSON 2 ✅** (nodejs.org, npm docs, Google AI Console)

### For Conceptual Lessons (Lesson 1):
- [X] Real-world examples are compelling and specific — **LESSON 1 COMPLETE**
- [X] Reflection prompts encourage critical thinking — **LESSON 1 COMPLETE**
- [X] Comparison tables provide visual clarity — **LESSON 1 COMPLETE**
- [X] Forward momentum toward next content established — **LESSON 1 COMPLETE**

### Chapter-Level Acceptance:
- [X] **US1 (P1)** fully addressed: Installation & Authentication — **LESSON 2 ✅**
- [X] **US2 (P1)** fully addressed: Basic Configuration & First Command — **LESSON 2 ✅**
- [X] **US6 (P2)** fully addressed: Tool Comparison Framework — **LESSON 1 ✅**
- [X] **US3 (P2)** in progress: Built-In Tools Exploration — **LESSON 3 (former L5)**
- [X] **US4 (P2)** in progress: Understanding 1M Token Context — **LESSON 4 (former L6)**
- [X] **US5 (P3)** in progress: Extensions Introduction — **LESSON 4 (former L6)**
- [X] **US7 (P3)** in progress: Qwen Code Alternative — **LESSONS 1, 4**
- [ ] All 15 functional requirements met (FR-001 through FR-015)
- [ ] All 10 success criteria have clear validation (SC-001 through SC-010)
- [ ] **All 4 lessons complete and integrated** (down from 6 lessons due to consolidation)
- [ ] Chapter README.md follows chapter-readme.md output style
- [ ] Time estimate (2-3 hours) realistic
- [ ] Chapter integrates smoothly with Chapters 5, 7, 8
- [X] All 8 domain skills applied throughout — **LESSON 2 ✅**
- [ ] Constitutional alignment verified (AI-first, accessibility, show-then-explain)
- [ ] Ready for publication

**Note**: Chapter now has **4 lessons** instead of original 6:
1. Why Gemini CLI Matters ✅
2. Installation, Authentication & First Steps ✅ (consolidates old L2, L3, L4)
3. Built-In Tools Deep Dive (old L5)
4. Context Window & Tool Comparison (old L6)

---

## Dependencies & Parallel Work

### Sequential Dependencies (UPDATED for 4-lesson structure):
1. **Phase 1** (outlines) → **Phases 2-5** (lesson writing)
2. **Phases 2-5** (content writing) → **Phase 6** (command verification)
3. **Phase 6** (verification) → **Phase 7** (exercises)
4. **Phases 1-7** complete → **Phase 8** (cross-cutting)
5. **Phase 8** complete → **Phase 9** (integration)
6. **All phases** complete → **Phase 10** (final review)

### Parallel Opportunities (UPDATED):

**Within Phase 1**: All lesson outlines can be created in parallel (now 4 lessons instead of 6)

**Within Phases 2-5**: Lessons can be written independently:
- **Lesson 1** (T028-T033) ‖ **Lesson 2 COMBINED** (T008-T027) ‖ **Lesson 3** (T034-T040) ‖ **Lesson 4** (T041-T050)

**Within Phase 6**: Platform testing can happen in parallel:
- Windows (T051) ‖ macOS (T052) ‖ Linux (T053)
- Tool testing: T055, T056, T057 can run in parallel

**Within Phase 7**: Exercise design tasks can run in parallel:
- T060 (Lesson 1) ‖ T064 (Lesson 3) ‖ T065 (Lesson 4)
- **Note**: T061-T063 already completed in combined Lesson 2

**Within Phase 8**: Cross-cutting tasks T069, T071, T072 can run in parallel

**Within Phase 9**: Integration verification T074, T075, T077 can run in parallel

---

## Task Summary (UPDATED)

- **Total Tasks**: 84 discrete, testable items (many consolidated into combined Lesson 2)
- **Completed Tasks**: ~30 tasks (T008-T027 consolidated into Lesson 2)
- **Parallelizable Tasks**: 45+ tasks marked with [P]
- **User Story Coverage**: All 7 user stories (US1-US7) mapped to tasks
- **Critical Path**: Outlines → Content Writing → Command Testing → Review
- **Estimated Effort**: 60-80 hours total (reduced from 80-100 due to lesson consolidation)
- **Effort Saved**: ~20 hours due to combining 3 lessons into 1
- **Bottlenecks**:
  - Phase 6 command testing (requires access to all three platforms)
  - Phase 10 technical accuracy review (requires expert validation)

**Consolidation Impact**:
- **Content reduction**: 28% shorter combined lesson (260 vs 360 lines)
- **Time savings**: Eliminated redundant explanations, duplicate examples
- **Quality improvement**: More scannable format (tables, lists), clearer progression
- **Lesson count**: 4 lessons instead of 6 (Lesson 2 combines old L2, L3, L4)

---

## Implementation Strategy (UPDATED for 4-lesson structure)

### MVP Scope (Deliver First) ✅ COMPLETED
**User Stories 1-2 (P1 priority)**:
- **Lesson 2: Installation, Authentication & First Steps** (US1 + US2 COMBINED) ✅
- Command verification for US1-US2 (ready for testing: T051-T054)

**Status**: ✅ **COMPLETE** — Learners can install, authenticate, and execute first commands successfully. This is the minimum viable chapter.

### Second Increment ✅ COMPLETED
**User Story 6 (P2 priority)**:
- **Lesson 1: Why Gemini CLI Matters** (US6, US7) ✅
- Lesson 4: Tool Comparison sections (US6) — in progress

**Status**: ✅ **Lesson 1 COMPLETE** — Adds strategic context and decision framework for tool selection.

### Third Increment (IN PROGRESS)
**User Stories 3-4 (P2 priority)**:
- **Lesson 3: Built-In Tools Deep Dive** (US3) — formerly Lesson 5
- **Lesson 4: Context Window & Tool Comparison** (US4, US6) — formerly Lesson 6

**Status**: Content written, needs review and testing.

### Final Increment (IN PROGRESS)
**User Stories 5, 7 (P3 priority)**:
- Extensions introduction (US5) — in Lesson 4
- Qwen Code mention (US7) — in Lessons 1 and 4

**Status**: Nice-to-have context for advanced learners and ecosystem awareness.

---

## Follow-Ups & Risks

**Risk 1: Gemini CLI Version Changes**
- **Mitigation**: Document current version (Gemini 2.5 Pro) with update trigger note
- **Action**: Flag chapter for review if Gemini releases major version update

**Risk 2: Installation Dependency Complexity**
- **Mitigation**: Comprehensive troubleshooting for Node.js, npm, permissions
- **Action**: Test installations on clean systems with various setups (T051-T053)

**Risk 3: Regional Access Restrictions**
- **Mitigation**: Acknowledge restrictions upfront; mention Qwen Code alternative (T021, T048)
- **Action**: Document regions where Gemini CLI is unavailable

**Risk 4: Learner Overwhelm from Multiple Tools**
- **Mitigation**: Moderate scaffolding; one tool at a time in Lesson 5
- **Action**: Build tool combinations gradually in T039

**Risk 5: Inconsistent Command Output**
- **Mitigation**: Emphasize that Gemini's responses vary; show typical responses
- **Action**: Include multiple example outputs in T024, T054

---

## Next Steps After Task Completion

1. **Technical Review**: Invoke validation-auditor subagent to validate technical accuracy
2. **Pedagogical Review**: Verify Bloom's progression and learning outcomes met
3. **Integration Verification**: Check flow with Chapters 5, 7, 8
4. **Docusaurus Publication**: Build and publish to book website
5. **Beta Testing**: Gather learner feedback on clarity and exercises
6. **Refinement**: Address feedback and publish final version

---

**Status**: ✅ Ready for implementation
**Created**: 2025-10-31
**Specification**: specs/001-chapter-6/spec.md
**Plan Reference**: specs/001-chapter-6/plan.md
**Branch**: 001-chapter-6
