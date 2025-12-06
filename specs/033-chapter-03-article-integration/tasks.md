# Tasks: Chapter 3 Lesson 6 Article Integration

**Feature Branch**: `033-chapter-03-article-integration`
**Input**: Design documents from `specs/033-chapter-03-article-integration/`
**Prerequisites**: plan.md (required), spec.md (required)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2)
- Include exact file paths in descriptions

---

## Phase 1: Setup & Research

**Purpose**: Verify article source and extract content for integration

- [X] T001 Verify article source "AI Is Quietly Creating Millionaires" by Simeon Ivanov on Entrepreneur.com. Confirm public availability and capture URL.
- [X] T002 Extract core thesis from article: intelligence as asset class, systems-based wealth, early adopter advantage, defensibility through complexity.
- [X] T003 Identify 2-3 key quotes from article that support "domain expertise creates defensible moat" argument.
- [X] T004 Analyze article tone and develop reframing strategy from entrepreneurial (wealth-building) to educational (defensibility-understanding).
- [X] T005 Read existing Lesson 6 content in `book-source/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/06-three-requirements.md` to identify exact insertion point (after Path 2, before Choosing Your Path).

---

## Phase 2: User Story 1 - Students Connect Domain Expertise to Economic Value (Priority: P1)

**Goal**: Students understand domain expertise creates defensible competitive advantage, not just technical capability.

**Independent Test**: Student can explain in 2-3 sentences WHY domain expertise creates defensible advantage (economic moat concept).

### Implementation Tasks

- [X] T006 [US1] Develop subsection outline for "Why Intelligence Is the New Competitive Asset" covering: (1) hook connecting to Lesson 3, (2) intelligence replaces effort, (3) both paths defensible, (4) why missing Req 1 is fatal.
- [X] T007 [US1] Write opening hook (50-75 words) connecting to Lesson 3 super orchestrator economics and establishing economic stakes of domain expertise.
- [X] T008 [US1] Write Core Insight 1 (75-100 words): Intelligence replaces effort as competitive advantage. Use article insights about early adopters and reframe in educational language.
- [X] T009 [US1] Write Core Insight 2 (100-125 words): Both paths (fine-tuning AND vertical intelligence) defensible because they embody accumulated knowledge. Explain why replication takes months.
- [X] T010 [US1] Write Core Insight 3 (50-75 words): Why missing Requirement 1 is fatal. Generic AI has no moat, competitor replicates in weeks.
- [X] T011 [US1] Write closing paragraph (25-50 words) connecting economic reasoning to student's path choice between fine-tuning and vertical intelligence.
- [X] T012 [US1] Insert complete subsection into `book-source/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/06-three-requirements.md` after Path 2 section (line ~125), before Choosing Your Path section.
- [X] T013 [US1] Perform cognitive load audit: Count new concepts introduced (target: ≤2). Verify total lesson concepts remain ≤7 (A2 limit).
- [X] T014 [US1] Perform A1-A2 language audit: Check for unexplained jargon, ensure vocabulary accessible, verify sentences are simple/compound (not complex).
- [X] T015 [US1] Verify explicit cross-reference to Lesson 3 super orchestrators exists in subsection text.

### Validation Tasks

- [X] T016 [P] [US1] Run constitutional compliance check: Grep for forbidden meta-commentary patterns (`What to notice`, `AI learns from`, `AI as Teacher`). Expect zero matches.
- [X] T017 [P] [US1] Verify Layer 1 Manual Foundation maintained: No "Try With AI" prompts within subsection, direct explanation approach used.
- [X] T018 [P] [US1] Verify word count: Subsection is 300-500 words (use word processor check).
- [X] T019 [US1] Test acceptance scenario: Have reviewer (or AI simulation) read subsection and answer "Why does competitor struggle to replicate domain intelligence?" Expect answer mentioning: months to build knowledge + embedded in system + competitive moat.

---

## Phase 3: User Story 2 - Summary Reflects Article Insights (Priority: P2)

**Goal**: Update summary file to include intelligence-as-moat concept for student quick reference.

**Independent Test**: Summary includes intelligence-as-moat and connects to Path 1/Path 2 defensibility.

### Implementation Tasks

- [X] T020 [P] [US2] Read existing summary file `book-source/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/06-three-requirements.summary.md` to identify "Key Mental Models" or "Domain Expertise" section.
- [X] T021 [US2] Draft 1-2 sentences for summary capturing: "Domain expertise (via fine-tuning OR vertical intelligence) creates competitive moat because intelligence is the new asset class. Competitors cannot replicate months of accumulated domain knowledge quickly."
- [X] T022 [US2] Insert summary update into appropriate section of `06-three-requirements.summary.md` (likely under "Domain Expertise" or "Key Mental Models").
- [X] T023 [US2] Verify summary language matches lesson's new subsection tone (educational, not entrepreneurial).

### Validation Tasks

- [X] T024 [P] [US2] Verify summary mentions "intelligence as competitive asset" or "intelligence as moat" explicitly.
- [X] T025 [US2] Test acceptance scenario: Have reviewer read summary and confirm they understand "intelligence = defensibility" connection without reading full lesson.

---

## Phase 4: Final Quality Assurance

**Purpose**: Cross-cutting validation and polish

- [X] T026 [P] Run full constitutional compliance validation across both modified files: All 7 principles + meta-commentary prohibition.
- [X] T027 [P] Verify article attribution: Subsection cites "Simeon Ivanov, Entrepreneur.com, 2025" or includes hyperlink to article source.
- [X] T028 [P] Verify no structural changes: Chapter still has 7 lessons + quiz, Lesson 6 still has 3 requirements sections.
- [X] T029 Perform final read-through: Ensure subsection flows naturally from Path 2 into Choosing Your Path without jarring tone shift.
- [X] T030 Mark lesson duration: Update lesson metadata if duration changes from 12 minutes to 14 minutes (due to 300-500 word addition).

---

## Dependencies & Execution Order

### User Story Dependencies
- **US1 (P1)** is independent, can start immediately after Phase 1 research
- **US2 (P2)** can run in parallel with US1 (different file), but logically follows US1 for tone consistency

### Suggested Execution
1. **Phase 1** (Setup): T001-T005 (sequential, 1-2 hours)
2. **Phase 2** (US1): T006-T019 (sequential within story, 3-4 hours)
3. **Phase 3** (US2): T020-T025 (can overlap with US1 T012 onwards, 30-45 min)
4. **Phase 4** (QA): T026-T030 (after US1+US2 complete, 30-45 min)

### Parallel Opportunities
- T016, T017, T018 (US1 validation) can run parallel after T012
- T020 (US2 read) can run parallel with T006-T011 (US1 writing)
- T024 (US2 validation) can run parallel with T016-T018
- T026, T027, T028 (final QA) can run parallel

**Total Estimated Time**: 5-8 hours

---

## Implementation Strategy

**MVP Scope**: User Story 1 only (subsection integration). This delivers core educational value.

**Incremental Delivery**:
1. **Sprint 1**: US1 (subsection) → Students get enhanced understanding
2. **Sprint 2**: US2 (summary) → Students get quick-reference reinforcement

**Testing Approach**: Acceptance scenarios embedded in task validation steps. No separate test suite needed (educational content, not code).

---

## Success Metrics

| Metric | Target | Validation Task |
|--------|--------|-----------------|
| Cognitive Load | ≤2 new concepts, total ≤7 | T013 |
| Word Count | 300-500 words | T018 |
| Language Level | A1-A2 appropriate | T014 |
| Constitutional | All 7 principles pass | T016, T026 |
| Cross-Reference | Explicit Lesson 3 mention | T015 |
| Summary Update | Intelligence-as-moat present | T024 |
| Article Integration | Authentic, substantive | T019, T025 |

---

## File Modification Summary

**Files Modified** (2 total):
1. `book-source/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/06-three-requirements.md` — Add 300-500 word subsection
2. `book-source/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/06-three-requirements.summary.md` — Add 1-2 sentences

**Files NOT Modified**: All other chapter/lesson files remain unchanged. No structural dependencies.

---

## Notes for Implementer

**Tone Calibration**: Article is entrepreneurial ("become a millionaire"). Lesson is educational ("understand defensibility"). Extract insights, not hype.

**Constitutional Compliance**: This is Layer 1 (Manual Foundation). No "Try With AI" prompts within subsection. Direct explanation only.

**Concept Inventory**: Existing lesson has 5 concepts. New subsection adds max 2. Total 7 (at A2 ceiling, acceptable because serves core objective).

**Cross-Reference Requirement**: MUST explicitly mention Lesson 3 super orchestrators to show continuity. Not optional.
