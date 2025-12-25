# Tasks: Chapter 3 Digital FTE Strategy Restructure

**Input**: Design documents from `/specs/001-ch3-fte-strategy/`
**Prerequisites**: spec.md (required), plan.md (required)
**Branch**: `001-ch3-fte-strategy`
**GitHub Issue**: #386

**Organization**: Tasks grouped by user story for independent implementation. This is content work (educational chapter), not engineering.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US4)
- Include exact file paths in descriptions

## Path Conventions

**Chapter Location**: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/`
- **Current**: `03-billion-dollar-ai/`
- **Target**: `03-digital-fte-strategy/`

**Content Source**: `/Agent_Factory_Building_Digital_FTEs.md`

---

## Phase 1: Setup (Structural Changes)

**Purpose**: Rename chapter folder and establish structure for 11 lessons

- [ ] T001 Search codebase for references to `03-billion-dollar-ai` path using `grep -r "03-billion-dollar-ai" apps/` to identify cross-references that need updating
- [ ] T002 Rename chapter folder from `apps/learn-app/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/` to `apps/learn-app/docs/01-Introducing-AI-Driven-Development/03-digital-fte-strategy/`
- [ ] T003 Update any cross-references found in T001 to use new path `03-digital-fte-strategy/`
- [ ] T004 Verify build passes after rename with `pnpm nx build learn-app`

**Checkpoint**: Chapter folder renamed, all references updated, build passes

---

## Phase 2: Foundational (Chapter README)

**Purpose**: Update chapter README before lesson work begins

- [ ] T005 Read current `apps/learn-app/docs/01-Introducing-AI-Driven-Development/03-digital-fte-strategy/README.md`
- [ ] T006 Update README.md with new chapter title "Chapter 3: The Digital FTE Strategy"
- [ ] T007 Update README.md lesson table to reflect 11 lessons with new titles:
  - Lesson 1: Your Expertise as Product
  - Lesson 2: Snakes and Ladders
  - Lesson 3: The FTE Advantage
  - Lesson 4: Vertical Intelligence
  - Lesson 5: Monetization Models Part 1
  - Lesson 6: Monetization Models Part 2
  - Lesson 7: PPP Market Entry Strategy
  - Lesson 8: Three Requirements
  - Lesson 9: When NOT to Use AI Agents
  - Lesson 10: Pause and Reflect
  - Lesson 11: Chapter Quiz
- [ ] T008 Update README.md learning objectives to align with spec success criteria

**Checkpoint**: README ready, lesson structure defined

---

## Phase 3: User Story 1 - Learn Monetization Strategy (Priority: P1) MVP

**Goal**: Reader can articulate which monetization model fits their domain with concrete pricing example

**Independent Test**: Complete Lessons 3, 5, 6 and verify monetization decision framework is actionable

### Implementation for User Story 1

**Lesson 3: The FTE Advantage (UPDATE)**

- [ ] T009 [US1] Read current `03-super-orchestrators.md` to understand baseline
- [ ] T010 [US1] Read Agent Factory slides Part 12 (Pages 70-75) for FTE economics content
- [ ] T011 [US1] Create new file `03-the-fte-advantage.md` in `apps/learn-app/docs/01-Introducing-AI-Driven-Development/03-digital-fte-strategy/`
- [ ] T012 [US1] Write Lesson 3 content per plan.md specifications:
  - REMOVE: FTE comparison table (now in Ch1 L10)
  - REMOVE: "Super Orchestrator" terminology
  - ADD: Sales presentation tips from slides (Pages 72-73)
  - ADD: ROI calculation methods
  - ADD: "Try With AI" section: Create Your ROI Argument
  - Target: 2000-2200 words
- [ ] T013 [US1] Delete old file `03-super-orchestrators.md`

**Lesson 5: Monetization Models Part 1 (NEW)**

- [ ] T014 [P] [US1] Read Agent Factory slides Part 11 (Pages 65-80) for monetization content
- [ ] T015 [P] [US1] Create new file `05-monetization-models-part-1.md` in chapter folder
- [ ] T016 [US1] Write Lesson 5 content per plan.md specifications:
  - Subscription model deep dive with examples
  - Success Fee model deep dive with examples
  - Decision matrix: subscription vs success fee
  - Digital SDR case study (from slides Pages 72-75)
  - "Try With AI" section: Assess Subscription vs Success Fee
  - Target: 1800-2000 words

**Lesson 6: Monetization Models Part 2 (NEW)**

- [ ] T017 [P] [US1] Create new file `06-monetization-models-part-2.md` in chapter folder
- [ ] T018 [US1] Write Lesson 6 content per plan.md specifications:
  - License model deep dive with examples
  - Marketplace model deep dive with examples
  - Complete 4-model decision matrix
  - Hybrid approaches section
  - "Try With AI" section: Design Your Revenue Model Strategy
  - Target: 1800-2000 words

**Checkpoint**: Monetization lessons complete, reader can select revenue model

---

## Phase 4: User Story 2 - Build Competitive Strategy (Priority: P2)

**Goal**: Reader can map competitive position and plan market entry

**Independent Test**: Complete Lessons 2, 7, 8 and verify frameworks are applicable to reader's domain

### Implementation for User Story 2

**Lesson 2: Snakes and Ladders (MINOR EDIT)**

- [ ] T019 [US2] Read current `02-snakes-and-ladders.md` to assess needed changes
- [ ] T020 [US2] Update terminology in `02-snakes-and-ladders.md`:
  - "Generic AI tools" → "General Agents"
  - "Custom solutions" → "Custom Agents"
- [ ] T021 [US2] Add bridge paragraph to Lesson 5 (monetization depends on competitive layer)
- [ ] T022 [US2] Verify "Try With AI" section exists; update if needed

**Lesson 7: PPP Market Entry Strategy (MINOR EDIT)**

- [ ] T023 [US2] Read current `05-ppp-strategy.md` and Agent Factory slides Part 13-14 (Pages 82-95)
- [ ] T024 [US2] Rename file to `07-ppp-strategy.md` with updated sidebar_position
- [ ] T025 [US2] Update terminology alignment (General Agents, Custom Agents)
- [ ] T026 [US2] Add timeline benchmarks from Agent Factory slides:
  - Phase 1 typical duration (3-6 months)
  - Phase 2 typical duration (6-12 months)
  - Phase 3 launch criteria
- [ ] T027 [US2] Verify "Try With AI" section exists; update if needed

**Lesson 8: Three Requirements (MINOR EDIT)**

- [ ] T028 [US2] Read current `06-three-requirements.md`
- [ ] T029 [US2] Rename file to `08-three-requirements.md` with updated sidebar_position
- [ ] T030 [US2] Add assessment checklist (1-10 scale self-evaluation)
- [ ] T031 [US2] Add mitigation strategies for each requirement gap
- [ ] T032 [US2] Verify "Try With AI" section exists; update if needed

**Checkpoint**: Competitive strategy framework complete

---

## Phase 5: User Story 3 - Understand Agent Skills as Assets (Priority: P2)

**Goal**: Reader can describe SKILL.md format and explain progressive disclosure benefits

**Independent Test**: Complete Lesson 4 and verify SKILL.md specification is clear

### Implementation for User Story 3

**Lesson 4: Vertical Intelligence (MAJOR UPDATE)**

- [ ] T033 [US3] Read current `04-vertical-intelligence.md` to understand baseline
- [ ] T034 [US3] Read Agent Factory slides Part 6 & 9 (Pages 45-55) for Agent Skills content
- [ ] T035 [US3] Read SKILL.md format specification from slides (Page 48)
- [ ] T036 [US3] Update `04-vertical-intelligence.md` per plan.md specifications:
  - KEEP: Paradigm shift insight (code reuse → intelligence reuse)
  - EXPAND: Five-component architecture with detailed examples
  - ADD: SKILL.md format specification (Page 48)
  - ADD: Folder structure diagram (Page 49)
  - ADD: Progressive disclosure and token efficiency (Page 50)
  - ADD: "Try With AI" section: Architect Your Vertical Skills
  - Target: 2200-2400 words

**Checkpoint**: Agent Skills as IP concept complete

---

## Phase 6: User Story 4 - Learn Guardrails and Risk Management (Priority: P3)

**Goal**: Reader can identify red flags in proposed agent deployment

**Independent Test**: Complete Lesson 9 and verify guardrails framework is applicable

### Implementation for User Story 4

**Lesson 9: When NOT to Use AI Agents (NEW)**

- [ ] T037 [US4] Read Agent Factory slides Part 15 (Pages 108-120) for security/compliance content
- [ ] T038 [US4] Create new file `09-when-not-to-use-agents.md` in chapter folder
- [ ] T039 [US4] Write Lesson 9 content per plan.md specifications:
  - Six Common Pitfalls table with fixes
  - Security and Compliance minimums
  - Industry-specific guardrails (HIPAA, SOC 2, PCI, GDPR)
  - Shadow Mode deployment strategy
  - Red Flag Detection Framework
  - "Try With AI" section: Risk-Assessment for Your Proposed Agent
  - Target: 2000-2200 words

**Checkpoint**: Guardrails lesson complete

---

## Phase 7: Foundation Lessons (Layer 1)

**Purpose**: Update Lesson 1 (foundation for all other lessons)

**Lesson 1: Your Expertise as Product (UPDATE)**

- [ ] T040 Read current `01-billion-dollar-question.md` to understand baseline
- [ ] T041 Read Chapter 1 Lesson 10 (`01-agent-factory-paradigm/10-the-digital-fte-vision.md`) to identify content to REMOVE
- [ ] T042 Create new file `01-your-expertise-as-product.md` in chapter folder
- [ ] T043 Write Lesson 1 content per plan.md specifications:
  - REMOVE: Chapter 1 overlap (market opportunity, Digital FTE definition, Five Powers)
  - ADD: Explicit callback to Chapter 1 Lesson 10
  - KEEP: Core question "What does your expertise enable?"
  - ADD: "Try With AI" section: Position Your Expertise
  - Target: 1500-1800 words
- [ ] T044 Delete old file `01-billion-dollar-question.md`
- [ ] T045 Delete old summary file `01-billion-dollar-question.summary.md`

**Checkpoint**: Foundation lesson complete

---

## Phase 8: Synthesis and Assessment

**Purpose**: Update reflection and quiz lessons

**Lesson 10: Pause and Reflect (UPDATE)**

- [ ] T046 Read current `07-pause-and-reflect.md`
- [ ] T047 Rename file to `10-pause-and-reflect.md` with updated sidebar_position
- [ ] T048 Update content with monetization-focused strategic planning prompts:
  - Expertise audit (connects to Lesson 1)
  - Competitive layer assessment (connects to Lesson 2)
  - Revenue model decision (connects to Lessons 5-6)
  - Market entry timeline (connects to Lesson 7)
  - Readiness check (connects to Lesson 8)
  - Risk mitigation strategy (connects to Lesson 9)
- [ ] T049 Update "Try With AI" section: Strategic Planning Session
- [ ] T050 Target: 1500-1800 words

**Lesson 11: Chapter Quiz (UPDATE)**

- [ ] T051 Read current `08_chapter_03_quiz.md`
- [ ] T052 Rename file to `11-chapter-quiz.md` with updated sidebar_position
- [ ] T053 Add 15+ new questions covering:
  - Monetization models (4-5 questions)
  - Competitive positioning (3-4 questions)
  - FTE economics (3-4 questions)
  - Vertical intelligence/SKILL.md (3-4 questions)
  - Guardrails and risk (3-4 questions)
  - Market entry strategy (2-3 questions)
- [ ] T054 Ensure question formats include: multiple choice, scenario-based, short answer

**Checkpoint**: Synthesis and assessment complete

---

## Phase 9: Summary Generation

**Purpose**: Generate .summary.md files for all 11 lessons

- [ ] T055 [P] Generate summary for `01-your-expertise-as-product.md` using summary-generator skill
- [ ] T056 [P] Generate summary for `02-snakes-and-ladders.md` using summary-generator skill
- [ ] T057 [P] Generate summary for `03-the-fte-advantage.md` using summary-generator skill
- [ ] T058 [P] Generate summary for `04-vertical-intelligence.md` using summary-generator skill
- [ ] T059 [P] Generate summary for `05-monetization-models-part-1.md` using summary-generator skill
- [ ] T060 [P] Generate summary for `06-monetization-models-part-2.md` using summary-generator skill
- [ ] T061 [P] Generate summary for `07-ppp-strategy.md` using summary-generator skill
- [ ] T062 [P] Generate summary for `08-three-requirements.md` using summary-generator skill
- [ ] T063 [P] Generate summary for `09-when-not-to-use-agents.md` using summary-generator skill
- [ ] T064 [P] Generate summary for `10-pause-and-reflect.md` using summary-generator skill
- [ ] T065 [P] Generate summary for `11-chapter-quiz.md` using summary-generator skill

**Checkpoint**: All summaries generated

---

## Phase 10: Polish & Validation

**Purpose**: Final quality checks and build validation

- [ ] T066 Run `pnpm nx build learn-app` to verify no build errors
- [ ] T067 Verify all 11 lessons have .summary.md companion files
- [ ] T068 Verify zero content overlap with Chapter 1 by comparing key sections
- [ ] T069 Verify each lesson has "Try With AI" section
- [ ] T070 Verify no meta-commentary exposing pedagogical framework (grep for forbidden patterns)
- [ ] T071 Verify SKILL.md format in Lesson 4 matches Page 48 specification exactly
- [ ] T072 Verify all internal cross-references work (no broken links)
- [ ] T073 Final README.md review for accuracy

**Checkpoint**: Chapter ready for publication

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - start immediately
- **Phase 2 (README)**: Depends on Phase 1 completion
- **Phases 3-8 (Content)**: Depend on Phase 2 completion; can run in parallel
- **Phase 9 (Summaries)**: Depends on all lesson content being complete
- **Phase 10 (Polish)**: Depends on Phase 9 completion

### User Story Dependencies

All user stories can proceed in parallel after Phase 2:

- **US1 (Monetization)**: Lessons 3, 5, 6 - no dependencies on other stories
- **US2 (Competitive Strategy)**: Lessons 2, 7, 8 - no dependencies on other stories
- **US3 (Agent Skills)**: Lesson 4 - no dependencies on other stories
- **US4 (Guardrails)**: Lesson 9 - no dependencies on other stories

### Parallel Opportunities

After Phase 2 completes:
- All user story phases (3-6) can run in parallel with different subagents
- Within Phase 9, all summary generation tasks [P] can run in parallel
- Within Phase 10, validation tasks can run in sequence

---

## Parallel Execution Example

```bash
# After Phase 2 completes, launch 4 content-implementer subagents in parallel:

Subagent 1 (US1): Lessons 3, 5, 6 (Monetization)
Subagent 2 (US2): Lessons 2, 7, 8 (Competitive Strategy)
Subagent 3 (US3): Lesson 4 (Agent Skills)
Subagent 4 (US4): Lesson 9 (Guardrails)

# Then sequentially:
- Phase 7: Lesson 1 (foundation)
- Phase 8: Lessons 10, 11 (synthesis/quiz)
- Phase 9: All summaries in parallel
- Phase 10: Validation
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (rename folder)
2. Complete Phase 2: README update
3. Complete Phase 3: US1 - Monetization lessons (3, 5, 6)
4. **STOP and VALIDATE**: Test monetization framework clarity
5. Continue with remaining stories

### Incremental Delivery

1. Setup + README → Structure ready
2. Add US1 (Monetization) → Core value delivered
3. Add US2 (Competitive Strategy) → Strategy frameworks added
4. Add US3 (Agent Skills) → Technical depth added
5. Add US4 (Guardrails) → Risk awareness added
6. Add Foundation + Synthesis → Chapter complete
7. Summaries + Validation → Publication ready

---

## Summary

| Metric | Value |
|--------|-------|
| **Total Tasks** | 73 |
| **Phase 1 (Setup)** | 4 tasks |
| **Phase 2 (README)** | 4 tasks |
| **US1 (Monetization)** | 10 tasks |
| **US2 (Competitive Strategy)** | 14 tasks |
| **US3 (Agent Skills)** | 4 tasks |
| **US4 (Guardrails)** | 3 tasks |
| **Foundation Lessons** | 6 tasks |
| **Synthesis/Assessment** | 9 tasks |
| **Summaries** | 11 tasks |
| **Validation** | 8 tasks |
| **Parallel Opportunities** | 15+ tasks marked [P] |
| **Suggested MVP** | Phase 1-3 (Setup + README + Monetization) |

---

## Notes

- [P] tasks = different files, can run in parallel
- [Story] label maps task to specific user story
- Content source: `/Agent_Factory_Building_Digital_FTEs.md`
- Each lesson should include "Try With AI" section
- No meta-commentary exposing pedagogical framework
- All new lessons target 1800-2400 words
- Quiz must have 15+ new questions
