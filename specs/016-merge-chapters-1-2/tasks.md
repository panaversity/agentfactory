# Tasks: Merge Chapters 1 & 2 into "Chapter 1: The Agent Factory Paradigm"

**Input**: Design documents from `/specs/016-merge-chapters-1-2/`
**Prerequisites**: plan.md (✅), spec.md (✅)
**Feature Branch**: `016-merge-chapters-1-2`
**GitHub Issue**: #384

**Organization**: Tasks grouped by lesson creation for parallel implementation.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[L#]**: Which lesson this task belongs to
- Include exact file paths in descriptions

## Path Conventions

**Content Location**: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/`
**New Chapter**: `01-agent-factory-paradigm/`
**Old Chapters** (to delete): `01-ai-development-revolution/`, `02-ai-turning-point/`

---

## Phase 1: Setup (Infrastructure) ✅

**Purpose**: Create new chapter directory structure

- [x] T001 Create new chapter directory `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/`
- [x] T002 [P] Read existing Ch1 README `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/readme.md` for context
- [x] T003 [P] Read existing Ch2 README `apps/learn-app/docs/01-Introducing-AI-Driven-Development/02-ai-turning-point/README.md` for context
- [x] T004 [P] Read Agent Factory slides `Agent_Factory_Complete_All_Pages.md` for L3 source content

**Checkpoint**: Directory ready, source content loaded ✅

---

## Phase 2: Foundation (Chapter README) ✅

**Purpose**: Create chapter README that sets context for all lessons

- [x] T005 Create chapter README at `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/README.md` with:
  - Title: "Chapter 1: The Agent Factory Paradigm"
  - sidebar_position: 1
  - Chapter overview (conceptual framework mental models)
  - Learning objectives for chapter
  - 9 lessons listed with durations (~165 min total)
  - No preface redundancy (avoid Digital FTE definition, monetization)

**Checkpoint**: Chapter README complete, lesson implementation can begin ✅

---

## Phase 3: Lesson 1 - The 2025 Inflection Point (Priority: P1) ✅

**Goal**: Establish evidence that AI coding reached production quality in 2024-2025

**Source Files**: Ch1-L01 + Ch2-L01 (consolidated)

### Implementation

- [x] T006 [P] [L1] Read source `01-ai-development-revolution/01-moment_that_changed_everything.md`
- [x] T007 [P] [L1] Read source `02-ai-turning-point/01-the-inflection-point.md`
- [x] T008 [L1] Create `01-agent-factory-paradigm/01-the-2025-inflection-point.md`:
  - YAML frontmatter (lesson=1, duration=15, skills metadata)
  - Embed video: `https://www.youtube.com/embed/jbQbx0Lp1iQ`
  - Content: YC W25 data, 84% adoption, ICPC breakthrough
  - Convergent evidence section
  - Try With AI: "Evaluate Evidence Quality" (NO meta-commentary)
  - Cognitive load: 3 concepts

**Independent Test**: Reader can cite 3 concrete evidence points for AI inflection ✅

---

## Phase 4: Lesson 2 - The Scale of the Shift (Priority: P1) ✅

**Goal**: Present economic scale, uniqueness, and enterprise validation

**Source Files**: Ch1-L02, Ch1-L03, Ch1-L07, Ch1-L08, Ch2-L04 (5 sections, 5 videos)

### Implementation

- [x] T009 [P] [L2] Read source `01-ai-development-revolution/02-three-trillion-developer-economy.md`
- [x] T010 [P] [L2] Read source `01-ai-development-revolution/03-software-disrupting-itself.md`
- [x] T011 [P] [L2] Read source `01-ai-development-revolution/07-opportunity-window.md`
- [x] T012 [P] [L2] Read source `01-ai-development-revolution/08-traditional-cs-education-gaps.md`
- [x] T013 [P] [L2] Read source `02-ai-turning-point/04-dora-perspective.md`
- [x] T014 [L2] Create `01-agent-factory-paradigm/02-the-scale-of-the-shift.md`:
  - YAML frontmatter (lesson=2, duration=25, skills metadata)
  - **Section 2.1**: $3T Developer Economy (video: HQQuxxCjmDs)
  - **Section 2.2**: Software Disrupting Itself (video: D05z2GBY44U)
  - **Section 2.3**: The Opportunity Window (video: A1yIQz3oiRw)
  - **Section 2.4**: What Traditional Education Misses (video: QcPRMFRVR5k)
  - **Section 2.5**: Enterprise Validation - DORA (video: N1jMySJgtng)
  - Try With AI: "Explore Economic Scale"
  - Cognitive load: 5 concepts

**Independent Test**: Reader can explain 3 reasons why AI disruption is different/faster ✅

---

## Phase 5: Lesson 3 - Two Paths to Building AI Products (Priority: P1) ✅ NEW

**Goal**: Introduce General Agents vs Custom Agents framework from Agent Factory slides

**Source Files**: Agent Factory slides (NEW content)

### Implementation

- [x] T015 [L3] Create `01-agent-factory-paradigm/03-two-paths-to-building-ai-products.md`:
  - YAML frontmatter (lesson=3, duration=18, skills metadata)
  - **NO video** (new conceptual content)
  - **Path A: General Agents** (Claude Code, Gemini CLI, Goose)
    - Reasoning systems with OODA loop
    - Your role: Director who specifies intent
  - **Path B: Custom Agents** (OpenAI SDK, Claude SDK, ADK)
    - Purpose-built for specific workflows
    - Your role: Builder who creates for others
  - **Key Insight**: General Agents BUILD Custom Agents
    - Agent Factory metaphor
  - Exact terminology from slides (Path A, Path B)
  - Try With AI: "Distinguish Your Path"
  - Cognitive load: 4 concepts

**Independent Test**: Reader can give examples of General vs Custom Agents ✅

---

## Phase 6: Lesson 4 - From Coder to Orchestrator (Priority: P2) ✅

**Goal**: Explain developer role evolution from typist to orchestrator

**Source Files**: Ch1-L05 (condensed)

### Implementation

- [x] T016 [P] [L4] Read source `01-ai-development-revolution/05-beyond-code-changing-roles.md`
- [x] T017 [L4] Create `01-agent-factory-paradigm/04-from-coder-to-orchestrator.md`:
  - YAML frontmatter (lesson=4, duration=15, skills metadata)
  - Embed video: `https://www.youtube.com/embed/9wY2BSvonUI`
  - Role shift narrative
  - Judgment layer (human vs AI capabilities)
  - Try With AI: "Assess Your Role"
  - Cognitive load: 3 concepts

**Independent Test**: Reader can describe what orchestration means vs typing code ✅

---

## Phase 7: Lesson 5 - Development Lifecycle Transformation (Priority: P2) ✅

**Goal**: Show how AI transforms each phase of SDLC

**Source Files**: Ch1-L04 (kept mostly intact)

### Implementation

- [x] T018 [P] [L5] Read source `01-ai-development-revolution/04-development-lifecycle-transition.md`
- [x] T019 [L5] Create `01-agent-factory-paradigm/05-development-lifecycle-transformation.md`:
  - YAML frontmatter (lesson=5, duration=18, skills metadata)
  - Embed video: `https://www.youtube.com/embed/sQqJ8aCInEA`
  - 5 phases: planning → coding → testing → deployment → operations
  - Each phase: what changes, what stays same
  - Try With AI: "Explore Phase Changes"
  - Cognitive load: 5 concepts

**Independent Test**: Reader can identify AI impact on 3 specific SDLC phases ✅

---

## Phase 8: Lesson 6 - The Autonomous Agent Era (Priority: P2) ✅

**Goal**: Trace AI tool evolution Gen 1-4 and current capabilities

**Source Files**: Ch1-L06 (kept mostly intact)

### Implementation

- [x] T020 [P] [L6] Read source `01-ai-development-revolution/06-autonomous-agent-era.md`
- [x] T021 [L6] Create `01-agent-factory-paradigm/06-the-autonomous-agent-era.md`:
  - YAML frontmatter (lesson=6, duration=18, skills metadata)
  - Embed video: `https://www.youtube.com/embed/3ZPIerZkZn4`
  - Timeline: Gen 1 (2021) → Gen 2 (2022) → Gen 3 (2023) → Gen 4 (2024-25)
  - Each generation capabilities
  - Current state: Gen 3→4 transition
  - Try With AI: "Trace Evolution"
  - Cognitive load: 4 concepts

**Independent Test**: Reader can identify which generation current tools belong to ✅

---

## Phase 9: Lesson 7 - User Intent Replaces User Interface (Priority: P2) ✅

**Goal**: Explain UX→Intent paradigm shift and Five Powers framework

**Source Files**: Ch2-L02 (kept mostly intact)

### Implementation

- [x] T022 [P] [L7] Read source `02-ai-turning-point/02-user-interface-intent.md`
- [x] T023 [L7] Create `01-agent-factory-paradigm/07-user-intent-replaces-user-interface.md`:
  - YAML frontmatter (lesson=7, duration=20, skills metadata)
  - Embed video: `https://www.youtube.com/embed/yq-IhPszMpM`
  - Old paradigm: Interface → 14 manual steps
  - New paradigm: Intent → orchestration
  - **Five Powers**: See, Hear, Reason, Act, Remember
  - How powers combine for autonomy
  - Agentic evolution (predictive → generative → agentic)
  - Try With AI: "Identify Powers in Action"
  - Cognitive load: 5 concepts

**Independent Test**: Reader can name Five Powers and explain one ✅

---

## Phase 10: Lesson 8 - The Modern AI Stack (Priority: P2) ✅

**Goal**: Present three-layer architecture and MCP interoperability

**Source Files**: Ch2-L05 (kept mostly intact)

### Implementation

- [x] T024 [P] [L8] Read source `02-ai-turning-point/05-ai-coding-agents.md`
- [x] T025 [L8] Create `01-agent-factory-paradigm/08-the-modern-ai-stack.md`:
  - YAML frontmatter (lesson=8, duration=18, skills metadata)
  - Embed video: `https://www.youtube.com/embed/RZCJTjwZQt4`
  - Three layers: Frontier Models, IDEs, Development Agents
  - MCP: "USB for AI tools"
  - Tool independence / vendor flexibility
  - Connection to L3 (where General Agents live in stack)
  - Try With AI: "Map Tools to Layers"
  - Cognitive load: 4 concepts

**Independent Test**: Reader can explain MCP purpose in one sentence ✅

---

## Phase 11: Lesson 9 - Spec-Driven Development Preview (Priority: P2) ✅

**Goal**: Bridge from conceptual framework to practical SDD skills

**Source Files**: Ch2-L03 (condensed)

### Implementation

- [x] T026 [P] [L9] Read source `02-ai-turning-point/03-development-patterns.md`
- [x] T027 [L9] Create `01-agent-factory-paradigm/09-spec-driven-development-preview.md`:
  - YAML frontmatter (lesson=9, duration=15, skills metadata)
  - Embed video: `https://www.youtube.com/embed/JyPatZzRNZ8`
  - Why specifications matter (intent clarity)
  - SDD workflow overview: spec → plan → tasks → implement
  - Connection to Digital FTE (reference preface, don't repeat)
  - Bridge to Part 2 (what comes next)
  - Try With AI: "Preview SDD Thinking"
  - Cognitive load: 3 concepts

**Independent Test**: Reader can list 4 SDD workflow phases in order ✅

---

## Phase 12: Chapter Quiz (Priority: P2) ✅

**Goal**: Assess understanding across all 9 lessons

**Source Files**: Both existing quizzes (rewritten)

### Implementation

- [x] T028 [P] [Quiz] Read source `01-ai-development-revolution/09_chapter_01_quiz.md`
- [x] T029 [P] [Quiz] Read source `02-ai-turning-point/05_chapter_02_quiz.md`
- [x] T030 [Quiz] Create `01-agent-factory-paradigm/10-chapter-quiz.md`:
  - YAML frontmatter (quiz=true)
  - 18 questions covering all 9 lessons
  - Question types: multiple choice, scenarios, matching
  - Coverage:
    - L1: Evidence recognition
    - L2: Economic scale
    - L3: Two Paths distinction (NEW)
    - L4: Role evolution
    - L5: Lifecycle phases
    - L6: Tool generations
    - L7: Five Powers
    - L8: Three-layer stack + MCP
    - L9: SDD workflow

**Independent Test**: Quiz covers all learning objectives from spec ✅

---

## Phase 13: Cleanup & Validation (Priority: P3)

**Purpose**: Delete old content, update references, validate

### Validation Tasks ✅

- [x] T031 Run preface redundancy check: `grep -i "3 trillion\|monetization\|subscription\|digital fte definition" 01-agent-factory-paradigm/*.md` (expect: 0 matches) ✅ PASS
- [x] T032 Run meta-commentary check: `grep -i "AI is teaching\|What to notice\|AI learned\|AI now knows" 01-agent-factory-paradigm/*.md` (expect: 0 matches) ✅ PASS
- [x] T033 Verify all 12 videos embedded correctly ✅ PASS
- [x] T034 Verify all lessons have Try With AI sections ✅ 9/9 lessons have Try With AI

### Deletion Tasks (AFTER validation passes) ✅

- [x] T035 Backup old chapters to `_archive/` if needed for reference ✅ Moved to specs/016-merge-chapters-1-2/_archive/
- [x] T036 Delete `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-ai-development-revolution/` ✅ Archived
- [x] T037 Delete `apps/learn-app/docs/01-Introducing-AI-Driven-Development/02-ai-turning-point/` ✅ Archived

### Update Tasks ✅

- [x] T038 Update Part 1 README with new Chapter structure (4 chapters with new Ch2 AIFF placeholder) ✅
- [x] T039 Create Chapter 2 placeholder (02-aiff-foundation) from issue #385 ✅
- [x] T040 Keep Chapter 3 and 4 numbering unchanged (03-billion-dollar-ai, 04-nine-pillars) ✅
- [x] T041 Verify sidebar navigation works correctly ✅ sidebar_position: 1, 2, 3, 4 verified

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup) → Phase 2 (README)
                      ↓
    ┌─────────────────┼─────────────────┐
    ↓                 ↓                 ↓
Phase 3 (L1)    Phase 4 (L2)    Phase 5 (L3-NEW)
    ↓                 ↓                 ↓
Phase 6 (L4)    Phase 7 (L5)    Phase 8 (L6)
    ↓                 ↓                 ↓
Phase 9 (L7)    Phase 10 (L8)   Phase 11 (L9)
    ↓                 ↓                 ↓
    └─────────────────┼─────────────────┘
                      ↓
              Phase 12 (Quiz)
                      ↓
              Phase 13 (Cleanup)
```

### Lesson Dependencies

- **L1**: No dependencies (foundational evidence)
- **L2**: No dependencies (parallel with L1)
- **L3**: No dependencies (NEW content from slides)
- **L4-L9**: Can run in parallel after Phase 2
- **Quiz**: Depends on all lessons complete
- **Cleanup**: Depends on validation passing

### Parallel Opportunities

**Maximum parallelism**: After Phase 2 (README), ALL 9 lessons + Quiz can be created in parallel

```bash
# Launch all lesson implementations in parallel:
Task: T008 [L1] Create lesson 1
Task: T014 [L2] Create lesson 2 (5 sections)
Task: T015 [L3] Create lesson 3 (NEW)
Task: T017 [L4] Create lesson 4
Task: T019 [L5] Create lesson 5
Task: T021 [L6] Create lesson 6
Task: T023 [L7] Create lesson 7
Task: T025 [L8] Create lesson 8
Task: T027 [L9] Create lesson 9
Task: T030 [Quiz] Create chapter quiz
```

---

## Implementation Strategy

### MVP First (L1 + L3)

1. Complete Phase 1-2: Setup + README
2. Complete Phase 3: L1 (evidence foundation)
3. Complete Phase 5: L3 (Two Paths - NEW content)
4. **STOP and VALIDATE**: Test new content independently
5. If L1 + L3 work, continue with remaining lessons

### Incremental Delivery

1. Setup + README → Foundation ready
2. L1 + L2 + L3 → Core framework established
3. L4-L6 → Evolution narrative complete
4. L7-L9 → Technical depth + bridge
5. Quiz → Assessment ready
6. Cleanup → Migration complete

### Content Implementer Agent Strategy

With parallel content-implementer agents:

1. Agent A: L1 + L4 + L7
2. Agent B: L2 (complex, 5 sections)
3. Agent C: L3 (NEW) + L6 + L9
4. Agent D: L5 + L8 + Quiz

---

## Summary

| Metric | Value |
|--------|-------|
| **Total Tasks** | 41 |
| **Completed** | 41 (T001-T041) ✅ |
| **Pending** | 0 |
| **Setup Tasks** | 4 ✅ |
| **Foundation Tasks** | 1 ✅ |
| **Lesson Tasks** | 25 ✅ |
| **Quiz Tasks** | 3 ✅ |
| **Validation Tasks** | 4 ✅ |
| **Cleanup Tasks** | 7 ✅ |
| **Parallel Opportunities** | 10 lessons ran in parallel ✅ |
| **MVP Scope** | L1 + L3 (evidence + two paths) ✅ |
| **Actual Duration** | ~45 minutes with 10 parallel agents |

---

## Notes

- [P] tasks = different files, no dependencies
- [L#] label maps task to specific lesson
- Each lesson should be independently completable
- Validate after each lesson for preface/meta-commentary compliance
- Commit after each lesson completion
- Close GitHub issue #384 after T041 completes
