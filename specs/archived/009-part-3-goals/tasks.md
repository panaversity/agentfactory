---
description: "Task list for Part-3 Goals implementation"
---

# Tasks: Part-3 Goals - Working with AI Coding Agents

**Input**: Design documents from `/specs/009-part-3-goals/`
**Prerequisites**: plan.md (completed), spec.md (completed)

**Tests**: No test tasks included (this is educational content, not software)

**Organization**: Tasks are grouped by user story to enable independent implementation and validation of each content piece.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files/sections, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths or section references in descriptions

## Path Conventions

- **Deliverable**: `book/docs/part-3/intro.mdx` (400-600 words)
- **Specs**: `specs/009-part-3-goals/` (planning artifacts)

---

## Phase 1: Setup (Content Infrastructure)

**Purpose**: Prepare content structure and validate prerequisites

- [X] T001 Validate Chapter 9 and Chapter 10 content exists and is accessible
- [X] T002 Review preface at `context/01_preface/readme.md` for voice/tone alignment
- [X] T003 [P] Create Docusaurus directory structure `book/docs/part-3/` if not exists
- [X] T004 [P] Review constitution principles 1, 5, 8, 12, 13 for beginner-tier requirements

---

## Phase 2: Foundational (Content Research)

**Purpose**: Gather all information needed to write accurate Part-3 goals

**⚠️ CRITICAL**: Must understand both chapters before writing introduction

- [X] T005 Extract key topics from Chapter 9 (Prompt Engineering): 8-element AIDD framework, prompt templates, effectiveness measurement
- [X] T006 Extract key topics from Chapter 10 (Context Engineering): context window, progressive loading, memory files, compression
- [X] T007 Identify relationship between chapters: prompting = what you SAY, context = what AI KNOWS
- [X] T008 Confirm prerequisite knowledge from Parts 1-2: AI vision, tool overview, basic terminal
- [X] T009 Validate word count target: 400-600 words strict constraint
- [X] T010 [P] List 5 key concepts to introduce (cognitive load limit): prompt engineering, context engineering, AI agents, validation, iteration

**Checkpoint**: Full understanding of Chapter 9 and Chapter 10 content achieved

---

## Phase 3: User Story 1 - Hook & Learning Overview (Priority: P1) 🎯 MVP

**Goal**: Write Sections 1-2 of introduction (250-300 words) establishing relevance and previewing both chapters

**Independent Test**: Reader understands WHY these skills matter and WHAT they'll learn in each chapter

### Implementation for User Story 1

- [X] T011 [US1] Write Section 1: The Hook (100-120 words) in `book/docs/part-3/intro.mdx`
  - Transition from "learned ABOUT AI" to "USE AI to build"
  - Paradigm shift: director vs manual coder
  - Foundation for everything that follows
- [X] T012 [US1] Write Section 2: What You'll Learn (150-180 words) in `book/docs/part-3/intro.mdx`
  - Chapter 9 preview: WHAT (prompting), WHY (clear = working code), HOW (8-element framework), OUTCOME (70% success)
  - Chapter 10 preview: WHAT (context management), WHY (generic vs project-specific), HOW (progressive loading), OUTCOME (consistency)
  - Link: Prompting = what you SAY, Context = what AI KNOWS
- [X] T013 [US1] Validate word count for Sections 1-2: should be 250-300 words combined
- [X] T014 [US1] Check tone: encouraging, empowering, no jargon without explanation
- [X] T015 [US1] Verify accurate representation of Chapter 9 content
- [X] T016 [US1] Verify accurate representation of Chapter 10 content

**Checkpoint**: Hook and learning overview complete; reader knows what to expect

---

## Phase 4: User Story 2 - Learning Path & Success Vision (Priority: P2)

**Goal**: Write Sections 3-4 of introduction (150-200 words) showing progression and painting success picture

**Independent Test**: Reader feels confident they can learn these skills and understands what they'll be able to do

### Implementation for User Story 2

- [X] T017 [US2] Write Section 3: The Learning Path (80-100 words) in `book/docs/part-3/intro.mdx`
  - Progression: simple prompts → technical constraints → multi-file context → AI orchestration
  - Reassurance: no programming required, learn by doing, AI handles syntax
- [X] T018 [US2] Write Section 4: Success Vision (70-100 words) in `book/docs/part-3/intro.mdx`
  - 5 concrete capabilities: effective prompts, context management, iteration, validation, orchestrator mindset
  - Bridge to Part-4: apply these skills to learn Python
  - From here on: AI collaboration as default workflow
- [X] T019 [US2] Validate word count for Sections 3-4: should be 150-200 words combined
- [X] T020 [US2] Check for "you will..." language (not "this chapter covers...")
- [X] T021 [US2] Verify bridge to Part-4 is clear and motivating

**Checkpoint**: Complete learning path and success vision written

---

## Phase 5: User Story 3 - Mindset Framing Throughout (Priority: P3)

**Goal**: Ensure entire introduction positions AI as collaborator (not tool) and emphasizes validation-first mindset

**Independent Test**: Reader understands they're learning to "orchestrate AI" not "use a tool"

### Implementation for User Story 3

- [X] T022 [US3] Review all 4 sections for agent-native language in `book/docs/part-3/intro.mdx`
  - Replace "use AI" with "direct AI agents" or "collaborate with AI"
  - Replace "AI tool" with "AI partner" or "AI co-pilot"
- [X] T023 [US3] Add validation emphasis throughout introduction
  - Mention in Section 4: "validate AI-generated code for correctness and security"
  - Frame as human responsibility, not AI failure
- [X] T024 [US3] Ensure no over-promising language
  - NOT: "AI will write all your code perfectly"
  - YES: "AI generates code from your specifications; you validate"
- [X] T025 [US3] Check for realistic framing of learning curve
  - NOT: "You must master complex prompting"
  - YES: "You'll learn simple patterns that get powerful results"

**Checkpoint**: Agent-native mindset and validation-first philosophy integrated throughout

---

## Phase 6: Polish & Quality Validation

**Purpose**: Final validation against all quality gates before publication

- [X] T026 [P] Validate total word count: MUST be 400-600 words (strict constraint) ✓ 570 words
- [X] T027 [P] Count concepts introduced: MUST be maximum 5 (cognitive load check) ✓ 5 concepts
- [X] T028 [P] Check for ableist language: remove "obviously", "simply", "just", "easy" ✓ None found
- [X] T029 [P] Verify no jargon without immediate context ✓ All terms explained
- [X] T030 Validate Section 1 (Hook): 100-120 words, exciting tone ✓ Pass
- [X] T031 Validate Section 2 (Learning Overview): 150-180 words, accurate chapter previews ✓ Pass
- [X] T032 Validate Section 3 (Learning Path): 80-100 words, builds confidence ✓ Pass
- [X] T033 Validate Section 4 (Success Vision): 70-100 words, concrete capabilities ✓ Pass
- [X] T034 [P] Constitution Principle 1 check: AI-first teaching philosophy evident ✓ Pass
- [X] T035 [P] Constitution Principle 5 check: beginner complexity tier maintained ✓ Pass
- [X] T036 [P] Constitution Principle 8 check: accessible to non-developers ✓ Pass
- [X] T037 [P] Constitution Principle 12 check: cognitive load limits respected ✓ Pass
- [X] T038 [P] Constitution Principle 13 check: concept-before-command pattern used ✓ Pass
- [X] T039 Read aloud to check for awkward phrasing (accessibility) ✓ Pass
- [X] T040 Verify Docusaurus MDX syntax is correct ✓ Pass
- [ ] T041 Test build: run Docusaurus build to ensure no errors (deferred - requires full Docusaurus setup)
- [X] T042 [P] Create final validation checklist in `specs/009-part-3-goals/validation-report.md`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all content writing
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can proceed sequentially (P1 → P2 → P3) for voice consistency
  - Or in parallel if multiple writers available (though sequential recommended for tone)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - writes Sections 1-2
- **User Story 2 (P2)**: Can start after US1 or in parallel - writes Sections 3-4 (recommended: after US1 for flow)
- **User Story 3 (P3)**: MUST start after US1 and US2 complete - reviews and refines all sections

### Within Each User Story

- Sections should be written in order for natural flow
- Validation tasks within story can run in parallel
- Voice/tone checks should happen after content complete
- Constitution checks can run in parallel during polish phase

### Parallel Opportunities

- Phase 1 Setup: T003 and T004 can run in parallel
- Phase 2 Foundational: T010 can run in parallel with T005-T009
- Phase 3 US1: T015 and T016 (validation tasks) can run in parallel after T012 complete
- Phase 4 US2: T020 and T021 can run in parallel after T018 complete
- Phase 6 Polish: T026-T028 (word count, concepts, language) can run in parallel
- Phase 6 Polish: T030-T033 (section validation) can run in parallel
- Phase 6 Polish: T034-T038 (constitution checks) can run in parallel
- Phase 6 Polish: T040-T042 (technical validation) can run after T039

---

## Parallel Example: User Story 1

```bash
# After writing Section 2 (T012), validate both chapters in parallel:
Task T015: "Verify accurate representation of Chapter 9 content"
Task T016: "Verify accurate representation of Chapter 10 content"

# These can run simultaneously since they check different chapters
```

## Parallel Example: Polish Phase

```bash
# Run all constitution checks simultaneously:
Task T034: "Constitution Principle 1 check"
Task T035: "Constitution Principle 5 check"
Task T036: "Constitution Principle 8 check"
Task T037: "Constitution Principle 12 check"
Task T038: "Constitution Principle 13 check"

# These are independent verification tasks
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup → Understand structure
2. Complete Phase 2: Foundational → Full knowledge of chapters
3. Complete Phase 3: User Story 1 → Hook + Learning Overview (250-300 words)
4. **STOP and VALIDATE**: Does reader understand WHY and WHAT?
5. If yes, proceed to US2; if no, refine US1

### Incremental Delivery

1. Complete Setup + Foundational → Ready to write
2. Add User Story 1 → Validate independently → Reader knows WHY/WHAT
3. Add User Story 2 → Validate independently → Reader knows HOW and SUCCESS VISION
4. Add User Story 3 → Validate independently → Agent-native mindset integrated
5. Polish Phase → Final quality validation
6. Each story adds value and can be reviewed independently

### Sequential Writing (Recommended)

With single writer (recommended for voice consistency):

1. Complete Setup + Foundational together
2. Write User Story 1 sections (Hook + Learning Overview)
3. Write User Story 2 sections (Learning Path + Success Vision)
4. Apply User Story 3 refinements (mindset framing throughout)
5. Polish phase: comprehensive validation

**Rationale**: Sequential writing maintains consistent voice and natural flow. Parallel writing risks tone inconsistencies.

---

## Notes

- [P] tasks = different sections/checks, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story delivers independent value: US1 = WHY/WHAT, US2 = HOW/SUCCESS, US3 = MINDSET
- Word count is strict: 400-600 words total (not per section, TOTAL)
- Cognitive load is strict: max 5 new concepts across entire introduction
- Voice consistency critical: write in one sitting if possible
- Read aloud before finalizing (catches awkward phrasing)
- Avoid: vague goals, jargon without context, intimidating language, over-promising outcomes

---

## Policy Note for Content Authors

Within this chapter introduction, following the standard lesson structure is not applicable since this is a part-level introduction, not a lesson. However, the tone and philosophy should align with the "Try With AI" approach used throughout the book:

- Position AI agents as collaborators for learning
- Encourage hands-on practice over passive reading
- Frame concepts with business/problem-solving context
- Validate that AI-generated understanding is correct

After Part-3 completion (Chapters 9-10), learners will use their preferred AI companion tool (Claude Code, Gemini CLI) for all subsequent chapters.
