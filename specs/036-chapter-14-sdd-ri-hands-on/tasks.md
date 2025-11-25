# Tasks: Chapter 14 — SDD-RI Hands-On with Spec-Kit Plus

**Input**: Design documents from `/specs/036-chapter-14-sdd-ri-hands-on/`
**Prerequisites**: spec.md (user stories), plan.md (lesson structure with content strategy)
**Project Type**: Educational Content (Docusaurus lessons)
**Target Directory**: `book-source/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/`

**Policy**: Within this chapter, each lesson must end with a single final section titled "Try With AI" (no "Key Takeaways" or "What's Next"). Students use their preferred AI companion tool (Claude Code, Gemini CLI).

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- **No Story Label**: Setup/Foundational/Polish tasks

## Content Strategy Key

From plan.md alignment decision:
- **KEEP & IMPROVE**: Same structure, better examples (video domain)
- **KEEP & EXTEND**: Add new content (e.g., Playwright MCP setup)
- **KEEP & PIVOT**: Same pattern, new domain examples
- **RECREATE**: New content replacing calculator with video/YouTube
- **NEW**: Entirely new lesson

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Prepare chapter structure and validate existing content

- [x] T001 Create feature branch `036-chapter-14-sdd-ri-hands-on` and verify checkout
- [x] T002 [P] Audit existing lessons in `book-source/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/` — document current state
- [x] T003 [P] Review Chapter 13 lessons for anti-convergence verification — ensure Chapter 14 modality differs
- [x] T004 Update `book-source/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/README.md` with new chapter overview (video/YouTube project framing)

**Checkpoint**: Chapter structure ready, content strategy validated

---

## Phase 2: Foundational Lessons (L01-L03) — Tool Foundation

**Purpose**: Foundation lessons that establish Spec-Kit Plus context before SDD-RI workflow

### Lesson 01: Foundation — KEEP & IMPROVE

**Goal**: Introduce Spec-Kit Plus architecture with improved video project framing

- [x] T005 [US1] Read existing `01-spec-kit-plus-foundation.md` and identify calculator-specific content
- [x] T006 [US1] Refactor `01-spec-kit-plus-foundation.md`:
  - Keep: Horizontal/Vertical Intelligence concepts
  - Update: Replace calculator references with video generation project context
  - Update: Add business framing (product demo videos for SaaS marketing)
  - Verify: "Try With AI" ending exists, no "Summary" or "What's Next"
- [x] T007 [US1] Validate cognitive load: ≤7 concepts for A2 tier

### Lesson 02: Installation — KEEP & EXTEND

**Goal**: Add Playwright MCP setup to existing installation lesson

- [x] T008 [US1] Read existing `02-installation-and-setup.md`
- [x] T009 [US1] Extend `02-installation-and-setup.md`:
  - Keep: Spec-Kit Plus installation steps
  - Add: Playwright MCP installation section (npm install @anthropic-ai/mcp-playwright)
  - Add: Session persistence configuration for browser automation
  - Add: Verification steps for Playwright MCP availability
  - Verify: "Try With AI" ending, no framework labels
- [x] T010 [US1] Validate cross-platform instructions (macOS, Linux, Windows)

### Lesson 03: Constitution — KEEP & PIVOT

**Goal**: Same constitution pattern, pivoted to video project context

- [x] T011 [US1] Read existing `03-constitution-phase.md`
- [x] T012 [US1] Refactor `03-constitution-phase.md`:
  - Keep: Constitution creation workflow, cascade effect explanation
  - Replace: Calculator quality standards → Video project quality standards
  - Update: Examples use video generation constraints (Gemini free tier, Playwright automation)
  - Verify: "Try With AI" ending exists
- [x] T013 [US1] Ensure constitution examples are reusable for Lessons 4-8

**Checkpoint**: Foundation complete — students understand Spec-Kit Plus and have tools installed

---

## Phase 3: User Story 1 — Execute SDD-RI Workflow (L04-L08) [P1]

**Goal**: Students execute complete `/sp.specify` → `/sp.plan` → `/sp.tasks` → `/sp.implement` workflow

**Independent Test**: Student has video file downloaded locally after completing Lesson 08

### Lesson 04: Specify Phase — RECREATE

**Goal**: Write video generation specification from scratch

- [x] T014 [P] [US1] Create new content for `04-specify-phase.md`:
  - Business context: Product demo video for SaaS
  - Spec template walkthrough: Intent, Constraints, Success Evals, Non-Goals
  - Student writes spec from scratch (not copy-paste)
  - AI collaboration: Pre-specification conversation pattern
- [x] T015 [US1] Include specification validation checklist (4 sections present)
- [x] T016 [US1] Add "Try With AI" activity: Specification refinement with Claude
- [x] T017 [US1] Validate: Three Roles framework invisible (no "AI as Teacher" labels)

### Lesson 05: Clarify Phase — KEEP & PIVOT

**Goal**: Refine specification through `/sp.clarify`

- [x] T018 [US1] Read existing `05-clarify-phase.md`
- [x] T019 [US1] Refactor `05-clarify-phase.md`:
  - Keep: Clarify workflow structure
  - Replace: Calculator clarification examples → Video spec clarification
  - Add: Edge cases for video generation (Gemini availability, session timeout)
  - Verify: "Try With AI" ending
- [x] T020 [US1] Ensure students refine their spec from Lesson 04

### Lesson 06: Plan Phase — RECREATE

**Goal**: Generate implementation plan using Playwright MCP + Gemini approach

- [x] T021 [P] [US1] Create new content for `06-plan-phase.md`:
  - `/sp.plan` command execution
  - Technical approach: Playwright MCP browser automation
  - Gemini.google.com integration (free tier, UI automation)
  - Session persistence pattern explanation
  - Error handling strategy (retry logic)
- [x] T022 [US1] Include plan evaluation checklist
- [x] T023 [US1] Add "Try With AI" activity: Plan gap identification and refinement
- [x] T024 [US1] Demonstrate AI as Teacher (suggests approach) invisibly through narrative

### Lesson 07: Tasks Phase — RECREATE

**Goal**: Break plan into atomic tasks with dependencies

- [x] T025 [P] [US1] Create new content for `07-tasks-phase.md`:
  - `/sp.tasks` command execution
  - Task atomicity principles (15-30 min completion)
  - Dependency ordering for video generation workflow
  - Acceptance criteria for each task
  - Checkpoint validation pattern
- [x] T026 [US1] Include task validation checklist
- [x] T027 [US1] Add "Try With AI" activity: Task refinement dialogue
- [x] T028 [US1] Demonstrate AI as Student (learns from feedback) invisibly

### Lesson 08: Implement Phase — RECREATE

**Goal**: Generate and execute video generation code

- [x] T029 [P] [US1] Create new content for `08-implement-phase.md`:
  - `/sp.implement` command execution
  - Playwright setup walkthrough (session persistence)
  - First implementation run and validation
  - Debugging & iteration loop (AI as Co-Worker)
  - Video file verification checklist
- [x] T030 [US1] Include iteration loop example (quality refinement)
- [x] T031 [US1] Add "Try With AI" activity: Full implementation cycle
- [x] T032 [US1] Validate: Student has video file locally after this lesson

**Checkpoint**: US1 complete — students have executed full SDD-RI workflow with tangible video output

---

## Phase 4: User Story 2 — Create Reusable Intelligence (L09) [P1]

**Goal**: Students identify patterns and create 2 skills using P+Q+P framework

**Independent Test**: Student creates valid `generate-video` and `upload-youtube` skill definitions

### Lesson 09: Designing Reusable Intelligence — KEEP STRUCTURE, NEW EXAMPLES

**Goal**: Extract skills from workflow using Persona + Questions + Principles

- [x] T033 [US2] Read existing `09-designing-reusable-intelligence.md` structure
- [x] T034 [P] [US2] Refactor `09-designing-reusable-intelligence.md`:
  - Keep: P+Q+P framework explanation, skill vs subagent distinction
  - Replace: Calculator pattern examples → Video generation patterns
  - Add: Pattern recognition from Lessons 04-08 workflow
  - Add: `generate-video` skill template (complete P+Q+P)
  - Add: `upload-youtube` skill template (complete P+Q+P)
- [x] T035 [US2] Include skill validation checklist
- [x] T036 [US2] Add "Try With AI" activity: Skill design and validation dialogue
- [x] T037 [US2] Verify: Skills are abstract enough to reuse in Lesson 11

**Checkpoint**: US2 complete — students have created 2 reusable skills

---

## Phase 5: Brownfield Adoption (L10) — Real-World Skills

**Goal**: Teach adoption of Spec-Kit Plus in existing projects

### Lesson 10: Brownfield Adoption — KEEP & IMPROVE

- [x] T038 Read existing `10-brownfield-adoption.md`
- [x] T039 [P] Improve `10-brownfield-adoption.md`:
  - Keep: Brownfield workflow, safety practices, backup procedures
  - Improve: Clarity on file overwrites vs preservation
  - Update: Examples reference video project context where relevant
  - Verify: "Try With AI" ending
- [x] T040 Validate: Content standalone (doesn't depend on video project completion)

**Checkpoint**: Students understand real-world Spec-Kit Plus adoption patterns

---

## Phase 6: User Story 3 — Complete Capstone (L11) [P2]

**Goal**: Apply skills to YouTube upload, demonstrating intelligence acceleration

**Independent Test**: Student has video uploaded to YouTube using created skills

### Lesson 11: YouTube Capstone — NEW

**Goal**: Compose skills to execute YouTube upload, demonstrate SDD-RI value

- [x] T041 [P] [US3] Create new `11-youtube-capstone.md`:
  - Section 1: Write YouTube upload specification (shorter than Lesson 04 spec)
  - Section 2: Reference skills from Lesson 09 in implementation
  - Section 3: Execute `/sp.implement` with skill composition
  - Section 4: Validation against success evals
  - Section 5: Reflection on acceleration (second project faster)
- [x] T042 [US3] Include capstone validation checklist:
  - Video uploaded to YouTube
  - Metadata matches spec
  - URL accessible
  - Reflection demonstrates understanding
- [x] T043 [US3] Add "Try With AI" activity: Full capstone cycle with reflection
- [x] T044 [US3] Ensure capstone demonstrates skill reuse value proposition

**Checkpoint**: US3 complete — students have end-to-end portfolio piece

---

## Phase 7: Quiz & Validation

**Purpose**: Chapter assessment and quality validation

### Quiz — RECREATE

- [x] T045 [P] Create new `12_chapter_14_quiz.md`:
  - Questions cover video domain (not calculator)
  - Assess: SDD-RI workflow understanding
  - Assess: P+Q+P skill creation
  - Assess: Intelligence accumulation value proposition
- [x] T046 Validate quiz answers are accurate and unambiguous

### Cross-Cutting Validation

- [x] T047 Run constitutional compliance check:
  - `grep -i "What to notice\|AI.*teach\|AI.*learn\|AI as\|AI now knows" book-source/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/*.md`
  - Expected: Zero matches (Three Roles framework invisible)
- [x] T048 Verify all lessons end with "Try With AI" (no "Summary", "What's Next", "Key Takeaways")
- [x] T049 Validate cognitive load per lesson (max 7 concepts for A2-B1 tier)
- [x] T050 Run factual verification on technical claims:
  - Playwright MCP installation commands
  - Gemini.google.com video generation availability
  - YouTube upload automation feasibility

---

## Phase 8: Polish & Documentation

**Purpose**: Final quality improvements

- [x] T051 [P] Update `README.md` with final lesson list (11 lessons + quiz)
- [ ] T052 [P] Verify all image references in `/img/part-4/chapter-14/` are valid
- [ ] T053 Ensure consistent frontmatter across all lessons (chapter: 14, correct lesson numbers)
- [ ] T054 Run Docusaurus build validation: `npm run build` passes
- [ ] T055 Create PR with comprehensive description of changes

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundation L01-L03)
    ↓
Phase 3 (US1: Workflow L04-L08) ← Linear (each lesson builds on previous)
    ↓
Phase 4 (US2: Intelligence L09) ← Requires L08 completion
    ↓
Phase 5 (Brownfield L10) ← Can run parallel with Phase 6
    ↓
Phase 6 (US3: Capstone L11) ← Requires L09 skills
    ↓
Phase 7 (Quiz & Validation)
    ↓
Phase 8 (Polish)
```

### Lesson Dependencies

```
L01 → L02 → L03 (Foundation: sequential)
L03 → L04 → L05 → L06 → L07 → L08 (US1 Workflow: linear)
L08 → L09 (Pattern recognition requires workflow execution)
L09 → L11 (Capstone requires skills)
L10 (Brownfield): Standalone after L03
```

### Parallel Opportunities

**Within Phase 2** (after T004):
- T005-T007 (L01) can run parallel with T008-T010 (L02)

**Within Phase 3**:
- T014 (L04 content) can run parallel with T021 (L06 content) — different files
- T025 (L07 content) can run parallel with T029 (L08 content) — different files

**Phase 5 + Phase 6**:
- L10 (Brownfield) can run parallel with L11 (Capstone) — independent content

**Within Phase 7**:
- T045-T046 (Quiz) can run parallel with T047-T050 (Validation)

---

## Parallel Execution Example

```bash
# After Phase 1 complete, launch Foundation lessons in parallel:
Task: T005 "Read existing 01-spec-kit-plus-foundation.md"
Task: T008 "Read existing 02-installation-and-setup.md"
Task: T011 "Read existing 03-constitution-phase.md"

# After Foundation complete, launch RECREATE lessons in parallel:
Task: T014 "Create new content for 04-specify-phase.md"
Task: T021 "Create new content for 06-plan-phase.md"
Task: T025 "Create new content for 07-tasks-phase.md"
Task: T029 "Create new content for 08-implement-phase.md"
```

---

## Implementation Strategy

### MVP First (User Story 1)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundation (L01-L03)
3. Complete Phase 3: US1 Workflow (L04-L08)
4. **VALIDATE**: Student can execute full SDD-RI workflow → video file generated
5. Deploy/demo Chapter 14 MVP

### Full Delivery

1. MVP + Phase 4: US2 (L09 skills creation)
2. Phase 5: Brownfield (L10)
3. Phase 6: US3 Capstone (L11)
4. Phase 7: Quiz + Validation
5. Phase 8: Polish
6. Full Chapter 14 ready

---

## Summary

| Phase | Tasks | Parallelizable | User Story |
|-------|-------|----------------|------------|
| Setup | T001-T004 | 2 | — |
| Foundation | T005-T013 | 3 | US1 |
| US1 Workflow | T014-T032 | 4 | US1 |
| US2 Intelligence | T033-T037 | 1 | US2 |
| Brownfield | T038-T040 | 1 | — |
| US3 Capstone | T041-T044 | 1 | US3 |
| Quiz & Validation | T045-T050 | 2 | — |
| Polish | T051-T055 | 2 | — |
| **Total** | **55 tasks** | **16 parallel** | |

### Task Count by User Story

- **US1 (SDD-RI Workflow)**: 28 tasks (L01-L08)
- **US2 (Reusable Intelligence)**: 5 tasks (L09)
- **US3 (Capstone)**: 4 tasks (L11)
- **Non-Story (Setup/Brownfield/Quiz/Polish)**: 18 tasks

### Suggested MVP Scope

**MVP = Phase 1 + Phase 2 + Phase 3 (US1)**
- 41 tasks covering Setup → Foundation → Complete SDD-RI Workflow
- Student outcome: Video file generated using Spec-Kit Plus workflow
- Validates: Core chapter value proposition before skills/capstone layers
