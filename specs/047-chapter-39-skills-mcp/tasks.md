# Tasks: Chapter 39 - Agent Skills & MCP Code Execution

**Input**: Design documents from `/specs/047-chapter-39-skills-mcp/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Chapter content**: `apps/learn-app/docs/06-AI-Native-Software-Development/39-agent-skills-mcp-code-execution/`
- **Skills**: `.claude/skills/`
- **Quality reference**: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-the-2025-inflection-point.md`

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Chapter README and structure preparation

- [ ] T001 Update chapter README at `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/06-AI-Native-Software-Development/39-agent-skills-mcp-code-execution/README.md` with learning arc, prerequisites, and success criteria from plan.md

---

## Phase 2: Foundational (Lessons 1-2) - Advanced Skill Patterns

**Purpose**: Foundation for all execution skills - persona, principles, composition
**Maps to**: FR-001, FR-002, FR-003, FR-004

### Lesson 1: Advanced Skill Patterns (L1 - Manual Foundation)

- [ ] T002 [L1] Lesson 1: Advanced Skill Patterns — Persona, Principles, Composition
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/06-AI-Native-Software-Development/39-agent-skills-mcp-code-execution/01-advanced-skill-patterns.md`
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-the-2025-inflection-point.md`
  - **SKILLS**:
    - `learning-objectives` (generate measurable outcomes: Bloom's Understand + Apply)
    - `exercise-designer` (manual exercise: design skill persona for student's domain)
  - **KEY CONCEPTS** (8): Skill persona definition, persona patterns for execution skills, Questions framework, Principles for behavior constraints, skill composition, skill references, reusability criteria, Layer 3 vs Layer 4 decisions
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

### Lesson 2: Skill Composition & Dependencies (L2 - AI Collaboration)

- [ ] T003 [L2] Lesson 2: Skill Composition & Dependencies
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/06-AI-Native-Software-Development/39-agent-skills-mcp-code-execution/02-skill-composition.md`
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-the-2025-inflection-point.md`
  - **SKILLS**:
    - `learning-objectives` (generate measurable outcomes: Bloom's Apply + Analyze)
    - `exercise-designer` (compose 2-3 existing skills, test integration)
    - `ai-collaborate-teaching` (Three Roles: AI teaches composition patterns, student teaches constraints, iterate on error handling)
  - **KEY CONCEPTS** (8): Skill dependency declaration, composition patterns, data flow between skills, error propagation, testing composed skills, circular dependency detection, skill versioning, capstone composition
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

**Checkpoint**: Foundation complete - students understand advanced skill patterns needed for execution skills

---

## Phase 3: User Story 1 - Build Skill that Wraps MCP (Priority: P1) 🎯 MVP

**Goal**: Students create skills that wrap MCP servers with intelligence layer (like `fetching-library-docs`)
**Maps to**: FR-005, FR-006, FR-007, FR-008, SC-001, SC-002

**Independent Test**: Student's MCP-wrapping skill triggers correctly, orchestrates MCP calls, and filters results with 30%+ token reduction

### Lesson 3: Anatomy of MCP-Wrapping Skills (L1 - Manual Foundation)

- [ ] T004 [US1] Lesson 3: Anatomy of MCP-Wrapping Skills
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/06-AI-Native-Software-Development/39-agent-skills-mcp-code-execution/03-mcp-wrapping-anatomy.md`
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-the-2025-inflection-point.md`
  - **SKILLS**:
    - `learning-objectives` (generate measurable outcomes: Bloom's Analyze)
    - `exercise-designer` (design MCP-wrapping skill spec on paper, no coding)
  - **REFERENCE SKILLS TO ANALYZE**:
    - `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/.claude/skills/fetching-library-docs/SKILL.md`
    - `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/.claude/skills/browsing-with-playwright/SKILL.md`
  - **KEY CONCEPTS** (9): MCP server vs skill wrapper, intelligence layer, trigger conditions, result filtering, error recovery, MCP call optimization, skill-specific configuration, fallback strategies, token efficiency (934→205 example)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

### Lesson 4: Build Your MCP-Wrapping Skill (L2 - AI Collaboration)

- [ ] T005 [US1] Lesson 4: Build Your MCP-Wrapping Skill
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/06-AI-Native-Software-Development/39-agent-skills-mcp-code-execution/04-build-mcp-wrapping-skill.md`
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-the-2025-inflection-point.md`
  - **SKILLS**:
    - `learning-objectives` (generate measurable outcomes: Bloom's Apply + Create)
    - `exercise-designer` (spec-first skill implementation with MCP server from Ch38)
    - `ai-collaborate-teaching` (Three Roles: AI teaches MCP client patterns, student teaches filtering priorities, iterate on filtering heuristics)
  - **KEY CONCEPTS** (9): Spec.md for skill, persona implementation, questions as filtering criteria, MCP client initialization, tool invocation, result filtering logic, error handling, skill testing, documentation
  - **DELIVERABLE**: MCP-Wrapping Skill Template (reusable for any MCP server)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

**Checkpoint**: User Story 1 complete - students can build MCP-wrapping skills with proper triggering and filtering

---

## Phase 4: User Story 2 - Build Skill that Executes Scripts (Priority: P1)

**Goal**: Students create skills that write Python/Bash scripts, execute them, analyze results, and iterate
**Maps to**: FR-009, FR-010, FR-011, FR-012, FR-013, SC-003, SC-004

**Independent Test**: Student's script-execution skill handles syntax and runtime errors, iterates to solution

### Lesson 5: Script Execution Fundamentals (L1 - Manual Foundation)

- [ ] T006 [US2] Lesson 5: Script Execution Fundamentals — Write-Execute-Analyze Loop
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/06-AI-Native-Software-Development/39-agent-skills-mcp-code-execution/05-script-execution-fundamentals.md`
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-the-2025-inflection-point.md`
  - **SKILLS**:
    - `learning-objectives` (generate measurable outcomes: Bloom's Understand + Apply)
    - `exercise-designer` (manual walkthrough of 2-3 script execution iterations, no coding)
  - **REFERENCE**: Anthropic code execution pattern from https://www.anthropic.com/engineering/code-execution-with-mcp
  - **KEY CONCEPTS** (9): Write-execute-analyze loop, syntax error detection, runtime error handling, timeout/resource constraints, partial execution state, safety constraints, output parsing, convergence criteria, comparison to MCP approach
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

### Lesson 6: Build Script-Execution Skill (L2 - AI Collaboration)

- [ ] T007 [US2] Lesson 6: Build Script-Execution Skill
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/06-AI-Native-Software-Development/39-agent-skills-mcp-code-execution/06-build-script-execution-skill.md`
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-the-2025-inflection-point.md`
  - **SKILLS**:
    - `learning-objectives` (generate measurable outcomes: Bloom's Apply + Create)
    - `exercise-designer` (csv-analysis-skill or data-transformation-skill with error scenarios)
    - `ai-collaborate-teaching` (Three Roles: AI teaches error parsing, student teaches data validation needs, iterate on robustness)
  - **KEY CONCEPTS** (9): Skill spec for script execution, script generation prompting, execution environment, error detection/parsing, iterative fix generation, iteration limits, output validation, state management, logging/observability
  - **DELIVERABLE**: Script-Execution-Skill Template (reusable for any computational task)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

**Checkpoint**: User Story 2 complete - students can build script-execution skills with write-execute-analyze loop

---

## Phase 5: User Story 3 & 4 - Workflow Orchestration with Error Recovery (Priority: P2)

**Goal**: Combine MCP + scripts into orchestrated workflows with proper error recovery
**Maps to**: FR-014, FR-015, FR-016, SC-004

**Independent Test**: Student's orchestrated skill handles multiple failure scenarios with state recovery

### Lesson 7: Full Workflow Orchestration (L3 - Intelligence Design)

- [ ] T008 [US3] [US4] Lesson 7: Full Workflow Orchestration — MCP + Scripts + Error Recovery
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/06-AI-Native-Software-Development/39-agent-skills-mcp-code-execution/07-workflow-orchestration.md`
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-the-2025-inflection-point.md`
  - **SKILLS**:
    - `learning-objectives` (generate measurable outcomes: Bloom's Analyze + Create)
    - `exercise-designer` (compose MCP-wrapping + script-execution skills into workflow)
  - **KEY CONCEPTS** (9): Workflow orchestration patterns, state management, error propagation/recovery, convergence criteria, logging/observability, performance optimization, fallback/retry logic, workflow specification, testing orchestrated workflows
  - **DELIVERABLE**: Orchestration Skill (reusable pattern for MCP + script combination)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

**Checkpoint**: User Stories 3 & 4 complete - students can orchestrate multi-skill workflows with error recovery

---

## Phase 6: User Story 5 - Ship Complete Code-Execution Skill (Priority: P3) 🎯 Capstone

**Goal**: Students build and ship a complete execution skill as Digital FTE
**Maps to**: FR-017, FR-018, FR-019, SC-005

**Independent Test**: Student's capstone skill passes acceptance tests, is documented, and packageable

### Lesson 8: Capstone — Shippable Agent Skill (L4 - Spec-Driven Integration)

- [ ] T009 [US5] Lesson 8: Capstone — Shippable Agent Skill
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/06-AI-Native-Software-Development/39-agent-skills-mcp-code-execution/08-capstone-shippable-skill.md`
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/mcp-3/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-the-2025-inflection-point.md`
  - **SKILLS**:
    - `learning-objectives` (generate measurable outcomes: Bloom's Create + Evaluate)
    - `exercise-designer` (full capstone project with spec → implementation → validation)
    - `ai-collaborate-teaching` (Three Roles: specification-driven implementation)
    - `assessment-builder` (capstone grading rubric: 25 points across 5 categories)
  - **KEY CONCEPTS** (10+): Domain specification, skill composition architecture, user interface design, safety/governance, acceptance testing, performance/cost metrics, documentation, deployment packaging, monitoring, monetization strategy, competitive differentiation
  - **DELIVERABLE**: Domain-Specific Execution Skill (Digital FTE)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

**Checkpoint**: User Story 5 complete - students have shippable execution skill as Digital FTE

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Quality validation across all lessons

- [ ] T010 [P] Run educational-validator on all 8 lessons for constitutional compliance
- [ ] T011 [P] Run fact-check-lesson on all lessons for technical accuracy
- [ ] T012 Verify all success criteria from spec.md are addressed:
  - SC-001: Lesson 3 teaches analysis of MCP-wrapping skills
  - SC-002: Lesson 4 exercise produces MCP-wrapping skill
  - SC-003: Lesson 6 exercise produces script-execution skill
  - SC-004: Lessons 6-7 teach error recovery
  - SC-005: Lesson 8 capstone produces shippable skill
  - SC-006: 90% exercises use AI collaboration
  - SC-007: B2 proficiency maintained (7-10 concepts per lesson)
- [ ] T013 [P] Update chapter README with final lesson titles and durations
- [ ] T014 Commit chapter with `/sp.git.commit_pr`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - establishes skill pattern foundation
- **User Story 1 (Phase 3)**: Depends on Foundational - MCP-wrapping skills
- **User Story 2 (Phase 4)**: Depends on Foundational - script-execution skills (can run parallel to US1)
- **User Stories 3-4 (Phase 5)**: Depends on US1 AND US2 completion - combines both skill types
- **User Story 5 (Phase 6)**: Depends on all previous - capstone integrates everything
- **Polish (Phase 7)**: Depends on all lessons complete

### Parallel Opportunities

**Within Phases 3 & 4** (after Foundational complete):
- Lesson 3 and Lesson 5 can run in parallel (both L1 foundation)
- Lesson 4 and Lesson 6 can run in parallel (both L2 implementation)

**Phase 7**:
- All validation tasks (T010, T011, T013) can run in parallel

---

## Parallel Example: Lessons 3 & 5

```bash
# After Phase 2 complete, launch L1 foundation lessons in parallel:
Task: "Lesson 3: Anatomy of MCP-Wrapping Skills"
Task: "Lesson 5: Script Execution Fundamentals"

# Then launch L2 implementation lessons in parallel:
Task: "Lesson 4: Build Your MCP-Wrapping Skill"
Task: "Lesson 6: Build Script-Execution Skill"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (README update)
2. Complete Phase 2: Foundational (Lessons 1-2)
3. Complete Phase 3: User Story 1 (Lessons 3-4)
4. **STOP and VALIDATE**: Students can build MCP-wrapping skills
5. Deploy/demo MVP

### Incremental Delivery

1. Phase 1 + 2 → Foundation ready
2. Add Phase 3 (US1: MCP-wrapping) → Test independently
3. Add Phase 4 (US2: Script execution) → Test independently
4. Add Phase 5 (US3-4: Orchestration) → Test independently
5. Add Phase 6 (US5: Capstone) → Full chapter complete
6. Phase 7 → Polish and commit

---

## Summary

| Metric | Count |
|--------|-------|
| **Total Tasks** | 14 |
| **Lessons** | 8 |
| **User Stories Covered** | 5 |
| **Parallel Opportunities** | 6 |
| **Skill Templates Created** | 3 (MCP-Wrapping, Script-Execution, Orchestration) |
| **Digital FTE Produced** | 1 (Capstone) |

**Suggested MVP Scope**: Phase 1-3 (Lessons 1-4) - Students can build MCP-wrapping skills

---

## Notes

- Each lesson task includes SUBAGENT, SKILLS, and VALIDATION blocks per content work requirements
- All output paths are absolute
- Quality reference lesson specified for all content-implementer invocations
- Three Roles (ai-collaborate-teaching) required for L2+ lessons
- Assessment-builder invoked for capstone grading rubric
