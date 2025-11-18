---
id: 0004
title: Actionable Task Generation
stage: tasks
date: 2025-01-18
surface: cli
model: claude-sonnet-4-5
feature: 001-011-chapter-11-context-engineering-rewrite
branch: 001-011-chapter-11-context-engineering-rewrite
user: mjs
command: /sp.tasks
labels: [tasks, user-story-organization, lesson-implementation, task-breakdown]
links:
  spec: specs/001-011-chapter-11-context-engineering-rewrite/spec.md
  ticket:
  adr:
  pr:
files:
  - path: specs/001-011-chapter-11-context-engineering-rewrite/tasks.md
    operation: create
    lines_changed: 427
tests:
  - name: Checklist Format Validation
    result: PASS
    notes: All 94 tasks follow format (checkbox + ID + [P]/[Story] labels + file paths)
  - name: User Story Organization
    result: PASS
    notes: Tasks grouped by 6 user stories (US1-US6) enabling independent implementation
  - name: Independent Test Criteria
    result: PASS
    notes: Each user story has clear independent test validating completion
---

## Prompt

Execute `/sp.tasks` workflow to generate actionable task breakdown organized by user story for immediate content-implementer execution.

**Context**: Chapter 11 Context Engineering rewrite with 6 user stories (P1-P3), 9-lesson plan (927 lines), 3 ADRs documenting pedagogical decisions.

**Task Generation Workflow**:
1. Load spec.md and extract 6 user stories with priorities (P1/P1/P2/P2/P3/P3)
2. Load plan.md and extract 9-lesson architecture
3. Map lessons to user stories (US1: Lessons 1-2, US2: Lesson 3, US3: Lessons 4-5, US4: Lesson 6, US5: Lesson 7, US6: Lesson 9, Validation: Lesson 8)
4. Generate tasks organized by user story in priority order
5. Include independent test criteria for each story
6. Create dependency graph showing completion order
7. Validate checklist format (checkbox + ID + [P]/[Story] labels + file paths)

**Critical Requirements**:
- NO automated tests (educational content, pedagogical validation only)
- Tasks enable independent user story implementation
- Each phase includes complete story with independent test
- Format: `- [ ] [ID] [P?] [Story] Description with file path`

## Response snapshot

**Generated tasks.md** (427 lines) with 94 tasks organized across 10 phases:

**Phase Structure**:
- Phase 1 (Setup): 4 tasks - Chapter directory initialization
- Phase 2 (Foundational): 6 tasks - Research verification, framework documentation
- Phase 3 (US1 - Lessons 1-2, P1): 10 tasks - Context window foundation, degradation symptoms
- Phase 4 (US2 - Lesson 3, P1): 10 tasks - Progressive loading with Three Roles
- Phase 5 (US3 - Lessons 4-5, P2): 12 tasks - Compression & isolation with Three Roles
- Phase 6 (US4 - Lesson 6, P2): 10 tasks - Memory files & persistence (Stage 3 intelligence)
- Phase 7 (US5 - Lesson 7, P3): 9 tasks - Tool selection framework
- Phase 8 (Validation - Lesson 8, P2): 9 tasks - Hands-on debugging scenarios
- Phase 9 (US6 - Lesson 9, P3): 12 tasks - Capstone spec-driven orchestration
- Phase 10 (Polish): 12 tasks - Cross-cutting validation, acceptance tests

**User Story Mapping**:
- US1 (P1): Lessons 1-2 (Manual foundation) - 10 tasks
- US2 (P1): Lesson 3 (Progressive loading) - 10 tasks
- US3 (P2): Lessons 4-5 (Compression & isolation) - 12 tasks
- US4 (P2): Lesson 6 (Memory files) - 10 tasks
- US5 (P3): Lesson 7 (Tool selection) - 9 tasks
- US6 (P3): Lesson 9 (Capstone) - 12 tasks
- Validation: Lesson 8 (Debugging) - 9 tasks

**Independent Test Criteria**:
- US1: Manual degradation diagnosis WITHOUT AI assistance
- US2: Progressive loading maintaining context <70% with Three Roles demonstrated
- US3: Compression vs isolation decision with reasoning
- US4: Memory file persistence across 2+ sessions
- US5: Tool selection with justified reasoning for 3 scenarios
- US6: Implementation-ready specification peer-reviewable (NO code)

**Dependency Graph**:
```
Phase 1 (Setup) → Phase 2 (Foundational) →
  Phase 3 (US1: MVP) →
    Phase 4 (US2) →
      Phase 5 (US3) →
        Phase 6 (US4) & Phase 7 (US5) →
          Phase 8 (Validation) →
            Phase 9 (US6: Capstone) →
              Phase 10 (Polish)
```

**Parallel Opportunities**:
- Phase 1: 3 tasks (T002-T004)
- Phase 2: 5 tasks (T006-T010)
- Phase 5: 2 file creation tasks (T031-T032)
- Phase 10: 9 validation tasks (T084-T091, T093)

**MVP Scope**: Phase 3 (US1: Lessons 1-2) - Manual foundation enabling all subsequent learning

**Format Validation**: ✅ All 94 tasks follow strict checklist format:
- Checkbox: `- [ ]`
- Task ID: Sequential (T001-T094)
- [P] marker: Parallel tasks labeled
- [Story] label: US1-US6 for user story tasks
- Description: Action + exact file path

## Outcome

- ✅ Impact: 94 actionable tasks generated, immediately executable by content-implementer agent
- 🧪 Tests: Checklist format (PASS), User story organization (PASS), Independent test criteria (PASS)
- 📁 Files: tasks.md (427 lines with 94 tasks across 10 phases)
- 🔁 Next prompts: /sp.analyze (cross-artifact consistency validation before implementation)
- 🧠 Reflection: Task generation successfully transformed 6 user stories + 9-lesson plan into executable workflow, enabling independent story implementation with clear test criteria and dependency management

## Evaluation notes (flywheel)

- Failure modes observed: None. Tasks demonstrate proper user story organization, complete independent test criteria, correct checklist format
- Graders run and results (PASS/FAIL): PASS - All 94 tasks include checkbox + ID + labels + file paths, user stories independently testable
- Prompt variant (if applicable): Standard task generation with educational content adaptation (no automated tests)
- Next experiment (smallest change to try): N/A - workflow succeeded, tasks ready for implementation phase
