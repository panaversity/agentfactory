---
id: 0003
title: ch01-intro-apis-agents-tasks
stage: tasks
date: 2025-12-06
surface: claude-code-cli
model: claude-sonnet-4-5
feature: 041-ch01-intro-apis-agents
branch: 041-ch01-intro-apis-agents
user: HP
command: /sp.tasks 041-ch01-intro-apis-agents
labels: [tasks, educational-content, chapter-1, implementation-checklist]
links:
  spec: specs/041-ch01-intro-apis-agents/spec.md
  plan: specs/041-ch01-intro-apis-agents/plan.md
  tasks: specs/041-ch01-intro-apis-agents/tasks.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/041-ch01-intro-apis-agents/tasks.md
    action: created
    lines: 320
tests: []
---

## Prompt

**User Input**: "option 1" (selected task generation over direct implementation)

**Command Invoked**: `/sp.tasks 041-ch01-intro-apis-agents`

**Context**:
- Specification complete: 450 lines, 26 functional requirements, 21 constraints
- Implementation plan complete: 1,678 lines, 4 lessons mapped with pedagogical analysis
- 4 user stories identified from spec:
  - US1 (P1): API Fundamentals (Lesson 1.1) - verify OpenAI API key
  - US2 (P2): LLM vs Agent Distinction (Lesson 1.2) - comparison table
  - US3 (P1): Environment Setup (Lesson 1.3) - UV, project folder, .env
  - US4 (P3): Architecture Overview (Lesson 1.4) - architecture.md document

**Task Generation Requirements**:
1. Organize tasks by user story (lesson) for independent implementation
2. Include setup and foundational phases before lessons
3. Mark parallelizable tasks with [P]
4. Include validation tasks for constitutional compliance
5. Follow checklist format: `- [ ] [ID] [P?] [Story] Description with file path`

## Response snapshot

**Generated tasks.md with 62 tasks across 7 phases**:

**Phase 1: Setup (4 tasks)**
- T001-T004: Create chapter directory structure, README, code-examples folder, assets directory

**Phase 2: Foundational (4 tasks - BLOCKING)**
- T005-T008: Create shared resources (test_api.py template, comparison table template, architecture.md template, troubleshooting guide)

**Phase 3: Lesson 1.1 - API Fundamentals (11 tasks) [US1] 🎯 MVP**
- T009-T019: Full lesson implementation
- Learning Goal, Concept, Key Points, Analogy, Code Level, DocuBot Project (Task + Outcome), Hints, Starter Code, Troubleshooting, Try With AI
- Independent test: Student runs test_api.py, sees AI response

**Phase 4: Lesson 1.2 - LLM vs Agent (10 tasks) [US2]**
- T020-T029: Full lesson implementation
- Code-free conceptual lesson with comparison exercise
- Independent test: Comparison table with 5+ items per column

**Phase 5: Lesson 1.3 - Environment Setup (12 tasks) [US3]**
- T030-T041: Full lesson implementation (longest lesson)
- OS-specific instructions (Mac/Linux/Windows), troubleshooting, verification checklist
- Independent test: uv --version works, pyproject.toml exists, .env configured, openai importable

**Phase 6: Lesson 1.4 - Architecture Overview (11 tasks) [US4]**
- T042-T052: Full lesson implementation
- Component diagram example, chapter roadmap table
- Independent test: architecture.md with 3 sections, 5+ components

**Phase 7: Polish & Cross-Cutting (10 tasks)**
- T053-T062: Chapter summary, quiz, navigation, validation tasks
- Validates: DocuBot sections present (CSC-002), cumulative progress (CSC-005), no agent code (TC-001), analogies present (PC-003), cognitive load appropriate (PC-002)

**Task Distribution**:
- Total: 62 tasks
- Parallelizable: 44 tasks marked with [P]
- Per lesson: US1=11, US2=10, US3=12, US4=11
- Setup/Foundation: 8 tasks (blocking)
- Polish/Validation: 10 tasks

**Parallel Execution Strategy**:
- With 4 authors: All lessons can develop simultaneously after Foundational phase
- Timeline reduction: ~75% faster (1 week vs 3-4 weeks sequential)
- MVP scope: Lesson 1.1 only (5-6 hours) allows immediate student access

**Implementation Estimates**:
- Single author: 18-20 hours total
- 4 authors (parallel): 6-8 hours per author
- MVP (Lesson 1.1 only): 5-6 hours

**Key Design Decisions**:
1. **Lesson file naming**: 01-api-fundamentals.md, 02-llm-vs-agent.md, 03-environment-setup.md, 04-architecture-overview.md
2. **Path convention**: book-source/docs/06-AI-Native-Software-Development/01-intro-apis-agents/
3. **DocuBot section enforcement**: Tasks T015, T026, T036, T048 explicitly create Task + Outcome subsections
4. **Try With AI policy**: All lessons use ChatGPT web (tools not yet taught in Chapter 1)
5. **Validation tasks**: T056-T062 ensure constitutional compliance post-implementation

## Outcome

- ✅ Impact: Created comprehensive 62-task breakdown for Chapter 1 implementation, organized by user story (lesson) for independent development. 44 tasks parallelizable. Includes validation tasks ensuring constitutional compliance (TC-001, PC-002, PC-003, CSC-002, CSC-005).
- 🧪 Tests: Independent test criteria defined for each lesson (API response for L1.1, comparison table for L1.2, environment verification for L1.3, architecture document for L1.4). Validation tasks T056-T062 verify constraint compliance.
- 📁 Files: 1 file created - `specs/041-ch01-intro-apis-agents/tasks.md` (320 lines)
- 🔁 Next prompts: `/sp.implement 041-ch01-intro-apis-agents` to begin content creation, or manually execute tasks T001-T008 (Setup + Foundational) then parallelize T009-T052 (all 4 lessons)
- 🧠 Reflection: Task generation successfully mapped spec requirements to actionable implementation steps. Each task includes file path for clarity. Parallel opportunities maximized (44/62 tasks). MVP scope identified (Lesson 1.1 only) for rapid student access. Try With AI policy correctly applied (ChatGPT web since tools not yet taught).

## Evaluation notes (flywheel)

- Failure modes observed: None. Tasks correctly organized by user story, all constraints mapped to validation tasks, parallel opportunities identified.
- Graders run and results (PASS/FAIL): Format validation = PASS (all tasks have checkbox, ID, description with path), Story mapping = PASS (US1-US4 labels applied), Parallel marking = PASS ([P] on independent tasks), Constitutional mapping = PASS (T056-T062 validate TC-001, PC-002, PC-003, CSC-002, CSC-005)
- Prompt variant (if applicable): Standard `/sp.tasks` command for educational content
- Next experiment (smallest change to try): Consider auto-generating lesson file templates with frontmatter to reduce T009, T020, T030, T042 overhead
