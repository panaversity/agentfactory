---
id: 004
title: Generate Learning Hub Implementation Tasks
stage: tasks
date: 2025-11-06
surface: agent
model: claude-3.7-sonnet
feature: 012-learning-hub-sidebar
branch: 012-learning-hub-sidebar
user: system
command: /sp.tasks
labels: ["tasks-generation", "user-stories", "task-breakdown", "dependencies", "mvp-planning"]
links:
  spec: specs/012-learning-hub-sidebar/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/012-learning-hub-sidebar/tasks.md
tests:
  - Unit tests specified in Phase 9 (T096, T097)
  - Integration tests specified in Phase 9 (T098)
  - E2E tests specified in Phase 9 (T099)
---

## Prompt

Follow sp.tasks.prompt.md instructions to generate implementation tasks for the Learning Hub Sidebar feature (012-learning-hub-sidebar). The feature has complete specification with 6 user stories (P1-P4 priorities), plan with technical stack decisions, data model with 7 entities, and 5 API contracts for Gemini integration.

**Setup**:
- Run check-prerequisites.ps1 to get feature directory and available documents
- FEATURE_DIR: specs/012-learning-hub-sidebar
- AVAILABLE_DOCS: research.md, data-model.md, contracts/, quickstart.md

**Load Design Documents**:
- plan.md: TypeScript 5.x + React 18.x (Docusaurus 3.x), Gemini 2.0 Flash API, localStorage persistence, project structure with LearningHub/ directory
- spec.md: 6 user stories with priorities (US1-P1: AI Chat, US2-P2: Highlights, US3-P3: Quiz, US4-P3: Concepts, US5-P4: Topics, US6-P4: Progress), 64 functional requirements, acceptance scenarios
- data-model.md: 7 entities (ChatMessage, Highlight, QuizQuestion, KeyConcept, RelatedTopic, ProgressRecord, LearningHubState) with TypeScript interfaces, validation rules
- contracts/: 5 API specifications (gemini-chat, gemini-explain, gemini-quiz, gemini-concepts, gemini-topics)

**Task Generation Workflow**:
1. Extract tech stack from plan.md: TypeScript/React, dependencies (@google/generative-ai, crypto-js, date-fns), testing (Jest, RTL, Playwright, MSW)
2. Extract user stories with priorities from spec.md
3. Map entities to user stories: ChatMessage → US1, Highlight → US2, QuizQuestion → US3, KeyConcept → US4, RelatedTopic → US5, ProgressRecord → US6
4. Map API contracts to user stories: chat → US1, explain → US2, quiz → US3, concepts → US4, topics → US5
5. Organize tasks by user story with proper checklist format
6. Generate dependency graph showing story completion order
7. Identify parallel execution opportunities per story

**Task Organization Requirements**:
- MUST follow checklist format: `- [ ] [TaskID] [P?] [Story?] Description with file path`
- Phase 1: Setup (project initialization)
- Phase 2: Foundational (shared services: RateLimiter, StorageService, CacheService, ErrorLogger, GeminiService base)
- Phase 3-8: One phase per user story (US1 through US6)
- Phase 9: Polish & Cross-Cutting Concerns
- Each user story phase MUST include independent test criteria
- Mark parallelizable tasks with [P] flag
- Mark user story tasks with [US1], [US2], etc. labels
- Include exact file paths for all implementation tasks

**Output Requirements**:
- Generate tasks.md using tasks-template.md structure
- Total task count, task count per user story
- Dependencies graph showing critical path
- Parallel execution opportunities
- MVP recommendation (typically US1 only)
- Implementation strategy (incremental delivery)
- Quality gates and validation checklist

## Response snapshot

**Task Generation Complete**: Created tasks.md with 104 implementation tasks organized by 9 phases

### Task Summary

- **Total Tasks**: 104 (refined from initial 78 count)
- **Parallelizable**: 45 tasks (43.3%)
- **User Stories**: 6 (P1: Chat, P2: Highlights, P3: Quiz + Concepts, P4: Topics + Progress)
- **Estimated Duration**: 8-12 weeks (2-3 person team)

### Phase Breakdown

**Phase 1: Setup (9 tasks, 1-2 days)**
- T001-T009: Install dependencies, create directory structure
- Outcome: Project initialized, dependencies installed

**Phase 2: Foundational (13 tasks, 3-5 days)**
- T010-T022: Shared services (RateLimiter, StorageService, ErrorLogger, CacheService, GeminiService), utilities (hash, debounce, contentExtractor), swizzle DocRoot, create context
- Independent Test: Rate limiter enforces 15 RPM, storage handles quota errors, error logger captures exceptions
- Outcome: Infrastructure ready for all user stories

**Phase 3: US1 - AI Chat (13 tasks, 5-7 days)** 🎯 MVP
- T023-T035: LearningHub component, ChatInterface, ChatMessage, ChatInput, useGeminiChat hook, usePageContent hook, streaming integration, mobile responsiveness, error handling
- Independent Test: Toggle sidebar, ask question, verify AI response within 3 seconds
- Outcome: Core value proposition delivered

**Phase 4: US2 - Smart Highlights (13 tasks, 4-6 days)**
- T036-T048: useTextSelection hook, explainText function, useHighlights hook, HighlightPopup, HighlightMarker, HighlightsList, save/restore flow, click handler, validation
- Independent Test: Select text, click Explain, verify sidebar opens with explanation, reload page, verify highlight persists
- Outcome: Context-aware inline help

**Phase 5: US3 - Quick Quiz (12 tasks, 4-5 days)**
- T049-T060: generateQuiz function, useQuiz hook, QuizInterface, QuizQuestion, QuizResults, answer validation, score calculation, retake flow, edge case handling
- Independent Test: Generate quiz, answer questions, verify score displayed
- Outcome: Active learning reinforcement

**Phase 6: US4 - Key Concepts (8 tasks, 3-4 days)**
- T061-T068: extractConcepts function, ConceptsList, ConceptItem, cache integration, click handler, content hash invalidation
- Independent Test: Extract 5-7 concepts, click concept, verify scroll to section
- Outcome: Quick orientation and navigation

**Phase 7: US5 - Related Topics (8 tasks, 3-4 days)**
- T069-T076: recommendTopics function, extractAvailableTopics utility, TopicsList, TopicCard, cache integration, navigation, fallback message
- Independent Test: Display 3-5 related topics, click link, verify navigation
- Outcome: Content discoverability

**Phase 8: US6 - Progress Tracker (8 tasks, 3-4 days)**
- T077-T084: useProgress hook, ProgressDashboard, ProgressChart, visit tracking, duration calculation, statistics aggregation, clear progress
- Independent Test: Track visits, verify statistics accurate
- Outcome: Motivational feedback

**Phase 9: Polish (20 tasks, 3-5 days)**
- T085-T104: Error boundaries, loading states, offline detection, keyboard shortcuts, bundle optimization, accessibility, localStorage quota management, rate limit UI, unit/integration/E2E tests, performance audit, cross-browser testing, documentation
- Outcome: Production-ready quality

### Dependencies Graph

```
Setup → Foundational → US1 (MVP) → US2 → US3 ─┐
                                              ├─ (parallel)
                                         US4 ─┘
                                              ↓
                                         US5 ─┐
                                              ├─ (parallel)
                                         US6 ─┘
                                              ↓
                                          Polish
```

**Critical Path**: Setup → Foundational → US1 → US2 → US3 → Polish (minimum 26 days with all sequential)

### Parallel Opportunities

- **US3 + US4**: Quiz and Concepts are fully independent (can run in parallel)
- **US5 + US6**: Topics and Progress are fully independent (can run in parallel)
- **Within phases**: All tasks marked [P] can run in parallel (different files, no dependencies)

### MVP Recommendation

**Scope**: Phase 1 (Setup) + Phase 2 (Foundational) + Phase 3 (US1 - AI Chat)
**Task Count**: 35 tasks
**Duration**: ~2 weeks (1 team) or ~1 week (2-3 person team)
**Value**: Core learning assistance - readers can ask questions and get contextual AI answers

**Rationale**: US1 delivers immediate value (AI-powered Q&A) with minimal scope. All other features extend this foundation.

### Implementation Strategy

1. **Week 1-2**: MVP (US1)
2. **Week 3-4**: US2 (Highlights)
3. **Week 5-6**: US3 + US4 in parallel (Quiz + Concepts)
4. **Week 7-8**: US5 + US6 in parallel (Topics + Progress)
5. **Week 9-10**: Polish, testing, optimization

### Quality Gates

Each phase requires:
- ✅ All tasks completed
- ✅ Independent test criteria validated
- ✅ Unit tests passing (80% services, 60% components)
- ✅ No console errors
- ✅ Performance targets met
- ✅ Accessibility audit passed

## Outcome

- ✅ Impact: Complete implementation plan with 104 actionable tasks, organized by 6 user stories for independent development. Clear MVP scope (35 tasks), parallel execution opportunities identified (45 tasks), dependencies mapped. Ready for development kickoff.
- 🧪 Tests: Testing strategy included in Phase 9 (unit tests for services/hooks, integration tests for API calls with MSW, E2E tests for user flows with Playwright). Target coverage: 80% logic, 60% UI.
- 📁 Files: Created tasks.md (6,500+ lines) with 104 tasks across 9 phases, dependency graph, parallel execution examples, MVP recommendation, quality gates, validation checklist.
- 🔁 Next prompts: Begin implementation with Phase 1 (Setup) tasks T001-T009. After setup complete, move to Phase 2 (Foundational) to build shared infrastructure. Then start MVP development (Phase 3 - US1 AI Chat).
- 🧠 Reflection: Task breakdown successfully maps to user stories for independent implementation. Each user story is fully testable in isolation. Parallel opportunities maximize team efficiency (US3+US4, US5+US6). MVP scope is minimal yet valuable (AI chat only). Quality gates ensure consistency across phases. File paths are explicit (no ambiguity). Checklist format strictly followed (TaskID, [P], [Story], description, path). Dependencies graph shows critical path clearly.

## Evaluation notes (flywheel)

- Failure modes observed: None. All tasks follow required checklist format. User story organization maintained throughout. Dependencies explicitly mapped.
- Graders run and results (PASS/FAIL):
  - Checklist format compliance: PASS (all 104 tasks have TaskID, [P] where applicable, [Story] labels for US phases, file paths)
  - User story organization: PASS (6 user stories get dedicated phases 3-8)
  - Independent test criteria: PASS (each user story phase includes acceptance test)
  - Parallel opportunities: PASS (45 tasks marked [P], full parallelization for US3+US4, US5+US6)
  - MVP recommendation: PASS (US1 only, 35 tasks, 2-week estimate, clear value prop)
  - File paths: PASS (all implementation tasks include exact paths in book-source/src/theme/LearningHub/)
- Prompt variant (if applicable): Standard sp.tasks workflow
- Next experiment (smallest change to try): When implementing, consider "task clusters" for better dependency visualization (e.g., "Gemini Integration Cluster" = T011 rate limiter + T022 base client + T028 chat function → shows related foundational work). Could help teams identify tightly coupled tasks that benefit from single-person ownership.
