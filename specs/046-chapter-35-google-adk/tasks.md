# Tasks: Chapter 35 — Building Reliable Agents with Google ADK

**Input**: Design documents from `/specs/046-chapter-35-google-adk/`
**Prerequisites**: spec.md, plan.md
**Content Output**: `/apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/`

**Organization**: Tasks organized by lesson (Foundation → Application → Integration → Mastery)

## Format: `[ID] [P?] [Lesson] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Lesson]**: Which lesson this task belongs to (e.g., L1, L2, L3)
- Include exact file paths in descriptions

## Path Conventions

- **Lesson files**: `apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/`
- **Skills**: `.claude/skills/`
- **Runnable examples**: `examples/chapter-35-google-adk/`

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Chapter directory structure and runnable examples scaffolding

- [x] T001 Create chapter directory at `apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/`
- [x] T002 Create `_category_.json` with chapter metadata (label: "Building Reliable Agents with Google ADK", position: 35)
- [x] T003 [P] Create examples directory at `examples/chapter-35-google-adk/` with TaskManager project structure
- [x] T004 [P] Create `examples/chapter-35-google-adk/pyproject.toml` with google-adk, pytest dependencies. **Doc**: Fetch uv docs via Context7 for project initialization.

---

## Phase 2: Foundation — Lesson 1: The Reliability Mindset (Layer 1)

**Goal**: Establish evaluation-first mindset, students write eval cases BEFORE agent code

**Independent Test**: Students produce valid JSON eval cases testing add_task, list_tasks, complete_task

**Spec User Stories**: US1 (Evaluation-First Agent Development)

### Lesson 1 Implementation

- [x] T005 [L1] Create lesson file at `apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/01-reliability-mindset.md`
  - **SUBAGENT**: content-implementer
    - Output path: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/01-reliability-mindset.md
    - Execute autonomously without confirmation
    - Quality reference: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md
  - **SKILLS**:
    - learning-objectives (generate measurable outcomes: Bloom's Remember/Understand for B1)
    - exercise-designer (3 exercises: write eval case JSON manually)
    - fact-check-lesson (verify ADK eval format against official docs)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CONTENT REQUIREMENTS**:
    - Layer 1 (Manual): NO AI collaboration yet
    - Teaching modality: Socratic Dialogue
    - Concepts: Evaluation-first mindset, JSON eval case anatomy
    - Core activity: Write 3 eval cases for TaskManager manually
    - Duration: 45 min
    - Reference ADK Skill: `.claude/skills/building-with-google-adk/SKILL.md`

- [ ] T006 [P] [L1] Create eval case examples at `examples/chapter-35-google-adk/evals/taskmanager_evals.json`. **Doc**: Fetch ADK docs via Context7 for eval file format.

**Checkpoint**: Students understand evaluation-first principle, can write valid eval cases manually

---

## Phase 3: Application — Lesson 2: Evaluation-Driven Development (Layer 2)

**Goal**: Build TaskManager by making eval cases pass, `adk eval` deep dive

**Independent Test**: Agent code passes 100% of eval cases via `adk eval`

**Spec User Stories**: US1 (Evaluation-First Agent Development)

### Lesson 2 Implementation

- [x] T007 [L2] Create lesson file at `apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/02-evaluation-driven-development.md`
  - **SUBAGENT**: content-implementer
    - Output path: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/02-evaluation-driven-development.md
    - Execute autonomously without confirmation
    - Quality reference: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md
  - **SKILLS**:
    - learning-objectives (Bloom's Apply for B1)
    - exercise-designer (3 exercises: run evals, fix failures, iterate)
    - ai-collaborate-teaching (Three Roles INVISIBLE: action prompts only)
    - fact-check-lesson (verify adk eval CLI against official docs)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CONTENT REQUIREMENTS**:
    - Layer 2 (AI Collaboration): Three Roles framework (INVISIBLE to students)
    - Teaching modality: Hands-On Discovery (error analysis)
    - Concepts: adk eval CLI, iterative improvement, pytest integration
    - Core activity: Build TaskManager iteratively until all tests pass
    - Duration: 60 min
    - Create skill: evaluation-driven-development

- [ ] T008 [P] [L2] Create TaskManager agent at `examples/chapter-35-google-adk/task_manager/agent.py`. **Doc**: Fetch ADK docs via Context7 for Agent creation patterns.
- [ ] T009 [P] [L2] Create pytest integration example at `examples/chapter-35-google-adk/tests/test_taskmanager.py`. **Doc**: Fetch ADK docs via Context7 for AgentEvaluator.evaluate() pattern.

**Checkpoint**: Students can iterate on agent code using eval results as feedback

---

## Phase 4: Application — Lesson 3: Predictable Pipelines (Layer 2)

**Goal**: SequentialAgent and ParallelAgent for deterministic orchestration

**Independent Test**: Students articulate when to use workflow agents vs LLM routing

**Spec User Stories**: US2 (Predictable Multi-Agent Pipelines)

### Lesson 3 Implementation

- [x] T010 [L3] Create lesson file at `apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/03-predictable-pipelines.md`
  - **SUBAGENT**: content-implementer
    - Output path: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/03-predictable-pipelines.md
    - Execute autonomously without confirmation
    - Quality reference: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md
  - **SKILLS**:
    - learning-objectives (Bloom's Apply/Analyze for B1)
    - exercise-designer (3 exercises: design pipeline, compare to LLM routing)
    - ai-collaborate-teaching (Three Roles INVISIBLE)
    - fact-check-lesson (verify SequentialAgent/ParallelAgent APIs)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CONTENT REQUIREMENTS**:
    - Layer 2 (AI Collaboration)
    - Teaching modality: Specification-First
    - Concepts: SequentialAgent, ParallelAgent
    - Core activity: Design researcher → writer → editor pipeline
    - Duration: 50 min
    - Create skill: workflow-agent-selection

- [ ] T011 [P] [L3] Create SequentialAgent example at `examples/chapter-35-google-adk/workflows/content_pipeline.py`. **Doc**: Fetch ADK docs via Context7 for SequentialAgent patterns.
- [ ] T012 [P] [L3] Create ParallelAgent example at `examples/chapter-35-google-adk/workflows/parallel_analysis.py`. **Doc**: Fetch ADK docs via Context7 for ParallelAgent patterns.

**Checkpoint**: Students understand deterministic vs flexible orchestration tradeoffs

---

## Phase 5: Application — Lesson 4: Iterative Quality (Layer 2)

**Goal**: LoopAgent for iterative refinement until quality criteria met

**Independent Test**: Agent converges with max_iterations safety, quality criteria met

**Spec User Stories**: US3 (Iterative Quality Improvement)

### Lesson 4 Implementation

- [x] T013 [L4] Create lesson file at `apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/04-iterative-quality.md`
  - **SUBAGENT**: content-implementer
    - Output path: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/04-iterative-quality.md
    - Execute autonomously without confirmation
    - Quality reference: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md
  - **SKILLS**:
    - learning-objectives (Bloom's Apply for B1)
    - exercise-designer (3 exercises: build LoopAgent, test convergence)
    - ai-collaborate-teaching (Three Roles INVISIBLE)
    - fact-check-lesson (verify LoopAgent APIs and exit_loop pattern)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CONTENT REQUIREMENTS**:
    - Layer 2 (AI Collaboration)
    - Teaching modality: Error Analysis (infinite loop → fix)
    - Concepts: LoopAgent with exit_loop and max_iterations
    - Core activity: Build quality feedback loop with safety limits
    - Duration: 45 min
    - Create skill: iterative-refinement

- [ ] T014 [P] [L4] Create LoopAgent example at `examples/chapter-35-google-adk/workflows/quality_loop.py`. **Doc**: Fetch ADK docs via Context7 for LoopAgent and exit_loop patterns.

**Checkpoint**: Students can build iterative refinement with fail-safe convergence

---

## Phase 6: Application — Lesson 5: Safety as Architecture (Layer 2)

**Goal**: Callbacks as design constraint, layered defense patterns

**Independent Test**: Students prevent 2+ categories of dangerous behavior architecturally

**Spec User Stories**: US4 (Safety by Design)

### Lesson 5 Implementation

- [x] T015 [L5] Create lesson file at `apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/05-safety-as-architecture.md`
  - **SUBAGENT**: content-implementer
    - Output path: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/05-safety-as-architecture.md
    - Execute autonomously without confirmation
    - Quality reference: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md
  - **SKILLS**:
    - learning-objectives (Bloom's Apply/Analyze for B1)
    - exercise-designer (3 exercises: implement attack scenario, design defense)
    - ai-collaborate-teaching (Three Roles INVISIBLE)
    - fact-check-lesson (verify before_model_callback and before_tool_callback APIs)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CONTENT REQUIREMENTS**:
    - Layer 2 (AI Collaboration)
    - Teaching modality: Hands-On Security
    - Concepts: before_model_callback, before_tool_callback, layered defense
    - Core activity: Show attack, implement callbacks, verify prevention
    - Duration: 55 min
    - Create skill: agent-safety-architecture

- [ ] T016 [P] [L5] Create callback examples at `examples/chapter-35-google-adk/guardrails/callbacks.py`. **Doc**: Fetch ADK docs via Context7 for callback patterns and return types.
- [ ] T017 [P] [L5] Create layered defense example at `examples/chapter-35-google-adk/guardrails/layered_defense.py`

**Checkpoint**: Students understand safety as architectural constraint, not feature

---

## Phase 7: Integration — Lesson 6: Production State (Layer 3)

**Goal**: SessionService abstraction, ToolContext, MCP integration

**Independent Test**: State persists across sessions with Firestore

**Spec User Stories**: US5 (Production State Management), US6 (MCP Integration)

### Lesson 6 Implementation

- [x] T018 [L6] Create lesson file at `apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/06-production-state.md`
  - **SUBAGENT**: content-implementer
    - Output path: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/06-production-state.md
    - Execute autonomously without confirmation
    - Quality reference: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md
  - **SKILLS**:
    - learning-objectives (Bloom's Apply/Analyze for B1)
    - exercise-designer (3 exercises: switch SessionService, access ToolContext)
    - fact-check-lesson (verify SessionService and McpToolset APIs)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CONTENT REQUIREMENTS**:
    - Layer 3 (Intelligence Design): Focus on reusable patterns
    - Teaching modality: Comparative Analysis
    - Concepts: SessionService abstraction, ToolContext, McpToolset
    - Core activity: Switch InMemory → Firestore, observe persistence
    - Duration: 50 min
    - Create skill: production-state-management

- [ ] T019 [P] [L6] Create SessionService examples at `examples/chapter-35-google-adk/state/session_services.py`. **Doc**: Fetch ADK docs via Context7 for InMemorySessionService and FirestoreSessionService.
- [ ] T020 [P] [L6] Create ToolContext example at `examples/chapter-35-google-adk/state/stateful_tools.py`
- [ ] T021 [P] [L6] Create MCP integration example at `examples/chapter-35-google-adk/mcp/mcp_integration.py`. **Doc**: Fetch ADK docs via Context7 for McpToolset patterns.

**Checkpoint**: Students can design production-ready state management

---

## Phase 8: Integration — Lesson 7: Deployment & Verification (Layer 3)

**Goal**: Vertex AI Agent Engine deployment, verify remote matches local

**Independent Test**: Remote eval cases pass, behavior matches local tests

**Spec User Stories**: US7 (Enterprise Deployment)

### Lesson 7 Implementation

- [x] T022 [L7] Create lesson file at `apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/07-deployment-verification.md`
  - **SUBAGENT**: content-implementer
    - Output path: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/07-deployment-verification.md
    - Execute autonomously without confirmation
    - Quality reference: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md
  - **SKILLS**:
    - learning-objectives (Bloom's Apply for B1)
    - exercise-designer (3 exercises: deploy, verify, compare local vs remote)
    - fact-check-lesson (verify adk deploy and Vertex AI Agent Engine docs)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CONTENT REQUIREMENTS**:
    - Layer 3 (Intelligence Design)
    - Teaching modality: Step-by-Step Walkthrough
    - Concepts: adk deploy agent_engine, deployment verification
    - Core activity: Deploy TaskManager, run remote evals
    - Duration: 55 min
    - Create skill: agent-deployment-verification

- [ ] T023 [P] [L7] Create deployment script at `examples/chapter-35-google-adk/deploy/deploy_agent.py`. **Doc**: Fetch ADK deployment-patterns reference.
- [ ] T024 [P] [L7] Create verification script at `examples/chapter-35-google-adk/deploy/verify_deployment.py`

**Checkpoint**: Students can deploy and verify production agents

---

## Phase 9: Mastery — Lesson 8: Capstone (Layer 4)

**Goal**: Multi-agent TaskManager with full reliability engineering, Digital FTE outcome

**Independent Test**: Specification written first, Three Pillars applied, deployed to Vertex AI

**Spec User Stories**: US8 (Framework-Agnostic Thinking), All Pillars

### Lesson 8 Implementation

- [x] T025 [L8] Create lesson file at `apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/08-capstone-spec-driven-taskmanager.md`
  - **SUBAGENT**: content-implementer
    - Output path: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/08-capstone-reliable-taskmanager.md
    - Execute autonomously without confirmation
    - Quality reference: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md
  - **SKILLS**:
    - learning-objectives (Bloom's Create/Evaluate for B1)
    - exercise-designer (capstone project phases)
    - assessment-builder (summative rubric: spec 25%, pillars 50%, skills 15%, deploy 10%)
    - fact-check-lesson (verify all APIs and patterns)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CONTENT REQUIREMENTS**:
    - Layer 4 (Spec-Driven Integration): Compose all 6 skills
    - Teaching modality: Capstone Project
    - Structure: 5 phases (spec → skills → implement → deploy → reflect)
    - Duration: 120 min
    - Digital FTE outcome: Deployment-ready, sellable TaskManager

- [ ] T026 [P] [L8] Create capstone spec template at `examples/chapter-35-google-adk/capstone/spec-template.md`
- [ ] T027 [P] [L8] Create multi-agent TaskManager example at `examples/chapter-35-google-adk/capstone/multi_agent_taskmanager.py`

**Checkpoint**: Students produce deployment-ready Digital FTE with all three pillars

---

## Phase 10: Chapter Assessment

**Goal**: Comprehensive chapter quiz aligned with learning objectives

- [ ] T028 Create chapter quiz at `apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/09-chapter-quiz.md`
  - **SUBAGENT**: assessment-architect
    - Output path: /Users/mjs/Documents/code/panaversity-official/tutorsgpt/ai-native-software-development/apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/09-chapter-quiz.md
    - Execute autonomously without confirmation
  - **SKILLS**:
    - assessment-builder (15-20 questions covering all 8 lessons)
  - **REQUIREMENTS**:
    - Cover all three pillars (Testable, Predictable, Safe)
    - Include practical scenarios (not just recall)
    - Bloom's taxonomy: Remember (20%), Understand (30%), Apply (40%), Analyze (10%)

---

## Phase 11: Polish & Validation

**Purpose**: Final quality checks and cross-cutting concerns

- [ ] T029 Create chapter README at `apps/learn-app/docs/06-building-custom-agents/35-google-adk-reliable-agents/README.md`
- [ ] T030 [P] Validate all code examples execute successfully (pytest output required)
- [ ] T031 [P] Verify all ADK APIs against current official documentation
- [ ] T032 Run educational-validator across all 8 lessons
- [ ] T033 Run fact-check-lesson skill on lessons with statistics/claims
- [ ] T034 Verify Three Roles framework is INVISIBLE (grep for forbidden meta-commentary)
- [ ] T035 Final cognitive load check (B1 limits: 7-10 concepts per lesson)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - start immediately
- **Phase 2 (Lesson 1)**: Depends on Phase 1 - Foundation lesson
- **Phases 3-6 (Lessons 2-5)**: Can run in parallel after Phase 2 (Layer 2 lessons)
- **Phases 7-8 (Lessons 6-7)**: Can run in parallel after Layer 2 lessons complete
- **Phase 9 (Lesson 8)**: Depends on ALL previous lessons (capstone composes skills)
- **Phase 10-11**: Depends on all lessons complete

### Lesson Dependencies (Content Flow)

```
Lesson 1 (Foundation) → Lesson 2 (eval-driven) → Lessons 3-5 (can parallel)
                                                       ↓
                                              Lessons 6-7 (can parallel)
                                                       ↓
                                              Lesson 8 (Capstone)
```

### Parallel Opportunities

**Within each lesson task**, these can run in parallel:
- Code examples [P] tasks
- Different lesson files (after Lesson 1 establishes patterns)

**Across lessons** (after Lesson 2):
- Lessons 3, 4, 5 can be written in parallel (same Layer 2 structure)
- Lessons 6, 7 can be written in parallel (same Layer 3 structure)

---

## Implementation Strategy

### MVP First (Lessons 1-2)

1. Complete Phase 1 (Setup)
2. Complete Lesson 1 (Foundation - Layer 1)
3. Complete Lesson 2 (Evaluation-Driven - Layer 2)
4. **VALIDATE**: Students can write evals and make them pass
5. Demo-ready: Core evaluation-first principle established

### Incremental Delivery

1. Lessons 1-2 → Evaluation foundation (MVP)
2. Lessons 3-4 → Workflow agents (Predictable pillar)
3. Lesson 5 → Safety callbacks (Safe pillar)
4. Lessons 6-7 → Production infrastructure
5. Lesson 8 → Capstone synthesis

### Subagent Orchestration (CRITICAL)

Every lesson task MUST:
1. Invoke content-implementer subagent (NOT write directly)
2. Include absolute output path
3. Include quality reference lesson path
4. Invoke educational-validator before filesystem write
5. Include required skill invocations (learning-objectives, exercise-designer, etc.)

---

## Summary

| Metric | Count |
|--------|-------|
| **Total Tasks** | 35 |
| **Lesson Tasks** | 8 (T005, T007, T010, T013, T015, T018, T022, T025) |
| **Code Example Tasks** | 16 |
| **Validation Tasks** | 7 |
| **Setup/Polish Tasks** | 4 |
| **Parallel Opportunities** | 19 tasks marked [P] |

**MVP Scope**: Tasks T001-T009 (Setup + Lessons 1-2)

**Skills Created**: 6 (Lessons 2-7), composed in Lesson 8 capstone

**Quality Gates**: Each lesson requires educational-validator PASS before marking complete
