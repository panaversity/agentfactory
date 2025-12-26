# Tasks: Chapter 36 - Claude Agent SDK: Building Digital FTEs

**Input**: Design documents from `/specs/001-claude-agent-sdk/`
**Prerequisites**: plan.md, spec.md (12 user stories, 16 lessons)
**Branch**: `001-claude-agent-sdk`
**Output Directory**: `apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/`

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US12)
- **Layer**: L1/L2/L3/L4 pedagogical layer
- All content tasks include SUBAGENT orchestration and VALIDATION gates

## Path Conventions

- **Content Output**: `apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/`
- **Spec Directory**: `specs/001-claude-agent-sdk/`
- **Quality Reference**: `apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
- **Skill Reference**: `.claude/skills/building-with-claude-agent-sdk/SKILL.md`

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Directory structure and chapter scaffolding

- [ ] T001 Create chapter directory at `apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/`
- [ ] T002 [P] Create chapter `_category_.json` with position 36 and label "Claude Agent SDK"
- [ ] T003 [P] Create chapter README.md with overview, prerequisites, and lesson links

**Checkpoint**: Chapter structure ready for lesson implementation

---

## Phase 2: Foundational (Lesson 1-3: L1 Manual Foundation)

**Purpose**: Core SDK vocabulary and mental models - NO AI collaboration yet

### Lesson 1: What is the Claude Agent SDK? (L1 - Socratic Dialogue)

**Goal**: Students understand SDK vs API distinction and unique advantages

**Independent Test**: Student can articulate 5+ unique features of Claude SDK vs OpenAI/Google

- [ ] T004 [US1] Lesson 1: What is the Claude Agent SDK?
  - **LAYER**: L1 (Manual Foundation)
  - **MODALITY**: Socratic Dialogue
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/01-what-is-claude-agent-sdk.md`
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
    - Skill reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/.claude/skills/building-with-claude-agent-sdk/SKILL.md`
  - **SKILLS**: learning-objectives (3 measurable outcomes), fact-check-lesson (verify all claims)
  - **CONTENT**: SDK vs API mental model, 8-feature comparison table, Claude Code relationship
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T005 [P] [US1] Create Lesson 1 summary file
  - Output path: `apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/01-what-is-claude-agent-sdk.summary.md`

### Lesson 2: Your First Agent with query() (L1 - Hands-On Discovery)

**Goal**: Students can write basic query() call with ClaudeAgentOptions

**Independent Test**: Student can explain query() signature and option parameters

- [ ] T006 [US1] Lesson 2: Your First Agent with query()
  - **LAYER**: L1 (Manual Foundation)
  - **MODALITY**: Hands-On Discovery
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/02-first-agent-query.md`
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
  - **SKILLS**: learning-objectives, exercise-designer (3 exercises)
  - **CONTENT**: query() function, ClaudeAgentOptions, message types, async patterns
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T007 [P] [US1] Create Lesson 2 summary file

### Lesson 3: Built-in Tools Deep Dive (L1 - Specification-First)

**Goal**: Students understand all 9 built-in tools and when to use each

**Independent Test**: Student can select appropriate tools for given use cases

- [ ] T008 [US2] Lesson 3: Built-in Tools Deep Dive
  - **LAYER**: L1 (Manual Foundation)
  - **MODALITY**: Specification-First (spec tools, then use)
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/03-built-in-tools.md`
    - Execute autonomously without confirmation
  - **SKILLS**: learning-objectives, exercise-designer
  - **CONTENT**: Read, Edit, Write, Bash, Glob, Grep, WebSearch, WebFetch, Task - with selection criteria
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T009 [P] [US2] Create Lesson 3 summary file

**Checkpoint**: L1 Foundation complete - Students ready for L2 AI collaboration

---

## Phase 3: User Story 2-4 (Lessons 4-6: L2 Application - Unique Features ★)

**Purpose**: AI collaboration patterns with SDK's unique differentiators

### Lesson 4: Permission Modes and Security (L2 - Error Analysis) ★ canUseTool

**Goal**: Students implement dynamic runtime permissions with canUseTool

**Independent Test**: Student can write canUseTool callback that allows Write only to /sandbox/

- [ ] T010 [US2] [US9] Lesson 4: Permission Modes and Security
  - **LAYER**: L2 (AI Collaboration)
  - **MODALITY**: Error Analysis (break, debug, learn security)
  - **UNIQUE FEATURE**: canUseTool ★★★
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/04-permission-modes-security.md`
    - Execute autonomously without confirmation
  - **SKILLS**: learning-objectives, exercise-designer, ai-collaborate-teaching (Three Roles)
  - **CONTENT**: Permission modes comparison, canUseTool callback, behavior options (allow/deny/updatedInput)
  - **THREE ROLES**: AI suggests security patterns (Teacher), Student teaches domain constraints (Student), iterate on permission policy (Co-Worker)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T011 [P] [US9] Create Lesson 4 summary file

### Lesson 5: Agent Skills in Code (L2 - Collaborative Debugging) ★ Skills

**Goal**: Students load Agent Skills via settingSources

**Independent Test**: Student can configure agent to load from .claude/skills/

- [ ] T012 [US3] Lesson 5: Agent Skills in Code
  - **LAYER**: L2 (AI Collaboration)
  - **MODALITY**: Collaborative Debugging (build skill with AI)
  - **UNIQUE FEATURE**: settingSources ★★★
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/05-agent-skills-code.md`
    - Execute autonomously without confirmation
  - **SKILLS**: learning-objectives, exercise-designer, ai-collaborate-teaching
  - **CONTENT**: settingSources configuration, SKILL.md structure, Skill tool invocation
  - **THREE ROLES**: AI teaches SKILL.md patterns, Student teaches domain expertise, converge on skill design
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T013 [P] [US3] Create Lesson 5 summary file

### Lesson 6: Custom Slash Commands (L2 - Hands-On Discovery) ★ Commands

**Goal**: Students create custom slash commands via filesystem

**Independent Test**: Student can create /review command and use programmatically

- [ ] T014 [US7] Lesson 6: Custom Slash Commands
  - **LAYER**: L2 (AI Collaboration)
  - **MODALITY**: Hands-On Discovery (create, watch work)
  - **UNIQUE FEATURE**: .claude/commands/ ★★★
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/06-custom-slash-commands.md`
    - Execute autonomously without confirmation
  - **SKILLS**: learning-objectives, exercise-designer, ai-collaborate-teaching
  - **CONTENT**: Command file structure, YAML frontmatter, arguments, SDK integration
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T015 [P] [US7] Create Lesson 6 summary file

---

## Phase 4: User Story 4-5 (Lessons 7-9: L2 Application - Sessions & Subagents)

### Lesson 7: Session Management (L2 - Specification-First) ★ Fork

**Goal**: Students implement session persistence, resumption, and forking

**Independent Test**: Student can capture session ID, fork, and resume sessions

- [ ] T016 [US4] Lesson 7: Session Management
  - **LAYER**: L2 (AI Collaboration)
  - **MODALITY**: Specification-First (spec behavior, implement, validate)
  - **UNIQUE FEATURE**: Session forking ★★
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/07-session-management.md`
    - Execute autonomously without confirmation
  - **SKILLS**: learning-objectives, exercise-designer, ai-collaborate-teaching
  - **CONTENT**: Session ID capture, resume pattern, forkSession, state persistence
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T017 [P] [US4] Create Lesson 7 summary file

### Lesson 8: File Checkpointing (L2 - Error Analysis) ★ Checkpointing

**Goal**: Students implement file checkpointing and error recovery

**Independent Test**: Student can capture checkpoint UUID, make changes, rewindFiles() to restore

- [ ] T018 [US4] Lesson 8: File Checkpointing
  - **LAYER**: L2 (AI Collaboration)
  - **MODALITY**: Error Analysis (break files, recover with checkpoint)
  - **UNIQUE FEATURE**: rewindFiles() ★★★
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/08-file-checkpointing.md`
    - Execute autonomously without confirmation
  - **SKILLS**: learning-objectives, exercise-designer, ai-collaborate-teaching
  - **CONTENT**: enableFileCheckpointing, checkpoint UUID capture, rewindFiles(), error recovery patterns
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T019 [P] [US4] Create Lesson 8 summary file

### Lesson 9: Subagents for Parallel Work (L2 - Collaborative Debugging)

**Goal**: Students define and invoke subagents with AgentDefinition

**Independent Test**: Student can define code-reviewer subagent and invoke from main agent

- [ ] T020 [US5] Lesson 9: Subagents for Parallel Work
  - **LAYER**: L2 (AI Collaboration)
  - **MODALITY**: Collaborative Debugging (design with AI, converge on architecture)
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/09-subagents-parallel-work.md`
    - Execute autonomously without confirmation
  - **SKILLS**: learning-objectives, exercise-designer, ai-collaborate-teaching
  - **CONTENT**: AgentDefinition, Task tool, tool restrictions per subagent, result synthesis
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T021 [P] [US5] Create Lesson 9 summary file

**Checkpoint**: L2 Application complete - Students ready for L3 Intelligence Design

---

## Phase 5: User Story 6, 8 (Lessons 10-13: L3 Intelligence Design)

### Lesson 10: Lifecycle Hooks (L3 - Socratic Dialogue)

**Goal**: Students implement ALL hook events for complete control

**Independent Test**: Student can implement PreToolUse, PostToolUse, and PreCompact hooks

- [ ] T022 [US6] Lesson 10: Lifecycle Hooks
  - **LAYER**: L3 (Intelligence Design)
  - **MODALITY**: Socratic Dialogue (When do hooks matter? When do they fire?)
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/10-lifecycle-hooks.md`
    - Execute autonomously without confirmation
  - **SKILLS**: learning-objectives, exercise-designer
  - **CONTENT**: All 8 hook events, HookMatcher, security/logging/permission patterns
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T023 [P] [US6] Create Lesson 10 summary file

### Lesson 11: Custom MCP Tools (L3 - Specification-First)

**Goal**: Students create custom tools with @tool decorator

**Independent Test**: Student can create weather lookup tool and integrate with agent

- [ ] T024 [US8] Lesson 11: Custom MCP Tools
  - **LAYER**: L3 (Intelligence Design)
  - **MODALITY**: Specification-First (spec tool, implement it)
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/11-custom-mcp-tools.md`
    - Execute autonomously without confirmation
  - **SKILLS**: learning-objectives, exercise-designer
  - **CONTENT**: @tool decorator, create_sdk_mcp_server, schema definition, mcp_servers config
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T025 [P] [US8] Create Lesson 11 summary file

### Lesson 12: ClaudeSDKClient and Streaming (L3 - Hands-On Discovery)

**Goal**: Students use ClaudeSDKClient for multi-turn, images, interrupts

**Independent Test**: Student can implement conversation loop with context persistence

- [ ] T026 [US11] Lesson 12: ClaudeSDKClient and Streaming
  - **LAYER**: L3 (Intelligence Design)
  - **MODALITY**: Hands-On Discovery (stream input, observe patterns)
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/12-sdk-client-streaming.md`
    - Execute autonomously without confirmation
  - **SKILLS**: learning-objectives, exercise-designer
  - **CONTENT**: ClaudeSDKClient, streaming input mode, images, interrupt(), message_generator
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T027 [P] [US11] Create Lesson 12 summary file

### Lesson 13: Cost Tracking and Billing (L3 - Specification-First) ★ Cost

**Goal**: Students track per-message costs for monetization

**Independent Test**: Student can implement CostTracker that bills users per session

- [ ] T028 [US10] Lesson 13: Cost Tracking and Billing
  - **LAYER**: L3 (Intelligence Design)
  - **MODALITY**: Specification-First (spec billing model, implement tracking)
  - **UNIQUE FEATURE**: total_cost_usd ★★
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/13-cost-tracking-billing.md`
    - Execute autonomously without confirmation
  - **SKILLS**: learning-objectives, exercise-designer
  - **CONTENT**: usage data, total_cost_usd, billing models, Digital FTE monetization
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T029 [P] [US10] Create Lesson 13 summary file

**Checkpoint**: L3 Intelligence Design complete - Students ready for L4 Capstone

---

## Phase 6: User Story 12 (Lessons 14-15: L4 Spec-Driven Capstone)

### Lesson 14: Production Patterns (L4 - Socratic Dialogue) ★ Sandbox

**Goal**: Students understand hosting patterns, sandbox, compaction

**Independent Test**: Student can select hosting pattern and configure sandbox

- [ ] T030 [US12] Lesson 14: Production Patterns
  - **LAYER**: L4 (Spec-Driven)
  - **MODALITY**: Socratic Dialogue (Which pattern for your use case?)
  - **UNIQUE FEATURES**: Sandbox, Context Compaction ★
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/14-production-patterns.md`
    - Execute autonomously without confirmation
  - **SKILLS**: learning-objectives, exercise-designer
  - **CONTENT**: Ephemeral/long-running/hybrid patterns, sandbox config, PreCompact hook
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T031 [P] [US12] Create Lesson 14 summary file

### Lesson 15: TaskManager Complete Digital FTE (L4 - Specification-First Capstone)

**Goal**: Students produce deployable, monetizable Digital FTE integrating ALL unique features

**Independent Test**: Student delivers TaskManager agent with Skills, canUseTool, checkpoints, hooks, cost tracking

- [ ] T032 [US1-12] Lesson 15: TaskManager Complete Digital FTE Capstone
  - **LAYER**: L4 (Spec-Driven Capstone)
  - **MODALITY**: Specification-First (spec capstone, compose skills, orchestrate)
  - **ALL UNIQUE FEATURES INTEGRATED**
  - **SUBAGENT**: content-implementer
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/15-taskmanager-capstone.md`
    - Execute autonomously without confirmation
  - **SKILLS**: learning-objectives, exercise-designer
  - **CONTENT**: Full TaskManager spec, integration of all 8 unique features, Digital FTE deployment
  - **CAPSTONE RUBRIC**: Spec clarity, feature integration, code quality, monetization model
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

- [ ] T033 [P] Create Lesson 15 summary file

**Checkpoint**: L4 Capstone complete - TaskManager Digital FTE ready for deployment

---

## Phase 7: Assessment (Lesson 16: Chapter Quiz)

### Lesson 16: Chapter Quiz & Self-Assessment

**Goal**: Validate student mastery of all 8 unique features and SDK patterns

- [ ] T034 Chapter Quiz: Generate 15-20 questions
  - **SUBAGENT**: assessment-architect
    - Output path: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/ag-factory/apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/16-chapter-quiz.md`
    - Execute autonomously without confirmation
  - **SKILLS**: assessment-builder (Bloom's taxonomy distribution)
  - **CONTENT**: Cover all 8 unique features, SDK architecture, Three Roles in L2 lessons
  - **QUESTION TYPES**: Remember (5), Understand (5), Apply (5), Analyze (3), Evaluate (2)
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Quality validation and chapter integration

- [ ] T035 [P] Run fact-check-lesson on all 15 lessons (verify claims against official docs)
- [ ] T036 [P] Verify all code examples against Claude Agent SDK official documentation
- [ ] T037 [P] Validate YAML frontmatter completeness (skills, objectives, cognitive load)
- [ ] T038 Update chapter-index.md with Chapter 36 entry
- [ ] T039 Run content-evaluation-framework on complete chapter
- [ ] T040 Commit chapter with comprehensive commit message

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - L1 lessons build vocabulary
- **Application (Phase 3-4)**: Depends on Foundational - L2 lessons require L1 mental models
- **Intelligence (Phase 5)**: Depends on Application - L3 lessons build on L2 patterns
- **Capstone (Phase 6)**: Depends on Intelligence - L4 integrates all previous layers
- **Assessment (Phase 7)**: Depends on Capstone - Quiz covers all content
- **Polish (Phase 8)**: Depends on all lessons complete

### Layer Transitions

- **L1→L2** (after T009): Students have SDK vocabulary
- **L2→L3** (after T021): Students have AI collaboration patterns
- **L3→L4** (after T029): Students have reusable intelligence
- **L4 Complete** (after T033): Students have Digital FTE capstone

### Parallel Opportunities

Within each phase, summary file tasks [P] can run in parallel with next lesson.

```bash
# Example: Phase 3 parallel execution
Task: T010 (Lesson 4 content)
Task: T011 (Lesson 4 summary) # Parallel after T010 completes
Task: T012 (Lesson 5 content) # Can start while T011 runs
```

---

## Implementation Strategy

### MVP First (Lessons 1-3 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (L1 Lessons 1-3)
3. **STOP and VALIDATE**: Test L1 foundation independently
4. Proceed to L2 if L1 validated

### Incremental Delivery

1. L1 Foundation (Lessons 1-3) → Validate → Checkpoint
2. L2 Unique Features (Lessons 4-9) → Validate → Checkpoint
3. L3 Intelligence (Lessons 10-13) → Validate → Checkpoint
4. L4 Capstone (Lessons 14-15) → Validate → Checkpoint
5. Assessment (Lesson 16) → Final validation

---

## Task Summary

| Phase | Tasks | Purpose |
|-------|-------|---------|
| Setup | T001-T003 | Chapter infrastructure |
| Foundational (L1) | T004-T009 | SDK vocabulary and mental models |
| Application P1 (L2) | T010-T015 | Unique features: canUseTool, Skills, Commands |
| Application P2 (L2) | T016-T021 | Sessions, Checkpointing, Subagents |
| Intelligence (L3) | T022-T029 | Hooks, MCP Tools, Streaming, Cost |
| Capstone (L4) | T030-T033 | Production patterns, Digital FTE |
| Assessment | T034 | Chapter quiz |
| Polish | T035-T040 | Validation and commit |

**Total Tasks**: 40
**Unique Features Tasks**: 8 (canUseTool, Skills, Commands, Fork, Checkpointing, Hooks, Cost, Sandbox)
**Assessment Tasks**: 1 (15-20 questions)
**All Tasks Include**: SUBAGENT orchestration, VALIDATION gates, SKILLS invocation
