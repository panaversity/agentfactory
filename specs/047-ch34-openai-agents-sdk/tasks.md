# Tasks: Chapter 34 — OpenAI Agents SDK (Production Mastery)

**Input**: Design documents from `/specs/047-ch34-openai-agents-sdk/`
**Prerequisites**: plan.md, spec.md
**Content Type**: Educational (8 lessons)
**Base Output Path**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-Building-and-Deploying-Agents/34-openai-agents-sdk/`

## Format: `[ID] [P?] [Lesson] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Lesson]**: Maps to lesson number (L1-L8)
- Include exact file paths in descriptions

## Critical Subagent Requirements

**Every lesson task MUST include**:
1. **SUBAGENT**: content-implementer with absolute output path
2. **VALIDATION**: educational-validator (MUST PASS before marking complete)
3. **SKILLS**: learning-objectives, exercise-designer, fact-check-lesson
4. **For L2+ lessons**: ai-collaborate-teaching skill for Three Roles

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Chapter folder structure and shared resources

- [ ] T001 Create chapter directory structure at `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-Building-and-Deploying-Agents/34-openai-agents-sdk/`
- [ ] T002 Create chapter README.md with overview, learning path, and prerequisites (links to Chapter 33)
- [ ] T003 [P] Create `_category_.json` with chapter metadata (position: 34, label: "OpenAI Agents SDK")
- [ ] T004 [P] Verify `building-with-openai-agents` skill exists at `.claude/skills/building-with-openai-agents/SKILL.md`

**Checkpoint**: Chapter structure ready for lesson implementation

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Ensure teaching resources and quality reference are accessible

**CRITICAL**: No lesson implementation can begin until this phase is complete

- [ ] T005 Read quality reference lesson at `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
- [ ] T006 [P] Verify GitHub examples cloned to `/tmp/openai-agents-python/` with key patterns:
  - `examples/customer_service/main.py`
  - `examples/agent_patterns/agents_as_tools.py`
  - `examples/handoffs/message_filter.py`
  - `examples/memory/advanced_sqlite_session_example.py`
  - `examples/basic/lifecycle_example.py`
- [ ] T007 [P] Verify OpenAI API key available for code execution validation: `export OPENAI_API_KEY`
- [ ] T008 Confirm TaskManager agent from Chapter 33 available as running example base

**Checkpoint**: Foundation ready - lesson implementation can now begin

---

## Phase 3: Lesson 1 — SDK Setup & First Agent (Layer 1: Manual Foundation)

**Goal**: Students set up SDK and run a functional agent without AI assistance

**Independent Test**: Student can run `python hello_agent.py` and receive response

**Maps to Evals**: Foundation for all subsequent lessons

### Implementation for Lesson 1

- [ ] T009 [L1] Lesson 1: SDK Setup & First Agent
  - **OUTPUT**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-Building-and-Deploying-Agents/34-openai-agents-sdk/01-sdk-setup-first-agent.md`
  - **SUBAGENT**: content-implementer
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
    - Skill reference: `.claude/skills/building-with-openai-agents/SKILL.md`
  - **SKILLS**:
    - `learning-objectives`: Generate measurable outcomes for SDK installation, Agent class, Runner.run_sync()
    - `exercise-designer`: 3 exercises (hello world, custom instructions, LiteLLM alternative)
    - `fact-check-lesson`: Verify all SDK imports and API patterns against official docs
  - **CONTENT REQUIREMENTS**:
    - Full YAML frontmatter with skills metadata, cognitive load (6 concepts)
    - LiteLLM alternative with `LitellmModel("anthropic/claude-3-5-sonnet")`
    - No AI collaboration yet (Layer 1 = manual foundation)
    - 3 "Try With AI" prompts for conceptual exploration only
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CODE EXECUTION**: All examples tested against live API with logs

**Checkpoint**: Lesson 1 complete — students can create and run basic agents

---

## Phase 4: Lesson 2 — Function Tools & Context Objects (Layer 1: Manual Foundation)

**Goal**: Students implement function tools and context objects manually

**Independent Test**: Context persists across multiple tool calls

**Maps to Evals**: SC-001 (context objects persist across tools and agents)

### Implementation for Lesson 2

- [ ] T010 [L2] Lesson 2: Function Tools & Context Objects
  - **OUTPUT**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-Building-and-Deploying-Agents/34-openai-agents-sdk/02-function-tools-context.md`
  - **SUBAGENT**: content-implementer
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
    - GitHub pattern: `/tmp/openai-agents-python/examples/customer_service/main.py` (AirlineAgentContext)
  - **SKILLS**:
    - `learning-objectives`: Generate outcomes for @function_tool, Pydantic context, state mutations
    - `exercise-designer`: 3 exercises (basic tool, context model, TaskManager context)
    - `fact-check-lesson`: Verify RunContextWrapper patterns against SDK docs
  - **CONTENT REQUIREMENTS**:
    - Full YAML frontmatter with skills metadata, cognitive load (7 concepts)
    - TaskManagerContext Pydantic model as running example
    - No AI collaboration yet (Layer 1 = manual foundation)
    - Code from `examples/basic/tools.py` as reference
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CODE EXECUTION**: All examples tested against live API with logs

**Checkpoint**: Lesson 2 complete — students understand context persistence pattern

---

## Phase 5: Lesson 3 — Agents as Tools & Orchestration (Layer 2: AI Collaboration)

**Goal**: Students build orchestrator with sub-agents using AI collaboration

**Independent Test**: Manager agent coordinates 3+ specialist agents

**Maps to Evals**: SC-002 (orchestrator with 3+ sub-agents as tools)

### Implementation for Lesson 3

- [ ] T011 [L3] Lesson 3: Agents as Tools & Orchestration
  - **OUTPUT**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-Building-and-Deploying-Agents/34-openai-agents-sdk/03-agents-as-tools.md`
  - **SUBAGENT**: content-implementer
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
    - GitHub patterns:
      - `/tmp/openai-agents-python/examples/agent_patterns/agents_as_tools.py`
      - `/tmp/openai-agents-python/examples/financial_research_agent/manager.py`
  - **SKILLS**:
    - `learning-objectives`: Generate outcomes for agent.as_tool(), custom_output_extractor, clone()
    - `exercise-designer`: 3 exercises (basic agent tool, custom extractor, TaskManager orchestrator)
    - `ai-collaborate-teaching`: Design Three Roles section:
      - AI as Teacher: Suggests agent.as_tool() pattern
      - AI as Student: Adapts to information needs
      - Co-Worker: Converges on dynamic composition
    - `fact-check-lesson`: Verify patterns against SDK docs
  - **CONTENT REQUIREMENTS**:
    - Full YAML frontmatter with skills metadata, cognitive load (5 concepts)
    - Three Roles demonstrated WITHOUT meta-commentary
    - TaskManager as planning/execution/validation orchestrator
    - Framework invisibility: No "AI as Teacher" labels in content
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CODE EXECUTION**: All examples tested against live API with logs

**Checkpoint**: Lesson 3 complete — students can compose multi-agent systems

---

## Phase 6: Lesson 4 — Handoffs & Message Filtering (Layer 2: AI Collaboration)

**Goal**: Students implement handoffs with callbacks and message filtering

**Independent Test**: Handoff injects runtime context before agent transfer

**Maps to Evals**: SC-003 (handoff with callback that injects runtime data)

### Implementation for Lesson 4

- [ ] T012 [L4] Lesson 4: Handoffs & Message Filtering
  - **OUTPUT**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-Building-and-Deploying-Agents/34-openai-agents-sdk/04-handoffs-filtering.md`
  - **SUBAGENT**: content-implementer
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
    - GitHub patterns:
      - `/tmp/openai-agents-python/examples/handoffs/message_filter.py`
      - `/tmp/openai-agents-python/examples/customer_service/main.py`
  - **SKILLS**:
    - `learning-objectives`: Generate outcomes for handoff(), on_handoff, HandoffInputData
    - `exercise-designer`: 3 exercises (basic handoff, callback injection, bidirectional)
    - `ai-collaborate-teaching`: Design Three Roles section:
      - AI as Teacher: Teaches on_handoff callbacks
      - AI as Student: Learns filtering semantics
      - Co-Worker: Discovers bidirectional handoffs
    - `fact-check-lesson`: Verify handoff_filters utilities against SDK
  - **CONTENT REQUIREMENTS**:
    - Full YAML frontmatter with skills metadata, cognitive load (6 concepts)
    - Three Roles demonstrated WITHOUT meta-commentary
    - TaskManager handoffs between planning/execution agents
    - Spanish handoff example from message_filter.py
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CODE EXECUTION**: All examples tested against live API with logs

**Checkpoint**: Lesson 4 complete — students understand handoff patterns

---

## Phase 7: Lesson 5 — Guardrails & Agent-Based Validation (Layer 2: AI Collaboration)

**Goal**: Students implement agent-based guardrails with structured output

**Independent Test**: Guardrail agent detects and blocks invalid input with reasoning

**Maps to Evals**: SC-004 (agent-based guardrail with structured output)

### Implementation for Lesson 5

- [ ] T013 [L5] Lesson 5: Guardrails & Agent-Based Validation
  - **OUTPUT**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-Building-and-Deploying-Agents/34-openai-agents-sdk/05-guardrails-validation.md`
  - **SUBAGENT**: content-implementer
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
    - GitHub patterns:
      - `/tmp/openai-agents-python/examples/agent_patterns/input_guardrails.py`
      - `/tmp/openai-agents-python/examples/agent_patterns/output_guardrails.py`
  - **SKILLS**:
    - `learning-objectives`: Generate outcomes for @input_guardrail, @output_guardrail, agent-based validation
    - `exercise-designer`: 3 exercises (simple guard, agent guard, PII detector)
    - `ai-collaborate-teaching`: Design Three Roles section:
      - AI as Teacher: Teaches agent-based validation
      - AI as Student: Learns semantic distinctions
      - Co-Worker: Discovers user-friendly messages
    - `fact-check-lesson`: Verify GuardrailFunctionOutput against SDK
  - **CONTENT REQUIREMENTS**:
    - Full YAML frontmatter with skills metadata, cognitive load (6 concepts)
    - Three Roles demonstrated WITHOUT meta-commentary
    - TaskManager input validation guardrail
    - Exception handling: InputGuardrailTripwireTriggered
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CODE EXECUTION**: All examples tested against live API with logs

**Checkpoint**: Lesson 5 complete — students can implement production guardrails

---

## Phase 8: Lesson 6 — Sessions & Conversation Memory (Layer 2: AI Collaboration)

**Goal**: Students persist conversations with AdvancedSQLiteSession and branching

**Independent Test**: Conversation branches correctly, history retrieved per branch

**Maps to Evals**: SC-005 (AdvancedSQLiteSession with branching)

### Implementation for Lesson 6

- [ ] T014 [L6] Lesson 6: Sessions & Conversation Memory
  - **OUTPUT**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-Building-and-Deploying-Agents/34-openai-agents-sdk/06-sessions-memory.md`
  - **SUBAGENT**: content-implementer
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
    - GitHub patterns:
      - `/tmp/openai-agents-python/examples/memory/sqlite_session_example.py`
      - `/tmp/openai-agents-python/examples/memory/advanced_sqlite_session_example.py`
  - **SKILLS**:
    - `learning-objectives`: Generate outcomes for SQLiteSession, AdvancedSQLiteSession, branching
    - `exercise-designer`: 3 exercises (basic session, usage tracking, branching)
    - `ai-collaborate-teaching`: Design Three Roles section:
      - AI as Teacher: Teaches AdvancedSQLiteSession
      - AI as Student: Learns reliability requirements
      - Co-Worker: Discovers efficient branching
    - `fact-check-lesson`: Verify store_run_usage(), create_branch_from_turn() against SDK
  - **CONTENT REQUIREMENTS**:
    - Full YAML frontmatter with skills metadata, cognitive load (6 concepts)
    - Three Roles demonstrated WITHOUT meta-commentary
    - TaskManager with session persistence per user
    - Branch switching demonstration
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CODE EXECUTION**: All examples tested against live API with logs

**Checkpoint**: Lesson 6 complete — students can persist and branch conversations

---

## Phase 9: Lesson 7 — Tracing, Hooks & Observability (Layer 2+3: AI Collaboration + Intelligence Design)

**Goal**: Students implement lifecycle hooks and multi-agent tracing, create reusable skill

**Independent Test**: Complete trace visible in OpenAI dashboard with custom spans

**Maps to Evals**: SC-006 (RunHooks with usage tracking), SC-007 (multi-agent trace)

### Implementation for Lesson 7

- [ ] T015 [L7] Lesson 7: Tracing, Hooks & Observability
  - **OUTPUT**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-Building-and-Deploying-Agents/34-openai-agents-sdk/07-tracing-observability.md`
  - **SUBAGENT**: content-implementer
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
    - GitHub patterns:
      - `/tmp/openai-agents-python/examples/basic/lifecycle_example.py`
      - `/tmp/openai-agents-python/examples/agent_patterns/deterministic.py`
  - **SKILLS**:
    - `learning-objectives`: Generate outcomes for RunHooks, trace(), custom_span(), group_id
    - `exercise-designer`: 3 exercises (lifecycle logging, custom spans, trace correlation)
    - `ai-collaborate-teaching`: Design Three Roles section:
      - AI as Teacher: Teaches observability patterns
      - AI as Student: Learns noise reduction
      - Co-Worker: Discovers trace correlation
    - `fact-check-lesson`: Verify gen_trace_id(), context.usage against SDK
  - **CONTENT REQUIREMENTS**:
    - Full YAML frontmatter with skills metadata, cognitive load (7 concepts)
    - Three Roles demonstrated WITHOUT meta-commentary
    - Layer 3 Intelligence Design: Create production-observability-stack skill
    - TaskManager with full observability infrastructure
    - OpenAI Platform trace URL generation
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CODE EXECUTION**: All examples tested against live API with logs and trace URLs

**Checkpoint**: Lesson 7 complete — students can observe production agent systems

---

## Phase 10: Lesson 8 — Capstone: Customer Support FTE (Layer 4: Spec-Driven Integration)

**Goal**: Students implement production-quality Digital FTE using specification-first approach

**Independent Test**: Complete customer support system with all patterns integrated

**Maps to Evals**: SC-008 (production-quality capstone matches customer_service example)

### Implementation for Lesson 8

- [ ] T016 [L8] Lesson 8: Capstone — Customer Support FTE
  - **OUTPUT**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-Building-and-Deploying-Agents/34-openai-agents-sdk/08-capstone-customer-support.md`
  - **SUBAGENT**: content-implementer
    - Execute autonomously without confirmation
    - Quality reference: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/01-Introducing-AI-Driven-Development/01-agent-factory-paradigm/01-digital-fte-revolution.md`
    - GitHub pattern: `/tmp/openai-agents-python/examples/customer_service/main.py` (FULL implementation)
  - **SKILLS**:
    - `learning-objectives`: Generate outcomes for spec-first design, component composition, validation
    - `exercise-designer`: 1 comprehensive capstone project (Customer Support FTE)
    - `ai-collaborate-teaching`: Design Three Roles for implementation iteration:
      - AI as Teacher: Suggests architectural improvements
      - Student: Validates against spec
      - Co-Worker: Converges on quality
    - `assessment-builder`: Design final chapter assessment
    - `fact-check-lesson`: Verify complete system against customer_service example
  - **CONTENT REQUIREMENTS**:
    - Full YAML frontmatter with skills metadata, cognitive load (synthesis)
    - Layer 4: Spec-first approach (spec.md BEFORE code)
    - Pattern composition from ALL Lessons 1-7
    - Complete AirlineAgentContext implementation
    - All agents: triage, FAQ, booking, escalation
    - All tools: faq_lookup_tool, update_seat, escalate_to_human
    - All guardrails: abuse detection, PII filtering
    - Full tracing with group_id correlation
    - Monetization reflection section
  - **VALIDATION**: educational-validator (MUST PASS before marking complete)
  - **CODE EXECUTION**: Complete system tested against live API with full conversation flows

**Checkpoint**: Lesson 8 complete — students can build production Digital FTEs

---

## Phase 11: Chapter Assessment

**Purpose**: Summative assessment for Chapter 34

- [ ] T017 Create chapter quiz using assessment-architect
  - **OUTPUT**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/p7-c/apps/learn-app/docs/07-Building-and-Deploying-Agents/34-openai-agents-sdk/quiz.md`
  - **SUBAGENT**: assessment-architect
    - Bloom's levels: Remember (SDK primitives), Understand (patterns), Apply (implementation), Analyze (debugging), Create (capstone)
    - CEFR alignment: B1-B2 → C1 progression
    - 10-15 questions covering SC-001 through SC-008
  - **VALIDATION**: Questions map to success criteria

---

## Phase 12: Polish & Cross-Cutting Concerns

**Purpose**: Final validation and integration

- [ ] T018 [P] Update chapter README.md with completed lesson links and prerequisites
- [ ] T019 [P] Verify all code examples execute without errors against live API
- [ ] T020 [P] Grep for meta-commentary violations ("Layer", "Stage", "AI as Teacher" in content)
- [ ] T021 [P] Validate cognitive load ≤ 10 concepts per lesson (B1 limit)
- [ ] T022 [P] Cross-reference all patterns against GitHub examples
- [ ] T023 Run full chapter validation with validation-auditor subagent
- [ ] T024 Commit chapter with all lessons and quiz

**Checkpoint**: Chapter 34 complete and validated

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all lessons
- **Lessons 1-2 (Phases 3-4)**: Layer 1 - must complete before Layer 2
- **Lessons 3-7 (Phases 5-9)**: Layer 2 - can proceed after Lessons 1-2
- **Lesson 8 (Phase 10)**: Layer 4 - requires ALL Lessons 1-7
- **Assessment (Phase 11)**: Requires all lessons
- **Polish (Phase 12)**: Requires lessons + assessment

### Lesson Dependencies

```
L1 (SDK Setup) ──────┐
                     ├──► L3 (Agents as Tools) ──┐
L2 (Function Tools) ─┘                           │
                                                 │
L1 + L2 ──► L4 (Handoffs) ───────────────────────┤
                                                 │
L1 + L2 ──► L5 (Guardrails) ─────────────────────┤
                                                 │
L1 + L2 ──► L6 (Sessions) ───────────────────────┤
                                                 │
L1-L6 ────► L7 (Tracing + Skill Creation) ───────┤
                                                 │
L1-L7 ────────────────────────────────────────► L8 (Capstone)
```

### Parallel Opportunities

**After Phase 2 (Foundational)**:
- Lessons 1-2 can run in parallel (both Layer 1)

**After Lessons 1-2**:
- Lessons 3, 4, 5, 6 can run in parallel (all Layer 2, different patterns)

**Sequential requirements**:
- Lesson 7 requires Lessons 1-6 (needs all patterns for observability)
- Lesson 8 requires ALL lessons (capstone synthesis)

---

## Parallel Example: Layer 2 Lessons

```bash
# After Lessons 1-2 complete, launch Layer 2 lessons in parallel:
Task T011: "Lesson 3: Agents as Tools"
Task T012: "Lesson 4: Handoffs & Message Filtering"
Task T013: "Lesson 5: Guardrails & Agent-Based Validation"
Task T014: "Lesson 6: Sessions & Conversation Memory"

# These can all run simultaneously with different content-implementer instances
```

---

## Implementation Strategy

### MVP First (Lessons 1-3 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Lesson 1 (SDK basics)
4. Complete Lesson 2 (Tools & Context)
5. Complete Lesson 3 (Agents as Tools)
6. **STOP and VALIDATE**: Core patterns teachable, orchestration works
7. Demo/review if ready

### Full Chapter Delivery

1. Setup + Foundational → Chapter structure ready
2. Lessons 1-2 (Layer 1) → Manual foundation established
3. Lessons 3-7 (Layer 2) → AI collaboration patterns + skill creation
4. Lesson 8 (Layer 4) → Capstone synthesis
5. Assessment + Polish → Chapter complete

---

## Summary

| Phase | Tasks | Parallelizable | Estimate |
|-------|-------|----------------|----------|
| Setup | T001-T004 | 2 | 30 min |
| Foundational | T005-T008 | 2 | 20 min |
| Lesson 1 | T009 | 0 | 60 min |
| Lesson 2 | T010 | 0 | 75 min |
| Lesson 3 | T011 | 0 | 100 min |
| Lesson 4 | T012 | 0 | 100 min |
| Lesson 5 | T013 | 0 | 100 min |
| Lesson 6 | T014 | 0 | 100 min |
| Lesson 7 | T015 | 0 | 125 min |
| Lesson 8 | T016 | 0 | 150 min |
| Assessment | T017 | 0 | 45 min |
| Polish | T018-T024 | 5 | 60 min |

**Total Tasks**: 24
**Parallel Opportunities**: 9 tasks can run in parallel at various stages
**Total Estimated Time**: ~15-16 hours (content implementation)

---

## Notes

- All lesson tasks REQUIRE subagent invocation (content-implementer)
- All lesson tasks REQUIRE validation gate (educational-validator)
- Framework invisibility: NO meta-commentary in final content
- Three Roles in L2+ lessons: Demonstrated through action, not labeled
- Quality reference: Chapter 1 Lesson 1 for all content calibration
- Skill reference: `.claude/skills/building-with-openai-agents/` for patterns
- GitHub examples: `/tmp/openai-agents-python/examples/` for code validation
