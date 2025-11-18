# Tasks: Chapter 11 Context Engineering Rewrite

**Input**: Design documents from `/specs/001-011-chapter-11-context-engineering-rewrite/`
**Prerequisites**: spec.md (6 user stories P1-P3), plan.md (9 lessons, 927 lines), ADRs (0013-0015)

**Tests**: NO tests required - Educational content chapter, validation through pedagogical review not automated testing

**Organization**: Tasks are grouped by user story (corresponding to lessons/lesson groups) to enable independent implementation and validation of pedagogical objectives.

## Format: `- [ ] [ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different lessons, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US6 map to lesson groups)
- Include exact file paths in descriptions

## Path Conventions

- Chapter directory: `book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/`
- Lesson files: `01-lesson-1.md`, `02-lesson-2.md`, etc. (9 total)
- README: `README.md` (chapter overview, auto-generated from learning objectives)

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Initialize chapter directory structure and prepare for lesson implementation

- [ ] T001 Create chapter directory at `book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/`
- [ ] T002 Create README.md with chapter overview, learning objectives (LO-001 through LO-006 from spec.md), and lesson navigation
- [ ] T003 [P] Set up directory for lesson files (01-lesson-1.md through 09-lesson-9.md placeholders)
- [ ] T004 [P] Verify YAML frontmatter template for lessons (sidebar_position, title fields)

---

## Phase 2: Foundational (Research Verification & Framework Documentation)

**Purpose**: Verify research citations and document reusable frameworks before lesson authoring

**⚠️ CRITICAL**: These artifacts must be complete and verified before lesson content can reference them

- [ ] T005 Verify Claude Sonnet 4.5 context window specs (200K standard, 1M extended) against Anthropic docs (2025-01-18)
- [ ] T006 [P] Verify Gemini 1.5 Pro context window specs (2M) against Google Cloud docs (2025-01-18)
- [ ] T007 [P] Verify Karpathy "LLM as CPU" principle citation from Y Combinator AI School session
- [ ] T008 [P] Verify Anthropic context degradation research from Engineering Blog
- [ ] T009 Document Six Components AIDD Context framework (Model Selection, Dev Tools, Knowledge & Memory, Guardrails, Code Context, Orchestration) for cross-lesson reference
- [ ] T010 [P] Create comparison table (Claude Code 200K vs Gemini CLI 2M) with verified 2025 specs for Lessons 1 and 7

**Checkpoint**: All research verified, frameworks documented - lesson authoring can begin in parallel

---

## Phase 3: User Story 1 - Context Window Foundation (Lessons 1-2, Priority: P1) 🎯 MVP

**Goal**: Students build intuitive understanding of context windows and degradation symptoms through manual tracking exercises (Stage 1 foundation)

**Independent Test**: Student can observe context window filling in AI session, identify 3+ degradation symptoms WITHOUT AI assistance, explain when compression vs isolation needed

### Implementation for User Story 1

- [ ] T011 [P] [US1] Create Lesson 1 file at `book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/01-context-windows-token-counting.md`
- [ ] T012 [P] [US1] Create Lesson 2 file at `book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/02-degradation-symptoms-manual-tracking.md`
- [ ] T013 [US1] Write Lesson 1 content: Context windows, token mechanics, manual observation exercises (7 concepts, Stage 1 manual-only, experiment→observe→learn discovery sequence per plan.md lines 54-96)
- [ ] T014 [US1] Write Lesson 2 content: Degradation symptoms (5 types), manual tracking methodology, compression/isolation/restart decision framework (8 concepts, Stage 1 manual-only per plan.md lines 98-143)
- [ ] T015 [US1] Create manual token estimation exercise with sample codebase transcript for Lesson 1
- [ ] T016 [US1] Create degradation symptom discovery exercise with 90-minute session transcript for Lesson 2
- [ ] T017 [US1] Integrate Karpathy "LLM as CPU" principle into Lesson 1 opening with proper citation
- [ ] T018 [US1] Integrate Anthropic degradation research into Lesson 2 mitigation framework section
- [ ] T019 [US1] Validate cognitive load: Lesson 1 (7 concepts ≤10 B1 limit), Lesson 2 (8 concepts ≤10 B1 limit)
- [ ] T020 [US1] Validate hands-on discovery modality: Experiments precede explanations in both lessons (Test-005 compliance)

**Checkpoint**: Lessons 1-2 complete, students can manually diagnose context issues without AI

---

## Phase 4: User Story 2 - Progressive Context Loading (Lesson 3, Priority: P1)

**Goal**: Students apply progressive loading strategy (Foundation→Current→On-Demand) with AI collaboration demonstrating complete Three Roles cycle

**Independent Test**: Given 50+ file codebase, student applies progressive loading with AI, maintains context <70%, demonstrates all three roles in session transcript

### Implementation for User Story 2

- [ ] T021 [US2] Create Lesson 3 file at `book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/03-progressive-loading-strategy.md`
- [ ] T022 [US2] Write Lesson 3 content: Progressive loading phases (Foundation/Current/On-Demand), Three Roles framework integration (9 concepts, Stage 2 AI collaboration per plan.md lines 146-216)
- [ ] T023 [US2] Create session transcript demonstrating Three Roles: AI as Teacher (suggests loading pattern), AI as Student (learns project constraints), Co-Worker (3+ iteration rounds) per plan.md lines 159-182
- [ ] T024 [US2] Design hands-on progressive loading exercise with 50-file FastAPI sample project
- [ ] T025 [US2] Document Foundation phase file selection criteria (project structure, core configs, 5-10 files)
- [ ] T026 [US2] Document Current Work phase file selection criteria (task-specific files based on feature scope)
- [ ] T027 [US2] Document On-Demand phase just-in-time fetching pattern (AI requests additional context)
- [ ] T028 [US2] Create "Try With AI" section with complete Three Roles collaboration example
- [ ] T029 [US2] Validate Three Roles demonstration: All three roles explicit with evidence (Test-003 compliance for Lesson 3)
- [ ] T030 [US2] Validate cognitive load: 9 concepts at B1 upper boundary (acceptable per plan.md line 196)

**Checkpoint**: Lesson 3 complete, students can apply progressive loading with AI collaboration

---

## Phase 5: User Story 3 - Context Compression & Isolation (Lessons 4-5, Priority: P2)

**Goal**: Students apply context compression (checkpoint creation, session restart) and isolation techniques (separate sessions for unrelated tasks) with Three Roles demonstrations

**Independent Test**: During long session (90+ minutes, 85%+ context), student decides compression vs isolation with reasoning, demonstrates Three Roles in both techniques

### Implementation for User Story 3

- [ ] T031 [P] [US3] Create Lesson 4 file at `book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/04-context-compression-session-restart.md`
- [ ] T032 [P] [US3] Create Lesson 5 file at `book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/05-context-isolation-parallel-tasks.md`
- [ ] T033 [US3] Write Lesson 4 content: Checkpoint structure (decisions+progress+next steps), compression ratio, session restart pattern, Three Roles in compression workflow (8 concepts, Stage 2 per plan.md lines 219-287)
- [ ] T034 [US3] Write Lesson 5 content: Task similarity scoring, context pollution risk, isolation decision framework, Three Roles in isolation decisions (8 concepts, Stage 2 per plan.md lines 290-357)
- [ ] T035 [US3] Create session transcript for Lesson 4: Three Roles in checkpoint creation (AI teaches structure, student refines with specifics, iterate on token budget 50→1000→500) per plan.md lines 232-254
- [ ] T036 [US3] Create session transcript for Lesson 5: Three Roles in isolation decision (AI teaches similarity scoring, student defines domain boundaries, converge on isolation criteria) per plan.md lines 304-325
- [ ] T037 [US3] Design compression exercise: 2-hour coding session reaching 85% utilization, student creates checkpoint, restarts with summary
- [ ] T038 [US3] Design isolation exercise: Mid-feature urgent bug scenario, student evaluates compression vs isolation decision
- [ ] T039 [US3] Document compression metrics: Context reclamation percentage (50%+ target), checkpoint token budget (500 tokens)
- [ ] T040 [US3] Document isolation criteria: Task similarity (domain boundaries), pollution risk (pattern cross-contamination)
- [ ] T041 [US3] Validate Three Roles demonstration: Lessons 4 and 5 show all three roles with 3+ iterations each (Test-003 compliance)
- [ ] T042 [US3] Validate cognitive load: Lesson 4 (8 concepts), Lesson 5 (8 concepts), both within B1 limit

**Checkpoint**: Lessons 4-5 complete, students can apply compression and isolation strategies with AI collaboration

---

## Phase 6: User Story 4 - Memory Files & Persistence (Lesson 6, Priority: P2)

**Goal**: Students design memory file architecture (CLAUDE.md, architecture.md, decisions.md) and create reusable memory-file-architecture skill (Stage 3 intelligence design)

**Independent Test**: Student designs memory file system for project, implements persistence across 2+ sessions, AI reads memory files and maintains context continuity without re-explanation

### Implementation for User Story 4

- [ ] T043 [US4] Create Lesson 6 file at `book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/06-memory-files-persistent-intelligence.md`
- [ ] T044 [US4] Write Lesson 6 content: Memory file types (CLAUDE.md structure, architecture.md templates, decisions.md ADR format), persistence strategy, skill encapsulation pattern (9 concepts, Stage 3 per plan.md lines 360-399)
- [ ] T045 [US4] Create CLAUDE.md template: Project conventions section, coding patterns section, anti-patterns section (500-1000 token structure)
- [ ] T046 [US4] Create architecture.md template: Component relationships section, design patterns section, system properties section
- [ ] T047 [US4] Create decisions.md template: ADR format (decision+context+rationale+alternatives)
- [ ] T048 [US4] Document persistence strategy: When to read (session initialization), when to update (architectural changes, new patterns, anti-patterns identified), conflict resolution (old vs new understanding)
- [ ] T049 [US4] Design memory file creation exercise: Student creates templates for sample project, demonstrates cross-session persistence
- [ ] T050 [US4] Encode memory-file-architecture as reusable skill: Templates + persistence strategy + update triggers (Stage 3 intelligence artifact per plan.md lines 371-379)
- [ ] T051 [US4] Validate reusable intelligence creation: Skill documented, reusable across projects, technology-agnostic
- [ ] T052 [US4] Validate cognitive load: 9 concepts at B1 upper boundary (acceptable per plan.md line 393)

**Checkpoint**: Lesson 6 complete, students can design persistent memory file systems

---

## Phase 7: User Story 5 - Tool Selection Framework (Lesson 7, Priority: P3)

**Goal**: Students understand decision criteria for Claude Code (200K context, deep reasoning) vs Gemini CLI (2M context, pattern analysis) and apply selection framework to scenarios

**Independent Test**: Given 3 scenarios (focused feature, large refactoring, legacy exploration), student selects appropriate tool with reasoning based on context requirements and task complexity

### Implementation for User Story 5

- [ ] T053 [US5] Create Lesson 7 file at `book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/07-tool-selection-framework.md`
- [ ] T054 [US5] Write Lesson 7 content: Claude Code vs Gemini CLI comparison, decision tree (task scope→context requirement→tool selection), 5+ scenario analysis (7 concepts, Stage 2-3 hybrid per plan.md)
- [ ] T055 [US5] Integrate verified comparison table from T010: Claude Sonnet 4.5 (200K/1M specs), Gemini 1.5 Pro (2M specs) with 2025 citations
- [ ] T056 [US5] Create decision tree diagram: Focused feature (5-10 files) → Claude Code, Large refactoring (50+ files) → evaluate tradeoffs, Legacy exploration (100K+ lines) → Gemini CLI
- [ ] T057 [US5] Design tool selection exercise: 3 scenarios with different context requirements, student justifies tool choice
- [ ] T058 [US5] Document Claude Code strengths: Deep reasoning, focused context, selective loading, precise control (200K window sufficient for most tasks)
- [ ] T059 [US5] Document Gemini CLI strengths: Massive context (2M), full codebase loading, pattern analysis across files, exploration without progressive loading
- [ ] T060 [US5] Create tool-selection-framework as reusable skill: Decision criteria + comparison table + scenario analysis (Stage 3 intelligence artifact)
- [ ] T061 [US5] Validate cognitive load: 7 concepts well within B1 limit (plan.md confirms acceptable)

**Checkpoint**: Lesson 7 complete, students can select appropriate AI tool based on context requirements

---

## Phase 8: Validation - Hands-On Debugging & Optimization (Lesson 8, Priority: P2)

**Goal**: Students diagnose failing sessions, optimize context usage, and apply learned patterns to fix common mistakes through scenario-based practice

**Independent Test**: Given 3 failing session scenarios (degradation, pollution, overload), student identifies issues, prescribes remediation, validates fixes

### Implementation for Validation Phase

- [ ] T062 [US-VAL] Create Lesson 8 file at `book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/08-hands-on-debugging-optimization.md`
- [ ] T063 [US-VAL] Write Lesson 8 content: Session diagnosis framework, remediation strategies, validation patterns (6 concepts, Stage 2 validation per plan.md)
- [ ] T064 [US-VAL] Create debugging scenario 1: Session with degradation symptoms (repetitive suggestions, forgotten patterns) → student diagnoses → prescribes compression checkpoint
- [ ] T065 [US-VAL] Create debugging scenario 2: Mixed-task session with context pollution (auth patterns in payment code) → student diagnoses → prescribes isolation
- [ ] T066 [US-VAL] Create debugging scenario 3: Overloaded session (95% context, no clear compression points) → student diagnoses → prescribes archival to memory files + restart
- [ ] T067 [US-VAL] Design optimization exercise: Student reviews 3 session transcripts, identifies improvement opportunities (better loading, earlier compression, isolation points)
- [ ] T068 [US-VAL] Document common mistakes: Loading all files upfront, ignoring degradation signals, mixing unrelated tasks, skipping memory files
- [ ] T069 [US-VAL] Validate hands-on discovery: Diagnosis scenarios precede remediation frameworks (Test-005 compliance)
- [ ] T070 [US-VAL] Validate cognitive load: 6 concepts well within B1 limit (review lesson per plan.md line 268)

**Checkpoint**: Lesson 8 complete, students can diagnose and fix context issues independently

---

## Phase 9: User Story 6 - Capstone: Spec-Driven Orchestration (Lesson 9, Priority: P3)

**Goal**: Students write complete specification for context-aware CLI tool orchestrating ALL accumulated intelligence (memory files + progressive loading + compression/isolation + multi-agent patterns) WITHOUT implementation code

**Independent Test**: Student independently writes comprehensive spec.md for context-aware tool, peer developer can understand complete behavior and implement from spec alone, specification includes measurable success criteria and testable requirements

### Implementation for User Story 6

- [ ] T071 [US6] Create Lesson 9 file at `book-source/docs/03-Markdown-Prompt-Context-Engineering/11-context-engineering-for-ai-driven-development/09-capstone-spec-driven-orchestration.md`
- [ ] T072 [US6] Write Lesson 9 content: Specification quality framework, accumulated intelligence orchestration, peer review rubric (Integration-level complexity, Stage 4 spec-driven per plan.md)
- [ ] T073 [US6] Document specification structure: Intent section (WHAT tool does), Success Criteria section (measurable outcomes), Functional Requirements section (context management capabilities)
- [ ] T074 [US6] Document memory file architecture integration: How tool uses CLAUDE.md/architecture.md/decisions.md from Lesson 6
- [ ] T075 [US6] Document progressive loading algorithm: Foundation/Current/On-Demand phase logic from Lesson 3
- [ ] T076 [US6] Document compression/isolation rules: When tool triggers checkpoint vs spawns new session from Lessons 4-5
- [ ] T077 [US6] Document multi-agent coordination: When to spawn specialized agents with isolated contexts from Six Components framework
- [ ] T078 [US6] Create specification quality rubric: Intent clarity (measurable), Success criteria (testable), Requirements (functional not implementation), Completeness (peer can implement)
- [ ] T079 [US6] Design capstone exercise: Student writes complete spec for context-aware CLI tool, peer reviews using rubric
- [ ] T080 [US6] Document specification-only constraint: ZERO implementation code, NO "use Redis", NO "implement with TypeScript" (spec mastery not coding skill per plan.md lines 240-242)
- [ ] T081 [US6] Validate capstone orchestration: Spec integrates memory files (L6) + progressive loading (L3) + compression/isolation (L4-5) + tool selection (L7) + degradation detection (L1-2) + multi-agent (Six Components) per plan.md lines 441-444
- [ ] T082 [US6] Validate spec-only constraint: NO programming implementation (Test-008 compliance)

**Checkpoint**: Lesson 9 complete, students can write implementation-ready specifications demonstrating accumulated intelligence orchestration

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Chapter-level validation, cross-lesson integration, final quality checks

- [ ] T083 Validate all lesson endings: Single "Try With AI" section per lesson, NO "What's Next", NO "Key Takeaways", NO "Summary", NO standalone "Safety Note" (Test-006 compliance for all 9 lessons)
- [ ] T084 [P] Validate hands-on discovery modality: All lessons follow experiment→observe→learn sequence, discoveries precede explanations (Test-005 compliance)
- [ ] T085 [P] Validate cognitive load management: Lesson concept counts (L1:7, L2:8, L3:9, L4:8, L5:8, L6:9, L7:7, L8:6, L9:Integration), all ≤10 B1 limit (Test-007 compliance)
- [ ] T086 [P] Validate Three Roles demonstrations: Lessons 3, 4, 5 show explicit role annotations with session transcripts and 3+ iterations (Test-003 compliance)
- [ ] T087 [P] Validate stage progression: Lessons 1-2 (Stage 1 manual), Lessons 3-5 (Stage 2 AI collaboration), Lessons 6-7 (Stage 3 intelligence), Lesson 9 (Stage 4 spec-driven), stage labels in plan.md NOT in student-facing text (Test-004 compliance)
- [ ] T088 [P] Validate research accuracy: All context window specs cite verified 2025 sources (Claude: Anthropic docs, Gemini: Google Cloud docs, Karpathy: Y Combinator, Anthropic research: Engineering Blog) per Test-001
- [ ] T089 [P] Validate Six Components framework: Preserved across multiple lessons (Model Selection, Dev Tools, Knowledge & Memory, Guardrails, Code Context, Orchestration), not isolated theory section (Test-012 compliance)
- [ ] T090 [P] Validate progressive loading with Three Roles: Lesson 3 demonstrates complete cycle (AI teaches pattern, student refines, convergence toward optimal strategy) per Test-013
- [ ] T091 [P] Validate anti-convergence from Chapter 10: Hands-on discovery modality documented (different from Chapter 10's direct teaching explain→demonstrate→practice) per Test-014
- [ ] T092 Update README.md with final learning objectives summary (LO-001 through LO-006) and lesson navigation
- [ ] T093 Verify YAML frontmatter consistency across all 9 lesson files (sidebar_position: 11, title formatting, Part 3 designation)
- [ ] T094 Final cross-lesson integration check: Lessons reference each other appropriately (L3 builds on L1-2, L4-5 apply L3 patterns, L6 enables persistence for L4-5, L9 orchestrates L1-8)

**Final Checkpoint**: Chapter 11 complete, all 9 lessons validated against acceptance tests, ready for content-implementer review and technical validation

---

## Dependencies & Execution Strategy

### User Story Completion Order

```
Phase 1 (Setup) → Phase 2 (Foundational Research) →
  ↓
Phase 3 (US1: Lessons 1-2) 🎯 MVP ←─── Foundation for all subsequent lessons
  ↓
Phase 4 (US2: Lesson 3) ←─────────────── Requires US1 complete (degradation understanding needed for loading strategy)
  ↓
Phase 5 (US3: Lessons 4-5) ←───────────── Requires US1-2 complete (degradation + loading knowledge needed for compression/isolation)
  ↓
Phase 6 (US4: Lesson 6) ←──────────────── Can start after US3 (compression/isolation inform persistence needs)
  ↓                            ↓
Phase 7 (US5: Lesson 7) ←──────────────── Requires US1 complete (context window understanding for tool comparison)
  ↓                            ↓
Phase 8 (Validation: Lesson 8) ←────────── Requires US1-5 complete (applies all learned patterns)
  ↓
Phase 9 (US6: Lesson 9 Capstone) ←──────── Requires US1-5 + Validation complete (orchestrates accumulated intelligence)
  ↓
Phase 10 (Polish) ←────────────────────── Requires all lessons complete
```

### Parallel Execution Opportunities

**After Phase 2 (Foundational) complete, limited parallelization possible**:

- **Setup tasks** (T001-T004): All parallelizable
- **Research verification** (T005-T010): All parallelizable
- **Lesson 1-2 creation** (T011-T012): File creation parallel, content writing sequential (T013-T014 must follow T011-T012)
- **Lessons 4-5 file creation** (T031-T032): Parallel within Phase 5
- **Validation tasks** (T083-T091 in Phase 10): All parallelizable after lessons complete

**Sequential dependencies**:
- Lesson 3 → Lessons 4-5 (progressive loading knowledge required for compression/isolation)
- Lessons 1-5 → Lesson 8 (validation applies all patterns)
- Lessons 1-8 → Lesson 9 (capstone orchestrates all intelligence)

### Implementation Strategy

**MVP Scope (User Story 1 - Lessons 1-2)**:
- Delivers foundation: Context window mechanics + degradation symptoms
- Enables manual diagnosis without AI (Stage 1 complete)
- Students can identify context issues and understand mitigation options
- Subsequent lessons build on this foundation incrementally

**Incremental Delivery**:
1. **Increment 1** (MVP): Lessons 1-2 (US1) - Manual foundation
2. **Increment 2**: Lesson 3 (US2) - Progressive loading with AI (first Three Roles demonstration)
3. **Increment 3**: Lessons 4-5 (US3) - Compression & isolation (complete Three Roles coverage)
4. **Increment 4**: Lessons 6-7 (US4-5) - Memory files + tool selection (intelligence design)
5. **Increment 5**: Lesson 8 (Validation) - Hands-on debugging
6. **Increment 6**: Lesson 9 (US6) - Capstone spec-driven orchestration

**Quality Gates**:
- After US1: Validate Stage 1 manual foundation complete
- After US2-3: Validate Three Roles demonstrated in 3+ lessons (Test-003)
- After US4: Validate reusable intelligence created (memory-file-architecture skill)
- After US6: Validate capstone spec-only (NO implementation code, Test-008)
- Phase 10: Validate all acceptance tests (Test-001 through Test-014)

---

## Summary

**Total Tasks**: 94
- Phase 1 (Setup): 4 tasks
- Phase 2 (Foundational): 6 tasks
- Phase 3 (US1): 10 tasks
- Phase 4 (US2): 10 tasks
- Phase 5 (US3): 12 tasks
- Phase 6 (US4): 10 tasks
- Phase 7 (US5): 9 tasks
- Phase 8 (Validation): 9 tasks
- Phase 9 (US6): 12 tasks
- Phase 10 (Polish): 12 tasks

**Tasks Per User Story**:
- US1 (Lessons 1-2, P1): 10 tasks
- US2 (Lesson 3, P1): 10 tasks
- US3 (Lessons 4-5, P2): 12 tasks
- US4 (Lesson 6, P2): 10 tasks
- US5 (Lesson 7, P3): 9 tasks
- US6 (Lesson 9, P3): 12 tasks
- Validation (Lesson 8, P2): 9 tasks

**Parallel Opportunities**:
- Phase 1: 3 tasks parallelizable (T002-T004)
- Phase 2: 5 tasks parallelizable (T006-T010)
- Phase 5: 2 file creation tasks parallel (T031-T032)
- Phase 10: 9 validation tasks parallel (T084-T091, T093)

**Independent Test Criteria**:
- US1: Manual degradation diagnosis without AI
- US2: Progressive loading maintaining context <70%
- US3: Compression vs isolation decision with reasoning
- US4: Memory file persistence across 2+ sessions
- US5: Tool selection with justified reasoning for 3 scenarios
- US6: Implementation-ready specification peer-reviewable

**MVP Scope**: User Story 1 (Lessons 1-2) - Manual foundation enabling all subsequent learning

**Format Validation**: ✅ All 94 tasks follow checklist format (checkbox + ID + [P]/[Story] labels + file paths)

---

## Policy Note for Lesson Authors

**Lesson Ending Convention**: Within this chapter, each lesson must end with a single final section titled **"Try With AI"** (no "Key Takeaways" or "What's Next").

**AI Tool Usage**:
- Before AI tools are taught (Part 1 chapters), use ChatGPT web interface in "Try With AI" sections
- After tool onboarding (Part 2+), instruct learners to use their preferred AI companion tool (e.g., Gemini CLI, Claude CLI)
- Optionally provide both CLI and web variants of exercises for accessibility

**Safety Integration**: Safety reminders must be integrated into "Try With AI" sections as 1-2 contextual sentences, NOT standalone "Safety Note" sections.
