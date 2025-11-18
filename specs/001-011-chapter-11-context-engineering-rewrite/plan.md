# Implementation Plan: Chapter 11 Context Engineering for AI-Driven Development

**Feature Branch**: `001-011-chapter-11-context-engineering-rewrite` | **Created**: 2025-01-18 | **Spec**: `/specs/001-011-chapter-11-context-engineering-rewrite/spec.md`

---

## Summary

**Objective**: Rewrite Chapter 11 (Context Engineering for AI-Driven Development) using hands-on discovery pedagogy, applying 4-stage teaching framework with explicit Three Roles demonstrations, cognitive load management (B1 tier), and capstone spec-driven project.

**Key Transformation**: Convert bloated theoretical content → Progressive hands-on discovery (experiment → observe → learn) with stage progression: Manual Foundation (Lessons 1-2) → AI Collaboration with Three Roles (Lessons 3-5) → Intelligence Design with reusable skills (Lessons 6-7) → Validation through scenarios (Lesson 8) → Spec-driven capstone (Lesson 9).

**Teaching Modality Shift** (Anti-Convergence from Chapter 10):
- Chapter 10: Direct Teaching (explain → demonstrate → practice)
- Chapter 11: Hands-On Discovery (experiment → observe → learn)

**Research Foundation**: Verified 2025 specifications for Claude Sonnet 4.5 (200K/1M context), Gemini 1.5 Pro (2M context), Karpathy "LLM as CPU" principle, Anthropic context degradation research.

---

## Constitutional Alignment

**Principles Applied**:
1. **Specification Primacy**: Capstone (Lesson 9) spec-only, no implementation
2. **Progressive Complexity**: B1 tier (7-10 concepts max per section)
3. **Factual Accuracy**: All context window specs verified against 2025 official sources
4. **Coherent Pedagogical Structure**: Foundation → Application → Integration → Validation → Mastery
5. **Intelligence Accumulation**: Memory files, compression, isolation skills reusable across projects
6. **Anti-Convergence Variation**: Hands-on discovery modality (different from Chapter 10's direct teaching)
7. **Minimal Sufficient Content**: Single "Try With AI" closing per lesson, no summary/key takeaways

**Stage Progression**: All 4 stages represented (Stage 1 manual foundation, Stage 2 AI collaboration with Three Roles, Stage 3 intelligence design, Stage 4 spec-driven capstone)

**Complexity Tier**: B1 Intermediate (7-10 concepts per section, moderate scaffolding, 3-4 options with selection criteria)

---

## Lesson Architecture (9 Lessons)

### Pedagogical Phases

| Phase | Lessons | Focus | Learning Outcome |
|-------|---------|-------|------------------|
| **Foundation** | 1-2 | Context windows, degradation symptoms, manual tracking | Understanding degradation patterns without AI |
| **Application** | 3-5 | Progressive loading, compression, isolation with Three Roles | Applying context strategies with AI collaboration |
| **Integration** | 6-7 | Memory files, tool selection | Designing reusable persistent intelligence |
| **Validation** | 8 | Hands-on debugging/optimization scenarios | Diagnosing and fixing context issues |
| **Mastery** | 9 | Spec-driven capstone: context-aware CLI tool specification | Orchestrating accumulated intelligence through specs |

---

## Detailed Lesson Plan

### Lesson 1: Context Windows and Token Counting (Stage 1: Manual Foundation)

**Stage Tag**: Stage 1 (Manual Foundation — No AI)

**Learning Objectives**:
- LO-101: Manually observe context window filling in AI sessions without relying on automated metrics
- LO-102: Identify the point at which context window approaches saturation (degradation threshold)
- LO-103: Distinguish between different types of content in context window (code, conversation, system prompts)

**Independent Test**: Student observes a pre-recorded Claude Code session incrementally loading files, manually tracks approximate token usage by counting words/code, identifies degradation symptoms (repetitive suggestions, forgotten patterns) without AI assistance.

**Cognitive Load Analysis**:
- **Concepts introduced (count: 7)**:
  1. Context window definition (working memory capacity)
  2. Token concept (atomic unit of text)
  3. Token counting methods (approximate estimation)
  4. Utilization percentage (0-100% metric)
  5. Content types in context (code, history, prompts, outputs)
  6. Saturation point (80%+ threshold)
  7. Degradation threshold concept
- **Chunking**: Context window mechanics (window + tokens + utilization) group as single chunk. Content types and degradation threshold separate chunks.
- **Progressive disclosure**: Manual observation first, degradation symptoms introduced second, framework explanation after hands-on discovery
- **Cognitive load**: 7 concepts ≤ B1 tier limit (10) → Within acceptable range ✅

**Discovery Sequence** (Hands-On Experimentation):
1. **Experiment**: Provide session transcript showing Claude Code with 100+ files, ask students to manually estimate token count by sampling (counting words in first 500 tokens, multiplying)
2. **Observe**: Show token usage graph overlaid on transcript, reveal actual vs estimated counts
3. **Learn**: Introduce token counting frameworks (word-to-token ratio: ~1.3 words = 1 token for English)

**Content Structure**:
- **Larmatta Opening**: Short scenario-based hook showing developer frustrated by repetitive AI suggestions
- **Hands-on exploration**: Manual token estimation exercise with sample codebase
- **Key concepts** (framework section): Token mechanics, utilization percentage calculation
- **Degradation observation**: Session transcript analysis identifying symptoms

**Skills Proficiency Mapping**:
- CEFR: B1 (Intermediate — students can independently estimate context usage)
- Bloom's: Understand + Apply (recognize tokens, calculate utilization)
- DigComp: Information Literacy (identify what content fills context), Problem Solving (estimate when window saturates)

**Try With AI**: Not yet (Stage 1 is manual only). Students validate context window understanding through self-check exercises without AI.

---

### Lesson 2: Degradation Symptoms and Manual Tracking (Stage 1: Manual Foundation)

**Stage Tag**: Stage 1 (Manual Foundation — No AI)

**Learning Objectives**:
- LO-201: Recognize 5+ degradation symptoms in failing AI sessions (repetitive code patterns, forgotten architectural decisions, performance slowdown, contradictory suggestions, inability to reference earlier context)
- LO-202: Manually track context utilization over session lifetime
- LO-203: Articulate when compression vs isolation vs restart is needed based on symptoms

**Independent Test**: Given a transcript from a 2-hour coding session showing degradation symptoms, student identifies specific symptoms, documents when they appear, and prescribes mitigation strategy (compression/isolation/restart) for each symptom.

**Cognitive Load Analysis**:
- **Concepts introduced (count: 8)**:
  1. Repetitive suggestion symptom
  2. Forgotten pattern symptom
  3. Performance degradation symptom
  4. Contradictory advice symptom
  5. Context reference loss symptom
  6. Manual tracking methodology
  7. Compression concept (save state, restart)
  8. Isolation concept (separate session)
- **Chunking**: Five degradation symptoms chunk as single group. Mitigation strategies (compression + isolation + restart) chunk as second group.
- **Progressive disclosure**: Symptoms introduced through session transcript analysis (discovery), mitigation strategies presented after symptom recognition (application)
- **Cognitive load**: 8 concepts ≤ B1 tier limit (10) → Within acceptable range ✅

**Discovery Sequence** (Hands-On Experimentation):
1. **Experiment**: Provide session transcript from long coding task (90+ minutes), ask students to mark points where AI gives repetitive suggestions or contradicts earlier statements
2. **Observe**: Overlay degradation metrics on transcript (token count, response quality score), correlate with student markings
3. **Learn**: Name degradation symptoms, introduce compression/isolation decision framework

**Content Structure**:
- **Scenario**: Developer working on feature implementation, watches AI quality degrade over 2 hours
- **Symptom discovery**: Manual transcript analysis exercise
- **Mitigation decision framework**: 3 options with selection criteria:
  - **Compression** (when): Same task, context pollution → Create checkpoint, restart with summary
  - **Isolation** (when): Different task, context switch needed → Spawn separate session
  - **Restart** (when): Severe degradation, recovery impossible → Start fresh
- **Manual tracking exercise**: Students manually create timeline of symptoms and mitigation decisions

**Skills Proficiency Mapping**:
- CEFR: B1 (Intermediate — diagnose issues and choose mitigation strategies)
- Bloom's: Analyze (identify symptoms from evidence), Evaluate (judge which mitigation fits)
- DigComp: Problem Solving (diagnose context issues), Information Literacy (track utilization)

**Try With AI**: Not yet (Stage 1 is manual only). Students validate degradation recognition through self-check quizzes.

---

### Lesson 3: Progressive Loading Strategy with AI Collaboration (Stage 2: Three Roles)

**Stage Tag**: Stage 2 (AI Collaboration with Three Roles)

**Learning Objectives**:
- LO-301: Apply three-phase progressive loading (Foundation → Current → On-Demand) to new codebase
- LO-302: Demonstrate AI as Teacher (AI suggests loading pattern student didn't know)
- LO-303: Demonstrate AI as Student (student teaches AI project-specific constraints)
- LO-304: Demonstrate Co-Worker convergence (iterate toward optimal loading strategy)

**Independent Test**: Given a 50-file codebase and a feature request, student independently applies progressive loading with AI collaboration, maintains context <70%, and demonstrates all three roles in session transcript (AI teaches pattern, student refines with project knowledge, both converge on strategy).

**Three Roles Demonstrations** (REQUIRED — Critical Test-003 compliance):

**Role 1: AI as Teacher**
- **Scenario**: Student loads basic project structure (5 files), starts implementing feature
- **AI teaching moment**: "For feature implementation, I recommend loading these layers: (1) Interface definitions + data models (2) Core business logic (3) Tests + examples. This Foundation → Current pattern keeps context focused while ensuring semantic completeness."
- **Student learns**: Progressive loading pattern with reasoning
- **Evidence**: Session transcript showing AI explaining pattern student didn't initially know

**Role 2: AI as Student**
- **Scenario**: AI suggests loading all middleware files for authentication feature
- **Student refines**: "We don't use all middleware — only JWT and rate limiting. Our project has custom auth pattern in services/auth/. Load that instead."
- **AI learns**: Project-specific constraints, refines suggestion
- **Convergence**: Both agree on minimal auth file set (3 files instead of 8)
- **Evidence**: Session transcript showing student correcting AI with domain knowledge

**Role 3: AI as Co-Worker**
- **Scenario**: Iterative refinement of loading strategy
  - Iteration 1: AI suggests "load everything for context completeness"
  - Student feedback: "Too much, context budget is limited"
  - Iteration 2: AI suggests "load only files you're modifying"
  - Student feedback: "Missing semantic context, AI can't understand interfaces"
  - Iteration 3: Convergence on "load interfaces + current implementation + tests"
  - Result: Solution neither had initially (balanced semantic + practical approach)
- **Evidence**: Session transcript with 3+ iteration rounds, convergence documented

**Cognitive Load Analysis**:
- **Concepts introduced (count: 9)**:
  1. Foundation phase (project structure, core configs)
  2. Current work phase (task-specific files)
  3. On-demand phase (just-in-time fetching)
  4. Progressive loading decision flow
  5. AI as Teacher role
  6. AI as Student role
  7. AI as Co-Worker role
  8. Iteration concept (convergence)
  9. Context budget constraint (70% utilization target)
- **Chunking**: Three phases chunk together (loading strategy). Three roles chunk together (collaboration pattern). Iteration and context budget separate concepts.
- **Progressive disclosure**: Hands-on loading exercise first (discovery), three roles framework introduced after observing need, decision flow framework applied in practice
- **Cognitive load**: 9 concepts ≤ B1 tier limit (10) → At upper boundary but within acceptable range ✅

**Discovery Sequence**:
1. **Experiment**: Provide 50-file codebase, ask student to work with AI to load context for new feature, naturally discovers phases
2. **Observe**: Track which files loaded in each phase, measure context utilization, observe AI output quality
3. **Learn**: Formalize as "Foundation → Current → On-Demand" pattern, introduce three roles framework

**Content Structure**:
- **Real-world scenario**: Student implementing new endpoint in existing FastAPI project (50+ files)
- **Hands-on exercise**: Use Claude Code to implement feature with progressive loading
- **Three Roles explicitly documented**: Session transcript with annotations showing each role
- **Decision framework**: Foundation/Current/On-Demand phases with selection criteria
- **Best practices**: What to load when, common mistakes (loading too much, too little)

**Skills Proficiency Mapping**:
- CEFR: B1 (Intermediate — apply pattern with AI collaboration)
- Bloom's: Apply (use progressive loading), Analyze (evaluate which files to load)
- DigComp: Problem Solving (determine file loading strategy), Communication (negotiate with AI)

**Try With AI**: Full lesson uses AI collaboration. Example transcript provided showing complete three roles cycle. Students execute similar pattern on own codebase.

---

### Lesson 4: Context Compression and Session Restart (Stage 2: Three Roles)

**Stage Tag**: Stage 2 (AI Collaboration with Three Roles)

**Learning Objectives**:
- LO-401: Create checkpoint summaries during long sessions (architectural decisions + progress + next steps, 500 tokens)
- LO-402: Restart sessions with compressed context and measure context reclamation
- LO-403: Demonstrate Three Roles (AI teaches compression pattern, student refines based on project, convergence toward optimal checkpoint)

**Independent Test**: During simulated 2-hour coding session at 85% context utilization, student creates checkpoint summary, restarts session with compressed context, continues task with reclaimed context space, documents reclamation metrics.

**Three Roles Demonstrations** (REQUIRED — Critical Test-003 compliance):

**Role 1: AI as Teacher**
- **Scenario**: Session at 85% utilization, code quality degrading
- **AI suggests**: "I recommend creating a checkpoint. Here's the pattern: (1) Summarize architectural decisions made so far (2) Document progress and completed tasks (3) List next steps with clear scope. Keep summary under 600 tokens. This becomes your new session prompt, reclaiming 50%+ context for continued work."
- **Student learns**: Checkpoint structure and purpose
- **Evidence**: Session transcript showing AI explaining checkpoint methodology

**Role 2: AI as Student**
- **Scenario**: AI creates generic checkpoint ("Implemented authentication, 3 tests passing, next: payment integration")
- **Student refines**: "We decided on JWT stateless auth with refresh token rotation — that's crucial for next phase. Also, 'payment integration' is vague — we specifically need to handle Stripe webhook verification and async charge processing."
- **AI learns**: Checkpoint needs specific architectural decisions and concrete next steps
- **Convergence**: Both agree revised checkpoint emphasizes architectural constraints and task clarity
- **Evidence**: Session transcript showing student improving checkpoint specificity

**Role 3: AI as Co-Worker**
- **Scenario**: Iteration on checkpoint content
  - Iteration 1: AI writes 50-token summary (too brief, loses context)
  - Student feedback: "Missing key design decisions"
  - Iteration 2: AI writes 1000-token summary (too long, defeats compression)
  - Student feedback: "Over the token budget"
  - Iteration 3: Convergence on 500-token checkpoint balancing completeness and efficiency
  - Result: Checkpoint format neither had initially (structured sections + token budget discipline)
- **Evidence**: 3+ iteration rounds documented

**Cognitive Load Analysis**:
- **Concepts introduced (count: 8)**:
  1. Checkpoint concept (session state snapshot)
  2. Compression ratio (50%+ reclamation)
  3. Checkpoint structure (decisions + progress + next steps)
  4. Token budget awareness (500-token target)
  5. Session restart pattern
  6. Context reclamation metric
  7. Three roles in compression context
  8. Iteration convergence
- **Chunking**: Checkpoint structure (content sections) chunks together. Compression mechanics (ratio, restart, reclamation) chunk together.
- **Cognitive load**: 8 concepts ≤ B1 tier limit (10) → Within acceptable range ✅

**Discovery Sequence**:
1. **Experiment**: Student works in long session until degradation appears, then manually creates checkpoint and restarts
2. **Observe**: Compare context reuse between checkpoint restart vs starting fresh (should be 50%+ reuse)
3. **Learn**: Formalize checkpoint structure, Three Roles pattern in compression workflow

**Content Structure**:
- **Real scenario**: 2-hour coding session building feature, experiencing degradation at 85% utilization
- **Checkpoint exercise**: Create structured checkpoint (decisions section, progress section, next steps section)
- **Session restart**: New session with checkpoint as initial context
- **Continuation**: Resume feature work with freed-up context
- **Three Roles transcript**: Complete cycle documented
- **Metrics dashboard**: Context utilization before/after compression

**Skills Proficiency Mapping**:
- CEFR: B1 (Intermediate — design and apply compression strategy)
- Bloom's: Create (design checkpoint format), Apply (execute restart pattern)
- DigComp: Problem Solving (diagnose compression need), Information Literacy (structure decision documentation)

**Try With AI**: Full Three Roles collaboration in checkpoint creation and session restart.

---

### Lesson 5: Context Isolation for Parallel Tasks (Stage 2: Three Roles)

**Stage Tag**: Stage 2 (AI Collaboration with Three Roles)

**Learning Objectives**:
- LO-501: Distinguish between tasks requiring compression (same context) vs isolation (separate context)
- LO-502: Apply isolation decision framework (task similarity scoring, context pollution risk)
- LO-503: Demonstrate Three Roles (AI teaches isolation pattern, student refines based on project workflow, convergence toward decision criteria)

**Independent Test**: Given 3 scenarios (mid-feature urgent bug, parallel features in same module, unrelated feature in different module), student independently decides: compress vs isolate with reasoning, and demonstrates Three Roles collaboration in isolation decision.

**Three Roles Demonstrations** (REQUIRED — Critical Test-003 compliance):

**Role 1: AI as Teacher**
- **Scenario**: Student working on authentication feature when critical payment processing bug appears
- **AI suggests**: "I recommend isolation. Here's why: authentication and payment are distinct domains (different models, business logic, tests). Mixing them in single context creates pollution — AI might suggest authentication patterns in payment code. Separate sessions preserve focus. Isolation pattern: (1) Save current auth session state (2) Create new isolated session for bug fix (3) Reconcile fixes when both complete."
- **Student learns**: Task similarity scoring and pollution concept
- **Evidence**: AI explaining decision framework

**Role 2: AI as Student**
- **Scenario**: AI suggests isolating two endpoint updates ("different endpoints, isolate them")
- **Student refines**: "No, both updates are in the same User API module, share same database models, same test patterns. They should stay in same session — isolation would duplicate context loading. This is compression scenario, not isolation."
- **AI learns**: Isolation applies to distinct domains, not just different files
- **Convergence**: Both agree isolation threshold is semantic similarity, not file location
- **Evidence**: Student teaching AI domain understanding

**Role 3: AI as Co-Worker**
- **Scenario**: Iteration on isolation criteria
  - Iteration 1: AI suggests "isolate if in different directories"
  - Student feedback: "Doesn't match how we organize code"
  - Iteration 2: AI suggests "isolate if different frameworks involved"
  - Student feedback: "Too strict, we sometimes refactor between frameworks"
  - Iteration 3: Convergence on "isolate if different business domains or high risk of pattern cross-contamination"
  - Result: Decision framework tailored to project neither had initially
- **Evidence**: Iteration rounds documented

**Cognitive Load Analysis**:
- **Concepts introduced (count: 8)**:
  1. Task similarity concept
  2. Context pollution risk
  3. Domain boundary concept
  4. Isolation vs compression decision point
  5. Isolation pattern mechanics (session spawning)
  6. Pattern cross-contamination risk
  7. Three roles in isolation decision
  8. Parallel session management
- **Chunking**: Isolation mechanics (when, how) chunk together. Pollution risks and similarity scoring chunk together.
- **Cognitive load**: 8 concepts ≤ B1 tier limit (10) → Within acceptable range ✅

**Discovery Sequence**:
1. **Experiment**: Student experiences context pollution (AI suggesting auth patterns in payment code), recognizes need for isolation
2. **Observe**: Compare results with isolated vs mixed sessions
3. **Learn**: Formalize as "task similarity" decision framework with Three Roles applied

**Content Structure**:
- **Scenario**: Mid-feature development when urgent bug arrives, student must decide isolation strategy
- **Decision exercise**: Evaluate 3+ task pairs (should isolate? compress?), reason through decisions
- **Three Roles collaboration**: AI and student iterate on isolation criteria
- **Parallel session management**: Running multiple sessions, synchronizing results
- **Pattern integration**: Combine with previous lessons (compression vs isolation framework)

**Skills Proficiency Mapping**:
- CEFR: B1 (Intermediate — evaluate and apply isolation decisions)
- Bloom's: Analyze (evaluate task similarity), Evaluate (judge isolation necessity)
- DigComp: Problem Solving (determine when to isolate), Communication (negotiate isolation decisions)

**Try With AI**: Full Three Roles collaboration in isolation decision-making.

---

### Lesson 6: Memory Files and Persistent Intelligence (Stage 3: Intelligence Design)

**Stage Tag**: Stage 3 (Intelligence Design — Creating Reusable Skills)

**Learning Objectives**:
- LO-601: Design memory file architecture (CLAUDE.md structure, architecture.md templates, decisions.md pattern)
- LO-602: Implement memory persistence strategy (when to read/write/update across sessions)
- LO-603: Create reusable memory file skill encapsulating templates and persistence logic

**Independent Test**: Student designs memory file system for project (CLAUDE.md with conventions, architecture.md with component relationships, decisions.md with ADRs), implements persistence across 2+ sessions, demonstrates AI reads memory files and maintains context continuity without re-explanation.

**Reusable Intelligence Artifact Created**:
- **Skill Name**: memory-file-architecture
- **What it does**: Templates + persistence strategy + update triggers for multi-session project context
- **Components**:
  - CLAUDE.md template (project conventions, patterns, anti-patterns)
  - architecture.md template (component relationships, design patterns, system properties)
  - decisions.md template (ADR format: decision + context + rationale + alternatives considered)
  - Persistence strategy (when to read at session start, when to update after session, conflict resolution)
  - Update triggers (architectural changes, new patterns discovered, anti-patterns identified)

**Cognitive Load Analysis**:
- **Concepts introduced (count: 9)**:
  1. Memory file concept (external persistent context)
  2. CLAUDE.md structure (conventions section)
  3. architecture.md structure (design patterns section)
  4. decisions.md structure (ADR format)
  5. Session initialization with memory files
  6. Memory file update triggers
  7. Persistence strategy (read/write/update pattern)
  8. Conflict resolution (old vs new understanding)
  9. Skill encapsulation pattern
- **Chunking**: Three memory file types (CLAUDE + architecture + decisions) chunk together. Persistence (when to read/write/update) chunks together.
- **Cognitive load**: 9 concepts ≤ B1 tier limit (10) → At upper boundary ✅

**Content Structure**:
- **Discovery starting point**: Show project that evolved over multiple sessions, notice AI repeating explanations in new sessions
- **Memory file design exercise**: Create templates for CLAUDE.md, architecture.md, decisions.md
- **Persistence strategy design**: How often to update files, what triggers updates, conflict resolution
- **Skill creation**: Encapsulate templates + strategy as reusable skill (memory-file-architecture)
- **Testing across sessions**: Implement memory files, start new session 3+ days later, verify AI reads memory and maintains continuity

**Skills Proficiency Mapping**:
- CEFR: B1 (Intermediate — design and implement memory system)
- Bloom's: Create (design memory architecture), Apply (implement templates), Analyze (evaluate persistence strategy)
- DigComp: Information Literacy (structure project knowledge), Problem Solving (design update strategy)

**Try With AI**: Students use AI to help structure memory files and persistence strategy, but focus is creating reusable intelligence artifact (skill).

---

### Lesson 7: Tool Selection Framework — Claude Code vs Gemini CLI (Stage 2-3 Hybrid)

**Stage Tag**: Stage 2-3 Hybrid (AI Collaboration + Intelligence Design)

**Learning Objectives**:
- LO-701: Apply decision framework for tool selection (Claude Code 200K context vs Gemini CLI 2M context)
- LO-702: Evaluate tradeoffs: context window size vs reasoning depth vs file selection control
- LO-703: Create reusable tool selection decision skill with scenario analysis

**Independent Test**: Given 5 scenarios (focused feature, large refactoring, legacy exploration, multi-day project, unknown codebase), student independently selects tool with justified reasoning based on context requirements, task complexity, tool capabilities.

**Tool Comparison Framework** (Verified 2025 Specifications):

| Dimension | Claude Code | Gemini CLI |
|-----------|------------|-----------|
| **Standard Context** | 200K tokens | 128K tokens |
| **Extended Context (if available)** | 1M tokens (tier 4+ organizations) | 2M tokens (extended) |
| **Output Capacity** | 64K tokens | Not publicly specified |
| **Context Awareness** | Tracks remaining token budget | Not publicly specified |
| **Typical Best For** | Focused development, deep reasoning | Large codebase exploration, pattern analysis |
| **File Selection** | Progressive loading (selective), full control | Load entire codebase (simpler, less control) |
| **Reasoning Depth** | Superior for complex architectural decisions | Pattern analysis across large codebases |
| **Sources** | Anthropic docs 2025, JuheAPI Guide 2025 | Google Cloud docs 2025 |

**Decision Framework**:

**Choose Claude Code when**:
- Task scope is 1-5 focused files (architecture, feature, module)
- Deep reasoning needed (complex architectural decisions, security design)
- Context budget is priority (long session continuity needed)
- Control over context loading matters (selective file addition)
- Standard development workflow (IDE integration, fast iteration)

**Choose Gemini CLI when**:
- Codebase is 100+ files, need to explore unfamiliar patterns
- Task is broad pattern analysis (refactor across codebase, style standardization)
- Extended context (2M) provides value (load entire codebase at once)
- Exploration phase (understand architecture before implementation)
- Less context control acceptable trade-off for exploration breadth

**Hybrid approach**:
- **Exploration phase**: Gemini CLI (2M context, load entire codebase, understand patterns)
- **Implementation phase**: Claude Code (200K context, focused work, deep reasoning)
- **Validation phase**: Could use either depending on scope

**Cognitive Load Analysis**:
- **Concepts introduced (count: 7)**:
  1. Claude Code specs (200K standard, 1M extended, 64K output)
  2. Gemini CLI specs (2M context for exploration)
  3. Context window tradeoff
  4. Reasoning depth capability
  5. File selection strategy differences
  6. Task complexity assessment
  7. Tool selection decision criteria
- **Chunking**: Tool specifications (Claude + Gemini) chunk together. Decision criteria (context, reasoning, control) chunk together.
- **Cognitive load**: 7 concepts ≤ B1 tier limit (10) → Well within acceptable range ✅

**Reusable Intelligence Artifact Created**:
- **Skill Name**: tool-selection-framework
- **What it does**: Decision criteria + comparison table + scenario-based guidance for Claude Code vs Gemini CLI
- **Components**:
  - Verified 2025 specs table (updatable)
  - Decision decision tree (task scope → context requirement → reasoning need → tool recommendation)
  - Scenario analysis (5+ real scenarios with tool selection + reasoning)
  - Hybrid approach guidance (when to use both tools in sequence)

**Content Structure**:
- **Verified comparison table**: Claude Code vs Gemini CLI with cited 2025 sources
- **Decision scenarios** (5 real-world examples):
  1. Implement single endpoint in existing 20-file module → Claude Code (focused, deep reasoning)
  2. Refactor API validation across 50-file codebase → Evaluate both (Gemini for exploration, Claude for implementation)
  3. Explore unfamiliar 100K-line legacy system → Gemini CLI (2M context for pattern analysis)
  4. Multi-day feature development → Claude Code (context persistence with memory files)
  5. Emergency bug fix in unknown module → Depends on module complexity/codebase size
- **Decision framework exercise**: Students evaluate scenarios, justify tool selection, practice decision-making
- **Skill creation**: Encapsulate decision framework as reusable tool-selection-framework skill

**Skills Proficiency Mapping**:
- CEFR: B1 (Intermediate — evaluate and apply decision framework)
- Bloom's: Analyze (compare tool capabilities), Evaluate (judge tool fit for task)
- DigComp: Problem Solving (determine appropriate tool), Information Literacy (understand tool limitations)

**Try With AI**: AI helps analyze scenarios and refine decision criteria, but students lead tool selection reasoning.

---

### Lesson 8: Hands-On Debugging and Optimization Scenarios (Stage 2: AI Collaboration)

**Stage Tag**: Stage 2 (Validation Phase — AI Collaboration)

**Learning Objectives**:
- LO-801: Diagnose failing context scenarios (degradation, pollution, saturation)
- LO-802: Apply remediation strategies (compression, isolation, progressive loading, memory files)
- LO-803: Measure optimization results (context utilization, response quality, session continuity)

**Independent Test**: Given 4 scenarios with failing context management (one high degradation, one pollution issue, one saturation problem, one persistence failure), student diagnoses root cause, applies appropriate strategy, validates improvement through metrics.

**Cognitive Load Analysis**:
- **Concepts introduced (count: 6 — review/reinforcement from previous lessons)**:
  1. Degradation diagnosis (review from Lesson 2)
  2. Pollution diagnosis (review from Lesson 5)
  3. Saturation diagnosis (review from Lesson 1)
  4. Persistence debugging (review from Lesson 6)
  5. Strategy selection (compression vs isolation)
  6. Metrics evaluation (context utilization, quality metrics)
- **Chunking**: Diagnosis (identifying problem type) as single chunk. Remediation (applying solution) as single chunk.
- **Cognitive load**: 6 concepts ≤ B1 tier limit (10) → Well within acceptable range ✅
- **Note**: This is consolidation lesson, not introducing many new concepts, but applying previous knowledge in integrated scenarios

**Content Structure**:
- **Scenario 1: Degradation Crisis**
  - Session at 90% utilization, AI giving repetitive suggestions, forgotten earlier decisions
  - Diagnosis exercise: Identify degradation symptoms
  - Remediation: Student creates checkpoint summary, restarts session
  - Validation: Measure context reclamation, response quality improvement

- **Scenario 2: Context Pollution**
  - Two unrelated features mixed in same session, AI confusing patterns between them
  - Diagnosis exercise: Identify cross-domain pattern contamination
  - Remediation: Student isolates into separate sessions
  - Validation: Compare output quality (isolated vs mixed)

- **Scenario 3: Saturation Problem**
  - Context window already 85%+ filled before implementing critical feature
  - Diagnosis exercise: Evaluate available context budget
  - Remediation: Student applies progressive loading (load only essential context)
  - Validation: Implement feature within reduced context budget

- **Scenario 4: Persistence Failure**
  - New session, memory files not loaded or outdated, AI doesn't remember project patterns
  - Diagnosis exercise: Identify persistence gap
  - Remediation: Update memory files, reload in new session
  - Validation: AI references memory file content, maintains continuity

**Skills Proficiency Mapping**:
- CEFR: B1 (Intermediate — diagnose and remediate issues)
- Bloom's: Analyze (identify problem type), Apply (select and execute remediation)
- DigComp: Problem Solving (diagnose context issues), Information Literacy (evaluate metrics)

**Try With AI**: Students work with AI to diagnose and remediate each scenario.

---

### Lesson 9: Capstone — Spec-Driven Context Orchestration Tool (Stage 4: Specification-Only)

**Stage Tag**: Stage 4 (Capstone — Spec-Driven Integration, NO Implementation)

**Learning Objectives**:
- LO-901: Write complete, implementation-ready specification for context-aware CLI tool
- LO-902: Orchestrate accumulated intelligence from Lessons 1-8 (memory files + progressive loading + compression/isolation + tool selection)
- LO-903: Validate spec completeness (clear intent, measurable success criteria, testable requirements)

**Independent Test**: Student writes spec.md for context-aware development tool that is implementation-ready. A peer developer who has NOT taken course can read spec and build system without asking clarifying questions. Spec includes: intent statement, measurable success criteria, context management architecture, progressive loading algorithm, compression/isolation decision logic, memory file persistence, multi-agent patterns. **ZERO lines of implementation code** (no Python, TypeScript, shell scripts, pseudo-code).

**Capstone Architecture** (What Spec Must Orchestrate):

```
Context-Aware CLI Tool
├── Session Management
│   ├── Progressive Loading (Lesson 3 strategy)
│   └── File Selection (Foundation → Current → On-Demand phases)
├── Context Lifecycle
│   ├── Compression Strategy (Lesson 4 checkpoint pattern)
│   ├── Isolation Decision (Lesson 5 task similarity framework)
│   └── Metrics Tracking (Lesson 1-2 monitoring)
├── Persistent Intelligence
│   ├── CLAUDE.md (project conventions)
│   ├── architecture.md (system design)
│   └── decisions.md (ADRs)
├── Tool Orchestration
│   ├── Claude Code vs Gemini CLI selection (Lesson 7 framework)
│   └── Multi-agent coordination (when to spawn subagents)
└── State Machine
    ├── Degradation detection (Lesson 2 symptoms)
    ├── Compression trigger (when context > 80%)
    ├── Isolation detection (task similarity threshold)
    └── Restart recovery (session recovery pattern)
```

**Cognitive Load Analysis**:
- This is INTEGRATION lesson, synthesizing all prior learning
- Students not introducing new isolated concepts, but orchestrating existing ones
- Complexity is in composition, not individual concept count
- B1 tier can handle this integration (they have Stage 1-3 foundation)

**Specification Quality Framework** (What Makes Spec "Implementation-Ready"):

**1. Intent Clarity (WHAT tool does, not HOW)**
- ✅ Good: "Context-aware CLI that monitors context utilization, automatically applies compression when degradation appears, persists project memory across sessions"
- ❌ Bad: "CLI tool using Redis to cache context, implemented in Python with SQLAlchemy, runs on Ubuntu servers"

**2. Success Criteria (Measurable, Falsifiable)**
- ✅ Good: "80%+ of development sessions maintain context utilization under 70%, zero manual re-explanation of project patterns in new sessions"
- ❌ Bad: "Tool works well, improves productivity, developers are happy"

**3. Functional Requirements (Testable, Observable)**
- ✅ Good: "When context utilization exceeds 80%, tool must create checkpoint summary within 2 minutes, restart session with summary, continue task with <40% utilization"
- ❌ Bad: "Tool should monitor context and optimize it"

**4. System Architecture (Components + Interactions, no implementation details)**
- ✅ Good: "System has (1) Context Monitor (tracks utilization), (2) Compression Engine (creates summaries), (3) Isolation Detector (scores task similarity), (4) Memory Manager (reads/writes persistent files). Monitor → Detector → Compression Engine → Memory Manager workflow."
- ❌ Bad: "We'll use FastAPI backend, React frontend, PostgreSQL database"

**5. Progressive Loading Algorithm** (From Lesson 3)
- Spec must define: Foundation phase logic (what auto-loads), Current work detection (how to identify task-relevant files), On-demand fetching (when to dynamically load)
- Example structure: "Foundation phase: Auto-load project/config.yaml, README.md, src/__init__.py. Current work phase: When user specifies task, load task-relevant files (TBD by user). On-demand: When AI requests file, fetch dynamically."

**6. Compression/Isolation Decision Logic** (From Lessons 4-5)
- Spec must define: When to compress (same task, 80%+ utilization, architectural decisions at risk), When to isolate (different domain, >X% dissimilarity to current context)
- Example: "Compression trigger: utilization > 80% AND session duration > 60 minutes. Isolation trigger: task similarity < 50% (calculated via: shared models? shared database? shared business domain?)"

**7. Memory Persistence Architecture** (From Lesson 6)
- Spec must define: CLAUDE.md structure (what goes in conventions section), architecture.md structure (what goes in design section), decisions.md structure (ADR format)
- Update triggers: "Update CLAUDE.md when new pattern discovered, update decisions.md when architectural decision made, update architecture.md when system design changes"

**8. Multi-Agent Coordination** (From Six Components framework)
- Spec must define: When to spawn specialized agents, what context each agent receives, how to consolidate results
- Example: "When task spans 3+ domains, spawn isolated agents per domain, compress each agent's final output (1K tokens), merge outputs in orchestrator agent"

**Content Structure**:

1. **Specification Framework Introduction**
   - What makes specs implementation-ready (clarity, completeness, measurability)
   - Specification structure: Intent → Success Criteria → Requirements → Architecture → Algorithms

2. **Lesson 9 Exercise: Write Spec.md**
   - Prompt: "Write complete specification for context-aware CLI tool that orchestrates all lessons 1-8 intelligence"
   - Constraints:
     - Intent must be clear (WHAT tool does)
     - Success criteria must be measurable (no "works well")
     - Requirements must be testable (observable behavior)
     - Architecture must include all components: Context Monitor, Compression Engine, Isolation Detector, Memory Manager
     - Progressive loading algorithm must be defined
     - Compression/isolation decision logic must be explicit
     - Memory persistence must be architected
     - Multi-agent coordination must be specified
     - **ZERO implementation details** (no language, no database, no infrastructure)

3. **Peer Review Protocol**
   - Exchange specs with peer developer (ideally from different cohort)
   - Reviewer assesses: Can I build this system from this spec alone?
   - Reviewer documents: What's clear? What's ambiguous? What's missing?
   - Student revises based on feedback

4. **Validation Checklist**
   - [ ] Spec describes WHAT tool does without HOW it's built
   - [ ] Intent is 1-2 paragraphs, crystal clear
   - [ ] Success criteria are measurable (includes numbers)
   - [ ] Functional requirements are observable (not vague)
   - [ ] System architecture includes all 4 components with interaction flow
   - [ ] Progressive loading algorithm defined (phases + logic)
   - [ ] Compression/isolation decision criteria explicit
   - [ ] Memory persistence architecture detailed
   - [ ] Multi-agent patterns specified
   - [ ] Zero implementation code present
   - [ ] Spec is 3-5 pages (complete but concise)

**Skills Proficiency Mapping**:
- CEFR: B1 (Intermediate — write complete specification)
- Bloom's: Create (design system architecture), Evaluate (assess spec completeness)
- DigComp: Information Literacy (structure technical information), Problem Solving (orchestrate components)

**Try With AI**: Students use AI as co-designer (AI suggests architecture patterns, student refines with accumulated context engineering knowledge), but spec quality and completeness is student's responsibility. AI does NOT implement; student writes spec.md only.

---

## Three Roles Demonstrations Compliance (Test-003)

**Required**: At least 3 lessons demonstrate all three roles

**Implemented in**:
- **Lesson 3 (Progressive Loading)**: FULL THREE ROLES
  - AI as Teacher: Suggests Foundation → Current → On-Demand pattern
  - AI as Student: Learns project-specific constraints
  - Co-Worker: Iterate toward optimal loading for THIS codebase

- **Lesson 4 (Context Compression)**: FULL THREE ROLES
  - AI as Teacher: Explains checkpoint structure
  - AI as Student: Learns specific decision requirements
  - Co-Worker: Iterate toward optimal checkpoint format

- **Lesson 5 (Context Isolation)**: FULL THREE ROLES
  - AI as Teacher: Explains isolation pattern and pollution risk
  - AI as Student: Learns project's task boundaries
  - Co-Worker: Iterate toward project-specific isolation criteria

**Evidence Format**: Session transcripts with annotations showing each role explicitly

---

## Reusable Intelligence Created (Stage 3)

**Skill 1: memory-file-architecture** (Lesson 6)
- Templates: CLAUDE.md, architecture.md, decisions.md
- Persistence strategy: Read on session start, update on design changes, conflict resolution
- Update triggers: New patterns, architectural decisions, anti-patterns identified
- **Reusable across**: All future projects requiring multi-session continuity

**Skill 2: tool-selection-framework** (Lesson 7)
- Decision tree: Task scope → Context requirements → Reasoning depth → Tool recommendation
- Comparison table: Claude Code vs Gemini CLI (verified 2025 specs)
- Scenario analysis: 5+ real-world scenarios with reasoning
- **Reusable across**: Any tool selection decision in AI-native development

**Skill 3: progressive-loading-strategy** (Lesson 3 formalized)
- Foundation/Current/On-Demand phase definitions
- File selection criteria for each phase
- Context budget management patterns
- **Reusable across**: Any large codebase exploration with context constraints

---

## Capstone Orchestration (Stage 4)

**Capstone Spec Must Orchestrate**:
1. **Memory Files** (Skill 1): How persistent intelligence persists across sessions
2. **Progressive Loading** (Skill 3): How context is managed during exploration
3. **Compression/Isolation** (Lessons 4-5): Decision logic for session lifecycle
4. **Tool Selection** (Skill 2): When to use Claude Code vs Gemini CLI
5. **Degradation Detection** (Lessons 1-2): Symptom recognition and response
6. **Multi-Agent Patterns** (Six Components framework): How specialized agents coordinate

**Capstone does NOT include**: Implementation code, database schemas, API endpoints, infrastructure details

---

## Cognitive Load Validation

| Lesson | Concepts Count | B1 Limit | Status |
|--------|---|---|---|
| 1: Context Windows | 7 | 10 | ✅ Within limit |
| 2: Degradation Symptoms | 8 | 10 | ✅ Within limit |
| 3: Progressive Loading | 9 | 10 | ✅ At boundary (acceptable) |
| 4: Context Compression | 8 | 10 | ✅ Within limit |
| 5: Context Isolation | 8 | 10 | ✅ Within limit |
| 6: Memory Files | 9 | 10 | ✅ At boundary (acceptable) |
| 7: Tool Selection | 7 | 10 | ✅ Well within |
| 8: Debugging Scenarios | 6 | 10 | ✅ Well within (review lesson) |
| 9: Spec Capstone | Integration | 10 | ✅ Synthesis, not new concepts |

**Chunking Strategy**:
- Related concepts grouped (e.g., degradation symptoms chunk as single unit, not 5 separate concepts)
- Progression from simple (context window tracking) to complex (orchestration)
- Each lesson builds on previous, reducing perceived novelty
- Practice and application spaces support cognitive load

---

## Acceptance Test Compliance Verification

**Test-003: Three Roles demonstrated in 3+ lessons**
- ✅ Lesson 3: Progressive Loading (AI Teacher + AI Student + Co-Worker)
- ✅ Lesson 4: Context Compression (AI Teacher + AI Student + Co-Worker)
- ✅ Lesson 5: Context Isolation (AI Teacher + AI Student + Co-Worker)
- ✅ Lesson 8 (optional): Debugging scenarios with AI collaboration

**Test-004: Stage progression explicit in plan, natural in student text**
- ✅ Plan.md: Each lesson tagged with stage (Stage 1, 2, 3, 4)
- ✅ Student text: No "Stage 2" language, natural progression through discovery
- ✅ Stages are planning tools, not student vocabulary

**Test-005: Hands-on discovery modality evidenced**
- ✅ Lesson 1: Manual observation → discover token counting
- ✅ Lesson 2: Transcript analysis → discover degradation symptoms
- ✅ Lesson 3-5: Hands-on exercises → discover patterns → learn frameworks
- ✅ Lesson 8: Scenario diagnosis → discover solutions
- ✅ Discovery precedes explanation in all lessons

**Test-006: Lesson endings follow minimal content protocol**
- ✅ Every lesson ends with SINGLE "Try With AI" section
- ✅ NO "What's Next", "Key Takeaways", "Summary" sections
- ✅ Safety integrated into "Try With AI" (1-2 contextual sentences)

**Test-007: Cognitive load managed (B1 tier)**
- ✅ All lessons: 6-9 concepts ≤ 10 limit
- ✅ Related concepts chunked together
- ✅ Options limited to 3-4 with selection criteria
- ✅ Progressive disclosure: simple before complex

**Test-008: Capstone is spec-only (NO implementation)**
- ✅ Lesson 9: Students write spec.md only
- ✅ NO Python code, NO TypeScript, NO shell scripts
- ✅ Spec includes: intent, success criteria, architecture, algorithms
- ✅ Specification quality validated through peer review

**Test-012: Six Components framework preserved**
- ✅ Model Selection: Tool selection framework (Lesson 7)
- ✅ Development Tools: Claude Code vs Gemini CLI
- ✅ Knowledge & Memory: Memory files (Lesson 6)
- ✅ Guardrails: Validation rules in isolation decision (Lesson 5)
- ✅ Code Context: Progressive loading (Lesson 3)
- ✅ Orchestration: Multi-agent patterns in capstone (Lesson 9)

**Test-013: Progressive loading demonstrated with Three Roles**
- ✅ Lesson 3: Complete three-roles cycle
- ✅ AI as Teacher: Suggests loading pattern
- ✅ AI as Student: Learns project constraints
- ✅ Co-Worker: Converge on optimal strategy
- ✅ Session transcript documents complete cycle

**Test-014: Anti-convergence from Chapter 10 validated**
- ✅ Chapter 10: Direct Teaching modality (explain → demonstrate → practice)
- ✅ Chapter 11: Hands-On Discovery modality (experiment → observe → learn)
- ✅ Different teaching approaches, no convergence on lecture style
- ✅ Evidence: Lessons start with experiments, explanations follow discoveries

---

## Research Foundation Integration

**Claude Sonnet 4.5 Specifications** (Lessons 1, 7)
- Standard context: 200K tokens (500 pages)
- Extended context (beta): 1M tokens for tier 4+ organizations
- Output capacity: 64K tokens
- Context awareness: Model tracks remaining token budget
- Source: Anthropic Documentation, JuheAPI Guide (verified 2025-01-18)
- Application: Context window mechanics (Lesson 1), tool selection framework (Lesson 7)

**Gemini 1.5 Pro Specifications** (Lessons 1, 7)
- Standard context: 128K tokens
- Extended context: 2M tokens (99% accuracy in Needle in Haystack evaluation)
- File handling: Process 1 hour video, 11 hours audio, 700K+ words, 30K+ lines of code
- Source: Google Developers Blog, Google Cloud Documentation (verified 2025-01-18)
- Application: Context window comparison (Lesson 1), tool selection for large codebases (Lesson 7)

**Karpathy "LLM as CPU" Principle** (Lesson 1 opening context)
- Core analogy: "LLM is the CPU, context window is the RAM, context engineering is filling RAM with right information for next step"
- Source: Andrej Karpathy, "Software Is Changing (Again)" at Y Combinator AI School
- Application: Conceptual framework for understanding context mechanics

**Anthropic Context Degradation Research** (Lessons 1-2, 5)
- Phenomenon: Transformer n² attention mechanism, less training on long sequences, gradual degradation
- Mitigation strategies: Compaction, structured note-taking, sub-agent architectures
- Best practices: Minimal token sets, curated tool sets, just-in-time fetching
- Source: Anthropic Engineering Blog — "Effective Context Engineering for AI Agents"
- Application: Degradation symptoms (Lesson 2), compression strategy (Lesson 4), memory files (Lesson 6)

---

## Teaching Modality Anti-Convergence

| Chapter | Teaching Modality | Method |
|---------|---|---|
| **Chapter 10** | Direct Teaching | Explain → Demonstrate → Practice |
| **Chapter 11** | Hands-On Discovery | Experiment → Observe → Learn ← **DIFFERENT** |

**Evidence in Lessons**:
- Lesson 1: Students observe token filling → discover token counting (not explained first)
- Lesson 2: Students identify degradation symptoms → learn framework (not taught first)
- Lessons 3-5: Students work through exercises → discover patterns → formalize as frameworks
- Lesson 8: Students diagnose failing scenarios → apply solutions (active problem-solving)
- Anti-convergence achieved through consistent discovery-first approach

---

## Implementation Tasks for Content-Implementer

The content-implementer agent will create 9 lesson markdown files following this plan:

1. `01-lesson-1.md` — Context Windows and Token Counting (Stage 1)
2. `02-lesson-2.md` — Degradation Symptoms and Manual Tracking (Stage 1)
3. `03-lesson-3.md` — Progressive Loading Strategy (Stage 2 with Three Roles)
4. `04-lesson-4.md` — Context Compression and Session Restart (Stage 2 with Three Roles)
5. `05-lesson-5.md` — Context Isolation for Parallel Tasks (Stage 2 with Three Roles)
6. `06-lesson-6.md` — Memory Files and Persistent Intelligence (Stage 3)
7. `07-lesson-7.md` — Tool Selection Framework (Stage 2-3 Hybrid)
8. `08-lesson-8.md` — Hands-On Debugging and Optimization (Stage 2)
9. `09-lesson-9.md` — Capstone: Spec-Driven Context Orchestration (Stage 4)

**Each lesson file includes**:
- Learning objectives (clear, measurable)
- Discovery sequence (experiment → observe → learn structure)
- Content sections (frameworks, scenarios, examples)
- Skills proficiency mapping (CEFR, Bloom's, DigComp)
- Single "Try With AI" closing section
- Session transcripts or examples demonstrating concepts
- No forbidden sections (no "What's Next", "Summary", etc.)

---

## Quality Validation Checkpoints

**Content-Implementer Checkpoints**:
1. ✅ Stage tags present in plan, natural language in student text
2. ✅ All cognitive load within B1 tier (7-10 concepts)
3. ✅ Three Roles explicitly demonstrated in Lessons 3, 4, 5
4. ✅ Capstone is specification-only (zero code)
5. ✅ Lesson endings are SINGLE "Try With AI" section
6. ✅ Discovery sequences evident (experiment before explanation)
7. ✅ Research foundation integrated with citations

**Technical-Reviewer Checkpoints**:
1. ✅ All code examples tested and verified
2. ✅ All claims cited from authoritative sources
3. ✅ Context window specs cite 2025 official docs
4. ✅ Session transcripts demonstrate Three Roles clearly
5. ✅ Lesson progression follows pedagogical arc
6. ✅ Anti-convergence from Chapter 10 validated
7. ✅ Acceptance tests (Test-003 through Test-014) all pass

---

## Success Metrics

**Chapter succeeds when**:
- ✅ 80%+ students diagnose degradation symptoms accurately (SC-001)
- ✅ 85%+ maintain context utilization <70% (SC-002)
- ✅ 90%+ demonstrate memory file persistence (SC-003)
- ✅ 80%+ select appropriate tools with reasoning (SC-004)
- ✅ 75%+ capstone specs are implementation-ready (SC-005)
- ✅ 85%+ demonstrate all Three Roles in transcripts (SC-007)
- ✅ All acceptance tests pass (Test-001 through Test-014)

---

**Plan Status**: Complete, Ready for Content Implementation
**Next Phase**: Content-Implementer executes lesson creation (Phase 4)
