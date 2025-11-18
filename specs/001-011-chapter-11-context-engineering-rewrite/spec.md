# Feature Specification: Chapter 11 Context Engineering Rewrite

**Feature Branch**: `001-011-chapter-11-context-engineering-rewrite`
**Created**: 2025-01-18
**Status**: Draft
**Input**: Rewrite Chapter 11 (Context Engineering for AI-Driven Development) through complete orchestration with deep research. Current chapter has substantial value BUT violates pedagogical methodology (bloated theory, doesn't follow 4-layer framework, missing CoLearning elements, cognitive overload for B1 tier).

**Chapter**: 11
**Part**: 3 (Markdown, Prompt & Context Engineering)
**Audience Tier**: B1 (Intermediate)
**Teaching Modality**: Hands-On Discovery (experiential learning)
**Prerequisite**: Chapter 10 (Prompt Engineering)

---

## Research Foundation

### Context Window Specifications (Verified 2025-01-18)

**Claude Sonnet 4.5:**
- **Standard context window**: 200K tokens (500 pages of text)
- **Extended context (beta)**: 1M tokens for tier 4+ organizations
- **Output capacity**: 64K tokens
- **Context awareness**: Model tracks remaining token budget throughout conversation
- **Source**: Anthropic Documentation, JuheAPI Guide (2025)

**Gemini 1.5 Pro:**
- **Standard context window**: 128K tokens
- **Extended context**: 2M tokens (1 hour video, 11 hours audio, 700K+ words, 30K+ lines of code)
- **Performance**: 99% accuracy in Needle in Haystack evaluation at 1M tokens
- **Source**: Google Developers Blog, Google Cloud Documentation (2025)

### Karpathy "LLM as CPU" Principle

**Core Analogy (Verified 2025-01-18):**
> "The LLM is the CPU, performing the core computation. The context window is the RAM, holding the working memory for a task. Context engineering is the delicate art and science of filling the context window with just the right information for the next step."

**Source**: Andrej Karpathy, "Software Is Changing (Again)" session at Y Combinator AI School

**Historical Context**: Karpathy positions LLMs as a new operating system where we interact through time-sharing centralized machines (OpenAI, Google, Anthropic) via text-based terminals—recreating the 1960s computing paradigm with modern AI.

### Anthropic Context Engineering Research

**Context Degradation Phenomenon ("Context Rot"):**
- Transformer architecture requires every token to attend to every other token (n² pairwise relationships)
- Models have less training experience with longer sequences
- Performance degrades gradually (not abrupt cliff) as context expands
- **Source**: Anthropic Engineering Blog - "Effective Context Engineering for AI Agents"

**Primary Mitigation Strategies (Anthropic-Validated):**

1. **Compaction**: Summarizing conversation history and reinitializing with compressed context
2. **Structured Note-Taking**: Persistent external memory (CLAUDE.md, NOTES.md) for tracking progress
3. **Sub-Agent Architectures**: Specialized agents with isolated contexts, returning condensed summaries (1K-2K tokens)

**Best Practices:**
- "Find the smallest possible set of high-signal tokens that maximize the likelihood of some desired outcome"
- Curate minimal, non-overlapping tool sets
- Implement "just-in-time" context strategies with dynamic retrieval
- Test minimal prompts first, add instructions based on observed failure modes

---

## User Scenarios & Testing

### User Story 1 - Context Window Foundation (Manual Exploration) (Priority: P1)

**As an** intermediate developer learning AI-driven development,
**I want to** manually track and observe context window usage in AI sessions,
**So that** I can build an intuitive understanding of degradation symptoms before learning mitigation strategies.

**Why this priority**: Without manual foundation (Stage 1), students cannot diagnose context issues or evaluate AI-suggested strategies. This is the prerequisite for all advanced context management.

**Independent Test**: Student can observe context window filling during a coding session, identify 3+ degradation symptoms (repetitive suggestions, forgotten patterns, performance drops) WITHOUT AI assistance, and explain when context needs compression vs isolation.

**Acceptance Scenarios**:

1. **Given** a fresh Claude Code session with 0% context utilization, **When** student incrementally loads project files and executes commands, **Then** student manually tracks token usage and observes the point at which AI suggestions become repetitive (degradation threshold identified)

2. **Given** a session at 80%+ context utilization showing degradation symptoms, **When** student reviews the session history, **Then** student identifies specific symptoms: repetitive code patterns, forgotten architectural decisions from earlier conversation, contradictory suggestions

3. **Given** two separate coding tasks (authentication + payment processing), **When** student attempts to mix both in single session vs separate sessions, **Then** student experiences context pollution firsthand and recognizes when isolation is needed

---

### User Story 2 - Progressive Context Loading (AI Collaboration) (Priority: P1)

**As an** intermediate developer working on large codebases,
**I want to** apply progressive loading strategies (Foundation → Current → On-Demand) with AI collaboration,
**So that** I can manage context effectively without overwhelming the AI with irrelevant files.

**Why this priority**: This is the most practical skill for daily development work. Developers immediately apply this to real projects. Demonstrates Three Roles (AI teaches loading patterns, student refines based on project specifics, convergence toward optimal strategy).

**Independent Test**: Given a codebase with 100+ files, student can independently apply progressive loading strategy: load only project structure + README (foundation), add files directly relevant to current task (current work), fetch additional context only when AI requests it (on-demand). AI stays focused, context window stays under 70%.

**Acceptance Scenarios**:

1. **Given** a new codebase and a feature request, **When** student works with AI to determine Foundation phase context, **Then** AI suggests loading project structure, core config files, and architecture docs (5-10 files), student evaluates relevance, they converge on minimal foundation set

2. **Given** foundation context loaded (30% context utilization), **When** student adds files for current feature implementation, **Then** AI teaches pattern: "For authentication feature, you need auth module + user model + middleware + tests (focused context)", student refines by adding project-specific files AI didn't know about

3. **Given** mid-development when AI needs additional context, **When** AI says "I need to see the database schema", **Then** student provides just-in-time context (fetches schema file dynamically), rather than pre-loading all database files upfront

---

### User Story 3 - Context Compression & Isolation (Advanced Techniques) (Priority: P2)

**As an** developer managing long AI sessions or complex multi-task projects,
**I want to** apply context compression (summarization, checkpoints) and context isolation (separate sessions),
**So that** I can reclaim context window space and prevent task interference.

**Why this priority**: Essential for professional work but builds on P1 foundations. Students need manual understanding (Story 1) and progressive loading (Story 2) before understanding when to compress vs isolate.

**Independent Test**: During a long coding session (90+ minutes, 85%+ context utilization), student can independently decide: compress via checkpoint summary + session restart, OR isolate by spawning separate session for new task. Student demonstrates decision framework: compress when continuing same task, isolate when switching contexts.

**Acceptance Scenarios**:

1. **Given** a session at 85% context utilization after 2 hours of feature development, **When** student observes degradation symptoms (AI forgetting earlier architectural decisions), **Then** student creates checkpoint summary (architectural decisions + progress + next steps in 500 tokens), restarts session with summary, continues work with reclaimed context space

2. **Given** mid-authentication-feature when urgent bug fix request arrives, **When** student evaluates mixing tasks vs isolation, **Then** student recognizes different contexts (authentication logic vs payment bug), creates new isolated session for bug fix, preserves original session for feature completion

3. **Given** multiple compression/isolation decisions during a week, **When** student reviews decision patterns, **Then** student articulates framework: compress for same-task continuity (90-min checkpoint rhythm), isolate for unrelated tasks (prevent context pollution)

---

### User Story 4 - Memory Files & Persistence (Stage 3 Intelligence Design) (Priority: P2)

**As an** developer working on projects spanning multiple days/weeks,
**I want to** create memory file templates (CLAUDE.md, architectural decision records) for persistent context,
**So that** AI remembers project patterns and decisions across sessions without re-explaining context.

**Why this priority**: Builds on compression/isolation skills (Story 3). Creates reusable intelligence that accumulates across projects.

**Independent Test**: Student can design and implement memory file system for a project: CLAUDE.md with project conventions, architecture.md with system design, decisions.md with ADRs. When starting new session days later, AI reads memory files and maintains context continuity (no re-explanation of project patterns needed).

**Acceptance Scenarios**:

1. **Given** a project with established architectural patterns (microservices, event-driven, specific tech stack), **When** student creates CLAUDE.md memory file, **Then** file contains: project structure, architectural principles, coding conventions, common patterns, and anti-patterns to avoid (500-1000 tokens)

2. **Given** memory file created in session 1, **When** student starts new session 3 days later with AI reading memory file, **Then** AI immediately understands project context (no re-explanation needed), suggests code following established patterns, references architectural decisions

3. **Given** evolution of architectural decisions during development, **When** student updates memory file with new decision record, **Then** future sessions reflect updated understanding, creating living documentation that accumulates intelligence

---

### User Story 5 - Tool Selection Framework (Claude Code vs Gemini CLI) (Priority: P3)

**As an** developer choosing AI development tools,
**I want to** understand decision criteria for Claude Code (200K context, deep reasoning) vs Gemini CLI (2M context, pattern analysis),
**So that** I can select the right tool for specific tasks (focused development vs large codebase exploration).

**Why this priority**: Practical tool selection knowledge, but depends on understanding context management fundamentals (Stories 1-4).

**Independent Test**: Given 3 scenarios (implement feature in 5-file module, refactor across 50-file codebase, explore unfamiliar 100K-line legacy system), student can independently select appropriate tool with reasoning: Claude Code for scenarios 1-2 (focused work, deep reasoning needed), Gemini CLI for scenario 3 (massive context window for exploration).

**Acceptance Scenarios**:

1. **Given** feature implementation task in focused module (5-10 files, 10K tokens), **When** student evaluates tool options, **Then** student selects Claude Code (reasoning: focused context fits 200K window, deep code generation needed, no exploration phase)

2. **Given** large-scale refactoring across 50-file codebase (pattern: update all API endpoints), **When** student evaluates tool options, **Then** student weighs tradeoffs: Claude Code with progressive loading (pros: precise control, selective context) vs Gemini CLI with full codebase load (pros: sees all patterns at once, 2M context sufficient), student justifies choice

3. **Given** legacy system exploration (100K lines, unfamiliar patterns, need to understand architecture), **When** student evaluates tool options, **Then** student selects Gemini CLI (reasoning: 2M context allows loading entire codebase, pattern analysis across files, exploration without progressive loading complexity)

---

### User Story 6 - Capstone: Spec-Driven Context Orchestration (Stage 4) (Priority: P3)

**As an** developer demonstrating context management mastery,
**I want to** write a complete specification for a context-aware CLI tool that orchestrates memory files + progressive loading + multi-agent patterns,
**So that** I can demonstrate specification-first thinking and accumulated intelligence composition WITHOUT implementing code.

**Why this priority**: Capstone project synthesizing all prior learning. Demonstrates spec-driven development methodology (specification mastery, not implementation skills).

**Independent Test**: Student independently writes comprehensive spec.md for context-aware development tool including: intent (WHAT tool does), success criteria (measurable outcomes), functional requirements (context management capabilities), memory file architecture (persistent intelligence), progressive loading algorithm (foundation → current → on-demand), and multi-agent orchestration (when to spawn specialized agents). Spec is complete enough that any developer could implement from it. NO CODE WRITTEN.

**Acceptance Scenarios**:

1. **Given** accumulated context management knowledge from Lessons 1-8, **When** student designs tool specification, **Then** spec includes: clear intent ("context-aware CLI tool for managing AI development sessions"), measurable success criteria (context window utilization <75%, session continuity across days, zero manual re-explanation), and system behavior WITHOUT implementation details

2. **Given** memory file patterns learned in Story 4, **When** student specifies memory system for tool, **Then** spec defines: memory file schema (project conventions, architectural decisions, coding patterns), persistence strategy (when to read/write/update), and context inheritance (how new sessions load accumulated intelligence)

3. **Given** progressive loading learned in Story 2 and compression/isolation learned in Story 3, **When** student specifies context orchestration algorithm, **Then** spec defines: foundation phase logic (what files to auto-load), current work detection (how tool identifies relevant files for task), on-demand fetching (when to dynamically load additional context), compression triggers (degradation symptoms → checkpoint), and isolation rules (task similarity scoring → new session)

4. **Given** complete spec written, **When** peer developer reviews spec, **Then** reviewer can understand tool's complete behavior, implement system from spec alone, and verify implementation against spec's acceptance tests (demonstrating specification quality)

---

### Edge Cases

- **What happens when** context window is at 95% utilization and no clear compression points exist (all content essential)?
  → Student recognizes hard limit, must archive least-recently-used context to external memory file, continue with reduced context (graceful degradation)

- **How does system handle** mixing context types (code + documentation + conversation history) approaching token limit?
  → Student prioritizes by task relevance: code > recent conversation > documentation, compresses documentation into summary (50% reduction), archives old conversation

- **What if** memory file contains outdated architectural decisions that conflict with current refactoring?
  → Student updates memory file with decision record (OLD: monolith, NEW: microservices, REASON: scalability), AI reads both old and new context, suggests migration path

- **How to manage** rapid task-switching (5+ unrelated tasks in 2-hour session)?
  → Student recognizes isolation pattern: create dedicated session per task (5 parallel sessions), use memory files for shared project context, final consolidation session synthesizes cross-task learning

- **What when** progressive loading strategy fails (AI repeatedly requests files, never satisfied)?
  → Student diagnoses root cause: task scope too broad, break into smaller focused tasks, or switch to Gemini CLI for full-codebase loading approach

---

## Requirements

### Functional Requirements

**FR-001: Context Window Mechanics (Stage 1 Foundation)**
- Chapter MUST provide hands-on exercises where students manually track context window usage (observe token counting, identify degradation symptoms through experimentation)
- Chapter MUST demonstrate degradation symptoms through experiential learning (students FEEL repetitive suggestions, OBSERVE forgotten patterns, not just read definitions)
- Chapter MUST teach monitoring strategies through discovery (students determine compression/isolation/restart thresholds by experimenting with overloaded sessions)

**FR-002: Six Components AIDD Context Framework (Preserved from Existing)**
- Chapter MUST present complete framework: Model Selection (Claude vs Gemini vs others), Development Tools (CLI vs IDE extensions), Knowledge & Memory (memory files, persistent notes), Guardrails (validation rules, safety checks), Code Context (file selection strategies), Orchestration (single vs multi-agent workflows)
- Chapter MUST integrate framework throughout lessons (not isolated theory section), demonstrating how components interact during real development workflows

**FR-003: Progressive Context Loading (Stage 2 - Three Roles Demonstrated)**
- Chapter MUST teach three-phase loading strategy: Foundation (project structure, core configs), Current Work (task-specific files), On-Demand (just-in-time fetching)
- Chapter MUST demonstrate THREE ROLES in at least one lesson: AI as Teacher (suggests loading pattern student didn't know), AI as Student (student refines AI's context-blind suggestion with project-specific knowledge), AI as Co-Worker (iterate together toward optimal loading strategy for THIS codebase)
- Chapter MUST provide decision framework for what to load when (not arbitrary rules, but reasoning about task requirements vs context budget)

**FR-004: Context Compression & Isolation Techniques**
- Chapter MUST teach compression through hands-on practice: creating checkpoint summaries (architectural decisions + progress + next steps in 500 tokens), restarting sessions with compressed context, measuring reclaimed space
- Chapter MUST teach isolation through experiential scenarios: mixing unrelated tasks in single session (observe pollution), separating into isolated sessions (observe clarity improvement), decision framework (compress vs isolate based on task similarity)

**FR-005: Memory Files & Persistence (Stage 3 Intelligence Design)**
- Chapter MUST guide students in creating reusable memory file templates: CLAUDE.md (project conventions), architecture.md (system design), decisions.md (architectural decision records)
- Chapter MUST demonstrate persistence across sessions: student creates memory file in session 1, starts fresh session days later with AI reading memory file, AI maintains context continuity without re-explanation
- Chapter MUST encode pattern as reusable skill: memory file structure, persistence strategy, update triggers (when to refresh memory files)

**FR-006: Tool Selection Framework (Claude Code vs Gemini CLI)**
- Chapter MUST provide decision criteria with validated specifications: Claude Code (200K context, 64K output, deep reasoning for focused development), Gemini CLI (2M context, pattern analysis for large codebase exploration)
- Chapter MUST preserve comparison table with verified 2025 specifications (context window sizes, file selection strategies, memory persistence patterns)
- Chapter MUST teach selection reasoning through scenarios: focused feature implementation → Claude Code, large-scale refactoring → evaluate tradeoffs, legacy system exploration → Gemini CLI

**FR-007: Capstone Spec-Driven Project (Stage 4 - NO Implementation)**
- Chapter MUST guide students in writing COMPLETE specification for context-aware CLI tool
- Capstone spec MUST orchestrate accumulated intelligence: memory files (FR-005), progressive loading (FR-003), compression/isolation (FR-004), multi-agent patterns (FR-002 orchestration component)
- Capstone MUST be specification-only (NO code implementation, demonstrating spec-first methodology where spec quality enables AI implementation)

**FR-008: Factual Accuracy Verification**
- All context window specifications MUST cite verified sources (Claude Sonnet 4.5: 200K standard/1M extended from Anthropic docs, Gemini 1.5 Pro: 2M from Google Cloud docs)
- Karpathy "LLM as CPU" principle MUST cite authoritative source (Y Combinator AI School session, verified via web search 2025-01-18)
- Anthropic context degradation patterns MUST cite source (Anthropic Engineering Blog: "Effective Context Engineering for AI Agents")
- All CLI commands and code examples MUST be tested and verified (no hallucinated commands)

**FR-009: Anti-Convergence from Chapter 10**
- Chapter 10 used DIRECT TEACHING modality (explain → demonstrate → practice)
- Chapter 11 MUST use HANDS-ON DISCOVERY modality (experiment → observe → learn from experience)
- Lessons MUST vary approach: Students discover degradation symptoms through overload exercises (not explained upfront), students experiment with compression vs isolation (learning by doing), students build intuition before receiving frameworks

**FR-010: Cognitive Load Management (B1 Tier)**
- Every lesson section MUST limit to 7-10 concepts maximum (B1 intermediate tier)
- Complex concepts MUST chunk into related groups (context window mechanics + degradation + monitoring = 1 chunk, not 3 separate concepts)
- Options presentation MUST limit to 3-4 choices with selection criteria (not 8+ overwhelming options)
- Progressive disclosure MUST apply: simple context strategies (progressive loading) introduced before advanced patterns (multi-agent orchestration)

**FR-011: Lesson Structure Compliance**
- Every lesson MUST end with SINGLE closing section: "Try With AI" (demonstrates concepts with AI collaboration)
- Lessons MUST NOT include: "What's Next" sections (navigation clutter), "Key Takeaways" sections (redundant summary), "Summary" sections (duplicates learning objectives), standalone "Safety Note" sections (safety integrated into "Try With AI" as 1-2 contextual sentences)

**FR-012: Stage Progression Documentation (Internal Planning, Not Student-Facing)**
- Plan.md MUST tag each lesson with stage (Stage 1: Manual Foundation, Stage 2: AI Collaboration, Stage 3: Intelligence Design, Stage 4: Spec-Driven Capstone)
- Student-facing lesson text MUST use natural language (NOT "Stage 2 Focus: ..." headers)
- Stage labels are PLANNING TOOLS for content creators, NOT vocabulary for students

### Key Entities

**Context Window:**
- Represents the AI model's working memory capacity measured in tokens
- Key attributes: size (Claude: 200K standard, Gemini: 2M), utilization percentage (0-100%), degradation threshold (typically 80%+)
- Relationships: Contains conversation history, loaded files, system prompts, tool outputs

**Memory File:**
- Represents persistent project intelligence stored externally to context window
- Key attributes: file path (CLAUDE.md, architecture.md, decisions.md), content (project conventions, architectural patterns, decision records), persistence strategy (when to read/write/update)
- Relationships: Loaded into context window at session start, updated during development, accumulated across sessions

**AI Session:**
- Represents a single conversation thread with an AI development tool (Claude Code or Gemini CLI)
- Key attributes: session ID, start time, context utilization over time, task focus (authentication feature, bug fix, refactoring), degradation indicators (repetitive suggestions count, forgotten patterns count)
- Relationships: Contains context window, references memory files, may spawn sub-sessions (multi-agent orchestration)

**Progressive Loading Phase:**
- Represents a stage in context loading strategy (Foundation, Current Work, On-Demand)
- Key attributes: phase name, files loaded, token count, loading trigger (session start, task switch, AI request)
- Relationships: Phases sequence (Foundation → Current → On-Demand), each phase adds to context window

**Compression Checkpoint:**
- Represents a summarized state of long AI session for context reclamation
- Key attributes: checkpoint ID, timestamp, summary text (500-1000 tokens), architectural decisions preserved, progress record, next steps
- Relationships: Created when context hits 85%+ utilization, used to restart session with reduced context, archival of old checkpoint history

---

## Success Criteria

### Measurable Outcomes

**SC-001: Context Degradation Diagnosis Skill**
Students can independently identify 5+ degradation symptoms in failing AI session (repetitive code suggestions, forgotten architectural decisions, performance slowdown, contradictory suggestions, inability to reference earlier conversation) and explain which symptom indicates compression need vs isolation need.
**Measurement**: Quiz with session transcripts showing degradation symptoms, students identify symptoms and prescribe mitigation (target: 80%+ accuracy).

**SC-002: Progressive Loading Application**
Students can apply progressive loading strategy to new codebase (100+ files) and maintain context window utilization under 70% throughout feature implementation while keeping AI focused and effective.
**Measurement**: Hands-on exercise where students implement feature in provided codebase, telemetry tracks context utilization over time (target: 85%+ of students stay under 70% utilization).

**SC-003: Memory File System Design**
Students can independently design and create memory file system for project including CLAUDE.md (project conventions), architecture.md (system design), decisions.md (ADRs), and demonstrate persistence across sessions (AI reads memory files in new session, maintains context continuity without re-explanation).
**Measurement**: Students submit memory file artifacts + demonstrate 2nd session where AI remembers patterns (target: 90%+ successfully demonstrate persistence).

**SC-004: Tool Selection Decision Framework**
Students can evaluate 3 scenarios (focused feature implementation, large-scale refactoring, legacy system exploration) and correctly select appropriate tool (Claude Code vs Gemini CLI) with justified reasoning based on context requirements, task complexity, and tool capabilities.
**Measurement**: Scenario-based assessment, students provide tool selection + reasoning (target: 80%+ select correctly with valid justification).

**SC-005: Specification Quality (Capstone)**
Students can write complete spec.md for context-aware tool that is implementation-ready (peer developer can build system from spec alone, spec includes measurable success criteria, functional requirements are testable, NO implementation details like "use Redis" or "implement with TypeScript").
**Measurement**: Peer review of capstone specs using rubric (intent clarity, success criteria measurability, requirement testability, absence of implementation details), (target: 75%+ specs rated "implementation-ready").

**SC-006: Context Window Efficiency**
After chapter completion, students working on real projects demonstrate improved context management: average context utilization stays under 75% (vs 90%+ pre-chapter), fewer session restarts due to confusion (50% reduction), AI generates code fitting existing patterns on first attempt (70%+ success rate vs 40% pre-chapter).
**Measurement**: Post-chapter telemetry from students' Claude Code/Gemini CLI sessions over 2-week period (target metrics: <75% utilization, 50% fewer restarts, 70%+ first-attempt quality).

**SC-007: Three Roles Demonstration**
In Stage 2 lessons, students demonstrate all three roles in collaboration: AI teaches student (AI suggests pattern student didn't know, student reports learning), Student teaches AI (student corrects AI's assumption with project-specific knowledge), Co-Worker convergence (student and AI iterate toward solution neither had initially).
**Measurement**: Students submit annotated session transcripts highlighting three roles, graded for role clarity (target: 85%+ submissions demonstrate all three roles).

---

## Constraints

**Audience Tier Constraint**:
- B1 Intermediate tier cognitive load limits apply: 7-10 concepts per section maximum
- Moderate scaffolding required (provide frameworks with 3-4 options and selection criteria, not 8+ overwhelming choices)
- Cannot assume advanced prerequisites (students know basic AI interaction from Chapter 10, but NOT advanced prompt engineering or multi-agent orchestration)

**Teaching Modality Constraint**:
- MUST use hands-on discovery (experiential learning) throughout chapter
- CANNOT use direct teaching/lecture style (violates anti-convergence requirement from Chapter 10)
- Students MUST discover patterns through exercises BEFORE receiving explanatory frameworks

**Stage Progression Constraint**:
- All 4 stages MUST be represented across lessons: Stage 1 (manual foundation, no AI), Stage 2 (AI collaboration with Three Roles), Stage 3 (intelligence design creating reusable skills), Stage 4 (spec-driven capstone)
- Stage 2 lessons MUST demonstrate all THREE ROLES: AI as Teacher, AI as Student, AI as Co-Worker (not just "use AI to help")
- Stage 4 capstone MUST be specification-only (NO implementation code)

**Factual Accuracy Constraint**:
- All technical specifications MUST cite verified sources (Anthropic docs, Google Cloud docs, Karpathy references)
- Context window sizes MUST use 2025-verified values: Claude Sonnet 4.5 (200K standard, 1M extended), Gemini 1.5 Pro (2M)
- No hallucinated CLI commands or API endpoints (all examples must be tested or cited from official documentation)

**Minimal Content Constraint**:
- Every section MUST map to specific learning objective (if section doesn't serve objective, remove it)
- Lesson endings MUST have SINGLE closing section: "Try With AI" (no "What's Next", "Key Takeaways", "Summary", standalone "Safety Note")
- Safety reminders MUST integrate into "Try With AI" as 1-2 contextual sentences (not standalone sections)

**Preservation Constraint**:
- MUST preserve valuable frameworks from existing chapter: Six Components of AIDD Context, Progressive Loading (Foundation → Current → On-Demand), Comparison Table (Claude Code vs Gemini CLI with verified specs), Memory File Patterns
- CANNOT discard existing chapter entirely (substantial value exists, transformation required not replacement)

**Capstone Scope Constraint**:
- Capstone project MUST be complete tool specification (NOT "focused feature addition")
- Specification MUST demonstrate orchestration of accumulated intelligence (memory files + progressive loading + compression/isolation + multi-agent patterns)
- Constraint: NO programming implementation (spec-only, demonstrating specification mastery per user requirement "no programming")

---

## Non-Goals

**Low-Level LLM Architecture:**
Transformer internals, attention mechanisms, token embeddings, model training processes are OUT OF SCOPE. Students do not need to understand how LLMs process tokens to effectively manage context. This belongs in advanced AI/ML courses, not AIDD methodology.

**Custom Model Fine-Tuning for Context Optimization:**
Fine-tuning LLMs for better context handling, adjusting model parameters, custom training on project-specific data are EXCLUDED. These are advanced topics for Part 8 (Turing LLMOps). B1 tier focuses on using existing models effectively, not customizing models.

**Multi-Model Orchestration Frameworks (LangChain, AutoGen):**
Framework-specific implementations, LangChain agents, AutoGen conversation patterns are NOT COVERED in Chapter 11. Part 6 (AI Native Software Development) covers orchestration frameworks. Chapter 11 teaches universal context engineering principles applicable across tools.

**Production Monitoring Infrastructure:**
Setting up observability pipelines, context usage dashboards, alerting systems for context degradation in production AI systems are OUT OF SCOPE. Part 7 (Cloud Native Development) covers production observability. Chapter 11 focuses on developer-facing context management during development.

**Security & Safety Guardrails Implementation:**
Implementing content filtering, prompt injection defenses, output validation systems, compliance monitoring for AI context are EXCLUDED. Part 7 Chapter 61 covers security/safety/governance. Chapter 11 assumes students work in safe development environments.

**Programming Language-Specific Context Patterns:**
Python-specific memory management, JavaScript-specific async context handling, language-specific debugging patterns are NOT PRIMARY FOCUS. While examples may use Python/TypeScript (book's languages), Chapter 11 teaches universal context engineering applicable to any language.

**CLI Tool Implementation:**
Building actual CLI tool from capstone specification, writing code, testing implementation, deployment are NOT PART OF CHAPTER. Capstone demonstrates specification mastery (spec-first methodology: we specify, AI implements). Implementation is AI's job, not learning objective.

**Why These Are Excluded:**
- Low-level LLM architecture: Prerequisite knowledge not assumed for B1 tier, tangential to AIDD methodology
- Fine-tuning: Advanced topic for later part, violates "use existing tools effectively" focus
- Orchestration frameworks: Covered in dedicated Part 6 chapters, framework-specific vs universal principles
- Production monitoring: Infrastructure concern for Part 7, not development workflow focus
- Security guardrails: Complex topic deserving dedicated chapter (Part 7 Chapter 61)
- Language-specific patterns: Chapter teaches universal principles, not language-locked techniques
- CLI implementation: Violates spec-first methodology and "no programming" capstone constraint

---

## Learning Objectives

By the end of Chapter 11, students will:

**LO-001: Diagnose Context Degradation**
Identify and explain 5+ degradation symptoms (repetitive suggestions, forgotten patterns, performance drops, contradictory advice, inability to reference earlier context) in failing AI sessions, and prescribe appropriate mitigation (compression checkpoint, session isolation, context restart) based on symptoms.

**LO-002: Apply Progressive Loading Strategy**
Independently apply three-phase progressive loading to new codebase: Foundation phase (project structure, core configs loaded at session start), Current Work phase (task-specific files added when implementing feature), On-Demand phase (additional context fetched just-in-time when AI requests it), maintaining context utilization under 75%.

**LO-003: Design Memory File Systems**
Create reusable memory file templates for project persistence including CLAUDE.md (project conventions, coding patterns), architecture.md (system design, component relationships), decisions.md (architectural decision records with rationale), and demonstrate cross-session persistence (AI reads memory files in new session, maintains context continuity).

**LO-004: Choose Appropriate AI Tool**
Evaluate development scenarios (focused feature work, large-scale refactoring, legacy system exploration) and select Claude Code (200K context, deep reasoning) vs Gemini CLI (2M context, pattern analysis) based on decision framework: context requirements + task complexity + tool capabilities, with justified reasoning.

**LO-005: Create Reusable Context Skills**
Design and encode recurring context management patterns as reusable skills: memory file templates (structure + persistence strategy + update triggers), compression workflow (checkpoint creation + session restart + context reclamation), isolation decision framework (task similarity scoring + session spawning rules).

**LO-006: Write Context-Aware Specifications**
Compose complete, implementation-ready specification (spec.md) for context-aware development tool demonstrating accumulated intelligence orchestration: memory file architecture + progressive loading algorithm + compression/isolation rules + multi-agent coordination, WITHOUT implementation details (spec-first methodology: clear intent, measurable success criteria, testable functional requirements).

---

## Pedagogical Structure

**Foundation Phase (Lessons 1-2):**
Context windows, degradation symptoms, manual tracking exercises. Students build intuitive understanding through hands-on experimentation BEFORE learning mitigation frameworks.

**Application Phase (Lessons 3-5):**
Progressive loading (Foundation → Current → On-Demand), context compression (checkpoint summaries, session restart), context isolation (separate sessions for unrelated tasks). Three Roles demonstrated in these lessons: AI as Teacher (suggests loading patterns), AI as Student (learns project-specific constraints), AI as Co-Worker (iterates toward optimal strategy).

**Integration Phase (Lessons 6-7):**
Memory files (persistent intelligence across sessions), tool comparison (Claude Code vs Gemini CLI decision framework), advanced strategies (just-in-time fetching, sub-agent architectures, hybrid approaches).

**Validation Phase (Lesson 8):**
Validate understanding through hands-on scenarios (debugging failing sessions, optimizing context usage, diagnosing and fixing common mistakes). Discovery-based learning: students identify issues through practice, not checklists.

**Mastery Phase (Lesson 9):**
Capstone spec-driven project: Write complete specification for context-aware CLI tool orchestrating all accumulated patterns (memory files + progressive loading + compression/isolation + multi-agent coordination). Specification-only, NO implementation (demonstrates spec mastery, composition of intelligence).

**Estimated Lesson Count: 9 lessons**
**Justification**: 7 major concepts (context windows, progressive loading, compression, isolation, memory files, tool selection, spec-driven orchestration) × B1 moderate complexity requiring hands-on discovery (not quick lecture) = 9 lessons to maintain cognitive load limits (7-10 concepts per lesson) while providing sufficient practice time.

---

## Assumptions

1. **Students completed Chapter 10 (Prompt Engineering):** Assumed prerequisite knowledge includes basic prompt crafting, understanding of AI responses, familiarity with conversational AI interaction. Chapter 11 builds on this: prompts operate WITHIN context, context quality determines prompt effectiveness.

2. **Students have access to Claude Code OR Gemini CLI:** Chapter requires hands-on experimentation with AI development tools. Students need active Claude Code or Gemini CLI accounts to complete exercises. Alternative: provide recorded session transcripts for students without tool access (fallback for accessibility, but hands-on preferred).

3. **Students work on codebases with 50+ files:** Context engineering concepts are most meaningful in non-trivial projects. Assumptions: students have access to open-source projects or can use provided sample codebases (prepared by course maintainers) with realistic complexity.

4. **Default to industry-standard practices when unspecified:** Memory file formats follow established conventions (Markdown for CLAUDE.md, YAML frontmatter for metadata). Compression checkpoint summaries use structured format (Decisions, Progress, Next Steps). Session isolation uses separate terminal tabs/windows (simple, universally available).

5. **Context utilization telemetry is observable:** Assumed: Claude Code and Gemini CLI provide some indication of context usage (token counts, context window percentage, or observable performance changes). If tools lack telemetry, students estimate utilization through indirect signals (response latency, output quality, repetition patterns).

6. **Hands-on discovery exercises are self-contained:** Each exercise provides necessary setup (sample project, initial context, task description). Students can complete exercises independently without instructor live demonstration. This supports asynchronous learning and course scalability.

7. **Comparison table accuracy maintained through 2025:** Context window specifications verified as of 2025-01-18. Assumption: specifications remain stable through 2025 (reasonable for established models). Maintenance plan: annual verification and table updates if specifications change.

---

## Acceptance Tests

- [ ] **Test-001: All research verified (2025-01-18)**
  Context window specifications cited from Anthropic docs (Claude Sonnet 4.5: 200K standard, 1M extended), Google Cloud docs (Gemini 1.5 Pro: 2M), Karpathy "LLM as CPU" principle cited from Y Combinator AI School session, Anthropic context degradation patterns cited from Engineering Blog. NO uncited claims.

- [ ] **Test-002: Comparison table updated**
  Claude Code vs Gemini CLI comparison table includes verified 2025 specifications: context window sizes (Claude: 200K/1M, Gemini: 2M), output capacities (Claude: 64K), context awareness features, file selection strategies, memory persistence patterns. Source citations included.

- [ ] **Test-003: Three Roles demonstrated in 3+ lessons**
  At least 3 Stage 2 lessons explicitly show: AI as Teacher (AI suggests pattern student didn't know, student learns something new), AI as Student (student corrects AI's assumption with project-specific knowledge), AI as Co-Worker (student and AI iterate toward solution neither had initially, convergence demonstrated).

- [ ] **Test-004: Stage progression explicit in plan, natural in student text**
  Plan.md tags lessons with stages (Stage 1, 2, 3, 4). Student-facing lesson markdown files use natural language (NO "Stage 2 Focus:" headers, NO "Layer 2" terminology, NO "Three Roles Framework" section headers). Stages are planning tools, not student vocabulary.

- [ ] **Test-005: Hands-on discovery modality evidenced**
  Lesson structure follows experiment → observe → learn pattern. Students discover degradation symptoms through overload exercises BEFORE reading definitions. Students experiment with compression vs isolation BEFORE receiving decision framework. Discovery precedes explanation in all lessons.

- [ ] **Test-006: Lesson endings follow minimal content protocol**
  Every lesson file ends with SINGLE closing section: "## Try With AI". NO forbidden sections present: "## What's Next", "## Key Takeaways", "## Summary", "## Safety Note" (standalone). Safety reminders appear INSIDE "Try With AI" as 1-2 contextual sentences.

- [ ] **Test-007: Cognitive load managed (B1 tier)**
  Every lesson section contains 7-10 concepts maximum. Related concepts chunked together (context window mechanics + degradation + monitoring = 1 chunk). Options limited to 3-4 with selection criteria (not 8+ overwhelming choices). Progressive disclosure: simple concepts (progressive loading) before complex (multi-agent orchestration).

- [ ] **Test-008: Capstone is spec-only (NO implementation)**
  Lesson 9 capstone guides students in writing complete spec.md for context-aware tool. Spec includes: intent (WHAT tool does), success criteria (measurable outcomes), functional requirements (context management capabilities), memory architecture, progressive loading algorithm, multi-agent orchestration rules. ZERO lines of implementation code (Python, TypeScript, shell scripts). Spec demonstrates specification mastery.

- [ ] **Test-009: Non-goals explicitly listed in spec**
  Specification includes complete "Non-Goals" section listing excluded topics: LLM architecture internals, custom model fine-tuning, multi-model orchestration frameworks (LangChain/AutoGen), production monitoring infrastructure, security guardrails implementation, language-specific patterns, CLI tool implementation. Each non-goal includes exclusion rationale.

- [ ] **Test-010: Success criteria measurable and falsifiable**
  All success criteria (SC-001 through SC-007) include specific measurement method: quiz accuracy percentages, telemetry targets, peer review rubrics, session transcript analysis. Each criterion is technology-agnostic (no "Redis cache hit rate"), user/outcome-focused (not implementation details), objectively verifiable.

- [ ] **Test-011: All CLI commands tested**
  If lessons include CLI commands (claude-code init, git commands, file operations), commands must be executed in test environment, output verified correct. Test logs attached to implementation artifacts. NO hallucinated commands.

- [ ] **Test-012: Six Components framework preserved**
  Framework appears across multiple lessons (not isolated theory section): Model Selection (Claude vs Gemini), Development Tools (CLI vs IDE), Knowledge & Memory (memory files), Guardrails (validation rules), Code Context (file selection), Orchestration (single vs multi-agent). Demonstrates component interactions during real workflows.

- [ ] **Test-013: Progressive loading demonstrated with Three Roles**
  At least one lesson shows progressive loading with complete Three Roles cycle: AI suggests loading strategy (Teacher), student refines with project-specific knowledge (Student teaches AI), they iterate toward optimal strategy for THIS codebase (Co-Worker convergence). Full cycle documented in lesson transcript/example.

- [ ] **Test-014: Anti-convergence from Chapter 10 validated**
  Chapter 10 teaching modality documented (direct teaching: explain → demonstrate → practice). Chapter 11 uses different modality (hands-on discovery: experiment → observe → learn). Evidence: lessons start with experiments/exercises, explanations follow discoveries, students learn patterns through practice before receiving frameworks.
