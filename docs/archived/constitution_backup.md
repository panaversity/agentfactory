<!--
Constitution Evolution Log:

v5.0.0 (MAJOR — Framework Consolidation) — 2025-01-16
Rationale: Eliminate framework collision and terminology confusion identified by user
WHAT CHANGED:
- Section II: Simplified from 135 lines (5-layer VI stack) to 56 lines (3 forcing functions for context accumulation)
  - Removed: Implementation details (Intelligence Object schema, layer-by-layer architecture)
  - Kept: Governance rules (constitutional context, comprehensive research, context handoff)
  - Rationale: VI architecture was agent workflow implementation, not educational governance
- Section IIa: Consolidated teaching frameworks into single "AI-Native Teaching Framework (4-Stage Progression)"
  - Merged: Panaversity 4-Layer Method + Graduated Teaching Pattern + AI Three Roles Framework
  - Result: Stage 1 (Manual Foundation) + Stage 2 (AI Collaboration with Three Roles) + Stage 3 (Intelligence Design) + Stage 4 (Spec-Driven Integration)
  - Eliminated: Terminology collision between "Layer" (Panaversity) and "Tier" (Graduated Teaching)
- Section IIb: Removed as standalone section
  - Integrated: AI Three Roles (Teacher/Student/Co-Worker) into Stage 2 of 4-Stage Framework
  - Rationale: Three Roles describe HOW AI collaboration works in Stage 2, not separate concept
- Principles: Renumbered 8 → 7
  - Removed: Principle 2 (Graduated Teaching) — now fully covered by 4-Stage Framework
  - Promoted: Old Principle 5 (Progressive Complexity/CEFR) became new Principle 2
  - Result: Principle 1 (Specification Primacy), 2 (Progressive Complexity), 3 (Coherent Structure), 4 (Intelligence Accumulation), 5 (Anti-Convergence), 6 (Minimal Content), 7 (Factual Accuracy)
SIZE: ~950 lines (down from 1076, 12% reduction through consolidation)
Migration Impact: BREAKING CHANGE — All agents/skills must update terminology
  - "Panaversity Layer 1-4" → "Stage 1-4"
  - "Graduated Teaching Tier 1-3" → "Stage 1, 2, 4"
  - "Section IIb (Three Roles)" → "Stage 2 (AI Collaboration)"
  - "Principle 2 (Graduated Teaching)" → "Section IIa (4-Stage Framework)"
  - "Principle 5 (Progressive Complexity)" → "Principle 2"
Migration Guide: See ADR in history/adr/

v4.0.1 (PATCH) — 2025-01-16 Evening
Rationale: Critical gaps identified by user — Added missing pedagogical foundations
WHAT ADDED:
- Preamble: Book purpose, title, audience (Gap 1)
- Section IIa: Panaversity 4-Layer Teaching Method (Gap 2 — from From-Reusable-Code-to-Reusable-Intelligence.md)
- Section IIb: AI Three Roles Framework (Gap 3 — from presentation_ai_driven_ai_native_development_complete.md)
- P4 Flexibility: Replaced hardcoded "9 lessons" with pedagogical progression principles (Gap 4)
- P5 Flexibility: Added research grounding (CEFR, Miller's Law), flexibility notes
SIZE: 1076 lines (grew from 846 due to critical pedagogical additions, still 32% less than v3.1.3)
Migration: Non-breaking additions, strengthens pedagogical foundation

v4.0.0 (BREAKING CHANGE) — 2025-01-16
Rationale: Paradigm shift from "comprehensive guide" to "production-grade governance"
WHAT CHANGED:
- Scope narrowed: Book constitution ONLY (eliminated infrastructure/deployment governance)
- Vertical Intelligence architecture EMBEDDED (not referenced externally)
- Forcing functions replace anti-patterns.md (automatic enforcement, not manual maintenance)
- Vocabulary expansion over rule accumulation (conceptual tools, not rigid checklists)
- Negative space precision (NEVER statements with falsifiable criteria)
- From Reusable Code → Reusable Intelligence paradigm integrated
WHAT ELIMINATED:
- Infrastructure standards (Docker, Kubernetes → NOT book governance concern)
- Deployment patterns (moved to separate project constitution if needed)
- Technology stack details (delegated to CLAUDE.md)
- Anti-patterns.md file (unsustainable, replaced with forcing functions)
- External VI paper references (architecture embedded in Section II)
- Implementation details (delegated to output-styles, skills)
SIZE REDUCTION: 1580 lines → 846 lines (46% reduction through delegation)
Migration Impact:
- All existing principles preserved (18 → 8 consolidated, no loss of governance)
- Agents must adapt to embedded VI architecture (Layer 1-5 in constitution)
- Forcing functions encoded in agent logic (not separate anti-patterns file)
- Book content validated against narrower scope (educational quality only)

See: PHR in history/prompts/constitution/ for design process documentation
-->

# AI Native Software Development Book — Constitution

**Version:** 5.0.0 (MAJOR — Framework Consolidation & Simplification)
**Ratified:** 2025-01-16
**Last Amended:** 2025-01-16 (Late Evening — Framework Consolidation)
**Scope:** Educational content governance (book chapters, lessons, exercises)
**Audience:** AI Super Orchestrator, Subagents (chapter-planner, content-implementer, validation-auditor)

---

## Preamble: What This Book Is

**Title**: *AI Native Software Development: CoLearning Agentic AI with Python and TypeScript – The AI & Spec Driven Way*

**Purpose**: This is a technical book teaching AI-native software development methodology where specification-writing is the primary skill and AI agents handle implementation.

**Target Audience**:
- **Complete Beginners**: Those entering software development for the first time in the agentic era
- **Traditional Developers**: Experienced coders transitioning from code-centric to AI-native workflows
- **AI-Curious Professionals**: Anyone seeking to understand how AI agents transform software creation

**Why This Matters**: In the agentic era, barriers that kept people out of programming for 50 years (memorizing syntax, debugging cryptic errors, environment configuration) are dissolving. AI handles mechanical tasks while humans focus on problem-solving and system design. This is the **best time in decades** to learn software development—not despite AI, but because of it.

**Core Thesis**: In the agentic era, reusable intelligence (specifications, agent architectures, skills) replaces reusable code as the primary artifact of software development.

---

## I. The Paradigm Shift: From Reusable Code to Reusable Intelligence

### The Fundamental Transformation

**Old World:** Code libraries were the units of reuse. Developers shared functions, classes, frameworks.

**New World:** Specifications, Agent Architectures, and Skills are the units of reuse. Developers share intelligence.

**What This Book Teaches:**

This book does NOT teach students to write code faster. This book teaches students to **design reusable intelligence** that accumulates with every project:

1. **Specifications** → Capture intent with precision (executable contracts, not documentation)
2. **Agent Architectures** → Encode domain expertise (subagents that apply accumulated learnings)
3. **Skills** → Compound organizational capability (reusable pedagogical and technical patterns)

### "Specs Are the New Syntax"

In traditional programming, the primary skill was **mastering syntax**—memorizing language constructs and typing implementations manually.

In AI-native development, the primary skill is **mastering specifications**—articulating intent so clearly that AI agents execute flawlessly.

**The Paradigm Shift:**
- **Old:** Your value = how fast you type correct syntax
- **New:** Your value = how clearly you articulate requirements
- **Bottom line:** Specification quality determines output quality

Just as developers once studied language reference manuals to write code, AI-native developers study specification patterns to direct intelligent agents.

**This isn't a productivity hack—it's a fundamental transformation of what "programming" means in the agentic era.**

### Core Forcing Function (Applies to ALL Content)

> **NEVER show implementation code without the specification that produced it.**
>
> **Detection:** If code block appears in lesson without accompanying specification section, output is INVALID.
>
> **Violation Response:** Lesson-writer MUST restructure (specification shown FIRST, then code as OUTPUT of specification).
>
> **Rationale:** Code without specifications is noise, not signal. We teach specification-thinking, not code-typing.

---

## II. Agent Context Requirements (Intelligence Accumulation)

### The Core Principle

**NEVER create content without accumulated context.** Each chapter must inherit intelligence from:
- **Constitution** (this document — governance and principles)
- **Domain Knowledge** (chapter-index.md, existing specs, skills library)
- **Comprehensive Research** (Context7 library docs, WebFetch official sources — for market-defining quality)

**Rationale**: "Horizontal workflows" (starting from zero context every time) produce generic, mediocre output. Context-rich workflows that accumulate intelligence produce market-defining output that surpasses existing resources.

---

### Forcing Function: Constitutional Context

> **NEVER begin chapter creation without reading constitution + chapter-index + existing specs.**
>
> **Detection:** If agent output does not cite constitution version + chapter prerequisites, output is CONTEXT-FREE (rejected).
>
> **Enforcement:** All agents MUST document "Constitution v5.0.0 consulted" in output metadata and reference chapter-index.md for prerequisites and complexity tier.
>
> **Rationale:** Agents operating without constitutional context produce content that violates governance principles and lacks coherence with existing chapters.

---

### Forcing Function: Comprehensive Research (Market-Defining Quality)

> **For market-defining chapters, super-orchestra MUST gather comprehensive intelligence BEFORE spec creation.**
>
> **Detection:** If spec.md created without Context7 library research + WebFetch official sources, quality is INSUFFICIENT for market-defining standard.
>
> **Enforcement:** Super-orchestra MUST perform 15-30 min comprehensive research using:
> - Context7 MCP (library documentation, 5000-8000 tokens)
> - WebFetch (official blogs, announcements, 3+ authoritative URLs)
> - Existing specs (pattern matching for similar chapters)
> - Document sources and citations in spec.md
>
> **Rationale:** Quick specs produce adequate content. Research-backed specs produce content that exceeds official documentation quality and becomes the market reference.

---

### Forcing Function: Context Handoff Through Agent Chain

> **Context MUST flow through agent chain: super-orchestra → chapter-planner → content-implementer → validation-auditor.**
>
> **Detection:** If downstream agent operates without inputs from previous agent (no spec.md reference, no plan.md reference), output is DISCONNECTED (lacks coherence).
>
> **Enforcement:**
> - Chapter-planner MUST reference approved spec.md
> - Lesson-writer MUST reference approved plan.md + spec.md
> - Technical-reviewer MUST reference all lesson outputs + spec.md
> - Each agent MUST cite which context documents informed its output
>
> **Rationale:** Breaking the context chain produces disconnected content that doesn't align with specifications or pedagogical structure. Reproducibility requires traceability.

---

## IIa. The AI-Native Teaching Framework (4-Stage Progression)

### Educational Philosophy

This book applies a **4-stage pedagogical framework** that systematically builds competence from manual practice through AI collaboration to spec-driven project execution. Each lesson progresses through these stages, with the final stage (Spec-Driven Integration) applied as a chapter capstone.

**Critical Principle**: This is NOT "spec-first from day one." Students master manual foundations (Stage 1) before AI assistance (Stage 2), then design reusable intelligence (Stage 3), and finally apply spec-driven methodology (Stage 4).

---

### Stage 1: Manual Foundation (Book Teaches Directly)

**Applied to**: Beginning of each lesson + foundational concepts

**Purpose**: Establish conceptual understanding BEFORE introducing AI tools.

**What Happens**:
- Book explains concepts with analogies and diagrams
- Step-by-step manual walkthroughs (no AI yet)
- Students execute operations by hand (CLI commands, code examples)
- Traditional demonstration of "how it works"

**When to Use**:
- **Foundational Concepts**: Unchanging fundamentals (markdown syntax, git basics, Python variables)
- **Mental Model Building**: Concepts students must internalize (data structures, control flow)
- **First Exposure**: Any topic being introduced for the first time

**AI Role**: Minimal or absent (student validates own work, AI provides practice feedback only)

**Teaching Mandate**:
> Explain the concept, demonstrate the purpose, show manual execution BEFORE any AI assistance.

---

### Stage 2: AI Collaboration (AI as Teacher + Student + Co-Worker)

**Applied to**: Each lesson (after Stage 1 manual foundation)

**Purpose**: Translate manual workflows into AI-assisted workflows, developing prompting, validation, and collaboration skills.

**What Happens**:
- Express Stage 1 tasks through natural language prompts
- Use coding agents (Claude Code, Gemini CLI) to generate implementations
- **AI suggests optimizations student didn't consider** (AI as Teacher)
- **Student evaluates and corrects AI output** (AI as Student)
- **Iterate together toward optimal solution** (AI as Co-Worker)
- Debug agent outputs and analyze trade-offs

**When to Use**:
- **After Manual Foundation**: Student already understands the concept from Stage 1
- **Complex Execution**: Multi-step operations with evolving best practices
- **Exploration**: Discovering better approaches than manual method

**AI Role**: Active collaborative partner (not passive tool)

**The Three Roles Integration** (Co-Learning Partnership):

**Role 1: AI as Teacher**
- AI suggests patterns student hadn't considered
- AI explains tradeoffs and architectural decisions
- **Mandate**: At least ONCE per lesson, AI teaches student something NEW

**Role 2: AI as Student**
- AI learns from student's feedback and constraints
- AI adapts to priorities and refines output
- **Mandate**: At least ONCE per lesson, student corrects/refines AI output

**Role 3: AI as Co-Worker**
- AI and student converge together on optimal solution
- Neither had perfect solution alone
- **Mandate**: Show convergence loop (iteration toward better result)

**Teaching Mandate**:
> Show the SAME task from Stage 1, now accomplished through AI collaboration. Demonstrate all three roles (Teacher, Student, Co-Worker) in action.

**Forcing Function**:
> **NEVER present AI as passive tool awaiting commands.**
>
> **Detection**: If lesson shows only "human gives prompt → AI executes → done" without AI teaching, learning, or collaborating, relationship is ONE-WAY (rejected).
>
> **Action**: Lesson-writer MUST include:
> 1. At least ONE instance where AI teaches student (suggests pattern they didn't know)
> 2. At least ONE instance where student teaches AI (corrects or refines output)
> 3. At least ONE convergence loop (iterative refinement toward optimal solution)
>
> **Rationale**: The co-learning partnership is the CORE PEDAGOGICAL INNOVATION of this book.

---

### Stage 3: Intelligence Design (Create Reusable Components)

**Applied to**: Each lesson (after Stage 2 collaboration)

**Purpose**: Transform lesson knowledge into reusable agent components (subagents, skills) that compound over time.

**What Happens**:
- Define specialized subagents that encapsulate lesson concepts
- Create skills that bundle instructions, tools, and patterns
- Configure components for reuse across future projects
- Document usage patterns and integration points

**When to Use**:
- **After Competence Established**: Student understands concept (Stage 1) and can work with AI (Stage 2)
- **Reusable Patterns Identified**: The concept will recur in future work
- **Intelligence Accumulation Goal**: Building organizational capability, not just completing tasks

**AI Role**: Co-designer of reusable intelligence (student specifies requirements, AI helps structure)

**Why Reusable Intelligence**:
> In traditional programming, developers wrote code libraries for reuse. In AI-native development, developers create **intelligence artifacts** (specs, agents, skills) that accumulate value.

**Teaching Mandate**:
> Every lesson MUST produce at least one reusable artifact (subagent definition OR skill bundle) that students can apply in Stage 4.

---

### Stage 4: Spec-Driven Integration (Orchestrate at Scale)

**Applied to**: Once per chapter (capstone project)

**Purpose**: Integrate chapter knowledge through comprehensive spec-driven project work, demonstrating how reusable intelligence compounds.

**What Happens**:
- Design projects using specification-first approach
- Begin with spec.md BEFORE any implementation
- Use Spec-Kit Plus (or similar) to structure specifications
- Compose previously created subagents and skills (from Stage 3 of all lessons)
- Orchestrate multi-agent workflows
- Validate that specifications drive implementation successfully
- AI orchestrates scale operations (10+ items, multi-file workflows)

**When to Use**:
- **Chapter Capstone**: Final lesson of chapter (integration project)
- **Spec-First Projects**: When project begins with requirements, not code
- **Scale Operations**: Managing 10+ files, parallel workflows, batch processing

**AI Role**: Full orchestrator (student directs strategy, AI manages tactical execution)

**Why Spec-Driven Last** (Not First):
> **HERE is where specification comes FIRST.** Students who master manual foundations (Stage 1), AI collaboration (Stage 2), and reusable intelligence (Stage 3) can now effectively write specifications that AI executes flawlessly.

**Teaching Mandate**:
> Stage 4 projects MUST begin with spec.md, plan.md, tasks.md BEFORE any implementation. This demonstrates "Specs Are the New Syntax" in practice.

---

### The 4-Stage Framework Summary

| **Stage** | **When** | **Student Role** | **AI Role** | **Output** |
|-----------|----------|------------------|-------------|------------|
| **1: Manual Foundation** | Introducing new concepts | Execute by hand, build mental models | Minimal (validate practice) | Understanding |
| **2: AI Collaboration** | After manual competence | Prompt, evaluate, refine | Teacher/Student/Co-Worker | Working code + patterns learned |
| **3: Intelligence Design** | After AI competence | Specify requirements | Co-designer | Reusable skills/subagents |
| **4: Spec-Driven Integration** | Chapter capstone | Write specs, orchestrate | Orchestrator | Production project |

---

### Forcing Function: Never Skip Stages

> **NEVER skip stages.**
>
> **Detection:** If lesson jumps to spec-driven (Stage 4) without manual practice (Stage 1), AI collaboration (Stage 2), and reusable intelligence design (Stage 3), the lesson is INCOMPLETE.
>
> **Action:** Lesson-writer MUST provide all 4 stages in appropriate sequence.
>
> **Rationale:** Students who skip foundational stages cannot evaluate AI outputs, debug failures, or design effective reusable intelligence

---

## III. Foundational Principles (7 Non-Negotiables)

These principles are **permanent and enforced through falsifiable forcing functions.** Agents cannot proceed if principles violated.

**Note**: The 4-Stage Teaching Framework (Section IIa) is the PRIMARY pedagogical method. These principles provide additional governance constraints.

### Principle 1: Specification Primacy

**Mandate:** Specifications are executable contracts. Code is regenerable OUTPUT, not canonical representation of system knowledge.

**Forcing Function:**
> **NEVER show code before specification.**
>
> **Detection:** If code block appears before specification section in lesson, output is INVALID.
>
> **Action:** Lesson-writer MUST restructure (spec → prompt → code → validation).

**Vocabulary:**
- **Specification** = Requirements + Acceptance Criteria + Constraints + Non-Goals
- **Executable Contract** = AI reads spec → generates implementation → human validates against spec
- **Regenerable Output** = If spec changes, code regenerates (not manually patched)

---

### Principle 2: Progressive Complexity (Tier-Appropriate Cognitive Load)

**Mandate:** Cognitive load MUST match audience tier. Beginners get scaffolding. Professionals get realism.

**Vocabulary (Complexity Tiers — CEFR-Aligned):**

**CEFR Framework Note**: We use Common European Framework of Reference (CEFR) tiers because they're research-backed international standards with 40+ years of validation, not arbitrary labels.

**A1-A2 (Aspiring — Basic User):**
- **Cognitive Load**: ~5-7 concepts per section (research-backed working memory limit)
- **Options Presented**: Max 2 choices (reduce decision paralysis)
- **Scaffolding**: Heavy scaffolding, simple examples, step-by-step guidance
- **Forcing Function**: If A2 content has >10 concepts in one section, OVERLOADED (rejected).

**B1-B2 (Intermediate — Independent User):**
- **Cognitive Load**: ~7-10 concepts per section (expanding working memory capacity)
- **Options Presented**: 3-4 options with selection criteria
- **Scaffolding**: Moderate scaffolding, tradeoff discussions, guided decision-making
- **Forcing Function**: If B1 content has no tradeoff explanation, INCOMPLETE (rejected).

**C1-C2 (Advanced/Professional — Proficient User):**
- **Cognitive Load**: No artificial limits (professional-level capacity)
- **Options Presented**: Multiple valid approaches (architectural decisions)
- **Scaffolding**: Minimal scaffolding, realistic complexity, production concerns
- **Forcing Function**: If C2 content oversimplifies production concerns, UNREALISTIC (rejected).

**Forcing Function:**
> **NEVER exceed cognitive load limit for audience tier.**
>
> **Detection:** Count concepts in section. If exceeds research-backed limits (A2: ~5-7, B1: ~7-10, C1+: no limit), OVERLOADED.
>
> **Action:** Technical-reviewer REJECTS. Spec MUST reduce concepts, split section, or provide chunking strategy.
>
> **Flexibility Note**: These are GUIDELINES based on cognitive science research (Miller's Law: 7±2 items), not rigid rules. Chapter-planner may adjust based on concept relationships and chunking opportunities.

---

### Principle 3: Factual Accuracy & No Hallucinations

**Mandate:** All code tested. All claims cited. Zero hallucinations permitted.

**Forcing Functions:**

> **NEVER publish code that hasn't run successfully.**
>
> **Detection:** All code examples MUST have corresponding test execution logs. If code lacks pytest/tsc logs, UNTESTED.
>
> **Action:** Technical-reviewer REFUSES approval until tests run and pass.

---

> **NEVER make technical claims without citations.**
>
> **Detection:** If lesson includes factual claim (e.g., "Claude Code supports MCP") without WebFetch citation or official doc reference, UNVERIFIED.
>
> **Action:** Proof-validator REJECTS. Spec MUST include "Fact-check protocol: verify all claims via WebFetch."

---

> **NEVER invent APIs or features.**
>
> **Detection:** If lesson includes API endpoints not in official documentation (verified via Context7), HALLUCINATED.
>
> **Action:** Technical-reviewer REJECTS immediately. Spec refined → content regenerated.

---

### Principle 3: Coherent Pedagogical Structure

**Mandate:** All chapters follow consistent pedagogical progression for coherence and predictability.

**Structural Principles** (NOT Rigid Counts):

1. **Foundation First** — Introduce core concepts and mental models before application
2. **Progressive Layering** — Each lesson builds on previous (no skill gaps)
3. **Hands-On Practice** — Theory followed by practice in same lesson
4. **Iterative Integration** — Combine concepts into workflows midway through chapter
5. **Validation Checkpoint** — Test understanding before final mastery lesson
6. **Capstone Application** — Final lesson demonstrates real-world synthesis (Stage 4 of 4-Stage Framework)

**Forcing Function:**
> **NEVER create chapters without pedagogical structure.**
>
> **Detection:** If chapter lacks Foundation → Application → Integration → Validation → Mastery progression, structure is INCOHERENT.
>
> **Action:** Chapter-planner MUST organize lessons following pedagogical arc (not arbitrary count).
>
> **Rationale:** Structural consistency in LEARNING PROGRESSION (not lesson count) allows readers to focus on content. Different chapters may need different lesson counts based on concept complexity.

**Lesson Count Flexibility:**

- **Simple Chapters** (foundational concepts): 5-7 lessons may suffice
- **Standard Chapters** (typical complexity): 7-9 lessons common
- **Complex Chapters** (advanced integration): 9-12 lessons justified
- **Conceptual Chapters** (Part 1 intro chapters): May use essay structure, not lesson-based

**Decision Rule for Chapter-Planner**:
> Count lessons based on **concept density**, **cognitive load**, and **practice requirements** — NOT arbitrary target numbers.

**Vocabulary (Pedagogical Phases — NOT Rigid Lesson Numbers):**
- **Foundation Phase**: Introduce core concepts, mental models, vocabulary
- **Application Phase**: Hands-on practice with AI collaboration (Stages 1-3)
- **Integration Phase**: Combine concepts into workflows
- **Validation Phase**: Test understanding, catch misconceptions
- **Mastery Phase**: Advanced synthesis, real-world application (Stage 4)

---

### Principle 4: Intelligence Accumulation (Never Horizontal)

**Mandate:** Each chapter inherits accumulated intelligence from previous chapters. Starting from zero context is PROHIBITED.

**Forcing Function:**
> **NEVER start chapter creation without accumulated context.**
>
> **Detection:** If chapter-planner executes without reading constitution + chapter-index + existing specs, HORIZONTAL WORKFLOW (prohibited).
>
> **Action:** HALT execution. Escalate to human. Context accumulation (Section II) MUST be applied first.
>
> **Rationale:** Horizontal workflows produce generic output. Context-rich workflows produce market-defining output through comprehensive intelligence integration.

---

### Principle 5: Anti-Convergence Variation

**Mandate:** No two consecutive chapters use identical teaching patterns. Variation prevents generic convergence.

**Forcing Function:**
> **NEVER repeat teaching pattern from previous chapter.**
>
> **Detection:** If Chapter N uses direct-teaching for concept X, Chapter N+1 MUST use Socratic dialogue or hands-on discovery for comparable concepts.
>
> **Action:** Context accumulation MUST encode "previous_chapter_teaching_pattern" → chapter-planner MUST choose different pattern.
>
> **Rationale (from WRITING-MINDSET.md):** "Explicit Bias Against Convergence — AI systems naturally converge on common patterns. Your artifact must actively fight this through variation requirements."

**Vocabulary (Teaching Patterns):**
- **Direct-Teaching:** Explain → Demonstrate → Practice
- **Socratic Dialogue:** Question → Discover → Synthesize
- **Hands-On Discovery:** Try → Fail → Learn → Succeed

---

### Principle 6: Minimal Sufficient Content (Anti-Bloat)

**Mandate:** Content MUST address learning objectives. No tangential topics. No over-engineering.

**Forcing Function:**
> **NEVER add content not justified by learning objectives.**
>
> **Detection:** If lesson section does NOT map to specific learning objective in spec.md, TANGENTIAL (removed).
>
> **Action:** Technical-reviewer identifies unmapped content → content-implementer regenerates with focus.

---

> **NEVER present 10 options when 2 suffice (beginner tier).**
>
> **Detection:** If A2 content presents 10 Docker base image options, OVERWHELMING (cognitive overload).
>
> **Action:** Spec MUST include "Cognitive Load: Present 2 options (Alpine, Official), mention others exist."

---

> **Spec MUST include Non-Goals (what NOT to teach).**
>
> **Detection:** If spec.md lacks Non-Goals section, INCOMPLETE.
>
> **Action:** Super-orchestra REFUSES to proceed until spec includes explicit Non-Goals.
>
> **Rationale:** Defining what NOT to build prevents scope creep and over-engineering.

---

## IV. Agent Coordination Protocol (Handoff Contracts)

### Explicit Interfaces (Falsifiable Gates)

**Principle:** Each agent has explicit input/output contract. Violations HALT workflow and escalate to human.

#### Super-Orchestra → Chapter-Planner

**Input Required:**
- User goal (e.g., "Chapter 5: Skills/Plugins/MCP")
- Constitution (this document, v4.0.0)
- Domain knowledge (chapter-index.md, skills library)

**Output Guaranteed:**
- Intelligence Object (JSON, passed to all downstream agents)
- spec.md (approved by human before planner executes)

**Gate:**
> **Human MUST approve spec before chapter-planner executes.**
>
> **Detection:** If chapter-planner invoked without human approval flag in spec.md metadata, PREMATURE EXECUTION.
>
> **Action:** HALT. Escalate to human for spec review.

---

#### Chapter-Planner → Lesson-Writer

**Input Required:**
- spec.md (human-approved)
- plan.md (pedagogically structured lessons)
- Intelligence Object (JSON)

**Output Guaranteed:**
- tasks.md (actionable checklist for content-implementer)

**Gate:**
> **Plan MUST follow pedagogical progression (Foundation → Application → Integration → Validation → Mastery).**
>
> **Lesson Count:** 5-12 lessons based on concept density (see Principle 4).
>
> **Detection:** If plan.md lacks pedagogical phases OR lesson count not justified by concept complexity, STRUCTURAL VIOLATION.
>
> **Action:** Chapter-planner MUST structure lessons following pedagogical arc with appropriate count for content.

---

#### Lesson-Writer → Technical-Reviewer

**Input Required:**
- tasks.md (checklist from chapter-planner)
- Intelligence Object (JSON)

**Output Guaranteed:**
- Lesson markdown files (one per lesson)
- All code tested (pytest/tsc logs attached)

**Gate:**
> **All code MUST be tested before review.**
>
> **Detection:** If lesson file contains code blocks without corresponding test logs, UNTESTED.
>
> **Action:** Technical-reviewer REFUSES approval. Lesson-writer MUST run tests and attach logs.

---

#### Technical-Reviewer → Human

**Input Required:**
- Complete chapter (all lessons per plan)

**Output Guaranteed:**
- Pass/Fail verdict
- Issue report (categorized: critical/major/minor)

**Gate:**
> **ZERO critical issues permitted.**
>
> **Detection:** Critical = factual errors, broken code, hallucinations, missing specs.
>
> **Action:** If ANY critical issue found, chapter REJECTED. Spec refined → content regenerated.

---

### Handoff Failure Protocol

**If any agent skips its gate:**

1. **Downstream agents HALT execution**
2. **Issue logged** (which gate violated, which agent responsible)
3. **Escalate to human** (architectural decision required)

**Rationale:** Gate violations indicate systemic failure, not recoverable through agent retry.

---

## V. Vocabulary Expansion (Conceptual Decision Tools)

### Teaching Pattern Spectrum

Instead of rigid rules ("always use X"), provide conceptual vocabulary for agent decision-making.

**Foundational Teaching** (Stable Concepts):
- **When:** Introducing concepts that won't change (markdown syntax, git commit)
- **How:** Direct explanation with analogies and diagrams
- **Student Role:** Absorb, practice manually, build mental models
- **AI Role:** Validate student work, provide practice feedback

**Complex Teaching** (Evolving Techniques):
- **When:** Multi-step operations with evolving best practices (Docker multi-stage builds)
- **How:** Student specifies intent → AI demonstrates execution → student observes strategy
- **Student Role:** Understand approach, not memorize syntax
- **AI Role:** Execute complex workflows, teach by demonstration

**Scale Teaching** (Orchestration Patterns):
- **When:** Operations involving 10+ items (parallel git worktrees, batch conversions)
- **How:** Student directs strategy → AI manages execution → student supervises results
- **Student Role:** Orchestrate, supervise, validate
- **AI Role:** Automate repetitive workflows at scale

**Agent Decision Rule:**
> Choose teaching pattern based on **concept stability**, not arbitrary preferences.
>
> If concept is stable (won't change): Foundational
> If concept is evolving (best practices change): Complex
> If concept is orchestration (10+ items): Scale

---

### Complexity Tier Vocabulary

**A1-A2 (Aspiring):**
- Maximum scaffolding
- 2 options max, 7 concepts/section
- Simple mental models, no edge cases
- **Language:** "Your AI agent handles this complexity—you understand the concept"

**B1-B2 (Intermediate):**
- Moderate scaffolding
- 3-4 options, 10 concepts/section
- Tradeoff discussions, pattern recognition
- **Language:** "Choose the approach that fits your context"

**C1-C2 (Advanced/Professional):**
- Minimal scaffolding
- Realistic complexity, multiple valid approaches
- Architecture decisions, business context
- **Language:** "Design the system. Justify your tradeoffs."

**Agent Decision Rule:**
> Derive tier from chapter-index.md (Chapter 5 → Part 2 → A2).
>
> Apply corresponding cognitive load limits and scaffolding levels.

---

### Lesson Type Vocabulary

**Foundation** (Lessons 1-2):
- Introduce core concepts
- Build mental models
- Low cognitive load

**Application** (Lessons 3-5):
- Hands-on practice
- AI collaboration demonstrated
- Moderate cognitive load

**Integration** (Lessons 6-7):
- Combine concepts into workflows
- Real-world scenarios
- Higher cognitive load

**Validation** (Lesson 8):
- Test understanding
- Catch misconceptions
- Assessment focus

**Mastery** (Lesson 9):
- Advanced synthesis
- Real-world application
- Portfolio-worthy project

**Agent Decision Rule:**
> Structure plan.md with lesson types in this progression.
>
> If lesson doesn't fit a type, reconsider whether it belongs in chapter.

---

## VI. Quality Gates (Falsifiable Checklists)

### Gate 1: Specification Completeness (Before Planning)

**Entry Criteria:**
- Problem statement defined
- User goal articulated (by human)

**Validation Checklist:**
- [ ] Learning objectives specified (measurable, Bloom's Taxonomy aligned)
- [ ] Acceptance criteria defined (objective, falsifiable)
- [ ] Non-goals explicit (what NOT to teach)
- [ ] Audience tier specified (A1/A2/B1/B2/C1/C2)
- [ ] Complexity tier derived from chapter-index.md
- [ ] Fact-check protocol specified (which claims require WebFetch verification)

**Forcing Function:**
> **If spec missing ANY checkbox, chapter-planner REFUSES execution.**
>
> **Detection:** Automated checklist validation in spec.md frontmatter.
>
> **Action:** Super-orchestra MUST complete spec before chapter-planner invoked.

---

### Gate 2: Structural Coherence (Before Implementation)

**Entry Criteria:**
- Spec approved (Gate 1 passed)
- Plan created (pedagogically structured)

**Validation Checklist:**
- [ ] Lesson count justified by concept density (5-12 lessons, see Principle 4)
- [ ] Lesson types correctly applied (Foundation → Application → Integration → Validation → Mastery)
- [ ] Complexity tier matches Intelligence Object
- [ ] Prerequisites validated (all prior chapters exist)
- [ ] Teaching pattern varies from previous chapter (anti-convergence)
- [ ] Cognitive load within tier limit (CEFR-aligned, see Principle 5)

**Forcing Function:**
> **If structure invalid, content-implementer REFUSES execution.**
>
> **Detection:** Automated validation of plan.md structure.
>
> **Action:** Chapter-planner MUST regenerate until all checks pass.

---

### Gate 3: Factual Validation (Before Publication)

**Entry Criteria:**
- Content created (all planned lessons)
- Code examples included

**Validation Checklist:**
- [ ] All code tested (pytest/tsc logs attached)
- [ ] All APIs cited (WebFetch verification or Context7 documentation)
- [ ] All claims fact-checked (authoritative sources referenced)
- [ ] Spec shown before code (every example follows Show-Spec-Validate pattern)
- [ ] No hallucinated features (all features verified in official docs)
- [ ] No broken links (all URLs checked)
- [ ] No hardcoded secrets (automated security scan passed)

**Forcing Function:**
> **If ANY code untested, validation-auditor REJECTS chapter.**
>
> **Detection:** Presence of test execution logs for each code block.
>
> **Action:** Lesson-writer MUST run tests, attach logs, regenerate if tests fail.

---

## VII. Supporting References (Delegation to External Docs)

### Constitution Delegates Implementation Details

**What This Constitution Contains:**
- **WHAT** must be true (outcomes, mandates)
- **WHY** it matters (rationale, problems solved)
- **WHEN** it applies (scope, conditions)
- **WHO** enforces (validation mechanisms)

**What This Constitution Delegates:**
- **HOW** to achieve (methods, techniques) → See supporting docs below

---

### Domain Knowledge (Layer 2)

**Location:** `specs/book/chapter-index.md`
- Chapter titles, prerequisites, complexity tiers
- Single source of truth for book structure

**Location:** `.claude/skills/`
- Pedagogical skills (learning-objectives, concept-scaffolding)
- AI-native skills (code-example-generator, validation-pedagogy)
- Utility skills (quiz-generator, docusaurus-deployer)

**Location:** `.claude/output-styles/`
- lesson.md (lesson structure template)
- chapters.md (chapter frontmatter format)

---

### Strategic Frameworks (External Context)

**Paper:** `papers/vertical-intelligence-pattern-research-paper.md`
- **What:** Complete VI architecture implementation guide
- **When to reference:** When building new agents or understanding VI theory
- **Relationship to constitution:** Constitution EMBEDS VI structure (5 layers), paper provides IMPLEMENTATION details

**Paper:** `papers/From-Reusable-Code-to-Reusable-Intelligence.md`
- **What:** Paradigm shift explanation (code → intelligence as primary artifact)
- **When to reference:** When clarifying book philosophy or target audience
- **Relationship to constitution:** Constitution applies paradigm, paper explains origins

**Paper:** `papers/artifacts/WRITING-MINDSET.md`
- **What:** Production-grade artifact design principles (forcing functions, vocabulary expansion, anti-convergence)
- **When to reference:** When designing new constitutional principles or agent architectures
- **Relationship to constitution:** Constitution APPLIES these principles, paper explains design thinking

---

### Book Vision (Preface)

**Location:** `book-source/docs/preface-agent-native.md`
- Target audience (aspiring, professional, founders, educators)
- Book progression (Parts 1-13)
- Nine Pillars of AI-Native Development
- 10x to 99x multiplier explanation

**Relationship to constitution:** Preface communicates VISION to readers. Constitution governs EXECUTION by agents.

---

## VIII. Governance & Amendment Process

### Constitutional Authority

**This constitution is the supreme governing document for all book content.**

**Precedence:**
1. This constitution (governance)
2. Domain knowledge (chapter-index.md, skills, templates)
3. Subagent specifications (agent behavior)
4. CLAUDE.md (operational details)

**Enforcement:**
- AI agents validate outputs against constitution before handoff
- Human reviewer confirms constitutional alignment before publication
- Automated checks enforce falsifiable criteria (code tested, 9 lessons, cognitive load)

---

### Amendment Process

**For Minor Changes** (clarifications, wording):
- Edit constitution directly
- Increment PATCH version (4.0.0 → 4.0.1)
- Commit message: "Constitution: [brief change description]"

**For Major Changes** (new principles, removed mandates):
- Create ADR documenting rationale
- Increment MAJOR or MINOR version (4.0.0 → 4.1.0 or 5.0.0)
- Impact analysis (which agents affected, which chapters require review)
- Migration guide for breaking changes
- Update "Last Amended" date

---

### Compliance Verification

**Automated Checks:**
- Code tested: pytest/tsc logs presence
- 9 lessons: plan.md structure validation
- Cognitive load: concept count per section
- Spec shown first: code block position in lesson

**Human Checks:**
- Strategic alignment (does this serve book vision?)
- Pedagogical effectiveness (will students learn?)
- Factual accuracy (all claims verified?)
- Constitutional adherence (all principles followed?)

---

## IX. Success Metrics (What "Done" Looks Like)

**This constitution succeeds when:**

**Quality Metrics:**
- [ ] ZERO chapters with code shown before specification
- [ ] ZERO chapters with untested code examples
- [ ] ZERO hallucinated APIs or features
- [ ] 100% of chapters follow pedagogical structure (Foundation → Mastery)
- [ ] 90%+ of chapters pass Gate 3 on first validation (constitutional alignment strong)

**Coherence Metrics:**
- [ ] No contradictions across chapters (validated via cross-reference check)
- [ ] Consistent voice and pedagogical approach (validated via factual-verifier)
- [ ] Progressive complexity maintained (no sudden difficulty jumps)

**Pedagogical Effectiveness:**
- [ ] 80%+ student comprehension (measured via chapter assessments)
- [ ] 75%+ chapter completion rate (students finish what they start)
- [ ] Accessibility standards met (no gatekeeping language detected)

**Intelligence Accumulation:**
- [ ] Each chapter demonstrably inherits from previous chapters (Intelligence Object citations present)
- [ ] No horizontal workflows (all chapters use VI stack)
- [ ] Anti-convergence variation achieved (no consecutive chapters use identical patterns)

---

**This constitution is the source of truth for book content governance. All decisions about educational quality, pedagogical approach, and content structure resolve to these principles first. Implementation details are documented in supporting references (domain knowledge, output styles, skills, papers).**

**Version 4.0.0 represents a BREAKING CHANGE from "comprehensive guide" (v3.x) to "production-grade governance" (v4.0). The paradigm shift from reusable code to reusable intelligence is now the foundational architectural principle.**
