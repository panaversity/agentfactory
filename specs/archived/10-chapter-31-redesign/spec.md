---
title: "Redesign Chapter 31: Spec-Kit Plus Hands-On"
feature: "chapter-31-redesign"
part: 5
chapter: 31
date_created: "2025-11-03"
status: "Draft"
authors: ["Panaversity Team"]
complexity_tier: "Intermediate"
---

# Specification: Redesign Chapter 31 — Spec-Kit Plus Hands-On

## Executive Summary

Chapter 31 teaches practical Spec-Kit Plus workflow through hands-on **calculator capstone project**. This chapter represents the natural progression from Chapter 30's tool landscape introduction (Kiro, Spec-Kit, Spec-Kit Plus, Tessel) to **actually practicing the Spec-Kit Plus methodology** chosen for this book.

**Context from Chapter 30**: Students compared four SDD approaches and understand why Spec-Kit Plus was chosen (GitHub Spec-Kit foundation + ADRs + PHRs + Vertical Intelligence). Now they need hands-on practice.

**This redesign**:
- Eliminates hallucinated commands and workflow confusion
- Centers on AIDD (AI-Driven Development) paradigm from preface
- Implements ADR-001 pedagogical architecture (calculator + 8 lessons + workflow isomorphism)
- Teaches complete Spec-Kit Plus workflow: Constitution → Specify → Plan → Tasks → Implement
- Demonstrates **Vertical Intelligence**: human collaborates with AI orchestrator, which delegates to specialized subagents
- Demonstrates cascade effect: spec quality → plan quality → code quality

**Delivery**: 8 lessons mirroring actual workflow phases (7 core + 1 capstone integration)

---

## Natural Progression from Chapter 30

**Where Chapter 30 Left Off**:
- **Lesson 1-3**: Experienced vibe-coding problem → learned SDD philosophy → built first spec (calculator)
- **Lesson 4**: Learned about Constitutions, ADRs, PHRs for team coordination
- **Lesson 5**: Understood historical context (why SDD emerged NOW in 2025)
- **Lesson 6**: **Compared four tools** → Kiro (simple) vs Spec-Kit (GitHub) vs **Spec-Kit Plus** (Panaversity) vs Tessel (spec-as-source)

**Critical Transition Point**:
Students know WHAT Spec-Kit Plus offers (ADRs + PHRs + Intelligence Templates) but haven't actually USED the complete workflow. Calculator exercise in Chapter 30 was manual specification writing. Chapter 31 teaches the **opinionated Spec-Kit Plus workflow** with tooling support.

**What Chapter 31 Delivers**:
1. **Workflow Mastery**: Actually practice Constitution → Specify → Plan → Tasks → Implement cycle
2. **Tool Usage**: Learn /sp.specify, /sp.plan, /sp.tasks, /sp.implement commands via Spec-Kit Plus
3. **Vertical Intelligence Experience**: Work with AI orchestrator (main collaborator) that delegates to specialized subagents (skills-based)
4. **Artifact Creation**: Generate ADRs, PHRs in practice (not just theory)
5. **Complexity Progression**: Manual spec-writing (Chapter 30 calculator) → Tool-assisted workflow (Chapter 31 calculator with Spec-Kit Plus)
6. **Foundation for Chapter 32**: Complete workflow mastery enables team parallelization patterns

**Pedagogical Alignment**:
Follows preface paradigm → Chapter 30 established "why" and "what" → Chapter 31 delivers "how" through sustained practice with actual tooling.

---

## Vertical Intelligence: AI Orchestrator + Specialized Subagents

**What is Vertical Intelligence?**

Vertical Intelligence is Spec-Kit Plus's AI collaboration architecture where:
- **Human** works with **AI Orchestrator** (main collaborator)
- **AI Orchestrator** delegates specialized tasks to **Subagents** (domain experts)
- **Subagents** have specialized skills (specification-writing, planning, code-generation, validation)

**Architecture**:
```
Human (Architect/Validator)
   ↓
AI Orchestrator (Main Collaborator)
   ↓ delegates to →
   ├─ Specification Subagent (spec-writing skills)
   ├─ Planning Subagent (architecture decision skills)
   ├─ Implementation Subagent (code-generation skills)
   └─ Validation Subagent (quality review skills)
```

**How it works in Chapter 31**:
- **Lesson 3**: Human describes calculator → Orchestrator delegates to Specification Subagent → Returns complete spec
- **Lesson 5**: Human approves spec → Orchestrator delegates to Planning Subagent → Returns implementation plan + ADRs
- **Lesson 7**: Human approves plan → Orchestrator delegates to Implementation Subagent → Returns code + PHRs

**Why this matters**:
- Human doesn't need to learn every specialized skill (spec templates, ADR formats, code patterns)
- Orchestrator knows WHICH subagent to use for WHICH task
- Subagents apply domain expertise consistently
- Human focuses on intent, validation, and decision-making (architectural role)

**Chapter 30 vs Chapter 31**:
- **Chapter 30**: Manual spec-writing (human does everything)
- **Chapter 31**: Vertical Intelligence (human collaborates with orchestrator + subagents)

---

## The Complete Spec-Kit Plus SDD Loop

**Critical Learning Objective**: Students must understand the complete SDD workflow loop and when/why to use each command.

### The Complete 7-Phase Workflow (with Automatic Documentation)

```
1. CONSTITUTION → 2. SPECIFY → 3. CLARIFY → 4. PLAN → 5. TASKS → 6. IMPLEMENT → 7. VALIDATE
     ↓                ↓             ↓            ↓          ↓            ↓              ↓
   Project         Feature       Refine      Design    Decompose    Generate       Review
   Rules           Spec          Spec        Plan      Tasks        Code           Quality
                                             ↓                      ↓
                                          Run /sp.adr            PHRs auto-created
                                          (if needed)            (can request explicitly)
```

**Key insights**:
- **ADRs** are created explicitly via `/sp.adr` after planning when significant decisions need documentation
- **PHRs** are created **automatically** by the system for major interactions; you can also request explicitly if system misses

### Best Practice Workflow Pattern

**Constitution → Commit → Feature Loop**

```
1. Create Constitution (once per project)
   ↓
2. Commit to Git ("Initial constitution")
   ↓
3. FOR EACH FEATURE:
   - Run /sp.specify (creates spec, PHR auto-created)
   - Run /sp.clarify (refines spec, PHR auto-created)
   - Run /sp.plan (generates plan, PHR auto-created)
   - Run /sp.adr <title> (document significant decisions)
   - Run /sp.tasks (decompose plan, PHR auto-created)
   - Run /sp.implement (generate code, PHR auto-created)
   - Validate & test
   - Commit to Git
   ↓
4. REPEAT for next feature
```

**Why this pattern?**
- Constitution is foundational → commit once, all features build on it
- Each `/sp.*` command automatically creates PHR for that session
- PHRs accumulate in `history/prompts/<feature>/` documenting entire feature journey
- Git commits capture milestones (constitution, each completed feature)

**What students learn**:
- Constitution happens once (not per feature)
- PHRs accumulate automatically as you work (8-10 per feature typical)
- Git workflow integrates with SDD workflow naturally

### Command Reference: What Each Command Does

#### 1. **Constitution Phase** (One-Time Project Setup)
- **Command**: None (write constitution manually)
- **When**: **Once per project** at initialization (not per feature)
- **What**: Define immutable rules that apply to ALL features
- **Human Role**: Define principles, standards, constraints
- **Output**: `.specify/memory/constitution.md`
- **Chapter 31**: Lesson 2
- **⚠️ Best Practice**: Create constitution → commit to git → THEN start feature work
- **Why**: Constitution is foundational; all features build on these rules

#### 2. **Specify Phase** (`/sp.specify` - Feature Start)
- **Command**: `/sp.specify` (launches specification workflow for new feature)
- **When**: Start of every new feature (after constitution exists)
- **What**: Define WHAT needs to be built (requirements, acceptance criteria, scope)
- **Human Role**: Write requirements, define success criteria, collaborate with AI
- **Vertical Intelligence**: AI helps structure spec, suggests missing requirements
- **Output**: `specs/<feature>/spec.md`
- **Chapter 31**: Lesson 3
- **⚠️ PHR Auto-Created**: System automatically creates PHR for this specification session

#### 3. **Clarify Phase** (`/sp.clarify`)
- **Command**: `/sp.clarify`
- **When**: After initial spec is written
- **What**: AI analyzes spec for gaps, ambiguities, missing edge cases
- **Human Role**: Answer AI's clarifying questions, refine spec
- **Vertical Intelligence**: Orchestrator delegates to Clarification Subagent
- **Output**: Updated `specs/<feature>/spec.md` with refinements
- **Chapter 31**: Lesson 4
- **⚠️ PHR Auto-Created**: System records clarification session (questions asked, answers provided)

#### 4. **Plan Phase** (`/sp.plan`)
- **Command**: `/sp.plan`
- **When**: After spec is clarified and approved
- **What**: AI generates HOW to build it (architecture, components, dependencies, sequencing)
- **Human Role**: Review architecture decisions, approve/modify plan
- **Vertical Intelligence**: Orchestrator delegates to Planning Subagent
- **Output**: `specs/<feature>/plan.md`
- **Chapter 31**: Lesson 5
- **⚠️ PHR Auto-Created**: System records planning session (architecture choices, discussions)
- **⚠️ Important**: After plan is complete, identify architectural decisions that need documentation (next phase)

#### 5. **ADR Documentation** (`/sp.adr`)
- **Command**: `/sp.adr <decision-title>`
- **When**: After plan phase, when significant architectural decisions were made
- **What**: Document WHY you chose architecture option X over Y (reasoning, tradeoffs, alternatives)
- **Human Role**: Identify which decisions are architecturally significant; provide context for tradeoffs
- **Examples**: "Why JWT vs sessions?", "Why PostgreSQL vs MongoDB?", "Why microservices vs monolith?"
- **Output**: `history/adr/<number>-<decision-title>.md`
- **Chapter 31**: Lesson 5 (2-3 ADRs for calculator architecture decisions)
- **⚠️ Important**: Not every decision needs an ADR - only those that:
  - Have long-term impact
  - Had multiple viable alternatives
  - Future developers will question

#### 6. **Tasks Phase** (`/sp.tasks`)
- **Command**: `/sp.tasks`
- **When**: After plan is approved (and ADRs are documented)
- **What**: AI decomposes plan into atomic work units (4-8 hour tasks)
- **Human Role**: Review task breakdown, verify dependencies
- **Vertical Intelligence**: Orchestrator delegates to Tasks Subagent
- **Output**: `specs/<feature>/tasks.md` with dependency graph
- **Chapter 31**: Lesson 6
- **⚠️ PHR Auto-Created**: System records task generation session (decomposition approach, dependency decisions)

#### 7. **Implement Phase** (`/sp.implement`)
- **Command**: `/sp.implement`
- **When**: After tasks are approved
- **What**: AI generates code following spec → plan → tasks
- **Human Role**: Validate generated code against acceptance criteria, provide feedback
- **Vertical Intelligence**: Orchestrator delegates to Implementation Subagent
- **Output**: Working code + passing tests
- **Chapter 31**: Lesson 7
- **⚠️ PHR Auto-Created**: System records implementation session (code generation, debugging, refinements)

#### 7. **Validate Phase** (Manual Review + Testing)
- **Command**: None (run tests, review code quality)
- **When**: After implementation completes
- **What**: Verify code meets spec, passes tests, follows constitution
- **Human Role**: Final validation, acceptance testing, quality review
- **Output**: Validated, production-ready feature
- **Chapter 31**: Integrated throughout Lessons 3-7

---

### PHR Auto-Creation: Behind the Scenes Documentation

**PHRs (Prompt History Records) are NOT a manual workflow command** - they're automatically created by the system.

**How PHRs Work:**
- ✅ **Automatic**: System auto-creates PHRs for major interactions (running `/sp.*` commands, significant prompts)
- ✅ **Background**: Happens automatically - you don't need to think about it during workflow
- ✅ **Explicit request**: If system misses something important, you can say: "Record this as a PHR"
- ✅ **Storage**: Auto-saved to `history/prompts/<feature>/` or `history/prompts/general/`

**What Gets Auto-Recorded:**
- Running `/sp.clarify`, `/sp.plan`, `/sp.tasks`, `/sp.implement` commands
- Significant debugging sessions with AI
- Architecture discussions during planning
- Implementation decisions during coding

**When to Explicitly Request PHR:**
- System missed recording an important interaction
- You had a breakthrough insight with AI help
- Novel prompt pattern that worked exceptionally well
- Complex debugging that future team should learn from

**What Students Learn in Chapter 31:**
- PHRs exist and capture AI collaboration history
- Where to find them (`history/prompts/`)
- How to request explicit PHR creation if needed
- Why they matter (audit trail, learning, compliance)

**Students DON'T need to manually create PHRs** - the system does it automatically. They just need to know:
1. PHRs are being created as they work
2. Where to find them later
3. How to request one if system misses something

---

### Why This Order Matters (Cascade Effect)

**The Cascade Principle**: Quality flows downhill from spec to code.

```
Clear Spec → Clear Plan → Clear Tasks → Clear Code
Vague Spec → Vague Plan → Confused Tasks → Broken Code
```

**Students will experience**:
- **Lesson 3-4**: Writing clear vs vague specs
- **Lesson 5**: How spec quality affects plan quality
- **Lesson 6**: How plan quality affects task clarity
- **Lesson 7**: How task clarity enables correct code generation

### Command Usage Checklist (Students Must Know)

By end of Chapter 31, students can answer:

**Core Workflow Commands (6 commands):**
- [ ] When do I use `/sp.specify`? (Start of new feature - launch specification workflow)
- [ ] When do I use `/sp.clarify`? (After writing initial spec - AI finds gaps/ambiguities)
- [ ] When do I use `/sp.plan`? (After spec is clarified and approved - AI generates architecture)
- [ ] When do I use `/sp.adr <title>`? (After plan - document significant architectural decisions)
- [ ] When do I use `/sp.tasks`? (After plan and ADRs - AI decomposes into atomic work)
- [ ] When do I use `/sp.implement`? (After tasks approved - AI generates code)

**Understanding the Workflow:**
- [ ] What's my role in each phase? (Intent/validation, not implementation)
- [ ] Why can't I skip phases? (Quality cascades; skipping = broken downstream)
- [ ] How does Vertical Intelligence help? (Orchestrator delegates to expert subagents)

**Documentation Understanding (ADRs and PHRs):**
- [ ] Where are ADRs stored? (`history/adr/`)
- [ ] Where are PHRs stored? (`history/prompts/<feature>/`)
- [ ] What makes a decision "ADR-worthy"? (Long-term impact, multiple alternatives, future questioning)
- [ ] Are PHRs created manually? (No - auto-created by system; can request explicitly if missed)
- [ ] When should I request "Record this as a PHR"? (Only if system missed something significant)
- [ ] What gets auto-recorded as PHRs? (Every `/sp.*` command session - specify, clarify, plan, tasks, implement)
- [ ] How many PHRs per feature typical? (8-10 PHRs accumulate automatically during feature development)

**Best Practice Understanding:**
- [ ] When is Constitution created? (Once per project, not per feature)
- [ ] What to commit after Constitution? (Constitution → commit to git → then start features)
- [ ] What workflow pattern for each feature? (specify → clarify → plan → adr → tasks → implement → validate → commit)

---

## User Scenarios & Testing

### User Story 1 — Student Writes SMART Acceptance Criteria (Priority: P1)

**Who**: Developer learning to write clear, testable requirements

**What they do**:
1. Start with vague acceptance criteria (e.g., "the system should give helpful feedback")
2. Ask AI companion why vague criteria produce useless code (experiential learning)
3. Refine using SMART framework: Specific, Measurable, Achievable, Relevant, Time-bound
4. Produce criteria so clear that AI can build directly from them without interpretation

**Why this priority**: This is the KEY SKILL that makes AIDD work. Poor criteria → poor code. Clear criteria → usable code.

**Independent Test**: Student can take vague requirements and convert to SMART criteria. AI companion validates that criteria are "clear enough to build from."

**Acceptance Scenarios**:

1. **Given** vague acceptance criteria, **When** student asks AI companion to build from them, **Then** the resulting code is generic or incorrect (demonstrating the problem)

2. **Given** same requirement, **When** student refines to SMART criteria and asks AI companion to build, **Then** resulting code is specific, testable, and matches intent

3. **Given** a feature requirement, **When** student writes SMART criteria, **Then** each criterion is testable (can be verified with pass/fail), measurable (includes numbers or specific outcomes), and achievable (doesn't require AI telepathy)

---

### User Story 2 — Student Understands Spec-Kit Plus Project Structure (Priority: P1)

**Who**: Developer setting up first Spec-Kit Plus project

**What they do**:
1. Initialize a Spec-Kit Plus project using `/sp.init` command via Claude Code
2. Explore the generated folder structure (`.specify/`, `specs/`, `history/`)
3. Understand WHY the structure enforces a workflow (Spec → Plan → Tasks)
4. Recognize templates and constitution as guides for thinking

**Why this priority**: Sets up the mental model for the entire workflow. Without understanding structure, later steps are mechanical rather than meaningful.

**Independent Test**: Student can explain the relationship between directories and describe what forces the cascade from Spec → Plan → Tasks.

**Acceptance Scenarios**:

1. **Given** an initialized Spec-Kit Plus project, **When** student explores the directory structure, **Then** they can explain: purpose of each directory, why three files (spec.md, plan.md, tasks.md) exist in sequence, why you can't skip directly to tasks

2. **Given** a specification and companion's explanation of the structure, **When** student reads the spec template, **Then** they understand which sections correspond to requirements vs. implementation

---

### User Story 3 — Student Experiences Specification Cascade with Calculator Project (Priority: P2)

**Who**: Developer ready to see SDD workflow in action

**What they do**:
1. Complete a specification for **calculator project** (add, subtract, multiply, divide, power operations)
2. Use `/sp.clarify` within Claude Code to get AI feedback on spec completeness
3. Iterate on spec based on feedback (edge cases: division by zero, negative numbers, floating point precision)
4. Run `/sp.plan` to generate implementation plan from specification
5. See how plan automatically breaks into components, dependencies
6. Run `/sp.tasks` to decompose plan into atomic units
7. Witness cascade: clear spec → clear plan → clear tasks

**Why this priority**: Demonstrates the core value of SDD: spec quality determines everything downstream.

**Independent Test**: Student can describe how improving their specification improved the quality of the generated plan and tasks.

**Acceptance Scenarios**:

1. **Given** a completed calculator specification, **When** student uses `/sp.clarify` within Claude Code, **Then** they receive feedback on gaps (division by zero handling, type preservation, precision expectations)

2. **Given** feedback, **When** student iterates on specification, **Then** they can articulate which gaps they filled (explicit error handling, float vs int rules, edge case coverage)

3. **Given** an improved specification, **When** student uses `/sp.plan`, **Then** the generated plan has clear components (core operations module, CLI interface, test suite)

4. **Given** a specification that was initially vague but then refined, **When** student compares initial plan to refined plan, **Then** they observe how specification quality determined plan quality

---

### User Story 4 — Student Implements Using /sp.implement (Priority: P2)

**Who**: Developer ready to build, with approved spec and plan

**What they do**:
1. Review generated tasks and plan
2. Run `/sp.implement` to orchestrate code generation
3. AI companion generates code and tests based on tasks
4. Student reviews generated code against acceptance criteria
5. Student validates that code matches spec
6. Student learns how to correct AI if it misunderstands

**Why this priority**: Completes the AIDD loop. Spec → Plan → Tasks → Implementation. Students experience the full workflow.

**Independent Test**: Student can describe how they validated AI-generated code against their original specification and what they did when AI misunderstood something.

**Acceptance Scenarios**:

1. **Given** approved spec, plan, and tasks, **When** student uses `/sp.implement`, **Then** AI generates code and tests for tasks

2. **Given** AI-generated code, **When** student compares it to acceptance criteria, **Then** they can verify pass/fail for each criterion

3. **Given** AI-generated code that partially misses the mark, **When** student provides feedback, **Then** AI refines its output and student sees how clear specs enable faster refinement

---

### User Story 5 — Student Builds Calculator Capstone End-to-End (Priority: P3)

**Who**: Developer consolidating learning through complete calculator project

**What they do**:
1. Start with calculator project idea (5 operations: add, subtract, multiply, divide, power)
2. Write vague 1-sentence description
3. Run full Spec-Kit Plus workflow via Vertical Intelligence: specify → clarify → plan → tasks → implement
4. Experience orchestrator delegating to specialized subagents at each phase
5. Validate final implementation against original specification
6. Reflect on how specification quality determined implementation quality

**Why this priority**: Capstone that consolidates all learning. Students experience complete workflow with Vertical Intelligence architecture.

**Independent Test**: Student produces working calculator that passes all acceptance criteria defined in their own specification.

**Acceptance Scenarios**:

1. **Given** a vague calculator idea, **When** student completes full Spec-Kit Plus workflow, **Then** they have: specification, plan, tasks, working code, passing tests, ADRs, PHRs

2. **Given** completed calculator, **When** student reviews code against spec, **Then** all acceptance criteria pass (including edge cases: division by zero raises error, power handles negative exponents, type preservation works)

3. **Given** any bugs or discrepancies, **When** student traces back to specification, **Then** they discover the root cause was in spec ambiguity (e.g., didn't specify power(0,0) behavior), not code error

4. **Given** completed workflow, **When** student reflects on Vertical Intelligence, **Then** they can articulate: which subagent handled which phase, how orchestrator delegated tasks, why human focused on validation not implementation details

---

### Edge Cases

- What happens if a specification has conflicting requirements? → Students learn to resolve conflicts in spec phase, not implementation
- What happens if acceptance criteria are impossible? → Students learn to validate feasibility during planning
- What happens if AI misunderstands the spec? → Students learn how to refine spec and re-prompt, demonstrating that clarity is iterative

---

## Requirements

### Functional Requirements

**FR-001**: Chapter must teach the ACTUAL Spec-Kit Plus workflow via Vertical Intelligence
- ✅ `/sp.clarify` is used within Claude Code to refine specs (not `/sp.specify`)
- ✅ `/sp.plan` generates plans from specs via Planning Subagent
- ✅ `/sp.tasks` decomposes plans into tasks via Tasks Subagent
- ✅ `/sp.implement` orchestrates implementation via Implementation Subagent

**FR-002**: Chapter must demonstrate Vertical Intelligence architecture
- ✅ Show human working with AI Orchestrator (main collaborator)
- ✅ Show orchestrator delegating to specialized subagents (Specification, Planning, Implementation, Validation)
- ✅ Explain why human focuses on intent + validation, not implementation details

**FR-003**: Chapter structure must reflect actual Spec-Kit Plus workflow
- **Correct workflow**: Constitution → Specify → Clarify → Plan → ADR → Tasks → Implement → Validate
- **Chapter focus**: Specify → Clarify → Plan → Tasks → Implement (core loop for students)

**FR-004**: Chapter must teach when students think MANUALLY vs. when Vertical Intelligence assists
- **Manual thinking** (Lesson 1-2): Understanding workflow, SMART criteria
- **Vertical Intelligence assisted** (Lessons 3-7): Orchestrator + subagents handle spec/plan/tasks/code generation
- **Human role throughout**: Intent definition, validation, decision-making (architectural focus)

**FR-005**: Each lesson must be independently testable
- Lesson 1: Student explains Chapter 30→31 transition and Vertical Intelligence architecture
- Lesson 2: Student initializes Spec-Kit Plus project and understands structure
- Lesson 3: Student writes complete calculator specification (manually or with Specification Subagent)
- Lesson 4: Student refines spec using `/sp.clarify`
- Lesson 5: Student generates plan using `/sp.plan` and creates ADRs
- Lesson 6: Student decomposes plan into tasks using `/sp.tasks`
- Lesson 7: Student runs `/sp.implement`, validates code, creates PHRs
- Lesson 8: Student completes full workflow independently (capstone)

**FR-006**: Chapter must maintain "learn by doing" approach
- Every lesson includes hands-on exercises with real projects
- No pure theory; all concepts demonstrated through concrete examples
- Students experience problems before learning solutions

**FR-007**: Chapter must show cascade effect: Spec → Plan → Tasks → Code
- Demonstrate how bad specs produce bad plans
- Show how refined specs improve plan quality
- Prove that clear specs enable AI to generate correct code

**FR-008**: Chapter must teach co-learning between human and AI
- Student: Intent → AI: Questions/Suggestions → Student: Validation → AI: Refinement
- Cycle continues with feedback and iteration

---

## Content Structure (7 Lessons + Capstone)

**Design Principle**: Workflow isomorphism — each lesson mirrors one phase of actual Spec-Kit Plus workflow

### Lesson 1: Installation & Setup (Duration: 1.5 hours)

**Focus**: Get Spec-Kit Plus installed and working with your AI tool (Claude Code or Gemini CLI)

**Content**:
- **AI-Native SDD Toolkit** (10 min): Spec-Kit Plus = AI-native Spec-Driven Development toolkit with:
  - **Horizontal Intelligence**: ADRs (decisions) + PHRs (AI interactions) capture organizational knowledge
  - **Vertical Intelligence**: AI orchestrator + specialized subagents execute workflow phases
  - Works independently with any AI tool (Claude Code, Gemini CLI, etc.)
- **Install Spec-Kit Plus** (20 min): Install framework itself (independent of AI tool)
- **AI Tool Setup** (20 min): Configure Claude Code OR Gemini CLI to work with Spec-Kit Plus
- **Verify Commands** (15 min): Test that `/sp.*` commands work in your AI tool
- **Initialize Calculator Project** (20 min): Run project initialization, understand directory structure (`.specify/`, `specs/`, `history/`)
- **Test Workflow** (5 min): Run one simple command end-to-end to verify complete setup

**Learning Objectives**:
- Install Spec-Kit Plus framework successfully
- Configure AI tool (Claude Code or Gemini CLI) to use Spec-Kit Plus commands
- Understand project directory structure (Constitution, specs, ADRs, PHRs)
- Initialize calculator project environment
- Verify complete working setup

**Key Deliverable**: Working Spec-Kit Plus + AI tool installation + initialized calculator project + verified commands functional

---

### Lesson 2: Constitution Phase — Project-Wide Rules (Duration: 1.5 hours)

**Workflow Phase**: Constitution (establishes immutable rules before any feature work)

**Learning Objectives**:
- Create project Constitution for calculator project
- Define quality, testing, error handling standards
- Understand difference: Constitution (global rules) vs Specs (feature-specific requirements)
- See how Constitution guides all downstream phases

**Key Deliverable**: Complete Constitution (.specify/memory/constitution.md) with calculator project rules

---

### Lesson 3: Specify Phase — Writing Complete Specifications (Duration: 2-2.5 hours)

**Workflow Phase**: Specify (define WHAT system should do)

**Learning Objectives**:
- Write complete specification for calculator (5 operations: add, subtract, multiply, divide, power)
- Use SMART framework for acceptance criteria
- Define user stories and functional requirements
- Identify edge cases: division by zero, negative exponents, floating point precision, type preservation

**Key Deliverable**: Complete calculator specification (specs/calculator/spec.md) with all operations specified

---

### Lesson 4: Clarify Phase — Refining Specs with /sp.clarify (Duration: 1.5 hours)

**Workflow Phase**: Clarify (identify gaps, resolve ambiguities)

**Learning Objectives**:
- Use /sp.clarify to analyze calculator specification completeness
- Respond to AI questions about calculator-specific ambiguities (division by zero handling, type preservation rules, power(0,0) behavior)
- Experience Vertical Intelligence: orchestrator delegates to Clarification Subagent
- Iterate on specification based on identified gaps

**Key Deliverable**: Refined calculator specification with all ambiguities resolved

---

### Lesson 5: Plan Phase — Architecture Decisions (/sp.plan) (Duration: 2 hours)

**Workflow Phase**: Plan (design HOW system will be built)

**Learning Objectives**:
- Use /sp.plan to generate implementation plan for calculator
- Create ADRs for significant decisions (error handling strategy, type system choices, operation implementation patterns)
- Experience Vertical Intelligence: orchestrator delegates to Planning Subagent
- Understand plan structure: components, dependencies, sequencing
- Experience cascade: clear spec → clear plan

**Key Deliverable**: Implementation plan.md + 2-3 ADRs documenting architecture decisions

---

### Lesson 6: Tasks Phase — Decomposition (/sp.tasks) (Duration: 1.5 hours)

**Workflow Phase**: Tasks (break plan into atomic work units)

**Learning Objectives**:
- Use /sp.tasks to decompose calculator plan into tasks
- Experience Vertical Intelligence: orchestrator delegates to Tasks Subagent
- Understand task dependencies and ordering (e.g., basic operations before power, type system before operations)
- Trace lineage: Spec requirement → Plan component → Task units
- Recognize task quality flows from plan quality

**Key Deliverable**: tasks.md with atomic units (4-8 hour tasks) and dependency graph

---

### Lesson 7: Implement Phase — AI-Driven Code Generation (Duration: 2.5-3 hours)

**Workflow Phase**: Implement (generate code from specs/plan/tasks)

**Learning Objectives**:
- Use /sp.implement to orchestrate AI code generation for calculator
- Experience Vertical Intelligence: orchestrator delegates to Implementation Subagent
- Create PHRs tracking AI interactions and decisions
- Validate generated calculator code against acceptance criteria (division by zero, type preservation, edge cases)
- Provide feedback to refine AI output
- Recognize validation as core AIDD skill

**Key Deliverable**: Working calculator code + 8-10 PHRs + passing tests for all 5 operations

---

### Lesson 8 (Capstone): Full Workflow Integration (Duration: 3-4 hours)

**Consolidation**: Apply complete workflow to new project independently

**Learning Objectives**:
- Complete end-to-end Spec-Kit Plus workflow without step-by-step guidance
- Apply Vertical Intelligence pattern independently (collaborate with orchestrator)
- Consolidate all learning from Lessons 1-7
- Demonstrate mastery of spec-first development
- Prepare for Chapter 32 (team parallelization patterns)

**Project Options**:
- **Simple**: Temperature Converter (Celsius ↔ Fahrenheit ↔ Kelvin with validation)
- **Complex**: Unit Converter System (length, weight, temperature with extensible architecture)

**Key Deliverable**:
- Complete workflow artifacts (Constitution, Spec, Plan, Tasks, ADRs, PHRs)
- Working code with passing tests
- Reflection document: spec quality → code quality cascade observed

---

## Complexity Tier & Proficiency

**Overall Complexity**: Intermediate (Part 5)

**CEFR Levels**:
- Lessons 1-3: A2 (Recognition + Simple Application)
- Lessons 4-6: B1 (Application to unfamiliar problems)
- Lesson 7: B1-B2 (Integration and validation)
- Capstone: B2 (Design decisions, real-world constraints)

**Bloom's Taxonomy**:
- Lessons 1-3: Understand + Apply
- Lessons 4-6: Apply + Analyze
- Lesson 7: Apply + Analyze + Evaluate
- Capstone: Create

---

## Success Criteria

### SC-001: Natural Progression from Chapter 30
- Lesson 1 explicitly recaps 4 tools from Chapter 30 Lesson 6
- Students understand why Spec-Kit Plus was chosen (not Kiro, GitHub Spec-Kit, or Tessel)
- Transition from manual spec-writing (Chapter 30) to tooled workflow (Chapter 31) is clear
- Each lesson builds naturally from previous lessons and Chapter 30 foundation

### SC-002: Workflow Isomorphism Achievement
- Lesson structure mirrors actual Spec-Kit Plus workflow phases:
  - Lesson 1: Introduction/Setup → Lesson 2: Constitution → Lesson 3: Specify → Lesson 4: Clarify → Lesson 5: Plan → Lesson 6: Tasks → Lesson 7: Implement → Lesson 8: Capstone
- Students experience workflow by doing workflow (not reading about it)
- Each lesson deliverable becomes input to next lesson

### SC-002a: Complete SDD Loop Understanding
- Students can draw the 7-phase workflow from memory: Constitution → Specify → Clarify → Plan → Tasks → Implement → Validate
- Students can explain each phase's purpose and output artifact
- Students understand the cascade principle: upstream quality determines downstream quality
- Students know when each phase requires human decision-making vs AI assistance
- Students can identify which Vertical Intelligence subagent handles each phase
- Students understand documentation:
  - ADRs created explicitly via `/sp.adr` after plan phase for significant decisions
  - PHRs created **automatically** by system (can request explicitly if system misses)

### SC-003: ADR-001 Pedagogical Architecture Implemented
- Calculator serves as core practice project (5 operations: add, subtract, multiply, divide, power)
- 7 core lessons + 1 capstone consolidation = 8 total lessons
- Complexity tier: Intermediate (Tier 2) advancing to Advanced (Tier 2→3 transition)
- Proficiency target: B1→B2 (independent application → analysis/evaluation)

### SC-004: Preface AIDD Paradigm Demonstrated
- Students practice co-learning cycle: intent → AI generation → human validation → refinement
- Students recognize their role: architects and validators, not just coders
- Specifications shown as interface between human intent and AI execution
- All code generation includes: spec → AI prompt → generated code → validation steps

### SC-005: Cascade Effect Proven
- Students observe and articulate: spec quality determines plan quality determines code quality
- Clear demonstration: vague spec → vague plan → broken code vs clear spec → clear plan → working code
- Each lesson shows how upstream clarity prevents downstream rework

### SC-006: Complete Artifact Creation
- Students create all Spec-Kit Plus artifacts in practice:
  - Constitution (Lesson 2)
  - Specification (Lesson 3)
  - Refinement record (Lesson 4)
  - Plan (Lesson 5)
  - 2-3 ADRs (Lesson 5)
  - Tasks (Lesson 6)
  - Code + 8-10 PHRs (Lesson 7)
  - Complete portfolio (Lesson 8)

### SC-007: Complete SDD Loop Command Mastery
- **Command Knowledge**: Students can explain all 6 Spec-Kit Plus workflow commands:
  - `/sp.specify` - When: Start new feature; What: Launch specification workflow
  - `/sp.clarify` - When: After spec written; What: AI identifies gaps/ambiguities
  - `/sp.plan` - When: After spec approved; What: AI generates architecture plan
  - `/sp.adr <title>` - When: After plan (significant decisions); What: Document architectural reasoning
  - `/sp.tasks` - When: After plan/ADRs; What: AI decomposes into atomic tasks
  - `/sp.implement` - When: After tasks approved; What: AI generates code
- **PHR Understanding**: Students know PHRs are **auto-created**, not manual commands
  - Every `/sp.*` command automatically creates PHR for that session
  - Typical feature generates 8-10 PHRs automatically
  - Can request explicitly: "Record this as a PHR" if system misses something
- **Best Practice Workflow**: Students understand Constitution → Commit → Feature loop
  - Constitution created once per project
  - Constitution committed to git before feature work
  - Each feature follows: specify → clarify → plan → adr → tasks → implement → validate → commit
- **Storage Locations**: Students know where artifacts go:
  - Constitution → `.specify/memory/constitution.md`
  - Specs → `specs/<feature>/spec.md`
  - Plans → `specs/<feature>/plan.md`
  - Tasks → `specs/<feature>/tasks.md`
  - ADRs → `history/adr/`
  - PHRs → `history/prompts/<feature>/`
- **Documentation Judgment**: Students can identify when to manually create ADRs (not when to create PHRs - system handles that)
- **Usage Context**: All commands shown in actual Claude Code usage (not terminal hallucinations)
- **Phase Understanding**: Students know why phase order matters (cascade effect)
- **Human Role Clarity**: Students understand their role in each phase (intent/validation vs implementation)
- **Command Selection**: Students can independently choose correct command for current workflow stage

### SC-008: Foundation for Chapter 32
- Calculator project demonstrates single-component workflow mastery
- Students have complete workflow mastery before Chapter 32's multi-component parallelization patterns
- Skills from Chapter 31 directly transfer to Chapter 32's team coordination scenarios

---

## Assumptions

1. Students have completed Chapter 30 (understand SDD philosophy, compared 4 tools, know why Spec-Kit Plus was chosen)
2. Spec-Kit Plus is available via Claude Code (accessed through `/sp.*` commands)
3. Claude Code is installed and configured for running `/sp.*` commands
4. Students have AI companion access (Claude Code serves as orchestrator)
5. Students understand calculator domain (basic arithmetic operations)
6. Python 3.13+ is available for code generation/execution
7. Students are intermediate-level (Python basics, CLI familiar, comfortable with specifications from Chapter 30)

---

## Constraints & Non-Goals

### In Scope
- ✅ Spec-Kit Plus workflow (Specify → Clarify → Plan → Tasks → Implement)
- ✅ Vertical Intelligence architecture (orchestrator + specialized subagents)
- ✅ Calculator capstone project (5 operations: add, subtract, multiply, divide, power)
- ✅ SMART acceptance criteria writing
- ✅ Cascade effect demonstration
- ✅ AIDD paradigm teaching
- ✅ ADR and PHR creation in practice

### Out of Scope
- ❌ Python syntax teaching (students know basics)
- ❌ Database design (JSON/files only)
- ❌ Web UI (CLI only)
- ❌ AI prompt engineering
- ❌ Deployment/DevOps
- ❌ Advanced architecture patterns (Parts 9-13)

---

## Risks & Mitigations

**Risk 1**: Students don't understand why manual thinking (Lessons 1-3) matters before tools
- **Mitigation**: Explicitly contrast manual thinking vs. tool-assisted thinking; show tools amplify thinking, not replace it

**Risk 2**: Spec-Kit Plus workflow is complex and students get lost
- **Mitigation**: Each lesson focuses on ONE phase; previous lessons build foundation; capstone integrates all

**Risk 3**: Specification is one-time activity, not iterative
- **Mitigation**: Lesson 4 emphasizes iteration; show spec being refined multiple times

**Risk 4**: Students blindly accept AI-generated code without validation
- **Mitigation**: Lesson 7 emphasizes validation as core skill; every criterion must pass

**Risk 5**: Calculator project too simple and students underestimate SDD value
- **Mitigation**: Emphasize workflow mastery over domain complexity; capstone offers extension opportunities (temperature/unit converter)

---

## Chapter Learning Objectives

By end of Chapter 31, students will be able to:

1. Write SMART acceptance criteria that prevent misinterpretation
2. Understand Spec-Kit Plus project structure and cascade
3. Use `/sp.clarify` to refine and validate specifications
4. Generate implementation plans using `/sp.plan`
5. Decompose plans into atomic tasks using `/sp.tasks`
6. Implement and validate code using `/sp.implement`
7. Apply Vertical Intelligence pattern: collaborate with AI orchestrator that delegates to specialized subagents
8. Recognize how specification quality cascades through entire workflow
9. Practice co-learning with AI: intent → generation → validation → refinement
10. Complete end-to-end SDD project from vague idea to working code
11. Understand why specs are the new syntax in AI-native development

---

## Dependencies

**Prerequisites**:
- Chapter 1: AI-native basics (or Chapter 30 philosophy)
- Python 3.13+
- Basic CLI familiarity

**Required Tools**:
- Spec-Kit Plus (via Claude Code `/sp.*` commands)
- Claude Code
- AI companion (orchestrator)

**Book Level**:
- Prepares students for Chapter 32 capstone
- Reinforces SDD paradigm from Parts 4-5
- Demonstrates Preface philosophy in practice

---

## Acceptance Criteria for This Specification

Specification is complete and ready for planning when:

- [ ] All 7 lessons clearly described with learning objectives
- [ ] Each lesson has independent, testable deliverable
- [ ] Capstone project is concrete and achievable
- [ ] **Complete SDD Loop section included** with 7-phase workflow diagram
- [ ] **Best Practice Workflow Pattern added** (Constitution → Commit → Feature loop)
- [ ] **ADR checkpoint shown** (explicit `/sp.adr` after plan phase)
- [ ] **PHR auto-creation explained** (automatic for every `/sp.*` command, 8-10 per feature)
- [ ] **All 6 workflow commands explicitly documented**: `/sp.specify`, `/sp.clarify`, `/sp.plan`, `/sp.adr`, `/sp.tasks`, `/sp.implement`
- [ ] **Each command shows PHR auto-creation note** (specify, clarify, plan, tasks, implement all create PHRs)
- [ ] **Command usage checklist provided** for student self-assessment (18 questions total)
- [ ] **ADR judgment criteria explained** (when to create explicitly)
- [ ] **PHR understanding explained** (auto-created for each command session, can request if missed)
- [ ] **Storage locations documented** (Constitution, specs, plans, tasks, ADRs, PHRs)
- [ ] All hallucinated or incorrect commands are corrected
- [ ] Actual Spec-Kit Plus workflow is accurately described
- [ ] AIDD paradigm from Preface is reflected
- [ ] Cascade effect demonstrated in content (with examples)
- [ ] Manual thinking distinguished from tool usage
- [ ] Vertical Intelligence architecture clearly explained (with diagram)
- [ ] Calculator project fully specified (core practice)
- [ ] Capstone project options provided (temperature/unit converter)
- [ ] No forward references without explanation
- [ ] Complexity matches "Intermediate" tier for Part 5
- [ ] Each lesson shows when students think vs. when tools assist

---

