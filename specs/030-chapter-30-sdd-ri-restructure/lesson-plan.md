# Chapter 30 Lesson Plan: SDD-RI Unified Methodology (8 Lessons)

**Chapter**: 30 — Understanding Spec-Driven Development
**Part**: 5 — Spec-Driven Development
**Audience Tier**: B1 (Intermediate)
**Total Lessons**: 8 (expanded from 5)
**Pedagogical Pattern**: Specification-First Teaching (meta: teaching specs by writing specs)

---

## Pedagogical Architecture

### Overall Chapter Arc

**Foundation (L1-2)**: Manual practice with specifications
**Application (L3-5)**: AI collaboration on spec creation
**Integration (L6-7)**: Reusability patterns (RI introduction)
**Validation (L8)**: Organizational patterns and tradeoffs

### Cognitive Load Distribution

**B1 Tier Limit**: 7-10 concepts per section, 1-2 concepts per lesson

**Total Chapter Concepts**: 10 concepts
- **SDD Concepts (L1-5)**: 6 concepts
- **RI Concepts (L6-8)**: 4 concepts

**Load per lesson**: 1.25 concepts average (well within B1 capacity)

### Stage Progression Mapping

- **Stage 1 (Manual)**: L1-2 (build mental model of specifications)
- **Stage 2 (AI Collaboration)**: L3-5 (write specs with AI)
- **Stage 3 (Intelligence Design)**: L6-7 (create reusable components)
- **Stage 3→4 Bridge**: L8 (organizational patterns, prepare for capstone)

---

## Lesson-by-Lesson Structure

### Lesson 1: Why Specifications Matter

**Stage**: 1 (Manual Foundation)
**Teaching Modality**: Problem-based discovery (show cost of vagueness)
**Duration**: 45-60 minutes

**Learning Objectives**:
- Diagnose vagueness in requirements and quantify real costs (time, rework, debugging)
- Articulate the difference between intent and implementation
- Explain why AI agents need specifications (not just prompts)

**Concepts Introduced** (2):
1. **Vagueness Cost**: Time waste, rework cycles, technical debt from unclear requirements
2. **Intent vs Implementation**: WHAT the system does vs HOW it does it

**Content Structure**:
- **Problem**: Real-world case study of project failure due to vague requirements
- **Analysis**: Break down where vagueness entered, how it propagated
- **Principle**: Specifications capture intent before implementation
- **Practice**: Identify vagueness in sample requirement documents

**AI Role**: Minimal (student practices manual analysis)

**Cognitive Load**: 2 concepts, simple examples, heavy scaffolding = **Low (A2-B1)**

**Prerequisites**:
- Completed Part 4 (Python Fundamentals)
- Understands basic software development concepts
- Has experienced working with AI coding assistants (from Parts 1-3)

**Outputs**:
- Student can identify vague vs precise requirements
- Student understands why "just prompt the AI" fails for complex projects

---

### Lesson 2: Anatomy of a Specification

**Stage**: 1 (Manual Foundation)
**Teaching Modality**: Direct teaching with annotated examples
**Duration**: 60-75 minutes

**Learning Objectives**:
- Identify the core sections of a production-ready spec (Intent, Constraints, Acceptance Criteria)
- Write a simple spec.md file manually (no AI yet)
- Evaluate spec quality using evals-first principle

**Concepts Introduced** (2):
1. **Spec.md Structure**: Intent, Requirements, Constraints, Non-Goals, Acceptance Tests
2. **Evals-First Principle**: Define success criteria before implementation

**Content Structure**:
- **Anatomy Walkthrough**: Section-by-section breakdown of spec.md
- **Example**: Annotated production spec from real project
- **Practice**: Students write spec for simple feature (todo: add dark mode toggle)
- **Validation**: Self-check against spec quality checklist

**AI Role**: None (manual practice builds foundation)

**Cognitive Load**: 2 concepts, structured template, clear examples = **Low-Medium (B1)**

**Prerequisites**:
- Lesson 1 completed (understands intent vs implementation)

**Outputs**:
- Student's first manually-written spec.md file
- Understanding of spec structure that enables L3 AI collaboration

---

### Lesson 3: Writing Specs with AI (Co-Learning Partnership)

**Stage**: 2 (AI Collaboration)
**Teaching Modality**: Three Roles Framework (AI as Teacher/Student/Co-Worker)
**Duration**: 60-75 minutes

**Learning Objectives**:
- Use AI to generate initial specification draft
- Refine AI-generated specs through iterative dialogue
- Demonstrate AI as Teacher (suggests missing requirements), Student (learns constraints), Co-Worker (converges on solution)

**Concepts Introduced** (1):
1. **AI as Spec Partner**: Iterative refinement through collaboration

**Content Structure**:
- **Role 1 — AI as Teacher**: AI suggests edge cases student didn't consider
- **Role 2 — AI as Student**: Student corrects AI's generic assumptions with domain constraints
- **Role 3 — AI as Co-Worker**: Convergence loop produces better spec than either alone
- **Practice**: Write spec for intermediate feature (API rate limiter)

**AI Role**: Active collaboration (Three Roles demonstrated)

**Cognitive Load**: 1 concept (AI collaboration pattern), builds on L1-2 foundation = **Medium (B1)**

**Prerequisites**:
- Lesson 2 completed (can write basic spec manually)
- Understands spec structure well enough to evaluate AI output

**Outputs**:
- Collaboratively-refined spec.md
- Experience with bidirectional learning (student ↔ AI)

**Constitutional Requirement**: MUST demonstrate all three roles (Teacher/Student/Co-Worker)

---

### Lesson 4: From Spec to Code (Specification Primacy)

**Stage**: 2 (AI Collaboration)
**Teaching Modality**: Specification-first workflow
**Duration**: 60-75 minutes

**Learning Objectives**:
- Execute specification-first workflow (spec.md → prompts → code → validation)
- Validate that generated code matches specification intent
- Debug specification-code misalignment issues

**Concepts Introduced** (1):
1. **Specification Primacy**: Code is OUTPUT of specification, not INPUT

**Content Structure**:
- **Workflow**: spec.md → AI prompt → code generation → acceptance test validation
- **Practice**: Generate code from L3 spec, validate alignment
- **Debugging**: Intentional spec-code mismatch, student diagnoses and fixes
- **Validation Loop**: Iterate spec → code → test until acceptance criteria met

**AI Role**: Code generation from spec (student validates)

**Cognitive Load**: 1 concept (workflow sequence), practical application = **Medium (B1)**

**Prerequisites**:
- Lesson 3 completed (has refined spec.md)
- Understands acceptance criteria (from L2)

**Outputs**:
- Working code generated from specification
- Validation that spec → code workflow produces correct implementation

---

### Lesson 5: Spec Quality & Tooling Landscape

**Stage**: 2 (AI Collaboration)
**Teaching Modality**: Evaluation framework + tool comparison
**Duration**: 60-75 minutes

**Learning Objectives**:
- Diagnose over-specification (too prescriptive) vs under-specification (too vague)
- Evaluate tooling options (SpecKit Plus, PRD tools, custom templates)
- Choose appropriate tooling for team scale and constraints

**Concepts Introduced** (2):
1. **Spec Quality Spectrum**: Over-specification (removes judgment) ↔ Under-specification (too vague)
2. **Tooling Tradeoffs**: Formal tools vs lightweight templates

**Content Structure**:
- **Problem**: Examples of over/under-specified specs and their failure modes
- **Framework**: Quality evaluation checklist (clarity, testability, completeness)
- **Tooling Survey**: SpecKit Plus, Tessl, PRD Agents, custom markdown
- **Decision Framework**: When to use formal tools vs simple templates
- **Practice**: Evaluate sample specs, choose tooling for hypothetical team

**AI Role**: Comparison analysis, tradeoff discussion

**Cognitive Load**: 2 concepts, decision framework provided = **Medium (B1)**

**Prerequisites**:
- Lessons 1-4 completed (has written and validated specs)
- Experience with spec → code workflow

**Outputs**:
- Spec quality evaluation capability
- Informed tooling choice for student's context
- **Transition setup**: "You can write great specs. Next: making them reusable."

---

### Lesson 6: Introduction to Reusable Intelligence (RI)

**Stage**: 3 (Intelligence Design)
**Teaching Modality**: Discovery-based (problem → pattern → solution)
**Duration**: 60-75 minutes

**Learning Objectives**:
- Recognize when patterns recur enough to justify encoding as reusable intelligence
- Distinguish between Skills (2-4 decisions) and Subagents (5+ decisions)
- Understand the Persona + Questions + Principles activation pattern

**Concepts Introduced** (2):
1. **Reusable Intelligence (RI)**: Skills, Subagents, Intelligence Libraries
2. **Encoding Decision Framework**: When to make specs reusable (frequency, complexity, organizational value)

**Content Structure**:
- **Problem**: "I'm writing similar specs repeatedly" (motivation for RI)
- **Solution**: Skills and Subagents encode recurring patterns
- **Decision Framework**:
  - **Frequency**: Pattern recurs 2+ times → consider encoding
  - **Complexity**: 2-4 decisions → Skill, 5+ decisions → Subagent
  - **Organizational value**: Applies across 3+ projects → worth encoding
- **Examples**: Show real skill (from `.claude/skills/`) and subagent (from `.claude/agents/`)
- **Practice**: Analyze student's past work, identify RI opportunities

**AI Role**: Co-designer (help identify patterns worth encoding)

**Cognitive Load**: 2 concepts (RI definition + decision framework), builds on L1-5 foundation = **Medium (B1)**

**Prerequisites**:
- Lessons 1-5 completed (mastered single-spec SDD)
- Has written 2+ specs (can recognize recurring patterns)

**Outputs**:
- Understanding of Skills vs Subagents distinction
- List of RI opportunities from student's projects

**Pedagogical Note**: "Aha moment" lesson — reveals full SDD-RI methodology

---

### Lesson 7: Designing Skills and Subagents (Reasoning Activation)

**Stage**: 3 (Intelligence Design)
**Teaching Modality**: Co-design workshop (student creates reusable component)
**Duration**: 75-90 minutes

**Learning Objectives**:
- Apply Persona + Questions + Principles pattern to activate reasoning (not prediction) mode
- Create a functional Skill document for recurring workflow
- Understand how reasoning-activated prompts produce context-specific outputs (not generic patterns)

**Concepts Introduced** (1):
1. **Persona + Questions + Principles (P+Q+P)**: Reasoning activation pattern

**Content Structure**:
- **Theory**: Reasoning mode vs Prediction mode in AI agents
- **Pattern**: P+Q+P structure breakdown:
  - **Persona**: "Think like X architect analyzing Y problem" (cognitive stance)
  - **Questions**: Context-specific analysis prompts (not generic checklists)
  - **Principles**: Decision frameworks (not rigid rules)
- **Workshop**: Student creates Skill for their recurring pattern
  - Choose pattern from L6 analysis
  - Write Persona (appropriate cognitive stance)
  - Write Questions (force context-specific reasoning)
  - Write Principles (decision frameworks with examples)
- **Validation**: Test skill on new context, verify reasoning (not template-filling)

**AI Role**: Co-designer (help refine P+Q+P structure)

**Cognitive Load**: 1 concept (P+Q+P pattern), hands-on practice, scaffolded workshop = **Medium-High (B1-B2)**

**Prerequisites**:
- Lesson 6 completed (understands Skills vs Subagents)
- Has identified RI opportunity to encode

**Outputs**:
- Student's first Skill document (following `.claude/skills/` format)
- Understanding of reasoning activation vs prediction mode

**Constitutional Reference**: This lesson teaches the core pattern from constitution.md Section 0 (Persona + Questions + Principles)

---

### Lesson 8: Organizational Patterns & Governance (Constitutions)

**Stage**: 3 → 4 Bridge (prepare for spec-driven capstone)
**Teaching Modality**: Case study analysis + organizational design
**Duration**: 60-75 minutes

**Learning Objectives**:
- Understand how Constitutions embed governance and coordination rules
- Evaluate spec-as-source vision (Tessl and similar frameworks)
- Apply organizational patterns: Intelligence Libraries, Agent Teams, Manager Patterns

**Concepts Introduced** (1):
1. **Constitutions**: Domain-level governance documents that encode principles, not rules

**Content Structure**:
- **Case Study**: This project's constitution.md as example
  - Show how 7 principles guide decisions across all chapters
  - Demonstrate reasoning frameworks vs rigid rules
  - Explain how constitution enables agent coordination without micromanagement
- **Intelligence Libraries**: Organizational knowledge accumulation
  - Skills library (`.claude/skills/`)
  - Subagent library (`.claude/agents/`)
  - Spec pattern library (`specs/`)
- **Spec-as-Source Vision**: Tessl and future of specifications
- **Practice**: Design simple constitution for student's team/project

**AI Role**: Governance advisor (help design constitution)

**Cognitive Load**: 1 concept (Constitutions), synthesis of L1-7 concepts = **Medium (B1)**

**Prerequisites**:
- Lessons 1-7 completed (full SDD-RI understanding)

**Outputs**:
- Understanding of organizational RI patterns
- Simple constitution document for student's context
- **Preparation for Part 6**: Ready to build agents using accumulated intelligence

**Forward Reference**: "Next chapters: SpecKit Plus hands-on (Ch 31), AI Orchestration (Ch 32), Tessl integration (Ch 33)"

---

## Cognitive Load Validation

### Per-Lesson Concept Count

| Lesson | New Concepts | Cumulative | Load Tier |
|--------|--------------|------------|-----------|
| L1 | 2 (Vagueness, Intent vs Impl) | 2 | Low (A2-B1) |
| L2 | 2 (Spec structure, Evals-first) | 4 | Low-Med (B1) |
| L3 | 1 (AI collaboration) | 5 | Medium (B1) |
| L4 | 1 (Spec primacy workflow) | 6 | Medium (B1) |
| L5 | 2 (Quality spectrum, Tooling) | 8 | Medium (B1) |
| L6 | 2 (RI, Encoding decision) | 10 | Medium (B1) |
| L7 | 1 (P+Q+P pattern) | 11 | Med-High (B1-B2) |
| L8 | 1 (Constitutions) | 12 | Medium (B1) |

**Average**: 1.5 concepts per lesson
**B1 Compliance**: ✅ All lessons within 1-2 concept limit

### Progressive Disclosure Strategy

**Foundation (L1-2)**: 4 concepts, simple examples, heavy scaffolding
**Application (L3-5)**: 4 concepts, builds on foundation, moderate scaffolding
**Integration (L6-7)**: 3 concepts, synthesis required, guided discovery
**Validation (L8)**: 1 concept, organizational synthesis, minimal scaffolding

**Load increases gradually** as students build competence ✅

---

## Anti-Convergence Validation

**Previous Chapter 29**: CPython and GIL
- **Modality**: Technical deep dive, direct teaching
- **Examples**: Python internals, GIL mechanics

**Chapter 30**: Spec-Driven Development
- **Modality**: Specification-first teaching (meta: teaching specs by writing specs)
- **Examples**: Production-relevant features (not toy apps)

**Variation Achieved**: ✅ Different teaching modality, different example domain

---

## Prerequisites and Dependencies

### Incoming Prerequisites (from Parts 1-4)

**From Part 1-3**:
- AI tool literacy (Claude Code, prompting basics)
- Markdown proficiency
- Git/GitHub workflows

**From Part 4 (Python)**:
- Programming fundamentals
- Experience writing code with AI assistance
- Understanding of testing and validation

### Outgoing Prerequisites (for Part 6+)

**Chapter 30 enables Part 6** (AI Native Software Development):
- Understanding of specifications as executable contracts
- Skills and Subagents concepts (RI foundation)
- Persona + Questions + Principles activation pattern
- Intelligence accumulation mindset

**Specific Part 6 dependencies**:
- Chapter 34 (Introduction to AI Agents) assumes L6-7 (Skills/Subagents)
- Chapter 42 (Test-Driven Agent Development) assumes L2 (Evals-first)
- Chapter 44 (Building Effective Agents) assumes L7 (P+Q+P pattern)

---

## Teaching Modality Breakdown

| Lesson | Primary Modality | Secondary Modality |
|--------|------------------|-------------------|
| L1 | Problem-based discovery | Case study analysis |
| L2 | Direct teaching | Annotated examples |
| L3 | Three Roles Framework | Iterative refinement |
| L4 | Specification-first workflow | Debugging practice |
| L5 | Evaluation framework | Tool comparison |
| L6 | Discovery-based | Pattern recognition |
| L7 | Co-design workshop | Hands-on creation |
| L8 | Case study analysis | Organizational design |

**Variety**: ✅ No two consecutive lessons use identical modality

---

## Success Criteria (Lesson Plan Quality)

### Pedagogical Arc Completeness
- ✅ Foundation phase (L1-2): Manual practice establishes mental models
- ✅ Application phase (L3-5): AI collaboration on specs
- ✅ Integration phase (L6-7): Reusability patterns introduced
- ✅ Validation phase (L8): Organizational synthesis

### Constitutional Compliance
- ✅ Principle 2 (Progressive Complexity): B1 tier, 1-2 concepts per lesson
- ✅ Principle 4 (Coherent Structure): Clear pedagogical arc
- ✅ Principle 5 (Intelligence Accumulation): Prepares for Part 6
- ✅ Principle 6 (Anti-Convergence): Varied teaching modalities
- ✅ Section IIa (4-Stage Framework): Stages 1-3 mapped to lessons

### Stage Transition Validation
- ✅ Stage 1→2 (L2→3): Students can write basic spec manually before AI collaboration
- ✅ Stage 2→3 (L5→6): Students recognize recurring patterns before encoding
- ✅ Stage 3→4 (L8→Ch31): Students ready for capstone orchestration

### Learning Objectives Clarity
- ✅ Every lesson has 3 measurable learning objectives
- ✅ Objectives build progressively (each lesson enables next)
- ✅ Final lesson synthesizes all prior objectives

---

## Implementation Notes

### Existing Content (L1-5)
**Current state**: 5 lessons exist covering SDD basics
**Required changes**: Minor updates only
- Add L5 forward reference: "Next: making specs reusable"
- Validate examples are production-relevant (not toy apps)
- Ensure cognitive load within limits (audit concept count)

### New Content (L6-8)
**Net-new implementation required**:
- L6: Full lesson creation (intro to RI)
- L7: Full lesson creation (P+Q+P workshop)
- L8: Full lesson creation (Constitutions case study)

### Validation Checklist (Per Lesson)
- [ ] Learning objectives measurable and achievable
- [ ] Concept count within B1 limit (1-2 per lesson)
- [ ] Teaching modality identified and varied from adjacent lessons
- [ ] AI role appropriate for stage (Manual → Collaboration → Co-design)
- [ ] Prerequisites satisfied by prior lessons
- [ ] Outputs feed into next lesson or part

---

## References

**Constitutional Grounding**:
- `.specify/memory/constitution.md` v6.0.0
  - Section IIa: 4-Stage Teaching Framework
  - All 7 Foundational Principles

**Domain Structure**:
- `specs/book/chapter-index.md` — Part 5 structure
- Chapter 30 existing content: `book-source/docs/05-Spec-Driven-Development/30-specification-driven-development-fundamentals/`

**Skills Library** (referenced in L6-7):
- `.claude/skills/` — Example skills for student analysis

**Strategic Decision**:
- `specs/030-chapter-30-sdd-ri-restructure/adr-001-expand-chapter-30-to-include-ri.md` — ADR documenting expansion rationale

---

**This lesson plan operationalizes the strategic decision to teach SDD-RI as unified methodology through 8 progressively-structured lessons that respect B1 cognitive load limits while preparing students for Part 6+ agent building.**
