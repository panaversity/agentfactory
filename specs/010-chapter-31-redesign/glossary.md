---
title: "Glossary: Spec-Kit Plus Key Terms"
chapter: 31
created: "2025-11-05"
status: "Complete"
---

# Glossary: Spec-Kit Plus Key Terms

Reference document for Chapter 31 terminology. All terms are listed alphabetically.

---

## A

**Acceptance Criteria**
Clear, testable conditions that define when a feature is complete and meets the specification. Acceptance criteria must be SMART (Specific, Measurable, Achievable, Relevant, Time-bound) so that AI can build directly from them without ambiguity.

**ADR (Architectural Decision Record)**
A document capturing a significant architectural decision, the alternatives considered, tradeoffs between them, and the rationale for the chosen approach. ADRs are created explicitly via `/sp.adr <title>` when decisions have long-term impact, multiple viable alternatives, or will be questioned by future developers.

**AIDD Paradigm (AI-Driven Development)**
A development methodology where human intent drives clear specifications, AI agents generate significant portions of the implementation from those specifications, and humans validate the outputs before deployment. The human role is strategic thinking and verification; the AI role is implementation generation.

**Atomic Tasks**
Work units sized for 1-2 hours of focused effort with clear acceptance criteria, dependencies, and expected outputs. Atomic tasks enable parallel execution and clear progress tracking without requiring context-switching overhead.

---

## C

**Cascade Effect**
The principle that quality flows downhill from specification to code: clear specifications produce clear plans, clear plans produce clear tasks, and clear tasks enable correct code generation. Conversely, vague specifications cascade into vague plans, confused tasks, and broken code.

**Checkpoint Pattern**
The deliberate workflow structure where an agent completes a work unit, a human reviews and approves it, and only then does the workflow continue to the next phase. This pattern prevents autonomous execution and ensures human remains the decision-maker at every milestone.

**Clarify Phase**
The workflow phase following initial specification where the AI orchestrator analyzes the spec for gaps, ambiguities, missing edge cases, and conflicting requirements. The human answers clarifying questions to refine and validate the specification before proceeding to planning.

---

## H

**Horizontal Intelligence**
The organizational knowledge captured in ADRs (Architectural Decision Records) and PHRs (Prompt History Records) that accumulate over time and across team members. Horizontal intelligence documents the reasoning, tradeoffs, and lessons learned from past decisions and interactions, enabling future team members to understand and extend systems.

---

## I

**Implementation Subagent**
A specialized AI collaborator delegated by the orchestrator to handle code generation, testing, and implementation details following the approved spec, plan, and tasks. The implementation subagent applies domain expertise in code patterns and testing strategies.

---

## P

**PHR (Prompt History Record)**
An automatically-created document capturing a significant interaction between a human and AI, including the context, the prompt given, the AI's response, and key decisions made. PHRs are stored in `history/prompts/<feature>/` and accumulate automatically as development work progresses (typically 8-10 per feature).

**Planning Subagent**
A specialized AI collaborator delegated by the orchestrator to handle architectural design, component decomposition, dependency analysis, and sequencing. The planning subagent applies domain expertise in architecture patterns and system design.

---

## S

**Spec-Kit Plus**
An AI-native Spec-Driven Development toolkit built on the GitHub Spec-Kit foundation, enhanced with ADRs (for decision documentation), PHRs (for collaboration history), and Vertical Intelligence (orchestrator + specialized subagents). Spec-Kit Plus is the methodology taught in this book.

**Specification Phase**
The initial workflow phase where the human collaborates with the AI orchestrator to define WHAT needs to be built: requirements, acceptance criteria, scope, and constraints. The specification is the authoritative document that downstream phases follow.

**Specification Subagent**
A specialized AI collaborator delegated by the orchestrator to help structure specifications, suggest missing requirements, identify conflicting constraints, and validate specification completeness. The specification subagent applies domain expertise in requirements engineering.

---

## T

**Tasks Phase**
The workflow phase where the AI orchestrator decomposes the approved plan into atomic work units (1-2 hour tasks) with clear acceptance criteria, dependencies, and outputs. Tasks provide the concrete checklist for implementation.

---

## V

**Validation-First**
A core principle that you never accept or deploy code without reading, understanding, and validating it first. Validation includes syntax checking, type safety, security review, functionality testing, and spec alignment—not trusting any AI-generated output at face value.

**Vertical Intelligence**
The AI collaboration architecture in Spec-Kit Plus where the human works with an AI orchestrator (main collaborator) that delegates specialized tasks to domain-expert subagents. The structure is: Human → Orchestrator → (Specification, Planning, Implementation, Validation Subagents). Vertical Intelligence specializes AI capabilities while maintaining human control.

---

## W

**Workflow Phases (SDD Loop)**
The complete seven-phase Spec-Driven Development workflow: 1) Constitution (project rules), 2) Specify (define what), 3) Clarify (refine spec), 4) Plan (design how), 5) ADR (document decisions), 6) Tasks (decompose work), 7) Implement (generate code). Each phase builds on the previous; skipping phases breaks the cascade effect.

---

## Additional Context

### When These Terms Appear in Chapter 31

- **Lessons 1-2**: Introduction to Spec-Kit Plus concepts; AIDD Paradigm, Checkpoint Pattern
- **Lesson 3**: Specification Phase; Acceptance Criteria, Specification Subagent
- **Lesson 4**: Clarify Phase; cascade effect demonstrated
- **Lesson 5**: Planning Subagent, ADR, Vertical Intelligence
- **Lesson 6**: Atomic Tasks, Tasks Phase
- **Lesson 7**: Implementation Subagent, Validation-First
- **Lessons 3-8**: PHR (auto-created), Horizontal Intelligence accumulation
- **Throughout**: Workflow Phases, Cascade Effect, Human Control

### Key Relationships

**Spec-Kit Plus** = Toolkit implementing AIDD Paradigm with Vertical Intelligence + Horizontal Intelligence
**AIDD Paradigm** = Intent (human) → Generation (AI) → Validation (human) → Decision (human)
**Vertical Intelligence** = Orchestrator delegates to specialized subagents for each workflow phase
**Horizontal Intelligence** = ADRs + PHRs accumulate as organizational knowledge over time
**Cascade Effect** = Specification quality determines all downstream quality (plan, tasks, code)
**Checkpoint Pattern** = Prevents autonomous execution; human approves each phase before proceeding
**Atomic Tasks** = Enables parallel execution and clear progress; 1-2 hour work units
**Validation-First** = Core safety principle; never trust AI output without verification

---

**Created**: 2025-11-05
**For**: Chapter 31 — Spec-Kit Plus Hands-On
**Reference**: Project Constitution v3.0.0
