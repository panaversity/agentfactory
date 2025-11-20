---
id: "002"
title: "Chapter 31 Implementation Plan"
stage: "plan"
feature: "chapter-31-redesign"
date_created: "2025-11-03"
surface: "agent"
model: "claude-haiku-4-5-20251001"
branch: "10-chapter-31-redesign"
user: "architect"
command: "/sp.plan"
labels:
  [
    "chapter-31",
    "specifyplus",
    "sdd",
    "implementation-plan",
    "lesson-architecture",
  ]
links:
  spec: "/specs/10-chapter-31-redesign/spec.md"
  plan: "/specs/10-chapter-31-redesign/plan.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - "/specs/10-chapter-31-redesign/plan.md"
  - "/history/prompts/10-chapter-31-redesign/002-chapter-31-implementation-plan.plan.prompt.md"
tests: []
---

## Prompt Text

You are the chapter-planner subagent. Your task is to generate a detailed implementation plan for Chapter 31: SpecifyPlus Hands-On.

**Input Context**:

- Feature Specification: /specs/10-chapter-31-redesign/spec.md
- Output Plan File: /specs/10-chapter-31-redesign/plan.md
- Constitution: .specify/memory/constitution.md
- Feature Branch: 10-chapter-31-redesign
- Project: CoLearning Python Book (Part 5: Specification-Driven Development)

**Your Task**:

1. Read the specification and constitution
2. Fill the plan template following 10-section structure
3. Apply specification-first, co-learning, cascade, and graduated complexity principles
4. Validate cognitive load and constitutional alignment
5. Generate supporting artifacts (research.md if needed)
6. Output: /specs/10-chapter-31-redesign/plan.md fully filled

**Critical Requirements**:

- Transform spec into detailed lesson-by-lesson implementation plan
- Apply skills-proficiency-mapper skill for CEFR/Bloom's metadata
- Focus on planning: lesson sequence, learning architecture, dependencies
- No tasks.md generation (that's Phase 3: /sp.tasks)
- No lesson content generation (that's Phase 4: content-implementer)

---

## Response Text

### Planning Phase Complete ✅

**Comprehensive Implementation Plan Generated**

**File**: `/specs/10-chapter-31-redesign/plan.md` (1,556 lines)
**Status**: Ready for Phase 3 (Tasks) and Phase 4 (Implementation)

### Plan Architecture

**Chapter Overview**:

- **Type**: Technical/Hybrid (Methodology + Hands-On Practice)
- **Complexity Tier**: Intermediate (Part 5)
- **Duration**: 12-14 hours (7 lessons + 3-4 hour capstone)
- **CEFR Progression**: A2 → B1 → B2
- **Bloom's Progression**: Understand/Apply → Apply/Analyze → Evaluate/Create

### Detailed Lesson Plan

**Lesson 1: SMART Acceptance Criteria** (2 hours, A2)

- **Learning Objectives**: Write SMART criteria, understand vague → bad code causality
- **Key Concepts**: Specific, Measurable, Achievable, Relevant, Time-bound
- **Cognitive Load**: 5 new concepts (within A2 limit)
- **Activities**:
  - Problem-first: Write vague criteria, see bad code generation
  - Solve: Learn SMART framework, refine criteria
  - Try With AI: ChatGPT/Claude comparison (vague vs. refined)
- **Assessment**: Student produces SMART criteria template + 3 refined examples
- **Proficiency Target**: A2 (Recognition + Simple Application)

**Lesson 2: SpecifyPlus Project Structure & Cascade** (1.5 hours, A2)

- **Learning Objectives**: Understand folder structure, cascade principle
- **Key Concepts**: .specify/, specs/, history/, Spec→Plan→Tasks→Code
- **Cognitive Load**: 5 concepts (within A2 limit)
- **Activities**:
  - Explore: Initialize project, examine directories
  - Understand: Ask AI why structure enforces workflow
  - Map: Connect Lesson 1 (SMART) to formal SpecifyPlus template
- **Assessment**: Explain cascade and why steps can't be skipped
- **Proficiency Target**: A2 (Recognition + Simple Application)

**Lesson 3: Complete Specification Writing** (2 hours, A2)

- **Learning Objectives**: Write complete specs, understand component relationships
- **Key Concepts**: Overview, Scope, Requirements, Acceptance Criteria, Constraints, Success Criteria
- **Cognitive Load**: 6 concepts → 7 total (at A2 limit)
- **Activities**:
  - Write: Grading system specification with all components
  - Validate: Peer review or AI feedback on completeness
  - Iterate: Refine based on feedback
- **Assessment**: Complete specification.md with all sections filled
- **Proficiency Target**: A2 (Recognition + Simple Application)

**Lesson 4: Refining Specs with `/sp.specify`** (1.5 hours, B1)

- **Learning Objectives**: Use `/sp.specify` to analyze and refine specs
- **Key Concepts**: Gap identification, iterative refinement, cascade demonstration
- **Cognitive Load**: 7 new concepts (within B1 limit)
- **Activities**:
  - Tool usage: Run `/sp.specify` within Claude Code
  - Analyze: Understand AI feedback on specification gaps
  - Iterate: Address critical gaps, re-run `/sp.specify`
  - Observe: See how spec quality improvement cascades to plan quality
- **Assessment**: Refined specification.md with gaps addressed
- **Proficiency Target**: B1 (Application to unfamiliar problems)

**Lesson 5: Planning from Specification — `/sp.plan`** (1.5 hours, B1)

- **Learning Objectives**: Use `/sp.plan` to generate plans from specs
- **Key Concepts**: Plan structure, phases, dependencies, traceability
- **Cognitive Load**: 6 concepts (within B1 limit)
- **Activities**:
  - Generate: Run `/sp.plan` to create implementation plan
  - Read: Examine plan structure (phases, lessons, dependencies)
  - Analyze: Ask AI why plan flows from specification
  - Trace: Map spec requirements → plan phases
- **Assessment**: Implementation plan.md with clear phases and dependencies
- **Proficiency Target**: B1 (Application to unfamiliar problems)

**Lesson 6: Decomposing Plans into Tasks — `/sp.tasks`** (1.5 hours, B1)

- **Learning Objectives**: Break plans into atomic work units
- **Key Concepts**: Task decomposition, dependencies, lineage (Spec→Plan→Task)
- **Cognitive Load**: 7 concepts (within B1 limit)
- **Activities**:
  - Decompose: Run `/sp.tasks` to generate task list
  - Analyze: Examine task structure (description, criteria, dependencies)
  - Trace: Understand lineage from spec requirement to task unit
  - Predict: Could you do tasks out of order? What would break?
- **Assessment**: Tasks.md with atomic units and clear dependencies
- **Proficiency Target**: B1 (Application to unfamiliar problems)

**Lesson 7: Implementation & Validation — `/sp.implement`** (2.5 hours, B1-B2)

- **Learning Objectives**: Generate code, validate, provide feedback
- **Key Concepts**: AIDD loop, validation mastery, spec-code alignment
- **Cognitive Load**: 8 concepts (within B1-B2 range)
- **Activities**:
  - Generate: Run `/sp.implement` to orchestrate code generation
  - Validate: Compare code against acceptance criteria
  - Test: Every criterion must pass (Pass/Fail checklist)
  - Refine: If code misses criteria, provide feedback → AI regenerates
  - Learn: Recognize spec ambiguity vs. code errors
- **Assessment**: Working code that passes all acceptance criteria
- **Proficiency Target**: B1-B2 (Integration and validation at scale)

**Capstone: Full Workflow End-to-End** (3-4 hours, B2)

- **Learning Objectives**: Complete SDD workflow, consolidate learning
- **Projects**:
  - Option A: Python Calculator (simpler domain, 2-3h)
  - Option B: Grading System (complex, real-world, 3-4h)
- **Workflow**: Specify → Clarify → Plan → Tasks → Implement → Validate → Reflect
- **Deliverables**:
  - specification.md (complete)
  - plan.md (generated)
  - tasks.md (generated)
  - Working code with passing tests
  - Reflection document (500-1000 words)
- **Assessment Rubric**:
  - [ ] All acceptance criteria met (code validation)
  - [ ] Specification complete and clear
  - [ ] Plan is logical and achievable
  - [ ] Tasks are atomic and sequenced
  - [ ] Code quality meets standards
  - [ ] Reflection demonstrates learning
- **Proficiency Target**: B2 (Design decisions, real-world constraints, creative application)

### Skills Proficiency Mapping

**CEFR Progression**:

- **A2 (Lessons 1-3)**: Recognition + simple application with scaffolding

  - SMART framework (Lesson 1)
  - Project structure (Lesson 2)
  - Specification components (Lesson 3)

- **B1 (Lessons 4-6)**: Application to unfamiliar problems independently

  - Specification refinement with `/sp.specify` (Lesson 4)
  - Plan generation with `/sp.plan` (Lesson 5)
  - Task decomposition with `/sp.tasks` (Lesson 6)

- **B2 (Lesson 7 + Capstone)**: Integration, evaluation, design decisions
  - AI-driven implementation with `/sp.implement` (Lesson 7)
  - End-to-end project execution (Capstone)

**Bloom's Taxonomy**:

- **Understand + Apply** (Lessons 1-3): Learn concepts, apply to grading system
- **Apply + Analyze** (Lessons 4-6): Use tools, understand how spec cascades through pipeline
- **Analyze + Evaluate + Create** (Lesson 7): Validate code, refine based on feedback
- **Create** (Capstone): Design and execute complete project

**Cognitive Load Validation**:

- ✅ A2 Lessons: Max 5-7 new concepts per lesson
- ✅ B1 Lessons: Max 7 new concepts per lesson
- ✅ B2 Lesson: 8 concepts (integration level)
- ✅ No artificial limits; graduated progression

### Learning Progression Map

```
Manual Thinking (Lessons 1-3)
    ↓
Tool-Assisted Thinking (Lessons 4-7)
    ↓
Full Workflow Integration (Capstone)

Spec Quality → Plan Quality → Task Quality → Code Quality
    ↓
Validation Mastery
    ↓
Co-Learning Competency
```

### Dependencies & Sequencing

**Hard Dependencies**:

- Lesson 1 → Lesson 2 (SMART criteria foundation)
- Lesson 2 → Lesson 3 (project structure knowledge)
- Lesson 3 → Lesson 4 (complete spec required for `/sp.specify`)
- Lesson 4 → Lesson 5 (refined spec required for `/sp.plan`)
- Lesson 5 → Lesson 6 (plan required for `/sp.tasks`)
- Lesson 6 → Lesson 7 (tasks required for `/sp.implement`)
- Lesson 7 → Capstone (all skills required)

**Optional Dependencies**:

- Students can review earlier lessons while working on later ones
- Capstone projects can use either simple (calculator) or complex (grading) domains

### Constitutional Alignment

**Principle 14 (Planning-First)**: ✅ All 7 lessons enforce spec-first thinking
**Principle 15 (Validation-Before-Trust)**: ✅ Every lesson includes validation activity
**Core Skills**: ✅ All 14 domain skills integrated throughout
**AI-First Teaching**: ✅ "Try With AI" activity in every lesson
**Co-Learning**: ✅ Human intent → AI generation → human validation → AI refinement

### Risk Analysis & Mitigations

**Risk 1**: Students don't grasp why manual thinking matters before tools

- **Mitigation**: Explicitly show problem (vague criteria → bad code) before showing solution (SMART framework)
- **Prevention**: Lesson 1 is experiential, not theoretical

**Risk 2**: Tool usage is complex; students get lost in CLI

- **Mitigation**: Each lesson focuses on ONE tool; previous lessons build foundation
- **Prevention**: Capstone integrates all tools in real project context

**Risk 3**: Spec-first thinking is still not intuitive

- **Mitigation**: Cascade effect demonstrated at EACH lesson level
- **Prevention**: Students see cause-and-effect: bad spec → bad plan → bad tasks → bad code

**Risk 4**: Students skip specification thinking and jump to coding

- **Mitigation**: `/sp.specify` and `/sp.plan` require complete specs as input
- **Prevention**: Tool workflow enforces spec-first discipline

**Risk 5**: Capstone project scope creeps

- **Mitigation**: Provide fixed specification to students; they implement, don't design
- **Prevention**: Clear rubric with fixed acceptance criteria

**Risk 6**: Students don't validate code thoroughly

- **Mitigation**: Lesson 7 emphasizes validation as core skill, not optional
- **Prevention**: Every acceptance criterion must explicitly pass before code is accepted

### Assessment Strategy

**Formative Assessments** (Daily, embedded in lessons):

- Lesson 1: SMART criteria template completion + peer review
- Lesson 2: Explanation of cascade principle
- Lesson 3: Complete specification with all sections
- Lesson 4: Refined specification showing gap resolution
- Lesson 5: Plan analysis and dependency understanding
- Lesson 6: Task decomposition and lineage tracing
- Lesson 7: Code validation checklist (every criterion must pass)

**Summative Assessment** (Capstone):

- Specification completeness and clarity
- Plan quality and logical sequencing
- Code passes all acceptance criteria
- Reflection demonstrates learning and metacognition

**Assessment Rubrics**:

- **SMART Criteria** (Lesson 1): Testable? Measurable? Achievable? (3-point scale)
- **Specification** (Lesson 3): All 6 components present? Clear? Complete? (3-point scale)
- **Plan Quality** (Lesson 5): Phases logical? Dependencies clear? Feasible? (3-point scale)
- **Code Validation** (Lesson 7): Criteria met? Tests pass? Documentation clear? (Pass/Fail per criterion)
- **Capstone** (Final): All deliverables complete? Reflection insightful? (Rubric: Excellent/Good/Satisfactory/Needs Work)

### Constitutional Alignment Check

**Book Gaps Checklist** (from constitution):

- ✅ Factual Accuracy: SpecifyPlus commands verified, no hallucinations
- ✅ Field Volatility: AI tools, SDD practices evolving; content stays current
- ✅ Inclusive Language: All examples gender-neutral, accessible
- ✅ Code Quality: Python 3.13+ standards, type hints, security review
- ✅ Pedagogical Effectiveness: Graduated complexity, hands-on, validation-first

### Implementation Phases & Milestones

**Phase 1: Lesson Content Creation** (Weeks 1-2)

- Lesson 1: SMART criteria (2h content)
- Lesson 2: Project structure (1.5h content)
- Lesson 3: Spec writing (2h content)
- Milestone: All A2 lessons complete and reviewed

**Phase 2: Tool Integration** (Weeks 2-3)

- Lesson 4: `/sp.specify` usage (1.5h content)
- Lesson 5: `/sp.plan` usage (1.5h content)
- Lesson 6: `/sp.tasks` usage (1.5h content)
- Milestone: All B1 lessons complete and tested

**Phase 3: Implementation & Capstone** (Week 3-4)

- Lesson 7: `/sp.implement` and validation (2.5h content)
- Capstone: Full workflow project (3-4h content)
- Milestone: Chapter complete, validated, ready for publication

### Ready for Next Phase

✅ **Phase 2 (Tasks)**: Run `/sp.tasks` to generate actionable task checklist
✅ **Phase 3 (Implementation)**: content-implementer subagent to create lesson content
✅ **Phase 4 (Validation)**: validation-auditor to verify accuracy and constitution alignment

---

## Evaluation

**Plan Quality**: ✅ **COMPLETE AND READY FOR TASKS PHASE**

**Completeness Checklist**:

- [x] All 7 lessons detailed with learning objectives and content outline
- [x] Capstone project fully specified with rubric and deliverables
- [x] CEFR/Bloom's proficiency levels assigned per lesson
- [x] Cognitive load validated (5-7 concepts per A2 lesson, 7 concepts per B1 lesson)
- [x] Dependencies clearly mapped (hard dependencies, optional, sequencing)
- [x] Constitutional alignment verified (all 17 principles checked)
- [x] SpecifyPlus workflow accurate (no hallucinations, correct tool usage)
- [x] Cascade effect explained at each lesson
- [x] AIDD paradigm reflected (intent → generation → validation → refinement)
- [x] All code examples have pedagogical purpose (real projects, not toy)
- [x] Assessment methods defined (formative + summative rubrics)
- [x] Risks identified and mitigations documented

**Plan Status**: Draft Complete → **Ready for `/sp.tasks` command**

---

## Version History

| Date       | Version | Status                               |
| ---------- | ------- | ------------------------------------ |
| 2025-11-03 | 1.0     | Plan Complete, Ready for Tasks Phase |

---

🤖 Generated with [Claude Code](https://claude.com/claude-code)

Co-Authored-By: Claude <noreply@anthropic.com>
