---
title: "Redesign Chapter 31: SpecifyPlus Hands-On"
feature: "chapter-31-redesign"
part: 5
chapter: 31
date_created: "2025-11-03"
status: "Draft"
authors: ["Panaversity Team"]
complexity_tier: "Intermediate"
---

# Specification: Redesign Chapter 31 — SpecifyPlus Hands-On

## Executive Summary

Chapter 31 teaches specification-driven development (SDD) through hands-on practice with SpecifyPlus, the CLI tool that operationalizes the SDD workflow. The chapter is being **redesigned to eliminate hallucinated commands, correct the actual SpecifyPlus workflow, and center on AIDD (AI-Driven Development) paradigm** taught in the preface.

Current issues: Lessons 2, 4, 5 reference incorrect terminal command usage. Chapter structure is unclear about when students think manually vs. when tools assist.

**Redesigned chapter will teach the actual SpecifyPlus workflow while maintaining hands-on, experiential learning: 7 lessons + capstone integration with `/sp.implement`.**

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

### User Story 2 — Student Understands SpecifyPlus Project Structure (Priority: P1)

**Who**: Developer setting up first SpecifyPlus project

**What they do**:
1. Initialize a SpecifyPlus project using `pip install specifyplus` or `uvx specifyplus init project-name`
2. Explore the generated folder structure (`.specify/`, `specs/`, `history/`)
3. Understand WHY the structure enforces a workflow (Spec → Plan → Tasks)
4. Recognize templates and constitution as guides for thinking

**Why this priority**: Sets up the mental model for the entire workflow. Without understanding structure, later steps are mechanical rather than meaningful.

**Independent Test**: Student can explain the relationship between directories and describe what forces the cascade from Spec → Plan → Tasks.

**Acceptance Scenarios**:

1. **Given** an initialized SpecifyPlus project, **When** student explores the directory structure, **Then** they can explain: purpose of each directory, why three files (spec.md, plan.md, tasks.md) exist in sequence, why you can't skip directly to tasks

2. **Given** a specification and companion's explanation of the structure, **When** student reads the spec template, **Then** they understand which sections correspond to requirements vs. implementation

---

### User Story 3 — Student Experiences Specification Cascade (Priority: P2)

**Who**: Developer ready to see SDD workflow in action

**What they do**:
1. Complete a specification for a real project (grading system or calculator)
2. Use `/sp.specify` within Claude Code to get AI feedback on spec completeness
3. Iterate on spec based on feedback
4. Run `/sp.plan` to generate implementation plan from specification
5. See how plan automatically breaks into phases, lessons, dependencies
6. Run `/sp.tasks` to decompose plan into atomic units
7. Witness cascade: clear spec → clear plan → clear tasks

**Why this priority**: Demonstrates the core value of SDD: spec quality determines everything downstream.

**Independent Test**: Student can describe how improving their specification improved the quality of the generated plan and tasks.

**Acceptance Scenarios**:

1. **Given** a completed specification, **When** student uses `/sp.specify` within Claude Code, **Then** they receive feedback on gaps, ambiguities, missing assumptions

2. **Given** feedback, **When** student iterates on specification, **Then** they can articulate which gaps they filled and why each matters

3. **Given** an improved specification, **When** student uses `/sp.plan`, **Then** the generated plan has clear phases, lessons, and dependencies

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

### User Story 5 — Student Builds Capstone Project End-to-End (Priority: P3)

**Who**: Developer consolidating learning through complete project

**What they do**:
1. Start with new project idea (Python calculator or grading system)
2. Write vague 1-sentence description
3. Run full SpecifyPlus workflow: specify → clarify → plan → tasks → implement
4. Validate final implementation against original specification
5. Reflect on how specification quality determined implementation quality

**Why this priority**: Capstone that consolidates all learning. Students need to experience the full pipeline once to be ready for Chapter 32.

**Independent Test**: Student produces working software artifact that passes all acceptance criteria defined in their own specification.

**Acceptance Scenarios**:

1. **Given** a vague project idea, **When** student completes full SpecifyPlus workflow, **Then** they have: specification, plan, tasks, working code, and passing tests

2. **Given** completed project, **When** student reviews code against spec, **Then** all acceptance criteria pass

3. **Given** any bugs or discrepancies, **When** student traces back to specification, **Then** they discover the root cause was in spec ambiguity, not code error

---

### Edge Cases

- What happens if a specification has conflicting requirements? → Students learn to resolve conflicts in spec phase, not implementation
- What happens if acceptance criteria are impossible? → Students learn to validate feasibility during planning
- What happens if AI misunderstands the spec? → Students learn how to refine spec and re-prompt, demonstrating that clarity is iterative

---

## Requirements

### Functional Requirements

**FR-001**: Chapter must teach the ACTUAL SpecifyPlus workflow
- ✅ `/sp.specify` is used within Claude Code to refine specs
- ✅ `/sp.plan` generates plans from specs
- ✅ `/sp.tasks` decomposes plans into tasks
- ✅ `/sp.implement` orchestrates implementation

**FR-002**: Chapter must eliminate all hallucinated or incorrectly-presented commands
- ✅ Clarify that `pip install specifyplus` or `uvx specifyplus init` creates project structure
- ✅ Show correct usage of `/sp.*` commands within development environments (Claude Code)
- ✅ Remove misleading references to terminal command execution

**FR-003**: Chapter structure must reflect actual SpecifyPlus workflow
- **Correct workflow**: Constitution → Specify → Clarify → Plan → ADR → Tasks → Implement → Analyze
- **Chapter focus**: Specify → Clarify → Plan → Tasks → Implement (core loop for students)

**FR-004**: Chapter must teach when students think MANUALLY vs. when tools assist
- **Manual thinking** (Lessons 1-3): Coaching clarity, writing SMART criteria
- **Tool-assisted thinking** (Lessons 4-7): Using `/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement`

**FR-005**: Each lesson must be independently testable
- Lesson 1: Student produces SMART criteria template
- Lesson 2: Student explains project structure and cascade
- Lesson 3: Student writes complete specification
- Lesson 4: Student iterates spec using `/sp.specify`
- Lesson 5: Student reads and understands generated plan
- Lesson 6: Student decomposes plan into tasks
- Lesson 7: Student runs `/sp.implement`, validates code

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

### Lesson 1: SMART Acceptance Criteria (Duration: 2 hours)

**Prerequisite**: Chapter 30 philosophy (or Chapter 1 basics)

**Learning Objectives**:
- Write acceptance criteria using SMART framework
- Understand how vague criteria produce useless code
- Experience the problem-solution cycle

**Key Deliverable**: SMART criteria template + refined criteria for 3 features

---

### Lesson 2: SpecifyPlus Project Structure & Cascade (Duration: 1.5 hours)

**Prerequisite**: Lesson 1

**Learning Objectives**:
- Understand folder structure (`.specify/`, `specs/`, `history/`)
- Recognize how structure enforces Spec → Plan → Tasks workflow
- Know why steps can't be skipped

**Key Deliverable**: Partially completed spec.md with Overview, Scope, Success Criteria

---

### Lesson 3: Complete Specification Writing (Duration: 2 hours)

**Prerequisite**: Lessons 1-2

**Learning Objectives**:
- Write complete specifications with all required components
- Understand relationship between components
- Recognize what makes a specification "ready for planning"

**Key Deliverable**: Complete specification.md with all sections

---

### Lesson 4: Refining Specs with /sp.specify (Duration: 1.5 hours)

**Prerequisite**: Lessons 1-3

**Learning Objectives**:
- Use `/sp.specify` within Claude Code to analyze spec
- Understand AI feedback on completeness
- Iterate on specification based on gaps identified
- Experience iterative nature of clarity

**Key Deliverable**: Refined specification.md with gaps addressed

---

### Lesson 5: Planning from Specification (/sp.plan) (Duration: 1.5 hours)

**Prerequisite**: Lessons 1-4

**Learning Objectives**:
- Use `/sp.plan` to generate implementation plan
- Understand how plan flows from specification
- Recognize dependencies and sequencing
- Experience cascade: clear spec → clear plan

**Key Deliverable**: Implementation plan.md with phases and dependencies

---

### Lesson 6: Decomposing Plan into Tasks (/sp.tasks) (Duration: 1.5 hours)

**Prerequisite**: Lessons 1-5

**Learning Objectives**:
- Use `/sp.tasks` to break plan into atomic work units
- Understand task dependencies
- Recognize that task quality flows from plan quality
- Trace lineage: Spec requirement → Plan phase → Task units

**Key Deliverable**: Tasks.md with atomic units and dependencies

---

### Lesson 7: Implementation with /sp.implement (Duration: 2-3 hours)

**Prerequisite**: Lessons 1-6

**Learning Objectives**:
- Use `/sp.implement` to orchestrate AI-driven code generation
- Validate generated code against acceptance criteria
- Provide feedback to refine AI output
- Recognize validation as core skill

**Key Deliverable**: Working code that passes all acceptance criteria

---

### Capstone Project: Full Workflow (Duration: 3-4 hours)

**Prerequisite**: All lessons 1-7

**Learning Objectives**:
- Complete full SpecifyPlus workflow end-to-end
- Consolidate all learning in single project
- Experience the value of SDD

**Projects**: Python Calculator (simple) or Grading System (complex)

**Key Deliverable**:
- Complete specification.md, plan.md, tasks.md
- Working code with passing tests
- Reflection document

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

### SC-001: Correct SpecifyPlus Workflow
- All commands shown in correct usage context
- No hallucinated terminal commands
- `/sp.*` commands shown within Claude Code

### SC-002: Cascade Effect Demonstrated
- Students can articulate: Spec quality → Plan quality → Task quality → Code quality
- Students experience improvement cycle

### SC-003: AIDD Paradigm Taught
- Students understand co-learning cycle
- Students recognize their role: intent + evaluation + feedback

### SC-004: Every Lesson Independently Testable
- Each lesson produces tangible deliverable
- Completion doesn't depend on previous lesson's perfection

### SC-005: Hands-On, Experiential Learning
- Every concept demonstrated through concrete project
- Students experience problems before learning solutions

### SC-006: Book Alignment
- Content reflects preface philosophy
- Prepares students for Chapter 32 capstone

### SC-007: Production-Grade Accuracy
- Workflow matches actual SpecifyPlus documentation
- All commands and tools accurately described

---

## Assumptions

1. Students have completed Chapter 30 or know Part 1 basics
2. SpecifyPlus is installed (`pip install specifyplus` or `uvx`)
3. Claude Code is available for running `/sp.*` commands
4. Students have AI companion access
5. Students understand grading system domain (rubrics, submissions)
6. Python 3.13+ is available
7. Students are intermediate-level (Python basics, CLI familiar)

---

## Constraints & Non-Goals

### In Scope
- ✅ SpecifyPlus workflow (Specify → Clarify → Plan → Tasks → Implement)
- ✅ SMART acceptance criteria writing
- ✅ Cascade effect demonstration
- ✅ AIDD paradigm teaching
- ✅ Two real projects

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

**Risk 2**: SpecifyPlus workflow is complex and students get lost
- **Mitigation**: Each lesson focuses on ONE tool; previous lessons build foundation; capstone integrates all

**Risk 3**: Specification is one-time activity, not iterative
- **Mitigation**: Lesson 4 emphasizes iteration; show spec being refined multiple times

**Risk 4**: Students blindly accept AI-generated code without validation
- **Mitigation**: Lesson 7 emphasizes validation as core skill; every criterion must pass

**Risk 5**: Grading system project too complex
- **Mitigation**: Provide complete specification to students; they implement against it; capstone adds complexity

---

## Chapter Learning Objectives

By end of Chapter 31, students will be able to:

1. Write SMART acceptance criteria that prevent misinterpretation
2. Understand SpecifyPlus project structure and cascade
3. Use `/sp.specify` to refine and validate specifications
4. Generate implementation plans using `/sp.plan`
5. Decompose plans into atomic tasks using `/sp.tasks`
6. Implement and validate code using `/sp.implement`
7. Recognize how specification quality cascades through entire workflow
8. Practice co-learning with AI: intent → generation → validation → refinement
9. Complete end-to-end SDD project from vague idea to working code
10. Understand why specs are the new syntax in AI-native development

---

## Dependencies

**Prerequisites**:
- Chapter 1: AI-native basics (or Chapter 30 philosophy)
- Python 3.13+
- Basic CLI familiarity

**Required Tools**:
- SpecifyPlus (pip or uvx)
- Claude Code
- AI companion

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
- [ ] All hallucinated or incorrect commands are corrected
- [ ] Actual SpecifyPlus workflow is accurately described
- [ ] AIDD paradigm from Preface is reflected
- [ ] Cascade effect demonstrated in content
- [ ] Manual thinking distinguished from tool usage
- [ ] Two real projects specified (calculator + grading system option)
- [ ] No forward references without explanation
- [ ] Complexity matches "Intermediate" tier for Part 5
- [ ] Each lesson shows when students think vs. when tools assist

---

