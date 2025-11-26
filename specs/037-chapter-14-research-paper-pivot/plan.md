# Implementation Plan: Chapter 14 — Master Spec-Kit Plus

**Branch**: `037-chapter-14-research-paper-pivot` | **Date**: 2025-11-26 | **Spec**: [spec.md](./spec.md)
**Type**: Educational Content (Book Chapter)
**Part**: 4 (SDD-RI Fundamentals)
**Chapter**: 14
**Proficiency Tier**: A2-B1

---

## Core Principle

**Each lesson teaches ONE Spec-Kit Plus command/concept.**

Research paper is the practice vehicle — simple, requires only markdown + AI, doesn't distract from learning Spec-Kit Plus.

---

## Lesson Structure (12 Lessons)

| # | Title | Spec-Kit Plus Focus | Duration | Bloom's |
|---|-------|---------------------|----------|---------|
| 01 | Spec-Kit Plus Foundation | What it is, why use it, architecture | 20 min | Understand |
| 02 | Installation and Setup | Configure tools, create project folder | 30 min | Apply |
| 03 | Constitution Phase | `/sp.constitution` command | 30 min | Apply |
| 04 | Specify Phase | `/sp.specify` command | 45 min | Create |
| 05 | Clarify Phase | `/sp.clarify` command | 30 min | Analyze |
| 06 | Plan Phase | `/sp.plan` command | 45 min | Create |
| 07 | Tasks Phase | `/sp.tasks` command | 30 min | Apply |
| 08 | Implement Phase | `/sp.implement` command | 60 min | Apply |
| 09 | Designing Reusable Intelligence | P+Q+P skill creation | 45 min | Create |
| 10 | Brownfield Adoption | Adding to existing projects | 30 min | Apply |
| 11 | Capstone | Skill reuse, acceleration | 60 min | Evaluate |
| 12 | Quiz | Assessment | 20 min | Evaluate |

**Total**: ~7.5 hours

---

## Lesson-by-Lesson Plan

### Lesson 01: Spec-Kit Plus Foundation

**Learning Objective**: Understand what Spec-Kit Plus is and why it implements SDD-RI.

**Spec-Kit Plus Concepts**:
1. What is Spec-Kit Plus (SDD-RI framework)
2. Horizontal Intelligence (ADRs, PHRs)
3. Vertical Intelligence (Skills, Subagents)
4. The workflow phases overview

**Key Teaching Points**:
- Spec-Kit Plus captures intelligence, not just delivers code
- Two outputs: working artifact + reusable intelligence
- Preview of all commands they'll learn

**Try With AI**: Explore P+Q+P concept, discuss intelligence compounding

---

### Lesson 02: Installation and Setup

**Learning Objective**: Configure Claude Code/Gemini CLI for Spec-Kit Plus and create project structure.

**Spec-Kit Plus Concepts**:
1. Spec-Kit Plus directory structure (`.specify/`, `.claude/`)
2. Constitution location and purpose
3. Specs folder organization
4. Skills folder location

**Key Teaching Points**:
- Create project folder for research paper practice
- Understand where artifacts go
- Verify AI companion can access project

**Try With AI**: Create project folder structure, verify setup

---

### Lesson 03: Constitution Phase

**Learning Objective**: Use `/sp.constitution` to define project-wide quality standards.

**Spec-Kit Plus Concepts**:
1. What is a constitution (project-wide rules)
2. `/sp.constitution` command syntax
3. Constitution sections (principles, constraints)
4. How constitution guides all phases

**Practice Vehicle**: Create constitution for research paper project
- Define quality standards (accuracy, clarity)
- Set constraints (length, audience)
- Establish success criteria

**Try With AI**: Run `/sp.constitution` and create paper constitution

---

### Lesson 04: Specify Phase

**Learning Objective**: Use `/sp.specify` to write clear specifications with success criteria.

**Spec-Kit Plus Concepts**:
1. What is a specification (WHAT not HOW)
2. `/sp.specify` command syntax
3. Required sections: Intent, Constraints, Success Evals, Non-Goals
4. SMART criteria for success evals

**Practice Vehicle**: Write specification for research paper
- Define topic and scope
- Set measurable success criteria
- List what paper will NOT cover

**Try With AI**: Run `/sp.specify` for paper specification

---

### Lesson 05: Clarify Phase

**Learning Objective**: Use `/sp.clarify` to identify gaps and refine specifications.

**Spec-Kit Plus Concepts**:
1. Why clarification matters (vague specs → vague output)
2. `/sp.clarify` command syntax
3. How clarify identifies edge cases
4. Iterating until spec is complete

**Practice Vehicle**: Refine paper specification
- AI identifies ambiguous requirements
- Student resolves gaps
- Spec becomes more precise

**Try With AI**: Run `/sp.clarify` to improve paper spec

---

### Lesson 06: Plan Phase

**Learning Objective**: Use `/sp.plan` to create implementation strategy.

**Spec-Kit Plus Concepts**:
1. What is a plan (HOW to build WHAT)
2. `/sp.plan` command syntax
3. Plan sections: approach, components, dependencies
4. How plan connects spec to tasks

**Practice Vehicle**: Create plan for research paper
- Break paper into sections
- Identify research approach
- Define component dependencies

**Try With AI**: Run `/sp.plan` to create paper plan

---

### Lesson 07: Tasks Phase

**Learning Objective**: Use `/sp.tasks` to decompose plan into atomic work units.

**Spec-Kit Plus Concepts**:
1. What are tasks (atomic, testable units)
2. `/sp.tasks` command syntax
3. Task structure: description, acceptance criteria
4. Checkpoint-driven execution

**Practice Vehicle**: Break paper into atomic tasks
- Each section becomes tasks
- Define completion criteria per task
- Order tasks by dependency

**Try With AI**: Run `/sp.tasks` to generate paper tasks

---

### Lesson 08: Implement Phase

**Learning Objective**: Use `/sp.implement` to execute tasks with AI collaboration.

**Spec-Kit Plus Concepts**:
1. What implementation means in SDD-RI
2. `/sp.implement` command syntax
3. Iterative execution with checkpoints
4. Validation against success criteria

**Practice Vehicle**: Implement first paper section
- Execute task with AI
- Validate against spec criteria
- Iterate until complete

**Try With AI**: Run `/sp.implement` to write first section

---

### Lesson 09: Designing Reusable Intelligence

**Learning Objective**: Create skills using P+Q+P framework.

**Spec-Kit Plus Concepts**:
1. What makes a pattern reusable
2. P+Q+P framework (Persona + Questions + Principles)
3. Skill file structure (`.claude/skills/`)
4. When to create skill vs subagent

**Practice Vehicle**: Create `section-writer` skill
- Identify recurring pattern from paper writing
- Define Persona (who thinks about this)
- Define Questions (what to consider)
- Define Principles (decision criteria)

**Try With AI**: Create P+Q+P skill from paper experience

---

### Lesson 10: Brownfield Adoption

**Learning Objective**: Apply Spec-Kit Plus to existing projects safely.

**Spec-Kit Plus Concepts**:
1. Greenfield vs brownfield adoption
2. Starting with constitution for existing projects
3. Incremental specification of existing features
4. Retrofitting intelligence capture

**Practice Vehicle**: Conceptual — examples from real projects

**Try With AI**: Plan Spec-Kit Plus adoption for hypothetical project

---

### Lesson 11: Capstone — Intelligence Acceleration

**Learning Objective**: Demonstrate that skill reuse accelerates work.

**Spec-Kit Plus Concepts**:
1. Skill invocation in `/sp.implement`
2. Measuring acceleration (time, effort, quality)
3. Intelligence compounding across projects
4. Portfolio value of skills

**Practice Vehicle**: Write second paper section using `section-writer` skill
- Invoke skill created in Lesson 09
- Compare effort to first section (Lesson 08)
- Articulate acceleration

**Try With AI**: Use skill for second section, reflect on acceleration

---

### Lesson 12: Chapter Quiz

**Assessment Focus**:
- Command recognition (which command for which purpose)
- Artifact matching (which file for which phase)
- P+Q+P component identification
- Acceleration reasoning

---

## Key Corrections from Previous Plan

| Previous Error | Corrected Approach |
|----------------|-------------------|
| Lessons taught research paper writing | Lessons teach Spec-Kit Plus commands |
| Research paper was the learning objective | Research paper is practice vehicle |
| Focus on paper structure and content | Focus on command syntax and workflow |
| "Try With AI" practiced writing | "Try With AI" practices Spec-Kit Plus commands |

---

## Validation Checklist

Before implementation, verify each lesson:
- [ ] Primary focus is Spec-Kit Plus command/concept
- [ ] Research paper is used only as practice example
- [ ] No teaching of writing methodology
- [ ] "Try With AI" runs Spec-Kit Plus command
- [ ] Cognitive load ≤7 concepts (A2-B1)

---

## Next Steps

1. **Generate tasks.md** via `/sp.tasks` — Break lessons into implementable tasks
2. **Implement lessons** via `/sp.implement` — Rewrite all lesson files
3. **Validate** — Ensure each lesson focuses on Spec-Kit Plus
4. **Update README** — Remove video generation references
5. **Generate summaries** — Create .summary.md files
