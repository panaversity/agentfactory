---
title: "Chapter 31: Spec-Kit Plus Hands-On"
chapter: 31
part: 5
status: "In Development"
duration: "13-15 hours (with capstone)"
skills_proficiency:
  - name: "Specification Writing"
    cefr_level: "A2-B1"
    category: "Technical"
  - name: "Acceptance Criteria (SMART Framework)"
    cefr_level: "A2"
    category: "Technical"
  - name: "Cascade Effect Thinking"
    cefr_level: "A2-B1"
    category: "Conceptual"
  - name: "Tool-Assisted Workflow"
    cefr_level: "B1"
    category: "Technical"
  - name: "Validation Mastery"
    cefr_level: "B1-B2"
    category: "Technical"
  - name: "AI Collaboration & Feedback"
    cefr_level: "B1-B2"
    category: "Soft"
generation_metadata:
  generated_by: "Claude Code (lesson-implementation specialist)"
  source_spec: "specs/010-chapter-31-redesign/spec.md"
  created: "2025-11-05"
  last_modified: "2025-11-05"
  git_author: "Claude Code"
  workflow: "Spec-Driven Development (SDD)"
  version: "1.0.0"
---

# Chapter 31: Spec-Kit Plus Hands-On

**Putting the Spec-Kit Plus Methodology Into Practice**

Welcome to Chapter 31, where you'll **practice the complete Spec-Kit Plus workflow** you learned about in Chapter 30. Instead of comparing tools, you'll now **use Spec-Kit Plus to build a real project** from vague idea to validated code.

This chapter is the bridge between understanding *why* specification-driven development works (Chapter 30) and building *real-world projects with teams* (Chapter 32). You'll experience the complete workflow, understand **why each phase matters**, and recognize how **specification quality cascades through every downstream step**.

---

## What You'll Learn

By the end of this chapter, you will be able to:

1. **Write SMART acceptance criteria** that prevent AI misinterpretation (Lesson 1)
2. **Understand Spec-Kit Plus structure** and why it enforces Spec → Plan → Tasks cascade (Lesson 2)
3. **Write complete specifications** with all required components (Lesson 3)
4. **Refine specifications** using `/sp.specify` and AI feedback (Lesson 4)
5. **Generate implementation plans** using `/sp.plan` from clear specifications (Lesson 5)
6. **Decompose plans into atomic tasks** using `/sp.tasks` with checkpoint control (Lesson 6)
7. **Implement and validate code** using `/sp.implement` with rigorous acceptance criteria verification (Lesson 7)
8. **Complete end-to-end Spec-Kit Plus workflow** on a real project capstone (Lesson 8)

**Most Importantly**: You'll experience the cascade effect directly. Clear specs enable clear plans. Clear plans enable clear tasks. Clear tasks enable AI to generate correct code. **This is the entire value of Spec-Kit Plus.**

---

## Chapter at a Glance

| Lesson | Focus | Duration | Proficiency |
|--------|-------|----------|-------------|
| **1** | SMART Acceptance Criteria & Clarity | 1.5 hours | A2 |
| **2** | Spec-Kit Plus Project Structure | 1.5 hours | A2 |
| **3** | Complete Specification Writing | 2 hours | A2 |
| **4** | Refining Specs with `/sp.specify` | 1.5 hours | B1 |
| **5** | Planning with `/sp.plan` | 2 hours | B1 |
| **6** | Task Decomposition with `/sp.tasks` | 1.5 hours | B1 |
| **7** | Implementation & Validation with `/sp.implement` | 2.5-3 hours | B1-B2 |
| **8** | Capstone Project (Full Workflow) | 3-4 hours | B2 |
| **Total** | Complete Workflow Mastery | 15-18 hours | A2→B2 |

---

## Core Concept: The Cascade Effect

The entire chapter rests on one insight:

> **Specification quality determines everything downstream.**

```
Clear Spec → Clear Plan → Clear Tasks → Working Code
Vague Spec → Vague Plan → Confused Tasks → Broken Code
```

You'll experience this cascade directly. In Lesson 1, you'll see what happens when you give vague requirements to AI. In Lessons 3-7, you'll see how each phase builds on the previous. By the capstone, you'll understand viscerally why **"write specs first" is not just philosophy—it's practical necessity.**

---

## Prerequisites

- **Chapter 30** (understanding SDD philosophy and why Spec-Kit Plus was chosen)
- **Python 3.13+** installed on your machine
- **Basic CLI familiarity** (navigating directories, running commands)
- **Access to Claude Code** or **Gemini CLI** (your AI orchestrator for Spec-Kit Plus commands)
- **Text editor** (VS Code, PyCharm, or your preference)

If you haven't completed Chapter 30, **start there first**. This chapter assumes you understand:
- Why AI-native development requires clear specifications
- What four SDD tools exist (Kiro, Spec-Kit, **Spec-Kit Plus**, Tessel)
- Why Spec-Kit Plus was chosen for this book

---

## Lesson Breakdown

### Lesson 1: SMART Acceptance Criteria
**Duration**: 1.5 hours | **Goal**: Understand clarity as your superpower

You'll start with a **problem-first approach**: experience what happens when you give vague requirements to AI (AI builds wrong code). Then learn the **SMART framework** to convert vague language into testable criteria. You'll practice on real examples, see the difference in AI output quality.

**Key Deliverable**: Template of SMART criteria for 3 features.

### Lesson 2: Spec-Kit Plus Project Structure
**Duration**: 1.5 hours | **Goal**: Understand why the structure enforces quality

Initialize a Spec-Kit Plus project locally. Explore the folders (`.specify/`, `specs/`, `history/`). Understand **why** the structure forces Spec → Plan → Tasks progression. Recognize that you can't skip directly from idea to tasks—the structure prevents it (a good thing!).

**Key Deliverable**: Initialized project; explanation of cascade.

### Lesson 3: Complete Specification Writing
**Duration**: 2 hours | **Goal**: Write production-ready specifications

Use the provided spec template to write **complete, unambiguous specifications**. All six components: Overview, Scope, Requirements, Acceptance Criteria, Constraints, Success Criteria. You'll choose between a simple calculator or more complex grading system. Work will be reviewed by peers.

**Key Deliverable**: Complete specification document (`spec.md`) with all components.

### Lesson 4: Refining Specs with `/sp.specify`
**Duration**: 1.5 hours | **Goal**: Iterate toward perfect clarity

Use the `/sp.specify` command within Claude Code to analyze your spec for gaps and ambiguities. AI will ask clarifying questions specific to your project (e.g., "How should the calculator handle division by zero?"). You'll refine your spec based on feedback, then re-run `/sp.specify` to verify improvements.

**Key Deliverable**: Refined specification addressing 3+ gaps identified by `/sp.specify`.

### Lesson 5: Planning from Specification
**Duration**: 2 hours | **Goal**: See how spec quality produces plan quality

Run `/sp.plan` on your refined spec. AI generates a detailed implementation plan with phases, dependencies, milestones. You'll analyze the plan: identify critical path, understand why Phase B depends on Phase A, trace requirements to plan phases. **Experience the cascade**: Did a clearer spec produce a clearer plan?

**Key Deliverable**: Implementation plan (`plan.md`) with dependency analysis.

### Lesson 6: Task Decomposition with `/sp.tasks`
**Duration**: 1.5 hours | **Goal**: Break plans into atomic, manageable work units

Run `/sp.tasks` on your plan. AI decomposes phases into tasks (1-2 hour completions) with acceptance criteria. You'll identify task dependencies, create a traceability matrix (spec requirement → plan phase → task unit), and understand the **checkpoint pattern**: agent completes Phase 1 → YOU review → YOU commit → YOU request Phase 2 (YOU stay in control).

**Key Deliverable**: Task checklist (`tasks.md`) with traceability and dependency mapping.

### Lesson 7: Implementation & Validation
**Duration**: 2.5-3 hours | **Goal**: Master validation as a professional skill

Run `/sp.implement` to orchestrate AI code generation. But here's the critical part: **you validate every line**. Read code without running. Understand intent. Comprehension check. Run tests. Verify acceptance criteria (create matrix: criterion → pass/fail). Security review. If failures, trace back to root cause (AI error or spec ambiguity?), refine spec if needed, regenerate.

**This lesson is where you prove the cascade effect.** Clear specs enabled AI to generate correct code. Poor validation catches problems early.

**Key Deliverable**: Working code + validation report + any refinement cycles.

### Lesson 8: Capstone Project
**Duration**: 3-4 hours | **Goal**: Complete workflow mastery, no scaffolding

Choose a project (simple calculator or complex grading system). Execute the **entire Spec-Kit Plus workflow independently**: write vague 1-sentence idea → write spec → run `/sp.specify` → refine → `/sp.plan` → `/sp.tasks` → `/sp.implement` → validate → reflect.

You'll produce:
- Complete specification (all components, refined)
- Implementation plan (phases, dependencies)
- Task checklist (atomic units with criteria)
- Working code (all tests pass, validation matrix 100%)
- Reflection document (how did spec quality determine code quality?)

By the end, you'll have a portfolio piece demonstrating complete SDD mastery and understanding why **specifications are the new syntax in AI-native development.**

---

## Workflows & Key Concepts

### Human Control & Checkpoint Pattern

**Critical principle**: YOU are in control. AI orchestrator (Claude Code/Gemini CLI) is your collaborator, not an autonomous agent.

**Checkpoint pattern**:
- Agent completes Phase 1 tasks → Returns to YOU
- YOU review code, run tests, verify spec
- YOU commit changes
- YOU decide: proceed to Phase 2 OR refine and regenerate

**Never**: "Generate all code" → AI runs autonomously → Returns 10,000 lines → Overwhelm

This mirrors professional practice. Never deploy unvalidated AI code.

### Cascade Effect: The Heart of SDD

```
Input: Vague requirement ("build a calculator")
    ↓
Lesson 1-3: Clear specification (all components, SMART criteria)
    ↓
Lesson 4: Refined spec (/sp.specify removes ambiguities)
    ↓
Lesson 5: Clear plan (/sp.plan - phases, dependencies)
    ↓
Lesson 6: Atomic tasks (/sp.tasks - testable units)
    ↓
Lesson 7: Correct code (/sp.implement - validation proves it)
Output: Working software where EVERY acceptance criterion passes
```

If code fails validation, ask: "Where's the gap?" Trace back:
- Failed criterion → Which task? → Which plan phase? → Which spec requirement?
- Often, you'll find spec ambiguity (not AI error)
- Refine spec → regenerate → re-validate (don't manually patch)

---

## The Calculator Project

### Your Practice Vehicle

Throughout Lessons 1-7, you'll use a **calculator project** as your hands-on practice:

**Core Features** (5 operations):
- **Add**: `2 + 3 = 5`
- **Subtract**: `5 - 3 = 2`
- **Multiply**: `4 * 3 = 12`
- **Divide**: `6 / 3 = 2` (but what about divide by zero?)
- **Power**: `2 ** 3 = 8` (and negative exponents)

**Why calculator?**
- Simple domain (everyone understands arithmetic)
- Rich enough to teach all SDD lessons
- Edge cases force you to think about specifications (what happens when divide by zero?)
- Teaches you that "simple" projects still need clear specs

### Capstone Project Options

For Lesson 8, you have two paths:

**Option A: Python Calculator (Simpler)**
- Build on lessons 1-7 practice
- Add/subtract/multiply/divide/power operations
- Add history tracking, error handling
- 2-3 hours implementation
- **Target**: Students consolidating core workflow

**Option B: Grading System (More Complex)**
- Multi-user system (teacher/student roles)
- Track submissions, compute weighted grades, generate reports
- Data persistence (JSON or file-based)
- 3-4 hours implementation
- **Target**: Students wanting additional challenge

Either path teaches complete SDD workflow. Start with calculator if you want to focus on *methodology*, choose grading system if you want *additional complexity*.

---

## Part 5 Context

Chapter 31 is part of **Part 5: Spec-Kit Plus Methodology** (Chapters 30-32).

| Chapter | Focus |
|---------|-------|
| **30** (Prior) | "Why Spec-Kit Plus?" - Compare 4 SDD tools, understand philosophy |
| **31** (This) | "How to use Spec-Kit Plus?" - Hands-on workflow, full practice |
| **32** (Next) | "Advanced patterns?" - Team workflows, complex specs, real-world scenarios |

### Connection to Chapter 30

Chapter 30 taught you **why clarity matters**. You compared four tools and concluded Spec-Kit Plus is best because it combines:
- **Horizontal Intelligence**: ADRs (architectural decisions) + PHRs (prompt history) capture organizational knowledge
- **Vertical Intelligence**: AI orchestrator delegates to specialized subagents per phase

Chapter 31 puts this into practice. You'll actually use Spec-Kit Plus, see how it enforces the cascade, understand viscerally why it was chosen.

### Bridge to Chapter 32

Chapter 31 is the **foundation for team scaling**. Once you master single-person workflow (Spec → Plan → Tasks → Code → Validate), Chapter 32 teaches how to:
- Parallelize work (multiple team members on different phases)
- Use ADRs and PHRs for async collaboration
- Scale Spec-Kit Plus from solo to team projects

---

## What You Need

### Software Prerequisites

- **Python 3.13+** - Code generation and testing
- **Claude Code** OR **Gemini CLI** - Your AI orchestrator for `/sp.*` commands
- **Text Editor** - VS Code, PyCharm, or your preference (syntax highlighting for Python)
- **Git** - For version control and checkpoint commits (recommended but optional)

### Knowledge Prerequisites

- Chapter 30 completion (SDD philosophy)
- Python basics (functions, variables, control flow)
- Familiarity with CLI (terminal navigation, running commands)
- Comfort with the idea that **clear writing leads to better code**

### Time Commitment

- **Core lessons** (1-7): 13-15 hours
- **Capstone project**: 3-4 hours
- **Total**: 15-18 hours
- **Pace**: Can be completed in 2-3 weeks at 5-7 hours/week, or intensive 2-3 days

---

## How to Use This Chapter

### Self-Directed Learning

Start with **Lesson 1**. Each lesson includes:
1. **Learning objectives** - What you'll be able to do
2. **Concept introduction** - Why this matters
3. **Hands-on practice** - Apply the concept with real examples
4. **Try With AI** - Practice activity with Claude Code or ChatGPT
5. **Assessment checklist** - Verify you've learned it

Progress sequentially. **Don't skip lessons.** Earlier lessons build foundation for later ones.

### In a Classroom Setting

- **Weekly 2-hour sessions**: 1 lesson per session + homework
- **Intensive cohort**: 2 lessons per day, capstone at end of week
- **Blended**: Some lessons async, checkpoint discussions synchronous

### Study Group Approach

Pair with classmates. Peer review each other's specifications (Lesson 3), plans (Lesson 5), validation reports (Lesson 7). Explaining to others deepens understanding.

---

## Success Criteria

### By End of Each Lesson

You should be able to:

| Lesson | Can You... |
|--------|-----------|
| **1** | ...identify SMART vs. vague criteria? ...explain why each component matters? |
| **2** | ...navigate Spec-Kit Plus folder structure? ...explain why each folder forces cascade? |
| **3** | ...write complete specs with all 6 components? ...explain relationships between components? |
| **4** | ...use `/sp.specify` to find gaps? ...refine specs based on feedback? |
| **5** | ...read and analyze plans? ...identify dependencies and critical path? |
| **6** | ...decompose plans into tasks? ...create traceability matrix (requirement → task)? |
| **7** | ...validate code against acceptance criteria? ...identify root cause of failures? |
| **8** | ...execute full workflow independently? ...articulate cascade effect? |

### By End of Chapter

You can:
- Write specifications that prevent AI misinterpretation
- Use all 6 Spec-Kit Plus commands: `/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement`, `/sp.adr`, understand PHR auto-creation
- Validate AI-generated code systematically
- Build complete projects from idea to tested code
- Explain why **specifications are the new syntax** in AI-native development
- Apply checkpoint-driven workflow (YOU stay in control)

---

## Key Principles

### 1. Specification-First, Always

Code follows spec. Plan follows spec. Never start with "what code should I write?" Start with "what does the user need?" Write that down clearly. Then build.

### 2. Cascade Effect is Real

Invest time in spec clarity. It pays dividends in every downstream phase. A 2-hour refinement of the spec might save 4 hours of planning and code troubleshooting.

### 3. Validation is Non-Negotiable

Never accept code you haven't read and understood. AI is powerful but requires human validation. Your job: verify AI understood your intent.

### 4. You Are in Control

AI is a tool. You define intent, make decisions, validate outputs. Checkpoints keep you in control. No autonomous loops that run for hours without your input.

### 5. Clarity is Your Superpower

In an age of AI, human clarity is the highest-value skill. Clear writing → clear specs → correct code. This is your competitive advantage.

---

## Lesson Navigation

### Quick Links to All 8 Lessons

1. [Lesson 1: SMART Acceptance Criteria](./01-smart-acceptance-criteria.md)
2. [Lesson 2: Spec-Kit Plus Project Structure](./02-specifyplus-project-structure.md)
3. [Lesson 3: Complete Specification Writing](./03-complete-specification-writing.md)
4. [Lesson 4: Refining Specs with /sp.specify](./04-refining-specs-with-sp-specify.md)
5. [Lesson 5: Planning from Specification](/05-planning-from-specification.md)
6. [Lesson 6: Task Decomposition with /sp.tasks](./06-decomposing-plans-into-tasks.md)
7. [Lesson 7: Implementation & Validation with /sp.implement](./07-implementation-and-validation.md)
8. [Lesson 8: Capstone Project](./08-capstone-project.md)

---

## Common Questions

### Q: Do I need to know Spec-Kit Plus syntax before starting?
**A**: No. Lessons start from zero. Lesson 1 teaches concepts; Lesson 4 is your first tool usage.

### Q: What if `/sp.specify` command doesn't work?
**A**: Lessons teach principles first. If tool breaks, you can still manually review specs using SMART framework. The tool amplifies thinking; clear thinking is foundational.

### Q: Can I skip ahead to code generation (Lesson 7)?
**A**: Not recommended. Lessons 1-6 build thinking skills. Skipping to code misses the entire SDD value. Lessons 1-3 teach clarity. Lessons 4-6 teach tools. Lesson 7 proves why it matters. Full sequence matters.

### Q: What if I don't like the calculator project?
**A**: Lessons 1-7 use calculator as standard example. Capstone (Lesson 8) lets you choose calculator or grading system. After capstone, apply SDD to *any* project.

### Q: How long do I spend on the capstone?
**A**: 3-4 hours for complete workflow. You're not learning new concepts (Lessons 1-7 did that). You're **integrating** all 7 lessons into one project. Focus on experience, not speed.

### Q: Will AI-generated code always work?
**A**: No. If your spec is ambiguous, AI generates ambiguous code. If AI misunderstands, code fails validation. Lesson 7 teaches how to: (1) identify failures, (2) trace root cause (spec or AI?), (3) refine and regenerate. Validation is core skill.

---

## Support & Resources

### During Each Lesson

- **"Try With AI" section** - Every lesson ends with guided AI practice activity
- **Assessment checklist** - Verify you've learned key concepts
- **Code examples** - Real, runnable examples (Python 3.13+ with type hints)

### Between Lessons

- **Reflection prompts** - Think about what you learned
- **Peer review suggestions** - Have a classmate review your work

### If You Get Stuck

1. Re-read the lesson objective (what are you supposed to be able to do?)
2. Check the assessment checklist (which point are you stuck on?)
3. Try the "Try With AI" activity (interact with Claude to clarify concepts)
4. Review the prior lesson (often earlier concept wasn't clear)

---

## Mindset for This Chapter

As you work through Chapter 31, remember:

> You're not learning to use tools. You're learning to think clearly, then amplify that thinking with tools. Specs are where all value starts. Code is where value is deployed. Everything in between is structure to prevent mistakes.

By the end, you'll understand why **in AI-native development, the person who writes the clearest spec wins.**

---

## Ready? Let's Begin

Start with **Lesson 1: SMART Acceptance Criteria**. You'll experience the problem (vague specs → broken code) and learn the solution (SMART framework → testable specs).

After Lesson 1, you'll understand why the next 7 lessons matter.

**Go to [Lesson 1: SMART Acceptance Criteria](./01-smart-acceptance-criteria.md)**

---

*Last Updated: November 5, 2025 | Part 5 of "AI-Native Development" | Complete Spec-Kit Plus Hands-On Chapter*
