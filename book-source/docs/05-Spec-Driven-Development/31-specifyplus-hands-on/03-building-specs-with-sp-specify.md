---
title: "Building Specs with /sp.specify"
chapter: 31
lesson: 3
duration_minutes: 90

skills:
  - name: "Specification Writing"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand, Apply"
    digcomp_area: "Content Creation, Communication"
    measurable_at_this_level: "Student writes complete spec with all 6 components; components are related and consistent"

  - name: "Requirements Clarity"
    proficiency_level: "A2"
    category: "Soft"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Communication & Problem-Solving"
    measurable_at_this_level: "Student identifies ambiguous requirements and clarifies them; explains relationships between requirements"

  - name: "Acceptance Criteria Mastery"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student writes detailed acceptance criteria using SMART framework; criteria are testable and complete"

  - name: "Iterative Clarity"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student recognizes specs are iterative; each iteration adds clarity; provides rationale for revisions"

learning_objectives:
  - objective: "Use /sp.specify to analyze specification completeness, understand AI feedback, and iterate on specifications to improve clarity"
    proficiency_level: "B1"
    bloom_level: "Apply, Analyze"
    assessment_method: "Given a feature idea, student uses /sp.specify, reviews feedback, refines spec, articulates which gaps were filled and why"

---

# Building Specs with /sp.specify

A specification isn't just acceptance criteria. It's a complete description of what you're building: the problem you're solving, the requirements you're meeting, the constraints you're working within, and the criteria for success.

This lesson shows you how to write complete specifications using SpecifyPlus that are so clear that anyone (human or AI) can read them and understand exactly what needs to be built.

---

## Specification Anatomy

These six sections form a coherent whole:

```
Overview (Why?)
  ↓
Scope (What's in/out?)
  ↓
Requirements (What must it do?)
  ↓
Acceptance Criteria (How to verify?)
  ↓
Constraints (What limits us?)
  ↓
Success Criteria (Did we succeed?)
```

Each section flows from the previous. The Overview sets context. Requirements detail what addresses that context. Acceptance Criteria verify requirements are met. Success Criteria measure if the whole effort succeeded.

---

## Build your first Spec with /sp.specify

`/sp.specify` is a special SpecifyPlus command to build specification version 0. In Spec phase our focus is on the What and WHY not HOW

### Where It Runs?

`/sp.specify` runs **within Claude Code** or your AI development environment. It's not a terminal command.

### Running /sp.specify

Let's start with a calculator feature idea that's pretty good but has some gaps:

```markdown
/sp.specify

Feature: Basic calculator operations with full testing.
User journeys:
- Add two numbers (positive, negative, zero, decimals)
- Subtract two numbers (all combinations)
- Multiply two numbers (including edge cases)
- Divide two numbers (we'll handle division by zero later)

Acceptance criteria:
- All operations work with whole numbers and decimals
- All operations return correct results
- All operations have full test coverage
- All functions use Python 3.12+ type hints
- All functions have clear docstrings

Success metrics:
- 100% test coverage for all operations
- Type checking passes with mypy
- Code follows our constitution rules
```

**Agent Does:**

- Creates new feature branch automatically
- Generates comprehensive spec file
- Defines user scenarios and edge cases
- Establishes acceptance criteria
- Sets up testing requirements

### What Makes a Spec "Ready for Planning"

A specification is ready to move to the plan phase when:

1. **Overview is clear**: Someone unfamiliar with the project can understand the problem in 2 minutes
2. **Scope is explicit**: "In" and "Out" lists are complete; no ambiguity
3. **Requirements are testable**: Each requirement can be verified (pass/fail)
4. **Acceptance Criteria are SMART**: From Lesson 1 — Specific, Measurable, Achievable, Relevant, Time-bound
5. **Constraints are realistic**: Timelines, budgets, and technical limits are grounded in reality
6. **Success Criteria are measurable**: You can run a report and say "Yes, we succeeded" or "No, we didn't"

You can optionally run `/sp.clarify` and your AI companion will ask a series of questions to improve the specification. 

Projects that skip this step often fail because they later discover they were building different things. "I thought you meant X, but you meant Y." The specification prevents that miscommunication.

If your spec has all six sections filled, you're ready. If one section is missing or vague, you need to revise.

---