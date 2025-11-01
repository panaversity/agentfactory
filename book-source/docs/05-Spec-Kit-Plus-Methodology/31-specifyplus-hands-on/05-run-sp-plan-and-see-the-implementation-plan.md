---
title: "Run /sp.plan and See the Implementation Plan"
chapter: 31
lesson: 5
duration: "1.5 hours"
skills:
  - name: "Planning Automation"
    proficiency: "A2"
    category: "Technical"
  - name: "Implementation Planning"
    proficiency: "B1"
    category: "Technical"
  - name: "Plan Evaluation"
    proficiency: "B1"
    category: "Conceptual"
learning_objectives:
  - "Use /sp.plan CLI command to generate implementation plans from specifications (A2)"
  - "Understand plan structure and breakdown of work units (B1)"
  - "Evaluate plan feasibility and dependencies (B1)"
---

# Run /sp.plan and See the Implementation Plan

## What You're About to Discover

Your specification is now refined. You've answered the hard questions. You've achieved clarity.

Now something magical happens: you're going to run `/sp.plan` and watch your **clear specification automatically transform into a detailed implementation plan.**

This is the proof-of-concept for specification-driven development: clear spec → clear plan. The tool doesn't have to guess. It reads what you specified and breaks it into lessons, learning objectives, phases, and milestones.

---

## Part 1: Run /sp.plan

**In your terminal:**

```bash
cd grading-system
/sp.plan
```

(Or: `uvx specifyplus plan`)

**What happens**:

The tool reads your `specs/grading-system/spec.md`, analyzes it, and generates `specs/grading-system/plan.md`.

This plan includes:

1. **Learning Outcomes** — What will an implementer learn by building this?
2. **Lesson Breakdown** — How to sequence the work
3. **Dependencies** — Which lessons must come before others
4. **Milestones** — Major checkpoints
5. **Technical Approach** — How to build each component
6. **Risks & Mitigations** — What could go wrong

---

## Part 2: Open and Examine Your Plan

Open `specs/grading-system/plan.md`. You'll see something like:

```markdown
---
title: "Grading System Implementation Plan"
feature: "grading-system"
base_spec: "spec.md"
---

# Implementation Plan: Grading System

## I. Project Overview & Success Criteria

[Summary of what you're building and how success is measured]

## II. Architecture & Technical Approach

### A. Data Layer
- Grade storage using JSON
- Rubric definitions using Python dataclasses
- File uploads stored in /uploads directory

### B. Grading Engine
- Template-based feedback generation
- Rubric evaluation logic
- Score calculation

### C. User Interface
- Command-line interface for teacher workflows
- File upload handling
- Grade review and adjustment

## III. Lesson Breakdown

### Phase 1: Foundation (Lessons 1-2)
- Lesson 1.1: Set up project structure and data models
- Lesson 1.2: Implement rubric definition system

### Phase 2: Core Grading (Lessons 3-5)
- Lesson 2.1: Build file upload handler
- Lesson 2.2: Implement grade storage
- Lesson 2.3: Create grading engine

### Phase 3: Polish & Testing (Lessons 6-7)
- Lesson 3.1: Comprehensive test suite
- Lesson 3.2: Documentation and user guide

## IV. Detailed Lesson Plan

[For each lesson: learning objectives, what gets built, acceptance criteria]

## V. Dependency Map

[Which lessons must complete before others can start]

## VI. Risk Analysis

[What could go wrong? How to mitigate?]
```

---

## Part 3: Analyze the Plan With Your Companion

Don't try to interpret the plan alone. Ask your companion to help you understand it.

**Paste the plan and ask:**

```
I ran /sp.plan on my grading system specification.
Here's the generated plan:

[paste your plan.md]

Can you help me understand this plan?

1. What surprised you about this plan?
2. Does the phase breakdown make sense?
3. Is the lesson sequence logical?
4. What would happen if I tried to do Phase 2 before completing Phase 1?
5. How did the plan flow from my specification?
```

Your companion will help you see things like:

> "The plan is well-structured. Here's what I notice:
>
> 1. **The phases are logical**: Foundation first (data models), then core grading, then testing
> 2. **Dependencies are clear**: You can't implement the grading engine without storing rubrics first
> 3. **The lesson breakdown matches your spec**: Because you specified 'template-based feedback,' the plan assumes that, not AI-generated feedback
> 4. **Learning objectives are specific**: Not 'learn Python' but 'implement rubric definition system'
> 5. **Your spec shaped every choice**: The plan is basically a detailed expansion of what you specified
>
> If you had written a vague spec, the plan would be vague too.
> If you had specified 'use AI for feedback,' the plan would have different lessons.
> The spec controls everything downstream."

---

## Part 4: Understand the Spec → Plan Connection

This is the critical insight: **Your specification shaped every decision in the plan.**

Let's trace one example:

### Your Spec Said:
```
Feedback Generation
- Mechanism: Template-based with basic text analysis
- Not AI-generated (to keep costs low)
- Feedback templates are built into the system
```

### The Plan Assumed:
```
Lesson 2.3: Create Grading Engine
- Objective: Implement template-based feedback generator
- Approach: Build feedback templates in YAML
- Implementation: Read template, match rubric criteria, insert evidence
- Testing: Verify feedback is 50-200 words and addresses 2+ criteria
```

If your spec had said "use AI-generated feedback," the plan would be completely different:
- Include an API integration lesson
- Add error handling for API failures
- Include cost budgeting
- Add token usage monitoring

**The point**: Clear spec → everyone knows what to build. Vague spec → people guess and build different things.

---

## Part 5: Extract the Lesson Structure

The plan breaks your project into lessons. Extract the structure:

```markdown
## Lesson Breakdown (From Plan)

### Phase 1: Foundation (2-3 lessons)
- [Lesson title and what gets built]
- [Lesson title and what gets built]

### Phase 2: Core Features (3-5 lessons)
- [Lesson title and what gets built]
- [Lesson title and what gets built]

### Phase 3: Polish & Testing (2-3 lessons)
- [Lesson title and what gets built]
- [Lesson title and what gets built]
```

This structure is your roadmap. Each lesson is a coherent unit of work that can be completed in 2-8 hours.

---

## Part 6: Verify Plan Completeness

**Ask your companion to validate the plan:**

```
Looking at my plan, can you tell me:

1. Does it cover everything in my specification?
2. Are there any features from my spec that aren't addressed?
3. Are there any lessons that seem unnecessary?
4. Is the sequence logical? Could any lessons be reordered?
5. Does this plan seem achievable in the time you'd estimate?
```

Your companion will either:
- Validate that the plan is complete ("Yes, every feature from your spec is addressed")
- Identify gaps ("You specified 'error handling' but don't have a dedicated lesson for it")
- Suggest improvements ("Lesson 2.3 could be split into two smaller lessons")

**If there are gaps**, you either:
1. Refine your spec to clarify what you meant
2. Update the plan to address the gap
3. Accept it as a limitation and note it

This feedback loop is normal and healthy.

---

## Part 7: Create a Plan Validation Checklist

Based on what you learned, create a checklist for validating implementation plans:

```markdown
## Plan Validation Checklist

- [ ] Every feature in the spec has at least one lesson
- [ ] Every lesson builds toward a specific goal
- [ ] Lessons are sequenced with dependencies first
- [ ] No lesson is estimated to take more than 8 hours
- [ ] All acceptance criteria from spec are testable in the lessons
- [ ] Learning objectives match the specification goals
- [ ] Risk mitigations are addressed in the plan
- [ ] I could hand this plan to someone else and they'd know what to build
- [ ] Phases are logical (foundation before features, testing at the end)
- [ ] Technical approach matches what I specified

When my plan passes this checklist, it's ready for task decomposition.
```

---

## Part 8: The Cascade in Action

Let's step back and see the full cascade:

```
Your vague idea:
  ↓
Lesson 1: You coach your companion to clarity
  ↓
Lesson 2-4: You write a clear specification
  ↓
/sp.specify: Tool refines your spec by pointing out gaps
  ↓
Your refined specification
  ↓
/sp.plan: Tool breaks spec into an implementation plan
  ↓
Your implementation plan
  ↓
Next lesson: Task decomposition (breaking plan into atomic tasks)
  ↓
Atomic tasks that can be built
```

Each step builds on the previous. Vagueness at the start compounds. Clarity at the start cascades.

---

## Try With AI

You're going to practice understanding and validating implementation plans by working with your companion.

### Prompt 1: Explain the Plan
```
I've run /sp.plan on my specification and generated an implementation
plan. Here's the plan:

[paste your plan.md]

Can you explain this plan to me in simple terms?
- What are the phases?
- What gets built in each phase?
- What's the learning progression?
- Why is it organized this way?
```

**Expected outcome**: Your companion breaks down the plan into digestible pieces. You understand the structure and the reasoning.

### Prompt 2: Trace Spec to Plan
```
I want to understand how my specification shaped this plan.

Pick one feature from my specification (for example, "teachers can
create rubrics with 3-5 criteria") and trace it through the plan.

Where does this feature appear in the plan?
What lessons build it?
How is it tested?
```

**Expected outcome**: You see the direct connection between spec decisions and plan structure. This reinforces that clear specs → clear plans.

### Prompt 3: Validate Completeness
```
Looking at my plan, I want to make sure it covers everything.

Here are the main features from my specification:
[list 3-5 main features]

For each feature, tell me:
- Is it addressed in the plan?
- Which lesson or lessons build it?
- Are there any features missing from the plan?
```

**Expected outcome**: You verify that the plan is complete. If gaps exist, you identify them and can decide how to address them.

---

## Safety & Next Steps

You've now moved from specification → refined spec → implementation plan. You've seen how clarity flows downstream.

In the next lesson, you're going to run `/sp.tasks` and watch the plan break down into **atomic tasks**—individual work units small enough to build in 2-4 hours.

This is where the work gets real: each task becomes something someone (human or AI-assisted) can actually execute.

**Next**: You'll learn to decompose your plan into tasks and understand why task granularity matters.
