---
title: "Planning from Specification (/sp.plan)"
chapter: 31
lesson: 5
duration_minutes: 90

skills:
  - name: "Plan Generation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student uses /sp.plan; reads plan structure; explains plan phases and dependencies"

  - name: "Cascade Understanding"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student traces requirement → plan phase and articulates how spec quality improved plan quality"

  - name: "Dependency Analysis"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student reads plan; identifies which phases must complete before others; explains rationale"

learning_objectives:
  - objective: "Use /sp.plan to generate implementation plans from specifications and understand plan dependencies"
    proficiency_level: "B1"
    bloom_level: "Apply, Analyze"
    assessment_method: "Given a refined specification, student uses /sp.plan, reads plan, explains phase structure and dependencies, traces requirement to plan"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (Plan vs. Spec, Plan Structure, Cascade Quality, Dependencies, Milestones, Deliverables, Feedback Loop) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Analyze plans for parallel opportunities; design risk mitigation strategies for dependent phases"
  remedial_for_struggling: "Simplified phase visualization; guided dependency mapping template"
---

# Lesson 5: Planning from Specification (/sp.plan)

## Opening Hook

You have a clear, refined specification. Excellent. Now you need to answer: **How will we build this? What are the phases? What must happen first, what next, what last?**

A specification answers "What?" and "Why?" A plan answers "How?" and "When?"

`/sp.plan` takes your specification and generates an implementation plan: phases, dependencies, milestones, risks. This lesson shows you how to use it and understand the plans it produces.

The key insight: **Clear specifications produce clear plans with explicit dependencies. Vague specifications produce confusing plans with missing phases.**

---

## Part A: Understanding Plans (20 min)

### Specification vs. Plan

| Specification | Plan |
|---|---|
| **Answers**: What to build? Why? | **Answers**: How to build? When? |
| **Describes**: Requirements, acceptance criteria, success measures | **Describes**: Phases, dependencies, milestones |
| **Purpose**: Contract with implementer | **Purpose**: Roadmap for development |
| **Audience**: Everyone (stakeholders, developers, AI) | **Audience**: Development team |
| **Length**: Typically 3-5 pages | **Length**: Typically 5-10 pages (more detailed) |

### Plan Structure

A complete plan has these sections:

#### **Phases: High-Level Steps**

Phases break the project into sequential or parallel work streams.

**Example Plan for Calculator**:
- Phase 1: Architecture & Data Structure (design input/output model, history schema)
- Phase 2: Core Functions (implement add, subtract, multiply, divide, exp, sqrt)
- Phase 3: Error Handling (validate inputs, format error messages)
- Phase 4: History & Commands (storage, retrieval, clear)
- Phase 5: Testing & Polish (unit tests, integration tests, performance review)

Each phase has a purpose and produces deliverables (code, tests, documentation).

#### **Dependencies: Phase Ordering**

Dependencies define which phases must complete before others can start.

**Example**:
```
Phase 1 (Architecture)
  ↓ (must complete before Phase 2)
Phase 2 (Core Functions)
  ↓ (must complete before Phase 3)
Phase 3 (Error Handling)
  ↓ (must complete before Phase 4)
Phase 4 (History)
  ↓ (must complete before Phase 5)
Phase 5 (Testing)
```

**Question**: Can you parallelize? For calculator:
- Phase 2 (core functions) and Phase 3 (error handling) could partially overlap
- But Phase 1 (architecture) must complete before anything else

#### **Milestones: Checkpoints**

Milestones mark "when" phases are done.

**Example**:
- EOD Monday (Phase 1 architecture review)
- EOD Wednesday (Phase 2 core functions coded + basic tests)
- EOD Thursday (Phase 3 error handling integrated)
- EOD Friday (Phase 4 history working)
- Following Monday (Phase 5 comprehensive tests + code review)

#### **Deliverables: What's Produced**

Each phase produces concrete outputs.

**Example**:
- Phase 1: Deliverable = Design document (input/output structure, history schema)
- Phase 2: Deliverable = Core functions (add.py, subtract.py, etc.) + unit tests
- Phase 3: Deliverable = Error handling module + error test cases
- Phase 4: Deliverable = History storage module + history tests
- Phase 5: Deliverable = Full test suite (80%+ coverage), code review, deployment ready

### The Cascade Effect: Spec Quality → Plan Quality

**Vague Spec Example**:
> "Build a calculator with history"

**Plan Generated** (vague too):
```
Phase 1: Build calculation
Phase 2: Build history
Phase 3: Testing
```

Problems:
- What's "calculation"? (single operation? multiple?)
- What's "history"? (in-memory? file? database?)
- No error handling mentioned
- Testing is too late (should be parallel)

**Clear Spec Example** (from Lesson 4):
```markdown
Requirements:
- Add, subtract, multiply, divide, exponentiation, square root
- History limited to 50 calculations
- Clear history command
- Response time ≤ 100ms
- Division by zero displays error message
```

**Plan Generated** (much clearer):
```
Phase 1: Design data structures (input, output, history schema)
Phase 2: Implement core functions (operations + unit tests parallel)
Phase 3: Implement error handling (division by zero, invalid input)
Phase 4: Implement history (storage, retrieval, limits)
Phase 5: Integration & performance testing
Phase 6: Code review & final polish
```

Notice the difference:
- **Vague spec** → 3 vague phases
- **Clear spec** → 6 specific phases with clear responsibilities

---

## Part B: Generate Plan with /sp.plan (20 min)

### Running the Command

Like `/sp.specify`, `/sp.plan` runs **within Claude Code**:

1. Open Claude Code
2. Paste your refined specification (from Lesson 4)
3. Run `/sp.plan`
4. Read the generated plan

### Example Output

When you run `/sp.plan` on your calculator spec, you get:

```markdown
# Implementation Plan

## Phase 1: Data Structure & Architecture
Objective: Design the data model for calculations and history

Deliverables:
- Input validation rules documented
- Calculation result schema defined
- History storage structure designed
- Error message format specified

Estimated Effort: 2-3 hours
Dependencies: None (start here)
Risks: If schema design is wrong, all later phases suffer

## Phase 2: Core Arithmetic Functions
Objective: Implement add, subtract, multiply, divide, exp, sqrt

Deliverables:
- add(a, b) → result
- subtract(a, b) → result
- multiply(a, b) → result
- divide(a, b) → result
- exp(a, b) → result  # exponentiation
- sqrt(a) → result

Unit Tests for each function

Estimated Effort: 4 hours
Dependencies: Phase 1 complete
Risks: Performance not meeting ≤100ms target

## Phase 3: Error Handling
Objective: Handle invalid inputs and edge cases

Deliverables:
- Input validation (non-numbers, invalid operators)
- Division by zero handling
- Error message formatting
- Error test cases

Estimated Effort: 3 hours
Dependencies: Phase 1 complete; Phase 2 can be parallel
Risks: Missing edge cases discovered during testing

## Phase 4: History Storage & Commands
Objective: Store and retrieve calculation history

Deliverables:
- save_calculation() function
- load_history() function
- clear_history() function
- History limit enforcement (50 items)
- Integration tests (history + core functions)

Estimated Effort: 3 hours
Dependencies: Phases 1, 2 complete
Risks: File I/O errors if storage not robust

## Phase 5: Integration & Performance Testing
Objective: Verify entire system works end-to-end

Deliverables:
- End-to-end test scenarios
- Performance benchmark (≤100ms verified)
- Full code coverage report
- Deployment checklist

Estimated Effort: 2 hours
Dependencies: All phases complete
Risks: Late-discovered performance issues

## Critical Path
1 → 2 → 4 → 5 (longest chain: 11 hours)
3 can be parallel with 2 (overlap possible)

## Timeline
- End of Day 1: Phases 1-2 (6+ hours)
- End of Day 2: Phases 3-4 (6+ hours)
- Day 3 morning: Phase 5 (2 hours)
- Total: ~14 hours
```

### Analyzing the Plan

Notice several things:

1. **Phases are specific**: Not "build calculation" but "build add(), subtract(), multiply()..."
2. **Dependencies are explicit**: "Phase 2 depends on Phase 1" and "Phase 3 can be parallel with Phase 2"
3. **Effort is estimated**: Each phase has hours (2-4 hours per phase = realistic)
4. **Risks are identified**: Each phase lists what could go wrong
5. **Deliverables are concrete**: Code files, tests, documentation

This is a plan you can actually execute. It's clear enough that a developer can pick up and start Phase 1 today.

---

## Part C: Student Practice (40 min)

### Exercise 1: Run /sp.plan on Your Spec

**Instructions**:

1. Open Claude Code
2. Paste your refined specification from Lesson 4
3. Run `/sp.plan`
4. Read the generated plan

**What to Document**:
- How many phases? (Should be 4-6)
- What's Phase 1? (Usually architecture/design)
- What's the last phase? (Usually testing/polish)
- Total estimated effort (sum of all phases)

### Exercise 2: Understand Dependencies

**Instructions**: Draw (or list) the dependency graph for your plan.

**Example format**:
```
Phase 1: Architecture
  ↓ (must complete before)
Phase 2: Core Implementation
  ↓ (must complete before)
Phase 3: Error Handling (can be parallel with 2)
  ↓ (must complete before)
Phase 4: History
  ↓ (must complete before)
Phase 5: Testing
```

**Questions to Answer**:
1. Which phase is the critical path (longest chain)?
2. Can any phases be parallel? If so, which ones?
3. Why can't Phase X start before Phase Y?

### Exercise 3: Trace a Requirement Through Spec to Plan

**Instructions**: Pick one requirement from your specification. Trace it through the plan.

**Example**:

Requirement from Spec: "Response time ≤ 100ms for all operations"

**Where it appears in Plan**:
- Phase 1: "Performance target documented (≤100ms)"
- Phase 2: "Core functions must not exceed 100ms per operation"
- Phase 5: "Performance benchmark (≤100ms verified)"

**Why it cascades**:
- If spec didn't mention this, no phase would address it
- Because spec is clear, it appears in multiple phases (design, implementation, testing)
- Clear requirements → thorough planning

### Exercise 4: Identify Plan Gaps

**Instructions**: Read your generated plan. Ask:

1. **Is anything missing?** (A phase you expected but don't see?)
2. **Is anything unclear?** (A phase where you don't understand the deliverables?)
3. **Are dependencies realistic?** (Can Phase B really start after Phase A?)
4. **Is effort realistic?** (Are the hour estimates achievable?)

**If you find gaps**: Note them. They might indicate vagueness in your specification (good catch!). You could refine the spec further and regenerate the plan.

---

## Part D: Cascade Demonstration (10 min)

### Compare Two Plans

You have two specifications:

**Spec A (Vague)**:
> "Build a calculator"

**Plan from Spec A**:
```
Phase 1: Implementation
Phase 2: Testing
```

vs.

**Spec B (Clear)** (your spec from Lesson 4):
```
Requirements: Add, subtract, multiply, divide, exp, sqrt
History: 50 calculation limit
Response time: ≤100ms
Error handling: Division by zero, invalid input
Non-functional: File storage, Python 3.13
```

**Plan from Spec B**:
```
Phase 1: Data Structure & Architecture
Phase 2: Core Arithmetic Functions
Phase 3: Error Handling
Phase 4: History Storage & Commands
Phase 5: Integration & Performance Testing
```

**Observation**: Clear spec → detailed, sequenced plan. Vague spec → simplistic, incomplete plan.

---

## Try With AI

**Tool**: Claude Code with `/sp.plan` command
**Duration**: 10 minutes

### Workflow

1. **Run `/sp.plan`** on your refined specification
2. **Review the generated plan**: Read all phases
3. **Ask Claude**: "What are the critical dependencies in this plan?"
4. **Ask Claude**: "Can I parallelize any phases? Which ones and why?"
5. **Trace a requirement**: "How does [requirement from spec] appear in the plan?"

### Expected Outcomes

- You see `/sp.plan` as a real planning tool, not a demo
- You understand how clear specifications produce clear plans
- You recognize dependencies and critical path
- You're ready for `/sp.tasks` (Lesson 6) with a solid plan
- You see the cascade in action: Spec quality → Plan quality

### Safety & Ethics Note

**Remember**: The plan is a roadmap, not a guarantee. Plans change when you discover new information. The value of planning is thinking through the work before you start, not predicting the future perfectly.

If your plan looks unrealistic during implementation, refine the plan. The specification doesn't change (unless requirements changed), but the plan can adapt to reality.

---

## Checkpoint: Are You Ready for Lesson 6?

Before moving forward, verify you can do these:

- [ ] Run `/sp.plan` on your specification within Claude Code
- [ ] Read and understand all phases in the generated plan
- [ ] Explain what each phase delivers and why
- [ ] Identify dependencies (which phases must complete before others)
- [ ] Identify the critical path (longest chain of dependent phases)
- [ ] Trace 1-2 requirements from spec through to plan phases
- [ ] Estimate total effort (sum of all phases)
