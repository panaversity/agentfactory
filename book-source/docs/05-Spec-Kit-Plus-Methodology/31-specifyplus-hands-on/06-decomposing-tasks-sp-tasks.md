---
title: "Decomposing Plans into Tasks (/sp.tasks)"
chapter: 31
lesson: 6
duration_minutes: 90

skills:
  - name: "Task Decomposition"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student uses /sp.tasks; reads task checklist; understands how tasks break down phases"

  - name: "Lineage Traceability"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student traces requirement → plan phase → task unit; explains relationship at each level"

  - name: "Quality Cascade Mastery"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Apply, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student explains how spec gaps → plan ambiguities → task confusion; demonstrates improvement through iteration"

learning_objectives:
  - objective: "Use /sp.tasks to decompose plans into atomic work units and trace lineage from specification to implementation"
    proficiency_level: "B1"
    bloom_level: "Apply, Analyze"
    assessment_method: "Given a plan, student uses /sp.tasks, reads task checklist, identifies task dependencies, traces 2+ requirements through full pipeline"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (Task Decomposition, Atomic Task, Task Dependencies, Priority Levels, Acceptance Criteria per Task, Effort Estimation, Lineage Traceability) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Design task dependencies for large teams; identify parallelizable work streams; estimate effort using empirical data"
  remedial_for_struggling: "Simplified task structure (only description + acceptance criteria); guided dependency mapping"
---

# Lesson 6: Decomposing Plans into Tasks (/sp.tasks)

## Opening Hook

You have a clear plan with phases, dependencies, and milestones. Excellent. But now you face a practical question: **How do I actually start working?**

A plan says "Phase 1: Data Structure & Architecture (2-3 hours)." But what does that mean day-to-day? Do I start with database design? Schema documentation? Both?

`/sp.tasks` breaks plans into **atomic work units**—small, focused, independently testable tasks that a developer can complete in 1-4 hours.

This lesson shows you how to decompose plans into tasks and understand the complete lineage: **Specification → Plan → Tasks → Code**.

---

## Part A: Understanding Tasks (15 min)

### What's an Atomic Task?

A task is atomic when it:

1. **Can be completed by one person in 1-4 hours** (fits in one work session)
2. **Has clear acceptance criteria** (you know when it's done)
3. **Has explicit dependencies** (you know what must finish before you start)
4. **Produces concrete output** (code file, test file, documentation)
5. **Doesn't depend on multiple other tasks** (minimizes blocking)

### Task Anatomy

Each task has these components:

#### **Task ID**: Unique identifier
```
Task 1.1 (Phase 1, Task 1)
Task 2.3 (Phase 2, Task 3)
```

#### **Description**: What you're doing
```
Task 2.1: Implement add() function
Task 2.2: Implement subtract() function
Task 3.1: Design error message format
```

#### **Acceptance Criteria**: How you know it's done
```
Acceptance: Function add(a, b) returns sum; works for positive, negative, zero
```

#### **Dependencies**: What must finish first
```
Dependencies: Task 1.2 (schema design)
```

#### **Priority**: MUST/SHOULD/NICE-TO-HAVE
```
Priority: MUST (required for MVP)
```

#### **Effort**: Estimated hours
```
Effort: 2 hours
```

### Task vs. Phase vs. Requirement

| Requirement (Spec) | Phase (Plan) | Task (Tasks) |
|---|---|---|
| "System calculates weighted grades" | "Phase 2: Implement grading algorithm" | Task 2.1: Code weighted_grade() function<br/>Task 2.2: Write unit tests<br/>Task 2.3: Test with sample data |
| High-level business need | Medium-level implementation step | Low-level concrete work unit |
| What matters to stakeholders | What matters to managers | What matters to developers |

### The Cascade: From Requirement to Task

Let's trace one requirement through all levels:

**Specification (Requirement)**:
> "System calculates weighted grades (homework 30%, quizzes 20%, exams 50%)"

**Plan (Phase)**:
```
Phase 2: Implement Grading Algorithm
- Design calculation formula
- Code weighted_grade() function
- Test with multiple student scenarios
```

**Tasks (Atomic Units)**:
```
Task 2.1: Design Grading Formula
Acceptance: Weighted grade formula defined and documented
Effort: 1 hour
Dependencies: Task 1.2 (data structure complete)

Task 2.2: Implement weighted_grade() Function
Acceptance: Function takes scores dict, weights dict, returns final grade (0-100)
Effort: 2 hours
Dependencies: Task 2.1

Task 2.3: Write Unit Tests for weighted_grade()
Acceptance: All test cases pass (normal weights, edge cases: all 0, all 100, single category)
Effort: 1.5 hours
Dependencies: Task 2.2

Task 2.4: Integration Test (Real Student Scenario)
Acceptance: Test with 10 real student score combinations; verify accuracy ±0.01%
Effort: 1 hour
Dependencies: Task 2.3
```

Notice:
- One requirement → one plan phase → four tasks
- Each task is small (1-2 hours)
- Each task has clear dependencies
- Each task produces concrete output (design doc, code, tests)

---

## Part B: Generate Tasks with /sp.tasks (20 min)

### Running the Command

Like `/sp.specify` and `/sp.plan`, `/sp.tasks` runs **within Claude Code**:

1. Open Claude Code
2. Paste your plan (from Lesson 5)
3. Run `/sp.tasks`
4. Read the generated task checklist

### Example Output

When you run `/sp.tasks` on your calculator plan, you get:

```markdown
# Task Checklist

## Phase 1: Data Structure & Architecture

### Task 1.1: Document Input Validation Rules
Priority: MUST
Effort: 1 hour
Dependencies: None
Acceptance: List all invalid input cases (non-numbers, invalid operators, empty input)
Output: validation_rules.md

### Task 1.2: Design Calculation Schema
Priority: MUST
Effort: 1.5 hours
Dependencies: Task 1.1
Acceptance: JSON schema defined for calculation record (operation, operands, result, timestamp)
Output: calculation_schema.json

### Task 1.3: Design History Storage Structure
Priority: MUST
Effort: 1 hour
Dependencies: Task 1.2
Acceptance: Storage approach documented (file format, max size, persistence rules)
Output: history_design.md

## Phase 2: Core Functions

### Task 2.1: Implement add() Function
Priority: MUST
Effort: 0.5 hours
Dependencies: Task 1.2
Acceptance: add(a, b) returns sum; handles positive, negative, zero, floats
Output: calculator.py (add function + docstring)

### Task 2.2: Implement subtract() Function
Priority: MUST
Effort: 0.5 hours
Dependencies: Task 1.2
Acceptance: subtract(a, b) returns difference
Output: calculator.py (subtract function)

### Task 2.3: Implement multiply() Function
Priority: MUST
Effort: 0.5 hours
Dependencies: Task 1.2
Acceptance: multiply(a, b) returns product
Output: calculator.py (multiply function)

### Task 2.4: Implement divide() Function
Priority: MUST
Effort: 0.5 hours
Dependencies: Task 1.2
Acceptance: divide(a, b) returns quotient; error on divide-by-zero handled in Phase 3
Output: calculator.py (divide function)

### Task 2.5: Implement exp() Function
Priority: MUST
Effort: 0.5 hours
Dependencies: Task 1.2
Acceptance: exp(a, b) returns a^b
Output: calculator.py (exp function)

### Task 2.6: Implement sqrt() Function
Priority: SHOULD
Effort: 0.5 hours
Dependencies: Task 1.2
Acceptance: sqrt(a) returns square root; error on negative input handled in Phase 3
Output: calculator.py (sqrt function)

### Task 2.7: Write Unit Tests for All Functions
Priority: MUST
Effort: 2 hours
Dependencies: Tasks 2.1-2.6
Acceptance: All functions tested (normal cases + edge cases); 95% code coverage
Output: test_calculator.py

## Phase 3: Error Handling

### Task 3.1: Implement Input Validation
Priority: MUST
Effort: 1 hour
Dependencies: Task 1.1
Acceptance: validate_input() function returns True for valid, False for invalid
Output: validation.py + unit tests

### Task 3.2: Implement Division by Zero Handling
Priority: MUST
Effort: 0.5 hours
Dependencies: Task 2.4
Acceptance: divide() returns error message "ERROR: Cannot divide by zero"
Output: Updated calculator.py

### Task 3.3: Implement Error Message Formatter
Priority: MUST
Effort: 1 hour
Dependencies: Tasks 3.1, 3.2
Acceptance: format_error(operation, reason) returns formatted error message
Output: error_formatter.py

## Phase 4: History Storage & Commands

### Task 4.1: Implement save_calculation() Function
Priority: MUST
Effort: 1.5 hours
Dependencies: Tasks 1.3, 2.7
Acceptance: Function saves calculation to file; verifiable by loading
Output: history.py

### Task 4.2: Implement load_history() Function
Priority: MUST
Effort: 1 hour
Dependencies: Task 4.1
Acceptance: Function retrieves all calculations in order; returns list of dicts
Output: history.py

### Task 4.3: Implement clear_history() Command
Priority: MUST
Effort: 0.5 hours
Dependencies: Task 4.2
Acceptance: Clears all history; file verified empty
Output: history.py

### Task 4.4: Integrate History with Core Functions
Priority: MUST
Effort: 1.5 hours
Dependencies: Tasks 2.7, 4.3
Acceptance: Each calculation automatically saved to history
Output: Updated calculator.py

### Task 4.5: Test History Limits (50 Item Cap)
Priority: MUST
Effort: 1 hour
Dependencies: Task 4.4
Acceptance: After 50 calculations, oldest is removed (FIFO)
Output: test_history.py

## Phase 5: Integration & Performance Testing

### Task 5.1: End-to-End Test Scenarios
Priority: MUST
Effort: 1.5 hours
Dependencies: Task 4.5
Acceptance: 10 real-world scenarios tested (mix of operations, errors, history)
Output: test_e2e.py

### Task 5.2: Performance Benchmark
Priority: MUST
Effort: 1 hour
Dependencies: Task 5.1
Acceptance: All operations complete ≤100ms; verified on standard laptop
Output: benchmark_report.md

## Priority Summary
- MUST: 15 tasks (critical for MVP)
- SHOULD: 1 task (nice to have)

## Critical Path
1.1 → 1.2 → 2.1 → 2.7 → 4.4 → 5.1 (11.5 hours minimum)

## Total Effort
MUST tasks: ~18 hours
SHOULD tasks: ~0.5 hours
Total: ~18.5 hours

## Parallelization Opportunities
- Tasks 2.1-2.6 (core functions) can run parallel (after 1.2)
- Task 3.x (error handling) can start after 1.1 (doesn't block critical path)
- Tasks 2.7, 3.3 (tests) can be written while implementation ongoing
```

### Analyzing the Task List

Notice:

1. **Tasks are small**: 0.5-2 hour tasks (you can complete one in a work session)
2. **Dependencies are explicit**: Task 2.4 depends on Task 1.2 (schema must exist first)
3. **Priorities are clear**: MUST vs. SHOULD vs. NICE-TO-HAVE
4. **Effort is estimated**: Each task has hours
5. **Acceptance criteria are testable**: Not "write add function" but "add(a, b) returns sum; handles positive, negative, zero"
6. **Parallelization is possible**: All Task 2.x functions can run parallel (after Task 1.2)

This task list is **actionable**. A developer picks up Task 1.1, reads the acceptance criteria, and starts working. They know exactly what "done" means.

---

## Part C: Student Practice (45 min)

### Exercise 1: Run /sp.tasks on Your Plan

**Instructions**:

1. Open Claude Code
2. Paste your plan (from Lesson 5)
3. Run `/sp.tasks`
4. Read the generated task checklist

**What to Document**:
- How many tasks total?
- How many are MUST? SHOULD? NICE-TO-HAVE?
- What's the longest single task (highest effort)?
- What's the critical path (longest chain of dependencies)?
- What's the total effort (sum of all tasks)?

### Exercise 2: Identify Task Dependencies

**Instructions**: Create a dependency graph (or list) for your tasks.

**Example format**:
```
Task 1.1 (no dependencies)
  ↓
Task 1.2 (depends on 1.1)
  ↓
Task 2.1 (depends on 1.2)
Tasks 2.2, 2.3, 2.4 (all depend on 1.2, can run parallel)
  ↓
Task 2.7 (tests, depends on 2.1-2.6)
```

**Questions to Answer**:
1. Which task could you start first?
2. Which tasks can run in parallel?
3. What's the longest chain (critical path)?
4. If you do tasks out of order, what breaks?

### Exercise 3: Trace Lineage (Spec → Plan → Task)

**Instructions**: Pick 2 requirements from your specification. Trace each through plan to tasks.

**Example**:

**Requirement 1 from Spec**: "Response time ≤ 100ms"

**Plan Phase**: Phase 5 (Integration & Performance Testing)

**Tasks**:
- Task 5.2: Performance Benchmark

**How it cascades**:
- Spec says what you must achieve (≤100ms)
- Plan phase identifies where to address it (performance testing)
- Task defines the concrete work (run benchmark, verify, document)

**Requirement 2 from Spec**: "Division by zero displays error message"

**Plan Phase**: Phase 3 (Error Handling)

**Tasks**:
- Task 3.2: Implement Division by Zero Handling
- Task 3.3: Implement Error Message Formatter

**How it cascades**:
- Spec says what should happen (error message)
- Plan identifies which phase (error handling)
- Tasks define concrete implementation (handle case, format message)

### Exercise 4: Quality Assessment

**Instructions**: Read your task checklist and evaluate:

1. **Are tasks atomic?** (Can one person complete in 1-4 hours?)
   - If you see a task that seems > 4 hours, it needs decomposing
   - If you see tasks < 30 min, they might combine

2. **Are dependencies realistic?** (Does Task B really need Task A to finish?)
   - Sometimes dependencies are too strict (conservative planning)
   - Sometimes dependencies are missing

3. **Are acceptance criteria testable?** (Can you verify "done" with pass/fail?)
   - "Write code" is not testable
   - "Function returns sum for positive, negative, zero inputs" is testable

---

## Part D: Full Lineage Demonstration (10 min)

### The Complete Pipeline: Spec → Plan → Task → Code

You now have the complete lineage. Let's visualize it:

```
SPECIFICATION (Lesson 3)
├─ Requirement: "System calculates grades accurately"
│
PLAN (Lesson 5)
├─ Phase 2: Implement Grading Algorithm
│
TASKS (Lesson 6)
├─ Task 2.1: Design weighted grade formula
├─ Task 2.2: Implement weighted_grade() function
├─ Task 2.3: Write unit tests
├─ Task 2.4: Integration test with real data
│
CODE (Lesson 7 - coming next)
├─ grading.py (contains weighted_grade function)
├─ test_grading.py (contains all tests)
├─ Output: Function that passes all acceptance criteria ✓
```

**Key insight**: Each level cascades from the previous.

- **Vague spec** → unclear plan → confusing tasks → code that doesn't work
- **Clear spec** → detailed plan → atomic tasks → working code

---

## Try With AI

**Tool**: Claude Code with `/sp.tasks` command
**Duration**: 10 minutes

### Workflow

1. **Run `/sp.tasks`** on your plan
2. **Read the task checklist**: Get oriented
3. **Pick one task**: Task 1.1 or Task 2.1
4. **Ask Claude**: "What exactly would I do to complete [task name]?"
5. **Trace backward**: "Which requirement from my spec does this task address?"

### Expected Outcomes

- You see `/sp.tasks` as a real work breakdown tool
- You understand task atomicity and dependencies
- You recognize the complete lineage: Spec → Plan → Task
- You're ready for `/sp.implement` (Lesson 7) with clear work units
- You see the cascade effect in full: specification quality → plan quality → task quality

### Safety & Ethics Note

**Remember**: Task estimates are educated guesses. Reality will vary. Some tasks take 30 minutes, others take 3 hours. The value of decomposition is clarity, not prediction accuracy.

If a task takes longer than estimated, that's data for future planning, not a failure. Adjust estimates and learn.

---

## Checkpoint: Are You Ready for Lesson 7?

Before moving forward, verify you can do these:

- [ ] Run `/sp.tasks` on your plan within Claude Code
- [ ] Understand all tasks and their purposes
- [ ] Identify task dependencies (which must complete before others)
- [ ] Identify the critical path (longest chain of dependent tasks)
- [ ] Trace 2 requirements from spec → plan phase → tasks
- [ ] Estimate total effort (sum of all tasks)
- [ ] Identify parallelizable work (tasks that can run simultaneously)
- [ ] Assess whether tasks are atomic (1-4 hours each)
