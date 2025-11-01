---
title: "Decompose Your Spec Into Atomic Tasks"
chapter: 31
lesson: 6
duration: "1.5 hours"
skills:
  - name: "Task Decomposition"
    proficiency: "B1"
    category: "Technical"
  - name: "Atomic Unit Definition"
    proficiency: "B1"
    category: "Technical"
  - name: "Dependency Mapping"
    proficiency: "B1"
    category: "Conceptual"
learning_objectives:
  - "Break a specification into atomic, independently-testable work units (B1)"
  - "Identify task dependencies and critical path (B1)"
  - "Design tasks that can be executed in parallel or sequence (B1)"
---

# Decompose Your Spec Into Atomic Tasks

## What You're About to Discover

You have a specification. You have a plan that breaks it into lessons. Now comes the final transformation: **breaking the plan into atomic tasks.**

An atomic task is small, complete, and testable. It's something one person can build in 2-4 hours. It doesn't depend on anything uncertain.

You're going to run `/sp.tasks`, see what emerges, and learn to evaluate task quality by asking: "Is this small enough that I could actually build it without getting stuck?"

---

## Part 1: Understanding Task Atomicity

Before you run the tool, let's clarify what "atomic" means.

**Too Big** (not atomic):
```
- Build the grading system
- Implement grading features
- Create the backend
```

These are so big that you'd get stuck halfway through because you don't know exactly what to do next.

**Too Small** (overly granular):
```
- Create rubric.py file
- Add __init__ to Rubric class
- Add name field to Rubric
- Add description field to Rubric
```

These are so small that task overhead kills productivity. Each task takes 10 minutes, but context switching and logging takes 5 minutes.

**Just Right** (atomic):
```
- Implement Rubric dataclass with name, description, and criteria
  - Acceptance: Can create/update/delete rubrics; changes persist to JSON
- Implement file upload handler for essays and code
  - Acceptance: Accepts .txt (max 10k words) and .py (max 1k lines); rejects oversized/invalid files
- Implement grade storage with persistence
  - Acceptance: Can save grades; can retrieve by student and assignment; grades survive application restart
```

Each task is big enough to be meaningful, but small enough that you know exactly what to do.

---

## Part 2: Run /sp.tasks

**In your terminal:**

```bash
cd grading-system
/sp.tasks
```

(Or: `uvx specifyplus tasks`)

**What happens**:

The tool reads your `specs/grading-system/spec.md` and `plan.md`, analyzes them, and generates `specs/grading-system/tasks.md`.

This file contains your complete task breakdown with:

1. **Task List** — Every work unit needed to complete the project
2. **Dependencies** — Which tasks must complete before others
3. **Acceptance Criteria** — How you know each task is done
4. **Effort Estimates** — How long each task should take
5. **Risk Notes** — Gotchas or tricky parts

---

## Part 3: Examine Your Tasks

Open `specs/grading-system/tasks.md`. You'll see something like:

```markdown
---
title: "Grading System - Tasks"
feature: "grading-system"
base_spec: "spec.md"
base_plan: "plan.md"
---

# Task Breakdown: Grading System

## Phase 1: Foundation

### Task 1.1: Project Setup and Data Model
**Duration:** 3-4 hours
**Depends on:** Nothing
**Description:**
Create project structure and define Rubric dataclass

**Acceptance Criteria:**
- [ ] Project structure created (src/, tests/, data/ directories)
- [ ] Rubric dataclass defined with fields: name, description, criteria
- [ ] Rubric can be instantiated and printed
- [ ] Unit tests written and passing for Rubric class
- [ ] No external dependencies (uses only Python stdlib)

---

### Task 1.2: Rubric Management
**Duration:** 3-4 hours
**Depends on:** Task 1.1
**Description:**
Implement create/read/update/delete operations for rubrics

**Acceptance Criteria:**
- [ ] Can create a new rubric with 3-5 criteria
- [ ] Can save rubric to JSON file
- [ ] Can load rubric from JSON file
- [ ] Can update existing rubric
- [ ] Can delete rubric (removes from storage)
- [ ] All operations tested

---

## Phase 2: Core Grading

### Task 2.1: File Upload Handler
**Duration:** 2-3 hours
**Depends on:** Task 1.1
**Description:**
Implement file upload validation for essays and code submissions

**Acceptance Criteria:**
- [ ] Accepts .txt files up to 10,000 words
- [ ] Accepts .py files up to 1,000 lines
- [ ] Rejects oversized files with clear error message
- [ ] Rejects invalid file formats
- [ ] Stores uploaded files with metadata (student name, date, filename)
- [ ] Tests cover all acceptance criteria

---

### Task 2.2: Grade Storage
**Duration:** 2-3 hours
**Depends on:** Task 1.2, Task 2.1
**Description:**
Implement persistent grade storage with retrieval

**Acceptance Criteria:**
- [ ] Can store grade with: student name, assignment ID, score (0-100), feedback, date
- [ ] Can retrieve grades by student name
- [ ] Can retrieve grades by assignment ID
- [ ] Grades persist across application restart
- [ ] Historical grades cannot be deleted (append-only)
- [ ] All operations tested

---

### Task 2.3: Feedback Generator
**Duration:** 3-4 hours
**Depends on:** Task 1.2, Task 2.1
**Description:**
Implement template-based feedback generation

**Acceptance Criteria:**
- [ ] Feedback generator reads rubric criteria
- [ ] Generates feedback 50-200 words
- [ ] Feedback addresses at least 2 rubric criteria
- [ ] Feedback includes specific examples from submission
- [ ] Response time < 3 seconds per submission
- [ ] All acceptance criteria testable and passing

---

## Phase 3: Polish

### Task 3.1: Test Suite & Documentation
**Duration:** 3-4 hours
**Depends on:** All Phase 2 tasks
**Description:**
Comprehensive testing and user documentation

**Acceptance Criteria:**
- [ ] > 80% code coverage
- [ ] All edge cases tested (empty submissions, oversized files, etc.)
- [ ] README with setup and usage instructions
- [ ] Example workflow documented
- [ ] All tests passing

---

## Dependency Map

Task 1.1 (Foundation)
  ├─→ Task 1.2 (Rubric Management)
  ├─→ Task 2.1 (File Upload)
  │
Task 1.2 + Task 2.1
  ├─→ Task 2.2 (Grade Storage)
  ├─→ Task 2.3 (Feedback)
  │
Task 2.2 + Task 2.3
  ├─→ Task 3.1 (Testing & Docs)

---

## Effort Summary

| Phase | Task Count | Total Hours | Critical Path |
|-------|-----------|-------------|--------------|
| Foundation | 2 | 6-8 | Yes |
| Core Grading | 3 | 7-10 | Yes |
| Polish | 1 | 3-4 | No |
| **Total** | **6** | **16-22** | |
```

---

## Part 4: Evaluate Task Quality With Your Companion

**Ask your companion to assess your tasks:**

```
I've generated a task breakdown for my grading system.
Here's my tasks.md:

[paste your tasks.md]

Can you evaluate the quality of these tasks?

1. Are they atomic (small enough to be buildable)?
2. Are dependencies clear?
3. Could I hand these to a developer (or AI) and they'd know what to do?
4. Are the acceptance criteria testable?
5. Is the effort estimation realistic?
6. Are there any tasks that are too big or too small?
```

Your companion will help you evaluate:

> "Your tasks look well-structured. Here's my assessment:
>
> **Atomic?** Yes. Each task is 2-4 hours, which is ideal.
>
> **Dependencies?** Clear. Task 1.1 is foundation, 1.2 depends on it, 2.x depend on both, 3.x depend on all.
>
> **Clarity?** Very good. Each task has a clear description and acceptance criteria.
>
> **Testable?** Yes. Each criterion is something you can verify (has field, saves to JSON, returns error message, etc).
>
> **Effort?** Reasonable for a beginner. Might be optimistic for edge cases, but good as a first estimate.
>
> **Suggestions for improvement:**
> - Task 2.3 (Feedback Generator) might be bigger than 3-4 hours if you haven't written text parsing before. Consider splitting.
> - Task 3.1 could be split into 'Testing' and 'Documentation' if those are different skills.
> - Add a task for 'Manual testing workflow' so you know how to test the whole system end-to-end.
"

---

## Part 5: Understand Task Atomicity Principles

From this exercise, extract the principles for writing good tasks:

```markdown
## Task Atomicity Principles

### Size
- 1-2 hours: Too small (context switching overhead)
- 2-4 hours: Just right (atomic)
- 4-8 hours: At the edge (consider splitting)
- 8+ hours: Too big (you'll get stuck)

### Completeness
- Task must be doable without information from future tasks
- Task might depend on past tasks, but not future ones
- No "TBD" or "figure this out later"

### Clarity
- Description answers: What am I building?
- Acceptance criteria answer: How do I know it's done?
- Someone could read this without asking questions

### Testability
- Each acceptance criterion is verifiable
- Not "works well" but "returns error message X when file exceeds 10k words"
- Not "readable code" but "has docstrings for all functions"

### Dependencies
- Clear statement of what must be done first
- No circular dependencies
- Critical path identified

### Independence
- Task doesn't need changes from other in-progress tasks
- Can be built in any order (respecting dependencies)
- Results are self-contained
```

---

## Part 6: Spot Task Quality Issues

Good tasks have clear patterns. Bad tasks have warning signs.

**Red Flags for Task Quality:**

| Red Flag | Good Task | Bad Task |
|----------|-----------|----------|
| **Vague duration** | "2-3 hours" | "TBD" or "a few weeks" |
| **Unclear acceptance** | "Can save grade with fields X, Y, Z" | "Implement grading" |
| **Hidden dependencies** | "Depends on Task 1.2" | "Depends on nothing (but actually needs Task 2.1)" |
| **Ambiguous scope** | "Implement file upload handler for essays (.txt)" | "Implement file handling" |
| **Unmeasurable criteria** | "Feedback is helpful" | "Feedback is specific to rubric (not generic)" |
| **Vague error handling** | "Handle oversized files: return error 'File exceeds 10k words'" | "Handle edge cases" |
| **Future unknowns** | "Uses JSON files for storage" | "Storage mechanism TBD" |

**Checklist for task quality:**

```markdown
- [ ] I can read this without asking questions
- [ ] Duration estimate is 2-4 hours
- [ ] Acceptance criteria are specific and testable
- [ ] I know what I depend on
- [ ] I know what depends on me
- [ ] No "figure this out later"
- [ ] No ambiguous words like "good," "helpful," "efficient"
```

---

## Part 7: Recognize the Cascade

Look at the full progression you've completed:

```
Lesson 1: Vague idea → Coached to clarity (six components)
Lesson 2: Clarity → Formalized in SpecifyPlus structure
Lesson 3: Clarity → Made even more specific (SMART criteria)
Lesson 4: Refined spec → Analyzed by tool (/sp.specify)
Lesson 5: Clear spec → Transformed to plan (/sp.plan)
Lesson 6: Plan → Broken into atomic tasks (/sp.tasks)
```

Each step is smaller, more detailed, more specific than the last.

- Spec is big-picture (what are we building?)
- Plan is medium-scope (how will we build it in phases?)
- Tasks are atomic (what's the next thing to build?)

And the quality at each level depends entirely on the clarity upstream.

---

## Try With AI

You're going to practice task evaluation and refinement by working with your companion.

### Prompt 1: Evaluate Your Tasks
```
I've generated tasks from my implementation plan.
Here's my task breakdown:

[paste your tasks.md, just the task list section]

For each task, tell me:
1. Is the duration realistic?
2. Are acceptance criteria clear and testable?
3. Could a developer (or AI) build this from the description alone?
4. Is anything missing?

If you find issues, tell me how to fix them.
```

**Expected outcome**: Your companion validates your tasks. If gaps exist, you identify them and refine the task descriptions.

### Prompt 2: Spot Vague Tasks
```
Here are some task descriptions I wrote (paraphrased):

- "Implement grading system"
- "Handle file uploads"
- "Make sure code is clean"
- "Test everything"

Can you tell me what's vague about these?
What would a CLEAR version look like?
```

**Expected outcome**: You develop a sense for vague vs. clear. You can spot problems in your own work.

### Prompt 3: Verify Dependencies
```
Here are my tasks and their dependencies:

[list your tasks with dependencies]

Are there any missing dependencies?
Are there any circular dependencies?
Is the critical path clear?
```

**Expected outcome**: You verify that tasks can actually be built in the order specified. No surprises partway through.

---

## Safety & Next Steps

You've now completed the **specification → plan → tasks** progression. This is professional software development.

The key lesson: **Task quality flows from spec clarity.** Clear spec → clear plan → clear, buildable tasks. Vague spec → vague plan → ambiguous, stuck-prone tasks.

In the next lesson (the final lesson before mini-projects), you're going to put everything together: **building a complete grading system specification end-to-end**, running it through all the tools, and delivering something ready for implementation.

**Next**: You'll integrate all lessons and create a production-quality specification with plan and tasks.
