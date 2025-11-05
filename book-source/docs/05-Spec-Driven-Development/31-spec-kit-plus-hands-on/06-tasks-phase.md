---
title: "Tasks Phase — Atomic Work Units and Checkpoints"
chapter: 31
lesson: 6
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Using /sp.tasks Command"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can run /sp.tasks to decompose plan into atomic work units"

  - name: "Understanding Atomic Task Definition"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands atomic task = 1-2 hour unit with single acceptance criterion"

  - name: "Recognizing Task Dependencies"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands which tasks must complete before others (dependency graph)"

  - name: "Human-Controlled Checkpoint Pattern"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student understands Agent→Review→Commit→Continue workflow and human's role in each checkpoint"

  - name: "Lineage Traceability"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can trace: specification requirement → plan component → task unit"

learning_objectives:
  - objective: "Use /sp.tasks to decompose calculator plan into atomic work units"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful execution of /sp.tasks and understanding of task breakdown"

  - objective: "Understand checkpoint pattern: Agent completes phase → Human reviews → Human commits → Agent continues"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of checkpoint pattern and human's control role"

  - objective: "Trace requirement lineage from specification through plan to tasks"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Ability to follow a requirement from spec to task unit"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (Tasks command, atomic units, dependencies, checkpoint pattern, lineage traceability, human control, task acceptance criteria) within B1 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Draw dependency graph; analyze what would happen if one task failed; identify critical path"
  remedial_for_struggling: "Focus on top 3-4 tasks; understand dependencies; delay deep analysis of all tasks"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/10-chapter-31-redesign/spec.md"
created: "2025-11-05"
last_modified: "2025-11-05"
git_author: "Claude Code"
workflow: "manual-implementation"
version: "1.0.0"
---

# Tasks Phase — Atomic Work Units and Checkpoints

You now have:
- ✅ A clear specification
- ✅ A detailed implementation plan
- ✅ Documented architecture decisions (ADRs)

Next: Break the plan into **atomic work units** (tasks) that you'll implement.

This lesson teaches the **checkpoint pattern**—the critical workflow practice that keeps YOU in control. The pattern is:

```
Agent: "Here's Phase 1 code"
You: "Review... looks good!"
You: "Commit to git"
You: "Tell me what's next"
Agent: "Phase 2"
```

NOT:

```
Agent: "Here's everything" (no human control)
```

The difference is huge. Checkpoints keep you in control and catch issues early.

---

## What Are Tasks?

A **task** is a unit of work that:
- Takes 1-2 hours to complete
- Has a single, clear acceptance criterion
- Depends on specific other tasks
- Can be reviewed and approved individually

### Task Properties

**Size**: 2-5 minutes
- Too small (>1 minute) = too many micro-tasks
- Too large (30+ minutes) = hard to review, hard to fix if wrong

**Criterion**: Single, testable
- "Write add operation" ✅
- "Write add operation and all tests" ❌ (two things)
- "Write something" ❌ (untestable)

**Independence**: Can be reviewed individually
- Doesn't require other tasks to be done first
- Or clearly depends on specific other tasks

---

## The Checkpoint Pattern (CRITICAL)

This is **the most important concept** in this lesson. The checkpoint pattern is how you maintain control of the workflow.

### Pattern Definition

```
Loop:
  1. Agent: "I've completed Phase X"
  2. Human: "Review the work"
  3. Human: "APPROVE" → Commit to git
  4. Human: "Tell me next step"
```

### Why Checkpoints Matter

**Without Checkpoints** (dangerous):
```
You: "Build my calculator"
Agent: "Done! 5000 lines of code, 47 files. All automated. You're welcome."
You: "Uh... wait, I need to review this..."
Agent: "Too late, already committed and deployed!"
```

**With Checkpoints** (controlled):
```
You: "Start implementation"
Agent: "Phase 1 (Core Operations) complete. 200 lines, ready for review."
You: "Read code... looks good. Commits. What's next?"
Agent: "Phase 2 (Tests) starting"
You: "Review tests... found a bug in edge case handling"
You: "Tell agent, agent fixes, re-reviews, commits"
Agent: "Phase 3..."
```

### Your Role in Each Checkpoint

**Step 1: Human Reviews**
- Read the generated code/tests
- Ask: "Does this match the spec?"
- Ask: "Are there bugs or edge cases missed?"
- Ask: "Is the code understandable?"

**Step 2: Human Decides**
- Approve ("Looks good, commit")
- Reject ("Fix this issue")
- Request clarification ("Explain this code")

**Step 3: Human Directs**
- "What's next?"
- You initiate next phase
- Agent doesn't autonomously continue

---

## Generating Your Tasks

### Step 1: Run /sp.tasks

In Claude Code, from your calculator-project directory:

```
/sp.tasks

My calculator specification is at specs/calculator/spec.md  
My implementation plan is at specs/calculator/plan.md  

Please decompose the plan into atomic work units (tasks), each ≤ 2 hours,  
testable, reversible, and with clear dependencies.

Use a TDD approach: for each operation (add, subtract, etc.),  
1️⃣ Write RED tests → 2️⃣ Implement → 3️⃣ Refactor.  
Pause after each group for human review before committing.

Also:
- Use Context7 MCP server for documentation lookups.
- Prefer CLI automation where possible.
- Ensure easy rollback and traceability.
```

### Step 2: Review Generated Tasks

The tasks.md should show:
- Task 1: [Description] - 1-2 hours - Depends on: Nothing
- Task 2: [Description] - 1.5 hours - Depends on: Task 1
- Task 3: [Description] - 2 hours - Depends on: Task 1, Task 2
- ...

---

## Understanding Your Task Breakdown (15 minutes)

Review your tasks and verify:

### Dependency Graph

Can you draw your tasks?

```
Task 1 (add) → Task 2 (subtract) → Task 3 (multiply, divide)
                                        ↓
                                    Task 4 (power)
                                        ↓
                                Task 5+ (tests, docs)
```

### Lineage Traceability

Pick one task. Can you trace it back?

```
Specification: "Calculator must add two numbers"
  ↓
Plan: "Phase 1: Core Operations - Implement basic arithmetic"
  ↓
Task 1.1: "Implement add(a, b) returning float, handling negative inputs"
  ↓
Acceptance Criterion: "add(5, 3) = 8.0, add(-2, 5) = 3.0, add('5', 3) raises TypeError"
```

If you can trace this lineage, your tasks are well-connected to your specification.

---

## Commit Your Tasks 

Commit the generated tasks to git:

```bash
/sp.git_commit_pr commit the current work in same branch
```

---

## Try With AI: Validate Task Breakdown

Use your AI companion to confirm your tasks are well-decomposed and ready for implementation.

### Setup

**Tool**: Claude Code (or your configured AI orchestrator)

**Context**: Your tasks.md file

**Goal**: Confirm tasks are atomic, dependencies are correct, and checkpoint pattern will work

### Prompt Set (Copy-Paste Ready)

**Prompt 1 — Task Atomicity Check**

Copy and paste this into Claude Code:

```
I've broken my calculator plan into tasks. 

For each task, is it:
1. Atomic? (Does it do one thing with one acceptance criterion?)
2. Sized appropriately? (1-2 hours, not too small or large?)
3. Independent? (Can it be reviewed and tested separately?)

Any tasks that are too big, too small, or trying to do multiple things?
```

**Prompt 2 — Dependency Validation**

After task validation, ask:

```
Looking at my task dependencies, are they correct?

Are these dependencies logical? Would you change the order?
What's the critical path (minimum tasks to complete before "done")?
```

**Prompt 3 — Checkpoint Pattern Understanding**

Finally, ask:

```
I'm about to start the implementation phase, and I want to use the
checkpoint pattern (Agent→Human Review→Commit→Next).
Is this the right approach? Any guidance on what to look for during review?
```

### Expected Outcomes

- **Tasks are atomic and appropriately sized**
- **Dependencies are correct**
- **You understand the checkpoint pattern**
- **Ready for implementation (Lesson 7)**
