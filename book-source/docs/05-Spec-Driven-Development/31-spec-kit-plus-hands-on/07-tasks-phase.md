---
title: "Tasks Phase - Atomic Work Units and Checkpoints"
chapter: 31
lesson: 7
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
generated_by: "content-implementer v3.0.0"
source_spec: "specs/10-chapter-31-redesign/spec.md"
created: "2025-11-05"
last_modified: "2025-11-05"
git_author: "Claude Code"
workflow: "manual-implementation"
version: "1.0.0"
---

# Tasks Phase - Atomic Work Units and Checkpoints

You now have:
- ✅ A clear specification
- ✅ A detailed implementation plan
- ✅ Documented architecture decisions (ADRs)

Next: Break the plan into **atomic work units** (tasks) that you'll implement.

This lesson teaches the **checkpoint pattern**-the critical workflow practice that keeps YOU in control. The pattern is:

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

#### 💬 AI Colearning Prompt
> "Why are tasks sized at 1-2 hours instead of larger chunks like 'full day' or smaller chunks like '15 minutes'? What's the advantage of this granularity for checkpoint-driven development?"

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

Here's how your calculator tasks depend on each other:

```
TDD Workflow: 🔴 RED (test) → 🟢 GREEN (implement) → 🔵 REFACTOR/DOCS

┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│  Task 1: 🔴 Write RED test: add()                              │
│      ↓                                                          │
│  Task 2: 🟢 Implement add()                                    │
│      ↓                                                          │
│  Task 3: 🔴 Write RED test: subtract()                         │
│      ↓                                                          │
│  Task 4: 🟢 Implement subtract()                               │
│      ↓                                                          │
│  Task 5: 🔴 Write RED test: multiply()                         │
│      ↓                                                          │
│  Task 6: 🟢 Implement multiply()                               │
│      ↓                                                          │
│  Task 7: 🔴 Write RED test: divide() + error cases            │
│      ↓                                                          │
│  Task 8: 🟢 Implement divide() + error handling               │
│      ↓                                                          │
│  Task 9: 🔴 Write RED test: power() + edge cases              │
│      ↓                                                          │
│  Task 10: 🟢 Implement power() + edge case handling           │
│      ↓                                                          │
│  Task 11: 🔵 Write documentation + finalize                    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

Pattern: Each operation follows RED → GREEN cycle
         Tests MUST exist before implementation
```

**Legend:**
- 🔴 **Red tasks** = Write failing tests first (TDD)
- 🟢 **Green tasks** = Implement code to make tests pass
- 🔵 **Blue tasks** = Documentation and polish

**Key Insight**: Tests MUST exist before implementation. You cannot implement Task 2 (add function) without Task 1 (add tests) being complete. This is the TDD (Test-Driven Development) pattern.

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

#### 🎓 Expert Insight
> In AI-native development, the checkpoint pattern transforms risk management. Without checkpoints, AI generates 5000 lines of code and you review it all at once (high risk: bugs are expensive to fix). With checkpoints, AI generates 200 lines, you review immediately, catch issues early (low risk: bugs are cheap to fix). Professional teams NEVER skip checkpoints—the cost of catching bugs late (in production) is 100x the cost of catching them at checkpoint review.

#### 🤝 Practice Exercise

> **Ask your AI**: "I've generated tasks for my calculator implementation. Can you review `specs/calculator/tasks.md` and tell me: (1) Are task sizes appropriate (1-2 hours, single testable criterion)? (2) Is the dependency graph correct (tests before implementation, TDD pattern followed)? (3) Can I trace Task 2 (implement add) back through the plan to the specification? (4) Are there tasks that are too large or too small? Then suggest improvements to the task breakdown."

**Expected Outcome**: Your AI should validate task granularity (e.g., "Implement all operations" = too large → split into per-operation tasks), confirm TDD pattern (tests exist before implementation), verify lineage traceability (specification → plan → task), and suggest optimal sizing for checkpoint-driven development.

---

## Commit Your Tasks 

Commit the generated tasks to git:

```bash
/sp.git_commit_pr commit the current work in same branch
```

---

## Common Mistakes

### Mistake 1: Tasks Too Large (8+ Hours)

**The Error**: "Task: Implement entire calculator (8-16 hours)"

**Why It's Wrong**: Large tasks hide complexity, delay feedback, and make checkpoints meaningless.

**The Fix**: Break into atomic units (1-2 hours each):
- ❌ Large: "Implement all operations"
- ✅ Atomic: "Implement add()" (30 min), "Implement multiply()" (30 min), "Implement divide() with error handling" (1 hour)

### Mistake 2: Ignoring Dependencies

**The Error**: Planning to implement tests before implementing functions

**Why It's Wrong**: Tasks have natural dependencies. Tests depend on functions existing.

**The Fix**: Map dependencies explicitly:
- Task 1: Implement add() → Task 2: Test add() (depends on Task 1)
- Task 3: Implement divide() → Task 4: Test divide() (depends on Task 3)

---

## Try With AI: Validate Task Breakdown

Use your AI companion to confirm your tasks are well-decomposed and ready for implementation.

### Setup

**Tool**: Claude Code (or your configured AI orchestrator)

**Context**: Your tasks.md file

**Goal**: Confirm tasks are atomic, dependencies are correct, and checkpoint pattern will work

:::tip ⚠️ Learning WITH AI (Not Generating FROM AI)

**What this exercise teaches:**
- ❌ **DON'T ask**: "Implement all these tasks for me"
- ❌ **DON'T ask**: "Write the code for Task 1"
- ✅ **DO ask**: "Are my tasks atomic (1-2 hours each)?"
- ✅ **DO ask**: "Are the dependencies correct?"
- ✅ **DO ask**: "Which tasks could I run in parallel?"

**Your role**: Validate task breakdown, understand dependencies, plan execution strategy
**AI's role**: Review atomicity, validate dependencies, suggest improvements

:::

### Prompt Set (Copy-Paste Ready)

**Prompt 1 - Task Atomicity Check**

Copy and paste this into Claude Code:

```
I've broken my calculator plan into tasks. 

For each task, is it:
1. Atomic? (Does it do one thing with one acceptance criterion?)
2. Sized appropriately? (1-2 hours, not too small or large?)
3. Independent? (Can it be reviewed and tested separately?)

Any tasks that are too big, too small, or trying to do multiple things?
```

**Prompt 2 - Dependency Validation**

After task validation, ask:

```
Looking at my task dependencies, are they correct?

Are these dependencies logical? Would you change the order?
What's the critical path (minimum tasks to complete before "done")?
```

**Prompt 3 - Checkpoint Pattern Understanding**

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
