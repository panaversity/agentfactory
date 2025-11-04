---
title: "SpecifyPlus Setup & Project Structure"
chapter: 31
lesson: 1
duration_minutes: 90

skills:
  - name: "Understanding Project Structure"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Understand, Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student identifies purpose of each directory (specs, .specify, history) and explains relationships"

  - name: "Recognizing Workflow Enforcement"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student explains why structure forces Spec → Plan → Tasks progression and what breaks if you skip steps"

  - name: "Cascade Thinking"
    proficiency_level: "A2"
    category: "Soft"
    bloom_level: "Understand, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student traces how a specification gap cascades to plan and tasks; predicts ripple effects"

learning_objectives:
  - objective: "Explain the SpecifyPlus project structure and understand why the structure enforces Spec → Plan → Tasks workflow"
    proficiency_level: "A2"
    bloom_level: "Understand, Apply"
    assessment_method: "Given an initialized SpecifyPlus project, student can navigate folders, explain purpose of each directory, and articulate why skipping steps breaks the cascade"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (Spec Folder, Plan Folder, Tasks Folder, Forced Sequence, Cascade Effect) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Design project structure for team environments; explain version control strategy for specs, plans, tasks"
  remedial_for_struggling: "Simplified folder diagram with one-sentence explanation of each; guided exploration checklist"
---

# SpecifyPlus Setup & Project Structure

SpecifyPlus answers SDD problems like how to manage Specification with a simple but powerful insight: **The folder structure itself enforces the workflow.** You can't jump from vague ideas straight to code. The structure forces you to think clearly at each step.

This lesson shows you how. When you see the structure, you'll understand why specification-driven development isn't optional—it's architecturally necessary.

---

## What SpecifyPlus Creates?

When you initialize a SpecifyPlus project, you get a specific folder structure. Let's explore it.

**Installation Command**:

```bash
pip install specifyplus
specifyplus --version
```

**Initialization Command**:

```
specifyplus init my-project
```

Or

```bash
uvx specifyplus init my-project
cd my-project
```

Select your AI Companion and the terminal. Next navigate in the project and open your IDE.

```bash
cd my-project
```

### **What You Get** (folder tree):

#### **1. `.specify/` Folder: Project Constitution & Templates**

This folder contains the rules and templates that guide your project.

**Constitution.md**: Defines project principles. Example principles:
- "Specifications are source code"
- "Validation before deployment"
- "AI as co-reasoning partner"

**Templates**: Provide structure for spec.md, plan.md, tasks.md. They ensure consistency across features.

**Key insight**: The templates aren't bureaucracy—they force you to think through every angle before implementing.

#### **2. `specs/` Folder: The Workflow Documents**

This is where the magic happens. It will appear once we create our first spec.

For each feature, you create three documents in sequence:

**spec.md** (Specification — WHAT to build):
- Problem statement
- Scope (in/out)
- Requirements (functional + non-functional)
- Acceptance criteria (your SMART criteria from Lesson 1!)
- Constraints
- Success criteria

**plan.md** (Implementation Plan — HOW to build):
- Phases (logical steps to building the feature)
- Dependencies (what must complete before what)
- Milestones (checkpoints)
- Risks and mitigations

**tasks.md** (Task Checklist — Atomic work units):
- Individual tasks (each 1-4 hours of work)
- Acceptance criteria for each task
- Dependencies between tasks
- Priority (MUST/SHOULD/NICE-TO-HAVE)

#### **3. `history/` Folder: Decision Tracking**

It will appear once we create our first spec.

- **prompts/**: Records of AI conversations used to build the project (for learning and traceability)
- **adr/**: Architecture Decision Records (why you made certain choices and what alternatives you considered)

**Key insight**: This folder isn't about blame or compliance. It's about learning. When you return to the project in 6 months, you can see why decisions were made.

### Why This Structure Matters

The structure enforces the **Spec → Plan → Tasks cascade**:

1. **You write a specification** → Everyone understands WHAT to build
2. **You write a plan** → You identify HOW and break work into phases
3. **You write tasks** → You create atomic units ready for implementation

If you try to skip steps:
- Skip spec → plan is vague → tasks are confusing → code misses the mark
- Skip plan → tasks don't sequence properly → implementation is chaotic
- Skip tasks → no clear work units → team is confused on priorities

The folder structure is literally **forcing clarity at each level**.

---

## The Cascade in Action

The requirements flows through all three documents:

- **Spec**: Captures business logic (what matters to users)
- **Plan**: Breaks it into implementation phases (data design, storage, UI)
- **Tasks**: Creates atomic work units (each task is 1-2 hours, independent, testable)

**The key insight**: Ambiguity in the specification creates confusion in the plan, which creates chaos in tasks.

### What Breaks if You Skip Steps

#### **Scenario 1: Skip the Plan, Jump Straight to Tasks**

You might create tasks like:
- Task 1: "Implement calculation history"
- Task 2: "Store calculations"
- Task 3: "Display history"

But these are too big! Are they 2-hour tasks or 20-hour tasks? What's the storage mechanism? How do you test "display history" without storage working first?

**Result**: Team gets confused. Work doesn't parallelize. Dependencies are unclear.

#### **Scenario 2: Skip the Spec, Assume Everyone Knows What to Build**

Without spec, the plan might propose different approaches:
- Does history persist to file? Database? Memory?
- What's the maximum history size?
- Does history clear on crash?

Different developers guess differently. Three people implement three different solutions. Integration fails.

**Result**: Rework, frustration, missed deadlines.

#### **Scenario 3: Vague Spec → Vague Plan → Confused Tasks**

If the spec says "user can see calculation history" without details (What's the format? How many? How sorted?), then:
- Plan might design for 10 calculations
- Tasks might assume JSON format
- Code gets built, then fails with 100 calculations
- Everything needs rework

**Result**: Validation fails. Iteration cycle repeats.

---

## Try With AI

**Tool**: ChatGPT web (or your AI companion if already set up)
**Duration**: 10 minutes

### Prompts

1. **Explain the Value**:
   ```
   Why would a development team use this folder structure?

   specs/my-feature/
   ├── spec.md     (what to build)
   ├── plan.md     (how to build it)
   └── tasks.md    (atomic work units)

   What breaks if you skip the plan and go straight from spec to tasks?
   ```

2. **Trace the Cascade**:
   ```
   Here's a vague specification: "Build a grading system that calculates student grades."

   If this spec is vague in SDD methadology, predict:
   1. What confusions would appear in the plan?
   2. What confusion would appear in the tasks?
   3. What problems would developers face implementing?
   ```

3. **Reverse Engineering**:
   ```
   Imagine you found a project with unclear, overlapping tasks that were never completed.
   You trace back and find the plan was vague.
   You trace back further and find the specification was ambiguous.

   How would you fix this? Where would you start?
   ```