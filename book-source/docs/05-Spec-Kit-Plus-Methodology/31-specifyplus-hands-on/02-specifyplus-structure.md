---
title: "SpecifyPlus Project Structure & the Cascade"
chapter: 31
lesson: 2
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

# Lesson 2: SpecifyPlus Project Structure & the Cascade

## Opening Hook

You've learned how to write SMART acceptance criteria. Now comes a crucial question: **Where do those criteria live? How do they connect to implementation plans? How do teams stay organized?**

SpecifyPlus answers this with a simple but powerful insight: **The folder structure itself enforces the workflow.** You can't jump from vague ideas straight to code. The structure forces you to think clearly at each step.

This lesson shows you how. When you see the structure, you'll understand why specification-driven development isn't optional—it's architecturally necessary.

---

## Part A: Explore the Structure (30 min)

### What SpecifyPlus Creates

When you initialize a SpecifyPlus project, you get a specific folder structure. Let's explore it.

**Initialization Command**:

```bash
pip install specifyplus
uvx specifyplus init my-project
```

Or if you already have SpecifyPlus:

```bash
specifyplus init my-project
cd my-project
```

**What You Get** (folder tree):

```
my-project/
├── .specify/                          # Project configuration
│   ├── constitution.md                # Project principles and rules
│   ├── templates/                     # Templates for specs, plans, tasks
│   │   ├── spec-template.md
│   │   ├── plan-template.md
│   │   └── tasks-template.md
│   └── config.yaml                    # Project settings
│
├── specs/                             # Where specifications live
│   └── my-feature/
│       ├── spec.md                    # Feature specification (WHAT to build)
│       ├── plan.md                    # Implementation plan (HOW to build)
│       └── tasks.md                   # Work units (atomic tasks)
│
├── history/                           # Version history and decisions
│   ├── prompts/                       # Prompt history (AI conversations)
│   └── adr/                          # Architecture Decision Records
│
├── README.md                          # Project overview
└── .gitignore                        # Git configuration
```

### The Three Key Folders

#### **1. `.specify/` Folder: Project Constitution & Templates**

This folder contains the rules and templates that guide your project.

**Constitution.md**: Defines project principles. Example principles:
- "Specifications are source code"
- "Validation before deployment"
- "AI as co-reasoning partner"

**Templates**: Provide structure for spec.md, plan.md, tasks.md. They ensure consistency across features.

**Key insight**: The templates aren't bureaucracy—they force you to think through every angle before implementing.

#### **2. `specs/` Folder: The Workflow Documents**

This is where the magic happens. For each feature, you create three documents in sequence:

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

## Part B: Understand the Cascade (35 min)

### The Cascade in Action

Let's trace one real example: "User can see calculation history."

**In spec.md** (the specification):
```markdown
## Requirement: Calculation History
- User can view all previous calculations in session
- History persists across calculator restarts
- User can clear history with one command
- History is limited to last 50 calculations (to prevent memory issues)
```

**In plan.md** (the implementation plan):
```markdown
## Phase 1: Design History Data Structure
- Design schema for storing calculations (operation, operands, result, timestamp)
- Define what "calculation" means (one operation or multiple linked operations?)

## Phase 2: Implement History Storage
- Implement save_calculation() function
- Implement load_history() function
- Handle file persistence (JSON or SQLite)

## Phase 3: Display History UI
- Implement view_history() command
- Implement clear_history() command
- Test with 50+ calculations to verify performance
```

**In tasks.md** (atomic work units):
```markdown
Task 1.1: Design Calculation Schema
- Acceptance: Define JSON schema for one calculation record
- Effort: 1 hour
- Dependencies: None (Phase 1 start)

Task 1.2: Design History Structure
- Acceptance: Propose file structure or database schema
- Effort: 1 hour
- Dependencies: Task 1.1

Task 2.1: Implement save_calculation()
- Acceptance: Function stores calculation to file/database
- Effort: 2 hours
- Dependencies: Task 1.2

Task 2.2: Implement load_history()
- Acceptance: Function retrieves all calculations in order
- Effort: 2 hours
- Dependencies: Task 2.1

Task 3.1: Implement view_history() Command
- Acceptance: Display last 10 calculations; show operation, result, timestamp
- Effort: 1.5 hours
- Dependencies: Task 2.2

Task 3.2: Implement clear_history() Command
- Acceptance: Clear all history or last N items; confirm before deletion
- Effort: 1 hour
- Dependencies: Task 2.2
```

### Observe the Cascade

Notice how one requirement flows through all three documents:

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

## Part C: Project Exploration (25 min)

### Exercise 1: Initialize Your Own Project

**Instructions**: Initialize a SpecifyPlus project locally.

```bash
uvx specifyplus init my-calculator
cd my-calculator
```

**What to Do**:
- Explore the folders
- Open `.specify/constitution.md` — read the first 3 principles
- Open `specs/README.md` — understand the feature folder structure
- Verify you see `.specify/`, `specs/`, and `history/` folders

### Exercise 2: Open and Read Templates

**Instructions**: Open these template files and read them carefully.

**File 1: `spec.md` template** (in `.specify/templates/` or `specs/your-feature/spec.md`)
- Identify the sections (Overview, Scope, Requirements, Acceptance Criteria, Constraints, Success Criteria)
- Note: These match the structure from Lesson 1 (SMART criteria go in Acceptance Criteria section)

**File 2: `plan.md` template** (in `.specify/templates/`)
- Identify the sections (Phases, Dependencies, Milestones)
- Notice: Plan describes HOW (phases), not WHAT (requirements)

**File 3: `tasks.md` template** (in `.specify/templates/`)
- Identify task structure (ID, Description, Acceptance Criteria, Dependencies, Effort, Priority)
- Notice: Each task is atomic (one person, 1-4 hours, single responsibility)

### Exercise 3: Map SMART to Template

**Instructions**: Take your 5 SMART criteria from Lesson 1, Exercise 2.

**Your Task**:
- Which section of spec.md would these criteria appear in?
- How would plan.md break down the first criterion into phases?
- How would tasks.md break those phases into atomic tasks?

**Example**:

Criterion from Lesson 1: "When user enters invalid input, display error message within 500ms that includes: (1) the operation name, (2) what was wrong, (3) an example of valid input."

**In spec.md**:
```markdown
## Acceptance Criteria
- Criterion: Invalid input handling (500ms response, error message format)
```

**In plan.md**:
```markdown
## Phase 1: Error Detection
- Identify all invalid input cases

## Phase 2: Error Messaging
- Build error message formatter (operation name + reason + example)

## Phase 3: Performance Testing
- Validate response time ≤ 500ms
```

**In tasks.md**:
```markdown
Task 1.1: List all invalid input cases
- Acceptance: Document at least 10 invalid input scenarios
- Effort: 1 hour

Task 2.1: Build error message formatter
- Acceptance: Function returns formatted error message
- Effort: 2 hours

Task 3.1: Performance test error handling
- Acceptance: Measure response time for error case; verify ≤ 500ms
- Effort: 1 hour
```

---

## Part D: Key Insight — Structure Forces Thinking

Write this down: **The SpecifyPlus folder structure is not bureaucracy. It's architecture. It forces you to think clearly at each level.**

Each folder serves a purpose:
- `.specify/` = Rules and templates (guard against chaos)
- `specs/` = Workflow documents (spec → plan → tasks)
- `history/` = Decision tracking (why we chose this approach)

This structure makes specification-driven development possible because it's literally impossible to skip steps. You can't write a good plan without a clear spec. You can't write good tasks without a clear plan.

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

   If this spec is vague, predict:
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

### Expected Outcomes

- You understand why spec → plan → tasks is a forced sequence, not optional
- You see how folder structure prevents the "jump straight to coding" trap
- You're ready for Lesson 3 with context on where specifications fit
- You understand the cascade: spec quality → plan quality → task quality → code quality

### Safety & Ethics Note

**Remember**: The structure isn't about compliance or micromanagement. It's about efficiency. Clear specifications prevent wasted time on misaligned implementation. Teams that follow this structure ship faster, not slower.

---

## Checkpoint: Are You Ready for Lesson 3?

Before moving forward, verify you can do these:

- [ ] Navigate the `.specify/`, `specs/`, and `history/` folders
- [ ] Explain the purpose of each folder in one sentence
- [ ] Describe why spec.md must come before plan.md
- [ ] Describe why plan.md must come before tasks.md
- [ ] Trace a requirement from your SMART criteria through all three documents (spec → plan → task)
- [ ] Explain what breaks if you skip the plan and jump to tasks
