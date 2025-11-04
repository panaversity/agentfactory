---
title: "Decomposing Plans into Atomic Tasks"
chapter: 31
lesson: 5
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

# Decomposing Plans into Atomic Tasks

You have a clear plan with phases, dependencies, and milestones. Excellent. But now you face a practical question: **How do I actually start working?**

A plan says "Phase 1: Data Structure & Architecture (2-3 hours)." But what does that mean day-to-day? Do I start with database design? Schema documentation? Both?

`/sp.tasks` breaks plans into **atomic work units**—small, focused, independently testable tasks that a developer can complete in 1-4 hours.

This lesson shows you how to decompose plans into tasks and understand the complete lineage: **Specification → Plan → Tasks → Code**.

---

## Understanding Tasks

### What's an Atomic Task?

A task is atomic when it:

1. **Can be completed by one person in 1-4 hours** (fits in one work session)
2. **Has clear acceptance criteria** (you know when it's done)
3. **Has explicit dependencies** (you know what must finish before you start)
4. **Produces concrete output** (code file, test file, documentation)
5. **Doesn't depend on multiple other tasks** (minimizes blocking)

### Task vs. Phase vs. Requirement

| Requirement (Spec) | Phase (Plan) | Task (Tasks) |
|---|---|---|
| "System calculates weighted grades" | "Phase 2: Implement grading algorithm" | Task 2.1: Code weighted_grade() function<br/>Task 2.2: Write unit tests<br/>Task 2.3: Test with sample data |
| High-level business need | Medium-level implementation step | Low-level concrete work unit |
| What matters to stakeholders | What matters to managers | What matters to developers |

---

## Generate Tasks with /sp.tasks 

Like `/sp.specify` and `/sp.plan`, `/sp.tasks` runs **within AI Companion (Gemini CLI/Claude Code)**:

1. Open AI Companion (Gemini CLI/Claude Code)
2. Run 

```/sp.tasks
Break plan into small tasks, each ≤ 3 minutes, testable, reversible.
Add dependencies between tasks; group into phases; mark deliverables per task. Group tasks by operations and for each operation like add use TDD approach so RED Tests, Green Tests and Refactor. After each group we pause for human review and on approval commit to github.

Focus on:
- Use Context7 MCP server and pull documentations instead of assuming you know the best.
- Use CLI commands where it;s more efficient than files manual creation.
- TDD approach (tests first for each operation)
- Small, step-by-step implementation
- Clear task dependencies
- Easy to undo changes
```

**Agent Does:**

- Breaks plan into numbered tasks (T001, T002, etc.)
- Groups tasks into logical phases
- Establishes task dependencies
- Creates tasks.md file with detailed breakdown

3. Review the tasks and manually update or ask gemini. Never rely on the initial output as magical best solution.


```
update @specs/001-feature-basic-calculator/tasks.md so For first task we will make project in same directory using uv init command. And after each user story we shall pause to ask for Human Review as an explicit task and on approval commit or iterate as requested
```

**Why This Matters:**
Now the individual operations become TASKS (T001, T002, etc.). Each task is small and testable. This is where the granularity from vibe coding appears, but with better structure and planning behind it.

---

## Try With AI

### Workflow

1. **Run `/sp.tasks`** on your plan
2. **Read the task checklist**: Get oriented
3. **Pick one task**: Task 1.1 or Task 2.1
4. **Ask**: "What exactly would I do to complete [task name]?"
5. **Trace backward**: "Which requirement from my spec does this task address?"