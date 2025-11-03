---
title: "Decompose Your Spec Into Atomic Tasks"
chapter: 31
lesson: 5
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

## What You're About to Do

Turn your plan into 6–10 atomic, testable tasks with clear dependencies and acceptance checks.

---

## Part 1: Understanding Task Atomicity

Guidelines: 2–4 hours per task, specific acceptance criteria, no “TBDs,” explicit dependencies.

---

## Part 2: Generate Tasks

Use `/sp.tasks` or draft manually from your plan; ensure traceability to spec requirements and acceptance criteria.

---

## Part 3: Choose Your Vertical Slice (Implement Next Lesson)

Pick ONE cohesive flow to implement now:

- Option A (recommended): Rubric → Submission → Grade + Feedback
  - Rubric CRUD (3–5 criteria) persisted to JSON
  - Submission validation for essays (.txt, ≤10k words) and code (.py, ≤1k lines)
  - Grade compute + 50–200 word template feedback referencing ≥2 criteria
- Option B: Submission → Grade (assume pre‑existing rubric)
- Option C: Rubric → Grade history (skip submission validation)

Constraints:
- Pure Python (stdlib), JSON persistence, CLI interaction acceptable
- Each step must have acceptance tests mapped to criteria

Add tasks for the slice (2–4 hours each), mark them in your `tasks.md` with a [slice] tag.

---

## Acceptance Checklist (Lesson 5)

- [ ] Generated tasks.md or decomposed plan into 6–10 atomic tasks
- [ ] Each task duration 2–4 hours with clear acceptance criteria
- [ ] Dependencies and critical path identified
- [ ] Traceability: each task maps to a spec requirement
- [ ] Prepared to open PR with tasks and plan artifacts
- [ ] Vertical slice selected and tagged in tasks.md
