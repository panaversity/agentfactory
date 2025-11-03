---
title: "Build Your Grading System Spec End-to-End"
chapter: 31
lesson: 6
duration: "3 hours"
skills:
  - name: "End-to-End Specification"
    proficiency: "B1"
    category: "Technical"
  - name: "Complex System Modeling"
    proficiency: "B1"
    category: "Technical"
  - name: "SDD Loop Application"
    proficiency: "B1"
    category: "Conceptual"
learning_objectives:
  - "Build a complete grading system specification from problem to implementation (B1)"
  - "Apply Spec-Kit Plus tools to a real-world problem (B1)"
  - "Execute the full SDD loop: Spec → Plan → Decompose → Validate (B1)"
---

# Build Your Grading System Spec End-to-End

## What You're About to Do

Integrate your skills to produce a complete spec and implement a minimal vertical slice to validate it end‑to‑end.

---

## Part 1: Finalize Your Specification

Complete `specs/grading-system/spec.md` using your checklists (Users, Scope, Success Criteria, Constraints, Error Handling, Acceptance Criteria). Resolve any [NEEDS CLARIFICATION] markers before proceeding.

---

## Part 2: Implement the Vertical Slice (Pure Python)

Scope (recommended Option A):
- Rubric CRUD (3–5 criteria) persisted to JSON
- Submission validation (essays .txt ≤10k words; code .py ≤1k lines)
- Grade compute + 50–200 word template feedback referencing ≥2 criteria

Guidelines:
- Pure Python 3.9+, stdlib only; JSON files for persistence
- Simple CLI acceptable; focus on matching acceptance criteria
- Write tests that map 1:1 to acceptance criteria

Suggested file layout:
```
src/
  rubric.py
  submission.py
  grading.py
  feedback_templates.py
  storage.py
tests/
  test_rubric.py
  test_submission.py
  test_grading.py
```

---

## Part 3: Tests Mapped to Acceptance Criteria

Create tests that directly mirror your acceptance checklist (names/descriptions should reference criteria). Keep fixtures small and deterministic.

---

## Part 4: Validate the Slice

- Run tests; confirm pass/fail against each criterion
- If a test fails due to ambiguity, refine the spec (update acceptance text) and re‑run

---

## Acceptance Checklist (Lesson 6)

- [ ] Spec finalized; no critical gaps
- [ ] Vertical slice implemented (pure Python, stdlib, JSON)
- [ ] Tests cover rubric, submission validation, grading + feedback
- [ ] Each test maps to a specific acceptance criterion
- [ ] Slice behaves according to performance/limits in spec

References:
- Analyze & Clarify: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/08_analyze_and_clarify/readme.md`
- Implementation: `https://github.com/panaversity/spec-kit-plus/blob/main/docs-plus/06_core_commands/07_implementation/readme.md`
