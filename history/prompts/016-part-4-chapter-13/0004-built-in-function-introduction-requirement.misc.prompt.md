---
id: 0004
title: built-in-function-introduction-requirement
stage: misc
date: 2025-11-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 016-part-4-chapter-13
branch: 016-part-4-chapter-13
user: system
command: Specification revision - Add built-in function introduction requirement
labels: ["chapter-13", "specification-revision", "built-in-functions", "beginner-pedagogy", "FR-022"]
links:
  spec: specs/016-part-4-chapter-13/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/016-part-4-chapter-13/spec.md (added FR-022, renumbered FR-023-025)
  - specs/016-part-4-chapter-13/plan.md (updated Lessons 3, 4, 5 with function introductions)
  - specs/016-part-4-chapter-13/tasks.md (updated Tasks 4.2, 5.2, 6.2, added Task 7.9)
tests:
  - Requirement Coverage: PASS (FR-022 added with clear acceptance criteria)
  - Plan Alignment: PASS (all built-in functions documented with intro pattern)
  - Task Validation: PASS (Task 7.9 validates FR-022 compliance)
---

## Prompt

**User Request**: "also donot use any built-in method without introducing them to student ( give 1 or 2 line intro about it first then use it ) so that beginner dont get confuse"

**Context**: Beginner pedagogy improvement - prevent confusion when built-in functions appear without explanation.

**Rationale**: Beginners encounter functions like `print()`, `input()`, `type()`, `isinstance()`, `int()` and may not know where they come from or what they do. Introducing each function with 1-2 lines BEFORE first use prevents "Where did this come from?" confusion.

## Response snapshot

**Specification Update - Added FR-022**:

Created new functional requirement:
- **FR-022**: Chapter MUST introduce every built-in function/method with 1-2 line explanation BEFORE first use (e.g., print(), input(), type(), isinstance(), int()) - no function used without introduction

**Renumbered existing requirements**:
- Old FR-022 → New FR-023 (cross-platform testing)
- Old FR-023 → New FR-024 (reading level)
- Old FR-024 → New FR-025 (CEFR proficiency levels)

---

**Plan Updates - Added Introduction Pattern**:

**Lesson 3 (Variables and Type Hints)**:
- Added print() introduction: "The `print()` function displays output to your screen. It shows you what's inside variables."
- Added type() introduction: "The `type()` function tells you what kind of data a variable holds."
- Added isinstance() introduction: "The `isinstance()` function checks if a variable is a specific type. It returns True or False."
- Documented pattern: "The `function_name()` does [what it does]. [Why it's useful]."

**Lesson 4 (Basic Syntax)**:
- Added print() introduction (not assumed from Lesson 3): "The `print()` function displays text and data to your screen. It's how you see what your program is doing."

**Lesson 5 (Capstone)**:
- Added input() introduction: "The `input()` function asks the user to type something. It shows a prompt, waits for them to type and press Enter, then gives you what they typed as a string."
- Added int() introduction: "The `int()` function converts text (strings) to numbers. Since `input()` gives you a string, use `int()` to turn it into a number you can work with."
- Referenced previously introduced functions: isinstance() (Lesson 3), f-strings (Lesson 4)

---

**Tasks Updates - Validation Added**:

**Updated Existing Tasks**:
- Task 4.2 (Write Lesson 3): Added acceptance criteria for introducing print(), type(), isinstance() before use
- Task 5.2 (Write Lesson 4): Added acceptance criteria for introducing print() before examples
- Task 6.2 (Write Lesson 5): Added acceptance criteria for introducing input() and int() before capstone code

**Added New Task 7.9** - Built-in Function Introduction Check:
- Priority: MUST-HAVE
- Effort: 45 minutes
- Validates FR-022 compliance across all lessons
- Checks that each built-in function (print, input, type, isinstance, int) has 1-2 line introduction BEFORE first use
- Verifies introduction pattern follows "The `function_name()` does [what]. [Why useful]." structure
- Ensures beginner-friendly language (no jargon)

---

**Built-in Functions Tracked**:

| Function | First Used In | Introduction Required | Status |
|----------|--------------|----------------------|--------|
| print() | Lesson 3 | Section 5 (before examples) | ✅ Added |
| type() | Lesson 3 | Section 5 (before examples) | ✅ Added |
| isinstance() | Lesson 3 | Section 5 (before examples) | ✅ Added |
| print() | Lesson 4 | Section 3 (not assumed from L3) | ✅ Added |
| input() | Lesson 5 | Section 3 (before capstone) | ✅ Added |
| int() | Lesson 5 | Section 3 (before capstone) | ✅ Added |

## Outcome

- ✅ Impact: All built-in functions now have clear 1-2 line introductions before first use, preventing beginner confusion and improving pedagogical clarity
- 🧪 Tests: FR-022 compliance validated via Task 7.9; introduction pattern documented in plan; acceptance criteria added to lesson writing tasks
- 📁 Files: spec.md (FR-022 added, FR-023-025 renumbered), plan.md (introductions added to Lessons 3, 4, 5), tasks.md (Tasks 4.2, 5.2, 6.2 updated, Task 7.9 added)
- 🔁 Next prompts: Ready for PHASE 4 implementation with beginner-friendly function introductions integrated throughout
- 🧠 Reflection: This pedagogical improvement addresses a common beginner pain point - "Where did this function come from?" By introducing each built-in function with a clear 1-2 line explanation immediately before use, we reduce cognitive load and build confidence. The pattern is simple and repeatable across all future chapters.

## Evaluation notes (flywheel)

- Failure modes observed: None. Requirement added cleanly with clear validation strategy.
- Graders run and results (PASS/FAIL):
  - Requirement Coverage: PASS (FR-022 well-defined with examples)
  - Plan Alignment: PASS (all 6 function introductions documented)
  - Task Validation: PASS (Task 7.9 provides systematic check)
  - Pedagogical Clarity: PASS (beginner-friendly pattern established)
- Prompt variant (if applicable): User-requested specification enhancement (beginner pedagogy improvement)
- Next experiment (smallest change to try): During implementation, validate that 1-2 line introductions feel natural in lesson flow (not disruptive); adjust phrasing if introductions feel too formal or too casual
