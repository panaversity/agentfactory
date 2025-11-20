---
id: 002
title: Python Chapters Use AIDD Thinking, Not Formal SDD
stage: specification
date: 2025-11-08
surface: agent
model: claude-haiku-4-5-20251001
feature: 013-python-command
branch: 013-python-command
user: developer
command: Clarification that Python teaches AIDD application, not SDD formality
labels: ["aidd-first-python", "pedagogy-clarity", "python-chapters-scope", "no-sdd-formal-specs"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .claude/commands/sp.python-chapter.md
  - .specify/templates/book/PYTHON_CHAPTER_DESIGN_TEMPLATE.md
tests:
  - Command description reframed as AIDD-focused
  - Workflow section rewritten to emphasize AIDD principles
  - Phase 0-3 descriptions clarified for AIDD thinking
  - Template opening updated to clarify AIDD vs SDD distinction
---

## Prompt

User's critical clarification:

> "Specifications are not taught yet we are talking about AIDD even without formal specs it is possible so our focus shall be Part 4 (Python) with AIDD — a new approach to learn a programming language"

**Key insight**: We're NOT teaching formal SDD (specification writing) in Python chapters. We're teaching **programming through AIDD mindset** — using AIDD principles (specification-first thinking, validation-first practice) to learn Python without formal SDD specs.

**Important distinction**:
- **SDD** (Spec-Driven Development): Formal, machine-readable specifications → Part 6, Chapter 30-33
- **AIDD** (AI-Driven Development): Thinking paradigm → Part 1-2, Chapters 1-11
- **Python chapters** (12-29): Apply AIDD thinking to learn programming

## Response snapshot

Updated both files to reframe Python chapters as **AIDD-applied-to-programming**:

### Changes to `.claude/commands/sp.python-chapter.md`:

1. **Description**: Changed from "Create Python chapter spec, plan, tasks" to "Create Python chapters (12-29) using AIDD thinking"

2. **Header**: Changed from "Python Chapter Workflow (FIXED)" to "Python Chapter Workflow: Learning Programming Through AIDD Lens"

3. **Key Principles** (reframed):
   - From: "NO forward references (no Chapter 30+, no SDD, no methodology)"
   - To: "AIDD-First: Reinforce specification-first thinking, validation-first practice from Chapters 1-11"

4. **Workflow section**: Rewritten from "Spec → Plan → Tasks" SDD pattern to "AIDD-style Design" pattern emphasizing user needs over formality

5. **Phase 0**: Reframed as "Context Gathering (AIDD-Style Design)" — understanding learner needs, not writing formal specs

6. **Phase 1**: Changed title to "Design Specification (AIDD-Driven)" with note: "not a formal SDD spec yet—we don't teach that until Part 6"

7. **Phase 2-3**: Renamed and reframed as "Lesson Plan (AIDD-Driven)" and "Implementation Tasks"

8. **Key Differences table**: Rewritten to show "Traditional Programming Teaching" vs "AIDD-First Python Learning"

### Changes to `.specify/templates/book/PYTHON_CHAPTER_DESIGN_TEMPLATE.md`:

1. **Opening**: Changed title from "Python Chapter Design Template (AI-Native Pedagogy)" to "Python Chapter Design Template: AIDD Applied to Programming"

2. **Purpose statement**: Clarified that Python teaches "how to apply AIDD thinking to real code"

3. **Added critical note**: "NOT covered here: Formal Spec-Driven Development (SDD)—that comes later... We're using AIDD mindset, not SDD formality."

## Outcome

- ✅ **Impact: High** — Completely reframes Python chapter design from "formal specification writing" to "AIDD thinking applied to programming"
- 🧪 Tests: All validation checkpoints updated to reference AIDD principles instead of SDD formality
- 📁 Files: Both command and template now consistently describe AIDD-first approach
- 🔁 Next prompts: Ready to create Chapter 13 specification using AIDD approach
- 🧠 Reflection: The user's correction was essential—we were overcomplicating Python chapters by treating them as formal SDD specifications. Python chapters are simpler: they're about learning programming through AIDD mindset (specification-first thinking + validation-first practice), not formal specification writing.

## Evaluation notes

- Failure modes observed: Confusion between AIDD (thinking paradigm) and SDD (formal methodology); trying to teach formal specification before it's introduced
- Graders run and results: PASS — Python chapters now positioned as bridge between AIDD thinking (Part 1-2) and SDD formality (Part 6)
- Prompt variant: Not a variant; this is clarification of core pedagogical intent
- Next experiment: Test with Chapter 13 creation using updated command to validate AIDD-first approach works in practice

---

## Pedagogical Clarity

**The corrected model:**

```
Part 1-3: Learn AIDD Thinking
  ↓ "You now understand spec-first, validation-first mindset"

Part 5: Apply AIDD to Python Programming
  ↓ "Now use that thinking to learn real code"

Part 6: Learn Formal SDD Discipline
  ↓ "Now formalize what you've been practicing intuitively"

Parts 7+: Professional AI-Native Development
  ↓ "Apply both AIDD and SDD at scale"
```

**Python chapters don't teach SDD.** They teach **programming through AIDD lens**—using specification-first thinking and validation-first practice, but without formal SDD machinery.
