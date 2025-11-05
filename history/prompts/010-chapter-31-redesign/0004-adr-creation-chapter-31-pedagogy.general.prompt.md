---
id: 0004
title: ADR Creation - Chapter 31 Pedagogy
stage: general
date: 2025-11-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: ch-31-specifyplus-redesign
branch: main
user: mjs
command: /sp.adr
labels: ["adr", "chapter-31", "pedagogy", "architecture-decision", "spec-kit-plus"]
links:
  spec: null
  ticket: null
  adr: history/adr/001-chapter-31-spec-kit-plus-workflow-pedagogy.md
  pr: null
files:
  - history/adr/001-chapter-31-spec-kit-plus-workflow-pedagogy.md
tests:
  - none (ADR documentation)
---

## Prompt

let' record our adrs

## Response snapshot

Created ADR-001 documenting the architectural decision for Chapter 31's pedagogical approach.

**Decision Context:**
- Chapter 30 established SDD fundamentals (theory + calculator practice)
- Chapter 31 must transition readers from understanding to professional practice
- Need to choose: project complexity, lesson structure, artifact creation pattern

**Decision Made:**
Teach complete 7-phase Spec-Kit Plus workflow through grading system capstone project with this structure:
- 7 lessons mirroring actual workflow (Introduction → Constitution → Specify → Plan → Tasks → Execute → Validate)
- Grading system project (multi-component, production-ready complexity)
- Tier 2→3 complexity transition (Intermediate to Advanced)
- B1→B2 proficiency target (Independent application to Analysis/Evaluation)
- Create: Constitution + Spec + Plan + Tasks + 5+ ADRs + 10+ PHRs

**Alternatives Considered:**
1. Simple calculator extension (rejected: insufficient complexity)
2. Multiple small projects (rejected: fragmented learning, no sustained capstone)
3. Enterprise LMS (rejected: too complex, violates graduated progression)
4. Pure workflow tutorial (rejected: theory-heavy, no hands-on practice)

**Why This Won:**
- Goldilocks complexity (justifies professional SDD without overwhelming)
- Workflow isomorphism (lesson structure = actual process)
- Cumulative achievement (single sustained project builds confidence)
- Natural ADR/PHR creation (grading decisions genuinely need documentation)
- Chapter 32 foundation (multi-component enables parallelization lesson)
- Domain familiarity (all readers understand grading)

**Significance Test:**
✅ Impact: Determines how thousands learn professional SDD practice
✅ Alternatives: 4 distinct pedagogical sequences evaluated
✅ Scope: Affects entire chapter (7 lessons, capstone, artifact strategy)

## Outcome

- ✅ Impact: First ADR created for ch-31-specifyplus-redesign feature; establishes pedagogical architecture for Chapter 31 redesign; documents rationale for grading system + 7-lesson structure
- 🧪 Tests: ADR significance test PASSED (impact + alternatives + scope all met)
- 📁 Files: Created ADR-001 in history/adr/001-chapter-31-spec-kit-plus-workflow-pedagogy.md
- 🔁 Next prompts: Draft Chapter 31 lesson outlines based on ADR-001 decisions; create specification for grading system capstone; design Intelligence Template integration
- 🧠 Reflection: ADR creation without feature branch context worked because this is a meta-decision about book architecture. Standard /sp.adr workflow expects feature branches, but architectural decisions about pedagogy are global and belong in main branch. Future ADRs for actual feature work should follow feature branch workflow.

## Evaluation notes (flywheel)

- Failure modes observed: Initial /sp.adr invocation failed due to main branch (not feature branch); adapted by manually creating ADR using template
- Graders run and results (PASS/FAIL): PASS - ADR meets clustering requirement (pedagogical architecture as integrated decision), lists 4 explicit alternatives with rationale, includes clear pros/cons
- Prompt variant (if applicable): Manual ADR creation (bypassed /sp.adr script due to branch constraint)
- Next experiment (smallest change to try): Test /sp.adr workflow on actual feature branch to validate standard script flow; consider whether pedagogical ADRs should have different routing than implementation ADRs
