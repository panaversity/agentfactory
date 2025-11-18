---
id: 0005
title: Chapter 31 Spec Redesign - Natural Progression
stage: spec
date: 2025-11-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 10-chapter-31-redesign
branch: 10-chapter-31-redesign
user: mjs
command: /sp.specify
labels:
  [
    "chapter-31",
    "spec-redesign",
    "natural-progression",
    "chapter-30-handoff",
    "spec-kit-plus",
  ]
links:
  spec: specs/10-chapter-31-redesign/spec.md
  ticket: null
  adr: history/adr/001-chapter-31-spec-kit-plus-workflow-pedagogy.md
  pr: null
files:
  - specs/10-chapter-31-redesign/spec.md (updated)
tests:
  - none (specification refinement)
---

## Prompt

Now we will redesign the chapter 31 the core of lessons look good but we are ready to iterate over it as well. Let's carefully plan what the good looks like following the preface goals and natural progression where chapter 30 last lesson established specifyplus and introduced 4 different tools where just one was specifyplus

## Response snapshot

Redesigned Chapter 31 specification to reflect natural progression from Chapter 30's tool landscape introduction.

**Critical Context Applied**:

- Chapter 30 Lesson 6 introduced 4 tools: Kiro (simple), Spec-Kit (GitHub), **Spec-Kit Plus** (Panaversity), Tessel (spec-as-source)
- Students compared tools and understood why Spec-Kit Plus was chosen (GitHub base + ADRs + PHRs + Intelligence Templates)
- Chapter 31 must transition from understanding tools to **practicing the Spec-Kit Plus workflow**

**Spec Refinements Made**:

1. **Added "Natural Progression from Chapter 30" section**:

   - Explicit bridge from Chapter 30's 6 lessons
   - Clarifies transition: manual spec-writing (Ch 30) → tooled workflow (Ch 31)
   - States what Chapter 31 delivers that Chapter 30 didn't

2. **Updated Executive Summary**:

   - Removed vague "SpecifyPlus" references
   - Made explicit: This is **Spec-Kit Plus** (one of four tools compared)
   - Referenced ADR-001 pedagogical architecture directly
   - Emphasized workflow isomorphism principle

3. **Revised Lesson Structure (7 core + 1 capstone = 8 total)**:

   - **Lesson 1**: Introduction & Ch 30→31 Bridge (recap 4 tools, why Spec-Kit Plus)
   - **Lesson 2**: Constitution Phase (Education Intelligence Template)
   - **Lesson 3**: Specify Phase (complete grading system spec)
   - **Lesson 4**: Clarify Phase (/sp.clarify to refine spec)
   - **Lesson 5**: Plan Phase (/sp.plan + ADR creation)
   - **Lesson 6**: Tasks Phase (/sp.tasks decomposition)
   - **Lesson 7**: Implement Phase (/sp.implement + PHR tracking)
   - **Lesson 8**: Capstone Integration (independent workflow application)

4. **Enhanced Success Criteria (8 criteria)**:
   - SC-001: Natural Progression from Chapter 30 (explicit tool recap)
   - SC-002: Workflow Isomorphism Achievement (lessons mirror phases)
   - SC-003: ADR-001 Pedagogical Architecture (grading system, 8 lessons, B1→B2)
   - SC-004: Preface AIDD Paradigm (co-learning, validation, spec-as-interface)
   - SC-005: Cascade Effect Proven (spec quality → code quality demonstrated)
   - SC-006: Complete Artifact Creation (Constitution, Spec, ADRs, PHRs, etc.)
   - SC-007: Correct Tool Usage (no hallucinated commands)
   - SC-008: Foundation for Chapter 32 (multi-component enables parallelization)

**Alignment Validated**:

- ✅ Preface AIDD paradigm: Co-learning cycle, human as architect/validator
- ✅ ADR-001 decisions: Grading system, 7-lesson workflow, Tier 2→3 transition
- ✅ Chapter 30 handoff: Tool comparison → chosen tool practice
- ✅ Workflow isomorphism: Each lesson = one actual workflow phase

## Outcome

- ✅ Impact: Specification updated to reflect natural Chapter 30→31 progression; clarified Spec-Kit Plus as chosen tool from 4-tool comparison; aligned with preface AIDD paradigm and ADR-001 pedagogy decisions
- 🧪 Tests: Validated against 8 success criteria; confirmed alignment with preface goals, Chapter 30 content, and ADR-001 architecture
- 📁 Files: Updated specs/10-chapter-31-redesign/spec.md (Executive Summary, Natural Progression section, Lesson Structure, Success Criteria)
- 🔁 Next prompts: Run /sp.clarify to identify specification gaps; create checklist to validate spec completeness before planning; generate plan.md implementing 8-lesson structure
- 🧠 Reflection: The "4 tools where only one was Spec-Kit Plus" context was critical - Chapter 31 isn't introducing a new tool, it's practicing the tool students already chose in Chapter 30 Lesson 6. Lesson 1 must explicitly recap this decision to prevent confusion. Natural progression matters more than isolated lesson quality.

## Evaluation notes (flywheel)

- Failure modes observed: Initial spec lacked explicit Chapter 30 handoff; didn't clarify tool selection context; could confuse students about which tool they're using
- Graders run and results (PASS/FAIL): PASS - Spec now explicitly bridges Chapter 30→31; clarifies Spec-Kit Plus as chosen tool; validates against preface and ADR-001
- Prompt variant (if applicable): User emphasized "natural progression" and "chapter 30 last lesson established specifyplus and introduced 4 different tools" - critical context that shaped redesign
- Next experiment (smallest change to try): Test whether Lesson 1's tool recap is sufficient or if we need a visual diagram comparing the 4 tools again
