---
id: 0006
title: Chapter 31 Spec Clarify - Terminology & Calculator
stage: spec
date: 2025-11-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 10-chapter-31-redesign
branch: 10-chapter-31-redesign
user: mjs
command: /sp.clarify
labels:
  [
    "chapter-31",
    "spec-clarify",
    "terminology-standardization",
    "calculator-capstone",
    "vertical-intelligence",
  ]
links:
  spec: specs/10-chapter-31-redesign/spec.md
  ticket: null
  adr: history/adr/001-chapter-31-spec-kit-plus-workflow-pedagogy.md
  pr: null
files:
  - specs/10-chapter-31-redesign/spec.md (updated - terminology, project, lessons)
tests:
  - none (specification refinement)
---

## Prompt

/sp.clarify you can run techincal review and then we can discuss how to redesign

**Follow-up user clarifications:**

1. Spec Kit Plus
2. We build it using /sp.specify command or here I though we build a calculator to understand the core. Yes it's unclear
3. It's Vertical Intelligence where we have an AI orchestrastor and subagents and these agents have specialized skills a human works with ai orchestrator as collaborator who uses these.

## Response snapshot

Performed comprehensive technical review identifying three critical issues:

**CRITICAL Issue: Terminology Inconsistency**

- Spec used both "SpecifyPlus" and "Spec-Kit Plus" interchangeably
- User clarified canonical name: "Spec Kit Plus"
- **Resolution**: Systematically replaced all "SpecifyPlus" with "Spec-Kit Plus" throughout specification

**MAJOR Issue: Undefined Capstone Project**

- Spec referenced "grading system" as capstone project (from ADR-001)
- User clarified: "we build a calculator to understand the core"
- **Resolution**: Changed all references from grading system to calculator project (5 operations: add, subtract, multiply, divide, power)

**MAJOR Issue: Intelligence Templates Undefined**

- Spec mentioned "Intelligence Templates" without explanation
- User clarified: "Vertical Intelligence where we have an AI orchestrastor and subagents and these agents have specialized skills"
- **Resolution**: Added new section "Vertical Intelligence: AI Orchestrator + Specialized Subagents" with architecture diagram and workflow examples

**Spec Updates Completed:**

1. **Title & Executive Summary**: Changed "SpecifyPlus" to "Spec-Kit Plus"; updated to calculator capstone
2. **New Section Added**: Vertical Intelligence architecture (lines 56-91)
3. **User Stories**: Updated to calculator project, Vertical Intelligence collaboration
4. **Lesson Descriptions (1-8)**: Updated all lessons to:
   - Calculator project context (not grading system)
   - Vertical Intelligence mentions (orchestrator delegating to subagents)
   - Correct command usage (`/sp.clarify` not `/sp.specify`)
5. **Success Criteria**: Changed SC-003, SC-006, SC-008 to reflect calculator
6. **Assumptions**: Added calculator domain familiarity
7. **Constraints & Risks**: Updated to calculator context
8. **Learning Objectives**: Added Vertical Intelligence learning objective
9. **Dependencies**: Updated to "Spec-Kit Plus (via Claude Code)"
10. **Acceptance Criteria**: Added Vertical Intelligence requirement

**Verification:**

- ✅ Grepped for "SpecifyPlus" - 0 matches (all standardized)
- ✅ Grepped for "grading system" - 0 matches (all changed to calculator)

## Outcome

- ✅ Impact: Specification terminology standardized to "Spec-Kit Plus"; project changed from grading system to calculator (simpler, better for core understanding); Vertical Intelligence architecture clearly defined
- 🧪 Tests: Verified via grep: 0 "SpecifyPlus" occurrences, 0 "grading system" occurrences
- 📁 Files: Updated specs/10-chapter-31-redesign/spec.md (title, summary, lessons 1-8, success criteria, learning objectives, dependencies, acceptance criteria)
- 🔁 Next prompts: Run /sp.plan to generate chapter plan based on updated spec; validate lesson structure matches calculator project complexity
- 🧠 Reflection: User clarifications were critical - "SpecifyPlus" vs "Spec-Kit Plus" affects all book content; calculator vs grading system is foundational pedagogical decision (ADR-001 may need update); Vertical Intelligence concept was implicit and needed explicit architectural explanation. Spec quality depends on terminology precision and clear project scope definition.

## Evaluation notes (flywheel)

- Failure modes observed: Initial spec had terminology inconsistency (SpecifyPlus vs Spec-Kit Plus); project scope mismatch with ADR-001 (grading system in ADR but calculator makes more pedagogical sense for Chapter 31); implicit architectural patterns (Vertical Intelligence) that needed explicit documentation
- Graders run and results (PASS/FAIL): PASS - Terminology standardized; project scope clarified to calculator; Vertical Intelligence architecture documented; all lessons updated consistently
- Prompt variant (if applicable): User provided three numbered clarifications addressing critical issues identified in technical review - extremely efficient clarification format
- Next experiment (smallest change to try): Test whether calculator project provides sufficient complexity for ADR creation (2-3 ADRs needed) or if we need to enhance with additional features (e.g., history, chaining operations)
