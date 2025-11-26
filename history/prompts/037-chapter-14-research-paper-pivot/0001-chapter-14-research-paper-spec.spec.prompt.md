---
id: "0001"
title: "Chapter 14 Research Paper Specification"
stage: spec
date: 2025-11-26
surface: claude-code
model: claude-opus-4-5-20251101
feature: 037-chapter-14-research-paper-pivot
branch: 037-chapter-14-research-paper-pivot
user: human
command: /sp.specify
labels: [chapter-14, sdd-ri, research-paper, pivot]
links:
  spec: specs/037-chapter-14-research-paper-pivot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/037-chapter-14-research-paper-pivot/spec.md
  - specs/037-chapter-14-research-paper-pivot/checklists/requirements.md
tests:
  - spec-architect validation: PASS (9/10)
---

## Prompt

User requested pivot of Chapter 14 from video generation project to research paper project. Context:
- Original design used Gemini.google.com + Playwright MCP for AI video generation
- Problems: MCP not taught until Ch38, external dependencies, cognitive overload
- User feedback: "Keep things simple", "No Python yet", "Your direction looks wrong"

Task: Create specification for Chapter 14 using Spec-Kit Plus workflow with simple research paper project that uses ONLY tools students already have (AI companion, bash, git, markdown).

## Response snapshot

Created comprehensive specification covering:
- 3 user stories (P1: workflow execution, P1: skill creation, P2: intelligence acceleration)
- 30 functional requirements (tool constraints, workflow coverage, pedagogical requirements)
- 13 success criteria with measurable thresholds
- Constitutional alignment (Section IIa, Principles 2, 3, 6, 7)
- Edge cases, assumptions, dependencies, non-goals

spec-architect validation: READY FOR PLANNING (9/10)

## Outcome

- Impact: Major pivot - chapter now teachable without external dependencies
- Tests: spec-architect validation PASS
- Files: spec.md (340 lines), requirements.md checklist
- Next prompts: /sp.plan 037-chapter-14-research-paper-pivot
- Reflection: Research paper is ideal domain - uses only markdown + AI, directly relevant to student interests

## Evaluation notes (flywheel)

- Failure modes observed: None - spec covers all constitutional requirements
- Graders run and results (PASS/FAIL): spec-architect PASS (9/10)
- Prompt variant (if applicable): N/A
- Next experiment: Proceed to planning phase
