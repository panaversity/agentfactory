---
id: 0001
title: chapter-14-sdd-ri-hands-on-specification
stage: spec
date: 2025-11-25
surface: claude-code
model: claude-opus-4-5
feature: 036-chapter-14-sdd-ri-hands-on
branch: 036-chapter-14-sdd-ri-hands-on
user: mjs
command: /sp.loopflow.v2 → /sp.specify
labels: [educational-content, sdd-ri, chapter-14, part-4, hands-on]
links:
  spec: specs/036-chapter-14-sdd-ri-hands-on/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/036-chapter-14-sdd-ri-hands-on/spec.md
    status: created
    lines: 340
  - path: specs/036-chapter-14-sdd-ri-hands-on/checklists/requirements.md
    status: created
    lines: 75
tests: []
---

## Prompt

User executed `/sp.loopflow.v2` with intent to create Chapter 14: SDD-RI Hands-On with Spec-Kit Plus.

**User Description**:
> So chapter 14 is speckitplus to build something make videos using playwright mcp on gemini.google.com and then upload them on youtube. So user login both accounts on playwright browser - mock ones. And it have skills like upload-youtube generate-video and other things. Maybe we start with simple specit workflow to just generate video so /sp.specify this video is for this idea - business. /sp.plan we use gemini.google.com with playwright mcp as its free then /sp.tasks and then /sp.implement. Once all done and user have video downloaded locally by their ai companion then the user asks their subagent do you see any reusable intelligence components from this session and convert them to skills. The Capstone asks to run same loop for uploading this video on youtube and research to have subagents and skills here as well or just skills so after this chapter user understand SDD, SDD-RI, SpeckitPlus for SDD-RI and Hands on SpecKitPlus of SDD-RI and have an end to end project to showcase all learned using panaversity teaching principles aligned with our book philosophy and goals.

**Clarification Questions Asked**:
1. Video Domain → Answer: Product Demo Videos
2. RI Complexity → Answer: Skills Only (no subagents for A2-B1)
3. Auth Setup → Answer: Session Persistence

## Response snapshot

Created comprehensive specification (340 lines) for Chapter 14: SDD-RI Hands-On with Spec-Kit Plus.

**Key Components**:
- Constitutional Grounding: Mapped to Principles 2, 3, 6, 7 and Section IIa
- Anti-Convergence: Ch 13 = problem-discovery → Ch 14 = hands-on project
- 3 User Stories: SDD workflow execution, Reusable intelligence creation, Capstone completion
- 21 Functional Requirements: Chapter structure, workflow coverage, RI creation, technical implementation, pedagogical requirements
- 14 Success Criteria: Quantified thresholds (80%, 90%, 100%)
- Edge Cases: Playwright MCP availability, authentication, video quality, skills creation
- Non-Goals: 6 explicit exclusions with rationale

**spec-architect Validation**:
- Verdict: READY FOR PLANNING
- Testability: 10/10 (all criteria falsifiable)
- Completeness: 10/10 (constraints, non-goals, edge cases present)
- Ambiguity: 9/10 (clear terms, Open Questions appropriate for planning)
- Traceability: 9/10 (prerequisites, constitution mapped)

## Outcome

- ✅ Impact: Spec created and validated. Chapter 14 will teach SDD-RI hands-on through video generation project with Playwright MCP + Gemini.google.com, culminating in skills creation (generate-video, upload-youtube) and YouTube upload capstone.
- 🧪 Tests: spec-architect validation passed all quality gates
- 📁 Files: Created spec.md (340 lines), checklists/requirements.md (75 lines)
- 🔁 Next prompts: `/sp.plan 036-chapter-14-sdd-ri-hands-on` to generate lesson structure
- 🧠 Reflection: LoopFlow Phase 0 (Constitutional Reasoning) correctly identified anti-convergence requirement (Ch 13 = theory, Ch 14 = hands-on). Clarification questions focused on genuine ambiguities (domain, RI complexity, auth approach).

## Evaluation notes (flywheel)

- Failure modes observed: None — spec generated with proper reasoning activation
- Graders run and results (PASS/FAIL): spec-architect PASS (10/10, 10/10, 9/10, 9/10, PASS)
- Prompt variant (if applicable): LoopFlow v2.0 with reasoning-activated prompts
- Next experiment (smallest change to try): Proceed to /sp.plan to test planning phase reasoning activation
