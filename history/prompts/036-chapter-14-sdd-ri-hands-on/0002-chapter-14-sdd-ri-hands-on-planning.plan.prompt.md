---
id: 0002
title: chapter-14-sdd-ri-hands-on-planning
stage: plan
date: 2025-11-25
surface: claude-code
model: claude-opus-4-5
feature: 036-chapter-14-sdd-ri-hands-on
branch: 036-chapter-14-sdd-ri-hands-on
user: mjs
command: /sp.loopflow.v2 → /sp.plan
labels: [educational-content, sdd-ri, chapter-14, part-4, hands-on, lesson-planning]
links:
  spec: specs/036-chapter-14-sdd-ri-hands-on/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - path: specs/036-chapter-14-sdd-ri-hands-on/plan.md
    status: created
    lines: 1422
tests: []
---

## Prompt

User approved Phase 1 (Specification) and requested continuation with `/sp.plan 036-chapter-14-sdd-ri-hands-on`.

**Context from Phase 1**:
- Spec created with 21 FRs, 14 SCs, 3 user stories
- Core workflow: Video generation + YouTube upload using Playwright MCP + Gemini.google.com
- Skills to create: `generate-video`, `upload-youtube` (P+Q+P framework)
- Proficiency tier: A2-B1 (max 7 concepts/section)
- Anti-convergence: Chapter 13 = problem-discovery → Chapter 14 = hands-on project

**Planning Phase Requirements**:
- Design 7-9 lessons following 4-Layer progression (L1 → L2 → L3 → L4)
- Ensure modality variation from Chapter 13
- Map lessons to constitutional frameworks (Three Roles invisible, cognitive load limits)
- Create intelligence accumulation arc (workflow → patterns → skills → composition)

## Response snapshot

Created comprehensive lesson plan (1,422 lines) with 9-lesson architecture:

**Pedagogical Architecture**:
- **Layer 1 (Foundation)**: Lessons 1-2 - Context + Manual Specification Writing
- **Layer 2 (Collaboration)**: Lessons 3-5 - Planning → Tasks → Implementation (Three Roles invisible)
- **Layer 3 (Intelligence)**: Lessons 6-7 - Pattern Recognition → Skill Creation (P+Q+P)
- **Layer 4 (Mastery)**: Lessons 8-9 - YouTube Spec → Capstone Composition

**Key Design Decisions**:
1. **9 lessons justified**: 8 core concepts require 2-3 lessons per layer for A2-B1 competence
2. **Cognitive load validated**: All lessons ≤6 concepts (within A2-B1 limits of 7)
3. **Three Roles invisible**: Student-facing language uses action narratives, not framework labels
4. **Intelligence accumulation arc**: Lessons 1-5 (execute) → 6-7 (extract skills) → 8-9 (reuse)
5. **Modality variation**: Chapter 13 = problem-discovery analysis → Chapter 14 = project-based execution

**Lesson Structure Summary**:
| Lesson | Layer | Focus | Duration |
|--------|-------|-------|----------|
| 1 | L1 | Project context + business framing | 45 min |
| 2 | L1 | Manual specification writing | 90 min |
| 3 | L2 | Plan with AI collaboration | 90 min |
| 4 | L2 | Task breakdown + dependency analysis | 90 min |
| 5 | L2 | Video generation implementation | 180 min |
| 6 | L3 | Pattern recognition from workflow | 90 min |
| 7 | L3 | P+Q+P skill creation | 120 min |
| 8 | L4 | YouTube upload specification | 60 min |
| 9 | L4 | Capstone composition + validation | 120 min |

## Outcome

- ✅ Impact: Comprehensive 9-lesson plan created with 4-Layer progression, constitutional compliance verified. Ready for `/sp.tasks` to generate atomic work units.
- 🧪 Tests: Constitutional compliance checklist PASSED (all 7 principles verified)
- 📁 Files: Created plan.md (1,422 lines) with lesson-by-lesson architecture
- 🔁 Next prompts: `/sp.tasks 036-chapter-14-sdd-ri-hands-on` to generate implementation tasks
- 🧠 Reflection: chapter-planner subagent successfully derived lesson structure from spec requirements. Key insight: Anti-convergence principle correctly identified Chapter 13's "problem-discovery" modality and designed Chapter 14 as "project-based execution" modality.

## Evaluation notes (flywheel)

- Failure modes observed: None — plan generated with proper constitutional reasoning
- Graders run and results (PASS/FAIL): Constitutional compliance checklist PASS (all items checked)
- Prompt variant (if applicable): LoopFlow v2.0 Phase 2 with reasoning-activated prompts
- Next experiment (smallest change to try): Proceed to /sp.tasks to test task generation from plan
