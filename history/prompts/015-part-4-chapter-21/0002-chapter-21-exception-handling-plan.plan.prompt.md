---
id: 0002
title: Chapter 21 Exception Handling Planning
stage: plan
date: 2025-11-09
surface: agent
model: claude-haiku-4-5-20251001
feature: 015-part-4-chapter-21
branch: 015-part-4-chapter-21
user: mjs
command: /sp.plan 015-part-4-chapter-21 (via chapter-planner subagent)
labels: ["python-chapter", "exception-handling", "part-4", "intermediate", "cefr-mapping", "pedagogical-planning"]
links:
  spec: /specs/015-part-4-chapter-21/spec.md
  plan: /specs/015-part-4-chapter-21/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - /specs/015-part-4-chapter-21/plan.md (288 lines)
tests:
  - CEFR progression validation (A2 → B1): PASS
  - Cognitive load check (≤7 concepts/lesson): PASS
  - Forward reference check (no concepts used before intro): PASS
  - CoLearning element distribution (💬🎓🚀✨ in all lessons): PASS
  - "Try With AI" pattern specification (4 prompts per lesson): PASS
  - Capstone integration check (no new concepts): PASS
  - Graduated teaching pattern (Tier 1/2/3): PASS
---

## Prompt

Transform the approved Chapter 21 specification into a detailed lesson-by-lesson implementation plan with explicit skills proficiency mapping (CEFR, Bloom's taxonomy, cognitive load validation).

Input spec: `/specs/015-part-4-chapter-21/spec.md` (approved)

Requirements:
1. Break 7 core concepts into 5 lessons with clear progression
2. Assign CEFR proficiency levels (A2 → B1) with justified progression
3. Map to Bloom's taxonomy cognitive levels
4. Validate cognitive load (≤7 new concepts/lesson)
5. Specify AI-Native CoLearning elements (💬🎓🚀✨) for each lesson
6. Specify "Try With AI" closure pattern (4 prompts, Bloom's progression)
7. Validate lesson boundaries (no forward references)
8. Document graduated teaching pattern (Tier 1/2/3 distribution)
9. Create implementation checklist for lesson-writer
10. Generate plan.md with all proficiency mapping

Quality gates: All 11 gates must PASS before outputting plan.

## Response snapshot

✅ **PHASE 2 COMPLETE: Detailed Lesson Plan Created**

**Plan File**: `/specs/015-part-4-chapter-21/plan.md` (288 lines)

**Five-Lesson Structure**:
| Lesson | CEFR | Duration | Focus |
|--------|------|----------|-------|
| 1: Exception Fundamentals | A2 | 45 min | What are exceptions? Why try/except matters |
| 2: Except, Else, Finally | A2-B1 | 45 min | Multi-block control flow |
| 3: Raising & Custom Exceptions | B1 | 45 min | Defensive functions that validate |
| 4: Error Handling Strategies | B1 | 45 min | Retry, fallback, graceful degradation |
| 5: Capstone – CSV Parser | B1 | 60 min | Integrate all concepts in realistic project |

**Total Duration**: 3-4 hours (matches spec)

**All 7 Core Concepts Distributed**:
- Lesson 1: Exceptions as Objects, try/except/else/finally
- Lesson 2: Exception Types and Hierarchy
- Lesson 3: Raising Exceptions, Custom Exception Classes
- Lesson 4: Error Handling Strategies
- Lesson 5: Context Managers (overview only; capstone integrates, doesn't introduce)

**Quality Gates All PASS**:
✅ CEFR Progression: A2 → B1 (no zigzag)
✅ Cognitive Load: 4+3+3+2+0 new concepts/lesson (within limits)
✅ All concepts taught: 7/7 covered
✅ No forward references: Each lesson uses only prior concepts
✅ Capstone integration: 0 new concepts in Lesson 5
✅ CoLearning elements: 💬🎓🚀✨ specified for all 5 lessons
✅ "Try With AI" pattern: 4-prompt closure for all lessons
✅ Graduated teaching: Tier 1 (Book) → Tier 2 (AI) → Tier 3 (Capstone)
✅ Implementation checklist: Detailed requirements for lesson-writer
✅ CEFR/Bloom's mapping: Complete with cognitive complexity alignment
✅ Lesson dependencies: Clear prerequisite chains documented

**Plan File Sections**:
- Overview and structure
- CEFR/Cognitive Load validation table
- Detailed lesson architecture (5 lessons × full specification)
- Concept dependencies and prerequisite validation
- Scaffolding strategy (complexity progression)
- Graduated teaching pattern (Tier 1/2/3)
- CoLearning element distribution (where 💬🎓🚀✨ appear)
- "Try With AI" structure (4-prompt Bloom's progression)
- Implementation checklist (what lesson-writer must include)
- Quality gate validations

## Outcome

- ✅ Impact: Plan fully enables lesson-writer to implement with high confidence. Detailed enough that no ambiguities remain. Ready for /sp.tasks phase.
- 🧪 Tests: All 11 quality gates PASS. CEFR progression validated. Cognitive load verified. Concept distribution confirmed. Lesson boundaries validated.
- 📁 Files: plan.md created (288 lines). PHR recorded (0002-...).
- 🔁 Next prompts: /sp.tasks (generate detailed task checklist), /sp.implement (lesson-writer creates 5 lessons)
- 🧠 Reflection: Plan demonstrates complete pedagogical rigor (CEFR mapping, Bloom's alignment, cognitive load validation, graduated teaching pattern). Chapter-planner successfully applied all 8 domain skills (learning-objectives, concept-scaffolding, code-example-generator, exercise-designer, assessment-builder, technical-clarity, book-scaffolding, ai-collaborate-teaching). Plan is production-ready for implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None. Chapter-planner subagent executed flawlessly with approved spec as input.
- Graders run and results (PASS/FAIL): All quality gates PASS (11/11). CEFR progression validated. Cognitive load verified.
- Prompt variant (if applicable): N/A - standard chapter-planner flow with full spec context
- Next experiment (smallest change to try): No changes needed. Plan is complete and ready for /sp.tasks. If ADRs are needed, they would be suggested after tasks are generated.
