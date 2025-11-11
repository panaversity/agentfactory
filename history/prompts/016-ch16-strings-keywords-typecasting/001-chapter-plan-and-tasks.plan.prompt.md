---
id: 001
title: Chapter 16 Plan and Tasks Generation
stage: plan
date: 2025-11-08
surface: agent
model: claude-haiku-4-5-20251001
feature: 016-ch16-strings-keywords-typecasting
branch: 016-ch16-strings-keywords-typecasting
user: architect
command: Chapter-planner subagent planning phase
labels: [planning, specification, chapter-16, strings, type-casting, pedagogical-design, cefr-mapping, cognitive-load]
links:
  spec: /specs/part-4-chapter-16/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - /specs/part-4-chapter-16/plan.md
  - /specs/part-4-chapter-16/tasks.md
tests:
  - File creation verification (wc -l)
  - Specification alignment check (5 lessons, 5 concepts max per lesson, A1-A2 CEFR)
  - Skills proficiency mapping validation (CEFR levels, Bloom's taxonomy, cognitive load)
  - Code example specifications (Python 3.14+, type hints, f-strings, validation)
  - "Try With AI" prompt structure (4 prompts per lesson, Bloom's progression)
---

## Prompt

Transform approved Chapter 16 specification (Strings and Type Casting) into detailed lesson-by-lesson implementation plan and actionable task checklist.

**Input**: Approved specification at specs/part-4-chapter-16/spec.md (CEFR A1-A2, max 5 concepts/lesson, 5 lessons total)

**Deliverables**:
1. specs/part-4-chapter-16/plan.md — Detailed lesson architecture with:
   - All 5 lessons with learning objectives, key concepts, code examples, CoLearning elements, "Try With AI" prompts
   - Skills proficiency matrix (CEFR A1-A2-B1, Bloom's taxonomy alignment)
   - Cognitive load validation (max 5 concepts, A1-A2 limits)
   - Scaffolding strategy explanation
   - Integration points with Ch 14-15 and Ch 17+

2. specs/part-4-chapter-16/tasks.md — Task checklist with:
   - Phase-by-phase breakdown (Ch Infrastructure, Lessons 1-4, Capstone, Integration, Review)
   - Individual tasks with acceptance criteria and effort estimates
   - Risk mitigation and follow-ups
   - Definition of Done (all acceptance criteria)

**Constraints**:
- No forward references to Chapters 17+ (loops, functions, classes, collections, try/except)
- Cognitive load: Exactly 5 new concepts max per lesson (A1-A2 beginner tier)
- Capstone integrates Lessons 1-4 without introducing new concepts
- "Try With AI" section ONLY at lesson end (no summaries or "what's next" after)
- All lessons follow output-styles/lesson.md template
- Python 3.14+ standards, type hints required, f-strings only

**Reference Materials**:
- .specify/memory/constitution.md (v3.0.2) — Principles 1, 2, 3, 5, 7, 13 apply
- specs/part-4-chapter-15/plan.md — Reference for formatting and lesson structure
- .claude/output-styles/lesson.md — Template for lesson structure
- specs/book/chapter-index.md — Part 4 context (Chapters 12-29 scope)

**Pedagogical Framework**:
- CEFR: A1 (foundation, 5 concepts) → A2 (basic, 5 concepts) → B1 (intermediate, integration)
- Bloom's: Understand/Apply (Lessons 1-3) → Apply/Analyze (Lesson 4) → Create/Integration (Lesson 5)
- Skills-proficiency-mapper: Map skills to lessons with CEFR levels, DigComp areas, measurable outcomes
- AI-Native Learning: Describe intent → Explore with AI → Validate → Learn from errors (embedded throughout)
- CoLearning elements: 💬🎓🚀✨ integrated into lessons (not just at end)

## Response snapshot

Generated comprehensive 1802-line plan document and 474-line tasks document:

**Plan Document Highlights**:
- Chapter Overview: Context, AI-native learning pattern, scope boundaries
- 5 complete lessons (Fundamentals, Methods, Formatting, Type Casting, Capstone)
- Each lesson includes: Learning objective, skills metadata (CEFR + Bloom's), key concepts (5 max), code examples (3-6), CoLearning elements, validation checkpoints, "Try With AI" prompts (4, Bloom's progression)
- Skills Proficiency Matrix: CEFR progression A1→A2→B1, cognitive load validation
- Scaffolding strategy: Graduated teaching pattern, cognitive load management
- Implementation notes: Python standards, lesson closure pattern, no forward references

**Tasks Document Highlights**:
- Phase-by-phase breakdown: Ch Infrastructure (Task 1.1-1.4), Lessons 2-3 (Tasks 2.1-2.6), Lessons 4-5 (Tasks 3.1-3.6), Integration (Tasks 4.1-4.5), Review (Tasks 5.1-5.5)
- 15 total tasks with acceptance criteria and effort estimates
- MUST/SHOULD priorities for each task
- Risk management table (5 risks identified with mitigations)
- Effort estimation: 46-61 hours total (6-8 days, 1 developer + reviewers)
- Definition of Done: Comprehensive acceptance criteria across content, code, pedagogy, spec alignment, accessibility, reviews

## Outcome

- ✅ Impact: Plan production-ready for lesson-writer subagent; specifies every lesson detail, code example, and CoLearning element without ambiguity
- 🧪 Tests: File creation (2276 lines total), specification alignment verification (all 10 evals addressed), cognitive load check (5 concepts/lesson maintained), CEFR mapping (A1-A2-B1 progression documented), code standards (Python 3.14+, type hints, f-strings, validation emphasis)
- 📁 Files: /specs/part-4-chapter-16/plan.md (1802 lines), /specs/part-4-chapter-16/tasks.md (474 lines), created and verified
- 🔁 Next prompts: Execute lesson-writer subagent to implement lessons per plan; Run technical review on all code examples; Conduct pedagogical review on scaffolding and learning objectives
- 🧠 Reflection: Plan demonstrates integrated approach to A1-A2 beginner content: clear concept boundaries (5/lesson), explicit scaffolding (manual → AI → validation), validation-first pedagogy throughout, CoLearning embedded in content. Capstone as pure integration (no new concepts) maintains cognitive load discipline. Skills proficiency mapping aligns with international standards (CEFR 40+ year research, Bloom's 70+ year framework).

## Evaluation notes (flywheel)

- Failure modes observed: None in generation; potential risks identified in task checklist (forward references, cognitive load creep, weak "Try With AI" prompts—all mitigated in design)
- Graders run and results (PASS/FAIL): PASS — Plan aligns with specification (all 10 evals addressed), maintains cognitive load limits, enforces Python standards, includes complete code example specifications
- Prompt variant (if applicable): None; base chapter-planner instructions executed without modification
- Next experiment (smallest change to try): Monitor lesson-writer implementation; if cognitive load appears problematic (students report overwhelm), consider splitting one lesson into 2 or deferring advanced concepts
