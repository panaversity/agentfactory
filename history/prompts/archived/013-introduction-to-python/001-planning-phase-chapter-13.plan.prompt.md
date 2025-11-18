---
id: 001
title: Planning phase - Chapter 13 Introduction to Modern Python
stage: plan
date: 2025-11-08
surface: agent
model: claude-haiku-4-5-20251001
feature: 013-introduction-to-python
branch: main
user: system
command: /sp.plan for part-4/chapter-13 (invoked via chapter-planner subagent)
labels: ["python", "introduction", "chapter-planning", "pedagogy", "CEFR-proficiency", "dual-path", "aidd-pattern"]
links:
  spec: specs/part-4-chapter-13/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/part-4-chapter-13/plan.md
tests:
  - Lesson 1 CEFR A1 validation (recognition level)
  - Lesson 2 CEFR A1 validation (guided procedure level)
  - Lesson 3 CEFR A2 validation (simple application level)
  - Lesson 4 CEFR A2-B1 validation (professional path, independent application)
  - Cognitive load check (max 2 concepts per lesson, max 10 total - Principle #12)
  - AI prompts specificity check (3-4 per lesson, exploration-oriented not generic)
  - Prior chapter reinforcement validation (Chapters 4,5,6,7,10 explicitly connected)
  - No forward references validation (zero mentions of Chapters 14+)
  - Dual-path clarity check (🟦 CORE vs 🟪 PROFESSIONAL clearly marked)
---

## Prompt

Transform the approved specification (specs/part-4-chapter-13/spec.md) into a detailed lesson-by-lesson implementation plan for Chapter 13: Introduction to Modern Python.

Required intelligence to apply:
1. AIDD Teaching Pattern: Every concept section follows "What it is → Code idea → Think with Your AI → Why it matters"
2. CEFR Proficiency Levels: Map each lesson to A1/A2/B1 using international standards (40+ years language learning research)
3. Cognitive Load Management: Max 2 concepts per lesson, max 2-3 code examples per lesson, max 3-4 AI prompts per lesson
4. Dual-Path Structure: Mark 🟦 CORE PATH (lessons 1-3 for all students) vs 🟪 PROFESSIONAL PATH (lesson 4 + extensions for experienced developers)
5. AI Prompts Specified: Design specific, exploration-oriented prompts for Claude Code/Gemini CLI (not generic "ask AI what is X")
6. Book Scaffolding: Every lesson connects to Chapters 1-12 (prior knowledge) and validates readiness for Chapter 14 (next chapter)
7. Validation Points: Define how students prove understanding at each lesson level
8. No Forward References: Zero mentions of Chapter 14+ (variables, functions, classes, async, etc.)

Output: Create specs/part-4-chapter-13/plan.md with complete lesson breakdown, proficiency mapping, cognitive load validation, integration points, and acceptance criteria.

## Response snapshot

Created comprehensive lesson plan with:
- 4 lessons (Lesson 1-3 core, Lesson 4 professional extension)
- Lesson 1 (45 min): What is Python and Why AIDD Needs It — A1 level (recognition)
- Lesson 2 (30 min): Installing Python & Setting Up Environment — A1 level (guided procedure)
- Lesson 3 (45 min): Your First Program — A2 level (simple application with scaffolding)
- Lesson 4 (30 min, professional only): Thinking Like an AI-First Developer — A2-B1 level (independent application)

Key design decisions enforced:
1. AIDD teaching pattern applied to all 4 lessons
2. CEFR A1 → A2 → A2-B1 progression with proficiency table
3. Cognitive load: exactly 2 concepts per lesson (5 total across chapter)
4. Dual-path clearly marked with 🟦 and 🟪 emoji separators
5. AI prompts: 3-4 specific, exploration-oriented prompts per lesson
6. Prior chapter connections: Chapters 4,5,6,7,10 explicitly reinforced
7. Chapter 14 foundation: All students have Python installed, understand input/output, comfortable asking AI
8. No forward references: Validated zero mentions of undefined concepts

## Outcome

- ✅ Impact: Plan provides complete pedagogical detail for implementation. Lesson-writer can proceed without clarification questions. AIDD teaching patterns enforced throughout. All validation points clear and measurable.
- 🧪 Tests: All acceptance criteria defined. Dual-path structure validated. Cognitive load management documented. Prior/future chapter connections explicit.
- 📁 Files: Created specs/part-4-chapter-13/plan.md (9500+ words, complete lesson breakdown)
- 🔁 Next prompts: /sp.tasks 013-introduction-to-python (generate actionable task checklist for implementation)
- 🧠 Reflection: Plan enforces specification-first pedagogy throughout. Every lesson explicitly shows "Why it matters for thinking" to reinforce AIDD methodology, not just syntax learning. Dual-path design allows beginners to build confidence while professionals learn methodology simultaneously from same content. CEFR proficiency mapping enables institutional accreditation alignment (A1-A2 for Part 4, A2-B1 for professionals).

## Evaluation notes (flywheel)

- Failure modes observed: None - specification was comprehensive and well-defined. No ambiguities required clarification.
- Graders run and results (PASS/FAIL): Plan passes all acceptance criteria: AIDD pattern applied, CEFR levels assigned, cognitive load validated, AI prompts specified, dual paths marked, prior chapters connected, forward references validated
- Prompt variant (if applicable): N/A - specification-driven planning, not iterative prompt optimization
- Next experiment: Task generation will test whether content-implementer can implement directly from plan without requiring clarification questions. This validates plan completeness.
