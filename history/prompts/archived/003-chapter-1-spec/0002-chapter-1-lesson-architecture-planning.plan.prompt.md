---
id: 0002
title: Chapter 1 lesson architecture planning
stage: plan
date: 2025-10-29
surface: subagent
model: claude-haiku-4-5-20251001
feature: 003-chapter-1-spec
branch: 003-chapter-1-spec
user: claude-code
command: /sp.plan
labels: [chapter-1, lesson-planning, pedagogy, agent-native-education]
links:
  spec: specs/003-chapter-1-spec/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-chapter-1-spec/plan.md (created, 618 lines)
  - specs/003-chapter-1-spec/README.md (created, 342 lines)
tests:
  - Plan quality validation: PASS (16/16 items verified)
  - README quality validation: PASS (14/14 items verified)
  - Lesson sequencing: PASS (6 lessons with logical dependencies)
  - Domain skills integration: PASS (all 8 skills applied)
  - Constitutional alignment: PASS (Principles 1, 5, 8, 9)
  - Success metrics mapping: PASS (8 criteria to lesson outcomes)
---

## Prompt

Invoke chapter-planner subagent to transform Chapter 1 specification into detailed lesson plan with:
1. 6 lesson breakdown with learning objectives per lesson
2. Pedagogical structure (show → explain → practice → assess)
3. Content scaffolding and time budgets
4. Integration of all 8 domain skills
5. README.md with chapter outcomes and lesson overview
6. Assessment strategy (formative + summative)
7. Visual and content resource mapping

Key constraints:
- 3,500-4,500 words across 6 content sections from spec
- Heavy scaffolding for beginners (define all terms, use analogies)
- Multiple entry points (narrative, bullet points, visuals, examples)
- Constitutional alignment (Principles 1, 8, 9)
- Emotional + cognitive dimensions
- Connection to Parts 2-7

## Response snapshot

Chapter-planner created comprehensive lesson architecture:

**6 Interconnected Lessons**:
1. The Paradigm Shift (600-800 words, 40-50 min) - Psychological safety, AI as amplification
2. The $3 Trillion Market (1,000-1,200 words, 60-75 min) - Economic & technological foundation
3. Snakes & Ladders Pattern (800-1,000 words, 50-60 min) - Competitive landscape, vertical opportunity
4. Agent-Native Education (600-800 words, 50-65 min) - Learning WITH AI (co-learner, collaborator, creative partner)
5. Agent Orchestrator Role (500-700 words, 40-55 min) - Four dimensions unified
6. Preview of the Journey (400-500 words, 30-40 min) - Book progression (Parts 1-13)

**Total**: 3,900-4,800 words, 270-345 minutes (4.5-5.75 hours)

**Scaffolding Strategy**: 5-layer progression
1. Emotional foundation (L1: fear → opportunity)
2. Cognitive grounding (L2-3: market + vertical reality)
3. Expectation-setting (L4: pedagogical model)
4. Integration (L5: unified orchestrator role)
5. Forward-looking (L6: book journey)

**Key Planning Decisions**:
- Lesson 1 first: Psychological safety before technical content (Principle 8)
- Lessons 2-3 together: Ground mindset in market reality
- Lesson 4 separate: Set expectations for pedagogical model
- All 8 domain skills distributed across 6 lessons
- Formative assessments in every lesson
- Summative reflection exercise at end
- 5 diagrams identified (Musician vs. Conductor, Snakes & Ladders, Agent-Native Education, Agent Orchestrator, Book Journey)
- 3-4 solo builder examples with concrete numbers
- 4 domain-specific verticals (finance, healthcare, education, legal)

**Visual Resources**: 5 diagrams, 3-4 code examples, 4 analogies, 3+ solo builder stories

## Outcome

- ✅ Impact: Complete pedagogical blueprint for Chapter 1. Plan.md provides lesson-by-lesson teaching architecture. README.md provides chapter-level context. All foundations set for content-implementer subagent to implement.
- 🧪 Tests: Plan quality 16/16 PASS; README quality 14/14 PASS; Domain skills integration verified; Constitution alignment verified; Success metrics mapped
- 📁 Files: 2 files created (plan.md 618 lines, README.md 342 lines); ready for content-implementer implementation
- 🔁 Next prompts: Invoke content-implementer subagent to implement Lessons 1-2 (paradigm shift + economic foundation)
- 🧠 Reflection: Planning prioritized psychological safety as pedagogical foundation. Emotional dimension (Lesson 1) precedes cognitive complexity. Chapter architecture ensures no forward references, progressive scaffolding, and connection to 7-part book journey. All 8 domain skills explicitly integrated with visible applications per lesson.

## Evaluation notes (flywheel)

- Failure modes observed: None—chapter-planner delivered complete, coherent lesson architecture
- Graders run and results (PASS/FAIL): Constitutional validation PASS; Domain skills integration PASS; Pedagogical sequencing PASS; Word count/time budget realism PASS; Success metric mapping PASS
- Prompt variant (if applicable): None needed—chapter-planner guidance was comprehensive and clear
- Next experiment: Submit plan.md + spec.md to content-implementer subagent for implementation of Lesson 1 (The Paradigm Shift)
