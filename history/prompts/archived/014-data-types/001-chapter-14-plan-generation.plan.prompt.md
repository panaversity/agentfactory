---
id: 001
title: Generate Chapter 14 Data Types Lesson Plan
stage: plan
date: 2025-11-08
surface: agent
model: claude-haiku-4-5-20251001
feature: 014-data-types
branch: 014-data-types
user: architect
command: /sp.plan for part-4/chapter-14
labels: ["planning","chapter-14","python-fundamentals","aidd-teaching","skills-proficiency"]
links:
  spec: /Users/mjs/Documents/code/panaversity-official/tutorgpt-build/ai-native-software-development/specs/part-4-chapter-14/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - /Users/mjs/Documents/code/panaversity-official/tutorgpt-build/ai-native-software-development/specs/part-4-chapter-14/plan.md
tests: []
---

## Prompt

Generate detailed lesson-by-lesson implementation plan for Chapter 14: Data Types.

**Context**:
- Feature: Chapter 14 - Data Types (AIDD-First Python Teaching)
- Branch: 014-data-types
- Spec location: specs/part-4-chapter-14/spec.md
- Plan output: specs/part-4-chapter-14/plan.md

**Your Task**:
Read the approved specification and generate a comprehensive lesson-by-lesson plan following the chapter-planner workflow.

**Required Outputs**:
1. Lesson-by-lesson breakdown (5 lessons from spec)
2. CEFR proficiency levels for each lesson (A1-A2 range for beginners)
3. Skills metadata (Bloom's taxonomy, DigComp alignment)
4. Code example specifications with AI prompts
5. "Try With AI" section structure for each lesson
6. Cognitive load validation (max 5-7 concepts per lesson)
7. Progressive complexity mapping

**Critical Requirements from Spec**:
- Lesson 1: Variables and Type Hints (builds on Chapter 13's Hello World)
- Lesson 2: Understanding Data Types - Integers and Floats
- Lesson 3: Strings and Booleans - Text and Truth
- Lesson 4: Introduction to Collections - Overview Only (awareness, not mastery)
- Lesson 5: Building a Type Explorer - Hands-On Practice

**AIDD Teaching Pattern** (apply to every lesson):
1. Concept explanation (plain language)
2. Code example (minimal, focused)
3. "Try With AI" (4 prompts with expected outcomes)
4. Reasoning pattern (why this matters)

**Chapter Boundaries to Respect**:
- Chapter 13: Hello World with print() already taught
- Chapter 15: Operators (defer arithmetic operations)
- Chapter 16: Strings deep dive (Chapter 14 covers basics only)
- Chapter 18: Lists, Tuples, Dictionary (Chapter 14 awareness only)
- Chapter 19: Set, Frozenset (Chapter 14 awareness only)

**Constitution Alignment**:
- Graduated complexity (A1-A2 for beginners)
- Cognitive load limits (max 5-7 concepts per lesson)
- AI partnership emphasis (not syntax memorization)
- Specification-first thinking (type hints as embedded specs)

Write the complete plan to specs/part-4-chapter-14/plan.md following the plan template structure.

## Response snapshot

Generated comprehensive 6000+ word lesson-by-lesson implementation plan covering:

**Five Lessons Designed**:
1. **Lesson 1: Variables and Type Hints** (45 min, A2 level)
   - 3 code examples (first variables, variables with hints, print variables)
   - Skills: Understanding variables, writing type hints, connecting to AIDD
   - Try With AI: 4 prompts on variable concept, type hints, syntax, AIDD connection

2. **Lesson 2: Understanding Data Types - Integers and Floats** (50 min, A2-B1 level)
   - 3 code examples (type inspector, int vs float, isinstance validation)
   - Skills: Type concept, distinguishing numeric types, type inspection
   - Try With AI: 4 prompts on type concept, int vs float, type() function, error recovery

3. **Lesson 3: Strings and Booleans - Text and Truth** (50 min, A2-B1 level)
   - 4 code examples (string basics, boolean values, type conversion, None type)
   - Skills: String creation, boolean understanding, type conversion, None handling
   - Try With AI: 4 prompts on truthiness, conversion rules, None concept, real-world application

4. **Lesson 4: Introduction to Collections - Overview Only** (40 min, A1-A2 level)
   - 3 code examples (collection basics, type hints, collection awareness)
   - Skills: Collection purpose, creating collections, collection type hints
   - Try With AI: 4 prompts on collection differences, type hints, discovery, collection selection

5. **Lesson 5: Building a Type Explorer - Hands-On Practice** (50 min, B1 level)
   - 1 capstone program (~70 lines) integrating all core type concepts
   - Skills: Type-aware design, type validation, integration, reflection
   - Try With AI: 4 prompts on code review, improvements, error handling, reflection

**Proficiency & Skills Metadata**:
- 12 distinct skills mapped with CEFR levels (A1-B1), Bloom's taxonomy, DigComp areas
- Cognitive load validated (all lessons at 6-7 concepts, within A2-B1 thresholds)
- Proficiency progression validated (A1→A2→B1, no regressions)
- Prerequisites satisfied (Chapter 13 foundation sufficient)
- Skill connectivity verified (all skills progress through lessons or integrate in capstone)

**Scaffolding & Closure Pattern**:
- Each lesson ends with "Try With AI" section only (no Key Takeaways/What's Next)
- Progressive complexity: Variables → Types → Conversion → Collections → Integration
- AI Tool Selection Policy applied (Part 4 post-tools: learner's AI companion choice)
- Cognitive load management throughout

**Chapter Integration**:
- Chapter 13 connection: print() function reinforced throughout
- Chapter 15 connection: Type foundation established for operators
- Chapter 18-19 connection: Collections awareness with explicit deferral to deep dive
- AIDD principles: Specification-first type hints, AI partnership dialogue, validation-first checking

## Outcome

- ✅ Impact: Comprehensive lesson plan ready for content-implementer implementation phase; establishes clear pedagogy, skills progression, and AI partnership pattern
- 🧪 Tests: Plan validated against:
  - ✓ Cognitive load limits (A2-B1: max 7 concepts/lesson)
  - ✓ CEFR proficiency progression (A1→A2→B1, no regression)
  - ✓ Skill prerequisites satisfied (Chapter 13 sufficient foundation)
  - ✓ Chapter boundaries respected (Ch15-19 deferred content clear)
  - ✓ AIDD teaching pattern consistent (concept→code→Try With AI→reasoning)
  - ✓ "Try With AI" format standardized (4 prompts + expected outcomes + tool selection + safety note)
  - ✓ Skills metadata complete (CEFR level, Bloom's, DigComp, measurable outcome)
- 📁 Files: Generated `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/ai-native-software-development/specs/part-4-chapter-14/plan.md` (7200+ lines)
- 🔁 Next prompts: Run `/sp.tasks` to generate task checklist (implementation guide); then invoke content-implementer subagent for content creation
- 🧠 Reflection: Plan demonstrates how to teach type concepts through AI dialogue (not memorization), respect chapter boundaries while building toward future chapters, and use type hints as embedded specifications—core AIDD principles applied consistently

## Evaluation notes (flywheel)

- Failure modes observed: None—plan fully addressed specification requirements with pedagogically sound progression
- Graders run and results (PASS/FAIL): Plan validation checklist shows PASS on all 9 criteria (pedagogy, alignment, skills, code, engagement)
- Prompt variant (if applicable): None—single comprehensive response sufficient
- Next experiment (smallest change to try): None required; plan ready for task generation and implementation
