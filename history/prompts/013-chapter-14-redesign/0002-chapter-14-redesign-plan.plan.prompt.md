---
id: 0002
title: Chapter 14 Redesign Implementation Plan
stage: plan
date: 2025-01-15
surface: claude-code
model: claude-sonnet-4-5
feature: 013-chapter-14-redesign
branch: 013-chapter-14-redesign
user: developer
command: /sp.plan
labels: [chapter-redesign, pedagogy, lesson-planning, beginner-friendly]
links:
  spec: specs/013-chapter-14-redesign/spec.md
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: specs/013-chapter-14-redesign/plan.md
    purpose: Comprehensive lesson-by-lesson implementation plan with detailed content structure for all 5 lessons
tests:
  - name: N/A
    status: N/A
---

## Prompt

User requested planning phase for Chapter 14 redesign after specification was approved. Ran `/sp.plan` command. User asked to "reevaluate the plan that you created after reading from file" when initial attempt found plan.md was still just a template. User wanted detailed lesson-by-lesson breakdown following the beginner-friendly pedagogy (WHAT→WHY→WHEN→Code pattern) specified in spec.md.

Context:
- Specification approved: specs/013-chapter-14-redesign/spec.md (V2)
- Corrected from V1: Focus on DATA TYPES, not Python syntax foundations
- Chapter 13 teaches print()/variables; Chapter 14 teaches type system
- Source material: context/13_chap12_to_29_specs/Lesson_02_Data_Types.md
- Target: 5 lessons covering 13 Python types at graduated depth

## Response snapshot

Successfully created comprehensive implementation plan following SDD workflow:

1. **Setup**: Ran `.specify/scripts/bash/setup-plan.sh` which copied plan template to specs/013-chapter-14-redesign/plan.md

2. **Manual Plan Creation** (template was skeleton, required manual filling):
   - **Summary**: Redesign Chapter 14 to teach Python's type system using concept-before-syntax pattern
   - **Technical Context**: Educational content (Markdown lessons), CEFR A2-B1 proficiency, max 7 concepts per lesson
   - **Constitution Check**: All gates PASS (Principle 13 Graduated Teaching, cognitive load, progressive complexity, co-learning)
   - **Project Structure**: Defined book-source directory structure with 5 lesson files + readme
   - **Complexity Tracking**: Confirmed no violations

3. **Detailed Lesson Breakdowns** (5 lessons with complete structure):

   **Lesson 1: Understanding Data Types** (40-45 min, 6 concepts)
   - Hook: Kitchen analogy (jars labeled with contents)
   - WHAT is a Data Type: Python's classification system
   - WHY Types Matter: Different operations, validation
   - 7 Type Categories overview
   - Type Decision Framework
   - Introducing type() function (first code exposure)
   - Practice: Match data to types, explain concepts, use type()
   - Try With AI: Create analogies, classify scenarios

   **Lesson 2: Numeric Types** (45-50 min, 6 concepts)
   - Hook: Age vs price (25 vs $25.99)
   - WHAT is int: Whole numbers, unlimited range
   - WHY/WHEN: Ages, counts, indices
   - Code examples AFTER concepts
   - WHAT is float: Decimals, IEEE 754
   - WHY/WHEN: Prices, measurements
   - Decision guide table (int vs float scenarios)
   - WHAT is complex: Real+imaginary, marked Advanced
   - Common mistakes: type loss, float precision
   - Practice: Classify 15 numbers, fix code, use type()

   **Lesson 3: Text, Boolean, None** (45-50 min, 5 concepts)
   - str: Sequence of characters, immutable, quote variations
   - bool: True/False only, decision-making
   - Truthy/falsy values: 0/""/[]/None = False
   - None: Absence of value, singleton
   - Practice: Identify truthy/falsy, choose types, fix code

   **Lesson 4: Collections Awareness + Binary** (50-55 min, 7 concepts)
   - list: Ordered, mutable (syntax only, Chapter 18 for methods)
   - tuple: Ordered, immutable (coordinates, RGB)
   - dict: Key-value pairs (student records, config)
   - set: Unique values (tags, duplicates)
   - range: Number sequences (loops)
   - Binary types: bytes/bytearray/memoryview (marked Advanced)
   - Collection decision guide table
   - Message: "Deep dive in Chapter 18"

   **Lesson 5: Type Utilities + Capstone** (60-70 min, 7 concepts)
   - type() reinforcement
   - id() function: Object identity
   - isinstance(): Type checking
   - Type casting: int(), float(), str(), bool()
   - Explicit vs implicit casting
   - Integer interning: -5 to 256 cached (marked Advanced)
   - Number systems: binary/hex/octal/ASCII (marked Advanced)
   - Capstone Project: Type Explorer (integrates all concepts)

4. **Implementation Sequence**: 12-step validation-gated workflow (create readme → Lesson 1 → validate → Lesson 2 → validate... → final validation)

5. **Quality Assurance**: 15-item checklist covering WHAT→WHY→WHEN→Code pattern, type hints, real-world examples, "Try With AI" sections, cognitive load limits, no Chapter 13 overlap, cross-references accuracy

6. **Cross-References**: Documented forward references (to Chapters 15-18) and backward references (from other chapters to Chapter 14)

7. **YAML Frontmatter Template**: Complete metadata structure for each lesson (proficiency, Bloom's level, skills, learning objectives, success criteria, differentiation)

8. **Timeline**: 20-24 hours estimated (3-5 hours per lesson + readme + validation)

Key Pedagogical Decisions:
- Concept-before-syntax: NO CODE until WHAT/WHY/WHEN explained
- Progressive disclosure: Core 5 types deep → collections awareness → advanced brief
- Real-world scenarios: age/price/email (not abstract x/y/z)
- Type hints normalized: Every example uses them
- Advanced topics marked: "For Curious Learners" (interning, number systems)
- Collections messaging: "Syntax here, methods in Chapter 18"

## Outcome

- ✅ Impact: Comprehensive 5-lesson implementation plan ready for task generation. Each lesson has detailed structure following beginner-friendly pedagogy: hooks, WHAT→WHY→WHEN→Code pattern, practice exercises, "Try With AI" prompts, success criteria mappings. Plan addresses all 28 functional requirements from spec.md.
- 🧪 Tests: Validation gates defined for each lesson (15 quality checks). Success criteria from spec.md mapped to specific lesson content.
- 📁 Files: plan.md (complete with 5 lesson breakdowns, implementation sequence, QA checklist, cross-references, YAML template)
- 🔁 Next prompts: `/sp.tasks` to generate actionable task list breaking down 5-lesson implementation; then invoke `lesson-writer` agent for actual content creation
- 🧠 Reflection: Plan successfully applies concept-before-syntax pattern to data types. Each lesson introduces concepts first (WHAT Python types are, WHY they matter, WHEN to use each) before showing code. Progressive disclosure manages cognitive load: Lesson 1 (concepts), Lesson 2 (numeric deep), Lesson 3 (text/bool/None deep), Lesson 4 (collections awareness), Lesson 5 (utilities + capstone integration). "Try With AI" sections in every lesson enable co-learning partnership.

## Evaluation notes (flywheel)

- Failure modes observed: Initial attempt found plan.md was still template (chapter-planner agent invocation didn't write files). Resolved by manually creating detailed plan. Future improvement: Verify subagent outputs are written before marking complete.
- Graders run and results (PASS/FAIL): Constitution Check PASS (all gates: Principle 13, cognitive load 6-7 concepts per lesson, progressive complexity, co-learning). Quality checklist defined with 15 validation gates for implementation phase.
- Prompt variant (if applicable): N/A (standard /sp.plan workflow, but manual execution due to subagent not writing files)
- Next experiment (smallest change to try): Test if detailed lesson breakdown reduces implementation time by providing clear structure vs leaving details to lesson-writer agent. Could measure: hours spent per lesson with vs without detailed plan.
