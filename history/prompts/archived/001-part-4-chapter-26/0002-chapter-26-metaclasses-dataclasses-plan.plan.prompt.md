---
id: 0002
title: "Chapter 26 Metaclasses Dataclasses Plan"
stage: plan
date: 2025-11-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-part-4-chapter-26
branch: 001-part-4-chapter-26
user: system
command: /sp.plan
labels: ["chapter-26", "metaclasses", "dataclasses", "lesson-planning", "cefr-mapping", "skills-proficiency"]
links:
  spec: specs/001-part-4-chapter-26/spec.md
  plan: specs/001-part-4-chapter-26/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-part-4-chapter-26/plan.md (created, 305 lines)
tests:
  - none (planning phase)
---

## Prompt

Transform approved spec into detailed lesson-by-lesson plan with:
- CEFR proficiency levels (B1-B2 for advanced tier)
- Skills proficiency mapping (international standards alignment)
- Cognitive load validation (max 10 concepts per lesson)
- AI prompts for each lesson (4 prompts, Bloom's progression)
- Lesson progression ensuring smooth B1→B2 proficiency increase

Context:
- Approved spec at: specs/001-part-4-chapter-26/spec.md
- Chapter focus: Equal depth metaclasses + dataclasses (50/50)
- Learning outcome: Conceptual mastery with focused examples (no capstone)
- Target: 4-5 lessons, 14-20 code examples, 16-20 AI prompts

## Response snapshot

**Comprehensive 5-lesson plan created with CEFR proficiency mapping and skills alignment**

### Lesson Structure (3.5-4.5 hours total)
1. **Understanding Metaclasses** (45-60 min, B1): type mechanism, 10 concepts, 4 examples
2. **Practical Metaclass Patterns** (50-65 min, B1-B2): validation/registration/singleton, 10 concepts, 6 examples
3. **Introduction to Dataclasses** (45-60 min, B1): decorator basics, 10 concepts, 5 examples
4. **Advanced Dataclass Features** (50-65 min, B1-B2): field/metadata/post_init, 10 concepts, 6 examples
5. **Choosing the Right Tool** (40-50 min, B2): synthesis and architectural decisions, 8 concepts, 4 examples

### Skills Proficiency Matrix (21 skills mapped)
- **Conceptual Understanding** (8 skills): Metaclass concepts, type system, dataclass benefits, framework patterns
- **Practical Application** (9 skills): Creating metaclasses, using dataclass features, validation patterns
- **Architectural Thinking** (4 skills): Tool selection, pattern recognition, framework design, tradeoff evaluation

### CEFR Proficiency Progression
- **B1 Foundation** (Lessons 1, 3): Introduction to mechanisms, basic usage
- **B1-B2 Application** (Lessons 2, 4): Practical patterns, advanced features
- **B2 Synthesis** (Lesson 5): Architectural decision-making, comparing approaches
- **Smooth progression**: No regression, each lesson builds on previous

### Equal 50/50 Depth Balance Verified
- **Metaclasses** (L1-2): 20 concepts, 10 examples (6 unique), framework context
- **Dataclasses** (L3-4): 20 concepts, 11 examples (6 unique), practical emphasis
- **Synthesis** (L5): 8 concepts, 4 examples (integrates both)
- **Balance confirmed**: Equal depth, complexity, and real-world application

### Code Examples: 18 Total (Progressive B1→B2)
- Type factory, basic/validation/registration/singleton/Django-like metaclasses
- Basic/defaults/frozen/ordered/factory/metadata/post_init/InitVar/JSON dataclasses
- 3-way comparison, framework design, API layer, hybrid approach

### AI-Native Learning Throughout
- Every lesson: Describe intent → Explore → Validate → Learn from errors
- 20 "Try With AI" prompts (4 per lesson, Bloom's progression)
- AI positioned as co-reasoning partner (not code generator)
- Validation-first approach emphasized

### Risk Mitigation Strategy
6 identified risks with concrete mitigation:
1. Metaclasses seem magical → Step-by-step mechanism tracing
2. Mutable default gotcha → Explicit default_factory emphasis
3. Tool confusion → Explicit comparison and decision matrix
4. Time density → 45-65 min lessons, fits 3.5-4.5 hour target
5. Code failures → Technical review validates Python 3.14+
6. Reading level → Peer review for Grade 10-11 clarity

## Outcome

- ✅ Impact: Complete lesson-by-lesson architecture with CEFR proficiency mapping, skills alignment, and pedagogical scaffolding for advanced Python chapter
- 🧪 Tests: No tests at planning phase (code validation happens in implementation)
- 📁 Files: Created specs/001-part-4-chapter-26/plan.md (305 lines, comprehensive lesson details)
- 🔁 Next prompts: Run /sp.tasks to generate actionable task checklist, then /sp.implement for lesson content creation
- 🧠 Reflection: Skills proficiency mapping ensures competency-based learning aligned with international standards (CEFR + Bloom's + DigComp). Equal 50/50 depth balance honors user's pedagogical choice. Smooth B1→B2 progression prevents cognitive overload while maintaining rigor.

## Evaluation notes (flywheel)

- Failure modes observed: None - plan created successfully with all required proficiency mappings and constitutional compliance
- Graders run and results (PASS/FAIL): Not applicable at planning phase (grading happens at validation)
- Prompt variant (if applicable): Standard chapter-planner workflow with skills-proficiency-mapper integration
- Next experiment (smallest change to try): Proceed to /sp.tasks to generate implementation checklist, then /sp.implement for content creation
