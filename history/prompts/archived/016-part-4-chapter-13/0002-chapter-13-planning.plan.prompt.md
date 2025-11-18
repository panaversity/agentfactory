---
id: 0002
title: chapter-13-planning
stage: plan
date: 2025-11-08
surface: agent
model: claude-sonnet-4-5-20250929
feature: 016-part-4-chapter-13
branch: 016-part-4-chapter-13
user: system
command: /sp.plan
labels: ["chapter-13", "python", "planning", "cefr-proficiency", "colearning-pedagogy"]
links:
  spec: specs/016-part-4-chapter-13/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/016-part-4-chapter-13/plan.md
  - specs/016-part-4-chapter-13/tasks.md
tests:
  - CEFR Progression Validation: A1→A1-A2→A2→A2→B1 PASS
  - Cognitive Load Validation: All lessons ≤5 concepts PASS
  - "Try With AI" Closure: 4 prompts per lesson, no summaries PASS
  - No Forward References: Zero OOP/control flow/functions/collections PASS
  - CoLearning Elements: 20+ elements (💬🎓🚀✨) embedded PASS
---

## Prompt

User approved PHASE 1 specification and requested PHASE 2: Planning + ADR Gate.

**Command**: `/sp.plan 016-part-4-chapter-13`

**Context Provided**:
- Approved specification with 11 success evals, 24 functional requirements
- Learning objectives (5 measurable)
- Content structure (5 lessons: 4 foundational + 1 capstone)
- CEFR requirements (A1-A2 beginner tier, max 5 concepts/lesson)
- CoLearning pedagogy requirements (💬🎓🚀✨ throughout)
- "Try With AI" closure pattern (4 prompts, no summaries)
- No forward references constraint (critical for Chapter 13)

**Skills Required**:
- skills-proficiency-mapper (CEFR A1→A2→B1 progression)
- learning-objectives (Bloom's taxonomy alignment)
- concept-scaffolding (cognitive load validation)
- ai-collaborate-teaching (CoLearning element placement)
- book-scaffolding (chapter structure, lesson flow)

**Subagent Invoked**: chapter-planner

## Response snapshot

Successfully created comprehensive lesson plan with detailed task breakdown:

**Plan.md** (1,194 lines):
- 5 lessons with complete architecture (Lesson 1: What Is Python?, Lesson 2: Installing Python, Lesson 3: Variables & Type Hints, Lesson 4: Basic Syntax, Lesson 5: Capstone)
- CEFR proficiency mapping (A1 → A1-A2 → A2 → A2 → B1 validated progression)
- Skills proficiency metadata (6 skills mapped with CEFR + Bloom's + DigComp levels)
- Cognitive load validation (all lessons ≤ 5 concepts)
- CoLearning elements placement (20+ strategic placements of 💬🎓🚀✨)
- "Try With AI" prompts (20 total: 4 per lesson, Bloom's progression Remember→Understand→Apply→Analyze)
- Code example specifications (7+ examples, all with type hints)
- Common mistakes addressed (15+ proactive coverage)
- Validation strategy (how to test each lesson's success)
- Risk analysis (potential challenges with mitigation)

**Tasks.md** (1,654 lines):
- 35 specific, atomic, testable tasks
- 8 implementation phases (Chapter Structure → Lessons 1-5 → Integration → Review)
- Task prioritization (20 MUST-HAVE, 12 SHOULD-HAVE, 3 NICE-TO-HAVE)
- Effort estimation (40-50 total hours)
- Clear acceptance criteria for each task
- Task dependencies mapped
- Quality assurance checkpoints embedded

**Critical Planning Decisions**:
1. **Type Hints as CORE** (not optional): Positioned as "describing intent" throughout, mandatory in all code examples, bridges to SDD in Part 5
2. **"Try With AI" ONLY Closure**: No summaries/checklists/key-takeaways after Try With AI section (constitutional compliance)
3. **CEFR A1→A2→B1 Progression**: Validated using skills-proficiency-mapper, ensures no regression, appropriate for beginner tier
4. **Zero Forward References**: Strict exclusion of OOP, control flow, functions (beyond built-ins), collections - preserves chapter focus
5. **CoLearning Throughout**: 20+ strategic placements (not just at lesson end), teaches AI collaboration as core skill

**Validation Results**:
- ✅ CEFR Progression: A1→A1-A2→A2→A2→B1 (no regression)
- ✅ Cognitive Load: All lessons 4-5 concepts (within A1-A2 limits)
- ✅ "Try With AI" Closure: Exactly 4 prompts per lesson, zero additional closures
- ✅ No Forward References: Zero mentions of forbidden concepts
- ✅ CoLearning Pedagogy: 20+ elements embedded throughout
- ✅ Specification Alignment: All 11 evals supported, all 24 FRs addressed
- ✅ Bloom's Taxonomy: Progressive levels (Remember→Understand→Apply→Analyze→Create)
- ✅ DigComp Integration: Digital competence areas mapped for institutional recognition

## Outcome

- ✅ Impact: Comprehensive lesson plan with task breakdown ready for implementation. All constitutional requirements met. Chapter 13 sets the pattern for remaining Part 4 chapters (14-29).
- 🧪 Tests: CEFR progression validated (A1→B1), cognitive load compliant (≤5 concepts), closure pattern correct (4 prompts only), zero forward references confirmed
- 📁 Files: plan.md (1,194 lines), tasks.md (1,654 lines) - both production-ready
- 🔁 Next prompts: Ready for PHASE 3 (/sp.tasks - already created by chapter-planner) OR ADR documentation for significant decisions OR proceed to PHASE 4 implementation
- 🧠 Reflection: Planning phase successfully applied all 5 required skills. Type hints positioned as core (not optional) establishes critical pattern for Part 4. CoLearning pedagogy embedded throughout (not bolted on at end). CEFR proficiency mapping enables institutional recognition and competency-based assessment.

## Evaluation notes (flywheel)

- Failure modes observed: None. All validation checkpoints pass.
- Graders run and results (PASS/FAIL):
  - CEFR Progression: PASS (A1→A1-A2→A2→A2→B1 validated)
  - Cognitive Load: PASS (all lessons 4-5 concepts, within beginner limits)
  - "Try With AI" Closure: PASS (exactly 4 prompts per lesson, no additional sections)
  - No Forward References: PASS (zero mentions of OOP, control flow, functions, collections)
  - CoLearning Elements: PASS (20+ strategic placements, not just at end)
  - Specification Alignment: PASS (all 11 evals, all 24 FRs supported)
  - Task Quality: PASS (35 atomic tasks, clear acceptance criteria, effort estimates realistic)
- Prompt variant (if applicable): Standard /sp.python-chapter workflow with chapter-planner subagent
- Next experiment (smallest change to try): Detect significant architectural decisions and suggest ADR documentation before proceeding to implementation
