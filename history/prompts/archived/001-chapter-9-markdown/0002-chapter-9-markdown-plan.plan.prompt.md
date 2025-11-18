---
id: 0002
title: chapter-9-markdown-plan
stage: plan
date: 2025-11-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-chapter-9-markdown
branch: 001-chapter-9-markdown
user: mjs
command: /sp.plan
labels: ["chapter-9", "markdown", "planning", "lesson-breakdown", "skills-proficiency", "three-tier-teaching"]
links:
  spec: ../../../specs/001-chapter-9-markdown/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-chapter-9-markdown/plan.md
tests:
  - Validation: Constitutional compliance verified for all 17 principles
  - Validation: All 24 functional requirements mapped to lessons
  - Validation: Cognitive load limits enforced (max 5 concepts per section)
  - Validation: Skills proficiency progression validated (A1→A2→B1)
---

## Prompt

```text
Let's formalize the plan to learn markdown using AIDD
```

**Context**: After completing and validating the Chapter 9 specification (spec.md), user requested planning phase to break down the spec into lesson-by-lesson implementation structure.

## Response snapshot

Invoked **chapter-planner subagent** to generate comprehensive implementation plan following the Spec-Driven Development (SDD) workflow.

**Plan Structure (8 Lessons)**:

**Foundation & Context** (Lesson 1):
- Why markdown matters for AIDD (specification language)
- Intent Layer concept introduction
- Duration: 40 min | Proficiency: A1 | Concepts: 5

**Tier 1: Book Teaches Directly** (Lessons 2-5):
- Lesson 2: Headings & hierarchy (40 min, A1, 2 concepts)
- Lesson 3: Lists for organizing ideas (40 min, A2, 3 concepts)
- Lesson 4: Code blocks & inline code (45 min, A2, 4 concepts)
- Lesson 5: Emphasis, links + First Spec Integration (50 min, A2, 4 concepts)

**Tier 2: AI Companion** (Lesson 6):
- Complex markdown via specification (50 min, B1, 2 concepts)
- Student specifies WHAT → AI generates HOW → Student validates

**Tier 3: AI Orchestration** (Lesson 7):
- Scaling markdown operations (50 min, B1, 1 concept)
- Student directs strategy → AI executes tactics

**Integration** (Lesson 8):
- Full AIDD cycle demonstration (60 min, B1+, 1 concept)
- Write spec → AI implements → Validate → Refine

**Key Planning Decisions**:

1. **Cognitive Load**: 22 total concepts across 8 lessons (avg 2.75/lesson, well below max 5)
2. **Proficiency Progression**: A1 (Lessons 1-2) → A2 (Lessons 3-5) → B1 (Lessons 6-7) → B1+ (Lesson 8)
3. **Three-Tier Pattern**: Explicitly structured to enforce Constitution Principle 13
4. **Validation-First**: Every lesson includes validation steps (FR-024 compliance)
5. **No "Ask Your AI" for Basics**: Tier 1 lessons teach foundational markdown directly (FR-020)

**Skills Proficiency Mapping** (using skills-proficiency-mapper v2.0):
- CEFR levels assigned based on 40+ years language learning research
- Bloom's taxonomy applied (Remember → Understand → Apply → Analyze)
- DigComp areas mapped (Information Literacy, Content Creation, Communication)
- Measurable outcomes defined for each proficiency level

**Constitutional Compliance Verified**:
- ✓ Principle 1: Every lesson ends with "Try With AI" closure
- ✓ Principle 12: Cognitive load managed (max 5 concepts per section)
- ✓ Principle 13: Graduated Teaching Pattern (Tier 1/2/3 explicit)
- ✓ Principle 15: Validation steps in all lessons
- ✓ All 17 core principles checked and enforced

**Success Criteria Coverage**:
- SC-001 (90%+ write specs): Lesson 5 hands-on exercise
- SC-002 (85%+ read AI output): Lesson 6 validation practice
- SC-003 (80%+ GitHub README): Lesson 8 final project
- SC-004 (75%+ AI complex markdown): Lesson 6 Tier 2 exercise
- SC-005 (Full AIDD cycle): Lesson 8 integration project
- SC-007 to SC-011: Supported across all lessons

**Risk Mitigation**:
- Cognitive overload → Three-tier spreading
- Low motivation → Strong Lesson 1 hook
- AI tool access → ChatGPT web fallback
- Complex syntax difficulty → Tier 2 pattern handles this
- Students rushing → Scaffolded exercises ensure mastery

## Outcome

- ✅ Impact: Complete lesson-by-lesson implementation plan ready for `/sp.tasks` phase
- 🧪 Tests: Constitutional compliance validated for all 17 principles, all 24 FRs mapped
- 📁 Files: plan.md (242 lines with detailed pedagogical guidance)
- 🔁 Next prompts: `/sp.tasks` to generate actionable task checklist
- 🧠 Reflection: Successfully applied skills-proficiency-mapper to assign CEFR levels and Bloom's taxonomy. Three-tier teaching pattern explicitly structured across lessons 1-8. Cognitive load managed with 22 concepts spread appropriately (avg 2.75/lesson). All spec requirements (FR-001 to FR-024) mapped to specific lessons with acceptance criteria.

## Evaluation notes (flywheel)

- Failure modes observed: None. Plan generated in single iteration with full constitutional compliance.
- Graders run and results (PASS/FAIL): Constitutional checklist PASSED (all 17 principles), Specification coverage PASSED (all 24 FRs mapped), Cognitive load validation PASSED (all lessons within limits)
- Prompt variant (if applicable): N/A - Standard chapter-planner workflow
- Next experiment (smallest change to try): When moving to tasks phase, validate that atomic tasks maintain lesson structure and don't fragment the three-tier teaching pattern across disparate work units.

## Key Planning Achievements

1. **Three-Tier Structure Formalized**: Clear progression from book teaching (Tier 1, Lessons 2-5) → AI Companion (Tier 2, Lesson 6) → AI Orchestration (Tier 3, Lesson 7) → Integration (Lesson 8)

2. **Skills Proficiency Research-Grounded**: Applied CEFR framework (40+ years language learning research), Bloom's taxonomy (70+ years cognitive science), and DigComp 2.1 (EU digital competence 2022) to every lesson

3. **Cognitive Load Scientifically Managed**:
   - Lesson 1: 5 concepts (foundation)
   - Lessons 2-5: 2-4 concepts each (Tier 1 practice)
   - Lessons 6-7: 1-2 concepts each (Tier 2-3 integration)
   - Lesson 8: 1 concept (synthesis)
   - Total: 22 concepts (avg 2.75/lesson, well within beginner limits)

4. **Zero Implementation Details**: Plan describes WHAT to teach and WHY, not HOW to implement technically (stays specification-focused)

5. **Every Lesson Independently Valuable**: Each lesson delivers measurable skill increment, supports scaffolding to next lesson

6. **Real-World Integration**: Lesson 8 culminates in GitHub README creation (portfolio evidence, SC-011)

## Lesson Breakdown Summary

| Lesson | Title | Duration | Tier | CEFR | Concepts | Key Skill |
|--------|-------|----------|------|------|----------|-----------|
| 1 | Why Markdown Matters | 40m | Foundation | A1 | 5 | Understanding Intent Layer |
| 2 | Headings & Hierarchy | 40m | Tier 1 | A1 | 2 | Creating structure with `#` |
| 3 | Lists for Organization | 40m | Tier 1 | A2 | 3 | Bullet & numbered lists |
| 4 | Code Blocks & Examples | 45m | Tier 1 | A2 | 4 | Fenced blocks + language tags |
| 5 | Emphasis, Links + First Spec | 50m | Tier 1 | A2 | 4 | Complete spec document |
| 6 | Complex Markdown with AI | 50m | Tier 2 | B1 | 2 | Specify → AI generates → validate |
| 7 | Scaling Markdown Operations | 50m | Tier 3 | B1 | 1 | Orchestrate multi-file tasks |
| 8 | Full AIDD Cycle Project | 60m | Integration | B1+ | 1 | Spec → Code → Validate cycle |

**Total**: 385 minutes (6.4 hours) of learning content across 8 lessons

## Constitutional Alignment Detail

**Principle 1** (AI-First Teaching): Every lesson ends with "Try With AI" using ChatGPT web (pre-tool phase)

**Principle 12** (Cognitive Load): 22 concepts / 8 lessons = 2.75 avg (well below max 5 for beginner tier)

**Principle 13** (Graduated Teaching):
- Tier 1 (Lessons 2-5): Book teaches foundational markdown syntax directly
- Tier 2 (Lesson 6): AI Companion handles complex syntax (tables, nested lists)
- Tier 3 (Lesson 7): AI Orchestration for scaling operations

**Principle 15** (Validation-First): Every lesson includes validation steps, Lesson 6 explicitly teaches reading/validating AI output

**Principles 5, 9** (Progressive Complexity, Show-Spec-Validate): Proficiency progresses A1→A2→B1→B1+, every code example shows spec → prompt → output → validation

## Next Phase Ready

The plan is complete and validated. The next command will generate the actionable task checklist:

```bash
/sp.tasks
```

This will produce `specs/001-chapter-9-markdown/tasks.md` with:
- Atomic, testable development tasks
- MUST/SHOULD/NICE-TO-HAVE prioritization
- Clear acceptance criteria per task
- Task dependencies and sequencing
- Effort estimates

The tasks will be ready for the content-implementer subagent to execute during the implementation phase.
