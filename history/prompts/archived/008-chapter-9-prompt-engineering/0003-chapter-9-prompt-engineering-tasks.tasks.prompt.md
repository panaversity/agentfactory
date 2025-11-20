---
id: 0003
title: chapter-9-prompt-engineering-tasks
stage: tasks
date: 2025-11-02
surface: agent
model: claude-sonnet-4-5-20250929
feature: 008-chapter-9-prompt-engineering
branch: 008-chapter-9-prompt-engineering
user: System
command: /sp.tasks
labels: ["chapter-9", "prompt-engineering", "tasks", "lesson-creation", "educational-content"]
links:
  spec: specs/008-chapter-9-prompt-engineering/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/008-chapter-9-prompt-engineering/tasks.md
tests:
 - Format validation: All 83 tasks follow checklist format (PASS)
 - Independent test criteria defined for all 8 lessons (PASS)
 - Lesson dependency mapping validated (PASS)
---

## Prompt

Generate actionable tasks.

**User Requirements**: Create immediately executable tasks for Chapter 9 educational content

**Context**: This is lesson/content creation (not software development). Tasks must organize 8 lessons mapped to 6 user stories from spec.md, with proper CEFR cognitive load validation and beginner-friendly pedagogical approach.

## Response snapshot

**83 Actionable Tasks Generated** for Chapter 9: Prompt Engineering for AI-Driven Development

**Task Organization** (Educational Content Adaptation):

**Phase 1: Setup** (4 tasks)
- Create chapter directory structure
- Create chapter intro file
- Create assets directory for diagrams
- Configure Docusaurus sidebar

**Phase 2: Foundational** (5 tasks) - BLOCKS all lesson writing
- "Try With AI" template
- Code example template (Spec → Prompt → Code → Validation)
- Exercise template
- Visual diagrams (context windows, prompt structure, validation)
- Technical action verbs list

**Phases 3-10: Lessons 1-8** (64 tasks total)
- **Lesson 1 (L1)**: 6 tasks - Understanding AI Agents (US1, P1)
- **Lesson 2 (L2)**: 8 tasks - Writing Clear Commands (US2, P2)
- **Lesson 3 (L3)**: 8 tasks - Providing Context (US2 continued, P2)
- **Lesson 4 (L4)**: 8 tasks - Specifying Logic (US2 continued, P2)
- **Lesson 5 (L5)**: 8 tasks - Validating Code (US5, P2 SAFETY-CRITICAL)
- **Lesson 6 (L6)**: 8 tasks - Constraints & Examples (US3, P3)
- **Lesson 7 (L7)**: 8 tasks - QDD & Roleplay (US4, P4)
- **Lesson 8 (L8)**: 9 tasks - Mastery & Capstone (US6, P5)

**Phase 11: Polish** (11 tasks)
- Consistent voice/tone review
- "Try With AI" policy validation
- CEFR cognitive load verification
- Code example pattern validation
- Visual diagram creation
- Beginner-friendly language proofread
- Docusaurus build test

**Key Features**:

1. **Lesson-Based Organization** (not typical software tasks):
   - Each lesson = independent learning unit
   - Tasks include: Write WHAT/WHY/HOW sections, Create exercises, Write "Try With AI" section, Add CEFR validation

2. **Cognitive Load Management**:
   - L1-L3: Max 5 new concepts (CEFR A1 limit)
   - L4-L7: 2-3 new concepts (transition to B1)
   - L8: 0 new concepts (synthesis only)
   - Validated in frontmatter for each lesson

3. **Concept-First Pattern** (every lesson):
   - WHAT (concept explanation) → WHY (value/motivation) → HOW (technique) → PRACTICE (exercises) → TRY WITH AI

4. **Educational Content Paths**:
   - Lessons: `book-source/docs/03-prompt-and-context-engineering/01-prompt-engineering/lesson-X-title.mdx`
   - Assets: `book-source/docs/03-prompt-and-context-engineering/01-prompt-engineering/assets/`

5. **"Try With AI" Policy** enforced:
   - Single final section per lesson (no "Key Takeaways" or "What's Next")
   - ChatGPT web fallback early (before tools taught)
   - Preferred AI tool later (Gemini CLI, Claude CLI)

6. **Independent Test Criteria** (per lesson):
   - L1: Explain context windows, identify good/bad prompts (quiz)
   - L2: Write 3 prompts generating usable AI responses
   - L3: Add 4-layer context producing project-specific code
   - L4: Write 8-step plan AI follows exactly
   - L5: Apply checklist identifying 3+ issues
   - L6: Add 3 constraints meeting requirements
   - L7: Initiate QDD, get 10 questions, tailored code
   - L8: Complete capstone project (portfolio-worthy)

**Parallel Opportunities**: 28 tasks marked [P] can run in parallel

**Dependencies**:
- Foundational BLOCKS all lessons (must complete first)
- L1-L2 should be sequential (foundation for others)
- L3-L7 can proceed in parallel after L1-L2 (with noted dependencies)
- L8 MUST be last (synthesizes all prior lessons)

**MVP Scope**: Lessons 1-2 only (65 minutes)
- Students understand AI agents AND generate code
- Complete value for beginners
- Validate with users before L3-L8

**Execution Strategy**:
1. MVP: L1-L2 → Test with 5-10 beginners → Refine
2. Incremental: Add L3-L4 → Test → Add L5 (safety) → Test → Add L6-L7 → Test → Add L8 (capstone)
3. Parallel team: After L1-L2, writers can tackle L3-L7 in parallel, then L8

## Outcome

- ✅ Impact: 83 actionable tasks generated for Chapter 9 educational content creation, organized by 8 lessons mapped to 6 user stories. Tasks adapted for lesson writing (not software dev) with Concept-First pattern, CEFR cognitive load validation, and beginner-friendly pedagogical approach.

- 🧪 Tests:
  - Format validation: **PASS** - All 83 tasks follow checklist format (checkbox, ID, [P]/[Lesson] labels, file paths)
  - Independent test criteria: **PASS** - Defined for all 8 lessons
  - Lesson dependencies: **PASS** - Foundational blocks all, L1-L2 sequential, L3-L7 can be parallel, L8 last
  - Cognitive load limits: **PASS** - Validated per lesson (L1-L3: 5 max, L4-L7: varies, L8: 0)

- 📁 Files:
  - specs/008-chapter-9-prompt-engineering/tasks.md (83 tasks across 11 phases)

- 🔁 Next prompts:
  - `/sp.implement` to begin lesson content creation (start with Setup + Foundational phases)
  - OR manual execution: Start with T001-T009 (Setup + Foundational) before lesson writing
  - Consider MVP validation: Complete L1-L2, test with 5-10 beginners, refine before L3-L8

- 🧠 Reflection: Tasks successfully adapted for educational content creation vs. traditional software development. Instead of models/services/endpoints, tasks focus on: writing lesson sections (WHAT/WHY/HOW), creating exercises, writing "Try With AI" sections, validating CEFR cognitive load. The lesson-based organization mirrors user story organization (each lesson independently completable). Parallel opportunities identified for content creators working simultaneously (28 [P] tasks). MVP scope (L1-L2) provides complete beginner value and validation checkpoint. Risk: Educational content tasks are less familiar than software tasks - lesson authors may need guidance on Concept-First pattern and CEFR validation.

## Evaluation notes (flywheel)

- Failure modes observed: None. Task generation workflow executed correctly: check-prerequisites → load spec.md + plan.md → adapt template for educational content → generate 83 tasks → validate format → create PHR.

- Graders run and results (PASS/FAIL):
  - Task Format Validation: **PASS** (all 83 follow checkbox, ID, labels, file paths)
  - Independent Test Criteria: **PASS** (8 lessons each have measurable test)
  - Dependency Mapping: **PASS** (Foundational blocks all, lesson sequence validated)
  - Cognitive Load Validation: **PASS** (CEFR limits specified per lesson)
  - Educational Content Adaptation: **PASS** (tasks focus on lesson writing, not code development)

- Prompt variant (if applicable): User requested "Generate actionable tasks" without specific adaptations, but context (educational content for beginners) required significant adaptation from standard software tasks template.

- Next experiment (smallest change to try): During lesson implementation (T010+), monitor if content creators successfully apply Concept-First pattern (WHAT → WHY → HOW → PRACTICE) and CEFR cognitive load validation. If writers struggle, create supplementary "Lesson Writing Guide" with examples and checklist.
