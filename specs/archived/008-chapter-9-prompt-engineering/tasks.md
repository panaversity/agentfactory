# Tasks: Chapter 9 - Prompt Engineering for AI-Driven Development

**Input**: Design documents from `/specs/008-chapter-9-prompt-engineering/`
**Prerequisites**: plan.md (8 lesson breakdown), spec.md (6 user stories with priorities)

**Context**: This is educational content creation, not software development. Tasks focus on writing lessons that teach prompt engineering to complete beginners.

**Organization**: Tasks are grouped by lesson (aligned with user stories from spec.md) to enable independent lesson creation and validation.

**Policy for Lesson Authors**: Within this chapter, each lesson must end with a single final section titled "Try With AI" (no "Key Takeaways" or "What's Next"). Before AI tools are taught (e.g., Part-1), use ChatGPT web in that section; after tool onboarding, instruct learners to use their preferred AI companion tool (e.g., Gemini CLI, Claude CLI), optionally providing CLI and web variants.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can be written in parallel (independent lessons, no dependencies)
- **[Lesson]**: Which lesson this task belongs to (L1, L2, L3, etc. - maps to user stories)
- Include exact file paths for content creation

## Path Conventions

**Educational Content Paths** (Docusaurus MDX):
- **Lessons**: `book-source/docs/03-prompt-and-context-engineering/01-prompt-engineering/lesson-X-title.mdx`
- **Exercises**: Embedded in lesson files (not separate)
- **Code Examples**: Embedded in lesson files with proper syntax highlighting

---

## Phase 1: Setup (Content Infrastructure)

**Purpose**: Initialize chapter structure and supporting materials

- [ ] T001 Create chapter directory structure at book-source/docs/03-prompt-and-context-engineering/01-prompt-engineering/
- [ ] T002 Create chapter intro file at book-source/docs/03-prompt-and-context-engineering/01-prompt-engineering/intro.md with chapter overview and learning objectives
- [ ] T003 [P] Create assets directory for diagrams and images at book-source/docs/03-prompt-and-context-engineering/01-prompt-engineering/assets/
- [ ] T004 [P] Configure Docusaurus sidebar for Chapter 9 in book-source/docs/03-prompt-and-context-engineering/sidebars.js

---

## Phase 2: Foundational Content (Prerequisites for All Lessons)

**Purpose**: Create shared resources and templates that all lessons depend on

**⚠️ CRITICAL**: These must be complete before lesson writing begins

- [ ] T005 Create "Try With AI" template with ChatGPT web fallback and AI tool selection guidance for use across all lessons
- [ ] T006 Create code example template showing Spec → AI Prompt → Generated Code → Validation pattern
- [ ] T007 Create exercise template with: Learning Objective, Instructions, Success Criteria, Solution Example
- [ ] T008 [P] Create visual diagrams for context windows, prompt structure, validation checklist in assets/
- [ ] T009 [P] Compile list of 8 technical action verbs (Create, Debug, Refactor, etc.) with definitions and examples

**Checkpoint**: Foundation ready - lesson writing can now begin in parallel

---

## Phase 3: Lesson 1 - Understanding AI Coding Agents (US1 - Priority P1) 🎯 MVP

**Goal**: Teach complete beginners how AI coding agents work differently from traditional tools, establishing foundational mental model

**Independent Test**: Student can explain context windows and identify good vs. bad prompts in a 5-question quiz

**Aligned with User Story 1** (spec.md): Understanding AI Coding Agents as Collaborative Partners

### Content for Lesson 1

- [ ] T010 [L1] Write "What: AI Agents Explained" section in lesson-01-understanding-ai-agents.mdx using contractor/blueprint analogy (15 min reading)
  - Content: Define AI agents, contrast with autocomplete, explain collaborative partnership
  - No jargon, use analogies

- [ ] T011 [L1] Write "Why: Clarity Matters" section in lesson-01-understanding-ai-agents.mdx (5 min reading)
  - Content: "Clear prompts = working code first try" with productivity statistics (55% increase)
  - Business value framing

- [ ] T012 [L1] Write "How: Context Windows" section in lesson-01-understanding-ai-agents.mdx (5 min reading)
  - Content: Explain context windows with visual diagram, what fits/doesn't fit
  - Include token-by-token generation concept

- [ ] T013 [L1] Create "Hands-On Exercise: Identify Good vs. Bad Prompts" in lesson-01-understanding-ai-agents.mdx (10 min activity)
  - Content: 5 prompts (good/bad pairs), student identifies better one with explanation
  - Success: 4/5 correct

- [ ] T014 [L1] Write "Try With AI" section in lesson-01-understanding-ai-agents.mdx
  - Content: Test vague vs. specific prompt with ChatGPT web (before tools taught)
  - Observe difference in AI responses

- [ ] T015 [L1] Add 5 new concepts counter and CEFR A1 validation to lesson-01-understanding-ai-agents.mdx frontmatter
  - Validate: AI agents, context window, token generation, clarity, AI as partner (5 total = A1 limit)

**Checkpoint**: Lesson 1 complete and independently readable. Students understand AI agent fundamentals.

---

## Phase 4: Lesson 2 - Writing Clear Commands (US2 - Priority P2)

**Goal**: Students write first prompts generating working code using strong technical verbs

**Independent Test**: Student writes 3 prompts with technical verbs producing usable AI responses

**Aligned with User Story 2** (spec.md): Constructing Basic Developer Prompts

### Content for Lesson 2

- [ ] T016 [P] [L2] Write "What: Technical Action Verbs" section in lesson-02-writing-clear-commands.mdx (10 min reading)
  - Content: List 8 verbs (Create, Debug, Refactor, etc.) with developer-focused definitions
  - Before/After examples (weak vs. strong)

- [ ] T017 [P] [L2] Write "Why: Strong Commands Guide AI" section in lesson-02-writing-clear-commands.mdx (5 min reading)
  - Content: Weak = confusion, Strong = specific solutions
  - Business metric: "70% success rate on first try"

- [ ] T018 [L2] Write "How: Command Structure" section in lesson-02-writing-clear-commands.mdx (5 min reading)
  - Content: Pattern: [Verb] + [Target] + [Desired Outcome]
  - 3 examples with full prompts

- [ ] T019 [L2] Create Exercise 1 "Identify Strong Commands" in lesson-02-writing-clear-commands.mdx (5 min activity)
  - Content: 5 prompts, identify strongest verb
  - Success: 4/5 correct

- [ ] T020 [L2] Create Exercise 2 "Rewrite Vague to Specific" in lesson-02-writing-clear-commands.mdx (10 min activity)
  - Content: Transform 3 vague prompts using technical verbs
  - Success: All 3 rewritten with strong verbs

- [ ] T021 [L2] Create Exercise 3 "Write Your Own + Test" in lesson-02-writing-clear-commands.mdx (5 min activity)
  - Content: Student writes 1 prompt, tests with AI
  - Success: AI generates relevant code

- [ ] T022 [L2] Write "Try With AI" section in lesson-02-writing-clear-commands.mdx
  - Content: Test vague vs. strong command with preferred AI tool
  - Example: "Create a Python function that takes two numbers and returns their sum"
  - Observe clarity difference

- [ ] T023 [L2] Add 5 new concepts counter and CEFR A1-A2 validation to lesson-02-writing-clear-commands.mdx frontmatter
  - Validate: Technical verbs, vague vs. specific, command structure, testing prompts, iterative refinement (5 total)

**Checkpoint**: Lesson 2 complete. Students can generate working code using strong commands.

---

## Phase 5: Lesson 3 - Providing Technical Context (US2 continued - Priority P2)

**Goal**: Students add 4-layer context transforming generic AI code into project-specific implementations

**Independent Test**: Student adds 4-layer context to basic prompt, AI produces project-specific code

**Aligned with User Story 2** (spec.md): Constructing Basic Developer Prompts (advanced)

### Content for Lesson 3

- [ ] T024 [P] [L3] Write "What: The AIDD Context Stack" section in lesson-03-providing-context.mdx (12 min reading)
  - Content: Four layers explained (project, code, constraints, developer)
  - Contractor/blueprints analogy

- [ ] T025 [P] [L3] Write "Why: Project-Specific Code" section in lesson-03-providing-context.mdx (5 min reading)
  - Content: Generic vs. contextual prompts comparison
  - "AI can't read your mind"

- [ ] T026 [L3] Write "How: Building Context Layers" section in lesson-03-providing-context.mdx (8 min reading)
  - Content: Example for each layer (FastAPI, Python 3.11, PEP 8, etc.)
  - Complete 4-layer prompt example

- [ ] T027 [L3] Create Exercise 1 "Identify Missing Context" in lesson-03-providing-context.mdx (5 min activity)
  - Content: Given prompt, identify which of 4 layers missing
  - Success: Correctly identify all missing layers

- [ ] T028 [L3] Create Exercise 2 "Add Context to Basic Prompt" in lesson-03-providing-context.mdx (10 min activity)
  - Content: Start with "Create user auth function", add all 4 layers
  - Test with AI, observe specificity improvement
  - Success: AI includes project-specific details

- [ ] T029 [L3] Create Exercise 3 "Use Existing Code as Context" in lesson-03-providing-context.mdx (5 min activity)
  - Content: Include code snippet, prompt AI to match style
  - Success: AI matches example pattern

- [ ] T030 [L3] Write "Try With AI" section in lesson-03-providing-context.mdx
  - Content: Compare generic vs. contextual prompt
  - Example: Email validation with FastAPI context
  - Execute student's contextual prompt from Exercise 2

- [ ] T031 [L3] Add 5 new concepts counter and CEFR A2 validation to lesson-03-providing-context.mdx frontmatter
  - Validate: 4-layer stack, project context, code context, constraint context, developer context (5 total)

**Checkpoint**: Lesson 3 complete. Students provide context for project-specific AI code.

---

## Phase 6: Lesson 4 - Specifying Logic (US2 continued - Priority P2)

**Goal**: Students write 5-8 numbered implementation steps preventing AI from guessing architecture

**Independent Test**: Student writes 8-step plan for to-do API endpoint; AI follows steps exactly

**Aligned with User Story 2** (spec.md): Constructing Basic Developer Prompts (logic layer)

### Content for Lesson 4

- [ ] T032 [P] [L4] Write "What: Logic Specification for AIDD" section in lesson-04-specifying-logic.mdx (12 min reading)
  - Content: Explicit step-by-step instructions (not vague requirements)
  - Examples: user registration, caching, payment processing

- [ ] T033 [P] [L4] Write "Why: Prevent AI Guessing" section in lesson-04-specifying-logic.mdx (5 min reading)
  - Content: Without logic = assumptions, With logic = your exact approach
  - "You architect, AI builds"

- [ ] T034 [L4] Write "How: Writing Implementation Steps" section in lesson-04-specifying-logic.mdx (8 min reading)
  - Content: Basic logic (5-8 steps), Advanced logic (design patterns, algorithms)
  - 2 complete examples with numbered steps

- [ ] T035 [L4] Create Exercise 1 "Recognize Logic vs. Requirements" in lesson-04-specifying-logic.mdx (5 min activity)
  - Content: 3 prompts, identify which specifies logic
  - Success: 3/3 correct

- [ ] T036 [L4] Create Exercise 2 "Write Implementation Steps" in lesson-04-specifying-logic.mdx (15 min activity)
  - Content: "Build to-do list API", write 8 steps for "create task" endpoint
  - Test with AI, verify AI follows steps
  - Success: Generated code matches logic flow

- [ ] T037 [L4] Create Exercise 3 "Add Logic to Existing Prompt" in lesson-04-specifying-logic.mdx (5 min activity)
  - Content: Take Lesson 3 contextual prompt, add 5-step logic
  - Observe improvement

- [ ] T038 [L4] Write "Try With AI" section in lesson-04-specifying-logic.mdx
  - Content: Compare vague vs. logic-specified payment processing
  - Execute student's 8-step logic from Exercise 2

- [ ] T039 [L4] Add 2 new concepts counter and CEFR A2→B1 transition validation to lesson-04-specifying-logic.mdx frontmatter
  - Validate: Implementation logic, numbered steps (2 total - transition to B1)

**Checkpoint**: Lesson 4 complete. Students specify logic, preventing AI assumptions.

---

## Phase 7: Lesson 5 - Validating AI-Generated Code (US5 - Priority P2 SAFETY-CRITICAL)

**Goal**: Students apply 5-step validation checklist to ALL AI code, recognizing red flags

**Independent Test**: Student reviews intentionally flawed code, identifies 3+ issues using checklist

**Aligned with User Story 5** (spec.md): Validating AI-Generated Code

### Content for Lesson 5

- [ ] T040 [P] [L5] Write "What: Validation-First Safety Culture" section in lesson-05-validating-code.mdx (10 min reading)
  - Content: Why this lesson NOW (after students generate code L2-L4)
  - "AI generates fast; YOU ensure correct and safe"
  - Professional responsibility

- [ ] T041 [P] [L5] Write "Why: Validation Matters" section in lesson-05-validating-code.mdx (5 min reading)
  - Content: AI makes mistakes (hallucinations, security issues)
  - Real-world consequences of blind trust
  - "Trust but verify" standard

- [ ] T042 [L5] Write "How: The 5-Step Validation Checklist" section in lesson-05-validating-code.mdx (5 min reading)
  - Content: 1) Read First, 2) Check Secrets, 3) Check Issues, 4) Test It, 5) Compare to Spec
  - Red flags list (hardcoded passwords, missing error handling, etc.)

- [ ] T043 [L5] Create Exercise 1 "Apply Validation Checklist" in lesson-05-validating-code.mdx (10 min activity)
  - Content: Flawed code sample (3 deliberate issues: hardcoded password, no error handling, no type hints)
  - Apply checklist, identify all 3
  - Success: All 3 issues found

- [ ] T044 [L5] Create Exercise 2 "Red Flag Recognition" in lesson-05-validating-code.mdx (5 min activity)
  - Content: 5 code snippets, identify red flags
  - Success: 4/5 correct

- [ ] T045 [L5] Create Exercise 3 "Validate and Fix" in lesson-05-validating-code.mdx (10 min activity)
  - Content: Generate code with AI, apply checklist, prompt AI to fix issues
  - Success: Final code passes all 5 steps

- [ ] T046 [L5] Write "Try With AI" section in lesson-05-validating-code.mdx
  - Content: Generate database connection code, validate, iterate to fix
  - Demonstrate fix prompt: "Fix to avoid hardcoded credentials and add error handling"

- [ ] T047 [L5] Add 2 new concepts counter and CEFR A2 validation to lesson-05-validating-code.mdx frontmatter
  - Validate: 5-step checklist, red flags (2 total)

**Checkpoint**: Lesson 5 complete. Validation is permanent habit. SAFETY-CRITICAL skill established.

---

## Phase 8: Lesson 6 - Technical Constraints and Examples (US3 - Priority P3)

**Goal**: Students add constraints and examples for production-ready, project-specific code

**Independent Test**: Student adds 3 constraints to basic prompt, AI meets all requirements

**Aligned with User Story 3** (spec.md): Adding Technical Constraints and Examples

### Content for Lesson 6

- [ ] T048 [P] [L6] Write "What: Technical Constraints in AIDD" section in lesson-06-constraints-examples.mdx (10 min reading)
  - Content: Specific requirements AI must follow
  - Categories: Dependency, Performance, Security, Code Quality, Integration

- [ ] T049 [P] [L6] Write "Why: Constraints Ensure Project Fit" section in lesson-06-constraints-examples.mdx (5 min reading)
  - Content: Without = latest features (may not match), With = exact needs
  - "Constraints are guardrails"

- [ ] T050 [L6] Write "How: Specifying Constraints and Examples" section in lesson-06-constraints-examples.mdx (5 min reading)
  - Content: Dependency constraints (Python 3.11, FastAPI 0.104)
  - Security constraints (bcrypt, rate limiting)
  - Code quality constraints (type hints, 80% coverage)
  - Code example showing existing service pattern

- [ ] T051 [L6] Create Exercise 1 "Add Constraints to Prompt" in lesson-06-constraints-examples.mdx (10 min activity)
  - Content: Basic prompt + 3 constraints (version, security, quality)
  - Test with AI
  - Success: AI output meets constraints

- [ ] T052 [L6] Create Exercise 2 "Provide Style Example" in lesson-06-constraints-examples.mdx (5 min activity)
  - Content: Include code snippet, prompt AI to match
  - Success: AI follows pattern

- [ ] T053 [L6] Create Exercise 3 "Combine Constraints + Examples" in lesson-06-constraints-examples.mdx (5 min activity)
  - Content: Command + Context + Logic + Constraints + Example
  - Test with AI
  - Success: Production-ready code

- [ ] T054 [L6] Write "Try With AI" section in lesson-06-constraints-examples.mdx
  - Content: Compare unconstrained vs. constrained database connection
  - Observe production-readiness difference

- [ ] T055 [L6] Add 3 new concepts counter and CEFR A2→B1 validation to lesson-06-constraints-examples.mdx frontmatter
  - Validate: Technical constraints, code examples, integration constraints (3 total)

**Checkpoint**: Lesson 6 complete. Students refine AI output for project fit.

---

## Phase 9: Lesson 7 - Question-Driven Development and Roleplay (US4 - Priority P4)

**Goal**: Students initiate QDD workflow and adopt specialized AI roles for expert responses

**Independent Test**: Student prompts AI to ask 10 questions before complex feature; tailored code results

**Aligned with User Story 4** (spec.md): Using Question-Driven Development with AI

### Content for Lesson 7

- [ ] T056 [P] [L7] Write "What: Question-Driven Development" section in lesson-07-qdd-roleplay.mdx (10 min reading)
  - Content: Most powerful AIDD technique
  - Process: Prompt → AI asks 10 questions → You answer → Tailored solution
  - Why: Prevents generic solutions

- [ ] T057 [P] [L7] Write "Why: QDD Produces Better Code" section in lesson-07-qdd-roleplay.mdx (5 min reading)
  - Content: Without QDD = generic, With QDD = tailored
  - "10 min answering questions saves hours debugging"

- [ ] T058 [L7] Write "How: The QDD Process" section in lesson-07-qdd-roleplay.mdx (5 min reading)
  - Content: 4 steps with authentication system example
  - Step 1: Initial prompt with question request
  - Step 2: AI asks 10 questions
  - Step 3: Provide answers
  - Step 4: AI generates tailored code

- [ ] T059 [L7] Write "Roleplay: Specialized Technical Expertise" section in lesson-07-qdd-roleplay.mdx (5 min reading)
  - Content: Generic vs. specialized roles
  - Backend engineer, frontend developer, DevOps examples
  - Style modifiers (pragmatic, defensive coding)

- [ ] T060 [L7] Create Exercise 1 "Practice QDD Workflow" in lesson-07-qdd-roleplay.mdx (20 min activity)
  - Content: Choose complex feature (payment processing)
  - Prompt AI for 10 questions
  - Provide answers
  - AI generates tailored code
  - Success: AI asks relevant questions, final code reflects answers

- [ ] T061 [L7] Create Exercise 2 "Adopt Specialized Role" in lesson-07-qdd-roleplay.mdx (10 min activity)
  - Content: Create backend engineer roleplay prompt
  - Test with same feature
  - Compare generic vs. specialized
  - Success: Specialized produces expert responses

- [ ] T062 [L7] Write "Try With AI" section in lesson-07-qdd-roleplay.mdx
  - Content: QDD workflow for task management REST API
  - Prompt: "Before implementing, ask me 10 technical questions"
  - Compare QDD vs. direct implementation

- [ ] T063 [L7] Add 3 new concepts counter and CEFR B1 validation to lesson-07-qdd-roleplay.mdx frontmatter
  - Validate: QDD workflow, AI roleplay, AI as consultant (3 total)

**Checkpoint**: Lesson 7 complete. Students use advanced collaboration techniques.

---

## Phase 10: Lesson 8 - Mastery Integration and Capstone (US6 - Priority P5)

**Goal**: Students create prompt templates and build portfolio-worthy capstone project using all 8 elements

**Independent Test**: Student completes capstone (REST API, data processor, or CLI tool) demonstrating all 8 framework elements

**Aligned with User Story 6** (spec.md): Building a Personal Prompt Library

### Content for Lesson 8

- [ ] T064 [P] [L8] Write "What: Systematizing Prompt Engineering" section in lesson-08-mastery-capstone.mdx (8 min reading)
  - Content: Reusable templates accelerate recurring tasks
  - Common tasks: New feature, Bug fix, Refactoring, Optimization, Testing

- [ ] T065 [P] [L8] Write "Why: Templates Save Time" section in lesson-08-mastery-capstone.mdx (3 min reading)
  - Content: Don't rewrite from scratch, ensure no forgotten elements
  - "Systematize what works"

- [ ] T066 [L8] Write "How: Building Prompt Templates" section in lesson-08-mastery-capstone.mdx (4 min reading)
  - Content: Template 1 (New API Endpoint), Template 2 (Bug Fix), Template 3 (Refactoring)
  - Show all 8 elements structure

- [ ] T067 [L8] Write "Capstone Project: Portfolio-Worthy Application" section in lesson-08-mastery-capstone.mdx (5 min reading)
  - Content: 3 project options (REST API, Data Processor, CLI Tool)
  - Requirements: All 8 elements, QDD workflow, validation checklist, 2+ templates, working code + tests + docs

- [ ] T068 [L8] Create Capstone Phase 1 "Planning" instructions in lesson-08-mastery-capstone.mdx (10 min activity)
  - Content: Choose project, use QDD (AI asks 10 questions), answer questions
  - Success: Clear project requirements from QDD

- [ ] T069 [L8] Create Capstone Phase 2 "Implementation" instructions in lesson-08-mastery-capstone.mdx (25 min activity)
  - Content: Use all 8 AIDD elements, generate code with AI, validate all code (5-step checklist), iterate
  - Success: Working code meeting requirements

- [ ] T070 [L8] Create Capstone Phase 3 "Documentation" instructions in lesson-08-mastery-capstone.mdx (10 min activity)
  - Content: Create README, document prompts used, include "built with AI" note, portfolio-ready
  - Success: Complete project with documentation

- [ ] T071 [L8] Write "Try With AI" section in lesson-08-mastery-capstone.mdx
  - Content: Execute capstone project with preferred AI tool
  - Students complete 45-minute portfolio project

- [ ] T072 [L8] Add 0 new concepts (synthesis only) and CEFR B1+ validation to lesson-08-mastery-capstone.mdx frontmatter
  - Validate: Synthesis of all 25 concepts from L1-L7

**Checkpoint**: Lesson 8 complete. Students have portfolio-worthy project demonstrating mastery.

---

## Phase 11: Polish & Cross-Cutting Concerns

**Purpose**: Final quality checks and chapter-level integration

- [ ] T073 [P] Review all 8 lessons for consistent voice, tone, and Concept-First pattern (WHAT → WHY → HOW → PRACTICE)
- [ ] T074 [P] Validate all "Try With AI" sections follow policy (single final section, ChatGPT fallback early, preferred tool later)
- [ ] T075 [P] Verify all lessons meet CEFR cognitive load limits (L1-L3: max 5, L4-L7: max 7-10, L8: 0 new)
- [ ] T076 Ensure all code examples follow Spec → Prompt → Code → Validation pattern
- [ ] T077 [P] Create visual diagrams referenced in lessons and save to assets/ (context window, prompt structure, validation checklist)
- [ ] T078 [P] Proofread all lessons for beginner-friendly language (no jargon without definition)
- [ ] T079 Test chapter intro.md navigation and learning objectives align with 8 lessons
- [ ] T080 [P] Validate Docusaurus build with all 8 lessons (no broken links, proper syntax highlighting)
- [ ] T081 Create chapter summary in intro.md with "What You'll Learn" and "What You'll Build" (capstone project)
- [ ] T082 [P] Add chapter to book navigation in main sidebars.js
- [ ] T083 Run final quality check against spec.md success criteria (80% working prompts, 90% understand clarity, etc.)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all lesson writing
- **Lessons 1-8 (Phases 3-10)**: All depend on Foundational phase completion
  - Lessons 1-2 SHOULD be written first (foundation for later lessons)
  - Lessons 3-7 CAN proceed in parallel after L1-L2 (if capacity allows)
  - Lesson 8 SHOULD be written last (synthesizes all prior lessons)
- **Polish (Phase 11)**: Depends on all 8 lessons being complete

### Lesson Dependencies

- **Lesson 1 (L1)**: Can start after Foundational - No dependencies on other lessons
- **Lesson 2 (L2)**: Can start after Foundational - References L1 concepts (sequential recommended)
- **Lesson 3 (L3)**: References L2 prompts - Sequential recommended
- **Lesson 4 (L4)**: References L3 contextual prompts - Sequential recommended
- **Lesson 5 (L5)**: References code from L2-L4 - Can start after L4
- **Lesson 6 (L6)**: References L2-L3 prompts - Can start after L3 (parallel with L4-L5)
- **Lesson 7 (L7)**: Independent advanced technique - Can start after L4 (parallel with L5-L6)
- **Lesson 8 (L8)**: Synthesizes ALL prior lessons - MUST be last

### Within Each Lesson

1. Write WHAT section (concept explanation)
2. Write WHY section (value/motivation)
3. Write HOW section (technique details)
4. Create PRACTICE exercises
5. Write "Try With AI" section
6. Add frontmatter (CEFR validation, concepts count)
7. Validate against cognitive load limits

### Parallel Opportunities

- **Setup tasks** (T001-T004): All can run in parallel
- **Foundational tasks** (T005-T009): All can run in parallel within Phase 2
- **Lesson writing**: After L1-L2 complete, L3-L7 can proceed in parallel (with noted dependencies)
- **Polish tasks** (T073-T083): Tasks marked [P] can run in parallel

---

## Parallel Example: Multiple Lessons

```bash
# After Foundational phase and L1-L2 complete, launch independent lessons:

# Parallel Track 1: Lesson 3 (Context)
Task T024-T031: Write Lesson 3 content

# Parallel Track 2: Lesson 6 (Constraints) - independent of L3
Task T048-T055: Write Lesson 6 content

# Parallel Track 3: Lesson 7 (QDD) - independent of L3, L6
Task T056-T063: Write Lesson 7 content

# Then Sequential: Lesson 4 (needs L3), Lesson 5 (needs L2-L4), Lesson 8 (needs all)
```

---

## Implementation Strategy

### MVP First (Lessons 1-2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all lessons)
3. Complete Phase 3: Lesson 1 (Understanding AI Agents)
4. Complete Phase 4: Lesson 2 (Writing Clear Commands)
5. **STOP and VALIDATE**: Test with 5-10 target beginners
   - Can students explain context windows?
   - Can students write prompts generating working code?
   - Is cognitive load manageable?
6. Refine based on feedback before proceeding to L3-L8

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add Lesson 1 → Test independently → Students understand AI agents (MVP!)
3. Add Lesson 2 → Test independently → Students generate code
4. Add Lessons 3-4 → Test → Students add context and logic
5. Add Lesson 5 → Test → Students validate code (safety-critical)
6. Add Lessons 6-7 → Test → Students use advanced techniques
7. Add Lesson 8 → Test → Students complete capstone project
8. Each lesson adds skills without breaking previous learning

### Parallel Team Strategy

With multiple content creators:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Writer A: Lessons 1-2 (sequential, foundation for others)
   - Writer B: Lessons 3-4 (after L1-L2 complete, sequential pair)
   - Writer C: Lessons 5-6 (L5 needs L2-L4, L6 can start earlier)
   - Writer D: Lesson 7 (independent advanced technique)
3. Writer A: Lesson 8 (after all others complete, synthesizes everything)
4. All writers: Polish phase in parallel

---

## Notes

- **[P] tasks**: Different lessons/files, no dependencies, can run in parallel
- **[Lesson] label**: Maps task to specific lesson (L1-L8) for traceability
- **Each lesson independently completable**: Students can stop after any lesson with working knowledge
- **"Try With AI" policy**: Single final section per lesson, ChatGPT fallback early, preferred tool later
- **Validation checkpoints**: Test lessons with target beginners before proceeding
- **Cognitive load limits**: Strictly enforce CEFR standards (L1-L3: 5 max, L4-L7: varies, L8: 0 new)
- **Concept-First pattern**: Every lesson follows WHAT → WHY → HOW → PRACTICE structure
- **Avoid**: Jargon without definition, forward references, exceeding concept limits, skipping validation

---

## Task Summary

**Total Tasks**: 83
- Phase 1 (Setup): 4 tasks
- Phase 2 (Foundational): 5 tasks
- Phase 3 (Lesson 1): 6 tasks
- Phase 4 (Lesson 2): 8 tasks
- Phase 5 (Lesson 3): 8 tasks
- Phase 6 (Lesson 4): 8 tasks
- Phase 7 (Lesson 5): 8 tasks
- Phase 8 (Lesson 6): 8 tasks
- Phase 9 (Lesson 7): 8 tasks
- Phase 10 (Lesson 8): 9 tasks
- Phase 11 (Polish): 11 tasks

**Parallel Opportunities**: 28 tasks marked [P] can run in parallel when dependencies met

**Independent Test Criteria**:
- Lesson 1: Explain context windows, identify good vs. bad prompts (5-question quiz)
- Lesson 2: Write 3 prompts generating usable AI responses
- Lesson 3: Add 4-layer context producing project-specific code
- Lesson 4: Write 8-step plan AI follows exactly
- Lesson 5: Apply checklist identifying 3+ issues in flawed code
- Lesson 6: Add 3 constraints meeting all requirements
- Lesson 7: Initiate QDD getting 10 questions, tailored code
- Lesson 8: Complete capstone project (portfolio-worthy application)

**Suggested MVP Scope**: Lessons 1-2 (25 + 40 = 65 min)
- Students understand AI agents AND generate working code
- Provides complete value for beginners getting started
- Can validate with users before investing in L3-L8

**Format Validation**: ✅ All 83 tasks follow checklist format with checkbox, ID, optional [P]/[Lesson] labels, description with file paths
