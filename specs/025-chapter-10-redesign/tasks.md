---
description: "Implementation task list for Chapter 10 redesign"
---

# Tasks: Chapter 10 - Prompt Engineering for Strategic Product Intelligence

**Input**: Design documents from `/specs/025-chapter-10-redesign/`
**Prerequisites**: plan.md (lesson structure), spec.md (user stories), RESEARCH-REPORT.md (verified platform capabilities)

**Organization**: Tasks are organized by pedagogical phase (Setup → Stage 1 → Stage 2 → Stage 3 → Stage 4) to enable incremental lesson implementation and validation.

## Format: `[ID] [P?] [Lesson?] Description`

- **[P]**: Can run in parallel (different lesson files, no dependencies)
- **[Lesson#]**: Which lesson this task belongs to (e.g., L1, L2, L3...L8)
- Include exact file paths in descriptions

## Path Conventions

- Chapter directory: `book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/`
- Lesson files: `01-understanding-ai-agents.md`, `02-writing-clear-commands.md`, etc.
- Skills directory: `.claude/skills/`
- Sample codebases: `book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/samples/`

## Policy Note for Lesson Authors

Within this chapter, each lesson MUST end with a single final section titled **"Try With AI"** (no "Key Takeaways" or "What's Next" sections). Before AI tools are taught (Part 1 lessons), use ChatGPT web in that section. After tool onboarding (this is Part 3), instruct learners to use their preferred AI companion tool (Gemini CLI or Claude Code), optionally providing both CLI and web variants.

---

## Phase 1: Setup (Preparation & Infrastructure)

**Purpose**: Prepare chapter directory structure, gather sample codebases, create template files

- [ ] T001 Create chapter directory at book-source/docs/03-Markdown-Prompt-Context-Engineering/10-prompt-engineering-for-aidd/
- [ ] T002 [P] Create samples/ subdirectory for example codebases
- [ ] T003 [P] Create skills/ subdirectory for reusable prompt skills
- [ ] T004 [P] Curate 5 sample codebases (small/medium/large) for lessons and capstone
  - Small: 10-15 files, single purpose (e.g., CLI tool)
  - Medium: 20-30 files, multi-module (e.g., FastAPI project)
  - Large: 50+ files, production-scale (e.g., open-source framework)
- [ ] T005 Create README.md with chapter overview and strategic framing (AI Product Manager perspective)
- [ ] T006 [P] Create CLAUDE.md template file in samples/templates/
- [ ] T007 [P] Create GEMINI.md template file in samples/templates/
- [ ] T008 [P] Create capstone-spec-template.md for lesson 8 specification writing

**Checkpoint**: Chapter infrastructure ready - lesson implementation can begin

---

## Phase 2: Stage 1 - Manual Foundation (Lessons 1-2)

**Purpose**: Build mental models WITHOUT AI tools (per Constitution Stage 1)

**⚠️ CRITICAL**: These lessons establish foundation - NO AI tool usage, only conceptual understanding

### Lesson 1: Understanding AI Agents as Codebase Analysts

**Goal**: Students understand how AI agents think differently than traditional tools

**Independent Test**: Student can explain context windows, token-by-token generation, and when AI helps vs. requires human verification (validated through self-reflection exercise)

- [ ] T009 [L1] Create 01-understanding-ai-agents.md in chapter directory
- [ ] T010 [L1] Write "What Should This Lesson Enable?" specification section (WHAT before HOW)
- [ ] T011 [L1] Write Foundation Understanding section: AI agents vs. autocomplete/search comparison table
- [ ] T012 [L1] Write Manual Reasoning Exercise: Socratic questions guiding codebase analysis discovery
- [ ] T013 [L1] Write Token-by-Token Generation Model section with vague vs. clear prompt examples
- [ ] T014 [L1] Write "Try With AI" section (self-reflection only, no AI tool usage per Stage 1)
- [ ] T015 [L1] Validate lesson cognitive load ≤7 concepts (Constitution B1 tier requirement)

**Checkpoint**: Lesson 1 complete - Validates foundation before collaboration

### Lesson 2: Writing Clear Commands (Specification-First Fundamentals)

**Goal**: Students learn to specify WHAT they want before HOW to ask

**Independent Test**: Student writes specification for codebase analysis task before writing prompt (validated through practice exercise)

- [ ] T016 [L2] Create 02-writing-clear-commands.md in chapter directory
- [ ] T017 [L2] Write "What Should This Lesson Enable?" specification section
- [ ] T018 [L2] Write Specification-First Pattern section: outcome definition before implementation
- [ ] T019 [L2] Write Socratic Discovery Exercise: Questions guiding students to derive prompt structure
- [ ] T020 [L2] Write manual practice exercise: Students write analysis specifications (no AI)
- [ ] T021 [L2] Write "Try With AI" section (self-validation, no tool usage per Stage 1)
- [ ] T022 [L2] Validate lesson cognitive load ≤7 concepts

**Checkpoint**: Stage 1 complete - Foundation established, ready for AI collaboration (Stage 2)

---

## Phase 3: Stage 2 - AI Collaboration (Lessons 3-5)

**Purpose**: Demonstrate Three Roles pattern (AI as Teacher, Student, Co-Worker) with bidirectional learning

**⚠️ CRITICAL**: ALL Stage 2 lessons MUST demonstrate Three Roles CoLearning (Constitution requirement)

### Lesson 3: The 4-Layer Context Model

**Goal**: Students understand context engineering through verified 4-layer framework

**Independent Test**: Student applies 4-layer context stack to sample codebase analysis (validated through guided exercise)

- [ ] T023 [L3] Create 03-four-layer-context-model.md in chapter directory
- [ ] T024 [L3] Write "What Should This Lesson Enable?" specification section
- [ ] T025 [L3] Write 4-Layer Context Stack section (grounded in RESEARCH-REPORT.md verified concepts)
- [ ] T026 [L3] Write Three Roles Demonstration section:
  - AI as Teacher: Shows context pattern student didn't know
  - Student as Teacher: Corrects AI's vague output
  - Co-Worker convergence: Iterate toward better analysis together
- [ ] T027 [L3] Write hands-on exercise using small sample codebase (10-15 files)
- [ ] T028 [L3] Write "Try With AI" section (ChatGPT web or Claude Code/Gemini CLI)
- [ ] T029 [L3] Validate Three Roles pattern explicitly demonstrated (NOT passive tool use)
- [ ] T030 [L3] Validate lesson cognitive load ≤10 concepts (B1 tier)

**Checkpoint**: Context engineering foundation established with bidirectional learning

### Lesson 4: Claude Code Tool Ecosystem

**Goal**: Students master Claude Code tools for codebase navigation (Read, Grep, Glob, Bash, Edit, Write)

**Independent Test**: Student selects correct tool for 5 distinct scenarios (validated through tool selection quiz)

- [ ] T031 [L4] Create 04-claude-code-tools.md in chapter directory
- [ ] T032 [L4] Write "What Should This Lesson Enable?" specification section
- [ ] T033 [L4] Write tool capability matrix (grounded in RESEARCH-REPORT.md 14 verified tools)
- [ ] T034 [L4] Write tool selection decision tree (when to use Read vs Grep vs Glob)
- [ ] T035 [L4] Write Three Roles Demonstration section:
  - AI as Teacher: Suggests tool orchestration pattern
  - Student as Teacher: Refines tool parameters for precision
  - Co-Worker convergence: Optimize tool chain together
- [ ] T036 [L4] Write hands-on exercise using medium sample codebase (20-30 files)
- [ ] T037 [L4] Write "Try With AI" section with Claude Code specific examples
- [ ] T038 [L4] Validate Three Roles pattern explicitly demonstrated
- [ ] T039 [L4] Validate lesson cognitive load ≤10 concepts

**Checkpoint**: Claude Code mastery established (OR students skip if using Gemini CLI)

### Lesson 5: Gemini CLI Commands and Custom Workflows

**Goal**: Students master Gemini CLI features (@filename, !command, custom TOML commands)

**Independent Test**: Student writes functional custom TOML command for specified workflow (validated through exercise)

- [ ] T040 [L5] Create 05-gemini-cli-workflows.md in chapter directory
- [ ] T041 [L5] Write "What Should This Lesson Enable?" specification section
- [ ] T042 [L5] Write Gemini CLI capability overview (grounded in RESEARCH-REPORT.md verified features)
- [ ] T043 [L5] Write @filename and !command syntax sections with examples
- [ ] T044 [L5] Write custom TOML command configuration section
- [ ] T045 [L5] Write Three Roles Demonstration section:
  - AI as Teacher: Shows TOML pattern student didn't know
  - Student as Teacher: Debugs command configuration with AI
  - Co-Worker convergence: Build complex workflow together
- [ ] T046 [L5] Write hands-on exercise: Create custom command for git history analysis
- [ ] T047 [L5] Write "Try With AI" section with Gemini CLI specific examples
- [ ] T048 [L5] Validate Three Roles pattern explicitly demonstrated
- [ ] T049 [L5] Validate lesson cognitive load ≤10 concepts

**Checkpoint**: Gemini CLI mastery established (OR students skip if using Claude Code) - Stage 2 complete

---

## Phase 4: Stage 3 - Intelligence Design (Lessons 6-7)

**Purpose**: Transform recurring patterns into reusable prompt skills (Persona + Questions + Principles)

**⚠️ CRITICAL**: These lessons CREATE intelligence objects that Lesson 8 capstone will COMPOSE

### Lesson 6: Creating Reusable Prompt Skills

**Goal**: Students design 3 reusable skills (codebase-navigation, architecture-analysis, security-audit)

**Independent Test**: Student creates functional prompt skill using Persona + Questions + Principles pattern (validated through skill execution)

- [ ] T050 [L6] Create 06-reusable-prompt-skills.md in chapter directory
- [ ] T051 [L6] Write "What Should This Lesson Enable?" specification section
- [ ] T052 [L6] Write Persona + Questions + Principles pattern explanation
- [ ] T053 [L6] Write Socratic Discovery section: Questions guiding skill design thinking
- [ ] T054 [L6] Create codebase-navigation skill in .claude/skills/ directory
  - Persona: "Think like engineer joining new project"
  - Questions: 5 analytical questions (purpose, entry points, modules, dependencies, data model)
  - Principles: Breadth-first exploration, data flow analysis, boundary identification
- [ ] T055 [L6] Create architecture-analysis skill in .claude/skills/ directory
  - Persona: "Think like solutions architect reviewing for enterprise"
  - Questions: 5 questions (pattern, scalability, tech stack, debt, risks)
  - Principles: Architecture emerges from constraints, tradeoffs over perfection
- [ ] T056 [L6] Create security-audit skill in .claude/skills/ directory
  - Persona: "Think like penetration tester"
  - Questions: 5 questions (attack surfaces, threat models, controls, likelihood×impact)
  - Principles: Likelihood × impact focus, defense in depth, assume breach
- [ ] T057 [L6] Write hands-on exercise: Students create 4th skill (performance-audit)
- [ ] T058 [L6] Write "Try With AI" section demonstrating skill composition
- [ ] T059 [L6] Validate skills are reusable (not technology-locked, work across codebases)

**Checkpoint**: Reusable intelligence objects created - Ready for composition in capstone

### Lesson 7: Project Memory Files (CLAUDE.md / GEMINI.md)

**Goal**: Students create persistent context files for AI tools

**Independent Test**: Student writes effective project memory file for sample codebase (validated through AI comprehension test)

- [ ] T060 [L7] Create 07-project-memory-files.md in chapter directory
- [ ] T061 [L7] Write "What Should This Lesson Enable?" specification section
- [ ] T062 [L7] Write CLAUDE.md structure and best practices (grounded in verified examples)
- [ ] T063 [L7] Write GEMINI.md structure and best practices
- [ ] T064 [L7] Write decision framework: What belongs in memory vs. prompts vs. skills
- [ ] T065 [L7] Write hands-on exercise: Create memory file for medium sample codebase
- [ ] T066 [L7] Write "Try With AI" section testing memory file effectiveness
- [ ] T067 [L7] Validate lesson cognitive load ≤7 concepts

**Checkpoint**: Stage 3 complete - Intelligence infrastructure ready for orchestration

---

## Phase 5: Stage 4 - Spec-Driven Integration (Lesson 8 Capstone)

**Purpose**: Orchestrate accumulated intelligence through specification-first capstone project

**⚠️ CRITICAL**: Capstone COMPOSES Lessons 1-7 work, does NOT introduce new isolated content

### Lesson 8: Capstone - Technical Assessment Report (2-Page Brief)

**Goal**: Students produce strategic technical assessment for vendor evaluation using accumulated skills

**Independent Test**: Student completes vendor evaluation producing 2-page brief rated "actionable" by engineering team (validates entire chapter learning)

- [ ] T068 [L8] Create 08-capstone-technical-assessment.md in chapter directory
- [ ] T069 [L8] Write "What Should This Lesson Enable?" specification section
- [ ] T070 [L8] Write capstone specification template:
  - Page 1: Architecture Overview (diagram, tech stack, key modules)
  - Page 2: Strategic Assessment (security findings, tech debt score, recommendation)
- [ ] T071 [L8] Write specification-first workflow section:
  1. Write specification (define success criteria BEFORE analysis)
  2. Select sample codebase (large: 50+ files)
  3. Compose skills (navigation → architecture → security from Lesson 6)
  4. Execute analysis (Claude Code OR Gemini CLI tools)
  5. Convergence loop (iterate, validate, refine)
  6. Produce 2-page brief
- [ ] T072 [L8] Write skill composition section (how to orchestrate 3 reusable skills)
- [ ] T073 [L8] Write validation checklist (cross-reference AI outputs with actual code)
- [ ] T074 [L8] Write "Try With AI" section (full capstone walkthrough with evaluation rubric)
- [ ] T075 [L8] Create capstone evaluation rubric:
  - Architecture accuracy (80% threshold per SC-001)
  - Security risk identification (3+ risks per SC-002)
  - Actionability rating (70% threshold per SC-010)
- [ ] T076 [L8] Validate capstone composes Lessons 1-7 (not standalone new content)

**Checkpoint**: Stage 4 complete - Chapter achieves strategic product intelligence capability

---

## Phase 6: Constitutional Validation & Polish

**Purpose**: Verify all constitutional requirements met, apply final quality checks

- [ ] T077 [P] Run constitutional compliance audit:
  - ✅ All 7 principles validated (Specification Primacy, Progressive Complexity, Factual Accuracy, Coherent Pedagogy, Intelligence Accumulation, Anti-Convergence, Minimal Content)
  - ✅ 4-Stage progression explicit (Stage 1: L1-2, Stage 2: L3-5, Stage 3: L6-7, Stage 4: L8)
  - ✅ B1 cognitive load limits (7-10 concepts max per lesson)
  - ✅ Three Roles demonstrated in Stage 2 (L3, L4, L5)
  - ✅ Teaching modality: Specification-first + Socratic dialogue (anti-convergence from Chapter 9)
- [ ] T078 [P] Run factual accuracy verification:
  - ✅ All Claude Code tools verified against RESEARCH-REPORT.md
  - ✅ All Gemini CLI commands verified against RESEARCH-REPORT.md
  - ✅ Unverified claims removed ("55% productive", "70% first try")
  - ✅ "8-element AIDD framework" rebranded or replaced
- [ ] T079 [P] Validate all success criteria mapped to lessons (11 SCs from spec.md)
- [ ] T080 [P] Validate evals-first pattern (learning objectives before content in each lesson)
- [ ] T081 Code example execution validation:
  - Run all Claude Code examples to verify they work
  - Run all Gemini CLI examples to verify they work
  - Attach execution logs to lessons
- [ ] T082 [P] Cross-reference prerequisite integration:
  - Chapter 7 (Bash) references validated in L4, L5
  - Chapter 8 (Git) references validated in L5
  - Chapter 9 (Markdown) references validated in L2, L6, L7
- [ ] T083 [P] Final README.md update with learning outcomes and chapter structure
- [ ] T084 Sample codebase documentation (README for each sample explaining purpose/complexity)

**Checkpoint**: Chapter validated against all constitutional requirements - Ready for publication

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Stage 1 (Phase 2)**: Depends on Setup completion - Lessons 1-2 MUST complete before Stage 2
- **Stage 2 (Phase 3)**: Depends on Stage 1 completion - Lessons 3-5 can proceed in parallel within stage
- **Stage 3 (Phase 4)**: Depends on Stage 2 completion - Lessons 6-7 CREATE intelligence for Stage 4
- **Stage 4 (Phase 5)**: Depends on Stage 3 completion - Lesson 8 COMPOSES Lessons 1-7
- **Validation (Phase 6)**: Depends on all lessons complete

### Lesson Dependencies

- **Lesson 1**: Depends on Setup - No other lesson dependencies
- **Lesson 2**: Depends on Lesson 1 (builds on mental models)
- **Lesson 3**: Depends on Lessons 1-2 (Stage 1 foundation required)
- **Lesson 4**: Depends on Lesson 3 (context model prerequisite)
- **Lesson 5**: Depends on Lesson 3 (context model prerequisite) - Can parallel with Lesson 4
- **Lesson 6**: Depends on Lessons 3-5 (Stage 2 patterns prerequisite)
- **Lesson 7**: Depends on Lessons 3-5 - Can parallel with Lesson 6
- **Lesson 8**: Depends on ALL Lessons 1-7 (capstone composition)

### Within Each Lesson

1. Specification section FIRST (WHAT before HOW)
2. Foundation/concept sections
3. Demonstrations/exercises
4. "Try With AI" section LAST
5. Validation checks (cognitive load, Three Roles if Stage 2)

### Parallel Opportunities

**Setup Phase**:
- T002-T008 can all run in parallel (different files/directories)

**Within Stages**:
- Lesson 4 and Lesson 5 can parallel (different platforms, independent)
- Lesson 6 and Lesson 7 can parallel (different skill types, independent)

**Validation Phase**:
- T077-T080, T082-T084 can all run in parallel (different validation types)

---

## Parallel Example: Stage 2 Lessons

```bash
# After Lessons 1-2 complete, launch Stage 2 lessons:
Task: "Create 04-claude-code-tools.md (Lesson 4)"
Task: "Create 05-gemini-cli-workflows.md (Lesson 5)"
# These can proceed independently - students choose platform
```

---

## Implementation Strategy

### MVP First (Stage 1 + Stage 2 Core)

1. Complete Phase 1: Setup
2. Complete Phase 2: Stage 1 (Lessons 1-2) - Foundation
3. Complete Phase 3: Stage 2 (Lessons 3-5) - Collaboration
4. **STOP and VALIDATE**: Test that students can perform basic codebase analysis
5. Proceed to Stage 3 + Stage 4 if foundation solid

### Incremental Delivery

1. Setup → Lessons 1-2 → Validate foundation (students understand mental models)
2. Add Lessons 3-5 → Validate collaboration (students demonstrate Three Roles)
3. Add Lessons 6-7 → Validate intelligence (students create reusable skills)
4. Add Lesson 8 → Validate composition (students orchestrate accumulated skills)
5. Each stage adds capability without invalidating previous work

### Parallel Lesson Writing Strategy

With multiple content writers:

1. Team completes Setup together
2. Writer A: Lessons 1-2 (Stage 1)
3. Writer B: Lessons 3, 6 (specification-first lessons)
4. Writer C: Lessons 4-5 (platform-specific lessons)
5. All writers collaborate on Lesson 8 (capstone requires full context)

---

## Notes

- **[P] tasks** = different files, no dependencies, safe to parallelize
- **[Lesson#] label** = maps task to specific lesson for traceability
- **Each lesson independently completable**: Follows constitution lesson structure
- **Specification-first throughout**: Every lesson shows WHAT before HOW
- **Three Roles mandatory in Stage 2**: Lessons 3-5 MUST demonstrate bidirectional learning
- **Capstone composes, not adds**: Lesson 8 orchestrates Lessons 1-7, no new isolated content
- **Evals-first pattern**: Success criteria appear before implementation in each lesson
- **Validation gates**: Stage completion checkpoints ensure quality before proceeding
- **Platform flexibility**: Students can complete using EITHER Claude Code OR Gemini CLI
