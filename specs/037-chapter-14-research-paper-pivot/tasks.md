# Tasks: Chapter 14 — Master Spec-Kit Plus (Research Paper Pivot)

**Input**: `specs/037-chapter-14-research-paper-pivot/`
**Prerequisites**: spec.md ✓, plan.md ✓
**Type**: Educational Content (Book Chapter Rewrite)

**Core Principle**: Each lesson teaches ONE Spec-Kit Plus command/concept. Research paper is the practice vehicle, NOT the learning objective.

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files)
- **[US1]**: Master Spec-Kit Plus Workflow (P1)
- **[US2]**: Create Reusable Intelligence (P1)
- **[US3]**: Experience Intelligence Acceleration (P2)

## Path Conventions

- **Lessons**: `apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/`
- **Summaries**: Same directory, `*.summary.md` files
- **Backup**: `specs/037-chapter-14-research-paper-pivot/backup/` (in feature branch)

---

## Phase 1: Setup

**Purpose**: Prepare for chapter rewrite with backup to feature branch specs folder

- [x] T001 Create backup directory at specs/037-chapter-14-research-paper-pivot/backup/
- [x] T002 Backup all existing lesson files to specs/037-chapter-14-research-paper-pivot/backup/
  - Copy all \*.md files from apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/
  - Include README.md, all lessons, all summaries, quiz
  - Preserves original content for reference/rollback
- [x] T003 [P] Document Spec-Kit Plus command syntax for reference during writing

**Checkpoint**: All original files backed up to feature branch specs folder

---

## Phase 2: Foundational (README Update)

**Purpose**: Update chapter introduction to reflect Spec-Kit Plus focus

- [x] T004 Rewrite README.md in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/README.md
  - Remove all video generation references
  - Remove MCP/Playwright mentions
  - Focus on Spec-Kit Plus workflow mastery
  - Keep research paper as simple practice vehicle
  - Update prerequisites (remove Python, Node.js requirements)

**Checkpoint**: README reflects Spec-Kit Plus focus, no video/MCP references

---

## Phase 3: User Story 1 - Master Spec-Kit Plus Workflow (P1) 🎯 MVP

**Goal**: Students learn each Spec-Kit Plus command through lessons 01-08

**Independent Test**: Student can explain and execute each command (`/sp.constitution`, `/sp.specify`, `/sp.clarify`, `/sp.plan`, `/sp.tasks`, `/sp.implement`)

### Lesson 01: Spec-Kit Plus Foundation

- [x] T005 [US1] Rewrite 01-spec-kit-plus-foundation.md in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/

  - Focus: What is Spec-Kit Plus, why use it
  - Teach: Horizontal Intelligence (ADRs, PHRs), Vertical Intelligence (Skills, Subagents)
  - Remove: All video generation context
  - Add: Preview of all commands they'll learn
  - End with: "Try With AI" exploring P+Q+P concept
  - Constraint: Max 7 concepts (A2-B1)

- [x] T006 [P] [US1] Generate 01-spec-kit-plus-foundation.summary.md using /summary-generator skill
  - Extract key concepts from rewritten lesson
  - Follow .summary.md format
  - Replace existing summary file

### Lesson 02: Installation and Setup

- [x] T007 [US1] Rewrite 02-installation-and-setup.md in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/

  - Focus: Configure Claude Code/Gemini for Spec-Kit Plus
  - Teach: Directory structure (`.specify/`, `.claude/`, `specs/`, `skills/`)
  - Remove: Python installation, Node.js, Playwright, video codec setup
  - Add: Create project folder for research paper practice
  - End with: "Try With AI" creating project structure
  - Constraint: No Python, no MCP, no npm

- [x] T008 [P] [US1] Generate 02-installation-and-setup.summary.md using /summary-generator skill

### Lesson 03: Constitution Phase

- [x] T009 [US1] Rewrite 03-constitution-phase.md in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/

  - Focus: `/sp.constitution` command - WHAT it does, HOW to use it
  - Teach: Constitution sections (principles, constraints), how constitution guides phases
  - Practice Vehicle: Create constitution for research paper (quality standards, audience)
  - Remove: Video quality standards, Gemini constraints
  - End with: "Try With AI" running `/sp.constitution`
  - Constraint: Command is the lesson, paper is just practice

- [x] T010 [P] [US1] Generate 03-constitution-phase.summary.md using /summary-generator skill

### Lesson 04: Specify Phase

- [x] T011 [US1] Rewrite 04-specify-phase.md in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/

  - Focus: `/sp.specify` command - WHAT it does, HOW to use it
  - Teach: Spec sections (Intent, Constraints, Success Evals, Non-Goals), SMART criteria
  - Practice Vehicle: Write specification for research paper
  - Remove: Video specification examples, Gemini prompts
  - End with: "Try With AI" running `/sp.specify`
  - Constraint: Command is the lesson, paper is just practice

- [x] T012 [P] [US1] Generate 04-specify-phase.summary.md using /summary-generator skill

### Lesson 05: Clarify Phase

- [x] T013 [US1] Rewrite 05-clarify-phase.md in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/

  - Focus: `/sp.clarify` command - WHAT it does, HOW to use it
  - Teach: Why clarification matters, identifying edge cases, iteration
  - Practice Vehicle: Refine paper specification
  - Remove: Video-specific edge cases, Playwright session handling
  - End with: "Try With AI" running `/sp.clarify`
  - Constraint: Command is the lesson, paper is just practice

- [x] T014 [P] [US1] Generate 05-clarify-phase.summary.md using /summary-generator skill

### Lesson 06: Plan Phase

- [x] T015 [US1] Rewrite 06-plan-phase.md in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/

  - Focus: `/sp.plan` command - WHAT it does, HOW to use it
  - Teach: Plan sections (approach, components, dependencies), HOW vs WHAT distinction
  - Practice Vehicle: Create plan for research paper sections
  - Remove: Playwright MCP architecture, video generation approach
  - End with: "Try With AI" running `/sp.plan`
  - Constraint: Command is the lesson, paper is just practice

- [x] T016 [P] [US1] Generate 06-plan-phase.summary.md using /summary-generator skill

### Lesson 07: Tasks Phase

- [x] T017 [US1] Rewrite 07-tasks-phase.md in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/

  - Focus: `/sp.tasks` command - WHAT it does, HOW to use it
  - Teach: Atomic tasks, acceptance criteria, checkpoint-driven execution
  - Practice Vehicle: Break paper into atomic section tasks
  - Remove: Video recording tasks, MCP task examples
  - End with: "Try With AI" running `/sp.tasks`
  - Constraint: Command is the lesson, paper is just practice

- [x] T018 [P] [US1] Generate 07-tasks-phase.summary.md using /summary-generator skill

### Lesson 08: Implement Phase

- [x] T019 [US1] Rewrite 08-implement-phase.md in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/

  - Focus: `/sp.implement` command - WHAT it does, HOW to use it
  - Teach: Iterative execution, validation against spec, checkpoints
  - Practice Vehicle: Implement first paper section
  - Remove: Video generation execution, Playwright automation
  - End with: "Try With AI" running `/sp.implement`
  - Constraint: Command is the lesson, paper is just practice

- [x] T020 [P] [US1] Generate 08-implement-phase.summary.md using /summary-generator skill

**Checkpoint**: Students can execute full Spec-Kit Plus workflow (US1 complete)

---

## Phase 4: User Story 2 - Create Reusable Intelligence (P1)

**Goal**: Students learn P+Q+P framework and create a skill

**Independent Test**: Student creates valid skill using Persona + Questions + Principles

### Lesson 09: Designing Reusable Intelligence

- [x] T021 [US2] Rewrite 09-designing-reusable-intelligence.md in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/

  - Focus: P+Q+P framework - Persona + Questions + Principles
  - Teach: What makes patterns reusable, skill file structure (`.claude/skills/`), skill vs subagent
  - Practice Vehicle: Create `section-writer` skill from paper writing experience
  - Remove: `generate-video` and `upload-youtube` skill examples
  - Add: Generic skill creation applicable to any domain
  - End with: "Try With AI" creating P+Q+P skill
  - Constraint: Skill creation is the lesson, not domain-specific

- [x] T022 [P] [US2] Generate 09-designing-reusable-intelligence.summary.md using /summary-generator skill

### Lesson 10: Brownfield Adoption

- [x] T023 [US2] Rewrite 10-brownfield-adoption.md in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/

  - Focus: Adding Spec-Kit Plus to existing projects
  - Teach: Greenfield vs brownfield, starting with constitution, incremental adoption
  - Remove: Video project brownfield examples
  - Add: Generic brownfield adoption strategies
  - End with: "Try With AI" planning adoption for hypothetical project
  - Constraint: Conceptual lesson, no specific domain

- [x] T024 [P] [US2] Generate 10-brownfield-adoption.summary.md using /summary-generator skill

**Checkpoint**: Students can create reusable skills (US2 complete)

---

## Phase 5: User Story 3 - Experience Intelligence Acceleration (P2)

**Goal**: Students experience that skill reuse accelerates work

**Independent Test**: Student uses skill for second task and articulates acceleration

### Lesson 11: Capstone

- [x] T025 [US3] Rewrite 11-youtube-capstone.md → 11-capstone.md in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/

  - Rename file: Remove "youtube" from filename
  - Focus: Intelligence acceleration through skill reuse
  - Teach: Skill invocation in `/sp.implement`, measuring acceleration, compounding
  - Practice Vehicle: Write second paper section using `section-writer` skill
  - Remove: All YouTube upload content, video publishing
  - Add: Comparison of effort (Lesson 08 vs Lesson 11)
  - End with: "Try With AI" reflecting on acceleration
  - Constraint: Acceleration proof is the lesson

- [x] T026 [P] [US3] Generate 11-capstone.summary.md using /summary-generator skill

  - New filename (not youtube-capstone.summary.md)

- [x] T027 [US3] Delete old 11-youtube-capstone.summary.md file

**Checkpoint**: Students experience intelligence acceleration (US3 complete)

---

## Phase 6: Assessment

**Goal**: Validate learning with quiz

- [x] T028 Rewrite 12_chapter_14_quiz.md in apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/
  - Focus: Spec-Kit Plus command mastery
  - Questions on: Command purpose, artifact types, P+Q+P components
  - Remove: Video generation questions, YouTube questions
  - Add: Command recognition, workflow ordering, acceleration reasoning
  - Constraint: Test Spec-Kit Plus knowledge, not domain knowledge

---

## Phase 7: Polish & Validation

**Purpose**: Final quality checks

- [x] T029 Validate all lessons end with "Try With AI" section only
- [x] T030 Validate no lessons contain video/MCP/Playwright references
- [x] T031 Validate each lesson focuses on Spec-Kit Plus command (not paper writing)
- [x] T032 Validate cognitive load ≤7 concepts per lesson (A2-B1)
- [x] T033 Run grep validation for forbidden terms: "video", "youtube", "playwright", "mcp", "gemini.google.com"
- [x] T034 Update chapter-index.md if title changed
- [x] T035 Verify backup exists in specs/037-chapter-14-research-paper-pivot/backup/

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup/Backup) → Phase 2 (README) → Phase 3 (US1) → Phase 4 (US2) → Phase 5 (US3) → Phase 6 (Quiz) → Phase 7 (Validation)
```

### Lesson + Summary Pattern

For each lesson:

1. Rewrite lesson file (T00X)
2. Generate new .summary.md immediately after (T00X+1)

This ensures summaries reflect current lesson content.

### Parallel Opportunities

**Within each lesson pair**:

- Summary generation can run in parallel with NEXT lesson rewrite
- Example: T006 (L01 summary) can run parallel with T007 (L02 lesson)

**Phase 7 (Validation)**:

- T029-T035 can mostly run in parallel

### Task Dependencies

| Task      | Depends On | Reason                                |
| --------- | ---------- | ------------------------------------- |
| T004+     | T001-T002  | Backup must complete first            |
| T006      | T005       | Summary depends on lesson rewrite     |
| T008      | T007       | Summary depends on lesson rewrite     |
| ...       | ...        | Same pattern for all lessons          |
| T029-T035 | T028       | Validation after all content complete |

---

## Implementation Strategy

### MVP First (Phase 1-3 Only)

1. Complete Setup with backup (Phase 1)
2. Update README (Phase 2)
3. Rewrite Lessons 01-08 with summaries (Phase 3 - US1)
4. **VALIDATE**: Run grep for forbidden terms
5. Core workflow complete - students can learn Spec-Kit Plus

### Full Delivery

1. MVP (Phases 1-3) ✓
2. Add skill creation lessons + summaries (Phase 4 - US2)
3. Add capstone + summary (Phase 5 - US3)
4. Update quiz (Phase 6)
5. Final validation (Phase 7)

---

## Summary

| Metric                  | Count |
| ----------------------- | ----- |
| **Total Tasks**         | 35    |
| **Lesson Rewrites**     | 12    |
| **Summary Generations** | 11    |
| **Setup/Backup Tasks**  | 3     |
| **Validation Tasks**    | 7     |
| **Quiz Task**           | 1     |
| **Cleanup Task**        | 1     |

### Backup Location

All original files preserved at:

```
specs/037-chapter-14-research-paper-pivot/backup/
├── README.md
├── 01-spec-kit-plus-foundation.md
├── 01-spec-kit-plus-foundation.summary.md
├── 02-installation-and-setup.md
├── ... (all original files)
└── 12_chapter_14_quiz.md
```

### Lesson Author Policy

Within this chapter, each lesson must end with a single final section titled **"Try With AI"** (no "Key Takeaways" or "What's Next"). Students use their AI companion (Claude Code or Gemini CLI) to practice the Spec-Kit Plus command taught in that lesson.

---

## Validation Commands

```bash
# Check for forbidden video/MCP references:
grep -ri "video\|youtube\|playwright\|mcp\|gemini.google.com" apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/

# Expected: Zero matches after rewrite

# Check lesson endings:
grep -l "Try With AI" apps/learn-app/docs/04-SDD-RI-Fundamentals/14-spec-kit-plus-hands-on/*.md

# Expected: All lesson files (except README and quiz)

# Verify backup exists:
ls specs/037-chapter-14-research-paper-pivot/backup/

# Expected: All original files present
```
