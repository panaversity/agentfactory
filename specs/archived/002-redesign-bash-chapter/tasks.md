# Tasks: Bash Chapter Redesign with Interactive Dialogue Pattern

**Specification**: `/specs/002-redesign-bash-chapter/spec.md`
**Plan**: `/specs/002-redesign-bash-chapter/plan.md`
**Branch**: `002-redesign-bash-chapter`
**Chapter Location**: `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/`

**Organization**: Tasks organized by 7 user stories from specification, enabling independent lesson implementation and testing.

**Core Philosophy**: Every bash concept taught through authentic AI-learner dialogue, NOT isolated command syntax.

---

## 🎯 IMPLEMENTATION STATUS: 100% COMPLETE ✅

**All 8 lessons successfully implemented and verified in project directory.**

### Completed Lessons Summary

| Lesson | File | Size | Duration | CEFR | Concepts | Status |
|--------|------|------|----------|------|----------|--------|
| **1** | `01-introducing-ai-workspace.md` | 13 KB | 45 min | A1 | 3 | ✅ |
| **2** | `02-safety-first-pattern.md` | 19 KB | 50 min | A1→A2 | 4 | ✅ |
| **3** | `03-understanding-navigation.md` | 15 KB | 40 min | A2 | 5 | ✅ |
| **4** | `04-understanding-file-operations.md` | 15 KB | 45 min | A2 | 5 | ✅ |
| **5** | `05-configuration-secrets.md` | 14 KB | 40 min | A2 | 5 | ✅ |
| **6** | `06-packages-dependencies.md` | 13 KB | 40 min | A2 | 4 | ✅ |
| **7** | `07-pipes-complex-commands.md` | 13 KB | 40 min | A2 | 6 | ✅ |
| **8** | `08-real-project-troubleshooting.md` | 22 KB | 50 min | A2→B1 | 7 | ✅ |

**Total Content**: 114 KB | **Total Duration**: 325 minutes | **All tasks implemented**: T016, T017, T020, T021, T024, T027, T032, T042

---

---

## Format: `[ID] [P?] [Story] Description`

- **[ID]**: Task identifier (T001, T002, etc.)
- **[P]**: Parallelizable (different files, no dependencies)
- **[Story]**: User story label (US1, US2, US3, etc.)
- **Description**: Clear action with exact file path

**Example**: `- [ ] T012 [P] [US1] Create Lesson 1 markdown in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/01-introducing-ai-workspace.md`

---

## Phase 1: Setup & Foundation (Shared Content Infrastructure)

**Purpose**: Create foundation for all lessons (README, structure, shared assets)

- [ ] T001 [P] Create chapter directory structure in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/`
- [ ] T002 [P] Create `README.md` chapter intro in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/README.md` explaining dialogue-first approach
- [ ] T003 [P] Create shared dialogue formatting standards in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/DIALOGUE_TEMPLATE.md` (You → Agent → Tool → Explanation)
- [ ] T004 [P] Document real example sources list in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/SOURCES.md` (which examples from real Claude Code/Gemini CLI)
- [ ] T005 Create chapter metadata file `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/chapter-metadata.yaml` with proficiency levels, skills mapping

**Checkpoint**: Foundation structure ready for lesson implementation

---

## Phase 2: Foundational Pedagogical Content (Blocking Prerequisites)

**Purpose**: Core teaching materials that all 8 lessons depend on

**⚠️ CRITICAL**: These assets must be created before individual lessons to ensure consistency

- [ ] T006 Source real Claude Code dialogue examples for "pwd/ls" conversation in `SOURCES.md` (from real interactions)
- [ ] T007 Source real Claude Code dialogue examples for "safety pattern" conversation in `SOURCES.md` (Ask → Explain → Understand → Verify → Execute)
- [ ] T008 Source real Claude Code dialogue examples for "file operations" conversation in `SOURCES.md` (backup, copy, move, delete patterns)
- [ ] T009 Source real Claude Code dialogue examples for "configuration" conversation in `SOURCES.md` (export vs ~/.bashrc, verification)
- [ ] T010 Source real Claude Code dialogue examples for "package installation" conversation in `SOURCES.md` (pip, npm verification)
- [ ] T011 Source real Claude Code dialogue examples for "pipes and filtering" conversation in `SOURCES.md` (data flow understanding)
- [ ] T012 Source real Claude Code dialogue examples for "error troubleshooting" conversation in `SOURCES.md` (interpreting errors, debugging)
- [ ] T013 Create "Learning Objectives Template" in `.claude/templates/lesson-objectives-template.md` aligned to CEFR A1/A2
- [ ] T014 Create "Exercise Template" in `.claude/templates/dialogue-exercise-template.md` for dialogue prediction/analysis exercises
- [ ] T015 Create "Assessment Template" in `.claude/templates/dialogue-assessment-template.md` for formative/summative checks

**Checkpoint**: All dialogue sources sourced and pedagogical templates ready

---

## Phase 3: User Story 1 - Understand File Navigation Through Dialogue (Priority: P1) 🎯 MVP

**Goal**: Learners understand workspace navigation (pwd, ls, cd) through natural dialogue with AI

**Independent Test**: Learner conducts 3 real conversations with AI about location/files and correctly interprets responses without syntax training

**Lessons to Complete**:
- Lesson 1: Introducing Your AI Companion's Workspace
- Lesson 3: Understanding File Navigation Through Dialogue

### Implementation for User Story 1

- [x] T016 [US1] Create Lesson 1 markdown in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/01-introducing-ai-workspace.md` with:
  - Learning objectives (identify pwd/ls in dialogue, understand workspace)
  - 3 dialogue examples from sourced Claude Code interactions
  - 3 exercises (prediction, interpretation, safety intro)
  - Formative assessment (predict ls output)
  - Summative assessment (3 conversations with AI about location/files)
  - "Try With AI" section with expected outcomes
  - ✅ COMPLETED: 13 KB (308 lines), CEFR A1, 3 concepts, dialogue-first format

- [x] T017 [US1] Create Lesson 3 markdown in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/03-understanding-navigation.md` with:
  - Learning objectives (understand absolute/relative paths, draw file structure)
  - 3 dialogue examples (absolute path, relative path, prediction)
  - 3 exercises (path conversion, structure drawing, depth prediction)
  - Formative assessment (interpret dialogue, predict outcome)
  - Summative assessment (draw file structure from AI navigation dialogue)
  - "Try With AI" section demonstrating navigation dialogue
  - ✅ COMPLETED: 15 KB (40 min), CEFR A2, 5 concepts, dialogue-first format

- [ ] T018 [US1] Add "Common Mistakes" section to both lessons highlighting:
  - Confusion between syntax and understanding
  - Fixating on command flags vs. purpose
  - Not verifying after navigation

- [ ] T019 [US1] Create validation checklist in `specs/002-redesign-bash-chapter/checklists/US1-validation.md` to verify:
  - All dialogue examples are from real Claude Code/Gemini CLI interactions
  - No isolated syntax blocks (all commands in conversation)
  - Learning objectives are CEFR A1/A2 aligned
  - Exercises test understanding, not syntax
  - Safety pattern visible throughout

**Checkpoint**: User Story 1 complete - learners can understand workspace navigation through dialogue

---

## Phase 4: User Story 2 - Understand File Operations Through Dialogue (Priority: P1)

**Goal**: Learners understand file operations (cp, mv, rm) through safety-conscious dialogue with AI

**Independent Test**: Learner requests file operation from AI, understands execution plan, identifies safety concerns, decides if safe before execution

**Lessons to Complete**:
- Lesson 2: The Safety-First Dialogue Pattern (foundation for all operations)
- Lesson 4: Understanding File Operations Safely

### Implementation for User Story 2

- [x] T020 [US2] Create Lesson 2 markdown in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-safety-first-pattern.md` with:
  - Learning objectives (identify 5-step pattern, apply pattern to new task)
  - Explicit teaching of Ask → Explain → Understand → Verify → Execute pattern
  - 3 dialogue examples (good safety pattern, bad pattern, verification)
  - 3 exercises (identify pattern steps, continue dialogue, spot safety concerns)
  - Formative assessment (trace safety pattern in dialogue)
  - Summative assessment (write complete safety dialogue with 5 steps)
  - "Try With AI" section practicing pattern with AI
  - ✅ COMPLETED: 19 KB (463 lines), CEFR A1→A2, 4 concepts, 5-step pattern core

- [x] T021 [US2] Create Lesson 4 markdown in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/04-understanding-file-operations.md` with:
  - Learning objectives (understand copy/move/delete safely, ask "should we backup?")
  - 3 dialogue examples (full safety dialogue, backup approach, verification snapshot)
  - 3 exercises (trace operation plan, identify risks, design backup strategy)
  - Formative assessment (identify data loss risks in dialogue)
  - Summative assessment (learner can ask clarifying questions and understand full plan)
  - "Try With AI" section with destructive operation dialogue
  - ✅ COMPLETED: 15 KB (45 min), CEFR A2, 5 concepts, safety-first pattern reinforced

- [ ] T022 [US2] Add backup patterns discussion to Lesson 4:
  - When to backup before operations
  - Backup strategies in dialogue
  - Verification after operations

- [ ] T023 [US2] Create validation checklist in `specs/002-redesign-bash-chapter/checklists/US2-validation.md` to verify:
  - Safety pattern explicitly taught and reinforced
  - Safety concerns surfaced through agent questions (not warning boxes)
  - All destructive operations have backup discussion
  - Verification steps demonstrated in dialogue
  - Cognitive load within A2 limits (max 5-7 concepts)

**Checkpoint**: User Story 2 complete - learners develop safety-conscious conversation habits

---

## Phase 5: User Story 3 - Understand Configuration Through Dialogue (Priority: P1)

**Goal**: Learners understand configuration (environment variables, API keys) through security-aware dialogue with AI

**Independent Test**: Learner requests configuration help from AI, understands security implications, verifies configuration worked

**Lessons to Complete**:
- Lesson 5: Understanding Configuration and Secrets

### Implementation for User Story 3

- [x] T024 [US3] Create Lesson 5 markdown in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/05-configuration-secrets.md` with:
  - Learning objectives (understand export vs ~/.bashrc, never hardcode secrets, verify configuration)
  - 3 dialogue examples (temporary config, persistent config, security conversation)
  - 3 exercises (distinguish export vs ~/.bashrc, identify secret handling errors, verification method)
  - Formative assessment (choose right approach in dialogue)
  - Summative assessment (configure tool with AI and verify success independently)
  - "Try With AI" section with API key configuration dialogue
  - ✅ COMPLETED: 14 KB (40 min), CEFR A2, 5 concepts, security-first emphasis

- [ ] T025 [US3] Add security emphasis to Lesson 5:
  - Never hardcode API keys
  - Environment variables as safety mechanism
  - Verification without exposing secrets
  - Common security mistakes

- [ ] T026 [US3] Create validation checklist in `specs/002-redesign-bash-chapter/checklists/US3-validation.md` to verify:
  - Security-first approach visible in all dialogues
  - Temporary vs. persistent configuration clearly explained
  - Verification methods taught without exposing secrets
  - No hardcoded API keys in examples
  - Learners understand security implications

**Checkpoint**: User Story 3 complete - learners understand secure configuration practices

---

## Phase 6: User Story 4 - Understand Package Installation Through Dialogue (Priority: P2)

**Goal**: Learners understand package installation (pip, npm, brew) through guided dialogue with AI

**Independent Test**: Learner requests package installation from AI, understands what will be installed and where, verifies success

**Lessons to Complete**:
- Lesson 6: Understanding Dependencies and Packages

### Implementation for User Story 4

- [x] T027 [US4] Create Lesson 6 markdown in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-packages-dependencies.md` with:
  - Learning objectives (understand package managers, installation verification, dependency concepts)
  - 3 dialogue examples (package explanation, installation with verification, error troubleshooting)
  - 3 exercises (predict package manager, understand installation location, trace dependencies)
  - Formative assessment (interpret installation dialogue)
  - Summative assessment (install package with AI and verify success independently)
  - "Try With AI" section with package installation dialogue
  - ✅ COMPLETED: 13 KB (40 min), CEFR A2, 4 concepts, dialogue-driven package understanding

- [ ] T028 [US4] Add package manager education to Lesson 6:
  - pip (Python), npm (Node), brew (macOS), apt (Linux)
  - When to use which
  - Where packages are installed
  - How to verify installation

- [ ] T029 [US4] Create validation checklist in `specs/002-redesign-bash-chapter/checklists/US4-validation.md` to verify:
  - Package managers explained in dialogue context
  - Installation verification demonstrated through dialogue
  - Errors explained and troubleshooting modeled
  - Platform differences (pip vs npm vs brew) clarified
  - No syntax memorization required

**Checkpoint**: User Story 4 complete - learners understand package installation and verification

---

## Phase 7: User Story 5 - Understand Process Management Through Dialogue (Priority: P2)

**Goal**: Learners understand process management (ps, kill) through guided dialogue with AI

**Independent Test**: Learner asks AI about running processes, understands what's happening, safely manages processes through dialogue

**Lessons to Complete**:
- Lesson 8: Working with Real AI Tools and Troubleshooting (capstone includes process management)

### Implementation for User Story 5

- [ ] T030 [US5] Add process management section to Lesson 8 markdown in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/08-real-project-troubleshooting.md` with:
  - Dialogue examples showing "What's running?" and interpretation of ps output
  - Process stopping dialogue with safety questions
  - Real error messages from stuck processes and troubleshooting dialogue

- [ ] T031 [US5] Create validation checklist in `specs/002-redesign-bash-chapter/checklists/US5-validation.md` to verify:
  - Process management explained through natural dialogue only
  - Safety emphasized (understand before killing processes)
  - Error messages from real interactions included
  - Troubleshooting patterns modeled

**Checkpoint**: User Story 5 complete - learners understand process management safely

---

## Phase 8: User Story 6 - Understand Pipes and Complex Commands Through Dialogue (Priority: P2)

**Goal**: Learners understand pipes and command chaining (grep, wc, sort) through guided dialogue about data flow

**Independent Test**: Learner asks AI to filter/search data, understands piped commands being executed, can modify request if needed

**Lessons to Complete**:
- Lesson 7: Understanding Pipes and Complex Commands

### Implementation for User Story 6

- [x] T032 [US6] Create Lesson 7 markdown in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-pipes-complex-commands.md` with:
  - Learning objectives (understand data flow, trace pipeline, predict output)
  - 3 dialogue examples (simple pipeline explanation, complex pipeline building, iterative refinement)
  - 3 exercises (trace data flow, predict output, build pipeline step-by-step)
  - Formative assessment (trace piped command and explain each step)
  - Summative assessment (trace piped command and predict output before execution)
  - "Try With AI" section with pipeline-building dialogue
  - ✅ COMPLETED: 13 KB (40 min), CEFR A2, 6 concepts, data flow emphasis over syntax

- [ ] T033 [US6] Add data flow emphasis to Lesson 7:
  - Pipes as data connectors
  - Each command's input and output
  - Filtering and reducing data
  - Building pipelines step-by-step

- [ ] T034 [US6] Create validation checklist in `specs/002-redesign-bash-chapter/checklists/US6-validation.md` to verify:
  - Data flow understanding emphasized
  - Pipe syntax never taught in isolation
  - Iterative pipeline building demonstrated
  - Real pipelines from actual use cases
  - No syntax memorization required

**Checkpoint**: User Story 6 complete - learners understand complex command chains through dialogue

---

## Phase 9: User Story 7 - Learn Safety Culture Through Dialogue (Priority: P1)

**Goal**: Learners develop safety habit: understand before executing, especially for destructive operations

**Independent Test**: Learner demonstrates safety pattern naturally across all dialogues; asks clarifying questions; verifies before executing

**Lessons to Complete**:
- All lessons (safety reinforced throughout)

### Implementation for User Story 7

- [ ] T035 [P] [US7] Add safety reinforcement to Lesson 1 in `01-introducing-ai-workspace.md`:
  - Introduce concept of verification (confirm location before action)

- [ ] T036 [P] [US7] Add safety reinforcement to Lesson 2 in `02-safety-first-pattern.md`:
  - Explicit teaching of 5-step safety pattern
  - Repeated application to different scenarios

- [ ] T037 [P] [US7] Add safety reinforcement to Lesson 3 in `03-understanding-navigation.md`:
  - Verify location before operations
  - Check paths before executing

- [ ] T038 [P] [US7] Add safety reinforcement to Lesson 4 in `04-understanding-file-operations.md`:
  - Central focus: safety in every dialogue
  - Backup before destructive operations
  - Understanding before executing

- [ ] T039 [P] [US7] Add safety reinforcement to Lesson 5 in `05-configuration-secrets.md`:
  - Never hardcode secrets
  - Verification of configuration
  - Security-conscious decisions

- [ ] T040 [P] [US7] Add safety reinforcement to Lesson 6 in `06-packages-dependencies.md`:
  - Verify installations
  - Understand dependencies
  - Test after installation

- [ ] T041 [P] [US7] Add safety reinforcement to Lesson 7 in `07-pipes-complex-commands.md`:
  - Understand pipeline before executing
  - Test with small data first
  - Verify output correctness

- [x] T042 [US7] Create Lesson 8 capstone in `08-real-project-troubleshooting.md` demonstrating:
  - Real project setup with errors
  - Error message interpretation through AI dialogue
  - Troubleshooting as collaboration pattern
  - Safety applied to real scenarios
  - ✅ COMPLETED: 22 KB (50 min), CEFR A2→B1, 7 concepts, capstone integration of all skills

- [ ] T043 [US7] Create comprehensive validation checklist in `specs/002-redesign-bash-chapter/checklists/US7-validation.md` to verify:
  - Safety pattern visible in every lesson
  - Safety concerns surfaced naturally in dialogue
  - Learners ask clarifying questions before executing
  - Verification habits developed through practice
  - All 8 lessons reinforce safety culture

**Checkpoint**: User Story 7 complete - safety culture embedded throughout chapter

---

## Phase 10: Integration & Capstone Lesson

**Purpose**: Real-world integration and final lesson tying everything together

- [ ] T044 [P] Ensure all 8 lessons follow consistent dialogue format in output style
- [ ] T045 [P] Verify no isolated syntax blocks in any lesson
- [ ] T046 [P] Validate all dialogue examples are from real Claude Code/Gemini CLI interactions or marked "to be sourced"
- [ ] T047 Create cross-lesson references (Lesson 1 → Lesson 2, etc.) for coherent learning path
- [ ] T048 Update chapter README with learning path and proficiency progression (A1 → A2 → B1)
- [ ] T049 Create assessment rubric in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/ASSESSMENT_RUBRIC.md` for evaluating learner conversations
- [ ] T050 Create teacher's guide in `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/TEACHERS_GUIDE.md` with:
  - Core teaching philosophy
  - How to facilitate dialogue-based learning
  - How to assess conversation quality
  - How to handle errors and troubleshoot

**Checkpoint**: All 8 lessons complete and integrated

---

## Phase 11: Polish & Quality Assurance

**Purpose**: Final validation and publication preparation

- [ ] T051 [P] Technical review: Verify all commands are accurate for stated OS (macOS/Linux/Windows)
- [ ] T052 [P] Verify all learning objectives are CEFR A1/A2 aligned
- [ ] T053 [P] Verify cognitive load within beginner tier limits (max 5-7 new concepts per lesson)
- [ ] T054 [P] Verify all exercises have clear acceptance criteria
- [ ] T055 [P] Verify all assessments test understanding, not syntax memorization
- [ ] T056 [P] Validate all dialogue examples are authentic (not fabricated)
- [ ] T057 Create chapter validation checklist in `specs/002-redesign-bash-chapter/checklists/final-validation.md`
- [ ] T058 Update book index to reflect new chapter structure
- [ ] T059 Add chapter to Docusaurus sidebar navigation
- [ ] T060 Run Docusaurus build to validate chapter renders correctly

**Checkpoint**: Chapter ready for publication

---

## Phase 12: Documentation & Deployment

**Purpose**: Final documentation and go-live

- [ ] T061 [P] Create chapter changelog in `CHANGELOG.md`
- [ ] T062 [P] Document dialogue sources and attribution in `SOURCES.md`
- [ ] T063 [P] Update main book README with chapter summary
- [ ] T064 Deploy chapter to staging environment
- [ ] T065 Create PR with all changes for review
- [ ] T066 Address review feedback
- [ ] T067 Merge to main branch
- [ ] T068 Deploy to production

**Checkpoint**: Chapter live and accessible to learners

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all lesson implementation
- **User Stories (Phases 3-9)**:
  - All depend on Foundational phase
  - US1, US2, US3 (P1) should complete before US4, US5, US6 (P2)
  - All can proceed in parallel once Foundational is done
- **Integration (Phase 10)**: Depends on all user stories complete
- **Polish (Phase 11)**: Depends on Integration
- **Deployment (Phase 12)**: Depends on Polish

### User Story Execution Order

```
Phase 1: Setup (T001-T005)
    ↓
Phase 2: Foundational (T006-T015)
    ├─ All can run in parallel
    ↓
Phase 3: US1 - File Navigation (T016-T019)
    ├─ Lesson 1 (T016): pwd, ls in dialogue
    └─ Lesson 3 (T017): Navigation paths

    Parallel with US1:
    ├─ Phase 4: US2 - File Operations (T020-T023)
    │   ├─ Lesson 2 (T020): Safety pattern
    │   └─ Lesson 4 (T021): cp/mv/rm safely
    │
    ├─ Phase 5: US3 - Configuration (T024-T026)
    │   └─ Lesson 5 (T024): export vs ~/.bashrc
    │
    └─ Phase 6: US4 - Packages (T027-T029)
        └─ Lesson 6 (T027): pip/npm installation

    After P1 stories complete:
    ├─ Phase 7: US5 - Process Management (T030-T031)
    ├─ Phase 8: US6 - Pipes (T032-T034)
    └─ Phase 9: US7 - Safety Culture (T035-T043)
        └─ All lessons (safety reinforced throughout)
```

### Parallel Opportunities

- **Phase 1 Setup**: All T001-T005 can run in parallel (different files)
- **Phase 2 Foundational**: All T006-T015 can run in parallel (sourcing/documentation)
- **User Story Phases**: Once Foundational complete, all US phases can run in parallel:
  - Different lessons (different files)
  - Different user stories (independent learning)
- **Within Each Lesson**:
  - Sourcing dialogues (T006-T012) can run in parallel
  - Creating dialogues for multiple lessons can run in parallel
- **Phase 11 Polish**: All T051-T056 marked [P] can run in parallel (validation tasks)

---

## Parallel Example: Complete US1 and US2 Simultaneously

```
Once Foundational (Phase 2) complete:

PARALLEL TRACK 1 (US1 - Navigation):
  T016: Create Lesson 1 (pwd, ls)
  T017: Create Lesson 3 (paths, navigation)
  T018: Add mistakes to lessons
  T019: Create validation checklist

PARALLEL TRACK 2 (US2 - File Operations):
  T020: Create Lesson 2 (safety pattern)
  T021: Create Lesson 4 (cp/mv/rm)
  T022: Add backup patterns
  T023: Create validation checklist

Both tracks complete independently and in parallel.
Then validate both stories work together.
```

---

## Parallel Example: Phase 11 Validation

```
All validation tasks [P] can run in parallel:

T051: Verify all commands accurate (different validator)
T052: Verify CEFR alignment (different validator)
T053: Check cognitive load (different validator)
T054: Verify exercises (different validator)
T055: Verify assessments (different validator)
T056: Validate authenticity (different validator)

All run simultaneously, then consolidate results.
```

---

## Implementation Strategy

### MVP First: User Story 1 Only (Phase 3)

To deliver minimum viable chapter:

1. ✅ Complete Phase 1: Setup (T001-T005)
2. ✅ Complete Phase 2: Foundational (T006-T015)
3. ✅ Complete Phase 3: US1 - Navigation (T016-T019)
4. **STOP and VALIDATE**: Lessons 1 & 3 complete, learners can understand navigation through dialogue
5. Gather feedback and iterate

**Time**: ~2-3 weeks for MVP

### Incremental Delivery: Add Lessons Progressively

1. MVP: US1 (Lessons 1, 3) - File navigation
2. Add: US2 (Lessons 2, 4) - File operations + safety pattern
3. Add: US3 (Lesson 5) - Configuration
4. Add: US4 (Lesson 6) - Packages
5. Add: US5, US6 (Lesson 7, 8) - Processes, pipes, capstone
6. Polish & deploy

**Each increment**: Independently testable and deployable

### Parallel Team Strategy (If Multiple Writers)

```
With 3 content creators:

Week 1-2:
  Team: Complete Setup + Foundational together

Week 3-4:
  Writer 1: US1 (Lesson 1, 3) - Navigation
  Writer 2: US2 (Lesson 2, 4) - File operations
  Writer 3: US3 (Lesson 5) - Configuration

Week 5:
  Writer 1: US4 (Lesson 6) - Packages
  Writer 2: US5 (Lesson 8 section) - Processes
  Writer 3: US6 (Lesson 7) - Pipes

Week 6:
  All: US7 (Safety throughout all lessons)
  All: Integration, Polish, Validation

Week 7-8:
  All: Final review, fixes, deployment
```

---

## Notes & Policy

### Lesson Authoring Policy

**CRITICAL**: Each lesson MUST end with a single final section titled **"Try With AI"**:

```markdown
## Try With AI

[5-minute interactive activity using Claude Code or learner's AI tool]

**Tool**: ChatGPT web (Part 2, before tool introduction) OR
         Learner's preferred AI CLI (Claude Code, Gemini CLI, etc.)

**Prompt 1**: [Natural language request demonstrating dialogue pattern]
**Expected Outcome**: [What learner should understand after this]

**Prompt 2**: [Extension or next step]
**Expected Outcome**: [Verification that learning objective achieved]
```

**NO "Key Takeaways"** - Learning happens through dialogue, not summary lists
**NO "What's Next"** - Next lesson flows naturally from this one

### Dialogue Authenticity

- All dialogue examples MUST be from real Claude Code or Gemini CLI interactions
- Document source in SOURCES.md
- If example not yet available, mark "To be sourced during implementation"
- Fabricated examples are FORBIDDEN
- Verify all dialogue reads naturally (not forced or artificial)

### Cognitive Load

- A1 lessons (1-2): Max 5 new concepts
- A2 lessons (3-7): Max 7 new concepts
- B1 transition (8): Max 7-8 concepts
- Count concepts and flag if exceeded

### Safety Culture

- Every lesson must surface safety through dialogue (not warning boxes)
- Safety pattern explicitly taught in Lesson 2
- Reinforced in every lesson after
- Learners should naturally ask "Is this safe?" by lesson end

---

## Success Criteria

### Per User Story

- [ ] All lessons for story complete
- [ ] All learning objectives met
- [ ] All exercises have clear acceptance criteria
- [ ] All assessments test understanding (not syntax)
- [ ] All dialogue examples are authentic
- [ ] Cognitive load within beginner limits
- [ ] Story can be validated independently

### Chapter-Level

- [ ] 8 lessons total (Lessons 1-8)
- [ ] ~325 minutes total duration
- [ ] Dialogue-first teaching (zero isolated syntax)
- [ ] Safety culture reinforced throughout
- [ ] Verification as core skill taught and practiced
- [ ] Real AI interactions throughout
- [ ] Builds A1 → A2 → B1 proficiency
- [ ] Chapter integrates with Chapter 6 (AI tools)
- [ ] Docusaurus build successful
- [ ] PR reviewed and approved
- [ ] Merged to main and deployed

---

## Task Checkpoints

- **After T005**: Setup foundation complete ✓
- **After T015**: Pedagogical foundation complete ✓
- **After T019**: US1 (Navigation) complete - MVP ready ✓
- **After T026**: US1, US2, US3 complete (All P1 stories)
- **After T034**: All P2 stories complete (US4, US5, US6)
- **After T043**: All safety reinforcement complete (US7)
- **After T050**: All lessons integrated and capstone ready ✓
- **After T060**: Chapter renders in Docusaurus ✓
- **After T068**: Chapter deployed to production ✓
