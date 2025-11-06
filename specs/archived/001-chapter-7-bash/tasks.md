# Tasks: Chapter 7 - Bash Essentials for AI-Driven Development

**Input**: Design documents from `/specs/001-chapter-7-bash/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: This is educational content creation, not software development. No automated tests are included.

**Organization**: Tasks are grouped by user story (priority levels P1, P2, P3) to enable independent implementation and delivery of each story as a complete, testable increment.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Educational content will be created in:
- `book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/` (final lesson files)
- Lesson files: `01-terminal-interface.md`, `02-navigation-files.md`, etc.
- Chapter README: `README.md` (chapter introduction and navigation)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize chapter structure and prepare content scaffolding

- [ ] T001 Create chapter directory structure at book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/
- [ ] T002 [P] Create placeholder lesson files (01-08) with YAML frontmatter in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/
- [ ] T003 [P] Create chapter README.md with navigation and introduction in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/README.md

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core reference materials and shared resources needed across all lessons

**⚠️ CRITICAL**: These must be complete before lesson content creation can begin

- [ ] T004 Research and verify all Bash commands work correctly on macOS, Linux, and Windows (Git Bash/WSL)
- [ ] T005 [P] Create command reference tables for navigation, file operations, and process management
- [ ] T006 [P] Document platform-specific differences and notes (macOS vs Linux vs Windows)
- [ ] T007 [P] Compile troubleshooting scenarios and solutions (command not found, permission denied, API key issues)
- [ ] T008 Create keyboard shortcut reference table (Ctrl+R, Tab, Ctrl+A/E/U/C)

**Checkpoint**: Foundation ready - lesson content creation can now begin in parallel

---

## Phase 3: User Story 1 - Beginner Learns Essential Bash Commands (Priority: P1) 🎯 MVP

**Goal**: Enable complete beginners with no terminal experience to learn fundamental Bash commands for filesystem navigation, file management, and working with AI CLI tools

**Independent Test**: Reader can open terminal, navigate to directory, create folder, create files, list contents, and set environment variable for API key without assistance

### Implementation for User Story 1 (Lessons 1-5: Part I)

#### Lesson 1: The Terminal Interface

- [ ] T009 [P] [US1] Write Lesson 1 opening hook (GUI vs CLI, why terminal matters) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/01-terminal-interface.md
- [ ] T010 [P] [US1] Create terminal anatomy section (prompt, command structure, output) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/01-terminal-interface.md
- [ ] T011 [P] [US1] Explain file system hierarchy (absolute vs relative paths) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/01-terminal-interface.md
- [ ] T012 [US1] Add platform differences section (macOS/Linux/Windows) with specific notes in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/01-terminal-interface.md
- [ ] T013 [US1] Create practice exercises for pwd and cd commands in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/01-terminal-interface.md
- [ ] T014 [US1] Write reflection prompt and forward bridge to Lesson 2 in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/01-terminal-interface.md

#### Lesson 2: Navigation and File Management

- [ ] T015 [P] [US1] Write Lesson 2 opening hook (building on terminal basics) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-navigation-files.md
- [ ] T016 [P] [US1] Document cd, pwd, ls commands with flags and examples in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-navigation-files.md
- [ ] T017 [P] [US1] Document mkdir, touch commands with practical scenarios in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-navigation-files.md
- [ ] T018 [US1] Document cp, mv, rm commands with safety patterns and warnings in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-navigation-files.md
- [ ] T019 [US1] Create real-world project setup scenario (create structure, files, navigation) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-navigation-files.md
- [ ] T020 [US1] Add 5 practice exercises progressing from simple to complex file operations in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-navigation-files.md
- [ ] T021 [US1] Write reflection prompt and forward bridge to Lesson 3 in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-navigation-files.md

#### Lesson 3: Viewing and Searching File Content

- [ ] T022 [P] [US1] Write Lesson 3 opening hook (working with file contents) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/03-viewing-searching.md
- [ ] T023 [P] [US1] Document cat, head, tail, less commands with use cases in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/03-viewing-searching.md
- [ ] T024 [P] [US1] Document grep command with pattern examples and common flags in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/03-viewing-searching.md
- [ ] T025 [US1] Explain pipes (|) and redirection (>, >>, 2>) with multi-command workflows in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/03-viewing-searching.md
- [ ] T026 [US1] Document find command with practical search scenarios in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/03-viewing-searching.md
- [ ] T027 [US1] Create realistic log file searching example with grep and pipes in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/03-viewing-searching.md
- [ ] T028 [US1] Add 4 practice exercises for searching and filtering content in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/03-viewing-searching.md
- [ ] T029 [US1] Write reflection prompt and forward bridge to Lesson 4 in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/03-viewing-searching.md

#### Lesson 4: Environment Variables and Package Management

- [ ] T030 [P] [US1] Write Lesson 4 opening hook (configuring your system) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/04-environment-packages.md
- [ ] T031 [P] [US1] Document export command for temporary environment variables in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/04-environment-packages.md
- [ ] T032 [P] [US1] Explain .bashrc/.zshrc for permanent environment variables with step-by-step editing guide in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/04-environment-packages.md
- [ ] T033 [US1] Create detailed API key setup walkthrough (export, .bashrc edit, source) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/04-environment-packages.md
- [ ] T034 [US1] Document pip (Python packages) with examples and virtual environments in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/04-environment-packages.md
- [ ] T035 [US1] Document npm (Node packages) with package.json example in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/04-environment-packages.md
- [ ] T036 [US1] Document Homebrew (macOS) and apt (Linux) with platform-specific instructions in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/04-environment-packages.md
- [ ] T037 [US1] Add 3 practice exercises for setting variables and installing packages in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/04-environment-packages.md
- [ ] T038 [US1] Write reflection prompt and forward bridge to Lesson 5 in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/04-environment-packages.md

#### Lesson 5: Process Management and Troubleshooting

- [ ] T039 [P] [US1] Write Lesson 5 opening hook (monitoring and troubleshooting) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/05-processes-troubleshooting.md
- [ ] T040 [P] [US1] Document ps, top, kill commands with process ID explanation in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/05-processes-troubleshooting.md
- [ ] T041 [P] [US1] Create troubleshooting decision tree for common errors in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/05-processes-troubleshooting.md
- [ ] T042 [US1] Document "command not found" error with diagnosis steps (which, PATH, installation) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/05-processes-troubleshooting.md
- [ ] T043 [US1] Document "permission denied" error with chmod and sudo solutions in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/05-processes-troubleshooting.md
- [ ] T044 [US1] Document API key not persisting error with .bashrc troubleshooting in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/05-processes-troubleshooting.md
- [ ] T045 [US1] Add 3 practice exercises for finding/stopping processes and troubleshooting errors in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/05-processes-troubleshooting.md
- [ ] T046 [US1] Write reflection prompt and forward bridge to Part II (Lesson 6) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/05-processes-troubleshooting.md

**Checkpoint**: At this point, User Story 1 (P1 - Beginner learns essential commands) should be fully functional. Reader can navigate filesystem, manage files, set environment variables, install packages, and troubleshoot common errors without assistance.

---

## Phase 4: User Story 2 - Reader Learns AI-Native Bash Workflows (Priority: P2)

**Goal**: Transform learning from traditional memorization to AI-augmented productivity by teaching readers to use natural language prompts with AI CLI tools instead of memorizing syntax

**Independent Test**: Reader can describe task in natural language to Claude Code or Gemini CLI, AI executes correct Bash command, and reader can explain when to learn command vs. ask AI

### Implementation for User Story 2 (Lessons 6, 8: Part II Foundation)

#### Lesson 6: Natural Language Prompts for Bash Tasks

- [ ] T047 [P] [US2] Write Lesson 6 opening hook (paradigm shift from memorization to AI collaboration) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-natural-language-prompts.md
- [ ] T048 [P] [US2] Explain AI-augmented workflow concept (describe → AI generates → validate → execute) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-natural-language-prompts.md
- [ ] T049 [P] [US2] Document 4 prompting patterns: clear request, problem description, constraints, multi-step in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-natural-language-prompts.md
- [ ] T050 [US2] Create natural language prompt templates for navigation commands (cd, ls, pwd) with 2-3 phrasings each in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-natural-language-prompts.md
- [ ] T051 [US2] Create natural language prompt templates for file operations (cp, mv, rm, mkdir) with 2-3 phrasings each in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-natural-language-prompts.md
- [ ] T052 [US2] Create natural language prompt templates for content operations (cat, grep, find) with 2-3 phrasings each in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-natural-language-prompts.md
- [ ] T053 [US2] Create natural language prompt templates for environment/packages (export, pip, npm) with 2-3 phrasings each in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-natural-language-prompts.md
- [ ] T054 [US2] Create natural language prompt templates for process management (ps, kill) with 2-3 phrasings each in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-natural-language-prompts.md
- [ ] T055 [US2] Add validation before execution section (always understand AI suggestion first) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-natural-language-prompts.md
- [ ] T056 [US2] Create iterative refinement example (task → prompt → AI response → refine → correct command) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-natural-language-prompts.md
- [ ] T057 [US2] Add 4 practice exercises for describing tasks and evaluating AI-generated commands in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-natural-language-prompts.md
- [ ] T058 [US2] Write reflection prompt and forward bridge to Lesson 7 in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/06-natural-language-prompts.md

#### Lesson 8: Real-World AI-Assisted Workflows (US2 Integration)

- [ ] T059 [P] [US2] Write Lesson 8 opening hook (integrating all concepts into realistic workflows) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/08-real-world-workflows.md
- [ ] T060 [P] [US2] Create Workflow 1: Project Setup via AI (directory structure, files, git init, API keys) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/08-real-world-workflows.md
- [ ] T061 [P] [US2] Create Workflow 2: Troubleshooting with AI assistance (error → describe to AI → diagnosis → solution) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/08-real-world-workflows.md
- [ ] T062 [US2] Create Workflow 3: File migration with AI (find files → copy/move → verify) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/08-real-world-workflows.md
- [ ] T063 [US2] Create decision matrix: when to memorize command vs. ask AI (frequency, complexity, safety) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/08-real-world-workflows.md
- [ ] T064 [US2] Document safety patterns for AI-generated commands (verify before execution, understand risks) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/08-real-world-workflows.md
- [ ] T065 [US2] Add 3 integrated practice exercises combining commands and AI assistance in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/08-real-world-workflows.md
- [ ] T066 [US2] Write reflection prompt and transition to Chapter 8 (Git & GitHub) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/08-real-world-workflows.md

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently. Reader has foundational command knowledge (US1) and can use AI to extend capabilities through natural language prompts (US2).

---

## Phase 5: User Story 3 - Reader Builds Professional Workflow Habits (Priority: P3)

**Goal**: Develop professional habits including recognizing dangerous commands, using keyboard shortcuts efficiently, creating aliases, and building muscle memory for common operations

**Independent Test**: Reader demonstrates command safety judgment (verifies before rm -rf), uses keyboard shortcuts (Ctrl+R, Tab), creates aliases, and explains when to memorize vs. ask AI

### Implementation for User Story 3 (Lesson 7: Professional Habits)

#### Lesson 7: Professional Bash Habits and Command Patterns

- [ ] T067 [P] [US3] Write Lesson 7 opening hook (efficiency and professionalism) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-professional-habits.md
- [ ] T068 [P] [US3] Document keyboard shortcut reference table (Ctrl+R, Tab, Ctrl+A/E/U/C/L/D) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-professional-habits.md
- [ ] T069 [P] [US3] Create keyboard shortcut practice exercises with scenarios in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-professional-habits.md
- [ ] T070 [US3] Explain command history and Ctrl+R for history search with examples in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-professional-habits.md
- [ ] T071 [US3] Document Tab completion with filesystem navigation examples in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-professional-habits.md
- [ ] T072 [US3] Create alias examples (ll='ls -la', gs='git status') with .bashrc setup in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-professional-habits.md
- [ ] T073 [US3] Document common command patterns table (find & act, search & filter, monitoring loops) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-professional-habits.md
- [ ] T074 [US3] Create pattern recognition exercises (identify command patterns in scenarios) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-professional-habits.md
- [ ] T075 [US3] Document when to memorize vs. ask AI framework (frequency, safety, complexity dimensions) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-professional-habits.md
- [ ] T076 [US3] Add safety awareness section (dangerous commands, always verify, understand before executing) in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-professional-habits.md
- [ ] T077 [US3] Add 4 practice exercises for shortcuts, aliases, and pattern recognition in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-professional-habits.md
- [ ] T078 [US3] Write reflection prompt and forward bridge to Lesson 8 in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/07-professional-habits.md

**Checkpoint**: All user stories (P1, P2, P3) should now be independently functional. Reader has command knowledge (US1), AI augmentation skills (US2), and professional habits (US3).

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple lessons and overall chapter quality

- [ ] T079 [P] Review all lessons for consistent tone and progressive scaffolding
- [ ] T080 [P] Verify all commands tested on macOS, Linux, and Windows (Git Bash/WSL)
- [ ] T081 [P] Validate all natural language prompt templates have 2-3 alternative phrasings
- [ ] T082 Cross-reference lessons to ensure concepts build progressively (no forward references)
- [ ] T083 [P] Add platform-specific callout boxes where behavior differs significantly
- [ ] T084 [P] Ensure all practice exercises have clear acceptance criteria
- [ ] T085 Create chapter summary section in README.md with key takeaways
- [ ] T086 [P] Add "Further Reading" section with curated external resources
- [ ] T087 Verify alignment with Part 2 README goals and Chapter 8 prerequisites
- [ ] T088 Final proofread for clarity, accessibility (grade 7 reading level), and engagement

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P2): Can start after Foundational - No dependencies on User Story 1 (independent implementation)
  - User Story 3 (P3): Can start after Foundational - No dependencies on User Story 1 or 2 (independent implementation)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - Lessons 1-5 are sequential within the story
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Lessons 6, 8 build on Part I content but US2 tasks are independently implementable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Lesson 7 is independent and can be written anytime after foundational research

**Note**: While the *reading experience* for learners is sequential (Lessons 1→8), the *content creation* can happen in parallel once foundational research is complete. Each lesson file is independent.

### Within Each User Story

- All lessons within a user story can be written in parallel (different files)
- Tasks marked [P] within a lesson can run in parallel
- Each lesson must follow the "lesson" output style (opening hook → content → examples → exercises → reflection)
- Complete all tasks for a lesson before marking lesson complete

### Parallel Opportunities

- **Phase 1 Setup**: All 3 tasks can run in parallel
- **Phase 2 Foundational**: Tasks T005-T008 can run in parallel after T004 completes
- **Phase 3 (US1)**: Within each lesson, all [P] tasks can run in parallel; different lessons can be written concurrently
- **Phase 4 (US2)**: Lessons 6 and 8 can be written in parallel; within each lesson, [P] tasks can run concurrently
- **Phase 5 (US3)**: All Lesson 7 [P] tasks can run in parallel
- **Phase 6 Polish**: Tasks T079-T081, T083-T084, T086 can run in parallel

---

## Parallel Example: User Story 1, Lesson 2

```bash
# Launch all parallel tasks for Lesson 2 together:
Task: "Write Lesson 2 opening hook in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-navigation-files.md"
Task: "Document cd, pwd, ls commands in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-navigation-files.md"
Task: "Document mkdir, touch commands in book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/02-navigation-files.md"

# Then sequentially:
Task: "Document cp, mv, rm commands with safety patterns" (depends on understanding mkdir/touch context)
Task: "Create project setup scenario" (depends on having all commands documented)
Task: "Add practice exercises" (depends on complete command coverage)
Task: "Write reflection and forward bridge" (depends on lesson content being complete)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only - P1)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T008) - CRITICAL
3. Complete Phase 3: User Story 1, Lessons 1-5 (T009-T046)
4. **STOP and VALIDATE**: Review US1 lessons independently - do they enable beginner to learn essential commands?
5. If validation passes, User Story 1 (MVP) is complete and ready for technical review

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 (Lessons 1-5) → Review independently → **MVP delivered**
3. Add User Story 2 (Lessons 6, 8) → Review independently → AI augmentation added
4. Add User Story 3 (Lesson 7) → Review independently → Professional habits added
5. Complete Phase 6: Polish → Final chapter ready for publication

### Parallel Team Strategy

With multiple content creators:

1. Team completes Setup + Foundational together (T001-T008)
2. Once Foundational is done:
   - **Creator A**: User Story 1, Lessons 1-3 (T009-T029)
   - **Creator B**: User Story 1, Lessons 4-5 (T030-T046)
   - **Creator C**: User Story 2, Lesson 6 (T047-T058)
   - **Creator D**: User Story 2, Lesson 8 (T059-T066)
   - **Creator E**: User Story 3, Lesson 7 (T067-T078)
3. Stories complete independently, then integrate and polish

---

## Task Summary

- **Total Tasks**: 88
- **Setup Phase**: 3 tasks
- **Foundational Phase**: 5 tasks (blocking)
- **User Story 1 (P1)**: 38 tasks (T009-T046) - 5 lessons
- **User Story 2 (P2)**: 20 tasks (T047-T066) - 2 lessons
- **User Story 3 (P3)**: 12 tasks (T067-T078) - 1 lesson
- **Polish Phase**: 10 tasks (T079-T088)

**Parallel Opportunities**: 52 tasks marked [P] can run in parallel within their phase/lesson context

**MVP Scope**: Phase 1 (Setup) + Phase 2 (Foundational) + Phase 3 (User Story 1) = 46 tasks total for MVP

---

## Notes

- [P] tasks = different files or independent sections, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story represents a complete, independently deliverable value increment
- No automated tests for educational content - validation through technical review and learner feedback
- All commands must be verified on macOS, Linux, and Windows (Git Bash/WSL) during foundational phase
- Commit after each lesson completion or logical group of tasks
- Stop at any checkpoint to validate story independently before proceeding
- Follow "lesson" output style consistently: opening hook → content → runnable examples → practice exercises → reflection prompt
