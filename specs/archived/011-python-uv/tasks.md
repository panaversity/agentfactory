---
description: "Task list for Chapter 11: Python UV - The Fastest Python Package Manager"
---

# Tasks: Chapter 11 - Python UV: The Fastest Python Package Manager

**Feature**: Chapter content teaching AI-Driven UV workflows (intent-first, AI executes commands)
**Input**: Design documents from `specs/011-python-uv/`
**Prerequisites**: spec.md (user stories), plan.md (lesson structure)

**Tests**: No automated tests for educational content. Validation via validation-auditor and manual testing of all UV commands.

**Organization**: Tasks are organized by lesson (maps to user stories in spec.md) to enable independent writing and review of each lesson.

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which lesson this task belongs to (L1=Lesson 1, L2=Lesson 2, etc.)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Chapter Infrastructure)

**Purpose**: Initialize chapter structure and shared assets

- [X] T001 Create chapter directory structure: `book-source/docs/04-Part-4-Python-Fundamentals/11-python-uv/`
- [ ] T002 [P] Create placeholder lesson files (lesson-1.md through lesson-6.md) with YAML frontmatter
- [ ] T003 [P] Create `book-source/static/img/part-4/chapter-11/` directory for diagrams and screenshots
- [ ] T004 [P] Update `book-source/sidebars.ts` with Chapter 11 entry and 6 lesson sub-entries
- [ ] T005 Verify UV is installed on authoring machine (all platforms: Windows/Mac/Linux for testing commands)

---

## Phase 2: Foundational (Shared Content Components)

**Purpose**: Create reusable content patterns and verify all UV commands work

**⚠️ CRITICAL**: All UV commands MUST be tested on Windows/Mac/Linux before writing lessons

- [ ] T006 Test all UV commands on Windows PowerShell, document outputs in `specs/011-python-uv/checklists/command-verification.md`
- [ ] T007 Test all UV commands on macOS, document outputs in `specs/011-python-uv/checklists/command-verification.md`
- [ ] T008 [P] Create sample AI prompt-response templates in `specs/011-python-uv/checklists/prompt-patterns.md`
- [ ] T009 [P] Create UV vs pip/poetry/conda comparison table (SVG diagram) in `book-source/static/img/part-4/chapter-11/tool-comparison.svg`
- [ ] T010 [P] Create "Project Structure Diagram" (pyproject.toml, uv.lock, .python-version, src/) in `book-source/static/img/part-4/chapter-11/project-structure.svg`
- [ ] T011 [P] Create "Dependency Resolution Visual" showing transitive dependencies in `book-source/static/img/part-4/chapter-11/dependency-tree.svg`
- [ ] T012 Create chapter index page at `book-source/docs/04-Part-4-Python-Fundamentals/11-python-uv/index.md` with chapter overview

**Checkpoint**: All commands verified, diagrams created, ready for lesson writing

---

## Phase 3: Lesson 1 - Why UV? Understanding Modern Python Package Management (Priority: P1) 🎯 MVP

**Goal**: Readers understand UV's value proposition (speed, unified tooling) and when to use UV vs pip/poetry/conda

**Independent Test**: Reader can explain "why UV exists" and choose appropriate tool for 3 scenarios without reciting commands

**Story**: User Story 1 from spec.md

### Implementation for Lesson 1

- [X] T013 [L1] Write opening hook: "The 30-Second Setup" timing comparison (UV vs pip) in `lesson-1.md` lines 1-50
- [X] T014 [L1] Write "What is a package manager?" section (beginner-friendly, analogy-based) in `lesson-1.md` lines 51-150
- [X] T015 [L1] Write "The Problem: Python's Fragmented Tooling" section with ecosystem diagram in `lesson-1.md` lines 151-250
- [X] T016 [L1] Write "The Solution: UV's Unified Approach" section (speed, simplicity, industry backing) in `lesson-1.md` lines 251-350
- [X] T017 [L1] Create decision framework table: "When to Use UV vs pip vs poetry vs conda" with scenarios in `lesson-1.md` lines 351-450
- [X] T018 [L1] Write "What is a Dependency?" explainer (plain language, examples) in `lesson-1.md` lines 451-500
- [X] T019 [L1] Write "Our Learning Approach: AI-Driven Development" section explaining intent → AI → understanding pattern in `lesson-1.md` lines 501-600
- [X] T020 [L1] Write "Try With AI" section with 3 exploration prompts (no UV installation yet, conceptual only) in `lesson-1.md` lines 601-700
- [X] T021 [L1] Add YAML frontmatter with skills metadata: "Evaluate Python Package Managers (A2)", "Recognize Speed Benefits (A1)" in `lesson-1.md` lines 1-20
- [X] T022 [L1] Proofread Lesson 1 for complexity tier compliance (max 7 concepts, beginner-friendly language, A2-B1 level)

**Checkpoint**: Lesson 1 complete - readers motivated and understand UV's purpose

---

## Phase 4: Lesson 2 - Installing UV with AI Collaboration (Priority: P2)

**Goal**: Readers successfully install UV using Claude Code/Gemini CLI, understanding platform detection and PATH configuration

**Independent Test**: Reader installs UV by expressing intent to AI, verifies installation, troubleshoots "command not found" errors

**Story**: User Story 2 from spec.md

### Implementation for Lesson 2

- [X] T023 [L2] Write "Pre-Installation: What We're About to Do" section in `lesson-2.md` lines 1-100
- [X] T024 [L2] Write "AI-Driven Installation" section with complete prompt-response example (Windows PowerShell) in `lesson-2.md` lines 101-250
- [X] T025 [P] [L2] Write macOS installation variant (Homebrew/curl) prompt-response in `lesson-2.md` lines 251-350
- [X] T026 [P] [L2] Write Linux installation variant (curl) prompt-response in `lesson-2.md` lines 351-450
- [X] T027 [L2] Write "Platform Detection: How AI Determines Your OS" explainer in `lesson-2.md` lines 451-550
- [X] T028 [L2] Write "Verification: Running uv --version" section with AI-guided verification in `lesson-2.md` lines 551-650
- [X] T029 [L2] Write "Troubleshooting: Command Not Found" section with PATH explanation (beginner-friendly) in `lesson-2.md` lines 651-800
- [X] T030 [L2] Include complete "PATH Configuration" explainer (computer's command directory analogy) in `lesson-2.md` lines 801-900
- [X] T031 [L2] Write "Try With AI" section with 4 installation/verification prompts in `lesson-2.md` lines 901-1000
- [X] T032 [L2] Add YAML frontmatter with skills metadata: "Execute Installation with AI (A2)", "Understand PATH Configuration (A2)" in `lesson-2.md` lines 1-20
- [X] T033 [L2] Proofread Lesson 2 for hands-on clarity and error scenario coverage (max 7 concepts, A2-B1 level)

**Checkpoint**: Lesson 2 complete - readers have UV installed and verified

---

## Phase 5: Lesson 3 - Creating Your First UV Project with AI (Priority: P3)

**Goal**: Readers create a Python project using AI (pyproject.toml, .python-version, src/) and understand project anatomy

**Independent Test**: Reader creates UV project by expressing intent, adds a simple dependency (requests), explains project structure

**Story**: User Story 3 from spec.md

### Implementation for Lesson 3

- [X] T034 [L3] Write "What is a Python Project?" section (structure, dependencies, configuration analogy) in `lesson-3.md` lines 1-150
- [X] T035 [L3] Write "The AI-Driven Creation Flow" section (intent → AI generates structure → explore) in `lesson-3.md` lines 151-250
- [X] T036 [L3] Write complete walkthrough: "Create a simple web-client project with UV and AI" (prompt-response-output-teaching) in `lesson-3.md` lines 251-500
- [X] T037 [L3] Include "uv init" command explanation (what AI does behind the scenes) in `lesson-3.md` lines 501-600
- [X] T038 [L3] Write "Anatomy of a UV Project" section with annotated directory tree (pyproject.toml, .python-version, src/, .venv) in `lesson-3.md` lines 601-750
- [X] T039 [L3] Write "Important Parts of pyproject.toml" section (brief, plain language, no deep TOML syntax) in `lesson-3.md` lines 751-900
- [X] T040 [L3] Write "Virtual Environments Explained" section (isolation analogy: separate toolboxes) in `lesson-3.md` lines 901-1000
- [X] T041 [L3] Write "Adding Your First Dependency" subsection (uv add requests) with complete prompt-response-output-teaching in `lesson-3.md` lines 1001-1150
- [X] T042 [L3] Write "Legacy vs Modern: pyproject.toml vs requirements.txt" comparison section in `lesson-3.md` lines 1151-1250
- [X] T043 [L3] Write "Try With AI" section with 4 project creation/exploration prompts in `lesson-3.md` lines 1251-1350
- [X] T044 [L3] Add YAML frontmatter with skills metadata: "Initialize UV Project (B1)", "Understand Project Configuration (A2)" in `lesson-3.md` lines 1-20
- [X] T045 [L3] Proofread Lesson 3 for conceptual clarity and hands-on completeness (max 7 concepts, B1 level)

**Checkpoint**: Lesson 3 complete - readers have working UV project with dependencies

---

## Phase 6: Lesson 4 - Managing Dependencies with AI (Priority: P4)

**Goal**: Readers add/update/remove dependencies using AI, understanding dependency resolution and version constraints

**Independent Test**: Reader manages dependencies (add production + dev, update, remove) by expressing needs to AI, handles conflicts

**Story**: User Story 4 from spec.md

### Implementation for Lesson 4

- [X] T046 [L4] Write "What Are Dependencies?" section (libraries your project needs, recipe ingredients analogy) in `lesson-4.md` lines 1-150
- [X] T047 [L4] Write "Adding Dependencies with AI" section with prompt-response example (uv add requests) in `lesson-4.md` lines 151-300
- [X] T048 [L4] Write "Development vs Production Dependencies" section (why separate them, lean deployments) in `lesson-4.md` lines 301-450
- [X] T049 [L4] Write complete prompt-response example: "Add pytest and pytest-cov as dev dependencies" (uv add --dev) in `lesson-4.md` lines 451-650
- [X] T050 [L4] Write "Dependency Resolution Magic" section (how UV finds compatible versions, transitive dependencies) in `lesson-4.md` lines 651-800
- [X] T051 [L4] Include dependency tree visual diagram reference in `lesson-4.md` lines 801-850
- [X] T052 [L4] Write "Updating Dependencies" section with prompt-response example (uv add package@latest) in `lesson-4.md` lines 851-1000
- [X] T053 [L4] Write "Listing Outdated Dependencies" prompt-response example (uv pip list --outdated) in `lesson-4.md` lines 1001-1100
- [X] T054 [L4] Write "Removing Dependencies" section with prompt-response example (uv remove package) in `lesson-4.md` lines 1101-1200
- [X] T055 [L4] Write "Conflict Resolution: When Dependencies Clash" section with error scenario and AI diagnosis in `lesson-4.md` lines 1201-1400
- [X] T056 [L4] Write "The Lockfile Concept" explainer (pinning exact versions for reproducibility) in `lesson-4.md` lines 1401-1500
- [X] T057 [L4] Write "Try With AI" section with 5 dependency management prompts in `lesson-4.md` lines 1501-1650
- [X] T058 [L4] Add YAML frontmatter with skills metadata: "Manage Project Dependencies (B1)", "Understand Dependency Resolution (B1)" in `lesson-4.md` lines 1-20
- [X] T059 [L4] Proofread Lesson 4 for scenario coverage and error handling clarity (max 7 concepts, B1 level)

**Checkpoint**: Lesson 4 complete - readers can manage dependencies confidently with AI

---

## Phase 7: Lesson 5 - Running Python Code in UV Projects with AI (Priority: P5)

**Goal**: Readers run scripts and tests using `uv run` via AI, understanding environment isolation and automatic activation

**Independent Test**: Reader runs script and tests by expressing intent, explains why environment isolation matters, debugs "module not found"

**Story**: User Story 5 from spec.md

### Implementation for Lesson 5

- [X] T060 [L5] Write "Why Environment Isolation?" section (preventing conflicts between projects, separate workspaces analogy) in `lesson-5.md` lines 1-150
- [X] T061 [L5] Write "UV's Automatic Activation" section (no manual venv activation needed) in `lesson-5.md` lines 151-250
- [X] T062 [L5] Write "Running Scripts with AI" section with prompt-response example (uv run python main.py) in `lesson-5.md` lines 251-400
- [X] T063 [L5] Write prompt-response example: "Run a sample client script that fetches data from public API using requests" in `lesson-5.md` lines 401-600
- [X] T064 [L5] Write "Running Tests" section with prompt-response example (uv run pytest) in `lesson-5.md` lines 601-750
- [X] T065 [L5] Write "One-Off Commands in Project Environment" section with example (uv run pip list) in `lesson-5.md` lines 751-850
- [X] T066 [L5] Write "Comparison: uv run vs regular python" section demonstrating the difference in `lesson-5.md` lines 851-1000
- [X] T067 [L5] Write "Troubleshooting: Module Not Found" section with error scenario and AI diagnosis in `lesson-5.md` lines 1001-1150
- [X] T068 [L5] Write "Try With AI" section with 5 execution and debugging prompts in `lesson-5.md` lines 1151-1300
- [X] T069 [L5] Add YAML frontmatter with skills metadata: "Execute Python in Project Environment (B1)", "Understand Environment Isolation (B1)" in `lesson-5.md` lines 1-20
- [X] T070 [L5] Proofread Lesson 5 for execution clarity and debugging scenario coverage (max 7 concepts, B1 level)

**Checkpoint**: Lesson 5 complete - readers can execute code in isolated environments with AI

---

## Phase 8: Lesson 6 - Team Collaboration and Reproducible Environments with AI (Priority: P6)

**Goal**: Readers share projects with teammates using lockfiles and `uv sync`, understanding reproducible environments

**Independent Test**: Reader prepares project for sharing, recreates environment from lockfile, explains pyproject.toml vs uv.lock difference

**Story**: User Story 6 from spec.md

### Implementation for Lesson 6

- [X] T071 [L6] Write "The Collaboration Problem: 'Works on My Machine'" section in `lesson-6.md` lines 1-150
- [X] T072 [L6] Write "Lockfiles to the Rescue" section (how pinned versions solve reproducibility) in `lesson-6.md` lines 151-300
- [X] T073 [L6] Write "Preparing Projects for Sharing" section with prompt-response example (ensure lockfile exists) in `lesson-6.md` lines 301-450
- [X] T074 [L6] Write "Teammate Setup Flow: Cloning and Syncing" section with prompt-response example (uv sync) in `lesson-6.md` lines 451-650
- [X] T075 [L6] Write "Production Deployments: Installing Without Dev Dependencies" section with prompt-response (uv sync --no-dev) in `lesson-6.md` lines 651-800
- [X] T076 [L6] Write "pyproject.toml vs uv.lock" comparison section (constraints vs pinned versions) in `lesson-6.md` lines 801-950
- [X] T077 [L6] Write "Lockfile Maintenance: When to Update" section (after dependency changes) in `lesson-6.md` lines 951-1050
- [X] T078 [L6] Write "Git Integration: Committing the Right Files" section (pyproject.toml, uv.lock, NOT .venv) in `lesson-6.md` lines 1051-1150
- [ ] T079 [L6] Create visual workflow diagram: Developer A → git → Developer B (with lockfile sync) in `book-source/static/img/part-4/chapter-11/collaboration-workflow.svg`
- [X] T080 [L6] Write "Try With AI" section with 5 collaboration/deployment prompts in `lesson-6.md` lines 1151-1300
- [X] T081 [L6] Add YAML frontmatter with skills metadata: "Share Projects Reproducibly (B1)", "Understand Lockfile Purpose (B1)" in `lesson-6.md` lines 1-20
- [X] T082 [L6] Proofread Lesson 6 for collaboration workflow clarity and production deployment coverage (max 7 concepts, B1 level)

**Checkpoint**: Lesson 6 complete - readers can collaborate on UV projects with reproducible environments

---

## Phase 9: Chapter Integration and Quality Assurance

**Purpose**: Cross-lesson coherence, technical validation, and chapter-level polish

- [ ] T083 Review all 6 lessons for cross-references and prerequisite flow (Lesson 1→2→3→4→5→6 progression)
- [ ] T084 Verify all AI prompt-response examples follow 4-part pattern: Prompt → Response → Output → Teaching
- [ ] T085 Verify all lessons respect A2-B1 complexity tier (max 7 concepts per lesson, AI as command executor)
- [ ] T086 [P] Test all UV commands in lessons on Windows PowerShell, document results in `specs/011-python-uv/checklists/windows-verification.md`
- [ ] T087 [P] Test all UV commands in lessons on macOS, document results in `specs/011-python-uv/checklists/macos-verification.md`
- [ ] T088 [P] Test all UV commands in lessons on Linux, document results in `specs/011-python-uv/checklists/linux-verification.md`
- [ ] T089 Verify all diagrams render correctly in Docusaurus build
- [ ] T090 Verify chapter navigation (previous/next links) works correctly in `book-source/sidebars.ts`
- [ ] T091 Create end-of-chapter quiz (15 questions, 75%+ passing) in `book-source/docs/04-Part-4-Python-Fundamentals/11-python-uv/quiz.md`
- [ ] T092 Create hands-on exercise: "Create a requests-based project with UV and AI" in `book-source/docs/04-Part-4-Python-Fundamentals/11-python-uv/exercise.md`
- [ ] T093 Proofread entire chapter for voice consistency, tone, and beginner-friendliness (Grade 7+ reading level)

**Checkpoint**: Chapter 11 complete and ready for validation-auditor validation

---

## Phase 10: Validation and Publication Preparation

**Purpose**: External review and final polish before publication

- [ ] T094 Invoke validation-auditor subagent for constitution alignment (AI-first teaching, spec-first, validation steps)
- [ ] T095 Address critical issues from validation-auditor (MUST fix before proceeding)
- [ ] T096 Address major issues from validation-auditor (should fix, human decision)
- [ ] T097 Run Docusaurus build test: `cd book-source ; pnpm build` - verify no errors
- [ ] T098 Visual inspection: formatting, code blocks, images, cross-references
- [ ] T099 Final editorial polish: voice, tone, flow, grammar
- [ ] T100 Update `specs/book/chapter-index.md` status for Chapter 11 to "Complete"

**Checkpoint**: Chapter 11 validated and ready for publication

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all lesson writing
- **Lessons (Phases 3-8)**: All depend on Foundational phase completion
  - Lessons MUST be written sequentially (L1→L2→L3→L4→L5→L6) due to prerequisite flow
  - Each lesson builds on concepts from previous lessons
- **Integration (Phase 9)**: Depends on all 6 lessons being complete
- **Validation (Phase 10)**: Depends on Integration phase completion

### Lesson Dependencies

- **Lesson 1 (Why UV)**: Can start after Foundational (Phase 2) - No dependencies on other lessons
- **Lesson 2 (Installation)**: Depends on Lesson 1 (motivation established) - Readers install UV
- **Lesson 3 (First Project)**: Depends on Lesson 2 (UV installed) - Readers create project
- **Lesson 4 (Dependencies)**: Depends on Lesson 3 (project created) - Readers add/manage packages
- **Lesson 5 (Running Code)**: Depends on Lesson 4 (dependencies installed) - Readers execute scripts
- **Lesson 6 (Collaboration)**: Depends on Lessons 3-5 (project management skills) - Readers share projects

### Within Each Lesson

- Opening sections before hands-on examples
- Conceptual understanding before command execution
- Basic prompts before advanced scenarios
- "Try With AI" section always last in lesson

### Parallel Opportunities

- Setup tasks (T002, T003, T004) can run in parallel
- Foundational tasks (T008, T009, T010, T011) can run in parallel (diagrams)
- Platform verification (T025, T026 in Lesson 2) can run in parallel
- Command testing (T086, T087, T088 in Phase 9) can run in parallel across platforms

---

## Parallel Example: Lesson 4 Implementation

```bash
# Task T046-T048 (conceptual sections) can be written in parallel by different writers:
git checkout -b lesson-4-concepts
# Writer A: T046 (What Are Dependencies?)
# Writer B: T047 (Adding Dependencies)
# Writer C: T048 (Dev vs Production)

# Task T049-T055 (prompt-response examples) can be written in parallel after concepts complete:
git checkout -b lesson-4-examples
# Writer A: T049 (dev dependencies example)
# Writer B: T052 (update dependencies example)
# Writer C: T054 (remove dependencies example)

# Sequential after examples: T056-T059 (integration and proofing)
```

---

## Implementation Strategy

**MVP Scope (Must Have)**:
- Lessons 1-3: Motivation → Installation → First Project (complete UV workflow foundation)
- At least 10 complete AI prompt-response examples (4-part pattern)
- All diagrams created and integrated
- All commands tested on at least 2 platforms (Windows + macOS or Linux)

**Post-MVP (Should Have)**:
- Lessons 4-6: Dependencies → Execution → Collaboration (professional workflows)
- All 20+ prompt-response examples complete
- All commands tested on all 3 platforms
- End-of-chapter quiz and hands-on exercise

**Nice to Have**:
- Advanced UV features appendix (Python version management, standalone scripts, tool installation)
- Video walkthrough demonstrating AI-Driven UV workflow
- Interactive quiz with immediate feedback

**Out of Scope**:
- Building and publishing Python packages to PyPI (future chapter)
- UV internals or Rust implementation details
- Private package index configuration
- Migration guide from pip/poetry to UV

---

## Success Criteria Mapping

| Success Criterion | Validated By Task(s) |
|-------------------|---------------------|
| SC-001: Explain UV's value in own words | T013-T022 (Lesson 1) |
| SC-002: Create UV project via AI | T034-T045 (Lesson 3) |
| SC-003: Manage dependencies via AI | T046-T059 (Lesson 4) |
| SC-004: Run code with environment isolation | T060-T070 (Lesson 5) |
| SC-005: Share project with teammates | T071-T082 (Lesson 6) |
| SC-006: Troubleshoot UV errors with AI | T029, T055, T067 (error scenarios in lessons 2, 4, 5) |
| SC-007: 15+ prompt-response examples | T083, T084 (verification in Phase 9) |
| SC-008: Understand when to use UV vs alternatives | T017 (decision framework in Lesson 1) |
| SC-010: 90% don't need direct UV docs | T094-T096 (validation-auditor validates AI-sufficiency) |
| SC-011: Can teach teammate about UV | T091, T092 (quiz and exercise test teaching ability) |

---

## Notes for Implementation

**AI-Driven Development Pattern**:
Every UV task MUST follow the 4-part pattern:
1. **Prompt**: Reader's natural language intent to Claude Code/Gemini CLI
2. **Response**: AI's command with explanation
3. **Output**: Expected terminal output
4. **Teaching**: AI's follow-up explanation (concepts, not syntax)

**Complexity Tier Compliance**:
- Max 7 new concepts per lesson section
- AI as primary command executor (students focus on understanding, not memorization)
- Beginner-friendly language (Grade 7+ reading level)
- Real-world scenarios (team collaboration, production deployment)

**Testing Requirements**:
- All commands MUST be tested on Windows PowerShell, macOS, Linux
- Document platform-specific variations
- Verify all error scenarios are reproducible
- Test with latest UV version (as of writing date)

**Cross-References**:
- Chapter 5 (Claude Code) and Chapter 6 (Gemini CLI) - AI CLI tools
- Chapter 7 (Bash Essentials) - terminal competency
- Chapter 8 (Git & GitHub) - collaboration workflow (Lesson 6)
- Chapter 12 (Introduction to Python) - next chapter uses UV projects

**Validation Checklist**:
- [ ] All lessons have YAML frontmatter with skills metadata
- [ ] All prompt-response examples complete and tested
- [ ] All diagrams created and render correctly
- [ ] All cross-references valid
- [ ] Chapter passes validation-auditor validation
- [ ] Docusaurus build successful
- [ ] Visual inspection complete
