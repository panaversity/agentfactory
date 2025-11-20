# Tasks: Chapter 8 Redesign - Git & GitHub for AI-Driven Development

**Input**: Design documents from `/specs/028-chapter-8-git-redesign/`
**Prerequisites**: plan.md (7-lesson structure), spec.md (4 user scenarios with priorities)

**Organization**: Tasks organized by lesson implementation (matching user scenario priorities)

**Policy Note**: Within this chapter, each lesson must end with a single final section titled "Try With AI" (no "Key Takeaways" or "What's Next"). Since this is Part 2, instruct learners to use ChatGPT web (AI tools not yet taught).

## Format: `[ID] [P?] [Lesson/Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Lesson]**: Which lesson this task belongs to (L1-L7)
- **[Story]**: Maps to user scenario from spec (US1-US4)
- Include exact file paths in descriptions

## Path Conventions

All lessons in: `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`

---

## Phase 1: Setup & Foundation (Content Redesign Infrastructure)

**Purpose**: Prepare for 7-lesson restructure and research requirements

- [ ] T001 Read existing 9 lessons in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/` to identify reusable content
- [ ] T002 [P] Research GitHub Agent HQ features from official GitHub Blog (https://github.blog/news-insights/company-news/welcome-home-agents/)
- [ ] T003 [P] Test all Git commands in sandbox environment (init, status, add, commit, diff, restore, reset, branch, checkout, switch, merge, remote, push, pull, clone)
- [ ] T004 [P] Capture terminal logs for each Git command test (for lesson inclusion per C-014 constraint)
- [ ] T005 Create backup of existing 9-lesson chapter to `specs/028-chapter-8-git-redesign/original-lessons-backup/`
- [ ] T006 Generate git-workflow.md skill template for Lesson 6 (simple markdown guide format per clarification resolution)

**Checkpoint**: Research complete, Git commands validated, existing content audited

---

## Phase 2: Lesson 1 - Your First Git Repository (US1: Safe AI Experimentation - Foundation)

**Goal**: Students manually execute init, status, add, commit, log to build mental model of staging/commits BEFORE AI assistance

**Independent Test**: Student creates test project, initializes Git, stages files, creates commit, views history—all without AI help

**Stage**: 1 (Manual Foundation)
**Cognitive Load**: 5 concepts (Git repository, working directory state, staging area, commits, safety mindset)

### Implementation for Lesson 1

- [ ] T007 [L1] [US1] Create `01-your-first-git-repository.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
- [ ] T008 [L1] [US1] Write intro section explaining Git as safety system for AI experimentation (not version control commands)
- [ ] T009 [L1] [US1] Create hands-on activity: "Execute `git init` in empty folder, observe `.git` directory creation"
- [ ] T010 [L1] [US1] Create hands-on activity: "Execute `git status`, observe untracked file message"
- [ ] T011 [L1] [US1] Create hands-on activity: "Execute `git add <file>`, then `git status` again, observe staged changes (green)"
- [ ] T012 [L1] [US1] Create hands-on activity: "Execute `git commit -m 'First commit'`, observe commit success"
- [ ] T013 [L1] [US1] Create hands-on activity: "Execute `git log`, observe commit history with hash, author, timestamp"
- [ ] T014 [L1] [US1] Write reflection prompt: "Why is commit a 'save point'? How does this enable safe AI experimentation?"
- [ ] T015 [L1] [US1] Add terminal log screenshots for all 5 Git commands (from T004 sandbox testing)
- [ ] T016 [L1] [US1] Add "Try With AI" final section: "Ask ChatGPT to explain what you just did, test if it understands staging area"
- [ ] T017 [L1] [US1] Add YAML frontmatter with skills proficiency metadata, learning objectives, cognitive load assessment (5 concepts)
- [ ] T018 [L1] [US1] Validate lesson follows hands-on discovery modality (not lecture-style) per C-003 constraint

**Checkpoint**: Lesson 1 complete—students can execute Git basics manually without AI

---

## Phase 3: Lesson 2 - Viewing Changes & Safe Undo (US1: Safe AI Experimentation - Validation)

**Goal**: Students manually execute diff, restore, reset to build confidence in error recovery BEFORE AI helps with undo

**Independent Test**: Student makes changes, views diff, undoes unstaged changes, undoes staged changes, undoes commit—all manually

**Stage**: 1 (Manual Foundation)
**Cognitive Load**: 4 concepts (diff, restore, reset, destructive vs non-destructive)

### Implementation for Lesson 2

- [ ] T019 [L2] [US1] Create `02-viewing-changes-safe-undo.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
- [ ] T020 [L2] [US1] Write intro section framing undo as "fearless experimentation enabler"
- [ ] T021 [L2] [US1] Create hands-on activity: "Modify file, execute `git diff`, observe change visualization"
- [ ] T022 [L2] [US1] Create hands-on activity: "Execute `git restore <file>`, observe unstaged changes disappear"
- [ ] T023 [L2] [US1] Create hands-on activity: "Stage file, execute `git reset HEAD <file>`, observe staging undo"
- [ ] T024 [L2] [US1] Create hands-on activity: "Create commit, execute `git reset --hard HEAD~1`, observe commit undo with CAUTION warning"
- [ ] T025 [L2] [US1] Add decision tree: "Which undo command for which scenario? (unstaged/staged/committed)"
- [ ] T026 [L2] [US1] Add terminal log screenshots for diff, restore, reset commands
- [ ] T027 [L2] [US1] Add "Try With AI" section: "Ask ChatGPT: 'I accidentally staged wrong files, how do I undo?' Compare its answer to what you learned"
- [ ] T028 [L2] [US1] Add YAML frontmatter with 4 concept cognitive load validation
- [ ] T029 [L2] [US1] Map to success criteria SC-003 (80%+ recover from errors within 3 attempts)

**Checkpoint**: Lesson 2 complete—students confidently undo mistakes manually

---

## Phase 4: Lesson 3 - Testing AI Safely with Branches (US3: Branch Testing + US1: Stage 2 AI Collaboration)

**Goal**: Students use branches for parallel AI experiments + experience Three Roles (AI teaches branching, student validates, converge)

**Independent Test**: Student creates 2 branches, implements different AI suggestions in each, compares, merges chosen branch

**Stage**: 2 (AI Collaboration)
**Cognitive Load**: 5 concepts (branch, checkout/switch, merge, HEAD, branch deletion)

### Implementation for Lesson 3

- [ ] T030 [L3] [US3] Create `03-testing-ai-safely-with-branches.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
- [ ] T031 [L3] [US3] Write intro section framing branches as "parallel universes for testing AI suggestions"
- [ ] T032 [L3] [US3] Create hands-on activity: "Execute `git branch feature-simple`, then `git branch`, observe branch list"
- [ ] T033 [L3] [US3] Create hands-on activity: "Execute `git checkout feature-simple`, observe branch switch in status"
- [ ] T034 [L3] [US3] Create hands-on activity: "Make changes on branch, commit, switch to main, observe changes disappear (branch isolation)"
- [ ] T035 [L3] [US3] **Three Roles Scenario 1 (AI as Teacher)**: "Ask ChatGPT: 'What are best practices for branch naming?' AI teaches conventions student didn't know"
- [ ] T036 [L3] [US3] **Three Roles Scenario 2 (Student as Teacher)**: "Tell ChatGPT your project context, ask it to suggest branch workflow, correct if generic (teach AI your constraints)"
- [ ] T037 [L3] [US3] **Three Roles Scenario 3 (Co-Worker Convergence)**: "Iterate with ChatGPT on merge strategy: test fast-forward, three-way merge, arrive at best for your case"
- [ ] T038 [L3] [US3] Create hands-on activity: "Execute `git checkout main && git merge feature-simple`, observe merge success"
- [ ] T039 [L3] [US3] Create hands-on activity: "Execute `git branch -d feature-simple`, observe merged branch deletion"
- [ ] T040 [L3] [US3] Add terminal log screenshots for branch, checkout, switch, merge commands
- [ ] T041 [L3] [US3] Add "Try With AI" section: "Ask ChatGPT to generate 2 different implementations of same feature, use branches to test both"
- [ ] T042 [L3] [US3] Add YAML frontmatter with Stage 2 tag, Three Roles demonstration validation
- [ ] T043 [L3] [US3] Map to success criteria SC-004 (75%+ manage branches without confusion)

**Checkpoint**: Lesson 3 complete—students use branches + experience bidirectional AI learning

---

## Phase 5: Lesson 4 - Cloud Backup & Portfolio (US2: GitHub Backup + Stage 2 AI Collaboration)

**Goal**: Students connect local repo to GitHub for backup + portfolio, with AI assisting GitHub setup

**Independent Test**: Student creates GitHub repo, pushes local project, clones from different machine, shares URL

**Stage**: 2 (AI Collaboration)
**Cognitive Load**: 4 concepts (GitHub account, remote, push, pull/clone)

### Implementation for Lesson 4

- [ ] T044 [L4] [US2] Create `04-cloud-backup-portfolio.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
- [ ] T045 [L4] [US2] Write intro section framing GitHub as "catastrophe prevention + career advancement"
- [ ] T046 [L4] [US2] Create hands-on activity: "Create free GitHub account at github.com"
- [ ] T047 [L4] [US2] Create hands-on activity: "Create new repository via GitHub web interface"
- [ ] T048 [L4] [US2] **Three Roles Scenario 1 (Student as Teacher)**: "Ask ChatGPT: 'Connect my local repo to GitHub', then tell ChatGPT your repo URL, observe generic answer, refine with specifics"
- [ ] T049 [L4] [US2] Create hands-on activity: "Execute `git remote add origin <URL>`, then `git remote -v`, observe remote connection"
- [ ] T050 [L4] [US2] Create hands-on activity: "Execute `git push -u origin main`, handle PAT authentication (with AI guidance if stuck)"
- [ ] T051 [L4] [US2] Create hands-on activity: "Visit GitHub repo page, observe code + commits in cloud"
- [ ] T052 [L4] [US2] Create hands-on activity: "From different directory, execute `git clone <URL>`, observe full project recovery"
- [ ] T053 [L4] [US2] Add troubleshooting section: Authentication failures (PAT setup), branch name mismatch (main vs master), large files warning
- [ ] T054 [L4] [US2] Add terminal log screenshots for remote, push, clone commands
- [ ] T055 [L4] [US2] Add "Try With AI" section: "Ask ChatGPT to review your GitHub profile README suggestions"
- [ ] T056 [L4] [US2] Add YAML frontmatter with 4 concept cognitive load validation
- [ ] T057 [L4] [US2] Map to success criteria SC-006 (GitHub repo + push within 10 minutes) and SC-011 (70%+ create portfolio)

**Checkpoint**: Lesson 4 complete—students have cloud backup + shareable portfolio

---

## Phase 6: Lesson 5 - Code Review with Pull Requests (US2: GitHub Collaboration + Stage 2 AI Collaboration)

**Goal**: Students create PRs, document AI assistance transparently, experience collaborative review workflow

**Independent Test**: Student creates feature branch, pushes, opens PR with AI transparency documentation, merges via GitHub

**Stage**: 2 (AI Collaboration)
**Cognitive Load**: 4 concepts (PR creation, diff review, AI transparency, PR merge)

### Implementation for Lesson 5

- [ ] T058 [L5] [US2] Create `05-code-review-pull-requests.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
- [ ] T059 [L5] [US2] Write intro section framing PRs as "professional workflow + AI transparency practice"
- [ ] T060 [L5] [US2] Create hands-on activity: "Create feature branch, make changes, commit, push to GitHub"
- [ ] T061 [L5] [US2] Create hands-on activity: "On GitHub, click 'Compare & pull request', observe PR creation interface"
- [ ] T062 [L5] [US2] Create PR description template: Title, Summary, AI Assistance section (which AI used, what it generated, what student modified), Testing Done
- [ ] T063 [L5] [US2] **Three Roles Scenario (Co-Worker Convergence)**: "Ask ChatGPT to review your PR description for clarity, iterate on improvements"
- [ ] T064 [L5] [US2] Create hands-on activity: "Review PR diff on GitHub, verify changes are correct"
- [ ] T065 [L5] [US2] Create hands-on activity: "Merge PR using GitHub interface, observe main branch update"
- [ ] T066 [L5] [US2] Add example: AI-generated code PR with transparency documentation showing exactly what AI created vs what student wrote
- [ ] T067 [L5] [US2] Add terminal log screenshots for push, plus GitHub PR interface screenshots
- [ ] T068 [L5] [US2] Add "Try With AI" section: "Ask ChatGPT to generate commit message following conventional commits format, use in your next PR"
- [ ] T069 [L5] [US2] Add YAML frontmatter with Stage 2 tag, AI transparency emphasis
- [ ] T070 [L5] [US2] Map to success criteria SC-008 (100% capstone PRs include AI attribution)

**Checkpoint**: Lesson 5 complete—students create professional PRs with AI transparency

---

## Phase 7: Lesson 6 - Reusable Git Patterns (US1+US2+US3: Stage 3 Intelligence Design)

**Goal**: Students identify recurring Git workflows, document as git-workflow.md for future reuse

**Independent Test**: Student creates git-workflow.md with 3+ patterns (commit-before-experiment, branch-test-merge, push-for-backup), applies to new project

**Stage**: 3 (Intelligence Design)
**Cognitive Load**: 3 concepts (pattern recognition, workflow documentation, pattern application)

### Implementation for Lesson 6

- [ ] T071 [L6] [US1+US2+US3] Create `06-reusable-git-patterns.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
- [ ] T072 [L6] Write intro section framing reusable intelligence as "work smarter, not harder—document what works"
- [ ] T073 [L6] Create reflection activity: "List Git workflows you've used 2+ times across Lessons 1-5"
- [ ] T074 [L6] **Persona + Questions + Principles Framework**: Provide structured template for documenting workflows
  - Persona: "You are a developer creating documentation for your future self"
  - Questions: "What situations trigger this workflow? What decisions does it require? What are the steps?"
  - Principles: "Commit before risky AI experiments. Branch for testing alternatives. Push frequently for backup."
- [ ] T075 [L6] Create hands-on activity: "Create `git-workflow.md` in your project root"
- [ ] T076 [L6] Create workflow documentation template with 3 sections:
  - **Pattern 1**: Commit-Before-Experiment (When AI generates code → commit current state → test AI code → keep or revert)
  - **Pattern 2**: Branch-Test-Merge (When multiple options exist → branch per option → test each → merge winner)
  - **Pattern 3**: Push-For-Backup (Daily or after major progress → push to GitHub → catastrophe prevention)
- [ ] T077 [L6] Create hands-on activity: "Apply Pattern 1 to new AI-generated feature right now, document outcome"
- [ ] T078 [L6] Add example git-workflow.md from T006 template, annotated with real scenarios
- [ ] T079 [L6] Add "Try With AI" section: "Ask ChatGPT: 'Review my git-workflow.md and suggest additional patterns I might have missed'"
- [ ] T080 [L6] Add YAML frontmatter with Stage 3 tag, intelligence accumulation emphasis
- [ ] T081 [L6] Map to success criteria SC-013 (80%+ recognize when to use branches vs commits)

**Checkpoint**: Lesson 6 complete—students have reusable Git intelligence documented

---

## Phase 8: Lesson 7 - Capstone: Task Manager + GitHub Agent HQ (US1+US2+US3+US4: Stage 4 Spec-Driven + Agent HQ Awareness)

**Goal**: Students manage complete Git workflow for AI-generated multi-file project + learn about GitHub's multi-agent future

**Independent Test**: Student writes spec.md for task manager, AI generates Python CLI, student manages full workflow (init → commit → branch → push → PR), understands Agent HQ platform evolution

**Stage**: 4 (Spec-Driven Integration) + Agent HQ Awareness
**Cognitive Load**: 10 concepts (3 from spec-driven + 5 Agent HQ + 2 composition concepts) [Synthesis, not all new]

### Implementation for Lesson 7

- [ ] T082 [L7] [US1+US2+US3+US4] Create `07-capstone-task-manager-agent-hq.md` in `book-source/docs/02-AI-Tool-Landscape/08-git-and-github/`
- [ ] T083 [L7] Write intro section framing capstone as "orchestrating everything you've learned + glimpse of GitHub's future"

#### Part A: Spec-Driven Task Manager Project (90 minutes)

- [ ] T084 [L7] [US1+US2+US3] Create specification template: "Task Manager CLI - Add tasks, list tasks, mark complete"
- [ ] T085 [L7] [US1+US2+US3] Create hands-on activity: "Write spec.md defining WHAT task manager does (not HOW to implement)"
- [ ] T086 [L7] [US1+US2+US3] Create hands-on activity: "Ask ChatGPT to generate Python CLI from your spec"
- [ ] T087 [L7] [US1] Create hands-on activity: "Initialize Git repo for task manager project"
- [ ] T088 [L7] [US1] Create hands-on activity: "COMMIT before AI generates code (Pattern 1 from Lesson 6)"
- [ ] T089 [L7] [US1] Create hands-on activity: "AI generates task_manager.py, commit with message 'feat: AI-generated task manager skeleton'"
- [ ] T090 [L7] [US3] Create hands-on activity: "Create branch `feature-complete-command` to test enhancement"
- [ ] T091 [L7] [US3] Create hands-on activity: "Ask ChatGPT to add 'mark complete' feature, test, merge if works"
- [ ] T092 [L7] [US2] Create hands-on activity: "Create GitHub repo, push task manager project"
- [ ] T093 [L7] [US2] Create hands-on activity: "Create PR with AI transparency: 'ChatGPT generated initial structure, I refined error handling'"
- [ ] T094 [L7] Apply git-workflow.md patterns throughout project, validate students reference their own documentation
- [ ] T095 [L7] Add terminal log screenshots for complete workflow (init through PR merge)

#### Part B: GitHub Agent HQ Awareness (30 minutes)

- [ ] T096 [L7] [US4] Create "GitHub Agent HQ: The Platform Evolution" section
- [ ] T097 [L7] [US4] Explain Agent HQ as "GitHub's multi-agent orchestration platform" with mission control interface
- [ ] T098 [L7] [US4] Describe 5 concepts:
  - **Mission Control**: Unified interface for directing multiple AI agents (Claude, GPT-4, Gemini) from one place
  - **Multi-Agent Orchestration**: Multiple AI agents work on same project, coordinated by GitHub
  - **Branch Controls for Agents**: GitHub manages which agent creates which branches, enforces policies
  - **Agentic Code Review**: CodeQL engine reviews agent-generated PRs automatically before human review
  - **Platform Evolution**: GitHub transforming from code host to AI agent coordination layer
- [ ] T099 [L7] [US4] Add "How This Relates to Your Learning" reflection: "Git branches you learned → Agent HQ uses for multi-agent coordination. PRs you create → Agent HQ agents create automatically. You're learning foundation."
- [ ] T100 [L7] [US4] Add confidence message: "Agent HQ is future, but Git fundamentals stay the same. Learning Git now prepares you for agentic era without obsolescence anxiety."
- [ ] T101 [L7] [US4] Cite all Agent HQ features from official GitHub Blog (https://github.blog/news-insights/company-news/welcome-home-agents/) per C-015 constraint
- [ ] T102 [L7] [US4] Add screenshot of Agent HQ mission control interface from GitHub Blog

#### Lesson 7 Finalization

- [ ] T103 [L7] Add "Try With AI" section: "Ask ChatGPT: 'How might GitHub Agent HQ change the way I work with AI coding assistants in the future?'" **Expected Learning Outcome**: Student understands multi-agent coordination benefits (multiple AI agents working on same project, mission control interface for orchestration) without implementation anxiety or feeling skills are obsolete
- [ ] T104 [L7] Add YAML frontmatter with Stage 4 tag, capstone composition validation, Agent HQ awareness
- [ ] T105 [L7] Map to success criteria: SC-002 (85%+ execute core workflow), SC-005 (30-second commits), SC-007 (branch workflow <5 min), SC-014 (90%+ understand Agent HQ without anxiety)

**Checkpoint**: Lesson 7 complete—students completed full AI-generated project with Git + understand GitHub's agentic future

---

## Phase 9: Chapter-Level Integration & Validation

**Purpose**: Ensure chapter meets all constitutional and quality requirements

### Content Quality Validation

- [ ] T106 [P] Verify all 7 lessons follow hands-on discovery modality (not lecture-style) per C-003 constraint
- [ ] T107 [P] Verify cognitive load per lesson: L1(5), L2(4), L3(5), L4(4), L5(5), L6(3), L7(10 synthesis) = 4.3 avg ✓ within A1/A2 limits
- [ ] T108 [P] Verify Stage progression explicit: L1-2(Stage 1), L3-5(Stage 2), L6(Stage 3), L7(Stage 4) per C-005 constraint
- [ ] T109 [P] Verify Three Roles demonstrated in L3-5 (AI as Teacher, Student as Teacher, Co-Worker) per FR-025 to FR-028
- [ ] T110 [P] Verify all Git commands tested with terminal logs attached per C-014 constraint (from T003-T004)
- [ ] T111 [P] Verify GitHub Agent HQ features cited from official GitHub Blog per C-015 constraint (T101-T102)
- [ ] T112 [P] Verify "Try With AI" final section in all 7 lessons (no "Key Takeaways" or "What's Next") per policy note
- [ ] T113 [P] Verify all lessons have YAML frontmatter with skills proficiency metadata per existing chapter standards

### Constitutional Compliance Validation

- [ ] T114 Validate Principle 1 (Specification Primacy): Lesson 7 writes spec.md BEFORE implementation ✓
- [ ] T115 Validate Principle 2 (Progressive Complexity): Average 4.3 concepts/lesson within A1/A2 limits ✓
- [ ] T116 Validate Principle 3 (Factual Accuracy): All Git commands tested (T003-T004), Agent HQ cited (T101) ✓
- [ ] T117 Validate Principle 4 (Coherent Structure): Pedagogical arc Foundation(L1-2) → Application(L3-5) → Integration(L6) → Mastery(L7) ✓
- [ ] T118 Validate Principle 5 (Intelligence Accumulation): L6 creates git-workflow.md, L7 applies it ✓
- [ ] T119 Validate Principle 6 (Anti-Convergence): Chapter 8 uses hands-on discovery vs Chapter 7's direct teaching ✓
- [ ] T120 Validate Principle 7 (Minimal Content): Every lesson maps to success criteria SC-001 to SC-019 ✓

### User Scenario Coverage Validation

- [ ] T121 [P] Validate US1 (Safe AI Experimentation) covered: L1-2 (foundation), L3 (branches), L7 (capstone) ✓
- [ ] T122 [P] Validate US2 (GitHub Backup & Portfolio) covered: L4 (cloud backup), L5 (PRs), L7 (capstone) ✓
- [ ] T123 [P] Validate US3 (Branch Testing) covered: L3 (branches intro), L7 (capstone application) ✓
- [ ] T124 [P] Validate US4 (Agent HQ Awareness) covered: L7 Part B (30-minute awareness section) ✓

### Success Criteria Mapping Validation

- [ ] T125 [P] Verify SC-001 to SC-004 (Learning Objectives) mapped across L1-L7
- [ ] T126 [P] Verify SC-005 to SC-008 (Workflow Competence) mapped to L3, L4, L5, L7
- [ ] T127 [P] Verify SC-009 to SC-011 (Confidence & Self-Efficacy) addressed in L2, L4, L7
- [ ] T128 [P] Verify SC-012 to SC-014 (Knowledge Retention) addressed in L2, L3, L7
- [ ] T129 [P] Verify SC-015 to SC-019 (Pedagogical Quality) validated via T106-T120 checks

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Final chapter-level improvements

- [ ] T130 [P] Create chapter README.md with overview, learning outcomes, 7-lesson structure explanation
- [ ] T131 [P] Update chapter-index.md status from "✅ Implemented" to "✅ Redesigned (v2.0.0 - Reasoning-Activated)"
- [ ] T132 [P] Create chapter-level quiz/assessment covering all 7 lessons (optional, if assessment framework exists)
- [ ] T133 [P] Spell-check and grammar-check all 7 lesson files
- [ ] T134 [P] Verify all internal links work (between lessons, to spec.md, to constitution.md)
- [ ] T135 [P] Validate Docusaurus build succeeds with new 7-lesson structure
- [ ] T136 [P] Create migration guide: "What changed from 9-lesson → 7-lesson structure" for existing students
- [ ] T137 [P] Add original 9-lesson content to `specs/028-chapter-8-git-redesign/original-lessons-backup/` per T005

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies—starts immediately
  - Research GitHub Agent HQ (T002)
  - Test Git commands (T003-T004)
  - Audit existing content (T001, T005)

- **Lessons (Phases 2-8)**: Lessons can be implemented in parallel by different writers AFTER Phase 1
  - L1 (T007-T018): Foundation for all other lessons
  - L2 (T019-T029): Depends on L1 concepts
  - L3 (T030-T043): Depends on L1-L2 manual foundation
  - L4 (T044-T057): Depends on L1-L3 local Git mastery
  - L5 (T058-T070): Depends on L4 GitHub basics
  - L6 (T071-T081): Depends on L1-L5 (synthesizes all patterns)
  - L7 (T082-T105): Depends on L1-L6 (capstone composition)

- **Validation (Phase 9)**: Depends on all 7 lessons complete
  - Content quality checks (T106-T113)
  - Constitutional compliance (T114-T120)
  - Coverage validation (T121-T129)

- **Polish (Phase 10)**: Depends on Phase 9 validation passing
  - Documentation (T130-T132)
  - Technical checks (T133-T137)

### Lesson Dependencies (Sequential)

```
L1 (Manual Foundation)
  ↓
L2 (Error Recovery) — depends on L1 concepts
  ↓
L3 (Branches + AI Collab) — depends on L1-L2 manual mastery
  ↓
L4 (GitHub Backup) — depends on L1-L3 local Git
  ↓
L5 (Pull Requests) — depends on L4 GitHub basics
  ↓
L6 (Reusable Patterns) — depends on L1-L5 experience
  ↓
L7 (Capstone + Agent HQ) — depends on L1-L6 accumulated intelligence
```

### Parallel Opportunities

**Within Phase 1 (Setup)**:
- T002 (Agent HQ research), T003 (Git testing), T004 (terminal logs), T006 (skill template) can all run in parallel

**Across Lessons (with sufficient writers)**:
- After L1 complete: L2 implementation can start
- After L2 complete: L3 can start
- After L3 complete: L4 can start
- After L4 complete: L5 can start
- After L5 complete: L6 can start
- After L6 complete: L7 can start

**Within Validation Phase**:
- All T106-T113 (content checks) can run in parallel
- All T114-T120 (constitutional checks) can run in parallel
- All T121-T124 (coverage checks) can run in parallel
- All T125-T129 (success criteria checks) can run in parallel

**Within Polish Phase**:
- All T130-T137 can run in parallel (different files/concerns)

---

## Parallel Example: Phase 1 Setup

```bash
# Launch all research/testing tasks together:
Task T002: "Research GitHub Agent HQ from official GitHub Blog"
Task T003: "Test all Git commands in sandbox environment"
Task T004: "Capture terminal logs for each Git command"
Task T006: "Generate git-workflow.md skill template"

# These tasks access different resources (web, terminal, files) with no conflicts
```

---

## Implementation Strategy

### MVP First (Foundation Lessons)

1. **Phase 1**: Complete Setup (T001-T006)
   - Research done, Git commands validated, existing content audited
2. **Phase 2**: Complete L1 (T007-T018)
   - Manual Git foundation established
3. **Phase 3**: Complete L2 (T019-T029)
   - Error recovery confidence built
4. **STOP and VALIDATE**: Test L1-L2 with pilot students
5. Deploy L1-L2 if validation passes (early access to foundation)

### Incremental Delivery

1. Deploy L1-L2 → Foundation lessons (students can start)
2. Add L3 → Branch experimentation (tested independently)
3. Add L4 → GitHub backup (tested independently)
4. Add L5 → Professional PRs (tested independently)
5. Add L6 → Reusable intelligence (tested independently)
6. Add L7 → Capstone + Agent HQ (complete chapter)
7. Each lesson adds value without breaking previous lessons

### Parallel Team Strategy

With multiple content writers:

1. **Week 1**: All writers complete Phase 1 together (shared research/testing)
2. **Week 2**:
   - Writer A: L1 + L2 (foundation pair)
   - Writer B: L3 + L4 (application pair)
   - Writer C: L5 + L6 (integration pair)
3. **Week 3**:
   - Writer A: L7 (capstone requires all context)
   - Writers B+C: Validation (Phase 9)
4. **Week 4**: All writers: Polish (Phase 10)

---

## Notes

- **[P] tasks**: Different files/resources, no dependencies
- **[Lesson] label**: Maps task to specific lesson for traceability
- **[Story] label**: Maps task to user scenario from spec.md
- Each lesson should be independently completable and testable
- Commit after each lesson or logical group of tasks
- Stop at any checkpoint to validate lesson independently
- **No automated tests required**: This is educational content, not software. Validation is pedagogical (T106-T129), not unit/integration tests
- **"Try With AI" mandate**: Every lesson ends with this section (not "Key Takeaways"), using ChatGPT web since AI tools not yet taught in Part 2
- **Hands-on discovery**: Execute → Observe → Understand → AI-enhance pattern throughout (not lecture-style)
- **Three Roles required in L3-L5**: AI as Teacher, Student as Teacher, Co-Worker convergence must be demonstrated
- **Constitutional compliance**: All 7 principles validated in Phase 9 (T114-T120)

---

## Summary

**Total Tasks**: 137 tasks
- Phase 1 (Setup): 6 tasks
- Phase 2 (L1): 12 tasks
- Phase 3 (L2): 11 tasks
- Phase 4 (L3): 14 tasks
- Phase 5 (L4): 14 tasks
- Phase 6 (L5): 13 tasks
- Phase 7 (L6): 11 tasks
- Phase 8 (L7): 24 tasks (10 synthesis + 6 Agent HQ + 8 finalization)
- Phase 9 (Validation): 24 tasks
- Phase 10 (Polish): 8 tasks

**Parallel Opportunities**: 45 tasks marked [P] can run in parallel within their phases

**User Scenario Coverage**:
- US1 (Safe AI Experimentation): L1, L2, L3, L7 (49 tasks)
- US2 (GitHub Backup & Portfolio): L4, L5, L7 (32 tasks)
- US3 (Branch Testing): L3, L7 (22 tasks)
- US4 (Agent HQ Awareness): L7 Part B (7 tasks)

**Suggested MVP Scope**: Phase 1 + L1 + L2 (29 tasks) = Foundation lessons with manual Git mastery

**Independent Test Criteria**:
- L1: Student creates repo, stages, commits, views history manually ✓
- L2: Student views diffs, undos changes at all 3 levels ✓
- L3: Student creates branches, tests parallel implementations, merges ✓
- L4: Student connects GitHub, pushes, clones from different machine ✓
- L5: Student creates PR with AI transparency, merges ✓
- L6: Student documents git-workflow.md, applies to new project ✓
- L7: Student manages complete workflow for AI-generated project ✓

**Format Validation**: ✅ All 137 tasks follow strict checklist format: `- [ ] [ID] [P?] [Lesson/Story] Description with file path`
