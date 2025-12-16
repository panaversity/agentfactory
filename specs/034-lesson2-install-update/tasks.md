# Tasks: Update Chapter 5 Lesson 2 Claude Code Installation

**Input**: Design documents from `/specs/034-lesson2-install-update/`
**Prerequisites**: spec.md (6 user stories), plan.md (5 implementation phases)

**Tests**: No automated tests required (educational content validation is manual)

**Organization**: Tasks are grouped by user story to enable independent validation of each platform's installation guidance.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different sections, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Windows, US2=macOS, US3=Linux/WSL, US4=npm, US5=Auth, US6=Summary)
- Include exact file paths in descriptions

## Path Conventions

- **Main lesson**: `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md`
- **Summary**: `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md.summary.md`

---

## Phase 1: Setup & Research (1-2 hours)

**Purpose**: Verify official documentation and extract exact commands for all platforms

- [ ] T001 Fetch official Claude Code installation docs from https://code.claude.com/docs/en/setup via WebFetch tool
- [ ] T002 Extract exact installation commands for Windows (PowerShell, curl/bash for Git Bash, CMD)
- [ ] T003 [P] Extract exact installation commands for macOS (Homebrew, curl/bash)
- [ ] T004 [P] Extract exact installation commands for Linux/WSL (curl/bash)
- [ ] T005 [P] Extract npm installation command and Node.js 18+ requirement
- [ ] T006 Extract authentication flows for Console API, Claude App (Pro/Max), and Enterprise
- [ ] T007 [P] Document system requirements (OS versions, RAM, Node.js optional, ripgrep)
- [ ] T008 [P] Document edge cases (Alpine Linux, geo-restrictions, auto-update disable)
- [ ] T009 [P] Document verification commands (`claude --version`, `claude doctor`)

**Gate**: User approves research findings before Phase 2

---

## Phase 2: Foundational Updates (Preserve Existing Structure)

**Purpose**: Update frontmatter and preserve critical sections before platform-specific changes

**⚠️ CRITICAL**: These sections must be preserved/updated before installation section restructure

- [ ] T010 Read current lesson file `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md` to understand existing structure
- [ ] T011 Verify frontmatter YAML metadata (title, sidebar_position: 2, chapter: 5, lesson: 2, duration_minutes: 18, pedagogical layers, skills, learning_objectives, cognitive_load)
- [ ] T012 [P] Update Prerequisites section: Add "Location: Anthropic supported countries" note, clarify "Node.js 18+ (only required for npm installation)"
- [ ] T013 [P] Preserve "Why This Matters: Terminal Integration for AI Workflows" section (no changes needed per spec)
- [ ] T014 [P] Preserve "🔀 Two Professional Paths Available" section referencing Lesson 3 (no changes needed)

**Checkpoint**: Foundational structure preserved - platform-specific updates can now begin in parallel

---

## Phase 3: User Story 1 - Windows Student Installation (Priority: P1) 🎯

**Goal**: Windows students can identify Windows section, follow PowerShell installation (recommended), and find alternatives (Git Bash, CMD, npm)

**Independent Test**: Windows student reads Windows section, installs via PowerShell, runs `claude --version`, sees version number without consulting external docs

**Tasks**:

- [ ] T015 [US1] Create "Installation: Windows" section with heading hierarchy (## Installation → ### Windows)
- [ ] T016 [US1] Add Windows decision tree visual: "Have PowerShell? ├─ Yes → Use PowerShell │ └─ No → Git Bash or CMD"
- [ ] T017 [US1] Document PowerShell installation: command `irm https://claude.ai/install.ps1 | iex`, "What this does" explanation, expected output
- [ ] T018 [US1] Document curl/bash (Git Bash) installation: command `curl -fsSL https://claude.ai/install.sh | bash`, "What this does", when to use (Git for Windows installed)
- [ ] T019 [US1] Document CMD installation: command `curl -fsSL https://claude.ai/install.cmd -o install.cmd && install.cmd && del install.cmd`, "What this does", when to use (no PowerShell/Git Bash)
- [ ] T020 [US1] Add npm alternative for Windows: command `npm install -g @anthropic-ai/claude-code`, prerequisites (Node.js 18+), warning against `sudo npm`
- [ ] T021 [US1] Add Windows-specific troubleshooting: PowerShell execution policy, Git Bash path configuration (`$env:CLAUDE_CODE_GIT_BASH_PATH`)
- [ ] T022 [US1] Add verification step for Windows: `claude --version` command with expected output format

---

## Phase 4: User Story 2 - macOS Student Installation (Priority: P1) 🎯

**Goal**: macOS students can identify macOS section, choose Homebrew (if installed) or curl/bash, and complete installation

**Independent Test**: macOS student reads macOS section, installs via Homebrew (or curl/bash if no Homebrew), runs `claude --version`, sees version

**Tasks**:

- [ ] T023 [P] [US2] Create "Installation: macOS" section with heading hierarchy (### macOS)
- [ ] T024 [P] [US2] Add macOS decision tree: "Have Homebrew? ├─ Yes → Use Homebrew │ └─ No → Use curl/bash"
- [ ] T025 [P] [US2] Document Homebrew installation: command `brew install --cask claude-code`, "What this does", when to use (Homebrew already installed)
- [ ] T026 [P] [US2] Document curl/bash installation for macOS: command `curl -fsSL https://claude.ai/install.sh | bash`, "What this does", when to use (no Homebrew or prefer official script)
- [ ] T027 [P] [US2] Add npm alternative for macOS: same command as Windows, Node.js 18+ prerequisite
- [ ] T028 [P] [US2] Add macOS-specific troubleshooting: Code signing verification ("Anthropic PBC"), notarization by Apple
- [ ] T029 [P] [US2] Add verification step for macOS: `claude --version` and `claude doctor` commands

---

## Phase 5: User Story 3 - Linux/WSL Student Installation (Priority: P1) 🎯

**Goal**: Linux/WSL students can identify Linux section, use curl/bash installation, verify with `claude doctor`

**Independent Test**: Ubuntu/Debian/WSL student reads Linux section, installs via curl/bash, runs `claude doctor`, sees system compatibility

**Tasks**:

- [ ] T030 [P] [US3] Create "Installation: Linux/WSL" section with heading hierarchy (### Linux/WSL)
- [ ] T031 [P] [US3] Document curl/bash installation for Linux: command `curl -fsSL https://claude.ai/install.sh | bash`, "What this does", sudo NOT required
- [ ] T032 [P] [US3] Add npm alternative for Linux: same command, Node.js 18+ prerequisite
- [ ] T033 [P] [US3] Add Linux-specific edge case: Alpine Linux requirements (`libgcc`, `libstdc++`, `ripgrep`, `USE_BUILTIN_RIPGREP=0`)
- [ ] T034 [P] [US3] Document WSL support: WSL 1 and WSL 2 both supported, same curl/bash method as native Linux
- [ ] T035 [P] [US3] Add verification step for Linux/WSL: `claude --version` and `claude doctor` showing installation type

---

## Phase 6: User Story 4 - Cross-Platform npm Alternative (Priority: P2)

**Goal**: Students with Node.js 18+ can find npm installation as alternative on any platform

**Independent Test**: Student with Node.js sees npm listed as alternative (not primary) method, installs successfully, sees security warning

**Tasks**:

- [ ] T036 [P] [US4] Verify npm installation documented in all 3 platform sections (Windows, macOS, Linux/WSL) as "Alternative: npm"
- [ ] T037 [P] [US4] Add npm security warning to each platform: "NEVER use `sudo npm install -g` - causes permission issues and security risks"
- [ ] T038 [P] [US4] Add npm troubleshooting section: Fixing npm permissions without sudo, link to npm docs
- [ ] T039 [P] [US4] Update Prerequisites section: "Node.js 18+ (only required for npm installation)" to clarify optional

---

## Phase 7: User Story 5 - Authentication Setup (Priority: P1) 🎯

**Goal**: Students can identify which of 3 authentication methods applies to them, complete auth flow, verify with test command

**Independent Test**: Student identifies auth method (Console/Claude App/Enterprise), authenticates, runs `claude "Hello!"`, gets response

**Tasks**:

- [ ] T040 [US5] Create "Authentication: Which Method Should I Use?" section with decision tree
- [ ] T041 [US5] Add decision tree visual: "Have Console account with billing? → Console API | Have Pro/Max subscription? → Claude App | Enterprise setup? → See third-party integrations"
- [ ] T042 [US5] Document Console API authentication (Method A): Run `claude`, select Option 2 (Console), OAuth flow, workspace creation, expected output
- [ ] T043 [US5] Document Claude App authentication (Method B): Run `claude`, select Option 1 (Claude account), login with Claude.ai credentials, unified subscription
- [ ] T044 [US5] Document Enterprise authentication (Method C): Brief description, link to https://code.claude.com/docs/en/third-party-integrations, not detailed steps
- [ ] T045 [US5] Add verification test command for all methods: `claude "Hello! Can you confirm Claude Code is working?"` with expected Claude response
- [ ] T046 [US5] Update authentication section to expand from 2 paths to 3 paths while preserving existing Console/Claude App explanations

---

## Phase 8: User Story 6 - Lesson Summary Update (Priority: P2)

**Goal**: Summary file accurately reflects new platform-specific installation methods and can be used standalone

**Independent Test**: External reviewer follows summary-only instructions, successfully installs Claude Code, knows when to check main lesson

**Tasks**:

- [ ] T047 [US6] Read current summary file `apps/learn-app/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/02-installation-and-authentication.md.summary.md`
- [ ] T048 [US6] Rewrite summary Introduction: Platform-specific installation methods (Windows/macOS/Linux), authentication paths (3)
- [ ] T049 [US6] Update Key Concepts section: Platform identification, recommended method per platform, 3 authentication types
- [ ] T050 [US6] Update Critical Patterns section: Decision trees (platform selection, auth method selection), verification commands
- [ ] T051 [US6] Update Common Mistakes section: Using wrong platform's method, forgetting verification, `sudo npm` warning
- [ ] T052 [US6] Add Quick Reference table: Platform → Recommended Method → Verification Command
- [ ] T053 [US6] Verify summary is standalone: Can student install using only summary? When do they need main lesson (troubleshooting)?

---

## Phase 9: Cross-Cutting Updates (All Stories)

**Purpose**: Updates that affect multiple sections or entire lesson cohesion

- [ ] T054 [P] Update "Try With AI" prompts to reflect new installation methods: Remove npm-centric prompts, add platform-specific examples
- [ ] T055 [P] Validate "Try With AI" section against Constitution v6.0.1 meta-commentary prohibition: No "What to notice", no "AI is teaching you", no role labels
- [ ] T056 [P] Add "Auto-Update Documentation" to Security & Best Practices section: Auto-update mechanism, `DISABLE_AUTOUPDATER=1`, settings.json
- [ ] T057 [P] Update duration metadata if content grew significantly (target: 18-22 minutes acceptable per plan)
- [ ] T058 [P] Verify all installation commands use code syntax highlighting (`bash or `powershell)
- [ ] T059 [P] Verify all "What this does" explanations use B1-appropriate vocabulary (no jargon like "package manager" without explanation)

---

## Phase 10: Validation & Quality Checks (2 hours)

**Purpose**: Ensure 100% factual accuracy, constitutional compliance, and pedagogical consistency

**Tasks**:

- [ ] T060 Compare all installation commands against official docs (https://code.claude.com/docs/en/setup) - verify exact match (0 discrepancies)
- [ ] T061 [P] Verify system requirements match official docs (Windows 10+, macOS 10.15+, Ubuntu 20.04+/Debian 10+, WSL 1/2, 4GB+ RAM)
- [ ] T062 [P] Check authentication flows match official docs (Console workspace creation, Claude App unified subscription, Enterprise link)
- [ ] T063 [P] Cognitive load count: Verify total concepts ≤10 for B1 (current plan: 8 concepts)
- [ ] T064 [P] Constitutional compliance check: Layer 1 preserved (no spec-first), no Three Roles exposure, no meta-commentary in "Try With AI"
- [ ] T065 [P] Cross-reference verification: Lesson 1 prerequisite context intact, Lesson 3 cross-reference present, Lesson 4 assumes working installation
- [ ] T066 [P] Pedagogical consistency: Installation is manual (Layer 1), AI collaboration comes later (Lessons 4-10)
- [ ] T067 Test installation commands on platforms (if possible): Windows PowerShell, macOS Homebrew, Ubuntu curl/bash, WSL
- [ ] T068 [P] Full read-through for flow and cohesion: Does updated content feel integrated or patchwork? Section transitions smooth?
- [ ] T069 [P] Verify frontmatter metadata unchanged except duration (if adjusted)

---

## Phase 11: Final Polish & Completion (1 hour)

**Purpose**: Final checks before merge

- [ ] T070 Spell-check and grammar review entire lesson
- [ ] T071 [P] Verify all links work (official docs, Lesson 1, Lesson 3, prerequisites)
- [ ] T072 [P] Check MDX compatibility (Docusaurus 3.x) - no syntax errors
- [ ] T073 [P] Verify image reference still valid: `/img/part-2/chapter-05/claude-code-installation-authentication-flow.png` (note if needs update)
- [ ] T074 Final approval checkpoint: All user stories complete, all success criteria validated, ready for commit

---

## Dependencies & Parallel Execution

### Dependency Graph (User Story Completion Order)

```
Phase 1 (Setup) ──┬──> Phase 2 (Foundational)
                  │
                  └──> Phase 3 (US1: Windows)    ─┐
                       Phase 4 (US2: macOS)      ─┼──> Phase 9 (Cross-Cutting)
                       Phase 5 (US3: Linux/WSL)  ─┤
                       Phase 6 (US4: npm)        ─┤
                       Phase 7 (US5: Auth)       ─┤
                       Phase 8 (US6: Summary)    ─┘
                                                   │
                                                   └──> Phase 10 (Validation) ──> Phase 11 (Polish)
```

### Independent User Stories (Can Implement in Parallel)

- **US1 (Windows)**, **US2 (macOS)**, **US3 (Linux/WSL)** are fully independent (different sections)
- **US4 (npm)** verifies npm present in all 3 platform sections (light dependency on US1/US2/US3 completion)
- **US5 (Auth)** is independent (separate section)
- **US6 (Summary)** depends on all others being complete (must match main lesson)

### Parallel Execution Opportunities

**After Phase 2 completes, these can run in parallel**:

- Phase 3 (Windows section) + Phase 4 (macOS section) + Phase 5 (Linux section) + Phase 7 (Auth section)
- Phase 6 (npm verification) starts after Phase 3/4/5
- Phase 8 (Summary) starts after all content phases complete
- Phase 9 (Cross-cutting) starts after all content phases complete

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)

**Core Value**: Students can install Claude Code on their platform
**MVP = User Stories 1, 2, 3, 5** (Windows, macOS, Linux, Auth)

**Rationale**:

- US1/US2/US3 (P1): Core installation for all platforms
- US5 (P1): Authentication required for working setup
- US4 (P2): npm alternative can be added after MVP
- US6 (P2): Summary update can follow main lesson completion

### Incremental Delivery

**Sprint 1** (MVP - 6 hours):

- Phase 1: Setup & Research (1-2 hrs)
- Phase 2: Foundational (0.5 hrs)
- Phase 3-5: Windows, macOS, Linux sections (3 hrs)
- Phase 7: Authentication (1 hr)
- Phase 10: Critical validation (factual accuracy, constitutional) (1 hr)

**Sprint 2** (Polish - 2-3 hours):

- Phase 6: npm alternative (0.5 hrs)
- Phase 8: Summary update (1 hr)
- Phase 9: Cross-cutting (0.5 hrs)
- Phase 10-11: Full validation + polish (1 hr)

---

## Task Summary

**Total Tasks**: 74
**Parallelizable**: 45 tasks (61%) marked with [P]

**Tasks per User Story**:

- Setup & Research (Phase 1): 9 tasks
- Foundational (Phase 2): 5 tasks
- US1 (Windows): 8 tasks
- US2 (macOS): 7 tasks
- US3 (Linux/WSL): 6 tasks
- US4 (npm): 4 tasks
- US5 (Auth): 7 tasks
- US6 (Summary): 7 tasks
- Cross-Cutting (Phase 9): 6 tasks
- Validation (Phase 10): 10 tasks
- Polish (Phase 11): 5 tasks

**Independent Test Criteria**:

- US1: Windows student installs via PowerShell, verifies with `claude --version`
- US2: macOS student installs via Homebrew, verifies with `claude --version`
- US3: Linux student installs via curl/bash, verifies with `claude doctor`
- US4: Student with Node.js finds npm as alternative, sees security warning
- US5: Student authenticates via appropriate method, tests with `claude "Hello!"`
- US6: External reviewer uses summary standalone, knows when to check main lesson

**Parallel Opportunities**:

- Platform sections (Windows/macOS/Linux) can be written simultaneously
- Validation tasks (factual accuracy, constitutional, cross-reference) can run in parallel
- Cross-cutting updates (Try With AI, auto-update docs) can be done independently

---

## Notes for Implementation

1. **CLI-First**: All commands extracted from official docs via WebFetch (Task T001)
2. **Doc Lookup**: No external doc lookup needed (educational content, not code implementation)
3. **Factual Accuracy**: Phase 10 validates 100% accuracy against https://code.claude.com/docs/en/setup
4. **Constitutional Compliance**: Task T055 validates "Try With AI" against v6.0.1 meta-commentary prohibition
5. **Pedagogical Layer**: Task T066 validates Layer 1 (Manual Foundation) preservation
6. **Format Consistency**: No canonical source lookup needed (installation docs are unique to this lesson)

---

## Policy Note for Lesson Authors

Within this chapter, each lesson must end with a single final section titled "Try With AI" (no "Key Takeaways" or "What's Next"). Before AI tools are taught (e.g., Part-1), use ChatGPT web in that section; after tool onboarding, instruct learners to use their preferred AI companion tool (e.g., Gemini CLI, Claude CLI), optionally providing CLI and web variants.
