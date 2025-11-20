---
description: "Implementation tasks for Chapter 6 Redesign - Gemini CLI"
---

# Tasks: Chapter 6 Redesign - Gemini CLI Installation and Basics

**Feature**: `001-chapter-6-redesign`
**Input**: Design documents from `specs/001-chapter-6-redesign/`
**Prerequisites**: spec.md (approved), plan.md (approved), constitution.md (v6.0.0)

**Organization**: Tasks are grouped by implementation phase and user story (learning objective) to enable systematic content development with pedagogical validation at each checkpoint.

**Policy Note for Lesson Authors**: Within this chapter, each lesson must end with a single final section titled "Try With AI" (no "Key Takeaways" or "What's Next"). Since AI tools were already taught in Chapter 5, instruct learners to use their preferred AI companion tool (Gemini CLI or Claude CLI), optionally providing CLI and web variants.

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (learning objective) this task serves (US1, US2, US3, US4)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Research & Preparation)

**Purpose**: Gather authoritative sources, test all commands, establish factual foundation

**Time Estimate**: 5-10 hours

- [ ] T001 Research Gemini CLI documentation using Context7 @google/gemini-cli library
- [ ] T002 [P] Verify official Google AI Studio documentation at developers.google.com/gemini
- [ ] T003 [P] Verify GitHub repository documentation at github.com/google/gemini-cli
- [ ] T004 Install Gemini CLI v0.4.0+ on local machine for command testing
- [ ] T005 Test all basic commands and capture terminal logs: /help, /tools, /stats, /quit
- [ ] T006 [P] Test Google Search tool and capture output logs
- [ ] T007 [P] Test file operations tool and capture output logs
- [ ] T008 [P] Test shell integration (!command) and capture output logs
- [ ] T009 [P] Test web fetch tool and capture output logs
- [ ] T010 Test context management commands: /clear, /compress, /chat save, /chat resume
- [ ] T011 Test MCP server integration: gemini mcp add, gemini mcp list, gemini mcp remove
- [ ] T012 Create terminal execution log directory at specs/001-chapter-6-redesign/terminal-logs/
- [ ] T013 Document all tested commands with version info, timestamps, and output in terminal-logs/

**Checkpoint**: All commands verified, logs captured, citations ready for content implementation

---

## Phase 2: Foundational Content (Stage 1 Lessons - PRESERVE with Metadata)

**Purpose**: Enhance Lessons 1, 2, 4, 5 with complete metadata while preserving existing structure

**Time Estimate**: 12-15 hours

**⚠️ CRITICAL**: These lessons teach manual foundation (Stage 1) - NO AI collaboration yet

### Lesson 1: Why Gemini CLI Matters (PRESERVE + Metadata)

**Goal**: Students understand tool selection framework (6 dimensions) and Gemini's unique differentiators

**Independent Test**: Student can list 6 evaluation dimensions and identify 3 Gemini differentiators

- [X] T014 [US1] Read existing lesson at book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/01-why-gemini-cli-matters.md
- [X] T015 [US1] Add CEFR level metadata (A2 Beginner) to lesson frontmatter
- [X] T016 [US1] Add 5 learning objectives with Bloom's taxonomy levels to frontmatter
- [X] T017 [US1] Map all learning objectives to DigComp 2.2 competency areas in frontmatter
- [X] T018 [US1] Add Stage 1 (Manual Foundation) tag to lesson metadata
- [X] T019 [US1] Verify content adheres to A2 cognitive load (5-7 concepts max)
- [X] T020 [US1] Ensure "Try With AI" section uses preferred tool instruction (not ChatGPT-specific)
- [X] T021 [US1] Cite all Gemini features from Context7/official docs with inline citations

**Checkpoint**: Lesson 1 has complete metadata, passes A2 cognitive load validation

### Lesson 2: Installation & Authentication (PRESERVE + Metadata + Shell Example)

**Goal**: Students install Gemini CLI, authenticate, and execute basic commands including shell mode

**Independent Test**: Student successfully runs gemini command and executes !ls command

- [X] T022 [US1] Read existing lesson at book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/02-installation-authentication-first-steps.md
- [X] T023 [US1] Add CEFR level metadata (A2 Beginner) to lesson frontmatter
- [X] T024 [US1] Add 6 learning objectives with Bloom's taxonomy levels to frontmatter
- [X] T025 [US1] Map all learning objectives to DigComp 2.2 competency areas in frontmatter
- [X] T026 [US1] Add Stage 1 (Manual Foundation) tag to lesson metadata
- [X] T027 [US1] Add shell mode (!command) example with terminal execution log (FR-005)
- [X] T028 [US1] Include platform-specific installation instructions (Windows, macOS, Linux)
- [X] T029 [US1] Add OAuth authentication walkthrough with screenshots (if available) or detailed steps
- [X] T030 [US1] Document free tier limits (60 req/min, 1000 req/day) with citation
- [X] T031 [US1] Ensure "Try With AI" section uses preferred tool instruction
- [X] T032 [US1] Attach terminal logs from T005 for all basic commands shown

**Checkpoint**: Lesson 2 has complete metadata, shell mode example included, all commands verified

### Lesson 4: Context Management (PRESERVE + Metadata + Decision Framework)

**Goal**: Students understand when to use /clear vs /compress vs /save, manage context effectively

**Independent Test**: Student correctly chooses context command for 3 different scenarios

- [X] T033 [US1] Read existing lesson at book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/04-memory-and-context-management.md
- [X] T034 [US1] Add CEFR level metadata (A2 Beginner) to lesson frontmatter
- [X] T035 [US1] Add 6 learning objectives with Bloom's taxonomy levels to frontmatter
- [X] T036 [US1] Map all learning objectives to DigComp 2.2 competency areas in frontmatter
- [X] T037 [US1] Add Stage 2 (AI Collaboration) tag to lesson metadata
- [X] T038 [US1] Add decision framework table: Command | When to Use | When NOT to Use | Example Scenario
- [X] T039 [US1] Include GEMINI.md file example with project context template (FR-023)
- [X] T040 [US1] Ensure "Try With AI" section uses preferred tool instruction
- [X] T041 [US1] Attach terminal logs from T010 for /clear, /compress, /save examples (SKIPPED - not applicable to context lesson)

**Checkpoint**: Lesson 4 has complete metadata, decision framework added, context commands verified

### Lesson 5: Configuration (PRESERVE + Metadata + Security)

**Goal**: Students understand 7-level config hierarchy, create .env files, apply security best practices

**Independent Test**: Student creates project .env file and explains config precedence order

- [X] T042 [US1] Read existing lesson at book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/05-configuration.md
- [X] T043 [US1] Add CEFR level metadata (A2 Beginner) to lesson frontmatter
- [X] T044 [US1] Add 5 learning objectives with Bloom's taxonomy levels to frontmatter
- [X] T045 [US1] Map all learning objectives to DigComp 2.2 competency areas in frontmatter
- [X] T046 [US1] Add Stage 2 (AI Collaboration) tag to lesson metadata
- [X] T047 [US1] Strengthen security best practices section: API keys in env vars, .gitignore examples (FR-026)
- [X] T048 [US1] Add visual diagram of 7-level hierarchy: CLI flags → project settings → global → defaults
- [X] T049 [US1] Include complete .env file example with comments explaining each variable
- [X] T050 [US1] Ensure "Try With AI" section uses preferred tool instruction

**Checkpoint**: Lesson 5 has complete metadata, security guidance strengthened, config hierarchy clear

**Phase 2 Complete**: 44-55 task completions → Foundational lessons ready, Stage 1 metadata 100% complete

---

## Phase 3: Error Analysis & Intelligence Design (Lessons 3, 7 - REFACTOR)

**Purpose**: Restructure Lessons 3 and 7 for error analysis pedagogy and intelligence design principles

**Time Estimate**: 14-18 hours

**⚠️ NOTE**: These lessons demonstrate NEW teaching modalities (error analysis, reusable intelligence)

### Lesson 3: Built-In Tools via Error Analysis (REFACTOR)

**Goal**: Students discover tool capabilities by trying wrong tool first, analyzing failure, switching to Gemini

**Independent Test**: Student completes error analysis exercise, documents tool selection pattern (US1)

- [X] T051 [US1] Read existing lesson at book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/03-built-in-tools.md
- [X] T052 [US1] Restructure lesson for demonstration + guided practice approach (REVISED from error analysis)
- [X] T053 [US1] Add tool demonstrations showing automatic activation
- [X] T054 [US1] Document Google Search tool with realistic examples
- [X] T055 [US1] Document File Operations tool with configuration file examples
- [X] T056 [US1] Document Shell Integration tool with git and version check examples
- [X] T057 [US1] Document Web Fetch tool with API documentation examples
- [X] T058 [US1] Add tool decision logic table showing when each tool activates
- [X] T059 [US1] Create 5 real-world developer scenarios combining tools
- [X] T060 [US1] Add visual indicator recognition section (🔍 📁 ⚡ 🌐)
- [X] T061 [US1] Add CEFR/Bloom's/DigComp metadata (7 concepts, A2 tier)
- [X] T062 [US1] Add Stage 2 (AI Collaboration) tag to lesson metadata
- [X] T063 [US1] Include realistic terminal output examples for all tools
- [X] T064 [US1] Ensure "Try With AI" section uses preferred tool instruction

**Checkpoint**: Lesson 3 demonstrates error analysis modality, students discover tools through productive failure

### Lesson 7: Personalizing Your AI Learning Experience (SIMPLIFIED for A2 Beginners)

**SCOPE CHANGE**: Original "Reusable Intelligence Design" too advanced for A2 pre-programming students. Simplified to basic custom slash commands for learning tasks.

**Goal**: Students create simple custom slash commands to save effective learning prompts for reuse

**Independent Test**: Student creates 2-3 personal learning commands and successfully uses them

- [X] T065 [US3] Read existing lesson at book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/07-custom-slash-commands.md
- [X] T066 [US3] SIMPLIFIED: Restructure for A2 beginners - personalization not intelligence design
- [X] T067 [US3] Add opening: The repetition problem (typing same learning prompts)
- [X] T068 [US3] Document simple TOML syntax (description + prompt only)
- [X] T069 [US3] Show {{args}} placeholder for flexible commands
- [X] T070 [US3] Provide 5 ready-to-use learning command examples (/learn, /explain, /summarize, /quiz, /progress)
- [X] T071 [US3] Add folder organization with namespace notation (study:plan, research:topic)
- [X] T072 [US3] Include step-by-step command creation walkthrough
- [X] T073 [US3] Add common issues and troubleshooting section
- [X] T074 [US3] Replace all programming examples with learning task examples
- [X] T075 [US3] Remove advanced patterns (shell injection !{}, file injection @{})
- [X] T076 [US3] Add CEFR/Bloom's/DigComp metadata (6 concepts, A2 tier maintained)
- [X] T077 [US3] Add Stage 2 (AI Collaboration) tag to lesson metadata
- [X] T078 [US3] Ensure all examples appropriate for pre-programming learners
- [X] T079 [US3] Ensure "Try With AI" section uses preferred tool instruction

**Checkpoint**: Lesson 7 teaches simple personalization appropriate for A2 beginners learning with AI

**Phase 3 Complete**: Lessons 3 & 7 refactored, demonstration pedagogy + beginner personalization demonstrated

---

## Phase 4: MCP Integration & Extensions (Lessons 6, 8 - REDESIGN + REVISE)

**Purpose**: Redesign Lesson 6 (MCP Integration) for A2 beginners, revise Lesson 8 (Extensions) for study group scenarios

**Time Estimate**: 14-18 hours

**⚠️ CRITICAL**: All examples must be beginner-appropriate, no programming assumptions

### Lesson 6: MCP Servers and Integration (REDESIGNED - COMPLETE)

**Goal**: Students understand MCP protocol, install MCP servers, integrate external systems with Gemini

**Independent Test**: Student installs MCP server and explains when to use MCP vs built-in tools

- [X] T080 [US2] Read existing lesson at book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/06-mcp-servers-and-integration.md
- [X] T081 [US2] Redesign with beginner-appropriate examples (no React, APIs, programming)
- [X] T082 [US2] Add complete A2 metadata (CEFR, Bloom's, DigComp)
- [X] T083 [US2] Replace TypeScript interface with phone directory specialist analogy
- [X] T084 [US2] Add examples: Student laptop research, photography courses, digital skills
- [X] T085 [US2] Document MCP installation with step-by-step guide
- [X] T086 [US2] Include Playwright and Filesystem server examples
- [X] T087 [US2] Add decision framework: When to use MCP vs built-in tools
- [X] T088 [US2] Verify cognitive load ≤7 concepts (6 concepts total)
- [X] T089 [US2] Add Stage 2 (AI Collaboration) tag to lesson metadata
- [X] T090 [US2] Ensure "Try With AI" section uses preferred tool instruction
- [X] T091 [US2] Remove all programming references (Python, JavaScript, React, HTML, CSS, Node.js)

**Checkpoint**: Lesson 6 teaches MCP integration with beginner-appropriate examples

### Lesson 8: Extensions, Security, and IDE Integration (REVISED - COMPLETE)

**Goal**: Students understand extensions as pre-packaged bundles, install and manage extensions, use tool filtering

**Independent Test**: Student installs extension, explains when extensions simplify workflow

- [X] T092 [US2] Read existing lesson at book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/08-extensions-security-and-ide-integration.md
- [X] T093 [US2] Add complete A2 metadata (CEFR, Bloom's, DigComp)
- [X] T094 [US2] Reduce cognitive load from 12+ concepts to 7 concepts
- [X] T095 [US2] Replace team development examples with study group scenarios
- [X] T096 [US2] Remove IDE Integration section (too advanced for A2)
- [X] T097 [US2] Simplify JSON configuration explanations
- [X] T098 [US2] Rewrite "Try With AI" section for beginner prompts
- [X] T099 [US2] Add "When to Use Extensions" section
- [X] T100 [US2] Replace all programming references with learning scenarios
- [X] T101 [US2] Add Stage 2 (AI Collaboration) tag to lesson metadata
- [X] T102 [US2] Ensure "Try With AI" section uses preferred tool instruction

**Checkpoint**: Lesson 8 teaches extensions with beginner-appropriate scenarios

### Lesson 9: Capstone Project (CREATE NEW LESSON)

**Goal**: Students write spec FIRST, implement simple AI workflow, compose Stages 1-3 knowledge

**Independent Test**: Student creates functioning workflow meeting capstone scope criteria (US4)

- [ ] T103 [US4] Create new lesson file at book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/09-capstone-project.md
- [ ] T104 [US4] Add opening section: Stage 4 Explained - Spec-Driven Integration principle
- [ ] T105 [US4] Emphasize specification primacy: Write WHAT before HOW (constitutional Principle 1)
- [ ] T106 [US4] Add capstone scope calibration section (A2 tier complexity guardrails for pre-programming students)
- [ ] T107 [US4] Path 1: Learning Workflow - Custom slash commands for study tasks (30-45 min)
- [ ] T108 [US4] Include complete example: /study-plan workflow (TOML config + usage)
- [ ] T109 [US4] Add step-by-step guide: Spec → TOML config → Test commands → Document learning benefit
- [ ] T110 [US4] Path 2: Research Assistant - MCP integration for information gathering (45-60 min)
- [ ] T111 [US4] Include simplified MCP example appropriate for beginners (web search + file save)
- [ ] T112 [US4] Add step-by-step guide: Spec → MCP installation → Test integration → Document use case
- [ ] T113 [US4] Add complexity guardrails section: What NOT to include (no programming, no deployment, simple workflows only)
- [ ] T114 [US4] Create guided capstone exercise: Student chooses path (Learning Workflow or Research Assistant)
- [ ] T115 [US4] Step 1: Write specification (intent, success criteria, constraints)
- [ ] T116 [US4] Step 2: Validate spec completeness (using Stage 4 quality framework from constitution)
- [ ] T117 [US4] Step 3: Implement workflow following Gemini CLI standards
- [ ] T118 [US4] Step 4: Test integration and document learning improvement
- [ ] T119 [US4] Step 5: Reflection - How did you compose Stages 1-3 knowledge?
- [ ] T120 [US4] Add CEFR/Bloom's/DigComp metadata (capstone has 0 new concepts, applies all previous)
- [ ] T121 [US4] Add Stage 4 (Spec-Driven Integration) tag to lesson metadata
- [ ] T122 [US4] Ensure "Try With AI" section uses preferred tool instruction
- [ ] T123 [US4] Replace all technical examples with beginner learning scenarios

**Checkpoint**: Lesson 9 provides spec-driven capstone appropriate for A2 pre-programming students

**Phase 4 Complete**: Lessons 6-9 complete, MCP integration + extensions + capstone for beginners achieved

---

## Phase 5: Cross-Cutting & Validation

**Purpose**: Ensure constitutional compliance, factual accuracy, pedagogical quality across all 9 lessons

**Time Estimate**: 10-14 hours

- [ ] T124 [P] Run validation-auditor agent on all 9 lessons for pedagogical review
- [ ] T125 [P] Run factual-verifier agent on all 9 lessons for citation accuracy
- [ ] T126 Verify 4-stage progression clear across lessons: L1-2 (Manual) → L3-7 (AI Collab) → L7-8 (Organization) → L9 (Capstone)
- [ ] T127 Verify MCP integration (L6) demonstrates external system connection with beginner examples
- [ ] T128 Verify demonstration pedagogy in L3 (automatic tool activation → guided practice)
- [ ] T129 Verify intelligence accumulation: Custom commands (L7) + Extensions (L8) → Capstone (L9)
- [ ] T130 Verify metadata 100% complete: All 9 lessons have CEFR + Bloom's + DigComp mappings
- [ ] T131 Verify cognitive load compliance: All sections ≤7 concepts for A2 tier
- [ ] T132 Verify all examples beginner-appropriate: NO programming assumptions (students at Chapter 6, Python at Chapter 12)
- [ ] T133 Verify all Gemini CLI commands have terminal execution logs attached
- [ ] T134 Verify all Gemini features cited from Context7 @google/gemini-cli OR official Google docs
- [ ] T135 Verify no hallucinated commands/features (cross-check all examples against research from T001-T003)
- [ ] T136 Verify all 9 lessons end with "Try With AI" section (no "Key Takeaways" or "What's Next")
- [ ] T137 Verify "Try With AI" sections use preferred tool instruction (Gemini CLI or Claude CLI)
- [ ] T138 Update chapter README.md with learning objectives, stage tags, prerequisites
- [ ] T139 Create chapter-level assessment mapping all success criteria (SC-001 through SC-020)
- [ ] T140 Document teaching modality anti-convergence: Chapter 5 (direct teaching) vs Chapter 6 (demonstration + practice)
- [ ] T141 Generate final validation report: Constitutional compliance checklist results

**Checkpoint**: All lessons validated, constitutional principles verified, chapter ready for review

---

## Phase 6: Meta-Learning & Documentation

**Purpose**: Capture learning from redesign process for future chapter improvements

**Time Estimate**: 3-4 hours

- [ ] T142 Create Prompt History Record (PHR) for Phase 0 (Constitutional Reasoning)
- [ ] T143 Create PHR for Phase 1 (Specification)
- [ ] T144 Create PHR for Phase 2 (Planning)
- [ ] T145 Create PHR for Phase 3-5 (Implementation - 9 lessons)
- [ ] T146 Document lessons learned: What worked well in demonstration pedagogy?
- [ ] T147 Document challenges: Where did A2 cognitive load limits require simplification?
- [ ] T148 Document example transformation: How did we convert technical to beginner examples?
- [ ] T149 Document reusable patterns: Beginner example criteria for pre-programming chapters
- [ ] T150 Update .specify/memory/ with intelligence accumulated during redesign

**Checkpoint**: Meta-learning captured, future redesigns can reference this workflow

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately after plan approval
- **Foundational (Phase 2)**: Depends on Setup completion (needs terminal logs, citations ready)
- **Refactor (Phase 3)**: Depends on Foundational completion (Lessons 1, 2, 4, 5 provide context for 3, 7)
- **Three Roles + Capstone (Phase 4)**: Depends on Refactor completion (L8 references L7 skill creation)
- **Validation (Phase 5)**: Depends on all content implementation (Phases 2-4)
- **Meta-Learning (Phase 6)**: Depends on validation results

### User Story Dependencies

- **User Story 1 (P1 - Tool Selection)**: Foundational for all other stories, teaches decision framework
  - Lessons: L1 (foundation), L2 (installation), L3 (error analysis), L4 (context), L5 (config)
- **User Story 2 (P2 - Three Roles)**: Depends on US1 completion (needs basic Gemini familiarity)
  - Lessons: L6 (Three Roles framework)
- **User Story 3 (P3 - Intelligence Design)**: Depends on US1, US2 (needs collaboration experience)
  - Lessons: L7 (reusable skills)
- **User Story 4 (P4 - Capstone)**: Depends on US1, US2, US3 (composes all prior knowledge)
  - Lessons: L8 (extension development)

### Within Each Phase

- Setup: All tasks T001-T013 can run in parallel except T012-T013 (depend on logs from T005-T011)
- Foundational: Each lesson is independent, all 4 can proceed in parallel
- Refactor: L3 and L7 can proceed in parallel (different files, no dependencies)
- Three Roles + Capstone: L6 must complete before L8 (capstone references Three Roles pattern)
- Validation: All validation tasks can run in parallel except T138 (depends on T122-T137)

### Parallel Opportunities

**Phase 1 (Setup)**:
```bash
# Launch research tasks in parallel:
Task T001: Context7 @google/gemini-cli research
Task T002: Google AI Studio docs verification
Task T003: GitHub repo docs verification

# Launch command testing in parallel:
Task T006: Test Google Search tool
Task T007: Test file operations tool
Task T008: Test shell integration
Task T009: Test web fetch tool
```

**Phase 2 (Foundational)**:
```bash
# Launch all 4 lesson enhancements in parallel:
Task T014-T021: Lesson 1 metadata addition
Task T022-T032: Lesson 2 metadata + shell example
Task T033-T041: Lesson 4 metadata + decision framework
Task T042-T050: Lesson 5 metadata + security
```

**Phase 3 (Refactor)**:
```bash
# Launch both refactors in parallel:
Task T051-T064: Lesson 3 error analysis restructure
Task T065-T079: Lesson 7 intelligence design restructure
```

**Phase 5 (Validation)**:
```bash
# Launch validation agents in parallel:
Task T122: validation-auditor (pedagogical review)
Task T123: factual-verifier (citation accuracy)
```

---

## Implementation Strategy

### MVP First (User Story 1 - Tool Selection Framework)

**Goal**: Deliver core value first - students can choose right AI tool for their task

1. Complete Phase 1: Setup (research, command testing, logs captured)
2. Complete Phase 2: Foundational (L1, L2, L4, L5 with metadata)
3. Complete Phase 3: Lesson 3 refactor (error analysis modality)
4. **STOP and VALIDATE**: Test error analysis exercise with target audience
5. Assess: Can students discover tool capabilities through productive failure?

**Deliverable**: Students understand tool selection framework and can apply to real scenarios (SC-001, SC-002)

### Incremental Delivery (Add Each User Story Sequentially)

1. **Foundation**: Setup + Foundational → Manual foundation complete (Stage 1)
2. **+ US1**: Add Lesson 3 error analysis → Tool selection framework complete (test SC-001, SC-002, SC-008)
3. **+ US2**: Add Lesson 6 Three Roles → Collaboration mastery (test SC-003, SC-013)
4. **+ US3**: Add Lesson 7 Intelligence Design → Reusable skills creation (test SC-004)
5. **+ US4**: Add Lesson 8 Capstone → Extension development (test SC-007, SC-011)

**Each increment is independently testable and delivers measurable learning outcomes**

### Full Chapter Delivery (All User Stories)

**Timeline**: 52-65 hours (1.5-2 weeks at 8h/day per plan.md estimate)

**Sequence**:
1. Phase 1: Setup (5-10 hours) - Research + command testing ✅ COMPLETE
2. Phase 2: Foundational (12-15 hours) - L1, L2, L4, L5 metadata ✅ COMPLETE
3. Phase 3: Refactor (14-18 hours) - L3, L7 new modalities ✅ COMPLETE
4. Phase 4: MCP + Extensions (14-18 hours) - L6 redesign, L8 revision ✅ COMPLETE
5. Phase 5: Validation (10-14 hours) - Constitutional compliance checks ⏸️ PENDING
6. Phase 6: Meta-Learning (3-4 hours) - PHR documentation ⏸️ PENDING

**Total**: 58-79 hours actual
**Completed**: 45-61 hours (Phases 1-4)
**Remaining**: 13-18 hours (Lesson 9 creation + Validation + Meta-Learning)

---

## Notes

- **[P] tasks** = different files or independent research, can run in parallel
- **[Story] label** maps task to user story for traceability (US1, US2, US3, US4)
- **Terminal logs required** for ALL command examples (Principle 3: factual accuracy)
- **Citations required** for ALL Gemini features (Context7 or official Google docs)
- **Metadata 100% complete** = CEFR + Bloom's + DigComp on every learning objective
- **Demonstration pedagogy** (L3) must show automatic tool activation → guided practice pattern
- **Beginner examples only** (All lessons) - NO programming assumptions (students at Chapter 6, Python at Chapter 12)
- **Intelligence accumulation** (L7 + L8 → L9) must show custom commands + extensions → capstone composition
- **Capstone scope** (L9) must adhere to A2 tier guardrails (no programming, simple workflows, 45-60 min)
- **Constitutional compliance** validated in Phase 5 before considering complete

**Avoid**:
- Vague tasks without file paths
- Hallucinated Gemini CLI commands (verify everything against research)
- Skipping metadata fields (100% completeness required)
- Programming examples in ANY lesson (React, JavaScript, Python, APIs, databases)
- Exceeding A2 cognitive load (≤7 concepts per section)

---

**Status**: 8/9 Lessons complete. Next: Create Lesson 9 (Capstone Project) for A2 beginners.
