# Tasks: Chapter 1 - Introduction to AI APIs & OpenAI Agents

**Input**: Design documents from `/specs/041-ch01-intro-apis-agents/`
**Prerequisites**: plan.md (1,678 lines), spec.md (450 lines)

**Tests**: Not applicable for educational content (no TDD required)

**Organization**: Tasks grouped by user story (lesson implementation). Each lesson is independently completable and testable.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story (lesson) this task belongs to (US1=L1.1, US2=L1.2, US3=L1.3, US4=L1.4)
- Include exact file paths in descriptions

## Path Conventions

- **Lesson files**: `book-source/docs/06-AI-Native-Software-Development/01-intro-apis-agents/`
- **Starter code**: `book-source/docs/06-AI-Native-Software-Development/01-intro-apis-agents/code-examples/`
- **Assets**: `book-source/static/img/06-AI-Native-Software-Development/01-intro-apis-agents/`

---

## Phase 1: Setup (Project Structure)

**Purpose**: Initialize chapter folder structure and shared resources

- [ ] T001 Create chapter directory structure at `book-source/docs/06-AI-Native-Software-Development/01-intro-apis-agents/`
- [ ] T002 [P] Create README.md for chapter overview with learning objectives, duration (2-3 hours), and lesson index
- [ ] T003 [P] Create code-examples/ subdirectory for starter code files
- [ ] T004 [P] Create assets directory at `book-source/static/img/06-AI-Native-Software-Development/01-intro-apis-agents/` for diagrams

**Checkpoint**: Chapter folder structure ready for lesson content

---

## Phase 2: Foundational (Shared Resources)

**Purpose**: Create shared resources used across multiple lessons

**⚠️ CRITICAL**: These must be complete before lesson content creation

- [ ] T005 Create `test_api.py` starter code template in code-examples/ with 3-line skeleton (import OpenAI, create client, comment for student code)
- [ ] T006 [P] Create comparison table template (Markdown table with headers: "What ChatGPT Can Do" | "What DocuBot Agent Will Do") in code-examples/
- [ ] T007 [P] Create architecture.md template in code-examples/ with 3 section headers (System Overview, Component Descriptions, Chapter Roadmap)
- [ ] T008 [P] Create troubleshooting guide snippet for common API errors (401 auth, network timeout, invalid key) — reused across lessons

**Checkpoint**: Shared resources ready - lesson implementation can begin in parallel

---

## Phase 3: User Story 1 - API Fundamentals (Lesson 1.1) (Priority: P1) 🎯 MVP

**Goal**: Students understand APIs via restaurant analogy and verify OpenAI API key works

**Independent Test**: Student runs `test_api.py`, sees AI-generated response in terminal, confirms API key authentication

**Lesson File**: `01-api-fundamentals.md`

### Implementation for Lesson 1.1

- [ ] T009 [P] [US1] Create lesson frontmatter (title, duration: 30-40 min, learning goal, sidebar position) in `01-api-fundamentals.md`
- [ ] T010 [P] [US1] Write "Learning Goal" section (2 min read): "Understand what APIs are using restaurant analogy, verify OpenAI API key works"
- [ ] T011 [P] [US1] Write "What You'll Learn (Concept)" section (5 min read): Explain APIs as communication bridges, request/response cycle, API keys, JSON format
- [ ] T012 [US1] Write "Key Points" section (3 min read): 4 bullet points covering APIs, requests, authentication, JSON (must use clear language for A1-A2 proficiency)
- [ ] T013 [US1] Write "Simple Analogy" section (2 min read): **THE RESTAURANT ANALOGY** - You (code) → Waiter (API) → Kitchen (OpenAI server) → Response (AI text). Use callout box formatting.
- [ ] T014 [US1] Write "Code Level" section (1 min read): Explain this lesson has minimal code (3-5 lines), just API verification, no agent complexity yet
- [ ] T015 [US1] Write "🤖 Apply to DocuBot Project" section with **Task** subsection (numbered steps: 1) Import OpenAI, 2) Create client, 3) Make chat completion call, 4) Print response) and **Outcome** subsection in italics: "*Confirmed working API key. You see a response from OpenAI, proving your setup is correct.*"
- [ ] T016 [US1] Write "💡 Hints" section with 5 progressive hints (Hint 1: Check OPENAI_API_KEY in .env → Hint 5: response.choices[0].message.content). Follow graduated disclosure: conceptual → directional → syntax pointer → near-complete solution.
- [ ] T017 [US1] Write "📝 Starter Code" section: Include Python code block with `from openai import OpenAI\n\nclient = OpenAI()\n\n# Your code here: make a chat completion call`
- [ ] T018 [US1] Write "Troubleshooting" section (3 min read): Common errors (401 auth, network timeout, import errors) with solutions. Include OS-specific notes for Windows vs Mac/Linux.
- [ ] T019 [US1] Add "Try With AI" section (Lesson 1 policy): Instruct students to use ChatGPT web (tools not yet taught) to ask clarifying questions about API concepts

**Checkpoint**: Lesson 1.1 complete - students can understand APIs and verify setup independently

---

## Phase 4: User Story 2 - LLM vs Agent Distinction (Lesson 1.2) (Priority: P2)

**Goal**: Students distinguish LLMs (text-only) from Agents (tools + memory) and understand why DocuBot needs agent capabilities

**Independent Test**: Student creates comparison table with 5+ items per column showing ChatGPT limitations vs DocuBot agent advantages

**Lesson File**: `02-llm-vs-agent.md`

### Implementation for Lesson 1.2

- [ ] T020 [P] [US2] Create lesson frontmatter (title, duration: 30-40 min, learning goal, sidebar position) in `02-llm-vs-agent.md`
- [ ] T021 [P] [US2] Write "Learning Goal" section (2 min read): "Understand fundamental difference between static LLM and dynamic agent"
- [ ] T022 [P] [US2] Write "What You'll Learn (Concept)" section (5 min read): LLM = text generation only, Agent = LLM + tools + memory + actions. Emphasize agent loop (keeps working until task done).
- [ ] T023 [US2] Write "Key Points" section (3 min read): 4 bullet points (LLM definition, Agent definition, Agent capabilities, Agent loop concept). A1-A2 language.
- [ ] T024 [US2] Write "Simple Analogy" section (3 min read): **SMART PERSON IN ROOM ANALOGY** - ChatGPT = person talking through door slot (limited), Agent = person with phone, computer, can leave room (tools). Use callout box.
- [ ] T025 [US2] Write "Code Level" section (1 min read): "NO CODE - We're building mental models. Code starts in Chapter 2."
- [ ] T026 [US2] Write "🤖 Apply to DocuBot Project" section with **Task** subsection (Create comparison table with 2 columns: "What ChatGPT Can Do" vs "What DocuBot Agent Will Do", list at least 5 items each) and **Outcome** subsection: "*A clear comparison showing why DocuBot needs to be an agent (search documents, remember context, format citations) not just a chatbot.*"
- [ ] T027 [US2] Write "💡 Hints" section with 4 progressive hints (Hint 1: Think about ChatGPT limitations → Hint 4: Examples for DocuBot column: "Search my uploaded documents", "Remember our conversation", "Cite which document the answer came from")
- [ ] T028 [US2] Write "Example Comparison" section (optional scaffolding): Show 2 example entries per column to guide students
- [ ] T029 [US2] Add "Try With AI" section: Instruct students to use ChatGPT web to explore examples of agent capabilities in other domains (coding agents, research agents)

**Checkpoint**: Lesson 1.2 complete - students understand LLM vs Agent distinction independently

---

## Phase 5: User Story 3 - Environment Setup (Lesson 1.3) (Priority: P1)

**Goal**: Students install UV, create docubot project, configure .env file, verify everything works

**Independent Test**: Run `uv --version` (shows version), check `pyproject.toml` exists, `.env` has API key, `uv add openai` succeeds, Python can import openai

**Lesson File**: `03-environment-setup.md`

### Implementation for Lesson 1.3

- [ ] T030 [P] [US3] Create lesson frontmatter (title, duration: 40-50 min, learning goal, sidebar position) in `03-environment-setup.md`
- [ ] T031 [P] [US3] Write "Learning Goal" section (2 min read): "Set up complete development environment ready for agent development"
- [ ] T032 [P] [US3] Write "What You'll Learn (Concept)" section (5 min read): UV package manager (fast, modern), virtual environments (isolate dependencies), environment variables (.env for secrets), folder structure (organized code)
- [ ] T033 [US3] Write "Key Points" section (3 min read): 4 bullet points (UV vs pip, virtual envs, .env files, consistent structure). A1-A2 language.
- [ ] T034 [US3] Write "Simple Analogy" section (2 min read): **CHEF'S STATION ANALOGY** - Chef organizes ingredients (dependencies), tools (UV), clean workspace (virtual env) before cooking (coding). Use callout box.
- [ ] T035 [US3] Write "Code Level" section (1 min read): "TERMINAL COMMANDS ONLY - Installing tools, creating folders. No Python code yet."
- [ ] T036 [US3] Write "🤖 Apply to DocuBot Project" section with **Task** subsection (5 numbered steps: 1) Install UV, 2) Create docubot folder, 3) Initialize UV project, 4) Create .env with API key, 5) Verify with simple test) and **Outcome** subsection: "*A fully configured project folder with UV, virtual environment, and .env file ready.*"
- [ ] T037 [US3] Write "💡 Hints" section with 6 progressive hints (Hint 1: Install UV command for Mac/Linux → Hint 6: Test by running test_api.py from Lesson 1.1). Include OS-specific commands (Windows PowerShell vs bash).
- [ ] T038 [US3] Write "OS-Specific Instructions" section (5 min read): Separate subsections for macOS/Linux (curl install) and Windows (PowerShell install). Include verification commands for each OS.
- [ ] T039 [US3] Write "Troubleshooting" section (5 min read): Corporate firewall (pip fallback), Python version too old (install 3.11+), permission errors (use --user flag), .env not loading (python-dotenv package)
- [ ] T040 [US3] Write "Verification Checklist" section: 5 checkboxes students can mark (UV installed, project created, .env configured, openai package added, import works)
- [ ] T041 [US3] Add "Try With AI" section: Instruct students to use ChatGPT web to troubleshoot specific OS issues if they encounter installation problems

**Checkpoint**: Lesson 1.3 complete - students have working environment independently

---

## Phase 6: User Story 4 - Architecture Overview (Lesson 1.4) (Priority: P3)

**Goal**: Students visualize complete DocuBot system, understand component relationships, see 16-chapter roadmap

**Independent Test**: Review `architecture.md` file - contains system diagram (text-based), 5+ components described, chapter roadmap mapping chapters to components

**Lesson File**: `04-architecture-overview.md`

### Implementation for Lesson 1.4

- [ ] T042 [P] [US4] Create lesson frontmatter (title, duration: 25-35 min, learning goal, sidebar position) in `04-architecture-overview.md`
- [ ] T043 [P] [US4] Write "Learning Goal" section (2 min read): "Understand the complete system you'll build and how each chapter contributes"
- [ ] T044 [P] [US4] Write "What You'll Learn (Concept)" section (5 min read): DocuBot is RAG chatbot (4 parts: Agent, RAG Pipeline, Backend API, Frontend UI), each chapter adds one piece, production-ready by end
- [ ] T045 [US4] Write "Key Points" section (3 min read): 4 bullet points (4 main components, chapter progression, production goal, seeing the end makes each step meaningful). A1-A2 language.
- [ ] T046 [US4] Write "Simple Analogy" section (2 min read): **HOUSE BLUEPRINT ANALOGY** - Architect's drawings before laying bricks, otherwise random brick placement. Architecture diagram is DocuBot "house blueprint". Use callout box.
- [ ] T047 [US4] Write "Code Level" section (1 min read): "NO CODE - Architecture documentation only."
- [ ] T048 [US4] Write "🤖 Apply to DocuBot Project" section with **Task** subsection (Create architecture.md with 3 parts: 1) System overview diagram, 2) Component descriptions, 3) Chapter roadmap) and **Outcome** subsection: "*A documented architecture that serves as your roadmap for the entire course.*"
- [ ] T049 [US4] Write "💡 Hints" section with 5 progressive hints (Hint 1: Simple box diagram: User → Frontend → Backend → Agent → Vector DB → Hint 5: Can draw on paper first, then describe in text)
- [ ] T050 [US4] Write "Component Diagram Example" section (3 min read): Show text-based ASCII diagram or Markdown flowchart as example (User → Frontend [ChatKit] → Backend [FastAPI] → Agent [OpenAI SDK] → Vector DB [Qdrant])
- [ ] T051 [US4] Write "Chapter Roadmap Table" section (2 min read): Markdown table mapping chapter ranges to components (Ch 1-3: Agent basics | Ch 4-9: Advanced features | Ch 10-11: RAG | Ch 12-13: Frontend | Ch 14-16: Production)
- [ ] T052 [US4] Add "Try With AI" section: Instruct students to use ChatGPT web to explore RAG architecture patterns and get ideas for organizing their architecture.md

**Checkpoint**: Lesson 1.4 complete - students have course roadmap independently

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Finalize chapter with summary, quiz, and validation

- [ ] T053 [P] Create chapter summary section in README.md: Recap 4 key learnings (APIs, Agents, Environment, Architecture) and DocuBot state after Chapter 1
- [ ] T054 [P] Create `05-chapter-01-quiz.md` with 10 multiple-choice questions covering all 4 lessons (2-3 questions per lesson topic). **Doc**: Reference quiz-generator skill for format if needed.
- [ ] T055 [P] Add navigation links to README.md: "What's Next" pointing to Chapter 2, "Prerequisites" showing Chapter 1 has none
- [ ] T056 Validate all lesson files have consistent frontmatter (title, duration, learning_goal, sidebar_position)
- [ ] T057 Validate all lessons have "🤖 Apply to DocuBot Project" section with Task + Outcome subsections (CSC-002 requirement)
- [ ] T058 Validate DocuBot sections show cumulative progress: L1.1 (test_api.py) → L1.2 (comparison table) → L1.3 (environment) → L1.4 (architecture.md)
- [ ] T059 Validate no agent code present in any lesson (TC-001 constraint - only API verification in L1.1)
- [ ] T060 Validate each lesson has clear analogy (PC-003 constraint): Restaurant, Smart Person, Chef, Blueprint
- [ ] T061 Validate cognitive load per lesson ≤ 5-7 concepts for A1-A2 tier (PC-002 constraint)
- [ ] T062 Run final readability check: All lessons use A1-A2 language (short sentences, concrete examples, minimal jargon)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all lessons
- **Lessons (Phase 3-6)**: All depend on Foundational completion
  - Lessons can proceed in parallel (different files, no shared dependencies)
  - Or sequentially by priority: US1 (P1) → US3 (P1) → US2 (P2) → US4 (P3)
- **Polish (Phase 7)**: Depends on all lessons complete

### User Story Dependencies

- **User Story 1 (Lesson 1.1 - P1)**: Can start after Foundational - No dependencies on other lessons
- **User Story 2 (Lesson 1.2 - P2)**: Can start after Foundational - Independent from other lessons (purely conceptual)
- **User Story 3 (Lesson 1.3 - P1)**: Can start after Foundational - Mentions Lesson 1.1's test_api.py for verification but can be written independently
- **User Story 4 (Lesson 1.4 - P3)**: Can start after Foundational - Independent from other lessons (documentation exercise)

**Note**: All lessons are independently testable. A student could theoretically complete just Lesson 1.1 and verify API connectivity without completing other lessons (though pedagogically, full chapter completion is recommended).

### Within Each Lesson

**Sequential steps per lesson**:
1. Frontmatter and Learning Goal (parallel tasks)
2. Concept sections (What You'll Learn, Key Points, Analogy, Code Level)
3. Apply to DocuBot Project section (Task + Outcome)
4. Hints section
5. Additional sections (Troubleshooting, Examples, OS-specific)
6. Try With AI section

**Parallel opportunities within lessons**:
- Frontmatter + Learning Goal can be written in parallel
- Concept sections (What You'll Learn, Key Points, Analogy) can be written in parallel once concepts are identified

### Parallel Opportunities

- **Phase 1 (Setup)**: All 4 tasks can run in parallel (different directories)
- **Phase 2 (Foundational)**: All 4 tasks can run in parallel (different files)
- **Phase 3-6 (Lessons)**: All 4 lessons can be developed in parallel by different authors (US1, US2, US3, US4)
- **Phase 7 (Polish)**: Tasks T053-T055 can run in parallel (different files), validation tasks T056-T062 must run sequentially after content complete

---

## Parallel Example: All 4 Lessons (Maximum Concurrency)

```bash
# After Foundational phase completes, launch all lessons in parallel:

# Author 1: Lesson 1.1 (API Fundamentals)
Tasks: T009-T019 (11 tasks for Lesson 1.1)

# Author 2: Lesson 1.2 (LLM vs Agent)
Tasks: T020-T029 (10 tasks for Lesson 1.2)

# Author 3: Lesson 1.3 (Environment Setup)
Tasks: T030-T041 (12 tasks for Lesson 1.3)

# Author 4: Lesson 1.4 (Architecture Overview)
Tasks: T042-T052 (11 tasks for Lesson 1.4)

# All lessons converge at Phase 7 (Polish)
Tasks: T053-T062 (10 validation tasks)
```

**Result**: With 4 authors, all lessons complete simultaneously after Foundational phase, reducing time-to-completion by ~75%.

---

## Implementation Strategy

### MVP First (Lesson 1.1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all lessons)
3. Complete Phase 3: Lesson 1.1 (API Fundamentals)
4. **STOP and VALIDATE**: Test Lesson 1.1 independently
   - Student can read lesson
   - Student can run test_api.py
   - Student sees AI response
   - Student confirms API key works
5. Publish Lesson 1.1 as MVP (students can start learning immediately)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add Lesson 1.1 → Test independently → Publish (MVP! Students can verify API setup)
3. Add Lesson 1.3 → Test independently → Publish (Students can now set up full environment)
4. Add Lesson 1.2 → Test independently → Publish (Students understand agent concepts)
5. Add Lesson 1.4 → Test independently → Publish (Students see full roadmap)
6. Each lesson adds value without breaking previous lessons

### Parallel Team Strategy

With 4 content authors:

1. **Team completes Setup + Foundational together** (4 people × 2 days = foundation ready)
2. **Once Foundational is done**:
   - Author A: Lesson 1.1 (API Fundamentals) - 11 tasks
   - Author B: Lesson 1.2 (LLM vs Agent) - 10 tasks
   - Author C: Lesson 1.3 (Environment Setup) - 12 tasks
   - Author D: Lesson 1.4 (Architecture Overview) - 11 tasks
3. **Lessons complete independently** (4 people × 3-4 days = all lessons ready)
4. **Team completes Polish together** (4 people × 1 day = validation + quiz)

**Total timeline with 4 authors**: ~1 week (vs 3-4 weeks sequential)

---

## Task Summary

**Total Tasks**: 62 tasks
- **Phase 1 (Setup)**: 4 tasks
- **Phase 2 (Foundational)**: 4 tasks (BLOCKING)
- **Phase 3 (Lesson 1.1 - US1)**: 11 tasks
- **Phase 4 (Lesson 1.2 - US2)**: 10 tasks
- **Phase 5 (Lesson 1.3 - US3)**: 12 tasks
- **Phase 6 (Lesson 1.4 - US4)**: 11 tasks
- **Phase 7 (Polish)**: 10 tasks

**Parallel Opportunities**: 44 tasks can run in parallel (marked with [P])

**Independent Test Criteria**:
- **Lesson 1.1**: Student runs test_api.py, sees AI response
- **Lesson 1.2**: Student creates comparison table with 5+ items per column
- **Lesson 1.3**: Run `uv --version`, check pyproject.toml, .env configured, import openai works
- **Lesson 1.4**: Review architecture.md with 3 sections, 5+ components, chapter roadmap

**Suggested MVP Scope**: Lesson 1.1 only (API Fundamentals) - allows students to start verifying their OpenAI setup immediately

**Estimated Timeline**:
- **Single author**: 18-20 hours (4-5 hours per lesson × 4 lessons + 2 hours setup/polish)
- **4 authors (parallel)**: 6-8 hours per author (with coordination overhead)
- **MVP (Lesson 1.1 only)**: 5-6 hours

---

## Notes

- **[P] tasks** = different files, no dependencies, safe to parallelize
- **[Story] label** maps task to lesson for traceability (US1=L1.1, US2=L1.2, US3=L1.3, US4=L1.4)
- **Each lesson independently completable**: Students can test lesson outcomes without needing other lessons complete
- **DocuBot project integration**: Every lesson has "🤖 Apply to DocuBot Project" section (CSC-002)
- **Cumulative progress**: test_api.py (L1.1) → comparison table (L1.2) → environment (L1.3) → architecture.md (L1.4)
- **No agent code**: Chapter 1 is purely foundational (TC-001 constraint enforced in Phase 7 validation)
- **A1-A2 language**: All content written for complete beginners (PC-002 constraint)
- **Teaching modality variation**: Restaurant analogy → Comparison exercise → Hands-on setup → Visualization (anti-convergence)
- **Try With AI policy**: All lessons use ChatGPT web (tools not taught yet) - policy applied consistently in Phase 3-6

**Format Validation**: All tasks follow checklist format with checkbox, ID, [P] marker (if parallel), [Story] label (for lessons), description with file path.
