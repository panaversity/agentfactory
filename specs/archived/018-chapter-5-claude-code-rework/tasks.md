# Tasks: Chapter 5 Claude Code Rework

**Input**: Design documents from `/specs/018-chapter-5-claude-code-rework/`
**Prerequisites**: plan.md (complete), spec.md (9 user stories), constitution.md, chapter-index.md

**Organization**: Tasks are grouped by user story (US1-US9) to enable independent lesson implementation and testing.

**Tests**: NOT applicable - this is educational content creation. Validation comes through technical review + proof validation (Phase 4).

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different lessons, no dependencies)
- **[Story]**: Which user story/lesson this task belongs to (e.g., US1, US2, US3...)
- Include exact file paths in descriptions

---

## Path Conventions

**Content Location**: `book-source/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/`

**Naming Convention**:
- `README.md` - Chapter introduction
- `01-origin-story.md` - Lesson 1 (US1)
- `02-installation-and-authentication.md` - Lesson 2 (US2)
- `03-subagents-and-orchestration.md` - Lesson 3 (US3)
- `04-skills-and-mcp.md` - Lesson 4 (US4)
- `05-hello-world-practice.md` - Lesson 5 (US5)
- `06-claude-md-context-files.md` - Lesson 6 (US6)
- `07-permission-management.md` - Lesson 7 (US7)
- `08-hooks-and-extensibility.md` - Lesson 8 (US8)
- `09-settings-hierarchy.md` - Lesson 9 (US9)

---

## Phase 1: Setup & Preparation

**Purpose**: Establish content structure and gather intelligence

- [ ] T001 Create chapter directory structure at book-source/docs/02-AI-Tool-Landscape/05-claude-code-features-and-workflows/
- [ ] T002 [P] Review existing Chapter 5 content (README.md, 01-origin-story.md, 02-installation-and-authentication.md) for salvageable sections
- [ ] T003 [P] Verify Context7 /anthropics/claude-code library access and fact-check all feature claims
- [ ] T004 [P] Review constitution v3.1.3 for Principle 13 (Graduated Teaching), Principle 18 (Three Roles), cognitive load limits (A2: max 7 concepts)

**Checkpoint**: Content structure ready, intelligence sources validated

---

## Phase 2: Foundational Content (Chapter README)

**Purpose**: Create chapter introduction that bridges from Part 1 and sets context for 9 lessons

**⚠️ CRITICAL**: README must be complete before individual lessons to ensure consistent framing

- [ ] T005 Write chapter README.md with:
  - "What You'll Learn" (9-10 bullet points covering all user stories)
  - Chapter Structure overview (9 lessons with durations)
  - Prerequisites (Chapters 1-4, Node.js 18+, terminal access)
  - Total Duration (55-75 minutes realistic estimate)
  - Bridge from Part 1: "You know WHY AI-driven development matters (Chapters 1-4). Now meet the FIRST TOOL: Claude Code."
  - Why Claude Code Is Chapter 5: Pillar 1 (AI CLI & Coding Agents), Part 2 tool literacy focus

- [ ] T006 Add constitutional alignment note to README:
  - Graduated Teaching Pattern (Tier 1: book teaches → Tier 2: AI companion → Tier 3: orchestration intro)
  - Three Roles Framework (AI as Teacher/Student/Co-Worker)
  - Cognitive Load Management (A2 tier: max 7 concepts per section)

**Checkpoint**: Chapter README complete and approved. Individual lessons can now be written in parallel.

---

## Phase 3: User Story 1 - Origin Story & Paradigm Shift (Priority: P1) 🎯 Foundation

**Goal**: Student understands passive vs agentic AI paradigm and recognizes when Claude Code adds value

**Independent Test**: Student can answer "When would you use Claude Code vs ChatGPT web?" with 3 concrete differences

### Implementation for User Story 1

- [ ] T007 [US1] Write Lesson 1 (01-origin-story.md) with sections:
  - Hook: YC data (25% startups with 95% AI-generated code)
  - What Is Claude Code: CLI tool with direct file/shell access
  - Paradigm Shift: Passive vs Agentic AI definitions
  - Comparison Table: 6-7 differences (context, action, state, feedback, tool access, learning, workflow)
  - 7 Real-World Examples: Lost Beginner, Dependency Nightmare, Code Review, Documentation, Test Writer, Migration, Learning Accelerator
  - Why Terminal Integration Matters: 5 reasons (direct file access, real-time execution, version control, workflow alignment, transparency)
  - Reflection prompts

- [ ] T008 [US1] Add "Try With AI" section to 01-origin-story.md:
  - Prompt 1: Analyze Your Workflow (explain 3 ways Claude Code differs)
  - Prompt 2: Tool Selection Reasoning (3 scenarios with reasoning)
  - Prompt 3: Personal Relevance (would Claude Code help YOU?)
  - Format: Chapter 1 clean style (3 prompts total, focused)

- [ ] T009 [US1] Add frontmatter to 01-origin-story.md:
  - `sidebar_position: 1`
  - `title: "The Claude Code Origin Story and Paradigm Shift"`
  - `duration: "8-10 min"`

**Checkpoint**: Lesson 1 complete. Maps to SC-002 (90% understand paradigm), Eval 2 (Conceptual Understanding).

---

## Phase 4: User Story 2 - Installation & Authentication (Priority: P1) 🎯 Blocker

**Goal**: Student successfully installs Claude Code on their platform and authenticates

**Independent Test**: `claude --version` returns version number; test command returns AI response

### Implementation for User Story 2

- [ ] T010 [US2] Write Lesson 2 (02-installation-and-authentication.md) with sections:
  - Prerequisites Check (Node.js 18+ verification command)
  - Installation Options: curl script (macOS/Linux), PowerShell (Windows), NPM (fallback)
  - Step-by-Step Installation: Direct commands for all platforms
  - Verification: `claude --version` command
  - Authentication Flow: First run `claude`, OAuth interactive prompt
  - Quick Test: `claude "Hello! Confirm you're working"`
  - Realistic timing: 2-5 min install, 1-2 min auth

- [ ] T011 [US2] Add Troubleshooting section to 02-installation-and-authentication.md:
  - "command not found: claude" (PATH not updated, restart terminal solution)
  - "npm ERR! code EACCES" (permissions, sudo or npm config solution)
  - "Authentication failed / rate limit" (wait 1 hour or Console API credits)
  - "Node.js not found" (install Node.js link + retry)
  - Platform-specific issues (macOS/Linux/Windows variations)

- [ ] T012 [US2] Add "Try With AI" section to 02-installation-and-authentication.md:
  - Prompt 1: Verify Installation (`claude "You're my first conversation. Confirm file access."`)
  - Prompt 2: Ask About Next Steps (`claude "What can you do that ChatGPT web can't?"`)
  - Prompt 3: Handle Troubleshooting (if needed: `claude "Help me debug this error: [error]"`)
  - Format: 3 focused prompts, now using Claude Code (just installed!)

- [ ] T013 [US2] Add Security and Best Practices section:
  - File system access (start in project dirs, not system dirs)
  - Command execution (review sudo commands before approval)
  - Cost management for Console API users (set limits, monitor usage)

- [ ] T014 [US2] Add frontmatter to 02-installation-and-authentication.md:
  - `sidebar_position: 2`
  - `title: "Installing and Authenticating Claude Code"`

**Checkpoint**: Lesson 2 complete. Maps to SC-001 (95% install success), SC-006 (platform compatibility), SC-007 (realistic durations), Eval 1 (Installation Success Rate).

---

## Phase 5: User Story 3 - Subagents & Orchestration (Priority: P2)

**Goal**: Student understands subagents (Explore, Plan) and recognizes orchestration patterns

**Independent Test**: Student explains "What is Explore agent and when does Claude invoke it?"

### Implementation for User Story 3

- [ ] T015 [P] [US3] Write Lesson 3 (03-subagents-and-orchestration.md) with sections:
  - Problem Context: Why context drift happens in long conversations
  - Subagent Explanation: Each agent specializes; coordination prevents confusion
  - Explore Agent Deep Dive: Efficient codebase search, pattern matching, rapid orientation
  - Plan Mode Explanation: Breaking complex problems into phases
  - When to Recognize: How to tell which agent is working
  - Real-world examples: Exploration (fast search), Plan Mode (multi-phase refactor)

- [ ] T016 [US3] Add "Try With AI" section to 03-subagents-and-orchestration.md:
  - Prompt 1: Recognize Explore Agent (`cd project; claude "Search for all network error handlers"`)
  - Prompt 2: Understand Plan Mode (`claude "Plan user authentication addition in phases"`)
  - Prompt 3: Evaluate Usefulness (`claude "Would you use Plan mode for [my task]? What phases?"`)

- [ ] T017 [US3] Add Tier 3 (AI Orchestration) note:
  - Competitive subagents introduced conceptually (NOT implemented)
  - Orchestration at scale mentioned (Part 5/6 advanced content)
  - Building custom subagents deferred to Chapter 32

- [ ] T018 [US3] Add frontmatter to 03-subagents-and-orchestration.md:
  - `sidebar_position: 3`
  - `title: "Subagents and Orchestration"`
  - `duration: "8-10 min"`

**Checkpoint**: Lesson 3 complete. Maps to SC-002 (paradigm understanding extended), Eval 2 (agent specialization).

---

## Phase 6: User Story 4 - Skills, Plugins, and MCP Integration (Priority: P3)

**Goal**: Student understands complete extensibility architecture (Skills + Plugins + MCP), installs plugin hands-on, and grasps relationship hierarchy for AI companion personalization

**Independent Test**: Student can: (1) explain agent skills progressive disclosure with PDF example, (2) install plugin via `/plugin` command, (3) explain MCP configuration with 2 examples, (4) articulate relationship hierarchy

### Implementation for User Story 4

- [ ] T019 [P] [US4] Write Lesson 4 Agent Skills Deep Dive section (04-skills-plugins-mcp.md):
  - Progressive disclosure pattern explained (3 levels: metadata in system prompt → full SKILL.md when relevant → referenced files only when needed)
  - SKILL.md structure demonstrated (YAML frontmatter with name/description + markdown instructional content + optional bundled files like Python scripts)
  - Filesystem discovery and dynamic loading explanation
  - PDF skill concrete example (form filling + field extraction using bundled Python script for deterministic operations beyond basic PDF comprehension)
  - When Claude invokes skills autonomously (based on task context, not user command)

- [ ] T020 [P] [US4] Write Lesson 4 Plugins System section (04-skills-plugins-mcp.md):
  - Plugin architecture explained (.claude-plugin/plugin.json manifest structure with metadata fields)
  - What plugins bundle (commands/, agents/, skills/, hooks/, .mcp.json directories and files)
  - Installation workflow demonstrated (`/plugin` command → browse marketplace → `/plugin marketplace add <url>` → `/plugin install <name>` → restart Claude Code → verify in `/help`)
  - Community plugin ecosystem introduced (anthropics/claude-code: feature-dev, code-review, security-guidance; Dan Ávila marketplace: DevOps, documentation, testing; Seth Hobson: 80+ specialized sub-agents)
  - Team configuration (.claude/settings.json with extraKnownMarketplaces for private marketplaces)
  - Use cases: team standards enforcement, framework-specific shortcuts, internal tool connections, debugging setups, deployment workflows

- [ ] T021 [P] [US4] Write Lesson 4 MCP Integration section (04-skills-plugins-mcp.md):
  - MCP (Model Context Protocol) definition (standard for external tool integration)
  - JSON configuration structure (mcpServers object → server name → command/args/oauth fields)
  - GitHub MCP server example (npx @anthropic-ai/mcp-server-github with OAuth clientId/clientSecret/scopes for repo and issues access)
  - Filesystem MCP server example (npx @modelcontextprotocol/server-filesystem with allowed path restrictions for security)
  - @-mentions activation (@github for GitHub queries, @filesystem for file operations)
  - How MCP configs fit inside plugins (plugins can contain .mcp.json files bundling external integrations)

- [ ] T022 [P] [US4] Write Lesson 4 Relationship Hierarchy section (04-skills-plugins-mcp.md):
  - Visual ASCII diagram showing: Plugins (containers) → contain → [Skills + Commands + Agents + Hooks + MCP]
  - Skills distinction: autonomous capabilities (Claude decides when to invoke based on task context)
  - Commands distinction: user-invoked slash commands (like `/plugin`, `/code-review`, `/feature-dev`)
  - Agents distinction: specialized subagents (Explore for codebase search, Plan mode for multi-step tasks)
  - Hooks distinction: event-triggered automation (PreToolUse validation, PostToolUse linting, SessionStart env loading)
  - MCP distinction: external integrations (GitHub, Filesystem, Google Drive, Jira, Slack, etc.)
  - Clear hierarchy explanation: Plugins are broadest container; Skills are one autonomous component among many

- [ ] T023 [US4] Add "Try With AI" section to 04-skills-plugins-mcp.md (4 prompts, includes hands-on plugin installation):
  - Prompt 1: Explore Plugins (`claude "/plugin"` → browse marketplace, see available plugins, understand installation options)
  - Prompt 2: Understand Skills Architecture (`claude "Explain agent skills in Claude Code. What is progressive disclosure? Give me the PDF skill example showing the 3 levels: metadata, core content, and referenced files."`)
  - Prompt 3: Install Example Plugin - HANDS-ON (`claude "/plugin marketplace add anthropics/claude-code"` → wait for confirmation → `claude "/plugin install feature-dev"` → wait for download → restart Claude Code → verify with `claude "/help"` showing new `/feature-dev` command)
  - Prompt 4: Plan Custom Plugin (`claude "If I wanted to create a [React/FastAPI/Django] plugin with shortcuts for [common workflow tasks], what would I include? Commands? Agents? Skills? Hooks? Give me the plugin.json structure."`)

- [ ] T024 [US4] Add SKILL.md Example to 04-skills-plugins-mcp.md:
  - Complete SKILL.md structure for PDF skill showing YAML frontmatter
  - Example with name: "pdf", description: "Manipulate PDF files - fill forms, extract fields, read content"
  - Markdown instructional content explaining capabilities
  - Reference to bundled Python script (pdf_parser.py) for deterministic field extraction
  - 3-level progressive disclosure annotation (Level 1: YAML loaded to system prompt, Level 2: full SKILL.md when PDF task detected, Level 3: pdf_parser.py loaded only when form extraction needed)

- [ ] T025 [US4] Add Plugin Architecture Diagram to 04-skills-plugins-mcp.md:
  - Directory structure visualization (.claude-plugin/, commands/, agents/, skills/, hooks/, .mcp.json)
  - plugin.json manifest example with metadata fields (name, version, author, description, components)
  - Component relationships clearly labeled with arrows
  - feature-dev plugin example showing all bundled components

- [ ] T026 [US4] Update frontmatter in 04-skills-plugins-mcp.md:
  - `sidebar_position: 4`
  - `title: "Skills, Plugins, and MCP Integration"` (updated title from "Skills and MCP Integration")
  - `duration: "10-12 min"` (updated duration from "6-8 min" to reflect hands-on plugin installation)

- [ ] T027 [US4] Add Edge Cases section to 04-skills-plugins-mcp.md:
  - Confusion between Skills/Plugins/MCP (with clear hierarchy explanation and visual diagram reference)
  - Plugin installation failures (network issues: check internet; permissions errors: may need sudo; restart not performed: plugins activate after restart)
  - Marketplace not accessible (firewall blocking: corporate networks; no plugins available: need to run `/plugin marketplace add <url>` first; custom marketplace: teams can host private with extraKnownMarketplaces)
  - Expectation of building custom skills/plugins now (redirect to Part 5/6: Chapters 32-33 for plugins, Chapter 39 for MCP servers; reassurance: focus on understanding and installing existing plugins for AIDD personalization)
  - Overwhelm from too many concepts (simplification: "Plugins are app stores, Skills are apps that run automatically, MCP connects external tools"; hands-on reduces cognitive load)

**Checkpoint**: Lesson 4 complete. Maps to SC-005 (skills progressive disclosure), SC-006 (plugin installation hands-on), SC-007 (relationship hierarchy), SC-008 (MCP configuration), Eval 8 (Skills and Plugins Personalization).

---

## Phase 7: User Story 5 - Hello World Practice (Priority: P2) 🎯 Confidence Builder

**Goal**: Student executes complete workflow (specify → execute → verify) and understands human-AI roles

**Independent Test**: All 4 prompts execute successfully; student can describe workflow pattern

### Implementation for User Story 5

- [ ] T023 [US5] Write Lesson 5 (05-hello-world-practice.md) with sections:
  - Workflow Explanation: Template - specify intent, Claude acts, verify
  - Workflow in Action: 6-step example (human specifies → Claude creates → human verifies → Claude runs → human evaluates → Claude iterates)
  - Role Clarity: Human as decider/verifier, Claude Code as executor
  - When to Intervene: If output wrong, specify correction

- [ ] T024 [US5] Add "Try With AI" section to 05-hello-world-practice.md (EXPANDED - 4 prompts for this lesson):
  - Prompt 1: Create File (`Create hello.py that prints "Hello World!" and current date`)
  - Prompt 2: Run and Test (`Now run hello.py and show output`)
  - Prompt 3: Modify and Improve (`Update date format to be more readable`)
  - Prompt 4: Reflect on What Happened (`What did you do? What did I do? Who decided? Who coded? Who verified?`)
  - Duration: ~8-10 minutes total for 4 prompts (this lesson more extensive)

- [ ] T025 [US5] Add Edge Cases section:
  - File Already Exists (specify uniqueness or ask to overwrite)
  - Python Not Installed (error message, defer to later chapter, suggest alternative)
  - Module Import Error (collaborate with Claude: "I got this error—what's wrong?")
  - Overwhelm by Options (guidance: focus on WORKFLOW, not code syntax details)

- [ ] T026 [US5] Add frontmatter to 05-hello-world-practice.md:
  - `sidebar_position: 5`
  - `title: "Hello World Practice"`
  - `duration: "10-12 min"`

**Checkpoint**: Lesson 5 complete. Maps to SC-003 (100% complete Hello World), SC-004 (85% identify tool strategy), SC-009 (Three Roles applied), Eval 3 (Hands-On Completion), Eval 4 (AI Usage Strategy).

---

## Phase 8: User Story 6 - CLAUDE.md Context Files (Priority: P1) 🎯 Foundational Practice

**Goal**: Student creates CLAUDE.md with AI assistance and understands persistent context benefit

**Independent Test**: CLAUDE.md file exists; auto-loads on session restart; student explains purpose

### Implementation for User Story 6

- [ ] T027 [P] [US6] Write Lesson 6 (06-claude-md-context-files.md) with sections:
  - Problem Statement: Repeating context every session is friction
  - CLAUDE.md Concept: Markdown file in project root; auto-loads on session start
  - Content Structure: Typical sections (overview, tech stack, directory structure, coding conventions, key commands, important notes)
  - Example CLAUDE.md: Real example for sample Python project with FastAPI
  - Creation Process: Ask Claude Code for help; review and refine
  - Verification: Restart session; confirm auto-loading

- [ ] T028 [US6] Add Example CLAUDE.md Structure in lesson:
  - Project Overview section
  - Technology Stack section
  - Directory Structure (with tree diagram)
  - Coding Conventions section
  - Key Commands section
  - Important Notes section
  - Complete 20-30 line example showing all sections

- [ ] T029 [US6] Add "Try With AI" section to 06-claude-md-context-files.md:
  - Prompt 1: Generate CLAUDE.md (`Create CLAUDE.md for my [project type] with sections: Overview, Stack, Structure, Conventions, Commands`)
  - Prompt 2: Refine Content (`Review CLAUDE.md. Adjust for: [corrections]`)
  - Prompt 3: Create File (`Create CLAUDE.md in project root`)
  - Prompt 4: Test Auto-Loading (Optional) (`Exit session. Next session: ask what you know about project without re-explaining`)

- [ ] T030 [US6] Add Edge Cases section:
  - CLAUDE.md Not Loading: Check file location (root), filename case (exact CLAUDE.md), restart session
  - Unclear What Goes In: Guidance - "What does Claude need to know to help effectively?"
  - Concern about File Size: Reassurance - "1-3 KB normal. Context cheap; clarity expensive."

- [ ] T031 [US6] Add frontmatter to 06-claude-md-context-files.md:
  - `sidebar_position: 6`
  - `title: "CLAUDE.md Context Files"`
  - `duration: "8-10 min"`

**Checkpoint**: Lesson 6 complete. Maps to SC-017 (90% create CLAUDE.md), Eval 6 (CLAUDE.md Creation).

---

## Phase 9: User Story 7 - Permission Management (Priority: P2)

**Goal**: Student understands permission system and can check current settings

**Independent Test**: Student explains "Why permissions?" and demonstrates `/permissions` command

### Implementation for User Story 7

- [ ] T032 [P] [US7] Write Lesson 7 (07-permission-management.md) with sections:
  - Safety Rationale: Why permission prompts exist; accidents prevented
  - Permission Modes: Default (asks every time), acceptEdits (auto-approve file changes), plan (multi-step tasks)
  - Checking Permissions: `/permissions` command demo
  - Working with Prompts: How to respond (approve/deny/edit)
  - Best Practices: Start permissive only after understanding implications

- [ ] T033 [US7] Add Permission Modes Comparison Table:
  - Columns: Mode, Behavior, Use Case, Risk
  - Rows: default, acceptEdits, plan
  - Concrete examples for each mode

- [ ] T034 [US7] Add "Try With AI" section to 07-permission-management.md:
  - Prompt 1: Check Current Permissions (`claude "/permissions"`)
  - Prompt 2: Understand Your Settings (`claude "Explain my current permission settings. What mode?"`)
  - Prompt 3: Discuss Trade-offs (`If I changed to 'acceptEdits', what changes? What would you still ask about?`)

- [ ] T035 [US7] Add Edge Cases section:
  - "Permission Denied" Blocking Workflow: Clarify intentional; approve or specify different action
  - Confusion between Denial and Error: Distinction - prompt = "May I?", error = "Something broke"
  - Concern about Memory: Reassurance - single denials not remembered; mode settings persistent

- [ ] T036 [US7] Add frontmatter to 07-permission-management.md:
  - `sidebar_position: 7`
  - `title: "Permission Management"`
  - `duration: "6-8 min"`

**Checkpoint**: Lesson 7 complete. Maps to SC-018 (85% explain permissions), Eval 7 (Permission System Understanding).

---

## Phase 10: User Story 8 - Hooks & Extensibility (Priority: P3)

**Goal**: Student understands hooks conceptually; knows customization possible for future

**Independent Test**: Student explains "What are hooks?" and gives 2 event examples

### Implementation for User Story 8

- [ ] T037 [P] [US8] Write Lesson 8 (08-hooks-and-extensibility.md) with sections:
  - Hooks Concept: Automated actions triggered by events
  - Event Types: PreToolUse (before command), PostToolUse (after command), SessionStart (open), SessionEnd (close)
  - Real-World Examples: Format code after edit, run tests after save, load env at start
  - Why It Matters: Professional workflows automate repetition
  - Building Custom: Mentioned briefly (Part 5/6 content)

- [ ] T038 [US8] Add Hook Examples with YAML-style descriptions:
  - Example 1: PostToolUse → Format code (prettier/black)
  - Example 2: PostToolUse → Run tests on saved file
  - Example 3: SessionStart → Load .env variables
  - Each with: Hook, Event, Action, Benefit

- [ ] T039 [US8] Add "Try With AI" section to 08-hooks-and-extensibility.md:
  - Prompt 1: Understand Hooks (`claude "Explain hooks. Give 2-3 useful examples."`)
  - Prompt 2: Identify Opportunities (`Think of repetitive task. claude "Tell me how a hook could help."`)
  - Prompt 3: Future Learning Path (`Can I build custom hooks? Hard? When learn? Prerequisites?`)

- [ ] T040 [US8] Add Edge Cases section:
  - Confusion between Hooks and Skills/MCP: Hooks = automate Claude behavior; Skills = extend capabilities
  - Expectation of Building Now: Redirect - "Part 5 content (Chapter 31-33). Know they exist now."
  - Hook Errors: Clarify - usually non-blocking; debugging advanced content

- [ ] T041 [US8] Add frontmatter to 08-hooks-and-extensibility.md:
  - `sidebar_position: 8`
  - `title: "Hooks and Extensibility"`
  - `duration: "5-7 min"`

**Checkpoint**: Lesson 8 complete. Maps to SC-019 (80% explain hooks), Eval 8 (Extensibility Awareness - partial).

---

## Phase 11: User Story 9 - Settings Hierarchy (Priority: P3)

**Goal**: Student understands settings exist at 3 levels with precedence order

**Independent Test**: Student explains "Where can settings be configured? What's precedence?"

### Implementation for User Story 9

- [ ] T042 [P] [US9] Write Lesson 9 (09-settings-hierarchy.md) with sections:
  - Settings Concept: Configuration files customize Claude Code behavior
  - Hierarchy Levels: User (~/.claude/settings.json), Project (.claude/settings.json), Local (.claude/settings.local.json)
  - Precedence Order: Local > Project > User (more specific overrides general)
  - Why It Matters: Team collaboration, personal customization, project safety
  - Not Configuring Now: Detailed configuration Part 5

- [ ] T043 [US9] Add Hierarchy Visualization:
  - Tree diagram showing USER LEVEL → PROJECT LEVEL → LOCAL LEVEL
  - For each: file path, applies to (scope), examples
  - Precedence arrows showing override direction

- [ ] T044 [US9] Add Example Scenario showing precedence:
  - User level: permission_mode = "default"
  - Project level: permission_mode = "acceptEdits"
  - Local level: (not set)
  - Result explanation: Project overrides user; local would override both

- [ ] T045 [US9] Add "Try With AI" section to 09-settings-hierarchy.md:
  - Prompt 1: Explain Hierarchy (`claude "Explain settings hierarchy. Where stored? Precedence? Not configuring yet."`)
  - Prompt 2: Show Your Settings (`claude "Show settings files that apply to my projects. Which exist? Where?"`)
  - Prompt 3: Plan for Later (`When I configure for team (Part 5), what settings would I modify? Project or user?`)

- [ ] T046 [US9] Add Edge Cases section:
  - Confusion about .claude/ Directory: Clarify - holds config; don't delete
  - Broke Configuration: Recovery - rename settings.json to reset; validate JSON; backups
  - Settings Not Applying: Restart Claude Code; check precedence; verify which level edited

- [ ] T047 [US9] Add frontmatter to 09-settings-hierarchy.md:
  - `sidebar_position: 9`
  - `title: "Settings Hierarchy"`
  - `duration: "4-6 min"`

**Checkpoint**: Lesson 9 complete. Maps to SC-020 (80% explain hierarchy), Eval 8 (Extensibility Awareness - complete).

---

## Phase 12: Polish & Validation

**Purpose**: Quality assurance and constitutional alignment verification

- [ ] T048 [P] Validate all 9 lessons follow Chapter 1 "Try with AI" format (3-4 focused prompts, clean structure)
- [ ] T049 [P] Verify Graduated Teaching Pattern enforced across all lessons (Tier 1/2/3 explicit)
- [ ] T050 [P] Check cognitive load limits: Each lesson ≤7 concepts per section
- [ ] T051 [P] Verify durations realistic: Lesson 1 (8-10 min), Lesson 2 (10-12 min), Lesson 3 (8-10 min), Lesson 4 (6-8 min), Lesson 5 (10-12 min), Lesson 6 (8-10 min), Lesson 7 (6-8 min), Lesson 8 (5-7 min), Lesson 9 (4-6 min) = 55-75 min total
- [ ] T052 [P] Run anti-over-engineering check: Installation direct commands (not "ask AI to install")
- [ ] T053 Review Chapter README for consistency with all 9 lessons
- [ ] T054 Cross-reference all success criteria mappings (SC-001 to SC-020)
- [ ] T055 Cross-reference all eval mappings (Eval 1 to Eval 8)
- [ ] T056 Verify all edge cases addressed in appropriate lessons
- [ ] T057 Check all frontmatter correct (sidebar_position, title, duration)
- [ ] T058 Validate all user stories (US1-US9) have complete lessons with acceptance scenarios covered

**Checkpoint**: Content ready for technical review (Phase 13).

---

## Phase 13: Technical Validation (Sandbox Testing)

**Purpose**: Sandbox verification of all installation/authentication/workflow commands

**⚠️ CRITICAL**: All commands must be tested before publication

- [ ] T059 Test installation on macOS: `curl -fsSL https://claude.ai/install.sh | bash` → verify `claude --version` works
- [ ] T060 Test installation on Linux: Same curl command → verify works
- [ ] T061 Test installation on Windows: PowerShell script → verify works
- [ ] T062 Test NPM fallback: `npm install -g @anthropic-ai/claude-code` → verify works on all platforms
- [ ] T063 Test authentication flow: Run `claude` → verify OAuth prompt appears → complete auth → verify test command
- [ ] T064 Test Hello World workflow (Lesson 5): Execute all 4 prompts → verify outputs match expected
- [ ] T065 Test CLAUDE.md creation (Lesson 6): Generate with AI → verify file created → restart session → verify auto-load
- [ ] T066 Test permissions command (Lesson 7): Run `/permissions` → verify output
- [ ] T067 Verify all troubleshooting solutions work: Test common errors → apply solutions → verify fixes
- [ ] T068 Measure actual durations: Time each lesson hands-on portions → verify within 20% of estimates
- [ ] T068A Test plugin installation workflow in sandbox:
  - Run `/plugin` → verify marketplace browser appears
  - Run `/plugin marketplace add anthropics/claude-code` → verify success message
  - Run `/plugin install feature-dev` → verify download/install
  - Restart Claude Code → verify plugin activated
  - Run `/help` → verify new commands appear (e.g., `/feature-dev`)
  - Document any errors or issues encountered

**Checkpoint**: All technical validation passed. Ready for technical review agent.

---

## Phase 14: Agent-Based Review & Finalization

**Purpose**: Invoke validation-auditor and factual-verifier agents for comprehensive quality check

- [ ] T069 Invoke validation-auditor agent for Chapter 5:
  - Check technical correctness (all Claude Code features accurate)
  - Verify code examples run (installation commands, workflows)
  - Validate pedagogical effectiveness (learning objectives met)
  - Check constitution alignment (Principles 13, 18, cognitive load)
  - Generate validation report

- [ ] T070 Address all CRITICAL issues from validation-auditor (blocking publication)
- [ ] T071 Address all MAJOR issues from validation-auditor (important but not blocking)
- [ ] T072 Review MINOR issues from validation-auditor (nice-to-have improvements)

- [ ] T073 Invoke factual-verifier agent for Chapter 5:
  - Verify factual accuracy (all claims against official docs)
  - Check against source materials (constitution, chapter-index, Context7 docs)
  - Cross-reference citations
  - Validate lesson flow and coherence

- [ ] T074 Address all HIGH priority factual-verifier findings
- [ ] T075 Address MEDIUM priority factual-verifier findings
- [ ] T076 Review LOW priority factual-verifier findings

- [ ] T077 Final constitutional alignment check:
  - Principle 13 (Graduated Teaching): ✓ across all 9 lessons
  - Principle 18 (Three Roles): ✓ across all 9 lessons
  - Core Philosophy #1 (AI Spectrum): ✓ Lesson 1 + Lesson 5
  - Cognitive Load (A2): ✓ max 7 concepts per section in all lessons
  - Anti-Over-Engineering: ✓ Lesson 2 direct commands, Chapter 1 format throughout

- [ ] T078 Run Docusaurus build test: Verify chapter renders correctly in book site
- [ ] T079 Create final validation report summarizing all checks passed

**Checkpoint**: Chapter 5 fully validated and ready for publication.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - creates README first
- **User Stories (Phases 3-11)**: All depend on Foundational (README) completion
  - Lessons 1-9 can then proceed in parallel (if multiple writers)
  - Or sequentially in priority order: P1 (Lessons 1, 2, 6) → P2 (Lessons 3, 5, 7) → P3 (Lessons 4, 8, 9)
- **Polish (Phase 12)**: Depends on all 9 lessons being complete
- **Sandbox Testing (Phase 13)**: Depends on lessons with technical commands (Lessons 2, 5, 6, 7)
- **Agent Review (Phase 14)**: Depends on all content complete and sandbox tested

### Lesson Dependencies

**No Cross-Lesson Dependencies**: Each lesson is independently complete (Graduated Teaching applied within each)

- Lesson 1 (US1): No dependencies - introduces paradigm
- Lesson 2 (US2): No dependencies - installation stands alone
- Lesson 3 (US3): Conceptually builds on Lesson 1 (paradigm) but independently complete
- Lesson 4 (US4): Conceptually builds on Lesson 1 (paradigm) but independently complete
- Lesson 5 (US5): Requires Lesson 2 (Claude Code installed) - hands-on practice
- Lesson 6 (US6): Requires Lesson 2 (Claude Code installed) - creates CLAUDE.md
- Lesson 7 (US7): Conceptually builds on Lesson 2 (authentication/usage) but independently complete
- Lesson 8 (US8): Conceptually builds on Lesson 1 (paradigm) but independently complete
- Lesson 9 (US9): Conceptually builds on Lesson 2 (installation/usage) but independently complete

### Within Each Lesson

Standard lesson structure:
1. Introduction/Hook (why this matters)
2. Core concepts (Tier 1: book teaches)
3. AI collaboration (Tier 2: AI companion helps)
4. Practice (Try with AI section)
5. Edge cases/troubleshooting (where applicable)

### Parallel Opportunities

- **Phase 1**: All setup tasks (T001-T004) can run in parallel
- **Phase 3-11**: All lessons can be written in parallel by different writers (after Phase 2 complete)
  - Lessons 1, 3, 4, 8, 9 (conceptual) can run in parallel
  - Lessons 2, 5, 6, 7 (technical/hands-on) can run in parallel
- **Phase 12**: All validation checks (T048-T058) can run in parallel
- **Phase 13**: Platform testing (T059-T062) can run in parallel
- **Phase 14**: Technical review and proof validation can run concurrently

---

## Parallel Example: All Conceptual Lessons

```bash
# Launch all conceptual lessons together (after Phase 2 README complete):
Task: "Write Lesson 1 (origin-story.md)" [US1]
Task: "Write Lesson 3 (subagents-and-orchestration.md)" [US3]
Task: "Write Lesson 4 (skills-and-mcp.md)" [US4]
Task: "Write Lesson 8 (hooks-and-extensibility.md)" [US8]
Task: "Write Lesson 9 (settings-hierarchy.md)" [US9]

# Launch all technical lessons together:
Task: "Write Lesson 2 (installation-and-authentication.md)" [US2]
Task: "Write Lesson 5 (hello-world-practice.md)" [US5]
Task: "Write Lesson 6 (claude-md-context-files.md)" [US6]
Task: "Write Lesson 7 (permission-management.md)" [US7]
```

---

## Implementation Strategy

### MVP First (Essential Lessons Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: README (Foundation)
3. Complete Phase 3: Lesson 1 (Paradigm - P1) → **STOP and VALIDATE**
4. Complete Phase 4: Lesson 2 (Installation - P1) → **STOP and VALIDATE**
5. Complete Phase 7: Lesson 5 (Hello World - P2) → **STOP and VALIDATE**
6. Test MVP (3 lessons): Paradigm → Installation → Practice
7. If MVP validates successfully, proceed to remaining 6 lessons

### Incremental Delivery (Priority Order)

1. P1 Lessons (Foundation): 1, 2, 6 → Essential core
2. P2 Lessons (Important): 3, 5, 7 → Expands understanding
3. P3 Lessons (Advanced Awareness): 4, 8, 9 → Prepares for Part 5/6

### Parallel Team Strategy

With multiple content writers:

1. Writer A: P1 Lessons (1, 2, 6) - Foundation and Practice
2. Writer B: P2 Lessons (3, 5, 7) - Concepts and Safety
3. Writer C: P3 Lessons (4, 8, 9) - Advanced Awareness

All proceed after Phase 2 (README) complete.

---

## Success Criteria Validation

Each task maps to specification success criteria:

| Task(s) | Success Criteria | Eval | Validation Method |
|---------|------------------|------|-------------------|
| T007-T009 | SC-002, SC-004, SC-013 | Eval 2, Eval 4 | Lesson 1 complete with paradigm comparison table |
| T010-T014 | SC-001, SC-006, SC-007 | Eval 1 | Lesson 2 complete; sandbox tested all platforms |
| T015-T018 | SC-002 (extended) | Eval 2 | Lesson 3 explains subagents clearly |
| T019-T027 | SC-005, SC-006, SC-007, SC-008 | Eval 8 | Lesson 4 comprehensive Skills/Plugins/MCP with hands-on plugin installation |
| T023-T026 | SC-003, SC-004, SC-009 | Eval 3, Eval 4 | Lesson 5 all prompts work in sandbox |
| T027-T031 | SC-017 | Eval 6 | Lesson 6 CLAUDE.md creation tested |
| T032-T036 | SC-018 | Eval 7 | Lesson 7 permissions command tested |
| T037-T041 | SC-019 | Eval 8 (partial) | Lesson 8 hooks concept explained |
| T042-T047 | SC-020 | Eval 8 (complete) | Lesson 9 hierarchy explained |
| T048-T058 | SC-008, SC-010, SC-011, SC-012 | All evals | Constitutional compliance validated |
| T059-T068 | SC-014, SC-015, SC-016 | Eval 5 | Technical accuracy verified |
| T068A | SC-006, SC-016 | Eval 8 | Plugin installation workflow validated in sandbox |
| T069-T079 | All SC, All Evals | All | Agent-based comprehensive review |

---

## Notes

- **Policy for Lesson Authors**: Within Chapter 5, each lesson ends with single "Try With AI" section (no "Key Takeaways" or "What's Next"). Format: `### Prompt N: Title` → code block → `**Expected outcome:**` description.
- **[P] tasks**: Different lessons/files, no dependencies - can run in parallel
- **[Story] labels**: Map tasks to user stories for traceability (US1 = Lesson 1, US2 = Lesson 2, etc.)
- **Each lesson**: Independently complete and testable
- **Anti-over-engineering check**: Installation (Lesson 2) must be direct commands, not "use AI to install"
- **Graduated Teaching**: Tier 1 (book teaches) → Tier 2 (AI companion) → Tier 3 (orchestration intro) enforced per lesson
- **Commit frequently**: After each lesson completion or logical group
- **Stop at checkpoints**: Validate lessons independently before proceeding
- **Constitutional alignment**: Every lesson must align with Principles 13, 18, Core Philosophy #1, cognitive load A2 limits
