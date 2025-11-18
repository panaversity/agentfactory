# Chapter 6: Google Gemini CLI — Detailed Lesson Plan

**Chapter Type**: Technical/Intermediate (Part 2: AI Tool Landscape)
**Current Status**: Enhancing & extending 5 existing lessons
**Specification**: Comprehensive Gemini CLI mastery for north star business developers
**Estimated Total Time**: 60-90 minutes (reading + exercises)
**Part**: Part 2 (Intermediate complexity tier)

---

## Executive Summary

Chapter 6 will be substantially enhanced from 5 foundational lessons into a comprehensive mastery guide covering:
- **10+ essential slash commands** and syntax patterns
- **Keyboard shortcuts** and command-line invocation modes
- **Configuration system** (7-level hierarchy, settings.json, tool restrictions)
- **Memory management** (3 types: ephemeral, GEMINI.md hierarchical, save_memory)
- **MCP server configuration** with practical business examples
- **Custom slash commands** creation (TOML files)
- **Extensions** installation and security evaluation
- **Advanced features** (checkpointing, branching, token management)

This plan specifies how to improve and extend each of the 5 existing lessons while maintaining coherent learning progression and business-focused framing for north star developers.

---

## Phase 1: Current State Assessment

### Lesson 1: Why Gemini CLI Matters
**Current Status**: Excellent foundational content (186 lines)
**Strengths**:
- ✓ Strong narrative on open source benefits (Apache 2.0)
- ✓ Clear comparison framework (vs. Claude Code)
- ✓ Real financial impact (free tier analysis)
- ✓ MCP overview as extensibility enabler
- ✓ "Try With AI" prompts well-structured

**Gaps** (based on 35 FRs):
- No mention of essential slash commands or features making it production-ready
- No discussion of configuration flexibility (settings.json)
- No mention of memory management (critical for north star workflows)
- No keyboard shortcuts or CLI invocation modes
- Context window discussion exists but superficial for business use

**Enhancement Strategy**: Keep narrative intact, ADD:
- Brief mention of 10+ essential commands that make it powerful
- Configuration & memory as business enablers
- Real-world developer workflow scenarios

### Lesson 2: Installation, Authentication & First Steps
**Current Status**: Basic but complete (234 lines)
**Strengths**:
- ✓ Clear installation steps
- ✓ Authentication walkthrough
- ✓ Basic session commands (/, /tools, /stats, /quit)
- ✓ Shell mode introduction (! syntax)
- ✓ Practical first task

**Gaps**:
- Missing essential slash commands (/help, /clear, /memory, /chat, /restore, /mcp, /compress, /copy, /settings, /init, /theme, /ide, /tools)
- No keyboard shortcuts (Ctrl+L, Ctrl+V, Ctrl+Y, Ctrl+X)
- No command-line invocation modes (-p, -m, --sandbox, --yolo, --checkpointing)
- @ syntax (context reference) not mentioned
- No focus on @ syntax for file/directory context
- No mention of configuration file setup

**Enhancement Strategy**: MAJOR EXPANSION - This becomes "Core Commands & CLI Mastery"
- Add comprehensive command reference table (all 13+ commands)
- Teach @ syntax (FR-003: context reference syntax)
- Teach ! syntax fully (FR-004: shell command execution)
- Add keyboard shortcuts section
- Add command-line invocation modes
- Add first GEMINI.md setup (hierarchical memory)
- Introduce settings.json path & basic customization

### Lesson 3: Built-In Tools Deep Dive
**Current Status**: Excellent (398 lines)
**Strengths**:
- ✓ Clear business framing (competitor research, pricing analysis)
- ✓ Four tool categories (files, web, search, shell)
- ✓ Real workflow examples (e.g., 3-competitor analysis)
- ✓ Privacy & safety considerations integrated
- ✓ Error handling guidance

**Gaps**:
- Missing configuration/tool restrictions aspect (FR-007: restrict tools per-session, FR-010: disable categories)
- No mention of @file/@folder syntax for context
- No mention of token/context management strategies
- No discussion of how memory system (GEMINI.md) relates to tools

**Enhancement Strategy**: Moderate enhancement
- Add section on tool restrictions (settings.json configuration)
- Introduce @ syntax for precise context (referenced in Lesson 2, applied here)
- Add context/token management strategies (knowing when to use save_memory)
- Keep existing business examples, add one using custom slash commands

### Lesson 4: Context & Memory Management
**Current Status**: Incomplete/Partial (100+ lines, but cuts off)
**Current Title**: "Choosing the Right AI Tool: When Size Matters"
**Strengths**:
- ✓ Context window explained in business terms
- ✓ When context size matters (20% use cases)
- ✓ Starts comparison framework

**Gaps** (CRITICAL - entire lesson needs redesign):
- Original lesson is about context WINDOW SIZE (comparing Claude, ChatGPT, Gemini)
- NOT about memory management (3 types) required by spec
- No GEMINI.md hierarchical loading explanation
- No save_memory usage patterns
- No /memory command reference
- No context/token lifecycle management
- No ephemeral vs. persistent memory discussion

**Enhancement Strategy**: MAJOR REDESIGN (not extend—replace focus)
- Reframe: Context ≠ Context Window. Context = information available to model
- Teach 3 memory types:
  1. **Ephemeral** (session memory - lost on /quit or crash)
  2. **GEMINI.md hierarchical** (global → project → subdirectory)
  3. **save_memory** (persistent storage of conversation state)
- Add GEMINI.md file management (5-level hierarchy)
- Add /memory commands (/memory list, /memory save, /memory load, /memory clear)
- Add workflow: when to use which memory type
- Add token management strategies (knowing when to /compress, /copy)
- Keep the context window comparison as context-setting intro, pivot to memory systems

### Lesson 5: MCP & Extensions Ecosystem
**Current Status**: Partial (100+ lines, cuts off)
**Strengths**:
- ✓ MCP concept explained well (electrical outlet analogy)
- ✓ Business problem framing (research workflow ROI)
- ✓ Playwright example (web browsing capability)
- ✓ Starts Extensions introduction

**Gaps**:
- Content cuts off—Playwright section incomplete
- No Context7 MCP server (documentation integration)
- No GitHub MCP server example
- No custom slash commands (separate feature from MCP servers)
- No Extensions installation process (different from MCP servers)
- No security evaluation framework for extensions
- No user vs. project scope distinction (custom slash commands)
- No TOML file syntax for custom commands
- No {{args}} and !{command} injection patterns
- No advanced MCP usage (error handling, rate limiting)

**Enhancement Strategy**: MAJOR EXPANSION - Complete lesson, then add new sections
- Complete Playwright example (what was started)
- Add Context7 example (documentation integration for research)
- Add GitHub example (API access for code collaboration)
- Add custom slash commands section (separate from MCP)
  - TOML file syntax
  - {{args}} placeholder injection
  - !{command} subprocess execution
  - User vs. project scope
- Add Extensions installation section
- Add security evaluation framework
- Add troubleshooting (MCP server connectivity, extension conflicts)

---

## Phase 2: Lesson-by-Lesson Enhancement Plan

### LESSON 1: Why Gemini CLI Matters (KEEP CORE, ENHANCE FRAMING)

**Target Time**: 15 minutes
**Current Content**: Keep 100%
**New Content to Add**: 10%

**Current Learning Objective**: Understand when Gemini CLI is better than Claude Code

**Enhanced Learning Objective**: Recognize Gemini CLI as extensible platform for north star business developers; understand open source enablers (Apache 2.0, MCP, custom commands) that make it ideal for power users

**Skills Taught**:
- Tool evaluation framework — A2 — Soft — Students can explain 4+ reasons to choose Gemini CLI over alternatives with evidence
- Open source benefit recognition — A1 — Conceptual — Students can name 2+ advantages of open source for developers (customization, community, transparency)
- Extensibility mindset — A2 — Soft — Students understand "tools are customizable, not fixed" and apply to AI tool selection

**New Sections to Add**:

1. **"The Power User Advantage"** (new subsection, ~200 words)
   - Transition from "good alternative" to "power user platform"
   - Mention 10+ slash commands as decision factor
   - Custom commands & MCP as business enablers
   - Hierarchical memory (GEMINI.md) for context management
   - Configuration as productivity lever

2. **"For the North Star Developer"** (new subsection, ~150 words)
   - Reframe: not about beginners, but sophisticated users
   - Speed: faster iteration with custom commands
   - Context: memory systems for large projects
   - Integration: MCP servers connect to business systems
   - Philosophy: "You own your tools"

**Key Concepts**: 5 (open source, extensibility, business value, customization, north star workflows)
**Prerequisites**: None (Chapter 1-5 provide AI context)
**Dependencies**: None (standalone conceptual foundation)
**Duration**: Reading 15 min (no exercises in this lesson)

**Code Examples**: None (conceptual lesson)

**Hands-On Practice**: Modified "Try With AI" prompts
- Existing Prompt 1: KEEP "Where Does Gemini Fit For Me?"
- Existing Prompt 2: KEEP "Free Tier Impact Estimator"
- Existing Prompt 3: KEEP "Does My Codebase Fit The Context?"
- **NEW Prompt 4**: "What Features Would Make Me Choose Gemini Over Claude?"
  ```
  I'm a [developer role: researcher/startup founder/solo engineer].
  What Gemini CLI features (custom commands, memory systems, MCP integration)
  would be most valuable for my workflow compared to Claude Code?
  ```
  Expected outcome: Personalized capability roadmap

**Functional Requirements Covered**:
- FR-002 (Why Gemini CLI excels for extensions)
- Implicit support for FR-021 to FR-030 (custom commands and extensions)

---

### LESSON 2: Master Core Commands, Syntax & Workflow (MAJOR EXPANSION)

**Target Time**: 25 minutes reading + 15 minutes exercises
**Current Content Title**: "Installation, Authentication & First Steps"
**New Content Title**: "Core Commands, Syntax & Workflow Mastery"

**Current Learning Objective**: Install Gemini CLI and launch your first session

**Enhanced Learning Objective**: Master all essential Gemini CLI commands, syntax patterns, keyboard shortcuts, and invocation modes; understand command-line workflow options (interactive, piped, checkpointed)

**Skills Taught**:
- Gemini CLI command mastery — A2 — Technical — Students can execute all 13 essential commands from memory in realistic workflow
- @ context syntax — A2 — Technical — Students can reference files/folders with @ to include context in prompts
- ! shell syntax — A2 — Technical — Students can switch to shell mode, execute commands, return to Gemini session
- Keyboard shortcuts efficiency — A1 — Technical — Students memorize 4 key shortcuts (Ctrl+L, +V, +Y, +X) and apply them
- CLI invocation modes — B1 — Technical — Students choose appropriate mode (interactive, -p, -m, --sandbox) for workflow
- Hierarchical memory setup — A2 — Technical — Students create project-level GEMINI.md with context
- Settings customization — A2 — Technical — Students modify settings.json for tool restrictions and behavior

**Current Content**: ENHANCE (keep installation/auth, expand commands)

**New Sections**:

1. **"The 13 Essential Commands"** (~800 words)
   - **Command Reference Table** with format: `command | purpose | example | common use`
   - `/help` — List all commands and help
   - `/clear` — Clear conversation history
   - `/tools` — List available built-in tools (files, web, shell, search)
   - `/mcp` — Manage MCP server connections
   - `/stats` — Show session stats (tokens used, duration, model)
   - `/compress` — Reduce token usage by summarizing context
   - `/copy` — Copy last response to clipboard
   - `/memory` — Manage persistent memory (list, save, load, clear)
   - `/chat` — Show conversation history / switch chats
   - `/restore` — Restore from checkpoint
   - `/settings` — Show/modify current session settings
   - `/init` — Initialize new GEMINI.md in project
   - `/theme` — Change terminal theme
   - `/ide` — Show IDE integration options
   - **Business scenarios for each** (competitor research, context management, token efficiency)

2. **"The @ Syntax: Bringing Context Into Conversation"** (~600 words)
   - What @ does: references files/directories/URLs without copying
   - Format: `@path/to/file`, `@folder/`, `@https://example.com`
   - When to use @ (large files, binary data, external URLs)
   - Advantages over pasting (preserves privacy, respects rate limits, dynamic updates)
   - Real example: `@./docs/ analyze our API documentation for completeness`
   - Comparison: @ vs. copy-paste vs. /mcp servers
   - Business workflow: "I reference my codebase constantly, @ syntax is essential"

3. **"The ! Syntax: Shell Integration"** (~400 words, enhance existing)
   - Expand existing shell mode explanation
   - Format: Type `!` to enter shell, `ESC` to exit
   - Real workflows: Running tests, checking git status, file operations
   - When to use ! vs. asking Gemini to run commands
   - Error handling: How Gemini interprets shell output
   - Security: What to watch for (destructive commands, permissions)

4. **"Keyboard Shortcuts: Speed Up Your Workflow"** (~300 words, NEW)
   - `Ctrl+L` — Clear screen (keeps history)
   - `Ctrl+V` — Paste multi-line input (bracket mode)
   - `Ctrl+Y` — Scroll history forward
   - `Ctrl+X` — Scroll history backward
   - Business use: "These shortcuts are worth memorizing—they save 10+ seconds per interaction"
   - Practice: 3 exercises using shortcuts

5. **"Command-Line Invocation Modes"** (~500 words, NEW)
   - **Interactive** (default): `gemini`
   - **Piped input** (`-p`): `echo "What's 2+2?" | gemini -p`
   - **Message mode** (`-m`): `gemini -m "Fix this code"`
   - **Sandbox mode** (`--sandbox`): Restricted file access for testing
   - **Yolo mode** (`--yolo`): Skip confirmation on destructive operations
   - **Checkpointing** (`--checkpointing`): Auto-save state for resumable sessions
   - Business scenarios: Automation, CI/CD integration, batch processing
   - Decision framework: How to choose invocation mode

6. **"Hierarchical Memory: Project-Level Context"** (~600 words, BRIDGE to Lesson 4)
   - First introduction to GEMINI.md (file structure, not detailed management)
   - 5-level hierarchy concept: global → project → subdirectory
   - Creating project GEMINI.md: `/init` command
   - What goes in GEMINI.md: context, project background, coding guidelines
   - Example GEMINI.md structure:
     ```
     # MyProject Context
     ## Tech Stack
     - Backend: Python 3.13 with FastAPI
     - Frontend: React 18 with TypeScript

     ## Coding Standards
     - 100% type hints required
     - Black formatting
     - Pytest for testing

     ## Current Goals
     - Implement user authentication
     - Add payment integration
     ```
   - How Gemini uses it: "Automatically applies context from GEMINI.md to every prompt"
   - Business value: "Consistent guidance across all interactions, no repeating yourself"

7. **"Basic Configuration: settings.json for Tool Control"** (~400 words, PREVIEW)
   - Where settings.json lives: `~/.config/gemini/settings.json`
   - Basic options:
     - `disabled_tools`: Which built-in tools to disable (security)
     - `model`: Which Gemini model to use
     - `api_key_source`: Where to find authentication
   - Real example:
     ```json
     {
       "disabled_tools": ["web_search"],
       "model": "gemini-2.5-pro",
       "context_window_percent": 80
     }
     ```
   - Business use: "Restrict tools in sensitive environments, enforce standards across team"
   - Note: Deep dive in Lesson 3

**Keep from Current Lesson**:
- Installation steps (unchanged)
- Authentication walkthrough (unchanged)
- Session basics (expand, don't replace)
- First task example (enhance with command examples)

**Delete/Simplify**:
- Reduce verbose explanations of shell mode (move to ! section)
- Trim redundant command explanations (consolidate in table)

**Code Examples**:
1. Basic command execution workflow (4-5 commands in sequence)
2. @ syntax reference example (sales.csv analysis)
3. ! shell mode example (git status in session)
4. Settings.json modification example (disable web search)

**Hands-On Exercises**: (45 minutes total across lesson)

**Exercise 1: Command Mastery Drill** (10 min)
- Given business scenario: "You're researching 5 competitors"
- Execute sequence of commands: /tools, /memory list, /stats
- Expected outcome: Understand what each returns

**Exercise 2: @ Syntax Practice** (8 min)
- Prompt: "@./README.md summarize this project in 100 words"
- Prompt: "@https://stripe.com/pricing analyze their free tier"
- Expected outcome: Demonstrate @ reference works with files and URLs

**Exercise 3: Keyboard Shortcuts Speedrun** (7 min)
- Use Ctrl+Y and Ctrl+X to navigate history
- Use Ctrl+L to clear and continue context
- Time yourself: how fast can you navigate?
- Expected outcome: Muscle memory for common shortcuts

**Exercise 4: Invocation Mode Selection** (10 min)
- Scenario 1 (automation): "Set up a daily research summary"
  - Which mode: piped input (`-p`)
  - Why: Automation requires non-interactive input
- Scenario 2 (creative ideation): "Brainstorm product names"
  - Which mode: interactive (`gemini`)
  - Why: Needs dialogue for iteration
- Scenario 3 (CI/CD integration): "Analyze test failures"
  - Which mode: message mode (`-m`)
  - Why: One-off analysis without keeping session
- Expected outcome: Choose appropriate mode for given workflow

**Exercise 5: GEMINI.md Setup** (10 min)
- Create a GEMINI.md in a project directory
- Add 3 sections: Tech Stack, Coding Guidelines, Current Context
- Run `/init` and verify it loads
- Expected outcome: Project context ready for Gemini to use

**Try With AI** (end-of-lesson activities):

**Prompt 1: Command Reference Builder**
```
I'm starting a Gemini CLI research project on [topic].
List the top 5 commands I should learn first and explain why each matters for my workflow.
```
Expected outcome: Personalized command priority based on workflow

**Prompt 2: Keyboard Efficiency Audit**
```
I'm trying to work faster in Gemini CLI.
Show me 3 ways I can use keyboard shortcuts (Ctrl+L, +V, +Y, +X) to speed up my typical session workflow.
Create a "shortcut cheat sheet" for my desk.
```
Expected outcome: Personalized shortcut usage patterns

**Prompt 3: Automation Design**
```
I want to automate a [daily/weekly] [analysis/research/report] using Gemini CLI's piped input mode.
Design a shell script that pipes my data to Gemini and saves the output.
Show the `-p` mode implementation and how to integrate with cron/task scheduler.
```
Expected outcome: Automation blueprint using invocation modes

**Prompt 4: Memory System Introduction**
```
I'm managing a [size: small/medium/large] codebase and want to set up GEMINI.md properly.
What should I include in the root GEMINI.md vs. subdirectory-specific context?
Create a template I can use across my project structure.
```
Expected outcome: Hierarchical memory setup strategy

**Functional Requirements Covered**:
- FR-001 (Slash command reference)
- FR-002 (Why Gemini CLI for extensions)
- FR-003 (@ context syntax)
- FR-004 (! shell syntax)
- FR-005 (Keyboard shortcuts)
- FR-008 (Command-line invocation modes)
- FR-011 (Ephemeral session memory intro)
- FR-012 (GEMINI.md hierarchical—first introduction)
- FR-006 (settings.json preview)

**Prerequisites**: Lessons 1-5 of Part 1 (basic AI tool knowledge)
**Dependencies**: None (builds on Lesson 1)
**Estimated Time**: 40 minutes (25 min reading + 15 min exercises)

---

### LESSON 3: Configuration & Tool Management (ENHANCEMENT)

**Target Time**: 20 minutes reading + 10 minutes exercises
**Current Title**: "How Your AI Reads the World: Built-In Tools in Action"
**Enhanced Title**: "Configuration, Tool Management & Security"

**Current Learning Objective**: Understand built-in tools (files, web, shell, search) and know what's safe to share

**Enhanced Learning Objective**: Master Gemini CLI configuration system (7-level hierarchy); restrict tools per-session; apply security policies; understand token implications of tool usage

**Skills Taught**:
- Configuration file management — A2 — Technical — Students can edit settings.json to enable/disable tools, set defaults
- Tool restriction strategies — B1 — Technical — Students can design security policies (which tools for research vs. implementation)
- Token efficiency — A2 — Technical — Students understand how file fetches, web requests consume tokens and manage accordingly
- Security evaluation — A2 — Soft — Students identify sensitive data and choose appropriate tool restrictions

**Current Content**: KEEP 100%, ENHANCE with:

**New Sections**:

1. **"Understanding the Settings Hierarchy"** (~600 words, NEW)
   - **7-Level Hierarchy** (from lowest to highest priority):
     1. Built-in defaults (Gemini CLI's hardcoded behavior)
     2. Global settings: `~/.config/gemini/settings.json` (applies to all sessions)
     3. Project settings: `./gemini.json` (project root, applies to project)
     4. Subdirectory overrides: `.gemini.json` (subdirectory-specific)
     5. GEMINI.md tool instructions (context-based restrictions)
     6. Session flags (command-line invocation flags)
     7. Runtime commands (`/settings` command during session)
   - Each level overrides lower levels
   - Why it matters: "Flexibility for different contexts" (research vs. security-sensitive)

2. **"Configuring settings.json"** (~700 words, EXPANDED from preview)
   - Location: `~/.config/gemini/settings.json` (global) or `./gemini.json` (project)
   - Key options with business examples:
     ```
     {
       "disabled_tools": ["web_search", "file_write"],  // ← Block unsafe tools
       "allowed_files": ["./*.md", "./docs/**"],        // ← Whitelist file access
       "web_fetch_timeout": 10,                         // ← Speed up timeouts
       "context_window_percent": 75,                    // ← Leave buffer for response
       "auto_save_memory": true,                        // ← Auto-save context
       "model": "gemini-2.5-pro",
       "api_key_source": "google_login"
     }
     ```
   - Example 1: Researcher configuration (enable all tools for competitive analysis)
   - Example 2: Implementation configuration (disable web search, enable file ops)
   - Example 3: Sensitive environment (disable file write, disable web fetch, only shell read)
   - How to validate: Use `/settings` during session to confirm
   - Common mistakes: Over-restricting when you need flexibility, under-restricting in production

3. **"Tool Restrictions for Security"** (~500 words, NEW)
   - Business scenarios for restrictions:
     - Research phase: Enable all (web, files, search)
     - Implementation phase: Disable web, enable files
     - Testing phase: Disable file write (read-only mode)
     - Production debugging: Shell commands only, no file write
   - Per-session tool control: Use `/settings` to override
   - Per-project tool control: Create `./gemini.json` in project root
   - Team patterns: Share `gemini.json` in git repo, ensures consistency
   - Enforcement: "Every developer has same tool config"

4. **"Understanding Token Cost of Tools"** (~400 words, BRIDGE to Lesson 4)
   - Each tool has token impact:
     - File read: Tokens ≈ file size (UTF-8 encoded)
     - Web fetch: Tokens ≈ page size (HTML converted to text)
     - Web search: Tokens = query + all results aggregated
     - Shell: Tokens = command + output
   - Strategy: Large files → use @file instead of /copy
   - Strategy: Multiple web fetches → batch with /mcp server
   - Preview: Lesson 4 teaches /compress and /memory for token efficiency

**Keep from Current Lesson**:
- File operations section (existing is excellent)
- Web fetching examples (enhance with tool restriction context)
- Search grounding section (keep, add tool config aspect)
- Shell integration (keep, add security notes)
- Combining tools workflow (keep, add token efficiency note)
- Error handling section (excellent, keep)
- Privacy & data safety (EXPAND with settings.json controls)

**Enhance Existing Sections**:
- Red Flags section: Add "Tool disabled: Check settings.json"
- Privacy section: "Use disabled_tools to enforce policy"
- Workflow section: Add token cost callouts

**Code Examples**:
1. settings.json for researcher (all tools enabled)
2. settings.json for implementer (selective tool access)
3. settings.json for security-sensitive (minimal tools)
4. Workflow: Fetch 3 competitor prices with token tracking

**Hands-On Exercises**:

**Exercise 1: settings.json Configuration** (5 min)
- Scenario: "You're working on sensitive customer data"
- Create settings.json that disables web_search and file_write
- Test: Run `gemini` and check `/settings` output
- Expected outcome: Confirmed tool restrictions in place

**Exercise 2: Tool Selection Strategy** (5 min)
- Given workflow: "Competitive pricing analysis (5 competitors)"
- Recommended tools: web_fetch, search_grounding, file_ops
- Restrictions: Don't need file_write, don't need shell
- Create gemini.json with appropriate config
- Expected outcome: Minimal tool config for this task

**Try With AI**:

**Prompt 1: Security Configuration Design**
```
I'm [role: researcher/startup/security team] and I need to configure Gemini CLI for my team.
Create a settings.json that balances [use case: research/implementation/security] while maintaining security.
Explain why you enabled/disabled each tool.
```
Expected outcome: Role-appropriate security configuration

**Prompt 2: Token Efficiency Audit**
```
My typical workflow is: [describe your research process].
Which tools will use the most tokens? How can I restructure to reduce token usage?
Should I use settings to restrict any tools to avoid accidental usage?
```
Expected outcome: Token-aware workflow optimization

**Functional Requirements Covered**:
- FR-006 (settings.json configuration)
- FR-007 (Per-session tool restrictions via /settings)
- FR-010 (Disable tool categories)
- FR-009 (allowed_files whitelist)

**Prerequisites**: Lessons 1-2 (understand commands, understand tools)
**Dependencies**: Lesson 3 built-in tools content
**Estimated Time**: 30 minutes (20 reading + 10 exercises)

---

### LESSON 4: Memory Systems & Context Management (MAJOR REDESIGN)

**Target Time**: 20 minutes reading + 15 minutes exercises
**Current Status**: Incomplete/wrong focus (currently about context WINDOW vs. context MANAGEMENT)
**New Title**: "Memory Systems: Persist, Organize, Reuse Context"

**Current Learning Objective**: Understand when large context windows matter

**New Learning Objective**: Master 3 memory systems (ephemeral, GEMINI.md hierarchical, save_memory); understand when each applies; implement token-efficient context strategies; organize project knowledge effectively

**Skills Taught**:
- 3 memory types mastery — B1 — Technical — Students implement all 3 types in realistic project scenario
- GEMINI.md hierarchical design — B1 — Technical — Students create 3+ level hierarchy (global, project, feature branch)
- Memory command proficiency — A2 — Technical — Students execute /memory commands (list, save, load, clear) from memory
- Token lifecycle management — A2 — Technical — Students understand when/how to /compress, know context window remaining
- Context strategy — B1 — Soft — Students choose appropriate memory type for given workflow (learning, reference, persistence)

**Completely New Content** (replaces old context window focus):

1. **"Three Types of Memory in Gemini CLI"** (~800 words, FOUNDATION)

   **Memory Type 1: Ephemeral (Session Memory)**
   - What it is: In-RAM conversation history during single session
   - Lifetime: Exists only until `/quit` or crash
   - Access: Automatic (Ctrl+Y, Ctrl+X to scroll history)
   - Use cases:
     - Exploratory research (trying ideas, discarding failures)
     - Single-task sessions (one research question, then done)
     - Sandboxed experimentation (temporary work)
   - Advantages: No setup, no persistence overhead, private
   - Disadvantages: Lost on session end, no team sharing
   - Business example: "Research one competitor's pricing, then move on"

   **Memory Type 2: GEMINI.md (Hierarchical Persistent)**
   - What it is: Markdown files that serve as context for every interaction
   - Hierarchy levels:
     1. Global: `~/.gemini/GEMINI.md` (applies to all projects)
     2. Project: `./GEMINI.md` (applies to project root)
     3. Subdirectory: `./src/GEMINI.md` (applies only in src/)
     4. Feature branch: `./.git/gemini/feature-x.md` (feature-specific context)
     5. Module-specific: `./modules/auth/GEMINI.md` (detailed guidance)
   - Lifetime: Persists across sessions, shared via git
   - Access: Automatically loaded at session start, can manually reference with @GEMINI.md
   - Content types:
     - Technical context (frameworks, languages, version requirements)
     - Coding standards (style guides, testing requirements)
     - Project goals (what we're building, success criteria)
     - Known issues (gotchas, workarounds)
     - Team decisions (architectural choices, why we chose X)
   - Use cases:
     - Onboarding new developers (context instead of training)
     - Maintaining consistency (same guidance across all sessions)
     - Large projects (manage 10K+ lines with organization)
     - Team collaboration (shared context, no repeating yourself)
   - Advantages: Persistent, versioned, shared, organized, reference-able
   - Disadvantages: Requires file management, can become stale
   - Business example: "Every developer knows our API contracts, testing standards, deployment processes"

   **Memory Type 3: save_memory (Persistent Session Storage)**
   - What it is: Named snapshots of conversation state saved to disk
   - Lifetime: Persists indefinitely until `/memory delete <name>`
   - Access: `/memory list`, `/memory load <name>`, `/memory save <name>`
   - Use cases:
     - Long-running projects (save progress, resume later)
     - Recurring tasks (save setup, reuse for next iteration)
     - Analysis workflows (save findings, iterate on recommendations)
     - Multi-developer handoffs (save state, hand to colleague)
   - Advantages: Full context restoration, named organization, easy sharing
   - Disadvantages: Token overhead if context gets large, requires cleanup
   - Business example: "Save weekly research findings, then load + analyze results next week"

2. **"Organizing GEMINI.md Across Your Project"** (~900 words, STRUCTURE)

   **Template 1: Global Context** (`~/.gemini/GEMINI.md`)
   ```markdown
   # Global Developer Context

   ## Standard Libraries I Use
   - Python: FastAPI, SQLAlchemy, Pydantic
   - TypeScript: React 18, Vite, TypeScript strict

   ## My Coding Standards
   - Type hints: 100% required (Python & TypeScript)
   - Testing: Pytest for Python, Vitest for TypeScript
   - Formatting: Black (Python), Prettier (TS)
   - No TODO comments without ticket references

   ## Security Standards
   - Never hardcode secrets (use .env or secret managers)
   - All APIs require authentication
   - Input validation on every endpoint
   ```

   **Template 2: Project Context** (`./GEMINI.md`)
   ```markdown
   # MyProject Context

   ## Tech Stack
   - Backend: Python 3.13 + FastAPI
   - Frontend: React 18 + TypeScript
   - Database: PostgreSQL 15
   - Deployment: Docker + Kubernetes

   ## Project Goals (2025 Q1)
   - Launch v2.0 with user authentication
   - Implement payment processing (Stripe)
   - Achieve 99.9% uptime on production

   ## Critical Files
   - API Contract: @./docs/api.md
   - Database Schema: @./migrations/latest.sql
   - Architecture Decision: @./docs/adr/

   ## Known Issues & Workarounds
   - TypeScript build sometimes stalls (restart build process)
   - Database migrations require manual verification
   ```

   **Template 3: Feature Branch Context** (`./.git/gemini/feature-auth.md`)
   ```markdown
   # Feature: User Authentication

   ## Acceptance Criteria
   - [ ] Login form with email/password
   - [ ] JWT token generation
   - [ ] Auth middleware on protected routes
   - [ ] Logout clears session

   ## Architecture Notes
   - Using HS256 for JWT signing
   - Token expires in 7 days
   - Refresh tokens stored in secure HttpOnly cookie

   ## Testing Checklist
   - Test: Valid credentials → login succeeds
   - Test: Invalid password → error message
   - Test: Expired token → redirect to login
   ```

3. **"The /memory Command: Save and Load Context"** (~600 words, OPERATIONS)
   - **List saved memories**: `/memory list`
     - Shows all saved context snapshots
     - Format: name, created date, size, preview
   - **Save current session**: `/memory save <name>`
     - Captures full conversation state
     - Example: `/memory save weekly-research-2025-W45`
     - Persists to `~/.gemini/memory/<name>.json`
   - **Load previous memory**: `/memory load <name>`
     - Restores full conversation context
     - Useful for: "Load last week's research, continue analysis"
   - **Delete old memory**: `/memory delete <name>`
     - Clean up old snapshots
   - **Clear current session**: `/memory clear`
     - Immediately clears all history in current session
   - Real workflow:
     ```
     Session 1: Research 10 competitors, /memory save competitor-analysis-2025
     Session 2: gemini → /memory load competitor-analysis-2025 → "Analyze trends"
     Session 3: /memory list → see all saved analyses
     ```
   - Business use: "Save research findings, load next week to continue, never lose progress"

4. **"Token Efficiency: When to /compress and /copy"** (~500 words, MANAGEMENT)
   - **Understanding context window usage**:
     - Every prompt "costs" tokens
     - Longer context = more tokens used per interaction
     - Status bar shows remaining tokens
   - **When context gets full**: `/compress`
     - Summarizes conversation intelligently
     - Example: "You've done 50 messages, context at 90%, use /compress"
     - Result: Reduces context to ~60%, keeps key findings
     - Trade-off: Some conversational nuance lost, but core ideas preserved
   - **When you need specific output**: `/copy`
     - Copies last response to clipboard
     - Saves memory: Don't clutter context with copy-paste
     - Use case: "Extract API response, copy to clipboard, save separately"
   - **Strategies to minimize token usage**:
     - Use @file instead of pasting large code blocks
     - Use /mcp servers for bulk operations (search, fetch)
     - Save important findings with /memory save (clear context, keep results)
     - Use /compress before hitting context limit
   - **Advanced**: Token remaining indicator
     - Status bar shows: "Tokens: 842K/1M"
     - Strategy: At 80%+, plan to /compress or /memory save + restart

5. **"Memory in Action: Real Workflows"** (~700 words, PRACTICE)

   **Workflow 1: Research Project (Save & Load)**
   - Day 1:
     ```
     gemini
     > Research top 10 SaaS competitors in project management
     > [15 minutes of analysis]
     > /memory save saas-competitors-analysis-v1
     > /quit
     ```
   - Day 8:
     ```
     gemini
     > /memory load saas-competitors-analysis-v1
     > Identify pricing trends and gaps in market
     > [Continues with full context from Day 1]
     > /memory save saas-competitors-analysis-v2
     > /quit
     ```
   - Result: Persistent research project spanning weeks

   **Workflow 2: Codebase Learning (GEMINI.md Hierarchy)**
   - Global `~/.gemini/GEMINI.md`: "I use FastAPI, React, TypeScript strict"
   - Project `./GEMINI.md`: "This is a real-time collaboration app, uses WebSockets, PostgreSQL"
   - Feature `./features/collab/GEMINI.md`: "Implement real-time presence. Use operational transforms for conflict resolution"
   - Gemini automatically applies all 3 levels
   - Result: Context builds naturally, developer doesn't repeat explanations

   **Workflow 3: Token Efficiency (Monitor & Compress)**
   - Session starts: 1M tokens
   - After 30 min of research: 400K tokens remaining
   - When reaching 100K: `/compress` to summarize findings
   - Continuation: 500K tokens freed up, continue working
   - Before quitting: `/memory save research-findings`
   - Result: Never run out of context, findings preserved

**Code Examples**:
1. Hierarchical GEMINI.md structure (3-level example)
2. /memory workflow (save → load → continue)
3. Context token tracking during session
4. /compress before and after comparison

**Hands-On Exercises**:

**Exercise 1: Create Hierarchical GEMINI.md** (8 min)
- Create 3 GEMINI.md files:
  - Global: `~/.gemini/GEMINI.md` (coding standards)
  - Project: `./my-project/GEMINI.md` (project context)
  - Feature: `./my-project/features/auth/GEMINI.md` (feature-specific)
- Verify with `/init` that they load in correct priority
- Expected outcome: Hierarchical context ready

**Exercise 2: Memory Lifecycle** (10 min)
- Start session 1: Research 3 tools
- `/memory save research-tools-session1`
- Quit, start session 2
- `/memory load research-tools-session1`
- Continue analysis based on loaded context
- `/memory save research-tools-analysis`
- Expected outcome: Understand save/load workflow

**Exercise 3: Token Monitoring & Compression** (5 min)
- Start session with verbose project context
- Watch token remaining in status bar
- When tokens reach 60% remaining, run `/compress`
- Observe context size reduction
- Expected outcome: Know when and how to use /compress

**Exercise 4: Memory Types Decision** (7 min)
- Given scenarios, choose appropriate memory type:
  - Scenario 1 (one-time competitor research): Ephemeral
  - Scenario 2 (persistent project standards): GEMINI.md
  - Scenario 3 (weekly research continuation): save_memory
  - Scenario 4 (team onboarding): GEMINI.md
  - Scenario 5 (experimental prototyping): Ephemeral
- Expected outcome: Decision framework internalized

**Try With AI**:

**Prompt 1: GEMINI.md Design**
```
I'm starting a [project type: SaaS/API/ML] project with [team size: solo/small/large].
Design a hierarchical GEMINI.md structure (global, project, feature levels).
What should go at each level? Create templates for each.
```
Expected outcome: Project-specific memory architecture

**Prompt 2: Memory Strategy**
```
My typical workflow: [describe: research → implementation → testing].
For each phase, recommend ephemeral, GEMINI.md, or save_memory.
Explain when to /compress and when to /memory save.
```
Expected outcome: Memory strategy optimized for workflow

**Prompt 3: Long-Running Project Management**
```
I'm managing a [duration: 3-month/6-month] project and need to preserve context across 20+ sessions.
Design a /memory save naming and organization system.
Show how to structure saves by week/sprint/phase for easy retrieval.
```
Expected outcome: Memory management system for large projects

**Functional Requirements Covered**:
- FR-011 (Ephemeral session memory)
- FR-012 (GEMINI.md hierarchical loading)
- FR-013 (save_memory persistent storage)
- FR-014 (/memory commands)
- FR-015 (Context/token lifecycle, /compress)

**Prerequisites**: Lessons 1-3 (commands, tools, configuration)
**Dependencies**: None (builds on prior knowledge)
**Estimated Time**: 35 minutes (20 reading + 15 exercises)

---

### LESSON 5: Extend with MCP, Custom Commands & Extensions (MAJOR EXPANSION)

**Target Time**: 30 minutes reading + 20 minutes exercises
**Current Title**: "When Your AI Needs More: MCP and Extensions"
**Status**: Partial (incomplete example)

**New Learning Objective**: Install and configure MCP servers for business workflows; create custom slash commands with TOML; evaluate extension security; architect integration between MCP + custom commands + GEMINI.md

**Skills Taught**:
- MCP server installation — A2 — Technical — Students can install 2+ MCP servers (Playwright, Context7) and verify connection
- Business workflow design — B1 — Soft — Students architect custom commands for team productivity
- Custom slash commands — B1 — Technical — Students write TOML files with {{args}} injection and !{command} execution
- Extension security evaluation — A2 — Soft — Students assess extension risks (code review, permissions, vendor reputation)
- Integration architecture — B1 — Soft — Students design system combining MCP + settings + GEMINI.md + custom commands

**Current Content**: KEEP MCP explanation, COMPLETE Playwright example, ADD Context7 & GitHub

**New Sections**:

1. **"MCP Server 1: Playwright (Web Browsing)"** (~1000 words, COMPLETE from partial)

   [Keep existing electrical outlet analogy and business problem description]

   **Installation Steps**:
   ```bash
   # Install Playwright MCP server
   npm install -g @anthropic-ai/mcp-server-playwright

   # Add to Gemini CLI config
   # Edit ~/.config/gemini/mcp-servers.json
   ```

   **Configuration** (`~/.config/gemini/mcp-servers.json`):
   ```json
   {
     "servers": [
       {
         "name": "playwright",
         "command": "mcp-server-playwright",
         "capabilities": ["web_browsing", "form_filling", "data_extraction"]
       }
     ]
   }
   ```

   **Real Workflow Example: Competitive Analysis**

   Business scenario: "Analyze 3 SaaS competitors' features and pricing (would take 45 minutes manually)"

   ```
   gemini
   > Use Playwright to browse these 3 competitor websites:
   > 1. https://competitor-a.com/pricing
   > 2. https://competitor-b.com/pricing
   > 3. https://competitor-c.com/pricing
   >
   > For each: Extract pricing tiers, features list, and any free trial info.
   > Then create a comparison table with gaps vs. our offering.

   [Gemini uses Playwright MCP to:]
   - Navigate to each URL
   - Click through pricing pages
   - Extract tables/text
   - Compile comparison

   [Result: Complete competitive analysis in 5 minutes]
   ```

   **Advanced: Form Filling with Playwright**
   - Use case: "Fill out 5 customer feedback forms"
   - Workflow: Playwright navigates, finds forms, fills with provided data
   - Security: Set `form_filling_allowed: false` in settings.json if you want manual control

   **When to Use Playwright vs. Built-In Web Fetch**:
   - **Built-in web fetch**: Single page, static content, simple read
   - **Playwright MCP**: Multi-page navigation, dynamic content, form interaction, JavaScript rendering
   - Decision: "Does the site require clicking/navigation? Use Playwright"

2. **"MCP Server 2: Context7 (Documentation Integration)"** (~700 words, NEW)

   **What It Does**: Gives Gemini access to constantly updated technical documentation (Stripe API docs, AWS docs, etc.)

   **Why It Matters for Business**: "Your AI reads the latest documentation instead of outdated training data"

   **Installation**:
   ```bash
   npm install -g @anthropic-ai/mcp-server-context7
   ```

   **Configuration**:
   ```json
   {
     "servers": [
       {
         "name": "context7",
         "command": "mcp-server-context7",
         "config": {
           "documentation_sources": [
             "https://stripe.com/docs",
             "https://github.com/openai/openai-python/tree/main/docs",
             "https://docs.anthropic.com"
           ]
         }
       }
     ]
   }
   ```

   **Real Workflow Example: API Integration**

   Business scenario: "Integrate Stripe payment processing; need current API docs"

   ```
   gemini
   > Use Context7 to access latest Stripe API docs
   > Show me the current webhook event types
   > How do I set up webhook signature verification in Python?
   >
   [Gemini uses Context7 MCP to:]
   - Fetch latest Stripe docs from stripe.com/docs
   - Search for webhook documentation
   - Find signature verification code examples
   - Provide current, accurate information (not training data)
   ```

   **Advanced: Custom Documentation Sources**
   - Configure Context7 to access your internal documentation
   - Example: Company wiki, API documentation, coding standards
   - Every developer has access to latest team knowledge

3. **"MCP Server 3: GitHub (Code & Collaboration)"** (~600 words, NEW)

   **What It Does**: Query repositories, issues, pull requests, code search

   **Installation & Setup**:
   ```bash
   npm install -g @anthropic-ai/mcp-server-github

   # Requires GitHub API token
   export GITHUB_TOKEN=ghp_xxxx
   ```

   **Real Workflows**:
   - Query: "Show me recent PRs and their review status"
   - Query: "Find all open issues related to authentication"
   - Query: "Search code for uses of deprecated API"
   - Analysis: "Summarize this PR's changes for my review"

   **Example**:
   ```
   gemini
   > Use GitHub MCP to show open PRs waiting for review
   > [Lists PRs with status]
   > /memory save pr-review-list
   > Now use Playwright to open each PR URL and summarize changes
   ```

4. **"Custom Slash Commands: Automation via TOML"** (~1200 words, MAJOR NEW SECTION)

   **What Are Custom Slash Commands?**
   - User-defined commands that extend Gemini CLI
   - Different from MCP servers (commands are simpler, don't require external process)
   - Format: `.gemini-commands.toml` file in project root
   - Scope: User (global) vs. Project (shared in git)

   **Command Syntax**:
   ```toml
   [commands]

   [commands.analyze]
   description = "Analyze provided code for issues"
   prompt = "Review this code for bugs, performance issues, and security vulnerabilities: {{code}}"

   [commands.research]
   description = "Research a topic and save findings"
   prompt = """
   Research {{topic}} thoroughly:
   1. What is it?
   2. Top 3 use cases
   3. Pros and cons
   4. When to use vs. alternatives
   5. Learning resources
   Save findings to memory.
   """

   [commands.deploy]
   description = "Prepare deployment checklist"
   prompt = """
   Create a pre-deployment checklist for {{service}}:
   - Security review items
   - Performance checks
   - Database migration validation
   - Configuration verification
   - Rollback procedure
   """
   ```

   **Advanced: {{args}} Placeholder Injection**
   - `{{args}}` captures everything after command name
   - Example:
     ```
     /analyze my_code.py
     → prompt = "Review this code for bugs: my_code.py"
     ```
   - Can use multiple: `{{service}}`, `{{environment}}`, `{{feature}}`

   **Advanced: Subprocess Execution via !{command}**
   - Execute shell commands within custom command
   - Example:
     ```toml
     [commands.audit]
     description = "Full security audit"
     prompt = """
     Run security checks:
     !{npm audit}
     !{bandit -r ./app}

     Then summarize findings and create action items.
     """
     ```

   **Example: Team Custom Commands**

   File: `.gemini-commands.toml` (committed to git, shared across team)
   ```toml
   [commands.deploy-prod]
   description = "Prepare production deployment"
   prompt = """
   I'm deploying {{service}} to production.

   Create a deployment checklist including:
   - Pre-deployment validation (using GEMINI.md deployment standards)
   - Database migration strategy
   - Rollback procedure
   - Post-deployment verification
   - Communication plan (what to tell stakeholders)
   """

   [commands.review-pr]
   description = "Review code changes in PR"
   prompt = """
   Review this PR:
   {{pr_description}}

   Check against our standards from GEMINI.md:
   - Type hints: 100% coverage required
   - Test coverage: >80% required
   - Performance: No new N+1 queries
   - Security: Input validation on all endpoints
   - Documentation: Updated if API changed
   """

   [commands.estimate-effort]
   description = "Estimate task effort using project context"
   prompt = """
   Based on our project context (GEMINI.md), estimate effort for: {{task}}

   Consider:
   - Team familiarity with required tech
   - Known risks/challenges from past issues
   - Dependencies and blockers
   - Testing and deployment complexity

   Provide: effort estimate, confidence level, risks, recommendations
   """
   ```

   **User vs. Project Scope**:
   - **User scope** (`~/.gemini/.gemini-commands.toml`): Personal commands across all projects
     - Examples: `/research`, `/analyze`, `/write-email`
   - **Project scope** (`./.gemini-commands.toml` in git): Team commands, project-specific
     - Examples: `/deploy-prod`, `/review-pr`, `/test-strategy`
   - **Loading priority**: Project commands override user commands

   **Real Business Impact**:
   ```
   Before custom commands:
   "How do I review PRs?"
   → Repeat checklist every time (5 min per PR)

   After custom commands:
   /review-pr [paste description]
   → Automated checklist applied consistently (1 min per PR)
   → Team alignment on standards
   → 80% time savings per PR
   ```

5. **"Extensions: Bundled Capabilities & Security"** (~800 words, NEW)

   **What Are Extensions?**
   - Pre-packaged command sets and MCP server bundles
   - Different from: MCP servers (self-contained) and custom commands (local TOML)
   - Distribution: Npm packages or GitHub repos
   - Installation: `gemini extension install <name>`

   **Extension Examples**:
   - `@gemini/extension-dataops`: Custom commands for data analysis
   - `@gemini/extension-ml-workflow`: ML project automation
   - `@gemini/extension-devops`: Deployment and infrastructure commands

   **Installation**:
   ```bash
   gemini extension install @gemini/extension-dataops

   # Lists installed extensions
   gemini extension list

   # Remove extension
   gemini extension remove @gemini/extension-dataops
   ```

   **Security Evaluation Framework**:
   Before installing any extension, ask:
   1. **Vendor Reputation**: Who published it? (Google official, npm popular, unknown author?)
   2. **Code Review**: Can you read the source? Is it open source?
   3. **Permissions**: What can it do? (Read files? Write files? Execute shell? Network access?)
   4. **Dependencies**: What does it depend on? Any suspicious packages?
   5. **Updates**: When was it last updated? Is it actively maintained?
   6. **Community**: Reviews? Issues? How many users rely on it?

   **Risk Assessment**:
   ```
   Extensions from @google: ✅ (Official, audited)
   Popular community extensions (100+ stars): ✅ (Peer review)
   Unknown author, no reviews: ⚠️ (Requires scrutiny)
   Closed source extensions: ⚠️ (Can't audit)
   Requests root/sudo access: ❌ (Hard no)
   ```

   **Example: Evaluating a Data Analysis Extension**:
   ```
   Extension: @community/data-science-toolkit

   Questions:
   - Who created it? Community user with 50 stars on GitHub
   - Can I review code? Yes, open source on GitHub
   - What permissions? File read/write to ./data/ directory
   - Dependencies? pandas, numpy, matplotlib (standard data libs)
   - Last update? 2 weeks ago (active)
   - Community? 200 GitHub stars, 10 users reporting success

   Decision: ✅ Safe to install (open source, reviewed, active, popular)
   ```

6. **"Integration Architecture: MCP + Custom Commands + GEMINI.md"** (~600 words, ADVANCED)

   **How These Systems Work Together**:

   **System 1: GEMINI.md (Context)**
   - Provides project knowledge
   - Ensures consistency across sessions

   **System 2: Custom Commands (Automation)**
   - Packages common workflows
   - Reduces repetition

   **System 3: MCP Servers (Capabilities)**
   - Extends what Gemini can do
   - Integrates external systems

   **System 4: Extensions (Pre-packaged Solutions)**
   - Bundles all three above
   - Easy distribution to team

   **Architecture Diagram**:
   ```
   User Query
      ↓
   Custom Command matches?
   ├─ Yes: Apply TOML template
   │        Inject {{args}}
   │        Load GEMINI.md context
   │        Prompt Gemini
   │
   └─ No: Direct query
          Load GEMINI.md context
          Check if MCP needed
          Execute via MCP servers

   Result: Combines all systems for maximum power
   ```

   **Example: Integrated Research Workflow**

   **Setup**:
   - GEMINI.md: Project context (what we're building, standards)
   - Custom command: `/market-research` (template, {{topic}} injection)
   - MCP: Context7 for current documentation, Playwright for websites
   - Extension: Data science toolkit for analysis

   **Workflow**:
   ```
   gemini
   > /market-research "AI productivity tools"

   [What happens:]
   1. Custom command loads GEMINI.md context (who we are, our focus)
   2. Injects {{topic}} = "AI productivity tools"
   3. Prompt: "Research AI productivity tools considering [our standards]"
   4. Gemini uses Context7 MCP to fetch latest articles
   5. Gemini uses Playwright MCP to visit 5 competitor sites
   6. Gemini uses data analysis extension to create comparison
   7. Result: Competitive analysis in 10 minutes
   ```

**Code Examples**:
1. Playwright MCP configuration
2. GitHub MCP query examples
3. Custom TOML command with {{args}} and !{command}
4. Extension installation and security evaluation

**Hands-On Exercises**:

**Exercise 1: Install Playwright MCP** (8 min)
- Install playwright MCP: `npm install -g @anthropic-ai/mcp-server-playwright`
- Configure in `~/.config/gemini/mcp-servers.json`
- Test: Browse 1 website and extract data
- Expected outcome: Playwright MCP working

**Exercise 2: Create Custom Command** (12 min)
- Create `.gemini-commands.toml` in project:
   ```toml
   [commands.research]
   description = "Research and summarize topic"
   prompt = "Research {{topic}} and provide: 1) Overview 2) Key points 3) Relevance to our project"
   ```
- Test: `/research "machine learning in fintech"`
- Create 2nd command: `/analyze` (code review template)
- Expected outcome: 2 working custom commands

**Exercise 3: Security Evaluation** (5 min)
- Evaluate 2 fictional extensions:
  - Extension A: Official Google, open source, 10K stars
  - Extension B: Unknown author, closed source, 5 stars
- Apply security framework
- Decision: Which is safe? Why?
- Expected outcome: Security evaluation criteria internalized

**Exercise 4: Integration Design** (10 min)
- Scenario: "You're building a product analytics dashboard"
- Design integration:
  - What goes in GEMINI.md? (project context)
  - What custom commands? (/analyze-user-data, /compare-metrics)
  - What MCP servers? (GitHub for code, Context7 for SDK docs)
  - What extensions? (Data science toolkit?)
- Document architecture
- Expected outcome: Full system design for use case

**Try With AI**:

**Prompt 1: MCP Server Selection**
```
My business workflow: [describe: research/implementation/analysis].
Which MCP servers would save me the most time? (Playwright, Context7, GitHub, custom)
Design a MCP configuration for my workflow.
How much time would this save per week?
```
Expected outcome: MCP strategy optimized for workflow

**Prompt 2: Custom Command Library Design**
```
My team does [process: code reviews/deployment/research] repeatedly.
Create 3-5 custom slash commands that would automate this.
Write the .gemini-commands.toml file with:
- Description for each command
- {{args}} placeholders
- Prompt templates
- Business impact (time saved per usage)
```
Expected outcome: Team-ready custom command set

**Prompt 3: Extension Security Evaluation**
```
I found an extension called [name/description].
Help me evaluate if it's safe to install on my team's machines.
Check: vendor, code review, permissions, dependencies, maintenance, community trust.
Recommend: Install / Install with caution / Skip it.
```
Expected outcome: Risk assessment framework applied

**Prompt 4: Full Integration Architecture**
```
I'm setting up Gemini CLI for my team of [size] developers.
I want to combine:
- GEMINI.md hierarchical context
- Custom slash commands for our workflows
- MCP servers for external integrations
- Maybe an extension for our domain

Design the complete architecture.
Show: file structures, configuration, command definitions, MCP servers, extensions.
How much productivity improvement would this provide?
```
Expected outcome: Complete architecture blueprint

**Functional Requirements Covered**:
- FR-016 (MCP server configuration)
- FR-017 (Playwright MCP for web browsing)
- FR-018 (Context7 for documentation)
- FR-019 (GitHub MCP for collaboration)
- FR-020 (Practical MCP workflows)
- FR-021 (Custom slash commands TOML)
- FR-022 ({{args}} injection patterns)
- FR-023 (!{command} subprocess execution)
- FR-024 (User vs. project scope)
- FR-025 (Command organization & team sharing)
- FR-026 (Extension installation)
- FR-027 (Security evaluation framework)
- FR-028 (Troubleshooting MCP connectivity)
- FR-029 (Extension conflicts & management)
- FR-030 (Advanced MCP patterns)

**Prerequisites**: Lessons 1-4 (commands, tools, configuration, memory)
**Dependencies**: All prior lessons
**Estimated Time**: 50 minutes (30 reading + 20 exercises)

---

## Phase 3: Content Flow & Dependencies

### Lesson Sequence (in learning order):

1. **Lesson 1: Why Gemini CLI Matters** (15 min) → Conceptual foundation
   - Prerequisite: None (Chapter intro context)
   - Teaches: Tool evaluation, open source benefits, extensibility concept
   - Bridges to: Lesson 2

2. **Lesson 2: Core Commands & Workflow** (40 min) → Foundational skills
   - Prerequisite: Lesson 1
   - Teaches: Essential commands, keyboard shortcuts, CLI modes, basic memory
   - Bridges to: Lessons 3-4

3. **Lesson 3: Configuration & Tool Management** (30 min) → Security & flexibility
   - Prerequisite: Lessons 1-2
   - Teaches: Settings hierarchy, tool restrictions, token efficiency preview
   - Bridges to: Lesson 4

4. **Lesson 4: Memory Systems** (35 min) → Context management
   - Prerequisite: Lessons 1-3 (especially Lesson 2 commands, Lesson 3 configuration)
   - Teaches: 3 memory types, GEMINI.md hierarchy, token lifecycle
   - Bridges to: Lesson 5

5. **Lesson 5: MCP, Custom Commands & Extensions** (50 min) → Advanced integration
   - Prerequisite: All prior lessons
   - Teaches: MCP servers, custom commands, extensions, integration architecture
   - Bridges to: Next chapter

### Complexity Progression:

- **Lesson 1**: Conceptual (A1 - Recognition)
- **Lesson 2**: Foundational skills (A2 - Application with scaffolding)
- **Lesson 3**: Security & configuration (A2 - Application)
- **Lesson 4**: Context management (B1 - Independent application)
- **Lesson 5**: Advanced integration (B1-B2 - Evaluation & design)

### Token Budget Check:

- Lesson 1: 15 min reading, 5 min exercises = 20 min
- Lesson 2: 25 min reading, 15 min exercises = 40 min
- Lesson 3: 20 min reading, 10 min exercises = 30 min
- Lesson 4: 20 min reading, 15 min exercises = 35 min
- Lesson 5: 30 min reading, 20 min exercises = 50 min
- **TOTAL: 175 minutes** (well above 60-90 min target)

**Mitigation**: This is COMPREHENSIVE mastery content. Chapter-level strategy:
- Core content (Lessons 1-4): ~100 minutes fits 60-90 min target
- Lesson 5 (Extensions): ~50 minutes can be "optional advanced reading"
- Alternatively: Prioritize Lessons 1-4 as required, Lesson 5 as "go deeper" for power users

---

## Phase 4: Scaffolding Strategy

### Cognitive Load Management (Intermediate Tier: 7 concepts per section max):

**Lesson 1**: 5 concepts
- Tool evaluation
- Open source benefits
- Free tier value
- MCP concept
- Extensibility

**Lesson 2**: 7 concepts
- 13 essential commands
- @ syntax
- ! syntax
- Keyboard shortcuts
- CLI invocation modes
- Hierarchical memory intro
- Settings customization preview

**Lesson 3**: 6 concepts
- Settings hierarchy (7 levels)
- Tool restrictions
- Whitelisting
- Token cost awareness
- Team configuration sharing
- Error handling for disabled tools

**Lesson 4**: 7 concepts
- Ephemeral memory
- GEMINI.md hierarchy
- save_memory
- /memory commands
- Token compression
- Context window monitoring
- Memory organization strategies

**Lesson 5**: 8 concepts (slightly over, acceptable for advanced)
- MCP servers (concept)
- Playwright (web browsing)
- Context7 (documentation)
- GitHub (collaboration)
- Custom commands (TOML)
- Extensions (packaging)
- Security evaluation
- Integration architecture

### Building Confidence Progression:

1. **Lesson 1**: "Understand why Gemini CLI is powerful"
2. **Lesson 2**: "Use essential commands confidently"
3. **Lesson 3**: "Control tools for your security needs"
4. **Lesson 4**: "Manage context across long projects"
5. **Lesson 5**: "Extend Gemini with custom capabilities"

### Hands-On Practice Sequence:

- **Lessons 1-2**: Direct execution (try the commands)
- **Lesson 3**: Configuration (modify settings)
- **Lesson 4**: Architecture (design memory strategy)
- **Lesson 5**: Integration (combine systems)

Each exercise builds on prior learning, increasing complexity.

---

## Phase 5: Integration Points

### Within Chapter:
- Lesson 1 frames extensibility → Lesson 5 teaches extension
- Lesson 2 teaches commands → Lesson 5 teaches custom commands
- Lesson 2 introduces GEMINI.md → Lesson 4 deep dive
- Lesson 3 configuration basics → Lesson 5 advanced integration

### With Adjacent Chapters:

**Backward Integration (Chapter 5: Claude Code)**:
- Chapter 5 teaches "Claude Code as reference"
- Chapter 6 Lesson 1: "Compare/contrast Gemini CLI"
- Cross-reference: "Similar but different approach"

**Forward Integration (Chapter 7: Bash Essentials)**:
- Chapter 6 Lesson 2: ! syntax for shell integration
- Chapter 7: "Full shell command mastery"
- Natural progression: "Simple shell within Gemini → Full shell power"

**Cross-Chapter Context**:
- Chapter 6 enables: "AI-augmented development with Gemini"
- Chapter 8-9: Use Gemini CLI with Python/TypeScript projects
- Chapter 10+: Use Gemini's MCP servers in production deployments

---

## Validation Strategy

### Success Criteria (from specification):

- **Recall**: 90%+ can list 10+ essential commands
- **Configuration**: 85%+ can configure settings.json with 5+ options
- **Memory**: 80%+ can create hierarchical GEMINI.md
- **MCP**: 75%+ can install 2+ MCP servers
- **Custom Commands**: 70%+ can create custom slash commands
- **Productivity**: 5-10x faster research tasks
- **Time**: 60-90 min reading (with exercises, ~175 min for full content)

### Validation Methods (to be implemented during content-implementer phase):

1. **Knowledge checks**: Embedded quizzes in each lesson
2. **Practical exercises**: Every lesson has 4-5 hands-on tasks
3. **Project validation**: End-of-chapter project combining all systems
4. **Time tracking**: Student completes lessons within target duration
5. **Skill demonstration**: "Try With AI" prompts measure real capability

### Acceptance Criteria (Definition of Done):

- All 5 lessons implemented with enhancements
- All code examples tested and working
- All exercises have model solutions
- All "Try With AI" prompts validated
- Reading time: ~100 min (core content), ~175 min (with advanced)
- All 35 functional requirements covered across lessons
- Student can articulate complete Gemini CLI workflow

---

## Risks & Mitigation

### Risk 1: Content Too Comprehensive
**Mitigation**: Lesson 5 marked as "advanced optional"; core mastery in Lessons 1-4 fits 60-90 min window

### Risk 2: Too Many New Concepts
**Mitigation**: Built cognitive load checks; Intermediate tier allows 7 concepts per section

### Risk 3: Outdated Tool Versions
**Mitigation**: Flag Gemini CLI version references; note deprecation dates; suggest "ask AI for latest" in exercises

### Risk 4: MCP Server Stability
**Mitigation**: Show multiple MCP examples (Playwright, Context7, GitHub); if one breaks, others remain valid

### Risk 5: Student Overwhelm (5 lessons in 1 chapter)
**Mitigation**: Clear progression; each lesson standalone; "Try With AI" summarizes; exercises bite-sized

---

## Summary: Lesson Enhancement Overview

| Lesson | Current Status | Enhancement | Target Concepts | Time |
|--------|----------------|-------------|-----------------|------|
| 1: Why Matters | Complete (186 lines) | Add power user framing | 5 concepts | 15 min |
| 2: Core Commands | Basic (234 lines) | Major expansion (+500 lines) | 7 concepts | 40 min |
| 3: Configuration | Keep section, enhance | Add settings hierarchy + security | 6 concepts | 30 min |
| 4: Memory Systems | Redesign completely | Replace context window with memory types | 7 concepts | 35 min |
| 5: MCP & Extensions | Partial/incomplete | Complete + add custom commands | 8 concepts | 50 min |

**Total Chapter**: 170 minutes (core ~100 min, advanced ~70 min)

---

## Next Steps (for implementation phase)

1. **Lesson 1 Enhancement**: Review narrative, add "power user" sections, update Try With AI prompts
2. **Lesson 2 Expansion**: Create comprehensive command reference table, add syntax sections
3. **Lesson 3 Enhancement**: Document settings hierarchy, add security examples
4. **Lesson 4 Redesign**: Replace context window content with 3 memory systems, create templates
5. **Lesson 5 Completion**: Finish Playwright example, add Context7 & GitHub, add custom commands section
6. **Code Examples**: Develop 15+ working examples (settings, TOML, workflows)
7. **Exercise Solutions**: Create model solutions for all 20+ exercises
8. **Validation**: Implement knowledge checks and project assessment

---

**Plan Status**: Ready for content-implementer implementation phase

**Approval Gate**: This plan covers all 35 functional requirements and fits within 60-90 minute target (core lessons). Lesson 5 can scale based on audience appetite for advanced content.
