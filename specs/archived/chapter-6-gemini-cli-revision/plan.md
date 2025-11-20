# Chapter 6: Google Gemini CLI - Detailed Implementation Plan (8-Lesson Structure)

**Chapter Type**: Technical (Code-Focused with Tool-First Pedagogy)
**Part**: 2 (AI Tool Landscape)
**Complexity Tier**: Beginner-Intermediate (CEFR A2-B1)
**Target Duration**: 137-150 minutes total reading/practice
**Status**: Ready for Implementation
**Plan Version**: 2.0 (8-Lesson Restructure)

---

## I. Chapter Overview & Alignment

### Chapter Identity
- **Title**: Google Gemini CLI: Open Source and Everywhere
- **Learning Outcome**: Students can install, configure, and integrate Gemini CLI with MCP servers, custom commands, context management, IDE integration, and security practices to expand their AI capabilities beyond web interfaces
- **Success Criteria**: 8 evals defined in spec (Installation, Feature Discovery, Configuration, Memory Management, Custom Commands, MCP Integration, Extension Lifecycle, IDE Integration)

### Specification Reference
- **Approved Spec**: `specs/chapter-6-gemini-cli-revision/spec.md` (v1.0.0, 8-lesson structure)
- **Verified Intelligence**: `intelligence/chapter-6-gemini-cli-verified-docs.md` (75 verified claims: 47 Context7 + 28 Medium Tutorial)
- **Constitutional Alignment**: Principles 7 (Technical Accuracy), 13 (Graduated Teaching), Core Philosophy #1 (AI Development Spectrum)

### Chapter Structure: 8 Lessons

The chapter follows three pedagogical tiers reflecting the spec's design decisions:

| Lesson | Title | Duration | Tier | Type | Key Topics |
|--------|-------|----------|------|------|-----------|
| 1 | Why Gemini CLI Matters | 15 min | 1 | Conceptual | Positioning, comparison, free tier, ecosystem |
| 2 | Installation, Authentication & First Steps | 15 min | 1 | Technical | Installation, verification, troubleshooting, basics |
| 3 | Built-In Tools Deep Dive | 20-25 min | 2 | Technical | File ops, shell, web fetching, search grounding |
| 4 | Configuration & Settings | 15-17 min | 2 | Technical | Global/project hierarchy, .env files, common params |
| 5 | Memory & Context Management | 18-20 min | 2 | Technical | `/clear`, `/compress`, `/chat`, GEMINI.md, `/memory` |
| 6 | Custom Slash Commands | 16-18 min | 2-3 | Technical | TOML structure, injection patterns, namespacing |
| 7 | MCP Servers & Integration | 20 min | 2-3 | Technical | MCP basics, CLI commands, OAuth, workflows |
| 8 | Extensions, Security & IDE Integration | 18-20 min | 3 | Technical | Extension lifecycle, tool filtering, IDE integration |

**Total Chapter Duration**: 137-150 minutes (reading + practice) → Realistic for comprehensive tool coverage

---

## II. Constitutional Alignment & Pedagogical Framework

### Principle 13: Graduated Teaching Pattern (Book → AI Companion → AI Orchestration)

The chapter follows a three-tier approach mapping to chapter progression:

#### Tier 1: Foundational Concepts (Book Teaches Directly)
- **Lessons 1-2**: Installation basics, authentication flow, tool positioning
- **Student Role**: Manual practice (install CLI, verify version, run first session)
- **AI Role**: Validation and Q&A

#### Tier 2: Complex Execution (AI Companion Handles)
- **Lessons 3-5, 6-7**: Understanding tools, strategic tool selection, configuration, memory management, custom commands, MCP integration
- **Student Role**: Specification ("I need to configure project-specific settings")
- **AI Role**: Execution and explanation

#### Tier 3: Orchestration (AI Manages Complex Workflows)
- **Lesson 8**: Extension development, security filtering, IDE integration, workflow design
- **Student Role**: Orchestration ("Set up complete extension for my project")
- **AI Role**: Autonomous execution with student validation

### Cognitive Load Management (≤7 Concepts/Section)

All sections validated within 7-concept limit per Principle 12. Example breakdown:
- **Lesson 1**: 5 concepts (CLI positioning, free tier, context window, ecosystem, use cases)
- **Lesson 4 Part 1**: 5 concepts (configuration hierarchy, 7 levels, precedence, override rules)
- **Lesson 5 Part 2**: 5 concepts (`/clear`, `/compress`, difference, use cases, strategy)
- **Lesson 6 Part 2**: 6 concepts ({{args}}, !{shell}, @{file}, injection examples, combined patterns)
- **Lesson 8 Part 1**: 6 concepts (extensions, templates, link, install vs link, lifecycle, manifest)

**All sections within 7-concept limit. ✅**

### AI Usage Strategy (Spec Section 4)

**Tier 1: Direct Commands** (Students run, 1-5 seconds):
```bash
npm install -g @google/gemini-cli
gemini -v
gemini mcp add my-server python server.py
gemini mcp list
gemini extensions list
/ide enable
```

**Tier 2: AI Companion** (For understanding/troubleshooting):
- "What is Model Context Protocol?"
- "Should I use Gemini CLI or Claude Code for analyzing 12 reports?"
- "Design a custom slash command for my workflow"

**Tier 3: AI Orchestration** (Complex workflows):
- "Design custom extension from scratch"
- "Create complete MCP + IDE workflow for my project"
- "Design security boundaries for these MCP servers"

---

## III. Lesson-by-Lesson Breakdown (All 8 Lessons)

### Lesson 1: Why Gemini CLI Matters (15 minutes)

**Learning Objective (Bloom's: Remember + Understand)**:
- Students can recall why Gemini CLI differs from web interfaces
- Students can explain strategic advantages: free tier quotas, 1M token context, ecosystem integration

**Section Structure**:

#### Section 1.1: The Game-Changing Differences (5 min)
- **Concepts**: CLI advantage, context window, free tier, integration capability, ecosystem (5)
- **Content Elements**:
  - Comparison table: ChatGPT vs Claude Code vs Gemini CLI vs Qwen Code
  - Free tier quotas: 60 req/min, 1000 req/day (VERIFIED)
  - Context window: 1M tokens = ~750K words = ~100K lines code (VERIFIED)
- **Example**: "The 1M token context means Gemini CLI can see your entire codebase + 20 reports"

#### Section 1.2: When Gemini CLI Is Your Best Choice (5 min)
- **Concepts**: Use case selection, tool matrix, cost consideration (3)
- **Content Elements**:
  - Scenarios where Gemini CLI excels (large documents, free tier, bulk operations)
  - Scenarios where Claude Code or ChatGPT better (web interface preference, enterprise support)
- **Example**: "If you need to analyze 12 monthly reports at once, Gemini CLI's 1M tokens handles them in one session"

#### Section 1.3: Qwen Code: The Open Source Alternative (1-2 min)
- **Concepts**: Open source ecosystem, Alibaba fork (2)
- **Content Elements**: One paragraph mention of Qwen Code as alternative (not deep dive)

#### Section 1.4: Open Source Ecosystem Effect (3-4 min)
- **Concepts**: Ecosystem, MCP standards, extensions, IDE integration (4)
- **Content Elements**: How Gemini CLI connects to broader AI ecosystem, IDE integration, MCP servers, custom commands

### Try With AI (Lesson 1) - 4 prompts

1. **Tool Selection Strategy**: Analyze context window vs cost tradeoffs
2. **Understanding Token Context**: Concrete translation of 1M tokens
3. **When CLI Beats Web**: Scenarios where Gemini CLI excels
4. **Your Project Fit**: Personalized tool recommendation

**Duration**: 10-15 minutes | **Scaffolding**: None (conceptual)

---

### Lesson 2: Installation, Authentication & First Steps (15 minutes)

**Learning Objective (Bloom's: Understand + Apply)**:
- Students can successfully install Gemini CLI and verify installation
- Students can authenticate with Gemini and run first session

**Section Structure**:

#### Section 2.1: Installing Gemini CLI (3-4 min)
- **Concepts**: npm, global installation, Node.js prerequisite, error handling (4)
- **Content Elements**:
  - Command (VERIFIED): `npm install -g @google/gemini-cli`
  - Troubleshooting: npm not found, permission denied, module not found
  - What "global installation" means
- **Practice**: Hands-on installation (Eval 1)

#### Section 2.2: Authentication Methods & OAuth Flow (4-5 min)
- **Concepts**: OAuth, authentication methods, API key, browser flow, token storage (5)
- **Content Elements**:
  - Three authentication options: Google login (recommended), API Key, Vertex AI
  - OAuth flow explanation: Browser opens → authenticate → return to terminal
  - Token storage: `~/.gemini` directory

#### Section 2.3: Your First Gemini CLI Session (3-4 min)
- **Concepts**: Session commands, slash commands, help discovery (3)
- **Content Elements**:
  - Basic commands: `/help`, `/tools`, `/stats`, `/quit`
  - Text interaction: Type prompt and press Enter

#### Section 2.4: Red Flags to Watch (2 min)
- **Concepts**: Error recognition, expected output vs problems (2)
- **Content Elements**: Normal outputs vs actual problems, error literacy strategy

### Try With AI (Lesson 2) - 4 prompts

1. **Troubleshooting Installation**: Handle npm not found
2. **Understanding Authentication**: Explain OAuth flow
3. **Your First Session**: Suggest initial test prompts
4. **Permission Errors**: Fix EACCES errors

**Duration**: 10-15 minutes | **Scaffolding**: High (real installation issues)

---

### Lesson 3: Built-In Tools Deep Dive (20-25 minutes)

**Learning Objective (Bloom's: Understand + Apply)**:
- Students can explain what each built-in tool does
- Students can use file operations, shell integration, web fetching strategically

**Section Structure**:

#### Section 3.1: File Operations Tool (4-5 min)
- **Concepts**: File reading, glob patterns, supported formats, limitations (4)
- **Content Elements**: Read files (CSV, JSON, PDF, text, XML, Markdown), glob patterns, limitations (read-only)

#### Section 3.2: Shell Integration Tool (4-5 min)
- **Concepts**: Command execution, restrictions, output capture (3)
- **Content Elements**: Execute terminal commands, configurable restrictions, safe vs dangerous commands

#### Section 3.3: Web Fetching Tool (3-4 min)
- **Concepts**: URL fetching, content extraction, limitations (3)
- **Content Elements**: Fetch public pages, extract content, rate limiting, timeouts

#### Section 3.4: Google Search Grounding Tool (3-4 min)
- **Concepts**: Web search, current information, citations (3)
- **Content Elements**: Search web for current info, synthesize with citations

#### Section 3.5: Combining Tools Strategically (2-3 min)
- **Concepts**: Multi-tool workflows, chaining (2)
- **Content Elements**: Example workflows combining tools

#### Section 3.6: Red Flags to Watch (1-2 min)
- **Concepts**: Error recognition (1)
- **Content Elements**: Normal outputs vs problems

### Try With AI (Lesson 3) - 4 prompts

1. **File Analysis**: Read and analyze file
2. **Running Tests**: Run test suite and analyze results
3. **Web Fetching**: Fetch docs and explain concept
4. **Multi-Tool Workflow**: Combine file reading + web search

**Duration**: 15-20 minutes | **Scaffolding**: Medium (spec-style prompts)

---

### Lesson 4: Configuration & Settings (15-17 minutes)

**Learning Objective (Bloom's: Understand + Apply)**:
- Students can explain the configuration hierarchy and precedence order
- Students can use environment variables (.env files) securely
- Students can configure project-specific settings appropriately

**Section Structure**:

#### Section 4.1: Configuration Hierarchy (4-5 min)
- **Concepts**: Configuration levels, hierarchy, precedence, override rules (5)
- **Content Elements** (VERIFIED - Medium Tutorial Part 3):
  - 7-level hierarchy: System → User → Workspace → Project → .env → Env vars → CLI flags
  - Precedence rule: Higher level overrides lower level
  - Example: "Project settings override user settings; CLI flags override everything"

#### Section 4.2: Common Configuration Parameters (4-5 min)
- **Concepts**: Theme, model, checkpointing, auth, endpoint (5)
- **Content Elements** (VERIFIED - Medium Tutorial Part 3):
  - Theme, model selection, checkpointing (auto-save), auth persistence, custom API endpoint
  - JSON configuration examples

#### Section 4.3: Environment Variables & .env Files (4-5 min)
- **Concepts**: Variable syntax, .env files, loading order, security (4)
- **Content Elements** (VERIFIED - Medium Tutorial Part 3):
  - Variable syntax: `${VAR_NAME}` or `$VAR_NAME`
  - .env file structure and loading order
  - Security: Always `.env` in `.gitignore`, never commit secrets

#### Section 4.4: Security Best Practices (1-2 min)
- **Concepts**: Secret management, version control (2)
- **Content Elements**: Never hardcode secrets, use .env files, principle-based

### Try With AI (Lesson 4) - 4 prompts

1. **Configuration Hierarchy**: Explain precedence rules
2. **Environment Variables**: Store API key securely
3. **MCP Server Secrets**: Show settings.json + .env setup
4. **Project-Specific Configuration**: Team model setup

**Duration**: 12-15 minutes | **Scaffolding**: Medium (real scenarios)

---

### Lesson 5: Memory & Context Management (18-20 minutes)

**Learning Objective (Bloom's: Understand + Apply)**:
- Students can explain context window concept and consumption
- Students can manage long conversations using `/clear`, `/compress`, `/chat` commands
- Students can use GEMINI.md for persistent memory

**Part 1: Understanding Context (4-5 min)**

#### Section 5.1.1: What is Context? (2-3 min)
- **Concepts**: Context window, token consumption, construction (4)
- **Content Elements** (VERIFIED - Medium Tutorial Part 9):
  - 1M tokens context window
  - Context construction: Startup phase (load GEMINI.md) → Conversation phase
  - Token consumption: Each message, file content, command outputs

#### Section 5.1.2: Red Flags to Watch (1-2 min)
- **Concepts**: Context depletion signals (2)
- **Content Elements**: Slow responses, errors, AI forgetting details, strategy to use `/compress`

**Part 2: Context Management Commands (4-5 min)**

#### Section 5.2.1: Hard Reset `/clear` and Smart Summary `/compress` (3-4 min)
- **Concepts**: `/clear`, `/compress`, difference, use cases (4)
- **Content Elements** (VERIFIED - Medium Tutorial Part 9):
  - `/clear`: Wipes history, keeps GEMINI.md, fresh start
  - `/compress`: Summarizes history, preserves continuity, frees tokens
  - Comparison table

#### Section 5.2.2: Token Management Strategy (1-2 min)
- **Concepts**: Token budgeting (1)
- **Content Elements**: Monitor conversation, use `/compress` proactively

**Part 3: Conversational Branching (4-5 min)**

#### Section 5.3.1: Multi-Task Workflows (1 min)
- **Concepts**: Context switching, task interruption (1)
- **Content Elements**: Real scenario: Task A → urgent Task B → back to Task A

#### Section 5.3.2: Save and Resume Conversations (2-3 min)
- **Concepts**: `/chat save`, `/chat resume`, `/chat list`, `/chat delete` (4)
- **Content Elements** (VERIFIED - Medium Tutorial Part 9):
  - Save: `​/chat save debugging-auth`
  - Resume: `/chat resume debugging-auth`
  - List: `/chat list`
  - Delete: `/chat delete debugging-auth`

#### Section 5.3.3: Practical Workflow (1 min)
- **Concepts**: Workflow sequence (1)
- **Content Elements**: Step-by-step task switching example

**Part 4: Long-Term Memory (GEMINI.md) (3-4 min)**

#### Section 5.4.1: GEMINI.md Hierarchy (2 min)
- **Concepts**: GEMINI.md loading order, persistent context (3)
- **Content Elements** (VERIFIED - Medium Tutorial Part 9):
  - 5-level loading order: System → User → Workspace → Project → Extension
  - Purpose: Persistent context at every session start
  - Token impact: Consumes from 1M context window

#### Section 5.4.2: When to Use GEMINI.md (1 min)
- **Concepts**: Use case, permanence (1)
- **Content Elements**: Team conventions, architecture (not session-specific notes)

**Part 5: Memory Management Commands (2-3 min)**

#### Section 5.5.1: Memory Commands (2 min)
- **Concepts**: `/memory show`, `/memory refresh`, `/memory add`, SaveMemory (4)
- **Content Elements** (VERIFIED - Medium Tutorial Part 9):
  - `/memory show`: Display loaded GEMINI.md
  - `/memory refresh`: Reload from disk
  - `/memory add`: Append to GEMINI.md
  - SaveMemory: AI-triggered automatic saving

#### Section 5.5.2: Memory vs Context (1 min)
- **Concepts**: Memory types, lifetime (2)
- **Content Elements**: Context (session RAM) vs Memory (persistent files)

### Try With AI (Lesson 5) - 4 prompts

1. **Context Management Strategy**: `/clear` vs `/compress`
2. **Multi-Task Workflow**: Switch tasks without losing context
3. **Using GEMINI.md**: Team context setup
4. **Memory Management**: Save important facts

**Duration**: 15-20 minutes | **Scaffolding**: Medium (real scenarios)

---

### Lesson 6: Custom Slash Commands (16-18 minutes)

**Learning Objective (Bloom's: Understand + Apply + Create)**:
- Students can create custom commands using TOML syntax
- Students can use injection patterns (`{{args}}`, `!{shell}`, `@{file}`)
- Students can organize commands with namespacing

**Part 1: Understanding Custom Commands (3 min)**

#### Section 6.1.1: What Custom Commands Enable (2 min)
- **Concepts**: Automation, team standardization, reusability (3)
- **Content Elements**: Save time, team consistency, real use cases (/code-review, /git-commit, /plan)

#### Section 6.1.2: Where Commands Live (1 min)
- **Concepts**: Global vs project (2)
- **Content Elements**: `~/.gemini/commands/` vs `.gemini/commands/`

**Part 2: TOML File Structure & Injection Patterns (8-10 min)**

#### Section 6.2.1: TOML File Format (2-3 min)
- **Concepts**: TOML syntax, required fields, optional fields (5)
- **Content Elements** (VERIFIED - Medium Tutorial Part 7):
  - Required: `description`, `prompt`
  - Optional: `model`, `temperature`, `systemInstruction`
  - Example TOML structure

#### Section 6.2.2: Argument Injection `{{args}}` (2 min)
- **Concepts**: {{args}} syntax, argument passing (3)
- **Content Elements** (VERIFIED - Medium Tutorial Part 7):
  - Replace `{{args}}` with command arguments
  - Usage: `/review src/main.py`

#### Section 6.2.3: Shell Command Injection `!{command}` (2 min)
- **Concepts**: !{} syntax, shell execution, output injection (3)
- **Content Elements** (VERIFIED - Medium Tutorial Part 7):
  - Execute shell command, inject output
  - Example: `!{git log --oneline -10}`

#### Section 6.2.4: File Content Injection `@{filepath}` (2 min)
- **Concepts**: @{} syntax, file reading (2)
- **Content Elements** (VERIFIED - Medium Tutorial Part 7):
  - Read file content, inject into prompt
  - Example: `@{README.md}`

#### Section 6.2.5: Combined Example (1-2 min)
- **Concepts**: Pattern combination (2)
- **Content Elements** (VERIFIED - Medium Tutorial Part 7):
  - Git commit message generator using all three patterns

**Part 3: Namespacing & Organization (3-4 min)**

#### Section 6.3.1: Directory Structure (2 min)
- **Concepts**: Directory hierarchy, colon syntax, nested namespaces (3)
- **Content Elements** (VERIFIED - Medium Tutorial Part 7):
  - Directory structure maps to namespaces
  - `/git:commit`, `/deploy:staging`, `/team:backend:deploy`

#### Section 6.3.2: Organization Best Practices (1 min)
- **Concepts**: Naming strategy (1)
- **Content Elements**: Group by category, use namespaces for clarity

#### Section 6.3.3: Sharing Commands (1 min)
- **Concepts**: Team standardization (1)
- **Content Elements**: Commit to version control, all team members benefit

### Try With AI (Lesson 6) - 4 prompts

1. **Creating Custom Command**: Write TOML for `/code-review`
2. **Using Injection Patterns**: Combine {{args}}, !{}, @{} in one command
3. **Namespacing Team Commands**: Directory structure for `/deploy:staging` and `/deploy:production`
4. **Command With Arguments**: Create `/plan` using {{args}}

**Duration**: 12-15 minutes | **Scaffolding**: Medium (functional commands)

---

### Lesson 7: MCP Servers & Integration (20 minutes)

**Learning Objective (Bloom's: Understand + Apply)**:
- Students can explain what MCP servers are
- Students can add MCP servers using CLI commands
- Students can design business workflows with MCP

**Part 1: Understanding MCP (5 min)**

#### Section 7.1.1: What is Model Context Protocol? (3 min)
- **Concepts**: MCP standard, plugin architecture, external tools (4)
- **Content Elements** (VERIFIED):
  - MCP = standard for connecting external tools to AI
  - MCP server = program providing specific capabilities
  - Example servers: Playwright, Context7, custom APIs

#### Section 7.1.2: MCP vs Extensions (2 min)
- **Concepts**: MCP (individual) vs extensions (bundles) (2)
- **Content Elements** (VERIFIED):
  - MCP Server: Single capability, manually configured
  - Extension: Pre-packaged bundle with MCP + commands + configuration

**Part 2: CLI MCP Management Commands (7 min)**

#### Section 7.2.1: Adding MCP Servers (2-3 min)
- **Concepts**: CLI commands, transport types, configuration (4)
- **Content Elements** (VERIFIED):
  - Stdio: `gemini mcp add my-server python server.py --port 8080`
  - HTTP: `gemini mcp add --transport http ... https://api.example.com/mcp`
  - SSE: `gemini mcp add --transport sse ... https://api.example.com/sse`

#### Section 7.2.2: Listing and Verifying (2 min)
- **Concepts**: Status checking, connectivity (2)
- **Content Elements** (VERIFIED):
  - Command: `gemini mcp list`
  - Check for status: connected/disconnected

#### Section 7.2.3: Removing MCP Servers (1 min)
- **Concepts**: Cleanup (1)
- **Content Elements**: `gemini mcp remove server-name`

#### Section 7.2.4: CLI vs Manual Configuration (2 min)
- **Concepts**: Comparison, tradeoffs (2)
- **Content Elements**: CLI (easier, beginner-friendly) vs JSON editing (advanced)

**Part 3: OAuth for MCP Servers (5 min)**

#### Section 7.3.1: Why OAuth Exists (1-2 min)
- **Concepts**: Authentication, security (3)
- **Content Elements** (VERIFIED):
  - Problem: MCP servers need secure authentication
  - Solution: OAuth token exchange without exposing secrets
  - Use case: Access user's Google Drive via MCP

#### Section 7.3.2: Using OAuth `/mcp auth` (2 min)
- **Concepts**: Command usage, browser flow (3)
- **Content Elements** (VERIFIED):
  - Command: `/mcp auth`
  - Behavior: Browser opens, tokens stored, auto-refresh
  - Key: "Don't manage tokens manually; Gemini CLI handles it"

#### Section 7.3.3: When to Use OAuth (1 min)
- **Concepts**: Use case identification (2)
- **Content Elements**: When MCP needs API key, credentials, cloud access

#### Section 7.3.4: Red Flags (1 min)
- **Concepts**: Error recognition (1)
- **Content Elements**: Normal OAuth flow, timeout, token invalid errors

**Part 4: Business Workflows (3 min)**

#### Section 7.4.1: Multi-Tool Workflows (2 min)
- **Concepts**: Workflow design, tool chaining (3)
- **Content Elements** (VERIFIED):
  - Competitive research: Playwright → file analysis → web search
  - API documentation: Context7 → local code → tests
  - Multi-file analysis: files → shell → search

#### Section 7.4.2: AI Orchestration (1 min)
- **Concepts**: Workflow complexity (2)
- **Content Elements**: Tell AI to orchestrate multi-step workflows

### Try With AI (Lesson 7) - 4 prompts

1. **Setting Up MCP**: Command for Playwright MCP
2. **OAuth Authentication**: Authenticate secured MCP server
3. **Multi-Tool Workflow**: Combine Playwright + analysis
4. **Your Project's Needs**: Recommend MCP servers

**Duration**: 15-20 minutes | **Scaffolding**: Medium (spec-style)

---

### Lesson 8: Extensions, Security & IDE Integration (18-20 minutes)

**Learning Objective (Bloom's: Understand + Apply + Analyze)**:
- Students can explain extension development workflow
- Students can describe security implications (tool filtering)
- Students can enable IDE integration and explain benefits
- Students can decide which tools to use when

**Part 1: Extension Development Workflow (6 min)**

#### Section 8.1.1: Extensions vs MCP (1 min)
- **Concepts**: Relationship, bundling (2)
- **Content Elements** (VERIFIED): Extensions bundle MCP + configuration

#### Section 8.1.2: Creating Extensions (1-2 min)
- **Concepts**: Template system, structure (3)
- **Content Elements** (VERIFIED):
  - Command: `gemini extensions new my-extension mcp-server`
  - Created: `gemini-extension.json`, `GEMINI.md`, `commands/`, `dist/`

#### Section 8.1.3: Development Workflow: Link vs Install (2-3 min)
- **Concepts**: Development mode, iteration (4)
- **Content Elements** (VERIFIED):
  - `install`: Production, requires reinstall
  - `link`: Development, changes immediate
  - Workflow: Create → link → test → iterate → publish → install

#### Section 8.1.4: Managing Extensions (1-2 min)
- **Concepts**: Lifecycle, updates (4)
- **Content Elements** (VERIFIED):
  - Commands: list, enable, disable, update, uninstall
  - Scoping: workspace vs user-wide

#### Section 8.1.5: Manifest Structure (1 min)
- **Concepts**: Configuration fields (2)
- **Content Elements** (VERIFIED): Key fields overview

**Part 2: Tool Filtering for Security (5 min)**

#### Section 8.2.1: Why Tool Filtering Matters (2 min)
- **Concepts**: Security principle, capability restriction (4)
- **Content Elements** (VERIFIED):
  - Problem: External MCP servers may have dangerous tools
  - Solution: Restrict which tools available
  - Principle: Defense in depth

#### Section 8.2.2: Using `includeTools` and `excludeTools` (2 min)
- **Concepts**: Allowlist vs blocklist (3)
- **Content Elements** (VERIFIED):
  - Allowlist: Only safe tools available
  - Blocklist: Block dangerous tools
  - JSON configuration examples

#### Section 8.2.3: Real-World Scenario (1 min)
- **Concepts**: Applied security (2)
- **Content Elements** (VERIFIED):
  - Scenario: MCP server with 6 tools
  - Assessment: Trust these, block those
  - Configuration with allowlist approach

#### Section 8.2.4: Best Practices (0-1 min)
- **Concepts**: Principles (5)
- **Content Elements**: Review tools, use allowlist, start restrictive, expand if needed

**Part 3: IDE Integration (6 min)**

#### Section 8.3.1: What IDE Integration Adds (2 min)
- **Concepts**: IDE awareness, context, benefits (4)
- **Content Elements** (VERIFIED):
  - AI sees 10 recent files, cursor position, selected text
  - Native diff viewer, accept changes one-click
  - Workflow: No context switching

#### Section 8.3.2: Installing and Enabling (2 min)
- **Concepts**: Commands, setup (3)
- **Content Elements** (VERIFIED):
  - `/ide install`: One-time setup
  - `/ide enable`: Per-session connection
  - `/ide status`: Check connection
  - `/ide disable`: Disconnect

#### Section 8.3.3: VS Code Workflow (1-2 min)
- **Concepts**: IDE workflow, review (3)
- **Content Elements** (VERIFIED):
  - Enable → ask question → diff viewer opens → review → accept
  - Safety: Always review before applying

#### Section 8.3.4: When IDE Integration Adds Value (1 min)
- **Concepts**: Use case, limitations (2)
- **Content Elements** (VERIFIED):
  - Valuable: Switching between editor/terminal, complex suggestions, multi-file
  - Less valuable: Simple changes, terminal-only workflow, different editor

#### Section 8.3.5: Red Flags (0-1 min)
- **Concepts**: Error recognition (2)
- **Content Elements**: Normal outputs vs problems

**Part 4: Choosing the Right Workflow (3 min)**

#### Section 8.4.1: Decision Framework (2 min)
- **Concepts**: Strategic selection, combination (3)
- **Content Elements** (VERIFIED):
  - Use MCP for: Single capability
  - Use Extensions for: Pre-configured bundle
  - Use IDE for: Code review in editor
  - Combine for: Large project with multiple needs

#### Section 8.4.2: Professional Setup Example (1 min)
- **Concepts**: Real-world integration (3)
- **Content Elements** (VERIFIED):
  - Example: Playwright MCP + Context7 + shell, tool filtering, IDE integration, extension bundle

#### Section 8.4.3: When to Ask AI (0-1 min)
- **Concepts**: Delegation (1)
- **Content Elements**: Describe needs; AI designs workflow

### Try With AI (Lesson 8) - 4 prompts

1. **Extension Development**: Create extension structure
2. **Security Configuration**: Evaluate and configure tool filtering
3. **IDE Integration Workflow**: Explain advantages
4. **Complete Workflow Design**: Design for project type

**Duration**: 15-20 minutes | **Scaffolding**: Medium-low (applying prior lessons)

---

## IV. Content Elements & Verified Intelligence Allocation

All examples sourced from `intelligence/chapter-6-gemini-cli-verified-docs.md`:

| Content | Lesson | Source | Verification |
|---------|--------|--------|--------------|
| Installation command | 2 | VERIFIED | npm install -g @google/gemini-cli |
| Free tier quotas | 1 | VERIFIED | 60 req/min, 1000 req/day |
| Context window | 1, 5 | VERIFIED | 1M tokens |
| Built-in tools | 3 | VERIFIED | File, Shell, Web, Search |
| Configuration hierarchy | 4 | VERIFIED | 7 levels + precedence |
| .env files | 4 | VERIFIED | ${VAR} syntax, security |
| Memory management | 5 | VERIFIED | /clear, /compress, /chat, GEMINI.md |
| Custom commands | 6 | VERIFIED | TOML structure, injection patterns |
| MCP CLI commands | 7 | VERIFIED | gemini mcp add/list/remove |
| OAuth | 7 | VERIFIED | /mcp auth |
| Extensions | 8 | VERIFIED | gemini extensions commands |
| Tool filtering | 8 | VERIFIED | includeTools, excludeTools |
| IDE integration | 8 | VERIFIED | /ide install, /ide enable |

**No Assumptions**: All technical claims reference verified intelligence; no placeholder examples

---

## V. "Try With AI" Prompt Design (Chapter 1 Format)

All "Try With AI" sections follow Chapter 1 clean format:

```markdown
### Prompt [N]: [Title]
<backticks>
[Specific, actionable prompt]
</backticks>

**Expected outcome**: [What student should receive from AI]
```

**Total**: 32 prompts across 8 lessons (4 per lesson)
**Balance**: Mix of understanding (Tier 2) and application (Tier 2/3)

---

## VI. Cognitive Load Validation Summary

All sections validated for ≤7 concept limit (Principle 12):

- **Lesson 1**: 5 concepts max ✅
- **Lesson 2**: 6 concepts max ✅
- **Lesson 3**: 7 concepts max ✅
- **Lesson 4**: 5 concepts max ✅
- **Lesson 5**: 5 concepts max ✅
- **Lesson 6**: 6 concepts max ✅
- **Lesson 7**: 6 concepts max ✅
- **Lesson 8**: 6 concepts max ✅

**Mitigation for Length** (137-150 min via 8 lessons):
- Clear subsection headings
- Each part is 3-6 minutes
- "Try With AI" prompts provide breaks
- Students can skip advanced topics on first read

---

## VII. Scaffolding Strategy

Progressive complexity across 8 lessons:

| Lesson | Tier | Practice |
|--------|------|----------|
| 1 | 1 | Reflection |
| 2 | 1 | Hands-on installation |
| 3 | 2 | Spec-style tool usage |
| 4 | 2 | Configuration writing |
| 5 | 2 | Memory/context management |
| 6 | 2-3 | Custom command creation |
| 7 | 2-3 | Workflow design |
| 8 | 3 | Application & synthesis |

---

## VIII. Integration Points

### Cross-Chapter References
- **Chapter 1**: Gemini CLI as tool enabling revolution
- **Chapter 4**: Use Gemini CLI for first AI project
- **Chapter 5**: Gemini CLI in tool landscape
- **Chapter 7**: MCP integration first exposure

### Prerequisite Knowledge
- Basic terminal usage (Chapter 3)
- Context window concept (explained here, reviewed from Chapter 5)
- Authentication flows (explained here)

---

## IX. Validation Strategy & Acceptance Criteria

**Post-Implementation Validation**:

**Technical Accuracy**:
- [ ] All CLI commands verified (mcp, extensions, ide, chat, memory, custom)
- [ ] All "Try With AI" prompts achievable
- [ ] No hallucinations (verified facts only)

**Constitutional Alignment**:
- [ ] Graduated teaching (Tier 1/2/3 mapped)
- [ ] Cognitive load limits (< 7 concepts/section)
- [ ] "Try With AI" format matches Chapter 1
- [ ] Principle 7 (technical accuracy) satisfied

**Pedagogical Quality**:
- [ ] Reading level: Grade 7-8
- [ ] Concept progression logical
- [ ] Prompts motivating and specific
- [ ] Durations realistic (137-150 min total)

**Sandbox Testing**:
- [ ] Fresh install successful
- [ ] All CLI commands work as documented
- [ ] OAuth flow works
- [ ] Custom commands functional
- [ ] IDE integration works in VS Code
- [ ] Cross-platform (Windows, macOS, Linux)

---

## X. Success Criteria (Definition of Done)

**All MUST-HAVE Items**:
- [ ] All 8 lessons written
- [ ] All examples from verified cache
- [ ] All "Try With AI" prompts (32 total) in Chapter 1 format
- [ ] Cognitive load ≤7 concepts/section
- [ ] Evals 1-8 achievable
- [ ] Constitutional principles satisfied

**All 8 Lessons Integrated**:
- [ ] Lesson 1: Tool positioning & ecosystem
- [ ] Lesson 2: Installation & authentication
- [ ] Lesson 3: Built-in tools
- [ ] Lesson 4: Configuration hierarchy & .env
- [ ] Lesson 5: Memory management & context
- [ ] Lesson 6: Custom slash commands
- [ ] Lesson 7: MCP servers & CLI commands
- [ ] Lesson 8: Extensions, security, IDE integration

**Quality Gates Passed**:
- [ ] Technical accuracy: 100% (verified)
- [ ] Constitutional alignment: PASS
- [ ] Sandbox testing: PASS
- [ ] Reading level: Grade 7-8
- [ ] Duration: 137-150 minutes

---

## XI. Timeline & Effort Estimate

| Phase | Task | Duration |
|-------|------|----------|
| Plan | This document | 2 hours |
| Implement | 8 lessons + Try With AI | 8-10 hours |
| Verification | Cross-check cache | 1.5 hours |
| Technical Review | Constitutional alignment | 1.5 hours |
| Sandbox Testing | Install CLI, run commands | 2 hours |
| Validation Report | Final quality gate | 1 hour |
| Finalization | Index, PHR | 0.5 hours |
| **TOTAL** | | **16.5-18 hours** |

---

## XII. Risk Mitigation

### Risk 1: Feature Creep
**Mitigation**: Stick to 8-lesson spec; defer advanced topics

### Risk 2: CLI Changes
**Mitigation**: Use verified cache (Jan 14, 2025); note update trigger

### Risk 3: Cross-Platform Issues
**Mitigation**: Verify on Windows/macOS/Linux; document variations

### Risk 4: Lessons Too Dense
**Mitigation**: Clear headings, "Try With AI" breaks, students can skip advanced topics

### Risk 5: OAuth Testing
**Mitigation**: Document conceptually; note "optional for beginners"

### Risk 6: TOML Syntax Errors
**Mitigation**: Error literacy section; encourage "ask AI if fails"

---

## XIII. Next Steps

**Upon Plan Approval**:

1. **Implementation** → Invoke `content-implementer` subagent
   - Write all 8 lessons
   - Apply skills contextually
   - Use verified cache exclusively
   - Follow Chapter 1 "Try With AI" format (32 prompts)

2. **Validation** → Invoke reviewers
   - `validation-auditor`: Constitutional alignment
   - `factual-verifier`: Quality gates
   - Sandbox testing: Commands verified

3. **Finalization** → Human review
   - Final polish
   - Update chapter index
   - Create PHR

---

**Plan Status**: ✅ READY FOR IMPLEMENTATION

**Key Decisions Locked In**:
1. 8-lesson structure (Foundation 1-3, Configuration 4-5, Extension 6-8)
2. Tier 1/2/3 AI usage embedded
3. All examples from verified intelligence (75 verified claims)
4. Cognitive load <7 concepts/section
5. 137-150 minutes total duration
6. Chapter 1 "Try With AI" format for 32 prompts
7. All Medium Tutorial + Context7 features integrated
