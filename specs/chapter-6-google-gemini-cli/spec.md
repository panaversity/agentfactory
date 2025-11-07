# Feature Specification: Comprehensive Gemini CLI Chapter Enhancement

**Feature Branch**: `feature/chapter-6-google-gemini-cli`
**Created**: 2025-11-07
**Status**: Draft
**Input**: User description: "Now we are not satisfied with current gemini cli coding agent chapter @book-source/docs/02-AI-Tool-Landscape/06-gemini-cli-installation-and-basics/ it is too thin and we are missing all the core commands like extensions etc. that make it fuel it to north start business developer"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Master Core Gemini CLI Commands and Workflow (Priority: P1)

A north star business developer needs to use Gemini CLI effectively for daily development workflows including file operations, web research, shell integration, and context management. They must understand all essential slash commands, @ syntax for file references, ! syntax for shell commands, and keyboard shortcuts that make Gemini CLI a productivity powerhouse.

**Why this priority**: Core command mastery is the foundation—without understanding basic commands, keyboard shortcuts, and syntax patterns, users cannot progress to advanced features. This is the MVP that makes Gemini CLI immediately useful for business development tasks.

**Independent Test**: Can be fully tested by completing 5-10 realistic business development tasks using only built-in commands (no extensions), such as analyzing competitor code, researching market documentation, executing git workflows, and managing project files. Success means completing tasks 3-5x faster than manual methods.

**Acceptance Scenarios**:

1. **Given** a business developer working on a competitor analysis, **When** they use `@./competitor-repo/` to reference an entire directory and ask "What architectural patterns does this codebase use?", **Then** Gemini CLI analyzes all files and provides a comprehensive architectural overview with file references
2. **Given** a developer needs to fetch and analyze pricing information from 3 competitor websites, **When** they use web_fetch tool with proper prompts, **Then** Gemini CLI retrieves content from all 3 sites and creates a comparison table
3. **Given** a developer working with a large codebase, **When** they use `/compress` to reduce context, **Then** token usage decreases by 60-80% while retaining key information for continued work
4. **Given** a developer needs to execute a series of git commands, **When** they enter `!` to toggle shell mode, **Then** they can execute multiple shell commands interactively with AI guidance
5. **Given** a developer makes a mistake during file editing, **When** they use `/restore` to list checkpoints and restore a previous state, **Then** files return to pre-edit state successfully
6. **Given** a developer working across multiple projects, **When** they use `/chat save project-name` to save conversation state, **Then** they can `/chat resume project-name` days later and continue where they left off
7. **Given** a developer wants to understand their token usage, **When** they use `/stats`, **Then** they see current token count, context window utilization, and compression status
8. **Given** a developer needs to copy AI-generated code, **When** they use `Ctrl+C` or `/copy`, **Then** the last response is copied to clipboard for pasting into their editor

---

### User Story 2 - Leverage Configuration System for Productivity (Priority: P2)

A business developer needs to customize Gemini CLI for their workflow through settings.json configuration, including model selection, tool restrictions, theme customization, vim mode, context management, and auto-accept policies that balance safety with speed.

**Why this priority**: Configuration transforms Gemini CLI from a generic tool into a personalized development assistant. This builds on P1 core commands by enabling workflow optimization and team-specific customization.

**Independent Test**: Can be fully tested by creating project-specific and user-specific configurations that demonstrably improve productivity, such as restricting dangerous shell commands, enabling auto-accept for safe operations, configuring custom themes, and setting context discovery limits.

**Acceptance Scenarios**:

1. **Given** a developer working on a sensitive codebase, **When** they configure `"excludeTools": ["run_shell_command(rm)", "run_shell_command(sudo)"]` in settings.json, **Then** Gemini CLI refuses to execute dangerous delete or sudo commands
2. **Given** a developer tired of approving safe git operations, **When** they configure `"tools.allowed": ["run_shell_command(git status)", "run_shell_command(git diff)"]`, **Then** these commands execute without confirmation prompts
3. **Given** a vim user, **When** they enable `"vimMode": true` in settings.json, **Then** Gemini CLI supports vim keybindings for editing prompts
4. **Given** a developer working with large codebases, **When** they configure `"context.discoveryMaxDirs": 500` and `"context.includeDirectories": ["../shared-libs"]`, **Then** Gemini CLI searches more directories and includes related codebases in context
5. **Given** a developer wanting custom loading messages, **When** they configure `"customWittyPhrases": ["Analyzing your empire...", "Consulting the business oracle..."]`, **Then** these custom phrases appear during AI processing
6. **Given** a team with shared coding standards, **When** they create a project `.gemini/settings.json` with tool restrictions and model preferences, **Then** all team members inherit these settings automatically when working in the project directory
7. **Given** a developer using environment variables for API keys, **When** they configure `"GEMINI_API_KEY": "$MY_API_KEY"` in settings.json, **Then** the variable is interpolated correctly from their .env file

---

### User Story 3 - Master Context and Memory Management (Priority: P2)

A business developer needs to understand the three types of memory in Gemini CLI (ephemeral context, GEMINI.md hierarchical instructions, and save_memory long-term facts) and use them strategically to maintain project knowledge, coding standards, and business context across sessions.

**Why this priority**: Effective memory management is what separates casual Gemini CLI use from professional-grade workflows. This enables persistent intelligence about projects, teams, and business domains.

**Independent Test**: Can be fully tested by setting up hierarchical GEMINI.md files (global, project, subdirectory), using save_memory for critical facts, and demonstrating that Gemini CLI maintains business context across multiple sessions and projects.

**Acceptance Scenarios**:

1. **Given** a developer working on multiple client projects, **When** they create `~/.gemini/GEMINI.md` with global coding standards and client-specific `project-A/.gemini/GEMINI.md` with business rules, **Then** Gemini CLI applies global standards everywhere but adds client-specific context when in project-A
2. **Given** a developer who uses specific libraries and frameworks, **When** they tell Gemini "Remember that we always use TypeScript strict mode and prefer functional programming patterns" and Gemini uses save_memory, **Then** future coding sessions automatically apply these preferences without re-explanation
3. **Given** a developer working on a feature with complex business logic, **When** they create a subdirectory-specific `.gemini/GEMINI.md` explaining domain terminology and business rules, **Then** Gemini CLI uses this context only when working in that subdirectory
4. **Given** a developer unsure what context is loaded, **When** they use `/memory show`, **Then** they see the complete, hierarchically-combined GEMINI.md content currently active
5. **Given** a developer who updated their global GEMINI.md file, **When** they use `/memory reload`, **Then** Gemini CLI refreshes context without restarting the CLI
6. **Given** a developer who learns a critical API key location, **When** they tell Gemini "Remember that our API keys are stored in .env.local, never .env" and it uses save_memory, **Then** this fact persists across all future sessions and prevents security mistakes

---

### User Story 4 - Extend with MCP Servers for Business Intelligence (Priority: P3)

A business developer needs to connect Gemini CLI to external systems through MCP (Model Context Protocol) servers, enabling capabilities like active web browsing (Playwright), live documentation access (Context7), GitHub integration, database queries, and custom business APIs.

**Why this priority**: MCP servers unlock Gemini CLI's full potential by connecting AI to real-world systems. This builds on P1-P2 by adding external data sources and automation capabilities.

**Independent Test**: Can be fully tested by configuring 2-3 MCP servers (Playwright, Context7, GitHub) and executing realistic business workflows like multi-site competitor research, framework documentation lookup, and automated repository analysis.

**Acceptance Scenarios**:

1. **Given** a developer researching 5 competitors, **When** they configure Playwright MCP server and prompt "Browse these 5 competitor pricing pages and create a comparison table", **Then** Gemini CLI autonomously navigates all 5 sites, extracts pricing data, and produces a structured comparison
2. **Given** a developer needing current Stripe API documentation, **When** they configure Context7 MCP server and ask "What changed in the latest Stripe API version?", **Then** Gemini CLI queries up-to-date documentation and summarizes changes with migration guidance
3. **Given** a developer managing GitHub repositories, **When** they configure GitHub MCP server and ask "List all open pull requests with 'bug' label across our 3 main repos", **Then** Gemini CLI queries GitHub API and returns a unified list with PR titles, authors, and ages
4. **Given** a developer wanting to verify MCP server status, **When** they use `/mcp` command, **Then** they see a list of configured MCP servers, their connection status, and available tools
5. **Given** a developer who needs to disable a misbehaving MCP server, **When** they edit settings.json to remove the server configuration and run `/mcp refresh`, **Then** the server is disconnected and its tools are no longer available

---

### User Story 5 - Create and Use Custom Slash Commands (Priority: P3)

A business developer needs to create custom slash commands using TOML files to automate repetitive workflows, such as generating standardized documentation, running test suites, analyzing code quality, and executing multi-step business processes.

**Why this priority**: Custom commands transform ad-hoc prompts into reusable, shareable team workflows. This is advanced customization that delivers high ROI for teams with repeated workflows.

**Independent Test**: Can be fully tested by creating 3-5 custom commands (e.g., `/research`, `/document`, `/test-suite`) that automate multi-step workflows and demonstrably save 10-30 minutes per execution compared to manual prompting.

**Acceptance Scenarios**:

1. **Given** a developer who frequently researches competitors, **When** they create `~/.gemini/commands/research.toml` with a template for competitive analysis, **Then** they can invoke `/research CompanyName` and Gemini CLI executes a standardized research workflow
2. **Given** a team with documentation standards, **When** they create a project-scoped `.gemini/commands/document.toml` that generates README files with specific sections, **Then** any team member can run `/document` and get consistently formatted documentation
3. **Given** a developer wanting to namespace commands, **When** they create `commands/git/review.toml`, **Then** the command is accessible as `/git:review` to avoid naming conflicts
4. **Given** a developer creating a command with user arguments, **When** they use `{{args}}` placeholder in the TOML prompt field, **Then** user-provided arguments are interpolated into the prompt correctly
5. **Given** a developer who wants to execute shell commands within a custom command, **When** they use `!{git log --oneline -10}` syntax in the TOML prompt, **Then** shell output is captured and included in the AI's context

---

### User Story 6 - Install and Manage Extensions for Domain Workflows (Priority: P3)

A business developer needs to discover, evaluate, install, and manage Gemini CLI extensions that bundle MCP servers, custom commands, context files, and configuration templates for specific domains (e.g., web development, data analysis, business intelligence, security auditing).

**Why this priority**: Extensions provide curated, pre-configured capabilities that would take hours to set up manually. This is the highest level of Gemini CLI mastery but requires understanding P1-P3 foundations.

**Independent Test**: Can be fully tested by researching available extensions, evaluating their security and utility, installing 1-2 extensions, and demonstrating how they provide capabilities beyond manual MCP server configuration.

**Acceptance Scenarios**:

1. **Given** a developer seeking business intelligence capabilities, **When** they ask Gemini "What extensions exist for competitive research and market analysis?", **Then** Gemini searches for reputable extensions and explains what each provides
2. **Given** a developer who found a useful extension, **When** they run `gemini extensions install <github-url>` or equivalent command, **Then** the extension installs with all included MCP servers, commands, and context files
3. **Given** a developer evaluating extension security, **When** they ask Gemini "Explain what this extension does and what access it needs" with the extension URL, **Then** Gemini analyzes the extension manifest and explains capabilities, data access, and potential risks
4. **Given** a developer who installed an extension but wants to remove it, **When** they use `gemini extensions uninstall <extension-name>`, **Then** all extension components are removed and settings are cleaned up
5. **Given** a developer creating a custom extension for their team, **When** they create a directory with `gemini-extension.json` including MCP servers, custom commands, and context files, **Then** team members can install it with a single command and get the full workflow bundle

---

### Edge Cases

- What happens when a user tries to reference a file with `@` syntax that doesn't exist? (Should fail gracefully with clear error message)
- How does Gemini CLI handle conflicting settings between project `.gemini/settings.json` and user `~/.gemini/settings.json`? (Project settings override user settings per documented precedence)
- What happens if an MCP server crashes or becomes unresponsive during operation? (Should timeout gracefully and allow user to continue with other tools)
- How does save_memory handle duplicate facts or contradictory information? (Appends all facts; user must manage contradictions manually)
- What happens when context window exceeds 1M tokens even after compression? (Should fail with clear message explaining token limit and suggesting strategies like splitting work or reducing included files)
- How does Gemini CLI behave when `/restore` checkpoint doesn't exist? (Shows list of available checkpoints instead of erroring)
- What happens if a custom slash command TOML file has syntax errors? (Should fail at command invocation with specific syntax error location)
- How does hierarchical GEMINI.md loading work when files contain contradictory instructions? (Later files in hierarchy override earlier ones; subdirectory > project > global)

## Requirements *(mandatory)*

### Functional Requirements

**Core Commands & Syntax:**
- **FR-001**: Chapter MUST explain all essential slash commands: `/help`, `/clear`, `/tools`, `/mcp`, `/stats`, `/compress`, `/copy`, `/memory`, `/chat`, `/restore`, `/settings`, `/init`, `/theme`, `/ide`
- **FR-002**: Chapter MUST explain `@` syntax for file and directory references with examples showing single file (@file.js), directory recursion (@./src/), and media files (images, PDFs)
- **FR-003**: Chapter MUST explain `!` syntax for shell command execution with examples of single commands (!git status) and shell mode toggle (! alone)
- **FR-004**: Chapter MUST document keyboard shortcuts: `Ctrl+L` (clear), `Ctrl+V` (paste), `Ctrl+Y` (YOLO toggle), `Ctrl+X` (editor)
- **FR-005**: Chapter MUST explain command-line invocation modes: interactive (gemini), single prompt (gemini -p), piped input, model selection (-m), sandbox mode (--sandbox), YOLO mode (--yolo), checkpointing (--checkpointing)

**Configuration System:**
- **FR-006**: Chapter MUST explain the 7-level configuration hierarchy and precedence rules: defaults → system defaults → user settings → project settings → system overrides → environment variables → command-line arguments
- **FR-007**: Chapter MUST document key settings.json options: model configuration, context management, tool restrictions/permissions, MCP server configuration, UI customization, security settings
- **FR-008**: Chapter MUST explain environment variable interpolation in settings.json using `$VAR_NAME` and `${VAR_NAME}` syntax
- **FR-009**: Chapter MUST show how to create project-specific and user-specific configurations with realistic examples
- **FR-010**: Chapter MUST document settings file locations for all platforms (Windows, macOS, Linux)

**Context & Memory Management:**
- **FR-011**: Chapter MUST explain three types of memory: ephemeral context (session-only), GEMINI.md hierarchical instructions (persistent, project-specific), save_memory facts (persistent, global)
- **FR-012**: Chapter MUST document GEMINI.md hierarchical loading: global (~/.gemini/GEMINI.md) → project/ancestor → subdirectory
- **FR-013**: Chapter MUST explain `/memory` commands: `/memory show`, `/memory add`, `/memory reload`
- **FR-014**: Chapter MUST document save_memory tool usage and when to use it vs. GEMINI.md files
- **FR-015**: Chapter MUST explain how to use `@file.md` syntax within GEMINI.md to modularize instructions

**MCP Servers:**
- **FR-016**: Chapter MUST explain Model Context Protocol (MCP) purpose: connecting AI to external systems and data sources
- **FR-017**: Chapter MUST document how to configure MCP servers in settings.json with command, args, allowed tools, and excluded tools
- **FR-018**: Chapter MUST provide realistic examples of popular MCP servers: Playwright (web browsing), Context7 (live documentation), GitHub (repository operations)
- **FR-019**: Chapter MUST explain `/mcp` command for listing configured servers and their status
- **FR-020**: Chapter MUST document MCP server security considerations and when to trust external servers

**Custom Slash Commands:**
- **FR-021**: Chapter MUST explain TOML-based custom command creation with description and prompt fields
- **FR-022**: Chapter MUST document file naming conventions: test.toml → /test, git/commit.toml → /git:commit
- **FR-023**: Chapter MUST explain user-scoped (~/.gemini/commands/) vs. project-scoped (.gemini/commands/) command locations
- **FR-024**: Chapter MUST document {{args}} placeholder for user arguments in custom commands
- **FR-025**: Chapter MUST explain !{command} syntax for shell command execution within custom commands

**Extensions:**
- **FR-026**: Chapter MUST differentiate between MCP servers (individual tools) and extensions (bundled capabilities)
- **FR-027**: Chapter MUST explain extension structure: gemini-extension.json manifest, MCP servers, custom commands, context files
- **FR-028**: Chapter MUST document extension installation, uninstallation, and management commands
- **FR-029**: Chapter MUST provide security evaluation criteria for third-party extensions: creator reputation, data access scope, maintenance status
- **FR-030**: Chapter MUST explain when to use extensions vs. individual MCP servers

**Advanced Features:**
- **FR-031**: Chapter MUST explain checkpointing system for undoing file modifications with `/restore`
- **FR-032**: Chapter MUST document conversation branching with `/chat save` and `/chat resume`
- **FR-033**: Chapter MUST explain token management: `/stats` for usage, `/compress` for context reduction, compression threshold settings
- **FR-034**: Chapter MUST document IDE integration mode with `/ide install` and `/ide enable`
- **FR-035**: Chapter MUST explain sandboxing options: true/false, Docker, Podman, custom profiles

### Key Entities *(educational content)*

- **Command Types**: Slash commands (/), file references (@), shell commands (!), keyboard shortcuts (Ctrl+X)
- **Configuration Layers**: System defaults, user settings, project settings, environment variables, command-line arguments
- **Memory Types**: Ephemeral context, GEMINI.md hierarchical instructions, save_memory persistent facts
- **MCP Components**: MCP servers (external capabilities), MCP protocol (standardized connection), MCP tools (exposed functions)
- **Custom Commands**: TOML definition files, user-scoped commands, project-scoped commands, namespaced commands
- **Extensions**: Extension manifest (gemini-extension.json), bundled MCP servers, bundled commands, bundled context

## Success Criteria *(mandatory)*

### Measurable Outcomes

**Skill Acquisition:**
- **SC-001**: 90% of readers can list and explain the purpose of all 10+ essential slash commands from memory after reading the chapter
- **SC-002**: 85% of readers can correctly configure settings.json with 5+ key settings (model, tools, context, MCP servers) without referring to documentation
- **SC-003**: 80% of readers can create a hierarchical GEMINI.md setup (global + project + subdirectory) that demonstrably improves AI responses for their specific codebase
- **SC-004**: 75% of readers can install and configure at least 2 MCP servers (e.g., Playwright, Context7) and use them for realistic business workflows

**Productivity Improvement:**
- **SC-005**: Developers using advanced Gemini CLI features (MCP servers, custom commands, proper configuration) complete competitive research tasks 5-10x faster than using built-in tools only
- **SC-006**: Teams with shared GEMINI.md files and custom commands report 30-50% reduction in time spent explaining project-specific context and coding standards to AI
- **SC-007**: Developers using proper context management (compression, hierarchical GEMINI.md) maintain productive sessions 2-3x longer before hitting context limits

**Tool Mastery:**
- **SC-008**: 70% of readers can create at least 1 custom slash command that automates a repeated workflow and saves 10+ minutes per use
- **SC-009**: 80% of readers can correctly evaluate extension security (creator, data access, maintenance) before installation
- **SC-010**: 75% of readers can troubleshoot common Gemini CLI issues (MCP server failures, context overload, configuration conflicts) using `/stats`, `/mcp`, and `/memory show`

**Workflow Integration:**
- **SC-011**: 85% of readers can articulate when to use Gemini CLI vs. Claude Code based on context window needs, customization requirements, cost, and workflow preferences
- **SC-012**: North star business developers report Gemini CLI as their primary AI development tool for at least 3 use cases: codebase analysis, competitive research, documentation management, or workflow automation

**Content Quality:**
- **SC-013**: Chapter reading time is 60-90 minutes with all hands-on exercises completed
- **SC-014**: Chapter complexity tier matches Part 2 (Intermediate): 3-4 options per decision, 7 concepts per section, tradeoff discussions included
- **SC-015**: 90% of code examples are directly runnable without modification on all three platforms (Windows, macOS, Linux)

## Assumptions *(documented reasoning)*

### Technical Assumptions
- Readers have completed Chapter 5 (prior AI tool experience) and understand basic terminal navigation
- Readers have Node.js installed (prerequisite from earlier chapters)
- Gemini CLI version 0.4.0 or later is used (2025 stable release with extensions support)
- Readers have access to a personal Google account for free tier API access

### Pedagogical Assumptions
- Readers learn best by seeing concrete examples before abstract explanations (show then tell)
- Complex features are better introduced through realistic business scenarios than academic examples
- Security considerations should be introduced at point of use, not in a separate section
- Readers will reference this chapter as a guide, not memorize all commands

### Scope Assumptions
- Chapter focuses on features that differentiate Gemini CLI from basic AI chat (commands, configuration, MCP, extensions)
- Installation and basic authentication are covered lightly (assumed from earlier lesson content)
- Advanced topics like custom MCP server development are mentioned but not taught in depth
- Comparison to Claude Code is framed as "when to use each" not "which is better"

### Content Structure Assumptions
- Each major feature (commands, configuration, memory, MCP, custom commands, extensions) deserves dedicated lesson section
- Hands-on exercises follow each major concept introduction
- Real-world business scenarios motivate feature learning
- Troubleshooting guidance is integrated throughout, not isolated

## Out of Scope *(explicit exclusions)*

### Not Covered in This Chapter
- **Custom MCP Server Development**: Creating your own MCP servers from scratch (too advanced; requires protocol knowledge)
- **Gemini CLI Source Code Deep Dive**: Internal architecture and codebase exploration (out of scope for business developers)
- **Qwen Code Fork**: Alibaba's fork is mentioned for context but not taught in detail (regional alternative)
- **AI Model Comparison**: Detailed Gemini 2.5 Pro vs. Claude Sonnet 4.5 capabilities (covered in Part 1)
- **Production Deployment**: Running Gemini CLI in CI/CD, air-gapped environments, or enterprise settings (Part 10+ content)
- **Programmatic API Usage**: Using Gemini API directly without CLI (different use case)
- **GitHub Actions Deep Dive**: Gemini CLI GitHub Actions mentioned but full CI/CD workflows are Part 10+ content

### Deferred to Later Chapters
- **Advanced Prompt Engineering**: Chapter-specific focus is tool features; prompting mastery is Chapter 9
- **Multi-Agent Workflows**: Orchestrating multiple AI agents is advanced topic for Parts 10-13
- **Cost Optimization at Scale**: Free tier is sufficient for learning; optimization is professional topic (Parts 10-13)

### Intentional Limitations
- Chapter does NOT teach every possible MCP server (focuses on 3-5 most valuable for business developers)
- Chapter does NOT provide exhaustive settings.json reference (focuses on most impactful settings)
- Chapter does NOT compare all AI coding tools (focuses on Gemini CLI vs. Claude Code decision framework)
- Chapter does NOT teach general terminal/command-line skills (prerequisite knowledge)

## Dependencies *(what must exist first)*

### Prerequisites from Earlier Chapters
- Chapter 1-3: General AI development concepts, tool landscape understanding
- Chapter 4: Claude Code familiarity (for comparison framework)
- Chapter 5: Prior AI coding assistant experience (understands prompting basics)
- Terminal/command-line comfort (navigation, running commands, environment variables)
- Node.js and npm installed (from earlier development environment setup)

### External Dependencies
- Google account (free tier access to Gemini API)
- Internet connection (API calls, MCP server package installation)
- Text editor (for editing settings.json and GEMINI.md files)
- Git (for examples using version control workflows)

### Content Dependencies
- Official Gemini CLI documentation (referenced for exhaustive settings)
- MCP server registry or discovery mechanism (for finding available servers)
- Extension discovery method (GitHub, official gallery, community resources)

## Risks & Challenges *(what could go wrong)*

### Pedagogical Risks
- **Risk**: Chapter becomes a reference manual instead of teaching journey (too encyclopedic)
  - **Mitigation**: Focus on 20% of features that deliver 80% of value; provide links to full documentation for completeness

- **Risk**: Readers feel overwhelmed by configuration options and advanced features
  - **Mitigation**: Clear progression from basic commands → configuration → memory → MCP → extensions; each builds on previous

- **Risk**: Examples fail on specific platforms (Windows, macOS, Linux differences)
  - **Mitigation**: Test all examples on all three platforms; provide platform-specific notes where needed

### Technical Risks
- **Risk**: Gemini CLI rapid development breaks examples (new versions, deprecated features)
  - **Mitigation**: Pin examples to specific version (0.4.0); provide version checking guidance; update chapter quarterly

- **Risk**: Free tier quotas change (Google reduces 1000 requests/day limit)
  - **Mitigation**: Teach quota monitoring with `/stats`; explain paid tier upgrade path if quotas are hit

- **Risk**: MCP servers become unavailable or unmaintained (Playwright, Context7, GitHub)
  - **Mitigation**: Choose servers with strong backing (official or widely adopted); teach readers how to evaluate server health

### Learning Risks
- **Risk**: Readers skip foundational commands and jump to extensions (premature complexity)
  - **Mitigation**: Explicit learning path guidance; prerequisites clearly stated; exercises build progressively

- **Risk**: Readers install malicious extensions without security evaluation
  - **Mitigation**: Security considerations taught at point of extension introduction; clear evaluation checklist provided

- **Risk**: Readers compare Gemini CLI to Claude Code and dismiss one or the other entirely
  - **Mitigation**: Frame as complementary tools with different strengths; decision framework based on use case, not absolute "better"

## Notes *(additional context)*

### Design Philosophy for This Chapter
This chapter transforms Gemini CLI from "another AI tool" into a customizable, extensible platform that adapts to business developer workflows. The goal is not command memorization but understanding the architecture: built-in → configured → extended → automated.

**Progression Logic:**
1. **Commands** (immediate value): Readers get productive quickly with slash commands, @ syntax, ! syntax
2. **Configuration** (personalization): Readers customize for their workflow and security needs
3. **Memory** (intelligence): Readers teach Gemini CLI their project and domain context
4. **MCP** (capability): Readers connect to external systems for data and automation
5. **Custom Commands** (automation): Readers codify repeated workflows
6. **Extensions** (ecosystem): Readers leverage community solutions for advanced needs

### Content Tone & Voice
- **Empowering, not prescriptive**: "Here's how to customize" not "You must configure this way"
- **Scenario-driven**: Real business problems motivate feature introduction
- **Security-conscious**: Trust but verify; explain risks clearly
- **Tool-agnostic mindset**: Right tool for right job; no Gemini CLI evangelism

### Key Differentiators to Emphasize
What makes Gemini CLI special for north star business developers:
1. **Open source = inspectable, modifiable, learnable** (not black box)
2. **Free tier = experimentation without cost anxiety** (1000 requests/day goes far)
3. **1M token context = entire small-medium projects in one session** (rare capability)
4. **MCP protocol = connect to any system** (not limited to vendor integrations)
5. **Extensibility = build vertical AI for your domain** (competitive advantage)

### Success Indicators for This Spec
This spec succeeds if:
- Chapter covers all essential Gemini CLI capabilities (commands, config, memory, MCP, custom commands, extensions)
- Content is actionable (readers can implement what they read)
- Complexity tier matches Part 2 (intermediate developers, not beginners or experts)
- Examples are realistic business scenarios, not toy demos
- Chapter positions Gemini CLI as complementary to Claude Code, not competitive

### Next Steps After Spec Approval
1. **Planning** (`/sp.plan`): Break chapter into 5-6 lessons with clear learning objectives
2. **Tasks** (`/sp.tasks`): Create task checklist with acceptance criteria for each lesson
3. **Implementation** (lesson-writer agent): Write lessons following approved plan
4. **Validation** (technical-reviewer): Ensure accuracy, runnability, pedagogical quality
5. **Integration**: Merge into main book structure and cross-reference with related chapters
