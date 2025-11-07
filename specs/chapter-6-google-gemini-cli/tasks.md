# Implementation Tasks: Comprehensive Gemini CLI Chapter Enhancement

**Feature**: Chapter 6 - Google Gemini CLI
**Branch**: `feature/chapter-6-google-gemini-cli`
**Created**: 2025-11-07
**Specification**: [spec.md](./spec.md)
**Plan**: [plan.md](./plan.md)

---

## Executive Summary

### Strategy Decision: Update vs. Create

After careful analysis of existing lessons, we will **UPDATE all 5 existing lessons** rather than create new ones. This approach:
- Preserves existing quality content (why it matters, installation basics, tool overview)
- Extends each lesson with missing core functionality
- Maintains lesson flow and numbering
- Focuses enhancement effort where gaps exist

**No new lessons will be created.** All 35 functional requirements will be addressed through strategic updates to the 5 existing files.

### Lesson-by-Lesson Strategy

| Lesson File | Current Lines | Strategy | Additions | Final Est. Lines |
|-------------|---------------|----------|-----------|------------------|
| `01-why-gemini-cli-matters.md` | ~190 | **Minor Update** | Decision framework section | ~220 (+30) |
| `02-installation-authentication-first-steps.md` | ~140 | **Major Expansion** | Commands, syntax, shortcuts reference | ~650 (+510) |
| `03-built-in-tools-deep-dive.md` | ~340 | **Major Expansion** | Configuration system, settings hierarchy | ~750 (+410) |
| `04-context-window-and-tool-comparison.md` | ~310 | **Complete Redesign** | Transform to memory management lesson | ~650 (+340) |
| `05-mcp-and-extensions-ecosystem.md` | ~330 | **Major Expansion** | MCP config, custom commands, extensions | ~850 (+520) |
| `README.md` | ~78 | **Complete Rewrite** | New learning objectives, time estimates | ~150 (+72) |

**Total Content**: ~1,210 lines (current) → ~3,270 lines (final) = **+2,060 lines of new content**

---

## Task Breakdown by Lesson

### LESSON 1: Why Gemini CLI Matters (01-why-gemini-cli-matters.md)

**Status**: ✅ **Minor Updates Only**
**Current State**: Excellent foundational content (190 lines)
**Current Sections** (KEEP ALL):
- Three game-changing differences (open source, free tier, MCP)
- When Claude Code is better vs. when Gemini CLI is better
- Comparison table (license, pricing, context, model, interface, tools, etc.)
- Open source ecosystem effect (Qwen Code fork example)

**What to REMOVE**: Nothing (all content is valuable and on-topic)

**What to ADD**: 1 new section (~30 lines)

---

#### Task 1.1: Add Decision Framework Section (MUST) ⭐

**Priority**: P2
**Effort**: 2 hours
**FRs Covered**: FR-001 (command understanding foundation)

**Acceptance Criteria**:
- [ ] Add new section: "## Making the Choice: A Decision Framework"
- [ ] Include 4-question decision flowchart:
  1. "Do I need to customize AI behavior extensively?" → Yes = Gemini CLI
  2. "Am I working with large codebases (10K+ lines)?" → Yes = Gemini CLI
  3. "Do I need zero-cost experimentation?" → Yes = Gemini CLI (free tier)
  4. "Do I prefer web-based tools?" → Yes = Claude Code
- [ ] Add practical example: "If analyzing entire competitor codebase → Gemini CLI. If exploratory conversation about architecture → Claude Code"
- [ ] Cross-reference with later lessons: "You'll learn customization in Lesson 3, memory in Lesson 4"
- [ ] Content matches intermediate tier (3-4 options, tradeoff discussions)

**Success Criteria**: 85% of readers can articulate when to choose each tool (SC-011 from spec)

---

### LESSON 2: Installation, Authentication & Core Commands (02-installation-authentication-first-steps.md)

**Status**: 🔥 **Major Expansion Required**
**Current State**: Good installation/auth content (~140 lines), but missing ALL core commands
**Current Sections** (KEEP):
- Prerequisites (Node.js, npm, terminal access)
- Installation with verification
- Authentication flow (theme selection, Google login)
- First launch experience

**What to REMOVE**:
- "Understanding the Gemini CLI Interface" section (if it's generic/redundant - to be confirmed during implementation)

**What to ADD**: 6 major new sections (~510 lines total)

---

#### Task 2.1: Add Essential Slash Commands Reference (MUST) ⭐⭐⭐

**Priority**: P1
**Effort**: 8 hours
**FRs Covered**: FR-001 (all essential slash commands)

**Acceptance Criteria**:
- [ ] Add new section: "## Essential Slash Commands: Your Command Toolkit"
- [ ] Document 14 commands with syntax, purpose, and business use case:
  - `/help` - Show available commands
  - `/clear` - Clear screen and context
  - `/tools` - List available tools
  - `/mcp` - List MCP servers and status
  - `/stats` - Show token usage and context
  - `/compress` - Reduce context size intelligently
  - `/copy` - Copy last response to clipboard
  - `/memory show` - View combined GEMINI.md context
  - `/memory add` - Add fact to memory
  - `/memory reload` - Refresh context files
  - `/chat save <tag>` - Save conversation state
  - `/chat resume <tag>` - Resume saved conversation
  - `/restore` - List/restore file checkpoints
  - `/settings` - Edit settings.json
  - `/init` - Generate starter GEMINI.md
  - `/theme` - Change CLI theme
  - `/ide install` - Install IDE integration
  - `/ide enable` - Enable IDE mode
- [ ] Each command includes:
  - One-line purpose
  - Syntax example
  - Business scenario ("Use `/compress` when working with large codebases to reduce token usage")
  - Expected output description
- [ ] Add "Try It Now" subsection with 3 commands to test immediately
- [ ] Cross-reference: "Memory commands explained in detail in Lesson 4"

**Success Criteria**: 90% of readers can list 10+ commands from memory (SC-001 from spec)

---

#### Task 2.2: Add @ Syntax for File References (MUST) ⭐⭐

**Priority**: P1
**Effort**: 4 hours
**FRs Covered**: FR-002 (@ syntax for files and directories)

**Acceptance Criteria**:
- [ ] Add new section: "## @ Syntax: Giving Your AI Context About Files"
- [ ] Explain concept: "Use @ to reference files and directories in your prompts"
- [ ] Document 4 usage patterns:
  - Single file: `@./src/app.js` - "Analyze this file"
  - Multiple files: `@./src/app.js @./src/utils.js` - "Compare these files"
  - Directory (recursive): `@./src/` - "Analyze all files in this directory"
  - Media files: `@./diagram.png` - "Explain this diagram"
- [ ] Supported file types list: .js, .ts, .py, .md, .json, .csv, .pdf, .png, .jpg, .mp4, .mp3
- [ ] Business example: "Competitor analysis: `@./competitor-repo/` What patterns do they use?"
- [ ] Add warning box: "Large directories may exceed context window. Use specific subdirectories when possible."
- [ ] Include 2 hands-on exercises:
  1. Create a test file, reference it with @, ask for summary
  2. Reference entire project directory, ask for architecture overview

**Success Criteria**: 80% can correctly use @ syntax for files and directories (measured via exercises)

---

#### Task 2.3: Add ! Syntax for Shell Commands (MUST) ⭐⭐

**Priority**: P1
**Effort**: 4 hours
**FRs Covered**: FR-003 (! syntax for shell execution)

**Acceptance Criteria**:
- [ ] Add new section: "## ! Syntax: Executing Shell Commands with AI Guidance"
- [ ] Explain two modes:
  - Single command: `!git status` - Execute one command
  - Shell mode toggle: Type `!` alone to enter/exit interactive shell mode
- [ ] Document safety features: Tool confirmation prompts, settings.json restrictions
- [ ] Business examples:
  - `!git log --oneline -10` - "Show recent commits"
  - Enter `!` mode → `git status` → `git diff` → `exit` - Interactive git workflow
  - `!npm run test` - "Run test suite"
- [ ] Add security callout: "Configure `excludeTools` in settings.json to block dangerous commands (covered in Lesson 3)"
- [ ] Cross-reference: "Shell mode automation with custom commands in Lesson 5"
- [ ] Include 2 hands-on exercises:
  1. Use `!` to check git status
  2. Enter shell mode, run 3 git commands interactively

**Success Criteria**: 75% can execute shell commands safely with ! syntax (measured via exercises)

---

#### Task 2.4: Add Keyboard Shortcuts Reference (MUST) ⭐

**Priority**: P1
**Effort**: 2 hours
**FRs Covered**: FR-004 (keyboard shortcuts)

**Acceptance Criteria**:
- [ ] Add new section: "## Keyboard Shortcuts: Speed Up Your Workflow"
- [ ] Document 4 essential shortcuts in table format:
  | Shortcut | Action | Use Case |
  |----------|--------|----------|
  | `Ctrl+L` | Clear screen | Clean workspace between tasks |
  | `Ctrl+V` | Paste text/image | Paste code or screenshot for analysis |
  | `Ctrl+Y` | Toggle YOLO mode | Auto-approve safe operations |
  | `Ctrl+X` | Open prompt in editor | Write longer, multi-line prompts |
- [ ] Explain YOLO mode: "Auto-approves tool execution (use carefully)"
- [ ] Business scenario: "When reviewing 10 files in sequence, use Ctrl+L between each to clear context"
- [ ] Add practice callout: "Try Ctrl+L now to clear your screen"

**Success Criteria**: 80% remember and use at least 2 keyboard shortcuts in daily workflow

---

#### Task 2.5: Add Command-Line Invocation Modes (MUST) ⭐

**Priority**: P1
**Effort**: 3 hours
**FRs Covered**: FR-005 (invocation modes)

**Acceptance Criteria**:
- [ ] Add new section: "## Command-Line Invocation: Different Ways to Start Gemini CLI"
- [ ] Document 6 invocation modes with syntax and use case:
  1. **Interactive REPL**: `gemini` - Standard conversational mode
  2. **Single prompt**: `gemini -p "your prompt"` - One-off questions (returns immediately)
  3. **Piped input**: `echo "analyze this" | gemini` - Scripting integration
  4. **Model selection**: `gemini -m gemini-2.0-flash-exp -p "prompt"` - Use specific model
  5. **Sandbox mode**: `gemini --sandbox -p "prompt"` - Isolated execution
  6. **YOLO mode**: `gemini --yolo -p "prompt"` - Auto-approve all operations
  7. **Checkpointing**: `gemini --checkpointing` - Enable file undo snapshots
- [ ] Business examples:
  - Single prompt: `gemini -p "What changed in latest Stripe API?"` - Quick research
  - YOLO mode: `gemini --yolo -p "Run tests and fix any failures"` - Automated workflows
- [ ] Security warning for --yolo mode: "Only use in trusted environments"
- [ ] Cross-reference: "Sandbox and checkpointing covered in Lesson 3"

**Success Criteria**: 70% can invoke Gemini CLI in at least 3 different modes

---

#### Task 2.6: Add Memory Introduction Teaser (MUST) ⭐

**Priority**: P1
**Effort**: 2 hours
**FRs Covered**: FR-011 (introduce memory concept early)

**Acceptance Criteria**:
- [ ] Add new section: "## Your First Memory: Teaching Gemini About Your Project"
- [ ] Introduce save_memory tool concept: "Tell Gemini facts it should remember across sessions"
- [ ] Simple example:
  - User: "Remember that I always use TypeScript strict mode"
  - Gemini uses save_memory internally
  - Verification: Next session, Gemini auto-applies TypeScript strict mode
- [ ] Explain persistence: "Saved memories persist forever until you delete them"
- [ ] Tease GEMINI.md: "There's also a more powerful memory system with GEMINI.md files (Lesson 4)"
- [ ] Business value: "Teach Gemini your coding standards, API key locations, project structure once—it remembers forever"
- [ ] Add 1 hands-on exercise: Save a fact about your project, restart Gemini, verify it remembers

**Success Criteria**: 80% understand save_memory concept and use it to store at least 1 project fact

---

#### Task 2.7: Update "Try With AI" Section (MUST) ⭐

**Priority**: P2
**Effort**: 2 hours
**FRs Covered**: Multiple (hands-on application)

**Acceptance Criteria**:
- [ ] Replace existing "Try With AI" section with 4 new prompts:
  1. **Command Exploration**: "List all available slash commands and explain when I'd use /compress vs /stats"
  2. **File Reference Practice**: "I have a file at ./data/sales.csv. Show me how to reference it with @ syntax and ask you to analyze top 3 products"
  3. **Shell Integration**: "Guide me through using ! syntax to check my git status, see uncommitted changes, and understand what files are modified"
  4. **Memory Setup**: "Help me save 3 important facts about my project using save_memory: preferred language, coding style, and project structure"
- [ ] Each prompt includes:
  - Clear objective
  - Expected outcome description
  - Link to relevant section in lesson
- [ ] Add success indicator: "You're ready for Lesson 3 when you can use @, !, and /commands confidently"

**Success Criteria**: 75% complete all 4 exercises successfully

---

**Lesson 2 Summary**:
- **Content Added**: ~510 lines across 6 major sections
- **FRs Covered**: FR-001, FR-002, FR-003, FR-004, FR-005, FR-011
- **Hands-On Exercises**: 8 total (2 per major concept)
- **Estimated Reading Time**: 40 minutes (up from 15 minutes)

---

### LESSON 3: Built-In Tools & Configuration (03-built-in-tools-deep-dive.md)

**Status**: 🔥 **Major Expansion Required**
**Current State**: Good built-in tools overview (~340 lines), missing configuration system entirely
**Current Sections** (KEEP):
- Tool 1: File operations (CSV, JSON, PDF analysis)
- Tool 2: Web fetching (live website content)
- Tool 3: Google Search grounding (current information with citations)
- Tool 4: Shell integration (command execution)
- Combining tools for complex workflows

**What to REMOVE**: Nothing (all tools content is valuable)

**What to ADD**: 4 major new sections (~410 lines total)

---

#### Task 3.1: Add Configuration System Overview (MUST) ⭐⭐⭐

**Priority**: P2
**Effort**: 6 hours
**FRs Covered**: FR-006 (7-level hierarchy), FR-010 (file locations)

**Acceptance Criteria**:
- [ ] Add new section: "## Customizing Gemini CLI: The Configuration System"
- [ ] Explain configuration philosophy: "Transform generic AI into personalized development assistant"
- [ ] Document 7-level hierarchy with precedence (lowest → highest):
  1. Default values (built-in)
  2. System defaults file (`/etc/gemini-cli/system-defaults.json`)
  3. User settings (`~/.gemini/settings.json`) ⭐ Most common
  4. Project settings (`.gemini/settings.json`) ⭐ Team-shared
  5. System overrides (`/etc/gemini-cli/settings.json`)
  6. Environment variables (`.env` files)
  7. Command-line arguments (highest priority)
- [ ] Create visual diagram (ASCII art or description) showing hierarchy
- [ ] File locations table for all platforms (Windows, macOS, Linux)
- [ ] Explain precedence: "Project settings override user settings. Command-line flags override everything."
- [ ] Business scenario: "Team shares project `.gemini/settings.json` with tool restrictions. Your personal `~/.gemini/settings.json` has vim mode enabled. Both apply."
- [ ] Add callout: "99% of users only need user and project settings files"

**Success Criteria**: 85% can configure settings.json with 5+ settings (SC-002 from spec)

---

#### Task 3.2: Add settings.json Deep Dive (MUST) ⭐⭐⭐

**Priority**: P2
**Effort**: 8 hours
**FRs Covered**: FR-007 (key settings options), FR-008 (env variable interpolation)

**Acceptance Criteria**:
- [ ] Add new section: "## Key Settings You Should Know"
- [ ] Organize by category with examples:

**Model Configuration**:
```json
{
  "model": {
    "name": "gemini-2.5-pro",
    "maxSessionTurns": 50,
    "compressionThreshold": 0.8
  }
}
```
- Explain: maxSessionTurns limits history, compressionThreshold auto-compresses at 80% capacity

**Tool Restrictions** (Security):
```json
{
  "tools": {
    "exclude": ["run_shell_command(rm)", "run_shell_command(sudo)"],
    "allowed": ["run_shell_command(git*)"]
  }
}
```
- Explain: Block dangerous operations, auto-approve safe git commands

**Context Management**:
```json
{
  "context": {
    "fileName": "GEMINI.md",
    "discoveryMaxDirs": 500,
    "includeDirectories": ["../shared-libs", "../docs"]
  }
}
```
- Explain: GEMINI.md for project context (Lesson 4), include related directories

**UI Customization**:
```json
{
  "theme": "Cyberpunk",
  "vimMode": true,
  "customWittyPhrases": ["Analyzing your empire..."]
}
```
- Explain: Personalize appearance and editing experience

**Environment Variable Interpolation**:
```json
{
  "GEMINI_API_KEY": "$MY_API_KEY",
  "paths": {
    "data": "${HOME}/project-data"
  }
}
```
- Explain: Use `$VAR` or `${VAR}` to reference environment variables
- Add security note: "Never hardcode API keys. Use env vars."

- [ ] Add 3 complete example configurations:
  1. **Beginner**: Minimal settings (theme, model)
  2. **Business Developer**: Tool restrictions, context dirs, auto-accept safe operations
  3. **Team Project**: Shared standards, restricted tools, custom commands path
- [ ] Cross-reference: "Full settings schema at github.com/google-gemini/gemini-cli/schemas"

**Success Criteria**: 80% can create custom settings.json for their workflow

---

#### Task 3.3: Add Tool Restrictions and Security (MUST) ⭐⭐

**Priority**: P2
**Effort**: 5 hours
**FRs Covered**: FR-007 (tool restrictions)

**Acceptance Criteria**:
- [ ] Add new section: "## Tool Safety: Restricting Dangerous Operations"
- [ ] Explain trust levels:
  - **Safe tools**: File reading, web search (no system modification)
  - **Moderate tools**: File writing (can modify your files)
  - **Dangerous tools**: Shell commands (can delete, modify system)
- [ ] Document restriction methods:
  - `excludeTools`: Block specific tools entirely
  - `allowed`: Auto-approve specific tools (skip confirmation)
  - Tool-specific restrictions: `run_shell_command(rm)` blocks only `rm`
- [ ] Security scenarios with configs:
  1. **Block all file deletion**:
     ```json
     {"tools": {"exclude": ["run_shell_command(rm)"]}}
     ```
  2. **Auto-approve git commands**:
     ```json
     {"tools": {"allowed": ["run_shell_command(git*)"]}}
     ```
  3. **Read-only mode** (block all writes):
     ```json
     {"tools": {"exclude": ["write_file", "run_shell_command"]}}
     ```
- [ ] Business example: "When onboarding new team member, start with read-only config until they understand the codebase"
- [ ] Add hands-on exercise: Create settings.json that blocks `rm` and `sudo`, auto-approves `git status`

**Success Criteria**: 90% can configure tool restrictions for their security needs

---

#### Task 3.4: Add Token Efficiency & Compression (MUST) ⭐

**Priority**: P2
**Effort**: 4 hours
**FRs Covered**: FR-033 (token management, /stats, /compress)

**Acceptance Criteria**:
- [ ] Add new section: "## Managing Your 1M Token Budget Efficiently"
- [ ] Explain token usage: "1M tokens ≈ 750K words ≈ 100K lines of code"
- [ ] Introduce `/stats` command:
  - Shows current token count
  - Shows context window utilization percentage
  - Shows compression status
- [ ] Introduce `/compress` command:
  - Intelligently summarizes context while preserving key information
  - Typically reduces tokens by 60-80%
  - Use when approaching context limit
- [ ] Configuration for auto-compression:
  ```json
  {
    "model": {
      "compressionThreshold": 0.8
    }
  }
  ```
  - Auto-compress when reaching 80% capacity
- [ ] Business workflow example:
  1. Start analysis of large codebase
  2. Use `/stats` periodically to check usage
  3. When at 70% capacity, run `/compress`
  4. Continue work with reduced context
- [ ] Add hands-on exercise: Reference large directory, use `/stats`, run `/compress`, check `/stats` again

**Success Criteria**: 75% can monitor and manage token usage effectively

---

#### Task 3.5: Update "Try With AI" Section (MUST) ⭐

**Priority**: P2
**Effort**: 2 hours
**FRs Covered**: Multiple (hands-on application)

**Acceptance Criteria**:
- [ ] Add 4 new configuration-focused prompts (keep existing tool prompts):
  1. **Basic Configuration**: "Help me create a `~/.gemini/settings.json` file with custom theme, vim mode enabled, and auto-compress at 75% capacity"
  2. **Security Setup**: "Create a project `.gemini/settings.json` that blocks `rm` and `sudo` commands but auto-approves all git operations"
  3. **Team Configuration**: "Design a team-shared settings.json for our React project with TypeScript: include context for `src/` and `tests/` directories, restrict file deletion, and set compression threshold to 80%"
  4. **Token Management**: "I'm analyzing a 50K-line codebase. Show me how to use /stats to monitor token usage and /compress when I hit 70% capacity"
- [ ] Add success indicator: "You're mastering configuration when you have both personal and project settings files customized"

**Success Criteria**: 70% create at least 1 custom settings.json file

---

**Lesson 3 Summary**:
- **Content Added**: ~410 lines across 4 major sections
- **FRs Covered**: FR-006, FR-007, FR-008, FR-010, FR-033
- **Hands-On Exercises**: 6 total (configuration practice)
- **Estimated Reading Time**: 30 minutes (up from 20 minutes)

---

### LESSON 4: Memory Management Systems (04-context-window-and-tool-comparison.md)

**Status**: 🔄 **Complete Redesign Required**
**Current Title**: "Choosing the Right AI Tool: When Size Matters"
**New Title**: "Memory Management: Teaching Gemini Your Project Context"

**Current State**: Lesson focuses on context window comparison (~310 lines)
**Current Sections** (REMOVE MOST):
- What is a context window (reading capacity analogy) - **REMOVE** (covered in Lesson 1)
- Simple comparison table (ChatGPT, Claude, Gemini) - **REMOVE** (covered in Lesson 1)
- When context size doesn't matter (80% of work) - **KEEP** (rename to "When Memory Matters")
- When context size matters (large codebases, multi-file analysis) - **TRANSFORM** to "When to Use Each Memory Type"

**Strategy**: This lesson will be **completely redesigned** to focus on Gemini CLI's 3 memory systems, not context window comparison. Context window comparison remains in Lesson 1.

**What to REMOVE** (~200 lines):
- Detailed context window explanations (move to Lesson 1 if not there)
- ChatGPT/Claude comparisons (redundant with Lesson 1)
- Generic "which tool to use" content (Lesson 1 covers this)

**What to ADD**: 4 major new sections (~540 lines total, replacing removed content = ~340 net new)

---

#### Task 4.1: Add Three Memory Types Overview (MUST) ⭐⭐⭐

**Priority**: P2
**Effort**: 6 hours
**FRs Covered**: FR-011 (three memory types)

**Acceptance Criteria**:
- [ ] **Rename file**: Consider renaming to `04-memory-management-systems.md` (or keep filename, change title in frontmatter)
- [ ] Add new introduction: "## Your AI's Three Brains: How Gemini Remembers"
- [ ] Explain memory philosophy: "Professional developers don't re-explain their project structure every session. Gemini CLI has 3 memory systems to learn your context once."
- [ ] Document 3 memory types with clear distinctions:

**1. Ephemeral Context (Session Memory)**
- **What**: Current conversation only
- **Lifetime**: Lost when session ends
- **Use Case**: Active discussion, current task focus
- **Example**: "Analyzing these 5 files right now"
- **Capacity**: 1M tokens

**2. GEMINI.md Files (Hierarchical Project Memory)**
- **What**: Persistent project instructions loaded automatically
- **Lifetime**: Permanent (file-based)
- **Use Case**: Project standards, coding conventions, architecture notes
- **Example**: "Always use TypeScript strict mode in this project"
- **Capacity**: Unlimited (limited by context window when loaded)
- **Hierarchy**: Global → Project → Subdirectory (covered in Task 4.2)

**3. save_memory (Long-Term Facts)**
- **What**: Discrete facts Gemini stores globally
- **Lifetime**: Permanent (stored in global GEMINI.md)
- **Use Case**: Personal preferences, API key locations, common patterns
- **Example**: "My API keys are always in .env.local, never .env"
- **Capacity**: Unlimited facts (kept concise)

- [ ] Create comparison table:
  | Memory Type | Persistence | Scope | Best For |
  |-------------|-------------|-------|----------|
  | Ephemeral | Session only | Current conversation | Active work |
  | GEMINI.md | Permanent | Project or global | Standards, architecture |
  | save_memory | Permanent | Global (all projects) | Personal facts |

- [ ] Business scenario: "Project GEMINI.md has React coding standards. Your personal save_memory knows you prefer functional components. Both apply when you work on this React project."

**Success Criteria**: 80% can explain when to use each memory type (SC-003 from spec)

---

#### Task 4.2: Add GEMINI.md Hierarchical System (MUST) ⭐⭐⭐

**Priority**: P2
**Effort**: 10 hours
**FRs Covered**: FR-012 (hierarchical loading), FR-013 (/memory commands), FR-015 (@file.md syntax)

**Acceptance Criteria**:
- [ ] Add new section: "## GEMINI.md Files: Your Project's Persistent Intelligence"
- [ ] Explain concept: "GEMINI.md is a markdown file that Gemini reads automatically at startup. Think of it as your project's README for AI."
- [ ] Document 3-level hierarchy with loading order:
  1. **Global** (`~/.gemini/GEMINI.md`): Personal preferences, global standards
  2. **Project** (`/project-root/.gemini/GEMINI.md`): Project-specific context
  3. **Subdirectory** (`/project-root/src/api/.gemini/GEMINI.md`): Component-specific notes
- [ ] Explain loading behavior:
  - All 3 levels load and combine hierarchically
  - More specific files (subdirectory) override general files (global)
  - View combined result with `/memory show`
- [ ] Provide 3 complete GEMINI.md templates:

**Template 1: Global GEMINI.md** (`~/.gemini/GEMINI.md`):
```markdown
# Global Development Preferences

## Coding Style
- Always use TypeScript strict mode
- Prefer functional programming patterns
- Use async/await over promises

## Security Rules
- API keys MUST be in .env.local, never .env
- Never commit .env files to git
- Always validate user input

## Project Structure Patterns I Use
- `src/` for source code
- `tests/` for test files
- `docs/` for documentation
```

**Template 2: Project GEMINI.md** (`/project/.gemini/GEMINI.md`):
```markdown
# E-Commerce Platform Project Context

## Technology Stack
- React 18 with TypeScript
- Node.js backend with Express
- PostgreSQL database
- Stripe for payments

## Business Domain
- B2B e-commerce for office supplies
- Target: Small businesses (10-100 employees)
- Key features: Bulk ordering, invoicing, account management

## Architecture
- Monorepo with 3 packages: web, api, shared
- API follows RESTful conventions
- Database uses row-level security

## Team Coding Standards
- Use React hooks, no class components
- All API routes require authentication
- Database queries must use parameterized statements
```

**Template 3: Subdirectory GEMINI.md** (`/project/src/api/.gemini/GEMINI.md`):
```markdown
# API Package Context

## This Package
- RESTful API endpoints
- Authentication: JWT tokens
- Rate limiting: 100 requests/min per user

## Key Files
- `routes/` - Express route definitions
- `middleware/` - Auth, validation, error handling
- `controllers/` - Business logic
- `models/` - Database schemas (Prisma)

## Development Guidelines
- All endpoints must have OpenAPI docs
- All mutations must have audit logging
- All responses must match TypeScript types in `shared/types`
```

- [ ] Explain `@file.md` syntax for modularization:
  ```markdown
  # Main GEMINI.md

  @security-rules.md
  @coding-standards.md
  @architecture.md
  ```
  - Gemini loads referenced files inline
  - Keeps main file concise
  - Share modules across projects

- [ ] Add 3 hands-on exercises:
  1. Create global `~/.gemini/GEMINI.md` with 3 personal preferences
  2. Create project `.gemini/GEMINI.md` with technology stack and standards
  3. Use `/memory show` to verify both files loaded

**Success Criteria**: 80% can create hierarchical GEMINI.md setup (SC-003 from spec)

---

#### Task 4.3: Add /memory Commands Reference (MUST) ⭐⭐

**Priority**: P2
**Effort**: 4 hours
**FRs Covered**: FR-013 (/memory commands)

**Acceptance Criteria**:
- [ ] Add new section: "## Memory Commands: Managing Your Context"
- [ ] Document 3 essential commands:

**1. `/memory show`** - View Combined Context
- Shows all loaded GEMINI.md files hierarchically combined
- Use when: "What context does Gemini have about my project?"
- Output: Complete GEMINI.md content (global + project + subdirectory)

**2. `/memory add <text>`** - Quick Fact Addition
- Appends text to global GEMINI.md under "## Gemini Added Memories"
- Use when: "Remember this important fact"
- Example: `/memory add Always use snake_case for database column names`

**3. `/memory reload`** - Refresh Context
- Reloads all GEMINI.md files without restarting CLI
- Use when: You edited GEMINI.md files and want changes active immediately
- Faster than restarting Gemini CLI

- [ ] Add workflow example:
  1. Create project GEMINI.md with coding standards
  2. Start Gemini CLI (auto-loads GEMINI.md)
  3. Ask coding question → Gemini applies standards from GEMINI.md
  4. Edit GEMINI.md to add new standard
  5. Run `/memory reload` to apply changes
  6. Ask another coding question → Gemini applies new standard

- [ ] Business scenario: "Mid-project, team decides on new naming convention. Update GEMINI.md, run `/memory reload`, continue working with new standard."

- [ ] Add 1 hands-on exercise: Create GEMINI.md, use `/memory show`, edit file, use `/memory reload`, verify with `/memory show`

**Success Criteria**: 75% can use /memory commands to manage context

---

#### Task 4.4: Add save_memory Tool Usage (MUST) ⭐⭐

**Priority**: P2
**Effort**: 4 hours
**FRs Covered**: FR-014 (save_memory vs GEMINI.md)

**Acceptance Criteria**:
- [ ] Add new section: "## save_memory: Your AI's Long-Term Knowledge Base"
- [ ] Explain when save_memory is used:
  - Gemini invokes save_memory automatically when you say "remember that..."
  - User cannot invoke save_memory directly (it's a tool Gemini uses)
  - Facts stored globally in `~/.gemini/GEMINI.md` under "## Gemini Added Memories"
- [ ] Differentiate from manual GEMINI.md:
  - **save_memory**: Quick facts, Gemini manages, conversational
  - **Manual GEMINI.md**: Structured documentation, you manage, organized by project
- [ ] Document save_memory workflow:
  1. You: "Remember that I always use Prettier for code formatting"
  2. Gemini uses save_memory internally
  3. Fact appended to `~/.gemini/GEMINI.md`
  4. Future sessions: Gemini auto-applies Prettier preference
- [ ] Show example `~/.gemini/GEMINI.md` after several save_memory operations:
  ```markdown
  ## Gemini Added Memories

  - I always use Prettier for code formatting
  - My API keys are stored in .env.local, never .env
  - I prefer functional programming with React hooks
  - My test files use Jest with React Testing Library
  - I organize imports: external libs, then internal modules
  ```
- [ ] When to use each:
  - **save_memory**: Spontaneous facts during conversation ("Oh, remember API keys go in .env.local")
  - **GEMINI.md**: Planned, structured project documentation (architecture, standards, domain knowledge)
- [ ] Add 2 hands-on exercises:
  1. Tell Gemini 3 facts to remember (coding style, file locations, preferences)
  2. Restart Gemini CLI, verify it remembers by asking questions that require those facts

**Success Criteria**: 80% understand save_memory vs GEMINI.md tradeoffs

---

#### Task 4.5: Update "Try With AI" Section (MUST) ⭐

**Priority**: P2
**Effort**: 2 hours
**FRs Covered**: Multiple (hands-on memory practice)

**Acceptance Criteria**:
- [ ] Replace all existing "Try With AI" prompts with 4 memory-focused exercises:
  1. **Memory Type Selection**: "I have 3 scenarios. Tell me which memory type to use: (1) Current analysis of competitor pricing (2) Our team's React component naming conventions (3) My personal preference for error handling patterns"
  2. **GEMINI.md Setup**: "Help me create a complete project GEMINI.md for a Django e-commerce API. Include: tech stack, architecture (REST API with PostgreSQL), coding standards (PEP 8, type hints), and business domain (B2C fashion retail)"
  3. **Hierarchical Memory**: "I have a monorepo with 3 packages: web, api, database. Design a GEMINI.md hierarchy: global file with my personal preferences, project root with monorepo architecture, and package-specific files for each package"
  4. **Memory Commands Workflow**: "Walk me through: (1) Creating a GEMINI.md with 3 coding standards (2) Using /memory show to verify it loaded (3) Adding a 4th standard via /memory add (4) Using /memory show again to confirm"
- [ ] Add success indicator: "You've mastered memory when Gemini knows your project context without you explaining it every session"

**Success Criteria**: 70% complete all 4 memory exercises successfully

---

**Lesson 4 Summary**:
- **Content Removed**: ~200 lines (context window comparison moved to Lesson 1)
- **Content Added**: ~540 lines across 4 major sections (3 memory systems)
- **Net Change**: +340 lines
- **FRs Covered**: FR-011, FR-012, FR-013, FR-014, FR-015
- **Hands-On Exercises**: 9 total (GEMINI.md creation, /memory commands, save_memory)
- **Estimated Reading Time**: 35 minutes (up from 25 minutes)

---

### LESSON 5: MCP, Custom Commands & Extensions (05-mcp-and-extensions-ecosystem.md)

**Status**: 🔥 **Major Expansion Required**
**Current State**: Good MCP introduction (~330 lines), but missing configuration details, custom commands, and extensions
**Current Sections** (KEEP):
- Part 1: Understanding MCP (business explanation, electrical outlet analogy)
- Part 2: Adding MCP servers (Playwright, Context7 examples)
- Part 3: Business workflows using MCP servers (competitor research, API documentation)
- Part 4: Gemini CLI Extensions (concept introduction)

**What to REMOVE**: Nothing (all content provides good foundation)

**What to ADD**: 5 major new sections (~520 lines total)

---

#### Task 5.1: Add MCP Server Configuration Deep Dive (MUST) ⭐⭐⭐

**Priority**: P3
**Effort**: 6 hours
**FRs Covered**: FR-017 (MCP configuration), FR-019 (/mcp command)

**Acceptance Criteria**:
- [ ] Add new section: "## Configuring MCP Servers: Complete Guide"
- [ ] Expand on existing settings.json examples with full configuration options:

**Basic MCP Server** (Playwright):
```json
{
  "mcpServers": {
    "playwright": {
      "command": "npx",
      "args": ["@playwright/mcp@latest"]
    }
  }
}
```

**MCP Server with Tool Filtering**:
```json
{
  "mcpServers": {
    "github": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-github"],
      "env": {
        "GITHUB_TOKEN": "$GITHUB_PAT"
      },
      "allowed": ["search_repositories", "get_file_contents"],
      "excluded": ["create_or_update_file", "push_files"]
    }
  }
}
```
- Explain: Restrict GitHub server to read-only operations

**Multiple MCP Servers**:
```json
{
  "mcpServers": {
    "playwright": {
      "command": "npx",
      "args": ["@playwright/mcp@latest"]
    },
    "context7": {
      "command": "npx",
      "args": ["-y", "@upstash/context7-mcp"]
    },
    "github": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-github"],
      "env": {
        "GITHUB_TOKEN": "$GITHUB_PAT"
      }
    }
  }
}
```

- [ ] Add `/mcp` command documentation:
  - Lists all configured MCP servers
  - Shows connection status (connected/disconnected)
  - Lists available tools from each server
  - Use when: "Which MCP servers are active?"
- [ ] Add `/mcp refresh` command:
  - Reconnects to MCP servers after configuration changes
  - Use when: You edited settings.json to add/remove servers
- [ ] Business scenario: "Configure Playwright for competitor research, Context7 for documentation, GitHub for repository analysis—all accessible through one Gemini CLI session"
- [ ] Add 2 hands-on exercises:
  1. Configure Playwright MCP server, use `/mcp` to verify connection
  2. Add Context7 to same settings.json, use `/mcp refresh`, verify both servers active

**Success Criteria**: 75% can install and configure 2+ MCP servers (SC-004 from spec)

---

#### Task 5.2: Add Practical MCP Workflows (MUST) ⭐⭐

**Priority**: P3
**Effort**: 5 hours
**FRs Covered**: FR-018 (Playwright, Context7, GitHub examples)

**Acceptance Criteria**:
- [ ] Expand existing "Business Workflows" section with 3 additional realistic examples:

**Workflow 4: GitHub Repository Analysis** (GitHub MCP):
```
Your goal: Understand competitor's open-source architecture by analyzing their GitHub repo

Prompt:
"Use the GitHub MCP server to:
1. Search for [competitor-name] repositories related to [domain]
2. Read the README from their main repository
3. List the 10 most recent pull requests
4. Analyze file structure (what directories exist)
5. Summarize their architecture based on code organization"

What Gemini does:
1. Searches GitHub for matching repositories
2. Retrieves README content
3. Fetches PR data via GitHub API
4. Analyzes directory structure
5. Provides architecture summary with links to specific files

Business value: Understand competitor tech stack without manual repo exploration (saves 1-2 hours)
```

**Workflow 5: Multi-Source Research** (Playwright + Context7):
```
Your goal: Research best practices for implementing payments in your app

Prompt:
"I'm adding Stripe payments to my e-commerce site. Use:
- Context7 to get latest Stripe API documentation
- Playwright to browse 3 competitor payment flows:
  - competitor-a.com/checkout
  - competitor-b.com/payment
  - competitor-c.com/buy

Create a comparison: what payment methods do competitors support, how does Stripe documentation recommend implementing each, and what's the recommended approach for my use case?"

What Gemini does:
1. Queries Context7 for current Stripe payment method docs
2. Launches Playwright to browse 3 competitor sites
3. Extracts payment method support from each
4. Synthesizes: official docs + competitor analysis + recommendation

Business value: Comprehensive research in 10 minutes vs. 2-3 hours manually
```

**Workflow 6: Live Documentation Debugging** (Context7):
```
Your goal: Understand breaking changes in framework update

Prompt:
"Our app uses Next.js 13. Next.js 14 was just released. Use Context7 to:
1. Fetch Next.js 14 migration guide
2. Identify breaking changes that affect App Router
3. Explain what code changes we need to make
4. Provide example before/after code for each breaking change"

What Gemini does:
1. Queries Context7 for Next.js 14 documentation
2. Identifies breaking changes section
3. Extracts App Router changes specifically
4. Generates migration examples

Business value: Immediate understanding of upgrade requirements (save 30-60 min research)
```

- [ ] Add hands-on exercise for each workflow (3 total)
- [ ] Include expected output examples (what success looks like)

**Success Criteria**: 70% can execute multi-tool workflows independently

---

#### Task 5.3: Add Custom Slash Commands Creation (MUST) ⭐⭐⭐

**Priority**: P3
**Effort**: 8 hours
**FRs Covered**: FR-021 (TOML structure), FR-022 (naming), FR-023 (user vs project scope), FR-024 ({{args}}), FR-025 (!{command})

**Acceptance Criteria**:
- [ ] Add new section: "## Custom Slash Commands: Automate Your Workflows"
- [ ] Explain concept: "Turn repeated multi-step prompts into single commands"
- [ ] Document TOML file structure with complete example:

**Basic Custom Command** (`~/.gemini/commands/research.toml`):
```toml
description = "Research a competitor and create analysis report"
prompt = """
Research {{args}} (competitor name) and provide:

1. Company overview (founding, size, funding)
2. Product offerings (main products/services)
3. Technology stack (if public/observable)
4. Pricing model
5. Target market
6. Recent news (last 3 months)

Create a structured markdown report with citations.
"""
```
- Invocation: `/research Shopify`
- `{{args}}` is replaced with "Shopify"

**Command with Shell Execution** (`~/.gemini/commands/git/review.toml`):
```toml
description = "Review recent git changes and suggest improvements"
prompt = """
First, get the recent commit log:
!{git log --oneline -10}

Then, show what changed in the last commit:
!{git diff HEAD~1}

Based on this context:
1. Summarize what changed
2. Identify any code quality issues
3. Suggest improvements
4. Check if commit messages are clear

Provide actionable feedback.
"""
```
- Invocation: `/git:review`
- `!{command}` executes shell commands, output included in prompt

**Command with Multiple Arguments** (`~/.gemini/commands/compare.toml`):
```toml
description = "Compare two files or technologies"
prompt = """
Compare {{args}} (provide two items separated by 'vs')

Analyze:
1. Key differences
2. Strengths and weaknesses
3. Use cases for each
4. Recommendation based on context

Format as comparison table.
"""
```
- Invocation: `/compare React vs Vue`
- `{{args}}` receives "React vs Vue"

- [ ] Document naming and scoping:
  - **User-scoped**: `~/.gemini/commands/cmd.toml` → `/cmd` (all projects)
  - **Project-scoped**: `.gemini/commands/cmd.toml` → `/cmd` (this project only)
  - **Namespaced**: `commands/git/commit.toml` → `/git:commit` (avoid name conflicts)
- [ ] Explain precedence: Project commands override user commands with same name
- [ ] Business scenarios:
  - **Team workflow**: Share `.gemini/commands/` in git for consistent team commands
  - **Personal automation**: User commands for your repetitive tasks
- [ ] Add 3 hands-on exercises:
  1. Create `/analyze` command that analyzes a file or directory structure
  2. Create `/test-plan` command that generates test cases from requirements
  3. Create namespaced `/docs:generate` command with shell execution

**Success Criteria**: 70% can create custom commands (SC-008 from spec)

---

#### Task 5.4: Add Extensions Installation & Management (MUST) ⭐⭐

**Priority**: P3
**Effort**: 6 hours
**FRs Covered**: FR-026 (extensions vs MCP), FR-027 (structure), FR-028 (installation), FR-029 (security evaluation)

**Acceptance Criteria**:
- [ ] Add new section: "## Extensions: Pre-Packaged Capabilities"
- [ ] Differentiate from MCP servers:
  - **MCP Server**: Individual tool (Playwright, Context7, GitHub)
  - **Extension**: Bundle of MCP servers + custom commands + GEMINI.md context + settings
- [ ] Explain extension structure (`gemini-extension.json`):
  ```json
  {
    "name": "business-intelligence",
    "version": "1.0.0",
    "description": "Competitive research and market analysis toolkit",
    "mcpServers": {
      "playwright": {
        "command": "npx",
        "args": ["@playwright/mcp@latest"]
      },
      "context7": {
        "command": "npx",
        "args": ["-y", "@upstash/context7-mcp"]
      }
    },
    "commands": ["research.toml", "compare.toml"],
    "contextFileName": "BI_CONTEXT.md",
    "excludeTools": []
  }
  ```
- [ ] Document installation methods:
  1. **From GitHub**: `gemini extensions install github:username/extension-name`
  2. **From URL**: `gemini extensions install https://github.com/user/repo`
  3. **From local directory**: `gemini extensions link /path/to/extension` (development)
- [ ] Add security evaluation framework:
  **Before installing extension, ask these 4 questions:**
  1. **Who created it?**
     - Official Gemini team? ✅ High trust
     - Reputable company? ⚠️ Verify reputation
     - Individual developer? ⚠️ Check GitHub activity, stars, issues
  2. **What does it include?**
     - View `gemini-extension.json` on GitHub
     - Check which MCP servers it installs
     - Review custom commands it adds
     - Read GEMINI.md context it loads
  3. **What data access does it need?**
     - Read-only tools? ✅ Lower risk
     - Write access to files? ⚠️ Moderate risk
     - Shell command execution? ⚠️ Higher risk
     - API keys required? ⚠️ Verify security practices
  4. **Is it maintained?**
     - Recent commits (last 3 months)? ✅ Active
     - Open issues addressed? ✅ Responsive
     - No activity for 6+ months? ⚠️ Potentially abandoned
- [ ] Add decision framework:
  - **Install extension if**: Bundles 3+ tools you need, from trusted source, actively maintained
  - **Install individual MCP servers if**: You only need 1-2 specific tools, want full control
- [ ] Business scenario: "Business Intelligence extension bundles Playwright + Context7 + custom `/research` and `/compare` commands. Install once, get complete competitive analysis toolkit."
- [ ] Add 2 hands-on exercises:
  1. Research available Gemini CLI extensions on GitHub (ask Gemini to search)
  2. Evaluate one extension using the 4-question security framework

**Success Criteria**: 80% can evaluate extension security (SC-009 from spec)

---

#### Task 5.5: Add Advanced MCP Features (SHOULD) ⭐

**Priority**: P3
**Effort**: 4 hours
**FRs Covered**: FR-020 (security considerations)

**Acceptance Criteria**:
- [ ] Add new section: "## Advanced: MCP Server Development & Security"
- [ ] Explain when to build custom MCP server:
  - You need integration with proprietary system (company database, internal API)
  - Public MCP servers don't exist for your tool
  - You want to share custom integration with team/community
- [ ] Reference official MCP server development guide:
  - Link: https://modelcontextprotocol.io/docs/building-servers
  - Mention: "Building MCP servers requires TypeScript/Python knowledge (advanced topic)"
- [ ] Document security best practices:
  1. **Principle of least privilege**: Only grant MCP servers permissions they need
  2. **Tool filtering**: Use `allowed` and `excluded` to restrict operations
  3. **Environment variable isolation**: Use `.env` for secrets, not hardcoded in settings.json
  4. **Review before auto-approval**: Don't add MCP server tools to `tools.allowed` until you trust them
- [ ] Add hands-on exercise: Configure GitHub MCP with read-only restrictions (no create/push)

**Success Criteria**: 60% understand when to build custom MCP servers vs. use existing ones

---

#### Task 5.6: Update "Try With AI" Section (MUST) ⭐

**Priority**: P3
**Effort**: 3 hours
**FRs Covered**: Multiple (hands-on application)

**Acceptance Criteria**:
- [ ] Replace existing "Try With AI" with 4 comprehensive exercises:
  1. **MCP Workflow Design**: "I want to research 5 competitors monthly (pricing, features, recent news). Design a complete workflow using Playwright and Context7 MCP servers. Include: settings.json configuration, prompt template, expected output format, time savings estimate"
  2. **Custom Command Creation**: "Create a `/competitor-report <company-name>` custom command that: (1) Uses Playwright to fetch their homepage and pricing page (2) Uses Context7 to research their technology stack (3) Generates a markdown report with sections: Overview, Products, Pricing, Tech Stack, Analysis"
  3. **Extension Evaluation**: "Find a Gemini CLI extension on GitHub for [your domain: web dev, data analysis, business intel]. Evaluate it using the 4-question security framework. Should I install it or build individual MCP servers? Explain your recommendation."
  4. **End-to-End Setup**: "I'm a business analyst researching SaaS competitors. Set up my complete Gemini CLI environment: (1) Install Playwright and Context7 MCP servers (2) Create 2 custom commands: /research and /compare (3) Configure tool restrictions (no file deletion) (4) Create project GEMINI.md with SaaS industry context"
- [ ] Add success indicator: "You've mastered extensibility when you have 3+ custom commands and 2+ MCP servers configured for your workflow"

**Success Criteria**: 65% complete at least 2 of 4 advanced exercises

---

**Lesson 5 Summary**:
- **Content Added**: ~520 lines across 5 major sections
- **FRs Covered**: FR-017, FR-018, FR-019, FR-020, FR-021, FR-022, FR-023, FR-024, FR-025, FR-026, FR-027, FR-028, FR-029
- **Hands-On Exercises**: 13 total (MCP config, workflows, custom commands, extensions)
- **Estimated Reading Time**: 50 minutes (up from 25 minutes)

---

### README UPDATE

**Status**: 📝 **Complete Rewrite Required**
**Current State**: Basic chapter overview (~78 lines)

---

#### Task 6.1: Rewrite Chapter README (MUST) ⭐⭐

**Priority**: P2
**Effort**: 3 hours
**FRs Covered**: All (chapter overview)

**Acceptance Criteria**:
- [ ] Replace existing README.md with comprehensive chapter overview
- [ ] Include updated frontmatter:
  ```yaml
  ---
  sidebar_position: 6
  title: "Chapter 6: Google Gemini CLI: Open Source and Everywhere"
  ---
  ```
- [ ] Add executive summary highlighting 5 enhanced lessons
- [ ] Create updated "What You'll Learn" section with all 35 functional requirements represented
- [ ] Add lesson breakdown table:
  | Lesson | Title | Time | Key Topics |
  |--------|-------|------|------------|
  | 1 | Why Gemini CLI Matters | 15 min | Open source, free tier, MCP, decision framework |
  | 2 | Installation & Core Commands | 40 min | Install, auth, slash commands, @/! syntax, shortcuts, invocation modes, save_memory |
  | 3 | Built-In Tools & Configuration | 30 min | File ops, web tools, settings.json hierarchy, tool restrictions, token management |
  | 4 | Memory Management Systems | 35 min | 3 memory types, GEMINI.md hierarchical, /memory commands, save_memory vs GEMINI.md |
  | 5 | MCP, Custom Commands & Extensions | 50 min | MCP config, Playwright/Context7/GitHub, custom TOML commands, extensions |
  | **Total** | **5 lessons** | **~170 min** | **35 functional requirements covered** |

- [ ] Add prerequisites section:
  - Completed Chapters 1-5 (AI tool landscape, Claude Code)
  - Node.js 20+ installed
  - Terminal/command-line comfort
  - Google account (for free tier)
- [ ] Add learning outcomes aligned with success criteria:
  - "Master 14+ essential slash commands for productivity (90% recall)"
  - "Configure Gemini CLI with settings.json for security and efficiency (85% proficiency)"
  - "Implement 3-tier memory system (ephemeral, GEMINI.md, save_memory) (80% mastery)"
  - "Install and use MCP servers for business intelligence (75% capability)"
  - "Create custom slash commands to automate workflows (70% creation)"
  - "Evaluate and install extensions securely (80% security evaluation)"
- [ ] Add "Chapter Completion Criteria":
  - ✅ Gemini CLI installed and authenticated
  - ✅ Personal `~/.gemini/settings.json` with 5+ custom settings
  - ✅ Project `.gemini/GEMINI.md` with coding standards
  - ✅ 2+ MCP servers configured (e.g., Playwright, Context7)
  - ✅ 2+ custom slash commands created
  - ✅ Can articulate when to use Gemini CLI vs. Claude Code
- [ ] Add cross-references:
  - Previous: Chapter 5 (AI tool landscape comparison)
  - Next: Chapter 7 (Advanced AI workflows)
- [ ] Update estimated total time: 170 minutes core + optional deep dives

**Success Criteria**: README accurately reflects all chapter enhancements and learning outcomes

---

## Cross-Lesson Tasks

### Task 7.1: Quality Assurance Review (MUST) ⭐⭐

**Priority**: P2
**Effort**: 8 hours
**FRs Covered**: All (quality validation)

**Acceptance Criteria**:
- [ ] Cross-platform testing:
  - Test all code examples on Windows (PowerShell and Command Prompt)
  - Test all code examples on macOS (Terminal)
  - Test all code examples on Linux (Ubuntu/Debian)
- [ ] Example validation:
  - All JSON snippets are syntactically valid
  - All TOML snippets are syntactically valid
  - All shell commands are safe and tested
  - All file paths use correct separators for platform
- [ ] Content consistency:
  - No contradictions between lessons
  - Cross-references are accurate
  - Terminology is consistent (e.g., "slash commands" vs "/commands")
  - All FR references are correct
- [ ] Complexity tier validation:
  - Part 2 (Intermediate) tier maintained: 3-4 options, 7 concepts/section
  - No beginner oversimplification
  - No professional-level complexity creep
- [ ] Accessibility:
  - Reading level appropriate (Grade 8-10)
  - Code examples have clear explanations
  - Technical jargon is defined on first use
  - All acronyms spelled out (MCP = Model Context Protocol)

**Success Criteria**: 90% of examples runnable without modification (SC-015 from spec)

---

### Task 7.2: Create Validation Checklist (MUST) ⭐

**Priority**: P2
**Effort**: 2 hours
**FRs Covered**: All (completion validation)

**Acceptance Criteria**:
- [ ] Create `checklists/implementation.md` in specs directory
- [ ] Include checklist for each lesson with acceptance criteria from tasks
- [ ] Add cross-cutting concerns:
  - [ ] All 35 functional requirements covered across 5 lessons
  - [ ] All 15 success criteria measurable
  - [ ] Estimated reading time: 60-90 minutes core content (target met: ~100 min)
  - [ ] Hands-on exercises: 40+ total across all lessons
  - [ ] Cross-platform testing complete (Windows, macOS, Linux)
  - [ ] All code examples validated
  - [ ] All GEMINI.md templates tested
  - [ ] All settings.json examples validated
  - [ ] All MCP server examples functional
  - [ ] All custom command examples working
  - [ ] README updated with accurate lesson overview
  - [ ] No contradictions between lessons
  - [ ] Complexity tier appropriate (Part 2: Intermediate)
- [ ] Add sign-off section for technical reviewer and proof validator

**Success Criteria**: Checklist enables systematic validation before chapter publication

---

### Task 7.3: Documentation & Cross-References (SHOULD) ⭐

**Priority**: P3
**Effort**: 3 hours
**FRs Covered**: N/A (navigation and discoverability)

**Acceptance Criteria**:
- [ ] Add "What's Next" section to each lesson linking to next lesson
- [ ] Add "Review" section to each lesson linking to prerequisite lessons
- [ ] Create visual aids (ASCII diagrams or descriptions for):
  - Configuration hierarchy (7 levels)
  - Memory types (3 systems)
  - MCP architecture (CLI → server → external)
  - GEMINI.md hierarchy (global → project → subdirectory)
- [ ] Add troubleshooting subsections:
  - Lesson 2: "Command not found" → Check installation
  - Lesson 3: "Settings not applying" → Check precedence
  - Lesson 4: "GEMINI.md not loading" → Check file locations
  - Lesson 5: "MCP server won't connect" → Check configuration, use `/mcp`
- [ ] Add glossary section to README:
  - Slash commands (/): Built-in CLI commands
  - @ syntax: File/directory reference
  - ! syntax: Shell command execution
  - MCP: Model Context Protocol (external tool integration)
  - GEMINI.md: Project context file (hierarchical)
  - save_memory: Long-term fact storage
  - Extensions: Bundled capabilities (MCP + commands + context)

**Success Criteria**: Readers can navigate between lessons and find answers to common issues

---

## Summary & Metrics

### Implementation Scope

| Category | Metric | Value |
|----------|--------|-------|
| **Lessons Updated** | Total | 5 (all existing) |
| **Lessons Created** | Total | 0 (update strategy) |
| **Content Added** | Lines | ~2,060 |
| **Content Removed** | Lines | ~200 (redundant context window content) |
| **Net Content** | Lines | ~1,860 new (from 1,210 → 3,270 total) |
| **Functional Requirements** | Covered | 35 of 35 (100%) |
| **Success Criteria** | Defined | 15 measurable outcomes |
| **Hands-On Exercises** | Total | 42 across all lessons |
| **Reading Time** | Core | ~100 minutes (target: 60-90) |
| **Reading Time** | With Advanced | ~175 minutes |

### Task Priority Breakdown

| Priority | Count | Effort (hours) | Description |
|----------|-------|----------------|-------------|
| **P1 (MUST)** | 21 tasks | 110 hours | Critical path: commands, syntax, memory, configuration |
| **P2 (MUST)** | 18 tasks | 73 hours | Important: configuration deep dive, memory systems, QA |
| **P3 (MUST)** | 5 tasks | 29 hours | Advanced: MCP, custom commands, extensions |
| **SHOULD** | 2 tasks | 7 hours | Nice-to-have: Advanced MCP, visual aids |
| **Total** | 46 tasks | 219 hours | ~9 weeks at 25 hrs/week |

### Functional Requirements Coverage

| Lesson | FRs Covered | Primary FRs |
|--------|-------------|-------------|
| **Lesson 1** | FR-001 | Decision framework foundation |
| **Lesson 2** | FR-001 to FR-005, FR-011 | All core commands, syntax, shortcuts, modes |
| **Lesson 3** | FR-006 to FR-010, FR-033 | Configuration hierarchy, settings, token management |
| **Lesson 4** | FR-011 to FR-015 | Memory types, GEMINI.md, /memory commands |
| **Lesson 5** | FR-016 to FR-030 | MCP, custom commands, extensions |
| **Advanced** | FR-031, FR-032, FR-034, FR-035 | Checkpointing, branching, IDE, sandbox (referenced throughout) |

### Success Criteria Mapping

All 15 success criteria from specification are measurable through exercises and "Try With AI" prompts:

- **SC-001 to SC-004**: Skill acquisition (commands, config, memory, MCP) - Measured via exercises
- **SC-005 to SC-007**: Productivity improvement - Demonstrated in workflows, measured in time savings
- **SC-008 to SC-010**: Tool mastery - Measured via custom command creation, extension evaluation
- **SC-011 to SC-012**: Workflow integration - Measured via decision framework exercises
- **SC-013 to SC-015**: Content quality - Time estimates, complexity tier, cross-platform testing

---

## Implementation Timeline

### Phase 1: Foundation (Weeks 1-3)
- Lesson 1 minor update (Task 1.1)
- Lesson 2 major expansion (Tasks 2.1-2.7)
- ~35 hours

### Phase 2: Configuration & Memory (Weeks 4-6)
- Lesson 3 expansion (Tasks 3.1-3.5)
- Lesson 4 redesign (Tasks 4.1-4.5)
- ~50 hours

### Phase 3: Advanced Features (Weeks 7-8)
- Lesson 5 expansion (Tasks 5.1-5.6)
- ~35 hours

### Phase 4: Integration & QA (Week 9)
- README rewrite (Task 6.1)
- Cross-lesson tasks (Tasks 7.1-7.3)
- Final validation
- ~20 hours

**Total Timeline**: 9 weeks at 25 hours/week (part-time) or 5-6 weeks at 40 hours/week (full-time)

---

## Risk Mitigation

### Risk 1: Content Overwhelm
**Risk**: Adding 2,060 lines might overwhelm readers
**Mitigation**:
- Core content remains ~100 min (within 60-90 min target with flexibility)
- Advanced sections (custom commands, extensions) marked as optional
- Progressive disclosure: basics → intermediate → advanced
- Hands-on exercises break up reading

### Risk 2: Platform Compatibility
**Risk**: Examples fail on specific OS (Windows, macOS, Linux)
**Mitigation**:
- Task 7.1: Explicit cross-platform testing requirement
- All file paths use platform-agnostic examples
- PowerShell vs Command Prompt differences noted for Windows
- Shell syntax compatible with bash/zsh (macOS/Linux) and PowerShell (Windows)

### Risk 3: Rapid Gemini CLI Evolution
**Risk**: New versions break examples
**Mitigation**:
- Pin examples to specific version (0.4.0+) in prerequisites
- Add note: "Examples current as of January 2025, verify with latest docs"
- Link to official documentation for exhaustive reference
- Focus on stable features (slash commands, configuration, MCP protocol)

### Risk 4: MCP Server Availability
**Risk**: Recommended MCP servers (Playwright, Context7, GitHub) become unavailable
**Mitigation**:
- Choose widely-adopted servers with strong backing
- Explain MCP server discovery process (teach fishing, not give fish)
- Include alternative server suggestions
- Emphasize building custom servers for critical dependencies

---

## Next Steps After Task Completion

1. **Phase Review**: Validate all 46 tasks completed with acceptance criteria met
2. **Cross-Platform Testing**: Run all examples on Windows, macOS, Linux (Task 7.1)
3. **Technical Review**: Invoke technical-reviewer subagent for accuracy validation
4. **Proof Validation**: Invoke proof-validator subagent for editorial quality
5. **Integration**: Merge enhanced chapter into main book structure
6. **Cross-References**: Update Chapters 5 and 7 to reference new Chapter 6 content
7. **Publication**: Update Docusaurus build and deploy

---

## Approval Required

Before proceeding to implementation (lesson-writer subagent), confirm:

- ✅ Strategy: Update all 5 existing lessons (no new lessons created)
- ✅ Scope: 2,060 lines of new content across 46 tasks
- ✅ Timeline: 9 weeks part-time or 5-6 weeks full-time
- ✅ Coverage: All 35 functional requirements mapped to specific lesson sections
- ✅ Quality: 42 hands-on exercises, cross-platform testing, QA validation

**Ready to proceed**: Yes / No / Modifications needed

---

**Task Specification Complete**
**Created**: 2025-11-07
**Specification**: specs/chapter-6-google-gemini-cli/spec.md
**Plan**: specs/chapter-6-google-gemini-cli/plan.md
**Tasks**: specs/chapter-6-google-gemini-cli/tasks.md (this document)