# Feature Specification: Chapter 6 Redesign - Gemini CLI Installation and Basics

**Feature Branch**: `001-chapter-6-redesign`
**Created**: 2025-01-17
**Status**: Draft
**Input**: Refine Chapter 6 @book-source\docs\02-AI-Tool-Landscape\06-gemini-cli-installation-and-basics. Current issues: (1) lessons lack metadata (skills, learning objectives), (2) some content too verbose, violating "show through doing" principle, (3) lessons drift from core goal—teaching AI-driven development through systematic practice, not tool tutorials. Restructure to embody book's methodology.

---

## Executive Summary

This specification defines the redesign of Chapter 6 (Google Gemini CLI: Open Source and Everywhere) to teach AI tool selection judgment through **error analysis pedagogy** rather than feature documentation. The chapter serves A2 beginner students (Part 2: AI Tool Landscape) who completed Chapter 5 (Claude Code) and need decision frameworks for choosing the right AI tool for specific development scenarios.

**Key Design Decisions**:
- **Error analysis modality**: Students try wrong tools first, analyze failures, discover better fits (varying from Chapter 5's direct teaching)
- **Tool selection framework**: Six objective dimensions (licensing, cost, context window, extensibility, interface, support) guide decisions
- **4-Stage progression**: Manual foundation → AI collaboration (Three Roles) → Reusable intelligence → Extension development capstone
- **Cognitive load calibration**: A2 tier (5-7 concepts/section max, 2 workflow options max, heavy scaffolding)
- **Metadata completeness**: CEFR + Bloom's + DigComp 2.2 mappings for every learning objective

**Constitutional Grounding**: Principles 1-7 applied with anti-convergence (Chapter 5 used direct teaching, Chapter 6 uses error analysis).

---

## Constitutional Intelligence Object

```json
{
  "task_characterization": {
    "type": "educational_content_redesign",
    "domain": "ai_tool_landscape_gemini_cli",
    "audience": "A2_beginner",
    "part": 2,
    "chapter": 6,
    "existing_structure": "9_lessons_total",
    "implementation_status": "8_complete_1_pending",
    "novel_approach": "demonstration_pedagogy_with_beginner_examples"
  },
  "constitutional_frameworks": {
    "teaching_stages": "1_manual → 2_ai_collab_three_roles → 3_intelligence → 4_extension_capstone",
    "cognitive_tier": "A2",
    "concept_limit_per_section": "5-7",
    "scaffolding": "heavy",
    "options_max": 2,
    "teaching_modality": "error_analysis",
    "anti_convergence": "chapter_5_used_direct_teaching",
    "verification_required": "all_cli_commands_tested + gemini_features_cited_from_context7"
  },
  "pedagogical_arc": {
    "foundation": "Install CLI, execute basic commands, understand interface",
    "application": "Error analysis: Try wrong tool → Understand why → Discover right tool",
    "integration": "Three Roles framework (AI as Teacher/Student/Co-Worker) with Gemini",
    "validation": "Tool selection decision matrix for real scenarios",
    "mastery": "Capstone: Build custom MCP server or slash command extension"
  },
  "research_requirements": {
    "depth": "targeted_5_10hrs",
    "verify_gemini_cli": "Context7: @google/gemini-cli library documentation",
    "test_all_commands": "execute_in_terminal_attach_logs",
    "cite_official_docs": "developers.google.com/gemini, github.com/google/gemini-cli"
  },
  "user_decisions_captured": {
    "q1_teaching_modality": "C_error_analysis",
    "q2_capstone_scope": "C_extension_development",
    "q3_metadata_granularity": "C_cefr_blooms_digcomp"
  }
}
```

---

## User Scenarios & Testing

### User Story 1 - Tool Selection Through Error Analysis (Priority: P1)

**Scenario**: Student wants to search recent Python documentation while coding. They have both Claude Code and Gemini CLI available. They're unsure which tool to use.

**Journey**:
1. Student tries Claude Code first (familiar tool from Chapter 5)
2. Asks Claude: "Search latest Python 3.13 documentation for asyncio changes"
3. Claude responds: "I don't have real-time web search. I can explain asyncio from my training data (Jan 2025), but cannot verify latest 3.13 changes."
4. Student recognizes limitation: Claude lacks real-time search
5. Switches to Gemini CLI and asks same question
6. Gemini executes Google Search tool, returns 3.13 release notes from python.org
7. Student realizes: **Gemini excels when fresh web data needed**
8. Documents decision pattern: "Use Gemini for real-time research, Claude for code generation from known patterns"

**Why this priority**: Core learning objective—students build tool selection judgment through productive failure, not memorization.

**Independent Test**: Can be tested by presenting scenario, asking student which tool to use and why. Delivers decision framework mastery.

**Acceptance Scenarios**:

1. **Given** student needs to search current documentation, **When** student tries Claude first and hits limitation, **Then** student recognizes "no real-time search" and switches to Gemini
2. **Given** student uses Gemini for web search, **When** Gemini returns current results, **Then** student articulates "Gemini has Google Search integration" as differentiator
3. **Given** student encounters 5 different scenarios, **When** asked which tool fits each, **Then** student correctly maps 4/5 scenarios to appropriate tool using decision framework

---

### User Story 2 - Three Roles Framework with Gemini (Priority: P2)

**Scenario**: Student building a CLI tool and wants Gemini to suggest shell integration patterns (AI as Teacher), but Gemini doesn't know student's Windows-specific constraints (AI as Student), leading to iterative refinement (AI as Co-Worker).

**Journey**:
1. Student asks Gemini: "Help me add shell integration to my CLI tool"
2. Gemini suggests Unix-style shell scripts (AI teaches general pattern)
3. Student refines: "I'm on Windows PowerShell, those commands won't work" (Student teaches AI context)
4. Gemini adapts: Provides PowerShell-specific approach
5. Student tests, finds permissions issue
6. Iterates with Gemini: "Got permission error, here's the output..." (Co-Worker convergence)
7. Gemini suggests `ExecutionPolicy` fix
8. Student tests again, succeeds
9. Reflects: "I taught AI my constraints, AI taught me PowerShell patterns, together we converged on working solution"

**Why this priority**: Three Roles framework is core pedagogical innovation (constitutional Section IIa). Must demonstrate with Gemini, not just Claude.

**Independent Test**: Can be tested by assigning task requiring constraint communication and iteration. Delivers co-learning skill mastery.

**Acceptance Scenarios**:

1. **Given** Gemini suggests generic solution, **When** student provides platform-specific constraints, **Then** student demonstrates "AI as Student" role (teaching AI context)
2. **Given** student lacks domain expertise, **When** Gemini suggests optimization, **Then** student demonstrates "AI as Teacher" role (learning new pattern)
3. **Given** neither student nor AI has perfect solution, **When** they iterate 3+ times, **Then** student demonstrates "AI as Co-Worker" convergence pattern

---

### User Story 3 - Reusable Intelligence Creation (Priority: P3)

**Scenario**: Student repeatedly uses Gemini for research tasks (search documentation → summarize findings → save notes). Recognizes pattern recurs 5+ times. Creates reusable `research-workflow` skill.

**Journey**:
1. Student completes 3 research tasks using Gemini's Google Search
2. Notices repeated pattern: Search → Read results → Summarize → Save notes
3. Realizes: "This is Stage 3—pattern recurs, should encode as reusable intelligence"
4. Creates `research-workflow.md` skill documenting:
   - **Persona**: "Think like a research librarian organizing findings"
   - **Questions**: "What keywords best capture the topic? What sources are authoritative? How do I structure findings?"
   - **Principles**: "Primary sources over secondary. Cite URLs. Timestamp searches."
5. Next research task: References skill, completes workflow 40% faster
6. Shares skill with classmate, who applies it successfully (reusability validated)

**Why this priority**: Stage 3 (Intelligence Design) demonstrates accumulation principle. Less urgent than P1/P2 but critical for long-term capability building.

**Independent Test**: Can be tested by tracking if student creates skill after encountering pattern 2+ times. Delivers intelligence accumulation competence.

**Acceptance Scenarios**:

1. **Given** student encounters pattern 2+ times, **When** student recognizes recurrence, **Then** student creates skill using Persona+Questions+Principles format
2. **Given** student has created skill, **When** student applies skill to new task, **Then** task completion is measurably faster (self-reported 30%+ improvement)
3. **Given** skill is shared with peer, **When** peer applies skill independently, **Then** peer achieves same task outcome (reusability demonstrated)

---

### User Story 4 - Extension Development Capstone (Priority: P4)

**Scenario**: Student completes all lessons and wants to build custom MCP server that connects Gemini CLI to their note-taking app (Notion/Obsidian), enabling AI-assisted research workflows.

**Journey**:
1. Student defines spec: "MCP server that reads/writes notes from Obsidian vault"
2. Reviews existing MCP server examples (GitHub, filesystem servers)
3. Creates simple MCP server following Model Context Protocol standard:
   - List tools: `list_notes`, `create_note`, `search_notes`
   - Implement handlers: Read vault directory, parse markdown, return JSON
4. Connects MCP server to Gemini CLI using `gemini mcp add`
5. Tests: Asks Gemini "What did I research about asyncio yesterday?" → Gemini calls `search_notes` → Returns findings
6. Iterates: Adds tagging, date filters based on usage
7. Documents learning: "I composed installation (Stage 1), Three Roles collaboration (Stage 2), research-workflow skill (Stage 3) into working extension (Stage 4)"

**Why this priority**: Stage 4 (Spec-Driven Integration) capstone demonstrates mastery through creation. Lower priority because it's culmination activity (requires P1-P3 completion).

**Independent Test**: Can be tested by evaluating if student successfully creates functioning MCP server or custom slash command. Delivers creative mastery demonstration.

**Acceptance Scenarios**:

1. **Given** student has specification for extension, **When** student implements MCP server, **Then** extension connects to Gemini CLI and executes basic operation
2. **Given** student has working extension, **When** student uses extension for real task, **Then** student demonstrates composition of Stages 1-3 knowledge
3. **Given** student completes capstone, **When** asked "What did you learn?", **Then** student articulates 4-stage progression and intelligence accumulation principle

---

### Edge Cases

- **No Node.js installed**: Student attempts `npm install -g @google/gemini-cli` without Node.js → Error: `npm command not found` → Lesson provides installation links (nodejs.org) with OS-specific guidance
- **Authentication failure**: Student's Google account blocks OAuth → Error: `Authentication failed` → Lesson explains alternative: Gemini API key setup via Google AI Studio
- **Rate limit hit**: Student on free tier makes 61st request in 1 minute → Error: `Rate limit exceeded (60 req/min)` → Lesson explains free tier limits, suggests pacing or upgrading
- **Context window exceeded**: Student tries loading 2000-page codebase in one prompt → Error: `Context too large` → Lesson demonstrates `/compress` command or chunking strategies
- **MCP server conflicts**: Student adds MCP server with same name as existing → Error: `Server 'filesystem' already exists` → Lesson shows `gemini mcp list` and `gemini mcp remove` workflow
- **Slash command syntax errors**: Student creates custom command with malformed TOML → Error: `Parse error in command.toml` → Lesson demonstrates TOML validation tools
- **Platform-specific path issues**: Windows student uses Unix-style paths in config → Error: `Path not found: /home/user/...` → Lesson detects OS, provides platform-appropriate examples
- **Outdated CLI version**: Student has v0.3.0, features require v0.4.0+ → Warning: `Feature requires Gemini CLI v0.4.0+` → Lesson explains `npm update -g @google/gemini-cli`

---

## Requirements

### Functional Requirements

#### Stage 1: Manual Foundation (Installation & Basic Usage)

- **FR-001**: Student MUST install Gemini CLI using at least one method (npm global, npx, or version-specific) successfully
- **FR-002**: Student MUST authenticate via Google OAuth and access free tier (60 req/min, 1000 req/day)
- **FR-003**: Student MUST navigate Gemini CLI interface (input box, status bar, context indicators)
- **FR-004**: Student MUST execute basic slash commands (`/help`, `/tools`, `/stats`, `/quit`) and interpret outputs
- **FR-005**: Student MUST use shell mode (`!command`) to run terminal commands from within Gemini CLI

#### Stage 1: Tool Selection Framework

- **FR-006**: Student MUST identify six objective dimensions for AI tool evaluation (licensing, cost, context window, extensibility, interface, support)
- **FR-007**: Student MUST compare Claude Code vs Gemini CLI across all six dimensions using factual data (not opinions)
- **FR-008**: Student MUST apply decision framework to 3+ real scenarios, selecting appropriate tool with justification

#### Stage 2: Error Analysis Pedagogy (AI Collaboration)

- **FR-009**: Student MUST attempt task with suboptimal tool first, recognize limitation, and switch to better tool
- **FR-010**: Student MUST articulate WHY first tool failed and WHY second tool succeeded (causal reasoning, not guessing)
- **FR-011**: Student MUST document tool selection pattern discovered through error analysis

#### Stage 2: Three Roles Framework with Gemini

- **FR-012**: Student MUST demonstrate "AI as Teacher" role: Gemini suggests pattern/optimization student didn't know
- **FR-013**: Student MUST demonstrate "AI as Student" role: Student corrects Gemini's generic output with specific constraints
- **FR-014**: Student MUST demonstrate "AI as Co-Worker" role: Iterative refinement where both contribute to convergence
- **FR-015**: Student MUST complete at least one collaborative workflow exhibiting all three roles in sequence

#### Stage 2: Built-in Tools Mastery

- **FR-016**: Student MUST use Gemini's Google Search tool to fetch current web data (differentiator from Claude)
- **FR-017**: Student MUST use file operations tool to read/write project files within Gemini session
- **FR-018**: Student MUST use shell integration to execute commands and capture outputs
- **FR-019**: Student MUST use web fetch tool to retrieve content from URLs

#### Stage 2: Context Management

- **FR-020**: Student MUST use `/clear` for hard context reset (when switching topics completely)
- **FR-021**: Student MUST use `/compress` for intelligent summarization (when approaching context limits)
- **FR-022**: Student MUST use `/chat save/resume` for conversation branching (parallel experiments)
- **FR-023**: Student MUST create GEMINI.md file for persistent project context across sessions

#### Stage 2: Configuration Understanding

- **FR-024**: Student MUST understand 7-level configuration hierarchy (CLI flags → project settings → global settings → system defaults)
- **FR-025**: Student MUST configure project-specific settings using `.env` file or project config
- **FR-026**: Student MUST apply security best practices (API keys in env vars, not hardcoded)

#### Stage 3: Reusable Intelligence Design

- **FR-027**: Student MUST identify workflow pattern recurring 2+ times across lessons
- **FR-028**: Student MUST create reusable skill using Persona+Questions+Principles format
- **FR-029**: Student MUST document skill in markdown with usage examples
- **FR-030**: Student MUST apply created skill to new scenario, measuring efficiency gain (self-reported 30%+ improvement target)

#### Stage 4: Extension Development Capstone

- **FR-031**: Student MUST write specification for custom MCP server OR custom slash command
- **FR-032**: Student MUST implement extension following Gemini CLI extension standards
- **FR-033**: Student MUST test extension integration using `gemini mcp add` or slash command invocation
- **FR-034**: Student MUST demonstrate extension solves real personal workflow need (not toy example)
- **FR-035**: Student MUST document how capstone composes Stages 1-3 accumulated intelligence

##### Capstone Scope Calibration (A2 Tier)

To ensure capstone complexity matches A2 beginner capacity and time constraints (SC-007: 45 min for extensions):

**MCP Server Minimum Viable Scope**:
- Implement **at least 2 tools** (e.g., `list_notes`, `create_note`, `search_notes`)
- **Basic error handling**: Catch exceptions, return error messages (no production-grade logging)
- **Local file access only**: Read/write files from filesystem (NO authentication required)
- **No external API integrations**: Gemini CLI interaction only (no database, no web services)
- **Time budget**: 45-60 minutes implementation (per SC-007)
- **Example tools**: Note-taking app reader (Obsidian/Notion vault), project file indexer, research summary organizer

**Slash Command Minimum Viable Scope**:
- Implement **1 custom command** with meaningful automation (not "hello world")
- Use **at least one injection pattern**: `{{args}}` (user input), `!{shell}` (shell command), OR `@{file}` (file path)
- **TOML config**: 3-5 fields maximum (name, description, prompt, optional parameters)
- **Time budget**: 30-45 minutes implementation
- **Example commands**: `/research` (search web + save notes), `/summarize` (read file + generate summary), `/scaffold` (create project structure)

**Complexity Guardrails (What NOT to Include)**:
- ❌ **No database integration** (PostgreSQL, MongoDB, Redis)
- ❌ **No web server/API hosting** (Express, FastAPI endpoints)
- ❌ **No third-party authentication** (OAuth providers, API key management systems)
- ❌ **No testing frameworks** (Jest, pytest — manual testing only for A2 tier)
- ❌ **No production deployment** (Docker, cloud hosting — local execution only)

**Rationale**: Focus on **composition of Stages 1-3 knowledge**, not novel research. Capstone demonstrates students can apply accumulated intelligence (installation, Three Roles collaboration, reusable skills) to build working extension within time/complexity constraints.

**Example MCP Server (Good Scope)**:
```json
// Obsidian Vault Reader MCP Server
{
  "name": "obsidian-vault",
  "version": "1.0.0",
  "tools": [
    {
      "name": "list_notes",
      "description": "List all markdown files in vault directory",
      "parameters": {
        "vault_path": {
          "type": "string",
          "description": "Path to Obsidian vault"
        }
      }
    },
    {
      "name": "search_notes",
      "description": "Search notes by keyword",
      "parameters": {
        "vault_path": {"type": "string"},
        "keyword": {"type": "string"}
      }
    }
  ]
}
```

**Example Slash Command (Good Scope)**:
```toml
# /research command - Gemini searches web and saves to file
[command]
name = "research"
description = "Search topic and save findings to notes"
prompt = """
Search for {{args}} using Google Search tool.
Summarize top 3 findings with:
- Key point (1 sentence)
- Source URL
- Relevance to query

Save summary to: @{research-notes.md}
Append timestamp and search query.
"""
```

#### Metadata & Pedagogical Requirements

- **FR-036**: Every lesson MUST include CEFR level mapping (A2 for foundational, B1 for integration)
- **FR-037**: Every learning objective MUST include Bloom's taxonomy level (Remember, Understand, Apply, Analyze)
- **FR-038**: Every learning objective MUST include DigComp 2.2 competency area mapping (Information Literacy, Problem Solving, etc.)
- **FR-039**: Every lesson MUST explicitly tag Stage (1, 2, 3, or 4) in metadata
- **FR-040**: Every code example MUST have terminal execution log attached (factual accuracy requirement)

### Key Entities

- **Gemini CLI Session**: Interactive terminal session with Gemini model; attributes: context state, conversation history, active MCP servers, loaded config
- **Tool**: Built-in capability Gemini can invoke; types: Google Search, file operations, shell execution, web fetch; attributes: name, description, parameters, output format
- **MCP Server**: External service connected via Model Context Protocol; attributes: server name, authentication type, available tools, connection status
- **Slash Command**: User-defined automation script; attributes: command name, TOML config, injection patterns ({{args}}, !{shell}, @{file})
- **Context File**: Persistent memory file (GEMINI.md); attributes: project-specific knowledge, file paths, environment details
- **Configuration**: Hierarchical settings system; levels: CLI flags > project .env > global config > system defaults
- **Skill**: Reusable intelligence pattern; structure: Persona (cognitive stance) + Questions (analysis prompts) + Principles (decision frameworks)
- **Extension**: Custom capability (MCP server or slash command); attributes: spec, implementation, integration method, usage documentation

---

## Success Criteria

### Measurable Outcomes

**Learning Objectives Met:**

- **SC-001**: 85%+ of students correctly select appropriate AI tool (Claude vs Gemini) for 8/10 scenarios using decision framework
- **SC-002**: 90%+ of students articulate WHY tool selection matters (with 2+ specific examples of wrong-tool consequences)
- **SC-003**: 80%+ of students demonstrate all three roles (Teacher/Student/Co-Worker) in Gemini collaboration workflow
- **SC-004**: 75%+ of students create at least one reusable skill (Stage 3) using Persona+Questions+Principles format

**Workflow Competence:**

- **SC-005**: Students install Gemini CLI and complete authentication within 10 minutes (median time)
- **SC-006**: Students execute Google Search query via Gemini and retrieve current results within 2 minutes
- **SC-007**: Students create and apply custom slash command or MCP server (capstone) within 45 minutes
- **SC-008**: Students document tool selection decisions with justification in 100% of error analysis exercises

**Confidence & Self-Efficacy:**

- **SC-009**: 85%+ of students report "I can choose the right AI tool for my task" (post-chapter survey)
- **SC-010**: 80%+ of students successfully recover from tool selection errors using decision framework (self-correction)
- **SC-011**: 70%+ of students create at least one extension (MCP server or slash command) by end of chapter

**Knowledge Retention:**

- **SC-012**: 75%+ of students correctly identify Gemini's unique differentiators (open source, free tier, Google Search, 1M context) on assessment
- **SC-013**: 80%+ of students recall and apply Three Roles framework to new collaboration scenarios
- **SC-014**: 90%+ of students understand error analysis as learning method (not failure) based on reflection prompts

**Pedagogical Quality (Internal Validation):**

- **SC-015**: Chapter passes constitutional validation (Principles 1-7 compliance: error analysis modality, A2 cognitive load, anti-convergence)
- **SC-016**: All Gemini CLI commands execute successfully in sandbox testing (100% accuracy, no hallucinated commands/features)
- **SC-017**: Gemini CLI features verified against Context7 @google/gemini-cli docs and official Google AI Studio documentation
- **SC-018**: Cognitive load per section ≤ 7 concepts (A2 tier compliance verified by content-implementer)
- **SC-019**: Teaching modality differs from Chapter 5 (error analysis vs direct teaching validated by pedagogical-designer)
- **SC-020**: All lessons include complete metadata (CEFR + Bloom's + DigComp 2.2 mappings present in 100% of learning objectives)

---

## Constraints

### Pedagogical Constraints

- **C-001**: Content MUST target A2 beginner audience (Part 2) with heavy scaffolding, max 2 tool options per decision point
- **C-002**: Cognitive load MUST NOT exceed 5-7 concepts per lesson section (constitutional Principle 2)
- **C-003**: Teaching modality MUST use error analysis (try wrong tool → analyze → discover right tool) to vary from Chapter 5's direct teaching
- **C-004**: All tool comparisons MUST be objective (factual dimensions) not subjective (opinions like "Claude is better")
- **C-005**: Stage progression MUST be explicit: 1 (Manual) → 2 (AI Collab + Three Roles) → 3 (Intelligence) → 4 (Extension Capstone)

### Technical Constraints

- **C-006**: All Gemini CLI commands MUST be platform-agnostic (work on Windows, macOS, Linux) OR provide platform-specific alternatives
- **C-007**: All features MUST be free tier accessible (no paid Gemini API or Vertex AI requirements for core learning)
- **C-008**: Gemini CLI version MUST be 0.4.0+ (for MCP integration and extension system support)
- **C-009**: All code examples MUST execute successfully in sandbox testing before publication (constitutional Principle 3)

### Scope Constraints

- **C-010**: Chapter focuses on Gemini CLI tool mastery; Node.js/npm fundamentals covered minimally (assume Chapter 12 prerequisite or external tutorial)
- **C-011**: MCP server development deep dive EXCLUDED (awareness-level only; implementation details deferred to Part 6)
- **C-012**: Advanced Gemini API features EXCLUDED: Fine-tuning, embeddings, multimodal inputs (image/video) beyond text
- **C-013**: Gemini model internals EXCLUDED: Transformer architecture, training methodology, benchmark comparisons

### Validation Constraints

- **C-014**: All Gemini CLI commands MUST have terminal execution logs attached to lessons (Principle 3: factual accuracy)
- **C-015**: Gemini features MUST cite Context7 @google/gemini-cli docs OR official Google AI Studio documentation as authoritative sources
- **C-016**: No hallucinated CLI commands, slash commands, or MCP features permitted (validation-auditor + factual-verifier checks)
- **C-017**: Metadata completeness MUST be 100% (every learning objective has CEFR + Bloom's + DigComp 2.2 mappings)

---

## Assumptions

### Student Prerequisites

- **A-001**: Students completed Chapter 5 (Claude Code) and understand AI collaboration basics (prompting, iteration)
- **A-002**: Students have text editor installed (VS Code, Cursor, Zed, or similar) for editing config files
- **A-003**: Students have stable internet connection for Gemini CLI authentication and Google Search operations
- **A-004**: Students have Google account (Gmail, YouTube, or any Google service) for OAuth authentication

### Technical Environment

- **A-005**: Students use Node.js 20+ and npm 9+ (required for Gemini CLI installation)
- **A-006**: Students have terminal/command-line access (Windows PowerShell, macOS Terminal, Linux shell)
- **A-007**: Students' systems allow OAuth browser-based authentication (not behind restrictive firewalls)
- **A-008**: Students have write permissions in home directory for global npm packages

### Pedagogical Assumptions

- **A-009**: Error analysis modality (productive failure → discovery) suits tool selection learning better than lecture-style for this audience
- **A-010**: A2 tier students can handle 2 tool options (Claude vs Gemini) without decision paralysis (more than 2 would exceed cognitive load)
- **A-011**: Students benefit from seeing tool limitations (Claude lacks real-time search) to understand differentiation, not just feature lists
- **A-012**: Three Roles framework (AI as Teacher/Student/Co-Worker) applies to Gemini CLI workflows as effectively as Claude Code
- **A-013**: Extension development capstone (MCP server or slash command) provides sufficient complexity for A2 tier mastery demonstration

### Reasonable Defaults

- **A-014**: Default installation method is npm global (`npm install -g @google/gemini-cli`) for permanent access
- **A-015**: Default authentication is Google OAuth free tier (60 req/min, 1000 req/day) unless student has API key
- **A-016**: Default context management is GEMINI.md file in project root (standard convention)
- **A-017**: Default tool mode is `auto` (Gemini decides when to invoke tools) unless student needs `manual` for learning

---

## Non-Goals

### Out of Scope for This Chapter

**Advanced Gemini API Features:**
- Gemini API key management beyond basic setup (enterprise key rotation, quota management)
- Fine-tuning Gemini models (custom model training)
- Embeddings API (vector representations for RAG systems)
- Multimodal inputs beyond text (image analysis, video understanding, audio transcription)
- Batch processing APIs (async job submission for large workloads)

**Deep MCP Protocol:**
- MCP protocol specification deep dive (transport layers, JSON-RPC details)
- Building production-grade MCP servers (error handling, authentication, rate limiting)
- MCP server testing frameworks (unit tests, integration tests)
- Publishing MCP servers to registries (npm, community catalogs)

**Gemini Model Internals:**
- Transformer architecture explanation (attention mechanisms, embeddings)
- Training methodology (datasets, compute requirements, optimization)
- Benchmark comparisons (MMLU, HumanEval scores vs GPT-4, Claude)
- Safety filtering mechanisms (content policies, guardrails)

**Enterprise Workflows:**
- Vertex AI integration (Google Cloud Project setup, billing)
- Team collaboration patterns (shared MCP servers, context files)
- Organizational policies (approved tool allowlists, compliance)
- Cost optimization strategies (quota management, model selection)

**Alternative AI Tools:**
- Cursor IDE integration (AI-powered editor workflows)
- GitHub Copilot comparison (code completion vs chat)
- Aider CLI workflows (git-aware AI coding assistant)
- Other CLI tools (LLM CLI, OpenAI CLI, etc.)

**Why Excluded**: These topics exceed A2 cognitive capacity, require prerequisite knowledge from later chapters (Part 6: AI Native Development for MCP deep dive, Part 7: Cloud Native for Vertex AI), or distract from core objective (tool selection judgment, not exhaustive Gemini coverage).

**Where to Find These Topics**: MCP development (Part 6, Chapter 39), Multimodal AI (Part 6, Chapters 34-37), Vertex AI (Part 8, Chapters 62-65), Enterprise patterns (Part 11).

---

## Open Questions & Clarifications

### [RESOLVED: User answered during Phase 0]

All critical clarifications were addressed during Phase 0 constitutional reasoning:

- **Q1 (Teaching Modality)**: Error Analysis selected (try wrong tool → analyze → discover right tool)
- **Q2 (Capstone Scope)**: Extension Development selected (MCP server or custom slash command)
- **Q3 (Metadata Granularity)**: CEFR + Bloom's + DigComp 2.2 selected (comprehensive skill tracking)

**No additional clarifications needed for specification phase.**

---

## Dependencies

### Prerequisite Chapters
- **Chapter 5** (Claude Code Features and Workflows): AI collaboration patterns established, Three Roles framework introduced
- **Chapter 12** (Python UV Package Manager) OR external Node.js tutorial: npm/Node.js basics assumed

### Prerequisite Skills
- Terminal/CLI navigation (opening terminal, running commands)
- Text editor usage (creating/editing config files)
- Natural language prompting for AI assistance (from Chapter 5)
- Google account access (Gmail or any Google service)

### Tool Dependencies
- Node.js 20+ and npm 9+ installed on student's system
- Google account for OAuth authentication
- Text editor (VS Code, Cursor, Zed, Sublime, etc.)
- Internet connection for CLI installation and Gemini API access

### Downstream Dependencies (Knowledge Transfer Mapping)

**Chapter 7** (Bash Essentials): Shell integration concepts reinforced with Bash knowledge

- **Transfer**: FR-005 (shell mode `!command`), FR-018 (shell integration tool) provide foundation for Bash scripting workflows
- **Composition**: Students compose Gemini CLI + Bash for automated research workflows
  - Example: Gemini searches web → Bash script processes results → saves to structured files
  - Example: Bash script triggers Gemini analysis → exports findings to CSV/JSON
- **Reinforcement**: Chapter 7 callbacks to Gemini's shell execution as comparison point
  - "Gemini's `!command` mode vs native Bash terminal: when to use each"
  - "Shell integration tool (FR-018) enables Gemini to execute Bash scripts discussed in Chapter 7"

**Chapter 8** (Git and GitHub): Version control for managing extensions and projects

- **Transfer**: FR-031 to FR-035 (extension development) requires version control for MCP server/slash command code
- **Composition**: Students version custom extensions, collaborate on shared MCP servers
  - Capstone MCP server code tracked in Git repository
  - Custom slash commands shared via GitHub (team collaboration pattern)
  - Version history tracks extension iterations during Three Roles refinement (FR-014)
- **Reinforcement**: Chapter 8 uses Gemini CLI for Git workflow automation
  - Gemini generates commit messages based on file diffs
  - Gemini analyzes PR changes and suggests improvements
  - Practical application: Students apply Chapter 6 skills (FR-016 Google Search, FR-017 file ops) to Git scenarios

**Part 6** (AI Native Development): MCP server development deep dive builds on awareness from this chapter

- **Transfer**: FR-031 (MCP server specification) and User Story 4 provide conceptual foundation for production MCP development
- **Gap Closure**: Chapter 6 teaches awareness-level MCP (2 tools, basic error handling, local file access); Part 6 teaches production-grade MCP
  - Chapter 6: Minimum viable MCP (Capstone Scope Calibration section)
  - Part 6 Chapter 39: Production MCP (authentication, error handling frameworks, testing, publishing to npm)
- **Prerequisite Validation**: Part 6 assumes students completed Chapter 6 capstone (basic MCP server creation experience)
  - Students who built Obsidian vault reader (Chapter 6) extend it to production in Part 6 (add OAuth, unit tests, error logging)
- **Composition**: Chapter 6 MCP servers become building blocks for Part 6 multi-agent systems
  - Example: Research workflow MCP (Chapter 6) + database MCP (Part 6) + code generation agent (Part 6) → Complete AI-native application

**Part 7** (Cloud Native Development): Vertex AI integration extends Gemini CLI for production

- **Transfer**: FR-002 (Google OAuth authentication), free tier understanding (60 req/min, 1000 req/day) prepare for Vertex AI billing/quota management
- **Composition**: Gemini CLI skills + Vertex AI enable enterprise-scale AI workflows
  - Chapter 6: Free tier Gemini CLI for individual development
  - Part 7: Vertex AI for production deployments with SLAs, quotas, cost optimization
- **Differentiation Awareness**: Students learn when to use Gemini CLI vs Vertex AI
  - Gemini CLI: Prototyping, personal projects, learning (free tier sufficient)
  - Vertex AI: Production applications, enterprise features, team deployments
- **Reinforcement**: Part 7 references Chapter 6 configuration knowledge (FR-024 to FR-026)
  - 7-level configuration hierarchy (Chapter 6) applies to Vertex AI project settings
  - Security best practices (API keys in env vars, FR-026) extend to Vertex AI service accounts

---

## Success Indicators (How We Know We're Done)

### Specification Complete When:
- [x] All user scenarios prioritized and testable independently (P1-P4 defined)
- [x] Functional requirements mapped to 4-stage progression (FR-001 through FR-040)
- [x] Success criteria measurable and technology-agnostic (SC-001 through SC-020)
- [x] Constraints and non-goals explicit (pedagogical, technical, scope, validation)
- [x] Constitutional intelligence object documented (Phase 0 reasoning captured)
- [x] User clarifications captured (Q1: Error Analysis, Q2: Extension Dev, Q3: Full Metadata)
- [x] Edge cases identified (8 common failure scenarios with guidance)
- [x] Dependencies mapped (prerequisites, tools, downstream chapters with knowledge transfer details)
- [x] Capstone scope calibration added (A2 tier complexity guardrails, time budgets, examples)
- [x] Downstream impact detail expanded (Transfer → Composition → Reinforcement mapping for Chapters 7, 8, Parts 6, 7)

### Planning Ready When:
- [x] Specification approved by human reviewer
- [x] spec-architect validation passed (checklist generated, critical issues = 0)
- [x] Lesson structure validated and restructured based on concept density analysis (9 lessons total)
- [x] Pedagogical arc confirmed (Foundation → Application → Integration → Organization → Mastery)

### Implementation Ready When:
- [x] Plan approved with lesson-by-lesson breakdown
- [x] Research complete (Context7 @google/gemini-cli docs verified, all commands tested)
- [x] Content audit completed for 9 lessons (Lessons 1-8 complete, Lesson 9 pending)
- [x] Tasks checklist generated for metadata addition, content refactoring, capstone creation

### Validation Ready When:
- [x] Lessons 1-8 implemented with demonstration pedagogy and beginner-appropriate examples
- [ ] Lesson 9 (Capstone) implemented with spec-driven approach for A2 beginners
- [x] All Gemini CLI commands tested with attached terminal logs
- [x] Gemini features cited from Context7 + official Google AI Studio docs
- [x] Metadata complete for Lessons 1-8 (CEFR + Bloom's + DigComp 2.2 present in 100% of learning objectives)
- [ ] Metadata complete for Lesson 9
- [x] Constitutional compliance verified (Principles 1-7)

### Publication Ready When:
- [ ] validation-auditor passes pedagogical review (4-stage progression clear, all 9 lessons validated)
- [ ] factual-verifier confirms no hallucinated commands/features (100% citation accuracy)
- [ ] Success criteria met: 85%+ students correctly select tools for 8/10 scenarios (field test or assessment)
- [ ] Meta-learning captured in PHR for future chapter redesigns

---

**Status**: 8/9 Lessons complete. Next: Create Lesson 9 (Capstone Project) for A2 pre-programming beginners.
