# Feature Specification: Chapter 5 Claude Code Rework

**Feature Branch**: `018-chapter-5-claude-code-rework`
**Created**: 2025-01-13
**Status**: Draft
**Input**: User description: "Chapter 5 rework - How It All Started: The Claude Code Phenomenon. We have to rework on chapter 5 claude code - it is missing a lot of work and many things are hallucinated or not explained. Now we have a sandbox so you can setup claude there and test everything from installation to helloworld etc. We can use context7 mcp tools and web search to fully understand claude code and rework on this chapter to make it well aligned with our book vision, preface and part1 students have just completed before this chapter"

---

## 🎯 Chapter Positioning: Beyond Official Documentation

**After completing this chapter, students will have the most powerful personalized AI companion for AI-Driven Development.**

### Why This Chapter Surpasses Official Anthropic Resources

This chapter is **MORE COMPREHENSIVE** than all official Anthropic Claude Code courses and documentation because it uniquely combines:

1. **Complete Learning Path** (Installation → Authentication → Personalization → AIDD Mastery)
   - Official docs: Focus on installation and basic usage
   - **This chapter**: End-to-end journey from zero to fully personalized AI companion

2. **Unified Extensibility Architecture** (Skills + Plugins + MCP in ONE cohesive lesson)
   - Official docs: Scattered across multiple pages (agent skills blog, plugins page, MCP protocol)
   - **This chapter**: Lesson 4 integrates all three with clear relationship hierarchy and visual diagrams

3. **Hands-On Personalization** (Real plugin installation with verification)
   - Official docs: Conceptual explanations of plugins
   - **This chapter**: Students install actual plugin (`/plugin marketplace add` → `/plugin install feature-dev`), restart Claude Code, verify activation with `/help`

4. **AIDD Mindset Integration** (Using AI companion for development, not just coding assistance)
   - Official docs: Tool-centric (what Claude Code can do)
   - **This chapter**: Workflow-centric (how to use Claude Code for AI-Driven Development)

5. **Progressive Disclosure Deep Dive** (3-level architecture explained with PDF skill example)
   - Official docs: Brief mention in engineering blog
   - **This chapter**: Complete explanation with SKILL.md structure, YAML frontmatter, bundled files, and concrete PDF skill use case

6. **Community Ecosystem Discovery** (3+ real marketplace examples)
   - Official docs: Generic marketplace references
   - **This chapter**: anthropics/claude-code (feature-dev, security-guidance), Dan Ávila marketplace (DevOps, docs, testing), Seth Hobson's 80+ sub-agents

7. **Constitutional Alignment** (Graduated Teaching, Three Roles, Cognitive Load Management)
   - Official docs: Linear tutorials
   - **This chapter**: Pedagogically designed for A2 cognitive load (7 concepts max), Tier 1-3 progression, co-learning partnership

8. **Sandbox-Validated Content** (Every command tested before publication)
   - Official docs: Some examples may be generic
   - **This chapter**: All installation commands, workflows, plugin installation tested on Windows/macOS/Linux

9. **MCP Configuration Mastery** (JSON structure with GitHub + Filesystem examples)
   - Official docs: MCP protocol specification
   - **This chapter**: Practical configuration with concrete examples, @-mentions activation, OAuth setup

10. **Relationship Clarity** (Plugins contain Skills/Commands/Agents/Hooks/MCP)
    - Official docs: Components mentioned separately
    - **This chapter**: Explicit visual hierarchy diagram showing Plugins as containers

### Outcome Statement

**By the end of this chapter, students will have:**
- ✅ Claude Code installed, authenticated, and working on their machine
- ✅ Understanding of agent skills with progressive disclosure pattern
- ✅ At least ONE community plugin installed and verified
- ✅ Knowledge of MCP configuration for external tool integration
- ✅ CLAUDE.md context file created for their project
- ✅ Awareness of permissions, hooks, and settings hierarchy
- ✅ **A fully personalized AI companion ready for AI-Driven Development**

**This is not just learning Claude Code—this is acquiring your AI development partner for the entire course.**

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Understand Claude Code's Paradigm Shift (Priority: P1)

**As a** beginner who has completed Part 1 (Chapters 1-4),
**I want to** understand what Claude Code is and how it differs from chat-based AI tools,
**So that** I can recognize when to use Claude Code versus when simpler tools suffice.

**Why this priority**: This is the foundational mental model required before any hands-on work. Without understanding the agentic vs passive paradigm, students won't know when Claude Code adds value.

**Independent Test**: Can be fully tested by asking students "When would you use Claude Code versus ChatGPT web interface?" and evaluating if they can articulate the context-aware, file-system-integrated advantage.

**Acceptance Scenarios**:

1. **Given** a student has read the origin story lesson, **When** presented with a scenario "I need to understand what a function does", **Then** student can explain whether this needs Claude Code or a simpler tool
2. **Given** a student understands the paradigm shift, **When** asked to compare passive AI vs agentic AI, **Then** student can list 3 key differences (context awareness, direct action, workflow integration)
3. **Given** a student reviews the comparison table, **When** deciding on tool selection, **Then** student chooses appropriately based on task complexity and context needs

---

### User Story 2 - Install and Authenticate Claude Code Successfully (Priority: P1)

**As a** student on Windows/macOS/Linux,
**I want to** install Claude Code and authenticate with my Claude.ai or Console account in under 5 minutes,
**So that** I have a working development partner ready for future chapters.

**Why this priority**: Installation is a blocker for all subsequent work. Must work on first attempt with clear troubleshooting guidance.

**Independent Test**: Student runs installation commands, authenticates successfully, and runs `claude "Hello! Can you confirm Claude Code is working?"` to get a response. Testable on any platform independently.

**Acceptance Scenarios**:

1. **Given** a student on macOS/Linux, **When** they run `curl -fsSL https://claude.ai/install.sh | bash`, **Then** Claude Code installs successfully within 30 seconds
2. **Given** a student on Windows, **When** they run PowerShell install script, **Then** Claude Code installs successfully
3. **Given** Claude Code is installed, **When** student runs `claude` for the first time, **Then** interactive authentication prompt appears
4. **Given** student has Claude.ai account, **When** they complete OAuth flow, **Then** authentication succeeds and test command works
5. **Given** student encounters error (PATH not set, Node.js missing), **When** they consult troubleshooting section, **Then** they find the exact error and solution

---

### User Story 3 - Understand and Use Subagents (Priority: P2)

**As a** student learning AI orchestration,
**I want to** understand what subagents are, when Claude Code uses them automatically, and how they prevent context drift,
**So that** I can recognize when specialized agents are working and understand the "team of agents" paradigm.

**Why this priority**: Subagents are a key differentiator (Explore agent, Plan mode) and introduce the orchestration concept that becomes central in Part 5 (Spec-Driven Development) and Part 6 (AI-Native Development).

**Independent Test**: Student can explain "What is the Explore subagent and when does Claude Code invoke it automatically?" and demonstrate running a codebase exploration query.

**Acceptance Scenarios**:

1. **Given** a student understands the subagent concept, **When** asked "Why use subagents instead of one conversation?", **Then** student explains context drift prevention and specialization
2. **Given** Claude Code automatically invokes Explore agent, **When** student asks "Where are errors handled in this codebase?", **Then** student recognizes the Explore agent is working (fast, efficient search)
3. **Given** a student learns about Plan mode, **When** working on a multi-step task, **Then** student understands Claude is breaking down the problem into phases

---

### User Story 4 - Understand Skills, Plugins, and MCP Integration (Priority: P3)

**As a** student learning to use and personalize my AI companion for AIDD,
**I want to** understand the complete extensibility architecture: agent skills (autonomous capabilities with progressive disclosure), plugins (bundled customizations), and MCP (external tool integration),
**So that** I can personalize Claude Code for my workflow, install community plugins, and prepare for building custom integrations in advanced chapters.

**Why this priority**: This is the COMPLETE extensibility foundation for AI companion personalization. Missing any piece creates cognitive overload later when students encounter plugins, skills, or MCP in workflows. Essential for AIDD (AI-Driven Development) where the AI companion must be customized to the developer's needs.

**Independent Test**: Student can: (1) explain agent skills progressive disclosure pattern with PDF skill example, (2) install a plugin via `/plugin` command, (3) explain MCP configuration with 2 external integration examples, (4) articulate the relationship hierarchy: Plugins contain Skills/Commands/Agents/Hooks/MCP.

**Acceptance Scenarios**:

1. **Given** a student learns about agent skills, **When** asked "What is progressive disclosure in skills?", **Then** student explains 3-level architecture: Level 1 (metadata in system prompt) → Level 2 (full SKILL.md when relevant) → Level 3 (referenced files only when needed)
2. **Given** a student sees PDF skill example, **When** reviewing the use case, **Then** student understands skills enable code execution (Python scripts) for deterministic operations like form field extraction
3. **Given** a student learns about SKILL.md structure, **When** asked "What's in a SKILL.md file?", **Then** student explains: YAML frontmatter (name, description) + markdown content + optional referenced files
4. **Given** a student learns about plugins, **When** asked "What does a plugin bundle?", **Then** student lists: commands + agents + skills + hooks + MCP configurations + .claude-plugin/plugin.json manifest
5. **Given** a student explores `/plugin` command, **When** running `/plugin` in Claude Code, **Then** student can browse marketplace, see available plugins, and understand installation syntax
6. **Given** a student installs example plugin, **When** running `/plugin marketplace add anthropics/claude-code` then `/plugin install feature-dev`, **Then** student successfully installs plugin and understands it requires restart
7. **Given** a student learns about MCP, **When** shown MCP configuration JSON, **Then** student understands: mcpServers object → server name → command/args/oauth structure
8. **Given** a student sees MCP examples, **When** reviewing GitHub and Filesystem MCP servers, **Then** student understands MCP enables @-mentions of external tools (@github, @filesystem)
9. **Given** a student understands hierarchy, **When** asked "How do Skills, Plugins, and MCP relate?", **Then** student explains: Plugins are containers that bundle Skills (autonomous), Commands (slash commands), Agents (subagents), Hooks (automation), and MCP configs (external integrations)
10. **Given** a student explores community plugins, **When** browsing marketplace examples (DevOps, security-guidance, pr-review-toolkit), **Then** student recognizes plugins solve specific workflow needs

---

### User Story 5 - Practice with "Hello World" and Basic Workflows (Priority: P2)

**As a** student with Claude Code installed,
**I want to** practice basic workflows (create file, run command, ask for explanation) using 3-4 focused prompts,
**So that** I build confidence and muscle memory before complex projects.

**Why this priority**: Hands-on practice solidifies understanding. Using Chapter 1's "Try with AI" format (clean, focused prompts) instead of verbose explanations.

**Independent Test**: Student completes all 3-4 prompts successfully, observing Claude Code's responses and understanding the workflow.

**Acceptance Scenarios**:

1. **Given** Claude Code is authenticated, **When** student runs Prompt 1 (create hello.py file), **Then** Claude Code creates the file with correct content
2. **Given** hello.py exists, **When** student runs Prompt 2 (run the file), **Then** Claude Code executes `python hello.py` and shows output
3. **Given** student encounters an error, **When** they ask Claude Code "What does this error mean?", **Then** Claude Code explains the error and suggests a fix
4. **Given** student has practiced, **When** asked to summarize the workflow, **Then** student can describe: specify intent → Claude acts → verify output

---

### User Story 6 - Create and Use CLAUDE.md Context Files (Priority: P1)

**As a** student starting projects with Claude Code,
**I want to** understand what CLAUDE.md files are and create one for my project with AI assistance,
**So that** Claude Code has persistent context about my project and I don't need to repeat explanations in every session.

**Why this priority**: CLAUDE.md is foundational for effective Claude Code usage (mentioned in Best Practices, Podcast, and Context7 docs). Without it, students repeat context in every session. This is Day 1 essential, not advanced customization.

**Independent Test**: Student can explain "What is CLAUDE.md and when would you create one?" and demonstrates creating a basic CLAUDE.md by asking Claude Code "Help me create a CLAUDE.md for this project" and understanding the generated content.

**Acceptance Scenarios**:

1. **Given** a student understands CLAUDE.md concept, **When** asked "What's the purpose of CLAUDE.md?", **Then** student explains it provides persistent project context (coding style, conventions, project structure)
2. **Given** a student has a simple Python project, **When** they ask Claude Code "Help me create a CLAUDE.md for this project", **Then** Claude Code generates appropriate context file with project details
3. **Given** CLAUDE.md exists in project root, **When** student starts new Claude Code session, **Then** Claude automatically loads context without student repeating information
4. **Given** student sees CLAUDE.md example, **When** reviewing the content, **Then** student understands typical sections (project overview, coding conventions, useful commands, project structure)

---

### User Story 7 - Understand Permission Management (Priority: P2)

**As a** student using Claude Code,
**I want to** understand why Claude Code asks for permission before file edits and bash commands,
**So that** I work safely and know how to check/manage permissions when needed.

**Why this priority**: Students will encounter permission prompts in first session. Understanding the permission system prevents confusion and teaches safety-first AI collaboration. Not about configuring permissions yet (that's Part 5), just understanding the concept.

**Independent Test**: Student can explain "Why does Claude Code ask permission before editing files?" and demonstrates using `/permissions` to check current settings.

**Acceptance Scenarios**:

1. **Given** Claude Code proposes a file edit, **When** permission prompt appears, **Then** student understands this is for safety and can choose "approve", "deny", or "edit"
2. **Given** student wants to see current permissions, **When** they run `/permissions` command, **Then** they see list of allowed/denied operations
3. **Given** student learns about permission modes, **When** asked to explain the difference, **Then** student can describe "default" (asks every time), "acceptEdits" (auto-approves file changes), and "plan" (for complex multi-step tasks)
4. **Given** student encounters "permission denied" for bash command, **When** reviewing the prompt, **Then** student understands this is safety feature (not a bug)

---

### User Story 8 - Understand Hooks and Extensibility (Priority: P3)

**As a** student exploring Claude Code capabilities,
**I want to** understand that Claude Code has hooks (automated actions on events) and is extensible,
**So that** I'm aware of customization possibilities for future advanced work.

**Why this priority**: Hooks and plugins are mentioned throughout documentation. Students should know they exist conceptually without building them yet (building hooks/plugins is Part 5/6 content). This prevents "wait, what's a hook?" confusion when they see references.

**Independent Test**: Student can explain "What are hooks in Claude Code?" and give 1-2 examples of when hooks trigger.

**Acceptance Scenarios**:

1. **Given** student reads about hooks concept, **When** asked "What are hooks?", **Then** student explains they're automated commands that run on events (like PostToolUse, SessionStart)
2. **Given** student sees hook example (PostToolUse running linter), **When** reviewing the scenario, **Then** student understands: "After Claude edits a file, automatically run formatter"
3. **Given** student learns hooks exist, **When** asked if they'll build hooks now, **Then** student understands: "Not yet—this is advanced customization for later chapters (Part 5/6)"
4. **Given** student explores Claude Code docs, **When** they encounter hook references, **Then** they have context for what hooks are

---

### User Story 9 - Understand Settings Hierarchy (Priority: P3)

**As a** student learning Claude Code configuration,
**I want to** understand that settings exist at user, project, and local levels,
**So that** I'm aware of customization options for future team collaboration.

**Why this priority**: Students will see `.claude/settings.json` references in documentation. Understanding the hierarchy conceptually (not configuring yet) prevents confusion. Detailed configuration is Part 5 content (team workflows, SDD practices).

**Independent Test**: Student can explain "Where can Claude Code settings be configured?" and understand precedence order.

**Acceptance Scenarios**:

1. **Given** student learns about settings hierarchy, **When** asked "Where can settings be configured?", **Then** student explains: user level (`~/.claude/settings.json`), project level (`.claude/settings.json`), local level (`.claude/settings.local.json`)
2. **Given** student sees settings precedence explanation, **When** asked "If same setting exists in user and project, which wins?", **Then** student explains: local > project > user (more specific overrides general)
3. **Given** student learns settings exist, **When** asked if they'll configure settings now, **Then** student understands: "Not yet—basic usage works without customization. Will learn team settings in Part 5"
4. **Given** student encounters `.claude/` directory, **When** they see settings.json file, **Then** they understand it's project configuration (not random file to delete)

---

### Edge Cases

- **What happens when** student's Claude.ai subscription reaches rate limit during installation/authentication?
  - Lesson includes fallback: "If you see rate limit error, wait 1 hour or use Claude Console API credits"

- **What happens when** `npm install -g` fails due to permissions on macOS/Linux?
  - Troubleshooting section includes: "Run with `sudo` or configure npm to install globally without sudo"

- **What happens when** student is on a restricted network (firewall blocks `claude.ai`)?
  - Troubleshooting section addresses: "Check with IT if corporate firewall blocks AI services. May need proxy configuration."

- **What happens when** student's Node.js version is outdated (below 18)?
  - Prerequisites clearly state Node.js 18+ required. Troubleshooting includes version check command and upgrade link.

- **What happens when** student runs Claude Code in a Git repository with uncommitted changes?
  - Lesson explains: "Claude Code can see your uncommitted changes—this is intentional for real-time collaboration. Use Git best practices."

- **What happens when** a beginner feels overwhelmed by subagents/skills/MCP in one chapter?
  - Content uses Graduated Teaching (Principle 13): foundational concepts taught directly, complex execution (MCP config) delegated to AI companion, orchestration introduced conceptually

- **What happens when** student creates CLAUDE.md but Claude Code doesn't load it?
  - Troubleshooting: "Ensure CLAUDE.md is in project root directory. Check file name is exactly CLAUDE.md (all caps). Restart Claude Code session to force reload."

- **What happens when** student accidentally denies permission for a legitimate operation?
  - Lesson explains: "You can undo by running the command again. Claude will ask permission again. Or use `/permissions` to review settings."

- **What happens when** student sees hook error messages during session?
  - Lesson clarifies: "Hook errors are usually non-blocking. Claude Code continues working. Advanced hook debugging is Part 5 content."

- **What happens when** student modifies settings.json and breaks configuration?
  - Safety note: "Settings validation happens on load. If Claude Code won't start, rename settings.json temporarily to reset. Keep backups before editing."

## Requirements *(mandatory)*

### Functional Requirements

#### Content Accuracy and Verification
- **FR-001**: All installation methods (curl script, PowerShell, Homebrew, NPM) MUST be tested in sandbox and verified working before publication
- **FR-002**: All technical claims about Claude Code (features, capabilities, workflows) MUST be fact-checked against official documentation (code.claude.com, Context7 library /anthropics/claude-code)
- **FR-003**: No hallucinated features or capabilities MUST appear in the final content (all features verified via official docs or Context7)
- **FR-004**: Installation duration MUST be realistic (2-5 minutes actual time, not inflated to 45+ minutes)

#### Constitutional Alignment
- **FR-005**: Content MUST align with book vision from constitution v3.1.3 (AI-native development, Specs Are the New Syntax)
- **FR-006**: Content MUST follow Principle 13 (Graduated Teaching Pattern): Book teaches foundational concepts → AI companion handles complex execution → AI orchestration introduced
- **FR-007**: Content MUST apply Principle 18 (Three Roles Framework): AI as Teacher (explains concepts), Student (learns preferences), Co-Worker (executes workflows)
- **FR-008**: Content MUST enforce Core Philosophy #1 (AI Development Spectrum): Clearly distinguish Assisted vs Driven vs Native paradigms
- **FR-009**: Chapter MUST respect cognitive load limits for A2 tier (aspiring beginners): max 7 concepts per section, 2 options max where choices are required

#### AI Usage Strategy (Anti-Over-Engineering)
- **FR-010**: Installation commands MUST be documented as direct commands (students run `curl | bash` or `npm install`, not "ask AI to install")
- **FR-011**: Simple operations (run `claude --version`, authenticate) MUST NOT be wrapped in elaborate AI workflows
- **FR-012**: "Try with AI" section MUST use Chapter 1 clean format: `### Prompt N: Title` → code block → `**Expected outcome:**` description (3-4 focused prompts total)
- **FR-013**: Content MUST teach WHEN to use Claude Code strategically (complex workflows, troubleshooting) vs when simpler tools suffice (running direct commands)
- **FR-014**: AI companion role MUST be reserved for: troubleshooting errors, understanding complex concepts (subagents/MCP), exploring codebase patterns

#### Agent Skills, Plugins, and MCP Architecture
- **FR-020**: Agent skills progressive disclosure pattern MUST be explained with 3-level architecture: Level 1 (metadata YAML frontmatter in SKILL.md loaded to system prompt), Level 2 (full SKILL.md content loaded when relevant), Level 3 (referenced files accessed only when needed)
- **FR-021**: SKILL.md structure MUST be demonstrated with concrete example showing: YAML frontmatter (name, description fields) + markdown instructional content + optional bundled files (reference.md, forms.md, Python scripts)
- **FR-022**: PDF skill MUST be used as concrete example: form filling and field extraction using bundled Python script for deterministic operations beyond basic PDF comprehension
- **FR-023**: Plugin architecture MUST be explained: .claude-plugin/plugin.json manifest + bundled components (commands/, agents/, skills/, hooks/, .mcp.json)
- **FR-024**: Plugin installation workflow MUST be demonstrated: `/plugin` command → browse marketplace → `/plugin marketplace add <url>` → `/plugin install <name>` → restart Claude Code
- **FR-025**: Community plugin ecosystem MUST be introduced with 3+ real examples: anthropics/claude-code (feature-dev, code-review, security-guidance), Dan Ávila marketplace (DevOps, documentation, testing), Seth Hobson's 80+ specialized sub-agents
- **FR-026**: MCP (Model Context Protocol) MUST be explained as external tool integration standard with JSON configuration structure: mcpServers object → server name → command/args/oauth
- **FR-027**: MCP examples MUST include: GitHub MCP server (npx @anthropic-ai/mcp-server-github with OAuth), Filesystem MCP server (npx @modelcontextprotocol/server-filesystem with path), demonstrating @-mentions for activation (@github, @filesystem)
- **FR-028**: Relationship hierarchy MUST be crystal clear with visual diagram: Plugins (containers) → contain → [Skills (autonomous capabilities) + Commands (slash commands) + Agents (subagents) + Hooks (automation) + MCP configs (external integrations)]
- **FR-029**: Personalization for AIDD MUST be emphasized: plugins enable workflow customization (team standards enforcement, framework-specific shortcuts, internal tool connections, debugging setups, deployment workflows)

#### Pedagogical Structure
- **FR-030**: Chapter MUST contain 9 lessons as defined in user stories: (1) Origin story and paradigm shift, (2) Installation and authentication, (3) Subagents, (4) Skills, Plugins, and MCP, (5) Hello World practice, (6) CLAUDE.md context files, (7) Permission management, (8) Hooks and extensibility, (9) Settings hierarchy
- **FR-031**: Each lesson MUST include: learning objectives, estimated duration (realistic), key concepts, hands-on practice (where applicable), and summary
- **FR-032**: Chapter README MUST provide: What You'll Learn (9-10 bullet points), Chapter Structure (9 lessons overview), Prerequisites (Chapters 1-4 completed), Duration (total realistic time 60-80 minutes)
- **FR-033**: Troubleshooting section MUST address common errors: permissions issues, PATH not set, Node.js version, network/firewall, authentication failures, CLAUDE.md not loading, settings.json syntax errors, plugin installation failures
- **FR-034**: Comparison table (passive AI vs agentic AI) MUST clearly show 6-7 key differences with concrete examples

#### CLAUDE.md and Context Management
- **FR-035**: CLAUDE.md concept MUST be explained with clear examples of what information belongs in context files (project overview, coding conventions, useful commands, project structure)
- **FR-036**: Students MUST create a CLAUDE.md file with AI assistance (Graduated Teaching: Book explains → AI helps create)
- **FR-037**: Content MUST demonstrate that CLAUDE.md auto-loads on session start (show before/after comparison)
- **FR-038**: CLAUDE.md troubleshooting MUST cover: file location (project root), file naming (exact CLAUDE.md case-sensitive), session restart requirement

#### Permission System Understanding
- **FR-039**: Permission concept MUST be explained with safety rationale (why Claude Code asks before file edits/bash commands)
- **FR-040**: Permission modes MUST be introduced conceptually: "default" (asks every time), "acceptEdits" (auto-approves file changes), "plan" (for multi-step tasks)
- **FR-041**: `/permissions` command MUST be demonstrated for checking current settings
- **FR-042**: Content MUST NOT teach permission configuration in detail (that's Part 5 team workflows), only awareness and checking

#### Hooks and Extensibility Awareness
- **FR-043**: Hooks concept MUST be explained with 2-3 concrete examples (PostToolUse running linter, SessionStart loading env vars, PreToolUse validation)
- **FR-044**: Content MUST clarify hooks are conceptual introduction only (building hooks is Part 5/6 advanced content)
- **FR-045**: Event types MUST be mentioned: PreToolUse, PostToolUse, SessionStart, SessionEnd, Stop (conceptual awareness, not implementation)
- **FR-046**: Hook configuration structure MUST be shown: matcher (tool name) → hooks array → type (command) → command (bash script path)

#### Settings Hierarchy Understanding
- **FR-047**: Settings hierarchy MUST be explained: user (`~/.claude/settings.json`), project (`.claude/settings.json`), local (`.claude/settings.local.json`)
- **FR-048**: Precedence order MUST be clear: local > project > user (more specific overrides general)
- **FR-049**: Content MUST NOT teach settings configuration in detail (that's Part 5 team collaboration), only awareness and hierarchy
- **FR-050**: Safety note MUST warn against editing settings.json without understanding (validation on load, backup before editing)

#### Part 1 Integration and Prerequisites
- **FR-051**: Content MUST assume students have completed Chapters 1-4: understand AI revolution, know why AIDD matters, familiar with Nine Pillars framework
- **FR-052**: Content MUST NOT repeat foundational concepts from Part 1 (what is AI-native development, why specs matter) except as brief context refreshers
- **FR-053**: Chapter introduction MUST explicitly bridge from Part 1: "You know WHY AI-driven development matters (Chapters 1-4). Now you'll meet the FIRST TOOL you'll use: Claude Code."

#### Technical Validation Requirements
- **FR-054**: All installation commands MUST be tested on Windows, macOS, and Linux in sandbox environment
- **FR-055**: Authentication flow MUST be verified for both Claude.ai and Claude Console accounts
- **FR-056**: "Hello World" workflow MUST be executed end-to-end in sandbox to verify commands and expected outputs
- **FR-057**: All screenshots/terminal output examples MUST reflect actual tested output, not fabricated examples
- **FR-058**: Plugin installation workflow MUST be tested: `/plugin` → marketplace add → install → restart → verify activation

### Key Entities *(educational content structure)*

- **Lesson**: Individual teaching unit within chapter
  - Attributes: title, duration, learning objectives, content sections, practice exercises, summary
  - Relationships: belongs to chapter, follows prerequisite lessons, contributes to chapter learning outcomes

- **Code Example**: Demonstrated workflow or command
  - Attributes: prompt text, command/code, expected output, explanation
  - Relationships: belongs to lesson, demonstrates specific concept

- **Troubleshooting Entry**: Common problem and solution
  - Attributes: error description, symptoms, platform-specific solutions, verification steps
  - Relationships: belongs to lesson, maps to installation/authentication workflows

- **Comparison Table**: Passive AI vs Agentic AI
  - Attributes: aspect (dimension of comparison), passive AI behavior, agentic AI behavior, concrete example
  - Relationships: belongs to origin story lesson, supports paradigm shift understanding

## Success Criteria *(mandatory)*

### Measurable Outcomes

#### Student Learning Outcomes
- **SC-001**: 95% of students can successfully install Claude Code on first attempt (tested via sandbox validation + beta reader feedback)
- **SC-002**: 90% of students can articulate the difference between passive AI (ChatGPT web) and agentic AI (Claude Code) when presented with scenario-based questions
- **SC-003**: 100% of students complete "Hello World" workflow successfully (all 3-4 prompts execute without errors in sandbox testing)
- **SC-004**: 85% of students can identify when to use Claude Code vs simpler tools based on task complexity (measured via quiz/reflection prompts)
- **SC-005**: 85% of students can explain agent skills progressive disclosure pattern (3 levels: metadata → core → referenced files) with PDF skill example
- **SC-006**: 80% of students can successfully install a plugin via `/plugin` command and understand marketplace ecosystem
- **SC-007**: 85% of students can explain the relationship hierarchy: Plugins contain Skills/Commands/Agents/Hooks/MCP
- **SC-008**: 75% of students can describe MCP configuration structure with 2 external integration examples (GitHub, Filesystem)
- **SC-009**: 90% of students successfully create a CLAUDE.md file with AI assistance and verify it auto-loads on session restart
- **SC-010**: 85% of students can explain why Claude Code asks for permissions and demonstrate using `/permissions` command
- **SC-011**: 80% of students can explain what hooks are and give 2 examples of hook events
- **SC-012**: 80% of students can explain the settings hierarchy (user/project/local) and precedence order

#### Technical Accuracy
- **SC-013**: Zero hallucinated features or capabilities (all claims verified against official documentation from code.claude.com and Context7)
- **SC-014**: 100% of installation methods work on target platforms (Windows/macOS/Linux) as verified by sandbox testing
- **SC-015**: All durations are realistic (installation ~2-5 minutes, authentication ~2 minutes, Skills/Plugins/MCP lesson ~10-12 minutes) within 20% margin of actual measured time
- **SC-016**: Plugin installation workflow verified in sandbox (marketplace add, install, restart, activation)

#### Constitutional Compliance
- **SC-017**: Content demonstrates Graduated Teaching Pattern (Principle 13): Direct commands for installation, AI companion for troubleshooting/exploration, orchestration introduced conceptually
- **SC-018**: Content applies Three Roles Framework (Principle 18): AI teaches concepts (skills/plugins architecture), learns student preferences (marketplace exploration), co-works on installation
- **SC-019**: Chapter respects A2 cognitive load limits: ≤7 concepts per section, ≤2 options where choices required

#### AI Usage Strategy Validation
- **SC-020**: Zero instances of "use AI to run `npm install`" or similar over-engineering (direct commands documented as direct commands)
- **SC-021**: "Try with AI" section uses Chapter 1 clean format (3-4 focused prompts, not 8+ verbose prompts)
- **SC-022**: Students understand WHEN to use Claude Code (complex workflows, exploration, troubleshooting, personalization) vs WHEN simpler tools suffice (direct commands, simple operations)

#### Integration and Coherence
- **SC-023**: Chapter explicitly bridges from Part 1 (references Chapters 1-4 concepts) and prepares for Part 2 continuation (Gemini CLI, Bash, Git) and Part 5/6 (custom plugin/skill development)
- **SC-024**: No contradictions with earlier chapters (methodology, terminology, tool positioning remain consistent)
- **SC-025**: Chapter aligns with Part 2 positioning: Tool literacy (introducing the tool AND personalizing it), not deep methodology (that's Part 5 SDD)

### Evals (How We Measure Success)

**Eval 1: Installation Success Rate**
- **Metric**: Percentage of students completing installation without external help
- **Target**: ≥95% success rate
- **Method**: Sandbox testing (3 platforms) + beta reader survey ("Did you complete installation successfully? Yes/No/Needed help")
- **Business Goal**: Minimize support burden and student frustration

**Eval 2: Conceptual Understanding (Paradigm Shift)**
- **Metric**: Student ability to distinguish passive AI vs agentic AI in scenario-based questions
- **Target**: ≥90% correct identification
- **Method**: Quiz at end of Lesson 1: Present 5 scenarios, student chooses "ChatGPT web" vs "Claude Code" with rationale
- **Business Goal**: Students develop mental model for strategic tool selection

**Eval 3: Hands-On Completion (Hello World)**
- **Metric**: Successful execution of all "Try with AI" prompts
- **Target**: 100% of prompts execute in sandbox without errors
- **Method**: Sandbox validation (run all prompts, verify outputs match expected)
- **Business Goal**: Build student confidence through working practice

**Eval 4: AI Usage Strategy Understanding**
- **Metric**: Student can articulate when to use AI vs direct commands
- **Target**: ≥85% provide correct reasoning
- **Method**: Reflection prompt: "When would you use Claude Code to install UV package manager? When would you just run `curl | bash`?" Score based on reasoning about complexity/context needs
- **Business Goal**: Prevent over-engineering pattern (using AI for everything)

**Eval 5: Technical Accuracy (Zero Hallucinations)**
- **Metric**: Number of factual errors or hallucinated features
- **Target**: 0 errors
- **Method**: Technical reviewer validates all claims against official docs (code.claude.com, Context7 /anthropics/claude-code, WebSearch results)
- **Business Goal**: Build trust and credibility with accurate information

**Eval 6: CLAUDE.md Creation and Usage**
- **Metric**: Student successfully creates CLAUDE.md with AI assistance and verifies auto-loading
- **Target**: ≥90% success rate
- **Method**: Sandbox validation (create simple project, ask Claude to generate CLAUDE.md, restart session, verify context loaded) + student reflection: "Did CLAUDE.md improve your session?"
- **Business Goal**: Establish foundational practice for persistent context management (improves all future Claude Code sessions)

**Eval 7: Permission System Understanding**
- **Metric**: Student can explain permission rationale and use `/permissions` command
- **Target**: ≥85% can explain + demonstrate
- **Method**: Quiz: "Why does Claude Code ask permission before editing files?" + practical test: "Run `/permissions` and explain what you see"
- **Business Goal**: Safety-first mindset for AI collaboration (prevents accidental destructive operations)

**Eval 8: Skills and Plugins Personalization**
- **Metric**: Student can explain agent skills progressive disclosure, install a plugin via `/plugin`, and articulate the relationship hierarchy
- **Target**: ≥80% can explain skills architecture + ≥75% successfully install plugin + ≥85% explain hierarchy
- **Method**: Multi-part assessment: (1) "Explain progressive disclosure in agent skills with PDF example", (2) Hands-on: `/plugin marketplace add anthropics/claude-code` → `/plugin install feature-dev` → verify activation, (3) "How do Skills, Plugins, and MCP relate?"
- **Business Goal**: Establish AI companion personalization foundation for AIDD workflows (students can extend Claude Code for their needs)

**Eval 9: Extensibility Awareness (Hooks + Settings)**
- **Metric**: Student can explain what hooks/settings are conceptually (not implementation)
- **Target**: ≥80% provide correct conceptual explanation
- **Method**: Reflection prompt: "What are hooks in Claude Code? Give 2 examples of hook events. Where can settings be configured?"
- **Business Goal**: Prepare students for advanced customization in Part 5/6 (prevent confusion when they encounter hook/settings references)

## Assumptions

1. **Platform Support**: We assume students are using Windows 10+, macOS 11+, or Linux distributions with Bash support (Ubuntu, Debian, Fedora, Arch)

2. **Internet Access**: Students have reliable internet connection for downloading Claude Code and authenticating (required for Claude API access)

3. **Node.js Availability**: For NPM installation method, students can install Node.js 18+ if not present (prerequisite clearly documented)

4. **Claude Account Access**: Students can create free Claude.ai account or have Claude Console API credits (free tier accounts work for this chapter's usage)

5. **Terminal Comfort**: Students have basic terminal access (can open terminal app, run commands) from Chapter 7 (Bash Essentials) or prior computer literacy

6. **Sandbox Environment**: We have access to sandbox environment for testing installation/authentication/workflows on all platforms before publication

7. **Context7 MCP Access**: We can use Context7 MCP tools to retrieve official Claude Code documentation (/anthropics/claude-code library)

8. **Part 1 Completion**: Students have completed Chapters 1-4 and understand: AI revolution context, AIDD concept, Nine Pillars framework, why specifications matter

9. **Reading Level**: Content targets Grade 7-8 reading level (A2 CEFR tier, aspiring developers audience)

10. **Hands-On Validation**: All code examples and workflows will be tested in sandbox before publication to ensure accuracy

## Dependencies

1. **Official Claude Code Documentation**: Content depends on accuracy of official docs at code.claude.com (must remain accessible)

2. **Context7 MCP Library**: Fact-checking depends on `/anthropics/claude-code` library remaining available and up-to-date

3. **Installation Scripts**: Content depends on Anthropic maintaining install scripts at `https://claude.ai/install.sh` (macOS/Linux) and `https://claude.ai/install.ps1` (Windows)

4. **NPM Package**: NPM installation method depends on `@anthropic-ai/claude-code` package availability

5. **Sandbox Environment**: Content validation depends on sandbox access for multi-platform testing (Windows/macOS/Linux)

6. **Chapter 1 Format**: "Try with AI" section format depends on Chapter 1's established clean prompt format (must be consistent)

7. **Constitution v3.1.3**: All content decisions resolve to constitution (Graduated Teaching, Three Roles, AI Spectrum, cognitive load limits)

8. **Chapter Index**: Lesson structure and prerequisites mapped to chapter-index.md (Chapter 5 in Part 2, follows Chapters 1-4)

9. **Prior Chapters (1-4)**: Content assumes students completed Part 1 and understand foundational concepts (AI revolution, AIDD, Nine Pillars)

10. **Subsequent Chapters**: Content must prepare students for Chapters 6-8 (Gemini CLI, Bash, Git) by establishing AI tool collaboration patterns

## Constraints

1. **Cognitive Load**: Maximum 7 concepts per section (A2 tier limit, Principle 12)

2. **Options Limit**: Maximum 2 options when presenting choices (A2 tier limit, Principle 12)

3. **Duration Accuracy**: All time estimates must reflect actual measured time within 20% margin (no inflated durations)

4. **No Over-Engineering**: Simple operations (installation, authentication) must NOT be wrapped in elaborate AI workflows

5. **Fact-Checking**: Zero tolerance for hallucinated features (all claims verified against official sources)

6. **Platform Coverage**: Installation instructions must cover Windows, macOS, and Linux equally (no platform bias)

7. **Chapter Length**: Total reading time target 60-80 minutes for all 9 lessons (realistic engagement window for comprehensive coverage including Skills/Plugins/MCP hands-on)

8. **Try with AI Format**: Must use Chapter 1 clean format (3-4 focused prompts, not 8+ verbose prompts)

9. **Constitutional Compliance**: All content must align with Principles 13 (Graduated Teaching), 18 (Three Roles), Core Philosophy #1 (AI Spectrum)

10. **No Forward References**: Cannot reference concepts from later chapters (MCP deep dive is Chapter 38, Spec-Driven Development is Part 5)

## Out of Scope

1. **Deep MCP Integration Hands-On**: Chapter introduces MCP conceptually, but actual hands-on MCP server development is Chapter 39 (Part 6)

2. **Custom Subagent Creation**: Chapter explains what subagents are, but building custom subagents is advanced content (later chapters)

3. **Claude Code VS Code Extension**: Focus is terminal-based CLI; VS Code extension mentioned briefly but not primary focus

4. **Advanced Workflows**: Git workflows, multi-file refactoring, project-wide changes are out of scope (those are later chapters with Git/Bash foundation)

5. **Spec-Driven Development**: SDD methodology is Part 5 (Chapters 30-33); this chapter only introduces Claude Code as a tool

6. **Production Deployment**: Docker, Kubernetes, cloud deployment are Part 7+ content

7. **Custom Plugin Development**: Building custom plugins from scratch (creating .claude-plugin/plugin.json manifest, structuring commands/agents/skills directories) is Part 5/6 content. Chapter covers: understanding plugin architecture, installing existing plugins from marketplace, recognizing personalization value for AIDD.

19. **Custom Skill Development**: Creating new SKILL.md files with YAML frontmatter, bundling Python scripts, structuring referenced files is Part 5/6 content. Chapter covers: understanding progressive disclosure pattern, recognizing when skills are invoked, seeing PDF skill example.

8. **API Key Management**: Advanced security practices (secrets management, API key rotation) are later chapters

9. **Multi-Agent Orchestration**: Chapter 32 (Part 5) covers agent teams and manager patterns (competitive subagents, parallel processing); this chapter only introduces the concept

10. **Troubleshooting Every Edge Case**: Troubleshooting section covers 80% common issues; rare edge cases directed to Claude Code docs or community support

11. **Writing Custom Hooks**: Chapter explains hooks conceptually with examples. Building custom hooks is Part 5/6 advanced content.

12. **Advanced Settings Configuration**: Chapter explains settings hierarchy. Detailed team settings configuration is Part 5 (SDD team workflows).

13. **Permission Policy Configuration**: Chapter introduces permission system. Configuring allow/deny rules is Part 5 (team security policies).

14. **CLAUDE.md Advanced Patterns**: Chapter teaches basic CLAUDE.md creation. Advanced patterns (multi-file context, project templates) are Part 5/6.

15. **Competitive Subagent Architecture**: Mentioned as advanced concept from podcast (multiple subagents challenging each other). Implementation is Chapter 32.

16. **Parallel Migration Workflows**: Mentioned as real-world use case. Implementation ($1000/month workflows) is Chapter 32 (AI Orchestration).

17. **Stop Hooks with Automated Testing**: Mentioned as best practice from podcast. Implementation is Chapter 33 (Advanced Workflows).

18. **Plan Mode Deep Dive**: Chapter introduces Plan mode concept. Deep usage patterns (doubling success rates) taught in context of SDD workflows (Part 5).
