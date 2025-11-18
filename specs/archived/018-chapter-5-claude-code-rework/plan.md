# Chapter 5: How It All Started: The Claude Code Phenomenon — Lesson Plan

**Chapter Type**: Technical & Conceptual Hybrid
**Part**: 2 (AI Tool Landscape)
**Target Audience**: A2 Tier (Aspiring Builders, Post-Part 1)
**Total Estimated Duration**: 60-80 minutes across 9 lessons
**Prerequisites**: Chapters 1-4 completed (understands AI revolution, AIDD concepts, Nine Pillars)
**Feature Branch**: `018-chapter-5-claude-code-rework`

---

## 🎯 Chapter Positioning

**This chapter delivers the most comprehensive Claude Code education available—surpassing all official Anthropic resources by uniquely integrating installation, authentication, personalization, and AIDD mindset into ONE cohesive learning journey.**

### Unique Value Proposition

After completing this chapter, students will have **a fully personalized AI companion for AI-Driven Development**, with:
- ✅ Claude Code installed, authenticated, and working on their machine
- ✅ Complete understanding of extensibility (Skills + Plugins + MCP unified in Lesson 4)
- ✅ At least ONE community plugin installed hands-on (marketplace → install → verify)
- ✅ CLAUDE.md context file created for persistent project knowledge
- ✅ Practical knowledge of permissions, hooks, and settings hierarchy
- ✅ AIDD workflow mindset (using AI companion for development, not just coding)

**This is not just learning Claude Code—this is acquiring your AI development partner for the entire course and beyond.**

---

## Chapter Strategic Context

**Learning Purpose**: After Chapters 1-4 (foundational mindset), students meet the FIRST TOOL they'll use: Claude Code. This chapter establishes:
1. **WHY** Claude Code exists (paradigm shift from passive to agentic AI)
2. **HOW** to use it (installation, authentication, basic workflows)
3. **WHAT** you can do with it (practical patterns emerging in industry)
4. **WHEN** to use it vs simpler tools (strategic decision-making)

**Constitutional Alignment**:
- **Principle 13 (Graduated Teaching)**: Tier 1 (book explains concepts directly), Tier 2 (AI helps troubleshoot/create), Tier 3 (orchestration introduced conceptually)
- **Principle 18 (Three Roles)**: AI as Teacher (explains features), Student (learns preferences), Co-Worker (executes workflows)
- **Core Philosophy #1 (AI Spectrum)**: Show progression from Assisted (simple commands) → Driven (specification-driven workflows) → Native (agent orchestration mentioned)
- **Cognitive Load (A2)**: Max 7 concepts per lesson, max 2 options where choices required

**Anti-Over-Engineering Enforcement**:
- Installation commands are DIRECT (students run `curl | bash` or `npm install`, not "ask AI to install")
- Simple operations (version check, basic commands) NOT wrapped in AI workflows
- "Try with AI" section uses Chapter 1 clean format (3-4 focused prompts per lesson max)
- Duration estimates reflect realistic measured time (not inflated)

---

## Lesson Architecture Overview

| # | Lesson Title | Duration | Priority | Key Concept | Tier 1 | Tier 2 | Tier 3 |
|---|--------------|----------|----------|-------------|--------|--------|--------|
| 1 | Origin Story & Paradigm Shift | 8-10 min | P1 | Passive vs Agentic AI | Compare AI types | Self-analysis | - |
| 2 | Installation & Authentication | 10-12 min | P1 | Direct Commands | Install + Auth | Troubleshoot | - |
| 3 | Subagents & Orchestration | 8-10 min | P2 | Specialized Agents | Explain Explore/Plan | Understand when | Orchestration intro |
| 4 | Skills, Plugins, & MCP | 10-12 min | P3 | Personalization | Explain architecture | Install plugin | Custom development defer |
| 5 | Hello World Practice | 10-12 min | P2 | Basic Workflows | Workflow template | Execute with AI | - |
| 6 | CLAUDE.md Context Files | 8-10 min | P1 | Persistent Context | Explain concept | Create with AI | Advanced patterns defer |
| 7 | Permission Management | 6-8 min | P2 | Safety-First | Explain rationale | Check permissions | Config defer to Part 5 |
| 8 | Hooks & Extensibility | 5-7 min | P3 | Event-Driven | Explain concepts | Examples | Building hooks defer |
| 9 | Settings Hierarchy | 4-6 min | P3 | Configuration | Explain hierarchy | Understand precedence | Detailed config defer |

---

## Lesson 1: Origin Story & Paradigm Shift

**Duration**: 8-10 minutes
**Priority**: P1 (Foundation for understanding Claude Code value)
**Prerequisites**: Completion of Chapters 1-4 (understands AI revolution context)

### Learning Objectives

- **Objective 1** (A1 - Remember): Recognize the difference between passive AI (ChatGPT web) and agentic AI (Claude Code) through concrete examples
  - Assessment: Can identify tool choice in 3 different scenarios
  - Bloom's Level: Remember/Understand
  - Skills Taught: "Recognizing AI Paradigm Differences" (A1, Conceptual)

- **Objective 2** (A2 - Understand): Articulate why Claude Code's integration with files and shell makes it fundamentally different from chat interfaces
  - Assessment: Can explain 3 key differences with examples
  - Bloom's Level: Understand
  - Skills Taught: "Distinguishing Context-Aware AI" (A2, Conceptual)

- **Objective 3** (A2 - Understand): Evaluate when Claude Code is the right choice vs when ChatGPT web or terminal commands suffice
  - Assessment: Reflection prompt showing reasoning about tool selection
  - Bloom's Level: Understand/Analyze
  - Skills Taught: "Strategic Tool Selection Thinking" (A2, Soft/Metacognitive)

### Teaching Tier Mapping

**Tier 1 (Book Teaches Directly)**:
- Paradigm definition: Passive vs Agentic AI (conceptual explanation)
- Comparison table: 6-7 key differences (context awareness, direct action, workflow integration, state persistence, multi-file awareness, real-time feedback, error recovery)
- Real-world examples from YC, industry adoption trends
- Why this moment is different (tool maturity, integration capabilities, adoption rate)

**Tier 2 (AI Companion Helps)**:
- Student reflects on their current workflow: "Where do you currently use ChatGPT? What frustrates you about switching contexts?"
- AI helps analyze: "For this task, would Claude Code or ChatGPT web be better? Why?"
- AI provides personalized scenarios: "Give me 3 examples from [their domain] where Claude Code would shine"

**Tier 3 (AI Orchestration)**:
- NOT applicable for this lesson (foundational concept only)

### Content Outline

1. **Hook (1-2 min)**: YC data showing 25% of startups with 95% AI-generated code (revisit Part 1 context)
2. **Passive AI Definition (1.5-2 min)**: ChatGPT web as example—stateless conversation, context reset, manual copy/paste, human drives workflow
3. **Agentic AI Definition (1.5-2 min)**: Claude Code—context awareness, direct file access, bash execution, persistent state, autonomous decision-making
4. **Comparison Table (2-3 min)**: Side-by-side comparison with concrete examples
5. **When to Choose (1-2 min)**: Simple heuristics (simple questions → web; complex projects with files/commands → Claude Code)

### Key Concepts (Cognitive Load Check)

- Passive vs Agentic (1 concept)
- Context awareness as differentiator (2 concepts: file context + execution context)
- Workflow integration benefits (3 concepts: reduced friction, persistent state, tool access)
- Strategic decision-making (4 concepts: task complexity, context needs, repetition, learning benefit)

**Cognitive Load**: 4 distinct concepts within A2 limit of 7 ✓

### Content Elements

**Comparison Table Example:**
| Aspect | ChatGPT Web (Passive) | Claude Code (Agentic) | Real-World Example |
|--------|----------------------|----------------------|-------------------|
| **Context** | Starts fresh each conversation | Remembers project state, file history | "Where is this function?" ChatGPT: "I don't know"; Claude Code: Searches codebase |
| **Action** | You copy/paste code, run manually | Directly edits files, runs commands | Create a file, test it, fix errors—all in one flow |
| **State** | No memory between sessions | CLAUDE.md provides persistent context | Day 2: Claude Code remembers project conventions |
| **Feedback Loop** | Slow (read response, find error, ask again) | Fast (propose → execute → see output → iterate) | Debug issue: Claude Code shows error, fixes it, verifies |
| **Tool Access** | No file/shell access | Full terminal and filesystem integration | "Run tests" → Claude Code executes, sees failures |
| **Learning** | Surface-level (single interaction) | Deep (builds project understanding over time) | Claude Code learns your patterns, suggests improvements |
| **Workflow** | Chat-interrupt model (you context-switch) | Continuous workflow (you stay in project) | One prompt spawns multiple files, commands, validation |

### Practice Approach

**Reflection Prompt** (no AI needed yet):
- Think about a project you've worked on (or imagined)
- Where would ChatGPT web be sufficient?
- Where would it feel frustrating to context-switch?
- How would Claude Code's file awareness and command execution change that?

### End-of-Lesson: Try With AI

**Tool**: ChatGPT web (at this point in book, students use what they know)
**Duration**: ~4-5 minutes total for 3 prompts

#### Prompt 1: Analyze Your Workflow
```
I just learned about Claude Code—an AI tool that can read files, run commands, and keep context across sessions. Think about a project or task I might want to build. What are 3 concrete ways Claude Code would be different from asking ChatGPT web questions?
```

**Expected Outcome**: Student articulates why file context + command execution changes the workflow. Moves from "tool" thinking to "paradigm" thinking.

#### Prompt 2: Tool Selection Reasoning
```
Give me 3 scenarios:
1. A task where ChatGPT web is perfectly fine
2. A task where Claude Code would be way better
3. A task where it's honestly hard to choose

For each, explain your reasoning in 1-2 sentences.
```

**Expected Outcome**: Student builds decision-making heuristics. Starts to see "when to use what" pattern emerging.

#### Prompt 3: Personal Relevance
```
Based on what I want to build (or what interests me), would Claude Code be useful? Why or why not? What would I need to learn first?
```

**Expected Outcome**: Personal stakes established. Student recognizes why this chapter matters for THEIR work.

### Success Criteria Mapping

- **SC-002**: 90% of students can articulate difference between passive and agentic AI (measured by reflection quality)
- **SC-004**: 85% can identify when to use Claude Code vs simpler tools (measured by Prompt 2 reasoning)
- **Eval 2**: Conceptual Understanding (paradigm shift) - 90% correct identification in scenario-based reflection

### Edge Cases Addressed

- **Over-eager learner**: "I want to just start using Claude Code now" → Explain: Installation and authentication need foundational understanding first; we'll do that next lesson
- **Skeptical learner**: "I don't get why this is different from ChatGPT" → Reassurance: The difference becomes clear in Hello World practice (Lesson 5); trust the progression
- **Confused by "agentic"**: Clarify: Agentic just means "can take actions" (edit files, run commands), not "works perfectly alone" (human still directs)

---

## Lesson 2: Installation & Authentication

**Duration**: 10-12 minutes
**Priority**: P1 (Blocker for all subsequent practice)
**Prerequisites**: Lesson 1, Chapter 4 (terminal comfort), Node.js 18+ installed

### Learning Objectives

- **Objective 1** (A1 - Remember): Successfully install Claude Code on their platform using direct commands
  - Assessment: Running `claude --version` returns version number
  - Bloom's Level: Apply
  - Skills Taught: "Installing CLI Tools Correctly" (A1, Technical)

- **Objective 2** (A1 - Understand): Authenticate with Claude AI/Console account and verify working session
  - Assessment: Running test command returns AI response
  - Bloom's Level: Apply
  - Skills Taught: "Authentication Workflow Understanding" (A1, Technical)

- **Objective 3** (A2 - Understand): Troubleshoot common installation issues using provided solutions
  - Assessment: Can identify error, find matching solution, apply fix
  - Bloom's Level: Apply/Analyze
  - Skills Taught: "Troubleshooting Installation Errors" (A2, Technical/Problem-Solving)

### Teaching Tier Mapping

**Tier 1 (Book Teaches Directly)**:
- Installation methods overview (3 main paths: curl script, PowerShell, NPM)
- Step-by-step instructions for each platform (Windows/macOS/Linux)
- Authentication flow explanation (OAuth, token storage, first-run setup)
- Verification steps (checking version, running first command)
- Realistic timing expectations (2-5 minutes actual installation, 1-2 minutes auth)

**Tier 2 (AI Companion Helps)**:
- Troubleshooting: "I got this error after running the install command..."
- Understanding: "Why is Node.js required? What does npm even do?"
- Decision-making: "Which installation method should I use for my setup?"
- Problem-solving: "My PATH variable isn't set—how do I fix this?"

**Tier 3 (AI Orchestration)**:
- NOT applicable (this is installation, not complex workflow)

### Content Outline

1. **Prerequisites Check (1 min)**: Node.js 18+ verification (provide check command)
2. **Installation Options (1-2 min)**: Explain 3 methods; recommend 1 for simplicity
3. **Step-by-Step Installation (3-4 min)**:
   - macOS/Linux: `curl -fsSL https://claude.ai/install.sh | bash`
   - Windows: PowerShell script
   - Fallback: `npm install -g @anthropic-ai/claude-code`
4. **Verification (1-2 min)**: `claude --version`
5. **Authentication (2-3 min)**: First run `claude` command, interactive auth flow
6. **Quick Test (1 min)**: `claude "Hello! Confirm you're working"`

### Key Concepts (Cognitive Load Check)

- Installation method selection (1 concept)
- Authentication flow (OAuth, token persistence) (2 concepts)
- Verification/testing workflow (3 concepts)
- Error diagnosis basics (4 concepts)

**Cognitive Load**: 4 concepts within A2 limit of 7 ✓

### Content Elements

**Code/Command Examples**:
```bash
# Check Node.js version (prerequisite)
node --version  # Should show v18.0.0 or higher

# Install via curl (macOS/Linux)
curl -fsSL https://claude.ai/install.sh | bash

# Verify installation
claude --version

# First authentication
claude  # Triggers interactive auth

# First test command
claude "Hello! Confirm you're working"
```

**Troubleshooting Section**:
- **Issue**: "command not found: claude"
  - **Cause**: PATH not updated after install
  - **Solution**: Restart terminal, or run `source ~/.bashrc` or `source ~/.zshrc`

- **Issue**: "npm ERR! code EACCES (permission denied)"
  - **Cause**: Global npm permissions issue
  - **Solution**: Either (a) use `sudo npm install -g ...` OR (b) configure npm for global without sudo

- **Issue**: "Authentication failed / rate limit"
  - **Cause**: Claude.ai subscription limit reached
  - **Solution**: Wait 1 hour OR use Claude Console API credits (explain both options)

- **Issue**: "Node.js not found"
  - **Cause**: Node.js not installed
  - **Solution**: Install Node.js from nodejs.org (provide link), then retry Claude Code install

### Practice Approach

**Hands-On** (not "Try with AI" yet):
- Follow installation steps directly
- Verify with `claude --version`
- Authenticate interactively
- Test with simple command

### End-of-Lesson: Try With AI

**Tool**: Claude Code (NOW that it's installed!)
**Duration**: ~2-3 minutes total

#### Prompt 1: Verify Installation Worked
```
claude "You're my first Claude Code conversation. Confirm you can see files in my current directory and respond with a greeting."
```

**Expected Outcome**: Proves Claude Code works, can access file context, and provides personalized response. Student feels "first win."

#### Prompt 2: Ask About Next Steps
```
claude "I just installed you. What can you do that ChatGPT web can't? Give me 2-3 concrete examples relevant to learning to code."
```

**Expected Outcome**: Claude Code explains its own capabilities in context of learner's situation. Reinforce Lesson 1 paradigm understanding through Claude's own voice.

#### Prompt 3: Handle Troubleshooting (if needed)
```
If installation failed: claude "Help me debug this error: [error message]. What might be wrong?"
```

**Expected Outcome**: If student hit issue, resolve it immediately. If not, skip this prompt.

### Success Criteria Mapping

- **SC-001**: 95% of students install successfully on first attempt (measured by `claude --version` success)
- **SC-006**: 100% installation works on target platforms (Windows/macOS/Linux) - verified in sandbox
- **SC-007**: Durations realistic: install 2-5 min, auth 1-2 min, test 1 min (within 20% of measured time)
- **Eval 1**: Installation Success Rate - 95% complete without external help

### Edge Cases Addressed

**Rate Limit During Auth**:
- Explanation: Claude.ai free tier has limits; if you hit them, wait 1 hour or switch to Console API credits
- Mitigation: Provide both paths explicitly

**Firewall/Network Issues**:
- Testing: Ask IT if corporate firewall blocks claude.ai API
- Workaround: May need proxy configuration (out of scope, but mention)
- Documentation: Link to troubleshooting section

**Uncommitted Git Changes**:
- Explanation: Claude Code will see uncommitted changes in repo—this is intentional
- Why: It needs full project context to help effectively
- Best Practice: Use git best practices, but don't block workflow

---

## Lesson 3: Subagents & Orchestration

**Duration**: 8-10 minutes
**Priority**: P2 (Understanding optional; critical for Part 5)
**Prerequisites**: Lessons 1-2 completed, Claude Code installed

### Learning Objectives

- **Objective 1** (A2 - Understand): Explain what subagents are and why Claude Code uses them instead of one big conversation
  - Assessment: Can describe Explore agent and Plan mode in own words
  - Bloom's Level: Understand
  - Skills Taught: "Understanding AI Agent Specialization" (A2, Conceptual)

- **Objective 2** (A2 - Understand): Recognize when Claude Code automatically invokes subagents and why that prevents context drift
  - Assessment: Can identify Explore agent activity in practice; understand fast search patterns
  - Bloom's Level: Understand
  - Skills Taught: "Recognizing Agent Orchestration Patterns" (A2, Conceptual)

- **Objective 3** (A2 - Understand): Evaluate when Plan mode helps with multi-step problems
  - Assessment: Reflection on tasks where breaking into phases would help
  - Bloom's Level: Understand/Analyze
  - Skills Taught: "Strategic Use of Orchestration Tools" (A2, Conceptual)

### Teaching Tier Mapping

**Tier 1 (Book Teaches Directly)**:
- Subagent concept: Specialized agents that focus on specific tasks
- Explore agent: Fast codebase search, pattern recognition, efficient file scanning
- Plan mode: Multi-step task decomposition, phased execution
- Why this matters: Prevents context degradation, maintains focus, improves quality
- Real-world benefits: Faster exploration, fewer hallucinations, cleaner outputs

**Tier 2 (AI Companion Helps)**:
- Student asks Claude Code: "What would Explore agent do if I asked you to find all error handlers in this codebase?"
- Understand strategy: "Why did you choose Plan mode for this task?"
- Recognize patterns: "Which subagent are you using right now to answer my question?"

**Tier 3 (AI Orchestration)**:
- Competitive subagents concept introduced (NOT implemented): "Multiple agents discussing approaches to same problem"
- Orchestration at scale: "How would Claude Code manage 5 specialized agents on a complex refactor?"
- Deferred to Part 5/6: "Building custom subagents is advanced content for Chapter 32"

### Content Outline

1. **Problem Context (1-2 min)**: Why context drift happens in long conversations; why single agent struggles at scale
2. **Subagent Explanation (2 min)**: Each agent specializes; coordination prevents confusion
3. **Explore Agent Deep Dive (2-3 min)**: Efficient codebase search, pattern matching, rapid orientation
4. **Plan Mode Explanation (2 min)**: Breaking complex problems into phases; why phases work
5. **When to Recognize (1-2 min)**: How to tell which agent is working; what speed/patterns indicate

### Key Concepts (Cognitive Load Check)

- Subagent specialization concept (1 concept)
- Explore agent for codebase search (2 concepts: search, pattern recognition)
- Plan mode for multi-step tasks (3 concepts: decomposition, phasing, orchestration)
- Recognition patterns (4 concepts: when Explore activates, when Plan helps, etc.)
- Context drift problem being solved (5 concepts)

**Cognitive Load**: 5 distinct concepts within A2 limit of 7 ✓

### Content Elements

**Exploration Example**:
```
Student: "Where are all the error handlers in this codebase?"

Claude Code (Explore Agent):
  - Scans directory structure quickly
  - Identifies patterns (except blocks, error callbacks, try/catch)
  - Returns focused results
  - Does NOT load entire codebase into context
  → Result: Fast, accurate, no context bloat
```

**Plan Mode Example**:
```
Student: "Refactor this monolithic service into microservices. Here's the architecture."

Claude Code (Plan Mode):
  1. Phase 1: Analyze current structure
  2. Phase 2: Design service boundaries
  3. Phase 3: Extract service A (authentication)
  4. Phase 4: Extract service B (payments)
  5. Phase 5: Test orchestration

Why phases: Each phase focuses narrowly, preventing context explosion.
```

### Practice Approach

**Observation Task** (in Lesson 5 Hello World):
- When you ask Claude Code "Where is the main function?", notice speed/efficiency
- That's Explore agent working
- Compare: Would ChatGPT web find this as fast? Why/why not?

### End-of-Lesson: Try With AI

**Tool**: Claude Code
**Duration**: ~4-5 minutes total

#### Prompt 1: Recognize Explore Agent
```
cd into a small project with multiple files. Then ask:
claude "Search this codebase for all places where we handle network errors. Tell me what you're searching for and what patterns you find."
```

**Expected Outcome**: Claude Code demonstrates efficient search. Student notices: "This wasn't a slow read of every file—it was strategic pattern matching." That's Explore agent in action.

#### Prompt 2: Understand Plan Mode
```
claude "I want to add user authentication to my app. Here's my current structure: [2-3 line summary]. Plan out how you'd approach this in phases."
```

**Expected Outcome**: Claude Code provides multi-phase plan. Student sees: "Instead of one big conversation, we're breaking this into manageable pieces." That's Plan mode value.

#### Prompt 3: Evaluate Usefulness
```
For a task you're actually working on:
claude "If I ask you to [complex multi-step task], would you use Plan mode? Why? What would the phases be?"
```

**Expected Outcome**: Claude Code explains orchestration strategy FOR THEIR TASK. Makes it personal, not abstract.

### Success Criteria Mapping

- **SC-002**: 90% understand paradigm of specialized agents (Eval 2 extended)
- **Eval 2**: Conceptual Understanding - Can explain Explore and Plan in own words
- Part 5 Prerequisite: Understanding subagent concept enables Chapter 32 (AI Orchestration) deeper work

### Edge Cases Addressed

**Confusion between Subagents and Plugins**:
- Subagents: AI agents specialized for tasks
- Plugins/Skills: Tool integrations (Google Drive, Jira, etc.)
- Clarification: We're talking about AI specialization now; tools/plugins come in Lesson 4

**Over-expecting Autonomy**:
- Clarification: Subagents are smart assistants, not fully autonomous
- You still direct the work; they just handle specific tasks efficiently
- Mental model: "Like hiring specialists on your team, not robots replacing you"

---

## Lesson 4: Skills, Plugins, and MCP Integration

**Duration**: 10-12 minutes
**Priority**: P3 (AI companion personalization foundation; custom development deferred to Part 5/6)
**Prerequisites**: Lesson 2 (Claude Code installed), basic familiarity with APIs

### Learning Objectives

- **Objective 1** (A2 - Understand): Explain agent skills progressive disclosure pattern with concrete example
  - Assessment: Can describe 3-level architecture (metadata → core → referenced files) and give PDF skill example
  - Bloom's Level: Understand
  - Skills Taught: "Understanding Progressive Disclosure Architecture" (A2, Conceptual)

- **Objective 2** (A2 - Understand): Recognize plugin architecture as container for bundled customizations
  - Assessment: Can list what plugins bundle (commands + agents + skills + hooks + MCP) and explain .claude-plugin/plugin.json purpose
  - Bloom's Level: Understand
  - Skills Taught: "Understanding Plugin Architecture" (A2, Conceptual)

- **Objective 3** (A2 - Understand): Describe MCP as external tool integration standard
  - Assessment: Can explain MCP configuration structure (mcpServers → command/args/oauth) and give 2 examples (GitHub, Filesystem)
  - Bloom's Level: Understand
  - Skills Taught: "Recognizing External Integration Patterns" (A2, Conceptual)

- **Objective 4** (A2 - Apply): Demonstrate installing a plugin via `/plugin` command and browsing marketplace
  - Assessment: Can execute `/plugin marketplace add anthropics/claude-code` → `/plugin install feature-dev` → restart → verify activation
  - Bloom's Level: Apply
  - Skills Taught: "Plugin Installation and Marketplace Navigation" (A2, Hands-on)

- **Objective 5** (A2 - Analyze): Distinguish between Skills (autonomous), Plugins (bundled), and MCP (external) in extensibility hierarchy
  - Assessment: Can explain relationship: "Plugins are containers that bundle Skills + Commands + Agents + Hooks + MCP configs"
  - Bloom's Level: Analyze
  - Skills Taught: "Extensibility Architecture Comprehension" (A2, Conceptual)

### Teaching Tier Mapping

**Tier 1 (Book Teaches Directly)**:
- Agent skills progressive disclosure: 3-level architecture (metadata → core → referenced files)
- SKILL.md structure: YAML frontmatter + markdown + bundled Python scripts
- PDF skill example: Form filling + field extraction with deterministic operations
- Plugin architecture: .claude-plugin/plugin.json manifest + bundled components
- What plugins bundle: commands + agents + skills + hooks + MCP configs
- Plugin installation workflow: `/plugin` → marketplace → add → install → restart
- MCP protocol: External tool integration standard with JSON configuration
- MCP examples: GitHub (@github with OAuth), Filesystem (@filesystem with path restrictions)
- Relationship hierarchy: Plugins as containers, Skills as autonomous components

**Tier 2 (AI Companion Helps)**:
- Exploring marketplace: `/plugin` → browse available plugins
- Understanding Skills architecture: "Explain progressive disclosure with PDF example"
- Installing plugin hands-on: `/plugin marketplace add anthropics/claude-code` → `/plugin install feature-dev`
- Planning custom plugin: "If I wanted [framework] plugin with [workflow], what would I include?"
- Troubleshooting: "Why isn't my plugin loading?" or "How do I verify plugin activation?"

**Tier 3 (AI Orchestration)**:
- Building custom plugins from scratch (deferred to Part 5/6: Chapter 32-33)
- Creating new SKILL.md files with YAML frontmatter (deferred to Part 5/6)
- Developing MCP servers (deferred to Part 6: Chapter 39 - MCP Server Development)
- Orchestrating multiple plugins for complex workflows (deferred to Part 5/6)
- Team-wide plugin configuration and distribution (deferred to Part 5)

### Content Outline

1. **Agent Skills Deep Dive** (3-4 min):
   - Progressive disclosure pattern explained (3 levels: metadata in system prompt → full SKILL.md when relevant → referenced files only when needed)
   - SKILL.md structure demonstration (YAML frontmatter with name/description + markdown instructional content + optional bundled files like Python scripts)
   - PDF skill concrete example (form filling and field extraction using bundled Python script for deterministic operations beyond basic PDF comprehension)
   - When Claude invokes skills autonomously (based on task context, not user command)
   - Filesystem-based discovery and dynamic loading

2. **Plugins System** (3-4 min):
   - What plugins bundle (commands + agents + skills + hooks + MCP configurations)
   - Plugin architecture (.claude-plugin/plugin.json manifest with metadata + component directories)
   - Installation workflow (`/plugin` command → browse marketplace → `/plugin marketplace add <url>` → `/plugin install <name>` → restart Claude Code → verify in `/help`)
   - Community plugin ecosystem (anthropics/claude-code: feature-dev, code-review, security-guidance; Dan Ávila marketplace: DevOps, documentation, testing; Seth Hobson: 80+ specialized sub-agents)
   - Team configuration (.claude/settings.json with extraKnownMarketplaces)
   - Use cases: team standards enforcement, framework-specific shortcuts, internal tool connections, debugging setups, deployment workflows

3. **MCP Integration** (2-3 min):
   - MCP (Model Context Protocol) definition (standard for external tool integration)
   - JSON configuration structure (mcpServers object → server name → command/args/oauth fields)
   - GitHub MCP server example (npx @anthropic-ai/mcp-server-github with OAuth clientId/clientSecret/scopes)
   - Filesystem MCP server example (npx @modelcontextprotocol/server-filesystem with allowed path restrictions)
   - @-mentions activation (@github for GitHub queries, @filesystem for file operations)
   - How MCP configs fit inside plugins (plugins can contain .mcp.json files)

4. **Relationship Hierarchy** (1-2 min):
   - Visual diagram: Plugins (containers) → contain → [Skills + Commands + Agents + Hooks + MCP]
   - Skills = autonomous capabilities (Claude decides when to invoke based on task)
   - Commands = user-invoked slash commands (like `/plugin`, `/code-review`, `/feature-dev`)
   - Agents = specialized subagents (Explore for codebase search, Plan for multi-step tasks)
   - Hooks = event-triggered automation (PreToolUse validation, PostToolUse linting, SessionStart env loading)
   - MCP = external integrations (GitHub, Filesystem, Google Drive, Jira, etc.)
   - Key distinction: Plugins are broadest container; Skills are one autonomous component among many

### Key Concepts (Cognitive Load Check)

- Agent skills with progressive disclosure (1 concept: 3-level architecture)
- SKILL.md structure (2 concepts: YAML + markdown + bundled files)
- Plugin architecture (3 concepts: containers bundling multiple components)
- Plugin installation workflow (4 concepts: command → marketplace → install → restart)
- MCP protocol (5 concepts: external integration standard with JSON config)
- Relationship hierarchy (6 concepts: Plugins contain Skills/Commands/Agents/Hooks/MCP)
- Personalization for AIDD (7 concepts: customizing AI companion for workflow)

**Cognitive Load**: 7 distinct concepts exactly at A2 limit of 7 ✓

### Content Elements

**SKILL.md Example Structure** (PDF Skill):
```markdown
---
name: pdf
description: Manipulate PDF files - fill forms, extract fields, read content
---

# PDF Skill

This skill enables Claude to work with PDF files beyond basic reading.

## Capabilities
- Extract form fields from PDFs
- Fill PDF forms programmatically
- Parse PDF structure deterministically

## Implementation
Uses bundled Python script (`pdf_parser.py`) for deterministic field extraction.

## Usage
Claude invokes this skill automatically when you mention PDF manipulation tasks.
```

**Plugin Architecture Example** (.claude-plugin/plugin.json):
```json
{
  "name": "feature-dev",
  "version": "1.0.0",
  "author": "Anthropic",
  "description": "7-phase structured workflow for feature development",
  "components": {
    "commands": ["commands/feature-dev.md"],
    "agents": ["agents/code-explorer.md", "agents/code-architect.md"],
    "skills": ["skills/code-analysis.md"],
    "hooks": ["hooks/pre-commit.sh"],
    "mcp": [".mcp.json"]
  }
}
```

**MCP Configuration Example** (GitHub + Filesystem):
```json
{
  "mcpServers": {
    "github": {
      "command": "npx",
      "args": ["-y", "@anthropic-ai/mcp-server-github"],
      "oauth": {
        "clientId": "your-client-id",
        "clientSecret": "your-client-secret",
        "scopes": ["repo", "issues"]
      }
    },
    "filesystem": {
      "command": "npx",
      "args": ["-y", "@modelcontextprotocol/server-filesystem", "/path/to/allowed/files"]
    }
  }
}
```

**Relationship Hierarchy Diagram**:
```
┌─────────────── Plugins (Containers) ───────────────┐
│                                                     │
│  ┌─────────────┐  ┌──────────────┐  ┌───────────┐ │
│  │   Skills    │  │   Commands   │  │   Agents  │ │
│  │ (autonomous)│  │ (user-invoked│  │(subagents)│ │
│  └─────────────┘  └──────────────┘  └───────────┘ │
│                                                     │
│  ┌─────────────┐  ┌──────────────┐                │
│  │    Hooks    │  │  MCP Configs │                │
│  │(event-based)│  │  (external)  │                │
│  └─────────────┘  └──────────────┘                │
└─────────────────────────────────────────────────────┘
```

### Practice Approach

**Hands-On Activity**:
- Install actual plugin from marketplace
- Explore plugin components after installation
- Verify activation in `/help` command
- Understand how plugins personalize Claude Code for AIDD workflows

**Reflection Task**:
- Which community plugins match your workflow needs?
- How would you use `feature-dev` or `code-review` plugins?
- What custom plugin would you build for your team? (aspirational)

### End-of-Lesson: Try With AI

**Tool**: Claude Code
**Duration**: ~4-5 minutes total (includes hands-on plugin installation)

#### Prompt 1: Explore Plugins
```
claude "/plugin"
```

**Expected Outcome**: Marketplace browser opens. Student sees available plugins, installation options. Understands `/plugin` is the entry point for personalization.

#### Prompt 2: Understand Skills Architecture
```
claude "Explain agent skills in Claude Code. What is progressive disclosure? Give me the PDF skill example showing the 3 levels: metadata, core content, and referenced files."
```

**Expected Outcome**: Clear explanation of how skills work, why progressive disclosure keeps context lean, and concrete PDF example showing Python script execution.

#### Prompt 3: Install Example Plugin (Hands-On)
```
claude "/plugin marketplace add anthropics/claude-code"
# Wait for confirmation

claude "/plugin install feature-dev"
# Wait for download/install

# Restart Claude Code (exit and re-run `claude`)

# Verify activation:
claude "/help"
# Should now show `/feature-dev` command
```

**Expected Outcome**: Student successfully installs real plugin, understands restart requirement, verifies activation. **This is hands-on personalization, not just reading about it.**

#### Prompt 4: Plan Custom Plugin (Aspiration)
```
claude "If I wanted to create a [React/FastAPI/Django] plugin with shortcuts for [common workflow tasks], what would I include? Commands? Agents? Skills? Hooks? Give me the plugin.json structure."
```

**Expected Outcome**: Student sees how plugins combine components, understands plugin.json structure, gets excited about future customization possibilities. Bridges to Part 5/6 custom development.

### Success Criteria Mapping

- **SC-005**: 85% can explain agent skills progressive disclosure (3 levels) with PDF example
- **SC-006**: 80% can successfully install plugin via `/plugin` command
- **SC-007**: 85% can explain relationship hierarchy (Plugins contain Skills/Commands/Agents/Hooks/MCP)
- **SC-008**: 75% can describe MCP configuration with 2 examples (GitHub, Filesystem)
- **Eval 8**: Skills and Plugins Personalization (multi-part: explain skills + install plugin + explain hierarchy)
- Part 5/6 Prerequisites: Foundational understanding of extensibility architecture enables custom plugin/skill development

### Edge Cases Addressed

**Confusion between Skills, Plugins, and MCP**:
- **Clear Hierarchy**: Plugins are containers → Skills are one component inside plugins (autonomous capabilities)
- **Skills**: Claude invokes automatically based on task context (PDF skill for PDF operations)
- **Plugins**: User installs via `/plugin` command (bundles commands + agents + skills + hooks + MCP)
- **MCP**: External tool integration protocol (GitHub, Filesystem) - contained within plugins
- **Key Distinction**: Skills = autonomous invocation; Commands = user invocation; MCP = external connections
- Visual diagram reinforces this hierarchy

**Plugin Installation Failures**:
- Network issues: "Check internet connection, try again"
- Permissions errors: "May need sudo or npm global install permissions"
- Restart not performed: "Plugins activate after restart - exit and re-run `claude`"
- Verification: "Run `/help` to see if new commands appear"

**Marketplace Not Accessible**:
- Firewall blocking: "Corporate networks may block marketplace URLs"
- No plugins available message: "Need to run `/plugin marketplace add <url>` first"
- Custom marketplace: "Teams can host private marketplaces with extraKnownMarketplaces in settings.json"

**Expectation of Building Custom Skills/Plugins Now**:
- Redirect: "Custom development is Part 5/6 advanced content (Chapters 32-33 for plugins, Chapter 39 for MCP servers)"
- Reassurance: "For now, focus on understanding architecture and installing existing plugins - that's AI companion personalization for AIDD"
- Bridge: "This lesson prepares you for custom development later by teaching the foundational architecture"

**Overwhelm from Too Many New Concepts**:
- Simplification: "Think of it as: Plugins are app stores, Skills are apps that run automatically, MCP connects external tools"
- Focus: "You don't need to memorize details - just know plugins exist, how to install them, and they personalize Claude Code"
- Hands-on reduces cognitive load: Installing one plugin makes everything concrete

---

## Lesson 5: Hello World Practice

**Duration**: 10-12 minutes
**Priority**: P2 (Critical hands-on confidence-building)
**Prerequisites**: Lessons 1-4, Claude Code installed and authenticated

### Learning Objectives

- **Objective 1** (A2 - Apply): Execute a complete Claude Code workflow: specify intent → Claude acts → verify output
  - Assessment: Successfully complete all 3-4 prompts; observe Claude Code responses
  - Bloom's Level: Apply
  - Skills Taught: "Executing Basic Claude Code Workflows" (A2, Technical)

- **Objective 2** (A2 - Apply): Understand the rapid feedback loop (propose → execute → observe → iterate)
  - Assessment: Can describe workflow pattern in own words after practicing
  - Bloom's Level: Apply/Understand
  - Skills Taught: "Understanding Agentic Feedback Loops" (A2, Conceptual)

- **Objective 3** (A2 - Analyze): Identify the roles of human (decider/verifier) vs Claude Code (executor)
  - Assessment: Reflection: "What did I do? What did Claude Code do?"
  - Bloom's Level: Analyze
  - Skills Taught: "Role Clarity in Human-AI Collaboration" (A2, Soft)

### Teaching Tier Mapping

**Tier 1 (Book Teaches Directly)**:
- Workflow template: Specify what you want → Run command → Review output → Iterate
- Best practice example: Create simple Python script, test it, verify output
- Role clarity: Human specifies intent; Claude Code executes
- When to intervene: If output is wrong, specify correction

**Tier 2 (AI Companion Helps)**:
- Claude Code creates the file with correct syntax
- Troubleshooting: "This error means..."
- Iteration: "How should I change this?"
- Explanation: "Why did I write it this way?"

**Tier 3 (AI Orchestration)**:
- NOT applicable for Hello World (this is foundational practice)

### Content Outline

1. **Workflow Explanation (1-2 min)**: Template - specify intent, Claude acts, verify
2. **Prompt 1 Execution (2-3 min)**: Create hello.py with Claude Code
3. **Prompt 2 Execution (2-3 min)**: Run the file, see output
4. **Prompt 3 Execution (1-2 min)**: Understand or modify based on result
5. **Prompt 4 Execution (1-2 min)**: Reflect on what just happened

### Key Concepts (Cognitive Load Check)

- Specification → execution → verification workflow (1 concept)
- Claude Code as executor (file writer, command runner) (2 concepts)
- Human as director/verifier (3 concepts)
- Iterative refinement loop (4 concepts)
- Success recognition (5 concepts)

**Cognitive Load**: 5 concepts within A2 limit of 7 ✓

### Content Elements

**The Workflow in Action**:
```
Step 1 (Human): Specify intent
"Create a Python file that prints 'Hello World' and the current date"

Step 2 (Claude Code): Execute
(Creates hello.py with correct imports and logic)

Step 3 (Human): Verify
(Sees file created, asks to run it)

Step 4 (Claude Code): Execute again
(Runs python hello.py, shows output)

Step 5 (Human): Evaluate
(Output looks correct! Or: "Change the date format to...")

Step 6 (Claude Code): Iterate
(Updates code based on feedback)
```

### Practice Approach

**All Hands-On (in "Try with AI" section)**:
- All 4 prompts executed sequentially
- Student observes each step
- Optional 5th reflection prompt

### End-of-Lesson: Try With AI

**Tool**: Claude Code
**Duration**: ~8-10 minutes total for 4 prompts (this lesson is more extensive)

#### Prompt 1: Create File
```
Create a Python file called hello.py that:
- Prints "Hello World!"
- Prints the current date and time
- Is clean and readable

Show me the file you create.
```

**Expected Outcome**: Claude Code generates file content. Student sees: "It understood my intent and created working code."

#### Prompt 2: Run and Test
```
Now run that hello.py file and show me the output.
```

**Expected Outcome**: Claude Code executes the file. Student sees: "The code actually works! No errors!"

#### Prompt 3: Modify and Improve
```
That output looks good, but I want the date format to be more readable (e.g., "Monday, January 13, 2025"). Update the code.
```

**Expected Outcome**: Claude Code modifies the code based on specification. Student sees: "I asked for a change, and it happened. This is the iterative loop in action."

#### Prompt 4: Reflect on What Happened
```
Now that you've seen this workflow: What did you (Claude Code) do? What did I (the human) do? Who decided what to build? Who wrote the code? Who verified it worked?
```

**Expected Outcome**: Claude Code explains the human-AI roles clearly. Student articulates: "I decided WHAT, you handled HOW, I verified it worked. We collaborated."

### Success Criteria Mapping

- **SC-003**: 100% of students complete Hello World successfully (all 4 prompts execute without errors)
- **SC-004**: 85% can identify when to use Claude Code vs ChatGPT (measured via Prompt 4 reflection)
- **Eval 3**: Hands-On Completion - 100% prompts execute in testing, with expected outputs matching
- **Eval 4**: AI Usage Strategy - Student understands human-AI role split

### Edge Cases Addressed

**File Already Exists**:
- Mitigation: Specify filename uniqueness or ask Claude to overwrite
- Teaching: "Claude Code will ask before overwriting; you have control"

**Python Not Installed**:
- Error: "python: command not found"
- Solution: "We'll need Python installed first. Let's do that in a later chapter. For now, try a different simple task like creating a markdown file."
- Defer: Python fundamentals covered in Part 4 (Chapters 12+)

**Module Import Error**:
- Example: "No module named datetime"
- Explanation: Datetime is standard library; error suggests Python environment issue
- Solution: Collaborate with Claude Code: "I got this import error—what might be wrong?"

**Overwhelm by Options**:
- Guidance: "You don't need to understand every line of the code—you need to understand the WORKFLOW (specify → execute → verify)."
- Reassurance: "Python syntax details come later. This lesson is about understanding Claude Code's role."

---

## Lesson 6: CLAUDE.md Context Files

**Duration**: 8-10 minutes
**Priority**: P1 (Foundational for effective Claude Code usage)
**Prerequisites**: Lessons 1-5, sample project to work with

### Learning Objectives

- **Objective 1** (A2 - Understand): Explain what CLAUDE.md is and why persistent context matters
  - Assessment: Can articulate: "CLAUDE.md is a file that tells Claude Code about my project"
  - Bloom's Level: Understand
  - Skills Taught: "Understanding Persistent Context Management" (A2, Conceptual)

- **Objective 2** (A2 - Apply): Create a CLAUDE.md file for a real or sample project with AI assistance
  - Assessment: CLAUDE.md exists in project root, contains relevant sections (overview, conventions, structure)
  - Bloom's Level: Apply
  - Skills Taught: "Creating Project Context Files" (A2, Technical)

- **Objective 3** (A2 - Understand): Recognize how CLAUDE.md auto-loads and improves future sessions
  - Assessment: Can explain: "Claude Code remembers my project conventions across sessions because of CLAUDE.md"
  - Bloom's Level: Understand
  - Skills Taught: "Leveraging Persistent Context" (A2, Conceptual)

### Teaching Tier Mapping

**Tier 1 (Book Teaches Directly)**:
- CLAUDE.md concept: A markdown file in project root that provides persistent context
- What goes in CLAUDE.md: Project overview, coding conventions, directory structure, useful commands, dependencies
- Why it matters: Prevents repeating context every session; Claude Code loads it automatically
- Auto-loading behavior: Session starts → Claude Code reads CLAUDE.md → Uses it for all interactions
- Benefit: Consistency (Claude remembers conventions), efficiency (no re-explaining), quality (better outputs)

**Tier 2 (AI Companion Helps)**:
- Student: "Help me create a CLAUDE.md for my project"
- Claude Code generates appropriate sections based on project
- Student reviews and refines
- Student verifies it loads on next session

**Tier 3 (AI Orchestration)**:
- Advanced CLAUDE.md patterns (deferred to Part 5/6)
- Multi-file context strategies (deferred)
- Team CLAUDE.md configurations (deferred)

### Content Outline

1. **Problem Statement (1-2 min)**: Repeating context every session is friction; persistent context solves this
2. **CLAUDE.md Concept (1-2 min)**: Markdown file in project root; auto-loaded on session start
3. **Content Structure (1-2 min)**: Typical sections (overview, conventions, structure, useful commands)
4. **Example CLAUDE.md (1 min)**: Real example for sample project
5. **Creation Process (1-2 min)**: Ask Claude Code for help; review and refine
6. **Verification (1 min)**: Restart session; confirm auto-loading

### Key Concepts (Cognitive Load Check)

- CLAUDE.md purpose (persistent context) (1 concept)
- Auto-loading mechanism (2 concepts: file location, session start loading)
- Content categories (3 concepts: project overview, conventions, structure)
- Benefit of reduced repetition (4 concepts)
- Consistency across sessions (5 concepts)

**Cognitive Load**: 5 concepts within A2 limit of 7 ✓

### Content Elements

**Example CLAUDE.md Structure**:
```markdown
# Project Context for [ProjectName]

## Project Overview
Brief description of what this project does and its main purpose.

## Technology Stack
- Python 3.13+ with type hints
- FastAPI for APIs
- PostgreSQL for data storage

## Directory Structure
```
project/
├── src/           # Main application code
├── tests/         # Test files
├── docs/          # Documentation
└── config/        # Configuration files
```

## Coding Conventions
- Type hints required on all functions
- Docstrings for all public functions
- Imports sorted: stdlib → third-party → local
- Max line length: 100 characters

## Key Commands
- `uv run pytest`: Run tests
- `uv run uvicorn main:app --reload`: Start dev server
- `uv run black src/`: Format code

## Important Notes
- Database config in `.env` (not in repo)
- Tests must pass before committing
- Breaking changes require discussion first
```

### Practice Approach

**Hands-On Creation**:
1. Student identifies a project (real or hypothetical)
2. Asks Claude Code: "Create a CLAUDE.md for my [project type] project"
3. Reviews generated content
4. Refines any inaccuracies
5. Saves to project root
6. Optional: Restart Claude Code session to verify auto-loading

### End-of-Lesson: Try With AI

**Tool**: Claude Code
**Duration**: ~5-6 minutes total

#### Prompt 1: Generate CLAUDE.md
```
I'm starting a new [Python/JavaScript/other] project for [brief description].
Create a CLAUDE.md file that tells you about my project conventions, structure, and how to help me.
Include sections for: Project Overview, Technology Stack, Directory Structure, Coding Conventions, and Key Commands.
```

**Expected Outcome**: Claude Code generates CLAUDE.md with all sections relevant to project type.

#### Prompt 2: Refine Content
```
Review the CLAUDE.md you created. Make sure it reflects my actual project (here are adjustments: [any corrections]).
```

**Expected Outcome**: Claude Code refines CLAUDE.md based on corrections. Student sees: "This now accurately describes my project."

#### Prompt 3: Create the File
```
Now create the CLAUDE.md file in my project root so it auto-loads in future sessions.
```

**Expected Outcome**: Claude Code creates the file. Student verifies file exists.

#### Prompt 4: Test Auto-Loading (Optional)
```
Exit this session (we'll restart Claude Code next). When I start a new session in this project, I'll ask you what you know about it. You should mention details from CLAUDE.md without me re-explaining.
```

**Expected Outcome**: Sets expectation for next session. Student will observe auto-loading in action.

### Success Criteria Mapping

- **SC-017**: 90% of students successfully create CLAUDE.md with AI assistance
- **Eval 6**: CLAUDE.md Creation and Usage - Sandbox validates creation and auto-loading behavior

### Edge Cases Addressed

**CLAUDE.md Not Loading on Next Session**:
- Diagnosis: Check file location (must be in project root), filename case (must be CLAUDE.md exactly)
- Solution: "Restart Claude Code explicitly; sometimes caching delays auto-load"
- Verification: Run `ls -la` to confirm file exists and is readable

**Unclear What Goes in CLAUDE.md**:
- Guidance: "Think of it as answering: 'What does Claude Code need to know to help me effectively?'"
- Examples: "My team uses this naming convention...", "Here's my directory structure...", "This is how we handle errors..."

**Concern about File Size**:
- Reassurance: "CLAUDE.md can be substantial (1-3 KB is normal). Context is cheap; clarity is expensive."
- Guidance: "Include what's essential; external docs can be referenced rather than duplicated"

---

## Lesson 7: Permission Management

**Duration**: 6-8 minutes
**Priority**: P2 (Safety-first understanding; configuration deferred)
**Prerequisites**: Lessons 1-5

### Learning Objectives

- **Objective 1** (A2 - Understand): Explain why Claude Code asks for permission before file edits and bash commands
  - Assessment: Can articulate safety rationale without prompting
  - Bloom's Level: Understand
  - Skills Taught: "Understanding AI Safety Principles" (A2, Conceptual/Soft)

- **Objective 2** (A2 - Apply): Demonstrate using `/permissions` command to check current settings
  - Assessment: Successfully run command; can read and interpret output
  - Bloom's Level: Apply
  - Skills Taught: "Managing AI Tool Permissions" (A2, Technical)

- **Objective 3** (A2 - Understand): Recognize permission modes (default/acceptEdits/plan) and when each applies
  - Assessment: Can explain differences and use cases for each mode
  - Bloom's Level: Understand
  - Skills Taught: "Understanding Permission Modes" (A2, Conceptual)

### Teaching Tier Mapping

**Tier 1 (Book Teaches Directly)**:
- Permission concept: Safety mechanism ensuring you approve AI actions before they happen
- Why it matters: Prevents accidental file deletion, unwanted code changes, destructive commands
- Permission modes: "default" (asks every time), "acceptEdits" (auto-approves file changes), "plan" (for multi-step tasks)
- Command for checking: `/permissions`
- Best practice: Start with default; configure more permissive modes as you build trust

**Tier 2 (AI Companion Helps)**:
- Student: "I keep getting permission prompts; what options do I have?"
- Understanding: "What does this permission denial mean?"
- Decision: "Should I auto-approve file changes or keep asking each time?"

**Tier 3 (AI Orchestration)**:
- Permission policy configuration (deferred to Part 5: Team security workflows)
- Detailed allow/deny rules (deferred)
- Organization-wide permission policies (deferred)

### Content Outline

1. **Safety Rationale (1-2 min)**: Why permission prompts exist; accidents they prevent
2. **Permission Modes (2-3 min)**: Default vs acceptEdits vs plan; use cases for each
3. **Checking Permissions (1 min)**: `/permissions` command demo
4. **Working with Prompts (1-2 min)**: How to respond (approve/deny/edit)
5. **Best Practices (1 min)**: Start permissive only after understanding implications

### Key Concepts (Cognitive Load Check)

- Permission safety concept (1 concept)
- Three permission modes (default, acceptEdits, plan) (2 concepts)
- Checking current settings (`/permissions`) (3 concepts)
- Risk understanding (4 concepts: why each prompt appears)
- Best practice judgment (5 concepts)

**Cognitive Load**: 5 concepts within A2 limit of 7 ✓

### Content Elements

**Permission Modes Comparison**:

| Mode | Behavior | Use Case | Risk |
|------|----------|----------|------|
| **default** | Asks every file edit, every command | Learning/cautious work | Slower workflow; safe |
| **acceptEdits** | Auto-approves file changes; still asks for bash | Trusted workflows | Could miss mistakes in code |
| **plan** | Uses Plan mode for multi-step tasks | Complex refactors | Orchestrated action may have errors |

**Permission Prompt Example**:
```
Claude Code: I'm going to update app.py to add the new authentication handler.
Approve this change?
  > Approve  /  Deny  /  Edit  /  Cancel
```

### Practice Approach

**Observation in Lesson 5**:
- When you created hello.py, Claude Code likely asked permission
- You approved (or may have denied)
- This is the permission system in action

**Interactive in Lesson 7**:
- Run `/permissions` to see current settings
- Discuss: Do you want default (safer) or acceptEdits (faster)?

### End-of-Lesson: Try With AI

**Tool**: Claude Code
**Duration**: ~3-4 minutes total

#### Prompt 1: Check Current Permissions
```
claude "/permissions"
```

**Expected Outcome**: Claude Code shows current permission settings. Student sees: "Here's what I have permission to do."

#### Prompt 2: Understand Your Settings
```
claude "Explain my current permission settings. Which mode am I in? What does that mean for our workflow?"
```

**Expected Outcome**: Claude Code explains current settings in student's context. Personalizes safety understanding.

#### Prompt 3: Discuss Trade-offs
```
If I changed to 'acceptEdits' mode, what changes? What would I trust you to do without asking every time? What would you still ask about?
```

**Expected Outcome**: Claude Code explains trade-offs. Student decides: "I'll stay in default for now" or "I'm comfortable with acceptEdits because..."

### Success Criteria Mapping

- **SC-018**: 85% of students can explain permission rationale and use `/permissions` command
- **Eval 7**: Permission System Understanding - 85% can explain + demonstrate correctly

### Edge Cases Addressed

**"Permission Denied" Blocking Workflow**:
- Clarification: This is intentional—you have control
- Solution: Either (a) approve the action, or (b) specify different action
- Reassurance: "Permissions are designed to prevent accidents, not block legitimate work"

**Confusion between Permission Denial and Error**:
- Distinction: Permission prompt = "May I do this?" (you decide). Error = "Something went wrong" (tech issue)
- Diagnosis: "If you see a prompt, respond to it. If you see an error message, that's different."

**Concern about Memory of Permission Choices**:
- Reassurance: "If you deny a permission once, Claude Code doesn't remember permanently. Each action is independent."
- Note: "Setting mode to acceptEdits is persistent; single-action denials are not."

---

## Lesson 8: Hooks & Extensibility

**Duration**: 5-7 minutes
**Priority**: P3 (Conceptual awareness; implementation deferred)
**Prerequisites**: Lessons 1-5

### Learning Objectives

- **Objective 1** (A2 - Understand): Define hooks as automated actions triggered by events
  - Assessment: Can explain hook concept and give 2 examples
  - Bloom's Level: Understand
  - Skills Taught: "Understanding Event-Driven Automation" (A2, Conceptual)

- **Objective 2** (A2 - Understand): Recognize common hook events (PreToolUse, PostToolUse, SessionStart, SessionEnd)
  - Assessment: Can list and briefly describe 3-4 hook events
  - Bloom's Level: Understand
  - Skills Taught: "Recognizing Extensibility Hooks" (A2, Conceptual)

- **Objective 3** (A2 - Understand): Evaluate importance of extensibility for professional workflows
  - Assessment: Reflection on automation opportunities in own work
  - Bloom's Level: Understand/Analyze
  - Skills Taught: "Strategic Automation Thinking" (A2, Soft)

### Teaching Tier Mapping

**Tier 1 (Book Teaches Directly)**:
- Hooks concept: Automated actions triggered by events
- Event types: PreToolUse (before executing command), PostToolUse (after executing), SessionStart (when session opens), SessionEnd (when session closes)
- Real-world examples: Format code after edit, run tests after save, load env vars at session start
- Why extensibility matters: Professional workflows need automation beyond basic usage
- Limitation: Building hooks is Part 5/6 content (not now)

**Tier 2 (AI Companion Helps)**:
- Student: "What hooks would be useful for my workflow?"
- Claude Code suggests: "I'd recommend a PostToolUse hook to run linters after edits"
- Example: Show hook configuration (conceptual, not implementation)

**Tier 3 (AI Orchestration)**:
- Building custom hooks (deferred to Part 5)
- Orchestrating multiple hooks (deferred)
- Hook error debugging (deferred)

### Content Outline

1. **Hooks Concept (1-2 min)**: Automated actions triggered by events
2. **Event Types (1-2 min)**: PreToolUse, PostToolUse, SessionStart, SessionEnd
3. **Real-World Examples (1-2 min)**: Format code, run tests, load config, start servers
4. **Why It Matters (1 min)**: Professional workflows automate repetition
5. **Building Custom (1 min)**: Mentioned briefly (Part 5/6 content)

### Key Concepts (Cognitive Load Check)

- Hooks as automated actions (1 concept)
- Event-triggered patterns (2 concepts: event definition, action execution)
- Common event types (3 concepts: PreToolUse, PostToolUse, Session events)
- Real-world benefits (4 concepts: reduces repetition, improves consistency, ensures quality)
- Future extensibility capability (5 concepts)

**Cognitive Load**: 5 concepts within A2 limit of 7 ✓

### Content Elements

**Hook Examples**:

**Example 1: Format Code After Edit**
```yaml
Hook: PostToolUse
Event: Every time file is edited
Action: Run formatter (prettier, black, etc.)
Benefit: Code automatically stays formatted; never need to manually run formatter
```

**Example 2: Run Tests After Save**
```yaml
Hook: PostToolUse
Event: Python file saved
Action: Run pytest on that file
Benefit: Immediate feedback on whether changes broke tests
```

**Example 3: Load Environment at Start**
```yaml
Hook: SessionStart
Event: Claude Code session starts
Action: Load .env and configure API keys
Benefit: Don't need to run config commands manually each session
```

### Practice Approach

**Reflection Task**:
- What repetitive tasks do you do in your workflow?
- Examples: "I always run tests after editing...", "I always format code before committing..."
- Those are hook candidates

### End-of-Lesson: Try With AI

**Tool**: Claude Code
**Duration**: ~3-4 minutes total

#### Prompt 1: Understand Hooks
```
claude "Explain what hooks are in Claude Code. Give me 2-3 real-world examples of hooks that would be useful in a development workflow."
```

**Expected Outcome**: Claude Code explains hooks with practical examples. Student sees: "Oh, I could automate [my repetitive task]."

#### Prompt 2: Identify Opportunities
```
Think about your typical workflow. What do you do repeatedly that could be automated with a hook? Tell me, and I'll explain how a hook could help.
```

**Expected Outcome**: Claude Code suggests hook solution for student's actual workflow. Makes it personal, not abstract.

#### Prompt 3: Future Learning Path
```
Can I build custom hooks? Is it hard? When would I learn that? What prerequisites do I need?
```

**Expected Outcome**: Claude Code explains: "Building hooks is Part 5/6 content. You'll learn enough in this book to build them by Chapter 33-34. Builds on concepts you're learning now."

### Success Criteria Mapping

- **SC-019**: 80% can explain hooks and give 2 examples
- **Eval 8**: Extensibility Awareness (partially; full coverage includes Lesson 9 settings)

### Edge Cases Addressed

**Confusion between Hooks and Skills/MCP**:
- **Hooks**: Automated actions triggered by events IN Claude Code
- **Skills/MCP**: Integrations connecting Claude Code to EXTERNAL tools
- Clarification: "Hooks automate Claude Code behavior; Skills extend Claude Code's capabilities"

**Expectation of Building Hooks Now**:
- Redirect: "Building custom hooks is Part 5 content (Chapter 31-33). For now, knowing they exist and what they do is the goal."
- Reassurance: "You'll build hooks by Chapter 34-35 once you understand the architecture."

**Hook Errors During Session**:
- Clarification: "Hook errors are usually non-blocking. Claude Code continues working even if a hook fails."
- Reassurance: "Debugging hook errors is advanced content. If a hook breaks, we'll address it then."

---

## Lesson 9: Settings Hierarchy

**Duration**: 4-6 minutes
**Priority**: P3 (Awareness; detailed configuration deferred)
**Prerequisites**: Lessons 1-6

### Learning Objectives

- **Objective 1** (A2 - Understand): Explain the three-level settings hierarchy (user/project/local)
  - Assessment: Can describe hierarchy and give example of how it works
  - Bloom's Level: Understand
  - Skills Taught: "Understanding Configuration Hierarchy" (A2, Conceptual)

- **Objective 2** (A2 - Understand): Identify precedence order (local > project > user) and why it matters
  - Assessment: Can explain: "If same setting exists in 2 places, [correct answer] takes priority"
  - Bloom's Level: Understand
  - Skills Taught: "Understanding Configuration Precedence" (A2, Conceptual)

- **Objective 3** (A2 - Understand): Recognize that settings files exist but detailed configuration is Part 5 content
  - Assessment: Can explain: "Settings control Claude Code behavior; I can configure them later for team workflows"
  - Bloom's Level: Understand
  - Skills Taught: "Recognizing Configuration Scope" (A2, Conceptual)

### Teaching Tier Mapping

**Tier 1 (Book Teaches Directly)**:
- Settings hierarchy: User level (`~/.claude/settings.json`), Project level (`.claude/settings.json`), Local level (`.claude/settings.local.json`)
- Precedence order: Local > Project > User (more specific overrides general)
- Why this matters: Team settings, personal preferences, project-specific customization
- Safety note: Don't edit settings.json unless you understand the structure
- Configuration scope: Detailed configuration is Part 5 (team workflows)

**Tier 2 (AI Companion Helps)**:
- Student: "Where is Claude Code configuration stored?"
- Understanding: "Which settings file applies in my situation?"
- Decision: "Should I modify user or project settings?"

**Tier 3 (AI Orchestration)**:
- Advanced settings configuration (deferred to Part 5)
- Team settings policies (deferred)
- Settings as code patterns (deferred)

### Content Outline

1. **Settings Concept (1 min)**: Configuration files that customize Claude Code behavior
2. **Hierarchy Levels (1-2 min)**: User (~/.claude/), Project (.claude/), Local (.claude/)
3. **Precedence Order (1 min)**: Local > Project > User
4. **Why It Matters (1 min)**: Team collaboration, personal customization, project safety
5. **Not Configuring Now (1 min)**: Detailed configuration is Part 5

### Key Concepts (Cognitive Load Check)

- Settings files customize behavior (1 concept)
- Three hierarchy levels (user, project, local) (2 concepts)
- Precedence order (local overrides project overrides user) (3 concepts)
- Directory locations (4 concepts)
- Configuration scope deferred to Part 5 (5 concepts)

**Cognitive Load**: 5 concepts within A2 limit of 7 ✓

### Content Elements

**Hierarchy Visualization**:
```
USER LEVEL (General)
~/.claude/settings.json
├─ Applies to ALL Claude Code projects
├─ Your personal preferences
└─ Example: default_model, theme

PROJECT LEVEL (Specific)
./.claude/settings.json
├─ Applies to THIS project only
├─ Team conventions
└─ Example: permission_mode, code_style_rules

LOCAL LEVEL (Override)
./.claude/settings.local.json
├─ Applies to THIS project, THIS computer only
├─ Sensitive overrides (API keys, credentials)
└─ Example: local_credentials, testing_flags

PRECEDENCE: local > project > user
(More specific overrides more general)
```

**Example Scenario**:
```
User level: permission_mode = "default" (ask every time)
Project level: permission_mode = "acceptEdits" (auto-approve file changes)
Local level: (not set)

Result: In this project, acceptEdits wins (project overrides user)
On a different project: User's "default" applies (no project override)
```

### Practice Approach

**Awareness Only**:
- Know the files exist
- Know where they're located
- Know the precedence order
- Don't edit them now (Part 5 content)

### End-of-Lesson: Try With AI

**Tool**: Claude Code
**Duration**: ~3-4 minutes total

#### Prompt 1: Explain Settings Hierarchy
```
claude "Explain Claude Code's settings hierarchy. Where are settings stored? What's the precedence order? I'm not configuring yet; I just want to understand how it works."
```

**Expected Outcome**: Claude Code explains hierarchy clearly. Student understands: "Settings exist at 3 levels; more specific overrides more general."

#### Prompt 2: Show Your Settings
```
claude "Show me the settings files that currently apply to my projects. Which ones exist? Where?"
```

**Expected Outcome**: Claude Code shows actual settings files in student's environment. Concrete, not abstract.

#### Prompt 3: Plan for Later
```
When I want to configure Claude Code for my team (in Part 5), what settings would I modify? Project level or user level?
```

**Expected Outcome**: Claude Code guides thinking: "Team settings go in project level; personal preferences in user level." Sets context for Part 5.

### Success Criteria Mapping

- **SC-020**: 80% can explain hierarchy and precedence order
- **Eval 8**: Extensibility Awareness (combined with Lesson 8 hooks understanding)

### Edge Cases Addressed

**Confusion about .claude/ Directory**:
- Clarification: ".claude/ directory holds Claude Code configuration and state files"
- Not to delete: "Don't delete this directory; Claude Code needs it"
- Safe to review: "You can look at settings.json to understand configuration"

**Broke Configuration by Editing**:
- Recovery: "If settings.json syntax is broken, rename it temporarily to reset Claude Code"
- Best practice: "Make backups before editing; validate JSON syntax after changes"
- Help: "Ask Claude Code to validate your settings JSON"

**Settings Not Applying**:
- Diagnosis: "Restart Claude Code to reload settings"
- Precedence confusion: "Check which level you edited; remember local > project > user"

---

## Content Flow & Dependencies

### Linear Lesson Progression

**Foundation Phase (Lessons 1-2, ~20 min)**:
- Lesson 1: Understand paradigm shift (why Claude Code matters)
- Lesson 2: Install and authenticate (get it working)
- **Gate**: Ability to run Claude Code locally

**Concept Phase (Lessons 3-4, ~15 min)**:
- Lesson 3: Subagents explain orchestration strategy
- Lesson 4: Skills/MCP explain extensibility
- **Gate**: Understand "bigger picture" features (not just basic usage)

**Practice Phase (Lesson 5, ~10-12 min)**:
- Lesson 5: Hello World hands-on workflow
- **Gate**: Confidence in basic workflow; understand human-AI role split

**Advanced Concepts Phase (Lessons 6-9, ~25-30 min)**:
- Lesson 6: CLAUDE.md for persistent context (foundational practice)
- Lesson 7: Permissions for safety understanding
- Lesson 8: Hooks for automation awareness
- Lesson 9: Settings hierarchy for configuration awareness
- **Gate**: Conceptual preparation for Part 5 (SDD, team workflows)

### Cross-Lesson Dependencies

```
Lesson 1 (Paradigm)
    ↓ (Why Claude Code matters)
Lesson 2 (Installation)
    ↓ (Now you can use it)
├→ Lesson 3 (Subagents) [conceptual, depends on Lesson 1-2]
├→ Lesson 4 (Skills/MCP) [conceptual, depends on Lesson 1-2]
├→ Lesson 5 (Hello World) [hands-on, depends on Lesson 1-2, informs Lessons 6-9]
    ↓ (Confidence to explore more)
├→ Lesson 6 (CLAUDE.md) [foundational practice, depends on Lesson 5]
├→ Lesson 7 (Permissions) [safety understanding, depends on Lesson 2-5]
├→ Lesson 8 (Hooks) [extensibility awareness, depends on Lesson 1-5]
└→ Lesson 9 (Settings) [configuration awareness, depends on Lesson 1-2]
    ↓
Part 2 Chapters 6-8 (Other tools, Git, Bash)
    ↓
Part 5 (SDD) - Use Claude Code at scale with specifications
```

### Prerequisite Satisfaction

- **Lessons 1-2**: Required before anything else (installation blocker)
- **Lessons 3-4**: Optional for basic usage, important for understanding tool scope
- **Lesson 5**: Required for building confidence; all subsequent lessons reference it
- **Lessons 6-9**: Can be done in any order after Lesson 5; each stands alone conceptually
- **Cognitive sequencing**: 6-9 gets progressively more "advanced" (context → safety → automation → configuration)

---

## Scaffolding Strategy

### Complexity Progression (A2 Tier)

**Lessons 1-2 (Concrete, Direct)**:
- Define concepts clearly (passive vs agentic)
- Step-by-step instructions
- Direct commands, not abstract theory
- Immediate verification (it works or it doesn't)
- Cognitive load: Paradigm + Installation = 2 major concepts

**Lesson 3-4 (Conceptual, with Examples)**:
- Introduce abstract ideas (subagents, MCP)
- Ground with real examples
- Not requiring hands-on practice yet
- Why it matters (future capability preview)
- Cognitive load: Orchestration + Extensibility = 2 new concepts

**Lesson 5 (Hands-On Integration)**:
- Apply concepts from Lessons 1-2 in practice
- Observe workflow in action
- Build confidence through success
- Understand human-AI collaboration concretely
- Cognitive load: Applied learning (retrieves prior concepts)

**Lessons 6-9 (Layered Conceptual Deepening)**:
- Each lesson adds one capability layer
- Connections to earlier lessons (CLAUDE.md uses installation knowledge, etc.)
- Optional for basic usage; required for Part 5 understanding
- Progressive sophistication (context → safety → automation → configuration)
- Cognitive load: 4-5 concepts per lesson, within A2 limit

### Cognitive Load Management

**Per-Lesson Limit**: Max 7 concepts per lesson (A2 tier), verified in each lesson outline

**Across-Chapter Pattern**:
- Direct instruction (Lesson 1-2): Define concepts, show examples
- Conceptual exploration (Lessons 3-4): "What's possible?" framing
- Hands-on practice (Lesson 5): Apply foundational concepts
- Specialized features (Lessons 6-9): Each stands alone; can explore in own time

**Breaks and Pacing**:
- Each lesson is 4-12 minutes (not overwhelming)
- Try with AI sections provide natural pause points
- Reflection prompts allow processing time
- No lesson requires sustained focus > 10 minutes

### Engagement Strategies

**For Skeptics**:
- Lesson 1: Show real data (YC 95% AI-generated code)
- Lesson 5: "Hello World" success builds conviction
- Lessons 6-9: "Here's what professionals use"

**For Eager Learners**:
- Lesson 3: Introduce subagents/orchestration early
- Lesson 4: MCP as "future capability" forward reference
- Lesson 5: "You can do this now!" confidence boost
- Lessons 6-9: "Here's what comes next in your journey"

**For Skeptical about AI**:
- Lesson 2: Emphasizes human control (installation manual, authentication human-driven)
- Lesson 7: Permissions frame safety-first approach
- Lesson 5: Human specifies; Claude executes; human verifies (human as director)

---

## Integration Points

### Bridge from Part 1 (Chapters 1-4)

**Explicit References**:
- Lesson 1: "You now understand why AI development matters. Claude Code is THE FIRST TOOL you'll use."
- Lesson 2: "Remember the Nine Pillars? Claude Code is Pillar 1—AI CLI & Coding Agents."
- Lesson 5: "This workflow (specify → execute → validate) is the AI-Driven methodology from Chapter 4."

**Conceptual Continuity**:
- Part 1 = Mindset + Why AI matters
- Part 2 = Tool literacy (THIS CHAPTER: Claude Code)
- Part 2 Chapters 6-8 = Other tools
- Part 5 = Deep SDD methodology (builds on Chapter 4 + Chapter 5 tool fluency)

### Prerequisites Verified

- ✅ Chapter 1: Understand $3 trillion disruption, why AI matters
- ✅ Chapter 2: AI is real change, not hype
- ✅ Chapter 3: Opportunity in AI (addressed career concerns)
- ✅ Chapter 4: Nine Pillars and AIDD framework
- **Chapter 5 Assumption**: All above completed; ready for first tool

### Bridge to Part 2 Chapters 6-8

**Chapter 6 (Gemini CLI)**:
- "You learned Claude Code (file-aware, integrated execution)"
- "Gemini CLI offers different capabilities and UI philosophy"
- Comparison framework from Lesson 1 applies

**Chapter 7 (Bash)**:
- "You've been running commands through Claude Code"
- "Understanding bash natively helps you direct AI more effectively"
- Hands-on in Lesson 5 + Lesson 2 installation previewed this

**Chapter 8 (Git)**:
- "Claude Code sees your git state (uncommitted changes, branches)"
- "Understanding Git helps you work with Claude Code's codebase awareness"
- Part 2 conclusion: Full developer toolkit

### Bridge to Part 5 (Spec-Driven Development)

**Conceptual Continuity**:
- Chapter 4 (Nine Pillars): "Spec-first development" is mentioned
- Chapter 5 (Claude Code): You experience spec → Claude executes workflow in Lesson 5
- Lesson 3 (Subagents): Introduces orchestration concept
- Chapter 30-33 (Part 5): Deep SDD methodology using Claude Code at scale

**Tool Fluency Prerequisite**:
- Part 5 assumes: You can use Claude Code comfortably (Chapters 5-8 establish this)
- Part 5 builds on: Specification-first thinking + tool orchestration (Chapter 5 Lesson 3 previews)
- Readiness check: By end of Chapter 8, students can orchestrate multiple tools for a project

---

## Validation Strategy

### How Learners Demonstrate Understanding

**Lesson 1 (Paradigm Shift)**:
- Assessment: Scenario-based reflection ("When would you use Claude Code vs ChatGPT?")
- Success: Can articulate 3 differences with concrete examples

**Lesson 2 (Installation)**:
- Assessment: `claude --version` returns version number
- Success: Installation succeeds on first attempt; authentication verified

**Lesson 3 (Subagents)**:
- Assessment: Explain Explore agent in own words after observing it in practice
- Success: Student recognizes fast search patterns as Explore agent working

**Lesson 4 (Skills/MCP)**:
- Assessment: Name 2 MCP integrations and what they enable
- Success: Student understands extensibility concept

**Lesson 5 (Hello World)**:
- Assessment: All 4 prompts execute successfully; expected outputs match
- Success: Working Python file created, run, and verified; understand human-AI role

**Lesson 6 (CLAUDE.md)**:
- Assessment: CLAUDE.md file exists in project root; contains appropriate sections
- Success: File auto-loads on next session; student sees context persistence benefit

**Lesson 7 (Permissions)**:
- Assessment: `/permissions` command executed; output explained
- Success: Student articulates why permissions exist and can check settings

**Lesson 8 (Hooks)**:
- Assessment: Can explain hook concept and give 2 real-world examples
- Success: Student identifies automation opportunity in own workflow

**Lesson 9 (Settings)**:
- Assessment: Explain hierarchy and precedence order without prompting
- Success: Student can identify which settings file applies in different scenarios

### Success Metrics Alignment

**Student Learning Outcomes (from Spec SC-001 to SC-020)**:

| Outcome | Measured By | Target | Lesson(s) |
|---------|------------|--------|-----------|
| SC-001: 95% install successfully | `claude --version` works | 95% | Lesson 2 |
| SC-002: 90% understand paradigm | Scenario-based reflection | 90% | Lesson 1 |
| SC-003: 100% complete Hello World | All 4 prompts execute | 100% | Lesson 5 |
| SC-004: 85% identify tool strategy | Reflection on when to use Claude Code | 85% | Lesson 1 + 5 |
| SC-006: 100% platform compatibility | Test on Windows/macOS/Linux | 100% | Lesson 2 |
| SC-007: Realistic durations | Install 2-5 min, auth 1-2 min | Within 20% | Lesson 2 |
| SC-009: Three Roles applied | Lesson 5 workflow shows roles | ✓ | Lesson 5 |
| SC-010: Cognitive load respected | Max 7 concepts per lesson | ✓ All lessons | All |
| SC-011: No over-engineering | Installation is direct command, not "use AI to install" | ✓ | Lesson 2 |
| SC-012: Try with AI format | 3-4 focused prompts per lesson | ✓ | All lessons |
| SC-013: Strategic understanding | Student explains when to use Claude Code | 85% | Lessons 1 + 5 |
| SC-017: 90% create CLAUDE.md | File exists with appropriate content | 90% | Lesson 6 |
| SC-018: 85% explain permissions | Can articulate safety rationale | 85% | Lesson 7 |
| SC-019: 80% explain hooks | Can give 2 hook examples | 80% | Lesson 8 |
| SC-020: 80% explain settings | Can describe hierarchy and precedence | 80% | Lesson 9 |

### Evals Alignment

| Eval | Definition | Measured By | Lesson(s) | Target |
|------|-----------|-------------|-----------|--------|
| Eval 1: Installation Success | % completing installation without help | Sandbox testing + survey | Lesson 2 | 95% |
| Eval 2: Conceptual Understanding | Paradigm shift (passive vs agentic) | Scenario quiz | Lesson 1 + 3 | 90% |
| Eval 3: Hands-On Completion | Hello World prompts execute correctly | Sandbox validation | Lesson 5 | 100% |
| Eval 4: AI Usage Strategy | When to use Claude Code vs alternatives | Reflection prompt | Lesson 1 + 5 | 85% |
| Eval 5: Technical Accuracy | Zero hallucinations in content | Technical review | All | 0 errors |
| Eval 6: CLAUDE.md Creation | File creation and auto-loading | Sandbox validation | Lesson 6 | 90% |
| Eval 7: Permission Understanding | Safety rationale and `/permissions` usage | Quiz + practical | Lesson 7 | 85% |
| Eval 8: Extensibility Awareness | Hooks and settings conceptual understanding | Reflection | Lessons 8 + 9 | 80% |

### Quality Gates

**Before Publishing Chapter**:
1. ✅ All 9 lessons have clear learning objectives and success criteria
2. ✅ Tier 1/2/3 teaching strategy explicit and justified per lesson
3. ✅ Installation tested on all 3 platforms (Windows/macOS/Linux) in sandbox
4. ✅ All Hello World prompts execute successfully with expected outputs
5. ✅ "Try with AI" format follows Chapter 1 clean style (3-4 prompts, focused)
6. ✅ Durations verified realistic (not inflated)
7. ✅ All success criteria (SC-001 to SC-020) can be measured by lesson assessments
8. ✅ All evals (Eval 1-8) have explicit measurement methods
9. ✅ No over-engineering: Installation is direct command, not elaborate AI workflow
10. ✅ Constitutional alignment verified (Principles 13, 18; AI Spectrum; Cognitive Load)

---

## Chapter Summary for Instructors

**What This Chapter Teaches**:
1. **Paradigm**: Why Claude Code (agentic, context-aware, integrated) differs from ChatGPT web (passive, stateless, chat interface)
2. **Installation**: How to get Claude Code running on their platform with proper authentication
3. **Concepts**: What subagents, Skills, MCP, CLAUDE.md, permissions, hooks, and settings hierarchy are
4. **Practice**: Hands-on Hello World workflow showing human-AI collaboration
5. **Preparation**: Conceptual foundation for Part 5 (SDD methodology) and Part 2 continuation (Gemini, Git, Bash)

**What This Chapter Does NOT Teach**:
- ❌ Deep MCP server development (Chapter 39, Part 6)
- ❌ Custom subagent building (Chapter 32, Part 5)
- ❌ Detailed settings configuration for teams (Part 5)
- ❌ Advanced CLAUDE.md patterns (Part 5/6)
- ❌ Building custom hooks (Part 5)
- ❌ Permission policy configuration (Part 5)

**Time Investment for Instructors**:
- Review + validation: 2-3 hours (read all lessons, test installation, verify all prompts)
- Content creation: Will be implemented in Phase 2 (lessons written from this plan)
- Testing: 1-2 hours per platform (3 platforms = 3-6 hours total for installation validation)
- Iteration: 1-2 hours for refinements based on beta reader feedback

**Readiness for Implementation**:
- ✅ Plan is complete and detailed
- ✅ All learning objectives defined and aligned to CEFR/Bloom's
- ✅ Graduated teaching tiers explicit per lesson
- ✅ Success criteria and evals mapped
- ✅ Edge cases identified and addressed
- ✅ Constitutional alignment verified
- ✅ Ready for Phase 2: Task decomposition and content writing

---

**Plan Status**: Complete and approved for Phase 2 (Task Decomposition)

**Next Step**: Generate tasks.md for implementation (Lesson-writing, code example creation, exercise design, review)
