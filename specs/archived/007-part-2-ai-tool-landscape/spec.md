# Feature Specification: Part 2 - AI Tool Landscape

**Feature Branch**: `007-part-2-ai-tool-landscape`
**Created**: 2025-10-30
**Status**: In Review
**Input**: User description: "Plan part-2 for our book covering AI Tool Landscape with 4 chapters on Claude Code, Gemini CLI, Bash, and Git."

---

## Part 2 Overview

**Purpose**: Part 2 introduces learners to the practical AI development tools and fundamental CLI/version control skills required for AI-driven development. It bridges the conceptual foundation from Part 1 to the hands-on development skills needed in Parts 3-7.

**Target Audience**: Developers new to AI-assisted coding who have completed Part 1 (or understand core AI concepts). Assumes basic computer literacy but no prior experience with CLI tools, Git, or AI development platforms.

**Book Context**:
- **Part 1 (Chapters 1-4)**: Foundation—AI transformation, opportunities, challenges, and mindset
- **Part 2 (Chapters 5-8)**: Tool literacy—Claude Code, Gemini CLI, Bash, Git (THIS SPEC)
- **Part 3+**: Python fundamentals and advanced programming with AI assistance

## User Scenarios & Testing *(mandatory)*

### User Story 1: New Developer Gets Started with Claude Code (Priority: P1)

**Scenario**: A developer completing Part 1 wants to immediately put AI tools into practice. They need to understand what Claude Code is, how to install it, and how to execute their first AI-assisted task.

**Why this priority**: This is the primary hook. If learners can't get Claude Code working and experience immediate value, they abandon the book. Claude Code is featured first and is the most fully-featured AI development platform covered.

**Independent Test**: Reader completes Claude Code installation, creates a simple test project, and executes one subagent task. Delivers concrete value: "I used an AI tool to write code autonomously."

**Acceptance Scenarios**:
1. **Given** a reader has finished Part 1, **When** they follow Chapter 5 instructions, **Then** Claude Code is installed and working
2. **Given** Claude Code is installed, **When** they follow the Quick Start example, **Then** they generate working code with subagents
3. **Given** a working Claude Code installation, **When** they enable Skills, **Then** they understand custom skill creation for domain advantage

---

### User Story 2: Developer Understands Gemini CLI as Open Alternative (Priority: P1)

**Scenario**: A developer learns that Gemini CLI is free, open-source, and extensible. They want to understand its unique value proposition, how it differs from Claude Code, and when to use it.

**Why this priority**: Principle 11 (Tool Diversity) requires honest comparison. Gemini CLI's free tier and open-source nature make it critical for accessible learning and for developers with budget constraints. Extensions (similar to Claude Code Skills) create competitive advantage.

**Independent Test**: Reader installs Gemini CLI, authenticates with Google account, executes a command, and understands its strengths vs. Claude Code.

**Acceptance Scenarios**:
1. **Given** a reader has Part 1 foundation, **When** they read Chapter 6, **Then** they understand Gemini CLI's free tier and rate limits
2. **Given** Gemini CLI installed, **When** they authenticate and execute a CLI command, **Then** it works and they see extension capabilities
3. **Given** understanding of both tools, **When** they review the comparison, **Then** they can articulate when to use each tool

---

### User Story 3: Developer Becomes Productive with Bash for AI Development (Priority: P1)

**Scenario**: A developer realizes they need to be comfortable with command-line tools to work effectively with Claude Code and Gemini CLI. They need practical Bash knowledge focused on AI development workflows, not comprehensive system administration.

**Why this priority**: Bash literacy is prerequisite for using AI tools effectively. Many readers will have never used CLI before. Part 2 must establish this foundation early. Focus is laser-targeted: navigation, file operations, package management, environment variables, and AI-specific patterns.

**Independent Test**: Reader completes 3-5 Bash commands independently (navigate directories, create files, manage environment variables) and uses AI tools to execute others confidently.

**Acceptance Scenarios**:
1. **Given** no prior CLI experience, **When** they complete Chapter 7 Part I, **Then** they can navigate directories, list files, and create projects
2. **Given** basic Bash knowledge, **When** they use natural language prompts in Chapter 7 Part II, **Then** they can request AI tools to execute Bash commands
3. **Given** Chapter 7 completion, **When** they set up API keys and environment variables, **Then** Claude Code and Gemini CLI work correctly

---

### User Story 4: Developer Understands Git & GitHub for AI-Driven Safety (Priority: P1)

**Scenario**: A developer learns that version control is CRITICAL when working with AI tools—you need checkpoints to undo AI mistakes, safe branches to experiment, and PRs to review AI-generated code before merging. Git becomes a safety net and portfolio tool.

**Why this priority**: This is non-negotiable safety training. When learners let AI tools modify code, they must have Git discipline. PRs enable code review culture. GitHub becomes portfolio for job seekers. This establishes professional workflows from the start.

**Independent Test**: Reader creates a local Git repository, makes commits, creates branches, opens a pull request, and can explain how to safely experiment with AI-generated code.

**Acceptance Scenarios**:
1. **Given** no prior Git experience, **When** they complete Chapter 8 Part I, **Then** they can initialize repos, commit, push, and create branches
2. **Given** basic Git knowledge, **When** they practice the AI safety workflows, **Then** they create checkpoint commits before AI changes and can rollback if needed
3. **Given** Git basics mastered, **When** they open a PR, **Then** they document what AI generated and can request review from teammates

---

### Edge Cases

- **Offline scenarios**: What if Claude Code API is down? How does Gemini CLI with free tier behave under rate limits? (Addressed: Principle 11 includes fallback strategies)
- **Platform differences**: Windows, Mac, Linux have different CLI behaviors. (Addressed: Each tool section includes platform-specific instructions)
- **Incomplete setup**: What if reader can't install tools due to corporate restrictions or permissions? (Addressed: Troubleshooting sections, fallback guidance)
- **Tool version changes**: AI tools update rapidly. How do we keep this current? (Addressed: Update triggers noted in Constitution Section II.C)

## Requirements *(mandatory)*

### Functional Requirements

**Chapter 5: Claude Code Phenomenon**

- **FR-501**: Readers MUST understand Claude Code's origin story, why it "changed everything," and its significance in AI development culture
- **FR-502**: Readers MUST be able to install Claude Code (CLI and web interface) on their platform
- **FR-503**: Readers MUST understand and use Subagents—specialized task delegates for parallel workflows
- **FR-504**: Readers MUST understand Agent Skills—custom domain-specific capabilities—and how to create them for competitive advantage
- **FR-505**: Readers MUST understand Output Styles and how to customize how AI presents information
- **FR-506**: Readers MUST be able to connect MCP servers (Model Context Protocol) to extend Claude Code functionality
- **FR-507**: Readers MUST practice with at least one complete Claude Code workflow from official documentation

**Chapter 6: Gemini CLI**

- **FR-601**: Readers MUST understand Gemini CLI's open-source nature, free tier, and rate limits (2000 requests/day free)
- **FR-602**: Readers MUST understand how Gemini CLI differs from Claude Code and when to choose each tool
- **FR-603**: Readers MUST be able to install and authenticate Gemini CLI with a personal Google account
- **FR-604**: Readers MUST understand Gemini CLI Extensions (conceptually similar to Claude Code Skills) and their competitive advantage
- **FR-605**: Readers MUST understand the relationship between Gemini CLI, Qwen Code (Alibaba fork), and ecosystem diversity
- **FR-606**: Readers MUST practice at least one Gemini CLI workflow end-to-end
- **FR-607**: Readers MUST understand Gemini 2.5 Pro's 1 million token context window and its implications for codebase analysis

**Chapter 7: Bash Essentials**

- **FR-701**: Readers with no CLI experience MUST learn core commands: pwd, cd, ls, mkdir, touch, rm, cp, mv
- **FR-702**: Readers MUST understand file operations: cat, head, tail, less, nano, and redirection (>, >>)
- **FR-703**: Readers MUST understand process management: running commands, background execution (&), killing processes
- **FR-704**: Readers MUST master environment variables for API keys and configuration (export, echo, env, .bashrc)
- **FR-705**: Readers MUST understand package managers appropriate to their platform: Homebrew (Mac), apt (Linux), pip/npm (language-specific)
- **FR-706**: Readers MUST understand command history and shortcuts (Ctrl+R, !!, Tab autocomplete)
- **FR-707**: Readers MUST understand pipes and command chaining (|, &&, >)
- **FR-708**: Readers MUST be able to use natural language prompts to request AI tools execute Bash commands
- **FR-709**: Readers MUST have a troubleshooting guide for common Bash issues (command not found, permission denied, API key issues)

**Chapter 8: Git & GitHub**

- **FR-801**: Readers MUST understand why Git is critical when working with AI tools (checkpoint before changes, undo safety, code review)
- **FR-802**: Readers MUST be able to initialize repositories, configure Git identity, and understand remote connections
- **FR-803**: Readers MUST master the daily workflow: git status → git add → git commit → git push → git pull
- **FR-804**: Readers MUST understand branching for safe experimentation and AI-generated code testing
- **FR-805**: Readers MUST understand and practice pull requests with code review, ideal for reviewing AI-generated changes
- **FR-806**: Readers MUST understand merge conflicts and how to resolve them (especially when AI and human changes collide)
- **FR-807**: Readers MUST understand GitHub authentication (tokens, SSH keys) and security best practices
- **FR-808**: Readers MUST understand .gitignore and how to prevent committing secrets, dependencies, and AI tool caches
- **FR-809**: Readers MUST practice GitHub as portfolio (projects are visible, demonstrate professional workflow)
- **FR-810**: Readers MUST be able to use natural language prompts to request AI tools execute Git workflows
- **FR-811**: Readers MUST understand Git workflows specific to AI development: checkpoint commits, branch-per-experiment, PR review of generated code

---

### Key Entities

**Chapter 5 Concepts**:
- **Claude Code**: AI-driven development assistant with subagents, skills, output styles, and MCP integration
- **Subagent**: Specialized autonomous agent delegated specific tasks in parallel workflows
- **Agent Skill**: Custom domain/company-specific capability extending Claude Code performance
- **Output Style**: Template controlling how Claude Code presents information
- **MCP Server**: External tools/integrations extending Claude Code capabilities via Model Context Protocol

**Chapter 6 Concepts**:
- **Gemini CLI**: Open-source, free-tier AI development CLI tool with extensions
- **Gemini 2.5 Pro**: Model powering Gemini CLI with 1M token context window
- **Extensions**: Gemini CLI equivalent to Agent Skills—extensible capabilities
- **Rate Limits**: 60 requests/minute, 2000/day free; comparison to Claude Code's free tier

**Chapter 7 Concepts**:
- **Shell/Bash**: Command-line interface for system interaction
- **Environment Variable**: Named value (API_KEY, PATH, NODE_ENV) accessible to programs
- **Package Manager**: Tool for installing/updating software (brew, apt, pip, npm)
- **Command Piping**: Connecting command outputs (|) to create workflows
- **Redirection**: Saving/appending command output to files (>, >>)

**Chapter 8 Concepts**:
- **Repository**: Project version control container (local or remote)
- **Commit**: Snapshot of code changes with message
- **Branch**: Parallel development line for features/experiments
- **Pull Request**: Proposal to merge changes with code review
- **Remote**: Connection to hosted repository (GitHub origin)
- **.gitignore**: File specifying what to exclude from version control

## Success Criteria *(mandatory)*

### Measurable Outcomes

**By end of Chapter 5** (Claude Code):
- **SC-501**: Reader successfully installs Claude Code and confirms working with `claude --version`
- **SC-502**: Reader executes one subagent-based task and understands parallel delegation
- **SC-503**: Reader understands Agent Skills concept and can explain potential competitive advantage

**By end of Chapter 6** (Gemini CLI):
- **SC-601**: Reader authenticates with Google account and executes first Gemini CLI command
- **SC-602**: Reader articulates at least 2 differences between Claude Code and Gemini CLI
- **SC-603**: Reader understands free tier rate limits and caching strategies

**By end of Chapter 7** (Bash):
- **SC-701**: Reader completes all 5 basic navigation commands independently (pwd, cd, ls, mkdir, touch)
- **SC-702**: Reader can set and verify environment variables (export, echo, env)
- **SC-703**: Reader successfully requests AI tool to execute 3+ Bash commands from natural language prompts
- **SC-704**: Reader can troubleshoot one common Bash error without assistance

**By end of Chapter 8** (Git & GitHub):
- **SC-801**: Reader initializes local repository, makes 3+ commits, and pushes to GitHub
- **SC-802**: Reader creates a feature branch, commits changes, and opens pull request
- **SC-803**: Reader can explain how to safely test AI-generated code using branches
- **SC-804**: Reader demonstrates rollback using git reset or checkout without AI assistance
- **SC-805**: Reader has public GitHub profile with evidence of learning (commits, PRs)

**Overall Part 2 Outcomes**:
- **SC-Part-2-A**: 90% of readers can set up and use at least ONE AI development tool (Claude Code or Gemini CLI) independently
- **SC-Part-2-B**: 100% of readers understand Git workflows required for safe AI code collaboration
- **SC-Part-2-C**: Reader sentiment: "I can now use AI development tools confidently" (survey target: 85%+ agreement)
- **SC-Part-2-D**: All external tool links live and current (quarterly verification)
- **SC-Part-2-E**: Code examples tested on Windows, Mac, Linux platforms

---

## Content Structure & Learning Progression

### Chapter 5: How It All Started - The Claude Code Phenomenon

**Themes**:
- Origin story (Anthropic didn't plan to build it, but it changed everything)
- Revolutionary capabilities (code reading, file modification, tool use, safe execution)
- 90% of Claude Code written by AI—implications
- From CLI to web interface—accessibility evolution
- Checkpoints, subagents, agent skills, hooks—modern features enabling complex workflows
- MCP integration enabling vertical skill libraries for competitive advantage

**Key Sections**:
1. **Origin & Impact** — PCMag article, YouTube coverage, what made it revolutionary
2. **Installation & Setup** — Command-line and web interface options
3. **Quickstart Workflow** — First Claude Code task, approval modes
4. **Subagents Deep Dive** — Parallel task delegation, when to use
5. **Agent Skills** — Creating vertical domain libraries for competitive advantage (core teaching point)
6. **Output Styles** — Customizing presentation format
7. **MCP Servers** — Connecting external tools (Git, databases, APIs)
8. **Common Workflows** — Copy from official docs with explanation

**Learning Outcomes**:
- Understand Claude Code's revolutionary role in AI development
- Install and configure Claude Code for productive use
- Execute parallel workflows using subagents
- Create custom agent skills for domain advantage
- Connect external tools via MCP

---

### Chapter 6: Google Gemini CLI - Open Source and Everywhere

**Themes**:
- Open source under Apache 2.0—implications and community
- Free tier: 60 req/min, 2000 req/day (double average developer usage)
- Gemini 2.5 Pro with 1M token context window—codebase analysis capability
- Built on MCP—extensible architecture
- Gemini CLI Extensions—similar to Claude Code Skills but open ecosystem
- Qwen Code fork by Alibaba (2000 req/day free)—ecosystem diversity

**Key Sections**:
1. **Google's Response** — Why Gemini CLI, open source strategy, community advantage
2. **Free Tier Reality** — Rate limits, tier structure, cost comparison
3. **Installation & Authentication** — Multi-platform, personal Google account
4. **1M Token Context** — What you can do with full codebase analysis
5. **Extensions Framework** — Creating custom capabilities
6. **Qwen Code Alternative** — Alibaba fork, different use cases
7. **Claude Code Comparison** — Objective, honest evaluation (Principle 11)
8. **Practical Workflows** — End-to-end examples matching Chapter 5 complexity

**Learning Outcomes**:
- Understand Gemini CLI as viable, free alternative to commercial tools
- Evaluate open-source vs. proprietary trade-offs
- Install and authenticate Gemini CLI
- Understand rate limits and scaling strategies
- Create custom extensions for domain advantage
- Compare tools objectively to choose appropriate tool per task

---

### Chapter 7: Bash Essentials for AI-Driven Development

**Themes**:
- Bash is the interface to every AI CLI tool—critical foundation
- Two-part structure: concrete commands (Part I) + natural language prompts (Part II)
- Minimal guide—90% of AI dev work covered by small command set
- Environment variables are how AI tools get credentials (API keys, secrets)
- Package managers (Homebrew, apt, pip, npm) install tools and dependencies
- Pipes and redirection create powerful workflows
- Troubleshooting empowerment—readers learn common issues and solutions

**Learning Outcomes**:
- Navigate filesystem and manage files independently
- Set up and verify environment variables for API keys
- Request AI tools to execute Bash commands via natural language
- Understand package management on your platform
- Troubleshoot common Bash issues
- Create efficient AI-assisted development workflows

---

### Chapter 8: Git & GitHub for AI-Driven Development

**Themes**:
- Git is CRITICAL for AI development safety (checkpoint before changes, easy rollback)
- GitHub is portfolio + backup + collaboration platform
- Two-part structure: concrete Git commands (Part I) + natural language prompts (Part II)
- Pull requests enable code review culture, perfect for reviewing AI-generated code
- Branches create safe experimentation spaces for AI agents
- .gitignore prevents committing secrets (CRITICAL with AI tools)
- Authentication tokens/SSH—security first

**Learning Outcomes**:
- Initialize repositories and push to GitHub (portfolio building)
- Use Git workflow for daily development checkpoints
- Create branches for safe experimentation with AI code
- Understand and practice pull requests with code review
- Rollback AI mistakes safely and quickly
- Prevent committing secrets and maintain .gitignore discipline
- Request AI tools to execute complex Git workflows
- Use GitHub as portfolio demonstrating professional development practices

---

## Assumptions & Design Decisions

**Assumption 1: Readers Have Completed Part 1**
- We assume foundational AI literacy (what AI is, key opportunities/challenges, AI-driven mindset)
- We don't re-explain concepts from Part 1
- Part 2 jumps immediately into tool installation and practice

**Assumption 2: Minimal Bash/Git Prior Knowledge**
- Many readers will have NEVER used command line before
- We design for absolute beginners while accommodating some prior experience
- We emphasize that learners don't need to memorize Bash—AI tools help them
- Two-part structure (commands + natural language prompts) caters to different learning styles

**Assumption 3: Tool Versions Will Change**
- Claude Code, Gemini CLI, and Bash all update regularly
- We capture current state (as of 2025-10-30) but include maintenance notes
- Links are verified at publication; quarterly updates required
- Tool sections include "check these official docs for current version"

**Assumption 4: Diverse Platform Backgrounds**
- Readers use Windows, Mac, or Linux
- We provide platform-specific instructions (package managers differ significantly)
- We test code examples on all three platforms before publication
- Troubleshooting sections address platform-specific issues

**Assumption 5: Safety Is Non-Negotiable**
- When learners use AI tools to modify code, they MUST have Git discipline
- We treat checkpoints, branches, and PRs as professional safety practices
- We model this behavior: "Always commit before letting AI make changes"
- We don't position this as optional or advanced—it's fundamental

---

## Constraints & Non-Goals

### In Scope
- ✅ Getting learners productive with Claude Code and Gemini CLI
- ✅ Foundational Bash skills for AI development workflows
- ✅ Git fundamentals and safety practices for AI-generated code
- ✅ Tool comparison (Claude Code vs. Gemini CLI) per Principle 11
- ✅ Natural language prompts showing how to request AI tools help with Bash and Git
- ✅ Practical, immediately-useful workflows

### Out of Scope
- ❌ Comprehensive Bash tutorial (too deep; we cover 90% of AI dev work)
- ❌ Git advanced workflows (rebase, cherry-pick, stashing complexity)
- ❌ System administration or DevOps depth
- ❌ Alternative version control systems (Mercurial, Fossil, etc.)
- ❌ Cloud platform CLIs (AWS CLI, gcloud, etc.) — beyond scope
- ❌ Docker, Kubernetes, or containerization — covered later in book

---

## Next Steps

1. **Specification Approval**: Review and approve this spec
2. **Clarifications** (if needed): Address any ambiguities before planning
3. **Planning Phase**: `/sp.plan` creates detailed lesson plans and task checklists
4. **Implementation**: `chapter-planner` → `content-implementer` → content creation
5. **Validation**: `validation-auditor` validates against constitution and quality standards

---

## References

**Context Documents Used**:
- `/context/06_chap5_spec/readme.md` — Chapter 5 overview and links
- `/context/07_chap6_spec/readme.md` — Chapter 6 overview and links
- `/context/08_chap7_spec/readme.md` — Chapter 7 overview and links
- `/context/09_chap8_specs/readme.md` — Chapter 8 overview and links
- `.specify/memory/constitution.md` — Project principles and quality standards

**External Resources** (to be verified during content creation):
- Claude Code docs: https://docs.claude.com/en/docs/claude-code/overview
- Gemini CLI docs: https://geminicli.com/docs/
- Python & system documentation: inline per chapter
