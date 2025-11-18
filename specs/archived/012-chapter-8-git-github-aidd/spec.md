# Feature Specification: Chapter 8 - Git & GitHub for AI-Driven Development

**Feature Branch**: `012-chapter-8-git-github-aidd`
**Created**: 2025-11-05
**Status**: Draft
**Input**: User description: "Design chapter 8 Git & GitHub for AI-Driven Development with IDE setup - interactive conversational lessons based on AIDD principles"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Learning Git Safety Basics with AI Assistant (Priority: P1)

A beginner developer wants to learn Git version control so they can safely work with AI coding assistants (Claude Code, Gemini CLI) without losing their work or creating unreversible mistakes.

**Why this priority**: Git is the foundational safety mechanism for AI-driven development. Without version control, students cannot safely experiment with AI-generated code. This is the absolute minimum viable skill for Part 2.

**Independent Test**: Can be fully tested by having a student initialize a Git repository, make changes with an AI assistant, commit those changes, and successfully undo unwanted AI-generated code using Git commands via natural language prompts to their AI assistant.

**Acceptance Scenarios**:

1. **Given** a student has never used Git before, **When** they follow the interactive setup lesson and ask their AI assistant "Initialize a Git repository here", **Then** they successfully create a Git repository and understand why it matters for AI development
2. **Given** an AI assistant has generated code changes, **When** the student asks "Show me what changed" and "Save these changes with a meaningful commit message", **Then** they successfully view diffs and create commits with descriptive messages
3. **Given** an AI tool made breaking changes, **When** the student asks "Undo the last commit but keep my files so I can see what went wrong", **Then** they successfully roll back using `git reset --soft` via AI assistance
4. **Given** the student wants to try an AI-generated refactoring, **When** they ask "Create a safe branch for testing AI changes", **Then** they create a feature branch, test changes, and either merge or discard

---

### User Story 2 - GitHub Integration for Backup and Portfolio (Priority: P2)

A student completing Part 2 wants to connect their local Git repositories to GitHub so they have cloud backup, can share their AI-assisted projects, and build a professional portfolio.

**Why this priority**: GitHub provides essential backup and enables collaboration/portfolio building. It's not required for local Git safety (P1), but it's the next logical step for professional development.

**Independent Test**: Student can create a GitHub account, connect a local repository to GitHub, push commits, and view their project online - all using natural language prompts to their AI assistant.

**Acceptance Scenarios**:

1. **Given** a student has a local Git repository with commits, **When** they ask their AI assistant "Connect this project to GitHub and push my code", **Then** they successfully authenticate, add remote origin, and push to GitHub
2. **Given** a student is working on multiple machines, **When** they ask "Get the latest code from GitHub", **Then** they successfully pull changes and understand syncing workflows
3. **Given** a student wants to showcase their work, **When** they navigate to their GitHub profile, **Then** they see their AI-assisted projects displayed professionally

---

### User Story 3 - IDE Setup for AI-Assisted Development (Priority: P2)

A student wants to set up a modern IDE (VS Code or similar) integrated with Git, GitHub, and AI assistants so they have a professional development environment.

**Why this priority**: IDE setup significantly improves productivity and provides visual Git tools. However, students can learn Git via CLI first (P1) before adding IDE integration.

**Independent Test**: Student can install VS Code (or their preferred IDE), configure Git integration, install GitHub Copilot or Cursor AI extension, and perform basic Git operations through the IDE interface.

**Acceptance Scenarios**:

1. **Given** a student has installed VS Code, **When** they follow the IDE setup lesson, **Then** they successfully configure Git integration, GitHub authentication, and see visual diff/staging tools
2. **Given** a student is editing code in their IDE, **When** they make changes and want to commit, **Then** they can use both IDE visual tools AND command-line prompts to AI assistants interchangeably
3. **Given** a student wants AI coding assistance in their IDE, **When** they install recommended extensions (GitHub Copilot, Cursor AI, etc.), **Then** they successfully integrate AI tools directly into their coding workflow

---

### User Story 4 - Pull Requests and Code Review with AI (Priority: P3)

A student working on a team project or contributing to open source wants to create pull requests, review AI-generated code with teammates, and understand professional Git workflows.

**Why this priority**: Pull requests are essential for team collaboration but not required for individual learning in Part 2. This prepares students for professional development but can be learned after Git basics.

**Independent Test**: Student can create a feature branch, push it to GitHub, open a pull request with description of AI assistance used, respond to review comments, and merge the PR.

**Acceptance Scenarios**:

1. **Given** a student completed a feature with AI assistance, **When** they ask "Create a pull request for this feature explaining what AI helped with", **Then** they successfully open a PR with clear description of changes and AI tools used
2. **Given** a teammate left review comments, **When** the student asks "Help me address the feedback and update my PR", **Then** they make requested changes, push updates, and the PR updates automatically
3. **Given** a PR is approved, **When** the student asks "Merge this PR safely", **Then** they complete the merge, delete the feature branch, and sync their local repository

---

### Edge Cases

- What happens when a student tries to commit files containing API keys or secrets?
- How does the system handle merge conflicts when an AI assistant modified files that were also changed on GitHub?
- What if a student accidentally runs `git reset --hard` and loses uncommitted work?
- How do students recover if they push sensitive data to a public GitHub repository?
- What happens when an AI assistant suggests Git commands that would force-push to main branch?
- What if Git installation fails due to administrator permissions on Windows?
- What if a student creates a GitHub account but doesn't receive the verification email?
- What if a student's chosen IDE doesn't detect Git after installation?
- What if a student's machine doesn't meet minimum IDE requirements (low RAM, old OS)?
- What if a student gets confused by multiple IDE options and can't decide which to install?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Chapter MUST teach Git initialization, staging, committing, and basic status checking using both direct commands AND natural language prompts to AI assistants
- **FR-002**: Chapter MUST teach students how to undo changes using Git reset, checkout, and revert - with emphasis on safety and when to use each command
- **FR-003**: Chapter MUST teach branch creation, switching, merging, and deletion for safe experimentation with AI-generated code
- **FR-004**: Chapter MUST include step-by-step GitHub account creation (with screenshots) including: visit github.com, fill registration form, verify email, choose free plan, and complete onboarding - written for absolute beginners
- **FR-004a**: Chapter MUST teach GitHub repository creation, remote configuration, push/pull, and basic authentication (SSH or Personal Access Token) with troubleshooting for common beginner issues
- **FR-005**: Chapter MUST teach pull request workflow including creation, review, addressing feedback, and merging - with examples of documenting AI assistance
- **FR-006**: Chapter MUST include comprehensive IDE installation guide with direct download links for Windows/Mac/Linux, step-by-step installation instructions (with screenshots), and first-launch setup
- **FR-006a**: Chapter MUST provide IDE options (recommend VS Code but also mention alternatives like Cursor, PyCharm Community, Zed) with download links and brief comparison to help students choose
- **FR-006b**: Chapter MUST include detailed Git extension installation within IDE (Source Control panel, GitLens, Git Graph) and AI coding extensions (GitHub Copilot, Cursor AI, Continue) with activation instructions
- **FR-007**: Chapter MUST demonstrate the conversational AIDD approach with example conversations showing students asking AI assistants to perform Git operations
- **FR-008**: Chapter MUST teach `.gitignore` patterns specific to AI development (exclude API keys, environment variables, AI tool caches)
- **FR-009**: Chapter MUST include a "Git safety checklist" for working with AI coding assistants (commit before major changes, review AI diffs, never commit secrets)
- **FR-010**: Chapter MUST provide natural language prompt templates for common Git workflows that students can use with Claude Code, Gemini CLI, or similar tools
- **FR-011**: Chapter MUST maintain Tier 1 (Beginner) cognitive load limits: max 2 tool options, max 5 new concepts per section
- **FR-012**: Chapter MUST follow two-part structure: Part I (core Git concepts and commands) + Part II (natural language prompts for AI assistants)
- **FR-013**: All code examples MUST be tested with actual AI assistants (Claude Code and Gemini CLI) to verify prompt effectiveness
- **FR-014**: Chapter MUST include platform-specific instructions (Windows, Mac, Linux) for Git installation and configuration
- **FR-015**: Chapter MUST include troubleshooting section for common beginner issues: Git not found after install, GitHub email verification not received, IDE not detecting Git, permission errors on Windows, path configuration issues

### Key Entities *(include if feature involves data)*

- **Git Repository**: Local project with version control, includes working directory, staging area, and commit history
- **Commit**: Snapshot of project at a point in time, includes message, author, timestamp, and changes
- **Branch**: Parallel version of code for safe experimentation, can be merged back to main
- **Remote**: GitHub repository connected to local repository, enables backup and collaboration
- **Pull Request**: Proposed changes from a branch, includes description, commits, reviews, and discussion
- **AI Assistant Session**: Conversational interaction where student requests Git operations using natural language, assistant translates to Git commands
- **Development Environment**: IDE setup including editor, Git integration, GitHub connection, and AI coding extensions

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 90% of students successfully initialize a Git repository, make commits, and undo changes using natural language prompts to AI assistants within 30 minutes of starting the chapter
- **SC-002**: 85% of students successfully create a GitHub account (following step-by-step instructions), push a project to GitHub, and view it online within the first hour
- **SC-002a**: 90% of students successfully install Git and verify installation (all platforms: Windows, Mac, Linux) without needing external help
- **SC-002b**: 80% of students successfully install and configure an IDE (VS Code or alternative) with Git integration within 30 minutes
- **SC-003**: 80% of students successfully use Git branches to test AI-generated code changes and either merge or discard the branch based on results
- **SC-004**: Students complete the chapter Git workflows 40% faster using natural language prompts to AI assistants compared to memorizing commands from traditional tutorials
- **SC-005**: 95% of students correctly identify when to commit before letting AI make major changes (safety checkpoint awareness)
- **SC-006**: 75% of students successfully set up a modern IDE with Git and AI integrations and can use both CLI and GUI tools interchangeably
- **SC-007**: Zero instances of students committing API keys or secrets after completing the `.gitignore` section (verified through chapter exercises)
- **SC-008**: 70% of students successfully create a pull request documenting AI assistance, respond to review feedback, and merge changes
- **SC-009**: Reading level remains at Grade 7 or below (Flesch-Kincaid) for beginner accessibility
- **SC-010**: Students report 4.5/5 or higher confidence in "I can safely experiment with AI-generated code using Git" post-chapter survey

### Quality Validation Criteria

- **QC-001**: All conversational examples must be tested with real AI assistants (Claude Code, Gemini CLI) and verified to produce correct Git commands
- **QC-002**: Chapter must not exceed Tier 1 cognitive load limits (max 2 options, max 5 concepts per section)
- **QC-003**: Every Git command introduced must have a corresponding natural language prompt template
- **QC-004**: Platform-specific instructions must be tested on Windows, Mac, and Linux
- **QC-005**: All code examples must run successfully and not introduce security vulnerabilities
- **QC-006**: Chapter structure must clearly separate Part I (concepts) from Part II (AI prompts) as outlined in source context

## Assumptions

- Students have completed Part 1 (Introducing AI-Driven Development) and understand why AI tools matter
- Students have completed Chapters 5-7 (Claude Code, Gemini CLI, Bash Essentials) and have a working AI assistant available
- Students have basic computer literacy (can install software, create accounts online)
- Students have internet access for GitHub and IDE setup
- Students are using a modern operating system (Windows 10+, macOS 10.15+, or recent Linux distribution)
- Default AI assistant for examples is Claude Code or Gemini CLI (most accessible to beginners)
- Default IDE for setup instructions is VS Code (most popular and beginner-friendly)
- Students will practice Git locally before connecting to GitHub (safety-first approach)

## Scope

### In Scope

- Git fundamentals: init, add, commit, status, diff, log
- Git safety: reset, checkout, revert, branch workflows
- GitHub basics: account creation, repository setup, push/pull, authentication
- Pull requests: creation, review, addressing feedback, merging
- IDE setup: VS Code installation, Git integration, GitHub connection, AI extensions
- Natural language prompt templates for all Git operations
- Conversational AIDD teaching style with example AI assistant interactions
- `.gitignore` patterns for AI development (secrets, API keys, caches)
- Git safety checklist for working with AI coding assistants
- Platform-specific instructions (Windows, Mac, Linux)

### Out of Scope

- Advanced Git features (rebase, cherry-pick, submodules, hooks)
- Git internals and architecture (plumbing commands, object database)
- GitHub Actions and CI/CD workflows (covered in later parts)
- Multiple remote repositories and complex merging strategies
- Git LFS (Large File Storage) for handling large assets
- GitHub Issues, Projects, and project management features
- Self-hosted Git servers (GitLab, Bitbucket)
- Git performance optimization for large repositories
- Advanced PR review tools (GitHub code owners, required reviewers, status checks)
- Resolving complex merge conflicts (beyond basic conflict markers)

## Dependencies

- **Chapter 5** (Claude Code): Students need a working AI assistant for conversational Git interactions
- **Chapter 6** (Gemini CLI): Alternative AI assistant option for Git prompt examples
- **Chapter 7** (Bash Essentials): Students need basic terminal navigation skills
- **Part 1** (Introducing AIDD): Students must understand the paradigm shift and why AI tools require Git safety
- **Internet access**: Required for GitHub setup and remote repository operations
- **Git software**: Must be installed on student's machine (covered in chapter setup section)

## Constraints

- Chapter must be completable in 3.5-4 hours as specified in Part 2 README (extended slightly to accommodate installation steps for absolute beginners)
- Reading level must remain Grade 7 or below (Flesch-Kincaid)
- Cognitive load must not exceed Tier 1 limits (max 2 options, max 5 concepts per section)
- All AI prompt examples must work with both Claude Code and Gemini CLI
- Platform-specific instructions required for Windows, Mac, and Linux
- No assumed prior Git or version control knowledge
- Must integrate naturally with existing Part 2 chapters (5-7)

## Non-Goals

- Teaching students to memorize Git commands (AI assistants handle command syntax)
- Covering every Git feature comprehensively (focus on 80/20 rule - most-used workflows)
- Making students Git experts (they should be safe and productive, not experts)
- Teaching GitHub as a social network (focus is developer workflow, not community features)
- Covering alternative version control systems (SVN, Mercurial, etc.)
- Deep dive into Git's internal architecture (students don't need to understand object database)

## Complexity Tier

**Tier 1: Beginner** (Parts 1-3)

This chapter targets complete beginners who have never used version control. Students are learning their first development tool in the context of AI-assisted coding.

**Cognitive Load Management**:
- Max 2 options: Teach GitHub (not GitLab, Bitbucket, self-hosted)
- Max 2 options: Teach VS Code or let AI choose IDE (not overwhelming with 5+ editor options)
- Max 5 concepts per section: Break complex topics (branching, pull requests) into digestible chunks
- One new skill per lesson: Each lesson focuses on mastering ONE Git workflow

**AI-as-Partner Pattern**:
- Students don't memorize commands; AI assistants execute them
- Focus on WHAT Git does and WHY it matters, not command syntax
- Natural language prompts are the primary interaction method
- Students' job: Understand concepts, ask questions, supervise execution

**Concept-First Pattern**:
- WHAT: Explain Git concepts without jargon (e.g., "Git is a time machine for your code")
- WHY: Explain why this matters for AI development (e.g., "AI can break things; Git lets you undo")
- HOW: Show the command (e.g., `git reset --soft HEAD~1`)
- PRACTICE: Interactive exercise with AI assistant

**Safety and Error Literacy**:
- Explicit warnings before destructive commands
- Clear recovery procedures for common mistakes
- AI assistant prompts include safety checks (e.g., "Confirm before deleting uncommitted work")

## Chapter Structure

### Part I: Git Concepts and Commands (Estimated: 100 minutes)

**Section 1: Why Git Matters with AI Tools** (15 min)
- The problem: AI can break your code
- Git as safety net: Save, undo, track, collaborate
- Conversational example: "Before AI refactors my code, what should I do?"

**Section 2: Essential Setup** (25 min)
- **Install Git** (platform-specific with download links):
  - Windows: Download from git-scm.com (direct link provided)
  - Mac: Install via Homebrew or download installer
  - Linux: Install via package manager (apt, yum, pacman)
  - Verification: Check installation with `git --version`
- **Configure Git** (first-time setup):
  - Set username: `git config --global user.name "Your Name"`
  - Set email: `git config --global user.email "you@example.com"`
- **Create GitHub Account** (step-by-step with screenshots):
  - Visit github.com (direct link)
  - Sign up form: username, email, password
  - Verify email address (check inbox/spam)
  - Choose Free plan
  - Complete welcome survey (optional but recommended)
  - Tour of GitHub interface
- Conversational example: "Set up Git and GitHub for my machine - I've never done this before"

**Section 3: The Daily Workflow** (25 min)
- Five core commands: init, status, add, commit, push
- Conversational example: "Save my work and back it up to GitHub"
- Practice exercise: Initialize repo, make changes, commit

**Section 4: Safety Net - Undoing Changes** (30 min)
- View changes: diff, log
- Undo changes: checkout, reset (soft vs hard), revert
- Conversational example: "The AI broke my code - take me back to before"
- Critical warning: When commands delete work permanently

### Part II: Natural Language Prompts for AI Assistants (Estimated: 100 minutes)

**Section 5: Branches for Experimentation** (25 min)
- Create, switch, merge, delete branches
- Conversational example: "Create a safe space to test AI changes"
- Practice: Test AI-generated refactor on branch, then decide merge or discard

**Section 6: GitHub Integration** (20 min)
- Connect local repo to GitHub
- Authentication (SSH or Personal Access Token)
- Push and pull
- Conversational example: "Upload my project to GitHub"

**Section 7: Pull Requests and Code Review** (30 min)
- Create PR with AI assistance documentation
- Review changes, address feedback
- Merge and cleanup
- Conversational example: "Create a PR and explain what AI helped build"

**Section 8: IDE Setup** (25 min)
- **Choose Your IDE** (beginner guidance):
  - **VS Code** (recommended): Most popular, beginner-friendly, free
    - Download: code.visualstudio.com (direct links for Windows/Mac/Linux)
  - **Cursor** (AI-first): Built-in AI coding assistant
    - Download: cursor.sh
  - **PyCharm Community** (Python-focused): Great for Python development
    - Download: jetbrains.com/pycharm/download
  - **Zed** (lightweight, fast): Modern minimalist editor
    - Download: zed.dev
  - Brief comparison table to help students choose
- **Install Your Chosen IDE** (step-by-step with screenshots):
  - Download installer for your operating system
  - Run installer (Windows: .exe, Mac: .dmg, Linux: .deb/.rpm)
  - First launch: Welcome screen, theme selection, initial setup
  - Verify installation: Create and save a test file
- **Configure Git Integration**:
  - Open Source Control panel (built-in to most IDEs)
  - Verify Git is detected (shows repository info)
  - Enable GitLens extension (VS Code) for enhanced Git features
  - Install Git Graph for visual commit history
- **Install AI Coding Extensions**:
  - GitHub Copilot (if available to student)
  - Cursor AI (if using Cursor IDE)
  - Continue extension (free AI coding assistant)
  - Activate and test: Write a comment, see AI suggestion
- **Visual Git Tools Tour**:
  - View diffs in side-by-side editor
  - Stage changes via checkboxes (no command line needed)
  - Write commit messages in GUI
  - Compare: CLI prompts to AI vs IDE visual tools (both are valid!)
- Conversational example: "I'm a complete beginner - help me choose and install an IDE, then connect it to Git"

### Try With AI Exercise (Capstone) (30 min)

**Scenario**: Build a small Python script with AI assistance, use Git for safety, push to GitHub, create a PR

**Steps**:
1. Ask AI assistant: "Initialize a Git repository for a Python calculator project"
2. Ask AI: "Generate a simple calculator.py with add, subtract, multiply, divide functions"
3. Review code, commit: "Save this calculator code"
4. Ask AI: "Add error handling to calculator.py"
5. Test, create branch: "Create a test branch before I make this change permanent"
6. If good, merge; if bad, discard branch
7. Connect to GitHub: "Upload this project to GitHub"
8. Create PR: "Create a pull request showing my calculator project"

**Learning Objectives Verified**:
- [ ] Student used Git commands via natural language prompts
- [ ] Student created safety checkpoints before AI changes
- [ ] Student used branches for experimentation
- [ ] Student connected project to GitHub
- [ ] Student can explain why each step matters

## Teaching Philosophy

This chapter follows the AI-Driven Development (AIDD) teaching approach:

**Learning WITH AI, not ABOUT AI**: Students use AI assistants to learn Git, not learn Git to eventually use AI. The AI is their co-pilot from day one.

**Conversational, not Imperative**: Students ask "Show me what changed" instead of memorizing "`git diff`". Natural language is the primary interface.

**Safety-First**: Every destructive operation includes warnings and recovery procedures. Students learn to commit before major changes as a reflex.

**Progressive Disclosure**: Start with 5 essential commands (init, status, add, commit, push). Add complexity only after basics are solid.

**Real Workflows, not Toy Examples**: All exercises reflect authentic AI-assisted development patterns. No hypothetical scenarios.

**Two-Part Structure**: Part I teaches concepts and commands. Part II teaches natural language prompt templates. Students learn both what Git does AND how to ask AI to do it.

---

**Ready for Planning Phase**: After approval, proceed to `/sp.plan` to break this specification into detailed lesson-by-lesson implementation.
