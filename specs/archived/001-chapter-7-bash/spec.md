# Feature Specification: Chapter 7 - Bash Essentials for AI-Driven Development

**Feature Branch**: `001-chapter-7-bash`
**Created**: 2025-10-31
**Status**: Draft
**Input**: User description: "Design chapter 7. Check the part 2 goal provided at @book-source\docs\02-AI-Tool-Landscape\README.md and the source content to generate specs @context\08_chap7_spec\"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Beginner Learns Essential Bash Commands (Priority: P1)

A complete beginner with no terminal experience needs to learn the fundamental Bash commands required to navigate the filesystem, manage files, and work with AI CLI tools like Claude Code and Gemini CLI. They need to understand commands well enough to follow along with later chapters and execute basic operations confidently.

**Why this priority**: This is the foundational knowledge that enables all subsequent work with command-line AI tools. Without these basics, learners cannot proceed with Claude Code or Gemini CLI workflows. This represents the minimum viable content for the chapter.

**Independent Test**: Reader can open a terminal, navigate to a directory, create a new folder, create files, list contents, and set an environment variable for an API key. Success is demonstrated by completing these tasks without assistance.

**Acceptance Scenarios**:

1. **Given** a reader has never used a terminal before, **When** they complete Part I of Chapter 7, **Then** they can navigate the filesystem using cd, pwd, and ls commands confidently
2. **Given** a reader needs to set up an API key, **When** they follow the environment variable instructions, **Then** they can export an API key and verify it persists across terminal sessions
3. **Given** a reader encounters a "command not found" error, **When** they refer to the troubleshooting section, **Then** they can diagnose and resolve the issue independently
4. **Given** a reader needs to create a project structure, **When** they use mkdir and touch commands, **Then** they can set up a new project directory with essential files
5. **Given** a reader is working with files, **When** they use cp, mv, and rm commands, **Then** they can manage files safely without data loss

---

### User Story 2 - Reader Learns AI-Native Bash Workflows (Priority: P2)

After mastering basic commands, a reader needs to learn the paradigm shift: using natural language prompts with AI CLI tools to execute Bash commands instead of memorizing syntax. They understand that AI tools can translate their intent into correct Bash commands, making them productive faster.

**Why this priority**: This transforms the learning experience from traditional memorization to AI-augmented productivity. It's what makes this book different from traditional Bash tutorials. While important, it builds on the foundational knowledge from P1.

**Independent Test**: Reader can describe what they want to accomplish in natural language to Claude Code or Gemini CLI, and the AI tool executes the correct Bash command. Reader can explain when to learn the command versus when to ask AI for help.

**Acceptance Scenarios**:

1. **Given** a reader knows the task they want to accomplish but not the exact command, **When** they provide a natural language prompt to their AI CLI tool, **Then** the AI executes the correct Bash command
2. **Given** a reader encounters a complex file operation, **When** they describe it in plain English to the AI, **Then** the AI generates and explains the appropriate command pipeline
3. **Given** a reader is working with environment variables, **When** they ask the AI to set up persistent API keys, **Then** the AI modifies the correct config file and sources it
4. **Given** a reader needs to troubleshoot a permission error, **When** they describe the problem to the AI, **Then** the AI suggests chmod or sudo solutions with explanations
5. **Given** a reader wants to find files matching a pattern, **When** they describe the search criteria, **Then** the AI constructs the appropriate find or grep command

---

### User Story 3 - Reader Builds Professional Workflow Habits (Priority: P3)

A reader who has learned basic commands and AI-assisted workflows now needs to develop professional habits: understanding when to use which approach, recognizing dangerous commands, using keyboard shortcuts efficiently, and building muscle memory for common operations.

**Why this priority**: These are refinements that improve productivity but aren't blocking for basic competence. A reader can be effective without perfecting these habits, but they enhance long-term efficiency.

**Independent Test**: Reader demonstrates good judgment about command safety (e.g., always checks before rm -rf), uses keyboard shortcuts (Ctrl+R for history search, Tab completion), and can explain when to memorize a command versus when to ask AI.

**Acceptance Scenarios**:

1. **Given** a reader is about to run a destructive command, **When** they review what it will do, **Then** they recognize the risk and verify the target before executing
2. **Given** a reader needs to repeat a recent command, **When** they use Ctrl+R or history, **Then** they quickly locate and execute it
3. **Given** a reader is typing a long path, **When** they use Tab completion, **Then** they complete paths efficiently without typos
4. **Given** a reader needs to create command aliases, **When** they edit their .bashrc, **Then** they can set up shortcuts for frequently used commands
5. **Given** a reader encounters an unfamiliar command, **When** they use man or ask their AI assistant, **Then** they understand what it does before running it

---

### Edge Cases

- What happens when a reader is on Windows and the Bash commands don't work in Command Prompt or PowerShell?
- How does the chapter handle readers who are using Git Bash, WSL, or different terminal emulators?
- What if a reader accidentally runs `rm -rf` on an important directory?
- How does the chapter address readers who need to use sudo for everything (permissions issues)?
- What happens when environment variables don't persist after closing the terminal?
- How does the chapter handle readers who copy commands from AI without understanding them?
- What if a reader's PATH is misconfigured and commands aren't found?
- How does the chapter address the difference between bash, zsh, fish, and other shells?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Chapter MUST provide a complete reference of essential Bash commands covering navigation (cd, pwd, ls), file operations (cp, mv, rm, mkdir, touch), and viewing files (cat, head, tail, less)
- **FR-002**: Chapter MUST include detailed instructions for setting environment variables both temporarily (export) and permanently (editing .bashrc/.zshrc)
- **FR-003**: Chapter MUST explain process management commands (ps, top, kill) necessary for working with long-running AI CLI tools
- **FR-004**: Chapter MUST cover pipes and redirection (|, >, >>, 2>) for chaining commands and saving output
- **FR-005**: Chapter MUST provide platform-specific package management instructions (Homebrew for macOS, apt for Ubuntu, pip for Python, npm for Node)
- **FR-006**: Chapter MUST include file search commands (find, grep, which) essential for locating code and configurations
- **FR-007**: Chapter MUST explain permissions (chmod, sudo) and troubleshooting "permission denied" errors
- **FR-008**: Chapter MUST provide natural language prompt templates for every Bash command covered, showing 2-3 alternative phrasings
- **FR-009**: Chapter MUST teach when to learn the Bash command directly versus when to use AI assistance
- **FR-010**: Chapter MUST include practical patterns for AI CLI work (checking installations, creating project structures, monitoring output, chaining operations)
- **FR-011**: Chapter MUST provide a troubleshooting section addressing common errors (command not found, permission denied, API key issues)
- **FR-012**: Chapter MUST include keyboard shortcuts that improve terminal productivity (Tab completion, Ctrl+R, Ctrl+C, Ctrl+A/E/U/L)
- **FR-013**: Chapter MUST follow the "lesson" output style with opening hooks, runnable examples, practice exercises, and reflection prompts
- **FR-014**: Chapter MUST align with Part 2 goals: making readers capable, not just knowledgeable about command-line tools
- **FR-015**: Chapter MUST scaffold from complete beginner (Part I) to AI-augmented workflows (Part II) progressively

### Key Entities *(include if feature involves data)*

- **Bash Command**: A specific command (e.g., `cd`, `ls`, `grep`) with syntax, common flags, use cases, and safety considerations
- **Natural Language Prompt**: Alternative phrasing that a reader can use with AI CLI tools to accomplish the same task as a Bash command
- **Environment Variable**: A key-value pair (e.g., `ANTHROPIC_API_KEY`) that persists configuration across sessions
- **Command Pattern**: Common multi-command workflows (e.g., "create project structure", "search and filter", "monitor process output")
- **Troubleshooting Scenario**: A specific error condition (e.g., "command not found") with diagnostic steps and solutions
- **Keyboard Shortcut**: Terminal keyboard combinations (e.g., Ctrl+R) that improve efficiency

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Readers who complete Chapter 7 can navigate to any directory, create folders, create files, and list contents within 30 seconds of being given a task
- **SC-002**: Readers can set an API key as an environment variable and verify it persists across terminal sessions without referring back to the chapter
- **SC-003**: 90% of readers can translate a task described in plain English into the correct Bash command OR know when to ask their AI assistant for help
- **SC-004**: Readers can identify dangerous commands (rm -rf, sudo operations) and verify what they will do before executing them
- **SC-005**: Readers complete all practice exercises in Part I and can execute equivalent operations using natural language prompts in Part II
- **SC-006**: Readers can troubleshoot common errors (command not found, permission denied, API key not set) independently using the troubleshooting section
- **SC-007**: Readers demonstrate productivity habits: using Ctrl+R for history search, Tab for completion, and command aliases for frequent operations
- **SC-008**: Readers understand the paradigm shift: Bash is the "native language" of development, but AI tools can translate their intent when needed
- **SC-009**: Readers can create a new project directory structure with proper initialization (mkdir, touch README, .gitignore, git init) in under 2 minutes
- **SC-010**: Readers report confidence in working with command-line AI tools (Claude Code, Gemini CLI) after completing this chapter

## Assumptions

- Readers have completed Part 1 (understanding AI-driven development paradigm) and Chapters 5-6 (installed Claude Code or Gemini CLI)
- Readers have access to a Unix-like terminal (macOS/Linux native, Windows via Git Bash or WSL)
- Readers are comfortable with basic computer operations (opening applications, typing, using a mouse)
- Readers have internet access to install packages and tools referenced in the chapter
- Readers understand that this is a minimal "get productive" guide, not an exhaustive Bash reference
- The chapter focuses on the 90% of Bash commands used in AI-driven development workflows, intentionally omitting advanced scripting and system administration

## Scope

### In Scope

- Essential Bash commands for navigation, file management, and process control
- Environment variable management (temporary and persistent)
- Practical package management commands for AI development tools
- Pipes, redirection, and command chaining for AI CLI workflows
- Natural language prompt templates for every command covered
- Troubleshooting common terminal errors
- Keyboard shortcuts and productivity tips
- Platform-specific instructions (macOS, Linux, Windows with Git Bash/WSL)
- Integration with Claude Code and Gemini CLI natural language workflows
- Two-part structure: Part I (learn commands), Part II (use AI to execute commands)

### Out of Scope

- Advanced Bash scripting (loops, conditionals, functions) - covered in later chapters if needed
- System administration tasks (user management, networking, services)
- In-depth shell customization (themes, plugins, advanced prompt configuration)
- Detailed comparison of different shells (bash vs zsh vs fish)
- Comprehensive command flag documentation - readers can use `man` or ask AI
- Windows Command Prompt or PowerShell native commands (focus is Unix-like environments)
- Security hardening and production-grade shell configuration
- Writing complex shell scripts - focus is on interactive command-line usage

## Dependencies

- Readers must have completed Chapter 5 (Claude Code installation) or Chapter 6 (Gemini CLI installation)
- Readers must have access to a Unix-like terminal environment (native or emulated on Windows)
- Chapter 8 (Git & GitHub) will build on Bash skills learned here
- Future Python chapters (Part 4 onward) assume readers can use the terminal confidently

## Constraints

- Chapter must remain accessible to complete beginners with no terminal experience
- Content must balance command memorization with AI-augmented workflows
- Examples must work across macOS, Linux, and Windows (Git Bash/WSL) with platform notes where behavior differs
- Chapter must align with "lesson" output style while covering reference material comprehensively
- Part I and Part II should be approximately equal in length and depth
- Total reading and practice time should be 3-4 hours as specified in Part 2 README

## Risks

- **Risk**: Readers may feel overwhelmed by command-line interface if they've never used a terminal
  - **Mitigation**: Start with very basic commands, use clear screenshots/examples, provide extensive troubleshooting guidance

- **Risk**: Readers may rely too heavily on AI prompts and not develop command-line fluency
  - **Mitigation**: Part I teaches commands directly first, Part II introduces AI augmentation after foundation is built

- **Risk**: Platform differences (macOS vs Linux vs Windows) may cause confusion
  - **Mitigation**: Provide explicit platform notes for commands that differ, recommend Git Bash for Windows users

- **Risk**: Readers may accidentally execute dangerous commands (rm -rf) and lose data
  - **Mitigation**: Include prominent warnings, teach verification habits, explain what commands do before running them

- **Risk**: Environment variables may not persist due to shell configuration issues
  - **Mitigation**: Provide step-by-step instructions for .bashrc/.zshrc editing, include troubleshooting for common issues

- **Risk**: Readers may skip Part I and jump to Part II, missing foundational understanding
  - **Mitigation**: Structure content so Part II explicitly references and builds on Part I concepts

## Notes

- This chapter is critical infrastructure for all subsequent work in the book - readers must feel confident in the terminal
- The two-part structure (commands first, then AI augmentation) is intentional to prevent over-reliance on AI without understanding
- Natural language prompt templates should feel natural and conversational, not stiff or robotic
- Examples should use realistic file names, directory structures, and scenarios from actual development work
- Chapter should reference Claude Code and Gemini CLI by name to connect abstract Bash knowledge to concrete tools readers have installed
