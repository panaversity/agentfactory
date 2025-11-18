# Feature Specification: Chapter 7 Bash Essentials Redesign

**Feature Branch**: `feature/chapter-7-bash-essentials`
**Created**: 2025-11-07
**Status**: Draft
**Input**: User description: "Redesign Bash Essentials chapter to follow AI-native teaching pattern where students learn to communicate bash needs in natural language, AI executes, and students observe and validate outcomes. Focus on graduated teaching: Tier 1 (foundational commands), Tier 2 (complex operations with AI companion), Tier 3 (orchestration at scale)."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Natural Language Navigation (Priority: P1)

**As a beginner developer**, I want to ask my AI companion "Where am I?" and "What files are here?" in plain English, so that I understand my file system location and context before performing any operations.

**Why this priority**: Navigation and location awareness is the foundation of all file operations. Without understanding location, students cannot safely supervise AI actions. This is the most critical skill.

**Independent Test**: Student can conduct a conversation with AI about their current directory, list files, and explain why location matters—all without typing a single bash command themselves.

**Acceptance Scenarios**:

1. **Given** student opens AI tool, **When** they ask "Show me where I am right now", **Then** AI executes `pwd`, explains the path, and student understands they're in a specific directory
2. **Given** student is in a directory, **When** they ask "What files can you see?", **Then** AI executes `ls`, shows files/folders, explains the difference, and student can identify which items are files vs directories
3. **Given** student needs to work in a project folder, **When** they ask "Navigate to my Documents folder and show me what's there", **Then** AI changes directory, confirms location, lists contents, and student validates they're in the correct place

---

### User Story 2 - Safety-First File Operations (Priority: P1)

**As a beginner developer**, I want to direct my AI to perform file operations (copy, move, delete) using natural language while following a 5-step safety pattern, so that I never accidentally destroy important files.

**Why this priority**: File operations are destructive and irreversible. Teaching safety BEFORE teaching commands prevents catastrophic mistakes. This is equally critical as navigation.

**Independent Test**: Student can direct AI to delete a folder, but ONLY after: (1) confirming location, (2) listing what will be deleted, (3) asking clarifying questions, (4) getting explicit confirmation, (5) executing and validating results.

**Acceptance Scenarios**:

1. **Given** student wants to back up files, **When** they ask "Create a backup of my project folder", **Then** AI explains the plan, shows what will be copied, asks for confirmation, executes, and validates the backup exists
2. **Given** student wants to delete old files, **When** they ask "Delete the temp folder", **Then** AI first shows current location, lists temp folder contents, asks "Are you sure?", waits for explicit yes, then executes
3. **Given** AI suggests a command with `rm -rf`, **When** student sees the command, **Then** student recognizes this is destructive, asks "What does -rf mean?" and "Can I undo this?", and only proceeds after understanding implications

---

### User Story 3 - Configuration and Environment Setup (Priority: P2)

**As a developer**, I want to tell my AI "Set up my environment variables for this API key" in plain language, so that I can configure my development environment securely without memorizing syntax or accidentally exposing secrets.

**Why this priority**: Environment configuration is essential for real projects but involves security concerns. Students need to understand secrets management without memorizing `.env` file syntax.

**Independent Test**: Student can direct AI to create `.env` file, add API key, configure `.gitignore`, and explain why secrets shouldn't be committed—all through conversation, not manual file editing.

**Acceptance Scenarios**:

1. **Given** student has an API key, **When** they ask "Store my OpenAI API key securely", **Then** AI creates `.env` file, adds key, updates `.gitignore`, and explains why keys stay local
2. **Given** student needs to install packages, **When** they say "Install the dependencies for this Python project", **Then** AI executes `uv sync` or `pip install -r requirements.txt`, explains what's happening, and shows successful installation
3. **Given** student sees an error about missing dependencies, **When** they ask "Why is this failing?", **Then** AI reads error message, identifies missing package, explains the cause, and offers to fix it

---

### User Story 4 - Complex Multi-Step Workflows (Priority: P3)

**As a developer**, I want to tell my AI "Set up 10 parallel git worktrees for features 1-10" and have it orchestrate the entire workflow, so that I learn strategic thinking instead of repetitive command execution.

**Why this priority**: Orchestration teaches scaling mindset. Once students understand basics (P1) and safety (P1-P2), they're ready to see AI automate repetitive tasks. This is advanced but builds on prerequisites.

**Independent Test**: Student can specify a multi-step workflow (e.g., "Clone this repo, create 5 branches, set up virtual environments in each"), supervise AI execution, and validate all steps completed correctly.

**Acceptance Scenarios**:

1. **Given** student needs multiple parallel workspaces, **When** they say "Create 10 git worktrees named feature-1 through feature-10", **Then** AI executes loop, creates all worktrees, reports progress, and student validates directory structure
2. **Given** student has a complex pipeline, **When** they describe "Read all .json files, extract 'name' field, write to summary.txt", **Then** AI writes pipeline command, explains each part, executes, and shows results
3. **Given** student encounters a failing script, **When** they ask "Debug this pipeline that's giving errors", **Then** AI reads error output, identifies the issue (missing file, wrong permission, syntax error), explains root cause, and proposes fix

---

### User Story 5 - Reading and Understanding AI-Generated Commands (Priority: P2)

**As a beginner developer**, I want to see bash commands my AI generates and understand what each part means (flags, pipes, redirects), so that I can supervise AI work confidently and catch mistakes before execution.

**Why this priority**: Supervision requires comprehension. Students don't need to memorize syntax, but they MUST understand what commands will do before saying "yes." This is essential for safety and trust.

**Independent Test**: Student can look at a command like `find . -name "*.py" | xargs grep "import os"` and explain in plain English: "This finds all Python files and searches them for 'import os'."

**Acceptance Scenarios**:

1. **Given** AI suggests `ls -lah`, **When** student asks "What do those flags mean?", **Then** AI explains `-l` (long format), `-a` (all files including hidden), `-h` (human-readable sizes)
2. **Given** AI proposes `cat file.txt | grep "error" | wc -l`, **When** student asks "Walk me through this pipeline", **Then** AI explains: read file, filter for "error", count matching lines
3. **Given** AI shows `chmod +x script.sh`, **When** student asks "What does this do and why?", **Then** AI explains: makes file executable, necessary to run the script, shows how to verify with `ls -l`

---

### Edge Cases

- **What happens when** student asks to delete a folder that doesn't exist? → AI shows location, attempts operation, encounters error, explains "folder not found," asks for clarification
- **What happens when** student's bash command from AI fails due to permissions? → AI reads "permission denied" error, explains cause, offers solutions (sudo, change ownership, check path)
- **What happens when** student asks for a command AI cannot execute directly (like creating files in some tools)? → AI transparently explains limitation, provides command for student to run manually, explains what it will do
- **What happens when** student tries to execute a destructive command in the wrong directory? → AI shows current location FIRST, asks "Are you sure this is the right place?", waits for explicit confirmation
- **What happens when** student wants to chain multiple complex commands? → AI breaks down into steps, explains each step, asks "Should I proceed to step 2?" between steps

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Chapter MUST teach natural-language-to-bash communication pattern (student specifies intent, AI translates to commands, student observes)
- **FR-002**: Chapter MUST follow graduated teaching tiers: Tier 1 (foundational commands taught directly), Tier 2 (complex syntax via AI companion), Tier 3 (orchestration at scale)
- **FR-003**: Chapter MUST include 5-step safety pattern: Ask → Explain → Understand → Verify → Execute
- **FR-004**: All bash commands MUST be shown with AI's natural language explanation (never raw commands without context)
- **FR-005**: Chapter MUST demonstrate real AI behavior (errors, limitations, clarifying questions) not idealized scenarios
- **FR-006**: Chapter MUST teach command comprehension skills (reading flags, pipes, redirects) without requiring memorization
- **FR-007**: Chapter MUST include validation and supervision techniques (how to know if operation succeeded, how to catch errors)
- **FR-008**: Examples MUST use real tools (Claude Code, Gemini CLI, ChatGPT Code Interpreter) with authentic dialogue transcripts
- **FR-009**: Chapter MUST address security concerns (secrets management, permission issues, destructive commands)
- **FR-010**: Exercises MUST require students to have AI conversations, not type bash commands manually
- **FR-011**: Chapter MUST apply cognitive load limits for beginners (max 5 concepts per lesson, max 2 tool options)
- **FR-012**: Chapter MUST include error literacy (reading error messages, understanding common failure modes)
- **FR-013**: Content MUST be organized by workflows/tasks (navigation, file operations, configuration) NOT by commands (ls, cd, mkdir)

### Key Entities *(educational content context)*

- **Lesson**: Focused unit teaching one workflow (navigation, safety pattern, file operations, etc.) with learning objectives, examples, exercises, and AI collaboration prompts
- **Dialogue Transcript**: Authentic conversation between student and AI showing natural language request, AI explanation, command execution, and validation
- **Safety Pattern**: 5-step framework ensuring understanding before execution (Ask, Explain, Understand, Verify, Execute)
- **Graduated Tier**: Teaching progression level (Tier 1: book teaches directly, Tier 2: AI companion executes complex syntax, Tier 3: AI orchestrates multi-step workflows)
- **Command Comprehension Checkpoint**: Exercise where student explains in plain English what a bash command does (without running it)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can conduct 5+ back-and-forth conversations with AI about file system navigation without typing a single bash command themselves
- **SC-002**: Students can identify and explain the 5 steps of safety pattern when shown a dialogue transcript
- **SC-003**: Students can direct AI to perform a file operation (copy/move/delete) and demonstrate all 5 safety steps in conversation
- **SC-004**: Students can read a multi-part bash command (with pipes) and explain what it does in plain English within 2 minutes
- **SC-005**: Students recognize red-flag commands (`rm -rf`, `sudo`, `chmod 777`) and ask clarifying questions before allowing execution
- **SC-006**: 80% of students can complete "orchestration challenge" (specify multi-step workflow, supervise AI execution, validate results) without instructor intervention
- **SC-007**: Students can troubleshoot common errors (permission denied, file not found, command not found) by reading error messages and asking AI for explanations
- **SC-008**: Content maintains Grade 7-8 reading level with no unexplained jargon (validated by readability analysis)
- **SC-009**: Exercises demonstrate supervision skills (asking clarifying questions, validating outputs) not command memorization
- **SC-010**: Chapter includes 3+ real dialogue transcripts from actual AI tools showing authentic behavior (errors, clarifications, limitations)

### Business Goals (Learning Outcomes)

- **BG-001**: Students develop confidence to use terminal through AI collaboration (reduce terminal intimidation)
- **BG-002**: Students build supervision habits that prevent file loss and system damage (safety-first mindset)
- **BG-003**: Students understand when to ask AI for help vs. when to execute themselves (appropriate tool use)
- **BG-004**: Students recognize the difference between memorizing syntax (old way) and specifying intent (AI-native way)
- **BG-005**: Students can explain to others "I don't know bash commands, but I know how to work with bash through AI collaboration"

## Scope *(mandatory)*

### In Scope

- Natural language communication patterns for file system navigation
- Safety-first dialogue framework for destructive operations
- Reading and interpreting bash commands AI generates (flags, pipes, redirects)
- Configuration tasks (environment variables, secrets management, package installation)
- Multi-step workflow orchestration (git worktrees, batch operations, pipelines)
- Error literacy (reading error messages, understanding common failure modes)
- Supervision techniques (validation, confirmation, clarifying questions)
- Real dialogue transcripts from Claude Code, Gemini CLI, and similar tools
- Cognitive load management (max 5 concepts per lesson, beginner-appropriate)

### Out of Scope

- Traditional bash scripting (functions, loops, conditionals) taught as syntax to memorize
- Advanced bash programming patterns (arrays, parameter expansion, process substitution)
- System administration tasks (cron jobs, systemd, service management)
- Low-level OS concepts (processes, file descriptors, signals) beyond basic understanding
- Shell scripting best practices for writing production scripts
- Alternative shells (zsh, fish, PowerShell) beyond noting they exist
- Bash history, aliases, and customization (teach through AI conversation if needed)
- Manual editing of bash configuration files (`.bashrc`, `.profile`) unless AI-guided

### Out of Scope (Pedagogical)

- Memorization of bash commands or syntax
- Command cheat sheets or reference tables
- Quiz questions testing command syntax recall
- Exercises requiring students to write bash scripts from scratch
- Content assuming prior terminal experience
- Unix philosophy or shell history (unless directly relevant to task)

## Dependencies *(mandatory)*

### Prerequisites (what students must know)

- **Chapter 5**: Claude Code features and workflows (or equivalent AI tool)
- **Chapter 6**: Gemini CLI installation (or equivalent AI CLI tool)
- Students have access to a terminal and one AI tool (Claude Code, Gemini CLI, ChatGPT Code Interpreter)
- Students understand concept of "files and folders" from everyday computer use
- Students can copy/paste text and have basic conversations with AI

### Dependent Chapters (what depends on this)

- **Chapter 8**: Git & GitHub (requires bash navigation and file operations)
- **Chapter 12**: Python UV package manager (requires terminal confidence)
- All future chapters involving command-line tools and development workflows

### External Dependencies

- AI tool with bash execution capability (Claude Code, Gemini CLI, ChatGPT Code Interpreter)
- Students using macOS, Linux, or WSL (Windows Subsystem for Linux) with bash shell
- Terminal application installed and accessible

## Assumptions *(mandatory)*

- Students have zero prior terminal/bash experience (true beginners)
- Students have successfully installed one AI tool capable of bash execution (from Chapters 5-6)
- Students are comfortable asking questions and having back-and-forth conversations with AI
- Students understand that AI is a tool they supervise, not magic that always works perfectly
- All bash commands will be demonstrated through AI execution, not manual typing (except when AI cannot execute directly)
- Students will follow safety pattern religiously (no shortcuts) even when it feels slow
- Reading level is Grade 7-8 (beginner-friendly, no academic jargon)
- Examples use cross-platform commands where possible (`ls` works on macOS/Linux/WSL)

## Constraints *(mandatory)*

### Technical Constraints

- Must work across macOS, Linux, and Windows WSL (common bash environments)
- Cannot assume students have admin/sudo access
- Cannot require installation of additional terminal tools beyond bash and git
- Must account for AI tool limitations (some cannot execute commands directly, only provide guidance)
- Must use commands available in bash 4.0+ (standard on most systems)

### Pedagogical Constraints

- **Cognitive Load**: Maximum 5 new concepts per lesson section (beginner tier)
- **Graduated Complexity**: Tier 1 content only (foundational) for Part 2 of book
- **No Forward References**: Cannot reference bash concepts not yet taught
- **Safety First**: Every destructive operation must show safety pattern in action
- **Real Behavior**: Must show authentic AI dialogue including errors and limitations (no "perfect" scenarios)
- **Validation Required**: Every example must show how to verify success (not just execute and assume it worked)

### Content Constraints

- Chapter must fit within 8 lessons (standard chapter length)
- Each lesson: 30-40 minutes for average beginner
- Total chapter time: ~5 hours (including exercises and AI practice)
- Must maintain consistent voice and tone with rest of book
- Must use existing lesson template structure (learning objectives, examples, exercises, "Try With AI" section)

## Non-Goals *(mandatory)*

- Teaching students to write bash scripts from scratch (out of scope for beginner tier)
- Making students bash experts who can solve complex scripting problems independently
- Teaching every possible bash command or flag (focus on workflows, not exhaustive coverage)
- Preparing students for system administration or DevOps roles (that's advanced content)
- Teaching shell customization or productivity hacks (not critical for AI-driven development)
- Creating bash command reference or cheat sheet (defeats AI-native approach)

## Risks and Mitigations *(mandatory)*

### Risk 1: Students Become Over-Reliant on AI, Never Learn Fundamentals

**Likelihood**: Medium | **Impact**: High

**Mitigation**:
- Explicitly teach Tier 1 foundational concepts (navigation, file types, paths) in the book BEFORE introducing AI execution
- Require students to predict what command will do BEFORE AI executes
- Include "explain this command" exercises where student must describe in plain English
- Emphasize supervision and validation skills, not blind execution
- Show that understanding intent matters more than memorizing syntax

### Risk 2: AI Tools Behave Inconsistently or Make Errors, Confusing Students

**Likelihood**: High | **Impact**: Medium

**Mitigation**:
- Use REAL dialogue transcripts showing actual errors and limitations
- Teach students to expect errors and how to troubleshoot them
- Include "What went wrong?" analysis sections for common failure modes
- Emphasize that AI is a tool they supervise, not an infallible authority
- Show multiple AI tools to demonstrate variability (Claude Code, Gemini CLI)
- Teach error literacy: reading error messages, understanding common failures

### Risk 3: Students Skip Safety Pattern to Save Time, Then Lose Important Files

**Likelihood**: Medium | **Impact**: Critical

**Mitigation**:
- Lead with scary stories (real examples of data loss from skipping verification)
- Make safety pattern the FIRST major lesson (Lesson 2) before any destructive operations
- Every single file operation example MUST show safety pattern in full
- Include exercises where students identify missing safety steps in flawed dialogues
- Emphasize: "Fast and wrong is slower than careful and right"
- Show that professional developers ALWAYS verify before destructive operations

### Risk 4: Cognitive Overload from Too Many Concepts in Early Lessons

**Likelihood**: Medium | **Impact**: High

**Mitigation**:
- Strictly enforce 5-concept maximum per lesson section
- Break complex workflows into multiple lessons (navigation → safety → operations → configuration → orchestration)
- Use consistent structure and patterns throughout chapter
- Repeat key concepts (safety pattern) across multiple lessons
- Provide concept count at the end of each lesson section for self-monitoring

### Risk 5: Students Using Windows Without WSL Have Different Bash Experience

**Likelihood**: Low | **Impact**: Medium

**Mitigation**:
- Explicitly state in prerequisites: "macOS, Linux, or Windows with WSL required"
- Note that PowerShell is different (out of scope) but AI can translate if needed
- Use cross-platform commands where possible (`ls`, `cd`, `pwd`, `mkdir`)
- Avoid macOS-specific or Linux-specific commands in core examples
- Acknowledge Windows users may see slight differences and should ask AI for translation

## Open Questions *(optional)*

1. Should we include a "Bash for AI Agents" section explaining how AI tools internally use bash? (Helps students understand the tool's perspective)
2. Should we demonstrate multiple AI tools side-by-side in same example to show variability? (Pro: realistic, Con: cognitive load)
3. Should we include a troubleshooting appendix with common error messages and their meanings? (Quick reference for students)
4. Should we show examples of bash commands students should NOT use (dangerous patterns) with explanations? (Safety education)
5. How much emphasis should we put on git bash integration vs. keeping git for Chapter 8? (Navigation uses `git status` naturally)

## Related Documentation

- **Constitution v3.0.0**: AI-native teaching principles, graduated complexity tiers, cognitive load management
- **Chapter Index**: Chapter 7 is Part 2 (AI Tool Landscape), beginner tier content
- **Lesson Template**: `.claude/output-styles/lesson.md` for consistent structure
- **Existing Chapter 7**: `/book-source/docs/02-AI-Tool-Landscape/07-bash-essentials/` (current implementation to be redesigned)

## Revision History

| Date | Version | Changes | Author |
|------|---------|---------|--------|
| 2025-11-07 | 1.0 | Initial specification created | Claude Code (Main Orchestrator) |
