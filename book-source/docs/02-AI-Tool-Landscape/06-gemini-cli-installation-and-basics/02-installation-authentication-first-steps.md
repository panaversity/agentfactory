---
sidebar_position: 2
title: Installation, Authentication & First Steps
---

# Installation, Authentication & First Steps

> **A Word Before We Begin**
> 
> Installing Gemini CLI is like meeting a new colleague—someone who's available 24/7 to help with your work, answer questions, and solve problems. In this lesson, you'll install and launch Gemini CLI, which will then automatically guide you through authentication. You'll be up and running in minutes.


## Prerequisites: What You Need

Make sure you have these before starting:

| Requirement | What It Is | How to Check |
|------------|-----------|-------------|
| **Node.js 20+** | Runtime for JavaScript applications | Open terminal, type: `node --version` |
| **npm** | Package manager (comes with Node.js) | Open terminal, type: `npm --version` |
| **Internet connection** | For installation and Google authentication | You have this already ✓ |
| **Google account** | For secure authentication | Gmail, YouTube, or any Google account |

### Don't Have Node.js 20+?

1. Visit [nodejs.org](https://nodejs.org/en/download)
2. Download the **LTS version** (Long Term Support—the stable version)
3. Follow the installer steps for your operating system
4. When asked about npm, keep the checkbox checked
5. Restart your computer

### Opening Your Terminal

**Windows:** Search "PowerShell" in your Start menu and open it

**macOS:** Press Cmd+Space, type "Terminal", press Enter

**Linux:** Press Ctrl+Alt+T (most distributions)

---

## Installation: One Command

Open your terminal and run this single command:

```bash
npm install -g @google/gemini-cli
```

This command downloads and installs Gemini CLI globally on your computer. You'll see text flowing by—this is normal. Wait for it to complete (usually takes 30-60 seconds).

### Verify Installation

After installation completes, verify it worked:

```bash
gemini -v
```

You should see a version number like `0.4.0` or higher. If you see this, installation is successful! ✓

---

## Authentication & First Launch

Now comes the magic—Gemini CLI handles authentication automatically. Simply type:

```bash
gemini
```

When you run this command for the first time, Gemini CLI launches and **automatically guides you through setup**:

### Step 1: Choose Your Theme

Gemini CLI will ask you to select a visual theme for the terminal interface. Choose whichever you prefer—this is just cosmetic. Use arrow keys to select and press Enter.

### Step 2: Choose Authentication Method

You'll see options for authentication:
- **Google login** (free tier: 60 requests/min, 1,000 requests/day)
- **Gemini API Key** (requires API setup)
- **Vertex AI** (requires Google Cloud Project)

**Select "Google login"** for the free tier. This is the beginner-friendly option.

### Step 3: Browser Opens

Your default web browser will automatically open with Google's login page. Simply:
1. Enter your Google account email
2. Enter your password
3. Click "Allow" when Google asks for permission

### Step 4: You're In!

After you authorize, your terminal displays the Gemini CLI interface, and you're ready to start. You'll see a prompt where you can type your questions and commands.

---

## Your First Task with Gemini

Now that you're inside Gemini CLI, you're ready to put your AI collaborator to work. Simply type your question or request and press Enter:

```
Help me understand what artificial intelligence means
```

Gemini will respond with an explanation. That's it—you're using Gemini CLI!

---

## Essential Slash Commands: Your Command Toolkit

Now that you're inside Gemini CLI, you have access to powerful slash commands that control your AI session. Think of these as your control panel—you don't need to memorize them, but knowing they exist helps you work more efficiently.

### Why Slash Commands Matter

When you type `/` followed by a command name, you're telling Gemini CLI to execute a specific function. This is different from asking Gemini a question—slash commands control **how** Gemini works, not **what** it knows.

### The Essential Commands

Here are the slash commands you'll use most often, organized by purpose:

#### Help & Information

- **`/help`** — Show all available commands and shortcuts
  - Use when: You forget what commands are available
  - Example: Type `/help` to see complete command list
  - Output: Table of all slash commands with brief descriptions

- **`/tools`** — List all available tools Gemini can use
  - Use when: You want to know what capabilities Gemini has (file operations, web search, shell commands)
  - Example: Type `/tools` to see tools like `read_file`, `web_fetch`, `run_shell_command`
  - Output: List of tool names with descriptions

- **`/stats`** — Show session statistics and token usage
  - Use when: You want to know how much context you're using
  - Example: Type `/stats` to see current token count, session duration, compression status
  - Output: Token count, % of context window used, session time
  - **Business value**: Monitor your 1M token budget to avoid hitting limits

#### Context Management

- **`/compress`** — Intelligently reduce context size while preserving key information
  - Use when: Your context is getting large (70-80% of capacity) and you want to continue working
  - Example: After analyzing 50 files, type `/compress` to reduce context by 60-80%
  - Output: "Context compressed from 800K tokens to 200K tokens"
  - **Business value**: Continue working on large codebases without restarting

- **`/clear`** — Clear the screen (not the context)
  - Use when: Your terminal is cluttered and you want a clean workspace
  - Example: Type `/clear` between different tasks for visual clarity
  - Output: Clean screen, conversation context preserved

- **`/copy`** — Copy Gemini's last response to clipboard
  - Use when: You want to paste Gemini's code or explanation elsewhere
  - Example: After Gemini generates code, type `/copy` then paste into your editor
  - Output: "Response copied to clipboard"

#### Memory & Context Files

- **`/memory show`** — View all loaded GEMINI.md context files
  - Use when: You want to see what context Gemini has about your project
  - Example: Type `/memory show` to see combined global + project + subdirectory GEMINI.md content
  - Output: Full text of all loaded context files
  - **Explained in detail in Lesson 4**: Memory Management

- **`/memory add <text>`** — Add a quick fact to your global memory
  - Use when: You want Gemini to remember something permanently
  - Example: `/memory add My API keys are always in .env.local, never .env`
  - Output: "Memory added to global GEMINI.md"
  - **Explained in detail in Lesson 4**: Memory Management

- **`/memory reload`** — Refresh GEMINI.md files without restarting
  - Use when: You edited a GEMINI.md file and want changes to apply immediately
  - Example: After updating project GEMINI.md, type `/memory reload`
  - Output: "Memory reloaded from all GEMINI.md files"

#### Conversation Management

- **`/chat save <tag>`** — Save current conversation state
  - Use when: You want to pause work on this topic and return later
  - Example: `/chat save competitor-research` saves entire conversation
  - Output: "Conversation saved as 'competitor-research'"
  - **Business value**: Branch conversations—try different approaches without losing progress

- **`/chat resume <tag>`** — Resume a saved conversation
  - Use when: You want to continue a previous conversation
  - Example: `/chat resume competitor-research` loads entire previous conversation
  - Output: Previous conversation context restored

- **`/restore`** — List and restore file checkpoints
  - Use when: Gemini made file changes you want to undo
  - Example: After Gemini edits 5 files, type `/restore` to list checkpoints, then `/restore <id>` to undo
  - Output: List of available file checkpoints with IDs
  - **Business value**: Safe experimentation—undo file changes if needed

#### Configuration & Customization

- **`/settings`** — Open settings.json for editing
  - Use when: You want to configure Gemini CLI (themes, tool restrictions, context settings)
  - Example: Type `/settings` to edit configuration file
  - Output: Opens settings.json in your default text editor
  - **Explained in detail in Lesson 3**: Configuration System

- **`/init`** — Generate a starter GEMINI.md file
  - Use when: Starting a new project and want a template context file
  - Example: Type `/init` to create `.gemini/GEMINI.md` with structure
  - Output: Creates GEMINI.md with sections: Project Context, Coding Standards, Architecture

- **`/theme`** — Change CLI visual theme
  - Use when: You want a different color scheme
  - Example: Type `/theme` to select from available themes
  - Output: Interactive theme selector

#### MCP Servers & Extensions

- **`/mcp`** — List configured MCP servers and their status
  - Use when: You want to verify which external tools are connected
  - Example: Type `/mcp` to see Playwright, Context7, GitHub servers
  - Output: List of MCP servers with connection status (connected/disconnected)
  - **Explained in detail in Lesson 5**: MCP Servers

#### IDE Integration

- **`/ide install`** — Install IDE integration (VS Code)
  - Use when: You want Gemini CLI integrated into your code editor
  - Example: Type `/ide install` to set up VS Code extension
  - Output: Installation instructions for IDE integration

- **`/ide enable`** — Enable IDE mode
  - Use when: You've installed IDE integration and want to activate it
  - Example: Type `/ide enable` to turn on IDE features
  - Output: "IDE mode enabled—workspace context auto-loaded"

#### Exit

- **`/quit`** — Exit Gemini CLI and return to your terminal
  - Use when: You're done with your session
  - Example: Type `/quit` to close Gemini
  - Alternative: Press **Ctrl+C** twice to force quit
  - Output: Returns to terminal prompt

### Try It Now: Practice with 3 Commands

Let's practice using slash commands right now. Open Gemini CLI and try these:

1. **Check your tools**: Type `/tools`
   - You'll see a list of all capabilities Gemini has (file operations, web search, shell commands)
   - This helps you understand what Gemini can do beyond answering questions

2. **Monitor your context**: Type `/stats`
   - You'll see how many tokens you're using and how much capacity remains
   - Example output: "Context: 15,234 tokens (1.5% of 1M limit)"

3. **Get help anytime**: Type `/help`
   - You'll see the complete command reference
   - Bookmark this mentally—when you forget a command, `/help` has the answer

**Key Insight**: You don't need to memorize these commands. Type `/help` whenever you need a reminder. The commands are there to make your work easier, not to add complexity.

---

## @ Syntax: Giving Your AI Context About Files

One of Gemini CLI's most powerful features is the ability to reference files and directories directly in your prompts. This is how you give Gemini access to your codebase, documents, or any file on your computer.

### The Concept: @ as "Look at This File"

When you type `@` followed by a file path, you're telling Gemini: **"Read this file and include it in our conversation."**

Think of `@` as pointing—you're pointing Gemini to specific files or folders you want it to analyze.

### Four Essential @ Patterns

#### 1. Single File Reference

**Syntax**: `@./path/to/file.ext`

**Example**:
```
@./src/app.js Explain what this file does
```

**What happens**: Gemini reads `src/app.js`, includes its content in context, then explains the code.

**Business use case**: Code review
```
@./src/payment-processor.js Does this code handle errors properly? Are there any security issues?
```

---

#### 2. Multiple File Comparison

**Syntax**: `@./file1.ext @./file2.ext`

**Example**:
```
@./src/old-api.js @./src/new-api.js What changed between these two files?
```

**What happens**: Gemini reads both files and compares them.

**Business use case**: Understanding refactoring
```
@./components/Button-v1.tsx @./components/Button-v2.tsx Explain the improvements in v2
```

---

#### 3. Directory (Recursive)

**Syntax**: `@./directory/`

**Example**:
```
@./src/ What architectural patterns does this codebase use?
```

**What happens**: Gemini recursively reads **all files** in the `src/` directory and subdirectories.

**⚠️ Warning**: Large directories can exceed context limits. Use specific subdirectories when possible.

**Business use case**: Codebase architecture analysis
```
@./competitor-repo/ Analyze their code structure. What frameworks and patterns do they use?
```

---

#### 4. Media Files (Images, PDFs, Audio, Video)

**Syntax**: `@./path/to/media.ext`

**Supported types**: `.png`, `.jpg`, `.jpeg`, `.gif`, `.pdf`, `.mp3`, `.mp4`, `.wav`

**Example (Image)**:
```
@./diagram.png Explain this architecture diagram and identify any potential issues
```

**Example (PDF)**:
```
@./quarterly-report.pdf Summarize the key findings from this Q3 report
```

**Example (Audio)**:
```
@./meeting-recording.mp3 Transcribe this meeting and extract action items
```

**Business use case**: Document analysis
```
@./competitor-pricing.pdf @./our-pricing.pdf Create a comparison table
```

---

### Practical Examples for Business Developers

**Scenario 1: Competitive Analysis**
```
@./competitor-repo/ What patterns do they use that we don't? Identify 3 features we should consider.
```

**Scenario 2: Code Documentation**
```
@./src/utils/ Generate documentation for all utility functions. Include parameters, return values, and examples.
```

**Scenario 3: Multi-File Debugging**
```
@./src/app.js @./src/database.js @./src/auth.js I'm getting a "user not found" error. Where's the bug?
```

**Scenario 4: Architecture Review**
```
@./project-root/ Analyze the project structure. Is it following best practices for a [your stack] application?
```

---

### Supported File Types

Gemini CLI can read and analyze these file types with `@` syntax:

**Code**: `.js`, `.ts`, `.jsx`, `.tsx`, `.py`, `.java`, `.go`, `.rb`, `.php`, `.c`, `.cpp`, `.cs`, `.swift`, `.kt`, `.rs`

**Data**: `.json`, `.csv`, `.xml`, `.yaml`, `.yml`, `.toml`, `.ini`, `.env`

**Documents**: `.md`, `.txt`, `.pdf`, `.doc`, `.docx`

**Media**: `.png`, `.jpg`, `.jpeg`, `.gif`, `.svg`, `.mp3`, `.mp4`, `.wav`, `.mov`

**Config**: `.gitignore`, `.dockerignore`, `.eslintrc`, `.prettierrc`, `Dockerfile`, `package.json`, `tsconfig.json`

---

### Important Considerations

**Context Window Limits**:
- Large directories may exceed the 1M token limit
- If Gemini says "context too large", reference specific subdirectories instead
- Use `/stats` to monitor context usage

**File Paths**:
- Relative paths: `@./src/app.js` (relative to current working directory)
- Absolute paths: `@/Users/you/project/src/app.js` (full system path)
- Use `.` for current directory, `..` for parent directory

**Security**:
- Gemini only reads files you explicitly reference with `@`
- It cannot access files you don't point to
- Always review what files you're sharing before using `@`

---

### Hands-On Exercise 1: Single File Analysis

**Task**: Create a test file and analyze it with Gemini.

1. Create a file `test.js`:
   ```javascript
   function calculateTotal(items) {
     return items.reduce((sum, item) => sum + item.price, 0);
   }
   ```

2. In Gemini CLI, type:
   ```
   @./test.js Explain what this function does and suggest improvements
   ```

3. Gemini will:
   - Read the file
   - Explain the `reduce` function
   - Suggest improvements (error handling, TypeScript types, etc.)

**Expected outcome**: Clear explanation + 2-3 improvement suggestions

---

### Hands-On Exercise 2: Directory Architecture Analysis

**Task**: Analyze your project structure.

1. Navigate to a code project directory in your terminal

2. In Gemini CLI, type:
   ```
   @./src/ Analyze the architecture and identify the main components
   ```

3. Gemini will:
   - Read all files in `src/`
   - Identify patterns (MVC, microservices, etc.)
   - List main components with their responsibilities

**Expected outcome**: Architecture summary with component breakdown

**⚠️ Note**: If you get "context too large", try a specific subdirectory:
```
@./src/components/ Analyze just the components directory
```

---

## ! Syntax: Executing Shell Commands with AI Guidance

Gemini CLI can execute terminal commands for you—with safety guardrails. The `!` syntax gives Gemini access to your shell, enabling it to run git commands, test suites, file operations, and more.

### The Concept: ! as "Run This Command"

When you type `!` followed by a command, you're telling Gemini: **"Execute this shell command in my terminal."**

There are two modes:
1. **Single command**: `!git status` — Run one command
2. **Interactive shell mode**: Type `!` alone — Enter shell session

---

### Mode 1: Single Shell Commands

**Syntax**: `!command arg1 arg2`

**Example**:
```
!git status
```

**What happens**:
1. Gemini asks for confirmation: "Execute `git status`? (yes/no)"
2. You approve
3. Command runs
4. Output is shown and included in conversation context

**Business use cases**:

```
!git log --oneline -10
Then ask: "Based on these commits, what features were added this week?"
```

```
!npm run test
Then ask: "Which tests failed and why?"
```

```
!ls -la
Then ask: "Are there any suspicious files or security issues?"
```

---

### Mode 2: Interactive Shell Mode

**Syntax**: Type `!` alone (no command after it)

**What happens**:
1. You enter **shell mode** (prompt changes to indicate shell)
2. You can run multiple commands interactively
3. Type `exit` or press `Esc` to return to Gemini mode

**Example workflow**:
```
!                      ← Enter shell mode
git status             ← Run command 1
git diff               ← Run command 2
git add .              ← Run command 3
git commit -m "fix"    ← Run command 4
exit                   ← Exit shell mode, back to Gemini
```

**Business value**: Execute multi-step git workflows without leaving Gemini.

---

### Safety Features: You're in Control

Gemini CLI has safety guardrails for shell commands:

**1. Confirmation Prompts**:
- Every shell command requires your approval
- You see exactly what will run before it executes
- You can decline with "no" or "n"

**2. Tool Restrictions** (configured in settings.json):
- Block dangerous commands (e.g., `rm -rf`, `sudo`)
- Auto-approve safe commands (e.g., `git status`, `ls`)
- **Explained in Lesson 3**: Configuration & Tool Restrictions

**3. Command Preview**:
- Gemini shows you the exact command before running
- You can modify or cancel if something looks wrong

---

### Practical Business Examples

#### Example 1: Git Workflow Analysis
```
!git log --oneline -10

Then ask: "Summarize what changed in the last 10 commits. Are there any breaking changes?"
```

**What Gemini does**:
1. Runs `git log --oneline -10`
2. Sees commit messages
3. Analyzes and summarizes changes
4. Identifies potential breaking changes

---

#### Example 2: Test Suite Debugging
```
!npm run test

Then ask: "Why did the authentication tests fail? Suggest fixes."
```

**What Gemini does**:
1. Runs test suite
2. Captures test output and error messages
3. Analyzes failures
4. Provides specific fix recommendations

---

#### Example 3: Dependency Investigation
```
!npm list --depth=0

Then ask: "Which of these dependencies are outdated? Which updates are risky?"
```

**What Gemini does**:
1. Lists installed packages
2. Identifies outdated versions
3. Warns about major version changes
4. Recommends safe update path

---

#### Example 4: Interactive Git Session
```
!                           ← Enter shell mode
git status                  ← Check status
git diff src/payment.js     ← See what changed
git add src/payment.js      ← Stage changes
git commit -m "Fix payment bug"  ← Commit
exit                        ← Back to Gemini

Then ask: "Create a detailed commit message for what I just committed, based on the diff."
```

---

### Security Considerations

**Safe Commands** (typically auto-approved or low-risk):
- `git status`, `git log`, `git diff` — Read-only git operations
- `ls`, `pwd`, `cat` — File system reading
- `npm list`, `npm outdated` — Package info
- `echo`, `date`, `whoami` — System info

**Dangerous Commands** (block or be very careful):
- `rm -rf` — Deletes files permanently
- `sudo` — Runs with admin privileges
- `dd` — Low-level disk operations
- `chmod 777` — Overly permissive file permissions
- `curl | bash` — Executes remote scripts

**Best Practice**: Configure `excludeTools` in settings.json to block dangerous commands entirely (Lesson 3).

Example settings.json snippet:
```json
{
  "tools": {
    "exclude": ["run_shell_command(rm)", "run_shell_command(sudo)"]
  }
}
```

---

### Hands-On Exercise 1: Git Status Check

**Task**: Use `!` to check git status and get AI analysis.

1. Navigate to a git repository

2. In Gemini CLI, type:
   ```
   !git status
   ```

3. Approve the command when prompted

4. Then ask:
   ```
   Based on this status, what files should I commit and what should I ignore?
   ```

**Expected outcome**: Gemini reads git status and recommends which changes to commit.

---

### Hands-On Exercise 2: Interactive Shell Mode

**Task**: Enter shell mode and run multiple git commands.

1. In Gemini CLI, type:
   ```
   !
   ```

2. You're now in shell mode. Run:
   ```
   git status
   git log --oneline -5
   git diff
   exit
   ```

3. Back in Gemini mode, ask:
   ```
   Summarize what I just saw. Are there uncommitted changes I should address?
   ```

**Expected outcome**: Gemini analyzes the shell output and provides recommendations.

---

## Keyboard Shortcuts: Quick Commands

Beyond slash commands, Gemini CLI offers keyboard shortcuts that work immediately without typing full command names. These are useful for frequent operations.

### Essential Keyboard Shortcuts

**Ctrl+C** — Exit current input (or force quit if pressed twice)
- Use when: You're in the middle of typing and want to start over
- Works inside Gemini CLI and shell mode

**Tab** — Autocomplete commands and file paths
- Use when: You're typing a slash command or file path
- Press once to complete, press again for alternatives
- Example: Type `/me` + Tab → autocompletes to `/memory`

**Escape** — Exit shell mode, return to Gemini mode
- Use when: You're in shell mode (`!`) and want to go back to Gemini
- Alternative: Type `exit`

**Ctrl+L** — Clear the screen (same as `/clear`)
- Use when: Terminal is cluttered and you want visual clarity
- Note: Does not clear conversation context

**Ctrl+K** — Toggle command palette
- Use when: You want to search for available commands
- Helpful when you forget a command name but remember part of it

**Ctrl+P** — Previous command (in shell mode)
- Use when: You want to repeat a command you just ran
- Cycles through command history

**Ctrl+R** — Search command history (in shell mode)
- Use when: You want to find a command you ran earlier
- Type partial command to search

**Ctrl+D** — Send EOF signal (exit at shell prompt)
- Use when: You're in shell mode and want to return to Gemini
- Alternative to typing `exit`

**Arrow Keys** (Up/Down) — Navigate command history
- Use when: You want to repeat or edit a previous command
- Works in both Gemini mode and shell mode

**Arrow Keys** (Left/Right) — Move cursor within command
- Use when: You need to edit a command before pressing Enter

---

## Understanding Gemini CLI Session Workflow

When you run `gemini`, you're entering an interactive session. Inside this session, you have access to powerful commands and can ask Gemini multiple questions without exiting.

### Session Workflow: A Real Example

Here's what a typical session looks like:

**Step 1: Launch Gemini**
```bash
gemini
```

**Step 2: Inside the session, ask your first question**
```
Explain what a REST API is using a real-world example
```

**Step 3: Gemini responds**
```
A REST API is a system that allows applications to communicate with each other...
[Gemini's detailed response]
```

**Step 4: Use @ syntax to analyze actual code**
```
@./src/api.js Does this API implementation follow REST principles?
```

**Step 5: Use ! syntax to verify with tests**
```
!npm run test -- api.test.js
Then ask: "Based on these test results, what's working and what needs fixing?"
```

**Step 6: Use slash commands to manage your session**
```
/stats
Shows how many tokens you've used
```

**Step 7: When you're done, exit**
```
/quit
```

---

## Try With AI: Practice All Core Features

Now that you understand the essential commands, syntax, and shortcuts, let's practice them in an integrated workflow.

### Exercise: End-to-End Gemini Session

Open Gemini CLI and work through this exercise:

**Setup**: Navigate to a project you have locally or create a simple test file.

**Part 1: Practice Slash Commands** (5 minutes)
1. Type `/stats` — Monitor your token usage
2. Type `/tools` — See what Gemini can do
3. Type `/help` — Review the command reference
4. Type `/memory show` — Check if there's any saved context (probably empty right now)

**Part 2: Practice @ Syntax** (5 minutes)
- If you have a code file:
  ```
  @./src/your-file.js Explain what this code does and suggest 1 improvement
  ```
- If you have a document:
  ```
  @./README.md Summarize the main points from this README
  ```

**Part 3: Practice ! Syntax** (5 minutes)
- Try a safe command:
  ```
  !git status
  Then ask: "What does this status tell me about my repository?"
  ```

**Part 4: Real Business Scenario** (10 minutes)
```
I need to understand our project's architecture.
@./src/ Can you analyze the code structure and identify the main layers?
Then tell me: Are there any architectural patterns I should recognize?
What would be the first thing you'd refactor and why?
```

**Expected outcomes**:
- You see how slash commands control your session
- You understand that `@` gives Gemini access to your files
- You see how `!` lets Gemini execute commands and include output in analysis
- You discover that combining all three (slash commands + @ + !) creates powerful workflows

**Business value**: You've now mastered the core syntax of Gemini CLI. Every feature you'll learn in future lessons builds on these foundations.

