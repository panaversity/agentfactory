---
sidebar_position: 3
title: "Core Commands & Slash Commands"
duration: "40 min"
---

# Core Commands & Slash Commands

## From Setup to Daily Use: Mastering Gemini CLI Commands

In Lesson 2, you installed Gemini CLI and saw it work for the first time. Now comes the crucial step: **learning the commands you'll use every day** to work efficiently with your AI development partner.

This isn't about memorizing syntax. It's about understanding *when* to use each command, *why* it matters, and *how* to orchestrate your AI work like a professional.

---

## Understanding Gemini CLI's Three Input Methods

Gemini CLI has three ways to interact:

**1. Slash Commands (/)** — Meta-level control
- Control Gemini CLI behavior
- Manage conversations and context
- Configure settings
- Access integrations

**2. At Commands (@)** — File/directory inclusion
- Reference files and folders
- Include content in prompts
- Automatic smart filtering

**3. Shell Commands (!)** — System access
- Execute bash/cmd commands
- Interactive shell mode
- AI-guided execution

Let's dive into each.

---

## Slash Commands: The Complete Reference

Gemini CLI provides 29+ slash commands. Here are the essential ones you'll use daily:

### Conversation Management

| Command | Purpose | Example |
|---------|---------|---------|
| **`/chat save <tag>`** | Save current conversation | `/chat save auth-implementation` |
| **`/chat resume <tag>`** | Restore saved conversation | `/chat resume auth-implementation` |
| **`/chat list`** | List saved conversations | `/chat list` |
| **`/chat delete <tag>`** | Remove saved conversation | `/chat delete auth-implementation` |
| **`/chat share file.md`** | Export conversation to file | `/chat share output.md` |
| **`/clear`** | Clear terminal display | `/clear` |

### Context & Memory Management

| Command | Purpose | Example |
|---------|---------|---------|
| **`/memory add <text>`** | Add to AI's context | `/memory add we use TypeScript strict mode` |
| **`/memory show`** | Display loaded context | `/memory show` |
| **`/memory refresh`** | Reload GEMINI.md files | `/memory refresh` |
| **`/memory list`** | Show active GEMINI.md paths | `/memory list` |
| **`/directory add <path>`** | Add workspace directory | `/directory add ./shared-libs` |
| **`/directory show`** | Show workspace directories | `/directory show` |

### System & Configuration

| Command | Purpose | Example |
|---------|---------|---------|
| **`/settings`** | Open settings editor | `/settings` |
| **`/theme`** | Change terminal theme | `/theme` |
| **`/vim`** | Toggle vim-mode editing | `/vim` |
| **`/editor`** | Select default editor | `/editor` |
| **`/auth`** | Change authentication | `/auth` |
| **`/help` or `/?`** | Show help | `/help` |
| **`/about`** | Show version info | `/about` |

### Tools & Integrations

| Command | Purpose | Example |
|---------|---------|---------|
| **`/tools [desc]`** | List available tools | `/tools` or `/tools read file` |
| **`/mcp`** | Show MCP servers & tools | `/mcp` |
| **`/extensions`** | List active extensions | `/extensions` |

### Optimization & Restoration

| Command | Purpose | Example |
|---------|---------|---------|
| **`/stats`** | Show token usage | `/stats` |
| **`/compress`** | Reduce context size | `/compress` |
| **`/copy`** | Copy last response | `/copy` |
| **`/restore [id]`** | Undo file modifications | `/restore` |

### Utility

| Command | Purpose | Example |
|---------|---------|---------|
| **`/bug`** | Report issue | `/bug command X doesn't work` |
| **`/privacy`** | View privacy notice | `/privacy` |
| **`/quit` or `/exit`** | Exit Gemini CLI | `/quit` |
| **`/init`** | Create GEMINI.md | `/init` |

---

## Deep Dive: Commands You'll Use Most

### 1. `/chat save` & `/chat resume` — Conversation Checkpointing

**Purpose**: Save conversations and resume them later without re-explaining context

**Syntax**:
```bash
/chat save my-project-v1
(Later, in new session...)
/chat resume my-project-v1
```

**When to use**:
- After completing a meaningful chunk of work
- Before starting something unrelated
- End of day (save progress)
- Beginning of day (resume work)

**Example Workflow**:
```
You: [Work on authentication for 2 hours]

You: /chat save auth-endpoints-complete
Gemini: ✓ Saved conversation as "auth-endpoints-complete"

(Later, next day...)

You: /chat resume auth-endpoints-complete
Gemini: ✓ Resumed conversation "auth-endpoints-complete"
        [Shows your conversation history]

You: Now let's add password reset functionality
(Continues seamlessly from yesterday)
```

**Why it matters**: Without saved conversations, you'd need to re-explain your project every session. With `/chat save/resume`, context persists.

---

### 2. `/memory add`, `/memory show`, `/memory refresh` — Context Management

**Purpose**: Manage AI's long-term knowledge about your project

These commands work with **GEMINI.md files** (covered in Lesson 4).

**Syntax**:
```bash
/memory add we use strict TypeScript and functional patterns
/memory show          # Shows all loaded context
/memory refresh       # Reloads GEMINI.md files
/memory list          # Shows which GEMINI.md files are loaded
```

**How it works**:
1. Gemini CLI looks for GEMINI.md files (global, project, subdirectory)
2. `/memory show` displays what's currently loaded
3. `/memory refresh` reloads if you update GEMINI.md
4. `/memory add` adds temporary facts to the session

**Example**:
```
You: /memory show
Gemini CLI:
  Loaded from: ~/.gemini/GEMINI.md
  - My preferences (Python + TypeScript)
  - My coding style

  Loaded from: ./GEMINI.md
  - Project architecture
  - Team standards

You: /memory add API returns ISO 8601 dates
Gemini: ✓ Added to context
```

---

### 3. `/stats` — Token Budget Awareness

**Purpose**: Monitor how many tokens you've used

**Syntax**:
```bash
/stats
```

**Output**:
```
Session Statistics:
- Tokens used: 125,000 / 1,000,000
- Remaining: 875,000
- Session duration: 2h 45m
- Messages: 42
```

**Why it matters**: Gemini CLI has a 1 million token context window. Long sessions can approach the limit. `/stats` tells you when to start a new conversation or use `/compress`.

---

### 4. `/compress` — Reduce Context Size

**Purpose**: Summarize conversation to free up tokens

**Syntax**:
```bash
/compress
```

**What happens**:
- Gemini replaces your conversation with a summary
- Frees up 40-60% of tokens
- Keeps key decisions and code context

**Example**:
```
You: [After 3 hours of work, 250K tokens used]
You: /stats
Gemini: Used 250,000 / 1,000,000 tokens

You: /compress
Gemini: ✓ Compressed conversation. Now using 90,000 tokens
        (Freed 160,000 tokens for continued work)
```

---

### 5. `/clear` — Start Fresh

**Purpose**: End current conversation and start new one

**Syntax**:
```bash
/clear
```

**Important**: Use `/chat save` before `/clear` if you want to preserve work.

```bash
You: /chat save current-work
Gemini: ✓ Saved

You: /clear
Gemini: ✓ Conversation cleared

You: [Starting completely new task with fresh context]
```

---

### 6. `/tools` — Discover Available Tools

**Purpose**: See what tools Gemini can use

**Syntax**:
```bash
/tools                # List all tools
/tools search         # Filter tools by keyword
```

**Output**:
```
Available Tools:
- GoogleSearch — Search the web
- WriteFile — Create/modify files
- ReadFile — Read file contents
- WebFetch — Fetch URL content
- RunShell — Execute shell commands
```

---

### 7. `/mcp` — Manage External Integrations

**Purpose**: List configured MCP servers and their tools

**Syntax**:
```bash
/mcp
```

**Output**:
```
Configured MCP Servers:
- playwright (✓ connected) — Web browsing
- context7 (✓ connected) — Documentation lookup
- github (✓ connected) — Repository access
```

---

### 8. `/directory add` & `/directory show` — Workspace Management

**Purpose**: Include additional directories in Gemini's context discovery

**Syntax**:
```bash
/directory add ../shared-libs        # Add a directory
/directory show                      # See all included directories
```

**Why useful**: If you work with monorepos or shared libraries, tell Gemini where to find them.

---

## At Commands: Including Files & Directories

### `@` Syntax for File References

**Purpose**: Include file or directory content in your prompt

**Syntax**:
```bash
@path/to/file.js                # Single file
@./src/                         # Entire directory
@./docs/                        # Read all files in directory
@image.png                      # Image file
@document.pdf                   # PDF file
```

**Examples**:
```bash
You: @./src/auth.ts explain this authentication flow

You: @./docs/ summarize the project documentation

You: Compare @./old-design.ts and @./new-design.ts

You: Read @./config.json and tell me the API endpoint
```

**Smart Filtering**: Gemini CLI automatically excludes:
- `node_modules/`
- `.env` files
- `.git/` directories
- Other common non-code files

---

## Shell Commands: System Access

### `!` Syntax for Shell Execution

**Purpose**: Run system commands with AI guidance

**Syntax**:
```bash
!<command>              # Run single command
!                       # Toggle shell mode
```

**Examples**:
```bash
You: !git status
Gemini: [Executes command, shows output, can discuss results]

You: !
Gemini: [Enters shell mode - all commands execute without !]
$ npm test
$ git log --oneline
$ exit                  # Exit shell mode back to chat
```

**Safety**: Gemini can warn you about dangerous commands before executing.

---

## Keyboard Shortcuts

Quick actions while typing:

| Shortcut | Action |
|----------|--------|
| `Ctrl+L` | Clear terminal (same as `/clear`) |
| `Ctrl+Z` | Undo last input |
| `Ctrl+Shift+Z` | Redo undone input |

---

## Practical Daily Workflow

Here's how these commands flow in real work:

### Morning: Resume Previous Work

```bash
$ gemini
Gemini: Loading context from ~/.gemini/GEMINI.md and ./GEMINI.md

You: /chat resume api-endpoints-in-progress
Gemini: ✓ Resuming conversation
        (Shows your conversation history from yesterday)

You: Let's continue with the user endpoints
(Continues exactly where you left off)
```

### Mid-Day: Monitor Budget

```bash
You: [After 2 hours of work]

You: /stats
Gemini: Used 200,000 / 1,000,000 tokens remaining

You: We're in good shape, let's keep going
```

### Switch Projects

```bash
You: /chat save api-endpoints-complete
Gemini: ✓ Saved

You: /clear
Gemini: ✓ Fresh conversation started

You: /chat resume frontend-redesign
Gemini: ✓ Resuming previous project
```

### End of Day: Save Progress

```bash
You: /chat save current-work-session
Gemini: ✓ Saved

You: Good progress today. See you tomorrow!
(Exit Gemini CLI)
```

### Next Day: Resume Seamlessly

```bash
$ gemini

You: /chat resume current-work-session
Gemini: ✓ Loaded. Where were we...
```

---

## Common Questions

**Q: What's the difference between `/chat save` and GEMINI.md?**

A:
- `/chat save` = Saves your CONVERSATION (what you discussed)
- `GEMINI.md` = Persistent PROJECT CONTEXT (what Gemini should know about your project)

Use both together:
- GEMINI.md tells Gemini about your architecture
- `/chat save` remembers what you've discussed this session

**Q: Can I use `/chat resume` to work with someone else's saved conversation?**

A: Not directly. Saved conversations are personal. But you can use `/chat share file.md` to export, then share the file.

**Q: If I use `/compress`, will I lose my conversation?**

A: No. It summarizes your conversation but keeps important decisions and context. You can still reference previous work.

**Q: What happens if I close Gemini without using `/chat save`?**

A: Your conversation is lost. Always save important work with `/chat save`.

**Q: Can I use shell commands safely?**

A: Yes. Gemini will warn you about dangerous commands (rm, sudo, etc.) before executing. Always read the warning before confirming.

---

## Exercises

### Exercise 1: Save Your First Conversation

Have a real conversation about something meaningful:

```bash
$ gemini
You: [Work on a real task for 20+ minutes]
You: /chat save my-first-saved-conversation
Gemini: ✓ Saved
```

Expected: You can now list and resume this conversation anytime.

### Exercise 2: List and Inspect

```bash
You: /chat list
(See all saved conversations)

You: /chat save another-one
You: /chat list
(See the new conversation)
```

### Exercise 3: Resume and Continue

Close Gemini CLI completely. Then:

```bash
$ gemini
You: /chat resume my-first-saved-conversation
```

Expected: Your conversation history is restored.

### Exercise 4: Check Your Token Budget

```bash
You: /stats
(See how many tokens you've used)

You: (Work for 10 more minutes)

You: /stats
(See tokens increased)
```

### Exercise 5: Reference a File

Create a simple file:

```bash
$ echo "console.log('hello')" > test.js
```

Then in Gemini:

```bash
You: @test.js what does this code do?
Gemini: (Shows file content and explains)
```

---

## Key Takeaways

- **29+ slash commands** provide meta-level control (don't memorize—reference as needed)
- **`/chat save` and `/chat resume`** are your superpowers for conversation persistence
- **`/memory show` and `/memory refresh`** work with GEMINI.md for project context (covered next)
- **`/stats` and `/compress`** keep you aware of token budget
- **`@` syntax** includes file content without typing `/copy-paste`
- **`!` syntax** runs shell commands with AI guidance
- **Commands are discoverable**—`/help` shows everything

Next lesson: You'll learn **GEMINI.md context files**, so Gemini automatically understands your project architecture and coding standards.

