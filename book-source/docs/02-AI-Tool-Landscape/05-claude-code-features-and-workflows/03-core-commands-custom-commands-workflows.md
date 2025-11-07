---
sidebar_position: 3
title: "Core Commands, Custom Commands & Workflows"
duration: "40 min"
---

# Core Commands, Custom Commands & Workflows

## From Setup to Daily Use: Mastering Claude Code Commands

In Lesson 2, you installed Claude Code and saw it work for the first time. Now comes the crucial step: **learning the commands you'll use every day** to work efficiently with your AI pair programmer.

This isn't just about memorizing syntax. It's about understanding *when* to use each command, *why* it matters, and *how* to build workflows that make development faster and more enjoyable.

---

## Command Reference Table

These are the commands you'll use most often. Don't try to memorize them all at once—use this table as a reference, then practice in the hands-on section.

| Command | Purpose | When to Use | Example |
|---------|---------|-------------|---------|
| **`claude`** | Start a session | Begin a new task or ask a question | `claude` |
| **`#`** | Save a Memory | Save key decisions in CLAUDE.md | `# We will use python with type hinting` |
| **`@filename`** | Reference a specific file | Give Claude context about a file | `@app.py what does this function do?` |
| **`/init`** | Set up project memory | Tell Claude about your project (creates CLAUDE.md) | `/init` |
| **`/clear`** | Start fresh | End current conversation, start new task | `/clear` |
| **`/compact`** | Summarize conversation | When approaching token limit | `/compact` |
| **`ESC`** | Stop generation | Claude is generating too much, stop it | Press `ESC` once |
| **`ESC ESC`** | Emergency stop | Claude won't stop, force quit | Press `ESC` twice quickly |
| **`/mcp`** | Manage external tools | List or add MCP servers | `/mcp list` |
| **`/usage`** | Check usage | Show plan usage daily/weekly limits | `/usage` |
| **`/permissions`** | Control access | Set what Claude can do | `/permissions` |

---

## Core Commands (Explained)

### 1. `claude` - Start a Conversation

**Purpose**: The main command to interact with Claude Code

**Syntax**:
```bash
claude                           # Start interactive conversation
claude "your prompt here"        # One-off request
```

**When to use**:
- Starting a new task
- Asking a question
- Getting help with an error

**Example**:
```bash
claude "Review the authentication logic in auth.py and suggest improvements"
```

**What happens**: Claude reads the file, analyzes the logic, and provides specific suggestions

---

### 2. `#` - Create Checkpoints

**Purpose**: Mark progress points in long conversations

**Syntax**:
```bash
# Your checkpoint message here
```

**When to use**:
- After completing a subtask
- Before starting a new feature
- To organize multi-step work

**Example**:
```bash
# Fixed the login bug
# Now working on password reset
```

**Why it matters**: Checkpoints help Claude (and you) track what's done vs. what's next. In long conversations, they prevent Claude from losing context.

---

### 3. `@filename` - Reference Files

**Purpose**: Tell Claude to look at a specific file

**Syntax**:
```bash
@filename.ext your question
```

**When to use**:
- Asking about a specific file
- Comparing multiple files
- Getting file-specific help

**Example**:
```bash
@models.py @views.py how do these two files interact?
```

**What happens**: Claude reads both files and explains their relationship

---

### 4. `/init` - Set Up Project Memory

**Purpose**: Initialize Claude Code for your project (creates CLAUDE.md)

**Syntax**:
```bash
/init
```

**When to use**:
- First time using Claude Code in a project
- When you want Claude to remember project context

**What it does**:
1. Asks about your project (name, language, purpose)
2. Creates CLAUDE.md with your answers
3. Claude reads this file in future sessions

**Time saved**: 2-5 minutes per session (no more repeating project context)

---

### 5. `/clear` - Start Fresh

**Purpose**: End current conversation, clear context

**Syntax**:
```bash
/clear
```

**When to use**:
- Switching to a completely different task
- Current conversation is getting confusing
- Want to start over without previous context

**Example scenario**:
```bash
# Working on feature A
claude "Help me with authentication"
# ... conversation ...

/clear

# Now working on feature B
claude "Help me with database migration"
```

**What gets cleared**: Conversation history, file references
**What stays**: Your project (files), CLAUDE.md

---

### 6. `/compact` - Summarize Conversation

**Purpose**: Condense long conversation to save tokens

**Syntax**:
```bash
/compact
```

**When to use**:
- Long conversation approaching token limit
- Want to keep context but reduce size
- Conversation has lots of back-and-forth

**What it does**:
- Claude summarizes the conversation
- Keeps key decisions and progress
- Removes redundant exchanges

**vs. `/clear`**: `/compact` keeps context (summarized), `/clear` removes all context

---

### 7. `ESC` and `ESC ESC` - Stop Generation

**Purpose**: Interrupt Claude when it's generating

**When to use**:
- Claude is generating too much detail
- Going in wrong direction
- Just want it to stop

**ESC (once)**: Polite stop - Claude finishes current thought and stops
**ESC ESC (twice)**: Force quit - immediate stop

**Example**:
```bash
claude "Explain dependency injection in detail"
# Claude starts generating paragraphs...
# Press ESC - Claude stops gracefully
```

---

### 8. `/mcp` - Manage External Tools

**Purpose**: List, add, or configure MCP servers (external tools)

**Syntax**:
```bash
/mcp list                                    # Show connected tools
/mcp add <name> <command>                   # Add a tool
```

**When to use**:
- You want Claude to access external data (web, databases, APIs)
- Need to check what tools are connected
- Setting up MCP servers (covered in Lesson 6)

**Example**:
```bash
/mcp list
```

**Output**: Shows all connected MCP servers (e.g., web search, docs, GitHub)

---

### 9. `/usage` - Check API Usage (Budget Awareness)

**Purpose**: Show plan usage limits

**Example Output**:
Settings:  Status   Config   Usage   (tab to cycle)

 Current session
 ███                                                6% used
 Resets 2pm (Asia/Karachi)

 Current week (all models)
 ███████████                                        22% used
 Resets Nov 12, 10pm (Asia/Karachi)

 Current week (Opus)
                                                    0% used

 Esc to exit
```

**Best practice**: Check `/usage` often to prevent unexpected session limit reached problems.

---

### 10. `/permissions` - Control Access (Security)

**Purpose**: Set what Claude Code can access and do

**Syntax**:
```bash
/permissions                      # Show current permissions
/permissions set <permission>    # Change a permission
```

**When to use**:
- First time setting up (define boundaries)
- Working with sensitive files
- Want to restrict Claude's actions

**Example**:
```bash
/permissions
```

**Output**:
```
Current permissions:
- Read files: ✓ Allowed
- Write files: ✓ Allowed (approval required)
- Execute commands: ✓ Allowed (approval required)
- Access network: ✗ Denied
```

---

## Custom Slash Commands (Team Workflow Automation)

### What Are Custom Slash Commands?

Custom slash commands are **reusable prompt templates** you create for tasks you do repeatedly.

**Think of them as**: Shortcuts for common workflows

**Stored in**: `.claude/commands/` directory in your project

---

### How to Create a Custom Command

**Step 1: Create the commands directory**

```bash
mkdir -p .claude/commands
```

**Step 2: Create a markdown file for your command**

Example: `.claude/commands/markdown-review.md`

```markdown
# Markdown Review Command

Review the Markdown in $ARGUMENTS for:
1. Potential bugs or edge cases
2. Style and readability
3. Security vulnerabilities

Provide:
- Summary of findings (critical/major/minor)
- Specific line numbers for issues
- Suggested fixes with examples

Format: Use a clear severity rating for each issue.
```

**Step 3: Use your custom command**

```bash
claude /markdown-review >> We are learning how to Create a custom command
```

**What happens**:
- `$ARGUMENTS` gets replaced with `>> We are learning how to Create a custom command`
- Claude executes the full prompt template
- You get consistent, structured reviews

---

### Why Custom Commands Matter

**Without custom commands**:
```bash
claude "Review auth.md for bugs, style issues, performance problems, and security vulnerabilities."

**With custom commands**:
```bash
claude /markdown-review `>> We are learning how to Create a custom command`
```
(One short command)

**Benefits**:
- ⏱️ **Time saved**: 30 seconds per use
- 🎯 **Consistency**: Same quality checks every time
- 👥 **Team alignment**: Share commands via Git (everyone uses same prompts)
- 📚 **Best practices**: Encode expertise into commands

---

## Try With AI

Use **Claude Code CLI** for this activity (since you're learning Claude Code commands).

### Prompt 1: Command Selection

```
I'm working on [describe your task]. Which Claude Code commands should I use, and in what order? Give me a step-by-step workflow with specific commands. Include when to use checkpoints, when to check /cost, and when to /clear or /compact.
```

**Expected outcome:** Step-by-step command workflow tailored to your task

---

### Prompt 2: Custom Command Creation

```
I do this task repeatedly: [describe repetitive task]. Help me create a custom slash command for it. Write the markdown file content I should save in .claude/commands/[name].md. Make it use $ARGUMENTS so I can pass different files/inputs.
```

**Expected outcome:** Ready-to-use custom command file content

---

### Prompt 3: Workflow Debugging

```
I tried the [Explore→Plan→Code / TDD] workflow but got stuck at [step]. What went wrong? Walk me through that step again with specific commands and what I should see at each point.
```

**Expected outcome:** Troubleshooting help with exact commands to try

---

### Prompt 4: Usage Optimization

```
I just ran /usage and my usage is higher than expected. Review my recent workflow: [describe what you did]. Which commands or approaches would reduce token usage while keeping the same quality? Give me 3-5 specific optimization tips.
```

**Expected outcome:** Practical tips to optimize usage
