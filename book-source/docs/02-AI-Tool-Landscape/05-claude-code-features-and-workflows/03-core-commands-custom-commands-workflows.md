---
sidebar_position: 3
title: "Core Commands, Custom Commands & Workflows"
duration: "40 min"
---

# Core Commands, Custom Commands & Workflows

## From Setup to Daily Use: Mastering Claude Code Commands

In Lesson 2, you installed Claude Code and saw it work for the first time. Now comes the crucial step: **learning the commands you'll use every day** to work efficiently with your AI pair programmer.

This isn't just about memorizing syntax. It's about understanding *when* to use each command, *why* it matters, and *how* to build workflows that make development faster and more enjoyable.

By the end of this lesson, you'll know:
- The 10 core commands and when to use each
- How to create custom slash commands for team workflows
- How to manage Claude Code's context and costs
- Two essential workflows (Explore→Plan→Code and Test-Driven Development)
- How to control what Claude can access (permissions and security)

**Time**: 40 minutes (longer than previous lessons because these are your daily-use skills)

---

## Command Reference Table

These are the commands you'll use most often. Don't try to memorize them all at once—use this table as a reference, then practice in the hands-on section.

| Command | Purpose | When to Use | Example |
|---------|---------|-------------|---------|
| **`claude`** | Start a conversation | Begin a new task or ask a question | `claude "Help me debug this script"` |
| **`#`** | Create a checkpoint | Save progress in long conversations | `# Fixed auth bug, starting on payments` |
| **`@filename`** | Reference a specific file | Give Claude context about a file | `@app.py what does this function do?` |
| **`/init`** | Set up project memory | Tell Claude about your project (creates CLAUDE.md) | `/init` |
| **`/clear`** | Start fresh | End current conversation, start new task | `/clear` |
| **`/compact`** | Summarize conversation | When approaching token limit | `/compact` |
| **`ESC`** | Stop generation | Claude is generating too much, stop it | Press `ESC` once |
| **`ESC ESC`** | Emergency stop | Claude won't stop, force quit | Press `ESC` twice quickly |
| **`/mcp`** | Manage external tools | List or add MCP servers | `/mcp list` |
| **`/cost`** | Check API usage | See how much you're spending | `/cost` |
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

### 9. `/cost` - Check API Usage (Budget Awareness)

**Purpose**: See how much you're spending on API calls

**Syntax**:
```bash
/cost
```

**When to use**:
- End of work session (check spending)
- After long conversation (track usage)
- Weekly budget check

**Example output**:
```
API Usage Summary:
- Total tokens used: 45,230
- Estimated cost: $0.42
- This session: 3,450 tokens ($0.03)
```

**Why it matters**: Claude Code uses Claude AI API, which has costs. `/cost` helps you stay within budget.

**Best practice**: Check `/cost` at the end of each session

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

**Security note**: Even with permissions enabled, Claude asks for approval before making changes. You're always in control.

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

Example: `.claude/commands/code-review.md`

```markdown
# Code Review Command

Review the code in $ARGUMENTS for:
1. Potential bugs or edge cases
2. Code style and readability
3. Performance concerns
4. Security vulnerabilities

Provide:
- Summary of findings (critical/major/minor)
- Specific line numbers for issues
- Suggested fixes with code examples

Format: Use a clear severity rating for each issue.
```

**Step 3: Use your custom command**

```bash
claude /code-review auth.py
```

**What happens**:
- `$ARGUMENTS` gets replaced with `auth.py`
- Claude executes the full prompt template
- You get consistent, structured code reviews

---

### Example Custom Commands

**1. `/generate-tests` - Auto-generate test cases**

`.claude/commands/generate-tests.md`:
```markdown
Generate comprehensive unit tests for $ARGUMENTS.

Requirements:
- Cover happy path and edge cases
- Include test for error handling
- Use appropriate testing framework (pytest for Python, Jest for JavaScript)
- Add descriptive test names

Create the test file in the appropriate tests/ directory.
```

**Usage**:
```bash
claude /generate-tests utils/parser.py
```

---

**2. `/document-api` - Generate API documentation**

`.claude/commands/document-api.md`:
```markdown
Generate API documentation for the endpoints in $ARGUMENTS.

For each endpoint include:
- HTTP method and path
- Request parameters (query, body)
- Response format (status codes, body structure)
- Example request/response
- Error codes and meanings

Format: OpenAPI 3.0 specification
```

**Usage**:
```bash
claude /document-api routes/api.py
```

---

### Why Custom Commands Matter

**Without custom commands**:
```bash
claude "Review auth.py for bugs, style issues, performance problems, and security vulnerabilities. Provide severity ratings and specific line numbers."
```
(Type this every time)

**With custom commands**:
```bash
claude /code-review auth.py
```
(One short command)

**Benefits**:
- ⏱️ **Time saved**: 30 seconds per use
- 🎯 **Consistency**: Same quality checks every time
- 👥 **Team alignment**: Share commands via Git (everyone uses same prompts)
- 📚 **Best practices**: Encode expertise into commands

---

### Sharing Custom Commands with Your Team

**1. Commit to version control**:
```bash
git add .claude/commands/
git commit -m "Add custom code review command"
git push
```

**2. Team members pull the repo**:
```bash
git pull
```

**3. Everyone now has the custom command**:
```bash
claude /code-review their-file.js
```

**Result**: Entire team uses same high-quality prompts

---

## Decision Framework: When to Use Each Command

### Scenario 1: Starting a New Feature

**Goal**: Build authentication for your app

**Commands to use**:
1. `/clear` - Start fresh (if you were working on something else)
2. `claude "Help me build authentication with email/password login"` - Start conversation
3. `#` - Create checkpoints as you progress
4. `/cost` - Check usage when done

---

### Scenario 2: Debugging an Error

**Goal**: Fix a bug in production

**Commands to use**:
1. `@error-log.txt` - Show Claude the error
2. `claude "This error appeared in production. What's the cause?"` - Ask for analysis
3. `@problematic-file.py` - Reference the file with the bug
4. `/cost` - Track spending after resolution

---

### Scenario 3: Long Refactoring Session

**Goal**: Refactor old codebase over several hours

**Commands to use**:
1. `claude` - Start interactive session
2. `#` - Checkpoint after each file refactored
3. `/compact` - When conversation gets long (around 20+ exchanges)
4. `ESC` - If Claude generates too much detail
5. `/cost` - Check budget periodically

---

### Scenario 4: Team Code Review

**Goal**: Review pull request code

**Commands to use**:
1. Create custom command `/code-review` (one-time setup)
2. Use it: `claude /code-review changes.py`
3. Get consistent, structured reviews
4. `/permissions` - Ensure Claude can't accidentally modify PR files

---

## Core Workflows

Now that you know the commands, let's see how they combine into workflows.

---

### Workflow 1: Explore → Plan → Code → Commit

**When to use**: Complex problems where you're uncertain about the approach

**Steps**:

**1. Explore** (Understand the problem)
```bash
claude "Read the codebase and help me understand how authentication currently works"
```

Claude reads relevant files and explains the system.

**2. Plan** (Design the solution)
```bash
# Finished exploring
# Now planning the solution

claude "I want to add OAuth login. What's the best approach given our current setup?"
```

Claude proposes an architecture.

**3. Code** (Implement with guidance)
```bash
# Approved the plan
# Starting implementation

claude "Let's implement the OAuth flow. Start with the redirect endpoint."
```

Claude helps you write the code.

**4. Commit** (Save your work)
```bash
# Implementation complete
# Ready to commit

git add .
git commit -m "Add OAuth authentication"
```

**Why this workflow works**:
- Explore first → understand context before coding
- Plan → ensure approach is sound
- Code → implement with confidence
- Commit → save working state

**Time estimate**: 30-90 minutes depending on complexity

---

### Workflow 2: Test-Driven Development (TDD)

**When to use**: Features that need high confidence (e.g., financial calculations, authentication, data processing)

**Steps**:

**1. Write the test first** (Define success criteria)
```bash
claude "Write a test for a function that calculates order total with tax and discounts. The function doesn't exist yet."
```

Claude creates `test_orders.py` with failing test.

**2. See the test fail** (Confirm it fails for the right reason)
```bash
pytest tests/test_orders.py
```

Output: `FAIL - Function 'calculate_total' not found`

**3. Implement the function** (Make it pass)
```bash
# Test is failing as expected
# Now implementing the function

claude "Implement the calculate_total function to make this test pass"
```

Claude writes the function.

**4. See the test pass** (Verify correctness)
```bash
pytest tests/test_orders.py
```

Output: `PASS - 1 test passed`

**5. Refactor safely** (Improve code while tests protect you)
```bash
claude "Refactor calculate_total to be more readable. Tests must still pass."
```

**Why this workflow works**:
- Test first → clear success criteria
- Failing test → confirms test works
- Implementation → focused on making test pass
- Passing test → confidence in correctness
- Refactor → improve code without breaking it

**Time estimate**: 10-20 minutes per function

---

## Context Management Patterns

### When to Use `/clear`

**Use `/clear` when**:
- ✅ Switching to completely different task
- ✅ Current conversation is off-track
- ✅ Claude seems confused or contradicting itself
- ✅ You want to start from zero (fresh context)

**Don't use `/clear` when**:
- ❌ Just changing topic within same project
- ❌ Conversation is long but still useful
- ❌ You'll need the history later

---

### When to Use `/compact`

**Use `/compact` when**:
- ✅ Conversation is getting very long (20+ exchanges)
- ✅ Token limit warning appears
- ✅ Want to keep progress but reduce size
- ✅ Conversation has lots of back-and-forth

**What `/compact` does**:
- Summarizes decisions made
- Keeps key code changes
- Removes redundant exchanges
- Reduces token usage by 50-70%

---

### When to Use Checkpoints (`#`)

**Use checkpoints when**:
- ✅ Multi-step task (3+ steps)
- ✅ After completing each subtask
- ✅ Before starting new phase
- ✅ Want clear progress markers

**Example flow**:
```bash
claude "Help me build a REST API"

# Created project structure
# Implemented user endpoints
# Added authentication
# Writing tests
# All tests passing - API complete
```

**Benefit**: Clear progress tracking, easier to resume if interrupted

---

## Practice Approach

### Part A: Learn Core Commands (10 minutes)

**Goal**: Try each command and see what it does

**Tasks**:
1. Check Claude Code version: `claude --version`
2. Start a conversation: `claude "Hello!"`
3. Check your permissions: `/permissions`
4. Check cost: `/cost`
5. Create a checkpoint: `# Tested basic commands`
6. Clear context: `/clear`

**Success criteria**: You've used 6 different commands successfully

---

### Part B: Create a Custom Command (10 minutes)

**Goal**: Build your first custom command

**Tasks**:
1. Create directory: `mkdir -p .claude/commands`
2. Create file: `.claude/commands/explain.md`
3. Add this content:
```markdown
Explain the code in $ARGUMENTS in simple terms.

Include:
- What it does (one sentence)
- How it works (step-by-step)
- When you'd use it
- Example usage

Format: Clear sections with headers.
```
4. Test it: `claude /explain your-file.py`

**Success criteria**: Your custom command works and generates a clear explanation

---

### Part C: Execute a Workflow (20 minutes)

**Goal**: Use commands in a real workflow

**Choose one**:

**Option A: Explore → Plan → Code**
- Explore: Ask Claude to explain a file in your project
- Plan: Ask Claude to suggest an improvement
- Code: Implement the improvement with Claude's help
- Verify: Check that it works

**Option B: Test-Driven Development**
- Write test: Ask Claude to write a test for a function that doesn't exist
- See fail: Run the test (it should fail)
- Implement: Write the function to make test pass
- See pass: Run test again (it should pass)

**Success criteria**: You've completed a full workflow using multiple commands

---

## Verification Checklist

**Your command mastery is working when**:

- ✓ You can start a conversation with Claude (`claude` or `claude "prompt"`)
- ✓ You know when to use `/clear` vs. `/compact`
- ✓ You understand checkpoints (`#`) and use them in multi-step tasks
- ✓ You can reference files with `@filename`
- ✓ You've created at least one custom command
- ✓ You check `/cost` at the end of sessions
- ✓ You understand `/permissions` and security boundaries
- ✓ You've completed a full workflow (Explore→Plan→Code OR TDD)
- ✓ You can stop Claude's generation with ESC when needed
- ✓ You know how to check MCP connections with `/mcp list`

**If all checked**: You're ready for Lesson 4 (Subagents)!

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

### Prompt 4: Cost Optimization

```
I just ran /cost and my usage is higher than expected. Review my recent workflow: [describe what you did]. Which commands or approaches would reduce token usage while keeping the same quality? Give me 3-5 specific optimization tips.
```

**Expected outcome:** Practical tips to reduce API costs
