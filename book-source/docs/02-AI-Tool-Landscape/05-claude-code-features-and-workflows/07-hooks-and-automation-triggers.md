---
sidebar_position: 7
title: "Hooks: Automating Before & After Actions"
duration: "25 min"
---

# Hooks: Automating Before & After Actions

## The Problem: Repetitive Manual Checks

Claude Code finishes editing your code. You manually check:

```
Claude edits: app.py
↓ (manual) Run linter
↓ (manual) Check types
↓ (manual) Run tests
```

**Repetitive. Every time.**

**What if Claude Code automatically ran these checks?**

That's what **hooks** do. Hooks intercept actions and run automation **before** or **after** them.

---

## What Are Hooks?

**Definition**: A hook is an automation trigger that runs before or after Claude Code executes a tool.

**Simple version:**
- **Before action**: Validate before executing (e.g., "Don't run dangerous commands")
- **After action**: Check results after executing (e.g., "Run lint after editing")

**Example flow:**

```
Claude: Edit app.py
  ↓
PostToolUse Hook Triggers
  ↓
Runs: npm run lint
  ↓
Output: ✓ Lint passed
```

---

## Step 1: Write Your First Hook (Foundation)

You'll write JSON directly so you understand exactly how hooks work.

### Create `.claude/settings.json`

In your project root, create a file named `.claude/settings.json`:

```json
{
  "hooks": {
    "SessionStart": [
      {
        "type": "command",
        "command": "echo '🚀 Claude Code session started for this project'"
      }
    ],
    "PreToolUse": [
      {
        "matcher": "Bash",
        "hooks": [
          {
            "type": "command",
            "command": "echo '⚡ Running bash command...'"
          }
        ]
      }
    ],
    "PostToolUse": [
      {
        "matcher": "Edit",
        "hooks": [
          {
            "type": "command",
            "command": "echo '✅ File edited successfully'"
          }
        ]
      }
    ]
  },
  "permissions": {
    "allow": [
      "Bash(*)",
      "Read",
      "Write",
      "Edit"
    ],
    "deny": [],
    "ask": []
  }
}
```

**Understanding the structure**:

**Hooks section**:
- **`SessionStart`**: Runs when Claude Code starts
- **`PreToolUse`**: Runs BEFORE a tool executes (with `matcher: "Bash"` = only bash)
- **`PostToolUse`**: Runs AFTER a tool completes (with `matcher: "Edit"` = only edits)
- **`type: "command"`**: Execute a shell command
- **`command`**: The actual shell command to run (`echo` in this case)

**Permissions section** (NEW):
- **`allow`**: Tools that Claude Code is permitted to use
  - `"Bash(*)"` → Allow any bash command
  - `"Write"` → Allow creating files
  - `"Edit"` → Allow editing files
  - `"Read"` → Allow reading files
- **`deny`**: Tools to explicitly block (empty = no blocks)
- **`ask`**: Tools that require user confirmation each time (empty = none)

**Why permissions?** Hooks run shell commands, which are powerful. Permissions tell Claude Code which tools are safe in this project and which need user approval.

### Test Your Hook

1. Save the file
2. Exit Claude Code: `exit`
3. Restart: `claude`
4. You should see: `🚀 Claude Code session started for this project`
5. Ask Claude: "List Python files in this project"
6. You should see: `⚡ Running bash command...` before the command runs
7. Ask Claude: "Add a comment to test.md"
8. You should see: `✅ File edited successfully` after the edit

**Why write it manually?** You now understand exactly what each hook does, how the JSON is structured, AND why permissions matter.

---

## Step 2: The Interactive Way (Easier)

Now that you understand hooks, Claude Code has a faster way to add them:

### Use the `/hooks` Command

Instead of writing JSON, type:

```
/hooks
```

You'll see an interactive menu:
- PreToolUse — Before tool execution
- PostToolUse — After tool execution
- Notification — When notifications are sent
- UserPromptSubmit — When the user submits a prompt
- SessionStart — When a new session is started

Select an event, enter your command, done. No JSON to write.

**When to use**: When you want to quickly add a simple hook without editing config files.

---

## Step 3: The AI-Native Way (Automation)

For more complex hooks or when you want AI to help, use Claude Code's AI-native approach:

### Tell Claude to Configure Your Hooks

Instead of writing JSON or using `/hooks`, just ask Claude:

**Example prompt**:
```
I want to add a hook that runs npm run lint after every file edit.
Use the official Claude Code hooks documentation to create the right
configuration. Here's the link: https://docs.claude.com/en/docs/claude-code/hooks
```

Claude will:
1. Fetch the official hooks documentation
2. Generate the correct JSON
3. Create or update `.claude/settings.json`
4. Test the hook with you

**Why this works**: Claude reads the official docs in real-time, ensuring your hooks match the latest Claude Code API.

---

:::note
Windows users: run hooks in WSL or Git Bash for best compatibility. If using PowerShell, replace `jq` with `ConvertFrom-Json`, skip `chmod`, and invoke scripts with `python path/to/script.py` instead of relying on POSIX executability.
:::

## Understanding Permissions (Critical for Hooks)

Hooks execute shell commands, which are powerful. The `permissions` section controls what Claude Code is allowed to do in your project.

### Permission Types

**`allow`** — Tools Claude Code can use automatically:
```json
"allow": [
  "Bash(*)",          // Allow any bash command
  "Write",            // Allow creating files
  "Edit",             // Allow editing files
  "Read"              // Allow reading files
]
```

**`deny`** — Tools to explicitly block:
```json
"deny": [
  "Bash(rm:*)"        // Block delete commands
]
```

**`ask`** — Tools that require user confirmation:
```json
"ask": [
  "Bash(git push:*)"  // Always ask before pushing
]
```

### Why This Matters for Hooks

Hooks are shell commands that run **automatically** (without user input). Permissions ensure:

1. **Security**: Only safe commands can run automatically
2. **Control**: You decide which tools need approval
3. **Predictability**: No surprises from automated actions

**Example**: A hook that runs `npm run lint` needs:
```json
"allow": ["Bash(npm:*)"]
```

Without this permission, the command will be denied by the permissions system and won't execute.

---

## Understanding Hook Events

Each hook fires at a specific lifecycle moment:

| Hook Event | Fires When | Use For |
|-----------|-----------|---------|
| **SessionStart** | Claude Code starts | Greeting, environment setup |
| **PreToolUse** | Before a tool executes | Validation, warnings |
| **PostToolUse** | After a tool completes | Confirmation, running tests |
| **UserPromptSubmit** | User submits a prompt | Logging, preprocessing |
| **Notification** | Claude sends a notification | Logging events |
| **Stop** | Claude finishes responding | Cleanup |
| **SubagentStop** | Subagent task completes | Collecting results |
| **PreCompact** | Before history compaction | Snapshots |
| **SessionEnd** | Session ends | Cleanup, saving state |

---

## How Matchers Work

Hooks can target specific tools with `matcher`. You have three options:

### Option 1: Target a Single Tool

**Available tool matchers**:
- `"Bash"` — Bash/shell commands
- `"Edit"` — File edits
- `"Read"` — File reads
- `"Write"` — File writes
- `"Glob"` — File pattern searches
- `"Grep"` — File content searches

**Example - Single tool**:

```json
{
  "hooks": {
    "PreToolUse": [
      {
        "matcher": "Bash",
        "hooks": [
          {
            "type": "command",
            "command": "echo '⚡ Running bash command...'"
          }
        ]
      }
    ]
  }
}
```

### Option 2: Target Multiple Tools (Pipe Notation)

Use the pipe character `|` to match multiple tools:

```json
{
  "hooks": {
    "PostToolUse": [
      {
        "matcher": "Edit|Write",
        "hooks": [
          {
            "type": "command",
            "command": "echo '✅ File operation complete'"
          }
        ]
      }
    ]
  }
}
```

Now the hook fires after EITHER Edit OR Write operations.

### Option 3: Target All Tools (Wildcard)

Use `"*"` to match all tools:

```json
{
  "hooks": {
    "PreToolUse": [
      {
        "matcher": "*",
        "hooks": [
          {
            "type": "command",
            "command": "echo '🔄 About to execute any tool...'"
          }
        ]
      }
    ]
  }
}
```

Now the hook fires before ANY tool is executed.

### Example - Different Messages for Different Tool Groups

```json
{
  "hooks": {
    "PreToolUse": [
      {
        "matcher": "Bash",
        "hooks": [
          {
            "type": "command",
            "command": "echo '🔧 Running command...'"
          }
        ]
      },
      {
        "matcher": "Edit|Write",
        "hooks": [
          {
            "type": "command",
            "command": "echo '✏️  Modifying file...'"
          }
        ]
      },
      {
        "matcher": "Read|Glob|Grep",
        "hooks": [
          {
            "type": "command",
            "command": "echo '📖 Searching for files...'"
          }
        ]
      }
    ]
  }
}
```

Now:
- Before bash: `🔧 Running command...`
- Before Edit/Write: `✏️  Modifying file...`
- Before Read/Glob/Grep: `📖 Searching for files...`

---

## Scope: Project vs. Global

### Project Hooks (What We Created)

**Location**: `.claude/settings.json` in your project

**Applies to**: Only this project

**When to use**: Project-specific automation

### Global Hooks (Optional)

**Location**: `~/.claude/settings.json` in your home folder

**Applies to**: ALL projects on your machine

**When to use**: Rules you want everywhere (e.g., block dangerous commands)

**Pro tip**: Start with project hooks. Add global hooks only when you want the rule in all projects.

---

## Common Hook Patterns

### Pattern 1: Friendly Greeting

```json
{
  "hooks": {
    "SessionStart": [
      {
        "type": "command",
        "command": "echo '👋 Welcome to [Project Name]!'"
      }
    ]
  }
}
```

### Pattern 2: Action Confirmations

```json
{
  "hooks": {
    "PostToolUse": [
      {
        "matcher": "Edit",
        "hooks": [
          {
            "type": "command",
            "command": "echo '💾 File saved'"
          }
        ]
      },
      {
        "matcher": "Bash",
        "hooks": [
          {
            "type": "command",
            "command": "echo '✨ Command executed'"
          }
        ]
      }
    ]
  }
}
```

### Pattern 3: Pre-Action Warnings

```json
{
  "hooks": {
    "PreToolUse": [
      {
        "matcher": "Bash",
        "hooks": [
          {
            "type": "command",
            "command": "echo '⚠️  Command about to run...'"
          }
        ]
      }
    ]
  }
}
```

---

## Try With AI

Practice all three approaches: manual, interactive, and AI-native.

### Exercise 1: Write Your First Hook (Foundation)

**Steps**:
1. Create `.claude/settings.json` in your project (copy the example from "Step 1" above, including BOTH hooks AND permissions sections)
2. Save the file
3. Exit Claude Code: `exit`
4. Restart: `claude`
5. You should see: `🚀 Claude Code session started for this project`
6. Ask Claude: "List files in this project"
7. You should see: `⚡ Running bash command...` appear before the command runs
8. Ask Claude: "Create a test.md file"
9. The file should be created, and you should see: `✅ File edited successfully`

**Expected outcome**: You understand:
- How hooks trigger at different lifecycle moments
- How permissions allow/block tools
- Why both sections are needed for hooks to work

### Exercise 2: Use the Interactive Way

**Steps**:
1. Type: `/hooks`
2. Select **PreToolUse**
3. When prompted, enter: `echo '🔧 Preparing to execute...'`
4. Type `/hooks` again
5. Select **PostToolUse**
6. When prompted, enter: `echo '✨ All done!'`
7. Restart Claude Code and test by asking: "List files"

**Expected outcome**: Hooks configured without manually editing JSON.

### Exercise 3: AI-Native Configuration

**Prompt to Claude**:
```
I need to add a hook that runs 'npm run lint' after every file edit to catch
style errors automatically. Use the official Claude Code hooks documentation
at https://docs.claude.com/en/docs/claude-code/hooks to create the right
configuration for my .claude/settings.json
```

**Claude will**:
1. Read the official hooks documentation
2. Generate the correct JSON configuration
3. Create/update your `.claude/settings.json`
4. Test it with you

**Expected outcome**: A production-ready hook configured by AI using official docs as source of truth.

---

## What's Next

You now understand **the automation foundation**:
- ✅ SessionStart — When Claude Code starts
- ✅ PreToolUse — Before actions execute
- ✅ PostToolUse — After actions complete

**In Lesson 8**, you'll see how **plugins package hooks together** with commands, agents, and skills—turning individual echoes into complete automated workflows.

For now, hooks are your way to **see automation in action** before building complex workflows.

