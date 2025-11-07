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

## Your First Hook: Echo Messages on Actions

Let's create a simple hook that **echoes messages** when different actions happen. This teaches the mechanics without complexity.

### Step 1: Create Project Settings File

In your project root, create `.claude/settings.json`:

```bash
mkdir -p .claude
```

**File**: `.claude/settings.json`

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
  }
}
```

### Step 2: Understand the Three Hooks

**Hook 1: SessionStart**
```json
"SessionStart": [
  {
    "type": "command",
    "command": "echo '🚀 Claude Code session started for this project'"
  }
]
```

**When it runs**: Every time you type `claude` to start a session

**Output**:
```
🚀 Claude Code session started for this project
```

**Hook 2: PreToolUse**
```json
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
```

**When it runs**: BEFORE Claude executes any bash command

**Output**:
```
⚡ Running bash command...
```

**Hook 3: PostToolUse**
```json
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
```

**When it runs**: AFTER Claude finishes editing a file

**Output**:
```
✅ File edited successfully
```

---

## Test Your Hooks

### Test 1: SessionStart Hook

```bash
cd your-project
claude
```

**What you'll see**:
```
🚀 Claude Code session started for this project

(Claude Code starts...)
```

The hook ran when the session started!

### Test 2: PreToolUse Hook

In Claude Code, ask Claude to run a bash command:

```
List all Python files in this project
```

**What you'll see**:
```
⚡ Running bash command...
find . -name "*.py" -type f
```

The hook echoed BEFORE the bash command executed.

### Test 3: PostToolUse Hook

In Claude Code, ask Claude to edit a file:

```
Add a comment to the top of app.py saying "Main application file"
```

**What you'll see**:
```
✅ File edited successfully
```

The hook echoed AFTER the file was edited.

---

## How Hooks Work: The Architecture

```
You ask Claude Code to do something
    ↓
╔═════════════════════════════════╗
║  Hook System Checks             ║
║  "Is there a hook for this?"    ║
╚════════════┬════════════════════╝
             │
    ┌────────┴────────┐
    │                 │
    ▼                 ▼
[PreToolUse]    [PostToolUse]
(Before)        (After)
    │                 │
    └────────┬────────┘
             ▼
    Claude Code executes action
```

---

## The Three Hook Types Explained

### Hook Type 1: SessionStart

**Runs**: When Claude Code starts (`claude` command)

**Use for**:
- Greeting messages
- Loading environment variables
- Project initialization

**Example in your settings.json**:
```json
{
  "hooks": {
    "SessionStart": [
      {
        "type": "command",
        "command": "echo '🚀 Project loaded, ready to code!'"
      }
    ]
  }
}
```

### Hook Type 2: PreToolUse

**Runs**: BEFORE Claude executes a tool (Bash, Edit, Read, etc.)

**Use for**:
- Validation before dangerous commands
- Warning messages
- Preparation steps

**Example - Warning before bash**:
```json
{
  "hooks": {
    "PreToolUse": [
      {
        "matcher": "Bash",
        "hooks": [
          {
            "type": "command",
            "command": "echo '⚠️  About to run a command...'"
          }
        ]
      }
    ]
  }
}
```

### Hook Type 3: PostToolUse

**Runs**: AFTER Claude executes a tool

**Use for**:
- Validation of results
- Cleanup tasks
- Confirmation messages

**Example - Message after edit**:
```json
{
  "hooks": {
    "PostToolUse": [
      {
        "matcher": "Edit",
        "hooks": [
          {
            "type": "command",
            "command": "echo '✅ Modification complete'"
          }
        ]
      }
    ]
  }
}
```

---

## How Matchers Work

Hooks can target specific tools with `matcher`:

**Available matchers**:
- `"Bash"` — Bash/shell commands
- `"Edit"` — File edits
- `"Read"` — File reads
- `"Write"` — File writes

**Example - Different messages for different tools**:

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
        "matcher": "Edit",
        "hooks": [
          {
            "type": "command",
            "command": "echo '✏️  Editing file...'"
          }
        ]
      }
    ]
  }
}
```

Now:
- Before bash commands: `🔧 Running command...`
- Before edits: `✏️  Editing file...`

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
  "SessionStart": [
    {
      "type": "command",
      "command": "echo '👋 Welcome to [Project Name]!'"
    }
  ]
}
```

### Pattern 2: Action Confirmations

```json
{
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
```

### Pattern 3: Pre-Action Warnings

```json
{
  "PreToolUse": [
    {
      "matcher": "Bash",
      "hooks": [
        {
          "type": "command",
          "command": "echo '⚠️  Running command (type 'n' to cancel)...'"
        }
      ]
    }
  ]
}
```

---

## Try With AI

Practice hooks with Claude Code.

### Exercise 1: Create Your First Hook

**Steps**:
1. Create `.claude/settings.json` in your project (use the example above)
2. Save the file
3. Run `claude` to start a session
4. You should see: `🚀 Claude Code session started for this project`
5. Ask Claude: "List files in this project" (uses bash)
6. You should see: `⚡ Running bash command...`

**Expected outcome**: All three hooks fire in the right sequence.

### Exercise 2: Customize Your Hooks

**Prompt**:
```
I've created hooks that echo messages. Now help me customize them:

1. Change the SessionStart message to include the project name
2. Change the Bash message to say something different
3. Change the Edit message to add a checkmark

Show me the updated settings.json
```

**Expected outcome**: Your personalized hooks with custom messages.

### Exercise 3: Explore Other Matchers

**Prompt**:
```
I want to add hooks for different tools. Show me how to add hooks that echo:
- "📖 Reading file..." when Claude reads files
- "📝 Creating file..." when Claude creates files
- "🗑️  Deleting file..." when Claude deletes files

Update my settings.json with these three new hooks.
```

**Expected outcome**: Hooks for Read, Write, and Delete operations.

---

## What's Next

You now understand **the automation foundation**:
- ✅ SessionStart — When Claude Code starts
- ✅ PreToolUse — Before actions execute
- ✅ PostToolUse — After actions complete

**In Lesson 8**, you'll see how **plugins package hooks together** with commands, agents, and skills—turning individual echoes into complete automated workflows.

For now, hooks are your way to **see automation in action** before building complex workflows.

