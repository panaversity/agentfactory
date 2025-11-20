# Initial Setup: Workspace Creation & Configuration

## Overview

After installation, Antigravity presents a clean interface asking you to create your first **workspace**. A workspace is Antigravity's organizational container for agents, projects, and artifacts. Think of it as a "development environment" where all your agents, code, and AI interactions live.

This section covers:
- Creating your first workspace
- Understanding workspace structure
- Configuring basic preferences
- Connecting to your file system

**Time estimate**: 15-20 minutes

---

## Understanding Workspaces

### What is a Workspace?

A workspace in Antigravity is:
- **Container**: Holds agents, projects, and work in one place
- **Isolated**: Separate from other workspaces (you can have multiple)
- **Persistent**: Your work saves automatically
- **Multi-agent**: One workspace can manage multiple agents working in parallel

### Real-world analogy
If a traditional IDE (Zed/Cursor) is like "one desk where you work on code," a workspace is like "one desk where multiple AI assistants can work on different parts of the same project simultaneously."

---

## Creating Your First Workspace

### Step 1: Launch Antigravity

Open Antigravity from your system. You should see the welcome screen with:
- Logo
- "Create Workspace" button
- "Open Existing Workspace" button (grayed out, since you don't have one yet)

### Step 2: Click "Create Workspace"

You see a dialog with three fields:

| Field | Description | Example |
|-------|-------------|---------|
| **Workspace Name** | Human-readable name | "My First Project", "Learning IDE" |
| **Workspace Location** | Where files store on disk | `/home/user/antigravity/my-project` |
| **Local AI (Optional)** | Use local Ollama or cloud API? | Cloud API (you'll configure in next section) |

### Step 3: Enter Workspace Details

1. **Workspace Name**: Type a meaningful name
   - Examples: "Temperature Converter", "Learning Workspace", "IDE Playground"
   - Good names describe what you'll build or learn here
   - Avoid generic names like "Test" or "Workspace1"

2. **Workspace Location**:
   - Default location is fine (typically `~/.antigravity/workspaces/your-workspace-name`)
   - Or customize to prefer a specific folder
   - Antigravity creates the folder automatically

3. **Local AI (optional)**:
   - Skip for now (leave unchecked)
   - You'll use cloud APIs (Anthropic, OpenAI, Google) instead
   - Local models covered in later lesson

### Step 4: Click "Create"

Antigravity initializes the workspace:
- Creates folder structure
- Sets up configuration files
- Initializes git repository (optional but recommended)
- Redirects to Agent Manager (main interface)

**Expected**: You see the Agent Manager with empty agent list and "Create Agent" button

---

## Agent Manager Interface

After workspace creation, you're in the **Agent Manager**—Antigravity's command center for managing agents.

### Key Areas

```
┌─────────────────────────────────────────────────────────────┐
│  Antigravity Agent Manager                         [⚙️ ⚡] │
├─────────────────────────────────────────────────────────────┤
│ Workspace: "My First Project"                                │
├──────────────┬──────────────────────────────────────────────┤
│ Agents       │ No agents yet                                 │
│ ├─ [empty]   │                                              │
│ ├─ (+) New   │ Click "Create Agent" to start               │
│ └─ ...       │                                              │
├──────────────┼──────────────────────────────────────────────┤
│ Settings ⚙️  │ AI Providers configuration                   │
└──────────────┴──────────────────────────────────────────────┘
```

### Main Controls

- **Agents list (left sidebar)**: Shows running/created agents
- **Create Agent button**: Launches agent creation dialog
- **Settings icon (top right)**: Opens workspace settings
- **AI Providers tab**: Where you configure API keys (next section!)

---

## Workspace Configuration

Before creating agents, configure your AI provider credentials. This happens in Settings.

### Step 1: Open Settings

Click the **⚙️ Settings icon** in the top-right corner of Agent Manager.

You see:
- Workspace settings
- AI Providers tab
- Local settings
- Advanced options

### Step 2: Select AI Providers Tab

Click **"AI Providers"** to see available options:
- Google AI (Gemini)
- Anthropic (Claude)
- OpenAI (GPT-4, GPT-4-mini)
- Ollama (local models)
- Custom (enterprise/self-hosted)

### Step 3: Verify Your API Keys Are Ready

Before configuring, you should have:
- [ ] Google AI API key (if using Gemini)
- [ ] Anthropic API key (if using Claude)
- [ ] OpenAI API key (if using GPT-4)

**Don't have these?** Jump to [03-ai-authentication.md](./03-ai-authentication.md) first, then come back here.

**Have them?** Continue below to input them.

---

## Configuring Workspace Preferences

In Settings, you can also configure:

### Basic Preferences

| Setting | Options | Default |
|---------|---------|---------|
| **Theme** | Light / Dark | Respects OS setting |
| **Font size** | 12-18pt | 14pt |
| **Auto-save artifacts** | On / Off | On |
| **Show agent logs** | On / Off | Off |

### Advanced Settings (Optional)

- **Artifact format**: Markdown or JSON
- **Agent timeout**: How long agents can run (5min-1hr)
- **Parallel agents**: How many agents can run simultaneously
- **Port configuration**: (Usually auto-detected)

**For now**: Leave these at defaults. You can adjust after you're comfortable with Antigravity.

---

## Understanding Workspace Structure

Once created, your workspace organizes as:

```
~/.antigravity/workspaces/your-workspace-name/
├── config.json                 # Workspace metadata
├── .git/                       # Git repository (if enabled)
├── agents/                     # Agent definitions
│   ├── agent-1.json
│   └── agent-2.json
├── artifacts/                  # Agent work products
│   ├── task-lists/
│   ├── implementation-plans/
│   └── walkthroughs/
└── projects/                   # Your code projects
    └── temp-converter/         # Example project folder
        ├── main.py
        └── test.py
```

### What Each Folder Does

- **config.json**: Workspace name, location, AI provider defaults
- **agents/**: Stores agent configurations (which AI model, instructions, etc.)
- **artifacts/**: Agent-generated Task Lists, Implementation Plans, Walkthroughs
- **projects/**: Your actual code and project files

**Key insight**: Artifacts are separate from code. When agents work, they produce artifacts (plans, task lists) visible in both Agent Manager and Editor. This is unique to Antigravity's design.

---

## Verification Checkpoint

Before moving to AI authentication, verify:

- [ ] Workspace created with meaningful name
- [ ] Workspace location confirmed
- [ ] Agent Manager opens without errors
- [ ] Settings accessible
- [ ] No error messages in console

**All verified?** Great! Now configure AI providers in [03-ai-authentication.md](./03-ai-authentication.md)

---

## Troubleshooting Setup

### Problem: "Workspace creation failed"
**Solution**:
1. Check that the directory path exists or is writable
2. Ensure Antigravity has permission to create folders:
   - macOS: Check folder permissions in Finder → Get Info
   - Linux: `chmod u+w ~/.antigravity`
   - Windows: Right-click folder → Properties → Security → Edit
3. Try a simpler workspace name (avoid special characters)
4. Try default location instead of custom path

### Problem: "Agent Manager won't open"
**Solution**:
1. Close Antigravity completely
2. Delete the workspace folder created
3. Restart Antigravity
4. Try creating workspace again with different name
5. Check logs: `~/.antigravity/logs/error.log`

### Problem: "Settings button not appearing"
**Solution**:
1. This is a UI loading issue (known in early versions)
2. Reload: Close workspace, click "Open Existing Workspace", select your workspace
3. Or restart Antigravity entirely

---

## What's Next?

You've created a workspace! Now it's time to connect to AI providers so agents can actually do work.

**Next step**: [03-ai-authentication.md](./03-ai-authentication.md) - Configure your AI provider API keys

**Time check**: You've spent ~15-20 minutes on setup. You have 40-45 minutes remaining.

---

## Workspace Best Practices

As you get comfortable with Antigravity, remember:

1. **One workspace per project** (or one workspace for all learning projects)
2. **Meaningful names** make it easy to find workspaces later
3. **Keep artifacts organized** — Antigravity stores them automatically
4. **Regular backups** — Workspace folders live in `~/.antigravity/`, backup regularly
5. **Multiple agents per workspace** — Create new agents for different tasks rather than creating new workspaces

---

## Quick Reference: Workspace Locations

| OS | Default Workspace Path |
|----|------------------------|
| **macOS** | `~/.antigravity/workspaces/` |
| **Linux** | `~/.antigravity/workspaces/` |
| **Windows** | `%USERPROFILE%\.antigravity\workspaces\` |

---

## Advanced: Git Integration (Optional)

When creating a workspace, you can optionally enable git integration. This creates a git repository in your workspace automatically.

**Advantage**: Antigravity can generate meaningful commit messages from agent work
**Disadvantage**: Slightly slower initialization

For this lesson, this is optional. Covered in detail in Lesson 7.
