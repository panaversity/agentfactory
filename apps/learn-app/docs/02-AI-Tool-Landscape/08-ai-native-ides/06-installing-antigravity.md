---
title: "Installing Antigravity IDE"
lesson_number: 6
proficiency_level: "A2-B1"
estimated_time: "60-75 minutes"
learning_objectives:
  - "Install Antigravity on your system (macOS, Linux, or Windows)"
  - "Understand the 3-surface architecture (Agent Manager, Editor, Integrated Browser)"
  - "Authenticate with Google DeepMind credentials"
  - "Create your first agent workspace"
  - "Verify installation by spawning a simple agent task"
---

# Lesson 6: Installing Antigravity IDE

## Why Antigravity After Zed and Cursor?

You have now experienced two distinct approaches to AI-native development: **Zed** (performance-focused, inline AI), and **Cursor** (VS Code evolution with autonomous agent mode). Antigravity represents a third architectural approach that you should understand.

### Three Reasons to Learn Antigravity Now

**1. Agent Control Plane Architecture**

Zed focuses on inline assistance. Cursor focuses on autonomous code generation with diffs. Antigravity introduces something different: a **control plane** for AI agents. Instead of asking an AI to write code within the editor, you spawn agents that work autonomously across multiple surfaces—managing themselves, editing code, and testing in an integrated browser. This is a fundamentally different model of human-AI collaboration.

**2. Three-Surface Design Philosophy**

Traditional IDEs are monolithic. Antigravity separates concerns into three distinct surfaces:
- **Agent Manager** (control hub for all agents)
- **Editor** (coding workspace with agent sidebar)
- **Integrated Browser** (Chrome with agent control signals)

This separation allows you and agents to work in parallel. While an agent tests code in the browser, you can review artifacts or prepare the next task. This is fundamentally different from waiting for a single interface to respond.

**3. Artifact-Driven Workflow**

Antigravity generates **Artifacts**—markdown files that track agent progress (Task List, Implementation Plan, Walkthrough). You approve plans *before* agents code, not after. This emphasizes specification quality and human oversight—core themes of this course.

## What You Will Do in This Lesson

Install Antigravity on your platform, understand its three-surface architecture, authenticate with Google credentials, create your first agent workspace, and verify installation with a simple test task. This is observational learning—you install, configure, and observe agent behavior without coding manually yourself.

---

## Part 1: Understanding Antigravity's Context

### Current Status (November 2025)

Antigravity is Google DeepMind's developer tool, representing their vision for AI-native development. As of November 2025, **Antigravity is in private beta with limited access via waitlist**.

**Access Status**:
- **Private Beta**: Invitation-only access
- **Waitlist**: Available at antigravity.google.com
- **Expected Public Beta**: Q1 2026 (estimated)
- **Current Users**: Early adopters, developers, selected companies

### What This Means for You

If you have access to the Antigravity beta, this lesson walks you through installation and first-use. If you're on the waitlist, you can follow along conceptually and understand the architecture, then install when access becomes available.

**For now**: Assume you either have beta access or will by the time you reach this lesson.

---

## Part 2: The 3-Surface Architecture Explained

Before installing, understand what Antigravity is. The key innovation is **three surfaces working together**:

### Surface 1: Agent Manager (Central Control Hub)

The Agent Manager is where you **manage and spawn agents**. Think of it as a dashboard for your AI workforce.

**What it does**:
- Lists all agents across all workspaces
- Shows agent status (idle, running, completed)
- Displays recent artifacts and tasks
- Creates new agents with custom instructions
- Configures agent behavior (autonomy level, model choice)

**Your interaction**: Open Agent Manager → Create agent → Set instructions → Give it a task → Monitor progress

**Visual layout**: Sidebar with agent list on left, agent details on right, global settings at bottom

### Surface 2: Editor (Coding Workspace)

The Editor is where **code lives and gets written**. Unlike Zed or Cursor where you write in the main editor, Antigravity's editor is where agents and you collaborate on code.

**What it does**:
- Tab-based file navigation (like VS Code)
- Syntax highlighting and formatting
- Agent sidebar showing suggested changes
- Change review and approval interface (like Cursor's diffs, but agent-aware)
- Tab autocomplete (similar to Zed/Cursor)

**Your interaction**: Editor shows code. Agent suggests changes in sidebar. You approve or request refinements.

**Visual layout**: Main editor on left (code), agent suggestions sidebar on right, file tabs at top

### Surface 3: Integrated Browser (Testing Environment)

The Integrated Browser is a **Chrome instance controlled by agents**. When an agent needs to test code, it opens the browser and interacts with it—entering data, clicking buttons, checking output.

**What it does**:
- Runs web applications created by agents
- Shows when agent has control (blue border around browser window)
- Allows manual interaction when not under agent control
- Captures screenshots for Walkthrough artifacts
- Tests across multiple browsers or screen sizes

**Your interaction**: Agent opens browser to test code. You watch the test happen. You see results in artifacts.

**Visual layout**: Separate window or browser panel, blue border indicates agent control

### Why Three Surfaces?

This design enables **parallel work**:

| You | Agent | Both |
|-----|-------|------|
| Review artifacts in Agent Manager | Implement code in Editor | Monitor via status indicators |
| Prepare next task description | Test in browser | Communicate progress through artifacts |
| Approve Implementation Plan | Generate code | Iterate on direction |

You are never blocked waiting for the agent to finish one thing before you start another. This is different from Zed and Cursor where the interface is often singular.

---

## Part 3: System Requirements

Before installing, verify your system meets minimum requirements:

### Hardware

**RAM**:
- Minimum: 16 GB
- Recommended: 32 GB (for large projects with multiple agents)

**CPU**:
- 4-core processor (Intel, AMD, Apple Silicon all supported)

**Disk Space**:
- Minimum: 2 GB for Antigravity
- Recommended: 10+ GB for projects and agent artifacts

**Network**:
- Stable internet connection (required for DeepMind API calls)
- 5+ Mbps bandwidth recommended

### Software

**Operating Systems**:
- macOS 12.0+ (Intel and Apple Silicon)
- Linux (Ubuntu 20.04+, Fedora 34+, Debian 11+)
- Windows 10 22H2+ or Windows 11 (with WSL2 for some features)

**Required Software**:
- Chrome 120+ (for integrated browser, may use system Chrome or bundled version)
- Node.js 18+ (for agent runtime, may be bundled)
- Google account (required for authentication)

**Optional but Recommended**:
- Bash or zsh (for terminal integration)
- Git (version control, agent can use it)

### Verification Checklist

Before installing, verify:

- [ ] System RAM is 16 GB or higher
- [ ] OS is current (within 1-2 years of release)
- [ ] 2+ GB disk space available
- [ ] Stable internet connection available
- [ ] Google account ready for authentication

---

## Part 4: Platform-Specific Installation

::::os-tabs

::macos
**Step 1: Download Antigravity**

1. Visit [antigravity.google.com/download](https://antigravity.google.com/download) (or equivalent beta link)
2. Select "macOS" (will auto-detect Apple Silicon vs Intel)
3. Download the .dmg file

**Step 2: Install**

1. Open the downloaded .dmg file
2. Drag **Antigravity.app** to the **Applications** folder
3. Wait for copy to complete (usually 30-60 seconds)

**Step 3: Launch**

Option 1 (from Applications):
- Open Finder → Applications → Double-click Antigravity.app

Option 2 (from terminal):
```bash
/Applications/Antigravity.app/Contents/MacOS/antigravity
```

Option 3 (add to PATH, optional):
```bash
# Add to ~/.zshrc or ~/.bash_profile
export PATH="/Applications/Antigravity.app/Contents/MacOS:$PATH"

# Then restart terminal and run:
antigravity
```

**Verification**: Antigravity launches, showing welcome screen with Agent Manager, Editor, and Browser panels.

::linux
Antigravity provides multiple installation methods for Linux:

**Option 1: Using AppImage (Recommended)**

```bash
# Download AppImage
curl -L https://antigravity.google.com/download/linux/appimage -o antigravity.AppImage

# Make executable
chmod +x antigravity.AppImage

# Run it
./antigravity.AppImage
```

**Option 2: Using Snap (if available)**

```bash
# Install from Snap Store
sudo snap install antigravity

# Launch
antigravity
```

**Option 3: Package Manager (Distribution-Specific)**

Ubuntu/Debian:
```bash
# Download .deb package
curl -L https://antigravity.google.com/download/linux/deb -o antigravity.deb

# Install
sudo apt install ./antigravity.deb

# Launch
antigravity
```

Fedora/RHEL:
```bash
# Download .rpm package
curl -L https://antigravity.google.com/download/linux/rpm -o antigravity.rpm

# Install
sudo dnf install antigravity.rpm

# Launch
antigravity
```

Arch Linux:
```bash
# Check AUR (Arch User Repository)
yay -S antigravity
antigravity
```

**Verification**: Run `antigravity --version` to confirm installation.

::windows
**Step 1: Check WSL2 (if needed)**

Some Antigravity features work better with WSL2 (Windows Subsystem for Linux 2). If you plan heavy terminal work with agents:

```powershell
# Check if WSL2 is installed
wsl --list --verbose

# If not installed:
wsl --install

# If installed, ensure default is WSL2:
wsl --set-default-version 2
```

WSL2 is optional for basic usage but recommended for advanced workflows.

**Step 2: Download and Install**

1. Visit [antigravity.google.com/download](https://antigravity.google.com/download)
2. Select "Windows"
3. Download the .exe installer
4. Run the installer (.exe file)
5. Follow setup wizard (choose install location, start menu, desktop shortcut)
6. Installer will launch Antigravity automatically

**Alternative: Using Winget**

```powershell
# Install via Windows Package Manager
winget install google.antigravity

# Launch
antigravity
```

**Verification**: Antigravity launches with welcome screen. If it does not launch, check Event Viewer for errors or re-run installer as Administrator.

::::

---

## Part 5: Initial Setup and Authentication

### First Launch: Welcome Screen

When you launch Antigravity for the first time, you'll see a welcome screen:

**Step 1: Google Account Sign-In**

1. Click "Sign In with Google"
2. Your default browser opens to accounts.google.com
3. Sign in with your Google account (or create one)
4. Authorize Antigravity to access your account
5. Browser redirects back to Antigravity (authentication complete)

Antigravity requires Google Sign-In for two reasons:
- API access to Google DeepMind services
- Workspace synchronization across devices

### Step 2: Workspace Selection

After authentication, choose workspace:

**Option 1: Create New Local Workspace**

- Workspace name: "my-dev" (or similar)
- Location: Local (stores in ~/.antigravity/workspaces/)
- Benefits: Fast, works offline for editing, all data local

**Option 2: Create Cloud Workspace**

- Workspace name: "my-dev-cloud"
- Location: Cloud (syncs to Google Cloud)
- Benefits: Accessible from multiple devices, automatic backups, shared with teammates

**For now**: Choose "Local" (simpler for first use).

### Step 3: Workspace Configuration

Antigravity presents basic configuration options:

**Theme**:
- Light (white background, dark text)
- Dark (dark background, light text)
- Auto (switches based on system settings)

Choose based on your preference (you can change later).

**Font Size**:
- Small (12px)
- Medium (14px)
- Large (16px)

Choose "Medium" (14px) for balanced readability.

**Enable Telemetry** (Optional):
- Helps Google improve Antigravity
- Disabling is fine; you can enable later

### Step 4: AI Model Selection

Choose default AI model:

**Available Models**:
- Gemini 2.0 (newest, recommended if available)
- Gemini 1.5 Pro (stable, widely used)
- Gemini 1.5 Flash (faster, less capable)

**Recommendation**: Choose Gemini 2.0 or 1.5 Pro. These are most capable for multi-step tasks.

---

## Part 6: Agent Manager Deep Dive

Once setup completes, you're in the main Antigravity interface. The **Agent Manager** is your primary control panel.

### Agent Manager Layout

**Left Sidebar: Agent List**
- Shows all agents in current workspace
- Each agent shows: name, status (idle/running/completed), last update time
- "Create New Agent" button at bottom
- Workspace selector at top

**Center Panel: Agent Details**
- Selected agent's information
- Current instructions
- Recent tasks and artifacts
- Status indicators

**Right Panel: Global Settings**
- Agent Manager settings
- Workspace configuration
- System preferences
- API configuration

### Creating Your First Agent

**Step 1: Click "Create New Agent"**

Fill in:
- **Agent Name**: "python-developer" (or similar, must be unique in workspace)
- **Description**: "Writes Python code and tests" (optional but helpful)
- **Instructions**: See next section

**Step 2: Agent Instructions**

Agent instructions are like `.cursorrules` but per-agent. They guide how the agent behaves.

**Example instructions for a Python-focused agent**:

```
You are a Python developer focused on writing clean, tested code.

When given a task:
1. Generate a Task List artifact with 3-5 steps
2. Create an Implementation Plan explaining your approach
3. Write code with type hints and docstrings
4. Write tests to verify correctness
5. Create a Walkthrough artifact with results and verification

Always ask clarifying questions if requirements are vague.
Prefer clarity over assumptions.
```

These instructions stay with the agent across all tasks.

**Step 3: Configure Agent Settings**

**Autonomy Level**:
- **Ask Always**: Agent stops before major actions (safe, slower)
- **Ask Sometimes**: Agent auto-approves small changes, asks for big ones (balanced)
- **Full Auto**: Agent completes tasks end-to-end without asking (fast, requires trust)

For first use: Choose **"Ask Sometimes"** (balanced).

**Model Selection**:
- Override default model for this agent
- Leave as default for now

**Artifact Verbosity**:
- **Minimal**: Just results
- **Detailed**: Full thinking and reasoning
- **Full**: Everything, including debug output

For learning: Choose **"Detailed"** (you'll see more reasoning).

**Step 4: Create Agent**

Click "Create" to spawn the agent. It now appears in Agent List as **Idle**.

---

## Part 7: Understanding Artifacts

Artifacts are the heart of Antigravity's workflow. Before completing your first task, understand what artifacts are.

### What Are Artifacts?

Artifacts are **markdown files that agents generate** to track their progress. They're not the code itself (code goes in Editor)—they're the agent's thinking, planning, and verification.

**Where Artifacts Live**:
```
~/.antigravity/workspaces/[workspace-name]/artifacts/[agent-name]/
```

Example:
```
~/.antigravity/workspaces/my-dev/artifacts/python-developer/
```

### Three Artifact Types

**Artifact 1: Task List** (`task-list.md`)

The agent's TODO tracker. Generated as first step of any task.

**Example**:
```markdown
# Task List: Create Hello World Web Page

- [ ] Create index.html with HTML structure
- [ ] Add CSS styling with blue background
- [ ] Test page in browser to verify display
```

**Your role**: Review the task list. Does it match what you asked for? Are the steps reasonable? Approve or ask agent to revise.

**Artifact 2: Implementation Plan** (`implementation-plan.md`)

The agent's strategy before coding. Explains approach, design decisions, potential challenges.

**Example**:
```markdown
# Implementation Plan

## Overview
Create a simple HTML page with "Hello, Antigravity!" text and blue background.

## Approach
1. Create index.html with semantic HTML
2. Inline CSS in <style> tag (no separate stylesheet for simplicity)
3. Use flexbox to center text vertically and horizontally
4. Test in browser to verify appearance

## Design Decisions
- Inline CSS: Simpler for single-page project, no build needed
- Flexbox: Modern, browser-compatible, handles centering elegantly
- Semantic HTML: <header> for page title region

## Potential Challenges
- Browser caching: May need hard refresh (Ctrl+Shift+R) to see changes
- Font sizing: Will adjust if text too small on initial view

## Success Criteria
- Page displays "Hello, Antigravity!" in center
- Background is blue
- Text is readable (good contrast)
```

**Your role**: Review plan before agent codes. If plan is wrong, tell agent to revise before implementation. This is the "spec-first" principle in action.

**Artifact 3: Walkthrough** (`walkthrough.md`)

Final report showing what was done, verification results, and next steps.

**Example**:
```markdown
# Walkthrough: Hello World Web Page

## Completed Tasks
- [x] Created index.html with HTML structure
- [x] Added CSS styling with blue background
- [x] Tested page in browser

## Implementation Summary
Created index.html with:
- Semantic HTML structure
- Inline CSS styling (blue background, centered text)
- Flexbox layout for vertical/horizontal centering

## Verification Results
✅ Page displays successfully in browser
✅ Text "Hello, Antigravity!" visible and centered
✅ Background color is blue
✅ No console errors

## Screenshots
[Agent captured browser screenshot here]

## What's Next
- Could add hover effects to text
- Could add animations
- Could refactor CSS into separate file for larger projects
```

**Your role**: Review verification. Did it work? Did the agent do what you asked?

### Artifact Review Workflow

**When agent generates task list**:
1. Agent pauses and waits for approval
2. You review task list
3. You either approve ("Looks good, proceed") or request changes ("Add a step for error handling")
4. Agent either proceeds or regenerates list

**This is different from Zed/Cursor** where you review code after it's written. In Antigravity, you review the *plan* first.

---

## Part 8: Verification Test

Now you'll install and verify Antigravity works by completing a simple task.

### Test Task Setup

**Task**: "Create a simple HTML page that displays 'Hello, Antigravity!' with a blue background"

This is intentionally simple—just enough to verify:
- Agent spawns correctly
- Generates artifacts
- Implements code
- Opens browser
- Tests successfully

### Step-by-Step Verification

**Step 1: Open Agent Manager and Select Your Agent**

In Antigravity, Agent Manager shows your "python-developer" agent (or whatever you named it) as **Idle**.

**Step 2: Create New Task**

Click "New Task" button. A dialog appears:

```
Task Description:
"Create a simple HTML page that displays 'Hello, Antigravity!' with a blue background"

Agent: [Select "python-developer"]

Autonomy: [Ask Sometimes]
```

Click "Create Task" → Agent starts working.

**Step 3: Agent Generates Task List**

First thing agent does: generates `task-list.md` and pauses.

You see in Agent Details panel:
```
Task: Create HTML page
Status: Waiting for Approval
Artifact: task-list.md (ready to review)
```

**Click "View Artifact"** to see:
```markdown
# Task List

- [ ] Create index.html file
- [ ] Write HTML structure (<!DOCTYPE>, <html>, <head>, <body>)
- [ ] Add CSS style for blue background
- [ ] Add text "Hello, Antigravity!" centered on page
- [ ] Test in browser
```

**Approve**: Click "Approve and Continue". Agent proceeds to next step.

**Step 4: Agent Generates Implementation Plan**

Agent generates `implementation-plan.md` and pauses again.

You see:
```
Task: Create HTML page
Status: Waiting for Approval
Artifact: implementation-plan.md (ready to review)
```

**Click "View Artifact"** to see:
```markdown
# Implementation Plan

## Approach
1. Create index.html in project root
2. Use HTML5 semantic structure
3. Inline CSS with flexbox for centering
4. Body background color: #0066FF (blue)

## Implementation Details
- DOCTYPE: html5
- Meta tags: viewport, charset UTF-8
- CSS: display: flex, justify-content: center, align-items: center
- Font size: 32px, white text for contrast

## Success Criteria
- ✅ Page renders without errors
- ✅ Text centered both horizontally and vertically
- ✅ Background color is blue
- ✅ No console errors in browser
```

**Approve**: Click "Approve and Continue". Agent proceeds to implementation.

**Step 5: Agent Implements Code in Editor**

Agent creates `index.html` file and opens it in the Editor surface.

You see the code:
```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Hello Antigravity</title>
    <style>
        body {
            margin: 0;
            padding: 0;
            height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            background-color: #0066FF;
            font-family: Arial, sans-serif;
        }

        h1 {
            color: white;
            font-size: 48px;
            text-align: center;
        }
    </style>
</head>
<body>
    <h1>Hello, Antigravity!</h1>
</body>
</html>
```

Agent shows this in Editor sidebar with "Approve Changes?" prompt.

**Review Changes**: Scan the code. Looks reasonable? Click "Approve".

**Step 6: Agent Opens Browser to Test**

Browser surface opens with blue border (indicating agent control).

You see:
- Blue background
- White centered text: "Hello, Antigravity!"
- No errors

Agent interacts with browser (scrolls, inspects, takes screenshot) to verify appearance.

**Step 7: Agent Generates Walkthrough**

Agent creates final artifact: `walkthrough.md`

You see:
```markdown
# Walkthrough: Hello, Antigravity! Page

## Tasks Completed
- [x] Created index.html
- [x] Implemented CSS styling
- [x] Tested in browser

## What Was Built
Simple HTML page with:
- Blue background (#0066FF)
- White centered text "Hello, Antigravity!"
- Flexbox layout for perfect centering
- Responsive design (works on mobile too)

## Verification
✅ Page renders correctly
✅ Text is centered
✅ Background color is correct
✅ No console errors
✅ Browser test passed

## Browser Screenshot
[Screenshot showing blue background with centered white text]

## Next Steps
Could enhance with:
- Hover effects on text
- Animations
- Multiple sections
```

**Step 8: Mark Task Complete**

Click "Task Complete" in Agent Manager. Agent status changes from "Running" to "Completed".

### Verification Checklist

After completing the test task, verify:

- [ ] Antigravity launched successfully with all 3 surfaces visible
- [ ] Agent created without errors
- [ ] Task List artifact generated and approved
- [ ] Implementation Plan artifact generated and approved
- [ ] Code implemented in Editor surface
- [ ] Browser opened with blue border (agent control visible)
- [ ] HTML page displayed correctly
- [ ] Walkthrough artifact generated with verification
- [ ] Task marked as Completed

If all checkboxes pass: **Installation successful!**

---

## Part 9: Troubleshooting Common Issues

### Issue 1: "Cannot launch Antigravity"

**Symptoms**: Clicking Antigravity does nothing or error appears immediately.

**Possible Causes**:
- Insufficient permissions (Windows)
- Corrupted installation
- Missing dependencies (Chrome, Node.js)

**Solutions**:

macOS:
```bash
# Check if Antigravity exists
ls /Applications/Antigravity.app

# Try launching from terminal (see detailed error)
/Applications/Antigravity.app/Contents/MacOS/antigravity
```

Windows:
```powershell
# Right-click installer.exe → "Run as Administrator"
# Or check Event Viewer for detailed error
```

Linux:
```bash
# Check dependencies
node --version  # Should be 18+
google-chrome --version  # Should be 120+

# Try AppImage directly
./antigravity.AppImage --verbose
```

### Issue 2: "Google sign-in fails"

**Symptoms**: Browser opens but authentication loop or "Connection refused" error.

**Possible Causes**:
- Internet connection issue
- Google account restrictions
- Antigravity not whitelisted for your account

**Solutions**:
1. Verify internet connection: `ping google.com`
2. Try signing in to Google manually: accounts.google.com
3. Check if beta access was revoked (email from Google)
4. Clear browser cache: Ctrl+Shift+Delete / Cmd+Shift+Delete
5. Try different Google account (if you have multiple)

### Issue 3: "Agent won't spawn"

**Symptoms**: Click "Create New Agent" but nothing happens or error appears.

**Possible Causes**:
- Google authentication incomplete
- API quota exceeded
- Agent name duplicate
- Node.js runtime issue

**Solutions**:
1. Verify sign-in: Antigravity menu → Account → Check authentication status
2. Wait 1 hour (quota resets)
3. Use different agent name (must be unique in workspace)
4. Restart Antigravity: quit and relaunch

### Issue 4: "Agent starts but doesn't generate artifacts"

**Symptoms**: Agent shows "Running" but no task list/plan/walkthrough appears.

**Possible Causes**:
- Autonomy settings misconfigured
- API connection interrupted
- Artifact verbosity too low
- Task description too vague

**Solutions**:
1. Check autonomy level: Agent settings → "Ask Sometimes" is recommended
2. Check internet connection (constant during task)
3. Set artifact verbosity: Agent details → "Detailed" or "Full"
4. Try again with clearer task: "Create HTML file with heading 'Test'"

### Issue 5: "Browser won't open or shows blank"

**Symptoms**: Browser surface loads but page stays blank or shows "Cannot reach server".

**Possible Causes**:
- HTML file not created in correct location
- File path incorrect
- Browser not authorized for local files
- Chrome not compatible version

**Solutions**:
1. Check file exists: `ls ~/.antigravity/workspaces/[name]/artifacts/`
2. Agent settings → Check Chrome version (should be 120+)
3. Manually open HTML: Browser menu → "Open Local File" → browse to file
4. Ask agent to create simpler file (no external dependencies)

### Issue 6: "Editor shows code but changes don't save"

**Symptoms**: Agent writes code in Editor, you approve, but file doesn't persist.

**Possible Causes**:
- File system permissions
- Workspace not properly initialized
- Agent trying to write outside workspace

**Solutions**:
1. Check workspace permissions: `ls -la ~/.antigravity/workspaces/`
2. Verify agent has write permission to workspace
3. Restart Antigravity and try again
4. Check if file actually exists: terminal → `find ~/.antigravity -name "*.html"`

---

## Part 10: Practice Checklist

Before moving to Lesson 7, complete this verification checklist:

**Installation**:
- [ ] Antigravity launches without errors on your platform
- [ ] Sign in with Google account successfully
- [ ] Create local workspace
- [ ] All 3 surfaces visible (Agent Manager, Editor, Browser)

**Configuration**:
- [ ] Create at least one agent ("python-developer" or similar)
- [ ] Set agent instructions for Python/web development
- [ ] Configure autonomy to "Ask Sometimes"
- [ ] Set artifact verbosity to "Detailed"

**Artifacts**:
- [ ] Understand what Task List artifact is
- [ ] Understand what Implementation Plan artifact is
- [ ] Understand what Walkthrough artifact is
- [ ] Know where artifacts are stored on your system

**Verification Test**:
- [ ] Complete "Hello, Antigravity!" test task start to finish
- [ ] Review and approve each artifact before agent proceeds
- [ ] See HTML page render correctly in browser
- [ ] View final Walkthrough artifact
- [ ] Task marked as Completed

**Optional Stretch**:
- [ ] Create second agent with different instructions (e.g., "web-designer")
- [ ] Modify agent instructions and see how it changes behavior
- [ ] Try autonomy level "Full Auto" for a simple task (watch speed difference)

---

## What You Learned

You installed Antigravity on your system and verified a complete agent workflow from task creation through browser testing. You understand the three-surface architecture (Agent Manager for control, Editor for coding, Integrated Browser for testing) and how artifacts (Task List, Plan, Walkthrough) guide agent behavior. You created your first agent, configured its instructions, and observed how it generates structured plans before implementing code—embodying the "specification-first" approach central to AI-native development.

---

## Next Lesson Preview

Lesson 7 explores Antigravity's advanced features: artifact-driven workflows, context-aware editing, parallel task execution, and how agents iterate based on feedback. You'll build a complete project (like the temperature converter in Lessons 3 and 5) but this time using Antigravity's three-surface system, experiencing how agent-driven development differs fundamentally from inline assistance or autonomous code generation alone.

---

## Additional Resources

**Official Resources**:
- Antigravity Documentation: [antigravity.google.com/docs](https://antigravity.google.com/docs)
- Google AI Studio (for API keys): [aistudio.google.com](https://aistudio.google.com)
- DeepMind Blog: [deepmind.google.com/blog](https://deepmind.google.com/blog)

**Video Tutorials**:
- "Your First Antigravity Agent" (10 minutes)
- "Understanding Three Surfaces" (8 minutes)
- "Artifact-Driven Development" (12 minutes)

**Community**:
- Antigravity Discord: [discord.gg/antigravity](https://discord.gg/antigravity)
- Google DeepMind Discussions: [discuss.deepmind.google](https://discuss.deepmind.google)
- Stack Overflow: Tag `antigravity-ide`

**Related Lessons in This Course**:
- Lesson 2: Installing Zed IDE (compare performance-focused approach)
- Lesson 4: Installing Cursor IDE (compare VS Code-based approach)
- Lesson 8: Comparative Capstone (side-by-side evaluation of all three)
