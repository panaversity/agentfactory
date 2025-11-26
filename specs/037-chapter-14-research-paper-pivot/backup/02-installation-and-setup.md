---
title: "Installation & Setup - Getting Spec-Kit Plus and Playwright MCP Running"
chapter: 14
lesson: 2
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Tool Configuration (Claude Code vs Gemini CLI)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can choose and configure an AI tool for Spec-Kit Plus work with Playwright MCP"

  - name: "MCP Installation and Configuration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can install @anthropic-ai/mcp-playwright, configure it in Claude Desktop, and verify functionality"

  - name: "Browser Session Persistence"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand, Apply"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can configure session persistence to maintain logged-in state across Playwright automation runs"

  - name: "Project Structure Navigation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand, Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify and explain the purpose of `.specify/`, `specs/`, and `history/` directories"

  - name: "Installation Verification"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can verify Spec-Kit Plus and Playwright MCP installation and run test commands successfully"

learning_objectives:
  - objective: "Install Spec-Kit Plus framework successfully on your development machine"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful installation verification (run `/sp.*` commands)"

  - objective: "Install and configure Playwright MCP for browser automation"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful Playwright MCP installation verification via AI tool"

  - objective: "Configure Claude Code or Gemini CLI with Playwright MCP and session persistence"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful MCP server connection test and session persistence demo"

  - objective: "Navigate and understand the Spec-Kit Plus project structure with MCP integration"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of folder hierarchy, artifact relationships, and MCP configuration"

  - objective: "Verify complete setup by running test commands for both Spec-Kit Plus and Playwright MCP"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful command execution with expected output for both frameworks"

cognitive_load:
  new_concepts: 8
  assessment: "8 new concepts (Spec-Kit Plus config, MCP installation, session persistence, Playwright basics, Claude Desktop config, project structure, tool configuration, verification) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore custom MCP configurations; set up multiple MCPs (playwright + filesystem); compare Playwright with other automation tools"
  remedial_for_struggling: "Step-by-step guided installation with video/visual aids; installation verification checklist; platform-specific troubleshooting guide"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/chapter-14-spec-kit-plus-hands-on/spec.md"
created: "2025-11-25"
last_modified: "2025-11-25"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.1.0"
---

# Installation & Setup - Getting Spec-Kit Plus and Playwright MCP Running

In Lesson 1, you learned WHAT Spec-Kit Plus is (three-tier architecture, Horizontal/Vertical Intelligence) and WHY this book uses it (implements SDD-RI concepts). Now you'll install both the Spec-Kit Plus framework AND Playwright MCP for browser automation. You'll configure your AI tool, set up session persistence for Gemini.google.com login, and verify everything works together.

By the end of this lesson, you'll have:
- Spec-Kit Plus framework installed and verified
- Playwright MCP installed and configured in Claude Desktop
- Session persistence enabled for maintaining logged-in state
- Your AI tool fully configured to work with both frameworks
- A test project initialized and ready for video generation work

---

## Prerequisites: Install Python First

:::caution Python Required
Spec-Kit Plus requires **Python 3.12 or higher**. If you haven't installed Python yet, complete the Python installation first before continuing.

**Quick Check** - Run this in your terminal:
```bash
python --version
```

**If you see Python 3.12+ (e.g., `Python 3.12.0`)**: Continue to the next section.

**If you see an error or a version below 3.12**: Follow the installation guide at [Installing Python](https://ai-native.panaversity.org/docs/Python-Fundamentals/introduction-to-python/installing-python) and return here when complete.
:::

---

## Install Spec-Kit Plus Framework

Now let's install the Spec-Kit Plus framework. This is independent of your AI tool choice.

### Installation Steps

**Step 1: Verify Python Version**

Spec-Kit Plus requires Python 3.12 or higher. Check your version first:

```bash
# Check Python version (must be 3.12+)
python --version

# If you see Python 3.11 or lower, upgrade Python first:
# - macOS: brew install python@3.12
# - Ubuntu: sudo apt install python3.12
# - Windows: Download from python.org
```

**Step 2: Install Spec-Kit Plus**

With Python 3.12+ confirmed, install Spec-Kit Plus:

```bash
# Install the latest version
pip install specifyplus

# Verify installation
specifyplus --version
```

**Expected Output:**
```
Spec-Kit Plus version 2.1.0
```

**Step 3: Initialize Your First Project**

```bash
# Create a new Spec-Kit Plus project
specifyplus init video-generator-project
```

**Interactive Prompts:**

During initialization, you'll see these prompts:

```
? Select AI Tool:
  > Claude Code
    Gemini CLI

? Select Terminal:
  > bash
    powershell (Windows only)
```

**Recommendations:**
- **AI Tool**: Choose **Claude Code** (recommended for this book with Playwright MCP support)
- **Terminal**: Choose **bash** (or powershell if on Windows without WSL)

**Step 4: Navigate to the project**
```bash
cd video-generator-project
```

**Step 5: Verify Project Structure**

After initialization, you should see the following directory structure:

```
video-generator-project/
├── .claude/
│   └── commands/                    # Slash commands for SDD workflow
│       ├── sp.adr.md                # Document architectural decisions
│       ├── sp.analyze.md            # Cross-artifact consistency checks
│       ├── sp.checklist.md          # Generate custom checklists
│       ├── sp.clarify.md            # Refine specifications
│       ├── sp.constitution.md       # Create project constitution
│       ├── sp.git.commit_pr.md      # Commit and create PRs
│       ├── sp.implement.md          # Generate code from tasks
│       ├── sp.phr.md                # Record prompt history
│       ├── sp.plan.md               # Generate implementation plans
│       ├── sp.specify.md            # Create specifications
│       └── sp.tasks.md              # Break plans into atomic tasks
│
├── .specify/
│   ├── memory/
│   │   └── constitution.md          # Project-wide rules and principles
│   │
│   ├── scripts/
│   │   └── bash/                    # Automation scripts
│   │       ├── check-prerequisites.sh
│   │       ├── common.sh
│   │       ├── create-adr.sh
│   │       ├── create-new-feature.sh
│   │       ├── create-phr.sh
│   │       ├── setup-plan.sh
│   │       └── update-agent-context.sh
│   │
│   └── templates/                   # Templates for specs, plans, tasks, ADRs, PHRs
│       ├── adr-template.md
│       ├── agent-file-template.md
│       ├── checklist-template.md
│       ├── phr-template.prompt.md
│       ├── plan-template.md
│       ├── spec-template.md
│       └── tasks-template.md
│
├── .git/                            # Git repository
├── CLAUDE.md                        # Agent instructions and guidelines
├── README.md                        # Project documentation
└── .gitignore                       # Git ignore rules
```

**Note**: The `specs/`, `history/prompts/`, and `history/adr/` directories will be created automatically when you start your first feature.

**Explanation of Key Directories**:

- **`.claude/commands/`** - Slash commands you'll use throughout the SDD workflow (/sp.specify, /sp.plan, etc.)
- **`.specify/memory/`** - Your project constitution (created once, referenced always)
- **`.specify/scripts/`** - Automation scripts for PHRs, ADRs, and feature setup
- **`.specify/templates/`** - Templates that guide spec, plan, task, ADR, and PHR creation
- **`CLAUDE.md`** - Agent instructions that guide your AI collaborator's behavior
- **`specs/`** - (Created later) Your feature specifications
- **`history/`** - (Created later) ADRs and PHRs for knowledge capture

---

## Install Playwright MCP

Playwright MCP (Model Context Protocol) is a browser automation tool that allows Claude and other AI models to interact with web pages. For this chapter, you'll use it with Gemini.google.com to automate video generation.

### What is Playwright MCP?

**Playwright** is a browser automation library (similar to Selenium, but newer and faster).

**MCP (Model Context Protocol)** exposes Playwright as a tool your AI model can use. It enables Claude to:
- Navigate web pages
- Fill forms
- Click buttons
- Take screenshots
- Extract text from web pages
- All while maintaining a persistent browser session

In Chapter 14, you'll use Playwright MCP to:
1. Open Gemini.google.com in a browser
2. Log into your Google account (session persists)
3. Automate video generation through Gemini's interface
4. Download the generated video
5. Upload it to YouTube

### Installation Steps

**Step 1: Verify Node.js is Installed**

Playwright MCP requires Node.js 16+ (for npm package management):

```bash
# Check Node.js version
node --version

# Check npm version
npm --version

# Expected output:
# v20.10.0  ✓ (or higher)
# 10.2.3    ✓ (or higher)
```

If Node.js is not installed:
- **macOS**: `brew install node`
- **Ubuntu**: `sudo apt install nodejs npm`
- **Windows**: Download from nodejs.org

**Step 2: Install Playwright MCP Globally**

```bash
# Install @anthropic-ai/mcp-playwright globally
npm install -g @anthropic-ai/mcp-playwright

# Verify installation
npx @anthropic-ai/mcp-playwright --version
```

**Expected Output:**
```
@anthropic-ai/mcp-playwright version 1.0.0
MCP Playwright ready for use with Claude
```

**Step 3: Configure Claude Desktop for Playwright MCP**

To use Playwright MCP with Claude, you need to configure it in Claude Desktop's configuration file.

**For macOS:**

1. Open the Claude Desktop configuration file:
```bash
open ~/.claude/claude_desktop_config.json
# If the file doesn't exist, create it
touch ~/.claude/claude_desktop_config.json
```

2. Add the Playwright MCP server configuration. If the file is empty or new, use this complete configuration:

```json
{
  "mcpServers": {
    "playwright": {
      "command": "npx",
      "args": ["-y", "@anthropic-ai/mcp-playwright"]
    }
  }
}
```

If the file already has other content, add the `"playwright"` block to the `mcpServers` object:

```json
{
  "mcpServers": {
    "existing-server": {
      // ... existing config
    },
    "playwright": {
      "command": "npx",
      "args": ["-y", "@anthropic-ai/mcp-playwright"]
    }
  }
}
```

**For Linux:**

1. Create or edit the configuration file:
```bash
mkdir -p ~/.claude
nano ~/.claude/claude_desktop_config.json
```

2. Add the Playwright MCP server configuration (same as macOS above)

3. Save and exit (Ctrl+O, Enter, Ctrl+X)

**For Windows:**

1. Open PowerShell as Administrator and navigate to your home directory:
```powershell
# Open config file with Notepad
notepad $env:USERPROFILE\.claude\claude_desktop_config.json

# If file doesn't exist, create it first:
New-Item -ItemType Directory -Force -Path "$env:USERPROFILE\.claude"
New-Item -ItemType File -Path "$env:USERPROFILE\.claude\claude_desktop_config.json"
```

2. Add the Playwright MCP server configuration (same JSON as above)

3. Save the file

**Step 4: Restart Claude Desktop**

For the configuration to take effect:

1. Close Claude Desktop completely (not just minimize)
2. Wait 2 seconds
3. Reopen Claude Desktop

You should see a message indicating MCP servers are loading. This confirms Playwright MCP is connected.

---

## Configure Browser Session Persistence

When using Playwright to interact with Gemini.google.com, you'll want to stay logged in between sessions. Browser session persistence saves your login state so you don't need to re-authenticate every time you run video generation.

### What is Session Persistence?

**Session persistence** means:
- Browser cookies and local storage are saved to disk
- When Playwright opens the browser again, your login state is maintained
- You log into Google once, then stay logged in for future video generation runs

**Without persistence**: Each time Playwright opens the browser, you'd need to log in again (very slow for iterative video generation).

**With persistence**: Log in once, then run video generation 10 times without re-authenticating.

### How to Set Up Session Persistence

When you use Playwright MCP in your video generation specification, you'll include a configuration that tells Playwright where to store browser data:

```javascript
// This is NOT code you write—Claude will generate this
// Just understand the concept:

const browserContext = await browser.newContext({
  storageState: '/path/to/session-storage'
  // storageState tells Playwright to save/restore login cookies and local storage
});
```

**In Practice (Chapter 14 Tasks)**:

When you write your specification for video generation, you'll specify:
- "Use Playwright MCP with session persistence enabled"
- "Store browser session in `.session-storage/gemini-session.json`"

Claude will then generate implementation code that:
1. First run: Opens browser, you manually log into Google, session is saved
2. Subsequent runs: Loads saved session, you're already logged in

### Manual First-Run Login

For your first video generation run:

1. **Specification** asks Claude to open Gemini.google.com
2. **You manually log in** to your Google account (once)
3. **Playwright saves** your login state
4. **Future runs** load your saved session automatically

---

## Verify Commands and Tools Work

Now let's test that everything is connected.

### Test 1: Access Spec-Kit Plus Commands

Open Claude Code (or your chosen AI tool) in the `video-generator-project` directory:

```bash
# In your terminal, from video-generator-project directory
# Launch Claude Code interface
claude

# OR GEMINI
gemini
```

Inside the terminal, verify Spec-Kit Plus commands are available:

```
# Type
/sp.
```

You should see core Spec-Kit Plus commands:
- `/sp.constitution` - Build your constitution
- `/sp.specify` - Launch specification workflow
- `/sp.clarify` - Refine and validate specs
- `/sp.plan` - Generate implementation plan
- `/sp.adr` - Document architectural decisions
- `/sp.tasks` - Decompose plan into tasks
- `/sp.implement` - Generate code
- `/sp.phr` - Record prompt history

If the command is recognized, your orchestrator is configured correctly.

### Test 2: Verify Playwright MCP is Available

In Claude Desktop (or your AI tool), ask:

```
Can you list the tools available to you right now?
What MCP servers are connected?
```

**Expected Response** should include:
```
MCP Servers:
- playwright (browser automation)
- [any other configured servers]

Available Playwright Tools:
- navigate: Go to a URL
- click: Click an element
- fill: Fill a text field
- screenshot: Take a page screenshot
- extract_text: Get text from page
[... other tools ...]
```

If Playwright MCP is NOT listed, check:
1. Configuration file exists: `~/.claude/claude_desktop_config.json`
2. JSON is valid (no syntax errors)
3. Claude Desktop has been restarted since configuration
4. Node.js is installed (run `node --version`)

#### Practice Exercise

> **Ask your AI**: "I've installed both Spec-Kit Plus and Playwright MCP. Can you verify my setup? Check that Spec-Kit Plus commands are available via `/sp.` and that Playwright MCP tools are accessible. Then explain what Playwright will allow me to do when generating videos with Gemini."

**Expected Outcome**: Your AI companion should confirm both frameworks are available, list key Playwright capabilities (navigate, click, screenshot, etc.), and explain how these tools enable browser automation for video generation.

---

## Common Mistakes

### Mistake 1: Confusing Spec-Kit Plus with Claude Code

**The Error**: "I installed Claude Code, so I have Spec-Kit Plus now."

**Why It's Wrong**: Spec-Kit Plus is a separate framework. Claude Code is just the AI tool that executes Spec-Kit Plus commands.

**The Fix**: Install both: `pip install specifyplus` (framework) AND configure Claude Code/Gemini CLI (AI tool).

### Mistake 2: Skipping Project Initialization

**The Error**: Creating folders manually instead of running `specifyplus init`

**Why It's Wrong**: You miss critical infrastructure (.specify/ templates, configuration files, directory structure).

**The Fix**: Always run `specifyplus init <project-name>` to set up proper structure.

### Mistake 3: Installing Playwright Locally Instead of Globally

**The Error**: Running `npm install @anthropic-ai/mcp-playwright` in project directory (without `-g`)

**Why It's Wrong**: Playwright MCP needs to be available system-wide for Claude Desktop to access it.

**The Fix**: Use `npm install -g @anthropic-ai/mcp-playwright` (with `-g` flag for global installation).

### Mistake 4: Incorrect MCP Configuration Path

**The Error**: Creating `claude_desktop_config.json` in wrong location:
- Wrong: `~/claude_desktop_config.json` (home directory)
- Wrong: `~/.config/claude_desktop_config.json` (wrong folder)

**Why It's Wrong**: Claude Desktop looks for config in `~/.claude/` specifically.

**The Fix**: File MUST be at: `~/.claude/claude_desktop_config.json`

### Mistake 5: Forgetting to Restart Claude Desktop

**The Error**: "I configured Playwright MCP but it's not showing up in Claude"

**Why It's Wrong**: Configuration changes require Claude Desktop restart (it reads config on startup).

**The Fix**:
1. Close Claude Desktop completely (quit, not minimize)
2. Wait 2 seconds
3. Reopen Claude Desktop
4. Check for MCP server connection message

### Mistake 6: JSON Syntax Errors in Configuration File

**The Error**: Missing commas, unclosed braces, or invalid JSON in `claude_desktop_config.json`

**Why It's Wrong**: JSON parsing fails silently; Claude Desktop won't load any MCP servers.

**The Fix**: Validate JSON syntax:
```bash
# macOS/Linux
python -m json.tool ~/.claude/claude_desktop_config.json

# If valid, no error. If invalid, error shows line number.
```

---

## Try With AI

Ready to verify your Spec-Kit Plus and Playwright MCP installation? Use these prompts:

**🔍 Verify Complete Setup:**
> "I've installed both Spec-Kit Plus and Playwright MCP. Walk me through verification: (1) Check that `/sp.` commands are available in my project, (2) Confirm Playwright MCP tools are connected to you, (3) Explain the difference between Spec-Kit Plus (framework) and Playwright MCP (browser automation). Show me the exact commands to run."

**🌐 Understand Session Persistence:**
> "Explain browser session persistence in the context of video generation with Gemini: Why do we need it? What happens on the first run vs subsequent runs? Where does session data get stored? How does this speed up my workflow when iterating on video specifications?"

**🛠️ Troubleshoot Configuration:**
> "Help me verify my Playwright MCP configuration is correct. I'll show you the contents of my ~/.claude/claude_desktop_config.json file. Check for: valid JSON syntax, correct server name ('playwright'), proper npm command format. Suggest any fixes needed."

**🚀 Apply to Video Generation:**
> "I'm about to start building AI-generated videos using Spec-Kit Plus + Playwright MCP. Based on my installation, outline exactly what happens when I run `/sp.specify` for video generation: What tools will Claude use? How will Playwright enable browser automation? Where does session persistence fit in? Give me the mental model of the complete workflow."

---
