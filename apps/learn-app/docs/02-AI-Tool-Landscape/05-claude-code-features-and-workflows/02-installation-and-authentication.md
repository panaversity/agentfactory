---
title: "Installing and Authenticating Claude Code"
sidebar_position: 2
chapter: 5
lesson: 2
duration_minutes: 20

# PEDAGOGICAL LAYER METADATA
primary_layer: "Layer 1"
layer_progression: "L1 (Manual Foundation)"
layer_1_foundation: "Terminal-based AI tool installation, authentication workflows, command execution verification"
layer_2_collaboration: "N/A"
layer_3_intelligence: "N/A"
layer_4_capstone: "N/A"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Claude Code Installation and Authentication"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can evaluate multiple installation methods (Homebrew, npm, pipx, direct download), select appropriate method for their OS, execute installation commands, authenticate with Claude.ai or Console API, and verify working installation"

learning_objectives:
  - objective: "Choose appropriate Claude Code installation method for your operating system"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Selection of installation method with justification based on OS and existing tools"
  - objective: "Install Claude Code successfully using one of four installation methods"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successful execution of installation commands with troubleshooting if needed"
  - objective: "Authenticate with either Claude.ai subscription or Console API account"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Completion of authentication workflow and API key configuration"
  - objective: "Verify Claude Code installation and authentication are working correctly"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Execution of `claude --version` and successful first session"
  - objective: "Understand security best practices for file access and command execution"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of sandbox mode, file access patterns, and command approval workflows"

# Cognitive load tracking
cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (4 installation methods, authentication types, API keys, terminal commands, security/sandboxing, verification) - within B1 limit of 10 ✓ (Re-staged from A2 to B1 due to concept count and intermediate complexity)"

# Differentiation guidance
differentiation:
  extension_for_advanced: "Set up multiple Claude Code installations with different API keys for different projects, configure custom shells (zsh/bash aliases), explore advanced authentication patterns"
  remedial_for_struggling: "Focus on single installation method (Homebrew for macOS, npm for Windows/Linux), use Claude.ai authentication (simpler than Console API)"

# Generation metadata
generated_by: "content-implementer v1.0.0 (029-chapter-5-refinement)"
source_spec: "specs/034-lesson2-install-update/spec.md"
created: "2025-01-17"
last_modified: "2025-12-06"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.1.0"

# Legacy compatibility (Docusaurus)
prerequisites:
  - "Terminal access (Windows/macOS/Linux)"
  - "Claude account (Claude.ai or Console API)"
---

# Installing and Authenticating Claude Code

In Lesson 1, you learned why Claude Code is revolutionary. Now comes the crucial step: **getting it working on your machine.**

This isn't just about following installation commands. It's about crossing the bridge from "interesting concept" to "tool I can actually use." By the end of this lesson, Claude Code will be installed, authenticated, and ready to assist with your development work.

---

## 🔀 Two Professional Paths Available

This lesson covers the **official Claude subscription setup** (Pro $20/month or Max $200/month). If you prefer a **free/minimal cost alternative**, see **Lesson 3: Free Claude Code Setup with Google Gemini**.

**Both paths teach identical Claude Code skills** (subagents, skills, MCP, hooks covered in Lessons 4-10).

| Path | Cost | What You Get |
|------|------|--------------|
| **This Lesson (Official)** | Pro: $20/month<br/>Max: $200/month | Official Anthropic models (Claude Sonnet/Opus), direct integration, official support |
| **Lesson 3 (Free/Minimal)** | $0 (free tier)<br/>Pay-as-you-go option | Use Claude Code agent with **any LLM** (Gemini, GPT, local models), backend abstraction architecture |

**Key difference**: Official path uses Anthropic's Claude models. Free path lets you use Claude Code's agentic architecture with any LLM backend through production-grade API routing.

---

---

## Why This Matters: Terminal Integration for AI Workflows

Terminal-based AI integration changes how you work with AI assistance. Unlike web-based tools where you describe code problems and manually copy-paste solutions, terminal-integrated AI sees your actual project context—files, dependencies, structure—and proposes changes to your real codebase.

**Workflow Impact**: You stay in your development environment (terminal + editor) instead of context-switching to a browser. Claude Code reads files, proposes diffs, executes commands, and sees results—all within the same workspace where you code. This removes the friction between intent (what you want) and execution (making it happen).

**Paradigm Connection**: This is the agentic difference from Lesson 1 in action. Web-based AI is a consultant you visit for advice. Terminal-integrated AI is a pair programmer actively collaborating in your workspace. The terminal access enables the paradigm shift from passive assistance to active collaboration.

**Real-World Context**: You'll use this terminal integration for automated file operations (creating, editing, refactoring), git workflows (commits, branches, PRs), and testing integration (run tests, see results, iterate). Installation isn't just setup—it's enabling a fundamentally different development workflow.

---

## Prerequisites: What You Need Before Installing

Before we begin, verify you have the following:

**1. Terminal Access**
- **Windows**: Command Prompt, PowerShell, or Windows Terminal
- **macOS**: Terminal app (Applications → Utilities → Terminal)
- **Linux**: Any terminal emulator (GNOME Terminal, Konsole, etc.)
- **WSL Users**: Any WSL 2 terminal with Ubuntu 20.04+ or Debian 10+

**2. Claude Account** (one of the following):
- **Option A**: Claude.ai subscription (Pro $20/month, Max $200/month, or Enterprise) - Sign up at: https://claude.ai
- **Option B**: Claude Console account with API credits - Create account at: https://console.anthropic.com
- **📍 Location**: Claude Code requires authentication from [Anthropic-supported countries](https://www.anthropic.com/supported-countries). Check availability during signup.

**3. System Requirements**
- **macOS**: 10.15 (Catalina) or later
- **Windows**: Windows 10 or later
- **Linux**: Ubuntu 20.04+ / Debian 10+
- **RAM**: 4GB minimum

**4. Optional (for npm installation only)**
- **Node.js**: Version 18 or later (only required if using npm installation method)

---

## Installation

Claude Code installation has been simplified with official installers for each platform. Choose your operating system below to see platform-specific installation methods.

![Installation flowchart showing Node.js check, npx/npm install options, authentication with API key or web login, and verification steps with decision points and success/error paths](/img/part-2/chapter-05/claude-code-installation-authentication-flow.png)

---

### Windows Installation

**⚠️ Important**: Claude Code requires a bash-compatible shell to run. On Windows, you need **either WSL or Git for Windows** installed—the installer downloads Claude Code, but it runs inside a bash shell.

**Decision Tree**:
```
Which shell environment do you have (or want to install)?
├─ WSL (Windows Subsystem for Linux)
│   └─ Method 1 (WSL) - RECOMMENDED for best experience
│
├─ Git for Windows (Git Bash)
│   └─ Method 2 (Git Bash + PowerShell installer)
│
├─ Neither installed yet
│   └─ Install WSL first (see below), then use Method 1
│
└─ I have Node.js 18+ in WSL or Git Bash
    └─ Method 3 (npm) - See Cross-Platform npm section below
```

#### Method 1: WSL (RECOMMENDED)

**Why recommended**: Full Linux environment, best compatibility, recommended by Anthropic.

**Step 1**: If you don't have WSL, install it first (run in PowerShell as Administrator):

```powershell
wsl --install
```

Restart your computer after installation.

**Step 2**: Open your WSL terminal (Ubuntu) and run:

```bash
curl -fsSL https://claude.ai/install.sh | bash
```

**What this does**: Downloads and executes the official installer script in your Linux environment.

**Requirements**: Windows 10 version 2004+ or Windows 11, WSL 1 or WSL 2

#### Method 2: Git Bash

**When to use**: You prefer Git for Windows over WSL, or WSL isn't available on your system.

**Step 1**: Install [Git for Windows](https://git-scm.com/downloads/win) if not already installed.

**Step 2**: Install Claude Code using PowerShell:

```powershell
irm https://claude.ai/install.ps1 | iex
```

**Step 3**: Configure Claude Code to use Git Bash (run in PowerShell):

```powershell
$env:CLAUDE_CODE_GIT_BASH_PATH="C:\Program Files\Git\bin\bash.exe"
```

To make this permanent, add the line to your PowerShell profile:

```powershell
notepad $PROFILE
# Add the line: $env:CLAUDE_CODE_GIT_BASH_PATH="C:\Program Files\Git\bin\bash.exe"
```

**Step 4**: Run Claude Code from Git Bash or PowerShell:

```bash
claude
```

**What this does**: The PowerShell command installs Claude Code; the environment variable tells it where to find bash.

**Requirements**: Git for Windows, PowerShell 5.1+

#### Windows Verification

Open your shell (WSL terminal or Git Bash) and check your installation:

```bash
claude --version
```

**Expected output** (version number may vary):
```
2.0.37 (Claude Code)
```

#### 💬 AI Colearning Prompt
> "Explain the difference between WSL and Git Bash for Windows developers. When would you choose one over the other for AI-native development workflows?"

---

### macOS Installation

**Decision Tree**:
```
Do you have Homebrew installed?
├─ Yes → Method 1 (Homebrew) - RECOMMENDED
│
├─ No, and I don't want to install Homebrew
│   └─ Method 2 (curl/bash)
│
└─ I have Node.js 18+
    └─ Method 3 (npm) - See Cross-Platform npm section below
```

#### Method 1: Homebrew (RECOMMENDED)

**Why recommended**: Standard macOS package manager, handles updates automatically, integrated with macOS ecosystem.

Open Terminal and run:

```bash
brew install --cask claude-code
```

**What this does**: Installs Claude Code using Homebrew's cask system (for GUI/binary applications). Installs to `/Applications`.

**Requirements**: Homebrew installed

**Don't have Homebrew?** Install it first:

```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

#### Method 2: curl/bash

**When to use**: You don't have Homebrew and prefer not to install it.

Open Terminal and run:

```bash
curl -fsSL https://claude.ai/install.sh | bash
```

**What this does**: Downloads and executes the official installer script.

**Requirements**: curl and bash (pre-installed on macOS)

#### macOS Verification

Check your installation:

```bash
claude --version
```

**Expected output** (version number may vary):
```
2.0.37 (Claude Code)
```

#### 🎓 Expert Insight
> Package managers like Homebrew transform tool management. Instead of manually downloading, installing, and updating dozens of developer tools, `brew upgrade` updates everything. This is why 90%+ of professional macOS developers use Homebrew.

---

### Linux/WSL Installation

**Decision Tree**:
```
Are you on Ubuntu/Debian/WSL?
├─ Yes → Method 1 (curl/bash) - RECOMMENDED
│
├─ I'm on Alpine Linux
│   └─ See Alpine Linux Special Configuration below
│
└─ I have Node.js 18+
    └─ Method 2 (npm) - See Cross-Platform npm section below
```

#### Method 1: curl/bash (RECOMMENDED)

**Why recommended**: Universal, minimal dependencies, works on all major distributions.

Open your terminal and run:

```bash
curl -fsSL https://claude.ai/install.sh | bash
```

**What this does**: Downloads and executes the official installer script, automatically detecting your distribution.

**Requirements**: 
- Ubuntu 20.04+ or Debian 10+
- curl and bash (pre-installed on most distributions)

**Supported Distributions**:
- Ubuntu 20.04, 22.04, 24.04
- Debian 10, 11, 12
- WSL 2 (with Ubuntu or Debian)

#### Alpine Linux Special Configuration

Alpine Linux requires additional C++ runtime libraries:

```bash
apk add libgcc libstdc++ ripgrep
export USE_BUILTIN_RIPGREP=0
```

**What this does**: 
1. Installs required libraries (libgcc, libstdc++) and ripgrep
2. Configures Claude Code to use system ripgrep instead of built-in version

**Make it permanent**: Add `export USE_BUILTIN_RIPGREP=0` to your shell profile (`~/.bashrc` or `~/.zshrc`).

#### Linux/WSL Verification

Check your installation:

```bash
claude --version
```

**Expected output** (version number may vary):
```
2.0.37 (Claude Code)
```

---

### Cross-Platform: npm Installation

**When to use this method**:
- You already have Node.js 18+ installed
- You prefer npm-based workflows
- You need to manage Claude Code versions via package.json
- Your platform isn't officially supported by platform-specific installers

**Platforms**: Windows, macOS, Linux, WSL

Open your terminal and run:

```bash
npm install -g @anthropic-ai/claude-code
```

**What this does**: Installs Claude Code globally via npm package manager.

**Requirements**: Node.js 18 or later (includes npm)

**Check Node.js version**:

```bash
node --version
```

If you see `v18.0.0` or higher, you're good to go.

#### 💬 AI Colearning Prompt
> "Explain the trade-offs between platform-specific installers (Homebrew, PowerShell) vs npm. When would a developer choose npm over the platform installer?"

---

### Auto-Update Configuration

Claude Code automatically checks for updates and prompts you to install them. To disable auto-updates (useful for corporate environments or version pinning):

**macOS/Linux/WSL**:

```bash
export DISABLE_AUTOUPDATER=1
```

Add to your shell profile (`~/.bashrc`, `~/.zshrc`, etc.) to make permanent.

**Windows PowerShell**:

```powershell
$env:DISABLE_AUTOUPDATER=1
```

Add to your PowerShell profile (`$PROFILE`) to make permanent.

**Manual update check**:

```bash
claude update
```

---

### Advanced: System Diagnostics

After installation, verify your system configuration:

```bash
claude doctor
```

**What this checks**:
- Installation integrity
- Authentication status
- System compatibility
- Network connectivity to Claude API

Use this command if you encounter issues during installation or authentication.

#### 🎓 Expert Insight
> In AI-native development, terminal comfort is a skill multiplier. The 5 minutes you invest learning basic terminal commands unlocks 10x productivity with AI tools. You're not becoming a "terminal expert"—you're removing the friction between intent and execution.

---
## Authentication: Connecting Claude Code to Your Account

Once installed, Claude Code needs to authenticate with your Claude account. There are **three authentication paths** depending on your account type and use case.

### Which Authentication Method Should I Use?

**Decision Tree**:

```
What type of Claude access do you have?
├─ Claude.ai subscription (Pro, Max, Team)
│   └─ Method 1: Claude App Authentication (MOST COMMON)
│
├─ Claude Console account with API credits
│   └─ Method 2: Console API Authentication
│
└─ Enterprise account (Bedrock, Vertex AI, Foundry)
    └─ Method 3: Enterprise Authentication
```

**If you have both subscription and Console API**: Use Method 1 (Claude App)—it's simpler and provides unified access.

#### 🎓 Expert Insight
> In AI-native development, authentication isn't just about access—it's about resource management. Claude.ai (subscription) vs Console API (pay-per-use) vs Enterprise (dedicated capacity) represents different cost models and usage patterns. Understanding your workflow determines which path saves money.

---

### Method 1: Claude App Authentication (Most Common)

**Who this is for**: Users with Claude Pro ($20/month), Claude Max ($200/month), or Team subscriptions.

**Benefits**: Unified access across Claude web app and Claude Code, simpler authentication flow.

In your terminal, run:

```bash
claude
```

**Expected output**:
```
 Claude Code can be used with your Claude subscription or billed based on API usage through your 
 Console account.

 Select login method:

 ❯ 1. Claude account with subscription · Pro, Max, Team, or Enterprise

   2. Anthropic Console account · API usage billing
```

Select Option 1. Your default browser opens to Claude.ai authentication. Log in, review permissions, and authorize.

Return to your terminal. You should see:

```
Logged in as mr.abc@gmail.com
Login successful. Press Enter to continue
```

Test your setup:

```bash
claude "Hello! Can you confirm Claude Code is working?"
```

**Expected output**: Claude responds confirming the connection works.

#### 🤝 Practice Exercise

> **Ask your AI**: "I just installed Claude Code. Create a simple 'Hello World' workflow that: (a) shows me Claude can read a file, (b) proposes a small change, (c) explains what it did. Use a safe test file."

**Expected Outcome**: Confidence that Claude Code can read, propose changes, and explain actions—plus understanding of the approval workflow.

---

### Method 2: Console API Authentication (Developers)

**Who this is for**: Developers with Claude Console API credits but no Claude.ai subscription. Pay-per-use model based on token consumption.

**Use case**: API-first workflows, programmatic access, usage-based billing.

In your terminal, run:

```bash
claude
```

**Expected output**:
```
 Claude Code can be used with your Claude subscription or billed based on API usage through your
 Console account.

 Select login method:

   1. Claude account with subscription · Pro, Max, Team, or Enterprise

 ❯ 2. Anthropic Console account · API usage billing
```

Select Option 2. Go to https://console.anthropic.com/settings/keys, create an API key, copy it, and paste when prompted.

You should see:

```
API key validated successfully
Login successful. Press Enter to continue
```

Test your setup:

```bash
claude "Hello! Can you confirm Claude Code is working?"
```

**Expected output**: Claude responds confirming the connection works.

**⚠️ Important for Console API Users**:
- Set usage limits in Console: https://console.anthropic.com/settings/limits
- Monitor token usage (displayed after each interaction)
- Console authentication uses API billing, not subscription credits
- Consider cost management strategies for high-volume usage

---

### Method 3: Enterprise Authentication (Advanced)

**Who this is for**: Enterprise customers using Amazon Bedrock, Google Vertex AI, or Anthropic Foundry (dedicated capacity).

**Use case**: Organizations with existing cloud infrastructure, compliance requirements, or dedicated capacity needs.

**Platform Options**:

#### Amazon Bedrock Integration

Claude Code can authenticate with Claude via AWS Bedrock:

**Requirements**:
- AWS account with Bedrock access
- Claude models enabled in Bedrock
- AWS CLI configured with appropriate credentials

**Configuration**: Contact your Enterprise administrator for Bedrock configuration details specific to your organization.

#### Google Vertex AI Integration

Claude Code can authenticate with Claude via Google Cloud Vertex AI:

**Requirements**:
- Google Cloud account with Vertex AI access
- Claude models enabled in Vertex AI
- Google Cloud SDK configured

**Configuration**: Contact your Enterprise administrator for Vertex AI configuration details specific to your organization.

#### Anthropic Foundry

Claude Code can connect to dedicated Claude capacity via Anthropic Foundry:

**Requirements**:
- Anthropic Foundry account with dedicated capacity
- Enterprise API keys

**Configuration**: Contact your Anthropic Enterprise support for Foundry setup.

**📚 Enterprise Documentation**: For detailed enterprise configuration, see https://docs.anthropic.com/en/api/claude-on-amazon-bedrock or contact your Enterprise administrator.

---
## Security and Best Practices

Before moving forward, let's address important security considerations:

**1. File System Access**

- Claude Code can read and write files in directories where you run it
- **Best Practice**: Start Claude Code sessions in project directories, not system directories
- Review file changes Claude proposes before approving them

**2. Command Execution**

- Claude Code can execute terminal commands with your permissions
- **Best Practice**: Review commands Claude suggests, especially `sudo` or administrative commands
- Claude Code will ask for approval before executing destructive actions

**3. Cost Management (Console API Users)**

- Set usage limits in Claude Console: https://console.anthropic.com/settings/limits
- Monitor usage regularly to avoid unexpected bills
- Claude Code displays token usage after each interaction

---

## Try With AI

Now that Claude Code is installed, let's build confidence through safe exploration and establish good security practices.

**🛡️ Establish Safety Boundaries:**

> "I just installed Claude Code and I'm nervous about file access and command execution. Help me set up safe boundaries: What directories should I AVOID running Claude Code in? What commands should I NEVER approve without careful review? Create a practical safety checklist I can follow until I'm more comfortable."

**🎯 Practice First Commands:**

> "I completed installation successfully! Give me 3-5 safe 'Hello World' style prompts I can try RIGHT NOW that will: (a) verify Claude Code works, (b) won't modify important files, (c) demonstrate basic capabilities. Include what I should expect to see for each prompt."

**🧪 Test Your Installation:**

> "Walk me through testing my Claude Code installation step-by-step. Start with checking the version, then test file reading (on a safe test file), then test a simple command execution (like checking current directory). Explain what each output means and how to know if something's wrong."

**🚀 Configure for Your Workflow:**

> "I work primarily with [describe your tech stack: Python/JavaScript/Go/etc.]. Help me verify Claude Code can handle my environment: check for required tools, test reading my project structure, and suggest first productive task I could try that's relevant to my actual work."
