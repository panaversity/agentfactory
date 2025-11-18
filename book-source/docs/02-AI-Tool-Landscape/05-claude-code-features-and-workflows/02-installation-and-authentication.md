---
title: "Installing and Authenticating Claude Code"
sidebar_position: 2
chapter: 5
lesson: 2
duration_minutes: 18

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
source_spec: "specs/029-chapter-5-refinement/spec.md"
created: "2025-01-17"
last_modified: "2025-01-17"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"

# Legacy compatibility (Docusaurus)
prerequisites:
  - "Terminal access (Windows/macOS/Linux)"
  - "Claude account (Claude.ai or Console API)"
---

# Installing and Authenticating Claude Code

In Lesson 1, you learned why Claude Code is revolutionary. Now comes the crucial step: **getting it working on your machine.**

This isn't just about following installation commands. It's about crossing the bridge from "interesting concept" to "tool I can actually use." By the end of this lesson, Claude Code will be installed, authenticated, and ready to assist with your development work.

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

**2. Claude Account** (one of the following):
- **Option A**: Claude.ai subscription (Pro or free tier) - Sign up at: https://claude.ai
- **Option B**: Claude Console account with API credits - Create account at: https://console.anthropic.com

---

## Installation

### Step 1: Install Claude Code Globally

Claude Code offers **four installation methods**. Choose the one that matches your operating system:

#### Method 1: macOS/Linux (Recommended)

```bash
curl -fsSL https://claude.ai/install.sh | bash
```

**What this does**: Downloads and runs the official installer script, automatically detecting your system and installing Claude Code.

#### Method 2: Homebrew (macOS)

```bash
brew install --cask claude-code
```

**What this does**: Installs Claude Code using Homebrew package manager (if you already use Homebrew).

#### Method 3: Windows

```powershell
irm https://claude.ai/install.ps1 | iex
```

**What this does**: Downloads and runs the PowerShell installer script for Windows systems.

#### Method 4: npm (Cross-Platform)

```bash
npm install -g @anthropic-ai/claude-code
```

**What this does**: Installs Claude Code via npm package manager (requires Node.js 18+).

**Which method?** macOS/Linux: Method 1 (curl) or 2 (Homebrew); Windows: Method 3 (PowerShell); Node.js users: Method 4 (npm, cross-platform)

#### 💬 AI Colearning Prompt
> "Explain why package managers like Homebrew and npm are used for installing developer tools instead of manual downloads. What problem do they solve?"

#### 🎓 Expert Insight
> In AI-native development, terminal comfort is a skill multiplier. The 5 minutes you invest learning basic terminal commands unlocks 10x productivity with AI tools. You're not becoming a "terminal expert"—you're removing the friction between intent and execution.

### Step 2: Verify Installation

Check that Claude Code is installed correctly:

```bash
claude --version
```

**Expected output** (version number may vary):
```
2.0.37 (Claude Code)
```

## Authentication: Connecting Claude Code to Your Account

Once installed, Claude Code needs to authenticate with your Claude account. There are **two authentication paths** depending on which account type you have.

### Which Authentication Method Should I Use?

**Decision Tree**:

```
Do you have a Claude.ai account?
├─ Yes → Use Claude.ai Authentication (Method A)
│        Most common for individual users
│
└─ No, but I have Claude Console API credits
   └─ Use Claude Console Authentication (Method B)
           Common for developers with API access
```

**If you have both**: Use Claude.ai authentication (Method A)—it's simpler and you can switch to Console authentication later if needed.

#### 🎓 Expert Insight
> In AI-native development, authentication isn't just about access—it's about resource management. Claude.ai (subscription) vs Console API (pay-per-use) represents different cost models. Understanding your usage patterns determines which path saves money.

---

### Authentication Method A: Claude.ai Account (Most Common)

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

### Authentication Method B: Claude Console API Account (Developers)

Use this if you have Claude Console API credits but no Claude.ai subscription.

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

Use Claude Code for this activity (preferred, since you just installed it). If you already have another AI companion tool set up (e.g., ChatGPT web, Gemini CLI), you may use that instead—the prompts are the same.

### Prompt 1: Security Boundaries

```
I have installed Claude Code - can you share 'security considerations' like file access and command execution. I'm nervous about this. Help me set up safe boundaries: What directories should I AVOID running Claude Code in? What commands should I NEVER approve? Create a 'safety checklist' I can follow until I'm more comfortable.
```

**Expected outcome:** Practical safety boundaries and approval criteria

### Prompt 2: First Test Commands

```
I completed installation successfully! Now I want to test it with a simple, safe first command. Give me 3-5 'Hello World' style prompts I can try RIGHT NOW that will: (a) show me Claude Code works, (b) won't break anything, (c) help me understand what it can do. Include expected outputs so I know if it's working correctly.
```

**Expected outcome:** Confidence-building first commands with expected results
