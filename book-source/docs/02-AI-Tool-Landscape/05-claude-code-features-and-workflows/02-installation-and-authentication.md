---
sidebar_position: 2
title: "Installing and Authenticating Claude Code"
duration: "25-30 min"
---

# Installing and Authenticating Claude Code

## From Concept to Reality: Getting Claude Code Running

In Lesson 1, you learned why Claude Code is revolutionary. Now comes the crucial step: **getting it working on your machine.**

This isn't just about following installation commands. It's about crossing the bridge from "interesting concept" to "tool I can actually use." By the end of this lesson, Claude Code will be installed, authenticated, and ready to assist with your development work.

We've designed this lesson to achieve a **95% first-attempt success rate**—meaning you should be up and running without needing external help. We'll cover Windows, macOS, and Linux with multiple installation methods, clear authentication paths, and comprehensive troubleshooting for common issues.

Let's get started.

---

## Prerequisites: What You Need Before Installing

Check these before you begin:

### 1. Terminal Access

- **Windows**: Command Prompt, PowerShell, or Windows Terminal
  - Press `Win + R`, type `cmd`, press Enter
- **macOS**: Terminal app
  - Press `Cmd + Space`, type "Terminal", press Enter
- **Linux**: Any terminal emulator
  - Usually `Ctrl + Alt + T`

### 2. Node.js 18+ (Required)

**Check if you have it**:
```bash
node --version
```

**Expected output**: `v18.x.x` or higher

**If not installed**:
- Download from: https://nodejs.org (choose LTS version)
- **Windows**: Run the .msi installer
- **macOS**: Run the .pkg installer OR use `brew install node`
- **Linux**: Use package manager (`sudo apt install nodejs` or equivalent)

**Verify npm is also installed**:
```bash
npm --version
```

### 3. Claude Account (Choose One)

- **Option A**: Claude.ai account (free or Pro)
  - Sign up: https://claude.ai
  - Most common for individual users
- **Option B**: Claude Console account (API credits)
  - Sign up: https://console.anthropic.com
  - Requires payment method

### 4. Internet Connection

- Needed for installation and authentication
- Claude Code requires connection to communicate with Claude AI

---

## Installation: Choose Your Method

**Recommended**: NPM installation (works on all platforms)

### Method 1: NPM Installation (All Platforms)

**Step 1: Install Claude Code Globally**

Open your terminal and run:

```bash
npm install -g @anthropic-ai/claude-code
```

**What this does**: Downloads and installs Claude Code globally, making it accessible from any directory.

**Common issue on macOS/Linux**: Permission errors

**If you see `EACCES` permission error**:
```bash
sudo npm install -g @anthropic-ai/claude-code
```

**Step 2: Verify Installation**

```bash
claude --version
```

**Expected output** (version number may vary):
```
2.0.30 (Claude Code)
```

✅ **If you see the version number**: Installation successful! Skip to [Authentication](#authentication-connecting-claude-code-to-your-account).

❌ **If you see "command not found"**: Continue to [Troubleshooting](#troubleshooting-installation-issues) below.

---

### Method 2: Shell Script (macOS/Linux Only)

**Alternative installation using curl**:

```bash
curl -fsSL https://claude.ai/install.sh | bash
```

**What this does**: Downloads and runs installation script

**After installation, verify**:
```bash
claude --version
```

---

### Platform-Specific Notes

**Windows**:
- Use PowerShell or Windows Terminal (recommended over old Command Prompt)
- May need to restart terminal after installation
- If "command not found" persists, close and reopen terminal

**macOS**:
- May need to grant terminal permissions in System Preferences → Security & Privacy
- If using Homebrew for Node.js, ensure `/opt/homebrew/bin` is in your PATH

**Linux**:
- May need `sudo` for npm global install
- Check that `~/.npm-global/bin` or `/usr/local/bin` is in PATH
- Restart terminal after installation

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

---

### Authentication Method A: Claude.ai Account (Most Common)

**Step 1: Start the Authentication Flow**

In your terminal, run:

```bash
claude
```

**Expected output**:
```
____
```

**What happens**: Your default browser opens to the Claude.ai authentication page.

**Step 2: Log In to Claude.ai**

1. If not already logged in, enter your Claude.ai credentials
2. Review the permissions Claude Code is requesting
3. Click "Allow" or "Authorize"

**Step 3: Confirm Authentication**

Return to your terminal. You should see:

```
✓ Successfully authenticated with Claude.ai
✓ API key stored securely
You're ready to use Claude Code!
```

**Step 4: Test Your Setup**

Run a simple test command:

```bash
claude "Hello! Can you confirm Claude Code is working?"
```

**Expected output**: Claude responds with a greeting confirming the connection works.

---

## Troubleshooting: Installation Issues

### Issue 1: "command not found: claude" (After Installation)

**Cause**: npm global binaries not in PATH

**Solution (macOS/Linux)**:
```bash
export PATH="$(npm config get prefix)/bin:$PATH"
```

**Solution (Windows - PowerShell)**:
```powershell
$env:Path += ";$(npm config get prefix)"
```

**Permanent fix**: Add to shell profile (`.bashrc`, `.zshrc`, or PowerShell profile)

**Verify**:
```bash
claude --version
```

---

### Issue 2: "EACCES: permission denied" (npm install)

**Cause**: npm trying to write to system directory without permissions

**Solution 1 (Quick - use sudo)**:
```bash
sudo npm install -g @anthropic-ai/claude-code
```

**Solution 2 (Better - change npm global directory)**:
```bash
mkdir ~/.npm-global
npm config set prefix '~/.npm-global'
export PATH=~/.npm-global/bin:$PATH
npm install -g @anthropic-ai/claude-code
```

---

### Issue 3: "node: not found" (Node.js not installed)

**Cause**: Node.js not installed or not in PATH

**Diagnostic command**:
```bash
which node
node --version
```

**Solution**: Install Node.js
- Download from: https://nodejs.org
- Verify after install: `node --version`

---

### Issue 4: npm install fails with network error

**Cause**: Network/firewall blocking npm registry

**Diagnostic**:
```bash
npm ping
```

**Solutions**:
1. Check internet connection
2. Try different network (not corporate VPN)
3. Configure npm proxy if behind firewall:
```bash
npm config set proxy http://proxy.company.com:8080
npm config set https-proxy http://proxy.company.com:8080
```

---

### Issue 5: Installation succeeds but claude command behaves strangely

**Cause**: Multiple versions or conflicting installations

**Diagnostic**:
```bash
which claude
npm list -g @anthropic-ai/claude-code
```

**Solution**: Clean reinstall
```bash
npm uninstall -g @anthropic-ai/claude-code
npm cache clean --force
npm install -g @anthropic-ai/claude-code
```

---

### Issue 6: "Unsupported platform" error

**Cause**: Old Node.js version or incompatible OS

**Diagnostic**:
```bash
node --version
```

**Solution**:
- Ensure Node.js 18+ (`node --version` should show v18 or higher)
- Update Node.js if needed
- Check OS compatibility (Windows 10+, macOS 10.15+, recent Linux)

---

**Still stuck?**
- Official docs: https://docs.claude.com/en/docs/claude-code/troubleshooting
- Check GitHub issues: https://github.com/anthropics/claude-code/issues
- Ask your AI pair programmer: "I'm getting this error during Claude Code installation: [paste error]. How do I fix it?"

---

## Pause and Reflect: You've Crossed the Bridge

**If your installation and authentication succeeded**: Take a moment to appreciate what you've accomplished. Claude Code is now installed and ready to assist you. You've moved from theory to practice.

**Reflection Questions**:
- Which installation method did you choose, and why?
- Did you encounter any issues? If so, which troubleshooting steps helped?
- How does it feel to have an AI assistant accessible directly from your terminal?

**If you're still troubleshooting**: Don't get discouraged. Installation challenges are normal, especially across different platforms and environments. Work through the troubleshooting section systematically, and don't hesitate to seek help from the community resources listed above.

---

## Your First Task: See Claude Code in Action

**Congratulations!** 🎉 Claude Code is installed and authenticated. Now let's use it for something real.

**Goal**: Complete a simple coding task to see immediate value
**Time**: 5-10 minutes

---

### Option A: Debug a Python Script (If you know Python)

**Step 1: Create a sample file**

Create a file called `calculator.py` with this intentionally buggy code:

```python
# calculator.py - Simple calculator with a bug

def add_numbers(a, b):
    return a + b

def multiply_numbers(a, b):
    return a * b

def divide_numbers(a, b):
    return a / b  # Bug: no zero check!

# Test the calculator
print("10 + 5 =", add_numbers(10, 5))
print("10 * 5 =", multiply_numbers(10, 5))
print("10 / 5 =", divide_numbers(10, 5))
print("10 / 0 =", divide_numbers(10, 0))  # This will crash!
```

**Step 2: Ask Claude Code to find and fix the bug**

```bash
claude "Review calculator.py and fix any bugs. Explain what was wrong."
```

**What Claude Code will do**:
1. Read the file
2. Identify the division-by-zero bug
3. Explain the issue
4. Offer to fix it with proper error handling

**Expected outcome**: Bug fixed with zero-check added

---

### Option B: Generate Documentation (If you prefer JavaScript)

**Step 1: Create a sample file**

Create `api.js`:

```javascript
// api.js - Simple API functions (no docs)

function fetchUser(userId) {
  return fetch(`https://api.example.com/users/${userId}`)
    .then(res => res.json());
}

function createPost(title, content, authorId) {
  return fetch('https://api.example.com/posts', {
    method: 'POST',
    body: JSON.stringify({ title, content, authorId })
  }).then(res => res.json());
}

function deletePost(postId) {
  return fetch(`https://api.example.com/posts/${postId}`, {
    method: 'DELETE'
  });
}
```

**Step 2: Ask Claude Code to generate documentation**

```bash
claude "Add JSDoc comments to all functions in api.js. Include parameter types and return values."
```

**Expected outcome**: Professional documentation added to all functions

---

### Option C: Just Explore (If you're still learning)

**Ask Claude Code to introduce itself**:

```bash
claude "I just installed you! Give me 3 simple tasks I can ask you to do right now to understand what you're capable of. Make them beginner-friendly."
```

**Expected outcome**: Claude suggests 3 safe, simple tasks you can try

---

### What You've Accomplished

✅ **Installation complete**: Claude Code is installed on your system
✅ **Authentication working**: You're connected to Claude AI
✅ **First task successful**: You've seen Claude Code in action
✅ **Immediate value**: You've solved a real problem (or generated real code)

**This took about 30 minutes total. You've crossed the bridge from "interesting concept" to "tool I can use."**

---

## Setting Up CLAUDE.md (Project Memory)

**What is CLAUDE.md?**

CLAUDE.md is a special file that acts as your project's "memory" for Claude Code. It contains:
- Project overview and purpose
- Coding standards and conventions
- Common tasks and how to execute them
- Important context Claude should remember

**Think of it as**: Instructions you'd give a new team member joining your project.

---

### Creating Your First CLAUDE.md

**Step 1: Create the file in your project root**

```bash
touch CLAUDE.md  # macOS/Linux
```

Or on Windows:
```bash
echo. > CLAUDE.md
```

**Step 2: Add basic project information**

Open CLAUDE.md and add this template:

```markdown
# Project: [Your Project Name]

## Overview
[One paragraph: What does this project do?]

## Purpose
[Why does this project exist? What problem does it solve?]

## Tech Stack
- Language: [Python/JavaScript/etc.]
- Framework: [if applicable]
- Key Dependencies: [list 3-5 main dependencies]

## Coding Standards
- [Example: "Use descriptive variable names"]
- [Example: "Add docstrings to all functions"]
- [Example: "Follow PEP 8 for Python" or "Use ESLint rules for JavaScript"]

## Common Tasks

### Run the project
```
[command to run the project]
```

### Run tests
```
[command to run tests]
```

### Install dependencies
```
[command to install dependencies]
```

## Important Notes
- [Any specific context Claude should remember]
- [Example: "Always use async/await, never callbacks"]
- [Example: "Database migrations live in /migrations folder"]
```

**Step 3: Tell Claude Code to read it**

```bash
claude "Read CLAUDE.md and summarize what you learned about this project."
```

**Expected outcome**: Claude confirms it understands your project context

---

### Why CLAUDE.md Matters

**Without CLAUDE.md**: You repeat project context in every conversation
- "This is a Python project using Flask..."
- "We follow PEP 8..."
- "Tests are in the /tests folder..."

**With CLAUDE.md**: Claude remembers automatically
- You: "Add tests for the new feature"
- Claude: [already knows testing framework, where tests go, coding standards]

**Time saved**: 2-5 minutes per Claude Code session

---

### When to Update CLAUDE.md

Update it when:
- ✅ You add a new major dependency
- ✅ Coding standards change
- ✅ Project structure changes
- ✅ You notice Claude repeatedly asking for the same context

**CLAUDE.md is iterative**: Start simple, add details as needed. It grows with your project.

---

## Verification Checklist

**Your installation is complete when:**

- ✓ `claude --version` shows a version number
- ✓ `claude "Hello"` gets a response from Claude AI
- ✓ You completed "Your First Task" successfully
- ✓ CLAUDE.md exists in your project (or you know how to create it)
- ✓ You understand the security boundaries (approval gates, diff review)
- ✓ You know where to find troubleshooting help

**If all checked**: You're ready to move to Lesson 3 (Core Commands)!

**If any unchecked**: Review the relevant section above or ask for help in the Try With AI section below.

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

### Prompt 1: Installation Troubleshooting

```
I'm trying to install Claude Code on [Windows / macOS / Linux] and I'm stuck at [describe where you're stuck]. Walk me through troubleshooting step-by-step. What should I check FIRST? What's the most common cause of this issue? Give me 3-5 diagnostic commands I can run to figure out what's wrong.
```

**Expected outcome:** Step-by-step troubleshooting guidance for installation issues

### Prompt 2: Authentication Setup

```
I successfully installed Claude Code but I'm confused about authentication. I have [Claude.ai account / Claude Console account / both / neither]. Which authentication method should I use? Walk me through the exact steps, including where to find my credentials and what to paste where. Be very specific.
```

**Expected outcome:** Crystal-clear authentication instructions for your specific situation

### Prompt 3: Security Boundaries

```
The lesson mentions 'security considerations' like file access and command execution. I'm nervous about this. Help me set up safe boundaries: What directories should I AVOID running Claude Code in? What commands should I NEVER approve? Create a 'safety checklist' I can follow until I'm more comfortable.
```

**Expected outcome:** Practical safety boundaries and approval criteria

### Prompt 4: First Test Commands

```
I completed installation successfully! Now I want to test it with a simple, safe first command. Give me 3-5 'Hello World' style prompts I can try RIGHT NOW that will: (a) show me Claude Code works, (b) won't break anything, (c) help me understand what it can do. Include expected outputs so I know if it's working correctly.
```

**Expected outcome:** Confidence-building first commands with expected results
