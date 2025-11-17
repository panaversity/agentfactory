---
sidebar_position: 2
title: "Essential Setup"
description: "Install Git, create GitHub account, configure with AI help"
duration_minutes: 20

# HIDDEN SKILLS METADATA
skills:
  - name: "Install Git with AI Guidance"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can install Git with AI help"

  - name: "Create GitHub Account"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student can create GitHub account"

  - name: "Configure Git Identity"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Digital Competence"
    measurable_at_this_level: "Student can set git username and email"

learning_objectives:
  - objective: "Install Git with AI guidance"
    proficiency_level: "A1"
    bloom_level: "Remember"
    assessment_method: "Git works on their computer"

  - objective: "Create GitHub account with AI help"
    proficiency_level: "A1"
    bloom_level: "Remember"
    assessment_method: "Can log into GitHub"

  - objective: "Configure Git identity"
    proficiency_level: "A1"
    bloom_level: "Remember"
    assessment_method: "Git knows their name and email"

cognitive_load:
  new_concepts: 3
  assessment: "3 concepts (install, account, config) within A1 limit ✓"

# Generation metadata
generated_by: "content-implementer"
source_spec: "specs/012-chapter-8-git-github-aidd/plan.md"
created: "2025-11-05"
last_modified: "2025-11-07"
version: "4.0.0"
---

# Essential Setup

Before using Git, you need:

1. **Git installed** on your computer
2. **GitHub account** for cloud backup
3. **Git configured** with your identity

This takes 20 minutes. Do it once, use forever.

---

## Step 1: Install Git

### Windows

1. Download: https://git-scm.com/download/win
2. Run the 64-bit installer
3. Click "Next" through all screens (defaults work fine)
4. Click "Finish"

**Verify installation**:

**You ask Gemini CLI**: "Check if Git is installed and show the version"

Gemini runs: `git --version`

You should see: `git version 2.42.0` (or similar)

✓ Version number means Git is installed.

---

**Troubleshooting**:

**Permission denied error?**
Right-click installer → "Run as Administrator"

**"Git not recognized" after install?**
**Ask Gemini CLI**: "Git command not found on Windows. Help me fix the PATH."

---

### Mac

1. Download: https://git-scm.com/download/mac
2. Run the installer
3. Follow the prompts
4. Enter your password when asked

**Verify**: Open Terminal, type `git --version`

**If errors occur**:
**Ask Gemini CLI**: "Can't install Git on Mac. Here's my error: [paste error]"

---

### Linux

Installation varies by distribution.

**You ask Gemini CLI**: "What command installs Git on [your distribution]?"

Examples:
- Ubuntu/Debian: `sudo apt install git`
- Fedora: `sudo dnf install git`
- Arch: `sudo pacman -S git`

**Verify**: `git --version`

---

## Step 2: Create GitHub Account

GitHub stores your code in the cloud.

### Create Account

1. Go to: https://github.com
2. Click "Sign up" (top right)
3. Provide:
   - Email address
   - Password (make it strong)
   - Username (public - choose carefully)
4. Verify email (check inbox/spam)
5. Choose **Free** plan

Your profile: `github.com/your-username`

---

**Common Issues**:

**Username taken?**
Try: `yourname-dev`, `yourname2024`, or `your-name`

**No verification email?**
Check spam, or click "Resend verification email"

**Too many questions?**
Skip surveys - just get to your dashboard

---

## Step 3: Configure Git

Git needs to know who you are. Every commit will show your name.

**You ask Gemini CLI**: "Configure Git with my name 'Your Name' and email 'your.email@example.com'"

Gemini runs:
- `git config --global user.name "Your Name"`
- `git config --global user.email "your.email@example.com"`

Sets up your identity. Use your real name and the same email as your GitHub account.

**Verify configuration**:

**You ask Gemini CLI**: "Show me my Git configuration"

Gemini runs: `git config --list`

You should see your name and email listed.

---

**Why this matters**:

Your name stamps every code change you save. This:
- Shows who made each change
- Builds your contribution history
- Links commits to your GitHub profile

**Use the same email** as GitHub so commits appear on your profile.

---

#### 💬 AI Colearning Prompt
> "Why does Git need my name and email for every commit? What happens if I skip this configuration?"

---

## Verification Checklist

**You ask Gemini CLI**: "Verify my Git setup: 1) Is Git installed? 2) Show my config 3) Can I access GitHub?"

Gemini checks:
- ✓ Git installed (shows version)
- ✓ Name and email configured
- ✓ GitHub account exists

If anything fails, ask Gemini to fix it.

---

## What You Learned

**When to use direct methods**:
- Downloading installers
- Clicking through setup wizards
- Creating online accounts

**When to use AI (Gemini CLI)**:
- Verifying installation worked
- Troubleshooting errors
- Platform-specific commands
- Configuration setup

This is the professional pattern: handle simple tasks directly, use AI when complexity appears.

---

#### 🎓 Expert Insight
> In AI-native development, you don't memorize installation steps for every platform—you understand what needs installing (Git, GitHub account, configuration). When you hit platform-specific complexity or errors, AI becomes your troubleshooting partner. Your job: recognize when you need help, not memorize every error code.

---

#### 🤝 Practice Exercise

> **Ask your AI**: "Verify my complete Git setup: check installation, show my configuration, and test GitHub connectivity. Then explain what each verification step checks for."

**Expected Outcome**: You'll understand not just IF Git works, but WHY each verification step matters (version confirms install, config shows identity, connectivity tests GitHub access).

---

## Try With AI

Verify your setup.

**Tool**: Gemini CLI (or Claude Code, ChatGPT)

### Exercise 1: Complete System Check

```
Run a complete Git setup verification:
1. Check Git is installed
2. Show my Git configuration
3. Confirm GitHub account works
4. Report any issues
```

### Exercise 2: Update Configuration

```
Change my Git email to [new-email].
Update configuration and verify the change worked.
```

### Exercise 3: Troubleshoot Issues

If you had problems:

```
I had trouble with [describe issue].
My system: [Windows/Mac/Linux]
Help me resolve this and verify Git works.
```
