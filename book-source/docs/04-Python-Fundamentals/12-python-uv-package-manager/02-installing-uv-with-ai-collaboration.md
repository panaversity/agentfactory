---
title: "Installing UV with AI Collaboration"
chapter: 12
lesson: 2
duration_minutes: 15

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Execute Direct Installation Commands"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can run platform-specific installation command and verify installation succeeded"

  - name: "Understand PATH Configuration"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain what PATH is (computer's command registry) and why installation adds UV to PATH"

  - name: "Troubleshoot Installation Issues with AI"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify common installation errors and work with AI to diagnose and resolve them"

learning_objectives:
  - objective: "Successfully install UV on your system (Windows/Mac/Linux) using direct commands"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Hands-on installation and verification"

  - objective: "Understand what happens during software installation and why PATH configuration matters"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Explanation of installation process"

  - objective: "Use AI for troubleshooting when installation issues occur"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Error scenario resolution with AI"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts: installation process, PATH environment variable, platform-specific differences, verification workflow, when to use AI (troubleshooting) vs. direct commands (simple tasks). Within A2 limit of 7 concepts. ✓"

differentiation:
  extension_for_advanced: "Explore manual installation from source; understand the different installation methods (curl vs. pip vs. Homebrew) and when each is appropriate"
  remedial_for_struggling: "Focus on core goal: Run the command for your platform, verify it worked. Use AI only if you encounter errors."

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/011-python-uv/plan.md"
created: "2025-01-13"
last_modified: "2025-01-13"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Installing UV with AI Collaboration

## What We're About to Do

You already understand **why** UV matters (from Lesson 1). Now let's get it installed on your computer.

Installation takes under 1 minute: you run one command, verify it worked, and you're done. Simple, deterministic tasks like this don't need AI—you just follow the direct instructions for your platform.

**Where AI becomes valuable:** When something goes wrong. If you encounter errors, AI can diagnose platform-specific issues and suggest fixes.

**In this lesson, you will:**
1. Run the installation command for your platform (macOS, Windows, or Linux)
2. Verify the installation worked
3. Understand what PATH is and why it matters
4. Learn when to use AI (troubleshooting) vs. direct commands (simple tasks)

Let's get started. You'll be done in under 5 minutes.

## Step 1: Install UV (Direct Command)

Pick your platform and run the command. That's it.

### macOS or Linux

Open your terminal and run:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**What this does:**
- Downloads the official UV installation script from Astral
- Executes it to install UV to `~/.local/bin/` (both macOS and Linux)
- Automatically adds UV to your PATH so you can use the `uv` command

**Time:** ~30 seconds

### Windows (PowerShell)

Open PowerShell and run:

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

**What this does:**
- Downloads the official UV installation script from Astral
- Executes it to install UV to `C:\Users\YourName\.local\bin`
- Automatically adds UV to your PATH
- **Important:** Restart PowerShell after installation

**Time:** ~30 seconds + PowerShell restart

## Step 2: Verify Installation

Close your terminal completely and open a new one. Then run:

```bash
uv --version
```

**Expected output:**
```
uv 0.9.9
```

If you see a version number, UV is installed correctly. You're done! Skip to the "When to Use AI" section below.

### What If You See "command not found"?

If you see:
```
uv: command not found
```

**This means your PATH isn't configured yet.** Now is when AI becomes useful.

Jump to the "Troubleshooting with AI" section below.

## Understanding PATH (2-Minute Explanation)

**PATH** is your computer's "registry" of command locations. When you type `uv --version`, your computer:
1. Looks through directories listed in PATH
2. Finds the `uv` executable
3. Runs it

**Without PATH configured:**
```
uv: command not found  ← Computer can't find UV
```

**With PATH configured:**
```
uv 0.9.9  ← Computer found UV and ran it
```

The installation script automatically adds UV's directory to your PATH. That's why you can type `uv` from anywhere and it works.

**Why this matters:** If something goes wrong during installation, understanding PATH helps you troubleshoot (with AI's help).

## When to Use AI: The Key Principle

This lesson demonstrates an important pattern:

**Simple, deterministic tasks (like installation):**
- ✅ Use direct commands
- ✅ Follow platform-specific instructions
- ✅ Takes under 1 minute

**Complex, ambiguous problems (like troubleshooting):**
- ✅ Use AI to diagnose platform-specific issues
- ✅ Get context-aware fixes
- ✅ Learn WHY something broke

**Don't use AI for everything.** Use it strategically when it adds value.

## Troubleshooting with AI

If you encountered errors during installation, **now** is when AI becomes valuable. Here are common scenarios:

### Error 1: "command not found" After Installation

**Open your AI CLI tool (Claude Code, Gemini CLI, or ChatGPT) and ask:**

```
After installing UV, I'm getting "uv: command not found".
I'm using [WINDOWS/MAC/LINUX].
How do I fix this?
```

AI will diagnose:
- Whether you need to restart your shell
- Whether PATH was configured correctly
- Platform-specific PATH configuration commands

### Error 2: Permission Denied

**Ask your AI:**

```
I'm getting "Permission Denied" when trying to install UV.
I'm using [WINDOWS/MAC/LINUX].
What should I do?
```

AI will suggest:
- Using `sudo` (macOS/Linux) if needed
- Checking antivirus settings (Windows)
- Alternative installation methods

### Error 3: Network Issues

**Ask your AI:**

```
I'm getting a network error when installing UV.
Error: [paste exact error message]
What should I do?
```

AI will help:
- Diagnose connection issues
- Suggest alternative download methods
- Check if Astral's servers are down

## What You Just Learned

**The core pattern:**
- ✅ Simple tasks (installation): Use direct commands
- ✅ Complex problems (troubleshooting): Use AI for diagnosis
- ✅ Don't over-engineer simple processes with AI
- ✅ PATH makes commands available system-wide

**Key insight:** AI-driven development doesn't mean "use AI for everything." It means using AI strategically when it adds value—troubleshooting, debugging, understanding complex systems—not for simple, deterministic tasks that take 30 seconds.

This pattern will continue throughout the chapter:
- **Creating projects**: Direct command (`uv init`)
- **Adding dependencies**: Direct command (`uv add package-name`)
- **Understanding why something failed**: AI collaboration
- **Debugging complex dependency conflicts**: AI collaboration

---

## Try With AI: Installation Troubleshooting Challenge

### Part 1: Predict Installation Issues (Your Turn First)

**Before installing UV**, predict what could go wrong:

**Your analysis task**:
1. The installation script is: `curl -LsSf https://astral.sh/uv/install.sh | sh`
2. Break down this command: What does each part do? (`curl`, `-LsSf`, `|`, `sh`)
3. List 3 things that could fail (network? permissions? PATH?)
4. For YOUR operating system (Windows/Mac/Linux), what's unique about installation?

Write down your predictions BEFORE running the command.

---

### Part 2: AI Explains Installation Process (Discovery)

Now ask AI to validate your understanding:

> "I'm about to install UV with this command:
> `curl -LsSf https://astral.sh/uv/install.sh | sh`
>
> Here's what I think each part does: [your breakdown from Part 1]
>
> Questions:
> 1. Is my understanding correct?
> 2. What does this script actually modify on my system? (PATH? files?)
> 3. For [YOUR OS], what's different about the installation?
> 4. How do I verify this is the OFFICIAL installer, not malicious?"

**Your evaluation task**:
- Compare AI's explanation to your predictions. What did you miss?
- Does AI mention PATH modification? Why does that matter?
- Can you now explain to someone else: "What is PATH and why does UV need to be in it?"

---

### Part 3: Student Teaches AI (Security & Troubleshooting)

Challenge AI with edge cases:

> "I have concerns about running installation scripts from the internet:
>
> **Security Scenario 1**: What if someone compromised astral.sh and replaced the script with malware?
> - How can I verify the script is safe BEFORE running it?
> - Show me how to inspect the script content first
> - What signs indicate a malicious vs. legitimate installer?
>
> **Troubleshooting Scenario 2**: After installation, I run `uv --version` and get 'command not found'
> - Diagnose: What are the 3 most common causes?
> - For EACH cause, give me the diagnostic command to check it
> - Then the fix command
>
> **Platform-Specific Issue**: I'm on [Windows/Mac/Linux] and the installation command looks different than the docs
> - Why are installation methods OS-specific?
> - What's the equivalent for my OS?"

**Your debugging task**:
- Run the diagnostic commands AI suggests. Do they work?
- Which troubleshooting step surprised you most?

---

### Part 4: Build Installation Verification Checklist (Convergence)

Create a post-installation validation workflow with AI:

> "Let's create a checklist I can use EVERY TIME I install a new command-line tool (not just UV):
>
> **Checklist Requirements**:
> 1. Pre-installation security check (verify source, inspect script)
> 2. Installation command (document what it does)
> 3. Post-installation verification (3 tests to confirm it worked)
> 4. Troubleshooting steps (what to check if `command not found`)
> 5. Rollback procedure (how to uninstall if needed)
>
> For UV specifically:
> - Test 1: `uv --version` (should show version number)
> - Test 2: `which uv` or `where uv` (should show installation path)
> - Test 3: `echo $PATH` (should include UV's bin directory)
>
> Make it generic enough I can reuse for other tools."

**Refinement**:
> "This checklist assumes I have terminal access. What if I'm installing in a restricted corporate environment where I can't modify PATH? How would the process change?"

---

**Time**: 20-25 minutes total
**Outcome**: You've built a systematic approach to installing command-line tools safely, diagnosing PATH issues, and creating reusable verification workflows—not just "run this command and hope it works."
