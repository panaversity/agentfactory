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

::::os-tabs

::macos
Open your terminal and run:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**What this does:**
- Downloads the official UV installation script from Astral
- Executes it to install UV to `~/.local/bin/`
- Automatically adds UV to your PATH so you can use the `uv` command

**Time:** ~30 seconds

::linux
Open your terminal and run:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**What this does:**
- Downloads the official UV installation script from Astral
- Executes it to install UV to `~/.local/bin/`
- Automatically adds UV to your PATH so you can use the `uv` command

**Time:** ~30 seconds

::windows
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

::::

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

## Try With AI

What happens when you run an installation script, and how do you fix it when things go wrong?

**🔍 Explore Installation Security:**
> "Explain what this command does: `curl -LsSf https://astral.sh/uv/install.sh | sh`. What security risks exist with downloading and piping to sh? Show me how to inspect the script content before running it to verify it's safe."

**🛡️ Diagnose PATH Configuration Issues:**
> "I installed UV but get 'command not found' when running `uv --version`. I'm on [Windows/Mac/Linux]. Ask me diagnostic questions to figure out what went wrong, then walk me through fixing it step-by-step."

**🧪 Test Your Troubleshooting Logic:**
> "Here's an installation error: 'Permission denied when installing UV on Mac'. What are the three most likely causes? For each cause, give me the diagnostic command to verify it and the fix. Explain why this happens on macOS."

**🚀 Build Installation Verification Steps:**
> "Create a post-installation checklist I can use for ANY command-line tool: verify installation succeeded, confirm PATH is configured, and troubleshoot 'command not found'. Keep it under 5 steps with specific commands I can run."

Always inspect installation scripts before executing them—the pattern `curl <url> | sh` downloads and runs code immediately.

---
