---
title: "Installing UV with AI Collaboration"
chapter: 12
lesson: 2
sidebar_position: 2
duration_minutes: 45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Execute Installation with AI"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can successfully install UV by expressing intent to Claude Code/Gemini CLI ('Install UV on my system') and verify installation"

  - name: "Understand PATH Configuration"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Technical Knowledge"
    measurable_at_this_level: "Student can explain what PATH is (computer's command directory) and why installation adds UV to PATH, troubleshooting 'command not found' errors with AI"

learning_objectives:
  - objective: "Apply AI-driven installation workflow to set up UV on Windows/Mac/Linux"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Successfully installs UV and verifies with `uv --version`"

  - objective: "Understand platform detection and PATH configuration"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Can explain PATH and troubleshoot 'command not found' errors"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (software installation process, PATH variable, platform differences, verification, AI platform detection, error troubleshooting, one-time setup) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Research UV's installation script internals; compare Windows MSI vs. curl installation methods; explore PATH modification mechanisms on different platforms"
  remedial_for_struggling: "Focus on Windows installation only; use AI to verify each step; build confidence with success before exploring cross-platform differences"

# Generation metadata (for traceability and maintenance)
generated_by: "lesson-writer v1.0"
source_spec: "specs/011-python-uv/spec.md"
created: "2025-11-04"
last_modified: "2025-11-04"
git_author: "GitHub Copilot"
workflow: "/sp.implement"
version: "1.0.0"
---

# Installing UV with AI Collaboration

In Lesson 1, you understood *why* UV matters—it's fast, unified, and represents modern Python tooling best practices. Now comes the practical work: installing UV on your system using AI as your interactive documentation.

This lesson demonstrates the AI-driven development workflow in action. You'll express your intent ("Install UV on my Windows/Mac/Linux system"), your AI collaborator will provide platform-specific commands, you'll execute them, and you'll verify the result. No memorization required—you'll understand *what's happening* conceptually while AI handles the mechanical details.

By the end of this lesson, UV will be installed and verified on your machine, and you'll understand what happened during installation (software download, PATH modification, verification workflow). You'll also know how to troubleshoot the most common installation error ("command not found") using AI guidance.

## Pre-Installation: What We're About to Do

Let's demystify software installation before we start. When you install UV (or any command-line tool), three things happen:

1. **Download**: Your system retrieves the UV executable (the program file) from the internet
2. **Extract and Place**: The downloaded file is extracted (if compressed) and placed in a standard location on your computer
3. **PATH Modification**: Your system's PATH environment variable is updated so you can run `uv` from any directory

**What is PATH?** Think of PATH as your computer's command registry—a list of directories where your system looks for programs when you type a command in the terminal.

When you type `uv --version` in PowerShell (Windows), Terminal (Mac), or a shell (Linux), your system searches every directory listed in PATH until it finds an executable file named `uv` (or `uv.exe` on Windows). If UV's installation directory isn't in PATH, your system can't find it—that's the "command not found" error you might encounter.

**The installation process** we're about to follow:
1. You'll tell your AI collaborator your operating system
2. AI will provide the appropriate installation command for your platform
3. You'll execute that command (AI will explain what it does)
4. You'll verify UV installed correctly by running `uv --version`
5. If verification fails, you'll troubleshoot with AI assistance

Ready? Let's install UV.

## AI-Driven Installation Workflow

This is where the AI-native approach shines. Instead of searching documentation, copying commands you don't understand, and hoping for the best, you'll express your intent clearly and let AI guide you through platform-specific installation.

### Step 1: Express Your Intent

Open your AI companion tool (Claude Code, Gemini CLI, or ChatGPT web interface) and use this prompt template:

**Prompt Template**:
```
Install UV Python package manager on my [Windows/Mac/Linux] system. 
Explain what the installation command does before I run it.
Include verification steps after installation.
```

**Example for Windows PowerShell**:
```
Install UV Python package manager on my Windows system using PowerShell. 
Explain what the installation command does before I run it.
Include verification steps after installation.
```

### Step 2: Review AI's Response

Your AI will provide a platform-specific installation command. Here's what to expect for each platform:

#### Windows (PowerShell)

**AI's Suggested Command**:
```powershell
irm https://astral.sh/uv/install.ps1 | iex
```

**AI's Explanation** (approximate):
> This command does two things:
> 1. `irm` (Invoke-RestMethod) downloads the UV installation script from Astral's official server
> 2. `| iex` (pipe to Invoke-Expression) executes that script immediately
>
> The script will:
> - Download the UV executable for Windows
> - Place it in `%USERPROFILE%\.local\bin\` (your user directory)
> - Modify your PATH environment variable to include that directory
> - No admin rights required (installs to your user profile)

**What you should verify before running**:
- URL is `https://astral.sh/uv/install.ps1` (official Astral domain)
- You understand this is downloading and executing a script
- You're in PowerShell (not Command Prompt or Git Bash)

#### macOS (Terminal with Homebrew or curl)

**Option 1: Homebrew** (if you have Homebrew installed):
```bash
brew install uv
```

**AI's Explanation**:
> Homebrew is macOS's package manager. This command:
> - Downloads UV from Homebrew's package repository
> - Installs it to `/opt/homebrew/bin/` (Apple Silicon) or `/usr/local/bin/` (Intel)
> - These directories are already in PATH on macOS by default
> - Homebrew handles dependency management and updates

**Option 2: curl** (if you don't have Homebrew):
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**AI's Explanation**:
> This command:
> - `curl -LsSf` downloads the UV installation script (with follow redirects, silent mode)
> - `| sh` pipes the script to the shell for immediate execution
> - Installs UV to `~/.local/bin/` (your home directory)
> - Modifies your shell configuration (~/.bashrc, ~/.zshrc) to add UV to PATH

#### Linux (curl)

**AI's Suggested Command**:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**AI's Explanation** (same as macOS curl method):
> Downloads and runs Astral's official installation script.
> Installs to `~/.local/bin/` and updates your shell configuration.

### Step 3: Execute the Command

**Before you run the command**:
1. **Verify the URL** is from an official Astral domain (`astral.sh`)
2. **Understand what happens**: Script downloads UV, places it in a user directory, modifies PATH
3. **Note: No sudo required** for user-space installs (if AI suggests `sudo`, ask why)

**Run the command** in your terminal/PowerShell. You should see output indicating download progress and installation success.

**Expected output** (approximate):
```
Downloading UV...
Installing UV to /Users/yourname/.local/bin/uv
Adding /Users/yourname/.local/bin to PATH in ~/.zshrc
UV successfully installed!
```

**Platform Note: Windows PowerShell might require execution policy changes**

If you see an error like:
```
... running scripts is disabled on this system
```

AI should suggest:
```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

This allows scripts signed by trusted publishers (like Astral) to run. After this change, re-run the UV installation command.

## Platform Detection: How AI Determines Your OS

When you tell AI "install UV on my system," AI typically:

1. **Checks context clues**: If you mention "PowerShell," AI assumes Windows
2. **Asks for clarification**: If ambiguous, AI might ask "Which operating system?"
3. **Provides multiple options**: AI might offer commands for all platforms and let you choose

**Why platform-specific commands?**
- **Windows**: Uses PowerShell scripts (.ps1) and places executables in `%USERPROFILE%\.local\bin\`
- **macOS**: Prefers Homebrew (if available) for package management, otherwise curl + shell script
- **Linux**: Uses curl + shell script, updates shell config files (bashrc, zshrc)

Each platform has different conventions for:
- Where to place executables (user directories differ)
- How to modify PATH (Windows registry vs. Unix shell config files)
- Package management ecosystems (Homebrew on Mac, apt/yum on Linux, no standard on Windows)

**AI's role**: Abstract these differences. You express intent ("install UV"), AI provides the right command for your platform. You don't memorize three different installation methods—you understand the *concept* (download, place, configure PATH) and let AI handle platform specifics.

## Verification: Running `uv --version`

After installation completes, verify UV is accessible:

### Verification Command

**Prompt to AI**:
```
Verify UV is installed and show me the version
```

**AI's suggested command**:
```bash
uv --version
```

**Expected output**:
```
uv 0.4.10
```
(Version number will vary; any version confirms installation succeeded)

### If Verification Succeeds

Congratulations! UV is installed and ready to use. You can now create projects, manage dependencies, and run Python code with UV.

**What just happened** (conceptually):
1. Installation script downloaded UV executable to a user directory
2. That directory was added to your PATH environment variable
3. When you typed `uv --version`, your system found the UV executable in PATH
4. UV printed its version number, confirming it's working

### If Verification Fails ("command not found")

If you see:
```
uv: command not found
```

Or (Windows):
```
'uv' is not recognized as an internal or external command...
```

**This means**: UV is installed but not in your PATH, or your current terminal session hasn't reloaded PATH yet.

**Proceed to the troubleshooting section below.**

## Troubleshooting: Command Not Found

The "command not found" error is the most common installation issue. It usually means one of two things:

### Issue 1: PATH Not Updated in Current Session

**The problem**: Installation modified your PATH, but your current terminal session started *before* the change and hasn't reloaded.

**Solution: Restart your terminal**
- **Windows**: Close PowerShell completely and open a new PowerShell window
- **Mac/Linux**: Close Terminal and open a new Terminal window, or run `source ~/.bashrc` (Bash) or `source ~/.zshrc` (Zsh)

**Then try** `uv --version` again.

**AI prompt if this doesn't work**:
```
I restarted my terminal and still get "uv: command not found". What should I check next?
```

### Issue 2: PATH Not Modified During Installation

**The problem**: Installation script didn't successfully add UV's directory to PATH.

**Solution: Manually verify and add to PATH**

**Prompt to AI**:
```
The UV installation didn't add UV to my PATH. Help me verify where UV is installed and add it to PATH manually on [Windows/Mac/Linux].
```

**AI will guide you through**:
1. **Finding UV's installation directory**:
   - Windows: Typically `%USERPROFILE%\.local\bin\`
   - Mac/Linux: Typically `~/.local/bin/`

2. **Checking if UV exists there**:
   - Windows: `Test-Path $env:USERPROFILE\.local\bin\uv.exe`
   - Mac/Linux: `ls ~/.local/bin/uv`

3. **Manually adding to PATH** (if needed):
   - **Windows** (PowerShell, persistent):
     ```powershell
     $env:PATH += ";$env:USERPROFILE\.local\bin"
     [Environment]::SetEnvironmentVariable("Path", $env:PATH, "User")
     ```
   - **Mac/Linux** (add to shell config):
     ```bash
     echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc  # or ~/.zshrc
     source ~/.bashrc  # or source ~/.zshrc
     ```

4. **Verify again**: `uv --version`

### Issue 3: Installation Failed Silently

**The problem**: Installation script encountered an error and UV never downloaded.

**Solution: Re-run installation with verbose output**

**Prompt to AI**:
```
The installation might have failed. Show me how to re-install UV with verbose output so I can see what's happening.
```

AI might suggest checking:
- Internet connection (can you reach `https://astral.sh`?)
- Disk space (is there room for UV executable?)
- Permissions (can the script write to user directories?)

**Re-run installation** and watch for error messages. Share any errors with AI for diagnosis.

## What Happened During Installation? (Plain Language Explanation)

Let's demystify what that installation command actually did:

1. **Downloaded a script**: The `irm` (Windows) or `curl` (Mac/Linux) part fetched a small installation program from Astral's servers

2. **Executed the script**: The `| iex` (Windows) or `| sh` (Mac/Linux) part ran that program immediately

3. **The script then**:
   - Detected your operating system and CPU architecture (Windows x64? Mac Apple Silicon? Linux x86?)
   - Downloaded the appropriate UV executable binary for your system
   - Created a directory in your user profile (`~/.local/bin` or equivalent)
   - Placed the UV executable in that directory
   - Modified your system's PATH variable to include that directory
   - Optionally updated shell configuration files (.bashrc, .zshrc) to persist the PATH change

4. **Result**: You can now type `uv` from any directory, and your system knows where to find it

**Why user directories?** Installing to user profile (`~/.local/bin`) means:
- No admin/sudo privileges needed
- Your installation doesn't affect other users on the same machine
- Safer (you can't accidentally break system-wide Python installations)

**Why modify PATH?** Without PATH modification, you'd need to type the full path every time: `~/.local/bin/uv --version`. PATH lets you use the short command `uv --version` from anywhere.

**One-time setup**: You install UV once per machine. After this, you'll use UV commands (uv init, uv add, uv run) repeatedly across many projects—no reinstallation needed unless you want to upgrade UV itself.

## Try With AI

Now let's practice the complete installation and verification workflow. Work through these prompts in sequence with your AI companion tool.

**Setup**: Use Claude Code, Gemini CLI, or ChatGPT web interface

### Prompt 1: Installation
```
Install UV package manager on my [Windows/Mac/Linux] system. 
Explain what the installation command does before I run it. 
Include verification steps after installation.
```

**What you're practicing**: Expressing clear intent, understanding commands before execution

**Expected outcome**: AI provides platform-specific command with explanation. You execute it after understanding what it does.

**Validation**: Did AI explain the download, installation location, and PATH modification? Did you understand before running?

### Prompt 2: Verification
```
Verify UV is installed and show me the version
```

**What you're practicing**: Confirming successful installation

**Expected outcome**: AI suggests `uv --version`. You see a version number printed.

**Validation**: Did you get a version number? If yes, installation succeeded. If "command not found," proceed to Prompt 3.

### Prompt 3: Troubleshooting (if needed)
```
I'm seeing "uv: command not found" after installation. What should I do?
```

**What you're practicing**: Diagnosing and fixing PATH issues with AI

**Expected outcome**: AI suggests restarting terminal first, then checking PATH configuration if that doesn't work

**Validation**: Did AI provide step-by-step troubleshooting? Did following AI's steps resolve the issue?

### Prompt 4: Conceptual Understanding
```
Explain what happened during UV installation in simple terms. 
Why did the installation need to modify my PATH? 
What would happen if PATH wasn't updated?
```

**What you're practicing**: Building conceptual understanding beyond command execution

**Expected outcome**: AI explains download → place → PATH modification workflow, explains PATH's purpose

**Validation**: Can you now explain to a peer what PATH is and why installation modifies it?

