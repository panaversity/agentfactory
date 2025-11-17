---
title: "Zed IDE for Python Development"
chapter: 12
lesson: 7
duration_minutes: 20

skills:
  - name: "Install and Configure Zed IDE"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student installs Zed on their platform and opens a Python project"

  - name: "Use Integrated Terminal in IDE"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student opens terminal in Zed and runs a command"

  - name: "Understand LSP and Code Intelligence"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student explains what LSP is and why it powers code intelligence"

learning_objectives:
  - objective: "Install Zed IDE on your platform and verify installation"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student runs `zed --version` successfully"

  - objective: "Open a uv project in Zed using command-line"
    proficiency_level: "A1"
    bloom_level: "Remember"
    assessment_method: "Student opens existing project with `zed .` and sees file tree"

  - objective: "Understand why Zed is AI-first and when to use it vs. alternatives"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Student explains Zed's design philosophy (speed, minimal overhead, AI collaboration)"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts: IDE purpose, Zed advantages, LSP concept, project opening, integrated terminal, file navigation, extensions. Within A2 limit. ✓"

differentiation:
  extension_for_advanced: "Configure Zed keybindings for faster Python navigation (Cmd+Shift+P, Cmd+P)"
  remedial_for_struggling: "Focus on basic: 'Zed is a text editor that shows your code and terminal together'"

generated_by: "lesson-writer v3.0.0"
source_spec: "specs/001-chapter-12-lightning-python-stack/plan.md"
created: "2025-01-15"
last_modified: "2025-01-15"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Zed IDE for Python Development

## Why Zed? The Modern IDE for AI Developers

You've installed Python tools and created projects with `uv`. Now it's time to **edit your code in a professional environment**—an IDE (Integrated Development Environment) that understands Python and helps you write better code faster.

**The IDE Choice Problem (2025):**

Most developers use one of three editors:
- **VS Code**: Very popular, powerful, but heavy (~200MB, 3+ seconds to open)
- **PyCharm**: Built for Python, feature-rich, but expensive and resource-hungry
- **Zed**: Fast, minimal, designed for AI collaboration (new, gaining traction)

We're learning **Zed** because it aligns with AI-native development. Instead of drowning in features, Zed gives you:
- **Speed**: Opens instantly, runs smoothly
- **AI-first design**: Built with AI collaboration in mind
- **Minimal overhead**: Just code and terminal, nothing bloated
- **Language Server Protocol (LSP)**: Powers real-time code intelligence without memorizing IDE settings

**Source**: Verified in intelligence/001-verified-tool-documentation.md

---

## What Is an IDE? (Tier 1: Understand the Concept)

An **IDE** combines three things:

1. **Text Editor** — Write your code (syntax highlighting, auto-complete)
2. **Terminal** — Run commands without leaving the editor
3. **Debugger & Diagnostics** — Understand errors and warnings in real-time

**Without an IDE**, you'd:
- Edit code in a plain text editor (Notepad, vim)
- Switch to terminal to run commands
- Switch back to editor to fix errors
- Repeat forever = painful

**With Zed**, all three are in one window. You edit, run, and fix without context switching.

---

## Installing Zed (Tier 1: Direct Command)

**Zed** runs on macOS, Windows, and Linux. Choose your platform:

### macOS
```bash
# Download from zed.dev and install via Homebrew
brew install zed

# Verify installation
zed --version
```

**Expected output:** `zed 0.xxx.x` (version number varies)

### Windows
1. Go to [zed.dev](https://zed.dev)
2. Download the Windows installer
3. Run the installer
4. Open PowerShell or Git Bash and verify:
```bash
zed --version
```

### Linux
```bash
# Most distributions (Ubuntu, Fedora, etc.)
curl https://zed.dev/install.sh | sh

# Verify
zed --version
```

**Platform Note**: On **Windows with WSL** (Windows Subsystem for Linux), install Zed from the Windows installer, not inside WSL. WSL can't run GUI apps directly.

---

## Opening a Python Project (Tier 1: Direct Usage)

You've created a uv project in earlier lessons. Now let's open it in Zed.

**From your terminal:**

```bash
# Navigate to your project
cd my-project

# Open entire project in Zed
zed .
```

**What happens:**
- Zed launches and shows your project in a **file tree** (left sidebar)
- Files and folders appear as a navigation menu
- The editor opens ready for editing

**See in Zed:**
- Left panel: File tree showing all files and folders
- Center panel: Editor (currently empty, click a file to edit)
- Bottom panel: Integrated terminal (ready for commands)

---

## Using the Integrated Terminal (Tier 1: Direct Usage)

Instead of switching between terminal and editor, Zed has a **terminal built in**.

**Open integrated terminal:**
- Mac/Linux: `Ctrl+`` (backtick, same key as ~)
- Windows: `Ctrl+`` (backtick)

**Terminal opens at the bottom of Zed.** Now you can run `uv` commands without leaving the editor:

```bash
# Inside Zed's integrated terminal
uv --version

# Install a library
uv add requests

# Run your code
uv run python main.py
```

**Benefit**: You see code on top, terminal output below. No switching windows.

---

## Understanding LSP: Code Intelligence (Tier 2: AI-Explained Concept)

You'll see a word appear in Zed's status bar: **LSP**. What is it?

**LSP = Language Server Protocol** — a standard way for editors to understand code.

Think of it like this:
- **You write code**: `def greet(name):`
- **LSP analyzes it**: "This is a function definition. The parameter `name` should have a type."
- **Editor shows you**: Error hints, auto-complete suggestions, documentation

**Where does LSP come from?**
- Tools like **Pyright** (type checker) and **Ruff** (formatter) speak LSP
- Zed listens to them and shows errors/hints in real-time

**What does it mean when you see "LSP not connected" in Zed?**
- Usually just means Pyright isn't installed yet (we'll fix that in Lesson 10)
- You can still edit code; type checking just isn't available
- Not a problem for now

**Source**: Verified in intelligence/001-verified-tool-documentation.md

---

## Zed Command Palette: The Power Tool (Tier 1: Direct Command)

Zed has a "command palette"—a searchable menu of all commands. You don't need to memorize anything; you just type what you want.

**Open command palette:**
- Mac: `Cmd+Shift+P`
- Windows/Linux: `Ctrl+Shift+P`

**Try these commands:**
- Type `format` → "Format Document" appears (we'll use this in Lesson 9)
- Type `terminal` → "Toggle Terminal" appears (we just used this)
- Type `Python` → Shows Python-related commands

**Key insight**: You never memorize Zed commands. You search for them. Same with AI: tell it what you want, it executes.

---

## File Navigation: Quick Moves (Tier 1: Direct Usage)

You'll spend lots of time jumping between files. Zed makes it fast:

**Quick file opener:**
- Mac: `Cmd+P`
- Windows/Linux: `Ctrl+P`

Type the filename and press Enter. Zed finds and opens it instantly.

**Example**: You're editing `main.py`, need to see `helpers.py`:
1. Press `Cmd+P` (or `Ctrl+P` on Windows)
2. Type `helpers`
3. Press Enter
4. `helpers.py` opens

---

## Platform-Specific Notes (Tier 1: Context)

### macOS Native
Zed works natively on macOS (Intel and Apple Silicon). No special setup needed. VSCode alternative: Microsoft's editor also works great on macOS.

### Windows WSL (Windows Subsystem for Linux)
If you use WSL for development:
- Install Zed on **Windows** (not inside WSL)
- Zed can edit WSL project files via the WSL extension
- Open terminal in Zed, it runs in WSL automatically
- This is advanced; if unsure, just use Windows native Python with Zed

### Linux
Zed works on Ubuntu, Fedora, Debian, and other distros. Installation via `curl` script shown above.

---

## The Next Step: Code Quality Tools (Tier 1: Preview)

So far, Zed is just an editor. But in the next lessons, we'll add:
- **Lesson 8**: Ruff (formatter/linter) — fixes code style automatically
- **Lesson 9**: Ruff config — customize rules for your team
- **Lesson 10**: Pyright (type checker) — catches bugs before running code

These tools integrate with Zed via LSP, making it a complete professional environment.

---

## Try With AI

Use your AI companion for these exercises.

### Prompt 1: Install & Verify (Tier 1 — Direct)

```
I'm on [macOS/Windows/Linux].
1. Install Zed IDE
2. Create a simple Python project with `uv init hello-world`
3. Open it in Zed with `zed .`
4. Take a screenshot showing the project tree and terminal

Show me the commands and what to expect at each step.
```

**Expected outcome:** Zed installed, project opens, you see file tree and integrated terminal.

### Prompt 2: Understand IDE Purpose (Tier 2 — AI Explains)

```
Why is it better to use Zed (with a terminal) than editing code in Notepad and running commands in a separate terminal window? What's the benefit of an IDE?
```

**Expected outcome:** AI explains integrated workflow, reduced context switching, better feedback loops.

### Prompt 3: Zed Keybindings (Tier 2 — Configuration)

```
Show me the most useful Zed keybindings for Python development (on [Mac/Windows/Linux]).
Include: opening terminal, finding files, formatting code.
```

**Expected outcome:** Quick reference for common commands you'll use repeatedly.

---

## Red Flags to Watch

**Problem**: "Zed says 'LSP not connected'"
- **What it means**: Pyright language server isn't installed yet
- **What to do**: This is normal! We install it in Lesson 10. For now, you can still edit code.
- **Not urgent**: Type checking isn't available, but that's OK for this lesson

**Problem**: "Terminal won't open in Zed"
- **What it means**: Keybinding might be different on your system
- **What to do**: Use Command Palette instead (`Cmd+Shift+P` or `Ctrl+Shift+P`), type "terminal", press Enter
- **Fallback**: Open external terminal; both work fine

**Problem**: "Zed is slow / not responding"
- **What it means**: Rare; usually a system resource issue
- **What to do**: Close and reopen Zed; if still slow, check your computer's CPU/memory (Activity Monitor on Mac, Task Manager on Windows)
- **Worst case**: Use VS Code as alternative (slower but more stable on older machines)
