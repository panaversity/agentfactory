---
title: "Installing Python 3.14+ on Your Computer"
chapter: 13
lesson: 2
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Python Installation & Verification"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can download Python from python.org; follow OS-specific instructions; run `python --version`; recognize correct output"

  - name: "Version Management Understanding"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Remember"
    digcomp_area: "Information & Data Literacy"
    measurable_at_this_level: "Student understands versioning notation (3.14+) and why we use specific versions"

  - name: "Troubleshooting with AI Assistance"
    proficiency_level: "A2"
    category: "Soft"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can describe installation errors to AI and follow troubleshooting steps provided"

learning_objectives:
  - objective: "Install Python 3.14+ successfully on your computer"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "`python --version` command returns Python 3.14+ or higher"

  - objective: "Verify installation using terminal commands"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Running test program produces expected output"

  - objective: "Troubleshoot installation errors with AI assistance"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Successfully resolves platform-specific errors using AI guidance"

cognitive_load:
  new_concepts: 4
  assessment: "4 new concepts (What is an installer, Python from python.org, Version importance, Terminal verification) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Explore Python version history; understand why we chose 3.14+ over older versions; research package manager options"
  remedial_for_struggling: "Work with pre-installed Python environments (cloud-based alternatives) if local installation proves problematic"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/016-part-4-chapter-13/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Installing Python 3.14+ on Your Computer

Now that you understand what Python is and why it matters, it's time to get it running on your computer. This lesson is your bridge from theory to practice. Without Python installed, you can't run any code. With Python installed, you unlock the ability to execute programs and start building.

This lesson includes platform-specific instructions for Windows, Mac, and Linux. Pick your operating system below.

## Why Installation Matters

You've learned that Python is a programming language, but that knowledge exists only in your mind. To run Python code, you need the **Python interpreter**—a program that reads Python code and executes it on your computer.

Think of it this way: The Python language is like musical notation. The Python interpreter is like a musician who can read that notation and play the music. Just as you need a musician to hear the music, you need the Python interpreter to run Python code.

An **installer** is a program that sets up the Python interpreter on your computer. It downloads the correct version for your operating system and configures it so you can run Python from the terminal.

## Getting Python from the Official Source

You'll download Python from **python.org**—the official Python website. This is critical: always download from python.org, never from third-party sources. Why? Because python.org is the authoritative source. It guarantees you're getting the genuine, secure version without modifications.

Go to [python.org](https://www.python.org) now. You'll see a big yellow download button. Click it.

## Why We Need Python 3.14+ Specifically

Python comes in versions. The version number looks like `3.14.0` or `3.14.1`.

**Version numbers work like this:**

- The first number (3) is the **major version**. Python 3 was a major rewrite from Python 2 (which is very old and no longer used).
- The second number (14) is the **minor version**. Each minor version adds features and improvements.
- The third number (0, 1, etc.) is the **patch version**. Patches fix bugs without adding features.

We specifically require Python 3.14+ because:

1. **Modern features**: Python 3.14 includes syntax and libraries that we'll use in this book
2. **Performance**: Newer versions are faster
3. **Security**: Older versions have known security vulnerabilities
4. **Type hints**: Python 3.14 has improved type hint support (critical for our AI-Native Learning approach)

When you see "3.14+", it means "version 3.14 or any newer version" (like 3.15, 3.20, etc.).

## Installation: Choose Your Operating System

### Windows Installation

**Step 1: Download the Installer**

From python.org, download the **Windows Installer** for Python 3.14+ (64-bit is recommended for most computers).

**Step 2: Run the Installer**

- Locate the downloaded file (usually in your Downloads folder)
- Double-click it to launch the installer
- **IMPORTANT**: Check the box that says **"Add Python to PATH"** before clicking Install. This is critical—it allows you to run Python from the terminal

**Step 3: Complete Installation**

- Accept the license agreement
- Choose "Install Now" (standard installation is fine)
- Wait for installation to complete
- Click "Close"

**Step 4: Verify Installation**

Open Command Prompt or PowerShell and run:

```bash
python --version
```

You should see output like: `Python 3.14.0` (or higher)

If you see an error like "python: command not found", you may need to add Python to PATH manually. Ask your AI tool: "I installed Python on Windows but `python` command not recognized. How do I add it to PATH?"

---

### Mac Installation

**Step 1: Download the Installer**

From python.org, download the **macOS Installer** for Python 3.14+ (choose the appropriate version for your Mac's processor—Apple Silicon or Intel).

**Step 2: Run the Installer**

- Locate the downloaded .pkg file
- Double-click to launch the installer
- Follow the prompts to install

**Step 3: Verify Installation**

Open Terminal and run:

```bash
python3 --version
```

You should see: `Python 3.14.0` (or higher)

**Important Mac note**: On Mac, the command is `python3` (not `python`) to ensure you're using Python 3 rather than old Python 2. Remember this distinction!

If you have any issues, ask your AI: "I installed Python on Mac but I'm not sure it worked. When I run `python3 --version` I get [paste the output]. Is this correct?"

---

### Linux Installation

Linux users typically install Python through a package manager rather than downloading an installer.

**For Ubuntu/Debian:**

```bash
sudo apt update
sudo apt install python3 python3-venv
```

**For Fedora/RHEL:**

```bash
sudo dnf install python3 python3-venv
```

**For Arch:**

```bash
sudo pacman -S python
```

**Step 2: Verify Installation**

```bash
python3 --version
```

You should see: `Python 3.14.0` (or higher)

Like on Mac, Linux uses `python3` as the command.

---

## Verification: Running Your First Python Test

Now that you've installed Python, let's verify it actually works. Open your terminal/command prompt and run this command:

```bash
python -c "print('Hello, Python!')"
```

(On Mac/Linux, use `python3` instead of `python`)

You should see:
```
Hello, Python!
```

If you see this, congratulations! Python is installed and working.

**If you see an error**, copy the entire error message and use the AI Colearning Prompt section below.

#### 💬 AI Colearning Prompt

If you encountered an error during installation:

> "I tried to install Python on [Windows/Mac/Linux] and got this error: [paste the complete error message]. What does this mean? What's the most likely cause? How do I fix it step-by-step?"

Provide complete context (your operating system and the exact error message), and your AI tool can give you platform-specific troubleshooting. This is a valuable skill: when something breaks, describing the problem clearly to your AI partner is half the battle toward fixing it.

#### ✨ Teaching Tips

- **Different computers, different setups**: If your installation differs from the guide, that's normal. Operating systems vary. If you encounter a unique error, your AI tool is perfect for platform-specific troubleshooting.

- **Copy the exact error message**: "Python not found" is less helpful to an AI than pasting the full error output. Be specific. This teaches you a professional skill: providing complete context when asking for help.

- **One verification isn't enough**: After running the basic verification, run the test program command above. That second check confirms Python is not just installed but actually executable.

#### 🚀 CoLearning Challenge

Once you've installed Python and verified it works, experiment with this:

```bash
python -c "print('Hello, World!')"
```

Now ask your AI:

> "Explain what the `-c` flag does in the `python` command. What does `-c` stand for? Can you show me another example of using `-c`?"

This challenge teaches you to explore with AI's help. You'll learn about command-line flags—a skill that extends far beyond Python.

## Common Mistakes to Avoid

**Mistake 1: Installing Python 2 instead of Python 3.14+**

Some older websites might offer Python 2. Don't use it. Python 2 is obsolete and doesn't have the features we need.

**How to check**: Run `python --version` (or `python3 --version` on Mac/Linux). The output must start with "3.14" or higher.

**How to fix**: Uninstall your current Python. Reinstall from python.org, ensuring you download Python 3.14+.

---

**Mistake 2: "Python command not found" in terminal**

This usually means Python isn't in your PATH (an environment variable that tells your computer where to find programs).

**Windows fix**: Uninstall Python. Reinstall it, and **make sure to check "Add Python to PATH"** during installation.

**Mac/Linux fix**: Use `python3` instead of `python`. Or ask your AI: "How do I make `python` work instead of `python3` on my system?"

---

**Mistake 3: Not sure if installation worked**

Test with this command:

```bash
python -c "print('Test successful')"
```

If you see "Test successful" printed, installation worked. If you see an error, take the error message and ask your AI for help.

---

## Try With AI

Use Claude Code or Gemini CLI for this section.

**Prompt 1: Recall – Python Installation Source (Bloom's Level 1: Remember)**

```
Where did you download Python from? Why did the lesson specify python.org
and not some other website? What's special about python.org?
```

**Expected Outcome**: You'll recall the official source and understand why downloading from trusted sources matters. This builds security awareness—a professional habit.

---

**Prompt 2: Understand – Version Numbers (Bloom's Level 2: Understand)**

```
The lesson says "Python 3.14+ is required."

Explain:
1. What do the numbers in "3.14+" mean? (What's 3? What's 14? What's the +?)
2. Why do we need a specific version instead of just "any Python"?
3. How did you check your version? (What command did you run?)
```

**Expected Outcome**: After the AI's response, you'll understand semantic versioning. You'll learn why version numbers matter in software—this concept applies far beyond Python.

---

**Prompt 3: Apply – Troubleshooting a Real Error (Bloom's Level 3: Apply)**

```
Imagine a friend says: "I installed Python but when I run `python --version`
I get an error: 'python: command not found'."

You're now the expert. What would you tell them?
- What's the likely cause?
- What should they try first?
- If that doesn't work, what's the next step?
```

**Expected Outcome**: You'll practice explaining troubleshooting steps to someone else. This deepens your own understanding and builds your ability to help others—a key professional skill.

---

**Prompt 4: Analyze – Installation Verification (Bloom's Level 4: Analyze)**

```
We verified Python installation in two ways:
1. `python --version` (shows the version number)
2. `python -c "print('Hello, Python!')"` (runs a test program)

Ask your AI: "Why use TWO checks instead of just one? What does each check
tell us that the other doesn't? When might `--version` work but the second
command fail? Why would that matter?"
```

**Expected Outcome**: You'll understand distinction between "is Python installed?" (version check) vs. "does Python actually work?" (functionality check). You'll learn professional verification strategies that you'll use throughout your development career.

