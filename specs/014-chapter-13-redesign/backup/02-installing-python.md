---
title: "Installing Python 3.14+ on Your Computer"
chapter: 13
lesson: 2
duration_minutes: 90

skills:
  - name: "Python Installation and Verification"
    proficiency_level: "A1-A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can download Python from python.org, follow OS-specific installation, and verify with `python --version`"

  - name: "Platform-Specific Troubleshooting with AI"
    proficiency_level: "A1-A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can provide error messages to AI and understand platform-specific solutions"

learning_objectives:
  - objective: "Install Python 3.14+ on Windows, Mac, or Linux and verify successful installation"
    proficiency_level: "A1-A2"
    bloom_level: "Apply"
    assessment_method: "`python --version` outputs Python 3.14+ and test program runs without error"

cognitive_load:
  new_concepts: 4
  assessment: "4 concepts (installer, python.org, version importance, terminal verification) within A1-A2 limit ✓"

differentiation:
  extension_for_advanced: "Explore Python installation from source code; compare package managers across platforms"
  remedial_for_struggling: "Focus on Windows installation first; use step-by-step screenshots"

generated_by: "content-implementer v3.0.0"
source_spec: "specs/016-part-4-chapter-13/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 2: Installing Python 3.14+ on Your Computer

Without a working Python installation, nothing else in this chapter happens. So let's get this right.

This lesson is **platform-specific**. Scroll to your operating system (Windows, Mac, or Linux) and follow those instructions. If something goes wrong, we'll show you how to troubleshoot with your AI companion.

## Why Installation Matters

You need Python installed on your computer to run Python programs. Just like you need Word installed to open .docx files, you need Python installed to run .py files.

We're specifically using **Python 3.14+** (the latest modern version). Why 3.14+ and not an older version? Because newer versions have features we use throughout Part 4. Older versions lack these features.

Installation is the bridge between "understanding Python" and "running Python code."

## Choose Your Operating System

Pick the section below that matches your computer:

## Windows Installation

### Step 1: Download Python

Go to **[python.org/downloads](https://www.python.org/downloads/)** in your web browser.

You'll see a large yellow button that says "Download Python 3.14.x" (or whatever the latest version is). Click it.

This downloads an installer file to your computer (usually to your Downloads folder).

### Step 2: Run the Installer

Find the downloaded file (it'll be named something like `python-3.14.x-amd64.exe`) and double-click it.

An installer window opens. This is where Windows installations often go wrong, so pay attention to this:

**CRITICAL**: You'll see a checkbox that says "Add Python to PATH."

**Check this box.** This allows your terminal to find Python.

Click through the remaining screens and select "Install Now." The installer will add Python to your computer.

### Step 3: Verify Installation

Open a terminal (Command Prompt or PowerShell). Type:

```
python --version
```

Press Enter. You should see output like:

```
Python 3.14.0
```

If it shows "Python 2.x" or a lower version, you need to reinstall. If it says "python: command not found," see "Troubleshooting" below.

### Step 4: Test with a Simple Program

Still in your terminal, type this exact command:

```
python -c "print('Hello, Python!')"
```

Press Enter. You should see:

```
Hello, Python!
```

Congratulations—Python is installed and working on Windows.

---

## Mac Installation

### Step 1: Download Python

Go to **[python.org/downloads](https://www.python.org/downloads/)** in your web browser.

Click the yellow "Download Python 3.14.x" button.

This downloads an installer file (usually to your Downloads folder).

### Step 2: Run the Installer

Find the downloaded file and double-click it.

A macOS installer window opens. Follow the prompts. Unlike Windows, you don't need to worry about PATH—macOS usually handles this automatically.

### Step 3: Verify Installation

Open Terminal (Applications → Utilities → Terminal). Type:

```
python3 --version
```

Press Enter. You should see:

```
Python 3.14.0
```

Note: On Mac, you might need to use `python3` instead of `python` if Python 2 is also installed. Both work—they're the same thing on modern Macs.

### Step 4: Test with a Simple Program

In your terminal, type:

```
python3 -c "print('Hello, Python!')"
```

Press Enter. You should see:

```
Hello, Python!
```

Python is now installed and working on Mac.

---

## Linux Installation

### Step 1: Use Your Package Manager

Linux doesn't require downloading from python.org. Your package manager can install Python.

**Ubuntu/Debian:**
```
sudo apt update
sudo apt install python3.14
```

**Fedora/Red Hat:**
```
sudo dnf install python3.14
```

**macOS (using Homebrew, if you prefer):**
```
brew install python@3.14
```

### Step 2: Verify Installation

Open a terminal. Type:

```
python3 --version
```

Press Enter. You should see:

```
Python 3.14.0
```

### Step 3: Test with a Simple Program

```
python3 -c "print('Hello, Python!')"
```

Press Enter. You should see:

```
Hello, Python!
```

Python is installed on Linux.

---

## Troubleshooting with AI Assistance

Did something go wrong? Don't panic. This is where AI excels.

#### 💬 AI Colearning Prompt
> "I tried to install Python on [Windows/Mac/Linux] and got this error: [paste the full error message here]. What does this mean, and how do I fix it step-by-step?"

#### 🎓 Expert Insight
> In AI-native development, installation errors are learning opportunities. System configuration (PATH variables, permissions, OS differences) is complex—AI excels at diagnosing these platform-specific issues. Your job: provide complete error context (OS version, exact error message). AI's job: translate cryptic system errors into actionable fixes. This troubleshooting partnership is essential for professional development.

#### 🤝 Practice Exercise

> **Ask your AI**: "Show me 3 different ways to verify Python 3.14 is installed correctly (beyond just `python --version`). For each method, explain what it validates that the others don't. Then create a one-line Python command that prints my Python version in a formatted message like 'Python version: X.X.X'."

**Expected Outcome**: You'll learn multiple verification strategies (version check, test execution, module availability), understand why redundant validation matters in professional development, and practice using Python's `-c` flag for quick commands.

---

## Common Mistakes

**Mistake 1**: "I installed Python 2 instead of Python 3.14"

Check your version: `python --version` should show "Python 3.14.x" or higher.

If it shows "Python 2.x", uninstall Python 2 and install Python 3.14 from python.org.

**Mistake 2**: "Python command not found" or "python: command not found"

This usually means Python isn't in your PATH (especially on Windows).

On Windows: Reinstall Python and check the "Add Python to PATH" checkbox.

On Mac/Linux: Try using `python3` instead of `python`.

If still stuck, ask your AI companion: "I get 'python: command not found' when I run `python --version`. How do I fix this?" Include your operating system.

**Mistake 3**: "I don't know if my installation is correct"

Run both these commands:

```
python --version
```

```
python -c "print('Hello')"
```

If both work and show correct output, your installation is correct. Done.

---

## Try With AI

Use your AI companion (Claude Code or Gemini CLI) for these prompts.

**Prompt 1: Recall – Where and Why**

```
Where did you download Python from? Why does the lesson specify python.org (not some random website)?
```

**Expected Outcome**: You recall the official source. You understand why downloading from the official website matters (authenticity, security).

---

**Prompt 2: Understand – Version Requirements**

```
The lesson says "Python 3.14+ is required." Explain:
1. Why do we need a specific version (why not any Python 3.x)?
2. What does "3.14+" mean? (What's the "+" symbol mean?)
3. How did you check your version?
```

**Expected Outcome**: You understand semantic versioning (3.14 means major.minor). You see "3.14+" as "version 3.14 or higher". You can explain why version matters.

---

**Prompt 3: Apply – Troubleshoot an Error**

```
Imagine a friend says: "I installed Python but when I run `python --version` I get an error: 'python: command not found'."

What would you tell them to do? You're the expert now!
Explain the likely cause and solution.
```

**Expected Outcome**: You apply your troubleshooting knowledge to a real problem. You practice explaining technical solutions—a valuable skill.

---

**Prompt 4: Analyze – Verification Strategies**

```
We verified Python installation with TWO commands:
1. `python --version`
2. `python -c "print('Hello')"`

Why use two commands instead of just one?
What does each command tell us?

Ask your AI: "What's the difference between `python --version` and running a test print statement?"
Think about: What information does each provide? Why do both matter?
```

**Expected Outcome**: You understand that version checking and functionality testing are different. You learn about verification strategies—a professional skill for validating that software works.

