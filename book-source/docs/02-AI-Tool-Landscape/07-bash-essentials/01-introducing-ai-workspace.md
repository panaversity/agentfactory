---
sidebar_position: 1
title: "Introducing Your AI Companion's Workspace"
chapter: 7
lesson: 1
duration_minutes: 35

skills:
  - name: "Understanding File System Navigation"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information"
    measurable_at_this_level: "Learner can identify current directory and file types from AI execution without memorizing commands"

  - name: "AI Collaboration Understanding"
    proficiency_level: "A1"
    category: "Soft"
    bloom_level: "Understand"
    digcomp_area: "Communication"
    measurable_at_this_level: "Learner understands AI has a 'location' and can show files; recognizes learner's role is supervision"

learning_objectives:
  - objective: "Identify your AI companion's current working directory by observing pwd in natural conversation"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Prediction task: before AI shows output, learner predicts what directory path will appear"

  - objective: "Interpret ls output to recognize file types and folder structure"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Analysis task: read ls dialogue output and identify which items are files vs folders"

  - objective: "Ask your AI companion 'Where are you?' and understand why this matters before any operation"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Reflection: explain why knowing location matters before AI takes actions"

cognitive_load:
  new_concepts: 3
  assessment: "3 new concepts (current directory, files vs folders, AI supervision) within A1 limit of 5 ✓"
---

# Introducing Your AI Companion's Workspace


Imagine hiring a contractor to renovate your house. Before they swing a hammer, you'd ask: "Where are you starting? Show me what you're looking at." You wouldn't let them work blind, and you wouldn't work blind either.

---

## What is the Terminal? (Start Here If New to Command Line)

**Already comfortable with Terminal/CLI?** [Skip to "Your AI Has a Location"](#your-ai-has-a-location-and-you-need-to-know-it)

### The Terminal: Your Text-Based Remote Control

The **Terminal** (also called Command Line or CLI) is a text interface where you type commands instead of clicking buttons. Think of it like texting instructions to your computer instead of using a mouse.

**Why developers use it:**
- **Precision**: You say exactly what you want, no hunting through menus
- **Speed**: One command can do what takes 20 clicks
- **Automation**: Commands can be saved and repeated
- **AI Collaboration**: Your AI companion operates here—understanding this workspace is essential for supervising its work

**Real example:**
- **With mouse**: Open Finder → Navigate to Documents → Create folder → Name it "my-project" → Open folder
- **With Terminal**: Type `mkdir Documents/my-project && cd Documents/my-project` (done in 2 seconds)

### How to Open the Terminal

**macOS:**
1. Press `Cmd + Space` (opens Spotlight)
2. Type "Terminal"
3. Press Enter

**Windows:**
1. Press `Win + R`
2. Type `cmd` or `powershell`
3. Press Enter

**Linux:**
- Press `Ctrl + Alt + T` (on most distributions)

**What you'll see:**
A black or white window with a blinking cursor. This is your **command prompt**—where you type instructions.

```bash
user@computer:~$
```

![Annotated terminal screenshot showing prompt (username@host), command (ls), arguments (-la /home), and output (file listing), with labels explaining each component and their purposes](/img/part-2/chapter-07/terminal-anatomy-annotated.png)

That `$` symbol means "I'm ready for your command."

### Your First Command: Say Hello

Type this and press Enter:

```bash
$ echo "Hello Terminal"
Hello Terminal
```

You just instructed your computer to display text. The `$` is not typed—it's just showing where commands start.

**What just happened:**
1. You typed `echo "Hello Terminal"` (the **command**)
2. You pressed Enter (the **execute** signal)
3. The Terminal printed the result: `Hello Terminal`

This is how all Terminal commands work: type instruction → press Enter → see result.

### Why This Matters for AI Collaboration

Your AI companion (Claude Code, ChatGPT Code Interpreter, Cursor, etc.) operates in the Terminal. When you ask it to:
- "Create a Python project"
- "Install dependencies"
- "Run tests"

...it's typing Terminal commands for you. **Understanding this workspace** means you can:
- **Supervise** what your AI is doing (verify it's in the right folder)
- **Catch mistakes** before they happen (see what files it's about to modify)
- **Build trust** (you're not blindly accepting AI actions)

This lesson teaches you how to ask your AI: "Where are you? What can you see? Show me before you act."

---

## Your AI Has a Location—And You Need to Know It

This lesson teaches you the same habit for working with your AI companion in the terminal. Your AI assistant has a **current location** in your computer's file system, and understanding that location is the first step to supervising its work safely. You're not learning bash commands. You're learning to have effective conversations with your AI about **where it is and what it can see**.

By the end of this lesson, you'll be able to ask your AI companion three simple questions:
1. **Where are you working right now?** (It will show you the path)
2. **What files can you see here?** (It will list them for you)
3. **Why does location matter?** (It affects what files it can access and operate on)

![Directory tree diagram showing typical workspace structure (home directory with Documents, projects, src folders), current working directory highlighted, and common file types indicated](/img/part-2/chapter-07/bash-workspace-directory-tree.png)

---

## Use `pwd` to Know Where Your AI Is Working

The `pwd` command is bash-speak for "print working directory." It shows you—or your AI—the current folder path.

### Step 1: You Try It

Open a terminal on your computer and type this command:

```bash
$ pwd
/Users/yourname/Documents
```

You'll see a file path. That's your **current directory**—where you're "standing" in your file system. All commands you run happen relative to this location.

**What to notice**: You just ran the same command your AI will run. The output is a path (like `/Users/yourname/Documents` or `/home/username/projects`). That's it.

### Step 2: Your AI Does the Same

Now ask your AI companion (Claude Code, ChatGPT Code Interpreter, or similar):

**Prompt:**
```
Show me your current working directory using pwd.
What directory are you in right now?
```

**Expected AI Output:**
```
Let me check my location.

$ pwd
/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python
```

The AI responds with the same command you just ran—because `pwd` works the same way for both of you.

### Step 3: Compare and Understand

**Your output**: `/Users/yourname/Documents`
**AI's output**: `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python`

Both are file paths. They're in different locations because you're on different computers. But the **command is identical**, and the **output means the same thing**: "Here's where I'm working right now."

**Key insight**: Your AI doesn't have secret powers. It runs the same commands you do. When you ask "Where are you?", the AI translates that to `pwd` and shows you the output. You understand the output because you already ran it yourself.

#### 💬 AI Colearning Prompt
> "Why does pwd show `/Users/yourname` on Mac but `/home/yourname` on Linux?"

---

## Use `ls` to See What Files Are in a Directory

The `ls` command lists files and folders in your current directory.

### Step 1: You Try It

In the same terminal, type:

```bash
$ ls
Desktop
Documents
Downloads
Library
Applications
README.md
```

You see a simple list. Some items are **folders** (like `Desktop`, `Documents`), and some are **files** (like `README.md`). How do you tell the difference? Look at the output carefully—or use a flag to make it clearer.

Try this command to see **more detail**:

```bash
$ ls -la
drwxr-xr-x   5 user  staff   160 Nov  2 03:06 .
drwxr-xr-x  20 user  staff   640 Nov  2 02:00 ..
-rw-r--r--   1 user  staff  2819 Oct 31 16:16 README.md
drwxr-xr-x  11 user  staff   352 Oct 31 16:16 Desktop
drwxr-xr-x  15 user  staff   480 Nov  2 03:06 Documents
drwxr-xr-x   9 user  staff   288 Nov  2 02:36 Downloads
```

**What to notice**:
- Items starting with `d` (like `drwxr-xr-x`) are **directories** (folders)
- Items starting with `-` (like `-rw-r--r--`) are **files**

You just learned to read the output yourself. Now you understand what `ls` is telling you.

### Step 2: Your AI Does the Same

Ask your AI:

**Prompt:**
```
List the files in your current directory using ls -la.
Show me what files and folders you can see from where you are.
```

**Expected AI Output:**
```
$ ls -la
drwxr-xr-x@  13 mjs  staff    416 Nov  2 03:06 .
drwxr-xr-x@   7 mjs  staff    224 Nov  2 03:00 ..
-rw-r--r--@   1 mjs  staff   2819 Oct 31 16:16 CLAUDE.md
-rw-r--r--@   1 mjs  staff   8340 Oct 31 16:16 README.md
drwxr-xr-x@  11 mjs  staff    352 Oct 31 16:16 book-source
drwxr-xr-x@  15 mjs  staff    480 Nov  2 03:06 specs
drwxr-xr-x@   9 mjs  staff    288 Nov  2 02:36 .claude
drwxr-xr-x@   7 mjs  staff    224 Nov  2 02:47 history
```

Again—same command, same output format. The AI's files are different from yours because it's in a different directory, but the **command and logic are identical**.

### Step 3: Compare and Interpret

**Your output** shows folders like `Desktop`, `Documents` (lines starting with `d`)
**AI's output** shows folders like `book-source`, `specs` (lines starting with `d`)

You can now read both because you know:
- `d` = directory (folder)
- `-` = file

When your AI says "I can see the `book-source` folder and `README.md` file here," you can verify it by looking at the `ls` output. You're not just trusting the AI—you're **reading the evidence yourself**.

#### 🎓 Expert Insight
> In AI-native development, you don't memorize `ls` flags like `-lah` or `-R`. You understand WHAT you need to see ("show hidden files" or "list recursively"), and your AI handles the syntax. Your job: know what question to ask.

---

## Why This Matters: The Supervision Pattern

Here's why understanding location is crucial:

**Without knowing location**, you might ask your AI: "Delete the backup folder."
Your AI might delete the wrong folder—maybe one you didn't intend.

**With knowing location**, you'd ask: "Delete the backup folder. But first, show me where we are and what I'll be deleting."

Then your AI shows you the location and files, and **you confirm** before anything gets deleted.

This is the supervision pattern: **Ask → Show Location → Show What's There → Verify → Execute**.

You're not responsible for remembering commands. You're responsible for saying "Yes, that's the right folder to delete" before your AI proceeds.

#### 🤝 Practice Exercise

> **Ask your AI**: "Navigate to my Documents folder, show me what's there, and identify which items are folders vs files. Then explain how you determined the difference."

**Expected Outcome**: You understand how AI uses `cd`, `ls -la`, and interprets the `d` prefix to identify directories.

---

## Try With AI: Side-by-Side Comparison

Now that you've run `pwd` and `ls` yourself, compare what happens when your AI does the same.

### Comparison Prompt

Open your AI tool (Claude Code, ChatGPT Code Interpreter, or similar) and ask:

**Prompt:**
```
Show me your current working directory using pwd.
Then show me all the files in this directory using ls -la.
```

**What to Compare**:

| Command | Your Computer | Your AI's Computer |
|---------|----------------|-------------------|
| `pwd` output | `/Users/yourname/Documents` | (AI's path) |
| `ls -la` output | (Your files and folders) | (AI's files and folders) |

**Observation**:
- Are the **commands identical**? (Both run `pwd` and `ls -la`)
- Is the **output format the same**? (Both show file paths and file listings)
- Are the **files different**? (Yes—different computers, different locations)

**Key Insight**: Your AI isn't using magic. It's running the same commands you ran. You can read and verify its output because you already understand what `pwd` and `ls` mean.

---

**Expected Response**:
Your AI will explain the supervision pattern:
1. Show location with `pwd`
2. List files with `ls -la`
3. Wait for you to confirm
4. Then perform the operation

**Why This Matters**: This is the foundation of safe collaboration. You're not blindly trusting AI—you're **verifying location and contents before any action**.
