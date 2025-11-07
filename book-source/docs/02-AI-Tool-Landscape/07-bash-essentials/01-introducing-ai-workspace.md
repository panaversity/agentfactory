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

## Your AI Has a Location—And You Need to Know It

This lesson teaches you the same habit for working with your AI companion in the terminal. Your AI assistant has a **current location** in your computer's file system, and understanding that location is the first step to supervising its work safely. You're not learning bash commands. You're learning to have effective conversations with your AI about **where it is and what it can see**.

By the end of this lesson, you'll be able to ask your AI companion three simple questions:
1. **Where are you working right now?** (It will show you the path)
2. **What files can you see here?** (It will list them for you)
3. **Why does location matter?** (It affects what files it can access and operate on)

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

---

## Why This Matters: The Supervision Pattern

Here's why understanding location is crucial:

**Without knowing location**, you might ask your AI: "Delete the backup folder."
Your AI might delete the wrong folder—maybe one you didn't intend.

**With knowing location**, you'd ask: "Delete the backup folder. But first, show me where we are and what I'll be deleting."

Then your AI shows you the location and files, and **you confirm** before anything gets deleted.

This is the supervision pattern: **Ask → Show Location → Show What's There → Verify → Execute**.

You're not responsible for remembering commands. You're responsible for saying "Yes, that's the right folder to delete" before your AI proceeds.

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
