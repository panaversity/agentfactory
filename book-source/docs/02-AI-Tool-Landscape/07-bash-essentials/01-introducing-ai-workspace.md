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

## Your AI Has a Location—And You Need to Know It

Imagine hiring a contractor to renovate your house. Before they swing a hammer, you'd ask: "Where are you starting? Show me what you're looking at." You wouldn't let them work blind, and you wouldn't work blind either.

This lesson teaches you the same habit for working with your AI companion in the terminal. Your AI assistant has a **current location** in your computer's file system, and understanding that location is the first step to supervising its work safely. You're not learning bash commands. You're learning to have effective conversations with your AI about **where it is and what it can see**.

By the end of this lesson, you'll be able to ask your AI companion three simple questions:
1. **Where are you working right now?** (It will show you the path)
2. **What files can you see here?** (It will list them for you)
3. **Why does location matter?** (It affects what files it can access and operate on)

---

## Example 1: Where Is Your AI Working?

Let's look at a real conversation. You open your AI tool (Claude Code, ChatGPT Code Interpreter, or Gemini CLI) and ask a natural question:

**You**: "Show me your current directory path. Where are you working right now?"

**Agent**: Let me check. I'll execute `pwd` to show you my location.

```
$ pwd
/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python
```

**Agent**: "I'm currently in `/Users/mjs/Documents/code/panaversity-official/tutorgpt-build/colearning-python`. This is the root of the CoLearning Python project. This is where I'll work when you ask me to edit files or run commands."

---

## Understanding the Conversation

In that dialogue, three important things happened:

**1. You Made a Request** (in plain language)
Your question wasn't "Run pwd." You asked "Where are you working?" Your AI companion understood your intent and decided the right command to show you the answer.

**2. Your AI Executed a Command** (you didn't have to type it)
The `pwd` command is bash-speak for "print working directory." But you didn't need to know that phrase. Your AI translated your English request into the right bash command.

**3. Your AI Explained the Output** (it didn't just show you raw output)
The path `/Users/mjs/Documents/...` is the "current directory"—the folder where your AI is currently working. Think of it like your AI saying "I'm standing here in this folder." All files it can see and operations it performs will happen relative to this location.

**Your Job**: Understand what location your AI is in. **Not** to memorize the pwd command. **Not** to type the command yourself. Just to supervise and verify: "Yes, you're in the right place."

---

## Example 2: What Files Can You See?

Now you ask a follow-up question:

**You**: "What files are in this project? Show me what you can see here."

**Agent**: I'll list the files in this directory with `ls -la` to show you everything, including hidden files.

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

**Agent**: "Here's what I can see from this location. The items starting with `d` are **directories** (folders). The items starting with `-` are **files**. For example:
- `book-source` is a folder containing the lesson content
- `specs` is a folder containing project specifications
- `CLAUDE.md` and `README.md` are text files

When you ask me to work with something in this project, it will be one of these items or something inside these folders."

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

## Exercise 1: Predict Before You Look

Before your AI shows you a file listing, try to predict what you'll see. This builds your understanding of what these commands do.

**Setup**: Imagine your AI is in the folder `/Users/mjs/Documents/projects/python-app/`

**Prediction**: What types of items would you expect to see in a Python project folder? (Hint: think about Python files, configuration files, folders for source code, tests, etc.)

**Then Your AI Executes**:

```
$ ls -la
total 48
-rw-r--r--   1 user  staff   45 Nov  1 12:00 README.md
-rw-r--r--   1 user  staff  120 Nov  1 12:00 requirements.txt
drwxr-xr-x   3 user  staff   96 Nov  1 12:00 src
drwxr-xr-x   3 user  staff   96 Nov  1 12:00 tests
-rw-r--r--   1 user  staff   89 Nov  1 12:00 .gitignore
```

**Reflection**: Did your prediction match? What surprised you? Why is `README.md` a file but `src` is a folder?

---

## Exercise 2: Reading a File Listing Dialog

You'll often see your AI show you file listings as part of a larger conversation. Practice interpreting what you see.

**Read this dialogue:**

```
You: "Show me what files are in my home directory."

Agent: I'll list them for you.

$ ls
Desktop
Documents
Downloads
Library
Applications
.bashrc
.ssh
```

**Questions to Answer**:
1. How many folders are in this listing? (Hint: these are the items without file extensions)
2. How many configuration files are there? (Hint: these start with a dot: `.`)
3. If the user asks "Where are my downloads?", what would you suggest?

---

## Exercise 3: Safety Check—Why Confirm Location?

This is a critical habit. Before your AI performs any operation on files, it should show you where it is.

**Scenario**: Your AI says "I'll delete the old_code folder to clean up your project."

**Before it executes**, what should you do?
1. Ask: "Where are you right now?"
2. Ask: "Show me what files are in this directory"
3. Verify: "Yes, I can see old_code. That's the one I want to delete."

**Why this matters**: If your AI is in the wrong directory, it might delete a folder you didn't intend. By checking location first, you supervise the operation and prevent mistakes.

Write a short dialogue (3-4 exchanges) where you ask your AI to delete a folder, but you first confirm the location before execution.

---

## Formative Assessment: Can You Predict What pwd Shows?

Pause here and test your understanding.

**Question 1**: Your AI opens a terminal. You ask "Where are you?" What will the response show you?
- A) A list of files and folders
- B) The path to the current directory (like `/Users/yourname/Documents`)
- C) A list of commands available in the terminal

*Correct answer: B. The pwd command shows the path—where your AI is currently working.*

**Question 2**: Your AI executes `ls` and shows output starting with `d` characters. What does the `d` mean?
- A) A deleted file
- B) A directory (folder)
- C) A default file

*Correct answer: B. The `d` prefix indicates a directory/folder.*

**Question 3**: Why is it important to ask "Where are you?" before asking your AI to delete files?
- A) It's just a polite question
- B) It confirms your AI is in the right location before performing a destructive operation
- C) It has no practical purpose; you can skip it

*Correct answer: B. Confirming location prevents accidental deletions of wrong folders.*

---

## Try With AI

**Tool**: Claude Code, ChatGPT Code Interpreter, or your preferred AI companion

**Setup**: Open your AI tool. You're about to ask it about its workspace.

### Prompt 1: Show Me Your Workspace

Copy and paste this prompt (or use it as inspiration for your own words):

```
Show me your current working directory using pwd.
Then show me all the files in this directory using ls.
Explain what the pwd and ls commands are showing me.
```

**Expected Outcome**: You should see:
- A file path (the current directory)
- A list of files and folders in that location
- Your AI's explanation of what each means

**Why This Matters**: This mirrors the real workflow. You ask a question. Your AI uses the right commands. It explains the output in plain language. You understand what's happening without needing to memorize bash syntax.

### Prompt 2: Verify Understanding

After seeing the output, ask a follow-up:

```
Now I want to navigate to [pick any folder you saw in the listing].
Show me the path and files there. Explain why you moved to this location.
```

**Expected Outcome**: Your AI shows a new path, new files, and explains the relationship. You're building a mental map of your file system through conversation.

### Optional: Safety Conversation

If you want to practice the supervision habit:

```
Imagine I asked you to delete a folder. Before you did that,
how would you make sure we're deleting the right one?
Walk me through the steps you'd take to verify it's safe.
```

**Expected Outcome**: Your AI describes showing the location, asking for confirmation, maybe suggesting a backup—the safety-first pattern that you'll formalize in the next lesson.
