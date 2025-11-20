---
sidebar_position: 4
title: "Understanding File Operations Safely"
chapter: 7
lesson: 4
duration_minutes: 45

skills:
  - name: "Safe File Operation Dialogue"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety"
    measurable_at_this_level: "Learner can request file operations in plain language; applies safety pattern to copy, move, delete; asks verification questions"

  - name: "Data Loss Prevention Understanding"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Safety"
    measurable_at_this_level: "Learner recognizes destructive operations; suggests backup strategies; understands undo limitations"

  - name: "Verification Through Dialogue"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Safety"
    measurable_at_this_level: "Learner can request before/after verification; interprets directory listings to confirm operations succeeded"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (backup strategy, copy operation, move operation, delete operation, verification) at A2 max limit ✓"
---

# Understanding File Operations Safely

## Before You Move Furniture, Make a Plan

Imagine reorganizing your home. You wouldn't rearrange furniture without knowing:
- What's moving where?
- Can I move it back if I change my mind?
- What if something breaks?

File operations work the same way. Your AI companion can copy, move, and delete files. Your job is to understand the plan, ask safety questions, and verify it worked.

This lesson teaches you to apply the 5-step safety pattern from Lesson 2 to real file operations. By the end, you'll be confident requesting file operations and supervising them safely.

---

## Use `cp` to Copy Files Safely

The `cp` (copy) command creates a copy of a file. This is the **safest** file operation because the original stays in place.

### Example 1: Copy a File

**Step 1: You Try It**

In your terminal, copy a file:

```bash
$ cp README.md README-backup.md
$ ls -la
-rw-r--r--  1 user  staff  2048 Nov  2 README.md
-rw-r--r--  1 user  staff  2048 Nov  2 README-backup.md
```

You now have two copies. The original `README.md` is unchanged. The new file `README-backup.md` is identical.

**What to notice**: You used `cp source destination`. The original file still exists. You can verify with `ls` that both files are there.

### Step 2: Your AI Does the Same

Ask your AI:

**Prompt:**
```
Copy an important file to create a backup.
Show me the cp command you'll use.
Then verify with ls that both the original and the copy exist.
Explain: Why is copy a safe operation?
```

**Expected AI Output:**
```
$ cp project.py project-backup.py
$ ls -la project*
-rw-r--r--  1 user  staff  1024 Nov  2 project.py
-rw-r--r--  1 user  staff  1024 Nov  2 project-backup.py
```

Your AI uses `cp` the same way you did.

### Step 3: Compare and Understand

**Your command**: `cp README.md README-backup.md`
**AI's command**: `cp project.py project-backup.py`

Both used `cp` to create a backup. Both checked with `ls` to verify the copy worked. Both originals are still in place.

**Key insight**: Copy is the safest operation. Nothing is deleted or moved. You can always delete the copy if you don't need it.

#### 💬 AI Colearning Prompt
> "What's the difference between `cp` and `cp -r`? Why does copying folders require a flag but copying files doesn't?"

---

## Use `cp -r` to Copy Entire Folders

The `-r` flag means "recursive"—copy everything inside the folder and all its subfolders.

**Step 1: You Try It**

Copy an entire folder using `cp -r`:

```bash
$ cp -r src/ src-backup/
$ ls -la
drwxr-xr-x  3 user  staff   96 Nov  2 src/
drwxr-xr-x  3 user  staff   96 Nov  2 src-backup/
```

You now have two folders with identical contents.

**What to notice**: You used `cp -r` (with the recursive flag) because folders contain files. Both folders exist. The original `src/` is unchanged.

### Step 2: Your AI Does the Same

Ask your AI:

**Prompt:**
```
Copy an entire folder to create a backup.
Show the cp -r command you'd use.
Then verify with ls that both folders exist.
Explain what the -r flag does and why we need it for folders.
```

**Expected AI Output:**
```
$ cp -r docs/ docs-backup/
$ ls -la
drwxr-xr-x  11 user  staff  352 Nov  2 docs/
drwxr-xr-x  11 user  staff  352 Nov  2 docs-backup/
```

Your AI uses `cp -r` the same way you did.

### Step 3: Compare and Understand

Both you and your AI used `cp -r` to copy entire folders. The **command is identical**. The **flag (`-r`) means the same thing**: copy recursively (everything inside).

---

## Why File Operations Matter for Safety

File operations have different risk levels:

| Operation | Command | Risk | Recovery |
|-----------|---------|------|----------|
| **Copy** | `cp file backup` | Low | Safe—original untouched |
| **Move** | `mv old-name new-name` | Medium | Original location changes |
| **Delete** | `rm file` | High | Difficult to undo |

**Before ANY operation**, you MUST:
1. **Show what files are affected** (list them first)
2. **Understand the command** (ask what it does)
3. **Ask "Can we undo this?"** (backup if risky)
4. **Verify the result** (check with `ls` afterward)

#### 🎓 Expert Insight
> In AI-native development, you don't memorize cp flags like `-r`, `-v`, or `-p`. You understand WHAT you need ("copy recursively with progress" or "preserve permissions"), and AI handles the syntax. Your job: verify the right files were copied to the right place.

---

## Try With AI: Side-by-Side Copy Comparison

Now that you've copied files yourself, compare what happens when your AI copies.

### Comparison Prompt

Open your AI tool and ask:

**Prompt:**
```
Let's practice safe file copying.
1. Show me a file you can copy
2. Copy it using the cp command
3. Verify with ls that both the original and copy exist
4. Explain: Why is copy safer than move or delete?
```

**What to Compare**:

| Step | You Do This | Your AI Does This |
|---|---|---|
| Copy a file | `cp README.md README-backup.md` | (AI's copy command) |
| Verify | `ls -la README*` | `ls -la` (to show both files) |
| Check original | Still exists | Still exists |

**Observation**: Here you taught AI the safety verification pattern—before any file operation, check what exists. AI learns project-specific safety habits from your requirements.

#### 🤝 Practice Exercise

> **Ask your AI**: "Copy my project folder to create a backup called 'project-backup'. Before copying, explain the difference between using `cp` vs `cp -r`, and verify both folders exist after copying."

**Expected Outcome**: You understand recursive copying and can verify successful file operations by checking with `ls`.

---

## Try With AI: Safety Questions Before Operations

Ask your AI:

**Prompt:**
```
Before you perform ANY file operation (copy, move, or delete),
what questions should I ask to stay safe?
Give me a safety checklist I should follow.
```

**Expected Response**:
Your AI will describe asking:
1. "Show me exactly which files will be affected"
2. "What command will you run?"
3. "Can we undo this if something goes wrong?"
4. "Should we create a backup first?"

This is the foundation of safe collaboration—thinking before acting.

**Key Insight**: Through this dialogue, you and AI established a safety protocol together. You specified the questions, AI internalized the pattern. This is collaborative safety culture.

---

