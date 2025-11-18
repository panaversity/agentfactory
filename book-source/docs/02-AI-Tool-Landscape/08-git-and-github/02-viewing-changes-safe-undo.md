---
sidebar_position: 2
title: "Viewing Changes & Safe Undo"
description: "Discover safe error recovery through deliberate mistake exploration—learn git diff, git restore, and git reset to undo changes fearlessly"
duration_minutes: 45

# HIDDEN SKILLS METADATA
skills:
  - name: "View Changes with git diff"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Competence"
    measurable_at_this_level: "Student can execute git diff and interpret output showing added/removed lines"

  - name: "Discard Changes with git restore"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Competence"
    measurable_at_this_level: "Student can restore modified files to committed state without permanent loss"

  - name: "Unstage Files with git reset"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Competence"
    measurable_at_this_level: "Student can move staged files back to working directory state"

  - name: "Undo Strategy Selection"
    proficiency_level: "A1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Critical Thinking"
    measurable_at_this_level: "Student can distinguish when to use diff/restore/reset based on scenario (unstaged/staged/committed)"

learning_objectives:
  - objective: "View changes using git diff and interpret diff output showing additions and deletions"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Student modifies file, executes git diff, and explains what + and - lines represent"

  - objective: "Discard unwanted unstaged changes using git restore"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Student makes mistake in file, uses git restore to recover, verifies recovery with git status"

  - objective: "Unstage accidentally staged files using git reset HEAD <file>"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Student stages wrong file, uses git reset HEAD to unstage, confirms file still exists"

  - objective: "Distinguish destructive vs non-destructive undo operations and know which command applies"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Student identifies correct undo command (diff/restore/reset) for given scenario"

cognitive_load:
  new_concepts: 4
  concepts_list:
    - "Diff (change visualization with + and - lines)"
    - "Restore (non-destructive undo for working directory changes)"
    - "Reset (non-destructive undo for staging area)"
    - "Undo strategy awareness (destructive vs non-destructive commands)"
  assessment: "4 concepts (within A1 limit of 5-7) ✓"

teaching_approach: "Hands-on discovery through deliberate error recovery (Execute → Observe Error → Understand Recovery → Apply to Staging)"
modality: "Discovery-based error recovery (varying from Lesson 1's basic execution) ✓"
# stage: "1 (Manual Foundation - NO AI assistance for Git operations)" # Internal scaffolding - hidden from students
# ai_involvement: "None for Git execution (Stage 1 requirement)" # Internal scaffolding - hidden from students

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/028-chapter-8-git-redesign/spec.md (US1 Priority P1)"
source_plan: "specs/028-chapter-8-git-redesign/plan.md (Lesson 2 details, lines 141-203)"
source_tasks: "specs/028-chapter-8-git-redesign/tasks.md (T019-T029)"
created: "2025-01-17"
last_modified: "2025-01-17"
version: "1.0.0"
---

# Viewing Changes & Safe Undo

## Fearless Experimentation Through Error Recovery

In Lesson 1, you discovered how commits save your work as "save points." But what happens between commits? You modify files, make mistakes, realize your approach is wrong. How do you recover?

**The question this lesson answers**: When you've edited a file and realize the changes are bad, how do you get back to the working version? When you've staged the wrong files, how do you undo staging without losing the files?

**What makes this possible**: Three Git commands that let you **explore mistakes safely** and recover from them:
- **`git diff`**: See what you changed
- **`git restore`**: Throw away changes you don't want
- **`git reset HEAD <file>`**: Unstage files you staged by accident

These commands are your **safety net for fearless AI experimentation**. Imagine you ask Claude Code to refactor your entire codebase. With these undo commands, you can commit first, let AI make changes, see what it did with `git diff`, and instantly restore to the working version if it went wrong.

---

## Phase 1: Visualizing Changes with `git diff`

### Setup: Create a Committed File

Let's start with a working project from Lesson 1. If you don't have one:

```bash
# Create a new folder for this lesson
mkdir git-lesson-2
cd git-lesson-2

# Initialize Git
git init

# Create a simple file
cat > shopping-list.txt << 'EOF'
Shopping List
=============

Groceries:
- Milk
- Eggs
- Bread
EOF

# Stage and commit
git add shopping-list.txt
git commit -m "Initial shopping list"
```

**What these commands do:**
- `mkdir git-lesson-2` and `cd git-lesson-2` = create and enter new folder
- `git init` = initialize Git in this folder
- `cat > shopping-list.txt << 'EOF'` = create a file with multiple lines of text
  - `cat >` = create new file and put text into it
  - `<< 'EOF'` = everything until "EOF" goes into the file (this lets you type multiple lines)
- `git add` and `git commit` = save this file as a commit (from Lesson 1)

**Verify**: Run `git log` to see your commit.

### Activity 1: Modify a File and View Changes

Now you'll intentionally change the file and observe what changed using `git diff`.

**Execute**:

```bash
# Modify the shopping-list.txt file
cat >> shopping-list.txt << 'EOF'

Household:
- Dish soap
- Paper towels
EOF

# Check the status
git status
```

**What this command does:**
- `cat >>` = **append** text to end of existing file (the `>>` means "add to end", not "replace")
- `<< 'EOF'` = same multi-line input technique (everything until "EOF")
- Result: adds household items to the shopping list without replacing groceries

**Observe**: You should see:
```
On branch master

Changes not staged for commit:
  (use "git add <file>..." to stage them)
  (use "git restore <file>..." to discard changes in working directory)
	modified:   shopping-list.txt
```

**Discovery Question**: "Git knows the file changed. But what exactly changed? How can I see the differences?"

### Activity 2: View Exact Changes with `git diff`

**Execute**:

```bash
# View changes
git diff shopping-list.txt
```

**Observe**: You'll see output like this:

```diff
diff --git a/shopping-list.txt b/shopping-list.txt
index abc1234..def5678 100644
--- a/shopping-list.txt
+++ b/shopping-list.txt
@@ -6,0 +6,5 @@ Groceries:
 - Bread
+
+Household:
+- Dish soap
+- Paper towels
```

**Breaking Down the Diff Output**:
- **`---` line**: What the file looked like in the last commit
- **`+++` line**: What the file looks like now
- **Green `+` lines**: New lines you added
- **Red `-` lines**: Lines you removed (we don't have any yet)
- **White lines**: Lines that didn't change (context)

**What You Learned**: `git diff` shows you exactly what changed since the last commit. No more guessing—you can see every addition and deletion.

#### 💬 AI Colearning Prompt
> "Explain how `git diff` output uses `+` and `-` symbols to show changes, and why seeing changes before committing prevents mistakes."

**Validation Checkpoint**: Can you identify which lines are new additions? (The lines with `+` prefix)

---

## Phase 2: Error Scenario - Deliberate Mistake

Now let's create an intentional mistake and discover how to recover from it.

### Activity 3: Introduce a Syntax Error

**Execute**:

```bash
# Add bad formatting to shopping-list.txt (messy content on purpose)
cat >> shopping-list.txt << 'EOF'

This line has no formatting and breaks the structure
ALLCAPS AND CONFUSING TEXT
!!!random symbols@@@###
EOF

# View the broken file
cat shopping-list.txt
```

**Observe**: Your nicely formatted list is now messy:
```
Shopping List
=============

Groceries:
- Milk
- Eggs
- Bread

Household:
- Dish soap
- Paper towels

This line has no formatting and breaks the structure
ALLCAPS AND CONFUSING TEXT
!!!random symbols@@@###
```

**Discovery Question**: "Oh no! I broke the file. How do I get back to the version that worked?"

### Activity 4: View All Changes Including the Mistake

**Execute**:

```bash
# View all changes since last commit
git diff shopping-list.txt
```

**Observe**: The diff shows all your changes, including the messy text:

```diff
+This line has no formatting and breaks the structure
+ALLCAPS AND CONFUSING TEXT
+!!!random symbols@@@###
```

**What You Learned**: `git diff` lets you see the bad changes before committing. This is your **inspection checkpoint** before saving.

---

## Phase 3: Understanding Recovery Paths

### Two Ways to Undo: Manual Edit vs. `git restore`

You have two options:

**Option A: Manually Edit the File**
- Open the file in your editor
- Delete the bad lines
- Save
- Verify with `git diff` again

**Option B: Use `git restore`**
- Let Git throw away all changes
- Instantly return to the last committed version
- Much faster and safer

### Activity 5: Discover `git restore`

**Execute**:

```bash
# Throw away all unstaged changes
git restore shopping-list.txt

# Verify the file is restored
cat shopping-list.txt
```

**Observe**: The file is back to the clean version! The messy text is gone.

**Verify**:

```bash
# View diff again
git diff shopping-list.txt
```

**Observe**: No output. This means the file matches the last commit exactly.

**Confirm with status**:

```bash
git status
```

**Observe**: The file is no longer listed as modified.

```
On branch master
nothing to commit, working tree clean
```

**What You Learned**: `git restore` is your **instant undo** for unstaged changes. It's non-destructive (the file still exists, you're just reverting to the last saved version).

#### 🎓 Expert Insight
> In AI-native development, `git restore` is your safety net when AI generates broken code. You don't need to manually revert changes—Git instantly recovers the working version. This enables fearless experimentation.

**Key Insight**: Unlike closing a document without saving in a word processor, `git restore` recovers the working version from your last commit. This is incredibly powerful—you can edit fearlessly knowing you can always go back.

---

## Phase 4: Undoing Staged Changes

Now let's explore the **staging area** recovery. This scenario happens when you accidentally stage the wrong file.

### Activity 6: Create and Stage a File You'll Change Your Mind About

**Execute**:

```bash
# Create a new file
cat > notes.txt << 'EOF'
These are my rough notes.
I don't want this in the project yet.
EOF

# Stage it by accident
git add notes.txt

# Check status
git status
```

**Observe**: You see:
```
On branch master

Changes to be committed:
  (use "git restore --staged <file>..." to unstage)
	new file:   notes.txt
```

**Discovery Question**: "Wait, I didn't mean to stage this file. How do I unstage it without losing it?"

### Activity 7: Discover `git reset HEAD <file>`

**Execute**:

```bash
# Unstage the file
git reset HEAD notes.txt

# Check status
git status
```

**Observe**: The file is still there, but no longer staged:

```
On branch master

Untracked files:
  (use "git add <file>..." to track them)
	notes.txt

nothing added to commit but untracked files exist
```

**Verify the file exists**:

```bash
# View its contents
cat notes.txt
```

**Observe**: The file contents are **exactly as you left them**. Nothing was lost.

**What You Learned**: `git reset HEAD <file>` unstages files without deleting them. The file returns to "untracked" or "modified" status, but it's still there. This is **non-destructive undo** for the staging area.

#### 🤝 Practice Exercise

> **Ask your AI**: "Create a Git scenario where I accidentally stage 5 files but only want to commit 3 of them. Show me how to unstage the 2 unwanted files, then explain the difference between unstaging and deleting."

**Expected Outcome**: You'll understand that `git reset HEAD` only changes staging status—files remain unchanged in your working directory.

---

## Understanding: Three Undo Commands and When to Use Each

You now know three Git commands. They have different purposes:

### Decision Tree: Which Command for Your Scenario?

**Scenario 1: You modified a file and want to see what changed**
```
Command: git diff <filename>
Effect: Shows the changes (+ and - lines)
Data Loss: None—just viewing
When to use: Before deciding if changes are good or bad
```

**Scenario 2: You modified a file and want to throw away changes**
```
Command: git restore <filename>
Effect: Undoes unstaged changes, returns file to last commit
Data Loss: None—file still exists, just reverted to committed version
When to use: After seeing bad changes in git diff, want instant undo
```

**Scenario 3: You staged a file by accident**
```
Command: git reset HEAD <filename>
Effect: Unstages file, returns to working directory state
Data Loss: None—file still exists and unchanged
When to use: Realized you staged the wrong file, want to keep working on it
```

**Scenario 4: You created a commit you want to undo** ⚠️ CAUTION
```
Command: git reset --hard HEAD~1
Effect: Deletes the last commit AND all its changes
Data Loss: YES—commit is gone forever
When to use: Only if the commit is recent and unpushed to GitHub
⚠️ WARNING: This is DESTRUCTIVE. Use only when absolutely sure.
```

### Key Distinction: Non-Destructive vs. Destructive

- **`git diff`**: Non-destructive (viewing only)
- **`git restore`**: Non-destructive (reverts to committed version, doesn't delete)
- **`git reset HEAD <file>`**: Non-destructive (unstages without deleting)
- **`git reset --hard`**: **DESTRUCTIVE** (deletes commit and changes)

**Why this matters for AI safety**: When AI generates code, you want non-destructive undo commands so you can recover instantly. The destructive commands are for nuclear options only.

---

## Try With AI

**Setup**: Open ChatGPT (chat.openai.com) and explore how AI understands these undo commands.

**Prompt Set**:

**Prompt 1 (Conceptual)**:
```
I accidentally staged the wrong files in Git.
I used: git reset HEAD wrongfile.txt

Does this command delete the file?
Explain what happens to the file after this command.
```

**Expected Outcome**: ChatGPT should explain that `git reset HEAD` only unstages the file—the file still exists and is unchanged. Compare its answer to what you learned in Activity 7. Does ChatGPT understand that this is non-destructive? If it says the file is deleted, that's a mistake—correct it by telling ChatGPT "Actually, the file still exists, just unstaged."

---

**Prompt 2 (Scenario-based)**:
```
I'm working with ChatGPT to edit my project files.
ChatGPT made changes that broke my project.
I already committed the broken changes to Git.

I want to undo this commit and go back to the working version.

What Git command should I use?
Is it safe to use?
What will happen to the broken changes?
```

**Expected Outcome**: ChatGPT should suggest `git reset --hard HEAD~1` or similar, and explain that this is destructive (the broken code disappears). Compare this to what you learned in Phase 3—this is the nuclear option, destructive undo for committed changes. Good to know, but we usually prefer `git restore` for uncommitted changes.

---

**Prompt 3 (Practical)**:
```
I edited three files for my project.
- notes.txt: I'm happy with these changes
- draft.txt: This has mistakes, I want to throw away changes
- temp.txt: I staged this by accident, I want to unstage it

For each file, tell me the Git command I should use.
Explain why each command is the right choice.
```

**Expected Outcome**: ChatGPT should distinguish:
- notes.txt: No command needed, keep the changes
- draft.txt: `git restore draft.txt` (unstaged changes undo)
- temp.txt: `git reset HEAD temp.txt` (unstage)

Verify that ChatGPT understands the distinction between unstaging (reset) and discarding changes (restore). Remember: `git diff`, `git restore`, and `git reset HEAD` are safe commands—they won't permanently delete anything (unlike `git reset --hard`).
