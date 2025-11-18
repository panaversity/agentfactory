---
sidebar_position: 4
title: "Safety Net: Undoing Changes"
description: "Learn how to undo mistakes safely with Git and AI"

# HIDDEN SKILLS METADATA
skills:
  - name: "View File Changes"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can ask AI to show what changed"

  - name: "Undo Uncommitted Work"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Information & Data Literacy"
    measurable_at_this_level: "Student can discard changes with AI help"

  - name: "Undo Committed Work Safely"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Information & Data Literacy"
    measurable_at_this_level: "Student can undo commits safely with AI guidance"

  - name: "Recognize Danger Zones"
    proficiency_level: "A1"
    category: "Soft"
    bloom_level: "Understand"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student knows when undoing is dangerous"

learning_objectives:
  - objective: "View changes before deciding to keep or undo"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Student asks AI to show changes"

  - objective: "Safely discard unwanted changes"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Student undoes changes with AI help"

  - objective: "Identify safe vs dangerous undo methods"
    proficiency_level: "A1"
    bloom_level: "Understand"
    assessment_method: "Student can explain which methods are safe"

cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (view, discard, undo commit, revert, danger zones) within A2 limit ✓"

# Generation metadata
generated_by: "content-implementer"
source_spec: "specs/012-chapter-8-git-github-aidd/plan.md"
created: "2025-11-05"
last_modified: "2025-11-07"
version: "3.0.0"
---

# Safety Net: Undoing Changes

Your AI refactored code. You tested it. Something broke. Now what?

**Good news**: Git lets you undo anything.

This lesson teaches you how to go back when things go wrong.

**Time**: 20 minutes

---

## Why Undo Matters with AI

AI makes changes fast. Sometimes:
- Code looks good but breaks things
- You commit something and realize it's wrong
- You want to try AI's idea but keep your old version

Git has your back. You can always undo.

---

## Three Undo Methods

**1. View Changes First**
See what changed before deciding to keep or undo.

**2. Discard Uncommitted Changes**
Throw away work you haven't saved yet (not committed).

**3. Undo a Commit**
Reverse something you already saved.

---

## Method 1: View What Changed

See exactly what's different before deciding.

**You ask Gemini CLI**: "Show me what changed since my last commit"

Gemini runs: `git diff`

Shows which files were modified, what lines were added/removed, and a summary of changes.

**Check specific files**:

**Ask Gemini CLI**: "Show me the changes in calculator.py"

Gemini runs: `git diff calculator.py`

Shows line-by-line differences for that specific file.

---

**Why this matters**:

You see what changed before deciding what to keep or undo. Never undo blindly.

---

## Method 2: Discard Uncommitted Changes

Throw away changes you haven't committed yet.

**You ask Gemini CLI**: "Discard all my changes and go back to the last commit"

Gemini runs: `git restore .` (or `git checkout .`)

All uncommitted work is deleted. Files return to their last committed state.

**Check it worked**: Ask "Show me the status" - should say "nothing to commit, working tree clean"

---

**Discard specific files**:

**Ask Gemini CLI**: "Discard changes to calculator.py only"

Gemini runs: `git restore calculator.py`

Only that file reverts. Other changes remain.

---

**⚠️ Safety Warning**:

This **deletes your work**. Uncommitted changes are gone forever.

✅ **When it's safe**: You haven't committed yet, so you can ask AI to regenerate the code if needed.

---

## Method 3: Undo a Commit

You committed something and now need to undo it. You have three options:

### Option A: Undo but Keep Changes (Safest)

**You ask Gemini CLI**: "Undo my last commit but keep the changes"

Gemini runs: `git reset --soft HEAD~1`

The commit is removed, but your code remains (unstaged).

Now you can:
- Fix the code
- Stage and commit again

✅ **Very safe**: Nothing is deleted. You can fix and recommit.

---

### Option B: Create a Reverse Commit (Safest for Shared Code)

Use this if you already pushed to GitHub.

**You ask Gemini CLI**: "Create a new commit that reverses my last commit"

Gemini runs: `git revert HEAD`

Creates a "revert commit" that undoes the changes.

Result:
- Bad commit stays in history
- New commit undoes it
- Everything is transparent

✅ **Safest for teams**: If others downloaded your code, this won't confuse them.

---

### Option C: Permanently Delete Commit (DANGEROUS)

**You ask Gemini CLI**: "Permanently delete my last commit"

Gemini runs: `git reset --hard HEAD~1` (after warning you)

⚠️ **WARNING**: This will permanently delete:
- The commit
- All changes in that commit
- Cannot be recovered

Gemini will warn and suggest safer alternatives before proceeding.

❌ **Use only when**: You committed a password/secret by accident. Then **change the password immediately**.

---

#### 💬 AI Colearning Prompt
> "What's the difference between 'git restore' (undo uncommitted) and 'git revert' (undo committed)? Why does Git have multiple undo methods?"

---

## When to Use Each Method

| Situation | What to Ask Gemini CLI |
|-----------|------------------------|
| AI changed code, not committed, don't like it | "Discard all my changes" |
| Committed but forgot to add a comment | "Undo last commit but keep changes" |
| Pushed bad code to GitHub | "Create a commit that reverses my last commit" |
| Committed a password accidentally | "Permanently delete my last commit" + change password |

---

## Complete Example Workflow

**You ask Gemini CLI**: "Show me what changed in calculator.py"

Gemini shows AI added NumPy optimization.

**You**: "Looks good. Stage and commit with message 'Optimize calculator'"

Commit created.

**You test the code** → It crashes with empty lists

**You ask Gemini CLI**: "Undo that commit but keep the changes so I can fix it"

Commit undone. Code still there (unstaged).

**You**: "Fix the empty list crash"

AI fixes the bug.

**You**: "Commit with message 'Optimize calculator and fix empty list bug'"

New commit created with working code.

---

## Safety Guidelines

**Always**:
- View changes before deciding
- Use "undo but keep changes" for most situations
- Ask Gemini to explain before dangerous operations

**Never**:
- Permanently delete commits unless absolutely necessary
- Undo without checking what you're undoing first

**The pattern**:
1. Try something with AI
2. Test it
3. If broken, undo safely
4. Fix with AI's help
5. Test again
6. Commit when it works

---

#### 🎓 Expert Insight
> In AI-native development, you don't memorize undo command syntax—you understand undo safety levels. When something breaks, you specify the safety level you need: "undo but keep my work" (safest), "create reverse commit" (team-safe), or "delete permanently" (dangerous). AI handles the `git restore/revert/reset` syntax. Your job: choose the right safety level for the situation.

---

#### 🤝 Practice Exercise

> **Ask your AI**: "Create a test scenario: make a change, commit it, then undo it using the safest method. Show me each step and explain why this method is safest."

**Expected Outcome**: You'll understand that safety isn't about memorizing commands—it's about understanding when to keep changes (soft reset), when to preserve history (revert), and when deletion is acceptable (never, unless secrets exposed).

---

## Try With AI

Practice safe undo operations.

**Tool**: Gemini CLI (or Claude Code, ChatGPT)

### Exercise 1: View Changes

```
I modified some files but haven't committed.
Show me what changed and explain the differences.
```

### Exercise 2: Discard Safely

```
I made changes I don't like.
Discard all changes and return to my last commit.
Then verify my working directory is clean.
```

### Exercise 3: Undo Commit (Safe)

```
I just committed but need to make one more change.
Undo my last commit but keep all the changes.
Then show me the status to verify.
```

### Exercise 4: Understanding Safety Levels

```
Explain these three methods:
1. Undo commit but keep changes
2. Create a reverse commit
3. Permanently delete commit

Which is safest? When would I use each?
```

### Exercise 5: Recovery Scenario

```
Scenario: I committed code, pushed to GitHub, then realized it breaks production.
What's the safest way to undo this?
Walk me through the steps.
```
