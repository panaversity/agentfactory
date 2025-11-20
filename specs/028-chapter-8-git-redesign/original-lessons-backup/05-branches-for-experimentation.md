---
sidebar_position: 5
title: "Branches for Experimentation"
description: "Test AI changes safely without risking your main code"
duration_minutes: 20

# HIDDEN SKILLS METADATA
skills:
  - name: "Create Feature Branch"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can create a branch and test changes safely"

  - name: "Test Before Merging"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can decide to merge or discard based on tests"

  - name: "Understand Branching Metaphor"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain why branches are essential for AI work"

learning_objectives:
  - objective: "Create a safe testing space with branches"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student creates and uses branches"

  - objective: "Test AI changes without affecting main code"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student tests on branch, then merges or discards"

  - objective: "Explain why branches make AI work safer"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Student explains safety benefits"

cognitive_load:
  new_concepts: 4
  assessment: "4 concepts (branch, test, merge, discard) within A2 limit ✓"

# Generation metadata
generated_by: "content-implementer"
source_spec: "specs/012-chapter-8-git-github-aidd/plan.md"
created: "2025-11-05"
last_modified: "2025-11-07"
version: "3.0.0"
---

# Branches for Experimentation

Your AI wants to refactor your entire codebase - 50+ changes across 10 files. You think: "What if it breaks everything?"

**Solution**: Create a branch. Test changes there. If good, merge. If bad, delete. Your main code stays safe.

**Time**: 20 minutes

---

## What Is a Branch?

A branch is a copy of your code where you test things safely.

```
Main Code (working version)
  ↓
Create Branch → Test Copy
  ↓
AI makes changes here
  ↓
Test it
  ↓
Works? → Merge back to main
Broken? → Delete the branch
```

Your main code never changes until you approve.

---

## Why Branches Matter with AI

**Without branches**:
- AI changes your main code directly
- Something breaks
- Hard to undo everything

**With branches**:
- AI changes the copy (branch)
- Test the copy
- If broken, delete it
- Main code untouched
- Try again differently

**Branches = Fearless experimentation**

---

## The Branch Workflow

### Step 1: Create a Branch

**You ask Gemini CLI**: "Create a branch called 'test-refactor'"

Gemini runs: `git checkout -b test-refactor` (or `git switch -c test-refactor`)

Creates and switches to the new branch.

**Check it worked**: Ask "Which branch am I on?" - should show `test-refactor`

---

### Step 2: Make Changes

Now AI can modify code on this branch. Your main code is protected.

**You**: "Refactor calculator.py to be faster"

AI modifies the code on the `test-refactor` branch.

**View changes**:

**Ask Gemini CLI**: "Show me what changed"

Gemini runs: `git diff main`

Shows differences between your branch and main.

---

### Step 3: Test the Changes

Test thoroughly before merging.

**You**: "Run tests on calculator.py"

AI runs your code and reports results:
- ✓ All tests pass → Safe to merge
- ✗ Tests fail → Fix or discard

---

### Step 4: Merge or Discard

**Option A: Keep Changes (Merge)**

If tests pass and you like the changes:

**You ask Gemini CLI**: "Merge this branch into main"

Gemini runs:
1. `git checkout main` (switch to main)
2. `git merge test-refactor` (merge changes)
3. `git branch -d test-refactor` (delete the branch)

Your main code now has the changes.

---

**Option B: Discard Changes**

If tests fail or you don't like the changes:

**You ask Gemini CLI**: "Delete this branch without merging"

Gemini runs:
1. `git checkout main` (switch to main)
2. `git branch -D test-refactor` (force delete branch)

Your main code is unchanged - exactly as before.

---

## Complete Example

**You**: "Create a branch to test adding NumPy to calculator"

Gemini runs: `git checkout -b try-numpy`

Branch created.

**You**: "Rewrite add() to use NumPy"

AI modifies `calculator.py` on the `try-numpy` branch.

**You**: "Test it"

AI runs tests → ⚠️ Crashes with empty lists

**You**: "Fix the empty list bug"

AI adds validation.

**You**: "Test again"

AI runs tests → ✓ All pass! 5x faster.

**You**: "Merge to main"

Gemini runs:
- `git checkout main`
- `git merge try-numpy`
- `git branch -d try-numpy`

Main code now uses NumPy safely.

---

## Working with Multiple Branches

**List all branches**:

**Ask Gemini CLI**: "Show me all branches"

Gemini runs: `git branch`

Shows all branches, with `*` marking your current one.

**Switch between branches**:

**Ask Gemini CLI**: "Switch to main branch"

Gemini runs: `git checkout main` (or `git switch main`)

**See current branch**:

**Ask Gemini CLI**: "Which branch am I on?"

Gemini runs: `git branch --show-current`

---

## When to Use Branches

**Use a branch when**:
- AI wants to make major changes
- Trying something experimental
- Not sure if changes will work
- Multiple people working on same code

**Don't need a branch for**:
- Tiny changes (fixing typos)
- Very simple updates
- Learning/practicing locally

**Rule**: If you're nervous about changes, use a branch.

---

#### 💬 AI Colearning Prompt
> "Why doesn't Git just let you undo changes on the main branch instead of requiring branches? What problem do branches solve that undo doesn't?"

---

## Key Commands Reference

| Task | Command |
|------|---------|
| Create and switch to branch | `git checkout -b branch-name` |
| List all branches | `git branch` |
| Switch to branch | `git checkout branch-name` |
| See current branch | `git branch --show-current` |
| Merge branch to main | `git checkout main` then `git merge branch-name` |
| Delete branch (after merge) | `git branch -d branch-name` |
| Force delete branch (no merge) | `git branch -D branch-name` |
| Compare branch to main | `git diff main` |

You don't need to memorize these - ask Gemini CLI in natural language.

---

## Safety Guidelines

**Always**:
- Test on a branch before merging
- Commit your work on the branch
- Check which branch you're on before making changes

**Never**:
- Make untested changes directly on main
- Delete a branch before confirming it's merged (if you want to keep changes)

**The pattern**:
1. Create branch
2. Make changes
3. Test thoroughly
4. If good → merge
5. If bad → delete
6. Main stays safe

---

#### 🎓 Expert Insight
> In AI-native development, you don't memorize branch commands—you understand experimentation safety. When AI suggests major changes, you specify "test this on a branch first" and AI handles `git checkout -b / merge / branch -d`. Your value is recognizing when changes need isolation (nervous about outcome = use branch), not typing branch syntax.

---

#### 🤝 Practice Exercise

> **Ask your AI**: "Create a test branch, make a small change, show me the diff compared to main, then merge it back. Explain what each step protects against."

**Expected Outcome**: You'll understand branches aren't about command syntax—they're about risk management. Test on branch = main protected. Works? Merge. Breaks? Delete. Your main code never at risk.

---

## Try With AI

Practice the complete branch workflow.

**Tool**: Gemini CLI (or Claude Code, ChatGPT)

### Exercise 1: Complete Branch Workflow

```
Walk me through this workflow:
1. Create a branch called "test-feature"
2. Make a small change to a file
3. Show me what changed compared to main
4. Test the change
5. Merge back to main
6. Delete the branch
7. Confirm I'm back on main

Explain each step as we go.
```

### Exercise 2: Discard a Branch

```
Practice discarding:
1. Create a branch called "experiment"
2. Make a change I'll discard
3. Switch back to main without merging
4. Delete the experiment branch
5. Prove my main branch is unchanged
```

### Exercise 3: Multiple Branches

```
Create two branches: "feature-a" and "feature-b"
Show me how to:
1. Switch between them
2. See which branch I'm on
3. List all branches
4. Delete both branches safely
```

### Exercise 4: Recovery Scenario

```
Scenario: I made changes on main by accident (should've used a branch).
How do I move those changes to a new branch and restore main?
Walk me through the recovery steps.
```
