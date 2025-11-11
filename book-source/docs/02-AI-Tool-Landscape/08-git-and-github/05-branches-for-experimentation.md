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
generated_by: "lesson-writer"
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

<Quiz title="Chapter 5 Quiz" questions={[{"question":"What are the six phases of the traditional software development lifecycle mentioned in the chapter?","options":{"a":"Discovery, Design, Development, Debugging, Deployment, Decommissioning","b":"Planning, Design, Implementation, Testing, Deployment, Operations","c":"Requirements, Architecture, Coding, Review, Release, Retirement","d":"Idea, Prototype, Build, Test, Launch, Iterate"},"correct_answer":"b","explanation":"The chapter explicitly lists the phases: \u0027The traditional software development lifecycle looks something like this: Planning → Design → Implementation → Testing → Deployment → Operations.\u0027"},{"question":"How does the AI-augmented approach to the \u0027Planning \u0026 Requirements\u0027 phase differ from the traditional approach?","options":{"a":"AI completely replaces product managers.","b":"AI helps extract requirements, suggest edge cases, and identify inconsistencies before development starts.","c":"The planning phase is skipped entirely.","d":"AI only helps with formatting the requirements document."},"correct_answer":"b","explanation":"The text states the AI-augmented approach includes: \u0027Natural language processing helps extract requirements... AI agents suggest edge cases... Automated analysis identifies inconsistencies and ambiguities...\u0027"},{"question":"In the \u0027Testing \u0026 Quality Assurance\u0027 phase, what is a key benefit of using an AI-augmented approach?","options":{"a":"It eliminates the need for human QA engineers.","b":"It only works for unit tests, not integration tests.","c":"AI can generate comprehensive test suites and identify edge cases that humans might miss.","d":"It makes testing slower but more thorough."},"correct_answer":"c","explanation":"The chapter highlights that with AI, it \u0027generates comprehensive test suites from requirements and code\u0027 and \u0027Automatically identifies edge cases developers didn\u0027t think to test.\u0027"},{"question":"What is the \u0027compounding effect\u0027 of AI transformation across the development lifecycle?","options":{"a":"Each phase becomes more complex and expensive.","b":"Improvements in one phase are isolated and do not affect other phases.","c":"An improvement in an early phase (like planning) leads to benefits and efficiencies in all subsequent phases.","d":"The total time for development increases due to the need for more reviews."},"correct_answer":"c","explanation":"The text explains, \u0027Each phase improvement compounds with others. When AI helps you identify edge cases during planning, you write better requirements. Better requirements lead to better architecture,\u0027 and so on."},{"question":"What is happening to specialized roles like \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 as a result of AI tools?","options":{"a":"These roles are becoming more distinct and siloed.","b":"The boundaries between these roles are blurring as AI enables individuals to handle more responsibilities.","c":"These roles are being completely eliminated.","d":"Only the developer role is changing."},"correct_answer":"b","explanation":"The chapter notes, \u0027The boundaries between \u0027developer,\u0027 \u0027QA engineer,\u0027 and \u0027DevOps engineer\u0027 are blurring. AI tools enable individual contributors to handle responsibilities that previously required dedicated specialists.\u0027"}]} />

