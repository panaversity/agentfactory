---
sidebar_position: 6
title: "Reusable Git Patterns - Creating Your Workflow Skill"
description: "Synthesize recurring Git patterns into a personal workflow documentation guide"
duration_minutes: 60
# stage: 3 # Internal scaffolding - hidden from students
# stage_label: "Intelligence Design" # Internal scaffolding - hidden from students

# HIDDEN SKILLS METADATA
skills:
  - name: "Workflow Pattern Recognition"
    proficiency_level: "A2"
    category: "Meta-Cognitive"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student identifies 3+ recurring patterns from L1-5"

  - name: "Reusable Documentation Creation"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student creates git-workflow.md they'll reuse in future"

  - name: "Pattern Application to Novel Context"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student applies documented pattern without lesson reference"

learning_objectives:
  - objective: "Identify recurring Git workflow patterns from Lessons 1-5"
    proficiency_level: "A2"
    bloom_level: "Create"
    assessment_method: "Student lists 3+ patterns encountered and when used"

  - objective: "Transform tacit Git knowledge into explicit workflow documentation"
    proficiency_level: "A2"
    bloom_level: "Create"
    assessment_method: "Student creates git-workflow.md personal reference"

  - objective: "Apply documented patterns to new projects without re-learning"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student applies pattern to new AI-generated feature scenario"

cognitive_load:
  new_concepts: 3
  assessment: "3 concepts (pattern recognition, documentation, application) at A2 synthesis level ✓"

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/028-chapter-8-git-redesign/spec.md"
source_plan: "specs/028-chapter-8-git-redesign/plan.md"
created: "2025-11-17"
last_modified: "2025-11-17"
version: "1.0.0"
git_author: "Content Implementer (Claude Code)"
workflow: "/sp.implement"
---

# Reusable Git Patterns - Creating Your Workflow Skill

## The Big Idea: From Learning to Intelligence

Over Lessons 1-5, you've executed Git operations so many times that certain patterns have become obvious:

- **Before every AI change**, you commit the current state
- **When testing alternatives**, you create branches instead of risking main
- **Before pushing to cloud**, you always verify locally first
- **After fixing mistakes**, you note what went wrong to prevent repeats

These aren't random actions. They're **recurring patterns**—the same workflows you've used 2+ times in different contexts.

This lesson transforms those tacit patterns into **explicit, reusable intelligence** you'll carry into every future project.

---

## Part 1: Pattern Recognition

### Reflection Activity: Your Own Git Workflow

Before creating documentation, look back at Lessons 1-5 and identify the patterns you actually used.

Answer these reflection questions in your notebook:

#### Question 1: Commit-Before-Experiment Pattern
"What did you do **BEFORE** every time you let AI make changes?"

Write your answer:
- _______________

Expected pattern: "I committed the current state first"

**Why this matters**: This is your safety pattern. Every professional developer does this instinctively. By recognizing it, you own it.

#### Question 2: Branch-for-Alternatives Pattern
"When did you **branch instead of committing directly** to main?"

Write your answer:
- _______________

Expected pattern: "When testing multiple AI suggestions" or "When experimenting before deciding"

**Why this matters**: Branches isolate risk. You learned to use them defensively.

#### Question 3: Test-Before-Merge Pattern
"What sequence of actions did you always follow? (Describe the steps)"

Write your answer:
- _______________

Expected pattern: "Branch → let AI implement → test locally → if good, merge; if bad, delete"

**Why this matters**: This is your quality gate. You prevent broken code reaching main.

---

### What You've Discovered

These aren't obscure Git techniques. These are the **three fundamental patterns of safe AI-assisted development**:

1. **Commit-Before-Experiment**: "I save the current working state before risking changes"
2. **Branch-Test-Merge**: "I test in isolation before integrating"
3. **Push-for-Backup**: "I backup to cloud regularly to prevent catastrophic loss"

Every professional Git user follows these patterns. The fact that you've naturally learned them means you're thinking like a professional developer.

#### 🎓 Expert Insight
> In AI-native development, you don't memorize Git commands as isolated facts. You recognize that certain patterns recur across projects: "commit before risky changes," "test alternatives on branches," "backup regularly." The professional move is documenting these patterns so future you (and teammates) apply them automatically. This is what transforms learning into intelligence—making tacit knowledge explicit and reusable.

---

## Part 2: Create `git-workflow.md`

### Using Persona + Questions + Principles Framework

This section uses the **Stage 3 Intelligence Design framework**. You'll think like a Git safety engineer creating documentation for your future self.

#### Your Persona: Git Safety Engineer

> "Think like a Git safety engineer who prevents catastrophic mistakes. Your mission: capture the workflow decisions that keep code safe, so that future you (or teammates) can apply them automatically."

This persona shifts your thinking from "what commands do I remember?" to "what patterns prevent disasters?"

#### Three Sections of Your Documentation

Create a file called **`git-workflow.md`** in your project root with three sections. Here's the template:

---

### Template: `git-workflow.md`

```markdown
# My Git Workflow for AI-Assisted Development

*Personal reference guide for safe, professional development*

## Pattern 1: Commit Before Experiment

### When to Use This
- Before asking AI to make any changes to code
- Before pulling from GitHub (to have restore point)
- Anytime you think "this might go wrong"

### The Workflow
```bash
# Check what I'm about to commit
git status

# Understand the changes
git diff

# Stage relevant files (be selective!)
git add [specific files or .]

# Commit with clear message about what I'm preserving
git commit -m "Before [task]: [brief context]"

# Example commit messages:
# "Before refactor: working calculator"
# "Before AI feature: existing authentication system"
```

### Questions to Ask Myself
- Do I understand what I'm committing?
- If this experiment fails, will I want to return to this exact state?
- Is my commit message clear enough for future me to understand what was being preserved?

### Principles
- **Commit Intentionally**: Every commit is a conscious decision, not automatic
- **Message Clarity**: "Before X" messages explicitly signal a restore point
- **Selective Staging**: Don't commit unrelated work—stay focused

---

## Pattern 2: Branch-Test-Merge for Alternatives

### When to Use This
- Testing two or more AI implementations of same feature
- When main branch is working and you want to keep it that way
- Experimenting with significant changes

### The Workflow

#### Step 1: Identify Decision Point
```bash
# You're about to ask AI to generate something significant
# Instead of committing directly to main, create a branch

git checkout -b feature/[clear-name]
# Examples:
# git checkout -b feature/add-validation
# git checkout -b feature/performance-optimization
```

#### Step 2: Let AI Implement on Branch
```bash
# Ask AI to implement on this branch
# AI makes changes → commits them → you test locally

# Check what was generated
git log --oneline -5

# View differences from main
git diff main
```

#### Step 3: Test the Implementation
```bash
# Execute your test suite / manual testing
# Note: DO NOT merge yet

# Questions to answer:
# - Does it work?
# - Is it better than alternatives?
# - Would you want this in main?
```

#### Step 4: Keep or Discard (The Decision)
```bash
# Option A: Changes are good → Merge to main
git checkout main
git merge feature/[name]
git branch -d feature/[name]

# Option B: Changes don't work → Discard entirely
git checkout main
git branch -D feature/[name]
# ^^ Note capital -D: force delete without merging
```

### Questions to Ask Myself
- Have I tested thoroughly before merging?
- Can I explain why this branch is better than alternatives?
- Is my branch name descriptive enough to remember its purpose later?

### Principles
- **Parallel Safety**: Never modify main directly when uncertain
- **Test Before Commit**: Merge only after validation
- **Easy Discard**: If branch fails, delete it—no harm done

---

## Pattern 3: Push for Backup and Recovery

### When to Use This
- Daily (or after meaningful commits)
- Before risky operations
- Before lending laptop to someone else
- After merging important features

### The Workflow

#### Step 1: Verify Local State
```bash
# Check that everything is committed locally
git status
# Output should show: "working tree clean"

# See recent commits
git log --oneline -5
```

#### Step 2: Push to GitHub
```bash
# First-time setup (one per repository):
git remote add origin [your-github-url]
git push -u origin main

# Subsequent pushes:
git push
```

#### Step 3: Verify on GitHub
```bash
# (Not a Git command, but important practice)
# 1. Open https://github.com/[your-username]/[repo-name]
# 2. Verify you see your commits there
# 3. Verify you see your branches there
# This confirms backup worked
```

### Questions to Ask Myself
- Is everything committed locally before I push?
- Can I access my code from GitHub right now?
- Would I recover my work if my computer broke today?

### Principles
- **Backup Discipline**: Remote is not optional; it's your safety net
- **Verify Backup**: Don't assume push worked—check GitHub
- **Frequency**: Push at least daily; push immediately after completing features

---

## My Decision Framework

When facing a Git decision:

**Q: "Should I commit directly to main?"**
- A: Only if it's a tiny fix and you're 100% confident
- Otherwise: Use Pattern 1 (Commit Before Experiment) or Pattern 2 (Branch-Test-Merge)

**Q: "Should I test this on a branch or commit first?"**
- A: If uncertain about outcome → Use Pattern 2 (Branch-Test-Merge)
- If just saving state → Use Pattern 1 (Commit Before Experiment)

**Q: "How often should I push to GitHub?"**
- A: At least daily. After every completed feature. Never go more than 1 hour of work without pushing.
- Pattern 3 (Push for Backup) prevents catastrophic loss.

---

## My Workflow at a Glance

```
1. Start day: git status (Am I clean?)
2. Before AI: Commit current state (Pattern 1)
3. AI generates: Create branch if uncertain (Pattern 2)
4. Test locally: Run manual tests
5. Decision: Keep (merge) or discard (delete branch)
6. Wrap up: git push to GitHub (Pattern 3)
7. End of day: Verify on GitHub (backup confirmed)
```

---
```

**Copy this template and fill it in your own words.** The best reference guide is one written in your own language, reflecting your understanding.

#### 💬 AI Colearning Prompt
> "Explain why documenting Git patterns is more useful than memorizing commands. How does pattern-based thinking help with AI-assisted development?"

---

## Part 3: Apply Pattern to New Scenario

### New Project: AI Authentication System

Now let's apply your documented patterns to a completely new scenario—**without looking back at Lessons 1-5 unless absolutely necessary**.

**Scenario**: You've been asked to build a user authentication system. AI offers three different approaches (password-based, token-based, OAuth). You need to:

1. Test all three approaches safely
2. Choose the best one
3. Integrate it into main
4. Backup to GitHub

**Your Task**: Using **only your `git-workflow.md` documentation**, complete these steps:

#### Step 1: Initialize Project (Using your documentation)
```bash
# Refer to your Pattern 3 setup section
# Initialize Git repository
[Your command here]
```

#### Step 2: Apply Pattern 1 Before AI Changes
```bash
# Refer to your Pattern 1
# Create initial commit capturing current state
[Your command here]
```

#### Step 3: Test Three Implementations (Using Pattern 2)
```bash
# Create branch 1 for password-based
[Your command here]
# Tell AI: "Implement password-based authentication"
# Test it locally...

# Create branch 2 for token-based
git checkout main
[Your command here]
# Tell AI: "Implement token-based authentication"
# Test it locally...

# Create branch 3 for OAuth
git checkout main
[Your command here]
# Tell AI: "Implement OAuth authentication"
# Test it locally...
```

#### Step 4: Choose Winner and Integrate
```bash
# Test results:
# - Password-based: Works but harder to scale
# - Token-based: Works, better performance
# - OAuth: Overkill for MVP

# Decision: Keep token-based approach

# Refer to your Pattern 2 merge section
[Your commands here to merge feature/token-auth and delete others]
```

#### Step 5: Backup to GitHub (Pattern 3)
```bash
# Refer to your Pattern 3 push section
[Your commands here to push to GitHub]
```

---

### Reflection on Pattern Application

**Answer these questions in your notebook:**

1. **Did you need to refer back to Lessons 1-5 to complete this scenario?**
   - If yes → Your documentation needs clarity (revise those sections)
   - If no → Your patterns are well-documented ✓

2. **What would you add to your `git-workflow.md` based on this experience?**
   - Example: "I realized I should always list branches before switching"
   - Example: "I added a checklist for testing before merge"

3. **How confident do you feel applying these patterns to your next project?**
   - 1 (Not confident) → 5 (Very confident)
   - Rate yourself and note why

---

## What Just Happened: Intelligence Design in Action

You've completed **Stage 3 (Intelligence Design)** by:

1. **Recognizing patterns** from your L1-5 experience (not learning new patterns, synthesizing ones you already know)
2. **Documenting explicitly** what was previously tacit knowledge (converting "I do this instinctively" to "Here's my workflow")
3. **Applying to novel context** using only your documentation (proving the documentation is sufficient)

This is the core of **reusable intelligence**: capturing lessons learned so you never have to relearn them.

#### 🤝 Practice Exercise

> **Ask your AI**: "Review my git-workflow.md documentation and suggest what error recovery scenarios I should add. Then explain how having documented patterns helps me work faster with AI-generated code."

**Expected Outcome**: You'll understand that workflow documentation reduces cognitive load when working under pressure, allowing you to focus on code quality instead of remembering Git commands.

---

## Try With AI

**Setup**: You have your `git-workflow.md`. Now validate and extend it.

**Tool**: ChatGPT, Claude Code, or your AI assistant

### Exercise 1: Validation Review

```
I created this git-workflow.md guide for myself.
[Paste your git-workflow.md]

Review it for:
1. Clarity: Would someone else understand these patterns?
2. Completeness: Are there scenarios I didn't cover?
3. Practicality: Could I actually follow these steps under pressure?
```

**Expected Outcome**: AI identifies gaps ("You don't explain what to do if a merge conflicts" or "Your branch naming could be more consistent"). You revise accordingly.

### Exercise 2: Extend with Error Recovery

```
My workflow handles the happy path.
What error scenarios should I document?

For each:
- When this goes wrong (scenario)
- What I'd see (error message)
- How to recover (commands)

Examples:
- Accidentally committed to main instead of branch
- Merged wrong branch by mistake
- Pushed without testing
```

**Expected Outcome**: AI suggests realistic error scenarios. You add a new "Error Recovery" section to your workflow.

### Exercise 3: Test Pattern Application

```
New scenario: I want to add a new feature.
Help me apply my git-workflow.md to this situation.

My feature: [Describe a feature you want to build]
My workflow: [Paste your git-workflow.md]

Step-by-step:
1. What should I do first? (Refer to my workflow, don't just tell me commands)
2. What pattern from my workflow applies?
3. What decisions do I need to make?
```

**Expected Outcome**: AI walks you through applying your own patterns, validating the documentation is sufficient.

The patterns you've documented will be your safety net throughout the rest of Part 2 and beyond.