---
sidebar_position: 3
title: "The Daily Workflow"
description: "Learn five essential Git tasks using simple conversation with your AI"
keywords: [git workflow, version control, AI collaboration]

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Initialize Git Repository"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can create a new Git repository using AI guidance"

  - name: "Track File Changes"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can stage files and create commits with AI help"

  - name: "Push to Remote"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can upload commits to GitHub via AI commands"

learning_objectives:
  - objective: "Initialize a Git repository by asking AI assistant"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Student creates repository with AI help"

  - objective: "Check repository status using natural language"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Student asks AI to show what changed"

  - objective: "Stage, commit, and push changes with AI guidance"
    proficiency_level: "A1"
    bloom_level: "Apply"
    assessment_method: "Student completes workflow with AI assistant"

cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (init, status, add, commit, push) at A1 limit ✓"

# Generation metadata
generated_by: "lesson-writer"
source_spec: "specs/012-chapter-8-git-github-aidd/plan.md"
created: "2025-11-05"
last_modified: "2025-11-07"
version: "3.0.0"
---

# The Daily Workflow

Every time you work with code, you follow this pattern:

1. Start tracking your project
2. Check what changed
3. Choose what to save
4. Save a snapshot
5. Backup to cloud

You do this by asking Gemini CLI in plain language. No memorizing commands.

**Time**: 20 minutes

---

## The AIDD Approach

**Traditional way**: Memorize `git init`, `git add`, `git commit`, `git push`

**AIDD way**: Ask Gemini CLI what you want

- You say: "Save my work"
- Gemini runs: The right Git command
- You understand: What happened and why

---

## Task 1: Start Tracking

Turn on Git for your project folder.

**You ask Gemini CLI**: "Initialize Git in this folder"

Gemini runs: `git init`

Git creates a `.git` directory to track your project history.

**Check it worked**: Ask Gemini "Show me if Git is tracking this folder"

---

**What if I run this twice?**

Git will say "already exists" - nothing breaks. Running `git init` multiple times is safe.

---

## Task 2: Check What Changed

See which files are new or modified.

**You ask Gemini CLI**: "What files have changed? Show me the status."

Gemini runs: `git status`

Shows you:
- New files (not tracked yet)
- Modified files (changed since last commit)
- Staged files (ready to save)

---

**Understanding "staged"**:

Staged = marked for your next commit. Like putting items in a shopping cart before checkout.

---

## Task 3: Choose What to Save

Select which files to include in your next commit.

**You ask Gemini CLI**: "Stage all my changes"

Gemini runs: `git add .`

All modified and new files are prepared for commit.

**Check it worked**: Ask "Show me what's staged"

---

**To stage specific files**:

**Ask Gemini CLI**: "Stage only calculator.py"

Gemini runs: `git add calculator.py`

Only that file gets added to the staging area.

---

## Task 4: Save a Snapshot

Create a permanent save point with a message.

**You ask Gemini CLI**: "Commit these changes with message 'Add calculator module'"

Gemini runs: `git commit -m "Add calculator module"`

Git saves your staged files as a commit.

**Check it worked**: Ask "Show me my commit history"

---

**Good commit messages**:

- ✓ "Add calculator module" (clear, describes what)
- ✓ "Fix division by zero bug" (clear action)
- ✗ "stuff" (too vague)
- ✗ "update" (doesn't say what changed)

Pattern: Complete this sentence: "This commit will ______"

---

## Task 5: Backup to Cloud

Upload your commits to GitHub.

**First time setup**:

1. Create a repository on GitHub.com
2. Copy the repository URL (looks like: `https://github.com/username/project.git`)

**You ask Gemini CLI**: "Connect to GitHub repo [URL] and push my commits"

Gemini runs: `git remote add origin [URL]` then `git push -u origin main`

Connects your local repository to GitHub and uploads your commits.

**Check it worked**: Visit your GitHub repository URL in a browser - you should see your code.

---

**Subsequent pushes**:

After the first setup, just ask:

**Ask Gemini CLI**: "Push my commits to GitHub"

Gemini runs: `git push`

---

## Complete Example Workflow

Let's put all five tasks together:

**You ask Gemini CLI**: "Create a new Python project called 'calculator' with Git"

Gemini creates the folder and initializes Git.

**You**: "Create a file calculator.py with basic add and subtract functions"

Gemini creates the file.

**You**: "What's the status? What files does Git see?"

Gemini shows: `calculator.py` (untracked file)

**You**: "Stage calculator.py"

File is ready to commit.

**You**: "Commit with message 'Add basic calculator functions'"

Commit created.

**You**: "Push to my GitHub repo at [your-repo-url]"

Code is now backed up on GitHub.

---

## What You're Actually Learning

You're not learning Git commands.

You're learning:
- **What Git does** (tracks versions, creates savepoints)
- **When to use it** (before making risky changes, after completing work)
- **How to ask** (clear, specific requests)
- **How to verify** (always check it worked)

This is the professional pattern: understand concepts, use AI for execution.

---

## Try With AI

Practice the complete workflow.

**Tool**: Gemini CLI (or Claude Code, ChatGPT)

### Exercise 1: Complete Workflow

```
Create a practice project:
1. Make a folder "git-practice"
2. Initialize Git
3. Create a file hello.py with print("Hello Git")
4. Check status
5. Stage the file
6. Commit with message "Initial commit"
7. Show me the commit history
```

### Exercise 2: Make Changes

```
I modified hello.py to print "Hello World!".
Help me:
1. See what changed (diff)
2. Stage the change
3. Commit with an appropriate message
4. Verify the commit was created
```

### Exercise 3: GitHub Backup

```
I want to backup this project to GitHub.
Guide me:
1. Create a GitHub repository (walk me through)
2. Connect my local project to GitHub
3. Push my commits
4. Verify I can see the code on GitHub
```

### Exercise 4: Check Your Understanding

```
Explain back to me:
1. What does "staging" mean?
2. What's the difference between a commit and a push?
3. Why do I write commit messages?
4. Can I have multiple commits before pushing?
```

<Quiz title="Chapter 3 Quiz" questions={[{"question":"What two figures are multiplied to calculate the ~$3 trillion aggregate economic output of the developer economy?","options":{"a":"100 million GitHub users and their average subscription fee.","b":"~30 million professional developers and $100,000 in annual generated value per developer.","c":"The number of software companies and their average annual revenue.","d":"The number of lines of code written per year and the value of each line."},"correct_answer":"b","explanation":"The chapter explicitly states the calculation: \u0027~30 million professional developers worldwide × $100,000 in annual generated value per developer = ~$3 trillion in aggregate economic output.\u0027"},{"question":"The chapter compares the $3 trillion developer economy to the GDP of which country?","options":{"a":"The United States","b":"China","c":"Canada","d":"France"},"correct_answer":"d","explanation":"The text makes a direct comparison: \u0027$3 trillion is approximately the GDP of France—the world\u0027s 7th or 8th largest economy...\u0027"},{"question":"What is the \u0027Acceleration Paradox\u0027 described in the chapter?","options":{"a":"AI coding tools are slowing down software production because of the need for more reviews.","b":"AI coding tools are accelerating software production, but the quality of software is decreasing.","c":"AI coding tools are accelerating software production, which is increasing the demand for software and developers, not reducing it.","d":"Only senior developers are getting faster, while junior developers are getting slower."},"correct_answer":"c","explanation":"The chapter explains the paradox: \u0027Traditional economic logic suggested that automation reduces demand for labor... What\u0027s actually happening is more subtle and more profound... This doesn\u0027t shrink the software market. It explodes it.\u0027"},{"question":"The chapter draws a historical parallel between the current software disruption and the transformation of which other industry?","options":{"a":"The automotive industry with the invention of the assembly line.","b":"The music industry with the shift to digital streaming.","c":"The printing industry with the introduction of desktop publishing software.","d":"The textile industry during the industrial revolution."},"correct_answer":"c","explanation":"The text makes a specific comparison: \u0027Software disrupting itself has few direct historical parallels, but one comparison stands out: The printing industry in the late 20th century.\u0027"},{"question":"What is the primary effect of AI coding tools on the software market, as described in the chapter?","options":{"a":"It is shrinking the market by automating the creation of common software.","b":"It is shifting the market from a Software-as-a-Service (SaaS) model to one of highly customized, individual software solutions.","c":"It is causing a decline in software quality and security.","d":"It is making software more expensive to create."},"correct_answer":"b","explanation":"The chapter explains, \u0027AI coding tools are enabling a shift toward highly customized, individual software solutions... This doesn\u0027t shrink the software market. It explodes it.\u0027"}]} />

