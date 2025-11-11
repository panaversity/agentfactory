---
sidebar_position: 9
title: "Capstone Exercise: Build with Git & GitHub"
description: "Integrate all Git, GitHub, and AI skills in a realistic calculator project demonstrating complete workflow"
keywords: [capstone, git workflow, github, calculator project, ai-assisted development]

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Project Setup with Git"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student initializes a Git repository with proper documentation and creates initial commit"

  - name: "AI-Assisted Development"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student uses AI to generate code while reviewing outputs and committing appropriately"

  - name: "Branch-Based Testing"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student creates feature branches, tests changes, and merges or discards appropriately"

  - name: "Safety Checkpoints"
    proficiency_level: "A2"
    category: "Soft"
    bloom_level: "Understand"
    digcomp_area: "Safety"
    measurable_at_this_level: "Student commits before major changes and reviews AI-generated code before merging"

  - name: "GitHub Portfolio"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student connects project to GitHub and creates a professional pull request with clear documentation"

learning_objectives:
  - objective: "Initialize a Git project with README and initial commit"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Verify git init, README.md created, and initial commit in git log"

  - objective: "Use AI to generate complete Python code for calculator with add, subtract, multiply, divide functions"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Verify calculator.py exists and is committed with descriptive message"

  - objective: "Create a feature branch and safely test AI-generated enhancements"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Verify feature branch created, changes tested, and merged or discarded with reasoning"

  - objective: "Connect local repository to GitHub and push code"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Verify repository visible on GitHub with complete commit history"

  - objective: "Create and merge a pull request documenting AI assistance"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Verify pull request created with clear description of AI involvement and changes"

cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (project setup, AI-assisted code generation, feature branches, GitHub integration, PR workflow) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Extend calculator with additional features (exponentiation, square root, history); push to GitHub with multiple PRs showing git workflow mastery"
  remedial_for_struggling: "Pair with peer or AI assistant for each step; focus on understanding one concept at a time; extend timeline to 45 minutes if needed"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/012-chapter-8-git-github-aidd/plan.md"
created: "2025-11-05"
last_modified: "2025-11-07"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Capstone: Build Your First Project

You've learned Git piece by piece. Now use EVERYTHING together to build a real project.

**The project**: Python calculator
**Who writes code**: AI (Gemini CLI)
**Your job**: Manage Git, make decisions, review AI's work

**Time**: 30 minutes

---

## What You'll Use

Build a calculator project using:
- ✅ Git (track changes)
- ✅ Branches (test safely)
- ✅ GitHub (backup online)
- ✅ Pull Requests (professional workflow)
- ✅ AI (generates all code)

**Important**: You're not learning Python. You're learning to MANAGE a project with Git while AI writes code.

---

## The Professional Workflow

```
1. Init project + first commit
   ↓
2. AI generates code → review → commit
   ↓
3. Create branch → AI adds feature → test → merge
   ↓
4. Push to GitHub
   ↓
5. Create Pull Request → review → merge
   ↓
6. Professional portfolio project!
```

---

## Step 1: Initialize Project

**What you're doing**: Create folder, start Git, make first commit

**You ask Gemini CLI**:

```
Create a folder called 'my-calculator'.
Navigate into it.
Initialize Git.
Create a README.md saying "Calculator Project".
Commit with message 'Initial project setup'.
```

Gemini runs:
```bash
mkdir my-calculator
cd my-calculator
git init
echo "# Calculator Project" > README.md
git add README.md
git commit -m "Initial project setup"
```

**Check it worked**:

**Ask Gemini CLI**: "Show me the commit history"

Gemini runs: `git log --oneline`

You should see: `Initial project setup`

---

## Step 2: Generate Calculator Code

**What you're doing**: AI writes calculator, you review and commit

**You ask Gemini CLI**:

```
Create calculator.py with these functions:
- add(a, b) - adds two numbers
- subtract(a, b) - subtracts two numbers
- multiply(a, b) - multiplies two numbers
- divide(a, b) - divides with zero-check

Show me the code first.
```

Gemini shows you:

```python
def add(a, b):
    return a + b

def subtract(a, b):
    return a - b

def multiply(a, b):
    return a * b

def divide(a, b):
    if b == 0:
        return 'Error: Cannot divide by zero'
    return a / b
```

**Review the code**:
- Does it have all 4 functions? ✓
- Does divide() handle zero? ✓
- Do you understand what it does? ✓

**You**: "Looks good. Save this file and commit it with message 'Add calculator module'."

Gemini runs:
```bash
# (saves calculator.py)
git add calculator.py
git commit -m "Add calculator module"
```

**Check it worked**:

**Ask Gemini CLI**: "Show me my commits"

You should see:
```
Add calculator module
Initial project setup
```

---

## Step 3: Test Changes on Branch

**What you're doing**: Create branch, add tests, verify safely

**You ask Gemini CLI**:

```
Create a branch called 'add-tests'.
Generate pytest tests for calculator.py.
Save to test_calculator.py.
Show me the tests first.
```

Gemini runs: `git checkout -b add-tests`

Then shows you the tests:

```python
import pytest
from calculator import add, subtract, multiply, divide

def test_add():
    assert add(2, 3) == 5

def test_subtract():
    assert subtract(5, 3) == 2

def test_multiply():
    assert multiply(4, 3) == 12

def test_divide():
    assert divide(10, 2) == 5

def test_divide_by_zero():
    assert divide(10, 0) == 'Error: Cannot divide by zero'
```

**Review the tests**:
- Tests all 4 functions? ✓
- Tests division by zero? ✓

**You**: "Good. Save and commit with message 'Add unit tests'."

Gemini runs:
```bash
# (saves test_calculator.py)
git add test_calculator.py
git commit -m "Add unit tests"
```

**Check branch status**:

**Ask Gemini CLI**: "Which branch am I on? Show me what's different from main."

Gemini runs:
- `git branch --show-current` → shows `add-tests`
- `git diff main` → shows test_calculator.py added

---

## Step 4: Test and Merge

**What you're doing**: Run tests, if pass → merge, if fail → fix

**You ask Gemini CLI**:

```
Run the tests using pytest.
If all tests pass, merge this branch to main.
If tests fail, tell me what broke.
```

Gemini runs: `pytest test_calculator.py`

Output:
```
====== test session starts ======
test_calculator.py .....     [100%]

====== 5 passed in 0.12s ======
```

All tests pass! ✓

**You**: "Merge to main and delete the branch."

Gemini runs:
```bash
git checkout main
git merge add-tests
git branch -d add-tests
```

**Check it worked**:

**Ask Gemini CLI**: "Show me my commit history and current branch"

You should see:
- Branch: `main`
- Commits: Add unit tests, Add calculator module, Initial project setup

---

## Step 5: Push to GitHub

**What you're doing**: Backup code to GitHub

### Create Repository on GitHub

1. Go to github.com
2. Click "New repository"
3. Name: `my-calculator`
4. Public
5. Don't initialize (you already have code)
6. Create repository
7. Copy the URL (e.g., `https://github.com/username/my-calculator.git`)

### Connect and Push

**You ask Gemini CLI**: "Connect to GitHub repo [paste URL] and push my code"

Gemini runs:
```bash
git remote add origin https://github.com/username/my-calculator.git
git branch -M main
git push -u origin main
```

GitHub will ask for authentication:
- Username: your GitHub username
- Password: your Personal Access Token

**Check it worked**:

Visit `github.com/username/my-calculator` in browser.

You should see:
- README.md
- calculator.py
- test_calculator.py
- All 3 commits

---

## Step 6: Create Pull Request

**What you're doing**: Add feature using professional PR workflow

**You ask Gemini CLI**:

```
Create a branch called 'improve-docs'.
Add docstrings to all functions in calculator.py.
Show me the updated code.
```

Gemini runs: `git checkout -b improve-docs`

Then shows updated code:

```python
def add(a, b):
    """Add two numbers and return the result."""
    return a + b

def subtract(a, b):
    """Subtract b from a and return the result."""
    return a - b

def multiply(a, b):
    """Multiply two numbers and return the result."""
    return a * b

def divide(a, b):
    """
    Divide a by b and return the result.
    Returns error message if b is zero.
    """
    if b == 0:
        return 'Error: Cannot divide by zero'
    return a / b
```

**You**: "Looks good. Commit with message 'Add function docstrings' and push to GitHub."

Gemini runs:
```bash
git add calculator.py
git commit -m "Add function docstrings"
git push -u origin improve-docs
```

### Create PR on GitHub

1. Go to `github.com/username/my-calculator`
2. Yellow banner appears: "improve-docs had recent pushes"
3. Click "Compare & pull request"
4. Title: "Improve documentation"
5. Description:

```markdown
## Changes
Added docstrings to all calculator functions

## AI Assistance
Gemini CLI generated the docstrings

## Testing
All existing tests still pass
```

6. Click "Create pull request"

### Merge the PR

1. Review the diff on GitHub
2. Click "Merge pull request"
3. Click "Confirm merge"
4. Click "Delete branch" (on GitHub)

### Pull Changes Locally

**You ask Gemini CLI**: "Pull the merged changes and delete local branch"

Gemini runs:
```bash
git checkout main
git pull
git branch -d improve-docs
```

**Check it worked**:

**Ask Gemini CLI**: "Show me current branch and latest commit"

Should show:
- Branch: `main`
- Latest commit: "Add function docstrings"

---

## Complete Workflow Summary

Here's what you did:

| Step | Action | Git Commands |
|------|--------|-------------|
| 1 | Initialize project | `git init`, `git commit` |
| 2 | Generate code | `git add`, `git commit` |
| 3 | Create test branch | `git checkout -b`, `git commit` |
| 4 | Test & merge | `pytest`, `git merge`, `git branch -d` |
| 5 | Push to GitHub | `git remote add`, `git push` |
| 6 | Pull Request | `git checkout -b`, `git push`, GitHub merge, `git pull` |

You used **every major Git concept** from this chapter!

---

## Success Checklist

Verify you completed everything:

**Your Project Has:**
- [ ] Git repository with 4+ commits
- [ ] calculator.py with 4 functions
- [ ] test_calculator.py with 5 tests
- [ ] README.md explaining project
- [ ] Code pushed to GitHub (public)
- [ ] Pull request created and merged
- [ ] Clean main branch

**You Can Now:**
- [ ] Start Git projects from scratch
- [ ] Use AI to generate code
- [ ] Review code before committing
- [ ] Create and use branches
- [ ] Merge safely after testing
- [ ] Push to GitHub
- [ ] Create professional PRs

**You Understand:**
- [ ] Git tracks every change
- [ ] Branches protect main code
- [ ] Testing before merging prevents bugs
- [ ] GitHub backs up your work
- [ ] AI writes code, you manage quality

---

## What You Accomplished

**Professional skills demonstrated**:
- Git version control
- Branch-based development
- Test-driven workflow
- Code review (reviewing AI output)
- GitHub collaboration
- Pull Request process

**You built**:
- Working Python calculator
- Comprehensive test suite
- Professional Git history
- GitHub portfolio project

**This workflow applies to ALL projects** - web apps, data science, mobile apps, anything. You now have the foundation.

---

## Optional Extensions

Want more practice?

### Add More Features

**You ask Gemini CLI**:

```
Create branch 'add-power'.
Add a power(a, b) function that calculates a^b.
Add tests.
Merge if tests pass.
```

Try also:
- `square_root(a)`
- `factorial(n)`
- `percent(a, b)`

Each feature: branch → code → test → merge

### Improve Code Quality

```
Review calculator.py for improvements.
Suggest better error handling, input validation, or logging.
Create a PR for each improvement.
```

### Build Something New

Use this same workflow to build:
- Todo list app
- Password generator
- File organizer
- Data analyzer

**The workflow stays the same**. Only the code changes.

---

## Key Commands Reference

You used these throughout the project:

| Purpose | Command |
|---------|---------|
| **Setup** | |
| Initialize Git | `git init` |
| First commit | `git add .` then `git commit -m "message"` |
| **Development** | |
| Create branch | `git checkout -b branch-name` |
| Switch branch | `git checkout branch-name` |
| Stage changes | `git add filename` |
| Commit changes | `git commit -m "message"` |
| **Safety** | |
| Check status | `git status` |
| View history | `git log --oneline` |
| See differences | `git diff` |
| **Merging** | |
| Merge branch | `git checkout main` then `git merge branch-name` |
| Delete branch | `git branch -d branch-name` |
| **GitHub** | |
| Connect remote | `git remote add origin [URL]` |
| Push | `git push -u origin main` |
| Pull | `git pull` |

Ask Gemini CLI in natural language - it handles the details.

## Congratulations!

You completed Chapter 8: Git & GitHub for AI-Driven Development.

You can now:
- ✅ Manage code with Git
- ✅ Test changes safely with branches
- ✅ Backup work on GitHub
- ✅ Create professional pull requests
- ✅ Use AI as your coding partner
- ✅ Build portfolio projects

**This is the foundation of professional software development.** Everything else builds on Git + AI + testing.

Start your next project today. The best way to learn is by building.

<Quiz title="Chapter 9 Quiz" questions={[{"question":"What is the \u0027Curriculum Lag Phenomenon\u0027 described in the chapter?","options":{"a":"University curricula are updated every 3-6 months, which is too fast for students.","b":"University curricula operate on a 2-4 year revision cycle, which is too slow to keep up with the 3-6 month evolution cycle of AI technology.","c":"Students are lagging behind in their coursework.","d":"The curriculum for computer science has not changed since the 1980s."},"correct_answer":"b","explanation":"The chapter defines the problem as: \u0027Universities operate on a 2-4 year curriculum revision cycle... Meanwhile, the AI coding landscape is evolving on a 3-6 month cycle.\u0027"},{"question":"Which of the following is listed as a critical skill gap between what traditional CS programs teach and what is needed in the AI era?","options":{"a":"Understanding algorithms and data structures.","b":"AI Collaboration and Prompt Engineering.","c":"Mathematical reasoning and proof techniques.","d":"Foundational computer architecture."},"correct_answer":"b","explanation":"The chapter identifies several gaps, with the first one being \u0027AI Collaboration and Prompt Engineering,\u0027 which is contrasted with the traditional focus on writing code from scratch."},{"question":"What is the chapter\u0027s recommendation for a student currently in a CS program?","options":{"a":"Quit the program immediately as it is obsolete.","b":"Ignore all AI tools and focus only on the university curriculum.","c":"Finish the degree for its foundational value but supplement it with practical AI skills from resources like this book.","d":"Focus only on theory and ignore practical skills."},"correct_answer":"c","explanation":"The text advises: \u0027Should you quit? No... Should you supplement your education? Absolutely. Treat university education as the foundation and add AI-driven development skills...\u0027"},{"question":"Why is \u0027Specification-Driven Development\u0027 identified as a critical missing piece in traditional education?","options":{"a":"Because it is a historical practice that is no longer relevant.","b":"Because AI agents require clear, comprehensive specifications to work effectively, making it an essential skill.","c":"Because it makes the development process slower and more bureaucratic.","d":"Because it is only useful for hardware engineering, not software."},"correct_answer":"b","explanation":"The chapter states, \u0027AI agents require clear specifications to work effectively. Vague requirements produce poor outputs. Spec-driven development isn\u0027t optional anymore...\u0027"},{"question":"What is the proposed \u0027Path Forward\u0027 to deal with the gaps in traditional CS education?","options":{"a":"A \u0027Hybrid Learning\u0027 approach, combining foundational theory from universities with practical, modern AI skills from other resources.","b":"Abandoning universities entirely in favor of online bootcamps.","c":"Forcing universities to update their curricula every month.","d":"Replacing all CS theory with vocational training on specific AI tools."},"correct_answer":"a","explanation":"The chapter concludes by recommending a \u0027Hybrid Learning\u0027 path: \u0027The solution isn\u0027t to abandon traditional education—it\u0027s to supplement it intelligently,\u0027 combining the best of both worlds."}]} />

