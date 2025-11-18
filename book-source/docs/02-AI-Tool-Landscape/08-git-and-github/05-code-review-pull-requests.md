---
sidebar_position: 5
title: "Code Review with Pull Requests"
description: "Create pull requests, review changes, and document AI assistance transparently for professional GitHub workflows"

# CONSTITUTIONAL METADATA
# stage: "2 (AI Collaboration with Three Roles)" # Internal scaffolding - hidden from students
teaching_modality: "Three Roles + Transparency Demonstration"
# ai_transparency: "CRITICAL - PR descriptions must include AI attribution" # Internal scaffolding - reflected in lesson content

# HIDDEN SKILLS METADATA
skills:
  - name: "Create Pull Request"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student can create a pull request with clear title, description including AI assistance section, and testing done"

  - name: "Review PR Diff"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can examine code diffs in a PR, understand what changed, and verify correctness"

  - name: "Document AI Assistance"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Critical Thinking"
    measurable_at_this_level: "Student transparently documents which AI generated which code and what they modified"

  - name: "Merge Pull Request"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can merge PR after review and observe main branch update"

learning_objectives:
  - objective: "Create pull request from feature branch to main"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student creates feature branch, pushes changes, opens PR on GitHub"

  - objective: "Review PR diff to evaluate changes"
    proficiency_level: "A2"
    bloom_level: "Analyze"
    assessment_method: "Student examines code changes, verifies they match intent"

  - objective: "Document AI assistance in PR description"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student includes AI attribution section: which AI was used, what it generated, what they modified"

  - objective: "Merge approved PR into main branch"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student merges PR and verifies changes appear on main"

estimated_time: "50 minutes"
cognitive_load: "4 concepts (within A2 limit)"
prerequisites:
  - "Lesson 1: Your First Git Repository"
  - "Lesson 2: Viewing Changes & Safe Undo"
  - "Lesson 3: Testing AI Safely with Branches"
  - "Lesson 4: Cloud Backup & Portfolio"

generated_by: "content-implementer v1.0.0"
source_spec: "specs/028-chapter-8-git-redesign/spec.md"
created: "2025-01-17"
last_modified: "2025-01-17"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Code Review with Pull Requests: AI-Generated Code Evaluation

In this lesson, you'll master the professional GitHub workflow where code is reviewed BEFORE merging to main. You'll also learn the critical practice of documenting AI assistance transparently—essential in AI-native development where all work includes some AI contribution.

**Why This Matters**: Pull requests (PRs) are how teams review code before it ships. In your case, they're also how you document and validate AI-generated code. This lesson teaches both professional practice AND transparency ethics.

You'll work with AI on PR best practices—AI will suggest approaches, you'll refine based on your context, and together you'll create clear, transparent documentation.

---

## Understanding Pull Requests: Code Review as Safety Mechanism

Before you create a PR, let's understand what it does.

### What Is a Pull Request?

A **Pull Request (PR)** is a GitHub feature that lets you propose merging code from one branch into another (usually main). It's like saying: *"Here are my changes. Please review them before merging."*

**The PR Workflow**:
1. You push feature branch to GitHub
2. You create PR on GitHub (comparing feature branch → main)
3. GitHub shows you a **diff** (what changed)
4. You review changes and add description
5. You merge PR when satisfied

**Why PRs Matter for AI Development**:
- **Safety check**: Gives you moment to evaluate AI-generated code before it affects main
- **Documentation**: Forces you to write WHY you made changes (and if AI helped)
- **Portfolio practice**: Shows employers you follow professional code review practices

**Key Difference from Commits**:
- **Commits** save snapshots locally (or pushed to GitHub)
- **PRs** formalize a review + merge decision (GitHub-specific feature)

---

## Concept 1: Pull Request Creation

Let's create your first PR.

### Setup: Feature Branch with Changes

Before creating a PR, you need:
1. A feature branch with committed changes
2. That branch pushed to GitHub
3. Changes ready for review

**If you haven't already**: Create a feature branch from Lesson 4:

```bash
# On main branch (verify first)
git checkout main

# Create feature branch for new feature
git checkout -b feature/enhanced-calculator

# Make some changes to a file
# (Edit feature-description.txt or similar)

# Stage and commit changes
git add .
git commit -m "feat: add error handling to calculator"

# Push to GitHub
git push -u origin feature/enhanced-calculator
```

### Creating PR on GitHub

Now create PR on GitHub's web interface (no terminal command—PR is GitHub-only feature).

**Step 1: Navigate to Your Repository**
- Go to github.com and open your repository
- You'll see: "feature/enhanced-calculator had recent pushes"
- Click "Compare & pull request" button

**Step 2: Fill Out PR Form**

GitHub shows a form with:
- **Base branch**: `main` (target—where you're merging TO)
- **Compare branch**: `feature/enhanced-calculator` (source—what you're merging FROM)
- **Title**: Brief description of what this PR does
- **Description**: Detailed explanation

**Example PR Title**:
```
feat: add error handling to calculator
```

**Step 3: Add PR Description**

This is where you explain your changes AND document AI assistance.

---

## Concept 2: PR Description with AI Transparency

Your PR description is critical. It serves three purposes:
1. **Summary**: What changed and why
2. **Testing**: How to verify it works
3. **AI Attribution** (NEW FOR AI-NATIVE DEV): Which AI helped, what it generated, what you modified

### PR Description Template

Use this template for every PR:

```markdown
## Summary
[Plain-language explanation of what this PR does]

## Changes
- [What file changed and why]
- [What file changed and why]

## AI Assistance
**AI Tool Used**: ChatGPT / Claude / Gemini / [your choice]

**What AI Generated**:
- Generated initial error handling structure
- Generated validation functions for inputs

**What I Modified**:
- Simplified error messages for clarity
- Added logging for debugging
- Tested edge cases AI missed

## Testing Done
- [How did you test this?]
- [Edge cases checked?]

## Screenshots (if applicable)
[Optional: show before/after if UI-related]
```

### Example: Real PR with AI Transparency

**Feature Branch**: `feature/enhanced-calculator`
**Changes**: Added error handling to feature-description.txt

**PR Description Example**:

```markdown
## Summary
Added error handling to calculator to prevent crashes on invalid input.

## Changes
- Updated feature-description.txt: added input validation before operations
- Added try/except blocks for division by zero

## AI Assistance
**AI Tool Used**: ChatGPT

**What AI Generated**:
- Initial try/except structure
- Input validation regex patterns
- Error message templates

**What I Modified**:
- Made error messages more user-friendly (not technical jargon)
- Added logging for debugging
- Tested with edge cases: negative numbers, decimals, text input
- Found bug where AI didn't handle empty input—fixed manually

## Testing Done
- Manual testing: positive numbers, negative numbers, decimals
- Edge case: division by zero → correctly caught and displayed error message
- Edge case: text input → correctly rejected with helpful message
```

**Key Transparency Elements**:
- ✅ Explicit AI tool named (ChatGPT)
- ✅ What AI generated listed
- ✅ What you modified listed
- ✅ Bug you found (AI didn't handle empty input) documented

#### 🎓 Expert Insight
> In AI-native development, transparency about AI assistance is professional practice. You don't memorize PR templates—you understand that documenting AI's role builds trust and helps reviewers evaluate code quality.

**Why This Matters**: Future employers/code reviewers will see you're transparent about AI usage. You don't hide it or overclaim credit. This builds trust.

---

## Concept 3: Reviewing PR Diff

Before merging, **always review the diff** to verify changes are correct.

### Understanding the Diff View

On GitHub PR page, click "Files Changed" tab. You'll see:

```diff
- Old line (what was removed)        [red background]
+ New line (what was added)          [green background]
  Unchanged line                      [no color]
```

### What to Look For

When reviewing a diff, ask these questions:

1. **Does it match the intent?**
   - "I wanted error handling... yes, I see try/except blocks"

2. **Are there bugs?**
   - Off-by-one errors in loops?
   - Missing edge cases?
   - Typos in variable names?

3. **Is there AI code I don't understand?**
   - Test it manually or ask AI to explain
   - Don't merge code you don't understand

4. **Are there security issues?**
   - Hardcoded passwords? (No!)
   - SQL injection vulnerabilities? (No!)
   - For A2-level: basic checks are fine—don't worry about advanced security yet

### Example Diff Review

```diff
@@ -15,6 +15,18 @@ def add(a, b):
 def divide(a, b):
-    return a / b
+    if b == 0:
+        raise ValueError("Cannot divide by zero")
+    return a / b
+
+def validate_input(value):
+    """Check if input is valid number."""
+    try:
+        float(value)
+        return True
+    except ValueError:
+        return False
```

**Your Thoughts**:
- ✅ "I can see division by zero check—good"
- ✅ "New validate_input function—makes sense for error handling"
- ✅ "Code is clear, I understand what it does"
- ✅ "Ready to merge"

#### 💬 AI Colearning Prompt
> "Explain what I should look for when reviewing a code diff in a pull request. What are common mistakes AI-generated code might have that I should catch?"

---

#### 🤝 Practice Exercise

> **Ask your AI**: "I'm creating a pull request for code you helped me generate. Show me how to write a transparent PR description that includes what you generated, what I modified, and what edge cases I tested. Then explain why this transparency matters for code reviews."

**Expected Outcome**: You'll understand that PR descriptions should document the collaboration process, not just the final result.

---

## Try With AI

Now practice this with ChatGPT or Claude.

### Setup

Have your GitHub PR open (or create a simple feature branch with changes and open a PR).

### Try With AI

**Prompt 1 (AI reviews your PR description)**:
```
I wrote this PR description for my GitHub pull request.
Please review it for clarity and suggest improvements:

[Copy your PR description here]
```

**Expected Outcome**: AI will identify missing details, confusing language, or unclear sections. You learn what reviewers look for.

---

**Prompt 2 (AI generates commit message format)**:
```
I'm creating a pull request where I added error handling to my calculator app
with help from ChatGPT. Write a professional PR title and bullet-point description
that documents:
- What the feature does
- What ChatGPT generated
- What I tested and modified
```

**Expected Outcome**: AI generates examples of professional transparency language. Use these as templates.

---

**Prompt 3 (Advanced - AI reviews your code diffs)**:
```
Here's the code diff from my pull request (changes to feature-description.txt):

[Paste your diff here]

Can you:
1. Summarize what changed
2. Identify any bugs or edge cases
3. Suggest improvements
```

**Expected Outcome**: AI acts like a code reviewer—catches issues you missed, suggests improvements.