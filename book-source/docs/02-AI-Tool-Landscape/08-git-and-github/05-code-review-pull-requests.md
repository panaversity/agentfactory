---
sidebar_position: 5
title: "Code Review with Pull Requests"
description: "Create pull requests, review changes, and document AI assistance transparently for professional GitHub workflows"

# CONSTITUTIONAL METADATA
stage: "2 (AI Collaboration with Three Roles)"
teaching_modality: "Three Roles + Transparency Demonstration"
ai_transparency: "CRITICAL - PR descriptions must include AI attribution"

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

**Stage 2 Focus**: You'll experience bidirectional learning with AI—AI suggests PR best practices, you refine based on your context, and together you'll create clear, transparent documentation.

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
# (Edit calculator.py or similar)

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
**Changes**: Added error handling to calculator.py

**PR Description Example**:

```markdown
## Summary
Added error handling to calculator to prevent crashes on invalid input.

## Changes
- Updated calculator.py: added input validation before operations
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

---

## Three Roles in Action: Pull Request Review Iteration

Now you'll experience bidirectional learning with AI around PR quality.

### Role 1: AI as Teacher

You create a basic PR description:

```markdown
## Summary
Added error handling

## AI Assistance
Used ChatGPT to generate code
```

**AI suggests** (you ask it to review your PR description):

*"Your PR description is good start, but missing important information:*
- *Which functions did AI generate?*
- *What edge cases did you test?*
- *Did you find any bugs in AI code?*
- *These details help reviewers understand your confidence in the code"*

**What You Learned**: AI taught you that transparency includes specifics, not just "I used AI." This is professional transparency.

---

### Role 2: AI as Student

AI writes a technical description:

```markdown
## AI Assistance
AI Tool: ChatGPT
Generated: Input validation function utilizing regex pattern matching with exception handling for type coercion scenarios.
```

**You correct** (this is too technical for non-programmers):

"Wait—I need non-programmers to understand this. Let me simplify it."

**AI adapts**:

```markdown
## AI Assistance
AI Tool: ChatGPT
What AI Generated:
- Function to check if input is a valid number
- Function to catch errors when math operations fail

What I Modified:
- Tested with different types of invalid input (text, symbols, empty)
- Added simpler error messages so users understand what went wrong
```

**What AI Learned**: Your PR will be reviewed by non-technical stakeholders (even if it won't be). Clear language > technical precision.

---

### Role 3: AI as Co-Worker (Convergence)

You and AI iterate on the PR description together:

**Iteration 1** (Starting point):
- Basic summary
- AI lists what it generated
- No detail on testing

**Iteration 2** (Student adds):
"Let me test some edge cases first, then I'll update the PR"

[You test: empty input, very large numbers, special characters]

**Iteration 3** (AI suggests):
"Add a testing section to your PR description:
- What edge cases did you find?
- Did AI code handle them?
- Which ones did you have to fix?"

**Iteration 4** (Convergence - final PR):

```markdown
## Summary
Added error handling to calculator to prevent crashes

## AI Assistance
AI Tool: ChatGPT

What AI Generated:
- Try/except blocks for catching errors
- Input validation function

What I Modified & Tested:
- Tested empty input → found bug! AI didn't handle this
- Fixed: Added check for empty string before validation
- Tested very large numbers → works correctly
- Tested special characters (!, @, #) → correctly rejected

## Testing Done
- Empty input: Fixed (was not handled by AI)
- Large numbers: Passed
- Special characters: Passed (rejected safely)
- Division by zero: Passed (AI code correct)
```

**Why This Is Co-Working**:
- AI suggested structure (teacher)
- You tested and found bugs (validation)
- AI suggested what to document (teacher again)
- Together, you created transparent, detailed PR (convergence)

Neither of you would write this alone. Collaboration made it better.

---

## Hands-On Activity 1: Create Feature Branch and Push

Let's create a real feature branch to work with.

### Step 1: Create Feature Branch

```bash
# Go to your project
cd your-project

# Make sure you're on main
git checkout main

# Create feature branch (replace "your-feature" with something real)
git checkout -b feature/add-multiplication
```

### Step 2: Make a Simple Change

Edit one of your project files (e.g., calculator.py, main.py, etc.):

```python
# Add a simple function
def multiply(a, b):
    """Multiply two numbers."""
    return a * b
```

### Step 3: Commit and Push

```bash
# Stage changes
git add .

# Commit with clear message
git commit -m "feat: add multiplication function"

# Push to GitHub
git push -u origin feature/add-multiplication
```

**Verification**: Visit github.com/your-username/your-repo. You should see:
- Your new branch in branch list
- "Compare & pull request" button

---

## Hands-On Activity 2: Create Pull Request on GitHub

Now create the PR on GitHub's web interface.

### Step 1: Click "Compare & Pull Request"

On your GitHub repository page, you'll see a yellow banner suggesting "Compare & pull request". Click it.

### Step 2: Verify Branches

GitHub shows:
- **Base**: `main` (where you're merging TO)
- **Compare**: `feature/add-multiplication` (what you're merging FROM)

Click "Create pull request" to continue.

### Step 3: Fill Out PR Form

**Title**:
```
feat: add multiplication function to calculator
```

**Description** (use template from Concept 2):
```markdown
## Summary
Added multiplication function to calculator for basic arithmetic operations.

## Changes
- calculator.py: Added multiply(a, b) function

## AI Assistance
**AI Tool Used**: [Optional - only if AI helped]

**What AI Generated**:
- [If no AI: "N/A - I wrote this function manually"]

**What I Modified**:
- [If no AI: "N/A - not applicable"]

## Testing Done
- Manual testing: multiply(3, 4) = 12 ✓
- Manual testing: multiply(0, 5) = 0 ✓
- Manual testing: multiply(-2, 3) = -6 ✓
```

### Step 4: Create Pull Request

Click "Create pull request" button. GitHub creates your PR!

**Congratulations!** Your first PR is created.

---

## Hands-On Activity 3: Review PR Diff

Now review what you just created.

### Step 1: Click "Files Changed" Tab

On your PR page, click "Files changed" tab. You'll see:

```diff
@@ -5,6 +5,9 @@ def divide(a, b):
     return a / b

+def multiply(a, b):
+    """Multiply two numbers."""
+    return a * b

 if __name__ == "__main__":
```

### Step 2: Verify Changes

Ask yourself:
- ✅ "Does this do what I intended?" (add multiplication)
- ✅ "Is the function correct?" (a * b is correct multiplication)
- ✅ "Is there any code I don't understand?" (No—I wrote it)
- ✅ "Are there bugs?" (No—tested manually)

### Step 3: Review Your Description

Scroll down and verify:
- Title is clear
- Summary explains the change
- AI transparency section (if applicable) is filled out
- Testing section shows what you tested

**If you're satisfied**: Ready to merge!

---

## Hands-On Activity 4: Merge Pull Request

Finally, merge your PR into main.

### Step 1: Look for "Merge Pull Request" Button

On your PR page, scroll down. You'll see:

**"This branch has no conflicts with the main branch."**

Below that: **"Merge pull request"** button

### Step 2: Click Merge

Click the green "Merge pull request" button.

GitHub will ask for confirmation. Click "Confirm merge".

### Step 3: Verify Merge Success

GitHub shows: **"Pull request successfully merged and closed"**

Your PR is now merged! Your feature branch code is now part of main.

### Step 4: Verify on Main Branch

Go back to your project folder and verify:

```bash
# Switch to main
git checkout main

# Pull latest changes
git pull

# Verify new function is there
# (Look at your code file—multiply function should exist)
```

**Success!** You've completed full PR workflow.

---

## The PR Transparency Pattern: AI-Generated Code Example

Let's look at a complete, realistic example with AI-generated code.

### Scenario

You asked ChatGPT to generate error handling for your calculator. Here's what a professional PR looks like.

**Feature Branch**: `feature/error-handling`

**PR Title**:
```
feat: add comprehensive error handling to calculator
```

**PR Description**:

```markdown
## Summary
Added error handling to calculator functions to gracefully handle invalid input and prevent crashes.

## Changes
- calculator.py: Added try/except blocks to divide(), subtract(), add(), multiply()
- calculator.py: Added new validate_input() function
- calculator.py: Added user-friendly error messages

## AI Assistance
**AI Tool Used**: ChatGPT (Claude Code interface)

**What AI Generated**:
- Complete try/except blocks for all arithmetic functions
- validate_input() function with regex pattern matching
- Default error message templates

**What I Modified & Why**:
1. Error Messages: Made them simpler
   - AI generated: "ValueError: Input does not match float pattern"
   - I changed to: "Error: Please enter a number"
   - Why: Users need plain language, not technical jargon

2. Edge Cases: Found bugs AI missed
   - AI didn't handle empty input ("" string)
   - I added explicit check: `if not input_str: raise ValueError("Input required")`
   - Why: AI generates good patterns but needs human validation for edge cases

3. Logging: Added debug output
   - AI didn't include logging
   - I added print statements for debugging
   - Why: Helps me see what's happening when errors occur

## Testing Done
- **Valid inputs**: 5 + 3, 10 - 2, 4 * 3, 20 / 4 ✓ All work
- **Edge cases**:
  - Empty input ("") → Correctly shows "Input required" ✓
  - Text input ("hello") → Correctly shows "Please enter a number" ✓
  - Division by zero (5 / 0) → Correctly shows "Cannot divide by zero" ✓
  - Very large numbers (999999999) → Works correctly ✓
  - Negative numbers (-5, -3) → Works correctly ✓

## Notes for Reviewer
This PR shows the collaboration pattern recommended for AI-native development:
1. Let AI generate initial implementation (faster)
2. Review and test thoroughly (your expertise)
3. Fix edge cases AI missed (common pattern)
4. Document what AI did vs. what you did (transparency)

The code is ready for merge—all tests passed, all edge cases handled.
```

---

### Diff for This PR

```diff
@@ -5,6 +5,29 @@ def add(a, b):
     return a + b

 def divide(a, b):
+    """Safely divide with error handling."""
+    try:
+        if b == 0:
+            raise ValueError("Cannot divide by zero")
+        return a / b
+    except Exception as e:
+        print(f"Error: {e}")
+        return None

 def subtract(a, b):
     return a - b

+def validate_input(value):
+    """Check if string is a valid number."""
+    if not value:  # Handle empty input—fixed AI gap!
+        raise ValueError("Input required")
+    try:
+        float(value)
+        return True
+    except ValueError:
+        raise ValueError("Please enter a number")
+
 if __name__ == "__main__":
     print(add(5, 3))
```

**Key Observations**:
- ✅ AI generated try/except pattern (good foundation)
- ✅ Human added division by zero check (specific to this function)
- ✅ Human added empty input check (edge case AI missed)
- ✅ Both contributed to final solution

This is **professional AI-native development**: AI does heavy lifting, you add expertise.

---

## PR Best Practices

When creating PRs with AI-generated code:

### Do:
- ✅ Test code before opening PR
- ✅ Review diffs carefully (especially AI code)
- ✅ Document what AI generated vs. what you modified
- ✅ Test edge cases—AI often misses them
- ✅ Add error messages that are user-friendly

### Don't:
- ❌ Merge PRs without reviewing diffs
- ❌ Hide or omit AI usage (transparency is strength)
- ❌ Trust AI code without testing
- ❌ Copy-paste AI code without understanding it
- ❌ Claim credit for AI-generated code (be honest)

---

## Troubleshooting Common PR Issues

### Issue: "This branch has conflicts with the main branch"

**Cause**: Someone else merged changes to main that conflict with your branch.

**Solution**:
1. Pull latest main: `git checkout main && git pull`
2. Merge main into your branch: `git checkout feature/your-feature && git merge main`
3. Resolve conflicts in your editor
4. Commit merge: `git add . && git commit -m "merge: resolve main conflicts"`
5. Push: `git push`
6. GitHub will automatically detect resolution

### Issue: "Cannot merge—branch out of date"

**Cause**: Main has changes since you created your branch.

**Solution**:
1. Same as conflicts above
2. If no conflicts, just push and GitHub will auto-update

### Issue: "Pull request review requested"

**Cause**: Repository has review policy requiring someone else to approve before merge.

**Situation at Your Level**: This won't happen yet (you control your own repo), but in professional settings:
1. Someone reviews your PR
2. They comment on code or request changes
3. You update code and push—PR automatically updates
4. Once approved: they merge (or you do)

---

## Three Roles Summary: Your GitHub Journey

Looking back at this lesson:

**Role 1: AI as Teacher**
- AI suggested transparency includes specific details (not just "used AI")
- AI suggested documenting edge cases and bugs found

**Role 2: AI as Student**
- AI initially wrote technical error descriptions
- You taught AI that non-programmers need plain language
- AI adapted and simplified

**Role 3: AI as Co-Worker (Convergence)**
- You created basic PR structure
- AI suggested what to document in testing section
- You tested and found bugs
- Together, you created transparent, detailed PR
- Neither alone would reach this quality

**This pattern repeats in real development**: Collaboration > solo work.

---

## Try With AI

Now practice this with ChatGPT or Claude.

### Setup

Have your GitHub PR open (or create a simple feature branch with changes and open a PR).

### Prompts to Try

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
Here's the code diff from my pull request (changes to calculator.py):

[Paste your diff here]

Can you:
1. Summarize what changed
2. Identify any bugs or edge cases
3. Suggest improvements
```

**Expected Outcome**: AI acts like a code reviewer—catches issues you missed, suggests improvements.

---

### Safety Note

When asking AI to review code:
- AI might suggest features you didn't intend (safe to ignore)
- AI might miss obvious bugs (review AI's review!)
- Always test code yourself before merging
- Don't trust AI reviews as replacement for your own judgment

---

### Optional Challenge

If you're feeling confident:

**Challenge Prompt**:
```
I want to practice the full GitHub workflow. Here's my feature idea:

[Describe a simple feature]

Can you:
1. Suggest what the PR description should include
2. Walk me through steps to create and merge PR
3. Show me what the diff might look like

Then I'll implement it myself and share the real PR with you for feedback.
```

**Expected Outcome**: You orchestrate entire workflow with AI guidance, then execute independently.

---

## Summary: Professional Code Review & Transparency

You've learned:

1. **Pull Requests**: Formal way to propose code changes on GitHub
2. **PR Descriptions**: Document WHAT changed, WHY, and HOW you tested it
3. **AI Transparency**: Always document which AI helped, what it generated, what you modified
4. **Diff Review**: Always look at changes before merging (AI code especially)
5. **Three Roles**: AI teaches best practices, you refine for your context, convergence improves quality

**Success Indicator**: You create PRs with clear titles, detailed descriptions including AI transparency, review diffs before merging, and confidently merge into main.

**Next Lesson**: In Lesson 6, you'll identify recurring Git patterns and create reusable workflow documentation (git-workflow.md) so you never re-learn the same Git workflows twice.

---

**Questions? Ask ChatGPT**: Use the "Try With AI" prompts above, or ask:

*"I'm learning GitHub pull requests. I just created my first PR. Can you explain why code review is important in professional development?"*

Learn from the explanation. Then come back to the hands-on activities above and create your own PR.

**Your Turn**: Create a PR today. Document AI assistance. Review the diff. Merge it.

That's the GitHub workflow professionals use every day.

