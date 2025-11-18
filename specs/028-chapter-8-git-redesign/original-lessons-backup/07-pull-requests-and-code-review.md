---
sidebar_position: 7
title: "Pull Requests and Code Review"
description: "Create pull requests documenting AI assistance, review changes, address feedback, and merge professionally"

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Create Pull Request"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student can create a pull request with clear title, description, testing instructions, and documentation of AI assistance"

  - name: "Review Code Changes"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can examine code diffs in a PR, understand what changed, and provide feedback on specific lines"

  - name: "Address Feedback"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Communication"
    measurable_at_this_level: "Student can update code based on review feedback, commit changes, and have PR automatically reflect updates"

  - name: "Merge Safely"
    proficiency_level: "A1"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can merge PR after approval and delete the feature branch from GitHub"

learning_objectives:
  - objective: "Create pull requests documenting changes and AI assistance"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student writes clear PR description with title, what changed, why, testing instructions, and AI assistance documentation"

  - objective: "Review code changes in a pull request by examining diffs"
    proficiency_level: "A2"
    bloom_level: "Analyze"
    assessment_method: "Student examines PR diff, identifies specific changes, and provides actionable feedback"

  - objective: "Handle code review feedback and update pull requests"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student makes requested changes, commits, and pushes (PR auto-updates)"

  - objective: "Merge pull requests and clean up branches"
    proficiency_level: "A1"
    bloom_level: "Remember"
    assessment_method: "Student merges PR and deletes feature branch after approval"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (Pull Request, Diff, Code Review, Discussion, Merge) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Explore advanced PR practices: code review standards, addressing specific comment threads, using PR templates for documentation consistency"
  remedial_for_struggling: "Focus on the pull request as a 'conversation with your code' using the given template; practice reviewing a simple PR diff before creating one"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/012-chapter-8-git-github-aidd/plan.md"
created: "2025-11-05"
last_modified: "2025-11-07"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Pull Requests and Code Review

You tested changes on your branch. Everything works! Now what?

**Bad approach**: Just merge to main and hope.

**Professional approach**: Create a Pull Request (PR) - final safety check before merging.

**Time**: 20 minutes

---

## What Is a Pull Request?

A **Pull Request** says: "I'm done with these changes. Please review before merging to main."

**Simple flow**:

```
Branch (your changes) → Pull Request → Review → Merge to Main
```

**Why this matters**:
- Last review chance before merging
- Shows exactly what changed (diff view)
- Documents why you made changes
- Professional workflow
- Transparent about AI assistance

---

## The PR Workflow

### Step 1: Push Your Branch to GitHub

Get your branch from local computer to GitHub.

**You ask Gemini CLI**: "Push my 'add-calculator' branch to GitHub"

Gemini runs: `git push -u origin add-calculator`

Branch uploads to GitHub.

**Check it worked**: Visit your repository on GitHub - you'll see a yellow banner about recent pushes.

---

### Step 2: Create Pull Request on GitHub

**On GitHub website**:

1. Look for yellow banner: "add-calculator had recent pushes"
2. Click green **"Compare & pull request"** button
3. You'll see the PR creation form

**OR**:

1. Click "Pull requests" tab
2. Click green "New pull request" button
3. Select your branch from dropdown

---

### Step 3: Write PR Description

**What to include**:

**Title** (short summary):
- Example: "Add calculator module"
- Keep it 5-10 words

**Description** (answer 3 questions):

1. **What changed?**
   - List main changes
   - Be specific

2. **What did AI do?**
   - Document AI's role
   - Be transparent

3. **How to test?**
   - Give simple steps
   - Anyone should be able to verify

---

**Example PR description**:

```markdown
## What Changed
Added calculator module with 4 functions:
- add() - adds two numbers
- subtract() - subtracts two numbers
- multiply() - multiplies two numbers
- divide() - handles division by zero

## AI Assistance
- Gemini CLI generated all 4 functions
- Created unit tests (8 tests)
- Added error handling for edge cases
- Fixed bugs during testing phase

## How to Test
Run: `python test_calculator.py`
Expected: All 8 tests pass
```

**You ask Gemini CLI**: "Help me write a PR description for my calculator changes"

Gemini will generate a template based on your commits and changes.

---

### Step 4: Review the Diff

GitHub shows a **diff** (difference view):

- **Green lines** (+): Code added
- **Red lines** (-): Code removed
- **Gray lines**: Context (unchanged)

**Check**:
- Does the diff look correct?
- Any unintended changes?
- All files you meant to include?

---

### Step 5: Create the PR

Click **"Create Pull Request"** button.

PR is now live! Anyone can:
- See your changes
- Leave comments
- Approve or request changes

---

## Handling Feedback

Someone leaves a comment: "Add better error messages"

**To update your PR**:

1. Make changes locally on your branch
2. Commit the changes:

**Ask Gemini CLI**: "Commit my improvements with message 'Add better error messages'"

Gemini runs: `git commit -m "Add better error messages"`

3. Push to GitHub:

**Ask Gemini CLI**: "Push my updates to GitHub"

Gemini runs: `git push`

**PR automatically updates** with your new commits. No need to create a new PR.

---

## Merging the Pull Request

**When to merge**:
- All tests pass
- You've reviewed the changes
- Any feedback is addressed
- (In teams: after approval from reviewers)

**On GitHub**:

1. Go to your PR page
2. Click green **"Merge pull request"** button
3. Click **"Confirm merge"**

Done! Changes are now in `main`.

---

## After Merging: Clean Up

**Delete the branch on GitHub**:

GitHub shows "Delete branch" button after merge - click it.

**Delete the branch locally**:

**You ask Gemini CLI**: "Delete my local 'add-calculator' branch"

Gemini runs:
1. `git checkout main` (switch to main)
2. `git pull` (get the merged changes)
3. `git branch -d add-calculator` (delete local branch)

Your workspace is clean!

---

## Complete Example

**You**: "I finished my calculator. Write a professional PR description."

**You ask Gemini CLI**: "Generate a PR description for my calculator changes"

Gemini creates:

```markdown
## Add Calculator Module

### Changes
- Created calculator.py with 4 operations
- Added comprehensive test suite
- Implemented error handling for division by zero

### AI Assistance
- Generated initial function implementations
- Created unit test framework
- Suggested error handling patterns
- Fixed edge case bugs

### Testing
Run: `python test_calculator.py`
All 8 tests should pass

### Files Changed
- calculator.py (new)
- test_calculator.py (new)
- README.md (updated usage instructions)
```

Copy this into GitHub's PR description field.

---

## Good vs Bad PR Descriptions

**Bad** ❌:
- Title: "stuff"
- Description: "added things"
- No mention of AI
- No test instructions

**Good** ✅:
- Title: "Add calculator module"
- Description: Lists changes, AI role, testing steps
- Clear and professional
- Anyone can understand what happened

---

## Key Commands Reference

| Task | Command |
|------|---------|
| Push branch to GitHub | `git push -u origin branch-name` |
| Push updates to PR | `git push` (while on branch) |
| Switch to main | `git checkout main` |
| Pull merged changes | `git pull` |
| Delete local branch | `git branch -d branch-name` |
| Force delete branch | `git branch -D branch-name` |

Most PR actions happen on GitHub website (create, merge, comment).

---

## Safety Guidelines

**Always**:
- Test before creating PR
- Write clear descriptions
- Document AI assistance
- Review the diff before creating
- Clean up branches after merge

**Never**:
- Merge PRs with failing tests
- Leave vague descriptions
- Hide AI's role
- Merge without reviewing changes
- Keep old branches forever

---

## Common Mistakes

**Mistake 1: Vague Titles**
- Bad: "Update"
- Good: "Add calculator with 4 operations"

**Mistake 2: No AI Documentation**
- Bad: Silent about AI help
- Good: "AI generated functions, I tested and reviewed"

**Mistake 3: Merging Too Fast**
- Bad: Create PR, immediately merge (no review)
- Good: Create PR, review diff, test, then merge

**Mistake 4: Not Cleaning Up**
- Bad: 20 old merged branches cluttering your repo
- Good: Delete branches after merge

---

#### 💬 AI Colearning Prompt
> "Why create a Pull Request instead of just merging branches directly? What problem does the PR review process solve?"

---

#### 🎓 Expert Insight
> In AI-native development, you don't memorize PR creation steps—you understand review intent. When changes are ready for main, you specify "create a PR documenting my changes and AI assistance" and AI helps generate professional descriptions. Your value is explaining WHAT changed and WHY (business context, testing approach), not clicking GitHub buttons in the right order.

---

#### 🤝 Practice Exercise

> **Ask your AI**: "Help me create a professional PR description for a calculator feature. Include: what changed, how AI helped, and testing instructions. Then explain why each section matters for reviewers."

**Expected Outcome**: You'll understand PRs aren't about mechanics—they're about communication. Description = tells reviewers what to expect. AI documentation = transparency about AI role. Testing instructions = makes review faster and safer.

---

## Try With AI

Practice the PR workflow.

**Tool**: Gemini CLI (or Claude Code, ChatGPT)

### Exercise 1: Create Your First PR

```
I have a branch "add-calculator" pushed to GitHub.
Walk me through creating a Pull Request:
1. Where do I click on GitHub?
2. What do I write in title and description?
3. How do I submit it?
```

### Exercise 2: Write Professional Description

```
Generate a PR description for:
- Calculator with add, subtract, multiply, divide
- AI generated all code
- 8 tests, all passing

Make it professional but simple.
```

### Exercise 3: Update a PR

```
Someone commented: "Add input validation"
Guide me through:
1. Making the changes locally
2. Committing them
3. Updating the PR on GitHub
```

### Exercise 4: Merge and Clean Up

```
My PR is approved! Help me:
1. Merge it on GitHub
2. Delete the remote branch
3. Delete my local branch
4. Get the merged changes locally
```

### Exercise 5: Review Process

```
Explain what happens when:
1. I push a branch to GitHub
2. I create a PR from that branch
3. I push more commits to the same branch
4. I merge the PR

Does the PR update automatically?
```
