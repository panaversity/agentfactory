---
title: "Anatomy of Effective Prompts"
chapter: 10
lesson: 2
part: 3
duration_minutes: 35

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Prompt Structure Mastery"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student manually structures prompts using Intent → Constraints → Success Criteria pattern"

learning_objectives:
  - objective: "Analyze prompt pairs (vague vs specific) identifying why specific prompts succeed"
    proficiency_level: "A2"
    bloom_level: "Analyze"
    assessment_method: "Comparison exercise with reasoning explanation"

  - objective: "Manually structure prompts using Intent → Constraints → Success Criteria pattern"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "3 prompt rewrites with structure annotations"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (structure components, action verbs, specificity, constraints, success criteria, anti-patterns) within A2 limit of 5-7 ✓"
---

# Anatomy of Effective Prompts

In Lesson 1, you learned that prompts are specifications. Now you'll learn the **anatomy**—the structural components that transform vague requests into precise, executable specifications.

Think of this like learning the parts of a blueprint. A blueprint isn't just "draw a building"—it has specific components: foundation plan, electrical layout, plumbing schematic, elevation views. Each component serves a purpose. Miss one, and the contractor can't build correctly.

**Prompts work the same way.** They have structural components that, when present, guide AI to produce exactly what you need. When these components are missing, AI fills gaps with guesses—and guesses produce unreliable outputs.

By the end of this lesson, you'll be able to look at any prompt and immediately identify what's missing, what's ambiguous, and how to structure it for maximum clarity.

---

## The Three-Part Prompt Structure

Every effective prompt contains three essential components:

```
INTENT → CONSTRAINTS → SUCCESS CRITERIA
```

**Intent**: WHAT you want AI to produce (the goal)
**Constraints**: Rules that MUST be followed (boundaries)
**Success Criteria**: How you'll know the output is good (validation)

Let's break down each component.

---

## Component 1: Intent (WHAT)

**Intent** answers the question: *What should AI produce?*

This is where you specify the **desired outcome**, not the process. You're defining the target, not the path.

### Weak Intent (Vague)

```
"Help me with Git"
```

**What's wrong?**
- Help how? (Tutorial? Command explanation? Bug fix?)
- Help with what Git task specifically?
- What's the end goal?

### Strong Intent (Clear)

```
WHAT: Explain the difference between 'git merge' and 'git rebase'
      for integrating feature branches into main branch
```

**Why this works:**
- Specific Git concept (merge vs rebase)
- Specific use case (feature branch integration)
- Clear output expected (explanation comparing two approaches)

---

### Technical Action Verbs: The Language of Intent

When writing intent, use **technical action verbs** that communicate precisely what AI should do.

Here are the eight core developer verbs:

#### 1. **Create** — Build something new from scratch

**Use when**: Starting new functions, files, features, or components

**Example**:
```
CREATE a Bash script that backs up all .md files from /docs to /backups/YYYY-MM-DD/
```

**Not**:
```
"I need a backup thing" ❌ (Vague—create what?)
```

---

#### 2. **Debug** — Identify and fix specific problems

**Use when**: Code crashes, throws errors, or produces wrong output

**Example**:
```
DEBUG the file permission error in backup.sh occurring when running as non-root user
```

**Not**:
```
"Fix the script" ❌ (Which script? What's broken?)
```

---

#### 3. **Refactor** — Improve code structure without changing behavior

**Use when**: Code works but is hard to read, has duplication, or needs cleanup

**Example**:
```
REFACTOR the nested if statements in validate_email.sh to use case statements,
preserving all validation logic
```

**Not**:
```
"Make this better" ❌ (Better how? Can behavior change?)
```

---

#### 4. **Analyze** — Examine code to identify patterns, issues, or improvements

**Use when**: Reviewing code, finding security issues, understanding complex logic

**Example**:
```
ANALYZE this Git repository's commit history for commits missing conventional
commit format, listing commit hashes and messages
```

**Not**:
```
"Look at this repo" ❌ (Look for what specifically?)
```

---

#### 5. **Optimize** — Improve performance, speed, or resource usage

**Use when**: Code is slow, uses too much memory, or has bottlenecks

**Example**:
```
OPTIMIZE the log parsing script to process 10,000 log lines in under 5 seconds
by using grep instead of line-by-line reading
```

**Not**:
```
"Speed this up" ❌ (By how much? What approach?)
```

---

#### 6. **Generate** — Produce boilerplate, templates, or repetitive content

**Use when**: Creating tests, documentation, configuration files, or templates

**Example**:
```
GENERATE markdown documentation for all Bash functions in utils.sh,
including function signature, parameters, and usage examples
```

**Not**:
```
"Write docs" ❌ (Docs for what? What format?)
```

---

#### 7. **Explain** — Describe how code works or why it's structured a certain way

**Use when**: Understanding unfamiliar code, onboarding, or creating tutorials

**Example**:
```
EXPLAIN how the git rebase --interactive command works, including what happens
to commits during the rebase process and common use cases
```

**Not**:
```
"What does this do?" ❌ (Too vague—what level of detail?)
```

---

#### 8. **Validate** — Check if code meets requirements or standards

**Use when**: Verifying correctness, compliance, or quality standards

**Example**:
```
VALIDATE that all Git commit messages in the last 20 commits follow
conventional commits format (<type>(<scope>): <description>)
```

**Not**:
```
"Check if this is right" ❌ (Right by what standard?)
```

---

### Verb Selection Exercise

**Match the task to the correct verb**:

1. You want AI to check if your Bash script follows best practices → ?
2. You want AI to build a new Git hook that runs tests before commit → ?
3. You want AI to make your documentation clearer without changing technical accuracy → ?
4. You want AI to find why your script exits with code 127 → ?

**Answers**:
1. **Validate** (checking against standards)
2. **Create** (building something new)
3. **Refactor** (improving structure, same content)
4. **Debug** (finding cause of error)

---

## Component 2: Constraints (MUST/MUST NOT)

**Constraints** are the rules AI must follow. They define boundaries, requirements, and limitations.

Without constraints, AI makes assumptions that may not match your needs.

### Types of Constraints

#### Technical Constraints
```
CONSTRAINTS:
- MUST use only Bash built-ins (no external commands)
- MUST handle filenames with spaces
- MUST work on both Linux and macOS
```

#### Scope Constraints
```
CONSTRAINTS:
- MUST NOT modify existing files
- MUST ONLY process .md files (ignore other types)
- MUST preserve directory structure
```

#### Quality Constraints
```
CONSTRAINTS:
- MUST include error handling for all file operations
- MUST follow Google Shell Style Guide
- MUST include usage documentation in comments
```

#### Format Constraints
```
CONSTRAINTS:
- MUST output results as JSON
- MUST use ISO 8601 date format (YYYY-MM-DD)
- MUST log to /var/log/backup.log
```

---

### Example: Constraints Transform Vague to Precise

**Without Constraints** (AI guesses):
```
CREATE a backup script
```

*AI might create:*
- Python script (you wanted Bash)
- Backs up everything (you wanted only .md files)
- Overwrites backups (you wanted versioned backups)
- No error handling (production-breaking)

---

**With Constraints** (AI knows exactly what to build):
```
CREATE a backup script

CONSTRAINTS:
- MUST be Bash script (no Python/Ruby)
- MUST back up ONLY .md files from /docs
- MUST create new backup folder with timestamp (backup_YYYY-MM-DD_HH-MM-SS)
- MUST NOT overwrite existing backups
- MUST log all operations to /var/log/backup.log
- MUST handle file permission errors gracefully (don't crash on single file failure)
- MUST run on macOS (use compatible commands)
```

*AI now produces exactly what you need.*

---

## Component 3: Success Criteria (HOW TO VALIDATE)

**Success Criteria** answer: *How do I know the output is good?*

This is where Jake Heller's advice comes in:

> "Define 'what good looks like' for every step. Create objective tests."
> — Jake Heller [[15:27-18:16]](https://www.youtube.com/watch?v=l0h3nAW13ao&t=927s)

Success criteria are **measurable, testable conditions** that validate AI output.

### Weak Success Criteria (Subjective)

```
- Should work well
- Should be fast
- Should be clean code
```

**Problem**: "Well," "fast," and "clean" are subjective. How do you test these?

---

### Strong Success Criteria (Measurable)

```
SUCCESS CRITERIA:
- Backs up all 47 .md files from /docs (verified by count)
- Creates backup folder with timestamp in format backup_YYYY-MM-DD_HH-MM-SS
- Completes backup of 1000 files in under 10 seconds
- Passes shellcheck with zero warnings
- Handles test case: file with spaces in name
- Handles test case: read-only file (logs error, continues)
```

**Why this works**: Each criterion is **testable**. You can objectively verify pass/fail.

---

### Success Criteria Types

#### Functional Criteria (Does it do what it should?)
```
- Backs up all .md files (none missed)
- Preserves file modification timestamps
- Returns exit code 0 on success, non-zero on failure
```

#### Performance Criteria (Is it fast enough?)
```
- Processes 1000 files in \<10 seconds
- Uses \<50MB memory during execution
- Completes incremental backup in \<2 seconds
```

#### Quality Criteria (Is the code good?)
```
- Passes shellcheck with zero warnings
- All functions have documentation comments
- Follows Google Shell Style Guide
```

#### Robustness Criteria (Does it handle edge cases?)
```
- Handles filenames with spaces, special characters
- Handles permission errors gracefully (logs, continues)
- Handles disk full scenario (stops, reports error)
```

---

## Putting It All Together: The Complete Prompt Anatomy

Let's see how **Intent → Constraints → Success Criteria** creates a complete, specification-quality prompt.

**Task**: You need a Git commit message generator for your project.

---

### Vague Version (Missing Components)

```
"Create a commit message"
```

**Missing**:
- Intent unclear (message for what changes?)
- No constraints (what format? what style?)
- No success criteria (what makes it "good"?)

---

### Complete Version (All Components Present)

```
INTENT:
GENERATE a Git commit message for changes in the current staging area

CONSTRAINTS:
- MUST follow Conventional Commits format: <type>(<scope>): <description>
- MUST use type from: feat, fix, docs, refactor, test, chore
- MUST include scope (e.g., "auth", "api", "docs")
- MUST write description in imperative mood ("add feature" not "added feature")
- MUST limit subject line to 50 characters
- MUST include body listing each changed file with brief explanation
- MUST NOT use generic descriptions ("update code", "fix bug")

SUCCESS CRITERIA:
- Message passes commitlint validation
- Subject line ≤ 50 characters
- Body explains WHY changes were made (not just WHAT changed)
- Teammate can understand changes without reading code diff
- Message follows team's existing commit message style
```

**Why this works:**
- **Intent**: Clear goal (conventional commit message)
- **Constraints**: Format specified, rules defined
- **Success Criteria**: Measurable validation (commitlint, character count, peer understandability)

---

## Anti-Patterns: What Kills Prompt Effectiveness

These are the common mistakes that turn specifications into vague requests:

### Anti-Pattern 1: Ambiguous Language

**❌ Bad**:
```
"Make the script more robust"
```

**What's wrong**: "Robust" means different things (error handling? input validation? performance? all of these?)

**✅ Good**:
```
"Add error handling for: file not found, permission denied, disk full. Script should log error and continue processing remaining files."
```

---

### Anti-Pattern 2: Missing Context

**❌ Bad**:
```
"Optimize this"
```

**What's wrong**: Optimize for what? Speed? Memory? Readability? No code provided.

**✅ Good**:
```
"Optimize backup.sh to process 10,000 files in \<5 seconds (currently takes 30 seconds). Bottleneck is line-by-line file reading. Suggest using find + xargs or parallel processing."
```

---

### Anti-Pattern 3: No Validation Criteria

**❌ Bad**:
```
"Create tests for the backup script"
```

**What's wrong**: What tests? How many? What scenarios?

**✅ Good**:
```
"Create Bash tests using bats framework covering:
- Happy path: 10 .md files backed up successfully
- Edge case: filename with spaces
- Error case: source directory doesn't exist
- Error case: destination directory not writable
All tests must pass with exit code 0."
```

---

### Anti-Pattern 4: Prescribing Implementation (Too Specific)

**❌ Bad** (Over-specified):
```
"Use a for loop iterating over files, then use cp with -p flag to copy each file, then use echo to print status"
```

**What's wrong**: You're telling AI **HOW** to implement, not **WHAT** to achieve. This removes AI's ability to suggest better approaches.

**✅ Good** (Specification-level):
```
"Back up all .md files preserving timestamps and permissions. Log each file copied."
```

*Let AI decide whether to use `for` + `cp`, `rsync`, `tar`, or another approach. Specify WHAT, not HOW.*

---

## Prompt Rewrite Exercise

**Your task**: Rewrite these three vague prompts into specification-quality prompts with **Intent → Constraints → Success Criteria**.

Do this on paper or in a markdown file BEFORE looking at the answers.

---

### Exercise 1: Vague Prompt

```
"Help me with Git branches"
```

**Your rewrite** (include Intent, Constraints, Success Criteria):

---

### Exercise 2: Vague Prompt

```
"Make documentation for this script"
```

**Your rewrite**:

---

### Exercise 3: Vague Prompt

```
"Fix the backup script"
```

**Your rewrite**:

---

## Example Solutions

### Exercise 1: Git Branches

**Specification-Quality Rewrite**:

```
INTENT:
EXPLAIN Git branching workflow for feature development, specifically:
- How to create feature branch from main
- How to commit changes to feature branch
- How to merge feature branch back to main
- How to handle merge conflicts

CONSTRAINTS:
- Assume reader knows basic Git (add, commit, push)
- Use command-line Git (not GUI tools)
- Include example commands for each step
- Explain WHEN to use merge vs rebase

SUCCESS CRITERIA:
- Reader can create and merge feature branch after reading
- All commands are copy-pasteable and working
- Merge conflict resolution explained with example
- Explanation takes \<5 minutes to read
```

---

### Exercise 2: Documentation

**Specification-Quality Rewrite**:

```
INTENT:
GENERATE markdown documentation for backup.sh script

CONSTRAINTS:
- Format: Markdown with code blocks
- Sections: Overview, Requirements, Usage, Examples, Error Handling
- Audience: Developers who will use/modify this script
- Include actual script path, invocation syntax, and exit codes
- Show example invocations with expected output

SUCCESS CRITERIA:
- Developer can run script correctly after reading docs
- All command examples are tested and working
- Error codes documented (what each exit code means)
- Documentation \<200 lines (concise, not exhaustive)
```

---

### Exercise 3: Fix Backup Script

**Specification-Quality Rewrite**:

```
INTENT:
DEBUG backup.sh script that fails with "Permission denied" error when backing up files to /backups

CONSTRAINTS:
- Script runs as user 'developer' (not root)
- Backup destination /backups is owned by root:admin
- MUST NOT change destination ownership (security requirement)
- MUST suggest solution that works with current permissions
- Preserve all existing functionality (file filtering, timestamp creation)

SUCCESS CRITERIA:
- Script successfully backs up files to /backups as non-root user
- No "Permission denied" errors in logs
- All .md files from /docs backed up correctly
- Test case: Run script as 'developer' user, verify backup created
```

---

## Key Pattern Recognition

By analyzing these rewrites, notice the pattern:

**Vague prompts are questions**: "Help me with X?"
**Specification prompts are requirements**: "Achieve Y under Z constraints validated by W criteria."

**Vague prompts focus on HOW**: "Use a loop to..."
**Specification prompts focus on WHAT**: "Back up all files..."

**Vague prompts omit validation**: No mention of how to verify success
**Specification prompts include tests**: Measurable criteria for verification

---

## Summary: The Anatomy Checklist

Before sending any prompt to AI, ask yourself:

**Intent (WHAT)**:
- [ ] Have I specified the exact output I want?
- [ ] Have I used a technical action verb (Create, Debug, Refactor, etc.)?
- [ ] Is it clear what "done" looks like?

**Constraints (MUST/MUST NOT)**:
- [ ] Have I specified technical requirements?
- [ ] Have I defined scope boundaries?
- [ ] Have I noted quality standards?
- [ ] Have I specified format/structure?

**Success Criteria (VALIDATION)**:
- [ ] Have I defined measurable pass/fail conditions?
- [ ] Can I objectively test if output meets criteria?
- [ ] Have I included edge cases and error scenarios?

**If any checkbox is unchecked**, your prompt is incomplete. AI will fill gaps with guesses.

---

## What You've Learned

You now understand prompt anatomy at the structural level:

1. **Three-part structure** (Intent → Constraints → Success Criteria)
2. **Eight technical action verbs** (Create, Debug, Refactor, Analyze, Optimize, Generate, Explain, Validate)
3. **Constraint types** (technical, scope, quality, format)
4. **Success criteria characteristics** (measurable, testable, objective)
5. **Anti-patterns** (ambiguity, missing context, no validation, over-specification)

In Lesson 1, you learned that **prompts are specifications**. In this lesson, you learned **what components make specifications complete**.

In the next lesson, you'll start working with AI directly to practice **iterative prompt refinement**—the process Jake Heller used to go from 60% to 97% accuracy. You'll see how AI can teach you better prompt patterns, how you teach AI your specific requirements, and how iteration produces results neither of you could achieve alone.

**You're building the specification mindset step by step.** Manual analysis first (Lessons 1-2), then AI collaboration (Lessons 3-5), then reusable intelligence creation (Lessons 6-7), then spec-driven orchestration (Lesson 8).

Keep practicing: Before your next AI interaction, sketch out Intent → Constraints → Success Criteria on paper. You'll be amazed how much better your results become.
