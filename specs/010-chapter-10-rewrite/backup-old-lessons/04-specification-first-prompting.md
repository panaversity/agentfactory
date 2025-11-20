---
title: "Specification-First Prompting"
chapter: 10
lesson: 4
part: 3
duration_minutes: 40

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Specification-First Prompting"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student writes success criteria BEFORE prompting AI"

learning_objectives:
  - objective: "Write success criteria and constraints BEFORE prompting AI"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Specification document for Bash backup script"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (specification-first principle, success criteria, acceptance tests, constraints, non-goals, output format, validation) at B1 threshold ✓"
---

# Specification-First Prompting

Here's a mistake that wastes hours of iteration time: **prompting AI before you know what success looks like**.

You've probably done this:

> "AI, create a backup script for my project files."

AI generates code. You run it. Something's wrong. You don't know if the problem is:
- Your unclear prompt?
- AI's implementation choice?
- Missing requirements you forgot to mention?

**The fix**: Write the specification BEFORE writing the prompt.

This lesson teaches Jake Heller's core principle from building CoCounsel (the $650M legal AI product):

> "Define 'what good looks like' for every step. Create objective tests."
>
> — Jake Heller, Founder of Casetext [[15:27-18:16]](https://www.youtube.com/watch?v=l0h3nAW13ao&t=927s)

By the end of this lesson, you'll write **specification documents** that define success criteria, constraints, and validation tests—BEFORE you prompt AI to implement anything. This transforms prompting from "hope AI guesses right" into "AI implements against clear requirements."

---

## The Specification-First Principle

Traditional prompting workflow:

```
1. Prompt AI with vague idea
2. Get implementation
3. Discover what you actually needed
4. Re-prompt with corrections
5. Repeat until exhausted
```

**Problem**: You're using AI as a requirements discovery tool. Expensive and frustrating.

---

**Specification-first workflow**:

```
1. Write specification document (WHAT you need)
2. Validate specification completeness
3. Prompt AI with specification
4. AI implements correctly (first or second try)
```

**Benefit**: AI becomes implementation tool, not requirements guesser. Faster, clearer results.

---

## What Is a Specification Document?

A specification defines **WHAT** should exist, not **HOW** to build it.

**Specification document structure**:

```markdown
WHAT: [What are you building?]

SUCCESS CRITERIA:
  - [Measurable outcome 1]
  - [Measurable outcome 2]
  - [Measurable outcome 3]

CONSTRAINTS:
  - [Limitation 1]
  - [Limitation 2]

NON-GOALS:
  - [What this does NOT need to do]

OUTPUT FORMAT:
  - [How results should be structured]

VALIDATION:
  - [Test 1: How to verify success]
  - [Test 2: How to verify success]
```

**Key insight**: This document **precedes** your prompt. You write this BEFORE asking AI to implement.

---

## Example: File Backup Script Specification

Let's say you need a backup script for project documentation. Here's the specification-first approach.

### Step 1: Write the Specification (Before Prompting)

```markdown
WHAT: Automated backup script for project markdown files

SUCCESS CRITERIA:
  - All .md files from /docs copied to /backups
  - Backup folder timestamped (backup_YYYY-MM-DD_HH-MM)
  - Log file created with backup summary
  - Returns count of files backed up

CONSTRAINTS:
  - Must preserve directory structure
  - Must skip hidden files (.git, .DS_Store, .env)
  - Must handle permission errors gracefully (log and continue)
  - Must work on macOS and Linux

NON-GOALS:
  - No compression (plain file copies only)
  - No remote/cloud backup (local only)
  - No incremental backup (full copy each time)

OUTPUT FORMAT:
  - Backup in: /backups/backup_2024-03-15_14-30/
  - Log file: /backups/backup_2024-03-15_14-30/backup.log
  - Log format: "✓ Copied: path/to/file.md" or "✗ Failed: path/to/file.md (reason)"

VALIDATION:
  - Test: Create test .md file, run script, verify file copied
  - Test: Create hidden file (.test), verify NOT copied
  - Test: Create read-only file, verify error logged gracefully
  - Test: Check log file contains accurate file count
```

### Step 2: Validate Specification Completeness

Before prompting AI, ask yourself:

**Completeness Questions**:
1. ✅ Are success criteria **measurable**? (Can I objectively test "all .md files copied"?)
2. ✅ Are constraints **specific**? (Not "handle errors well" but "log and continue on permission errors")
3. ✅ Are non-goals **explicit**? (AI won't waste time on compression if told "no compression")
4. ✅ Is output format **defined**? (Timestamp format specified: YYYY-MM-DD_HH-MM)
5. ✅ Are validation tests **actionable**? (I can literally perform each test)

If "no" to any → Specification incomplete. Refine before prompting.

### Step 3: Prompt AI With Specification

NOW you prompt AI. Your prompt becomes simple:

```markdown
Create a Bash script meeting this specification:

[Paste entire specification document]

Implementation requirements:
- Use rsync for file copying (preserves timestamps)
- Use find command to locate .md files
- Create backup directory if doesn't exist
- Return 0 on success, 1 on failure
```

**Notice**: The **specification** defines WHAT. The **prompt** adds HOW hints (rsync, find) but AI already knows the target.

### Step 4: AI Implements Against Clear Requirements

AI generates:

```bash
#!/bin/bash

# Configuration
SOURCE_DIR="./docs"
BACKUP_ROOT="./backups"
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M")
BACKUP_DIR="$BACKUP_ROOT/backup_$TIMESTAMP"
LOG_FILE="$BACKUP_DIR/backup.log"

# Create backup directory
mkdir -p "$BACKUP_DIR"

# Initialize log
echo "Backup started: $(date)" > "$LOG_FILE"

# Counter
count=0

# Find and copy .md files
while IFS= read -r -d '' file; do
  # Skip hidden files
  if [[ "$file" == */\.* ]]; then
    continue
  fi

  # Create target directory structure
  rel_path="${file#$SOURCE_DIR/}"
  target_dir="$BACKUP_DIR/$(dirname "$rel_path")"
  mkdir -p "$target_dir"

  # Copy file
  if cp "$file" "$target_dir/"; then
    echo "✓ Copied: $rel_path" | tee -a "$LOG_FILE"
    ((count++))
  else
    echo "✗ Failed: $rel_path (permission denied)" | tee -a "$LOG_FILE"
  fi
done < <(find "$SOURCE_DIR" -name "*.md" -print0)

# Summary
echo "Backup completed: $count files copied" | tee -a "$LOG_FILE"

exit 0
```

**What just happened**:
- AI had clear success criteria → generated exactly what's needed
- Constraints explicit → skips hidden files, handles errors gracefully
- Non-goals stated → no compression code wasted
- Output format defined → uses exact timestamp format
- Validation tests provided → you can verify immediately

**Result**: First implementation ~90% correct. Compare to vague prompting: ~60% correct after 3-4 iterations.

---

## The Specification Checklist

Before prompting AI for ANY implementation, validate your specification against this checklist:

### 1. Success Criteria (Measurable Outcomes)

**Question**: Can I objectively test whether each criterion is met?

**Examples**:

❌ **Vague**: "Script should be reliable"
✅ **Measurable**: "Script returns 0 on success, 1 on failure"

❌ **Vague**: "Backup should be organized"
✅ **Measurable**: "Backup folder named backup_YYYY-MM-DD_HH-MM"

❌ **Vague**: "Handle errors well"
✅ **Measurable**: "Permission errors logged with filename and reason, script continues"

### 2. Constraints (Limitations and Requirements)

**Question**: What MUST or MUST NOT happen?

**Examples**:

**Technical Constraints**:
- "Must work on macOS and Linux" (not Windows)
- "Use Bash 4+ features" (version requirement)

**Scope Constraints**:
- "Skip hidden files (.git, .DS_Store)"
- "Local backup only (no remote)"

**Quality Constraints**:
- "Preserve directory structure"
- "Log all operations"

### 3. Non-Goals (Explicit Exclusions)

**Question**: What should AI NOT implement (to avoid wasted effort)?

**Examples**:

- "No compression" → AI won't add gzip/tar logic
- "No incremental backup" → AI won't track file changes
- "No email notifications" → AI won't add SMTP code

**Why this matters**: AI models are trained on "common feature sets." If you're building a simple backup script, AI might add compression, email alerts, scheduling—features you don't need. Non-goals prevent scope creep.

### 4. Output Format (Structure of Results)

**Question**: Exactly how should results be presented?

**Examples**:

**File Structure**:
```
/backups/
  backup_2024-03-15_14-30/
    docs/
      chapter1.md
      chapter2.md
    backup.log
```

**Log Format**:
```
Backup started: 2024-03-15 14:30:22
✓ Copied: docs/chapter1.md
✓ Copied: docs/chapter2.md
Backup completed: 2 files copied
```

**Return Values**:
- Exit code 0 on success
- Exit code 1 on failure
- Stdout: Summary message
- Stderr: Error details

### 5. Validation Tests (Acceptance Criteria)

**Question**: How will I prove this works correctly?

**Test structure**:
```markdown
VALIDATION:
  - Test: [Setup] → [Action] → [Expected Result]
  - Test: [Setup] → [Action] → [Expected Result]
```

**Examples**:

```markdown
VALIDATION:
  - Test: Create test.md in /docs → Run script → Verify test.md in /backups/backup_*/docs/
  - Test: Create .hidden.md → Run script → Verify .hidden.md NOT copied
  - Test: Make file read-only → Run script → Verify error logged, script continues
  - Test: Run script twice → Verify two separate backup folders with timestamps
```

**Key principle**: If you can't write a test for a success criterion, the criterion is too vague.

---

## Common Specification Mistakes

### Mistake 1: Success Criteria Are Implementation Details

❌ **Wrong** (HOW, not WHAT):
```markdown
SUCCESS CRITERIA:
  - Use rsync command
  - Loop through files with find
```

*This is implementation, not outcome.*

✅ **Right** (WHAT outcome):
```markdown
SUCCESS CRITERIA:
  - All .md files copied to backup folder
  - Original file timestamps preserved

IMPLEMENTATION HINTS:
  - Suggest rsync (preserves timestamps)
```

*Specification defines outcome. Prompt can suggest HOW.*

---

### Mistake 2: Constraints Are Missing Edge Cases

❌ **Wrong** (vague constraint):
```markdown
CONSTRAINTS:
  - Handle errors appropriately
```

*What's "appropriately"? AI will guess.*

✅ **Right** (specific behavior):
```markdown
CONSTRAINTS:
  - Permission errors: Log error, continue to next file
  - Disk full: Log error, exit with code 1
  - Source directory missing: Print error message, exit with code 1
```

---

### Mistake 3: Non-Goals Assumed (Not Stated)

❌ **Wrong** (assumption):
```markdown
[Non-goals section omitted]
```

*AI adds compression, cloud sync, email alerts (common backup features).*

✅ **Right** (explicit exclusions):
```markdown
NON-GOALS:
  - No compression (plain copies)
  - No cloud/remote backup
  - No email notifications
  - No scheduling/cron setup
```

*AI stays focused on requirements only.*

---

### Mistake 4: Validation Tests Not Actionable

❌ **Wrong** (vague test):
```markdown
VALIDATION:
  - Verify backup works correctly
```

*"Works correctly" is not testable.*

✅ **Right** (specific test steps):
```markdown
VALIDATION:
  - Test: Create 3 .md files → Run script → Verify all 3 in backup folder
  - Test: Check log file → Verify "3 files copied" message
  - Test: Verify backup folder name matches timestamp format
```

*Each test has clear setup, action, expected result.*

---

## Specification Template (Copy This)

Use this template for your next AI implementation request:

```markdown
WHAT: [One-sentence description of what you're building]

SUCCESS CRITERIA:
  - [Measurable outcome 1]
  - [Measurable outcome 2]
  - [Measurable outcome 3]

CONSTRAINTS:
  Technical:
    - [Technical requirement or limitation]
  Scope:
    - [What's included/excluded]
  Quality:
    - [Quality standard to meet]

NON-GOALS:
  - [Feature you explicitly don't need]
  - [Common feature to avoid]

OUTPUT FORMAT:
  - [File structure / data structure / return format]

VALIDATION:
  - Test: [Setup] → [Action] → [Expected result]
  - Test: [Setup] → [Action] → [Expected result]
  - Test: [Setup] → [Action] → [Expected result]
```

**Usage**: Fill this out BEFORE prompting AI. Validate completeness. THEN prompt with specification.

---

## Practice Exercise: Git Commit Message Specification

Before looking at the solution, write a specification for this task:

**Task**: "I need AI to generate conventional commit messages for my Git changes."

Use the template above. Define:
1. Success criteria (what makes a good commit message?)
2. Constraints (format rules? length limits?)
3. Non-goals (what NOT to include?)
4. Output format (exact structure?)
5. Validation tests (how to verify quality?)

**Spend 5 minutes** writing this specification.

---

**Solution** (Compare yours):

```markdown
WHAT: Generate conventional commit message from Git staged changes

SUCCESS CRITERIA:
  - Message follows conventional commits format: <type>(<scope>): <description>
  - Type matches change category (feat/fix/docs/refactor/test/chore)
  - Description under 50 characters, imperative mood
  - Body lists each change with bullet points
  - Breaking changes marked with "BREAKING CHANGE:" footer

CONSTRAINTS:
  Technical:
    - Parse `git diff --staged` output
    - Identify file types changed (code, tests, docs)
  Scope:
    - Scope extracted from changed file paths (e.g., "auth" from src/auth/)
  Quality:
    - Description explains WHY, not just WHAT
    - Body provides context for reviewers

NON-GOALS:
  - No emoji in commit messages (keep professional)
  - No auto-commit (generate message only)
  - No commit history rewriting

OUTPUT FORMAT:
  ```
  feat(auth): add JWT token refresh endpoint

  - Implemented /api/auth/refresh endpoint
  - Added token expiration validation
  - Updated authentication middleware

  BREAKING CHANGE: Auth tokens now expire after 24 hours
  ```

VALIDATION:
  - Test: Stage auth file changes → Verify scope="auth"
  - Test: Add new feature → Verify type="feat"
  - Test: Fix bug → Verify type="fix"
  - Test: Count description characters → Verify ≤50
  - Test: Check imperative mood → Verify not past tense ("add" not "added")
```

**How does yours compare?**
- Are your success criteria measurable?
- Did you specify output format precisely?
- Did you include validation tests?

---

## What You've Learned

You've learned the specification-first approach that Jake Heller used to build a $650M product:

1. **Write specification BEFORE prompting** — Define success criteria, constraints, non-goals
2. **Validate specification completeness** — Use the 5-question checklist
3. **Prompt AI with specification** — AI implements against clear requirements
4. **Verify with tests** — Validation tests defined upfront

**Core principle**: "Define 'what good looks like' before prompting AI."

In the next lesson, you'll learn **Question-Driven Development (QDD)**: how to prompt AI to ask YOU clarifying questions, ensuring nothing critical gets missed in your specification.

---

## Try With AI

**Your Task**: Write a specification for a Git status summary script, then prompt AI to implement it.

**Requirements**:
- Script shows: modified files, staged files, untracked files
- Outputs color-coded summary (green=staged, red=modified, yellow=untracked)
- Returns count of each category

**Part 1: Write the Specification** (5 minutes)

Use the specification template. Define all five components:
1. SUCCESS CRITERIA
2. CONSTRAINTS
3. NON-GOALS
4. OUTPUT FORMAT
5. VALIDATION

**Part 2: Validate Completeness** (2 minutes)

Check your specification:
- Are success criteria measurable?
- Are constraints specific (not vague)?
- Are non-goals explicit?
- Is output format precisely defined?
- Can you perform each validation test?

**Part 3: Prompt AI** (3 minutes)

Open your AI coding assistant (Claude Code, GitHub Copilot Chat, Cursor, etc.) and prompt:

```markdown
Create a Bash script meeting this specification:

[Paste your specification]

Implementation notes:
- Use `git status --porcelain` for machine-readable output
- Use ANSI color codes for terminal colors
```

**Part 4: Test Against Validation Criteria** (5 minutes)

Run the generated script. Verify each validation test:
- Test: Stage a file → Verify shown in green
- Test: Modify unstaged file → Verify shown in red
- Test: Create untracked file → Verify shown in yellow
- Test: Check counts → Verify accurate

**Part 5: Reflection** (2 minutes)

Ask yourself:
1. Did AI's first implementation meet all success criteria? (If yes, specification worked)
2. What did you forget to specify? (If AI guessed wrong, specification was incomplete)
3. How many iterations to working solution? (Specification-first should reduce this to 1-2)

**Key Learning**: The quality of your specification determines the quality of AI's first implementation. Garbage specification → garbage output. Excellent specification → excellent output (first try).
