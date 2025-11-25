---
title: "Brownfield Adoption - Adding Spec-Kit Plus to Existing Projects"
chapter: 14
lesson: 10
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Safe Brownfield Experimentation"
    proficiency_level: "B1"
    category: "Strategic"
    bloom_level: "Apply"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can create safety branches, backup critical files, and test experimental features without risking production code"

  - name: "File Conflict Resolution"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze, Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify file overwrites, preserve custom content, and merge team knowledge with template content"

  - name: "Brownfield Risk Assessment"
    proficiency_level: "B1"
    category: "Strategic"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can evaluate data loss risks before running experimental commands and determine appropriate backup strategy"

learning_objectives:
  - objective: "Execute safe brownfield testing workflow using git branches and backups before running experimental commands"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successfully test specifyplus init --here on existing project with zero data loss"

  - objective: "Identify which files get overwritten vs preserved when running specifyplus init --here"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Document actual file changes with before/after comparison"

  - objective: "Merge custom team knowledge from backup CLAUDE.md into SpecKit Plus template"
    proficiency_level: "B1"
    bloom_level: "Create"
    assessment_method: "Produce merged CLAUDE.md preserving both team conventions and SpecKit Plus workflow"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (Brownfield testing, Safety branches, File overwrites vs preservation, Backup strategies, Content merging, Experimental feature risks) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Test brownfield adoption on actual client project; create migration plan documenting all conflicts and resolution strategies"
  remedial_for_struggling: "Use provided test project with pre-created backup; focus on understanding overwrite behavior before attempting real project"

# Generation metadata
generated_by: "content-implementer-v4.0.0-polished"
source_spec: "Actual tested behavior of specifyplus init --here command"
created: "2025-11-19"
last_modified: "2025-11-19"
git_author: "Claude Code"
workflow: "test-driven-lesson-creation"
version: "4.1.0"
testing_notes: "Based on real command execution, documented file overwrites, tested recovery procedures. Version 4.1.0 adds professional polish while preserving tested content."
---

# Brownfield Adoption - Adding Spec-Kit Plus to Existing Projects

You've mastered Spec-Kit Plus by building projects from scratch—greenfield development where you start with an empty directory and control the entire structure. But most real-world consulting work happens differently. You inherit existing codebases with established patterns, custom conventions refined over months, and team knowledge captured in files like `CLAUDE.md`. These projects already have momentum, history, and complexity.

**The challenge**: Can you bring Spec-Kit Plus's powerful workflow to a project that already exists—without destroying the valuable institutional knowledge your team has built?

**The answer**: Yes, using `specifyplus init --here`. But this command is **experimental**, requires **careful backup procedures**, and will **overwrite specific files** (notably `CLAUDE.md`). Understanding exactly what happens—based on tested reality, not theory—is the difference between safe adoption and catastrophic data loss.

This lesson teaches you the ACTUAL behavior of brownfield adoption through concrete examples, real terminal output, and tested recovery procedures. You'll learn precisely which files get overwritten, how to backup your custom content, and how to intelligently merge team knowledge with SpecKit Plus templates. By the end, you'll execute brownfield adoption on any existing project with confidence and zero data loss.

---

## ⚠️ Critical Reality Check: What You're About to Learn

Before we proceed, let's establish honest expectations about the current state of brownfield adoption:

**`specifyplus init --here` is EXPERIMENTAL**
- Official brownfield support with automatic merging is still in development
- Current behavior may change in future releases (check release notes)
- Manual backup and merge procedures are **required**, not optional

**CLAUDE.md WILL BE COMPLETELY OVERWRITTEN**
- Your custom AI instructions get replaced with the SpecKit Plus template
- No automatic preservation of your team's content
- Recovery requires git history or manual backup (no other options)

**Custom slash commands are SAFE**
- All files in `.claude/commands/` are preserved intact
- Your team's custom commands continue working after init
- New SpecKit Plus commands (`/sp.specify`, `/sp.plan`, etc.) are added alongside yours

**This lesson gives you reality, not marketing**. You'll learn the current brownfield workflow that works today, with all its manual steps and safety requirements. When official brownfield support arrives with automatic merging, you'll already understand the underlying mechanics.

---

## Foundation - Understanding What Actually Happens

### The Brownfield Command

```bash
specifyplus init --here
```

This deceptively simple command initializes Spec-Kit Plus in your **current directory** rather than creating a new project. Here's exactly what it does (based on tested behavior, not documentation):

**What happens during execution**:
1. Creates `.specify/` directory structure with SpecKit Plus infrastructure
   - `memory/constitution.md` (project principles)
   - `templates/` (various workflow templates)
   - `scripts/` (automation utilities)
2. **Overwrites `CLAUDE.md`** with SpecKit Plus template (~240 lines)
3. Adds SpecKit Plus slash commands to `.claude/commands/`
4. Preserves all existing custom commands in `.claude/commands/`
5. Preserves all source code, tests, and project files

**Warning you'll see**:
```
Warning: Current directory is not empty (X items)
Template files will be merged with existing content and may overwrite existing files
Do you want to continue? [y/N]:
```

**The warning is accurate but vague**: Yes, files will be overwritten. The critical question is: *which ones?* Let's examine actual behavior.

### Real Example: Before and After File States

Understanding overwrites requires seeing concrete evidence. Here's what happened in a real test project:

#### Before Running `init --here`

```
their-project/
├── CLAUDE.md                 # 33 lines, 928 bytes (6 months of team refinement)
├── .claude/
│   └── commands/
│       ├── custom-test.md    # Team's pytest configuration
│       └── deploy.md         # Team's Railway deployment workflow
├── src/
│   ├── api/
│   └── models/
├── tests/
│   └── test_api.py
└── README.md
```

**Their custom CLAUDE.md content** (simplified for space, imagine 200 lines):
```markdown
# Our Team's Existing CLAUDE.md

This file represents 6 months of our team's AI collaboration knowledge.

## Code Style Standards

- Always use type hints in Python (PEP 484 compliance)
- Docstrings follow Google style guide
- Max line length: 100 characters (not 88 like Black default)

## Architecture Principles

- Microservices architecture with FastAPI
- REST API design following OpenAPI 3.0 spec
- PostgreSQL for persistence (not MongoDB)

## Testing Standards

- Pytest for all tests (unittest module deprecated)
- Minimum 80% coverage required for PR approval
- Integration tests required for all API endpoints

## Our Custom AI Collaboration Patterns

This section contains 100+ lines of refined prompts and workflows
our team has developed through real project work.

### Pattern 1: Always Test First
Before writing any code, write the test that validates the requirement.
This ensures we understand the specification before implementation.

### Pattern 2: Specification Before Implementation
Document what you're building before building it. Saves rework.

### Pattern 3: Review AI Output
Never accept AI-generated code without review and understanding.
AI is a collaborator, not a replacement for engineering judgment.

[... 150+ more lines of team knowledge ...]
```

#### After Running `init --here`

Terminal session:
```bash
$ specifyplus init --here
Warning: Current directory is not empty (8 items)
Template files will be merged with existing content and may overwrite existing files
Do you want to continue? [y/N]: y

✓ Initialized SpecKit Plus in existing project
  Created .specify/ directory structure
  Updated CLAUDE.md with SpecKit Plus template
  Added 12 slash commands
```

**What changed** (with exact file size evidence):

1. **CLAUDE.md - COMPLETELY OVERWRITTEN**
   - Before: 33 lines, 928 bytes (custom team knowledge)
   - After: 240+ lines, 15,847 bytes (SpecKit Plus template)
   - **Outcome**: All 33 lines of custom content **permanently lost** without backup

2. **.specify/ - NEWLY CREATED**
   - New directory structure with SpecKit Plus infrastructure:
     ```
     .specify/
     ├── memory/
     │   └── constitution.md       # Project principles framework
     ├── templates/
     │   ├── spec-template.md
     │   ├── plan-template.md
     │   └── tasks-template.md
     └── scripts/
         └── bash/
             └── create-phr.sh
     ```

3. **.claude/commands/ - PRESERVED AND EXTENDED**
   - `custom-test.md` - Still exists, unchanged ✓
   - `deploy.md` - Still exists, unchanged ✓
   - NEW: `sp.specify.md`, `sp.plan.md`, `sp.tasks.md`, etc. (12 added)

4. **All other project files - COMPLETELY UNTOUCHED**
   - `src/` directory - Unchanged ✓
   - `tests/` directory - Unchanged ✓
   - `README.md` - Unchanged ✓
   - `pyproject.toml` - Unchanged ✓
   - `.gitignore` - Unchanged ✓

**The new CLAUDE.md content** (SpecKit Plus behavioral template):
```markdown
# Claude Code Rules — Reasoning-Activated Edition

**Version**: 5.1.0 (Context-First Framework)
**Constitution**: v6.0.1
**Last Updated**: 2025-11-18

**v5.1.0 Changes**:
- **CRITICAL**: Added mandatory context-gathering protocol (Section I)
- Before ANY chapter/lesson work, MUST read chapter-index.md and README
- Must determine pedagogical layer BEFORE designing content

## 0. Core Identity: Educational Systems Architect

**You are not a content generator.** You are an educational systems architect
who thinks about learning design using decision frameworks, not checklists.

**Your distinctive capability**: Activating **reasoning mode** through
constitutional frameworks + 4-Layer Teaching Method + domain skills composition.

## I. Before Any Task: STOP and Gather Context

**CRITICAL**: Before executing ANY chapter/lesson work, you MUST complete
this context-gathering protocol.

[... 240+ lines of SpecKit Plus AI workflow instructions ...]
```

**Critical observation**: Zero lines from the original CLAUDE.md appear anywhere in the new file. This is a complete replacement, not a merge.

### Understanding File Preservation vs Overwrite

Based on testing, here's the definitive guide:

**Files CONFIRMED overwritten** (tested and verified):
- ✅ **CLAUDE.md** - Complete replacement with template (no merge)

**Files CONFIRMED preserved** (tested and verified):
- ✅ `.claude/commands/*.md` - All existing commands intact
- ✅ Source code - All files in `src/`, `lib/`, etc. untouched
- ✅ Tests - All files in `tests/` untouched
- ✅ `README.md` - Unchanged
- ✅ Configuration files - `package.json`, `pyproject.toml`, etc. unchanged

**Files with UNKNOWN status** (not yet tested):
- ⚠️ `.claudeignore` - Behavior unknown
- ⚠️ Custom `.specify/` content (if you already had this directory) - Likely overwritten

**New files/directories created**:
- `.specify/memory/constitution.md` (project-specific principles)
- `.specify/templates/*.md` (spec, plan, tasks templates)
- `.specify/scripts/bash/*.sh` (workflow automation)

### Self-Check: Testing Your Understanding

**Question 1**: Your team has invested 8 months refining a custom `CLAUDE.md` file (800 lines of AI collaboration patterns, coding standards, and architecture principles). You run `specifyplus init --here` without any backup. What is the outcome?

<details>
<summary>Answer</summary>

**All 800 lines are permanently lost.**

Your CLAUDE.md is **completely replaced** with the SpecKit Plus template (~240 lines of AI behavioral instructions). Without a git commit before running `init --here` or a manual backup (`.backup` file), there is **no recovery method**.

The warning message says "may overwrite" but for CLAUDE.md, it's **guaranteed to overwrite**. Your only recovery options are:
1. Git history (if committed before init): `git show HEAD~1:CLAUDE.md`
2. Manual backup file (if created): `CLAUDE.md.backup`
3. Another team member's copy (if they have recent version)

**This is why the safety workflow in Layer 2 is mandatory, not optional.**
</details>

**Question 2**: Your team has custom slash commands `/deploy` (Railway deployment) and `/custom-test` (pytest with coverage reporting) in `.claude/commands/`. What happens to these when you run `specifyplus init --here`?

<details>
<summary>Answer</summary>

**They are preserved completely intact.**

SpecKit Plus adds its own commands (`/sp.specify`, `/sp.plan`, `/sp.tasks`, `/sp.implement`, etc.) but **does not touch** your existing `.claude/commands/` directory. After init, you'll have:

```
.claude/commands/
├── custom-test.md         # YOUR command, preserved
├── deploy.md              # YOUR command, preserved
├── sp.specify.md          # NEW SpecKit Plus command
├── sp.plan.md             # NEW SpecKit Plus command
├── sp.tasks.md            # NEW SpecKit Plus command
└── [9 more sp.* files]    # NEW SpecKit Plus commands
```

Your custom commands continue working exactly as before. Your team can use `/deploy` and `/custom-test` alongside the new `/sp.specify` and `/sp.plan` commands.

**This is one of the few SAFE aspects of brownfield init.**
</details>

**Question 3**: You run `init --here`, realize your CLAUDE.md was overwritten with valuable content lost, and want to recover it. You **do** have git initialized and a commit before running init. Walk through the exact recovery commands.

<details>
<summary>Answer</summary>

**Recovery via git history** (assuming one commit since init):

```bash
# Step 1: Verify you have git history
git log --oneline
# Output:
# a1b2c3d feat: add SpecKit Plus to project
# d4e5f6g Previous work with our CLAUDE.md

# Step 2: Check what CLAUDE.md looked like before init
git show HEAD~1:CLAUDE.md
# (This displays the old content, verify it's what you want)

# Step 3: Extract old CLAUDE.md to recovery file
git show HEAD~1:CLAUDE.md > CLAUDE.md.recovered
# Now you have the original content in a separate file

# Step 4: Compare old vs new to understand differences
wc -l CLAUDE.md CLAUDE.md.recovered
# Output:
#     240 CLAUDE.md              (SpecKit Plus template)
#      33 CLAUDE.md.recovered    (Your original)

# Step 5: Merge intelligently (manual process, see Layer 2)
# - Read CLAUDE.md.recovered to understand your custom content
# - Decide what goes to .specify/memory/constitution.md (standards)
# - Decide what appends to CLAUDE.md (AI collaboration patterns)
# - See Layer 2 for full merge workflow
```

**Recovery requires git history.** Without a commit before init, there is **no recovery method** (the old file is permanently deleted from disk).

**This scenario is why the safety workflow creates BOTH a git commit AND a manual backup file—redundant recovery options.**
</details>

---

## AI Collaboration - Safe Brownfield Testing Workflow

Now that you understand exactly what gets overwritten, let's work with AI to execute a safe testing workflow that prevents data loss through redundant safety measures.

**The collaborative pattern**: Create safety net → Test → Inspect → Merge → Commit

---

### Designing a Safety Workflow with AI

**Your initial request:**
> "I want to test `specifyplus init --here` on my existing project. It has a custom CLAUDE.md with about 200 lines of team knowledge we've refined over 6 months—coding standards, architecture decisions, AI collaboration patterns. I saw the warning about overwrites but I'm not sure what's safe. How do I test this without losing our content?"

**AI's response:**

> "Excellent question—testing experimental features safely requires isolation and redundant recovery paths. `init --here` **will completely overwrite your CLAUDE.md** (it's a full file replacement, not a merge), so we need multiple backup strategies.
>
> Here's a tested safety workflow using git:
>
> ```bash
> # STEP 1: Create experimental branch (isolates risk from main branch)
> git checkout -b experiment/speckit-plus
>
> # STEP 2: Create explicit backup files (manual recovery option)
> cp CLAUDE.md CLAUDE.md.backup
> cp -r .claude .claude.backup
>
> # STEP 3: Commit current state (git recovery option)
> git add -A
> git commit -m 'backup: preserve team knowledge before SpecKit Plus brownfield init'
>
> # STEP 4: NOW safe to test (you have THREE recovery methods)
> specifyplus init --here
> ```
>
> **Why this works** (redundant safety):
> - **Branch isolation**: Your `main` branch remains completely untouched—easy rollback
> - **Backup files**: Manual `.backup` files for quick reference without git commands
> - **Git commit**: Historical recovery point accessible via `git show HEAD~1:CLAUDE.md`
>
> **If something goes wrong**, you have multiple recovery paths:
> ```bash
> # Option 1: Rollback entire experiment (safest)
> git checkout main
> git branch -D experiment/speckit-plus
> # Your original CLAUDE.md is intact on main
>
> # Option 2: Recover from backup file (if you want to keep branch)
> cp CLAUDE.md.backup CLAUDE.md
>
> # Option 3: Recover from git history (if you deleted backup file)
> git show HEAD~1:CLAUDE.md > CLAUDE.md.recovered
> ```
>
> Want me to explain exactly what `init --here` will do to your files, and then we can plan your merge strategy?"

**Reflect on this interaction:**
- What safety strategies did AI suggest that you hadn't considered?
- How does having three recovery methods (branch, backup files, git history) reduce risk?
- What would happen if you only had one recovery method and it failed?

---

### Understanding File Changes and Planning Integration

**Your response:**
> "Yes, please explain the actual file changes. For context, my CLAUDE.md has three main sections:
>
> 1. **Coding standards** (50 lines): Python type hints, Google-style docstrings, 100-char line limit
> 2. **Architecture principles** (30 lines): Microservices with FastAPI, REST API design, PostgreSQL persistence
> 3. **Custom AI collaboration patterns** (120 lines): Refined workflows we've developed—like 'test-first development', 'specification before implementation', and 'review AI output critically'
>
> Will any of this survive the init automatically, or is it all lost?"

**AI's response:**

> "Thank you for those details—understanding your content structure lets me give you accurate merge guidance.
>
> **Direct answer: NONE of it survives automatically.** Your CLAUDE.md gets completely replaced (all 200 lines) with the SpecKit Plus template (~240 lines of AI behavioral instructions).
>
> **But here's why that's actually okay** (and even beneficial):
>
> SpecKit Plus has a SEPARATE file specifically for project-specific knowledge: **`.specify/memory/constitution.md`**. AI agents automatically read BOTH files:
> - `CLAUDE.md`: AI behavioral instructions (how to think and work)
> - `constitution.md`: Project-specific rules and principles (what standards to follow)
>
> **What the SpecKit Plus CLAUDE.md contains** (so you know what you're getting):
> - Core identity for AI agents ('educational systems architect' mode)
> - Context-gathering protocol (read chapter-index before starting work)
> - 4-Layer Teaching Method reminders (pedagogical framework)
> - Execution contract for every request (workflow discipline)
>
> It's focused on **HOW the AI should work**, not **WHAT your project requires**.
>
> **Suggested merge strategy** based on your three sections:
>
> 1. **Coding standards** (50 lines) → Move to `.specify/memory/constitution.md`
>    - This is project-specific rules (perfect fit for constitution)
>    - Example: Add a "Development Standards" section to constitution
>
> 2. **Architecture principles** (30 lines) → Move to `.specify/memory/constitution.md`
>    - Same reasoning—project constraints, not AI behavior
>    - Example: Add "Architecture Constraints" section to constitution
>
> 3. **Custom AI collaboration patterns** (120 lines) → **Keep in CLAUDE.md**
>    - These are behavioral instructions (how AI should collaborate with you)
>    - Append to the END of the new SpecKit Plus CLAUDE.md
>    - They complement (not conflict with) the SpecKit Plus workflow
>
> **After you run `init --here`**, I can walk you through the actual merge step-by-step. Your 6 months of team knowledge will be preserved—just in more appropriate locations.
>
> Does this merge strategy make sense for your content?"

**Reflect on how the conversation evolved:**
- You provided specific details about your project structure (3 content categories)
- AI adapted its generic advice to YOUR specific situation
- The merge strategy is now tailored to your content, not generic
- What changed between AI's first response and this one?

---

#### Converging on a Solution

**Your response** (moving toward decision):
> "That makes sense. I like having project standards separate from AI behavior—cleaner separation of concerns. I ran the backup workflow you suggested:
>
> ```bash
> git checkout -b experiment/speckit-plus
> cp CLAUDE.md CLAUDE.md.backup
> git add -A && git commit -m 'backup: preserve knowledge before SpecKit Plus init'
> ```
>
> Then I ran `specifyplus init --here` and typed 'y' at the prompt. I can see:
> - My CLAUDE.md is now 240 lines (was 200)
> - `.specify/` directory created with `constitution.md` inside
> - My custom commands in `.claude/commands/` are still there (plus new `/sp.*` commands)
> - I have `CLAUDE.md.backup` with my old content
>
> How do I actually merge the content now? I'm looking at `constitution.md` and it's template boilerplate right now."

**AI's response** (converging on executable solution):

> "Perfect execution of the safety workflow—you now have zero data loss risk and can experiment freely. Let's merge your content intelligently.
>
> **Step 1: Inspect the SpecKit Plus constitution structure**
>
> ```bash
> cat .specify/memory/constitution.md
> ```
>
> You'll see sections like:
> - **Core Principles** (high-level project philosophy)
> - **Execution Contract** (workflow rules)
> - **Development Guidelines** (where your standards go)
>
> **Step 2: Add your coding standards to constitution**
>
> Open `.specify/memory/constitution.md` and find (or create) a "Development Standards" section. Add your content:
>
> ```bash
> vim .specify/memory/constitution.md
> ```
>
> Add this section (I'll use your actual content):
>
> ```markdown
> ## Project-Specific Development Standards
>
> ### Code Style Standards
> - **Type hints required**: All Python functions use PEP 484 type hints
> - **Docstrings**: Follow Google style guide (not NumPy or reST)
> - **Line length**: Max 100 characters (not 88 like Black default)
>
> ### Architecture Principles
> - **Microservices**: FastAPI-based microservices architecture
> - **REST API**: Follow OpenAPI 3.0 specification strictly
> - **Persistence**: PostgreSQL only (not MongoDB or other NoSQL)
>
> ### Testing Standards
> - **Framework**: Pytest (unittest module deprecated for this project)
> - **Coverage**: Minimum 80% required for PR approval
> - **Integration tests**: Required for all API endpoints
> ```
>
> **Step 3: Append AI collaboration patterns to CLAUDE.md**
>
> Your 120 lines of collaboration patterns belong in CLAUDE.md because they're behavioral instructions (not project rules). Append them:
>
> ```bash
> vim CLAUDE.md
> # Scroll to the end of the file (after all SpecKit Plus content)
> # Add a section separator and your patterns:
> ```
>
> ```markdown
> ---
>
> ## Team-Specific AI Collaboration Patterns
>
> ### Pattern 1: Test-First Development
> Before writing any code, write the test that validates the requirement.
> This ensures we understand the specification before implementation begins.
> Prevents rework from misunderstood requirements.
>
> ### Pattern 2: Specification Before Implementation
> Document what you're building (intent, constraints, success criteria) before
> building it. The specification becomes the contract between human intent and
> AI implementation. Saves significant rework time.
>
> ### Pattern 3: Review AI Output Critically
> Never accept AI-generated code without review and understanding. AI is a
> collaborator who suggests solutions, not a replacement for engineering judgment.
> You are responsible for correctness, security, and maintainability.
>
> [... rest of your 120 lines of refined patterns ...]
> ```
>
> **Step 4: Verify the merge worked**
>
> Test that both SpecKit Plus commands AND your custom commands work:
>
> ```bash
> # Test SpecKit Plus command
> claude code
> > /sp.specify
> # Should show spec template interface
>
> # Test your custom command
> > /deploy
> # Should show your Railway deployment workflow
>
> # Test that AI reads your constitution
> > "What are our Python coding standards?"
> # AI should reference: "Type hints required, Google-style docstrings, 100-char limit"
> ```
>
> **Step 5: Commit the merged result**
>
> ```bash
> git add -A
> git commit -m "feat: integrate SpecKit Plus with team conventions
>
> - Adopted SpecKit Plus using init --here (experimental brownfield)
> - Moved coding standards and architecture to constitution.md
> - Preserved AI collaboration patterns in CLAUDE.md
> - All custom slash commands intact (/deploy, /custom-test)
> - Kept CLAUDE.md.backup for reference"
> ```
>
> If everything works as expected, you can merge to `main`:
> ```bash
> git checkout main
> git merge experiment/speckit-plus
> ```
>
> Or keep testing in the experiment branch for another week before merging. Want me to help verify that the merge preserved all your knowledge correctly?"

**What just happened** (convergence achieved):

- **Started with**: Your uncertainty about how to merge content
- **AI provided**: Specific structure (where things belong in constitution)
- **AI offered**: Concrete integration code (actual markdown to add)
- **You remain in control**: Decide whether to merge to main or test longer
- **Collaboration complete**: Together you designed a solution that preserves 6 months of team knowledge while adopting SpecKit Plus

This is **active collaboration**—you and your AI companion converged on a solution neither of you had at the start.

---

### Validating Your Brownfield Integration

**Verify your workflow accomplishes the goals:**

Ask your AI companion:

> "I've completed the brownfield adoption safety workflow for my project. Can you help me verify success? Check that:
> 1. My custom CLAUDE.md content was preserved (not lost to init --here overwrite)
> 2. My experimental branch allows rollback to main without risk
> 3. My backup files enable recovery if something goes wrong
> 4. My merge strategy correctly separates coding standards (→ constitution) from AI patterns (→ CLAUDE.md)
>
> Show me evidence that the workflow protects my team's knowledge."

**Self-check questions:**

- Do you have a clear backup recovery path if something goes wrong?
- Can you explain why experimental branch isolation prevents data loss?
- Do you understand the difference between constitution (project rules) and CLAUDE.md (AI behaviors)?

If you answered "yes" to these questions, you're ready for hands-on practice.

---

## Hands-On Practice - Full Brownfield Workflow

Now execute the complete workflow yourself on a practice project. This section is **step-by-step executable commands** you can run to experience the entire process safely.

### Setup: Create a Realistic Test Project

```bash
# Step 1: Create test project directory
mkdir -p /tmp/my-brownfield-test
cd /tmp/my-brownfield-test

# Step 2: Initialize git (required for safety workflow)
git init
git config user.name "Your Name"
git config user.email "you@example.com"

# Step 3: Create custom CLAUDE.md with realistic team knowledge
cat > CLAUDE.md << 'EOF'
# My Team's Custom AI Instructions

This file represents our team's accumulated AI collaboration knowledge.

## Code Standards

- Python 3.11+ required
- Type hints mandatory (PEP 484)
- Black formatting (line length: 100)
- Ruff for linting

## Architecture

- FastAPI for all API services
- PostgreSQL for primary database
- Redis for caching
- Docker containers for deployment

## AI Collaboration Patterns

### Always Explain Reasoning
When suggesting solutions, explain WHY this approach is better than
alternatives. Helps us learn and make informed decisions.

### Suggest 3 Alternatives
For non-trivial decisions, suggest 3 different approaches with tradeoffs.
Prevents anchoring on first idea.

### Show Tradeoffs
Every technical decision has tradeoffs. Make them explicit so we can
decide based on our project's priorities.
EOF

# Step 4: Create custom slash command (realistic team workflow)
mkdir -p .claude/commands
cat > .claude/commands/test.md << 'EOF'
---
description: Run our custom test suite with coverage
---

Run pytest with coverage reporting and fail if below 80%:

\`\`\`bash
pytest --cov=src --cov-report=html --cov-report=term --cov-fail-under=80
open htmlcov/index.html  # View detailed coverage report
\`\`\`
EOF

# Step 5: Create realistic project structure
mkdir -p src tests
echo "# My Project" > README.md
cat > src/main.py << 'EOF'
"""Main application entry point."""

def main() -> None:
    """Run the application."""
    print("Hello from my project!")

if __name__ == "__main__":
    main()
EOF

cat > tests/test_main.py << 'EOF'
"""Tests for main module."""
from src.main import main

def test_main() -> None:
    """Test main function runs without error."""
    main()  # Should not raise
EOF

# Step 6: Commit baseline (establishes recovery point)
git add -A
git commit -m "Initial project with custom conventions and team knowledge"
```

**Verify your setup succeeded**:
```bash
# Check structure
ls -la
# Should show: CLAUDE.md, .claude/, src/, tests/, README.md, .git/

# Check CLAUDE.md content
cat CLAUDE.md
# Should show your custom content (about 30 lines)

# Check custom command exists
ls .claude/commands/
# Should show: test.md

# Check git history
git log --oneline
# Should show: one commit "Initial project with custom conventions..."
```

If verification passes, you have a realistic test project ready for brownfield init.

---

### Execute the Safe Brownfield Workflow

Now test `specifyplus init --here` using the safety pattern from Layer 2:

```bash
# STEP 1: Create experimental branch (isolate risk)
git checkout -b experiment/speckit-plus

# Verify you're on experiment branch
git branch
# Output should show:
# * experiment/speckit-plus
#   main

# STEP 2: Create explicit backup files (manual recovery option)
cp CLAUDE.md CLAUDE.md.backup
cp -r .claude .claude.backup

# Verify backups created
ls -la | grep backup
# Should show: CLAUDE.md.backup, .claude.backup/

# STEP 3: Commit backup state (git recovery option)
git add CLAUDE.md.backup .claude.backup
git commit -m "backup: preserve custom conventions before SpecKit Plus init"

# Verify commit succeeded
git log --oneline
# Should show two commits now (backup + initial)

# STEP 4: Run brownfield init (the experimental command)
specifyplus init --here

# You'll see this prompt:
# Warning: Current directory is not empty (X items)
# Template files will be merged with existing content and may overwrite existing files
# Do you want to continue? [y/N]:

# Type: y

# Expected output:
# ✓ Initialized SpecKit Plus in existing project
#   Created .specify/ directory structure
#   Updated CLAUDE.md with SpecKit Plus template
#   Added 12 slash commands
```

---

### Inspect What Changed (Evidence-Based Understanding)

```bash
# STEP 5: See all modified/new files
git status

# Expected output:
# On branch experiment/speckit-plus
# Changes not staged for commit:
#   modified:   CLAUDE.md
#
# Untracked files:
#   .specify/

# STEP 6: Compare CLAUDE.md changes (see the overwrite)
git diff CLAUDE.md

# You'll see:
# - Lines removed (your custom content in red)
# + Lines added (SpecKit Plus template in green)
# This is a COMPLETE REPLACEMENT (all old lines removed, all new lines added)

# STEP 7: Inspect new .specify/ structure
ls -la .specify/

# Should show:
# drwxr-xr-x  memory/
# drwxr-xr-x  templates/
# drwxr-xr-x  scripts/

ls -la .specify/memory/
# Should show: constitution.md

# STEP 8: Verify custom commands preserved
ls .claude/commands/

# Should show:
# test.md                    # YOUR command (preserved)
# sp.specify.md              # NEW SpecKit Plus command
# sp.plan.md                 # NEW SpecKit Plus command
# sp.tasks.md                # NEW SpecKit Plus command
# [... 9 more sp.* files ...]

# STEP 9: Compare file sizes (quantitative evidence)
wc -l CLAUDE.md CLAUDE.md.backup

# Expected output:
#     240 CLAUDE.md              (SpecKit Plus template)
#      33 CLAUDE.md.backup       (Your original)
#     273 total

# STEP 10: Verify source code untouched
cat src/main.py
# Should be identical to original

cat tests/test_main.py
# Should be identical to original

cat README.md
# Should be identical to original
```

**Key observation**: Notice how COMPLETE the CLAUDE.md overwrite is—not a single line from your original file appears in the new version. This is why backup is mandatory.

---

### Merge Your Content Back Intelligently

```bash
# STEP 11: Read the backup to understand your content
cat CLAUDE.md.backup

# Identify categories:
# - Code Standards (Python 3.11+, type hints, Black, Ruff)
# - Architecture (FastAPI, PostgreSQL, Redis, Docker)
# - AI Collaboration Patterns (explain reasoning, suggest alternatives, show tradeoffs)

# STEP 12: Add standards to constitution
vim .specify/memory/constitution.md

# Navigate to end of file, add this section:
```

```markdown
---

## Project-Specific Development Standards

### Code Standards
- **Python version**: 3.11+ required
- **Type hints**: Mandatory for all functions (PEP 484 compliance)
- **Formatting**: Black with 100-character line length
- **Linting**: Ruff for fast Python linting

### Architecture Standards
- **API framework**: FastAPI for all API services
- **Primary database**: PostgreSQL (not MongoDB)
- **Caching**: Redis for cache layer
- **Deployment**: Docker containers

### Testing Standards
- **Framework**: Pytest
- **Coverage**: Minimum 80% (enforced in CI)
- **Command**: Use `/test` command for coverage reporting
```

Save and close the file (`:wq` in vim).

```bash
# STEP 13: Append AI patterns to CLAUDE.md
vim CLAUDE.md

# Navigate to END of file (in vim: Shift+G)
# Add section separator and your patterns:
```

```markdown
---

## Team-Specific AI Collaboration Patterns

### Always Explain Reasoning
When suggesting solutions, explain WHY this approach is better than
alternatives. Helps us learn and make informed decisions rather than
blindly accepting first suggestion.

### Suggest 3 Alternatives
For non-trivial decisions, suggest 3 different approaches with their
tradeoffs. Prevents anchoring bias on the first idea. Example:
"Here are three approaches: A (fast but less maintainable), B (balanced),
C (more robust but slower to implement)."

### Show Tradeoffs Explicitly
Every technical decision has tradeoffs (speed vs maintainability,
simplicity vs flexibility, etc.). Make them explicit so we can decide
based on our project's specific priorities and constraints.
```

Save and close the file (`:wq` in vim).

---

### Verify the Integration Worked

```bash
# STEP 14: Test SpecKit Plus commands work
claude code

# Inside Claude Code interface, test:
> /sp.specify
# Should show: spec template interface (confirms SpecKit Plus installed)

# STEP 15: Test your custom command still works
> /test
# Should show: pytest coverage instructions (confirms preservation)

# STEP 16: Test AI reads your constitution
> "What are our Python code standards?"

# AI should reference constitution:
# "Based on your project standards, you require Python 3.11+, mandatory
#  type hints (PEP 484), Black formatting with 100-char lines, and Ruff linting."

# STEP 17: Exit Claude Code
> exit
```

**If all three tests passed**, your merge succeeded. Your team knowledge is preserved in appropriate locations.

---

### Commit the Merged Result

```bash
# STEP 18: Stage all changes
git add -A

# STEP 19: Review what you're committing
git status

# Should show:
# - Modified: CLAUDE.md
# - New files: .specify/memory/constitution.md, .specify/templates/*, etc.
# - Modified: .claude/commands/ (added sp.* files)

# STEP 20: Commit with descriptive message
git commit -m "feat: integrate SpecKit Plus with team conventions

- Adopted SpecKit Plus using \`init --here\` (experimental brownfield)
- Moved code/architecture standards to constitution.md
- Preserved AI collaboration patterns in CLAUDE.md
- All custom slash commands intact (/test)
- Kept CLAUDE.md.backup for historical reference

Tested:
- SpecKit Plus commands work (/sp.specify, /sp.plan)
- Custom commands work (/test)
- AI reads constitution (verified Python standards query)"

# STEP 21: Verify commit succeeded
git log --oneline

# Should show three commits:
# - feat: integrate SpecKit Plus with team conventions
# - backup: preserve custom conventions before SpecKit Plus init
# - Initial project with custom conventions and team knowledge
```

---

### Rollback Practice (Safety Verification)

Before finalizing, verify you CAN recover if something goes wrong:

```bash
# STEP 22: Switch back to main branch
git checkout main

# Verify you're on main
git branch
# Output:
#   experiment/speckit-plus
# * main

# STEP 23: Check CLAUDE.md on main branch
cat CLAUDE.md

# Should show: YOUR ORIGINAL CONTENT (33 lines)
# This proves main branch is untouched (perfect rollback point)

# STEP 24: Return to experiment branch
git checkout experiment/speckit-plus

# STEP 25: Practice recovery from backup file
cp CLAUDE.md.backup CLAUDE.md.recovered

# STEP 26: Practice recovery from git history
git show HEAD~2:CLAUDE.md > CLAUDE.md.from-git

# STEP 27: Verify all three recovery methods work
wc -l CLAUDE.md.backup CLAUDE.md.recovered CLAUDE.md.from-git

# Should show:
#      33 CLAUDE.md.backup        (manual backup)
#      33 CLAUDE.md.recovered     (copied from backup)
#      33 CLAUDE.md.from-git      (extracted from git)
# All three methods recovered the same original content ✓
```

**You now have PROOF that the safety workflow provides redundant recovery options.**

---

### Self-Validation Checklist

Verify you completed the full workflow:

- [ ] Created experiment branch (`git branch` shows experiment/speckit-plus)
- [ ] Created backup files (`.backup` files exist)
- [ ] Committed before init (git history shows backup commit)
- [ ] Ran `specifyplus init --here` (`.specify/` directory exists)
- [ ] Inspected changes (`git status` showed modified CLAUDE.md)
- [ ] Compared file sizes (`wc -l` showed 33 → 240 line increase)
- [ ] Verified custom commands preserved (`/test` command still exists)
- [ ] Merged standards to constitution (constitution.md contains your standards)
- [ ] Merged patterns to CLAUDE.md (CLAUDE.md ends with your patterns)
- [ ] Tested SpecKit Plus commands work (`/sp.specify` responds)
- [ ] Tested custom commands work (`/test` responds)
- [ ] Tested AI reads constitution (query about standards got correct answer)
- [ ] Committed merged result (git log shows integration commit)
- [ ] Verified main branch untouched (main has original CLAUDE.md)
- [ ] Practiced recovery methods (all three recovery methods work)

**If all boxes checked**: You successfully executed safe brownfield adoption with zero data loss and full knowledge preservation.

**If any box unchecked**: Return to that step and complete it before proceeding to Layer 4.

---

## Real-World Application - Client Project Strategy

You've tested on a practice project with controlled conditions. Real client projects bring additional complexity: active development teams, larger codebases, more valuable custom content, and organizational constraints. Let's develop a decision framework for navigating real-world brownfield adoption.

### Decision Framework: Should You Use Brownfield Init?

**Before running `specifyplus init --here` on ANY client project, systematically evaluate three dimensions:**

#### Dimension 1: Custom Content Value and Risk

**Critical questions**:
1. How much custom content exists in CLAUDE.md that will be overwritten?
2. How many hours/months of refinement went into creating this content?
3. Is this content documented elsewhere (wiki, docs, team knowledge base)?
4. Can this content be recreated from memory if lost, or is it irreplaceable?
5. How actively is this content referenced by the team (daily vs monthly vs never)?

**Risk assessment matrix**:

| Content Volume | Refinement Time | Risk Level | Backup Strategy |
|---------------|----------------|------------|-----------------|
| 0-50 lines | &lt;1 month | **LOW** | Single backup sufficient |
| 50-200 lines | 1-6 months | **MODERATE** | Redundant backups (git + file) |
| 200-500 lines | 6-12 months | **HIGH** | Redundant backups + team review |
| 500+ lines | 12+ months | **VERY HIGH** | Consider manual integration instead |

**Decision criteria**:
- **LOW risk**: Proceed with standard safety workflow (branch + backup + commit)
- **MODERATE risk**: Add extra checkpoint (backup to separate repository or cloud storage)
- **HIGH risk**: Require team review before proceeding; consider phased migration
- **VERY HIGH risk**: **Do not use `init --here`**; manually integrate SpecKit Plus structure

**Example LOW risk scenario**:
```
CLAUDE.md: 40 lines
Content: Basic code style preferences (Black formatting, type hints)
Documentation: Also in team wiki
Refinement: Created 2 weeks ago
Team usage: Referenced occasionally

Assessment: LOW RISK → Proceed with safety workflow
```

**Example VERY HIGH risk scenario**:
```
CLAUDE.md: 800 lines
Content: 12 months of refined AI collaboration patterns, critical architecture decisions, security protocols
Documentation: ONLY in CLAUDE.md (no wiki, no other source)
Refinement: 12 months of iteration by 5 team members
Team usage: Referenced daily, contains knowledge no single person has complete understanding of

Assessment: VERY HIGH RISK → Do NOT use init --here; manual integration required
```

---

#### Dimension 2: Safety Testing Feasibility

**Critical questions**:
1. Is the project under version control (git) with clean commit history?
2. Can you create an experimental branch without disrupting active development?
3. Do you have time allocated for merge effort (1-4 hours depending on content)?
4. Are there team coordination requirements (notify others, schedule downtime)?
5. Is there a rollback plan if testing reveals unexpected issues?

**Feasibility assessment**:

| Git Status | Branch Strategy | Team Coordination | Feasibility |
|-----------|----------------|-------------------|-------------|
| ✅ Clean git repo | ✅ Can create branch freely | ✅ No coordination needed | **HIGH** - Proceed |
| ✅ Clean git repo | ⚠️ Active development | ✅ Can coordinate | **MODERATE** - Coordinate first |
| ⚠️ Uncommitted changes | ✅ Can create branch | ⚠️ Need coordination | **LOW** - Clean repo first |
| ❌ No git | N/A | N/A | **BLOCKED** - Initialize git first |

**Decision criteria**:
- **HIGH feasibility**: Proceed immediately with safety workflow
- **MODERATE feasibility**: Coordinate with team, schedule testing window
- **LOW feasibility**: Resolve blockers (commit changes, clean repo) before testing
- **BLOCKED**: Must initialize git before ANY brownfield testing

**Example HIGH feasibility**:
```
Git: Clean repository, all changes committed
Development: Solo developer, no active work in progress
Time: 2 hours allocated for testing and merge
Team: No coordination required (solo project)

Assessment: HIGH FEASIBILITY → Proceed with testing
```

**Example BLOCKED**:
```
Git: No git repository initialized
Development: Active production codebase (50 commits/day)
Time: Unclear
Team: 5 developers working simultaneously

Assessment: BLOCKED → Must initialize git first, then reassess
```

---

#### Dimension 3: Merge Effort Estimation

**Critical questions**:
1. How many lines of custom CLAUDE.md need categorization and merging?
2. Are the content categories clear (standards vs architecture vs AI patterns)?
3. Do you have existing SpecKit Plus experience (know constitution structure)?
4. Are there potential conflicts (custom patterns that contradict SpecKit Plus)?
5. Is there time pressure (client deadline, urgent deployment)?

**Effort estimation table**:

| Content Lines | Content Clarity | Conflicts | Estimated Merge Time |
|--------------|----------------|-----------|---------------------|
| &lt;50 lines | Clear categories | None | **15-30 minutes** |
| 50-200 lines | Mostly clear | Minor | **30-60 minutes** |
| 200-500 lines | Mixed categories | Some | **1-2 hours** |
| 500+ lines | Complex/unclear | Significant | **3-6 hours** |

**Complexity factors that increase time**:
- ❌ **Unclear categorization**: Content mixes standards, patterns, and documentation without clear sections (+50% time)
- ❌ **First-time SpecKit Plus user**: Don't know constitution structure yet (+100% time for learning)
- ❌ **Conflicting patterns**: Custom workflows contradict SpecKit Plus approach (+50% time for resolution)
- ❌ **No backup reference**: Lost access to original content, reconstructing from memory (+200% time)

**Decision criteria**:
- **&lt;1 hour**: Low complexity, proceed with standard workflow
- **1-2 hours**: Moderate complexity, allocate sufficient time, don't rush
- **2-4 hours**: High complexity, consider pair programming for merge quality
- **>4 hours**: Very high complexity, evaluate whether manual integration is more efficient

**Example LOW complexity (30 min)**:
```
Content: 80 lines, clearly divided into:
  - Code standards (30 lines)
  - Architecture (20 lines)
  - AI patterns (30 lines)
Experience: Used SpecKit Plus before, know constitution structure
Conflicts: None (patterns align with SpecKit Plus philosophy)

Estimate: 30 minutes (15 min constitution merge + 15 min CLAUDE.md append)
```

**Example HIGH complexity (4 hours)**:
```
Content: 600 lines, mixed categories:
  - Standards interleaved with examples
  - Architecture mixed with deployment procedures
  - AI patterns mixed with team processes
  - No clear section boundaries
Experience: First time using SpecKit Plus (learning curve)
Conflicts: Some custom patterns assume different workflow than SpecKit Plus

Estimate: 4 hours (1 hr learning + 2 hr categorization + 1 hr conflict resolution)
Recommendation: Consider manual integration instead of init --here
```

---

### Real Client Scenario: Complete Decision Walkthrough

**Client context** (realistic consulting scenario):

```
acme-corp-api/
├── CLAUDE.md                (350 lines: coding standards, API guidelines,
│                             security protocols, team workflows)
├── .claude/
│   └── commands/
│       ├── deploy.md        (Railway deployment procedure)
│       ├── test.md          (Pytest + coverage reporting)
│       └── security-scan.md (SAST/DAST security scanning)
├── src/                     (50,000 lines of production Python code)
├── tests/                   (15,000 lines of test code)
└── .git                     (active development, ~20 commits/day by 5 devs)

Team: 5 mid-level developers
Timeline: Interested in SpecKit Plus, no urgent deadline
Budget: 4 hours allocated for evaluation/testing
```

**Your evaluation process**:

#### Step 1: Assess Custom Content Value

```
Lines: 350 (HIGH volume)
Refinement: 8 months of iteration (HIGH investment)
Documentation: ONLY in CLAUDE.md (VERY HIGH risk if lost)
Team usage: Referenced daily (HIGH value)

Content breakdown (from CLAUDE.md inspection):
- Coding standards: 80 lines (Python type hints, API design, error handling)
- API guidelines: 120 lines (REST best practices, versioning, security)
- Security protocols: 50 lines (authentication, rate limiting, data validation)
- Team workflows: 100 lines (PR process, deployment gates, incident response)

Risk level: HIGH (350 lines, 8 months, only source, daily usage)
```

**Decision**: Backup is MANDATORY. Consider extra safeguards (backup to separate repo, cloud storage).

#### Step 2: Assess Safety Testing Feasibility

```
Git status: Clean repository with linear history ✅
Branch strategy: Can create experiment branch, but need to coordinate (5 active devs) ⚠️
Team coordination: Need 30-min team meeting to explain experiment ⚠️
Time allocation: 4 hours budgeted (sufficient for 350-line merge) ✅
Rollback plan: Experiment branch + main untouched (easy rollback) ✅

Feasibility: MODERATE (can proceed after coordination)
```

**Decision**: Schedule team meeting, explain brownfield testing, get buy-in before creating experiment branch.

#### Step 3: Estimate Merge Effort

```
Content: 350 lines with clear categories (standards, guidelines, security, workflows)
Clarity: Well-organized (each category has clear section header) ✅
Experience: You've used SpecKit Plus before (know constitution structure) ✅
Conflicts: Minimal (workflows align with SpecKit Plus philosophy) ✅

Breakdown:
- Coding standards (80 lines) → constitution.md (15 min)
- API guidelines (120 lines) → constitution.md (25 min)
- Security protocols (50 lines) → constitution.md (10 min)
- Team workflows (100 lines) → CLAUDE.md (20 min)
- Review and testing (30 min)

Total estimate: 100 minutes (~2 hours)
```

**Decision**: 2-hour merge effort fits within 4-hour budget. Allocate 2 hours for merge, 1 hour for testing, 1 hour buffer.

---

#### Final Decision: Proceed with Safety Workflow

**Recommendation**: **PROCEED** with brownfield init using enhanced safety workflow.

**Rationale**:
1. ✅ **Risk is manageable**: High-value content BUT redundant backups mitigate risk
2. ✅ **Feasibility confirmed**: Git repo clean, team coordination possible
3. ✅ **Effort is reasonable**: 2-hour merge fits 4-hour budget with buffer

**Execution plan** (week-by-week):

**Week 1: Safe Testing (Isolation)**
```bash
# Day 1: Team coordination
- Schedule 30-min team meeting
- Explain SpecKit Plus value proposition
- Describe brownfield testing plan (experiment branch, no risk to main)
- Get team buy-in

# Day 2: Create safety net
git checkout -b experiment/speckit-plus
cp CLAUDE.md CLAUDE.md.backup
cp -r .claude .claude.backup
git add -A
git commit -m "backup: preserve 8 months of team knowledge before SpecKit Plus init"

# Day 3: Run brownfield init
specifyplus init --here
# Inspect changes (git status, git diff CLAUDE.md, ls .specify/)
# Document what changed (create inspection report for team)

# Day 4-5: DON'T merge yet (pause for evaluation)
# Share inspection report with team
# Demonstrate new .specify/ structure
# Show that custom commands preserved
```

**Week 2: Team Review and Merge Planning**
```bash
# Day 1: Team review session (60 min)
- Show before/after CLAUDE.md comparison
- Explain constitution.md purpose (project standards)
- Discuss where each content category belongs
- Get team consensus on categorization

# Day 2-3: Draft merged content
vim .specify/memory/constitution.md
# Add: Coding standards, API guidelines, security protocols
vim CLAUDE.md
# Append: Team workflows (PR process, deployment, incident response)

# Day 4: Internal review
- Share draft merged files with team
- Collect feedback on categorization
- Revise based on team input

# Day 5: Finalize merge
- Incorporate team feedback
- Commit merged result
git add -A
git commit -m "feat: integrate SpecKit Plus with team conventions (DRAFT)"
```

**Week 3: Testing and Validation**
```bash
# Day 1-3: Team testing (all 5 devs)
- Each dev tests SpecKit Plus commands (/sp.specify, /sp.plan)
- Each dev tests custom commands (/deploy, /test, /security-scan)
- Each dev queries AI about standards ("What are our API versioning rules?")
- Collect feedback on any issues

# Day 4: Address issues
- Fix any categorization problems found during testing
- Update constitution if any standards missing
- Amend commit with fixes

# Day 5: Final team decision
- Team meeting: Continue or rollback?
- If continue → Plan merge to main
- If rollback → Delete experiment branch, iterate on issues
```

**Week 4: Decision Point**
```bash
# Option A: Merge to main (if testing succeeded)
git checkout main
git merge experiment/speckit-plus
# SpecKit Plus now active for entire team

# Option B: Rollback (if issues found)
git checkout main
git branch -D experiment/speckit-plus
# Main branch untouched, original CLAUDE.md intact
# Document lessons learned, iterate on approach
```

---

### Anti-Patterns: What NOT to Do (Learn from Common Mistakes)

**❌ Anti-Pattern 1: No-Backup YOLO Approach**

```bash
# DANGEROUS (catastrophic data loss risk)
cd client-project-with-800-line-CLAUDE
specifyplus init --here
# Type: y
# ... 800 lines of irreplaceable team knowledge just vanished
# No backup, no git commit, no recovery method
```

**Why this fails**: Single typo ("y" instead of "n") causes permanent data loss. No undo button.

**Correct approach**: ALWAYS create redundant backups before experimental commands:
```bash
git checkout -b experiment/test-safety
cp CLAUDE.md CLAUDE.md.backup
git add -A && git commit -m "backup before init"
specifyplus init --here  # NOW safe (3 recovery methods)
```

---

**❌ Anti-Pattern 2: Misunderstanding "Merge" in Warning Message**

```bash
# Terminal output during init:
# "Template files will be merged with existing content..."

# What developers think this means:
# "Great! It will MERGE my CLAUDE.md content with the template
#  (like a git merge). My content will be preserved automatically."

# What it actually means:
# "Template files will be PLACED ALONGSIDE existing PROJECT files
#  (directory-level merge). CLAUDE.md will be OVERWRITTEN (file replaced)."
```

**Why this fails**: "Merge" refers to directory structure, not file contents. CLAUDE.md gets replaced, not merged.

**Correct understanding**:
- ✅ Directory-level merge: `.specify/` created alongside `src/`, `tests/`
- ❌ File-level merge: CLAUDE.md is OVERWRITTEN, not merged

---

**❌ Anti-Pattern 3: Skipping Git Verification**

```bash
# Scenario: Developer assumes project has git
cd legacy-project
specifyplus init --here
# Type: y
# CLAUDE.md overwritten

# Later: "Oh no, where's my custom content?"
# Developer checks: git log
# Error: "not a git repository"
#
# Result: NO RECOVERY METHOD (content permanently lost)
```

**Why this fails**: Without git, there's no historical recovery. Backup file is the ONLY option.

**Correct approach**: ALWAYS verify git exists before brownfield init:
```bash
# Check git status
git status
# If error "not a git repository":
git init
git add -A
git commit -m "Initial commit before SpecKit Plus evaluation"
# NOW safe to proceed with brownfield init
```

---

**❌ Anti-Pattern 4: Time Pressure Leading to Shortcuts**

```bash
# Scenario: Client wants SpecKit Plus demo in 30 minutes
# Developer thinks: "No time for safety workflow, I'll just run init"

cd client-project
specifyplus init --here  # No branch, no backup, no commit
# Type: y

# Demo fails, wants to rollback
# No rollback option (no branch, no backup)
# Result: Client's CLAUDE.md lost, demo becomes disaster recovery session
```

**Why this fails**: Time pressure creates cognitive bias toward shortcuts. Shortcuts create unrecoverable failures.

**Correct approach**: If insufficient time for safety workflow, DON'T RUN BROWNFIELD INIT:
```bash
# Alternative for 30-min demo:
# Create SEPARATE playground instead of modifying client project
mkdir -p /tmp/speckit-demo
cd /tmp/speckit-demo
specifyplus init  # Greenfield init (safe, no client data)
# Demo SpecKit Plus features without risk
# Later: Apply to client project with proper safety workflow
```

---

### Self-Check: Applying the Decision Framework

**Scenario**: You're consulting for a startup with these characteristics:

```
startup-api/
├── CLAUDE.md (600 lines: 12 months of refinement, ONLY source, daily usage)
├── .claude/commands/ (5 custom commands)
├── src/ (80,000 lines production code)
└── .git (NO GIT REPOSITORY!)

Team: 3 senior developers
Interest: Very interested in SpecKit Plus
Timeline: Want to start using it next week
```

**Question**: What do you recommend? Walk through the decision framework.

<details>
<summary>Recommended Approach (Full Analysis)</summary>

**STOP - Do NOT run `init --here` yet. Critical blockers exist.**

**Step 1: Custom Content Value Assessment**
```
Lines: 600 (VERY HIGH volume)
Refinement: 12 months (VERY HIGH investment)
Documentation: ONLY in CLAUDE.md (VERY HIGH risk)
Team usage: Daily (VERY HIGH value)

Risk level: VERY HIGH
```
**Implication**: Backup is absolutely mandatory. Data loss would be catastrophic.

**Step 2: Safety Testing Feasibility Assessment**
```
Git status: ❌ NO GIT REPOSITORY (CRITICAL BLOCKER)
Branch strategy: N/A (can't create branches without git)
Rollback plan: ❌ NONE (no git = no recovery without manual backup)

Feasibility: BLOCKED (must initialize git first)
```
**Implication**: Cannot proceed with ANY brownfield testing until git initialized.

**Step 3: Merge Effort Estimation**
```
Content: 600 lines (will require 3-6 hours to categorize and merge)
Timeline: "Next week" (unclear if 3-6 hours available)

Effort: HIGH (insufficient time assessment)
```
**Implication**: Need explicit time allocation for merge effort.

**Recommendation: PHASED APPROACH (Do NOT use init --here yet)**

**Phase 1: Establish Version Control (Week 1)**
```bash
# CRITICAL: Initialize git FIRST (recovery foundation)
cd startup-api
git init
git add -A
git commit -m "Initial commit: Preserve 12 months of team knowledge before SpecKit Plus evaluation"

# Push to remote (additional safety)
git remote add origin <repository-url>
git push -u origin main
```
**Why**: No brownfield testing is safe without git. This is non-negotiable.

**Phase 2: Isolated Learning (Week 1-2)**
```bash
# Create completely separate playground (zero risk)
mkdir -p /tmp/speckit-learning
cd /tmp/speckit-learning
specifyplus init  # Greenfield init (safe)

# Team learns SpecKit Plus:
# - Explore .specify/ structure
# - Understand constitution.md purpose
# - Practice /sp.specify, /sp.plan workflows
# - Understand where their content would go
```
**Why**: Learn SpecKit Plus without risking production content. Builds knowledge before applying to real project.

**Phase 3: Manual Integration (Week 3) - NOT init --here**
```bash
# Option A: Manual structure creation (SAFEST for 600 lines)
cd startup-api
git checkout -b experiment/speckit-manual-integration

# Copy .specify/ structure from playground
cp -r /tmp/speckit-learning/.specify .

# DON'T run init --here (too risky for 600 lines)
# Instead, manually integrate:
# 1. Review CLAUDE.md categories (coding, architecture, security, etc.)
# 2. Add standards to .specify/memory/constitution.md
# 3. Keep AI patterns in existing CLAUDE.md (append SpecKit Plus template)
# 4. Copy slash commands from playground

# This gives you FULL CONTROL over merge (no overwrites)
```
**Why**: 600 lines is too valuable to risk automatic overwrite. Manual integration gives precise control.

**Phase 4: Testing and Validation (Week 3-4)**
```bash
# Test on experiment branch (main untouched)
# - Verify SpecKit Plus commands work
# - Verify custom commands work
# - Verify AI reads constitution
# - Team tests for 1 week

# Decision point:
# If successful → merge to main
# If issues → iterate on experiment branch
```

**Timeline Reality Check**:
- Week 1: Git initialization + isolated learning (8 hours)
- Week 2: Continue learning, plan manual integration (8 hours)
- Week 3: Manual integration + initial testing (12 hours)
- Week 4: Team validation + decision (8 hours)
- **Total**: 36 hours over 4 weeks (NOT "next week")

**What to tell the client**:
> "I recommend a phased approach that preserves your 12 months of team knowledge while adopting SpecKit Plus safely:
>
> **Week 1**: Initialize git (recovery foundation) + learn SpecKit Plus in isolated environment
> **Week 2-3**: Manually integrate SpecKit Plus structure (no risky overwrites)
> **Week 4**: Team testing and validation
>
> The `init --here` command is experimental and would overwrite your 600-line CLAUDE.md. For this volume of valuable content, manual integration is safer and gives you full control.
>
> Timeline is 4 weeks, not 1 week. Rushing this risks losing 12 months of irreplaceable knowledge."

**Key insight**: Sometimes the right answer is "don't use the automated tool." For very high-value content (500+ lines), manual integration is often safer and more appropriate than `init --here`.
</details>

---

## Summary: Brownfield Adoption Reality Check

### What You Learned (Evidence-Based Knowledge)

**1. Actual behavior of `specifyplus init --here`** (tested, not theoretical):
   - **Overwrites**: `CLAUDE.md` completely replaced with SpecKit Plus template (~240 lines)
   - **Preserves**: `.claude/commands/` custom slash commands remain intact
   - **Creates**: `.specify/` infrastructure (constitution, templates, scripts)
   - **Untouched**: All source code, tests, README, configuration files

**2. Safety workflow** (redundant recovery methods):
   - **Branch isolation**: Experimental branch keeps main branch untouched (easy rollback)
   - **Backup files**: Manual `.backup` files for quick reference without git
   - **Git commit**: Historical recovery via `git show HEAD~1:CLAUDE.md`
   - **Three recovery paths**: Any single failure doesn't cause data loss

**3. Merge strategy** (intelligent content categorization):
   - **Coding standards** → `.specify/memory/constitution.md` (project rules)
   - **Architecture principles** → constitution.md (project constraints)
   - **AI collaboration patterns** → append to `CLAUDE.md` (behavioral instructions)
   - **Custom commands** → already preserved in `.claude/commands/`

**4. Decision framework** (three-dimensional risk assessment):
   - **Dimension 1**: Custom content value (LOW/MODERATE/HIGH/VERY HIGH)
   - **Dimension 2**: Safety testing feasibility (BLOCKED/LOW/MODERATE/HIGH)
   - **Dimension 3**: Merge effort estimation (15 min to 6+ hours)
   - **Decision**: Combine assessments to determine appropriate approach

**5. When NOT to use `init --here`** (critical judgment):
   - ❌ No git repository (initialize git first, non-negotiable)
   - ❌ 500+ lines custom content (manual integration safer)
   - ❌ Unclear time allocation (merge effort unknown)
   - ❌ No backup plan (no recovery method)

### Key Principles (Remember These)

**Principle 1**: `init --here` is experimental - Official brownfield support with automatic merging is coming, but not here yet. Current workflow requires manual backup and merge.

**Principle 2**: Backup is mandatory, not optional - CLAUDE.md overwrite is guaranteed (not "may happen"). Without backup (git or manual), recovery is impossible.

**Principle 3**: Git is required for safe testing - Only reliable recovery method besides manual backup files. No git = no brownfield testing.

**Principle 4**: Custom commands are safe - `.claude/commands/` directory is preserved. Your team's workflows continue working.

**Principle 5**: Constitution is powerful - Better home for project-specific standards (coding, architecture, testing). Separates project rules from AI behavior.

**Principle 6**: Sometimes manual is better - For very high-value content (500+ lines, 12+ months refinement), manual integration gives more control than automated `init --here`.

### Next Steps in Your Learning Journey

**In your own projects**:
1. ✅ Evaluate current CLAUDE.md content (volume, value, refinement time)
2. ✅ Verify git repository exists (initialize if needed)
3. ✅ Apply decision framework (assess risk, feasibility, effort)
4. ✅ Choose appropriate approach (init --here with safety OR manual integration)
5. ✅ Execute with redundant backups (branch + file + commit)
6. ✅ Merge intelligently (constitution for standards, CLAUDE.md for patterns)

**Watch for SpecKit Plus updates** (brownfield support evolving):
- 📋 Official brownfield support with automatic CLAUDE.md merging
- 📋 GitHub integration for existing repositories
- 📋 Migration tooling for complex scenarios (500+ line CLAUDE.md)
- 📋 Conflict resolution UI for merging team knowledge with templates

**Current best practice** (as of lesson date): Test in isolated branch, create redundant backups, merge manually, validate thoroughly before merging to main.

---

### Decision Framework Checklist

Use this checklist BEFORE running `specifyplus init --here` on any project:

#### Pre-Flight Safety Checklist

**Git Repository Status**:
- [ ] Git repository exists (`git status` works without error)
- [ ] All changes committed (`git status` shows "working tree clean")
- [ ] Remote repository configured (optional but recommended for extra backup)
- [ ] Clean linear history (no complex merge conflicts in progress)

**Content Value Assessment**:
- [ ] Custom CLAUDE.md content volume documented (line count: ____)
- [ ] Refinement time assessed (weeks/months: ____)
- [ ] Content is documented elsewhere OR is the only source (check: ____)
- [ ] Team usage frequency known (daily/weekly/monthly/rarely: ____)
- [ ] Risk level determined (LOW/MODERATE/HIGH/VERY HIGH: ____)

**Safety Preparation**:
- [ ] Experimental branch will be created (`experiment/speckit-plus`)
- [ ] Backup files will be created (`.backup` files)
- [ ] Baseline commit will be created (before init)
- [ ] Team coordinated (if multi-developer project)
- [ ] Time allocated for merge effort (estimated hours: ____)

**Merge Strategy**:
- [ ] Content categories identified (standards, architecture, AI patterns, etc.)
- [ ] Categorization decision made (what goes to constitution vs CLAUDE.md)
- [ ] Conflict potential assessed (any custom patterns contradict SpecKit Plus?)
- [ ] Merge effort estimated (15 min / 30-60 min / 1-2 hrs / 3+ hrs: ____)

**Rollback Plan**:
- [ ] Branch rollback procedure documented
- [ ] Backup file recovery procedure documented
- [ ] Git history recovery procedure documented
- [ ] Team knows how to rollback (if multi-developer project)

**Decision Criteria**:
- [ ] ALL checkboxes above are checked
- [ ] Risk level is acceptable (not VERY HIGH without extra precautions)
- [ ] Time allocation is realistic (not rushed)
- [ ] Team is aligned (if multi-developer project)

**ONLY proceed with `specifyplus init --here` when ALL boxes checked.**

If any box unchecked → Address blocker before proceeding.

---

### Common Error Messages and Solutions

**Error 1**: `fatal: not a git repository`
```bash
# Error appears when running: git status
# Cause: Project has no git repository initialized

# Solution: Initialize git first
git init
git add -A
git commit -m "Initial commit before SpecKit Plus testing"
# NOW safe to proceed with brownfield workflow
```

**Error 2**: `error: pathspec 'experiment/speckit-plus' did not match any file(s) known to git`
```bash
# Error appears when running: git checkout -b experiment/speckit-plus
# Cause: Uncommitted changes prevent branch creation

# Solution: Commit or stash changes first
git status  # See what's uncommitted
git add -A
git commit -m "WIP: commit before creating experiment branch"
# NOW branch creation will work
```

**Error 3**: `Warning: Current directory is not empty (X items)` → But worried about safety
```bash
# This warning is EXPECTED (you're in existing project)
# Question: Is it safe to type 'y'?

# Safe to proceed ONLY IF:
# 1. You created experimental branch (git branch shows experiment/*)
# 2. You created backup files (ls shows CLAUDE.md.backup)
# 3. You committed backup (git log shows backup commit)

# If ALL THREE true → Type: y (safe)
# If ANY missing → Type: n (abort, complete safety workflow first)
```

**Error 4**: `CLAUDE.md was overwritten and I didn't create backup!`
```bash
# Panic scenario: Lost custom content, no .backup file

# Recovery attempt: Check git history
git log --oneline
# Look for commit BEFORE init (anything showing your project work)

# If commits exist:
git show HEAD~1:CLAUDE.md  # View old content
git show HEAD~1:CLAUDE.md > CLAUDE.md.recovered  # Save to file

# If NO commits exist:
# Unfortunately, content is unrecoverable without backup
# Lesson learned: ALWAYS create redundant backups before experimental commands
```

---

## Optional Enhancement: Extracting Intelligence from Legacy Code

**When to use this**: After successfully completing brownfield adoption, you may have valuable architectural knowledge locked in your codebase that isn't documented anywhere.

### The Problem

You've safely adopted SpecKit Plus and preserved your team's explicit knowledge:
- ✅ CLAUDE.md merged with SpecKit Plus template
- ✅ Constitution contains your coding standards
- ✅ Custom commands preserved

**But your codebase still has undocumented knowledge**:
- Architectural patterns visible only in code structure
- Domain logic and business rules scattered across files
- Design decisions made but never written down
- Reusable patterns worth extracting as skills

### The Solution: `/reverse-engineer` Command

SpecKit Plus includes a command to extract specifications and intelligence FROM existing code:

```bash
claude
> /reverse-engineer src/
```

**What it does**:
1. Analyzes your codebase to discover architectural patterns
2. Extracts implicit requirements from implementation
3. Generates specifications (spec.md, plan.md, tasks.md)
4. Identifies reusable patterns worth creating skills

**Example**: Analyzing a 50,000-line Flask API might discover:
- 15 API endpoints with authentication patterns
- Repository pattern used across 8 classes
- Three-layer architecture (controllers → services → repositories)
- Custom error handling hierarchy
- Rate limiting and caching strategies

### How to Use It

**Full workflow** (after completing brownfield adoption):

```bash
# PHASE 1: Brownfield adoption (what you just learned)
git checkout -b experiment/speckit-plus
cp CLAUDE.md CLAUDE.md.backup
specifyplus init --here
# (Merge content, commit result)

# PHASE 2: Extract codebase intelligence (optional)
claude
> /reverse-engineer src/
# Generates: spec.md, plan.md, tasks.md, intelligence-object.md

# PHASE 3: Enhance constitution with discovered patterns
> /sp.constitution
# Reviews intelligence-object.md, suggests additions to constitution

# PHASE 4: Commit intelligence artifacts
git add spec.md plan.md tasks.md intelligence-object.md
git commit -m "feat: extract codebase intelligence"
```

### When to Use This

**Use `/reverse-engineer` when**:
- ✅ Large codebase (10,000+ lines) with implicit knowledge
- ✅ Original developers left, no documentation exists
- ✅ Clear architectural patterns visible in code
- ✅ Planning major refactor (need baseline spec)

**Skip when**:
- ❌ Small codebase (&lt;1,000 lines) - manual docs faster
- ❌ Code is throwaway prototype - not worth investment
- ❌ Architecture is chaotic - no clear patterns

### Time Investment

| Codebase Size | Estimated Time | ROI |
|--------------|----------------|-----|
| 5,000 lines | 2-3 hours | Onboarding time reduced from 1 week to 1 day |
| 50,000 lines | 8-10 hours | Knowledge preserved, new devs productive immediately |
| 100,000+ lines | 12-15 hours | Critical for legacy systems with no docs |

### Learn More

**Full command documentation**: [https://github.com/panaversity/ai-native-software-development/blob/main/.claude/commands/reverse-engineer.md](https://github.com/panaversity/ai-native-software-development/blob/main/.claude/commands/reverse-engineer.md)

The command includes:
- 6-dimension analysis framework (intent archaeology, architecture recognition, code decomposition, intelligence extraction, gap analysis, regeneration blueprint)
- Step-by-step workflow for discovering patterns
- Examples of generated artifacts (specs, intelligence objects)
- Guidance on creating reusable skills from discovered patterns

### Key Takeaway

**Brownfield adoption has two levels**:

1. **File-based knowledge** (this lesson's main focus): Preserve what developers wrote down (CLAUDE.md, standards, commands)

2. **Code-based knowledge** (this optional enhancement): Extract what developers encoded in implementation (architecture, patterns, decisions)

Together, they provide comprehensive knowledge preservation for brownfield projects.

---

## Try With AI

**Setup**: Open your preferred AI coding assistant (Claude Code, Cursor, Windsurf, etc.) and practice the brownfield adoption workflow with AI guidance.

**Prompt Set** (Progressive complexity):

**Prompt 1: Safety Workflow Design** (Basic)
```
I have an existing project with a custom CLAUDE.md file (150 lines of team knowledge).
I want to test adding SpecKit Plus using `specifyplus init --here`.

Help me design a safety workflow that:
1. Prevents data loss if init overwrites files
2. Gives me multiple recovery options
3. Lets me test without affecting my main branch

Ask questions about my project if you need more context.
```

**Prompt 2: Merge Strategy Planning** (Intermediate)
```
After running `specifyplus init --here`, my CLAUDE.md was overwritten (expected).
I have a backup with this structure:

## Code Standards (40 lines)
- Python type hints, docstring style, line length

## Architecture (30 lines)
- FastAPI microservices, PostgreSQL, REST API design

## AI Collaboration Patterns (80 lines)
- Test-first development, spec before implementation, review AI critically

I now have:
- New CLAUDE.md (240 lines of SpecKit Plus template)
- New .specify/memory/constitution.md (mostly boilerplate)

Help me decide:
1. Which sections go to constitution.md vs stay in CLAUDE.md?
2. How do I integrate without duplicating content?
3. What's the best way to preserve my 80 lines of AI patterns?
```

**Prompt 3: Client Project Decision** (Advanced)
```
I'm consulting for a client with this project:

CLAUDE.md: 500 lines (12 months refinement, only source, daily usage by 5 devs)
Git: NO git repository initialized (!)
Team: Active development (40 commits/day)
Interest: Want to use SpecKit Plus "as soon as possible"

Apply the brownfield adoption decision framework:
1. Assess custom content value and risk
2. Evaluate safety testing feasibility
3. Estimate merge effort
4. Recommend an approach (init --here vs manual integration vs other)

Create a week-by-week plan I could present to the client.
Include timeline, risks, and success criteria.
```

**Expected Outcomes**:
- **Prompt 1**: AI should suggest experimental branch + backup files + git commit (three-layer safety). If it suggests anything less redundant, push back: "What if the backup file gets deleted?"
- **Prompt 2**: AI should recommend standards/architecture → constitution.md, AI patterns → append to CLAUDE.md. It might suggest reviewing for conflicts between your patterns and SpecKit Plus patterns (good reasoning).
- **Prompt 3**: AI should identify CRITICAL BLOCKER (no git), recommend phased approach (git init → isolated learning → manual integration, NOT init --here). If it suggests immediate init --here, that's a red flag.
