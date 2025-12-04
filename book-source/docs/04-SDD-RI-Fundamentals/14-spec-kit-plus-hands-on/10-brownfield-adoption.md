---
title: "Brownfield Adoption"
chapter: 14
lesson: 10
duration_minutes: 30
proficiency_level: "B1"
cognitive_load:
  new_concepts: 4
learning_objectives:
  - objective: "Understand greenfield vs brownfield development contexts"
    proficiency_level: "B1"
    bloom_level: "Understand"
  - objective: "Apply constitution-first approach to existing projects"
    proficiency_level: "B1"
    bloom_level: "Apply"
  - objective: "Execute safe brownfield adoption workflow using git and backups"
    proficiency_level: "B1"
    bloom_level: "Apply"
  - objective: "Identify integration points between existing code and Spec-Kit Plus"
    proficiency_level: "B1"
    bloom_level: "Analyze"
generated_by: "content-implementer v1.0.0"
created: "2025-11-26"
last_modified: "2025-11-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Brownfield Adoption

Most of what you've learned in Chapter 14 applies to **greenfield projects**—new codebases where you start from scratch and control the entire structure. But real work is different. You'll inherit existing projects with months or years of accumulated code, decisions, and team knowledge.

**Brownfield adoption** is the process of bringing Spec-Kit Plus into an existing project. The challenge isn't technical—it's strategic. How do you add a structured workflow framework without losing the institutional knowledge your team has already captured? How do you protect against data loss when the framework overwrites key files like `CLAUDE.md`?

This lesson teaches you a safe, proven workflow for brownfield adoption that preserves existing knowledge, prevents accidental data loss, and integrates Spec-Kit Plus incrementally.

---

## Foundation — Greenfield vs Brownfield

### What's the Difference?

**Greenfield Project**: You start with an empty directory. You have no existing:
- Code to protect
- Team conventions to preserve
- Architectural decisions made
- Custom tools or workflows

Running `specifyplus init` in a greenfield project is straightforward—you're building from scratch.

**Brownfield Project**: You inherit an existing codebase with:
- Working code in `src/`, `lib/`, or equivalent directories
- Custom `CLAUDE.md` with team knowledge (coding standards, architecture principles, collaboration patterns)
- Custom slash commands in `.claude/commands/` (project-specific workflows)
- Git history containing architectural decisions and design rationale
- Team members relying on existing conventions

Running `specifyplus init --here` (the brownfield command) in this context requires careful strategy to avoid data loss.

### The Core Problem

When you run `specifyplus init --here`, the command initializes Spec-Kit Plus in your **existing directory**. Here's what happens:

**Files that get OVERWRITTEN** (complete replacement):
- `CLAUDE.md` — Your custom AI instructions get replaced with the Spec-Kit Plus template (~240 lines)
- Any existing `.specify/` directory (if present) gets reset

**Files that are PRESERVED** (completely safe):
- All custom slash commands in `.claude/commands/` are preserved
- All source code (`src/`, `lib/`, etc.) remains untouched
- All tests, configuration files, and project artifacts survive intact
- Your git history is unaffected

**The Risk**: Your `CLAUDE.md` contains months or years of team knowledge. Without a backup before running init, that content is permanently lost—no recovery mechanism exists.

### Example: Three Real Project Types

Let's see how brownfield adoption affects different project types:

**Scenario 1: Existing Blog/Website Project**
```
blog-project/
├── CLAUDE.md          (200 lines of team conventions)
├── src/
│   ├── pages/
│   ├── components/
│   └── styles/
├── README.md
└── .claude/commands/
    └── deploy-netlify.md   (custom deployment workflow)
```

After `init --here`:
- CLAUDE.md (200 → 240 lines) — Your content replaced with template
- `.specify/` directory created
- `deploy-netlify.md` preserved and working
- Source code completely untouched

**Scenario 2: Existing API Project**
```
api-project/
├── CLAUDE.md          (150 lines of coding standards)
├── src/
│   ├── routes/
│   ├── models/
│   └── middleware/
├── tests/
└── .claude/commands/
    ├── run-tests.md
    └── format-code.md
```

After `init --here`:
- CLAUDE.md replaced (your standards lost without backup)
- `.specify/` created with constitution template
- Both custom commands preserved
- Code and tests completely safe

**Scenario 3: Documentation Project**
```
docs-project/
├── CLAUDE.md          (100 lines of documentation standards)
├── docs/
│   ├── guides/
│   ├── api/
│   └── tutorials/
└── .claude/commands/
    └── publish-docs.md
```

After `init --here`:
- CLAUDE.md replaced
- `.specify/` created
- Custom publish command preserved
- Documentation source intact

---

## Collaboration — Safe Brownfield Testing Workflow

The key to safe brownfield adoption is **redundant safety measures**: git branches + manual backups + clear inspection of what actually changed.

### Designing the Workflow

**Your situation**: You have an existing project with a custom `CLAUDE.md` that your team values. You want to test `specifyplus init --here` but can't afford to lose the content.

**AI's response**:
> "The solution is creating multiple recovery points before testing. Here's the proven workflow:
>
> ```bash
> # STEP 1: Create experimental branch (isolates risk from main)
> git checkout -b experiment/specifykit
>
> # STEP 2: Create backup copies (manual recovery option)
> cp CLAUDE.md CLAUDE.md.backup
> cp -r .claude .claude.backup
>
> # STEP 3: Commit current state (git recovery option)
> git add -A
> git commit -m 'backup: preserve team knowledge before Spec-Kit Plus init'
>
> # STEP 4: NOW test safely (you have multiple recovery paths)
> specifyplus init --here
> ```
>
> **Why this approach works**:
> - **Experiment branch**: Your `main` branch stays completely untouched
> - **Backup files**: If you need your original content, `.backup` files have it
> - **Git commit**: Historical recovery via `git show HEAD~1:CLAUDE.md`
> - **Multiple recovery paths**: If one method fails, you have alternatives
>
> Want me to walk through exactly what changes, and how to merge your content afterward?"

### Understanding What Actually Changed

After running init, you inspect the results:

```bash
# Check what changed
git diff --name-only

# Output:
# CLAUDE.md
# .specify/memory/constitution.md
# .specify/memory/context-quality-memory.md
# .specify/templates/spec-template.md
# .specify/templates/plan-template.md
# .specify/templates/tasks-template.md
# [... more .specify/ files ...]
```

Your `CLAUDE.md.backup` still has the original content. Now you can make an informed decision about merging:

**Option 1: Content goes to Constitution** (Recommended for standards and architecture decisions)

Your team's coding standards, architecture principles, and development conventions belong in `.specify/memory/constitution.md`—the project-specific principles file that AI agents read alongside `CLAUDE.md`.

**Option 2: Content appends to CLAUDE.md** (For behavioral collaboration patterns)

Your team's AI collaboration patterns (like "test-first development" or "review AI output critically") belong appended to the end of `CLAUDE.md`, since they're instructions for HOW AI should work with your team.

### Merging Strategy

**For a real project**, here's how you'd merge (using example content):

1. **Read your backup**: `cat CLAUDE.md.backup`
2. **Identify content categories**:
   - Coding standards → Move to `constitution.md`
   - Architecture principles → Move to `constitution.md`
   - AI collaboration patterns → Append to new `CLAUDE.md`
3. **Execute the merge**:
   ```bash
   # Add standards to constitution
   echo "
   ## Project Development Standards
   [paste your coding standards here]
   " >> .specify/memory/constitution.md

   # Append patterns to CLAUDE.md
   echo "
   ## Team AI Collaboration Patterns
   [paste your collaboration patterns here]
   " >> CLAUDE.md
   ```
4. **Verify nothing was lost**:
   ```bash
   # Compare old vs new (backup has all your content)
   diff CLAUDE.md.backup CLAUDE.md.backup.recovered
   ```

---

## Practice — Identifying Your Project's Content

Before you adopt Spec-Kit Plus on a real project, understand what you'd need to preserve.

### Self-Check: Content Categories

For each category, decide where your team's content would go:

**Category 1: Development Standards**
- Type hints requirements (Python)
- Line length limits (80, 100, 120 chars)
- Import ordering conventions
- Naming conventions

**Decision**: These are project rules → Move to `constitution.md`

**Category 2: Architecture Principles**
- Technology choices (Flask vs FastAPI, PostgreSQL vs MongoDB)
- Design patterns (microservices, monolith, serverless)
- Deployment strategy (Docker, Kubernetes, serverless)

**Decision**: These are project constraints → Move to `constitution.md`

**Category 3: AI Collaboration Patterns**
- "Specification first, then code"
- "Review AI output before merging"
- "Test coverage minimum 80%"
- Preferred AI tools and models

**Decision**: These are behavioral instructions → Append to `CLAUDE.md`

### Planning Your Adoption

Write down your actual content before running init:

**What's in your current CLAUDE.md?** (Estimate line counts and categories)
- Development standards: ___ lines
- Architecture principles: ___ lines
- AI collaboration patterns: ___ lines
- Custom workflow notes: ___ lines

**What custom commands do you rely on?** (These are safe)
- `/deploy` or `/deploy-staging`?
- `/test` or `/test-coverage`?
- `/format` or `/lint`?
- Others: ___________________

**What would break if CLAUDE.md disappeared?**
- Team coding standard consensus lost?
- Architecture decisions undocumented?
- AI collaboration practices forgotten?

---

## Try With AI

You're evaluating whether to adopt Spec-Kit Plus on a real project. Let's plan the actual adoption strategy.

**Setup**: Open your project directory with Spec-Kit Plus ready.

**Prompt Set**:

```
Prompt 1 (Understanding your current state):
"My project already has a custom CLAUDE.md with our team's coding standards and
architecture principles. I want to use Spec-Kit Plus but can't lose this content.
What's my safe adoption strategy? Should I back up first? What will actually get overwritten?"

Prompt 2 (Planning the workflow):
"Here's what's in our CLAUDE.md:
[paste your actual content or describe it]

Walk me through the exact steps to:
1. Create a safe testing environment
2. Run specifyplus init --here
3. Merge my team's knowledge with the Spec-Kit Plus template"

Prompt 3 (Merging strategy for your content):
"After running init, how do I decide what content goes to constitution.md vs
appends to CLAUDE.md? Give me specific examples based on [your project type: blog, API, docs]"
```

**Expected Outcomes**:
- Clear understanding of what's safe to overwrite (you know in advance)
- Concrete backup and recovery plan before running init
- Decision framework for where your team's knowledge belongs in Spec-Kit Plus structure

**Safety Note**: Always create both a git branch AND manual backup files before running experimental commands. One backup method is good; two is better; three is gold standard.

---

## Checkpoint: Reflect on Your Adoption Path

Before moving to the capstone, consider:

- **Greenfield vs Brownfield**: Which applies to your next project? (You'll use different workflows)
- **Content Preservation**: What team knowledge would you need to preserve before adopting Spec-Kit Plus?
- **Recovery Readiness**: Could you recover your CLAUDE.md if it got overwritten accidentally? (Git history? Manual backup?)
- **Incremental Adoption**: Would you adopt Spec-Kit Plus all at once, or incrementally? Why?
