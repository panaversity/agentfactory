---
title: "Team Collaboration and Reproducible Environments with AI"
chapter: 12
lesson: 6
sidebar_position: 6
duration_minutes: 75

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Share Projects Reproducibly"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Collaboration"
    measurable_at_this_level: "Student can prepare project for sharing (verify lockfile, commit correct files to git), guide teammates through setup ('Clone the repo and run uv sync'), and explain why lockfiles ensure identical environments"

  - name: "Understand Lockfile Purpose"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can differentiate pyproject.toml (constraints) from uv.lock (exact versions), explain when lockfiles update, describe production deployment strategies (--no-dev), and diagnose 'works on my machine' issues"

learning_objectives:
  - objective: "Apply team collaboration workflows (git + lockfiles) to share UV projects reproducibly"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successfully prepares project for sharing, recreates teammate's environment using lockfile"

  - objective: "Understand lockfile mechanics, pyproject.toml vs uv.lock differences, and production deployment"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Can explain why lockfiles solve 'works on my machine', when to use --no-dev, and git best practices"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (lockfile reproducibility, uv sync, pyproject vs lock, git workflow, dev vs prod install, lockfile maintenance, 'works on my machine' problem) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore lockfile diff analysis (comparing versions across branches); research dependency pinning strategies (exact vs. range); investigate CI/CD integration (GitHub Actions with uv sync)"
  remedial_for_struggling: "Focus on clone → uv sync workflow first; use AI to verify each step; build confidence with successful environment recreation before exploring production deployment"

# Generation metadata
generated_by: "lesson-writer v1.0"
source_spec: "specs/011-python-uv/spec.md"
created: "2025-11-04"
last_modified: "2025-11-04"
git_author: "GitHub Copilot"
workflow: "/sp.implement"
version: "1.0.0"
---

# Team Collaboration and Reproducible Environments with AI

You've created projects, managed dependencies, and run code in isolated environments. Now comes the critical team challenge: **How do you ensure your teammates (or CI/CD systems, or production servers) have the exact same environment?**

This lesson answers the "works on my machine" problem—one of software development's most frustrating issues. You'll learn how UV's lockfiles (`uv.lock`) create reproducible environments, how to share projects via git, and how to set up teammates for instant productivity with `uv sync`.

By the end of this lesson, you'll confidently prepare projects for collaboration, recreate environments from lockfiles, understand production deployment strategies, and know exactly what to commit to git (and what to ignore). You'll use AI to guide these workflows while understanding the principles behind reproducibility.

## The Collaboration Problem: "Works on My Machine"

Let's establish why environment reproducibility matters through a real scenario.

### Scenario: The Mysterious Bug

**Monday morning**:
- **You**: "I deployed the new feature to production. Works perfectly!"
- **Teammate**: "I'm getting errors when running the tests on my machine."
- **Production Server**: "The feature crashes with 'AttributeError' on customer requests."

**Investigation reveals**:
- **Your machine**: `requests==2.33.0` (latest version with new features)
- **Teammate's machine**: `requests==2.28.0` (installed three months ago)
- **Production server**: `requests==2.31.0` (installed last week)

**Three different environments, three different behaviors.** Your code works because you wrote it against 2.33.0's API. The teammate's older version doesn't have the method you used. Production's middle version has a bug you didn't encounter.

### The Root Cause

**Without lockfiles**:
- Each developer runs `pip install requests` (or `uv add requests`)
- Each gets *whatever version is latest at that moment*
- Environments drift apart over weeks/months
- Code that works for one person breaks for others

**With lockfiles (UV's solution)**:
- You create the project with `requests==2.33.0`
- UV records this in `uv.lock`: "requests==2.33.0"
- Teammates run `uv sync`, get *exactly* 2.33.0
- Production deploys with `uv sync`, gets *exactly* 2.33.0
- **Identical environments, identical behavior**

### Analogy: Recipe Cards vs. Exact Ingredient List

**Without lockfile** (pyproject.toml only):
- Recipe says "flour (version 2.0 or newer)"
- Baker 1 uses flour 2.3, Baker 2 uses flour 2.5
- Cakes taste slightly different (flour formulations vary)

**With lockfile** (pyproject.toml + uv.lock):
- Recipe says "flour (version 2.0 or newer)"
- Lockfile says "we successfully used King Arthur Flour version 2.3.1, lot #12345"
- All bakers use *exactly* that brand, version, and lot
- Cakes taste identical

**Key concept**: Lockfiles are the "recipe cards" that record exactly what worked, so everyone can reproduce your success.

## Lockfiles to the Rescue: How uv.lock Enables Reproducibility

Let's understand what `uv.lock` contains and how it works.

### What's in uv.lock?

`uv.lock` is a machine-generated file (you never edit it manually) that records:

1. **Exact versions** of every package (including transitive dependencies)
2. **Checksums** (hashes to verify package integrity)
3. **Source URLs** (where packages were downloaded from)
4. **Python version** (what version the lockfile was created with)

**Example snippet** (simplified):
```toml
[[package]]
name = "requests"
version = "2.33.0"
source = { registry = "https://pypi.org/simple" }
dependencies = [
    { name = "certifi", version = "2024.7.4" },
    { name = "charset-normalizer", version = "3.3.2" },
    { name = "idna", version = "3.7" },
    { name = "urllib3", version = "2.2.2" },
]
```

**What this means**: When you run `uv sync`, UV installs `requests==2.33.0` *and* its exact dependency versions (not latest compatible, but the exact versions you used).

### pyproject.toml vs. uv.lock: Two Files, Different Jobs

| File | Purpose | Who Edits | Format | Example |
|------|---------|-----------|--------|---------|
| **pyproject.toml** | Your *intent* (what you want) | **You** (manually or via `uv add`) | Human-readable | `requests>=2.32.3` |
| **uv.lock** | Exact *resolution* (what you got) | **UV** (auto-generated) | Machine-optimized | `requests==2.33.0` |

**Analogy**:
- **pyproject.toml**: "I want a fast car" (high-level requirement)
- **uv.lock**: "Here's the VIN number of the specific Tesla Model 3 you bought" (exact record)

**Why both?**:
- **pyproject.toml**: Lets you update later (when `requests==2.34.0` comes out, your constraint `>=2.32.3` allows it)
- **uv.lock**: Guarantees reproducibility *right now* (teammates get 2.33.0, not 2.34.0, until you explicitly update)

### When UV Updates uv.lock

UV regenerates `uv.lock` automatically when you:
- Add a dependency: `uv add package` (resolves new package, pins all versions)
- Update a dependency: `uv add package@latest` (resolves new version, pins it)
- Remove a dependency: `uv remove package` (re-resolves without removed package)
- Explicitly sync: `uv sync` (recreates environment from lockfile)

**You never run**: `uv lock` (no such command—locking happens automatically during add/remove/sync)

## Preparing Your Project for Sharing with AI

Let's walk through preparing a project for teammates or deployment.

### Pre-Share Checklist

Before sharing your project (via git, zip file, or CI/CD), ensure:

**Prompt to AI**:
```
I want to share my UV project with my team. 
What files should exist and what should I verify before pushing to git?
```

**AI's Response** (approximate):

> **Pre-Share Checklist**:
>
> **1. Verify lockfile exists**:
> ```bash
> ls uv.lock
> ```
> (Or `dir uv.lock` on Windows—should list the file)
>
> If missing, create it:
> ```bash
> uv sync
> ```
> (UV generates `uv.lock` from your current dependencies)
>
> **2. Verify .gitignore excludes .venv/**:
> Your `.gitignore` should contain:
> ```
> .venv/
> __pycache__/
> *.pyc
> .pytest_cache/
> ```
> **Never commit**:
> - `.venv/` (too large, machine-specific)
> - `__pycache__/` (compiled Python bytecode)
>
> **Always commit**:
> - `pyproject.toml` (your dependency specifications)
> - `uv.lock` (exact versions for reproducibility)
> - Source code (src/, tests/, etc.)
>
> **3. Test local reproducibility**:
> Delete `.venv/` and recreate:
> ```bash
> rm -rf .venv    # (or `rmdir /s .venv` on Windows)
> uv sync
> uv run pytest   # Verify tests still pass
> ```
> If tests pass, your lockfile is good.
>
> **4. Commit to git**:
> ```bash
> git add pyproject.toml uv.lock src/ tests/
> git commit -m "Add project dependencies and tests"
> git push
> ```

**Run through this checklist**. Confirm `uv.lock` exists, `.gitignore` is correct, and tests pass after fresh `uv sync`.

### Cross-Platform Note

`uv.lock` is cross-platform—same file works on Windows, macOS, and Linux (UV handles platform-specific package variants internally). One lockfile, all platforms.

## Teammate Setup Flow: Cloning and Syncing with AI

Now let's see the teammate's perspective—recreating your environment.

### Scenario: Your Teammate Joins the Project

**Your teammate** receives the git repository. They've never worked on this project before.

**Prompt they give to their AI**:
```
I just cloned a Python project that uses UV. 
How do I set up the environment to start working?
```

**AI's Response** (approximate):

> **Setup Steps for UV Project**:
>
> **1. Install UV** (if not already installed):
> - Windows PowerShell: `powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"`
> - macOS/Linux: `curl -LsSf https://astral.sh/uv/install.sh | sh`
>
> **2. Navigate to project directory**:
> ```bash
> cd project-name
> ```
>
> **3. Sync environment from lockfile**:
> ```bash
> uv sync
> ```
>
> **What `uv sync` does**:
> - Reads `uv.lock` to see exact package versions
> - Creates `.venv/` directory
> - Installs all packages with exact versions from lockfile
> - Sets Python version from `.python-version` (if present)
> - Result: **Identical environment to original developer**
>
> **Expected output** (example):
> ```
> Using Python 3.13.0 from .python-version
> Creating virtual environment at .venv
> Resolving dependencies from uv.lock... (0.2s)
> Downloaded 13 packages in 1.5s
> Installed 13 packages in 0.4s
>  + requests==2.33.0
>  + certifi==2024.7.4
>  + ...
> ```
>
> **4. Verify installation**:
> ```bash
> uv run pytest
> ```
> Tests should pass (same environment = same behavior).
>
> **5. Start working**:
> - Edit code in `src/`
> - Run tests: `uv run pytest`
> - Add dependencies: `uv add package-name`
> - Commit changes (including updated `uv.lock` if you added dependencies)

**Key magic**: `uv sync` reads the lockfile and recreates the *exact* environment. No version guessing, no compatibility issues. What worked for you works for your teammate.

### Troubleshooting Sync Issues

**Common issue**: Teammate runs `uv sync` but sees warnings or errors.

**Prompt to AI**:
```
I ran 'uv sync' but got this warning:
"Warning: uv.lock is out of sync with pyproject.toml"
What does this mean and how do I fix it?
```

**AI's Response**:

> **Problem**: Someone edited `pyproject.toml` (added/changed dependency) but didn't regenerate `uv.lock`.
>
> **Solution**: Regenerate lockfile:
> ```bash
> uv lock --upgrade
> ```
> (Or simply `uv sync` again—UV detects mismatch and updates lockfile)
>
> **Prevention**: Always commit both `pyproject.toml` and `uv.lock` together. If you add a dependency, commit both files in the same commit.

## Production Deployments: Installing Without Dev Dependencies

When deploying to production (servers, Docker containers, cloud functions), you don't need testing/linting tools. Let's install lean production environments.

### Scenario: Deploying to a Server

Your project has:
- **Production dependencies**: `requests`, `fastapi`, `pydantic` (runtime needs)
- **Dev dependencies**: `pytest`, `pytest-cov`, `ruff`, `mypy` (development tools)

**Production doesn't need dev tools**. Installing them wastes space, increases attack surface, and slows startup.

**Prompt to AI**:
```
I'm deploying my UV project to a production server. 
How do I install only production dependencies (excluding dev tools)?
```

**AI's Response** (approximate):

> **Production Installation Command**:
> ```bash
> uv sync --no-dev
> ```
>
> **What `--no-dev` does**:
> - Reads `uv.lock` for exact versions (same as regular `uv sync`)
> - Installs only packages in `[project.dependencies]` section
> - **Skips** packages in `[project.optional-dependencies]` dev group
> - Result: Lean environment (100-200 MB smaller without pytest, ruff, etc.)
>
> **Example**:
> - **Development install** (`uv sync`): 25 packages, 350 MB
> - **Production install** (`uv sync --no-dev`): 12 packages, 150 MB
>
> **When to use**:
> - Docker builds: `RUN uv sync --no-dev` (smaller image)
> - Cloud functions: Deploy script runs `uv sync --no-dev` (faster cold starts)
> - Server deployments: Production doesn't run tests (testing happens in CI/CD)
>
> **Security benefit**: Fewer packages = smaller attack surface. Dev tools sometimes have vulnerabilities that don't matter in development but are risks in production.

### Docker Example

**Dockerfile snippet** (simplified):

```dockerfile
FROM python:3.13-slim

WORKDIR /app

# Copy dependency files
COPY pyproject.toml uv.lock ./

# Install UV and sync production dependencies
RUN pip install uv && uv sync --no-dev

# Copy application code
COPY src/ ./src/

# Run application
CMD ["uv", "run", "python", "-m", "src.main"]
```

**Key line**: `uv sync --no-dev` (production deployment, no dev tools)

## Git Integration: Committing the Right Files

Let's clarify what belongs in version control and what doesn't.

### Git Best Practices for UV Projects

**Always commit to git**:
- ✅ `pyproject.toml` (dependency specifications)
- ✅ `uv.lock` (exact versions for reproducibility)
- ✅ `.python-version` (Python version for project)
- ✅ Source code (`src/`, `tests/`, etc.)
- ✅ Documentation (`README.md`, `docs/`, etc.)
- ✅ Configuration files (`.gitignore`, CI/CD configs)

**Never commit to git**:
- ❌ `.venv/` (virtual environment directory—too large, machine-specific)
- ❌ `__pycache__/` (compiled Python bytecode—regenerated automatically)
- ❌ `*.pyc` (individual compiled files)
- ❌ `.pytest_cache/` (pytest cache—temporary)
- ❌ `.mypy_cache/` (mypy cache—temporary)
- ❌ `*.egg-info/` (package metadata—build artifacts)

### Setting Up .gitignore

**Prompt to AI**:
```
Create a .gitignore file for my UV Python project that excludes 
virtual environments, caches, and build artifacts.
```

**AI creates** (typical `.gitignore`):

```gitignore
# Virtual environments
.venv/
venv/
ENV/

# Python caches
__pycache__/
*.py[cod]
*$py.class

# Testing
.pytest_cache/
.coverage
htmlcov/

# Type checking
.mypy_cache/

# Distribution / packaging
dist/
build/
*.egg-info/

# IDE
.vscode/
.idea/
*.swp
*.swo

# OS
.DS_Store
Thumbs.db
```

**Add this to your project root**. Commit `.gitignore` itself to git.

### Committing Dependency Changes

When you add/update/remove dependencies:

**Workflow**:
1. **Make change**: `uv add new-package`
2. **Both files update**: `pyproject.toml` and `uv.lock` change
3. **Commit both**:
   ```bash
   git add pyproject.toml uv.lock
   git commit -m "Add new-package for feature X"
   ```

**Never commit one without the other**. They must stay in sync.

## Lockfile Maintenance: When to Update

Lockfiles aren't static—you'll update them as projects evolve.

### When Lockfiles Update

**Automatic updates** (UV regenerates `uv.lock`):
- Adding dependency: `uv add package`
- Updating dependency: `uv add package@latest`
- Removing dependency: `uv remove package`
- Syncing environment: `uv sync` (if lockfile is out of sync with pyproject.toml)

**Manual lockfile refresh** (rare):
- If you manually edited `pyproject.toml` (e.g., changed version constraint), run:
  ```bash
  uv lock --upgrade
  ```

### Updating All Dependencies

**Scenario**: You want to update all packages to their latest compatible versions (monthly maintenance).

**Prompt to AI**:
```
Update all packages in my UV project to their latest versions 
that are compatible with my constraints in pyproject.toml.
```

**AI's Response**:

> **Update Workflow**:
>
> **1. Check what's outdated**:
> ```bash
> uv pip list --outdated
> ```
> Review list of packages with newer versions.
>
> **2. Update all packages**:
> ```bash
> uv lock --upgrade
> uv sync
> ```
> (`--upgrade` resolves to latest compatible versions, `sync` installs them)
>
> **3. Test thoroughly**:
> ```bash
> uv run pytest
> ```
> Verify all tests pass (updates can introduce breaking changes).
>
> **4. Commit new lockfile**:
> ```bash
> git add uv.lock
> git commit -m "Update dependencies to latest versions"
> ```
>
> **When to do this**:
> - Monthly/quarterly scheduled updates
> - After security vulnerability announcements
> - Before starting major new features (start with fresh dependencies)
>
> **When NOT to do this**:
> - Mid-feature development (wait until feature is complete)
> - Right before production deployment (update earlier, test longer)

## Collaboration Workflow: Developer A → Git → Developer B

Here's the complete collaboration flow.

### The Workflow Steps

**Developer A** (creates feature):
1. Creates UV project: `uv init my-project`
2. Adds dependencies: `uv add requests pytest --dev`
3. Writes code and tests
4. Commits to git: `pyproject.toml`, `uv.lock`, `src/`, `tests/`
5. Pushes to GitHub/GitLab

**Git Repository**:
- Stores: `pyproject.toml`, `uv.lock`, source code
- Does NOT store: `.venv/` (excluded by `.gitignore`)

**Developer B** (joins project):
1. Clones repository: `git clone https://github.com/team/my-project`
2. Syncs environment: `uv sync` (reads `uv.lock`, creates `.venv/`)
3. Runs tests: `uv run pytest` (works identically to Developer A)
4. Makes changes, commits (including updated `uv.lock` if dependencies changed)
5. Pushes to git

**Key insight**: Developer B gets Developer A's *exact environment* via `uv.lock`. No "works on my machine" issues—both use `requests==2.33.0` (not whatever version happens to be latest when B clones).

### Workflow Representation

```
Developer A                Git Repository              Developer B
-----------                --------------              -----------
uv init                         →                      git clone
uv add requests                                             ↓
  ↓ (creates uv.lock)                                  uv sync
git commit                      ←                         ↓
  (pyproject.toml,                                     (reads uv.lock,
   uv.lock, src/)                                       creates .venv/)
git push                        →                      uv run pytest
                                                            ↓
                                                       (identical env)
```

**Synchronization point**: `uv.lock` in git ensures both developers have identical environments.

## Try With AI

Practice the complete collaboration workflow with your AI companion tool.

**Setup**: Use your UV project from previous lessons (or create a new one)

### Prompt 1: Prepare for Sharing
```
Verify my UV project is ready to share with teammates. 
Check that uv.lock exists, .gitignore excludes .venv/, 
and tests pass after fresh uv sync.
```

**What you're practicing**: Pre-share checklist validation

**Expected outcome**: AI checks files, runs `uv sync` + `pytest`, confirms project is shareable

**Validation**: Can you explain why each item on the checklist matters?

### Prompt 2: Simulate Teammate Clone
```
Simulate a teammate cloning my project. 
Delete .venv/, then recreate the environment from uv.lock. 
Verify tests still pass.
```

**What you're practicing**: Lockfile-based environment recreation

**Expected outcome**: AI deletes `.venv/`, runs `uv sync`, runs tests (should pass with identical env)

**Validation**: Did `uv sync` recreate the exact environment? Do tests pass?

### Prompt 3: Production Deployment
```
Show me how to install only production dependencies 
(excluding dev tools like pytest). 
Compare the size/package count to a full installation.
```

**What you're practicing**: Lean production deployments

**Expected outcome**: AI runs `uv sync --no-dev`, lists packages (fewer than with dev), explains benefits

**Validation**: Can you explain when to use `--no-dev`? What packages were excluded?

### Prompt 4: Update Dependencies
```
Update all packages in my project to their latest compatible versions. 
Show me what changed in uv.lock, then verify tests still pass.
```

**What you're practicing**: Dependency maintenance

**Expected outcome**: AI runs `uv lock --upgrade`, shows version changes, runs tests

**Validation**: Do you understand the difference between updating (new versions) and syncing (exact versions from lockfile)?

### Prompt 5: Git Workflow
```
Stage and commit my project files for git, ensuring I include 
pyproject.toml and uv.lock but exclude .venv/. 
Show me the .gitignore configuration.
```

**What you're practicing**: Version control best practices

**Expected outcome**: AI shows `.gitignore` contents, stages correct files, prepares commit

**Validation**: Can you explain why `.venv/` is excluded but `uv.lock` is included?

