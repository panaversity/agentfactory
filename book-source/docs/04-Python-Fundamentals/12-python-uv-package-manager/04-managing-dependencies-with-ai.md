---
title: "Managing Dependencies with AI"
chapter: 12
lesson: 4
sidebar_position: 4
duration_minutes: 90

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Manage Project Dependencies"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can add production and development dependencies by expressing needs ('Add pytest for testing'), understand dependency groups (dev vs. production), and verify installations in pyproject.toml"

  - name: "Understand Dependency Resolution"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain how UV resolves version conflicts (constraint satisfaction), interpret dependency tree output, and ask AI to resolve incompatibilities when they occur"

learning_objectives:
  - objective: "Apply dependency management workflows (add, update, remove packages) using AI"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successfully manages dependencies for a real project with AI guidance"

  - objective: "Understand dependency resolution, version constraints, and dev vs. production dependencies"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Can explain how UV resolves conflicts and why dependencies are separated"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (dependencies definition, production vs dev, resolution, version constraints, lockfiles, transitive dependencies, update/remove workflows) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Research semantic versioning (SemVer); explore dependency resolution algorithms; analyze uv.lock structure in detail; compare UV's resolver to pip's"
  remedial_for_struggling: "Focus on adding/removing single dependencies first; use AI to verify each step; build confidence before exploring conflicts and complex scenarios"

# Generation metadata
generated_by: "lesson-writer v1.0"
source_spec: "specs/011-python-uv/spec.md"
created: "2025-11-04"
last_modified: "2025-11-04"
git_author: "GitHub Copilot"
workflow: "/sp.implement"
version: "1.0.0"
---

# Managing Dependencies with AI

In Lesson 3, you created your first UV project with a single dependency (`requests`). Now you'll learn to manage dependencies professionally—adding production and development packages, updating versions, handling conflicts, and understanding how UV's resolver ensures everything works together.

This lesson demonstrates the full dependency management lifecycle using AI as your guide. You'll never memorize command flags or version syntax. Instead, you'll understand _what dependencies are_, _why projects separate production and development packages_, and _how UV resolves version conflicts_—then let AI handle the mechanical execution.

By the end of this lesson, you'll confidently add testing tools, update outdated packages, troubleshoot conflicts, and maintain clean dependency lists. You'll understand the lockfile's role in reproducibility and how UV's resolver protects you from "dependency hell."

## What Are Dependencies?

Let's establish a clear definition before diving into management workflows.

**A dependency is an external library your project needs to function.** Think of dependencies as ingredients in a recipe—you don't make flour from scratch every time you bake bread. You buy it from specialists who do it better.

### Why Projects Need Dependencies

Modern software is built through composition, not from scratch:

- **HTTP requests?** Use `requests` (10+ years of development, millions of users)
- **Testing?** Use `pytest` (de facto Python standard)
- **Data processing?** Use `pandas` (battle-tested for analytics)
- **Web APIs?** Use `fastapi` (modern, async-friendly framework)

**Analogy: Recipe Ingredients**

Your Python project is like cooking a meal:

- **Main ingredients** (production dependencies): What you need to serve the dish (requests for HTTP, fastapi for web APIs)
- **Cooking tools** (development dependencies): What you need to prepare the dish (pytest for testing, ruff for linting) but don't serve to guests
- **Recipe card** (pyproject.toml): Lists all ingredients
- **Lockfile** (uv.lock): Records exact brands and versions you used

**Why not write everything yourself?**

- **Time**: Building an HTTP client takes months; installing `requests` takes seconds
- **Quality**: Popular libraries are debugged by thousands of projects
- **Security**: Maintained libraries get security patches
- **Focus**: Spend time on your unique value, not reinventing wheels

**Common Python dependencies you'll encounter**:

- **requests**: HTTP operations (API calls, web scraping)
- **pytest**: Testing framework (unit tests, integration tests)
- **fastapi**: Building web APIs (REST endpoints, async support)
- **pydantic**: Data validation (type checking, serialization)
- **numpy**: Numerical computing (arrays, linear algebra)
- **pandas**: Data analysis (spreadsheet-like operations)

## Adding Dependencies with AI: Production Packages

Let's start with adding a production dependency—a package your project needs at runtime.

### Scenario: Adding requests for HTTP Operations

You're building a project that needs to fetch data from web APIs. You need the `requests` library.

**Prompt to AI**:

```
Add the 'requests' library to my project as a production dependency.
Explain what this command does and what changes in my project.
```

**AI's Response** (approximate):

> I'll add `requests` as a production dependency using UV.
>
> **Command**:
>
> ```bash
> uv add requests
> ```
>
> **What this does**:
>
> 1. UV resolves `requests` and its dependencies (certifi, urllib3, charset-normalizer, idna)
> 2. Downloads all packages from PyPI
> 3. Installs them into your project's `.venv/` directory
> 4. Updates `pyproject.toml` to list `requests` in the dependencies section
> 5. Updates `uv.lock` with exact versions for reproducibility
>
> **Expected output**:
>
> ```
> Resolving dependencies... (0.5s)
> Downloaded 5 packages in 0.8s
> Installed 5 packages in 0.2s
>  + certifi==2024.7.4
>  + charset-normalizer==3.3.2
>  + idna==3.7
>  + requests==2.32.3
>  + urllib3==2.2.2
> ```
>
> **What changed**:
>
> - `pyproject.toml` now includes `requests>=2.32.3` in the `dependencies` list
> - `uv.lock` has been updated/created with exact versions
> - Five packages are now in your `.venv/lib/` directory

**Run the command**. Then verify the change:

**Verification Prompt**:

```
Show me the dependencies section of my pyproject.toml
```

**AI shows**:

```toml
[project]
dependencies = [
    "requests>=2.32.3",
]
```

**Key concept**: The `>=2.32.3` means "version 2.32.3 or any compatible newer version" (following semantic versioning—minor/patch updates are compatible, major updates aren't).

## Development vs. Production Dependencies: Why Separate Them?

Not all dependencies are equal. Some are needed at runtime (production), others only during development.

### Production Dependencies

**What they are**: Packages your application needs to run in production (deployed servers, user machines)

**Examples**:

- `requests`: Your app makes HTTP calls
- `fastapi`: Your app is a web API
- `pydantic`: Your app validates data

**When they're installed**: Always—production environments need these

### Development Dependencies

**What they are**: Packages developers need for building/testing but users never see

**Examples**:

- `pytest`: Running tests during development
- `pytest-cov`: Measuring test coverage
- `ruff`: Linting and formatting code
- `mypy`: Type checking
- `sphinx`: Building documentation

**When they're installed**: Development and CI/CD only—not in production deployments

### Why Separate Them?

**Lean Production Deployments**:

- Production containers/servers don't need testing tools
- Smaller deployments = faster startup, lower costs
- Fewer packages = smaller attack surface (security)

**Example**: A Docker container for your web API needs `fastapi` and `pydantic` (runtime), but not `pytest` and `ruff` (development tools). Installing dev dependencies in production wastes 100+ MB and adds unnecessary packages.

**Analogy**: You need kitchen tools (pots, knives) to cook, but you don't deliver the tools with the meal. Your customers get the food (production code + runtime dependencies), not your cookware (testing/linting tools).

## Adding Development Dependencies with AI

Let's add testing tools as development dependencies.

**Scenario**: You want to write tests for your project using pytest.

**Prompt to AI**:

```
Add pytest and pytest-cov as development dependencies to my project.
Explain the difference between dev and production dependencies.
```

**AI's Response** (approximate):

> I'll add `pytest` and `pytest-cov` as development dependencies using the `--dev` flag.
>
> **Command**:
>
> ```bash
> uv add --dev pytest pytest-cov
> ```
>
> **What `--dev` does**:
>
> - Marks these packages as development-only
> - They're listed in `[project.optional-dependencies]` section under `dev` group
> - Production installations (with `--no-dev`) skip these packages
>
> **Expected output**:
>
> ```
> Resolving dependencies... (0.3s)
> Downloaded 8 packages in 0.5s
> Installed 8 packages in 0.15s
>  + iniconfig==2.0.0
>  + packaging==24.1
>  + pluggy==1.5.0
>  + pytest==8.3.3
>  + pytest-cov==5.0.0
>  + coverage==7.6.1
>  + ...
> ```
>
> **Development vs. Production**:
>
> - **Production dependencies**: Needed when your app runs (requests, fastapi, pydantic)
> - **Development dependencies**: Needed when you build/test (pytest, ruff, mypy)
> - **Why separate**: Production deployments stay lean (faster, cheaper, more secure)

**Verify the change**:

**Prompt**:

```
Show me the optional-dependencies section of pyproject.toml
```

**AI shows**:

```toml
[project.optional-dependencies]
dev = [
    "pytest>=8.3.3",
    "pytest-cov>=5.0.0",
]
```

**Key concept**: Development dependencies live in `[project.optional-dependencies]` under the `dev` group. When you run `uv sync --no-dev` (production installation), these are skipped.

## Dependency Resolution Magic: How UV Finds Compatible Versions

When you add a dependency, UV doesn't just install the latest version. It performs **dependency resolution**—computing a set of package versions that all work together.

### The Resolution Problem

Dependencies have their own dependencies (called **transitive dependencies**). Version conflicts can arise:

**Example scenario**:

- You want `package-a` (depends on `shared-lib>=2.0,<3.0`)
- You want `package-b` (depends on `shared-lib>=1.5,<2.5`)
- Compatible range: `shared-lib>=2.0,<2.5` (satisfies both)

UV's resolver:

1. **Collects constraints**: Reads requirements from all packages
2. **Finds compatible range**: Computes versions that satisfy all constraints
3. **Selects versions**: Chooses latest versions within compatible ranges
4. **Locks versions**: Records exact versions in `uv.lock`

### Transitive Dependencies

**What they are**: Dependencies of your dependencies (packages you didn't explicitly request but are needed).

**Example**:

- You add `requests`
- `requests` depends on `certifi`, `urllib3`, `charset-normalizer`, `idna`
- UV automatically installs all five packages

**Why you see them**: Your project transitively depends on them (your code → requests → urllib3 → ...).

**Example dependency tree**:

```
your-project
├── requests (2.32.3)
│   ├── certifi (2024.7.4)
│   ├── charset-normalizer (3.3.2)
│   ├── idna (3.7)
│   └── urllib3 (2.2.2)
└── pytest (8.3.3)  [dev]
    ├── iniconfig (2.0.0)
    ├── packaging (24.1)
    └── pluggy (1.5.0)
```

**Key concept**: You specified 2 packages (`requests`, `pytest`), but UV installed 13 packages (including transitive dependencies). UV handles this complexity automatically.

## Updating Dependencies: Keeping Packages Current

Dependencies evolve—new versions fix bugs, add features, patch security issues. You'll need to update them periodically.

### Updating a Specific Package

**Scenario**: You want the latest version of `requests` with recent security fixes.

**Prompt to AI**:

```
Update the 'requests' package to the latest version.
Explain what changes and why updates matter.
```

**AI's Response** (approximate):

> I'll update `requests` to the latest compatible version.
>
> **Command**:
>
> ```bash
> uv add requests@latest
> ```
>
> (Or simply `uv add requests` again—UV resolves to latest compatible)
>
> **What happens**:
>
> 1. UV checks PyPI for the newest `requests` version
> 2. Resolves dependencies with new version
> 3. Updates `pyproject.toml` with new version constraint
> 4. Updates `uv.lock` with new exact version
> 5. Downloads and installs updated package
>
> **Expected output**:
>
> ```
> Resolving dependencies... (0.4s)
> Downloaded 1 package in 0.3s
> Installed 1 package in 0.1s
>  ~ requests==2.32.3 -> 2.33.0
> ```
>
> **Why update**:
>
> - **Security patches**: Fix vulnerabilities (critical for production)
> - **Bug fixes**: Resolve issues you might encounter
> - **New features**: Access improvements (if needed)
> - **Compatibility**: Keep pace with ecosystem changes
>
> **When to update**:
>
> - Regular schedule (monthly security updates)
> - After vulnerability announcements (check GitHub Security Advisories)
> - When you need new features
> - Before starting new development work

### Listing Outdated Dependencies

Before updating, see what's outdated.

**Prompt to AI**:

```
Show me which packages in my project have newer versions available
```

**AI's Response** (approximate):

> I'll check for outdated packages.
>
> **Command**:
>
> ```bash
> uv pip list --outdated
> ```
>
> **Expected output**:
>
> ```
> Package              Version    Latest    Type
> -------------------  ---------  --------  --------
> requests             2.32.3     2.33.0    production
> pytest               8.3.2      8.3.3     dev
> ```
>
> **What this shows**:
>
> - `requests` is one patch version behind (2.32.3 → 2.33.0)
> - `pytest` is one patch version behind (8.3.2 → 8.3.3)
>
> **Next steps**:
>
> - Review changelogs for breaking changes
> - Update individually (`uv add requests@latest`) or all at once
> - Test your project after updating

**Key concept**: Outdated doesn't mean broken. Update when you need new features, security patches, or are starting new work. Don't update mid-project without testing.

## Removing Dependencies: Cleaning Up Unused Packages

Projects evolve—sometimes you no longer need a dependency. Removing unused packages keeps your project lean and secure.

**Scenario**: You experimented with `beautifulsoup4` for HTML parsing but decided not to use it.

**Prompt to AI**:

```
Remove the 'beautifulsoup4' package from my project.
It's no longer needed.
```

**AI's Response** (approximate):

> I'll remove `beautifulsoup4` and any dependencies that were only needed by it.
>
> **Command**:
>
> ```bash
> uv remove beautifulsoup4
> ```
>
> **What happens**:
>
> 1. UV removes `beautifulsoup4` from `pyproject.toml`
> 2. UV checks if other packages still need its dependencies
> 3. Removes orphaned dependencies (packages no longer needed by anything)
> 4. Updates `uv.lock` to reflect changes
> 5. Uninstalls packages from `.venv/`
>
> **Expected output**:
>
> ```
> Removed beautifulsoup4
>  - beautifulsoup4==4.12.3
>  - soupsieve==2.5
> ```
>
> **Why remove unused packages**:
>
> - **Security**: Fewer packages = smaller attack surface
> - **Clarity**: Easy to see what your project actually needs
> - **Performance**: Slightly faster installs (fewer packages to process)
> - **Maintenance**: Fewer dependencies to update

**Verification**:

**Prompt**:

```
Show me my current dependencies list
```

**AI shows**: `beautifulsoup4` is no longer listed in `pyproject.toml`.

**Key concept**: Removing a package is safe—UV ensures dependencies shared by other packages aren't removed (only orphaned packages are cleaned up).

## Conflict Resolution: When Dependencies Clash

Sometimes two packages want incompatible versions of a shared dependency. UV detects conflicts and guides resolution.

### Understanding Conflicts

**Conflict scenario**:

- `package-a` requires `shared-lib>=3.0`
- `package-b` requires `shared-lib<3.0`
- **No compatible version exists** (3.0+ vs &lt;3.0)

### Example Conflict Resolution with AI

**Scenario**: You try adding two packages that have conflicting requirements.

**Error message** (hypothetical):

```
Error: Unable to resolve dependencies
  package-a (2.0.0) requires shared-lib>=3.0
  package-b (1.5.0) requires shared-lib<3.0
  No version of shared-lib satisfies both constraints
```

**Prompt to AI**:

```
I'm getting this dependency conflict error when trying to add package-a and package-b:
[paste error message]
Help me understand what's wrong and how to fix it.
```

**AI's Response** (approximate):

> **What's happening**:
> You have a dependency conflict. Two packages need incompatible versions of `shared-lib`:
>
> - `package-a` needs version 3.0 or newer
> - `package-b` needs version older than 3.0
> - No single version satisfies both
>
> **Resolution options**:
>
> 1. **Update `package-b`**: Check if newer version supports `shared-lib>=3.0`
>
>    ```bash
>    uv add package-b@latest
>    ```
>
> 2. **Use older `package-a`**: If `package-b` is critical, try older `package-a`
>
>    ```bash
>    uv add package-a@<2.0
>    ```
>
> 3. **Find alternatives**: One package might have a substitute
>
>    - Ask AI: "What's an alternative to package-b that works with shared-lib 3.0?"
>
> 4. **Wait for compatibility**: If both packages are essential, check GitHub issues—maintainers might be working on compatibility
>
> **How to investigate**:
>
> - Check package changelogs: `uv show package-b` (see version history)
> - Search GitHub issues: "package-b shared-lib 3.0 compatibility"
> - Ask AI: "Why does package-b require shared-lib&lt;3.0? Is there a workaround?"

**Real-world example**: This is rare with popular packages (maintainers coordinate), but can happen with:

- **Old unmaintained packages**: Haven't updated constraints
- **Beta/experimental packages**: API still evolving
- **Conflicting ecosystems**: Mixing packages from different communities

**Key concept**: Dependency conflicts are UV's way of preventing runtime errors. The resolver protects you from installing broken combinations. When conflicts arise, investigate with AI—usually one package needs updating.

## The Lockfile Concept: Reproducibility Through Version Pinning

You've seen `uv.lock` appear throughout this lesson. Let's understand what it does and why it matters.

### What is uv.lock?

`uv.lock` is a **lockfile**—a record of _exact versions_ of every package installed (including transitive dependencies).

**Purpose**: **Reproducible environments**. When you (or a teammate) run `uv sync`, UV installs the _exact_ versions recorded in `uv.lock`, guaranteeing identical setups.

### pyproject.toml vs. uv.lock

| File               | Purpose                 | Format            | Example            |
| ------------------ | ----------------------- | ----------------- | ------------------ |
| **pyproject.toml** | Constraints (ranges)    | Human-readable    | `requests>=2.32.3` |
| **uv.lock**        | Exact versions (pinned) | Machine-optimized | `requests==2.33.0` |

**Analogy**:

- **pyproject.toml**: "I need flour version 2.0 or compatible newer" (constraint)
- **uv.lock**: "I used flour version 2.3.1 from Brand X" (exact record)

### Why Lockfiles Matter

**Problem without lockfiles**: PyPI packages update constantly. If you install `requests>=2.32.3` today, you get 2.33.0. Your teammate installs tomorrow, gets 2.34.0 (a new patch version). Subtle differences cause "works on my machine" bugs.

**Solution with lockfiles**: `uv.lock` pins `requests==2.33.0`. Everyone gets 2.33.0 until you explicitly update the lockfile. Environments are identical.

### When UV Updates uv.lock

UV regenerates `uv.lock` automatically when you:

- Add a dependency (`uv add package`)
- Update a dependency (`uv add package@latest`)
- Remove a dependency (`uv remove package`)

You never edit `uv.lock` manually—it's generated by UV's resolver.

### Committing Lockfiles to Git

**Always commit both**:

- `pyproject.toml` (your dependency specifications)
- `uv.lock` (exact versions for reproducibility)

**Never commit**:

- `.venv/` (virtual environment directory—too large, machine-specific)

**Why commit uv.lock**: Teammates and CI/CD systems run `uv sync` to recreate your exact environment. Without `uv.lock`, they'd resolve dependencies fresh and potentially get different versions.

## Try With AI

Practice the complete dependency management workflow with your AI companion tool.

**Setup**: Ensure you have a UV project from Lesson 3 (or create a new one)

### Prompt 1: Add Production Dependency

```
Add the 'httpx' library to my project as a production dependency.
Explain what httpx is and why I might choose it over requests.
```

**What you're practicing**: Adding runtime dependencies with context

**Expected outcome**: AI adds `httpx`, explains it's an async-compatible HTTP client (alternative to requests)

**Validation**: Is `httpx` in `pyproject.toml` dependencies? Can you explain when to use httpx vs. requests?

### Prompt 2: Add Development Dependencies

```
Add pytest and pytest-cov as development dependencies to my project.
Explain how to run tests later.
```

**What you're practicing**: Separating dev tools from production packages

**Expected outcome**: AI adds pytest to `[project.optional-dependencies]` dev group

**Validation**: Are pytest packages in the dev section? Do you understand why they're separate from production dependencies?

### Prompt 3: Check for Updates

```
Show me which packages in my project have newer versions available.
For any outdated packages, explain whether I should update them.
```

**What you're practicing**: Maintenance awareness

**Expected outcome**: AI lists outdated packages with context (security updates vs. new features)

**Validation**: Do you understand the difference between patch/minor/major updates?

### Prompt 4: Update a Package

```
Update the [package-name] package to the latest version.
Show me what changed in pyproject.toml and uv.lock.
```

**What you're practicing**: Updating dependencies safely with AI guidance

**Expected outcome**: AI updates package, shows version change, explains what's new

**Validation**: Did `uv.lock` update? Can you see the version number change?

### Prompt 5: Troubleshoot a Conflict (Simulation)

```
If I tried to add two packages with conflicting dependencies,
how would I know? Show me what the error looks like and how to resolve it.
```

**What you're practicing**: Understanding conflict resolution strategies

**Expected outcome**: AI explains conflict detection, shows example error, lists resolution options

**Validation**: Can you explain why conflicts occur and what steps to take when encountering one?
