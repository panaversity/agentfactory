---
title: "Managing Dependencies with AI"
chapter: 12
lesson: 4
duration_minutes: 35

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Manage Project Dependencies"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can add production and development dependencies by expressing needs to Claude Code or Gemini CLI, understand dependency groups (dev vs. production), and verify installations in pyproject.toml"

  - name: "Understand Dependency Resolution"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain how UV resolves version conflicts, interpret dependency tree output, and ask AI to resolve incompatibilities when they occur"

  - name: "Apply Semantic Versioning"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain major.minor.patch versioning system and recognize what version constraints mean (e.g., ^1.0.0 vs. ==1.2.3)"

  - name: "Distinguish Development vs. Production Dependencies"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain why testing tools are marked as dev-only and recognize which packages belong in which category"

learning_objectives:
  - objective: "Add production dependencies to a UV project by expressing intent to AI"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student successfully adds packages (requests, Flask) using `uv add` via Claude Code or Gemini CLI"

  - objective: "Add development dependencies with the --dev flag and explain the distinction"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student uses `uv add --dev pytest` and explains why testing tools aren't needed in production"

  - objective: "Understand semantic versioning (major.minor.patch) and version constraints"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Student interprets version strings and explains what they mean (e.g., ^1.0.0 allows 1.x.x but not 2.0.0)"

  - objective: "Resolve dependency conflicts using AI-assisted troubleshooting"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Given a conflict error, student works with AI to diagnose and propose a solution"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts: What is a dependency, production vs. development dependencies, dependency resolution, version constraints, semantic versioning (major.minor.patch), uv.lock file purpose, transitive dependencies. Within B1 limit of 10 concepts. ✓"

differentiation:
  extension_for_advanced: "Explore dependency graph with `uv tree` command; optimize production deployments by removing dev dependencies; analyze version conflict resolution strategies"
  remedial_for_struggling: "Focus on core pattern: 'Use Claude Code to add packages. Production = runtime. Development = testing/tools. That's it.' Don't get lost in version constraint syntax—let AI handle it."

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/011-python-uv/plan.md"
created: "2025-11-13"
last_modified: "2025-11-13"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Managing Dependencies with AI

## What Are Dependencies? (And Why You Need Them)

Before we manage dependencies, let's be clear about what they are.

A **dependency** is a piece of code someone else wrote that you want to use in your project. You're not building everything from scratch—you're reusing code that others have published.

Here are some real examples:

- **requests** — Makes it easy to fetch data from the internet (weather APIs, news feeds, etc.)
- **Flask** — Builds web applications (the framework that handles HTTP requests)
- **pytest** — Tests your code (makes sure it works as intended)
- **black** — Automatically formats your code to look clean and consistent
- **pandas** — Works with data (spreadsheets, CSV files, databases)

**Why use dependencies instead of writing everything yourself?**

1. **Speed** — Don't spend 3 days writing HTTP code when `requests` does it in seconds
2. **Quality** — Published libraries are tested by thousands of developers
3. **Maintenance** — When a library is updated, you get bug fixes automatically
4. **Collaboration** — Your team knows these libraries; shared knowledge = faster development

Without dependencies, you'd be reinventing wheels constantly. With them, you focus on solving your unique problem.

#### 💬 AI Colearning Prompt

> "Why would a Python developer choose to use the `requests` library instead of building their own HTTP code? What are the tradeoffs?"

This question helps you understand that dependencies are a **choice**, not a requirement. Sometimes you build from scratch; usually, you reuse existing code.

## The Dependency Lifecycle: Add, Update, Remove

Managing dependencies follows a simple three-step pattern:

1. **Adding** — "I need this library for my project"
2. **Updating** — "There's a newer version; should I upgrade?"
3. **Removing** — "I'm not using this package anymore; clean it up"

Let's walk through each scenario.

## Scenario 1: Adding Production Dependencies

**Situation:** You're building a web client and need to fetch data from an API.

Adding a package is straightforward:

```bash
# Add requests as a production dependency
uv add requests
```

**Output:**
```
Resolved 5 packages in 0.15s
Added requests==2.31.0
```

**What happened:**
1. UV downloaded `requests` version 2.31.0 and its dependencies (`urllib3`, `charset-normalizer`, etc.)
2. UV updated your `pyproject.toml` to list `requests==2.31.0`
3. UV created/updated `uv.lock` to pin exact versions for reproducibility
4. UV installed everything into your virtual environment

Now you can import and use `requests` in your Python code.


## Understanding Transitive Dependencies

Notice something interesting in the example above: **UV installed 5 packages, but you only asked for 1.**

Why?

Because `requests` depends on other libraries to work:
- `requests` needs `urllib3` (for HTTP connections)
- `requests` needs `charset-normalizer` (for handling different text encodings)
- And those libraries have their own dependencies

**Transitive dependencies** are "dependencies of dependencies." You don't specify them; UV finds them automatically.

**Visual example:**

```
Your Project
└── requests
    ├── urllib3
    ├── charset-normalizer
    │   └── chardet
    ├── idna
    └── certifi
```

When you add `requests`, you get the entire tree—automatically.

**Why does this matter?**

1. **You don't have to specify transitive dependencies** — UV handles it
2. **Version conflicts can happen** — If two libraries need different versions of the same dependency, UV detects it
3. **Updates can be complex** — When you update `requests`, its dependencies might update too

This is why **dependency resolution** (which we'll cover later) is powerful.

#### 💬 AI Colearning Prompt

> "Show me the dependency tree for the `requests` library. What are all the packages that get installed when I add requests? Explain why each one is needed."

This helps you understand that adding one package can bring in many others—and that's okay.

## Scenario 2: Adding Development Dependencies

**Situation:** You're writing code and want to test it with `pytest` (a testing tool).

**The key question:** Should testing tools live in your main project dependencies, or somewhere else?

**Answer:** Development-only. Here's why:

- **Production** is where your code runs for users (a web server, an API, a desktop app)
- **Development** is where you write and test code (your laptop, your CI/CD system)

A user running your web app doesn't need `pytest`. **Testing tools are for developers, not users.**

```bash
# Add pytest and pytest-cov as development dependencies
uv add --dev pytest pytest-cov
```

**Output:**
```
Resolved 8 packages in 0.18s
Added pytest==7.4.3
Added pytest-cov==4.1.0
```

**What's different:**

**Production Dependencies** (in `pyproject.toml`):
```toml
dependencies = ["requests==2.31.0"]
```

**Development Dependencies** (in `pyproject.toml`):
```toml
[tool.uv]
dev-dependencies = ["pytest==7.4.3", "pytest-cov==4.1.0"]
```

When you deploy to production, you install only production dependencies. When developing locally, you get both.

## Scenario 3: Updating a Package

**Situation:** You've been using `requests==2.31.0` for a month. Now version 2.32.0 is available with bug fixes and new features.

```bash
# Update a specific package to latest
uv add requests@latest

# Or update everything
uv sync --upgrade
```

Your `uv.lock` file updates with the new exact version. Your teammates should run `uv sync` to get the same version.

**When to update:**
- ✅ **Do update:** Security fixes, bug fixes, new features you need
- ⚠️ **Be careful:** Major version changes (1.x.x to 2.0.0) can break your code
- ❌ **Don't update:** If it's not broken and you don't need the new features

## Scenario 4: Removing a Package

**Situation:** You added `pandas` for data analysis, but ended up solving the problem differently.

```bash
# Remove a package
uv remove pandas
```

UV removes `pandas` and any dependencies that were only needed by `pandas`. Your `pyproject.toml` no longer lists it.

**Simple rule:** If you're not using it, remove it.

## Semantic Versioning: Understanding Version Numbers

You've seen version numbers like `2.31.0`. What do they mean?

**Semantic versioning** uses three numbers: `MAJOR.MINOR.PATCH`

| Part | Meaning | Example |
|------|---------|---------|
| **MAJOR** | Big breaking changes | `1.0.0` → `2.0.0` (old code might break) |
| **MINOR** | New features, backward-compatible | `1.0.0` → `1.1.0` (old code still works) |
| **PATCH** | Bug fixes, backward-compatible | `1.1.0` → `1.1.1` (security fix) |

**In practice:**

- `requests==2.31.0` — Use **exactly** version 2.31.0
- `requests^2.31.0` — Use 2.31.0 or newer, but not 3.0.0 (minor/patch updates okay; major updates risky)
- `requests>=2.31.0` — Use 2.31.0 or any newer version (most flexible; riskiest)

**Why different constraints?**

Imagine you're building a library others will use. You publish it with `requests^2.31.0`. When users install your library:

- If `requests` version `2.32.0` is available, they get it (safe: minor version)
- If `requests` version `3.0.0` is available, they won't get it (risky: major version could break compatibility)

**Here's the thing:** You don't need to memorize these rules. **AI handles version constraints for you.**

Your job: When you see a version string, ask AI what it means.

#### 💬 AI Colearning Prompt

> "If I specify `requests^2.31.0`, which versions would be acceptable? What about `>=2.31.0`? Why would I choose one over the other?"

This helps you understand the philosophy behind version constraints without memorizing syntax.

## Dependency Resolution: What Happens When UV Adds a Package

When you add a package, UV does something complex but invisible to you: **dependency resolution.**

Here's what's happening under the hood:

1. **Fetch metadata** — UV checks PyPI for the package and its dependencies
2. **Build version tree** — UV maps all transitive dependencies
3. **Resolve conflicts** — If two packages need different versions of the same library, UV finds compatible versions
4. **Lock versions** — UV pins exact versions in `uv.lock` for reproducibility
5. **Install** — UV downloads and installs everything into your virtual environment

**Example of conflict resolution:**

Imagine:
- Your project needs `requests>=2.30.0`
- You add `httpx`, which needs `requests<3.0.0`

These constraints are compatible (both need 2.x versions). UV picks a version that satisfies both—say, `requests==2.32.0`.

**What if they're incompatible?**

UV tells you there's a conflict and suggests solutions. This is where you (with AI help) decide: update one package, accept an older version, or find an alternative.

#### 💬 AI Colearning Prompt

> "Create a scenario where two packages have conflicting dependency requirements. Show me what UV's error message would look like and how to fix it."

This helps you prepare for real-world conflicts without panic.

## The uv.lock File: Guaranteeing Reproducibility

You've heard about `uv.lock`, but what does it actually do?

**Situation:** You create a project with `requests==2.31.0`. Your teammate clones your project a month later. By then, the latest version of `requests` is `2.33.0`.

**Without a lockfile:** Your teammate might accidentally get `2.33.0` instead of `2.31.0`. Your environments don't match. Code works on your laptop, breaks on theirs. Debugging nightmare.

**With a lockfile:** The `uv.lock` file pins every single version, including transitive dependencies. Your teammate runs `uv sync` and gets your exact environment.

**In your `uv.lock` file:**

```toml
[[package]]
name = "requests"
version = "2.31.0"
source = { type = "registry", url = "https://pypi.org/simple", editable = false }
dependencies = [
    { name = "charset-normalizer", version = ">=2" },
    { name = "idna", version = ">=2.5" },
    { name = "urllib3", version = "!=1.25.0,!=1.25.1,<2.0,>=1.21.1" },
    { name = "certifi", version = ">=2017.4.17" },
]
```

**Key insight:** This file is **autogenerated**. You never edit it manually. It's version controlled (committed to Git) so your team stays in sync.

**When does uv.lock update?**
- When you run `uv add package-name` (adds new exact version)
- When you run `uv update package-name` (updates to new version)
- When you run `uv sync --upgrade` (updates all outdated packages)

## Handling Dependency Conflicts

**Scenario:** You're adding two packages that have incompatible dependency requirements.

When you encounter a conflict like: `"scikit-learn==1.0.0 requires numpy>=1.14.6, which is incompatible with numpy==1.20.0"`, you have options:

**Option 1 (Recommended): Let UV choose compatible versions**
```bash
uv add scikit-learn numpy
```

**Option 2: Accept the library's version requirement**
```bash
uv add scikit-learn==1.0.0 "numpy>=1.14.6"
```

**Option 3: Use an older version**
```bash
uv add scikit-learn==0.24.0 numpy==1.20.0
```

**When to use AI:** For complex conflicts, ask your AI tool to analyze the error and suggest solutions with tradeoffs explained.

## Checking What's Installed

To view your project's dependencies:

```bash
# View your project's dependencies
uv tree
```

**Output shows:**
```
my-app
├── requests==2.31.0
│   ├── charset-normalizer==3.3.2
│   ├── idna==3.6
│   ├── urllib3==2.1.0
│   └── certifi==2023.7.22
└── Flask==3.0.0
    ├── click==8.1.7
    ├── itsdangerous==2.1.2
    └── Jinja2==3.1.2
```

This shows your **dependency tree**—what you directly added (top level) and what those packages need (nested).

To check for outdated packages:

```bash
uv pip list --outdated
```

## Real-World Workflow: Building a Complete Project

**Scenario:** You're building a weather app that needs `requests` for API calls, `pytest` for testing, and `black` for formatting.

**Step 1: Create the project** (you did this in Lesson 3)
```bash
uv init weather-app
cd weather-app
```

**Step 2: Add dependencies**
```bash
# Production dependency
uv add requests

# Development dependencies
uv add --dev pytest black
```

**Step 3: Your pyproject.toml now looks like:**

```toml
[project]
name = "weather-app"
requires-python = ">=3.13"
dependencies = ["requests==2.31.0"]

[tool.uv]
dev-dependencies = ["pytest==7.4.3", "black==23.12.0"]
```

Clean, reproducible, ready to share with your team.

## Summary: The Dependency Management Pattern

Every time you manage dependencies, follow this mental model:

1. **Identify your need** — "I need to test my code" or "I need to fetch data from the internet"
2. **Tell AI your intent** — "Add pytest" or "Add requests"
3. **Let AI handle execution** — Version resolution, lockfile updates, dependency installation
4. **Understand the result** — Ask AI to explain what changed and why
5. **Keep production lean** — Production = runtime only; development = testing/tooling

**You don't memorize commands. You articulate intent. AI does the rest.**

## Try With AI: Dependency Conflict Resolution Challenge

### Part 1: Design Dependency Structure (Your Turn First)

**Before asking AI**, plan your weather app dependencies:

**Your app needs**:
1. Fetch weather data from an API
2. Run automated tests
3. Format code consistently
4. Display data in a web interface

**Your categorization task**:
- For EACH need, choose a library (requests? Flask? pytest? black?)
- Decide: Production dependency or development-only? Write WHY.
- Predict: Which libraries will bring in transitive dependencies?

---

### Part 2: AI Reveals the Dependency Tree (Discovery)

Now ask AI to show the full picture:

> "I'm building a weather app. I need:
> 1. `requests` for API calls
> 2. `Flask` for web interface
> 3. `pytest` for testing
> 4. `black` for formatting
>
> For EACH: Show me the complete dependency tree (transitive dependencies included). Which should be production vs. dev?"

**Your evaluation task**:
- Compare AI's categorization to yours. Did you get it right?
- Count total packages installed. Does the number surprise you?
- Can you now explain to someone else why `requests` needs `urllib3`?

---

### Part 3: Student Teaches AI (Create and Resolve Conflicts)

Challenge AI with a realistic conflict scenario:

> "I want to add BOTH:
> - `scikit-learn==1.0.0` (for machine learning)
> - `numpy==1.20.0` (for numerical computing)
>
> But they have a version conflict. Show me:
> 1. The exact error UV would produce
> 2. WHY the conflict happens (examine version requirements)
> 3. THREE different solutions with tradeoffs for each
>
> Then I'll challenge your recommendation: What if I MUST use numpy==1.20.0 for compatibility with another tool? What's the solution now?"

**Your decision task**:
- Evaluate AI's three solutions. Which would YOU choose? Why?
- Can you identify a scenario where AI's "best" solution wouldn't work?

---

### Part 4: Build a Real Dependency Strategy (Convergence)

Create a complete dependency management workflow with AI:

> "Let's design a dependency strategy for MY weather app:
>
> **Requirements**:
> 1. Add production dependencies with appropriate version constraints
> 2. Add dev dependencies in a separate group
> 3. Show me the resulting `pyproject.toml` structure
> 4. Create commands to sync environment on a teammate's machine
> 5. Add a policy: When should we update dependencies? When should we NOT?
>
> Walk me through each command and explain what changes in the project files."

**Refinement if needed**:
> "This strategy doesn't handle security updates to dependencies. Add a process for monitoring and applying security patches without breaking compatibility."

---

**Time**: 30-35 minutes total
**Outcome**: You've designed a production-ready dependency management workflow, debugged conflicts with AI assistance, and built mental models for categorizing production vs. development dependencies.

