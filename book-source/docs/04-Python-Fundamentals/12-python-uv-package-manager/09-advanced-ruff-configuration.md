---
title: "Ruff Configuration with AI"
chapter: 12
lesson: 9
duration_minutes: 20

skills:
  - name: "Configure Ruff in pyproject.toml"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student adds `[tool.ruff]` section with custom settings and rules apply"

  - name: "Select and Ignore Ruff Rules"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student selects rule categories (E, F, B, I) or ignores specific rules"

  - name: "Integrate Ruff with Zed LSP"
    proficiency_level: "A2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student configures Zed to show Ruff diagnostics inline"

learning_objectives:
  - objective: "Understand why configuration matters: standardizing rules across your team"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student explains: config is version-controlled so teammates follow same rules"

  - objective: "Use AI to generate Ruff configuration without memorizing TOML syntax"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student asks AI for config, adds it to pyproject.toml, runs ruff check to verify"

  - objective: "Configure Zed to show Ruff errors in real-time while coding"
    proficiency_level: "A2"
    bloom_level: "Apply"
    assessment_method: "Student edits .zed/settings.json, sees Ruff diagnostics inline in editor"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts: [tool.ruff] section, line-length config, target-version, rule categories (E/F/B/I/N/D), select vs ignore, format-on-save, LSP integration. Within B1 limit. ✓"

differentiation:
  extension_for_advanced: "Explore all rule categories (D for docstrings, N for naming conventions); configure strict mode for learning"
  remedial_for_struggling: "Start with just line-length and one rule category (F for safety); add more rules gradually"

generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-chapter-12-lightning-python-stack/plan.md"
created: "2025-01-15"
last_modified: "2025-01-15"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Advanced Ruff Configuration

## Why Configure Ruff? (Tier 1: Motivation)

In Lesson 8, you used Ruff's **defaults**: format with double quotes, 88-character lines, standard PEP 8 rules.

But what if your team prefers:
- Single quotes instead of double quotes?
- 100-character lines instead of 88?
- Stricter rules (docstring checking, strict naming)?
- Ignoring certain rules (e.g., E501 line length)?

**Solution**: **Configure Ruff in `pyproject.toml`** so your team all follows the same rules.

**Key benefit**: Configuration is **version-controlled** (part of your git repo). Everyone on the team gets identical settings automatically when they clone the project.

---

## The TOML Format (Tier 1: Brief Introduction)

Before configuring Ruff, you need to understand **TOML** files (the format `pyproject.toml` uses).

TOML is simple:
- `key = value` for settings
- `[section.subsection]` for grouping
- Comments with `#`

**Example pyproject.toml structure:**
```toml
[project]
name = "my-project"
version = "0.1.0"

[tool.ruff]
line-length = 88
```

**Key insight**: You don't memorize TOML syntax. In **Tier 2**, you'll ask AI to generate configurations, and you'll just paste them in. The goal is understanding **what** to configure, not memorizing **how**.

---

## Basic Ruff Configuration (Tier 2: AI-Assisted)

Open your project's `pyproject.toml` file in Zed.

**Current structure (after `uv init`):**
```toml
[project]
name = "my-project"
version = "0.1.0"
requires-python = ">=3.13"

[dependency-groups]
dev = ["ruff>=0.14.0"]
```

Now you'll add a **`[tool.ruff]` section** to customize behavior.

**Tell your AI companion:**
```
I have a Python project using uv and Ruff. My team prefers:
- 88-character line length (Ruff default)
- Python 3.13
- Import sorting enabled
- Unused import detection

Update my pyproject.toml with a [tool.ruff] section that configures this.
```

**AI will generate something like:**
```toml
[tool.ruff]
line-length = 88
target-version = "py313"

[tool.ruff.lint]
select = ["E", "F", "I"]

[tool.ruff.format]
quote-style = "double"
```

**Copy-paste this into your `pyproject.toml`.**

**Verify it works:**
```bash
uv run ruff check .
```

Ruff now uses your custom configuration.

**Source**: Verified in intelligence/001-verified-tool-documentation.md

---

## Rule Categories Explained (Tier 1: Understand Options)

Ruff organizes rules into **categories** (single letters). You enable/disable whole categories:

| Category | Purpose | Examples |
|----------|---------|----------|
| **E** | **pycodestyle** — PEP 8 style | Spacing, indentation, blank lines |
| **F** | **Pyflakes** — Logic errors | Unused imports, undefined variables |
| **B** | **flake8-bugbear** — Likely bugs | Mutable defaults, unreachable code |
| **I** | **isort** — Import sorting | Alphabetize imports, group them |
| **N** | **pep8-naming** — Naming rules | Function names in lowercase |
| **D** | **pydocstyle** — Docstrings | Docstring formatting (advanced) |

**Example: Select import sorting + bug detection:**
```toml
[tool.ruff.lint]
select = ["F", "I", "B"]  # Pyflakes, isort, bugbear
```

**Example: Select all but ignore long lines:**
```toml
[tool.ruff.lint]
select = ["E", "F"]       # pycodestyle + Pyflakes
ignore = ["E501"]         # But ignore line-too-long
```

**Key insight**: Start simple (select F for safety), add more rules as your team agrees.

---

## Configure Line Length & Target Version (Tier 1: Common Settings)

Every project needs these two settings:

### **line-length**
How many characters wide should a line be before Ruff complains?

```toml
[tool.ruff]
line-length = 88  # Industry standard (Black-compatible)
```

**Common values:**
- `88`: Industry default (Black's choice)
- `100`: Slightly more lenient
- `120`: Very permissive
- `79`: Strict (PEP 8 original, rarely used anymore)

### **target-version**
Which Python version should Ruff assume?

```toml
[tool.ruff]
target-version = "py313"  # Python 3.13
```

**Why it matters**: Ruff adjusts rules based on Python version. Python 3.13 has features older versions don't.

**Common values:**
- `"py313"`: Python 3.13 (latest, what we use)
- `"py312"`: Python 3.12
- `"py311"`: Python 3.11

---

## Complete Configuration Example (Tier 2: Reference)

Here's a **professional pyproject.toml** with complete Ruff setup:

```toml
[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"

[project]
name = "my-project"
version = "0.1.0"
description = "Brief description"
authors = [{name = "Your Name", email = "you@example.com"}]
requires-python = ">=3.13"

[dependency-groups]
dev = ["ruff>=0.14.0", "pyright>=1.1.400"]

[tool.ruff]
line-length = 88
target-version = "py313"

[tool.ruff.lint]
select = ["E", "F", "B", "I"]  # Enable: style, safety, bugs, imports
ignore = ["E501"]              # But don't complain about line length

[tool.ruff.format]
quote-style = "double"         # Use "hello" not 'hello'
indent-style = "space"         # Use spaces, not tabs
```

**Save this to `pyproject.toml` and run:**
```bash
uv run ruff check .
```

Ruff now follows your configuration.

**Source**: Verified in intelligence/001-verified-tool-documentation.md

---

## Integrate with Zed: Format-on-Save (Tier 2: Editor Integration)

Wouldn't it be nice if **Zed automatically formatted your code when you save?**

Open Zed's settings:
- Mac: `Cmd+,` (comma)
- Windows/Linux: `Ctrl+,`

In the settings file, add this **Python-specific section**:

```json
{
  "languages": {
    "Python": {
      "formatter": "language_server",
      "format_on_save": true
    }
  }
}
```

**Now when you save a Python file** (Cmd+S or Ctrl+S), Ruff automatically formats it.

**Benefit**: You write code freely; Ruff keeps it clean. No manual `uv run ruff format .` needed.

**Source**: Verified in intelligence/001-verified-tool-documentation.md

---

## Ignoring Rules Selectively (Tier 2: Fine-Tuning)

Sometimes a specific rule doesn't fit your project. You can **ignore it for a single line or the whole file.**

**Ignore one rule on a specific line:**
```python
x = 1  # noqa: F841  <- Line not used; ignore F841 error
```

**Ignore all rules for a line:**
```python
import everything_we_need  # noqa
```

**Ignore a rule project-wide:**
```toml
[tool.ruff.lint]
ignore = ["E501", "D100"]  # Ignore: line-too-long, missing module docstring
```

**When to use:**
- One-off exceptions: `# noqa` on the line
- Team decision: Add to `ignore` list in `pyproject.toml`

---

## Tier 3 Preview: Scaling Configuration (Tier 2: Awareness)

As projects grow, you might want:
- **Shared configuration**: One `pyproject.toml` template for all team projects
- **CI/CD integration**: Ruff checks run automatically on every git push
- **Pre-commit hooks**: Ruff runs before allowing commits

These are **Lesson 12 topics**. For now, just know: your `pyproject.toml` is the single source of truth for your team's code standards.

---

## Try With AI: The Quality Toolchain Integration Challenge

### Part 1: Analyze Your Codebase (Your Turn First)

**Before asking AI**, examine your existing code to identify patterns worth enforcing:

1. Open an existing Python project (or create test files with various styles)
2. Look for inconsistencies:
   - Mixed quote styles (single vs double)
   - Different line lengths
   - Unsorted imports
   - Missing or excessive blank lines

**Your pattern-recognition task**: Document what you find:
- Which inconsistencies would confuse a new teammate?
- Which patterns are team preferences vs objective errors?
- What would you want enforced automatically vs left flexible?

---

### Part 2: AI Explains Configuration Strategy (Discovery)

Share your findings with AI:

> "I analyzed my Python codebase and found:
>
> - Some files use 88-char lines, others use 120
> - Imports are not sorted consistently
> - Mix of single and double quotes
> - Some files have 2 blank lines before functions, others have 1
>
> Questions:
> 1. Which of these should I enforce via Ruff config vs leave to developer choice?
> 2. What's the difference between [tool.ruff], [tool.ruff.lint], and [tool.ruff.format] sections?
> 3. How do I choose which rule categories (E, F, B, I, N, D) to enable?
> 4. Can I have different rules for tests vs production code?"

**Your evaluation task**:
- Can you explain why `line-length = 88` goes in `[tool.ruff]` but `select = ["E", "F"]` goes in `[tool.ruff.lint]`?
- What's the relationship between enabling category "E" and then ignoring specific rules like "E501"?

---

### Part 3: Student Teaches AI (Tool Conflicts)

Challenge AI with realistic multi-tool scenarios:

> "I'm using Ruff, Pyright, and Black together. Here are conflicts I'm seeing:
>
> **Conflict A**: Ruff and Black disagree on line breaking. Ruff formats one way, Black reformats differently. How do I make them compatible?
>
> **Conflict B**: Pyright says I have type errors but Ruff doesn't catch them. Are they checking different things? Should both be enabled?
>
> **Conflict C**: My Zed settings have format-on-save enabled, but when I save, sometimes nothing formats. Other times it formats twice. What's the LSP coordination issue?
>
> For EACH:
> 1. Show me the pyproject.toml config that prevents the conflict
> 2. Explain which tool should take precedence and why
> 3. Show me how to verify they're working together (test command)
> 4. What's the proper tool execution order: format → lint → type-check?"

**Your debugging task**:
- Install Black alongside Ruff: `uv add black --dev`
- Run both formatters on the same file
- Compare outputs—do they produce identical results?
- Use AI's config to make Ruff Black-compatible

---

### Part 4: Build Comprehensive Quality Configuration (Convergence)

Create a complete quality toolchain config with AI:

> "Generate a production-ready pyproject.toml configuration integrating:
>
> **Ruff settings**:
> - Line length: 88 characters
> - Python target: 3.13
> - Enable: style (E), safety (F), bugs (B), import sorting (I)
> - Ignore: E501 (line too long, since we set explicit length)
> - Format: double quotes, space indentation
>
> **Pyright settings** (preview for Lesson 10):
> - Type checking mode: basic
> - Exclude: .venv, __pycache__
>
> **Tool coordination**:
> - Ensure Ruff and Pyright don't conflict
> - Configure for Zed LSP integration
> - Add comments explaining each section
>
> For EACH setting:
> - The TOML structure
> - Why this value (industry standard, team preference, or safety)
> - How to verify it works (command to test)"

**Refinement**:
> "Now generate the matching Zed settings.json that:
> 1. Uses Ruff for formatting (format-on-save enabled)
> 2. Shows Ruff diagnostics inline
> 3. Shows Pyright type errors inline
> 4. Excludes .venv from file watching
> 5. Sets Python-specific tab width to 4 spaces
>
> Include both user-level settings (apply to all projects) and workspace settings (this project only)."

---

**Time**: 30-35 minutes
**Outcome**: You'll have a professional-grade quality toolchain with Ruff + Pyright integration, proper configuration hierarchy, and IDE integration that catches errors in real-time while maintaining team code standards.

---

## Red Flags to Watch

**Problem**: "Ruff says my pyproject.toml has a syntax error"
- **What it means**: TOML formatting mistake (missing =, wrong quotes, wrong brackets)
- **What to do**: Ask AI: "Check my pyproject.toml for syntax errors and fix them"
- **Pro tip**: TOML is finicky; let AI handle it

**Problem**: "Format-on-save isn't working in Zed"
- **What it means**: Ruff LSP not connected, or Zed settings wrong
- **What to do**: Check `.zed/settings.json` has correct Python formatter config; verify Ruff installed (`uv add ruff --dev`)
- **Fallback**: Run `uv run ruff format .` manually (format-on-save is convenience, not required)

**Problem**: "My rules look right but Ruff still complains"
- **What it means**: Ruff might be using a cached configuration; or you didn't save the file
- **What to do**: Save `pyproject.toml`, restart Zed (close and reopen), try again
- **Nuclear option**: Delete `.venv` folder and run `uv sync` to rebuild everything

