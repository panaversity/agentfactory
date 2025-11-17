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

generated_by: "lesson-writer v3.0.0"
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

## Try With AI

Use your AI companion for these exercises.

### Prompt 1: Generate Configuration (Tier 2 — AI Generates)

```
My team wants Ruff configuration that:
- Enforces import sorting (isort)
- Catches unused imports and variables (Pyflakes)
- Enforces PEP 8 style (pycodestyle)
- Allows 88-character lines
- Uses double quotes for strings
- Targets Python 3.13

Generate the [tool.ruff], [tool.ruff.lint], and [tool.ruff.format] sections for pyproject.toml.
```

**Expected outcome:** AI generates complete TOML configuration; you copy-paste it into your file.

### Prompt 2: Add Format-on-Save (Tier 2 — Zed Integration)

```
Show me the Zed settings.json configuration to:
1. Use Ruff as the formatter
2. Format on save automatically
3. Target Python files

I'm on [macOS/Windows/Linux].
```

**Expected outcome:** Settings snippet for Zed; you add it and test format-on-save works.

### Prompt 3: Troubleshoot Configuration (Tier 2 — AI Diagnoses)

```
I added this to my pyproject.toml:

[tool.ruff]
line-length = 88

[tool.ruff.lint]
select = ["E", "F"]

But when I run `uv run ruff check .`, I still get E501 (line too long) errors.
Why is Ruff complaining about line length when I set line-length = 88?
```

**Expected outcome:** AI explains: you selected E (pycodestyle) which includes E501; to ignore it, add `ignore = ["E501"]`.

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

