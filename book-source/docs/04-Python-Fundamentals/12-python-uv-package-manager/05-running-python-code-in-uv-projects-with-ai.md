---
title: "Running Python Code in UV Projects with AI"
chapter: 12
lesson: 5
sidebar_position: 5
duration_minutes: 60

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Execute Code in UV Environment"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can run scripts and tests in UV projects by expressing intent ('Run this Python script using UV'), understand environment isolation, and troubleshoot ModuleNotFoundError by verifying dependencies with AI"

  - name: "Understand Environment Isolation"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain why UV creates isolated environments (.venv/), describe how isolation prevents conflicts between projects, and compare uv run vs. regular python execution with examples"

learning_objectives:
  - objective: "Apply UV execution workflows (uv run) to scripts and tests in isolated environments"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Successfully runs scripts, pytest tests, and one-off commands using AI guidance"

  - objective: "Understand environment isolation, automatic activation, and ModuleNotFoundError troubleshooting"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Can explain how UV isolates dependencies and diagnose missing package errors with AI"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (environment isolation, uv run, automatic activation, script execution, test execution, ModuleNotFoundError, one-off commands) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore uv run --extra for optional dependency groups; debug pytest test failures with --pdb; analyze .venv/ directory structure; compare venv behavior to conda/virtualenv"
  remedial_for_struggling: "Focus on running single script first; use AI to verify each step; build confidence with successful executions before troubleshooting errors"

# Generation metadata
generated_by: "lesson-writer v1.0"
source_spec: "specs/011-python-uv/spec.md"
created: "2025-11-04"
last_modified: "2025-11-04"
git_author: "GitHub Copilot"
workflow: "/sp.implement"
version: "1.0.0"
---

# Running Python Code in UV Projects with AI

You've installed UV, created projects, and managed dependencies. Now comes the crucial step: **running your Python code** in the isolated environment you've built.

This lesson answers the question: "How do I actually execute my scripts and tests in a UV project?" You'll learn why `python script.py` isn't enough, what `uv run` does, and how UV automatically activates the correct environment with the right dependencies.

By the end of this lesson, you'll confidently run scripts, execute pytest tests, debug "ModuleNotFoundError" issues, and understand why environment isolation matters. You'll use AI to handle execution commands while focusing on understanding what's happening behind the scenes.

## Why Environment Isolation Matters

Before we run code, let's understand the problem UV solves.

### The Multi-Project Problem

Imagine you have three Python projects on your computer:

**Project A**:
- Uses `requests==2.28.0` (older version)
- Built two years ago, still works

**Project B**:
- Uses `requests==2.33.0` (latest version)
- New project, needs current features

**Project C**:
- Uses `httpx` (alternative to requests)
- Different HTTP library entirely

**Without isolation**: If you install packages globally (system-wide), all three projects share the same packages. Updating `requests` for Project B breaks Project A. Installing `httpx` for Project C doesn't affect A or B, but your global Python environment becomes cluttered with dozens of packages from different projects.

**With isolation (UV's approach)**: Each project has its own `.venv/` directory containing dependencies specific to that project. Project A has `requests==2.28.0`, Project B has `requests==2.33.0`, Project C has `httpx`. They coexist peacefully.

### Analogy: Kitchen Toolboxes

Think of each project as a recipe:
- **Global installation**: One shared kitchen drawer with all utensils mixed together (chaos when multiple recipes need different tools)
- **Isolated environments**: Each recipe has its own toolbox with exactly the tools needed (organized, no conflicts)

**Why this matters for you**:
- Work on multiple projects without version conflicts
- Safely experiment with packages (changes stay isolated)
- Delete a project's environment without affecting others
- Reproduce environments exactly (same packages, same versions)

## UV Auto-Activation: The Magic Behind `uv run`

Traditional Python workflows require **manual activation**:

```bash
# Traditional workflow (what you DON'T need to do with UV)
cd my-project
source .venv/bin/activate      # Activate environment (Linux/Mac)
.venv\Scripts\activate.ps1     # Activate environment (Windows)
python script.py               # Now uses isolated environment
deactivate                     # Deactivate when done
```

**UV's workflow** (automatic):

```bash
# UV workflow (what you DO with UV)
cd my-project
uv run script.py              # UV automatically uses .venv/
```

**What `uv run` does automatically**:
1. **Detects project root**: Finds `pyproject.toml` to identify your project
2. **Locates environment**: Finds `.venv/` directory with dependencies
3. **Activates temporarily**: Uses packages from `.venv/` for this command only
4. **Executes command**: Runs `python script.py` (or `pytest`, or whatever you specified)
5. **Deactivates automatically**: Command finishes, environment deactivation happens automatically

**Key benefit**: You never manually activate/deactivate. UV handles it per command.

## Running Python Scripts with AI

Let's run a real script using UV and AI.

### Scenario: Running a Simple API Client

You've created a script `src/fetch_quote.py` that fetches a random quote from an API using the `requests` library:

```python
# src/fetch_quote.py
import requests

def get_random_quote():
    url = "https://api.quotable.io/random"
    response = requests.get(url)
    data = response.json()
    return f'"{data["content"]}" — {data["author"]}'

if __name__ == "__main__":
    quote = get_random_quote()
    print(quote)
```

**Prompt to AI**:
```
Run the script src/fetch_quote.py in my UV project. 
Explain how UV ensures the script uses the correct dependencies.
```

**AI's Response** (approximate):

> I'll run your script using UV's automatic environment activation.
>
> **Command**:
> ```bash
> uv run src/fetch_quote.py
> ```
>
> **What UV does**:
> 1. Finds `pyproject.toml` in your project root
> 2. Locates `.venv/` with your installed packages (including `requests`)
> 3. Temporarily activates the environment
> 4. Runs `python src/fetch_quote.py` using `.venv/bin/python`
> 5. Your script imports `requests` from `.venv/lib/` (not system Python)
>
> **Expected output** (example):
> ```
> "The only way to do great work is to love what you do." — Steve Jobs
> ```
>
> **Why this works**:
> - Your project has `requests` in dependencies (added in Lesson 3-4)
> - UV installed `requests` into `.venv/` when you added it
> - `uv run` ensures Python finds `requests` in `.venv/`, not globally
>
> **Compare to regular python**:
> - If you ran `python src/fetch_quote.py` (without `uv run`), Python would use the global environment
> - If `requests` isn't installed globally: `ModuleNotFoundError: No module named 'requests'`
> - If a different `requests` version is installed globally: Potential compatibility issues

**Try it**: Run the command. See the quote appear. This is environment isolation in action.

### Cross-Platform Note

`uv run` works identically on Windows, macOS, and Linux. The underlying activation mechanism differs by platform, but UV abstracts this away. You use the same command everywhere.

## Running Tests with UV: Executing Pytest

Testing is development work (using dev dependencies). Let's run tests in the isolated environment.

### Scenario: Running a Test Suite

You've added `pytest` as a dev dependency (Lesson 4). Your test file `tests/test_fetch_quote.py`:

```python
# tests/test_fetch_quote.py
import pytest
from src.fetch_quote import get_random_quote

def test_get_random_quote():
    """Test that get_random_quote returns a non-empty string."""
    quote = get_random_quote()
    assert isinstance(quote, str)
    assert len(quote) > 0
    assert '—' in quote  # Quote format includes em dash separator
```

**Prompt to AI**:
```
Run my pytest tests using UV. 
Explain how UV handles development dependencies during test execution.
```

**AI's Response** (approximate):

> I'll run pytest using UV, which will use the dev dependencies installed in your `.venv/`.
>
> **Command**:
> ```bash
> uv run pytest
> ```
> (Or `uv run pytest tests/` to specify directory)
>
> **What UV does**:
> 1. Activates `.venv/` automatically (same as for scripts)
> 2. Runs `pytest` from `.venv/bin/pytest` (or `.venv\Scripts\pytest.exe` on Windows)
> 3. Pytest discovers tests (files matching `test_*.py` or `*_test.py`)
> 4. Executes tests using Python and dependencies from `.venv/`
> 5. Reports results
>
> **Expected output** (example):
> ```
> ========================= test session starts =========================
> platform win32 -- Python 3.13.0, pytest-8.3.3, pluggy-1.5.0
> rootdir: C:\Users\you\projects\my-project
> collected 1 item
>
> tests\test_fetch_quote.py .                                     [100%]
>
> ========================== 1 passed in 0.52s ==========================
> ```
>
> **Development dependencies in action**:
> - `pytest` is installed in `.venv/` because you added it with `--dev` (Lesson 4)
> - UV includes dev dependencies when running tests locally
> - Production deployments (with `uv sync --no-dev`) wouldn't include pytest (keeps production lean)
>
> **Why this matters**:
> - Tests run with exact dependency versions from `uv.lock`
> - Same test results for you and teammates (reproducible environments)
> - No global pytest installation needed (project-specific tooling)

**Run it**: Execute the command. See tests pass (green dots). This is automated testing in an isolated environment.

### Running Specific Tests

You can run individual test files or functions:

**Prompt to AI**:
```
Run only the test_get_random_quote function from tests/test_fetch_quote.py
```

**AI's Response**:
```bash
uv run pytest tests/test_fetch_quote.py::test_get_random_quote
```

**What this does**: Pytest runs only the specified test function (useful for debugging specific failures).

## One-Off Commands: Checking Installed Packages

Sometimes you need to verify what's installed in your environment without writing a script.

**Prompt to AI**:
```
Show me all packages installed in my UV project's environment, 
including their versions.
```

**AI's Response** (approximate):

> I'll list installed packages using `pip list` (UV supports standard pip commands within the environment).
>
> **Command**:
> ```bash
> uv pip list
> ```
>
> **Expected output** (example):
> ```
> Package              Version
> -------------------- -------
> certifi              2024.7.4
> charset-normalizer   3.3.2
> idna                 3.7
> iniconfig            2.0.0
> packaging            24.1
> pluggy               1.5.0
> pytest               8.3.3
> pytest-cov           5.0.0
> requests             2.33.0
> urllib3              2.2.2
> ```
>
> **What this shows**:
> - All packages in `.venv/` (production and dev)
> - Exact versions currently installed
> - Includes transitive dependencies (packages you didn't explicitly add)
>
> **Use cases**:
> - Verify a package is installed: "Is `requests` version 2.33.0 present?"
> - Debug import errors: "Did UV install this package?"
> - Compare versions: "Am I using the latest `pytest`?"

**Alternative**: Use `uv pip show <package>` to see detailed info about one package (dependencies, location, homepage).

## Troubleshooting ModuleNotFoundError: The Most Common Mistake

You'll encounter this error when running scripts. Let's understand and fix it.

### The Error Scenario

You run a script:

```bash
uv run src/my_script.py
```

**Error message**:
```
Traceback (most recent call last):
  File "src/my_script.py", line 1, in <module>
    import pandas as pd
ModuleNotFoundError: No module named 'pandas'
```

**What this means**: Your script tried to import `pandas`, but it's not installed in your project's `.venv/` directory.

### Diagnosis and Fix with AI

**Prompt to AI**:
```
I'm getting "ModuleNotFoundError: No module named 'pandas'" 
when running my script with uv run. Help me fix this.
```

**AI's Response** (approximate):

> **Problem**: Your script imports `pandas`, but it's not listed in your project's dependencies.
>
> **Diagnosis**:
> Let's check if `pandas` is installed:
> ```bash
> uv pip list | grep pandas
> ```
> (Or `uv pip list | findstr pandas` on Windows)
>
> **Result**: No output (pandas isn't installed)
>
> **Solution**: Add `pandas` to your dependencies:
> ```bash
> uv add pandas
> ```
>
> **What this does**:
> 1. UV resolves `pandas` and its dependencies (numpy, python-dateutil, pytz, etc.)
> 2. Downloads and installs them into `.venv/`
> 3. Updates `pyproject.toml` to include `pandas` in dependencies
> 4. Updates `uv.lock` with exact versions
>
> **After installation, run your script again**:
> ```bash
> uv run src/my_script.py
> ```
>
> **Now it works**: Python finds `pandas` in `.venv/lib/` and imports successfully.
>
> **Key lesson**: Every import in your code must have a corresponding dependency in `pyproject.toml`. If you import it, you need to add it.

### Common Variations of This Error

**Variant 1: Wrong package name**

```
ModuleNotFoundError: No module named 'PIL'
```

**Prompt to AI**: "I need the PIL module but 'pip add PIL' doesn't work"

**AI's explanation**: PIL is installed via the `Pillow` package (historical name change). Add `Pillow`, import as `PIL`:
```bash
uv add Pillow
```

**Variant 2: Import path mismatch**

```python
# Your code
from mypackage.module import function  # Wrong path
```

**AI's diagnosis**: Check the package documentation for correct import path. Some packages use different names for imports vs. installation.

### Prevention Strategy

**Before running a script**:

**Prompt to AI**:
```
I wrote a script that imports requests, pandas, and matplotlib. 
What dependencies do I need to add to my UV project?
```

**AI lists**:
```bash
uv add requests pandas matplotlib
```

**Proactive approach**: List imports → Add dependencies → Run script. Avoids ModuleNotFoundError entirely.

## Comparing `uv run` vs. Regular `python`: Side-by-Side

Let's demonstrate the difference concretely.

### Experiment Setup

**Scenario**: You have a UV project with `requests` installed. Your system Python doesn't have `requests` globally.

**Script** (`src/test_import.py`):
```python
import sys
print(f"Python executable: {sys.executable}")

import requests
print(f"Requests version: {requests.__version__}")
print("Success! Requests imported correctly.")
```

### Using `uv run` (Correct Method)

**Prompt to AI**:
```
Run src/test_import.py using UV
```

**Command**:
```bash
uv run src/test_import.py
```

**Output** (example):
```
Python executable: C:\Users\you\projects\my-project\.venv\Scripts\python.exe
Requests version: 2.33.0
Success! Requests imported correctly.
```

**What happened**: UV used Python from `.venv/`, which has `requests` installed.

### Using Regular `python` (Wrong Method - Demonstration Only)

**Command** (don't use this in UV projects):
```bash
python src/test_import.py
```

**Output** (example):
```
Python executable: C:\Python313\python.exe
Traceback (most recent call last):
  File "src/test_import.py", line 4, in <module>
    import requests
ModuleNotFoundError: No module named 'requests'
```

**What happened**: System Python (C:\Python313\) doesn't have `requests` installed. Wrong environment.

### Key Takeaway

**In UV projects, always use `uv run`**:
- ✅ `uv run script.py` (uses project's `.venv/`)
- ✅ `uv run pytest` (uses dev dependencies)
- ✅ `uv pip list` (shows project packages)
- ❌ `python script.py` (uses system Python, wrong environment)
- ❌ `pytest` (uses global pytest if installed, wrong environment)

**Exception**: After manually activating the environment (advanced users), `python` works—but UV's auto-activation via `uv run` is simpler and less error-prone.

## Try With AI

Practice running code in UV projects with your AI companion tool.

**Setup**: Use your UV project from previous lessons (or create a new one with `requests` dependency)

### Prompt 1: Run a Script
```
Create a simple Python script in src/ that makes an HTTP GET request 
to https://api.github.com/zen and prints the response text. 
Then run it using UV.
```

**What you're practicing**: Script creation and execution with AI

**Expected outcome**: AI creates script, runs it with `uv run`, you see GitHub's Zen quote

**Validation**: Did the script run successfully? Can you explain why `uv run` was necessary?

### Prompt 2: Run Tests
```
Create a test file in tests/ that tests the script from Prompt 1 
(mock the HTTP request to avoid hitting the real API). 
Then run the tests with pytest.
```

**What you're practicing**: Test-driven development with UV

**Expected outcome**: AI creates test with mocking, runs `uv run pytest`, tests pass

**Validation**: Do you understand why pytest is a dev dependency? Can you run tests repeatedly?

### Prompt 3: Verify Installed Packages
```
Show me all packages installed in my UV project, 
then explain why 'certifi' and 'urllib3' appear even though 
I didn't explicitly add them.
```

**What you're practicing**: Understanding transitive dependencies

**Expected outcome**: AI runs `uv pip list`, explains transitive dependencies (certifi/urllib3 are dependencies of requests)

**Validation**: Can you identify which packages you added vs. which are transitive?

### Prompt 4: Troubleshoot Missing Package
```
I want to add a script that uses the 'python-dotenv' library 
to load environment variables from a .env file. 
What do I need to do before running this script?
```

**What you're practicing**: Proactive dependency management

**Expected outcome**: AI instructs to add `python-dotenv` first, then create/run script

**Validation**: Do you understand the workflow? (Add dependency → Write code → Run with `uv run`)

### Prompt 5: Run One-Off Command
```
Without creating a script, use UV to run a one-line Python command 
that prints the current working directory.
```

**What you're practicing**: One-off commands with `uv run`

**Expected outcome**: AI shows `uv run python -c "import os; print(os.getcwd())"`

**Validation**: Can you explain how `uv run python -c` works? (Runs Python interpreter with inline code)
