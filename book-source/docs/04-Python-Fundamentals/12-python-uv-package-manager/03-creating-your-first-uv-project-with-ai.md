---
title: "Creating Your First UV Project with AI"
chapter: 12
lesson: 3
sidebar_position: 3
duration_minutes: 60

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
skills:
  - name: "Initialize UV Project"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create a new UV project by expressing intent ('Create a simple web-client project'), understanding the generated structure (pyproject.toml, .python-version, src/) and why each component exists"

  - name: "Understand Project Configuration"
    proficiency_level: "A2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can explain pyproject.toml's purpose (project metadata and dependencies) in plain language and recognize it as Python's modern standard (vs. requirements.txt)"

learning_objectives:
  - objective: "Apply AI-driven project initialization to create a Python project with proper structure"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Creates working UV project with dependencies using AI guidance"

  - objective: "Understand project anatomy (pyproject.toml, virtual environments, src/)"
    proficiency_level: "A2"
    bloom_level: "Understand"
    assessment_method: "Can explain purpose of each generated file/directory"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (Python project structure, pyproject.toml, .python-version, src/, virtual environment, dependency specification vs installation, modern vs legacy config) within B1 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Research pyproject.toml TOML syntax deeply; compare to package.json (Node.js), Cargo.toml (Rust); explore PEP 621 specification"
  remedial_for_struggling: "Focus on minimal project (just add requests); explore structure by asking AI questions; build confidence before complexity"

# Generation metadata
generated_by: "lesson-writer v1.0"
source_spec: "specs/011-python-uv/spec.md"
created: "2025-11-04"
last_modified: "2025-11-04"
git_author: "GitHub Copilot"
workflow: "/sp.implement"
version: "1.0.0"
---

# Creating Your First UV Project with AI

In Lessons 1-2, you understood why UV matters and installed it on your system. Now comes the practical application: creating a real Python project with professional structure using AI as your guide.

This isn't about memorizing `uv init` syntax or learning TOML configuration formats. This is about understanding *what a Python project is* (organized code with dependencies and configuration), *why structure matters* (reproducibility, collaboration, maintainability), and *how to use AI* to generate and customize projects instantly.

By the end of this lesson, you'll have a working UV project with a production dependency (the `requests` library for HTTP operations). You'll understand what each file does, why virtual environments matter, and how modern pyproject.toml configuration replaces the legacy requirements.txt approach.

## What Is a Python Project?

Before we generate project files, let's establish what we're building conceptually.

**A Python project is organized code with three essential components:**

1. **Configuration** (pyproject.toml): Metadata about your project (name, version, author) and the dependencies it needs
2. **Code** (src/ directory): Your Python modules and packages—the actual application logic
3. **Environment** (virtual environment): An isolated space containing project-specific dependencies

**Analogy: Recipe + Ingredients + Kitchen**

Think of a Python project like a recipe:
- **Recipe card** (pyproject.toml): Lists ingredients (dependencies) and instructions (configuration)
- **Ingredients** (dependencies): External libraries your code needs (requests, pytest, etc.)
- **Kitchen workspace** (virtual environment): A dedicated prep area where you store ingredients for *this recipe only*, separate from other recipes

**Why structure matters:**
- **Reproducibility**: Anyone can recreate your environment from pyproject.toml
- **Isolation**: Project A's dependencies don't conflict with Project B's
- **Collaboration**: Teammates get identical setups
- **Professionalism**: Organized projects are maintainable projects

Traditional Python tutorials start with "Hello World" in a loose `.py` file. That works for 10-line scripts but falls apart for real applications. We're starting with professional structure from day one—and AI makes it effortless.

## The AI-Driven Creation Flow

Here's the workflow we'll follow:

1. **Express Intent**: "Create a Python project called 'web-client' that can make HTTP requests"
2. **AI Generates Structure**: AI runs `uv init` and `uv add requests`, explains each component
3. **We Explore**: Tour the generated files, ask questions, understand purpose
4. **We Validate**: Verify project works, dependencies installed, structure correct

**Traditional approach** (what we're skipping):
- Read UV documentation about `uv init` flags
- Memorize pyproject.toml format
- Manually create src/ directories
- Learn TOML syntax for dependency specification
- Time investment: Hours to understand, days to internalize

**AI-native approach** (what we're doing):
- Describe what you want to build
- AI handles command syntax and file generation
- You focus on understanding concepts (configuration, dependencies, isolation)
- Time investment: Minutes to create, hours to understand (through exploration)

Ready? Let's create your first project.

## Complete Walkthrough: Create a Web-Client Project with UV and AI

Open your AI companion tool and follow this step-by-step workflow.

### Step 1: Express Your Intent

**Prompt to AI**:
```
Create a new Python project called 'web-client' using UV. 
This project will make HTTP requests, so add the 'requests' library as a dependency.
Explain what commands you're running and what files will be created.
```

### Step 2: Review AI's Proposed Commands

AI should suggest something like:

```bash
# Step 1: Create project structure
uv init web-client

# Step 2: Navigate into project
cd web-client

# Step 3: Add requests as a dependency
uv add requests
```

**AI's Explanation** (approximate):

> **`uv init web-client`**: Creates a new project directory called 'web-client' with:
> - `pyproject.toml` (project configuration file)
> - `.python-version` (specifies which Python version to use)
> - `src/web_client/` directory (where your code goes; note: hyphens → underscores)
> - `src/web_client/__init__.py` (marks this as a Python package)
> - `.venv/` virtual environment (created automatically for isolation)
> - `README.md` (project documentation)
>
> **`uv add requests`**: Adds the `requests` library to your project:
> - Updates `pyproject.toml` to list 'requests' as a dependency
> - Creates `uv.lock` (lockfile pinning exact versions)
> - Downloads and installs `requests` (and its dependencies) into `.venv/`

### Step 3: Execute the Commands

Run the commands AI provided. You'll see output like:

```
$ uv init web-client
Initialized project at /Users/yourname/web-client

$ cd web-client

$ uv add requests
Adding requests>=2.31.0 to dependencies
Resolving dependencies... (0.5s)
Downloaded 5 packages in 0.8s
Installed 5 packages in 0.2s
 + certifi==2024.7.4
 + charset-normalizer==3.3.2
 + idna==3.7
 + requests==2.32.3
 + urllib3==2.2.2
```

**What just happened:**
- UV created project structure in `web-client/` directory
- Added `requests` to pyproject.toml
- Resolved compatible versions for `requests` and its dependencies (certifi, urllib3, etc.)
- Downloaded and installed all packages into the project's virtual environment
- Created `uv.lock` to pin exact versions for reproducibility

Total time: 2-5 seconds (compare to 30-60 seconds with pip)

### Step 4: Explore the Generated Structure

Ask AI to show you what was created:

**Prompt to AI**:
```
Show me the directory structure of the web-client project. 
What does each file and directory do? (Brief, plain language)
```

**AI's Response** (approximate):

```
web-client/
├── .python-version          # Python version specification (e.g., 3.11)
├── pyproject.toml           # Project configuration and dependencies
├── uv.lock                  # Lockfile (exact versions for reproducibility)
├── README.md                # Project documentation
├── .venv/                   # Virtual environment (dependencies live here)
│   ├── bin/                 # Executable scripts (python, pip, etc.)
│   ├── lib/                 # Installed packages (requests, urllib3, etc.)
│   └── ...
└── src/
    └── web_client/          # Your code goes here (note: hyphen → underscore)
        └── __init__.py      # Marks this as a Python package
```

**Purpose of each component**:

- **`.python-version`**: Tells UV which Python version to use (e.g., `3.11`). Ensures everyone uses compatible Python.

- **`pyproject.toml`**: Project configuration file. Contains:
  - **Metadata**: Project name, version, description, author
  - **Dependencies**: Libraries this project needs (requests in our case)
  - **Build settings**: How to package this project (if publishing)

- **`uv.lock`**: Lockfile that records exact versions of every package installed (including transitive dependencies). Guarantees reproducible environments.

- **`README.md`**: Documentation for your project. Explain what it does, how to use it, how to contribute.

- **`.venv/`**: Virtual environment directory. Contains isolated copies of Python and all installed packages. You never edit files here manually—UV manages it.

- **`src/web_client/`**: Your application code goes here. UV uses a `src/` layout (best practice for Python projects) instead of putting code in the project root.

- **`__init__.py`**: Empty file that marks `web_client/` as a Python package. Allows `import web_client` to work.

## Anatomy of a UV Project: Key Files Explained

Let's examine the critical files more closely.

### pyproject.toml: The Project Configuration File

Open `pyproject.toml` in your editor. You'll see something like:

```toml
[project]
name = "web-client"
version = "0.1.0"
description = "Add your description here"
readme = "README.md"
requires-python = ">=3.11"
dependencies = [
    "requests>=2.32.3",
]

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
```

**Breaking it down** (plain language):

- **`[project]` section**: Metadata about your project
  - `name`: Project identifier (used when publishing to PyPI)
  - `version`: Semantic version (0.1.0 = initial development)
  - `description`: One-line summary of what this does
  - `readme`: Points to README.md for detailed documentation
  - `requires-python`: Minimum Python version needed (`>=3.11` means Python 3.11 or newer)
  - `dependencies`: List of required packages (`requests>=2.32.3` means "requests version 2.32.3 or compatible")

- **`[build-system]` section**: How to package this project (if publishing to PyPI). Uses Hatchling (a modern build tool). You won't touch this initially.

**What you should understand** (not memorize):
- Dependencies live in the `dependencies =` list
- Python version requirement prevents compatibility issues
- TOML format uses `[sections]` and `key = value` pairs (human-readable)

**What you'll never memorize**: TOML syntax details. When you need to customize, ask AI: "Add pytest to dev dependencies" or "Change project description to X."

### .python-version: Python Version Pinning

Content:
```
3.11
```

**Purpose**: Tells UV which Python version this project expects. When you (or a teammate) run UV commands, UV uses this Python version automatically.

**Why this matters**:
- Different Python versions have different features (3.10 doesn't have structural pattern matching, 3.11 does)
- Projects can specify minimum versions to ensure compatibility
- UV respects this file—no manual version switching needed

### uv.lock: The Lockfile

This file contains exact versions of every package:

```toml
[[package]]
name = "certifi"
version = "2024.7.4"
source = { registry = "https://pypi.org/simple" }
# ... detailed dependency metadata ...

[[package]]
name = "requests"
version = "2.32.3"
source = { registry = "https://pypi.org/simple" }
dependencies = [
    { name = "certifi" },
    { name = "charset-normalizer" },
    { name = "idna" },
    { name = "urllib3" },
]
# ...
```

**Purpose**: Records *exact* versions of all installed packages (including transitive dependencies like `certifi`, which `requests` depends on).

**Why lockfiles matter**:
- **Reproducibility**: When a teammate runs `uv sync`, they get *identical* versions
- **Stability**: Your project won't break because a dependency updated
- **Debugging**: You know exactly which versions were working (or causing issues)

**You don't edit uv.lock manually**. UV regenerates it automatically when you add/update dependencies.

## Virtual Environments Explained: Isolation Matters

When UV initialized your project, it created a `.venv/` directory. This is your **virtual environment**—an isolated Python installation specific to this project.

### Why Virtual Environments?

**The Problem Without Isolation**:

Imagine you have two projects:
- **Project A**: Needs `requests==2.25.0` (old version)
- **Project B**: Needs `requests==2.32.0` (new version)

If you install both globally (system-wide Python), you can only have *one* version of `requests` at a time. Installing for Project B breaks Project A.

**The Solution: Virtual Environments**:

Each project gets its own `.venv/` directory with its own copy of Python and packages. Project A's `.venv/` has `requests==2.25.0`, Project B's has `requests==2.32.0`. They don't conflict because they're isolated.

**Analogy: Separate Toolboxes**

Think of each project as a carpentry job:
- **Shared toolbox** (system Python): You have one hammer, one screwdriver. When you need a different hammer for Job B, you replace the hammer—breaking Job A.
- **Separate toolboxes** (virtual environments): Each job has its own complete toolbox. Job A keeps its hammer, Job B gets a different hammer. No conflicts.

### UV's Automatic Environment Handling

Traditional Python workflow:
```bash
python -m venv .venv          # Create environment
source .venv/bin/activate     # Activate (Mac/Linux)
# OR
.venv\Scripts\activate.ps1    # Activate (Windows)
pip install requests          # Install packages
```

**UV workflow**:
```bash
uv init my-project            # Environment created automatically
uv add requests               # Packages installed automatically
uv run python script.py       # Runs in environment automatically
```

**You never manually activate virtual environments with UV.** UV detects `.venv/` and uses it automatically. This reduces cognitive load—you think about dependencies, not activation mechanics.

## Adding Your First Dependency: How UV Handles Packages

We already added `requests` in our initial project creation. Let's understand what happened.

**Prompt to AI** (conceptual explanation):
```
Walk me through what happened when we ran 'uv add requests'. 
What steps did UV perform? Why do we need the 'requests' library?
```

**AI's Explanation** (approximate):

> When you ran `uv add requests`:
>
> 1. **UV read** your `pyproject.toml` to understand your project
> 2. **UV resolved dependencies**: 
>    - You requested `requests`
>    - `requests` depends on `certifi`, `charset-normalizer`, `idna`, `urllib3`
>    - UV computed compatible versions for all packages (constraint satisfaction)
> 3. **UV downloaded** packages from PyPI (Python Package Index)
> 4. **UV installed** packages into `.venv/lib/python3.11/site-packages/`
> 5. **UV updated** `pyproject.toml` to list `requests` in dependencies
> 6. **UV created/updated** `uv.lock` with exact versions
>
> **Why requests?** The `requests` library provides a simple API for making HTTP requests (GET, POST, etc.). Instead of writing low-level network code, you use `requests.get("https://api.example.com")` and it handles everything (connections, headers, error handling, JSON parsing).

**What is a dependency?** (Review from Lesson 1):

A dependency is code someone else wrote that your project needs. `requests` is a dependency because:
- You want to fetch data from web APIs
- Writing HTTP client code from scratch is complex (hundreds of edge cases)
- The `requests` library is battle-tested by millions of projects
- You save weeks of development time by reusing it

**Projects need dependencies to avoid reinventing wheels.** Modern software is built by composing reliable libraries, not writing everything from scratch.

## Legacy vs. Modern: pyproject.toml vs. requirements.txt

If you've seen older Python tutorials, you might have encountered `requirements.txt`. Let's clarify the difference.

### requirements.txt (Legacy Approach)

**Format**:
```
requests==2.32.3
certifi==2024.7.4
urllib3==2.2.2
```

**Limitations**:
- **No metadata**: Can't specify project name, version, Python requirement
- **Flat list**: Doesn't distinguish production vs. development dependencies
- **Manual maintenance**: You write exact versions by hand or `pip freeze`
- **No standard**: Every project uses different conventions

**Workflow**:
```bash
pip install -r requirements.txt    # Install dependencies
pip freeze > requirements.txt      # Update requirements.txt
```

### pyproject.toml (Modern Approach)

**Format** (as we saw):
```toml
[project]
name = "web-client"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = ["requests>=2.32.3"]

[project.optional-dependencies]
dev = ["pytest>=7.4.0"]
```

**Advantages**:
- **Standardized**: PEP 621 specification (official Python standard)
- **Metadata-rich**: Project info, Python requirements, build configuration—all in one file
- **Grouped dependencies**: Production vs. development vs. optional
- **Human-readable**: TOML syntax is clear and well-structured
- **Tool interoperability**: Poetry, UV, pip—all understand pyproject.toml

**Workflow**:
```bash
uv add requests                    # Automatically updates pyproject.toml
uv add --dev pytest                # Adds to dev dependencies
uv sync                            # Installs exact versions from uv.lock
```

**Migration path**: Many projects are moving from requirements.txt to pyproject.toml. UV uses pyproject.toml exclusively (the modern standard).

**Summary**:
- **requirements.txt**: Legacy, flat, manual, no standard
- **pyproject.toml**: Modern, structured, automatic, standardized

## Try With AI

Now practice the complete project creation workflow. Work through these prompts sequentially.

**Setup**: Ensure UV is installed (Lesson 2), use your AI companion tool

### Prompt 1: Create a Custom Project
```
Create a new Python project called [choose-your-name] using UV. 
Add the 'requests' library as a dependency. 
Explain what commands you're running and why.
```

**What you're practicing**: End-to-end project creation workflow

**Expected outcome**: AI provides `uv init` and `uv add` commands, project created successfully

**Validation**: Does the project directory exist? Is `requests` in `pyproject.toml`? Can you see `.venv/` directory?

### Prompt 2: Project Tour
```
Show me the directory structure of my project. 
Explain what each file and directory does (brief, plain language).
Don't assume I know technical terms.
```

**What you're practicing**: Exploring generated structure with AI as guide

**Expected outcome**: AI provides directory tree with explanations for each file

**Validation**: Do you understand what pyproject.toml, .python-version, uv.lock, and .venv/ do?

### Prompt 3: Understanding Configuration
```
Show me the important parts of pyproject.toml and explain what they mean. 
I don't need to understand TOML syntax deeply—just the key concepts.
```

**What you're practicing**: Configuration file comprehension

**Expected outcome**: AI highlights `[project]` section, dependencies list, Python version requirement

**Validation**: Can you identify where dependencies are listed? Where project metadata is?

### Prompt 4: Modern vs. Legacy Comparison
```
How is pyproject.toml different from the old requirements.txt approach? 
Why did Python move to pyproject.toml?
```

**What you're practicing**: Understanding tooling evolution

**Expected outcome**: AI explains metadata, standardization, tool interoperability benefits

**Validation**: Can you explain why modern projects use pyproject.toml to a peer?

