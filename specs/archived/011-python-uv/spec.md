# Feature Specification: Chapter 11 - Python UV: The Fastest Python Package Manager

**Feature Branch**: `011-python-uv`  
**Created**: 2025-11-04  
**Status**: Draft  
**Input**: User description: "Write chapter 11 in Part 4 of the book. The title of the chapter will be 'Python UV: The Fastest Python Package Manager'. The chapter will cover the AI Driven Way of understanding and using UV for building project. In the AI Driven way the students do not try to memorize the commands or syntax but understand the concepts and use Claude Code or Gemini CLI to give UV commands. Use AI CLIs as co developers and teachers and execute the intent of the developer."

## Business Goals & Success Evals *(Evals-First Methodology)*

### Business Goals

**What can students DO after this chapter?**
- **Employability**: Students can manage Python projects professionally using modern tooling (UV), making them competitive in the job market where UV adoption is rapidly growing
- **Productivity**: Students leverage AI to handle package management 10x faster than manual approaches, focusing creative energy on problem-solving rather than memorizing commands
- **AI-Native Mindset**: Students internalize "intent expression" as the primary skill over syntax memorization—the core competency for AI-native development
- **Real-World Application**: Students can set up production-ready Python projects with proper dependency management, virtual environments, and team collaboration workflows

**What real-world problems can they solve?**
- Start new Python projects in seconds (not hours) with proper structure
- Install dependencies without conflicts or version hell
- Share projects with teammates with guaranteed environment reproducibility
- Troubleshoot package issues by describing problems to AI instead of reading Stack Overflow for hours
- Manage multiple Python projects on the same machine without interference

**How does this connect to their career goals?**
- UV is backed by Astral (creators of Ruff), showing strong industry momentum
- Package management competency is a universal requirement for Python developers
- AI-assisted workflows are becoming the professional standard (this chapter teaches that workflow)
- Understanding dependency management demonstrates professional maturity to employers

### Success Evals

**How do we measure successful learning?**

1. **Quiz Assessment**: Students score 75%+ on end-of-chapter assessment covering:
   - UV's value proposition (speed, unified tooling)
   - When to use UV vs. pip/poetry/conda (decision-making)
   - Core concepts: virtual environments, lockfiles, dependency resolution
   - AI-driven workflow: expressing intent, validating AI outputs

2. **Hands-On Exercise**: Students complete "Create FastAPI Project with UV and AI" without instructor help:
   - Express intent to AI: "Create a new FastAPI project with UV"
   - AI guides through: initialization, dependency installation, environment setup
   - Run development server successfully
   - Student explains what happened conceptually (not command-by-command recitation)

3. **Real-World Application**: Students successfully prepare a project for team collaboration:
   - Clone a UV project (or create one)
   - Ensure reproducible environment (lockfiles present)
   - Add a new dependency using AI
   - Commit and share with teammate simulation
   - Teammate can recreate exact environment with `uv sync`

4. **Teaching Ability**: Students can teach UV to a peer using concepts + AI demonstration:
   - Explain "why UV" without technical jargon (speed, simplicity)
   - Show peer how to use Claude Code/Gemini CLI to execute UV commands
   - Peer completes a basic task (create project, add dependency) with student's guidance
   - Student answers "how is this different from pip?" conceptually

5. **AI Interaction Quality**: Students demonstrate effective AI collaboration:
   - Ask clear intent-based questions ("How do I add pytest for testing?")
   - Understand AI responses (not just copy-paste commands)
   - Validate AI suggestions ("Why did you use --dev flag?")
   - Troubleshoot errors by describing symptoms to AI

### Quality Standards

**What defines acceptable quality for this chapter?**

- **Technical Accuracy**: 100% - All commands tested on Windows/Mac/Linux; all examples work
- **Constitution Alignment**: 100% - Passes validation-auditor validation (AI-first teaching, intermediate complexity tier, spec-first methodology)
- **Pedagogical Effectiveness**: 90%+ of readers complete chapter using AI as interactive documentation (measured via feedback survey: "Did you need to consult UV docs directly?")
- **Code Quality**: All examples include complete prompt → response → output → teaching cycle
- **Accessibility**: Grade 7+ reading level; technical terms explained in plain language

### Connection to Evals → Spec → Implement → Validate Workflow

This specification follows the professional AI-native pattern:
1. **Evals Defined** ↑ (above) — Clear success criteria before writing detailed requirements
2. **Spec Written** ↓ (below) — User stories and requirements designed to meet evals
3. **Implement** (future `/sp.implement`) — Content created to satisfy spec
4. **Validate** (future validation-auditor) — Verify implementation meets evals and spec

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Understanding UV's Purpose and Value Proposition (Priority: P1)

A beginner Python learner wants to understand what UV is, why it exists (the problem it solves), and how it differs from pip/poetry/conda without getting overwhelmed by technical details or memorizing commands.

**Why this priority**: Without understanding the "why," readers won't appreciate UV's value or be motivated to learn it. This foundational knowledge is essential before any hands-on work. In the AI-Driven Development approach, understanding concepts precedes executing commands.

**Independent Test**: Can be fully tested by having a reader with basic Python knowledge explain back: (1) what problem UV solves (slow package installation and project setup), (2) how UV achieves speed (Rust implementation, unified tool), and (3) when they would use UV versus traditional tools—WITHOUT reciting any UV commands.

**Acceptance Scenarios**:

1. **Given** a reader has used pip for simple package installation, **When** they read about UV's speed advantages, **Then** they understand why a 10-100x speedup matters for their workflow (less waiting, faster iteration)
2. **Given** a reader is confused by Python's tooling landscape (pip, venv, virtualenv, pipenv, poetry, conda), **When** they learn UV's unified approach, **Then** they understand how UV replaces multiple tools with one consistent interface
3. **Given** a reader is skeptical about "yet another Python tool," **When** they see UV's backing (Astral, creators of Ruff) and adoption metrics, **Then** they recognize UV as a strategic long-term bet rather than a fad
4. **Given** a reader doesn't understand Rust or low-level implementation, **When** they read about UV being "written in Rust," **Then** they understand the user benefit (speed) without needing technical deep-dive into Rust internals

---

### User Story 2 - AI-Driven Installation and Setup (Priority: P2)

A reader wants to install UV on their system using Claude Code or Gemini CLI as their AI co-developer, understanding the installation process conceptually rather than memorizing platform-specific commands.

**Why this priority**: Installation is the practical entry point, but in the AI-Driven approach, the reader focuses on communicating intent to AI ("I want to install UV on Windows") rather than remembering installation commands. This establishes the AI-as-partner pattern.

**Independent Test**: Can be fully tested by having a reader successfully install UV by giving clear intent to Claude Code/Gemini CLI (e.g., "Install UV package manager on my Windows system"), verifying installation, and explaining to the AI why they're installing it—without manually typing curl commands or installer URLs.

**Acceptance Scenarios**:

1. **Given** a reader is on Windows/Mac/Linux, **When** they ask Claude Code "Install UV Python package manager on my system," **Then** Claude Code determines the platform, provides the correct installation command with explanation, and executes it (with reader approval)
2. **Given** UV installation completes, **When** the reader asks Claude Code "Verify UV is installed correctly," **Then** Claude Code runs `uv --version` and confirms successful installation
3. **Given** a reader wants to understand what happened, **When** they ask Claude Code "Explain what the UV installation command did," **Then** Claude Code explains: download location, PATH modification, and where UV was installed—teaching through explanation, not memorization
4. **Given** a reader encounters an installation error, **When** they copy the error message to Claude Code, **Then** Claude Code diagnoses the issue (permissions, PATH problems, etc.) and provides corrective commands with explanation

---

### User Story 3 - AI-Driven Project Initialization (Priority: P3)

A reader wants to create their first Python project with UV, using AI to understand project structure, dependencies, and configuration without memorizing `uv init`, `uv add`, or file formats.

**Why this priority**: After installation, creating a project is the next natural step. In AI-Driven Development, the reader expresses intent ("Create a new Python project for a FastAPI web app") and the AI handles syntax, file creation, and configuration details.

**Independent Test**: Can be fully tested by having a reader successfully create a new UV project by describing their intent to Claude Code/Gemini CLI, ending up with a working project structure (pyproject.toml, virtual environment, installed dependencies) without manually typing `uv init` or editing TOML files.

**Acceptance Scenarios**:

1. **Given** a reader wants to start a new Python project, **When** they tell Claude Code "Create a new Python project called 'my-api' using UV," **Then** Claude Code runs `uv init my-api`, explains the created structure (pyproject.toml, .python-version, src/), and shows why each component exists
2. **Given** a project is initialized, **When** the reader says "I need FastAPI, uvicorn, and pydantic for this project," **Then** Claude Code runs `uv add fastapi uvicorn pydantic`, explains what's happening (dependency resolution, installation), and shows the updated pyproject.toml
3. **Given** a reader sees the pyproject.toml file, **When** they ask Claude Code "Explain what's in pyproject.toml and why it matters," **Then** Claude Code walks through each section: project metadata, dependencies, Python version constraints—focusing on concepts, not syntax memorization
4. **Given** a reader wants to work on the project later, **When** they ask "How do I activate the project environment?" **Then** Claude Code explains UV's approach (automatic virtual environment activation) and demonstrates with `uv run python --version`
5. **Given** a reader is used to requirements.txt, **When** they ask "Where's requirements.txt? How do I see my dependencies?" **Then** Claude Code explains UV's pyproject.toml-first approach, shows `uv pip freeze` for traditional format, and explains why modern Python uses pyproject.toml

---

### User Story 4 - AI-Driven Dependency Management (Priority: P4)

A reader wants to add, update, remove, and understand dependencies in their UV project using AI to handle the commands while they focus on intent and understanding the dependency resolution process.

**Why this priority**: Dependency management is a core skill but requires understanding project structure first. In AI-Driven Development, readers learn by expressing needs ("Add pytest for testing," "Update this outdated package") and observing how AI translates that into UV commands.

**Independent Test**: Can be fully tested by having a reader successfully add development dependencies, update packages, handle version conflicts, and explain dependency groups (dev, test, docs) by working with Claude Code—without memorizing `uv add --dev` syntax or TOML version specifiers.

**Acceptance Scenarios**:

1. **Given** a reader needs testing tools, **When** they tell Claude Code "Add pytest and coverage tools as development dependencies," **Then** Claude Code runs `uv add --dev pytest pytest-cov`, explains the `--dev` flag (development-only dependencies), and shows where they appear in pyproject.toml
2. **Given** a project has outdated dependencies, **When** the reader asks "Which of my packages have updates available?" **Then** Claude Code runs `uv pip list --outdated`, explains the output, and offers to update specific packages with reasoning
3. **Given** a reader wants to update a specific package, **When** they say "Update FastAPI to the latest version," **Then** Claude Code runs `uv add fastapi@latest`, shows the version change, and explains any downstream dependency updates triggered by UV's resolver
4. **Given** a dependency conflict occurs, **When** the reader pastes the error to Claude Code, **Then** Claude Code explains the conflict (e.g., "Package A needs B>=2.0, but C needs B<2.0"), suggests resolution strategies (pin versions, find compatible alternatives), and helps implement the fix
5. **Given** a reader wants to remove an unused package, **When** they say "Remove the requests library, I don't need it anymore," **Then** Claude Code runs `uv remove requests`, confirms removal, and shows the updated pyproject.toml

---

### User Story 5 - AI-Driven Project Execution and Task Running (Priority: P5)

A reader wants to run Python scripts, tests, and custom tasks within their UV project environment using AI to handle activation, execution, and troubleshooting—understanding environment isolation without memorizing activation commands.

**Why this priority**: Running code is the payoff for project setup. In AI-Driven Development, readers focus on "run my FastAPI server" or "execute tests" and AI handles `uv run` syntax, environment activation, and common execution patterns.

**Independent Test**: Can be fully tested by having a reader successfully run scripts, tests, and start a development server by giving intent to Claude Code, understanding why environment isolation matters, without manually typing `uv run` commands.

**Acceptance Scenarios**:

1. **Given** a reader has written a Python script in their project, **When** they tell Claude Code "Run the main.py script," **Then** Claude Code executes `uv run python main.py`, explains that UV automatically uses the project's virtual environment, and shows the output
2. **Given** a reader wants to start a FastAPI development server, **When** they say "Start the FastAPI app with hot reload," **Then** Claude Code runs `uv run uvicorn main:app --reload`, explains what's happening (server startup, port binding, auto-reload), and confirms the server is running
3. **Given** a reader has pytest installed, **When** they ask "Run all my tests," **Then** Claude Code executes `uv run pytest`, shows test results, and offers to help debug any failures
4. **Given** a reader wants to understand the difference between `uv run` and regular execution, **When** they ask "Why do I need 'uv run' instead of just 'python main.py'?" **Then** Claude Code explains environment isolation, shows what happens without `uv run` (wrong Python version, missing packages), and demonstrates the difference
5. **Given** a reader wants to run a one-off command in the project environment, **When** they say "Show me all installed packages in this project," **Then** Claude Code runs `uv run pip list`, displays packages, and explains this is the project-specific environment

---

### User Story 6 - AI-Driven Collaboration and Environment Reproducibility (Priority: P6)

A reader wants to share their UV project with teammates or deploy it, using AI to understand lockfiles, version pinning, and reproducible environments without memorizing the purpose of uv.lock or `uv sync` commands.

**Why this priority**: Collaboration is essential in real projects but builds on individual project management skills. In AI-Driven Development, readers learn by asking "How do I ensure my teammate has the same environment?" and AI explains lockfiles and sync mechanisms.

**Independent Test**: Can be fully tested by having a reader clone a UV project (or receive one from a teammate), successfully recreate the exact environment using Claude Code guidance, and explain why lockfiles matter—without memorizing `uv sync` syntax.

**Acceptance Scenarios**:

1. **Given** a reader has a working UV project, **When** they ask Claude Code "Prepare this project for my teammate to use," **Then** Claude Code ensures uv.lock exists (runs `uv lock` if needed), commits pyproject.toml and uv.lock to git, and explains why both files matter
2. **Given** a reader clones a UV project from GitHub, **When** they tell Claude Code "Set up this project on my machine," **Then** Claude Code runs `uv sync`, explains it's installing the exact versions from uv.lock, and verifies the environment matches the original developer's
3. **Given** a reader wants to understand lockfiles, **When** they ask "What's the difference between pyproject.toml and uv.lock?" **Then** Claude Code explains: pyproject.toml specifies constraints ("FastAPI >=0.100.0"), uv.lock pins exact versions ("FastAPI 0.104.1"), ensuring reproducibility across machines
4. **Given** a reader updates a dependency, **When** they ask "Do I need to update anything else for my team?" **Then** Claude Code explains the lockfile is automatically updated, demonstrates with `git diff uv.lock`, and confirms teammates will get the same versions after `uv sync`
5. **Given** a reader wants to deploy to production, **When** they ask "How do I install dependencies in production without dev tools?" **Then** Claude Code runs `uv sync --no-dev`, explains production-only installation, and shows how to verify the production environment

---

### User Story 7 - AI-Driven Understanding of UV's Advanced Features (Priority: P7)

A reader wants to explore UV's advanced capabilities (Python version management, script execution, tools installation) using AI to discover features as needed rather than reading comprehensive documentation.

**Why this priority**: Advanced features are discovered through use, not upfront learning. In AI-Driven Development, readers ask "Can UV install different Python versions?" when they encounter that need, and AI teaches just-in-time.

**Independent Test**: Can be fully tested by having a reader successfully use at least two advanced UV features (Python version switching, standalone script execution, or global tool installation) by asking Claude Code about specific needs without pre-studying UV documentation.

**Acceptance Scenarios**:

1. **Given** a reader needs Python 3.12 for a specific project, **When** they ask Claude Code "Can UV help me use Python 3.12 for this project?" **Then** Claude Code explains UV's built-in Python management, runs `uv python install 3.12`, and shows how to pin it in pyproject.toml
2. **Given** a reader has a standalone script (not a full project), **When** they ask "Can I run this single Python script with specific dependencies without creating a full project?" **Then** Claude Code explains UV's inline script dependencies, shows how to add `# /// script` metadata, and runs the script with `uv run`
3. **Given** a reader wants to install global tools (like Ruff or Black), **When** they ask "How do I install Python tools globally with UV?" **Then** Claude Code runs `uv tool install ruff`, explains the difference between project dependencies and global tools, and shows how to run installed tools
4. **Given** a reader wants to experiment in a Python REPL, **When** they say "Start a Python shell with NumPy and Pandas available," **Then** Claude Code runs `uv run --with numpy --with pandas python`, launches a REPL with temporary dependencies, and explains this is for exploration, not project work
5. **Given** a reader discovers a UV feature organically, **When** they ask "What else can UV do that I might not know about?" **Then** Claude Code provides a curated list of features relevant to the reader's current context (e.g., workspace management if they have multiple projects, build tools if they're packaging)

---

### Edge Cases

- What happens when a reader tries to use UV in a project that already has a requirements.txt or Pipfile? (Migration scenario)
- How does UV handle platform-specific dependencies (Linux vs. Windows vs. macOS) and how does AI explain this?
- What happens if a reader's Python version is incompatible with a project's requirements? (UV's Python management vs. system Python)
- How does Claude Code handle UV errors (network issues, disk space, permission problems) and guide debugging?
- What happens when a reader wants to use both conda and UV? (Compatibility and trade-offs)
- How does UV work in corporate environments with private package indexes or proxy servers? (Authentication and configuration)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Chapter MUST explain UV's core value proposition (speed, unified tooling) in beginner-friendly language without requiring prior knowledge of pip, poetry, or conda
- **FR-002**: Chapter MUST demonstrate AI-Driven Development workflow: reader expresses intent → AI provides commands with explanations → reader understands concepts, not syntax
- **FR-003**: Chapter MUST include examples of Claude Code and Gemini CLI prompts for every UV task, showing both the prompt and the AI's response (command + explanation)
- **FR-004**: Chapter MUST cover UV installation across Windows, macOS, and Linux using AI CLI tools, emphasizing platform detection by AI
- **FR-005**: Chapter MUST teach project initialization (`uv init`) through AI interaction, explaining the created files (pyproject.toml, .python-version, virtual environment) conceptually
- **FR-006**: Chapter MUST demonstrate dependency management (add, update, remove) via AI prompts, with AI explaining dependency resolution and version constraints
- **FR-007**: Chapter MUST show how to run code in UV projects (`uv run`) using AI, explaining environment isolation and why it matters
- **FR-008**: Chapter MUST explain environment reproducibility (lockfiles, `uv sync`) through AI-guided scenarios, emphasizing team collaboration
- **FR-009**: Chapter MUST introduce at least three advanced UV features (Python version management, standalone scripts, tool installation) through AI-discovery pattern
- **FR-010**: Chapter MUST include troubleshooting scenarios where reader asks AI to debug UV errors (network, permissions, conflicts)
- **FR-011**: All code examples MUST show: (1) Reader's prompt to AI, (2) AI's response (command + explanation), (3) Expected output/result, (4) AI's follow-up teaching
- **FR-012**: Chapter MUST reference official UV documentation (docs.astral.sh/uv/) as the authoritative source while positioning AI as the interactive learning interface
- **FR-013**: Chapter MUST compare UV to pip/poetry/conda conceptually (when to use each) without requiring readers to learn all tools
- **FR-014**: Chapter MUST align with Part 4's complexity tier (beginner-to-intermediate): max 7 concepts per section, AI as decision-maker for commands, focus on understanding over memorization
- **FR-015**: Chapter MUST include at least one complete walkthrough: "Create a FastAPI project with UV and AI" showing end-to-end workflow (init → add deps → run → iterate)

### Key Entities *(include if feature involves data)*

- **UV Project**: A Python project managed by UV, consisting of pyproject.toml (dependency specification), uv.lock (pinned versions), virtual environment (.venv), and source code
- **Dependency**: A Python package required by the project, categorized as production or development, with version constraints specified in pyproject.toml and exact versions in uv.lock
- **Python Version**: The interpreter version specified in .python-version or pyproject.toml, managed by UV's built-in Python installer
- **AI Prompt**: The reader's natural language instruction to Claude Code or Gemini CLI, expressing intent without syntax knowledge
- **AI Response**: The AI's structured reply including: command to execute, explanation of what the command does, expected outcome, and teaching context

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Readers can explain UV's core value (speed, unified tooling) in their own words without referencing commands or syntax (tested via "explain back" exercise)
- **SC-002**: Readers can successfully create a new UV project by giving intent to Claude Code/Gemini CLI without memorizing `uv init`, `uv add`, or `uv run` syntax (tested via practical exercise with AI)
- **SC-003**: Readers can add, update, and remove dependencies using AI prompts, understanding dependency resolution conceptually (tested via AI-guided dependency management task)
- **SC-004**: Readers can run Python scripts, tests, and servers in UV projects using AI commands, explaining why environment isolation matters (tested via execution scenarios with AI)
- **SC-005**: Readers can set up a UV project for team collaboration (lockfile, sync) by asking AI "how to share my project," without pre-memorizing commands (tested via collaboration scenario)
- **SC-006**: Readers can troubleshoot common UV errors (network, permissions, conflicts) by describing issues to AI and understanding AI's diagnostic process (tested via error debugging exercise)
- **SC-007**: Chapter examples include prompt-response pairs for at least 15 distinct UV tasks, each showing: prompt, AI response (command + explanation), output, and teaching follow-up
- **SC-008**: Readers understand when to use UV vs. pip/poetry/conda based on project needs, not tool evangelism (tested via decision-making scenario)
- **SC-009**: Readers can discover UV's advanced features (Python management, standalone scripts) just-in-time by asking AI when they encounter specific needs (tested via exploration exercise)
- **SC-010**: 90% of readers complete the chapter without needing to consult UV documentation directly—AI serves as the interactive documentation interface (measured via feedback survey)
- **SC-011**: Readers can teach a teammate about UV by explaining concepts and demonstrating AI-Driven workflow, not by listing commands (tested via peer teaching exercise)

### Assumptions

- Readers have completed Chapters 1-10 (AI-Driven Development foundations, AI CLI tools, Prompt & Context Engineering)
- Readers have Claude Code or Gemini CLI installed and authenticated from Part 2
- Readers have basic terminal/command-line competency from Chapter 7 (Bash Essentials)
- Readers understand how to use AI CLI tools to execute commands and ask questions
- **Python syntax knowledge NOT assumed** - UV concepts taught independently of Python programming
- Readers have internet access for UV installation and package downloads
- Readers have basic computer literacy (file systems, installing software, navigating directories)
- Chapter focuses on "what is a Python project" and "how to manage dependencies" conceptually
- Understanding of virtual environments will be explained from first principles (no prior knowledge required)
- Chapter focuses on UV for project management, not building distributable Python packages (publishing to PyPI is out of scope)

### Out of Scope

- Building and publishing Python packages to PyPI (covered in later chapters on distribution)
- Deep dive into TOML syntax or manual editing of pyproject.toml (AI handles this)
- Comparison of virtual environment implementations (venv vs. virtualenv internals)
- Advanced dependency resolution algorithms or constraint solving theory
- Setting up private package indexes or corporate artifact repositories (enterprise concerns)
- Using UV with Jupyter notebooks or data science workflows (covered separately in data science chapters)
- Migrating large legacy projects from pip/poetry to UV (advanced migration strategies)
- UV's internal architecture or Rust implementation details
- Performance benchmarking or optimization of UV itself

### Prerequisites

**This chapter is positioned as the FIRST chapter in Part 4, serving as a bridge from Part 3 (Prompt & Context Engineering) to Python fundamentals. It sets up the tooling students will use for ALL subsequent Python chapters (12-29).**

- **Required**: Chapters 1-4 (AI-Driven Development foundations, Nine Pillars methodology)
- **Required**: Chapter 5 (Claude Code) or Chapter 6 (Gemini CLI)—readers must have at least one AI CLI tool set up
- **Required**: Chapter 7 (Bash Essentials)—basic terminal/command-line competency for executing commands
- **Recommended**: Chapter 8 (Git & GitHub)—helpful for collaboration scenarios but not essential
- **Recommended**: Chapters 9-10 (Prompt & Context Engineering)—enhances AI interaction quality
- **Not required**: Python syntax knowledge—Chapter 12 will teach Python; UV taught as "project management tool" conceptually
- **Not required**: Prior experience with pip, poetry, conda, or any package manager (UV is taught from first principles)
- **Not required**: Understanding of virtual environments (chapter explains this concept from scratch)
- **Not required**: Prior programming experience in any language

**Pedagogical Positioning**: This chapter teaches "how to manage Python projects" BEFORE teaching "how to write Python code." Professional workflow: set up tooling first, then write code. Students learn UV conceptually (what is a dependency, what is a virtual environment, why speed matters) without needing to write Python code yet.

### Complexity Tier

**Intermediate (Part 4: Python Fundamentals)**

- **Cognitive Load**: Max 7 new concepts per section
- **AI Assistance Level**: AI as primary command executor and explainer; reader focuses on intent and understanding
- **Options Presented**: 2-3 alternatives (UV vs. pip/poetry) with clear guidance on when to use each
- **Error Handling**: AI-guided troubleshooting with explanation of what went wrong and why
- **Assessment**: Readers demonstrate understanding through AI-guided tasks, not command memorization tests

### Related Documentation

- Official UV Documentation: https://docs.astral.sh/uv/
- UV Installation Guide: https://docs.astral.sh/uv/getting-started/installation/
- UV Features Overview: https://docs.astral.sh/uv/getting-started/features/
- UV Project Guide: https://docs.astral.sh/uv/guides/projects/
- Astral (UV creators): https://astral.sh/
- Python Packaging User Guide: https://packaging.python.org/ (context for why UV exists)

---

## Appendix: Sample Prompt-Response Patterns

**Purpose**: These examples illustrate the expected format for all UV examples in the chapter. Each example MUST include: (1) Reader's prompt, (2) AI's response with command + explanation, (3) Expected output, (4) AI's teaching follow-up.

### Example 1: Project Initialization

**Reader Prompt to Claude Code**:
```
I want to create a new Python project for a FastAPI web application. 
Set up the project structure with UV.
```

**Claude Code Response**:
```bash
# I'll create a new UV project for your FastAPI app.
# First, let me initialize the project:

uv init my-fastapi-app

# ✅ Created project structure:
# - my-fastapi-app/
#   ├── pyproject.toml (project configuration)
#   ├── .python-version (Python version lock)
#   └── src/ (your source code goes here)

# Now let's add FastAPI and its dependencies:

uv add fastapi uvicorn pydantic

# ✅ Installing packages...
# ✅ Resolved dependencies in 0.8s
# ✅ Installed 25 packages in 1.2s
```

**Expected Terminal Output**:
```
Initialized project `my-fastapi-app` at `/path/to/my-fastapi-app`
Using Python 3.13.0
Resolved 25 packages in 823ms
Downloaded 25 packages in 1.1s
Installed 25 packages in 189ms
 + annotated-types==0.6.0
 + anyio==4.2.0
 + fastapi==0.109.0
 + pydantic==2.5.3
 + pydantic-core==2.14.6
 + uvicorn==0.27.0
 [... additional packages ...]
```

**AI Teaching Follow-up**:
> "Perfect! UV created your project with a `pyproject.toml` file (think of it like package.json in Node.js) 
> that tracks your dependencies. The `.python-version` file ensures everyone on your team 
> uses the same Python version. UV also created a virtual environment automatically—
> you don't need to activate it manually. When you run `uv run python`, UV uses the 
> project's environment automatically.
> 
> Notice how fast that was? UV installed 25 packages in under 2 seconds. With pip, 
> this would take 20-30 seconds. That's the 10-100x speedup we talked about.
> 
> Want to see what packages are installed? Ask me: 'Show me all installed packages in this project.'"

---

### Example 2: Adding Development Dependencies

**Reader Prompt to Claude Code**:
```
I need to add testing tools to my project. I want pytest and coverage tools, 
but only for development, not production.
```

**Claude Code Response**:
```bash
# I'll add pytest and pytest-cov as development dependencies.
# The --dev flag marks them as dev-only (won't be installed in production):

uv add --dev pytest pytest-cov

# ✅ Resolved dependencies in 0.5s
# ✅ Installed 8 packages in 0.3s
```

**Expected Terminal Output**:
```
Resolved 8 packages in 487ms
Downloaded 8 packages in 421ms
Installed 8 packages in 298ms
 + coverage==7.4.1
 + iniconfig==2.0.0
 + packaging==23.2
 + pluggy==1.3.0
 + pytest==7.4.4
 + pytest-cov==4.1.0
 + tomli==2.0.1
```

**AI Teaching Follow-up**:
> "Done! I added pytest and pytest-cov to your `pyproject.toml` under `[project.optional-dependencies]` 
> with a 'dev' group. This means:
> 
> - When you run `uv sync`, these packages install (for development)
> - When you deploy to production with `uv sync --no-dev`, they're skipped
> - Your teammates will get the same dev tools when they run `uv sync`
> 
> This keeps your production environment lean (faster deploys, smaller containers) 
> while giving developers all the tools they need.
> 
> Want to run your tests? Ask me: 'Run pytest with coverage reporting.'"

---

### Example 3: Troubleshooting Installation Error

**Reader Prompt to Claude Code**:
```
I tried to install UV but got this error:
"uv: command not found"

What went wrong?
```

**Claude Code Response**:
```markdown
This error means UV installed successfully, but your shell can't find it yet. 
The UV executable isn't in your system's PATH.

Let me help you fix this based on your platform:

**For macOS/Linux**:
The installer should have added UV to your PATH automatically, but you need to 
restart your terminal or reload your shell configuration:

# Restart your terminal, OR run:
source ~/.bashrc    # For bash
source ~/.zshrc     # For zsh

**For Windows (PowerShell)**:
The installer adds UV to your PATH, but PowerShell needs to reload:

# Close and reopen PowerShell, OR run:
$env:Path = [System.Environment]::GetEnvironmentVariable("Path","Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path","User")

Let me verify UV is working now:
```

```bash
uv --version
```

**Expected Output After Fix**:
```
uv 0.1.6 (Rust 1.75.0)
```

**AI Teaching Follow-up**:
> "Great! UV is now accessible. The 'command not found' error is common after installing 
> new command-line tools—it just means your terminal hasn't refreshed its list of 
> available commands yet.
> 
> **What is PATH?** Think of it as your computer's 'phonebook' for commands. When you 
> type 'uv', your terminal looks through the PATH to find where the 'uv' program lives.
> 
> The UV installer added the UV location to PATH, but your current terminal session 
> was using the old PATH. Restarting the terminal loads the updated PATH.
> 
> This is a one-time setup. From now on, 'uv' will work in all new terminal sessions.
> 
> Ready to create your first project? Ask me: 'Create a new Python project with UV.'"

---

**Implementation Note**: During the `/sp.implement` phase, ensure EVERY UV task (installation, project creation, dependency management, running code, collaboration) follows this 4-part pattern: Prompt → Response → Output → Teaching. This reinforces the AI-Driven Development methodology and ensures readers learn through AI interaction, not command memorization.

