---
sidebar_position: 11
title: "Chapter 13: Python UV Quiz"
---

# Chapter 13: Python UV — The Fastest Python Package Manager Quiz

Test your understanding of UV package management, dependency handling, code quality tools, and professional Python workflows.

<Quiz
  title="Chapter 13: Python UV Package Manager Assessment"
  questions={[    {
      question: "Why was UV created when pip, poetry, and conda already existed?",
      options: [
        "To make package management more complex and configurable",
        "To replace Python entirely with Rust programming",
        "To unify fragmented tooling into one fast tool",
        "To add blockchain technology to dependency management"
      ],
      correctOption: 2,
      explanation: "UV was created to solve Python's tooling fragmentation problem. Instead of using multiple tools (pip for packages, venv for environments, poetry for locking), UV provides one fast, unified tool written in Rust. This addresses the confusion beginners face when choosing between pip, pipenv, poetry, and conda. UV isn't replacing Python or adding complexity—it's simplifying the ecosystem by consolidating functionality into a single, fast tool that handles package installation, virtual environments, version locking, and Python version management.",
      source: "Lesson 1: Why UV? Understanding Modern Python Package Management"
    },
    {
      question: "What performance advantage does UV offer compared to pip?",
      options: [
        "UV runs ten to one hundred times faster",
        "UV uses less disk space for packages",
        "UV automatically fixes all code bugs instantly",
        "UV works offline without internet connection required"
      ],
      correctOption: 0,
      explanation: "UV is 10-100x faster than pip because it's written in Rust (a systems programming language) rather than Python. For example, pip might take 8-12 seconds to install requests, while UV completes it in 0.5-2 seconds. On large projects with 50+ dependencies, pip can take 2+ minutes while UV finishes in under 10 seconds. This speed advantage comes from Rust's performance, not from using less disk space or offline capabilities. UV doesn't fix bugs automatically—it's a package manager, not a code analyzer.",
      source: "Lesson 1: Why UV? Understanding Modern Python Package Management"
    },
    {
      question: "When should you use conda instead of UV?",
      options: [
        "When building web applications with Flask or Django",
        "When working on any new Python project today",
        "When you want the fastest package installation available",
        "When project requires specialized data science packages conda manages"
      ],
      correctOption: 3,
      explanation: "Conda is recommended when you need specialized data science packages that conda's ecosystem manages better, particularly those with complex non-Python dependencies (like scientific computing libraries with C/Fortran components). For general Python projects, web apps, or new projects, UV is the modern default due to its speed and unified approach. UV handles standard PyPI packages excellently. Conda is slower than UV and was designed specifically for data science workflows. Flask and Django work perfectly with UV since they're standard Python packages available on PyPI.",
      source: "Lesson 1: Why UV? Understanding Modern Python Package Management"
    },
    {
      question: "What does the PATH environment variable do during UV installation?",
      options: [
        "It downloads UV source code from GitHub repositories",
        "It registers UV command locations for system access",
        "It creates virtual environments for all projects automatically",
        "It configures Python version to use for scripts"
      ],
      correctOption: 1,
      explanation: "PATH is your computer's registry of command locations. When you type 'uv --version', your system searches directories listed in PATH to find the UV executable. The installation script automatically adds UV's directory (like ~/.local/bin/) to your PATH so you can run 'uv' from anywhere. PATH doesn't download code, create environments, or manage Python versions—it simply tells your system where to find executable programs. Without PATH configured correctly, you'd get 'command not found' errors.",
      source: "Lesson 2: Installing UV with AI Collaboration"
    },
    {
      question: "When installing UV, what's the appropriate role for AI?",
      options: [
        "Never use AI since installation is too complex",
        "Always use AI to run every installation step",
        "Use AI for troubleshooting installation errors only",
        "Ask AI to rewrite installation scripts before running"
      ],
      correctOption: 2,
      explanation: "Installation is a simple, deterministic task—you run one platform-specific command and verify it worked. AI isn't needed for this straightforward process. However, AI becomes valuable when something goes wrong: permission errors, PATH configuration issues, or network problems. AI can diagnose platform-specific problems and suggest fixes. The pattern is: direct commands for simple tasks, AI for complex troubleshooting. You shouldn't use AI for every step (over-engineering), but you also shouldn't avoid it when genuinely stuck. AI doesn't need to rewrite installation scripts—the official scripts work correctly.",
      source: "Lesson 2: Installing UV with AI Collaboration"
    },
    {
      question: "What is a Python project in UV terms?",
      options: [
        "A folder with code and configuration files",
        "Only the Python scripts without any metadata",
        "A single file containing all project dependencies",
        "The virtual environment directory created by UV"
      ],
      correctOption: 0,
      explanation: "A Python project is a folder containing your code (Python files) and configuration files that tell UV what your code needs (pyproject.toml for dependencies, .python-version for Python version). It's not just scripts alone—the configuration is essential for reproducibility. It's not a single file; projects have multiple files organized in directories. The .venv/ virtual environment is part of the project structure but not the project itself—it's generated from the configuration and can be recreated anytime with 'uv sync'.",
      source: "Lesson 3: Creating Your First UV Project"
    },
    {
      question: "What role does pyproject.toml serve in a UV project?",
      options: [
        "It stores the actual installed package files",
        "It defines project metadata and lists dependencies",
        "It contains all Python source code for project",
        "It tracks Git commit history for the repository"
      ],
      correctOption: 1,
      explanation: "pyproject.toml is your project's configuration file, containing metadata (name, version, description) and a list of dependencies your project needs. When you run 'uv add requests', it's added to this file. This is the modern Python standard, replacing old requirements.txt files. It doesn't store actual package files—those go in .venv/. It doesn't contain your source code—that's in separate .py files. It doesn't track Git history—that's Git's job. Think of it as your project's contract with Python, declaring what it needs to run.",
      source: "Lesson 3: Creating Your First UV Project"
    },
    {
      question: "Why don't you commit the .venv directory to Git?",
      options: [
        "UV automatically deletes it before every Git commit",
        "Git cannot handle virtual environment files correctly",
        "Virtual environments contain secret API keys only",
        "It's large and teammates recreate it from lockfile"
      ],
      correctOption: 3,
      explanation: ".venv/ can be 50+ MB of dependencies—too large for Git and unnecessary. Teammates can recreate the exact same environment in seconds by running 'uv sync', which reads uv.lock to install identical package versions. Git handles virtual environment files fine technically, but there's no reason to bloat your repository. .venv/ doesn't contain secrets (those go in .env files). UV doesn't auto-delete it—.venv/ is in .gitignore by default. The key principle: commit specifications (pyproject.toml, uv.lock), not installations (.venv/).",
      source: "Lesson 3: Creating Your First UV Project"
    },
    {
      question: "What is a transitive dependency in UV?",
      options: [
        "A development dependency used only in testing environments",
        "A package you directly add with uv add",
        "A dependency of dependency installed automatically by UV",
        "A Python version requirement specified in dot file"
      ],
      correctOption: 2,
      explanation: "Transitive dependencies are 'dependencies of dependencies.' When you add 'requests', it needs 'urllib3', 'charset-normalizer', 'idna', and 'certifi' to work. Those are transitive dependencies—you didn't specify them, but UV installs them automatically because 'requests' requires them. This is different from direct dependencies (what you explicitly add). Development dependencies are a separate concept (dev-only vs production). Python version requirements are also separate. UV handles the entire dependency tree automatically, so you only specify what you directly need.",
      source: "Lesson 4: Managing Dependencies with AI"
    },
    {
      question: "Why mark pytest as a development dependency?",
      options: [
        "Production users don't need testing tools runtime",
        "Pytest runs faster in development mode only",
        "Development dependencies are free on PyPI always",
        "Pytest cannot be installed in production environments"
      ],
      correctOption: 0,
      explanation: "Testing tools like pytest are for developers writing and testing code, not for end users running your application in production. When you deploy a web app, users don't need pytest—they just need the app to run. Using --dev flag (uv add --dev pytest) separates these concerns. Production deploys with 'uv sync --no-dev' skip dev tools, keeping deployments lean, fast, and secure. Pytest doesn't run faster in dev mode—it's about whether it's needed at all. All PyPI packages cost nothing regardless of dependency type. Pytest can be installed anywhere; it's just unnecessary in production.",
      source: "Lesson 4: Managing Dependencies with AI"
    },
    {
      question: "In semantic versioning 2.31.5, what does the 5 represent?",
      options: [
        "Major version indicating breaking changes to API",
        "Patch version for bug fixes backward compatible",
        "Minor version with new features backward compatible",
        "Build number that increments with each release"
      ],
      correctOption: 1,
      explanation: "Semantic versioning uses MAJOR.MINOR.PATCH format (2.31.5). The PATCH (5) indicates bug fixes that are backward-compatible—your code won't break. MINOR (31) adds new features without breaking existing functionality. MAJOR (2) introduces breaking changes that might require code updates. The pattern helps you assess risk: updating from 2.31.5 to 2.31.6 (patch) is safe, 2.31.5 to 2.32.0 (minor) is usually safe, but 2.31.5 to 3.0.0 (major) might break your code. Build numbers exist in some systems but aren't part of semantic versioning's three-part structure.",
      source: "Lesson 4: Managing Dependencies with AI"
    },
    {
      question: "What does the uv.lock file guarantee for your team?",
      options: [
        "Security vulnerabilities are automatically fixed in packages",
        "All code changes are tracked with timestamps",
        "Project runs faster with optimized dependency loading",
        "Everyone gets identical package versions ensuring reproducibility"
      ],
      correctOption: 3,
      explanation: "uv.lock pins exact versions of every package, including transitive dependencies. When you have requests==2.31.0 and your teammate clones the project months later (when 2.33.0 exists), running 'uv sync' installs 2.31.0—your exact version. This prevents 'works on my machine' bugs caused by version mismatches. The lockfile doesn't track code changes (that's Git's job), doesn't optimize performance (just ensures consistency), and doesn't fix security issues (you still need to update packages when vulnerabilities are found). Its sole purpose: reproducibility across developers, time, and environments.",
      source: "Lesson 4: Managing Dependencies with AI"
    },
    {
      question: "What happens when two packages need conflicting dependency versions?",
      options: [
        "UV finds compatible versions or reports conflict",
        "UV installs both versions in separate environments",
        "UV always chooses the newest version available",
        "UV blocks all installations until you downgrade"
      ],
      correctOption: 0,
      explanation: "UV performs dependency resolution: if Package A needs requests>=2.30.0 and Package B needs requests<3.0.0, UV finds a version satisfying both (like 2.32.0). If requirements are truly incompatible (A needs >=3.0, B needs <3.0), UV reports the conflict and suggests solutions (update one package, use older version, find alternative). UV doesn't install multiple versions of the same package in one environment—that would cause import chaos. It doesn't blindly choose the newest—it respects version constraints. It doesn't block everything—it only fails when constraints are mathematically unsatisfiable.",
      source: "Lesson 4: Managing Dependencies with AI"
    },
    {
      question: "What does 'uv run python script.py' do differently than 'python script.py'?",
      options: [
        "It checks code for syntax errors first",
        "It runs code ten times faster using compilation",
        "It activates project environment automatically before running",
        "It creates backup of script before execution"
      ],
      correctOption: 2,
      explanation: "'uv run' automatically activates your project's virtual environment, giving Python access to project-specific packages. Without 'uv run', you're using system Python, which doesn't have your project's dependencies—you'd get ModuleNotFoundError. This automatic activation eliminates manual 'source .venv/bin/activate' commands. 'uv run' doesn't make code faster (same Python interpreter), doesn't check syntax (Python does that when parsing), and doesn't create backups (that's not a package manager's job). The sole purpose: seamlessly run code in project-isolated environments without manual activation steps.",
      source: "Lesson 5: Running Python Code in UV Projects with AI"
    },
    {
      question: "Why do virtual environments prevent dependency conflicts?",
      options: [
        "Virtual environments download packages from secure mirrors",
        "Each project gets separate isolated package versions",
        "They compress dependencies to save disk space",
        "They automatically update packages to latest versions"
      ],
      correctOption: 1,
      explanation: "Virtual environments give each project its own 'toolbox' of packages. Project A can have requests 2.28.0 while Project B has 2.31.0—no conflict because they're in separate .venv/ directories. Without isolation, installing version 2.31 for Project B would overwrite Project A's 2.28, breaking Project A. Virtual environments don't provide secure mirrors (that's PyPI's job), don't compress packages (they're installed normally), and don't auto-update (you control updates). The core concept: isolation prevents one project's changes from affecting another.",
      source: "Lesson 5: Running Python Code in UV Projects with AI"
    },
    {
      question: "How do you run pytest tests in a UV project?",
      options: [
        "Configure pytest to auto detect UV environments",
        "Install pytest globally then run pytest directly",
        "Type python pytest without any environment activation",
        "Use uv run pytest to activate environment"
      ],
      correctOption: 3,
      explanation: "'uv run pytest' activates your project environment (where pytest is installed with 'uv add --dev pytest') and runs tests. Installing pytest globally is the old way and defeats environment isolation—you'd have one pytest version for all projects. Running 'python pytest' would fail—pytest is a command-line tool, not a Python module. Pytest doesn't auto-detect UV environments—you explicitly activate with 'uv run'. This pattern applies to all commands: 'uv run' ensures project isolation, whether running scripts, tests, or servers.",
      source: "Lesson 5: Running Python Code in UV Projects with AI"
    },
    {
      question: "What's the difference between pyproject.toml and uv.lock?",
      options: [
        "Pyproject is optional; lockfile is always required",
        "Pyproject is for Python; lockfile for JavaScript",
        "Pyproject has constraints; lockfile has exact versions",
        "Pyproject tracks code; lockfile tracks Git changes"
      ],
      correctOption: 2,
      explanation: "pyproject.toml specifies constraints ('requests>=2.31.0'—version 2.31 or higher is fine), while uv.lock pins exact versions ('requests==2.32.1'—exactly this version). pyproject.toml is flexible for development; uv.lock is strict for reproducibility. Both are Python-specific (no JavaScript). pyproject.toml is required (defines your project); uv.lock is generated automatically. Neither tracks code or Git changes—that's Git's job. Think of it as recipe (pyproject.toml: roughly 2-3 cups flour) vs. exact meal (uv.lock: exactly 2.5 cups King Arthur flour, batch #2024-10).",
      source: "Lesson 6: Team Collaboration and Reproducible Environments"
    },
    {
      question: "When a teammate clones your UV project, what should they run first?",
      options: [
        "Run uv sync to recreate exact environment",
        "Run uv init to create new project",
        "Run pip install requirements dot text file",
        "Run python setup dot py install command"
      ],
      correctOption: 0,
      explanation: "'uv sync' reads uv.lock and recreates your exact environment—same package versions, same Python version. This takes seconds and guarantees reproducibility. 'uv init' creates a new project from scratch—wrong for cloning existing projects. 'pip install -r requirements.txt' is the old way and ignores UV's lockfile. 'python setup.py install' is deprecated and unrelated to UV. The modern workflow: clone repo, run 'uv sync', verify with 'uv run pytest'. If tests pass, the environment is correct and development can begin.",
      source: "Lesson 6: Team Collaboration and Reproducible Environments"
    },
    {
      question: "Why should both pyproject.toml and uv.lock be committed to Git?",
      options: [
        "Git requires both files for Python projects",
        "Pyproject defines needs; lockfile ensures exact reproducibility",
        "Lockfiles contain secret keys needed for deployment",
        "Both files store different Python source code"
      ],
      correctOption: 1,
      explanation: "pyproject.toml declares what your project needs (dependencies, metadata), while uv.lock ensures everyone gets the exact same versions. Both are essential for team collaboration: pyproject.toml for specifications, uv.lock for reproducibility. Git doesn't require them technically—you choose what to commit. Lockfiles don't contain secrets (secrets go in .env, which is NOT committed). Neither file contains source code—they're configuration files. If you only commit pyproject.toml, teammates might get different versions months later, causing 'works on my machine' bugs. Commit both for reliable team workflows.",
      source: "Lesson 6: Team Collaboration and Reproducible Environments"
    },
    {
      question: "How do you deploy to production without development dependencies?",
      options: [
        "Copy only dot venv directory to server",
        "Delete pyproject toml before deploying to servers",
        "Manually uninstall pytest and black from environment",
        "Run uv sync no dev flag"
      ],
      correctOption: 3,
      explanation: "'uv sync --no-dev' installs only production dependencies (requests, flask, etc.) and skips dev tools (pytest, black, etc.). This keeps production lean, fast, and secure—no unnecessary packages. Deleting pyproject.toml would break the deployment (you need configuration). Manually uninstalling is inefficient and error-prone. Copying .venv/ to servers is bad practice—you should rebuild environments on deployment servers using 'uv sync --no-dev' to ensure clean, reproducible deployments. The pattern: dev has everything ('uv sync'), production has only what's needed ('uv sync --no-dev').",
      source: "Lesson 6: Team Collaboration and Reproducible Environments"
    },
    {
      question: "What happens if you encounter a Git conflict in uv.lock?",
      options: [
        "Run uv lock to regenerate with all changes",
        "Manually edit lockfile to merge conflicts by hand",
        "Delete lockfile and continue without it completely",
        "Copy your version over teammate version always"
      ],
      correctOption: 0,
      explanation: "Never manually edit uv.lock—it's auto-generated and complex. When Git shows uv.lock conflicts (you added package-a, teammate added package-b), run 'git merge --abort', 'git pull', then 'uv lock' to regenerate the lockfile with all changes. UV resolves both additions automatically. Manual editing risks corrupting the file. Deleting the lockfile loses reproducibility—bad idea. Blindly using your version ignores teammate's changes—also wrong. The key: let UV regenerate lockfiles; they're not human-editable files.",
      source: "Lesson 6: Team Collaboration and Reproducible Environments"
    },
    {
      question: "What is Ruff and what problem does it solve?",
      options: [
        "Testing framework replacement for pytest and unittest",
        "Package manager alternative to UV for dependencies",
        "Unified linter and formatter replacing three tools",
        "Python version manager handling multiple Python installations"
      ],
      correctOption: 2,
      explanation: "Ruff is a single tool that replaces three separate tools: Black (formatter), Flake8 (linter), and isort (import sorter). Before Ruff, you needed all three to write clean code. Ruff does the job of all three, 10-100x faster because it's written in Rust. It finds code errors (unused imports, style violations) and fixes formatting (spacing, quotes, indentation). Ruff isn't a package manager (that's UV), testing framework (that's pytest), or version manager (that's pyenv/UV's Python management). It's specialized for code quality: formatting + linting in one fast tool.",
      source: "Lesson 8: Ruff Linter & Formatter Basics"
    },
    {
      question: "What's the difference between formatting and linting?",
      options: [
        "Formatting compiles code; linting executes for errors",
        "Formatting checks types; linting removes unused code",
        "Formatting runs tests; linting generates documentation automatically",
        "Formatting fixes style; linting finds likely bugs"
      ],
      correctOption: 3,
      explanation: "Formatting makes code look consistent: fixing spacing ('x=1' → 'x = 1'), standardizing quotes, proper indentation. Linting finds likely bugs and style problems: unused imports, unused variables, lines too long, potential errors like using assignment (=) instead of comparison (==). Formatting is about appearance; linting is about correctness. Neither checks types (that's Pyright), generates docs (that's Sphinx), compiles code (Python is interpreted), or runs tests (that's pytest). Ruff does both: 'ruff format' for formatting, 'ruff check' for linting.",
      source: "Lesson 8: Ruff Linter & Formatter Basics"
    },
    {
      question: "How do you automatically fix code formatting with Ruff?",
      options: [
        "Run uv run ruff check with fix argument",
        "Run uv run ruff format dot command",
        "Edit pyproject toml to enable auto format",
        "Install Ruff extension in Zed for magic"
      ],
      correctOption: 1,
      explanation: "'uv run ruff format .' formats all Python files in your project, fixing spacing, quotes, indentation automatically. The dot (.) means 'current directory and subdirectories.' 'ruff check' is for linting (finding problems), not formatting—adding --fix to 'ruff check --fix' auto-fixes linting errors but that's different from formatting. Editing pyproject.toml configures rules but doesn't run formatting. Zed extensions can format-on-save, but the manual command is 'uv run ruff format .'. Formatting is a command you run, not magic configuration.",
      source: "Lesson 8: Ruff Linter & Formatter Basics"
    },
    {
      question: "What does 'ruff check' identify that 'ruff format' doesn't?",
      options: [
        "Unused imports and variables that cause bugs",
        "Incorrect spacing and quote style inconsistencies only",
        "Missing semicolons and incorrect indentation levels always",
        "Type errors and missing function return annotations"
      ],
      correctOption: 0,
      explanation: "'ruff check' (linting) finds bugs and code smells: unused imports (F401), unused variables, lines too long, potential bugs. 'ruff format' (formatting) only fixes style: spacing, quotes, indentation. Linting catches logical problems; formatting handles aesthetics. Python doesn't use semicolons (that's JavaScript)—not relevant here. Type errors are caught by Pyright (type checker), not Ruff. Ruff doesn't enforce type annotations unless you enable specific rules. The key distinction: check = find problems, format = fix appearance.",
      source: "Lesson 8: Ruff Linter & Formatter Basics"
    },
    {
      question: "Where do you configure Ruff rules for your team?",
      options: [
        "In dot env file with secret keys",
        "In global Ruff config in home directory",
        "In pyproject toml under tool ruff section",
        "In requirements text for pip installation"
      ],
      correctOption: 2,
      explanation: "Ruff configuration goes in pyproject.toml under [tool.ruff] section. This makes configuration version-controlled (part of Git repo), so all teammates follow identical rules automatically. Configuring globally (~/.config/ruff/) affects all projects—bad for team consistency. .env is for secrets, not tool configuration. requirements.txt is the old way of listing dependencies, unrelated to Ruff config. The pattern: tool configuration in pyproject.toml (Ruff, pytest, etc.) ensures teams share identical settings without manual setup.",
      source: "Lesson 9: Ruff Configuration with AI"
    },
    {
      question: "Why use AI to generate Ruff configuration?",
      options: [
        "AI replaces Ruff completely with better tool",
        "AI makes Ruff run faster with optimization",
        "AI prevents all linting errors automatically beforehand",
        "AI creates config without memorizing TOML syntax"
      ],
      correctOption: 3,
      explanation: "AI helps you generate Ruff configuration in TOML format without memorizing syntax. You describe what you want ('strict mode with 100-character lines, ignore E501'), AI produces the [tool.ruff] section you paste into pyproject.toml. AI doesn't make Ruff faster (Rust already does that), doesn't prevent errors (Ruff finds them after you write code), and doesn't replace Ruff (AI generates config for Ruff). This is the AI-native pattern: express intent, let AI handle implementation details. You understand what to configure, AI handles how.",
      source: "Lesson 9: Ruff Configuration with AI"
    },
    {
      question: "What does Pyright catch that Ruff doesn't?",
      options: [
        "Formatting inconsistencies in code style only",
        "Type mismatches before running code at all",
        "Unused imports and variables in functions always",
        "Security vulnerabilities in third party dependencies"
      ],
      correctOption: 1,
      explanation: "Pyright is a static type checker that catches type mismatches before code runs: passing int when function expects str, calling non-existent methods, wrong argument counts. Ruff handles formatting and linting (style, unused imports), not types. While Ruff can find unused imports, that's linting, not type checking. Security vulnerability scanning is a different tool (like Safety or Snyk), not Pyright. Pyright's specialty: static analysis of types using Python's type hints (def greet(name: str) -> str). It catches ~30-50% of bugs without running code—purely by analyzing types.",
      source: "Lesson 10: Pyright Type Checker"
    },
    {
      question: "Why add type hints to Python code?",
      options: [
        "Type hints are required by Python interpreter",
        "Make Python code run faster with optimization",
        "Catch bugs statically without running code first",
        "They automatically generate unit tests for functions"
      ],
      correctOption: 2,
      explanation: "Type hints let tools like Pyright catch type errors before running code. If you pass greet(42) when the function expects greet(name: str), Pyright errors immediately—no need to run and debug. This catches 30-50% of bugs statically. Type hints don't make code faster (Python ignores them at runtime), aren't required (Python runs with or without them), and don't generate tests (that's a different tool). They're optional annotations for static analysis tools. Dynamic Python becomes partly static, catching errors during development rather than production.",
      source: "Lesson 10: Pyright Type Checker"
    },
    {
      question: "What command installs Pyright as a development dependency?",
      options: [
        "Run uv add dev pyright command",
        "Run pip install pyright globally for all",
        "Run uv run pyright to auto install",
        "Add pyright to production dependencies in toml"
      ],
      correctOption: 0,
      explanation: "'uv add --dev pyright' installs Pyright as a development dependency (for type checking during development, not needed in production). Installing globally with pip defeats environment isolation. 'uv run pyright' runs Pyright but doesn't install it—you'd get an error if it's not installed. Adding to production dependencies wastes space—type checkers aren't needed at runtime. The pattern: linters, formatters, type checkers, test frameworks go in dev dependencies (--dev flag). Production only gets runtime requirements.",
      source: "Lesson 10: Pyright Type Checker"
    },
    {
      question: "In a UV project, which files should you commit to Git?",
      options: [
        "All files including pycache and compiled bytecode",
        "Only dot venv directory with packages installed",
        "Only source code files without configuration ever",
        "Pyproject toml, uv lock, and python version"
      ],
      correctOption: 3,
      explanation: "Commit pyproject.toml (project configuration), uv.lock (exact versions), and .python-version (Python version specification). These let teammates recreate your exact environment. Don't commit .venv/ (50+ MB, recreated with 'uv sync'), __pycache__/ (Python cache, auto-generated), or .pyc files (compiled bytecode, auto-generated). You must commit source code—that's your project! UV creates .gitignore automatically with correct exclusions. The principle: commit specifications and code, not generated artifacts.",
      source: "Lesson 6: Team Collaboration and Reproducible Environments"
    },
    {
      question: "When should you update the uv.lock file?",
      options: [
        "Every day to get latest package versions",
        "When adding, updating, or removing any package",
        "Only when Git shows merge conflicts manually",
        "Never because UV generates it automatically forever"
      ],
      correctOption: 1,
      explanation: "uv.lock updates automatically when you 'uv add package-name' (adds new), 'uv add package@latest' (updates existing), or 'uv remove package-name' (removes). You don't update it daily—that would cause unnecessary churn. If Git shows conflicts, you regenerate with 'uv lock', not manually edit. While UV generates it automatically, you trigger updates by managing dependencies. The lockfile reflects your current dependency state—every dependency change updates it. After updating, commit the new lockfile so teammates stay synchronized.",
      source: "Lesson 4: Managing Dependencies with AI"
    },
    {
      question: "What does 'uv init my-project' create?",
      options: [
        "Git repository with full commit history included",
        "Virtual environment with all PyPI packages preinstalled",
        "Project folder with pyproject toml and structure",
        "Python script that prints hello world message"
      ],
      correctOption: 2,
      explanation: "'uv init my-project' creates a project folder with: pyproject.toml (configuration), .python-version (Python version), README.md (description), main.py (starter script), and .gitignore (exclusions). It also initializes a Git repository automatically. It doesn't create .venv/ yet (that happens when you add dependencies), doesn't preinstall PyPI packages (you add what you need), and does include Git initialization but not commit history (that comes from your commits). It's project scaffolding, not a complete application.",
      source: "Lesson 3: Creating Your First UV Project"
    },
    {
      question: "Why is UV faster than pip and poetry?",
      options: [
        "UV is written in Rust not Python",
        "UV downloads smaller compressed package versions always",
        "UV skips dependency resolution for speed boost",
        "UV caches packages in cloud storage offsite"
      ],
      correctOption: 0,
      explanation: "UV is 10-100x faster because it's written in Rust, a systems programming language designed for performance, rather than Python. Pip and Poetry are written in Python—like using a hand shovel vs. power shovel. UV doesn't download smaller packages (same packages from PyPI), doesn't skip resolution (still resolves dependencies correctly), and doesn't use cloud caching (uses local disk cache like other tools). The speed comes purely from Rust's performance advantages. Same job, different implementation language, dramatically different speed.",
      source: "Lesson 1: Why UV? Understanding Modern Python Package Management"
    },
    {
      question: "What is the purpose of .python-version file?",
      options: [
        "It stores Python source code for project",
        "It specifies which Python version project requires",
        "It lists all package versions for dependencies",
        "It tracks Python deprecation warnings during runtime"
      ],
      correctOption: 1,
      explanation: ".python-version contains just the Python version specification (like '3.12'). When teammates clone your project, UV knows which Python version to use, ensuring consistency. It doesn't contain source code (that's in .py files), doesn't list package versions (that's pyproject.toml and uv.lock), and doesn't track warnings (that's runtime behavior). This file works with UV's Python version management: if you have 3.12 and a teammate has 3.11, UV can install 3.12 for them automatically. Single source of truth for Python version requirements.",
      source: "Lesson 3: Creating Your First UV Project"
    },
    {
      question: "How do you add multiple packages in one command?",
      options: [
        "Edit lockfile to include packages manually first",
        "Run uv add then list packages in toml",
        "Run each uv add command separately always",
        "Run uv add requests flask pandas together"
      ],
      correctOption: 3,
      explanation: "'uv add requests flask pandas' adds all three packages at once, resolving dependencies together. This is more efficient than three separate 'uv add' commands. You don't list packages in toml first—'uv add' updates toml automatically. Running separately works but wastes time. Never manually edit the lockfile—UV generates it. The pattern: 'uv add' handles multiple packages, updates pyproject.toml and uv.lock, resolves dependencies, and installs everything in one operation.",
      source: "Lesson 4: Managing Dependencies with AI"
    },
    {
      question: "What happens when you run 'uv sync' in a cloned project?",
      options: [
        "UV creates venv and installs locked versions",
        "UV upgrades all packages to latest available",
        "UV pushes your environment to remote Git",
        "UV compiles Python code to machine bytecode"
      ],
      correctOption: 0,
      explanation: "'uv sync' reads uv.lock and creates .venv/ with exact package versions specified in the lockfile. This recreates the original developer's environment perfectly. It doesn't upgrade packages (that's 'uv lock --upgrade'), doesn't push to Git (that's 'git push'), and doesn't compile code (Python compiles .py → .pyc automatically at runtime). 'uv sync' is the onboarding command: clone repo, run 'uv sync', verify with 'uv run pytest'. If tests pass, environment is correct and you're ready to develop.",
      source: "Lesson 6: Team Collaboration and Reproducible Environments"
    },
    {
      question: "What does 'uv remove package-name' do?",
      options: [
        "Marks package as deprecated in PyPI registry",
        "Only deletes package from venv directory locally",
        "Removes package and updates toml and lockfile",
        "Uninstalls package globally from system Python entirely"
      ],
      correctOption: 2,
      explanation: "'uv remove package-name' removes the package from pyproject.toml, updates uv.lock, and uninstalls it from .venv/. It's a complete removal: specification + installation + lockfile. It doesn't just delete from .venv/ (that would leave it in toml), doesn't affect PyPI (you can't deprecate packages you don't own), and doesn't touch system Python (UV works in isolated environments). If the package was only needed by the removed package (transitive dependency no longer needed), UV removes that too. Clean, complete removal.",
      source: "Lesson 4: Managing Dependencies with AI"
    },
    {
      question: "How does UV handle Python version management?",
      options: [
        "UV converts Python code to single version",
        "UV only works with system installed Python",
        "UV requires pyenv for all Python installations",
        "UV can install and manage multiple Python versions"
      ],
      correctOption: 3,
      explanation: "UV includes Python version management: it can install and manage multiple Python versions, similar to pyenv. When you specify 'requires-python = \">=3.12\"' in pyproject.toml and .python-version contains '3.12', UV ensures that version is used. If it's not installed, UV can install it. UV doesn't require system Python or pyenv—it's self-sufficient. UV doesn't convert code between Python versions—it manages which interpreter runs your code. This unification (package management + environment management + Python version management) is why UV replaces multiple tools.",
      source: "Lesson 1: Why UV? Understanding Modern Python Package Management"
    },
    {
      question: "Why would you use 'uv add package@latest'?",
      options: [
        "To install package without version locking ever",
        "To update specific package to newest version",
        "To download package source code for inspection",
        "To mark package as latest in toml"
      ],
      correctOption: 1,
      explanation: "'uv add package@latest' updates a specific package to its newest version. If you have requests==2.31.0 and want 2.32.0, this command updates it. UV still locks the version in uv.lock (reproducibility is maintained). It doesn't skip version locking—UV always locks. It doesn't download source for inspection (that's 'uv pip download --no-binary'). It doesn't just mark as latest in toml—it actually updates the installation and lockfile. Use when you need new features, security fixes, or bug fixes from a package update.",
      source: "Lesson 4: Managing Dependencies with AI"
    },
    {
      question: "What is the relationship between UV and pip?",
      options: [
        "UV and pip must be used together",
        "UV requires pip to function correctly always",
        "UV is modern replacement for pip installation",
        "UV converts pip commands to UV format"
      ],
      correctOption: 2,
      explanation: "UV replaces pip for package installation, offering 10-100x faster installation plus virtual environment management, locking, and Python version management. UV doesn't require pip—it's a standalone tool. You don't use them together; you choose UV instead of pip for modern projects. UV doesn't convert pip commands—it has its own command structure ('uv add' vs 'pip install'). UV is pip's spiritual successor, learning from pip's limitations and delivering a unified, fast solution. For new projects in 2025, UV is the recommended default.",
      source: "Lesson 1: Why UV? Understanding Modern Python Package Management"
    },
    {
      question: "When running commands with 'uv run', what happens automatically?",
      options: [
        "Project virtual environment activates automatically before execution",
        "Code is compiled to machine language first",
        "All tests run to verify correctness immediately",
        "Git commit creates with command output logged"
      ],
      correctOption: 0,
      explanation: "'uv run' automatically activates your project's .venv/ before running the command, eliminating manual 'source .venv/bin/activate' steps. You just 'uv run python script.py' or 'uv run pytest' and environment activation happens invisibly. It doesn't compile code (Python is interpreted), doesn't run tests (unless you explicitly 'uv run pytest'), and doesn't create Git commits (that's 'git commit'). The magic is seamless environment activation: you think about what to run, not environment management. Professional workflow without friction.",
      source: "Lesson 5: Running Python Code in UV Projects with AI"
    },
    {
      question: "What error indicates missing dependency in your UV project?",
      options: [
        "PermissionDenied when running uv run command",
        "SyntaxError in Python code structure always",
        "TypeMismatchError from Pyright type checker only",
        "ModuleNotFoundError when importing the package name"
      ],
      correctOption: 3,
      explanation: "ModuleNotFoundError (like 'No module named requests') means Python can't find the package—either it's not installed or you're not using 'uv run' to activate the environment. Fix: 'uv add requests' then 'uv run python script.py'. SyntaxError is code structure problem, unrelated to dependencies. TypeMismatchError is from Pyright (type checking), not missing packages. PermissionDenied is file system permission issue. ModuleNotFoundError is the diagnostic signature for missing dependencies.",
      source: "Lesson 5: Running Python Code in UV Projects with AI"
    },
    {
      question: "How do you verify UV installation worked correctly?",
      options: [
        "Check if dot venv directory exists anywhere",
        "Run uv version to see version number",
        "Run python to verify interpreter updated successfully",
        "Look for UV logo in terminal prompt"
      ],
      correctOption: 1,
      explanation: "'uv --version' (or 'uv version') displays UV's version number if installation succeeded. If you see 'uv 0.5.9', it worked. If you see 'command not found', PATH isn't configured. .venv/ doesn't exist until you create a project and add dependencies. Running 'python' tests Python, not UV—they're separate. Terminal prompts don't show UV logos (UV isn't a shell). The verification pattern for CLI tools: run 'tool --version'. Success = version number. Failure = 'command not found' → troubleshoot PATH.",
      source: "Lesson 2: Installing UV with AI Collaboration"
    },
    {
      question: "What makes Ruff different from Black and Flake8?",
      options: [
        "Ruff combines both roles in one tool",
        "Ruff only formats code without any linting",
        "Ruff requires manual installation of both tools",
        "Ruff works only with TypeScript not Python"
      ],
      correctOption: 0,
      explanation: "Ruff combines formatting (Black's job) and linting (Flake8's job) plus import sorting (isort's job) in one tool. Before Ruff, you needed three separate tools. Ruff doesn't only format—it does format + lint. It replaces Black/Flake8/isort, not requires them. Ruff is Python-specific (not TypeScript—that's ESLint/Prettier). The unification is Ruff's core value: one tool, one command structure ('ruff format', 'ruff check'), 10-100x faster than the three-tool workflow. Written in Rust for performance.",
      source: "Lesson 8: Ruff Linter & Formatter Basics"
    },
    {
      question: "What command shows your project's dependency tree?",
      options: [
        "Read pyproject toml to see tree",
        "Run uv list to show all packages",
        "Run uv tree to display dependencies",
        "Run uv deps tree command always"
      ],
      correctOption: 2,
      explanation: "'uv tree' displays your dependency tree: top-level packages you directly added, plus nested transitive dependencies each requires. Visual representation shows why installing 'requests' brings in 'urllib3', 'certifi', etc. There's no 'uv list' or 'uv deps tree' command (though 'uv pip list' shows installed packages without tree structure). Reading pyproject.toml shows what you directly added, not the full tree of transitive dependencies. 'uv tree' is the diagnostic command for understanding dependency relationships.",
      source: "Lesson 4: Managing Dependencies with AI"
    },
    {
      question: "What does 'ruff check --fix' do?",
      options: [
        "Manually prompts for each error requiring user input",
        "Automatically fixes linting errors when possible safely",
        "Formats code style like ruff format command",
        "Runs tests to verify fixes work correctly"
      ],
      correctOption: 1,
      explanation: "'ruff check --fix' automatically fixes linting errors that are safe to auto-fix: removing unused imports, fixing obvious style violations. Some errors require manual intervention (complex logic bugs). It doesn't prompt for each fix—it applies safe fixes automatically. It's not the same as 'ruff format' (that's for style formatting, not linting errors). It doesn't run tests (that's 'pytest'). The workflow: 'ruff check' (see errors) → 'ruff check --fix' (auto-fix safe ones) → manually fix remaining → verify with 'ruff check' (should show zero errors).",
      source: "Lesson 8: Ruff Linter & Formatter Basics"
    },
    {
      question: "How do you configure Ruff to allow longer lines?",
      options: [
        "Install Ruff extension with longer line version",
        "Run ruff with line length flag every time",
        "Edit Ruff source code to change defaults",
        "Add line length config in pyproject toml"
      ],
      correctOption: 3,
      explanation: "Add 'line-length = 120' in [tool.ruff] section of pyproject.toml. This makes the setting permanent and version-controlled—all teammates get it automatically. Running with flags every time ('ruff check --line-length 120') works but isn't persistent. Editing Ruff source code is impossible (and wrong—it's a third-party tool). There's no extension for longer lines—it's a config setting. The pattern: configuration goes in pyproject.toml (persistent, team-shared), not command-line flags (ephemeral, personal).",
      source: "Lesson 9: Ruff Configuration with AI"
    },
    {
      question: "What does semantic versioning constraint ^2.31.0 mean?",
      options: [
        "Allow version two point thirty one or higher below three",
        "Exactly version two point thirty one point zero only",
        "Any version above two point thirty one including three",
        "Block all versions containing two point thirty one"
      ],
      correctOption: 0,
      explanation: "^2.31.0 means 'version 2.31.0 or higher, but not 3.0.0 or above' (stays within major version 2). Minor and patch updates are allowed (2.32.0, 2.31.1), but major version changes (3.0.0) are blocked since they might break compatibility. This isn't exact (that's ==2.31.0), isn't unlimited (that's >=2.31.0 allowing 3.x), and doesn't block the specified version. The caret (^) balances safety (block breaking changes) with flexibility (allow safe updates). Common in dependency specifications for libraries.",
      source: "Lesson 4: Managing Dependencies with AI"
    },
    {
      question: "What should you do if 'uv sync' fails with dependency errors?",
      options: [
        "Manually edit pyproject toml to fix version",
        "Delete lockfile and reinstall everything from scratch completely",
        "Use AI to diagnose conflict and suggest resolutions",
        "Ignore error and continue with broken environment"
      ],
      correctOption: 2,
      explanation: "When 'uv sync' fails with dependency conflicts, use AI to analyze the error message and suggest solutions. AI can explain which packages conflict, why constraints are incompatible, and propose fixes (update specific packages, use older versions, find alternatives). Deleting the lockfile loses reproducibility and doesn't solve the underlying conflict. Manually editing pyproject.toml might work but requires deep understanding of the conflict. Ignoring errors leaves your environment broken—you can't run code successfully. The AI collaboration pattern: let AI diagnose complex problems and explain tradeoffs before you choose a solution.",
      source: "Lesson 4: Managing Dependencies with AI"
    }
  ]}
/>
