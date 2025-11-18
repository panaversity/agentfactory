# Chapter 12: The Lightning Python Stack — Lesson Plan

**Chapter Type**: Technical/Tool-Focused
**Chapter Objective(s)**:
- Understand why and when to use Zed IDE, Ruff linter/formatter, and Pyright type checker
- Set up a complete professional Python development toolchain (uv + Zed + Ruff + Pyright)
- Use AI assistance to configure complex tools without memorizing syntax
- Integrate all tools into a cohesive workflow with quality gates

**Estimated Total Time**: 150 minutes (6 new lessons × 20-30 min each)
**Part**: 4 (Python Fundamentals)
**Status**: Planning Phase for Lessons 7-12 (Lessons 1-6 uv content already exists)

**Prerequisite Chapters**:
- Chapter 7 (Bash Essentials) — Terminal commands, PATH, environment variables
- Chapter 8 (Git and GitHub) — Version control basics, .gitignore
- Chapter 11 (Context Engineering) — AI collaboration patterns, effective prompting
- Chapter 12, Lessons 1-6 — uv package manager fundamentals

**Success Criteria Alignment** (from spec.md):
- SC-001: 80%+ of readers can explain why each tool exists and when to use it
- SC-002: 75%+ of readers successfully set up complete Lightning Python Stack
- SC-003: 85%+ of readers can create a new Python project with all tools integrated
- SC-004: 70%+ of readers can troubleshoot common integration issues using AI
- SC-005: 80%+ demonstrate understanding when to use direct commands vs. AI assistance
- SC-006: Readers use AI for complex configurations rather than memorizing syntax
- SC-007: Chapter maintains B1 cognitive load limits (≤7 new concepts per section)
- SC-008: Realistic lesson durations, no inflation
- SC-009: All tool-specific examples use verified facts (100% verified via intelligence doc)
- SC-010: Smooth transition from uv (lessons 1-6) to integrated toolchain (lessons 7-12)
- SC-011: Readers understand progression: foundational uv → IDE → code quality → integration
- SC-012: Readers know pytest, pre-commit, MkDocs exist and have working config examples

---

## Section Architecture: Lessons 7-12

### Lesson 7: Zed IDE for AI-First Python Development (20 min)

**Objective** (Bloom's: Understand/Apply): Readers will be able to install Zed IDE, open a uv project, use integrated terminal, and understand why Zed is "AI-first"

**Skills Taught**:
1. **Install and Configure Zed IDE** — A2 — Technical — Apply
   - Measurable: Student installs Zed on their platform and opens a Python project

2. **Use Integrated Terminal in IDE** — A1 — Technical — Remember
   - Measurable: Student opens terminal in Zed and runs a command

3. **Understand LSP and Code Intelligence** — A2 — Conceptual — Understand
   - Measurable: Student explains what LSP is and why it powers code intelligence (inline errors, autocomplete)

**Key Concepts** (≤7 per B1 tier):
1. IDE purpose (editor + debugging + collaboration)
2. Zed advantages (speed, AI integration, minimal overhead)
3. LSP (Language Server Protocol) as abstraction
4. Opening projects from terminal
5. Integrated terminal vs. external terminal
6. File navigation in tree view
7. Extensions/plugins for Python support

**Prerequisites**:
- Bash terminal familiarity (Chapter 7)
- uv project structure knowledge (Chapter 12, Lesson 2)
- Basic file navigation understanding

**Duration**: 20 minutes (estimated)

**Content Outline**:
- **Section 1: Why Zed?** (2 min)
  - Brief history: VS Code, Sublime, modern IDEs
  - Why Zed matters: speed + AI-first design
  - Comparison: Zed vs VS Code (both good, different philosophies)

- **Section 2: Install Zed** (5 min)
  - Download link (zed.dev)
  - Platform-specific installation (macOS, Windows, Linux)
  - Verify installation: `zed --version`

- **Section 3: Open a Python Project** (3 min)
  - Create/use existing uv project
  - Open in Zed: `zed .` command
  - Show project tree, settings, menu navigation

- **Section 4: Integrated Terminal** (3 min)
  - Open integrated terminal in Zed
  - Run `uv run python --version` to verify
  - Show difference from external terminal (context awareness)

- **Section 5: Basic Keybindings** (4 min)
  - Command palette: Cmd+Shift+P (Mac) / Ctrl+Shift+P (Linux/Windows)
  - Find file: Cmd+P
  - Show code folding, minimap, breadcrumb

- **Section 6: LSP Configuration Preview** (3 min)
  - Mention: Settings will configure Pyright/Ruff LSPs in later lessons
  - Show `.zed/settings.json` file structure (without deep dive)

**Content Elements**:
- Minimal code: Just open an existing Python file with simple code
- Example file (if needed):
  ```python
  # Simple Python file to verify Zed opens correctly
  print("Hello from Zed!")
  ```
- Screenshots showing: Project tree, integrated terminal, command palette
- Platform-specific notes (Windows WSL, macOS native, Linux distros)

**Practice Approach**:
- **Exercise 1 (Direct)**: Install Zed, open a uv project, run a command in integrated terminal
- **Acceptance Criteria**: Student opens Zed, navigates project, sees file tree and runs `uv --version` successfully
- **Troubleshooting**: LSP errors ignored for now (covered in lesson 10)

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code (Tier 1/2 mix; for basic editing in Zed + orchestration)
- **Tier 1 Direct Prompt**: "Install Zed IDE from zed.dev and open your current uv project with `zed .`"
- **Tier 2 AI Companion Prompt**: "I want to customize Zed's keybindings for faster Python navigation. What changes should I make to `.zed/settings.json`?"
- **Tier 3 Orchestration Prompt**: "Set up Zed the same way on two different computers with identical settings and extensions"
- **Expected Outcome**: Zed installed, project opens, terminal works; reader understands LSP is coming next
- **Safety Note**: None (IDE installation is safe; security considerations deferred to broader toolchain lesson)

**Cognitive Load Assessment**: 7 concepts, all within B1 limit (A2 tier acceptable for Part 4 content)

---

### Lesson 8: Ruff Linter & Formatter Basics (15 min)

**Objective** (Bloom's: Understand/Apply): Readers will be able to install Ruff via uv, run formatting and linting commands, and understand why Ruff replaces three separate tools

**Skills Taught**:
1. **Install Ruff via uv** — A1 — Technical — Remember
   - Measurable: Student runs `uv add ruff --dev` successfully

2. **Run Ruff Format Command** — A1 — Technical — Apply
   - Measurable: Student runs `uv run ruff format .` and sees files reformatted

3. **Run Ruff Check (Linter)** — A1 — Technical — Apply
   - Measurable: Student runs `uv run ruff check .` and identifies lint errors

**Key Concepts** (≤7 per B1 tier):
1. Linting vs. formatting distinction
2. Ruff replaces Black, isort, Flake8
3. PEP 8 style standards (brief)
4. Unused imports detection (F401)
5. Line length and quote style conventions
6. `--fix` flag for auto-fixes
7. pyproject.toml for configuration

**Prerequisites**:
- uv basics (Chapter 12, Lesson 2)
- Python code examples (Chapter 13)
- Understanding of code style importance

**Duration**: 15 minutes

**Content Outline**:
- **Section 1: What Does Ruff Do?** (2 min)
  - Problem: Too many tools (Black, isort, Flake8, etc.)
  - Solution: Ruff does all three in one, 10-100x faster
  - Why it matters: Consistency + automation

- **Section 2: Install Ruff** (2 min)
  - Command: `uv add ruff --dev`
  - Verify: `uv run ruff --version`
  - Shows installation in dependency-groups

- **Section 3: Format Python Code** (4 min)
  - Create sample Python file with messy formatting
  - Command: `uv run ruff format .`
  - Show before/after: spacing, quotes, line breaks
  - Explain each change (PEP 8 reference)

- **Section 4: Lint and Check Code** (4 min)
  - Add lint violations to sample file (unused imports, bad naming)
  - Command: `uv run ruff check .`
  - Show error codes (F401, E501, etc.)
  - Mention: Errors marked `[*]` are fixable with `--fix`

- **Section 5: Auto-Fix Violations** (2 min)
  - Command: `uv run ruff check . --fix`
  - Show removal of unused imports automatically
  - Explain: Not all violations auto-fixable

- **Section 6: Configuration Preview** (1 min)
  - Mention: Lesson 9 covers detailed pyproject.toml config
  - Brief: Ruff reads `[tool.ruff]` section

**Content Elements**:
- **Sample messy code**:
  ```python
  import os
  import   sys
  x=1+2
  def foo(   a,b   ):
      return a+b
  ```
- **After formatting**:
  ```python
  def foo(a, b):
      return a + b
  ```
- **Linting violations shown**:
  ```
  F401 [*] `os` imported but unused
  F401 [*] `sys` imported but unused
  ```

**Practice Approach**:
- **Exercise 1**: Write messy Python code, run `uv run ruff format .`, observe changes
- **Exercise 2**: Add lint violations (unused imports), run `uv run ruff check .`, understand error codes
- **Acceptance Criteria**: Student formats file successfully, sees at least 2 violations and understands what each means

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code (Tier 1/2)
- **Tier 1 Direct Prompt**: "Run Ruff formatter on your project and show me the changes"
- **Tier 2 AI Companion Prompt**: "Why did Ruff change my quotes from single to double? Can I use single quotes instead?"
- **Tier 3 Orchestration Prompt**: "Format all Python files in 5 different projects using the same Ruff rules"
- **Expected Outcome**: Reader understands Ruff is powerful automation; lesson 9 covers configuration customization
- **Safety Note**: None (formatting is safe)

**Cognitive Load**: 7 concepts, within B1 limit

---

### Lesson 9: Advanced Ruff Configuration (20 min)

**Objective** (Bloom's: Apply): Readers will be able to configure Ruff in pyproject.toml using AI assistance, select rule categories, and integrate Ruff with Zed for automatic formatting on save

**Skills Taught**:
1. **Configure Ruff in pyproject.toml** — B1 — Technical — Apply
   - Measurable: Student adds `[tool.ruff]` section with custom settings and rules apply

2. **Select and Ignore Ruff Rules** — B1 — Technical — Apply
   - Measurable: Student selects rule categories (E, F, B, I) or ignores specific rules (E501)

3. **Integrate Ruff with Zed LSP** — A2 — Technical — Apply
   - Measurable: Student configures Zed to show Ruff diagnostics inline

**Key Concepts** (≤7 per B1 tier):
1. `[tool.ruff]` section structure
2. `line-length` configuration
3. `target-version` for Python version compatibility
4. Rule categories (E, F, B, I, N, D)
5. `select` vs `ignore` directives
6. Format-on-save in Zed
7. LSP integration concept (from lesson 7)

**Prerequisites**:
- Lesson 8 (Ruff basics)
- TOML file format familiarity (brief intro)
- Zed basics (Lesson 7)

**Duration**: 20 minutes

**Content Outline**:
- **Section 1: Why Configure Ruff?** (2 min)
  - Problem: Default rules might not match team preferences
  - Solution: Configure in pyproject.toml (version-controlled, shared)
  - Why not memorize: Use AI to generate configs

- **Section 2: Ruff Configuration File** (3 min)
  - Location: pyproject.toml (root of uv project)
  - Basic structure with comments
  - Show existing uv project's structure (will add [tool.ruff] section)

- **Section 3: Line Length & Target Version** (3 min)
  - `line-length = 88` (Black-compatible default)
  - `target-version = "py313"` (Python 3.13)
  - Explanation: Ruff respects these across all rules

- **Section 4: Select & Ignore Rules** (6 min)
  - Rule categories explained:
    - E: pycodestyle errors (PEP 8)
    - F: Pyflakes (undefined names, unused imports)
    - B: flake8-bugbear (likely bugs)
    - I: isort (import sorting)
    - N: pep8-naming (naming conventions)
    - D: pydocstyle (docstring style)
  - Configuration approach:
    ```toml
    [tool.ruff.lint]
    select = ["E", "F", "B", "I"]  # Enable these
    ignore = ["E501"]               # But ignore line-too-long
    ```
  - Explanation: Select what you want, ignore exceptions

- **Section 5: Format-on-Save in Zed** (4 min)
  - Show Zed settings.json configuration
  - Add language-specific Python format settings
  - Demonstrate: Save file → Ruff formats automatically
  - Mention: Configuration templated; use AI to generate

- **Section 6: Verification** (2 min)
  - Run `uv run ruff check .` with new config
  - Show that rules are applied correctly
  - Mention: Lesson 11 integrates all tools together

**Content Elements**:
- **Complete pyproject.toml example**:
  ```toml
  [project]
  name = "lightning-stack-example"
  version = "0.1.0"

  [tool.ruff]
  line-length = 88
  target-version = "py313"

  [tool.ruff.lint]
  select = ["E", "F", "B", "I"]
  ignore = ["E501"]

  [tool.ruff.format]
  quote-style = "double"
  indent-style = "space"
  ```

- **Zed settings.json (relevant portion)**:
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

**Practice Approach**:
- **Exercise 1 (AI-Assisted)**: Ask AI to create a pyproject.toml [tool.ruff] section with your preferences (e.g., "Enable import sorting and bug detection, ignore line length")
- **Exercise 2**: Add configuration to project, run `uv run ruff check .`, verify rules apply
- **Exercise 3 (Optional)**: Configure Zed to format on save
- **Acceptance Criteria**: pyproject.toml has valid [tool.ruff] section, `ruff check` applies rules correctly, Zed shows Ruff diagnostics (if LSP working)

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code (Tier 2/3 focus; this is AI-heavy configuration work)
- **Tier 2 AI Companion Prompt**: "I want strict Ruff configuration: enable import sorting, bug detection, and docstring checks. Add security rules if available. Update my pyproject.toml."
- **Tier 3 Orchestration Prompt**: "Apply the same Ruff configuration to 5 Python projects, preserving existing settings"
- **Tier 1 Direct Prompt**: "Show me the [tool.ruff] section from pyproject.toml and explain what rules are enabled"
- **Expected Outcome**: Readers prefer AI for configuration; they understand configuration structure, not memorized syntax
- **Safety Note**: Configuration is safe; wrong rules just disable checks (not destructive)

**Cognitive Load**: 7 concepts, within B1 limit

---

### Lesson 10: Pyright Type Checker (20 min)

**Objective** (Bloom's: Understand/Apply): Readers will be able to install Pyright via uv, run type checking, understand type hints in Python 3.13+, and configure type checking modes

**Skills Taught**:
1. **Install and Use Pyright** — A1 — Technical — Apply
   - Measurable: Student installs Pyright and runs type checking successfully

2. **Write and Understand Type Hints** — B1 — Technical — Apply
   - Measurable: Student adds type hints to functions and identifies type errors

3. **Configure Type Checking Modes** — A2 — Technical — Apply
   - Measurable: Student configures Pyright mode (basic/strict) and understands tradeoffs

**Key Concepts** (≤7 per B1 tier):
1. Type hints syntax (Python 3.13+ style: `str | None`)
2. Return type annotations
3. Function parameter types
4. Type errors vs runtime errors
5. Type checking modes (basic, standard, strict)
6. Catching bugs before running code
7. Pyright as LSP in Zed

**Prerequisites**:
- Python function basics (Chapter 13)
- Type hints concept (brief intro if needed)
- uv basics (Chapter 12, Lesson 2)

**Duration**: 20 minutes

**Content Outline**:
- **Section 1: Why Type Checking?** (2 min)
  - Problem: Python is dynamically typed; bugs hide until runtime
  - Solution: Pyright catches type errors before running code
  - Benefit: Catches 30-50% of bugs statically (industry data)

- **Section 2: Install Pyright** (2 min)
  - Command: `uv add pyright --dev`
  - Verify: `uv run pyright --version`
  - Note: Pyright is Microsoft tool, very stable

- **Section 3: Type Hints Basics** (4 min)
  - Python 3.13+ syntax (using `|` instead of `Union`)
  - Function parameter types: `def greet(name: str) -> str:`
  - Show simple example with and without types
  - Mention: Type hints are optional syntax; Python still runs without them

- **Section 4: Run Pyright Type Checker** (4 min)
  - Create Python file with type error:
    ```python
    def add_numbers(x: int, y: int) -> int:
        return str(x) + str(y)  # Error: returns str, not int
    ```
  - Command: `uv run pyright test.py`
  - Show error output: clear message about type mismatch
  - Fix the code, re-run, show success

- **Section 5: Type Checking Modes** (5 min)
  - Basic (default): Type hints checked, some leniency
  - Standard: Stricter type checking
  - Strict: Maximum type safety, catches `None` handling
  - Explanation: Choose mode based on project needs
  - Note: Lesson 11 integrates into workflow

- **Section 6: Zed LSP Integration Preview** (3 min)
  - Mention: Zed will show type errors inline (Lesson 11)
  - Benefit: Real-time feedback while coding
  - Preview: `.zed/settings.json` will configure Pyright LSP

**Content Elements**:
- **Type error example**:
  ```python
  def greet(name: str) -> str:
      return f"Hello, {name}"

  result = greet(42)  # Error: int is not str
  ```
- **Type hint variations**:
  ```python
  def process(data: str | None) -> int:
      """Accept string or None, return integer."""
      return len(data) if data else 0
  ```
- **Pyright output**:
  ```
  error: Argument of type "Literal[42]" cannot be assigned to parameter "name" of type "str" in function "greet"
    "Literal[42]" is not assignable to "str" (reportGeneralTypeIssues)
  1 error, 0 warnings, 0 informations
  ```

**Practice Approach**:
- **Exercise 1**: Write function without type hints, add hints, run Pyright
- **Exercise 2**: Intentionally add type mismatch, run Pyright, understand error message
- **Exercise 3 (Optional)**: Configure Pyright to strict mode, see more errors
- **Acceptance Criteria**: Student runs Pyright successfully, identifies at least one type error, understands what it means

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code (Tier 1/2)
- **Tier 1 Direct Prompt**: "Install Pyright and run it on my project to check for type errors"
- **Tier 2 AI Companion Prompt**: "I'm getting a type error about None values. Fix my code to handle optional parameters correctly"
- **Tier 3 Orchestration Prompt**: "Add comprehensive type hints to all functions in my 3-file project and fix any type errors Pyright finds"
- **Expected Outcome**: Readers understand type checking value; Pyright is powerful safety net
- **Safety Note**: Type checking is safe; errors are suggestions, not enforcement

**Cognitive Load**: 7 concepts, within B1 limit

---

### Lesson 11: Complete Workflow Integration (30 min)

**Objective** (Bloom's: Apply/Analyze): Readers will be able to create a new Python project with all tools integrated, configure all tools to work together, and understand when each tool runs in the workflow

**Skills Taught**:
1. **Integrate Ruff + Pyright in Zed** — B1 — Technical — Apply
   - Measurable: Student configures Zed LSP for both tools and sees diagnostics

2. **Set Up Complete pyproject.toml** — B1 — Technical — Apply
   - Measurable: Student adds sections for uv, Ruff, and Pyright in correct format

3. **Understand Tool Workflow Sequence** — B1 — Technical — Analyze
   - Measurable: Student explains when Ruff format, Ruff check, and Pyright run in their workflow

**Key Concepts** (≤7 per B1 tier):
1. Workflow sequence: format → lint → type check
2. LSP provides real-time feedback in editor
3. Each tool's responsibility in pipeline
4. Pre-commit hooks automation (awareness level)
5. Configuration inheritance (uv settings apply to all tools)
6. Error prioritization (type errors > lint warnings)
7. Cross-platform consistency

**Prerequisites**:
- Lessons 7-10 (all individual tools)
- Understanding of tool purposes and configs

**Duration**: 30 minutes

**Content Outline**:
- **Section 1: Workflow Overview** (3 min)
  - Show the sequence: code → format → lint → type check → commit
  - Explain: Each tool catches different issues
  - Benefit: Catches bugs and style problems early

- **Section 2: Project Setup from Scratch** (5 min)
  - Start with `uv init my-project`
  - Create basic Python file with intentional issues
  - Add Ruff and Pyright: `uv add ruff pyright --dev`
  - Verify installations

- **Section 3: Complete pyproject.toml** (8 min)
  - Show structure with all sections:
    ```toml
    [project]
    name = "lightning-stack-example"
    version = "0.1.0"

    [dependency-groups]
    dev = ["ruff>=0.14.0", "pyright>=1.1.400"]

    [tool.ruff]
    line-length = 88
    target-version = "py313"

    [tool.ruff.lint]
    select = ["E", "F", "B", "I"]
    ignore = ["E501"]

    [tool.pyright]
    typeCheckingMode = "standard"
    pythonVersion = "3.13"
    ```
  - Explanation: Each tool reads its own section

- **Section 4: Zed Configuration for Dual LSP** (8 min)
  - Create/edit `.zed/settings.json` with:
    ```json
    {
      "languages": {
        "Python": {
          "language_servers": ["pyright", "ruff"],
          "formatter": "language_server",
          "format_on_save": true
        }
      },
      "lsp": {
        "pyright": {
          "initialization_options": {
            "python": {
              "pythonPath": ".venv/bin/python"
            }
          }
        }
      }
    }
    ```
  - Explanation: Zed runs both Pyright and Ruff, shows diagnostics inline

- **Section 5: Step-by-Step Workflow** (4 min)
  - Open project in Zed
  - Write Python code with deliberate errors
  - Watch Ruff errors appear inline (red underlines)
  - Watch Pyright type errors appear inline (different underlines)
  - Save file → Ruff formats automatically
  - Re-run type check: `uv run pyright`
  - Show cleaner code

- **Section 6: Awareness-Level Tools Mention** (2 min)
  - Mention but don't deep-dive: pytest (testing), pre-commit (hooks), MkDocs (docs)
  - Show working config snippets (copy-pasteable)
  - "We'll learn these in later chapters, but here's a working starting point"

**Content Elements**:
- **Complete working project example**:
  - pyproject.toml with all sections (shown above)
  - .zed/settings.json with Ruff + Pyright LSP (shown above)
  - Sample Python file with issues to demonstrate tools

- **Example code with issues**:
  ```python
  import os
  import sys

  def calculate_total(items: list) -> int:
      """Calculate total with type error."""
      return sum(items)  # Error: sum returns int, but passing list of unknown type

  def process_data(data: dict | None) -> str:
      """Process optional data."""
      return data["key"]  # Error: accessing dict without checking if None
  ```

**Practice Approach**:
- **Exercise 1**: Create new uv project, add Ruff and Pyright
- **Exercise 2**: Configure pyproject.toml with all tool sections
- **Exercise 3**: Configure Zed for dual LSP
- **Exercise 4**: Write code with style and type errors, verify tools catch them
- **Exercise 5 (Optional)**: Add pytest and pre-commit config snippets
- **Acceptance Criteria**:
  - New project created with all tools installed
  - pyproject.toml has all sections with valid syntax
  - Zed shows Ruff and Pyright diagnostics inline
  - Format-on-save works
  - All tools run successfully via `uv run`

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code (Tier 2/3 focus)
- **Tier 2 AI Companion Prompt**: "Configure my Zed settings and pyproject.toml so Ruff formats on save and Pyright checks types in real-time"
- **Tier 3 Orchestration Prompt**: "Set up 3 new Python projects with the complete Lightning Stack (uv + Ruff + Pyright + Zed) configuration"
- **Tier 1 Direct Prompt**: "Explain the workflow: when does Ruff format run, when does Pyright check run, what order?"
- **Expected Outcome**: Readers feel professional toolchain power; understand integration; ready for advanced configuration
- **Safety Note**: Configuration is safe; worst case is disabled checks

**Cognitive Load**: 7 concepts (workflow sequence, LSP, tool responsibilities, inheritance, error prioritization, configs, consistency)

---

### Lesson 12: Professional Template & Troubleshooting (25 min)

**Objective** (Bloom's: Apply/Create): Readers will be able to use a professional pyproject.toml template, troubleshoot common integration issues using AI, and scale their setup across projects

**Skills Taught**:
1. **Use Professional pyproject.toml Template** — B1 — Technical — Apply
   - Measurable: Student copies template and customizes for their project needs

2. **Troubleshoot Common Tool Integration Issues** — B1 — Technical — Apply
   - Measurable: Student diagnoses and fixes LSP errors, missing dependencies, path issues

3. **Apply Settings Consistently Across Projects** — B1 — Technical — Apply
   - Measurable: Student creates 2+ projects using same configuration consistently

**Key Concepts** (≤7 per B1 tier):
1. Template-based project creation
2. Configuration reuse and inheritance
3. Common error patterns (LSP not starting, tools not found)
4. Environment setup (Python path, virtual env)
5. Troubleshooting workflow (read error → check config → ask AI)
6. Documentation and README best practices
7. Scaling setup across teams

**Prerequisites**:
- All previous lessons (7-11)
- Understanding of complete workflow

**Duration**: 25 minutes

**Content Outline**:
- **Section 1: Professional Template Introduction** (2 min)
  - Problem: Every project needs same config, easy to miss settings
  - Solution: Template file that can be copied and customized
  - Benefit: Consistency, speed, fewer mistakes

- **Section 2: Complete Professional pyproject.toml** (6 min)
  - Show annotated template with:
    - Project metadata (name, version, authors, license)
    - Python requirements (version constraints)
    - Production dependencies (if any)
    - Dev dependencies (Ruff, Pyright, pytest, pre-commit awareness)
    - All tool configurations [tool.ruff], [tool.pyright]
    - Optional: pytest config, pre-commit config reference
  - Explanation of each section and customization points

- **Section 3: Common Integration Issues** (8 min)
  - **Issue 1: "Ruff: no such file or directory"**
    - Cause: Tool not installed or not run via uv
    - Fix: Use `uv run ruff`, not just `ruff`
    - Prevention: Include in docs, CI/CD uses uv run

  - **Issue 2: "Pyright: LSP not starting"**
    - Cause: Zed can't find Python LSP, Pyright not installed
    - Fix: Check `uv add pyright --dev` completed, check Zed logs
    - Prevention: Verify installation before Zed config

  - **Issue 3: "Type errors but code runs fine"**
    - Cause: Type hints are optional; Pyright checks don't enforce at runtime
    - Fix: Understanding: Type errors catch static issues, not all runtime bugs
    - Prevention: Education (lesson 10)

  - **Issue 4: "Format-on-save not working in Zed"**
    - Cause: LSP not configured, Ruff not installed, wrong formatter selected
    - Fix: Check settings.json formatter field, verify Ruff installed
    - Prevention: Use provided settings.json template

  - **Issue 5: "pyproject.toml has syntax errors"**
    - Cause: TOML formatting mistakes (missing =, wrong quotes)
    - Fix: Use AI to validate/rewrite configuration
    - Prevention: Validate TOML before using

- **Section 4: Troubleshooting Workflow** (4 min)
  - Step 1: Read error message carefully
  - Step 2: Check installation: `uv run <tool> --version`
  - Step 3: Check configuration: open pyproject.toml, look for typos
  - Step 4: Check Zed settings: verify LSP configuration
  - Step 5: Ask AI: "I'm seeing [error]. Can you diagnose and fix?"
  - Emphasis: AI is your troubleshooting partner; don't memorize error codes

- **Section 5: Scaling to Multiple Projects** (3 min)
  - One-liner template copy: Take template, customize name/version
  - Shared config approach: .python-version, base pyproject.toml template
  - Team approach: Git template repo with standard configs
  - Mention: CI/CD will validate (awareness, not deep dive)

- **Section 6: README & Documentation** (2 min)
  - README should include: "Quick Start" section with setup steps
  - Document Ruff/Pyright rules your project enforces
  - Include troubleshooting FAQ
  - Point to official docs for tool updates

**Content Elements**:
- **Professional pyproject.toml Template** (complete, annotated):
  ```toml
  [build-system]
  requires = ["hatchling"]
  build-backend = "hatchling.build"

  [project]
  name = "my-project"
  version = "0.1.0"
  description = "Brief description"
  authors = [{name = "Your Name", email = "you@example.com"}]
  license = {text = "MIT"}
  requires-python = ">=3.13"

  [dependency-groups]
  dev = [
      "ruff>=0.14.0",
      "pyright>=1.1.400",
      "pytest>=7.0",  # Awareness level
  ]

  [tool.ruff]
  line-length = 88
  target-version = "py313"

  [tool.ruff.lint]
  select = ["E", "F", "B", "I", "N"]
  ignore = ["E501"]

  [tool.ruff.format]
  quote-style = "double"
  indent-style = "space"

  [tool.pyright]
  typeCheckingMode = "standard"
  pythonVersion = "3.13"
  reportMissingImports = "error"
  reportUnknownVariableType = "warning"

  # [tool.pytest.ini_options]  # Awareness level
  # testpaths = ["tests"]
  # addopts = "-v --strict-markers"
  ```

- **Troubleshooting Decision Tree** (visual, text format):
  ```
  Error appears?
  ├─ "command not found" → Check installation (uv add tool --dev)
  ├─ "LSP not starting" → Verify Zed settings.json, check Python path
  ├─ "TOML syntax error" → Ask AI to validate/fix configuration
  ├─ "Formatting not working" → Check format_on_save = true in Zed
  └─ "Still broken?" → Ask AI with error message + config file
  ```

- **README Quick Start Template**:
  ```markdown
  ## Quick Start

  1. Install dependencies: `uv sync`
  2. Open project: `zed .`
  3. Write code; Ruff formats on save, Pyright checks types
  4. Run linter: `uv run ruff check .`
  5. Run type checker: `uv run pyright`
  ```

**Practice Approach**:
- **Exercise 1**: Start new project, copy professional template, customize for your project
- **Exercise 2**: Intentionally break configuration (e.g., add typo), troubleshoot using workflow
- **Exercise 3**: Create 2-3 projects using same template to verify consistency
- **Exercise 4 (Optional)**: Write README quick start for your project
- **Acceptance Criteria**:
  - New project created from template in <5 minutes
  - All tools work (format, lint, type check)
  - Successfully troubleshoot one simulated issue
  - Projects have consistent configurations

**End-of-Lesson: Try With AI**
- **Tool**: Claude Code (Tier 2/3 focus; orchestration emphasis)
- **Tier 3 Orchestration Prompt**: "Create 5 new Python projects using this template and verify all tools work in each one"
- **Tier 2 AI Companion Prompt**: "My Pyright is reporting tons of type errors. Help me configure pyright in pyproject.toml to be less strict for this legacy project"
- **Tier 1 Direct Prompt**: "What does this pyproject.toml configuration do? Which tools does it set up?"
- **Expected Outcome**: Readers feel confident creating and scaling projects; understand they can ask AI for troubleshooting
- **Safety Note**: Configuration changes are safe; worst case disables checks

**Cognitive Load**: 7 concepts (templates, reuse, error patterns, diagnosis, troubleshooting workflow, scaling, documentation)

---

## Content Flow & Dependencies

**Smooth Progression**:
1. **Lesson 6 (uv) → Lesson 7 (Zed)**: "Now that uv manages packages, let's open projects in a professional IDE"
2. **Lesson 7 → Lesson 8 (Ruff)**: "Let's automate code formatting and linting while editing"
3. **Lesson 8 → Lesson 9 (Ruff Config)**: "Customize Ruff to match your team's preferences"
4. **Lesson 9 → Lesson 10 (Pyright)**: "Add static type checking to catch bugs before running code"
5. **Lesson 10 → Lesson 11 (Integration)**: "Bring all tools together in one workflow"
6. **Lesson 11 → Lesson 12 (Professional Use)**: "Use professional templates and troubleshoot like experts"

**Cross-Lesson References**:
- Lessons 7-10 each introduce one tool individually
- Lesson 11 integrates all tools taught in 7-10
- Lesson 12 provides reusable templates from lessons 8-10
- Each lesson's "Try With AI" builds toward orchestration capability (Tier 1 → Tier 2 → Tier 3)

---

## Scaffolding Strategy

**Tier 1 (Foundational, Book Teaches)**:
- Installation procedures (Zed, Ruff, Pyright)
- Basic commands (`zed .`, `uv run ruff format .`, `uv run pyright`)
- Understanding why each tool exists
- Reading error messages

**Tier 2 (Complex Execution, AI Companion)**:
- Configuring pyproject.toml rules
- Configuring Zed LSP settings.json
- Selecting Ruff rule categories
- Setting Pyright checking modes
- Integrating multiple tools

**Tier 3 (Scaling & Orchestration, AI Orchestration)**:
- Creating template-based projects at scale
- Automating tool setup across 5+ projects
- Troubleshooting workflows for teams
- Scaling consistency across repositories

**Cognitive Load Management**:
- Each lesson introduces ≤7 new concepts (B1 limit)
- Commands are direct and repeatable
- Configuration is template-driven, not memorized
- AI handles complexity; students understand strategy

---

## Integration Points

**With Chapter 12, Lessons 1-6 (uv)**:
- Lessons 7-12 assume students completed uv setup
- Lesson 12 template includes dependency-groups from Lesson 2
- Lesson 11 verifies `uv add` and `uv run` commands work

**With Chapter 7 (Bash)**:
- Terminal commands used throughout (cd, ls, run commands)
- Environment variables mentioned briefly

**With Chapter 8 (Git)**:
- .gitignore for Python best practices (mentioned in awareness)
- Version control integration (pre-commit hooks in awareness)

**With Chapter 11 (Context Engineering)**:
- "Try With AI" exercises use effective prompting patterns
- Specification-first mindset: "Tell AI what you want, not how to configure"

**With Future Chapters**:
- Chapter 42 (Testing): pytest integration (awareness in Lesson 12)
- Chapter 54 (CI/CD): pre-commit hooks, GitHub Actions (awareness in Lesson 12)
- Part 7 (Production): Docker, Kubernetes (deferred, not mentioned)

---

## Validation Strategy

**For Conceptual Understanding** (Lessons 7, 10):
- Quiz: "Why use Zed over VS Code?" (trade-offs understanding)
- Quiz: "What does Pyright catch that Python doesn't?" (type safety value)
- Scenario: "When should you enable strict type checking?" (decision-making)

**For Hands-On Skills** (Lessons 8-9, 11-12):
- Exercise: Install tool, run command, show output
- Exercise: Configure tool, verify configuration applies
- Exercise: Create project from template, verify all tools work
- Exercise: Troubleshoot simulated error, diagnose correctly

**For Integration** (Lessons 11-12):
- Integrated Exercise 1: Create project → add tools → configure → use all together
- Integrated Exercise 2: Create 3 projects using same template, verify consistency
- Integrated Exercise 3: Troubleshoot real error from student's project

**Success Criteria Measurement**:
- SC-001: Quiz on tool purposes (80%+ pass = concept understood)
- SC-002: Create Lightning Stack project in <30 min (hands-on)
- SC-003: All tools integrated and working (automated check)
- SC-004: Troubleshoot issue using AI (scenario-based)
- SC-005: Explain when to use Tier 1 vs Tier 2 vs Tier 3 (understanding)
- SC-006: Use AI for complex config without memorizing syntax (observation)
- SC-007: Count concepts per lesson ≤7 (design verification)
- SC-008: Verify lesson durations match estimates (timing check)
- SC-009: Cite verified intelligence doc for all claims (spot-check citations)

---

## YAML Frontmatter Specification

All lessons (7-12) MUST include YAML frontmatter matching this structure (from Lesson 1 example):

```yaml
---
title: "[Lesson Title]"
chapter: 12
lesson: [7-12]
duration_minutes: [estimated time]

skills:
  - name: "[Skill Name]"
    proficiency_level: "[A1/A2/B1]"
    category: "[Technical/Conceptual/Soft]"
    bloom_level: "[Remember/Understand/Apply/Analyze/Evaluate/Create]"
    digcomp_area: "[Information/Communication/Content/Safety/Problem-Solving]"
    measurable_at_this_level: "[Specific action student demonstrates]"
  - name: "[Skill 2]"
    # ... repeat ...

learning_objectives:
  - objective: "[Bloom's verb] [content] [context]"
    proficiency_level: "[A1/A2/B1]"
    bloom_level: "[Remember/Understand/Apply/Analyze/Evaluate/Create]"
    assessment_method: "[How student demonstrates this]"
  - objective: "[Objective 2]"
    # ... repeat ...

cognitive_load:
  new_concepts: [number ≤7 for B1]
  assessment: "[Brief validation that concept count is appropriate]"

differentiation:
  extension_for_advanced: "[Challenge for faster learners]"
  remedial_for_struggling: "[Simplified version for struggling learners]"

generated_by: "content-implementer v3.0.0"
source_spec: "specs/001-chapter-12-lightning-python-stack/spec.md"
created: "[DATE]"
last_modified: "[DATE]"
git_author: "Claude Code"
---
```

---

## Key Teaching Principles Applied

1. **Principle 13 (Graduated Teaching)**:
   - Tier 1: Book teaches installation, basic commands, tool purposes
   - Tier 2: AI companion handles configuration complexity
   - Tier 3: AI orchestrates multi-project setup and troubleshooting

2. **Principle 18 (Three Roles Framework)**:
   - AI as Teacher: Demonstrates configuration patterns and techniques
   - AI as Student: Accepts feedback, refines configurations
   - AI as Co-Worker: Orchestrates multi-project setup, troubleshoots with students

3. **Principle 9 (Show-Spec-Validate)**:
   - Show: CLI commands and expected output
   - Spec: Configuration TOML files (specification of tools)
   - Validate: Run tools, verify they work, demonstrate benefits

4. **Principle 5 (Progressive Complexity)**:
   - Lesson 7: Install IDE (foundational, direct commands)
   - Lesson 8: Use formatter (direct, one command)
   - Lesson 9: Configure formatter (AI-assisted, complex syntax)
   - Lesson 10: Type checker (new skill, similar pattern)
   - Lesson 11: Integration (applies all lessons)
   - Lesson 12: Professional use (orchestration at scale)

5. **Principle 12 (Cognitive Load Consciousness)**:
   - B1 tier: 7 concepts per lesson (max for intermediate)
   - Simple sequencing: One tool at a time before integration
   - Minimal code examples (3-5 lines, demonstrate tool not teach Python)
   - AI handles syntax, students understand strategy

6. **Principle 8 (Accessibility & Inclusivity)**:
   - Platform-specific instructions (Windows/Mac/Linux)
   - Error literacy: "Red flags to watch" section in each lesson
   - No assumed IDE expertise
   - Alt tools mentioned (VS Code, PyCharm) but Zed primary

---

## Success Criteria Alignment Summary

| Success Criteria | Lesson(s) | Validation Method |
|------------------|-----------|-------------------|
| SC-001: Tool purpose understanding | 7, 8, 10, 11 | Quiz + Scenario exercise |
| SC-002: Complete setup without help | 11, 12 | Hands-on: create project in <30 min |
| SC-003: Integrated workflow | 11 | All tools working, no errors |
| SC-004: Troubleshoot with AI | 12 | Simulate error, diagnose correctly |
| SC-005: Direct vs AI judgment | All | "Try With AI" tier selection exercise |
| SC-006: AI for complex config | 9, 11, 12 | Observation: students ask AI for TOML |
| SC-007: B1 cognitive load | All | Count concepts: all ≤7 |
| SC-008: Realistic durations | All | Time estimates match practice |
| SC-009: Verified examples | All | Cite intelligence/001-verified-tool-documentation.md |
| SC-010: Smooth transition from uv | 7 | Exercise: open uv project in Zed |
| SC-011: Progression understanding | 11, 12 | "Explain the workflow" exercise |
| SC-012: Awareness-level tools | 12 | Mention pytest, pre-commit, MkDocs with configs |

---

## Notes for Implementation Team

1. **YAML Frontmatter**: Use exact structure above; this enables institution integration for competency assessment
2. **Verified Claims**: Every tool example must cite `specs/001-chapter-12-lightning-python-stack/intelligence/001-verified-tool-documentation.md`
3. **Minimal Code Examples**: Keep ≤5 lines per example; demonstrate tool functionality, not teach Python
4. **AI Tool Selection**:
   - Pre-tool chapters (1-6) use ChatGPT web for "Try With AI"
   - Lesson 7+ can reference Claude Code, Gemini CLI (taught in Part 2)
   - Variants provided where multiple tools available
5. **Platform Notes**: Include Windows WSL, macOS native, Linux standard paths
6. **Accessibility**: Every error message explained; no "just run this" without context
7. **Test in Sandbox**: Verify all commands work before publishing (tools.yaml version pins)

---

**PLAN STATUS**: Ready for Task Decomposition (Phase 3)
