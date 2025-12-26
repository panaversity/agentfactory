---
sidebar_position: 8
title: "Packaging & Publishing MCP Servers"
description: "Transform your MCP server from local development to a distributable package. Learn pyproject.toml configuration, building, testing in clean environments, and publishing to PyPI or private registries."
keywords: ["packaging", "pyproject.toml", "entry points", "PyPI", "distribution", "uv build", "pip install", "versioning", "B1-B2"]
chapter: 38
lesson: 8
duration_minutes: 45

skills:
  - name: "Python Package Configuration (pyproject.toml)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Programming and Algorithm Design"
    measurable_at_this_level: "Student can structure a complete pyproject.toml with metadata, dependencies, and entry points"
  - name: "Entry Points for CLI Tools"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Programming and Algorithm Design"
    measurable_at_this_level: "Student can define entry points that enable installed MCP servers to run as command-line tools"
  - name: "Build Automation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Programming and Algorithm Design"
    measurable_at_this_level: "Student can use uv build to generate distribution artifacts (wheel, sdist)"
  - name: "Testing in Isolated Environments"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Quality Management"
    measurable_at_this_level: "Student validates that installed packages work without development dependencies"
  - name: "Distribution & Registry Publishing"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student understands PyPI publishing and private registry alternatives"

learning_objectives:
  - objective: "Structure a production-ready pyproject.toml with complete metadata, dependencies, and build configuration"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates pyproject.toml that passes validation and builds without errors"
  - objective: "Configure entry points that enable installed MCP servers to run as executable commands"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student defines entry point and verifies command is available after installation"
  - objective: "Use uv build to create wheels and source distributions"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student builds package and inspects distribution contents"
  - objective: "Test installed packages in clean environments to catch missing dependencies and configuration errors"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Student validates package in isolated environment and identifies environment-specific issues"
  - objective: "Understand versioning, publishing workflows, and registry selection for different distribution scenarios"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Student articulates versioning strategy and explains PyPI vs private registry tradeoffs"

cognitive_load:
  new_concepts: 6
  assessment: "Moderate - Integrates pyproject.toml, entry points, build tools, and testing concepts into end-to-end workflow. Students have seen each concept in isolation; this lesson is about orchestration."

differentiation:
  extension_for_advanced: "Implement CI/CD pipeline using GitHub Actions to automate testing and publishing; explore poetry, hatch, and build backends; configure private PyPI servers"
  remedial_for_struggling: "Focus on the four essential sections: [project], [build-system], [project.scripts], and dependencies. Defer build customization (extras, classifiers) to later"
---

# Packaging & Publishing MCP Servers

You've built a working MCP server. It runs locally. It handles requests correctly. Your team uses it successfully. Now the question becomes: how do you share it beyond your development environment?

This is where packaging transforms your local experiment into an enterprise-grade product. When you package an MCP server, you're not just collecting files—you're encoding metadata, dependencies, entry points, and compatibility guarantees that enable others to install and run it with a single command.

The difference is profound. An unpackaged server requires:
- "Clone this repo"
- "Install Python 3.11+"
- "Install dependencies with pip install -r requirements.txt"
- "Run with python -m module_name"
- Hope the user has the right environment

A packaged server requires:
- `pip install my-mcp-server`
- `my-mcp-server` (command runs immediately)

The `pyproject.toml` file is your contract with users. It specifies what your server needs and how to use it. This lesson teaches you to write that contract clearly and completely.

## From Local to Distributable: The Packaging Arc

Before diving into syntax, understand what happens when you package:

**Development phase** (where you are now):
```
your-project/
├── src/
│   └── my_mcp_server/
│       ├── __init__.py
│       ├── server.py
│       └── tools/
└── tests/
```

You run: `python -m my_mcp_server`

**Package phase** (what this lesson teaches):
```
dist/
├── my_mcp_server-0.1.0-py3-none-any.whl  # Binary package (wheel)
└── my_mcp_server-0.1.0.tar.gz             # Source package (sdist)
```

User runs: `pip install my_mcp_server-0.1.0-py3-none-any.whl` or `pip install my-mcp-server` from PyPI

**Installation phase** (what users experience):
```
site-packages/
└── my_mcp_server/
    ├── __init__.py
    ├── server.py
    └── tools/

bin/  (or Scripts/ on Windows)
└── my-mcp-server  # Entry point script created automatically
```

User runs: `my-mcp-server` (command available in PATH)

The leap from "python -m" to installed command happens because of entry points defined in `pyproject.toml`. That's the key transformation this lesson teaches.

## Anatomy of pyproject.toml

The `pyproject.toml` file declares everything about your package: what it is, what it needs, and how to use it. Here's the complete structure:

```toml
[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"

[project]
name = "my-mcp-server"
version = "0.1.0"
description = "An MCP server for project management integration"
readme = "README.md"
license = {text = "MIT"}
authors = [
    {name = "Your Name", email = "you@example.com"}
]
keywords = ["mcp", "server", "project-management"]
classifiers = [
    "Development Status :: 3 - Alpha",
    "Environment :: Console",
    "Intended Audience :: Developers",
    "License :: OSI Approved :: MIT License",
    "Programming Language :: Python :: 3",
    "Programming Language :: Python :: 3.11",
    "Programming Language :: Python :: 3.12",
    "Topic :: Software Development :: Libraries"
]
requires-python = ">=3.11"
dependencies = [
    "mcp>=0.1.0",
    "pydantic>=2.0",
    "httpx>=0.24.0",
    "uvicorn>=0.23.0"
]

[project.optional-dependencies]
dev = [
    "pytest>=7.0",
    "pytest-asyncio>=0.21",
    "black>=23.0",
    "ruff>=0.1"
]

[project.urls]
Homepage = "https://github.com/yourname/my-mcp-server"
Documentation = "https://my-mcp-server.readthedocs.io"
Repository = "https://github.com/yourname/my-mcp-server.git"
Issues = "https://github.com/yourname/my-mcp-server/issues"

[project.scripts]
my-mcp-server = "my_mcp_server.server:main"

[tool.hatch.build.targets.wheel]
packages = ["src/my_mcp_server"]
```

Let's understand each section:

**[build-system]**: Tells pip how to build your package
- `requires`: Dependencies needed during build (hatchling handles wheel creation)
- `build-backend`: The module that creates the wheel

**[project]**: Package metadata
- `name`: PyPI identifier (use hyphens: my-mcp-server, not my_mcp_server)
- `version`: Follows semantic versioning (0.1.0 = major.minor.patch)
- `description`: One-line summary for PyPI listing
- `readme`: Path to full description (typically README.md)
- `authors`: Contact information
- `requires-python`: Minimum Python version (ensure it matches your dependencies)

**[project.optional-dependencies]**: Extra dependency groups
- `dev = [...]` installs when user runs `pip install my-mcp-server[dev]`
- Other groups: `docs`, `test`, etc.

**[project.urls]**: Links shown on PyPI package page

**[project.scripts]**: Entry points (crucial for MCP servers)
- Format: `command-name = "package.module:function"`
- Creates executable script when installed

**[tool.hatch.build.targets.wheel]**: Where to find source code
- `packages`: Point to actual code directory

## Entry Points: The Bridge Between Installation and Execution

Entry points are the secret sauce that transforms a package into a runnable command. When you specify:

```toml
[project.scripts]
my-mcp-server = "my_mcp_server.server:main"
```

You're telling pip: "When this package is installed, create an executable command called `my-mcp-server` that calls the `main` function in the `my_mcp_server.server` module."

Here's what your server module needs:

```python
# src/my_mcp_server/server.py
import asyncio
from contextlib import asynccontextmanager

from mcp.server import Server
from pydantic import BaseModel

# Your tool implementations...

# Create the server
server = Server("my-mcp-server")

@server.list_tools()
async def list_tools() -> list[Tool]:
    # Return available tools

@server.call_tool()
async def call_tool(name: str, arguments: dict) -> list[TextContent | ImageContent]:
    # Handle tool invocations

# Entry point function that pip will call
def main():
    """Entry point for the MCP server."""
    asyncio.run(server.run_stdio())

if __name__ == "__main__":
    main()
```

The `main()` function must be at module level and accept no arguments. When installed, pip creates a script wrapper that imports your module and calls `main()`.

## Building Your Package

With `pyproject.toml` complete, building is straightforward using `uv build`:

```bash
# Build both wheel and source distribution
uv build

# Check what was created
ls -la dist/

# Output:
# my_mcp_server-0.1.0-py3-none-any.whl
# my_mcp_server-0.1.0.tar.gz
```

This creates:
- **Wheel (.whl)**: Binary package, fastest to install, includes pre-processed metadata
- **Source distribution (.tar.gz)**: Full source code, users can inspect or modify

Both go to the `dist/` directory.

## Testing in a Clean Environment

Here's where many developers skip steps—and where packages fail in production. Just because the package builds doesn't mean it installs correctly in a fresh environment.

Simulate what users will experience:

```bash
# Create a clean virtual environment for testing
python -m venv test-env
source test-env/bin/activate  # or test-env\Scripts\activate on Windows

# Install the wheel you just built
pip install dist/my_mcp_server-0.1.0-py3-none-any.whl

# Verify the command works
my-mcp-server --help

# Or start the server (may need to stop after verifying it starts)
timeout 3 my-mcp-server || true
```

This tests three critical things:
1. **Wheel contents are correct** (would fail if entry point is malformed)
2. **All dependencies are declared** (would fail if you forgot an import dependency)
3. **Installation process works** (would fail with permission errors, missing files, etc.)

If the command isn't found, check:
- `which my-mcp-server` shows the location of the wrapper script
- `pip list` confirms the package installed
- `pip show my-mcp-server` shows package details including location

If the command fails when run, check:
- The entry point path is correct in `pyproject.toml`
- The `main()` function exists and takes no arguments
- All imports work (check by running `python -c "from my_mcp_server.server import main; main()"`)

## Versioning Strategy

Semantic versioning ensures users understand what changed:

**0.1.0** = MAJOR.MINOR.PATCH

- **MAJOR** (0): Incompatible API changes (breaking changes to tool signatures, resource URIs, etc.)
- **MINOR** (1): Backward-compatible feature additions (new tools, new resources)
- **PATCH** (0): Backward-compatible bug fixes

Examples:
- Start at `0.1.0` (unstable, features may change)
- Add a tool → `0.2.0` (users can upgrade safely)
- Fix a bug in existing tool → `0.2.1`
- Remove or rename a tool → `1.0.0` (breaking change, signal major version bump)
- After 1.0.0, follow strict semantic versioning

Before releasing, increment the version in `pyproject.toml`:

```bash
# After making changes
# 1. Update version
sed -i 's/version = "0.1.0"/version = "0.2.0"/' pyproject.toml

# 2. Create a git tag (if using git)
git tag v0.2.0
git push origin v0.2.0

# 3. Build and publish
uv build
```

## Publishing: PyPI vs Private Registries

Once your package is built and tested, you have distribution options:

**PyPI (Python Package Index)**
- Public repository at pypi.org
- Users install with: `pip install my-mcp-server`
- Best for: Open-source tools, public utilities
- Pros: Discoverable, worldwide distribution, official Python registry
- Cons: Public visibility, requires account and API token
- How: `pip install twine && twine upload dist/*`

**Private PyPI Servers**
- Run your own package index (Artifactory, Nexus, simple HTTP server)
- Users install with: `pip install -i https://private.registry.com my-mcp-server`
- Best for: Enterprise tools, proprietary servers, internal distribution
- Pros: Complete control, no public exposure, can require authentication
- Cons: Requires infrastructure, users need registry URL
- How: Depends on your registry platform

**GitHub Releases**
- Attach wheel to release
- Users install with: `pip install https://github.com/yourname/repo/releases/download/v0.1.0/my_mcp_server-0.1.0-py3-none-any.whl`
- Best for: Simple projects, rapid iteration
- Pros: No external registry needed, clear versioning with git tags
- Cons: Harder for users, GitHub dependency
- How: Automate with GitHub Actions

**Local Development**
- Users install from source: `pip install -e .` (editable mode)
- Best for: Contributors developing locally
- Pros: Code changes immediately reflected without reinstalling
- Cons: Not for production use

For this lesson, you'll use local testing. Publishing to PyPI or private registries is a later topic. The key principle: **any of these approaches work once your pyproject.toml is correct.**

## Common Mistakes to Avoid

**1. Forgetting entry point arguments**
```toml
# WRONG: Entry point function called with arguments
my-mcp-server = "my_mcp_server.server:main"  # main() must take NO arguments

# WRONG: Pointing to wrong function
my-mcp-server = "my_mcp_server.main"  # Should be my_mcp_server.server:main
```

**2. Module name vs package name mismatch**
```toml
# WRONG: PyPI name (hyphen) used as import name
name = "my-mcp-server"
packages = ["my-mcp-server"]  # Hyphens don't work in Python imports!

# RIGHT:
name = "my-mcp-server"
packages = ["my_mcp_server"]  # Python uses underscores
```

**3. Missing dependencies**
```python
# Your server imports this...
from pydantic_settings import BaseSettings

# But pyproject.toml doesn't declare it
dependencies = ["mcp>=0.1.0"]  # pydantic-settings missing!

# Result: Package installs but fails at runtime
```

**4. Requiring a different Python version than declared**
```toml
requires-python = ">=3.10"
# But your code uses 3.11-specific features like tomllib without conditional import
# Users on Python 3.10 get errors at runtime
```

**5. Entry point function with required arguments**
```python
def main(config_path: str):  # WRONG: entry points can't pass arguments
    pass

# Users get: TypeError: main() missing 1 required positional argument
```

## Try With AI

**Setup**: You have a working MCP server at `/src/my_mcp_server/`. You need to package it for distribution.

**Prompt 1: Generate a complete pyproject.toml**

Ask AI: "Generate a complete pyproject.toml for an MCP server called 'my-mcp-server' that provides tools for task management. The package should:
- Require Python 3.11+
- Depend on mcp>=0.1.0, pydantic>=2.0, httpx>=0.24.0
- Have an entry point at my_mcp_server.server:main
- Include MIT license
- Have development dependencies for testing with pytest and linting with ruff

Generate the full file structure."

**What you're learning**: You understand the relationship between PyPI package names (hyphens) and Python import names (underscores). You see how entry points bridge the installation system to your code.

**Prompt 2: Validate your pyproject.toml structure**

Take the pyproject.toml from Prompt 1 and ask AI: "Review this pyproject.toml. Identify any potential issues:
1. Are all required sections present?
2. Does the entry point match our actual module structure at src/my_mcp_server/server.py?
3. Are all dependencies that the server imports actually declared?
4. Is the Python version requirement realistic for our dependencies?"

**What you're learning**: You're checking your own work like a build system would. This catches misconfigurations before users experience them.

**Prompt 3: Create the end-to-end workflow**

Ask AI: "Write a shell script that:
1. Validates pyproject.toml syntax
2. Builds the package using uv build
3. Creates a clean test environment
4. Installs the package in that environment
5. Tests that the entry point command works
6. Cleans up

Include error handling and clear output at each step."

**What you're learning**: You're automating the validation workflow. This is the foundation of CI/CD pipelines that test packages on every commit. Understanding this workflow prevents broken packages from reaching users.

**Validation**: Your script successfully completes all six steps, and the entry point command runs in the clean environment. If it fails, you've identified a configuration issue that needs fixing before publishing.

---

**Safety note during Prompt 3**: When testing entry points, use `timeout` to prevent hanging. Example: `timeout 3 my-mcp-server || true` starts the server, kills it after 3 seconds (safe shutdown test), and ignores the timeout exit code. This validates the server runs without waiting for manual interrupt.

---

**Key insight**: The relationship between what you write in `pyproject.toml` and what users experience is direct:
- Correct `name` and `version` → Users find and install specific package versions
- Correct `dependencies` → Package installs without "missing module" errors
- Correct `entry point` → Users can run your server with a single command
- Correct Python requirement → Package works on supported versions

Every field in `pyproject.toml` is a promise to users. Writing it correctly is the difference between a tool they use once and then abandon, and an MCP server they rely on in production.
