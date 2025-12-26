---
sidebar_position: 8
title: "Packaging & Distribution"
description: "Package MCP servers for installation and distribution: pyproject.toml configuration, entry points, build tools, local testing, and Claude Desktop integration"
keywords: ["MCP Packaging", "pyproject.toml", "Entry Points", "uv build", "Distribution", "Installation", "Claude Desktop Configuration", "Production Deployment"]
chapter: 38
lesson: 8
duration_minutes: 18

# HIDDEN SKILLS METADATA
skills:
  - name: "Configuring pyproject.toml for MCP Server Distribution"
    proficiency_level: "C2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write complete pyproject.toml with correct metadata, dependencies, and entry points for MCP server distribution"

  - name: "Implementing Entry Point Patterns for CLI Commands"
    proficiency_level: "C2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design entry points that start MCP servers and can be invoked from command line after installation"

  - name: "Building and Testing Python Packages Locally"
    proficiency_level: "C2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can build wheels using uv, test installations locally, and verify package contents"

  - name: "Configuring Claude Desktop for Custom MCP Servers"
    proficiency_level: "C2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can add installed MCP servers to Claude Desktop config and test client connections"

learning_objectives:
  - objective: "Understand the pyproject.toml format and configure it correctly for MCP server distribution with all required metadata"
    proficiency_level: "C2"
    bloom_level: "Understand"
    assessment_method: "Explanation of pyproject.toml sections (project, build-system) and their purpose"

  - objective: "Create entry points that expose MCP server CLI commands for installation and use"
    proficiency_level: "C2"
    bloom_level: "Apply"
    assessment_method: "Implementation of [project.scripts] entry point that starts server"

  - objective: "Build, test, and debug Python packages locally before distribution"
    proficiency_level: "C2"
    bloom_level: "Apply"
    assessment_method: "Successful uv build, installation, and execution of installed package"

  - objective: "Configure installed MCP servers in Claude Desktop and verify functionality"
    proficiency_level: "C2"
    bloom_level: "Apply"
    assessment_method: "Added server to Claude Desktop config and confirmed tools appear in Claude"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (pyproject.toml structure, project metadata, dependencies, entry points, build systems, installation testing, Claude Desktop config) within C2 limit (no artificial ceiling) ✓"

differentiation:
  extension_for_advanced: "Research packaging best practices (semantic versioning, changelog maintenance, PyPI publishing); explore building with different backends (setuptools, poetry); implement automated CI/CD for package publishing"
  remedial_for_struggling: "Start with minimal pyproject.toml; focus on entry point pattern in isolation; practice installation steps separately before combining"
---

# Packaging & Distribution

Throughout this chapter, you've built powerful MCP servers: injecting Context, calling LLMs through clients, reporting progress, controlling file access, and handling errors gracefully. Now comes a critical step that separates toy projects from **production-ready components**: packaging your server so others can install and use it.

A packaged MCP server is fundamentally different from code in a git repository. With packaging, users run **one command** to install your entire server, including dependencies, and immediately use it in Claude Desktop or other MCP clients. No cloning repos, no installing dependencies manually, no configuration headaches.

This lesson teaches you the packaging patterns that turn your MCP server code into an **installable Digital FTE component** that integrates seamlessly into Claude and other workflows.

## From Code to Package: The Transformation

When you have working MCP server code, the journey to distribution involves these steps:

1. **Define metadata** (pyproject.toml) — What is this package? Who made it? What's required?
2. **Create entry points** (command-line starters) — How does the server start after installation?
3. **Build the package** (wheel creation) — Compress code and dependencies into installable form
4. **Test locally** (installation verification) — Does it install cleanly? Does it work?
5. **Configure in clients** (Claude Desktop) — Register the server so Claude can use it
6. **Distribute** (PyPI, custom repos) — Make it available for others to install

Let's work through each step.

## The pyproject.toml: Project Metadata and Dependencies

Your `pyproject.toml` file is the **contract** between your code and the outside world. It answers fundamental questions:

- What is this project called?
- What version is it?
- What Python versions does it support?
- What dependencies must be installed?
- How do users run it after installation?

### Minimal but Complete pyproject.toml

Here's the structure for an MCP server:

```toml
[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"

[project]
name = "my-mcp-server"
version = "0.1.0"
description = "MCP server for domain-specific tasks"
readme = "README.md"
requires-python = ">=3.11"
authors = [
    {name = "Your Name", email = "you@example.com"},
]
keywords = ["mcp", "agent", "tools"]
classifiers = [
    "Development Status :: 3 - Alpha",
    "Intended Audience :: Developers",
    "License :: OSI Approved :: MIT License",
    "Programming Language :: Python :: 3",
    "Programming Language :: Python :: 3.11",
    "Programming Language :: Python :: 3.12",
]

dependencies = [
    "mcp>=1.6.0",
    "httpx>=0.27.0",
    "pydantic>=2.0.0",
]

[project.scripts]
my-mcp-server = "my_mcp_server:main"

[project.urls]
Homepage = "https://github.com/yourusername/my-mcp-server"
Repository = "https://github.com/yourusername/my-mcp-server"
Documentation = "https://github.com/yourusername/my-mcp-server/blob/main/README.md"
```

**What each section does:**

| Section | Purpose | Example |
|---------|---------|---------|
| `[build-system]` | Specifies how to build the package | `hatchling` is modern, lightweight builder |
| `[project]` | Core metadata (name, version, description) | Used by PyPI, pip, and installation tools |
| `dependencies` | Runtime requirements (what pip installs) | MCP, httpx for HTTP requests, pydantic for validation |
| `[project.scripts]` | Entry points (CLI commands created on install) | `my-mcp-server` command becomes available after `pip install` |
| `[project.urls]` | Project links (documentation, repository) | Help users find source code and docs |

### Understanding the Entry Point Pattern

The `[project.scripts]` section is how your installed server becomes executable:

```toml
[project.scripts]
my-mcp-server = "my_mcp_server:main"
```

This line says:
- When user runs `my-mcp-server` command, execute the `main()` function from module `my_mcp_server`
- During installation, pip creates a CLI wrapper script that calls this function

### Your Server's Entry Point Module

To make this work, you need a `main()` function in your package:

```python
# my_mcp_server/__init__.py
from mcp.server.fastmcp import FastMCP

mcp = FastMCP("my-server")

# ... define your tools, resources, prompts ...

def main():
    """Entry point for installed server."""
    mcp.run()

if __name__ == "__main__":
    main()
```

**Why this structure?**

1. **FastMCP global**: Your `mcp` object is defined at module level, so all decorators (@mcp.tool, @mcp.resource) can attach to it
2. **main() function**: Called by the entry point when user runs `my-mcp-server` command
3. **if __name__ == "__main__"**: Allows testing by running the module directly: `python -m my_mcp_server`

### Specifying Dependencies Precisely

Your `dependencies` list should include only what's **essential** for the server to run:

```toml
dependencies = [
    "mcp>=1.6.0",        # MCP framework (minimum version)
    "httpx>=0.27.0",     # For HTTP requests in your tools
    "pydantic>=2.0.0",   # Data validation (MCP requires this)
]
```

**Dependency version specifications:**

| Format | Meaning | Use Case |
|--------|---------|----------|
| `mcp>=1.6.0` | At least 1.6.0 | Framework is stable; new minor versions are safe |
| `mcp~=1.6.0` | 1.6.x (not 1.7) | Stricter; only patch version changes allowed |
| `mcp==1.6.0` | Exactly 1.6.0 | Most restrictive; pinned version |
| `mcp>=1.6.0,<2.0` | 1.6.0 to 1.x | Common pattern; avoid major version jumps |

For MCP servers, use `>=1.6.0` for framework dependencies. MCP follows semantic versioning, so minor versions are backward compatible.

## Building Your Package

The `uv build` command creates a **wheel** (`.whl` file)—a compressed archive containing your code and a manifest.

### Step 1: Prepare Project Structure

Ensure your project layout matches this pattern:

```
my-mcp-server/
├── my_mcp_server/           # Package directory (matches [project] name with underscores)
│   ├── __init__.py          # Contains FastMCP instance and main()
│   ├── tools.py             # Tool implementations
│   ├── resources.py         # Resource implementations
│   └── prompts.py           # Prompt definitions
├── pyproject.toml           # Package metadata
├── README.md                # Documentation
└── LICENSE                  # License (e.g., MIT)
```

**Critical note**: The directory name (`my_mcp_server`) must match your `[project] name` with hyphens converted to underscores.

### Step 2: Build the Wheel

```bash
# Navigate to project directory
cd my-mcp-server

# Build the package
uv build

# Output:
# Building sdist (source distribution)...
# Building wheel...
# Successfully built my_mcp_server-0.1.0-py3-none-any.whl
```

The `uv build` command creates two artifacts:
- `*.tar.gz` (source distribution) — for pip to install from source if needed
- `*.whl` (wheel) — binary distribution, faster to install

### Step 3: Inspect Package Contents

```bash
# List files in the wheel
unzip -l dist/my_mcp_server-0.1.0-py3-none-any.whl | head -30

# Output shows your code, metadata, entry points:
# my_mcp_server/__init__.py
# my_mcp_server/tools.py
# my_mcp_server-0.1.0.dist-info/entry_points.txt  ← Entry point manifest
# my_mcp_server-0.1.0.dist-info/METADATA           ← Project metadata from pyproject.toml
```

## Local Testing: Installation and Verification

Before distributing to users, test your package locally:

### Step 1: Install the Built Package

```bash
# Install the wheel (requires pip or uv)
uv pip install dist/my_mcp_server-0.1.0-py3-none-any.whl

# Or with pip:
pip install dist/my_mcp_server-0.1.0-py3-none-any.whl
```

After installation, the entry point is available as a command:

```bash
# Test that the command exists
which my-mcp-server
# Output: /path/to/venv/bin/my-mcp-server

# Check what it does
my-mcp-server --help
# Output: Usage: my-mcp-server [options]
#         MCP server for domain-specific tasks
```

### Step 2: Verify the Server Runs

```bash
# Start the server
my-mcp-server

# Expected output:
# Starting MCP server...
# Server running on stdio transport
# (Server waits for client connections; press Ctrl+C to stop)
```

The server should start without errors. If it crashes:

1. **Check imports**: Verify all modules in `my_mcp_server/` can be imported
2. **Check dependencies**: Confirm all `dependencies` from pyproject.toml are installed
3. **Check main()**: Ensure `main()` function is defined and calls `mcp.run()`

### Step 3: Verify Entry Point Works

```bash
# List installed packages
pip show my-mcp-server

# Expected output:
# Name: my-mcp-server
# Version: 0.1.0
# Summary: MCP server for domain-specific tasks
# Location: /path/to/site-packages
# Requires: mcp, httpx, pydantic

# List available entry points
pip show --files my-mcp-server | grep entry_points

# Or check directly
cat /path/to/site-packages/my_mcp_server-0.1.0.dist-info/entry_points.txt

# Expected output:
# [console_scripts]
# my-mcp-server = my_mcp_server:main
```

## Claude Desktop Configuration

After installing your MCP server, register it with Claude Desktop so Claude can discover and use its tools.

### Step 1: Locate Claude Desktop Config

Claude Desktop stores server configurations in a JSON file:

**macOS/Linux:**
```bash
~/.config/Claude/claude_desktop_config.json
```

**Windows:**
```
%APPDATA%\Claude\claude_desktop_config.json
```

### Step 2: Add Your Server to the Config

Edit the config file (or create it if it doesn't exist):

```json
{
  "mcpServers": {
    "my-server": {
      "command": "my-mcp-server"
    }
  }
}
```

**Configuration breakdown:**

```json
{
  "mcpServers": {                    // Top-level section for all MCP servers
    "my-server": {                   // Server ID (internal name, can be anything)
      "command": "my-mcp-server"     // Exact command from [project.scripts]
    }
  }
}
```

### Step 3: Restart Claude Desktop

Close and reopen Claude Desktop. After restart, Claude will:
1. Start your MCP server via the `my-mcp-server` command
2. Discover all tools, resources, and prompts your server defines
3. Make them available in the Claude interface

### Step 4: Verify Tools Appear

In Claude Desktop, check the bottom-left corner. You should see:
- A "Tools" or "Integrations" menu
- Your server name (e.g., "my-server")
- List of available tools from your server

If tools don't appear:
1. Restart Claude Desktop completely
2. Check that `my-mcp-server` command is accessible (can you run it from terminal?)
3. Review your server's tool definitions (@mcp.tool decorators)

## Distribution Strategies

Once your package is built and tested locally, you have options for distribution:

### Option 1: Share as Wheel File (Direct Installation)

Users can install directly from your wheel:

```bash
pip install my-mcp-server-0.1.0-py3-none-any.whl
```

**Advantages:**
- No PyPI account needed
- Users can install from GitHub releases, S3, etc.

**Disadvantages:**
- Users need full filename (no `pip install my-mcp-server` without version)
- No automatic updates

### Option 2: Publish to PyPI (Official Python Package Index)

Upload your package to PyPI so users can install with a single command:

```bash
# User installation (after you publish):
pip install my-mcp-server
```

**Advantages:**
- Standard Python installation experience
- Automatic dependency resolution
- `pip install my-mcp-server` without version numbers
- PyPI hosts documentation

**Disadvantages:**
- Requires PyPI account
- Public name registration (first come, first served)
- Version management responsibility

Publishing to PyPI involves:
1. Creating a PyPI account (pypi.org)
2. Building and signing your package
3. Uploading with `twine` or `uv publish`

(Detailed PyPI publishing is beyond this lesson's scope; it's typically handled in a "Distribution & Publishing" chapter at the end of the course.)

### Option 3: Private Distribution

For organizational use, distribute through:
- **GitHub releases**: Users download `.whl` from your releases page
- **Private PyPI**: Enterprise-grade package repository
- **Internal packages repo**: Organization-hosted mirror

## Try With AI

The patterns in this lesson are straightforward, but attention to detail matters. Work with AI to verify your packaging is correct.

### Prompt 1: Validate Your pyproject.toml

**What you're learning**: How to identify potential configuration issues before they cause installation failures.

Ask Claude:

```
I've created a pyproject.toml for my MCP server. Please review it for completeness and correctness:

[paste your pyproject.toml]

Check:
1. Is [build-system] correctly specified?
2. Does [project] have all required fields (name, version, description, requires-python)?
3. Are dependencies correctly specified with version constraints?
4. Is the [project.scripts] entry point correctly formatted?
5. Are there any common mistakes or missing fields?
```

**Expected result**: Claude identifies any structural issues, missing metadata, or version constraint problems before you try to build.

### Prompt 2: Debug Installation Failures

**What you're learning**: How to interpret installation errors and fix underlying packaging issues.

If installation fails, ask Claude:

```
My MCP server installation failed with this error:

[paste error message]

My pyproject.toml is:
[paste pyproject.toml]

My package structure is:
[paste directory listing]

My __init__.py contains:
[paste __init__.py code]

What's wrong, and how do I fix it?
```

**Expected result**: Claude traces from the error to the root cause (missing file, incorrect module path, dependency issue, etc.) and suggests specific fixes.

### Prompt 3: Generate Complete pyproject.toml

**What you're learning**: How to leverage AI to generate correctly-formatted packaging boilerplate while you focus on your domain logic.

Ask Claude:

```
Generate a complete pyproject.toml for an MCP server with these specifications:

- Name: research-assistant
- Version: 0.2.1
- Description: MCP server that performs research tasks
- Python requirement: 3.11+
- Dependencies: mcp>=1.6.0, httpx>=0.27.0, pydantic>=2.0.0, requests>=2.31.0
- Entry point command: research-assistant
- Author: Your Name, email@example.com
- License: MIT
- GitHub: https://github.com/yourusername/research-assistant

Format it ready to copy into a new pyproject.toml file.
```

**Expected result**: Claude generates a complete, properly-formatted pyproject.toml that you can immediately use as your project's packaging configuration.
