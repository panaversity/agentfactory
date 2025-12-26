---
sidebar_position: 1
title: "Project Setup with FastMCP"
description: "Scaffold your first MCP server project using Python's uv tool and the FastMCP framework"
keywords: ["MCP", "FastMCP", "Python SDK", "uv", "project setup", "server initialization", "pyproject.toml", "uvicorn"]
chapter: 38
lesson: 1
duration_minutes: 45

# HIDDEN SKILLS METADATA
skills:
  - name: "Python Project Scaffolding with uv"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can initialize a new Python project with uv, understand virtual environments, and structure project directories"

  - name: "FastMCP Server Initialization"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can create and run a basic FastMCP server, understand the initialization syntax, and verify server startup"

  - name: "Python Configuration Management"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can read and modify pyproject.toml to configure dependencies and project metadata"

  - name: "MCP Inspector Connection"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Testing & Validation"
    measurable_at_this_level: "Student can connect to a running MCP server using MCP Inspector and verify server functionality"

learning_objectives:
  - objective: "Initialize a new Python project structure with uv and understand virtual environment benefits"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student successfully creates project directory with uv and verifies virtual environment activation"

  - objective: "Configure FastMCP server with proper pyproject.toml structure and dependency declarations"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student modifies and understands pyproject.toml, can explain dependency declarations"

  - objective: "Create a minimal FastMCP server that runs successfully and responds to MCP Inspector connection"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates server.py, runs it, and verifies connection via MCP Inspector"

  - objective: "Understand the MCP protocol flow from initialization through tool registration"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student can explain what happens when MCP server starts and how it communicates with clients"

cognitive_load:
  new_concepts: 5
  assessment: "5 concepts (uv workflow, pyproject.toml structure, FastMCP initialization, uvicorn server, MCP protocol flow) matches B1 tier (7-10)"

differentiation:
  extension_for_advanced: "Explore uv's optional dependency groups for testing and development workflows; experiment with different project structures for monorepos"
  remedial_for_struggling: "Focus on uv basics first (create → add → run); defer pyproject.toml details until server works; use MCP Inspector incrementally"
---

# Project Setup with FastMCP

You've completed Chapter 37 and understand MCP fundamentally—the protocol, resources, tools, and how servers and clients communicate. You know *how* MCP works.

Now comes the transition: from understanding MCP servers to *building* them.

This lesson marks the beginning of practical MCP development. You'll move your knowledge from theory into hands-on practice. By the end of this lesson, you'll have a working MCP server running locally, connected through MCP Inspector, ready to implement your first tools.

## Why Project Structure Matters

Before you write a single line of server code, the project structure itself communicates intent. A well-organized MCP server project tells the next developer (or your future self):

- What the server's purpose is
- How dependencies are managed
- Where configuration lives
- How tests should be organized

This lesson teaches you the *standard* way to structure an MCP server project using FastMCP. This is the scaffolding every production MCP server follows.

### The Three Layers of Setup

Project setup has three distinct layers. Understanding each layer helps you debug when something goes wrong:

1. **Project Layer** — Directory structure, dependencies, and virtual environment (handled by `uv`)
2. **Server Layer** — FastMCP instantiation, server configuration, and tool/resource registration
3. **Transport Layer** — How the server communicates (stdio for CLI tools, HTTP for web services)

This lesson covers all three.

## Creating Your Project with uv

### What is uv?

`uv` is a modern Python project manager. You might have used `pip` or `pipenv` before. `uv` is faster, simpler, and handles the entire project lifecycle from initialization through dependency management.

**Output expectations:**
When you run `uv init`, you should see:
```
Created project directory at ./my-mcp-server
Created pyproject.toml
Created README.md
```

Let's start:

```bash
uv init my-mcp-server
```

**Output:**
```
Created project directory at ./my-mcp-server
Created pyproject.toml
Created README.md
```

Navigate into your project:

```bash
cd my-mcp-server
```

**Output:**
```
# (No output—you're now in the project directory)
```

Verify the structure:

```bash
ls -la
```

**Output:**
```
total 24
drwxr-xr-x   5 user  staff  160 Dec 26 10:15 .
drwxr-xr-x  12 user  staff  384 Dec 26 10:14 ..
-rw-r--r--   1 user  staff   82 Dec 26 10:15 README.md
-rw-r--r--   1 user  staff  283 Dec 26 10:15 pyproject.toml
drwxr-xr-x   3 user  staff   96 Dec 26 10:15 my_mcp_server
-rw-r--r--   1 user  staff    0 Dec 26 10:15 my_mcp_server/__init__.py
```

Notice three key things:

1. **pyproject.toml** — Configuration file declaring dependencies and project metadata
2. **my_mcp_server/** — Python package directory (underscores instead of hyphens)
3. **README.md** — Project documentation

This is the standard Python project structure.

### Understanding pyproject.toml

Open `pyproject.toml` and you'll see:

```toml
[project]
name = "my-mcp-server"
version = "0.1.0"
description = ""
requires-python = ">=3.10"
dependencies = []
```

This file declares:
- **name**: The project's name
- **version**: Current version (follow semantic versioning)
- **requires-python**: Minimum Python version MCP servers need 3.10+
- **dependencies**: Empty for now—you'll add FastMCP here

Let's add the FastMCP dependency:

```bash
uv add mcp
```

**Output:**
```
Resolved 8 packages in 0.24ms
Installed 8 packages in 0.15s
+ annotated-types==0.7.0
+ mcp==0.10.0
+ pydantic==2.9.2
+ pydantic-core==2.27.0
+ typing-extensions==4.12.2
+ uvicorn==0.30.0
+ httpx==0.27.0
+ python-dotenv==1.0.0
```

Now check `pyproject.toml`:

```bash
cat pyproject.toml
```

**Output:**
```toml
[project]
name = "my-mcp-server"
version = "0.1.0"
description = ""
requires-python = ">=3.10"
dependencies = [
    "mcp>=0.10.0",
]
```

Notice `uv` automatically added the correct dependency declaration. The `mcp` package includes FastMCP, all required serialization libraries, and the async runtime.

### Verifying the Virtual Environment

`uv` automatically created and activated a virtual environment. Verify it:

```bash
which python
```

**Output:**
```
/Users/[username]/.cache/uv/installs/cpython-3.11.8/bin/python
```

You're running Python from the virtual environment, not your system Python. This isolation prevents dependency conflicts with other projects.

## Creating Your First FastMCP Server

Now you'll create the actual server code. This is intentionally minimal—a "hello world" of MCP servers.

### Create server.py

Create a new file called `server.py` in your project directory:

```bash
touch server.py
```

Open `server.py` and add this code:

```python
#!/usr/bin/env python3
"""
My First MCP Server

A minimal FastMCP server demonstrating basic initialization.
This server does nothing yet—it just proves the server architecture works.
"""

from mcp.server.fastmcp import FastMCP

# Initialize the FastMCP server with a name
mcp = FastMCP("my_mcp_server")

if __name__ == "__main__":
    # Run the server with stdio transport
    mcp.run()
```

Let's break down what each line does:

**Line 1-5**: Shebang and docstring
- `#!/usr/bin/env python3` — Tells the system to use Python 3 to run this script
- The docstring documents what this server does

**Line 7**: Import
- `from mcp.server.fastmcp import FastMCP` — Import the FastMCP class from the installed `mcp` package

**Line 10**: Server instantiation
- `mcp = FastMCP("my_mcp_server")` — Create a server instance with the name `my_mcp_server`
- This name must match your project name (with underscores instead of hyphens)

**Line 12-14**: Main execution
- `if __name__ == "__main__":` — Run this block only when script is executed directly (not imported)
- `mcp.run()` — Start the server
- By default, uses stdio transport (communicates via standard input/output)

### Make the Script Executable

```bash
chmod +x server.py
```

**Output:**
```
# (No output—file permissions changed)
```

Verify:

```bash
ls -la server.py
```

**Output:**
```
-rwxr-xr-x   1 user  staff  389 Dec 26 10:20 server.py
```

The `x` in `-rwxr-xr-x` means the file is executable.

## Starting Your Server

### Run the Server

```bash
python server.py
```

**Output:**
```
(Server starts and waits for connections)
(No visible output—the server is listening on stdio)
```

The server is now running. It's waiting for an MCP client to connect. Don't stop it yet—leave it running in your terminal.

### Verify in Another Terminal

Open a *new* terminal window (keep the server running):

```bash
ps aux | grep server.py
```

**Output:**
```
user    12345  0.1  0.3  456789  12345 ??  S     10:20AM   0:00.15 python server.py
user    12346  0.0  0.0  408484    728 s000  S+    10:21AM   0:00.00 grep server.py
```

Your server process is running (PID 12345).

## Understanding the MCP Protocol Flow

Right now your server does nothing. Let's understand what's happening under the hood:

### Server Initialization Sequence

When you run `mcp.run()`, the FastMCP framework:

1. **Binds to stdio** — Opens standard input/output streams
2. **Advertises capabilities** — When a client connects, the server sends an `initialize` message describing what tools/resources it supports
3. **Waits for requests** — The server listens for tool invocations, resource requests, and other MCP messages
4. **Responds to requests** — When a client sends a request, the server processes it and sends a response

Right now your server advertises zero tools and zero resources. It's a valid MCP server—just not a useful one yet.

### Message Flow (Conceptual)

```
Client                                    Server
  |                                         |
  |------------ initialize request ------->|
  |<------- initialize response (caps) -----|
  |                                         |
  |<---- (client now knows server's capabilities)
  |                                         |
  |------- tool invocation request ------->|
  |<------ tool invocation response --------|
  |                                         |
```

Your current server responds to the `initialize` message saying "I have 0 tools" and that's valid. Tools come next lesson.

## Connecting via MCP Inspector

MCP Inspector is a visual tool for testing and debugging MCP servers. It connects to a running server, shows you available tools and resources, and lets you invoke tools manually.

### Install MCP Inspector

MCP Inspector is a Node.js application. Install it globally:

```bash
npm install -g @modelcontextprotocol/inspector
```

**Output:**
```
up to date, audited 1 package in 0.08s
(or similar installation message)
```

Verify installation:

```bash
mcp-inspector --version
```

**Output:**
```
@modelcontextprotocol/inspector/1.0.0 (or similar version number)
```

### Connect to Your Running Server

With your server still running (`python server.py` in the first terminal), open a new terminal and run:

```bash
mcp-inspector python server.py
```

**Output:**
```
Started server on http://localhost:5173
Open http://localhost:5173 in your browser
```

Now open your browser to `http://localhost:5173`.

You should see:

1. **Server info panel** — Shows "my_mcp_server" and connection status (green "connected")
2. **Tools section** — Empty (no tools yet)
3. **Resources section** — Empty (no resources yet)
4. **Server info output** — Shows the initialize message with server capabilities

This visual interface proves your server is working. The MCP Inspector connected successfully, received the server's capabilities, and rendered the interface.

### What You're Seeing

In the Inspector interface, you can see:

- **Connection Status**: Green dot = connected ✓
- **Server Name**: "my_mcp_server"
- **Available Tools**: Empty list (we'll add tools next lesson)
- **Available Resources**: Empty list (resources come in Lesson 4)

The fact that you have an empty but functional server is actually a checkpoint. Many MCP implementations fail at this basic setup step. You've passed it.

## Understanding the Configuration: pyproject.toml Deep Dive

Now let's review what `uv` created for us and understand each section:

### Full Current pyproject.toml

```toml
[project]
name = "my-mcp-server"
version = "0.1.0"
description = ""
requires-python = ">=3.10"
dependencies = [
    "mcp>=0.10.0",
]
```

### What Each Section Does

**[project]** — Project metadata (used by package registries and installers)
- `name`: How others will refer to your server
- `version`: Follows semantic versioning (MAJOR.MINOR.PATCH)
- `description`: What you'll add after you understand what the server does
- `requires-python`: MCP requires Python 3.10+ (async features)
- `dependencies`: List of required packages (in this case, just `mcp`)

### Optional: Add a Description

Let's add a meaningful description:

```bash
# Edit pyproject.toml
```

Change the description line from:
```toml
description = ""
```

To:
```toml
description = "My first MCP server demonstrating FastMCP basics"
```

### Optional: Add Project Metadata

Most projects also add:

```toml
authors = [{name = "Your Name", email = "your.email@example.com"}]
readme = "README.md"
```

This helps others understand who maintains the project.

## Configuring the Server Name Convention

Your server is named `my_mcp_server`. This follows the FastMCP naming convention:
- Use lowercase letters and underscores
- Use the format `{service}_mcp` or `{service}_server`
- Not tied to specific features or versions

Examples:
- `github_mcp` (for GitHub API integration)
- `slack_mcp` (for Slack integration)
- `stripe_mcp` (for payment processing)

Your server is named `my_mcp_server` because it's your first server. When you build real servers, you'll name them for the service they integrate (e.g., `jira_mcp`, `notion_mcp`, etc.).

## Practice Exercises

### Exercise 1: Modify and Verify Server Name

**Task**: Change your server name to something descriptive for a domain you care about (e.g., `weather_mcp` if you plan to add weather tools, `notes_mcp` if you'll add note-taking features).

**Steps**:
1. Edit `server.py` and change `FastMCP("my_mcp_server")` to `FastMCP("your_domain_mcp")`
2. Save the file
3. Stop your running server (Ctrl+C in the terminal)
4. Start it again: `python server.py`
5. Verify in MCP Inspector that the server name changed

**Expected output**: MCP Inspector should show your new server name in the connection panel.

### Exercise 2: Explore pyproject.toml

**Task**: Update your project's metadata to match a real project description.

**Steps**:
1. Open `pyproject.toml`
2. Add a meaningful description in the `description` field
3. Add your name and email in the `authors` field
4. Save the file
5. Verify that `uv` doesn't complain: `uv pip compile pyproject.toml`

**Expected output**: `uv` successfully validates your configuration.

### Exercise 3: Verify Virtual Environment Isolation

**Task**: Prove that your project's virtual environment is separate from your system Python.

**Steps**:
1. In your project directory, check which Python is active: `which python`
2. Note the path (it should be in `.cache/uv/` or similar)
3. Deactivate the virtual environment: `deactivate` or close the terminal
4. In a new terminal (outside the project), check Python again: `which python`
5. Navigate back to your project: `cd my-mcp-server`
6. Check Python again—it should activate automatically if you're using `uv`

**Expected behavior**: Your project always uses its own isolated Python, not your system Python.

### Exercise 4: Read and Modify the Server Script

**Task**: Add comments to `server.py` explaining what each section does.

**Steps**:
1. Open `server.py`
2. Add inline comments explaining:
   - What the shebang does
   - What the import does
   - What `FastMCP()` initialization creates
   - What `mcp.run()` does
3. Save and run: `python server.py`
4. The behavior should be identical—comments don't affect execution

**Expected output**: Server starts normally; your comments document the code.

## Troubleshooting: Common Setup Issues

### Issue: "python: command not found"

**Cause**: Your terminal isn't in the project directory or virtual environment isn't active.

**Solution**:
```bash
cd my-mcp-server
# Virtual environment activates automatically with uv
python --version  # Should show Python 3.10+
```

### Issue: "mcp: No module named 'mcp'"

**Cause**: FastMCP wasn't installed, or you're using system Python instead of the virtual environment.

**Solution**:
```bash
uv add mcp  # Install if missing
which python  # Verify you're in the project's venv
```

### Issue: MCP Inspector shows "disconnected"

**Cause**: Server crashed or isn't accepting stdio connections.

**Solution**:
```bash
# Check if server process is running
ps aux | grep server.py

# Check for startup errors
python server.py  # Run directly to see error messages
```

### Issue: Port 5173 already in use (MCP Inspector error)

**Cause**: Another service is using the port.

**Solution**:
```bash
# Find what's using port 5173
lsof -i :5173

# Kill the process or use a different port
mcp-inspector --port 5174 python server.py
```

## Summary: What You've Accomplished

You've completed the setup layer. Here's what you did:

1. **Created a Python project** with `uv init` — Established directory structure, pyproject.toml, and virtual environment
2. **Added FastMCP dependency** — Installed the official MCP SDK via `uv add mcp`
3. **Wrote server.py** — Created a minimal but valid MCP server with proper initialization
4. **Started the server** — Ran `python server.py` and verified it accepts connections
5. **Connected via MCP Inspector** — Used the visual client to verify the server is working
6. **Understood the protocol flow** — Grasped how MCP servers advertise capabilities and wait for requests

Your server currently does nothing except respond to the `initialize` message. Next lesson, you'll add tools—the actual functionality that makes this server useful.

## Next Steps

You're now ready to build on this foundation:
- **Lesson 2**: Add your first tool to the server (implement a function that MCP clients can call)
- **Lesson 3**: Handle complex parameters and validate inputs with Pydantic
- **Lesson 4**: Expose data through MCP resources
- **Lesson 5**: Bundle domain expertise in prompt templates

The setup you've done here is the same setup you'll use for all future MCP servers. You've learned the *standard way* to do this—when you build production servers at companies, this is exactly how you'll start.
