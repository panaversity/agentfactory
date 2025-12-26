---
sidebar_position: 8
title: "Debugging and Troubleshooting MCP"
chapter: 37
lesson: 8
duration_minutes: 12
description: "Master MCP debugging using the Inspector tool, common error patterns, and systematic troubleshooting workflows for production reliability."
keywords: [MCP, debugging, inspector, troubleshooting, connection, JSON-RPC, logging, error diagnosis]

# HIDDEN SKILLS METADATA
skills:
  - name: "Using the MCP Inspector for Interactive Debugging"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can launch the MCP Inspector, connect to a server, list tools, execute tools with test inputs, and interpret Inspector results to diagnose failures"

  - name: "Diagnosing MCP Connection Problems Systematically"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify whether failures are caused by server startup, tool discovery, authentication, or tool execution; use appropriate diagnostic steps for each category"

  - name: "Understanding stdio vs HTTP Debugging Differences"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can explain why logging requirements differ between stdio servers (logs to stderr) and HTTP servers (network inspection); choose appropriate diagnostic tools for each"

  - name: "Interpreting JSON-RPC Message Flow"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can read JSON-RPC request/response pairs, identify message structure errors, and trace how arguments flow from client through server to execution"

  - name: "Implementing Error Recovery Strategies for Production MCP"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "System Thinking"
    measurable_at_this_level: "Student can design retry logic, timeout handling, and fallback patterns for unreliable MCP connections; anticipate failure modes in production environments"

learning_objectives:
  - objective: "Understand that debugging MCP failures requires a systematic approach: diagnose whether the failure is in server startup, tool discovery, configuration, or tool execution"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Given a failing MCP scenario, student correctly identifies the failure category without running code"

  - objective: "Apply the MCP Inspector as a primary debugging tool: launch it, connect to servers, list tools, and execute tools with test inputs to isolate failures"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Hands-on: Use Inspector to diagnose and fix a misconfigured MCP server; verify tools are discoverable and callable"

  - objective: "Diagnose common MCP error patterns (connection refused, tool not found, invalid arguments, authentication failed) and apply targeted fixes"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Error pattern matching exercise: Given error messages, identify root cause and recommend fix from decision tree"

  - objective: "Understand logging requirements differ between stdio (stderr redirection) and HTTP (network inspection) servers; configure appropriate logging for each transport"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Scenario analysis: Given a server type, identify correct logging approach and potential issues with incorrect logging"

  - objective: "Use Inspector workflow as systematic validation: start → connect → list → test → verify to gain confidence in MCP server reliability before production deployment"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Complete full Inspector workflow on provided server; document verification results for each step"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (Inspector tool, stdio vs HTTP debugging, JSON-RPC interpretation, connection troubleshooting, common errors, error recovery) within B1 limit (7-10 concepts) ✓ — Concepts naturally chunk: debugging approach (1), tool categories (Inspector, transport types = 1 chunk), error patterns (1), recovery strategies (1)"

differentiation:
  extension_for_advanced: "Research Inspector source code; analyze how it implements JSON-RPC client logic. Design advanced logging infrastructure for multi-server orchestration with correlation IDs. Build custom Inspector extension for specialized server types."
  remedial_for_struggling: "Focus on the four failure categories (startup, discovery, configuration, execution); practice Inspector workflow step-by-step on filesystem server (simplest case) before moving to complex servers. Use decision tree repeatedly until pattern becomes automatic."
---

# Debugging and Troubleshooting MCP

You've configured MCP servers. Tools are loaded. Your agent starts making requests. And then... nothing works the way you expected.

Maybe the server won't start. Maybe it starts but tools don't appear. Maybe tools appear but fail when called. Maybe they succeed but return the wrong data. The failures can be subtle—a typo in a configuration path, an environment variable not expanding, an authentication header missing, a timeout exceeded.

**In production environments, silent failures are worse than loud ones.** An MCP server that crashes on startup gives you an error message. An MCP server that returns wrong data while appearing to work successfully is harder to detect and more dangerous.

This lesson teaches you to debug systematically. You'll learn the MCP Inspector—a tool designed specifically for testing MCP servers in isolation. You'll recognize common error patterns and their causes. You'll understand why debugging differs between stdio and HTTP servers. By the end, you'll have confidence that your MCP servers are working correctly before deploying them to production.

## Understanding the Debugging Problem

Before diving into tools, let's understand what makes MCP debugging different from traditional software debugging.

### Three Layers of Potential Failure

| Layer | What Fails | How You Know | What You Debug |
|-------|-----------|-------------|----------------|
| **Server Startup** | Server process won't launch | Process exits immediately; exit code nonzero | Server installation, dependencies, configuration parsing |
| **Tool Discovery** | Server starts but tools don't appear | Client lists tools; list is empty or incomplete | Environment variables, authentication, permissions |
| **Tool Execution** | Tools exist but calls fail | Error on tool invocation or timeout | Tool arguments, external service connectivity, API credentials |

Most debugging tools address one layer. **The MCP Inspector addresses all three**—it's designed to test the entire MCP pipeline systematically.

### Why stdio and HTTP Debugging Differ

**stdio servers** (local Python/Node processes):
- Communication through stdin/stdout
- Logs must go to stderr (stdout is reserved for protocol)
- Failures are process-level (crashes, exit codes)
- Testing requires launching subprocesses

**HTTP servers** (remote or containerized):
- Communication over HTTP/SSE
- Logs go to server's logging infrastructure
- Failures are request-level (timeouts, 500s, CORS)
- Testing requires network inspection

This fundamental difference means your debugging approach must adapt to the transport type.

## The MCP Inspector: Interactive Debugging

The MCP Inspector is a browser-based debugging tool built into the MCP ecosystem. It's the primary way to test MCP servers in isolation, before integrating them into Claude Code or other clients.

### Starting the Inspector

The Inspector command differs depending on your server's language:

**For any Node.js server**:
```bash
npx @modelcontextprotocol/inspector /path/to/server.js
```

**Output:**
```
Model Context Protocol Inspector
Starting inspector on http://localhost:5173
Open http://localhost:5173 in your browser
```

**For Python servers**:
```bash
mcp dev your_server.py
```

**Output:**
```
MCP Inspector starting
Listening on http://localhost:5000
Open browser to http://localhost:5000
```

The Inspector opens a web interface where you can:
1. Connect to a running MCP server
2. List all available tools
3. Call tools with test inputs
4. View request/response pairs
5. Inspect error messages

**Output:**
```
Output: {"result": "success", "data": [...]}
Raw Response: {"jsonrpc": "2.0", "id": 1, "result": {...}}
```

### Inspector Workflow: The Five-Step Validation

Use this workflow to systematically validate any MCP server:

**Step 1: Start Inspector and Connect**
```bash
mcp dev your_server.py
# Open http://localhost:5000
# Click "Connect"
```

**Check**: Does the Inspector connect and display "Connected to MCP Server"? If not, the server isn't running or the port is wrong.

**Step 2: List Tools**
```
Click "Tools" tab
```

**Check**: Do your expected tools appear in the list? Count matches your server's tool definitions?

| Expected Tool | Result | Status |
|--------------|--------|--------|
| `list_files` | appears, schema correct | ✓ PASS |
| `read_file` | appears, schema correct | ✓ PASS |
| `write_file` | missing from list | ✗ FAIL |

**Step 3: Test with Simple Input**
```
Select a tool: list_files
Input: (leave empty or minimal)
Click "Call Tool"
```

**Check**: Does the tool execute? What's the response?
- Success: Tool returns valid result, no errors
- Failure: Tool returns error message explaining what went wrong

**Step 4: Test with Complex Input**
```
Select tool: read_file
Input: {"path": "/etc/passwd"}
Click "Call Tool"
```

**Check**: Does tool behave correctly with realistic inputs? Does it:
- Accept valid arguments?
- Reject invalid arguments with clear error?
- Handle edge cases (empty paths, nonexistent files)?

**Step 5: Verify Error Messages**
```
Intentionally provide bad input
Example: read_file with {"path": ""}
Click "Call Tool"
```

**Check**: Is the error message helpful? Does it guide toward the fix?
- Good: "Path cannot be empty. Expected: /absolute/path/to/file"
- Bad: "Error" or "Invalid"

### Inspector Output: Understanding Response Structure

When you call a tool in the Inspector, you see both the formatted result and the raw JSON-RPC message:

```json
**Formatted Result:**
{
  "content": "File contents here"
}

**Raw JSON-RPC Response:**
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "content": "File contents here"
  }
}
```

The raw response reveals the protocol layer:
- `"jsonrpc": "2.0"` — Protocol version
- `"id": 1` — Matches your request ID
- `"result"` — Tool succeeded (vs. `"error"` if it failed)

If you see an error instead:
```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "error": {
    "code": -32602,
    "message": "Invalid params: location must be a string"
  }
}
```

The error tells you:
- `code: -32602` — Parameter validation error (standard JSON-RPC error code)
- `message` — Specific problem (location expected string, got something else)

## Common MCP Error Patterns

Most MCP debugging falls into one of these categories. Learning to recognize them saves hours of troubleshooting.

### Error Pattern Reference Table

| Error | What Happened | Most Likely Cause | How to Fix |
|-------|--------------|------------------|-----------|
| **Connection refused** | Client can't reach server | Server not running, wrong port, firewall blocking | `mcp dev server.py` in correct directory; verify port not in use |
| **Tool not found** | Tool appears in list but calls fail | Tool name mismatch between definition and call | Check exact tool name in Inspector vs. what client sends |
| **Invalid arguments** | Tool returns parameter validation error | Arguments don't match `inputSchema` | Validate argument types match schema; use Inspector to test with correct format |
| **Authentication failed** | Tool fails with auth error (e.g., GitHub token invalid) | Environment variable not set or wrong token | Check `echo $GITHUB_TOKEN`; verify token has required permissions |
| **Silent failure (no output)** | Tool runs but produces no output or timeout | stdout logging instead of stderr; network timeout | Redirect logs to stderr; increase timeout; check network connectivity |
| **CORS error (HTTP servers)** | Browser can't reach HTTP server | Server not configured for CORS; browser requesting from wrong origin | Add CORS headers; verify Origin header matches server allowlist |
| **File path error** | Tool can't find file or directory | Relative path vs. absolute path mismatch | Use absolute paths or verify working directory; print actual resolved path in error |

### Detailed Examples

**Example 1: Tool Not Found**

```bash
# Inspector shows no tools
# But server code defines them

# Wrong: Forgetting to register tool
class MyServer:
    def __init__(self):
        # Missing: self.register_tool(...)
        pass

# Fix: Register tools in initialization
class MyServer:
    def __init__(self):
        self.register_tool("list_files", self.list_files)
        self.register_tool("read_file", self.read_file)
```

**Example 2: Invalid Arguments**

```bash
# Inspector: read_file tool fails with validation error
# Expected schema: {"path": string}
# Sent: {"path": 123}  # number instead of string

# Fix: Ensure arguments match schema
# Schema says "path": {"type": "string"}
# Must call with: {"path": "/absolute/path"}
```

**Example 3: Environment Variable Not Set**

```bash
# Inspector: GitHub tool fails with "Authentication failed"
# Code tries to use: os.environ["GITHUB_TOKEN"]

# Problem: Variable not in environment
$ echo $GITHUB_TOKEN
# (empty)

# Fix: Set before running Inspector
$ export GITHUB_TOKEN="ghp_xxxxxxxxxxxxxxxxxxxxxxxxxxxx"
$ mcp dev github_server.py
```

**Example 4: Logging in Wrong Stream**

```python
# Wrong: Logs go to stdout (breaks protocol)
print("DEBUG: Attempting to authenticate...")  # Goes to stdout!

# Right: Logs go to stderr (doesn't interfere)
import sys
print("DEBUG: Attempting to authenticate...", file=sys.stderr)

# Or use logging module (defaults to stderr)
import logging
logging.info("Attempting to authenticate...")
```

## Debugging stdio vs. HTTP Servers

The debugging approach differs significantly based on transport type.

### stdio Servers (Python/Node local processes)

**How they communicate**: stdin/stdout pipes

**Why debugging differs**: The protocol uses stdout, so any logging to stdout breaks the protocol. Tools must send logs to stderr.

**Debugging checklist**:

1. **Server startup issues**
   ```bash
   mcp dev your_server.py 2>&1 | grep -i error
   # Capture both stdout (protocol) and stderr (logs)
   ```

2. **Tool not appearing**
   ```bash
   # Check server initialization
   # Add debug logging to stderr:
   logging.debug("Registering tool: list_files", file=sys.stderr)
   ```

3. **Tool execution failures**
   ```bash
   # Add try/catch with detailed stderr logging
   try:
       result = handle_request()
   except Exception as e:
       logging.error(f"Tool failed: {e}", file=sys.stderr)
       raise
   ```

**Example logging configuration for Python**:
```python
import logging
import sys

# Configure logging to stderr
logging.basicConfig(
    stream=sys.stderr,  # CRITICAL: Not stdout!
    level=logging.DEBUG,
    format='%(asctime)s [%(levelname)s] %(message)s'
)

# Now safe to use logging
logging.debug("Server starting...")
logging.info("Tool registered: list_files")
logging.error("Authentication failed")
```

### HTTP Servers (Remote or containerized)

**How they communicate**: HTTP requests/responses

**Why debugging differs**: Network involved, no direct access to stdout/stderr, CORS restrictions possible.

**Debugging checklist**:

1. **Connection issues**
   ```bash
   # Test network connectivity
   curl -X POST http://localhost:8000/mcp \
     -H "Content-Type: application/json" \
     -d '{"jsonrpc":"2.0","method":"initialize"}'
   ```

2. **CORS errors**
   ```
   # Error: "No 'Access-Control-Allow-Origin' header"
   # Fix: Ensure server includes CORS headers
   ```

3. **Request/response inspection**
   ```
   # Use browser DevTools → Network tab
   # Watch HTTP requests to server
   # Inspect request headers, response status, body
   ```

4. **Server-side logging**
   ```bash
   # Access server logs (depends on deployment)
   # Docker: docker logs <container>
   # Cloud: provider's log viewer
   # Local: check log file configuration
   ```

## Systematic Troubleshooting Decision Tree

When something breaks, follow this decision tree to isolate the problem:

```
MCP Server Problem?
│
├─ Server starts?
│  ├─ NO → Check configuration syntax (JSON validation)
│  │       Check dependencies installed (pip install / npm install)
│  │       Check file paths (absolute vs. relative)
│  │
│  └─ YES → Tools appear in Inspector?
│     ├─ NO → Check environment variables (echo $VAR_NAME)
│     │       Check server initialization (tools registered?)
│     │       Check authentication (API keys set?)
│     │
│     └─ YES → Tools execute successfully?
│        ├─ NO → Check inputSchema matches arguments
│        │       Check external service connectivity
│        │       Check tool implementation for bugs
│        │
│        └─ YES → Results correct?
│           ├─ NO → Tool logic error (debug tool code)
│           │
│           └─ YES → Server ready for production
```

## Try With AI

**Setup:** You have three MCP servers with different failure modes. Use Claude or another LLM to help diagnose and fix each one.

**Prompt 1: Connection Diagnosis**

```
I'm trying to debug an MCP server. When I run:
mcp dev my_weather_server.py

I get:
ModuleNotFoundError: No module named 'requests'

What's wrong and how do I fix it?

Then, ask me: What does "requests" module do? Why might it be used in an MCP weather server?
```

**What you're learning**: Recognizing server startup failures vs. runtime failures; understanding dependency issues as a category of failure.

**Prompt 2: Tool Discovery Investigation**

```
I have an MCP server that starts successfully. When I open the Inspector and connect, the server appears connected but the Tools tab is empty. No tools are listed.

Here's the server code structure:
- Server class has __init__ method
- __init__ calls super().__init__()
- Server class has several methods like list_weather, get_temperature

Why would tools not appear even though the server runs?

Also tell me: What's the relationship between the methods in my server class and the tools that appear in the Inspector?
```

**What you're learning**: Tool registration as distinct from tool definition; understanding why code can run without errors but still produce no tools.

**Prompt 3: Error Message Interpretation**

```
I'm using the Inspector to test my file_reader tool. When I call it with:
{"path": "/home/user/test.txt"}

The Inspector returns:
{
  "jsonrpc": "2.0",
  "id": 3,
  "error": {
    "code": -32602,
    "message": "Invalid params: 'path' is required but got string"
  }
}

This error doesn't make sense—I did provide path as a string. What's really happening?

Then ask me: If you were designing an error message, how would you make this clearer?
```

**What you're learning**: Reading JSON-RPC error codes; understanding schema validation failures; recognizing when error messages mislead rather than help.

---

✓ **You're now equipped to debug MCP systematically.** You understand the Inspector workflow, recognize error patterns, and know how to approach different failure modes. In production, these skills prevent silent failures that would otherwise go undetected until a customer reports data corruption or missed workflow triggers.

**In the capstone project, you'll orchestrate multiple MCP servers in a production configuration. Everything you've learned—architecture, configuration, and debugging—converges into a reliable, production-ready system.**
