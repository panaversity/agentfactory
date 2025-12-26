---
sidebar_position: 7
title: "Testing & Debugging Your MCP Server"
description: "Master MCP Inspector and server debugging workflows to validate tools, resources, and error handling"
keywords: ["MCP", "testing", "debugging", "MCP Inspector", "error handling", "server logs", "validation", "development workflow"]
chapter: 38
lesson: 7
duration_minutes: 50

# HIDDEN SKILLS METADATA
skills:
  - name: "MCP Inspector Interactive Testing"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Testing & Validation"
    measurable_at_this_level: "Student can launch MCP Inspector, connect to a running server, invoke tools interactively, and validate server responses"

  - name: "Server Log Analysis"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Debugging"
    measurable_at_this_level: "Student can read server stderr output, identify error sources, distinguish between validation errors and runtime exceptions"

  - name: "Common MCP Error Recognition"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can recognize 5 common error patterns (startup, tool registration, validation, encoding, exception) and apply corresponding fixes"

  - name: "Iterative Debugging Workflow"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can reproduce errors, diagnose root causes, apply fixes, and validate corrections systematically"

learning_objectives:
  - objective: "Use MCP Inspector to interactively test tools and resources with different parameters"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student successfully launches Inspector, invokes multiple tools, and validates responses"

  - objective: "Read and interpret server error messages to diagnose validation, encoding, and runtime failures"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Student identifies error source in server logs and explains fix strategy"

  - objective: "Recognize and fix 5 common MCP server errors (startup, registration, validation, encoding, exception)"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Given broken server code, student identifies error pattern and applies corresponding fix"

  - objective: "Apply systematic debugging workflow: reproduce → diagnose → fix → validate"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student debugs broken server code using full workflow, documents findings"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (Inspector usage, error categories, log reading, debugging workflow, common patterns, validation cycles) matches B1 tier (7-10)"

differentiation:
  extension_for_advanced: "Build custom Inspector clients using MCP SDK to automate testing; create test fixtures for different error scenarios; implement monitoring for production servers"
  remedial_for_struggling: "Focus on Inspector basics first (launch → connect → invoke); defer error analysis until comfortable with interactive testing; use provided error examples before reading logs"
---

# Testing & Debugging Your MCP Server

You've built tools, resources, and prompts. Your server works locally. Now comes the harder part: **making sure it actually works consistently, reliably, and correctly**.

Debugging an MCP server is different from debugging regular Python. Your server isn't running in the REPL where you can inspect variables directly. It's running in the background, communicating through JSON-RPC. Errors happen at the interface—parameter validation, JSON encoding, type mismatches—and the error messages can be cryptic.

This lesson teaches you the debugging workflow that production MCP developers use: interactive testing through MCP Inspector, systematic log analysis, and rapid iteration cycles. By the end, you'll recognize common error patterns instantly and fix them with confidence.

## The Debugging Mindset

Debugging an MCP server requires thinking at two levels simultaneously:

1. **Your code level**: Python logic, exceptions, type hints
2. **The protocol level**: JSON-RPC messages, schema validation, serialization

A tool might work perfectly when you call it directly in Python, but fail when an agent invokes it through the MCP protocol. The error isn't in your logic—it's in how your function signature communicates intent to the protocol.

This lesson teaches you to think at both levels.

## Part 1: MCP Inspector as Your Debugging Headquarters

MCP Inspector is a visual debugging tool included with the MCP Python SDK. It connects to a running MCP server and lets you interact with tools and resources interactively, seeing exactly what parameters you're sending and what responses come back.

### Launching Inspector

First, make sure your server is running:

```bash
# In terminal 1: Start your server
uv run python src/server.py
```

Then in a **separate terminal**, launch Inspector:

```bash
# In terminal 2: Launch Inspector
npx @modelcontextprotocol/inspector uv run python src/server.py
```

This command:
1. Starts a fresh instance of your server
2. Connects the Inspector UI to that instance
3. Opens `http://localhost:5173` in your browser

**Expected output:**
```
Inspector running at http://localhost:5173
Waiting for server connection...
[Server starts and connects]
```

Once connected, you'll see:
- **Tools** section: List of all registered tools with their schemas
- **Resources** section: List of all available resources
- **Testing panel**: Where you invoke tools and see live responses

### Your First Interactive Test

Let's test the `calculate` tool we built in previous lessons. In the Testing panel:

1. **Select a tool**: Click "calculate" from the Tools list
2. **Fill parameters**:
   - `operation`: "add"
   - `a`: 5
   - `b`: 3
3. **Invoke**: Click the blue "Call" button

**Expected response:**
```json
{
  "result": 8,
  "cached": false
}
```

If you see this response, your tool works. If you see an error, the error message tells you exactly what went wrong.

### Reading Inspector Error Messages

Inspector catches and displays every error your server encounters. Error messages have this structure:

```
Error: [Category] - [Message]

Details:
[Full error trace]
```

**Example 1: Parameter Validation Failure**
```
Error: InvalidRequest - Tool parameter 'operation' must be one of: ['add', 'subtract', 'multiply', 'divide']. Got: 'power'
```

**Your action**: Operation "power" isn't in the allowed list. Either use a valid operation or update the tool to accept "power".

**Example 2: Type Mismatch**
```
Error: InvalidRequest - Parameter 'a' expected type number, got string: "hello"
```

**Your action**: The field `a` expects a number (int or float). You sent a string. Check your parameter types.

**Example 3: Missing Required Parameter**
```
Error: InvalidRequest - Parameter 'operation' is required but not provided
```

**Your action**: You forgot to fill in the "operation" field. It's marked required in the schema.

### The Inspector Debugging Cycle

Effective debugging follows this pattern:

1. **Set up**: Server running, Inspector open
2. **Invoke**: Call a tool with test parameters
3. **Observe**: Check response or error
4. **Diagnose**: Read error message carefully
5. **Fix**: Update code based on diagnosis
6. **Reload**: Press "Reload Server" in Inspector (or restart terminal)
7. **Retry**: Invoke again with same parameters to verify fix

**Key principle**: Never proceed to the next parameter until the current one works. This isolates errors.

## Part 2: Common MCP Server Errors & Solutions

As you build and test, you'll encounter patterns. Here are the five most common:

### Error Pattern 1: Server Won't Start (Startup Failure)

**Symptoms**:
- Inspector says "Waiting for server connection..." but never connects
- Terminal shows Python exception before server even starts

**Common causes**:
- Missing environment variable (e.g., API key)
- Syntax error in `src/server.py`
- Missing import statement
- Port already in use

**Diagnostic command**:
```bash
# Run server directly to see startup errors
python src/server.py
```

**Example error**:
```
ImportError: No module named 'mcp.server.fastmcp'
```

**Fix**: Install missing dependency:
```bash
uv add mcp
```

**Prevention**: Add fail-fast validation at server startup:

```python
from mcp.server.fastmcp import FastMCP
import os

# Validate environment BEFORE starting server
if not os.getenv("ANTHROPIC_API_KEY"):
    raise RuntimeError(
        "ANTHROPIC_API_KEY not set. Set it with: "
        "export ANTHROPIC_API_KEY=sk-..."
    )

mcp = FastMCP("my_server")
```

### Error Pattern 2: Tool Not Appearing in Inspector

**Symptoms**:
- Server connects successfully
- Tool list is empty or missing your new tool
- Inspector shows other tools but not yours

**Common causes**:
- `@mcp.tool()` decorator missing
- Decorator syntax wrong (`@mcp.tool` vs `@mcp.tool()`)
- Function name is `_private()` (leading underscore hides from MCP)
- Server reloaded but Inspector cache not cleared

**Diagnostic checklist**:
1. Does your function have `@mcp.tool()` decorator?
2. Is it `@mcp.tool()` with parentheses, not `@mcp.tool`?
3. Is it a public function (no leading underscore)?

**Example fix**:

```python
# WRONG: Missing parentheses
@mcp.tool
def calculate(a: int, b: int) -> int:
    return a + b

# RIGHT: Parentheses required
@mcp.tool()
def calculate(a: int, b: int) -> int:
    return a + b
```

**Prevention**: After adding a new tool, always:
1. Save file
2. Reload Inspector
3. Verify tool appears in list
4. Try invoking before moving to next feature

### Error Pattern 3: Parameter Validation Failure

**Symptoms**:
- Tool appears in Inspector
- Inspector shows error when you try to invoke
- Error mentions parameter type or constraint

**Common causes**:
- Type hint doesn't match your intent
- Missing docstring for parameter (becomes "No description" in schema)
- Overly strict validation (Union types, Literal enums)
- Parameter name mismatch between function signature and invocation

**Example error**:
```
Error: InvalidRequest - Parameter 'priority' must be one of: ['high', 'medium', 'low']. Got: 'urgent'
```

**Root cause**: The tool accepts a `Literal["high", "medium", "low"]` but you sent "urgent".

**Fix options**:
- Send correct value: Use "high" instead of "urgent"
- Expand tool: Update Literal to `Literal["high", "medium", "low", "urgent"]`

**Example code**:

```python
from typing import Literal

@mcp.tool()
def create_task(
    title: str,
    priority: Literal["high", "medium", "low"] = "medium"
) -> dict:
    """Create a task.

    Args:
        title: Task title (required)
        priority: Priority level - choose from high, medium, or low
    """
    return {"task": title, "priority": priority}
```

**Prevention**: Document parameter constraints in docstrings:

```python
@mcp.tool()
def create_task(
    title: str,
    priority: Literal["high", "medium", "low"] = "medium"
) -> dict:
    """Create a task.

    Args:
        title: Task title (required, 1-200 characters)
        priority: Priority level. Must be one of: high, medium, low

    Returns:
        Dictionary with created task id and metadata
    """
```

### Error Pattern 4: Garbled Output / JSON Encoding Failure

**Symptoms**:
- Tool invocation seems to work
- Response looks corrupted or empty
- Inspector shows `null` or incomplete data

**Common cause**: You're printing to stdout instead of returning from the function.

**Example code (WRONG)**:

```python
@mcp.tool()
def fetch_data(query: str) -> dict:
    results = {"data": [1, 2, 3]}
    print(results)  # WRONG: prints to stdout
    # Function returns None, not results!
```

**What happens**:
1. `print()` sends JSON to stdout
2. MCP protocol uses stdout for JSON-RPC messages
3. Your print() corrupts the protocol
4. Inspector receives garbled message

**Fix**: Return the data, don't print it.

```python
@mcp.tool()
def fetch_data(query: str) -> dict:
    results = {"data": [1, 2, 3]}
    return results  # RIGHT: returns data
```

**If you need logging**: Use `sys.stderr`:

```python
import sys

@mcp.tool()
def fetch_data(query: str) -> dict:
    sys.stderr.write(f"Fetching data for: {query}\n")  # Log to stderr
    results = {"data": [1, 2, 3]}
    return results  # Return to MCP protocol
```

### Error Pattern 5: Exception in Tool (Runtime Error)

**Symptoms**:
- Tool starts executing
- Error happens inside your code (divide by zero, index out of bounds, etc.)
- Inspector shows the exception trace

**Example error**:
```
Error: InternalError - Traceback (most recent call last):
  File "src/server.py", line 45, in divide
    return a / b
ZeroDivisionError: division by zero
```

**Fix strategies**:

**Option 1: Validate input before operation**

```python
@mcp.tool()
def divide(a: float, b: float) -> float:
    """Divide two numbers."""
    if b == 0:
        raise ValueError("Cannot divide by zero")
    return a / b
```

**Option 2: Use type system to prevent bad input**

```python
from typing import Annotated

@mcp.tool()
def divide(
    a: float,
    b: Annotated[float, "Divisor (must be non-zero)"]
) -> float:
    """Divide two numbers.

    Args:
        a: Dividend
        b: Divisor (must not be zero)
    """
    return a / b
```

**Option 3: Handle exception gracefully**

```python
@mcp.tool()
def divide(a: float, b: float) -> dict:
    """Divide two numbers, handling division by zero."""
    try:
        result = a / b
        return {"result": result, "error": None}
    except ZeroDivisionError:
        return {
            "result": None,
            "error": "Cannot divide by zero"
        }
```

## Part 3: The Complete Debugging Workflow

When something breaks, follow this systematic approach:

### Step 1: Reproduce the Error

Set up exactly the conditions that cause failure:

```
1. What were you testing?
2. What parameters did you use?
3. What error did you see?
4. Can you make it happen again?
```

**In Inspector**:
- Use the same tool
- Fill in the same parameters
- Click "Call"
- Note the exact error message

### Step 2: Diagnose the Root Cause

Read the error message carefully. Match it to one of the five patterns above.

**Questions to ask**:
- Is the error about startup (server never connects)?
- Is the error about registration (tool doesn't appear)?
- Is the error about validation (parameter type mismatch)?
- Is the error about encoding (garbled output)?
- Is the error about exceptions (runtime error)?

**Example workflow**:

```
Error: InvalidRequest - Parameter 'count' expected type integer, got float: 5.5

Diagnosis:
- Not a startup error (server is running)
- Not a registration error (tool appeared)
- This IS a validation error (type mismatch)
- Parameter 'count' expects integer but received 5.5
```

### Step 3: Apply the Fix

Based on diagnosis, apply the appropriate fix from the patterns above.

```
Original code:
def list_items(category: str, count: int = 10) -> list:
    ...

Fix applied:
def list_items(category: str, count: float = 10.0) -> list:
    ...

OR

Accept both types:
from typing import Union
def list_items(category: str, count: Union[int, float] = 10) -> list:
    ...
```

### Step 4: Validate the Fix

Make sure the fix actually works:

1. **Save file**
2. **In Inspector**: Click "Reload Server" button
3. **Test again**: Use the same parameters that caused error
4. **Verify**: Tool should work without error

**Don't stop at "error gone"**. Verify that:
- Tool returns expected data
- Return values have correct types
- No error messages appear in server logs

### Step 5: Document & Test Edge Cases

Once basic case works, test variations:

```
If testing list_items(category="all", count=10):
- Also test: count=1 (minimum)
- Also test: count=100 (large value)
- Also test: count=-1 (invalid value)
- Also test: count=0 (edge case)
```

Each variation teaches you where your error handling is weak.

## Common Errors Reference Table

| Error Pattern | How to Recognize | What's Wrong | How to Fix |
|---------------|------------------|--------------|-----------|
| **Startup Failure** | Server never connects to Inspector | Import error, syntax error, missing env var | Run `python src/server.py` directly to see error |
| **Tool Not Registered** | Tool doesn't appear in Inspector Tools list | Missing `@mcp.tool()` decorator or wrong syntax | Add `@mcp.tool()` decorator, reload Inspector |
| **Parameter Validation** | Inspector rejects parameters with type/constraint error | Type mismatch or parameter not in allowed list | Check parameter type hint, use correct types/values |
| **Garbled Output** | Response is null/incomplete/corrupted | `print()` to stdout corrupts JSON-RPC protocol | Use `return`, not `print()`, for tool output |
| **Exception in Tool** | RuntimeError/IndexError/ZeroDivisionError in Inspector | Unhandled exception in tool code | Add input validation or exception handling |

## Try With AI

Your MCP server has a tool that sometimes fails, and you need to diagnose why. Work with AI to identify the error pattern and apply the fix.

### Part 1: Describe the Problem

Ask AI to help diagnose a server error:

```
I have an MCP server with a tool that's failing. Here's what happened:

1. I launched MCP Inspector
2. Connected to my server successfully
3. Found the "search" tool in the Tools list
4. Filled in parameters: query="python", limit=10
5. Clicked "Call"
6. Got this error:

Error: InvalidRequest - Parameter 'limit' expected type integer, got float: 10.0

The tool signature is:
@mcp.tool()
def search(query: str, limit: int = 5) -> list:
    """Search documentation by keyword."""
    return results[:limit]

Why is it treating 10 as float when I clearly set it as integer?
What's the fix?
```

**What you're learning**: You're describing the exact context AI needs to diagnose—reproduction steps, error message, and code. This precision accelerates diagnosis.

### Part 2: Evaluate AI's Suggestion

AI might suggest:
```
The issue is that you're explicitly passing 10.0 (float) in the Inspector UI.
Even though your type hint says `int`, the Inspector accepts 10.0 and passes it as-is.

The fix is to:
1. Type 10 (not 10.0) in the Inspector
2. Or update the type hint to accept both: `limit: Union[int, float]`
3. Or add input validation to coerce to int: `limit: int = int(float(limit))`
```

**Your evaluation**:
- Does this explanation make sense given the error?
- Which fix seems most appropriate?
- What would happen with each fix?

**Ask yourself**:
- "Is the issue in my code or my test input?"
- "What's the principle here—type safety or flexibility?"
- "For an MCP tool, which approach is more robust?"

### Part 3: Verify and Refine

Tell AI what you tried and ask for refinement:

```
I tried option 1 (typing 10 instead of 10.0) and it worked.
But I want to make my tool more flexible—users might send either integer or float.

Should I update the type hint to Union[int, float]?
And if I do that, does my return value (results[:limit]) still work correctly?

Here's my updated code:

@mcp.tool()
def search(
    query: str,
    limit: Union[int, float] = 5
) -> list:
    """Search documentation by keyword."""
    return results[:limit]

Does this handle both integer and float limits correctly?
```

**What you're learning**: You're learning the difference between fixing the immediate problem vs making code more robust. AI can help you think through tradeoffs.

### Part 4: Build Confidence Through Testing

Based on AI's feedback, test multiple scenarios:

```
My tool now accepts Union[int, float]. I want to verify it handles all these cases:
1. limit=5 (integer, typical case)
2. limit=5.0 (float, unusual but possible)
3. limit=5.9 (float with decimal, will it work?)
4. limit=0 (edge case, what happens?)
5. limit=-1 (invalid, how should I handle?)

Can you help me write test cases in the Inspector for each scenario?
What should the expected behavior be for each?
```

**What you're learning**: You're learning to think systematically about edge cases, not just happy paths. This is what separates debugging from testing.

### Part 5: Document Your Learning

Reflect on the debugging process:

```
Looking back at this debugging session:
1. I initially thought the problem was in my code
2. But it was actually in how I was testing (10.0 vs 10)
3. Then I learned that accepting Union[int, float] is more flexible
4. Now I'm testing edge cases to make sure nothing breaks

This taught me something about the difference between:
- Type validation (what types are allowed)
- Input validation (what values are allowed)
- Error recovery (how to handle invalid inputs gracefully)

Is this the right mental model for debugging MCP tools?
```

**What you're learning**: You're meta-learning—thinking about your own thinking. This separates junior developers from professionals.

---

**Key Takeaway**: Debugging isn't about guessing fixes. It's about systematic reproduction, careful diagnosis, targeted fixes, and thorough testing. MCP Inspector makes this cycle fast because you get instant feedback on every change.
