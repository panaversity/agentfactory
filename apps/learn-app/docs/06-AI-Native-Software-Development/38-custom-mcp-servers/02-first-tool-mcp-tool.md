---
sidebar_position: 2
title: "Your First Tool (@mcp.tool)"
description: "Understand the @mcp.tool decorator by building a simple tool that teaches Python type hints as JSON schema generation"
proficiency_level: B1
cognitive_load:
  new_concepts: 6
  estimated_difficulty: B1
estimated_time: 50 minutes
learning_objectives:
  - "Understand how @mcp.tool decorator converts functions into MCP tools"
  - "Implement type hints that automatically generate JSON schemas"
  - "Write comprehensive docstrings that populate tool descriptions"
  - "Return structured data from tools"
  - "Recognize basic error handling patterns"
  - "Test tools locally before integration"
skills:
  mcp_tool_decorator:
    proficiency: B1
    bloom_level: Understand
  python_type_hints:
    proficiency: B1
    bloom_level: Apply
  json_schema_generation:
    proficiency: B1
    bloom_level: Understand
  function_documentation:
    proficiency: B1
    bloom_level: Apply
generated_by: content-implementer v2.0.0
source_spec: specs/038-chapter-38-custom-mcp-servers/spec.md
created: 2025-12-26
last_modified: 2025-12-26
git_author: Claude Code
workflow: /sp.implement
version: 1.0.0
---

# Your First Tool (@mcp.tool)

In Chapter 37, you learned that MCP tools are functions exposed to agents. In this lesson, you'll build your first tool using FastMCP's `@mcp.tool` decorator. By the end, you'll understand how a simple Python function becomes a structured capability that Claude Code can invoke with type-safe parameters and schema validation.

The key insight: **Type hints are specifications**. When you write `name: str`, you're not just helping Python's type checker—you're writing a contract that FastMCP converts to JSON Schema, which agents use to understand what parameters your tool accepts.

## Understanding the @mcp.tool Decorator

The `@mcp.tool` decorator is how you register functions as MCP tools. It transforms a regular Python function into something agents can discover, understand, and invoke.

### How It Works

When you write:

```python
from mcp.server.fastmcp import FastMCP

mcp = FastMCP("my_server")

@mcp.tool()
def greet(name: str) -> str:
    """Greet a user by name."""
    return f"Hello, {name}!"
```

FastMCP does three things:

1. **Extracts metadata** from your function:
   - Name: `greet`
   - Description: From your docstring ("Greet a user by name.")
   - Parameters: From type hints (`name: str`)
   - Return type: From the return annotation (`-> str`)

2. **Generates JSON Schema** from type hints:
   - `name: str` becomes `{"type": "string"}`
   - Validates inputs BEFORE your function runs
   - Prevents type errors at the source

3. **Registers the tool** with the MCP server:
   - Agents can discover it via the server's tool list
   - Agents can invoke it by name with validated parameters
   - The server automatically serializes responses

**Why this matters**: You don't write JSON Schema by hand. You write clean Python code with type hints, and FastMCP generates the schema automatically. This is the principle of specification-driven development applied to tool implementation.

### Layer 1 Foundation: Manual Implementation

You're building tools manually in this lesson (not using AI) because understanding how type hints map to schemas teaches you to write better tool signatures. When you later use Claude Code to generate tools, you'll recognize which signatures are underspecified or ambiguous—and you'll know how to fix them.

## Concept 1: Type Hints as Specifications

Type hints are more than Python syntax. They're **specifications** that define what each parameter accepts.

### Basic Types

The simplest type hints map directly to JSON Schema:

```python
from mcp.server.fastmcp import FastMCP

mcp = FastMCP("types_demo")

# String parameter
@mcp.tool()
def uppercase(text: str) -> str:
    """Convert text to uppercase."""
    return text.upper()

# Integer parameter
@mcp.tool()
def multiply(a: int, b: int) -> int:
    """Multiply two integers."""
    return a * b

# Float parameter
@mcp.tool()
def divide(a: float, b: float) -> float:
    """Divide two floats."""
    if b == 0:
        raise ValueError("Cannot divide by zero")
    return a / b

# Boolean parameter
@mcp.tool()
def toggle(enabled: bool) -> str:
    """Toggle a boolean flag."""
    return f"Feature {'enabled' if enabled else 'disabled'}"
```

**Generated JSON Schema for `uppercase` tool:**

```json
{
  "name": "uppercase",
  "description": "Convert text to uppercase.",
  "inputSchema": {
    "type": "object",
    "properties": {
      "text": {
        "type": "string",
        "description": "The text to convert (from parameter name)"
      }
    },
    "required": ["text"]
  }
}
```

Notice: FastMCP inferred the schema from the type hint. You wrote `text: str`. It generated `"type": "string"`. This is automatic.

### The Generator Pattern

When multiple tools use the same pattern, Python's `typing` module lets you create reusable parameter specifications:

```python
from typing import Optional, List, Dict

@mcp.tool()
def search_users(query: str, limit: Optional[int] = 20) -> List[Dict[str, str]]:
    """Search for users by query.

    Args:
        query: Search string (e.g., "john", "team:marketing")
        limit: Maximum results to return (default 20)

    Returns:
        List of matching user dictionaries with id, name, email
    """
    # Implementation would query a database
    # For now, return example data
    return [
        {"id": "U123", "name": "John Doe", "email": "john@example.com"},
        {"id": "U456", "name": "Jane Smith", "email": "jane@example.com"}
    ]
```

**Generated schema for `search_users`:**

```json
{
  "name": "search_users",
  "description": "Search for users by query.",
  "inputSchema": {
    "type": "object",
    "properties": {
      "query": {
        "type": "string",
        "description": "Search string (e.g., 'john', 'team:marketing')"
      },
      "limit": {
        "type": "integer",
        "description": "Maximum results to return (default 20)",
        "default": 20
      }
    },
    "required": ["query"]
  }
}
```

Notice: `Optional[int] = 20` became `"required": ["query"]` (not `limit`). FastMCP inferred that `limit` is optional because it has a default value. This is why type hints matter—they encode contract information that FastMCP uses.

## Concept 2: Docstrings as Tool Descriptions

Docstrings serve two purposes:

1. **Human documentation**: Developers read docstrings to understand what your tool does
2. **Schema metadata**: FastMCP extracts docstrings to populate the MCP tool description

### Standard Docstring Pattern

Use the "summary + detailed explanation" pattern:

```python
@mcp.tool()
def create_reminder(
    message: str,
    days_from_now: int
) -> str:
    """Create a reminder for a future date.

    This tool schedules a reminder that will trigger at the specified date.
    The message will be delivered via your notification system. Reminders
    can be edited or deleted after creation.

    Args:
        message: The reminder text (max 200 characters)
        days_from_now: How many days until reminder triggers (1-365)

    Returns:
        Confirmation string with reminder ID and scheduled date
    """
    # Implementation
    return f"Reminder scheduled for {days_from_now} days from now"
```

**Why this matters**: The first line ("Create a reminder for a future date") becomes the tool description that agents see. The detailed explanation helps agents understand edge cases and constraints.

**Bad docstring pattern** (too vague):

```python
@mcp.tool()
def create_reminder(message: str, days_from_now: int) -> str:
    """Do something with reminders."""
    # Agents can't understand intent from "Do something"
    pass
```

Good docstrings make your tools discoverable and understandable to agents.

## Concept 3: JSON Schema Generation from Types

This is the mechanical process where type hints become schema constraints. Understanding this teaches you to write specifications that constrain invalid inputs at the boundary.

### From Simple Types to Constraints

Pydantic (the validation library under FastMCP) uses type hints to constrain inputs:

```python
from pydantic import BaseModel, Field
from typing import Optional, List

# Define a data model with constraints
class UserSearchInput(BaseModel):
    """Input validation model for user search."""
    query: str = Field(..., description="Search string", min_length=2, max_length=200)
    limit: Optional[int] = Field(default=20, ge=1, le=100, description="Results limit")

@mcp.tool()
def search_users(params: UserSearchInput) -> str:
    """Search users by query with pagination."""
    # Pydantic validates params BEFORE this function runs
    # If query is empty, Pydantic rejects it (min_length=2)
    # If limit > 100, Pydantic rejects it (le=100)
    return f"Searching for: {params.query} (limit: {params.limit})"
```

**Generated schema:**

```json
{
  "name": "search_users",
  "description": "Search users by query with pagination.",
  "inputSchema": {
    "type": "object",
    "properties": {
      "query": {
        "type": "string",
        "description": "Search string",
        "minLength": 2,
        "maxLength": 200
      },
      "limit": {
        "type": "integer",
        "description": "Results limit",
        "minimum": 1,
        "maximum": 100,
        "default": 20
      }
    },
    "required": ["query"]
  }
}
```

**The Pattern**:
- `Field(...)` means required
- `Field(default=X)` means optional with default
- `min_length=2` becomes `"minLength": 2` in schema
- `ge=1` becomes `"minimum": 1` in schema
- These constraints validate inputs BEFORE your function executes

This is specification-driven development at the function level. Your constraints prevent invalid usage before code runs.

## Concept 4: Return Values and Schema

What your tool returns affects how agents use it. Return types should be structured and predictable.

### Simple Return Types

```python
@mcp.tool()
def get_status() -> str:
    """Get server status."""
    return "Server is running"
```

Return type `-> str` tells agents: "This returns text."

### Structured Return Types

For complex data, return dictionaries or Pydantic models:

```python
from typing import Dict, List, Any

@mcp.tool()
def get_user(user_id: str) -> Dict[str, Any]:
    """Get user details by ID.

    Returns:
        Dictionary with keys: id, name, email, created_at, active
    """
    return {
        "id": user_id,
        "name": "John Doe",
        "email": "john@example.com",
        "created_at": "2024-01-15",
        "active": True
    }
```

Return type `-> Dict[str, Any]` tells agents: "This returns a dictionary with string keys and any values."

**Output format matters**: Agents reason better about structured data. Always return dictionaries or lists, not raw text when possible.

## Concept 5: Basic Error Handling in Tools

Tools should handle errors gracefully. Errors in tool execution need clear messages so agents can understand what went wrong.

### The Pattern

```python
@mcp.tool()
def divide(a: float, b: float) -> str:
    """Divide two numbers.

    Args:
        a: The dividend
        b: The divisor (cannot be zero)

    Returns:
        String with result or error message
    """
    # Check constraint BEFORE attempting operation
    if b == 0:
        # Return error message (tools often return strings)
        return "Error: Cannot divide by zero"

    try:
        result = a / b
        return f"Result: {result}"
    except Exception as e:
        # Catch unexpected errors
        return f"Error: {type(e).__name__}: {str(e)}"
```

**Why this pattern**:
- Validation errors (b == 0) are checked first with clear messages
- Try-except catches unexpected errors
- Always return something (not raise exceptions) so agents can parse the response

When tools raise exceptions, agents can't handle them. When tools return error messages, agents can read and act on them.

### Specific Error Messages

Generic errors are unhelpful. Be specific:

```python
@mcp.tool()
def fetch_user(user_id: str) -> str:
    """Fetch user by ID."""
    # Bad: "Error occurred"
    # Good: Include the constraint violation
    if not user_id.startswith("U"):
        return "Error: User ID must start with 'U' (e.g., 'U123456')"

    if len(user_id) < 3:
        return "Error: User ID must be at least 3 characters (e.g., 'U123')"

    # If we get here, basic validation passed
    # Now attempt the actual operation
    try:
        user = lookup_user(user_id)
        return f"User: {user['name']} ({user['email']})"
    except KeyError:
        return f"Error: User {user_id} not found"
```

**The flow**:
1. Validate input format and constraints
2. Attempt operation
3. Catch specific errors with actionable messages

## Concept 6: Testing Tools Locally

Before integrating tools into a server that agents use, test them locally to ensure they work correctly.

### A Simple Test Script

Create a file called `test_tools.py` in the same directory as your server:

```python
"""Test tools before running the MCP server."""

# Import the server and tools
from my_server import mcp, greet, multiply

# Test 1: Basic function call
result = greet("Alice")
assert result == "Hello, Alice!", f"Expected 'Hello, Alice!' but got '{result}'"
print("✓ Test 1 passed: greet function works")

# Test 2: Type validation
result = multiply(3, 4)
assert result == 12, f"Expected 12 but got {result}"
print("✓ Test 2 passed: multiply function works")

# Test 3: Error case
result = divide(10, 0)
assert "Error" in result, f"Expected error message but got '{result}'"
print("✓ Test 3 passed: error handling works")

# Test 4: Edge case
result = multiply(-5, 2)
assert result == -10, f"Expected -10 but got {result}"
print("✓ Test 4 passed: negative numbers work")

print("\nAll tests passed!")
```

**Run tests:**

```bash
python test_tools.py
```

**Output:**
```
✓ Test 1 passed: greet function works
✓ Test 2 passed: multiply function works
✓ Test 3 passed: error handling works
✓ Test 4 passed: negative numbers work

All tests passed!
```

### What to Test

For each tool, test:

1. **Happy path**: Normal input → expected output
2. **Boundary conditions**: Edge cases (empty strings, zero, max values)
3. **Error cases**: Invalid input → graceful error message
4. **Type validation**: Wrong type → Pydantic rejects it

## Putting It Together: A Complete Simple Tool

Let's build a tool that uses all these concepts:

```python
#!/usr/bin/env python3
"""Example MCP server with a single tool."""

from mcp.server.fastmcp import FastMCP
from typing import Optional

# Initialize server
mcp = FastMCP("greeting_server")

# Define tool
@mcp.tool()
def greet(
    name: str,
    title: Optional[str] = None,
    enthusiastic: bool = False
) -> str:
    """Greet someone by name with optional title.

    This tool generates personalized greeting messages. You can specify
    an optional title (like "Dr." or "Professor") and whether the greeting
    should be enthusiastic with an exclamation mark.

    Args:
        name: The person's name (required)
        title: Optional title (e.g., "Dr.", "Ms.", "Professor")
        enthusiastic: If True, add exclamation mark to greeting

    Returns:
        A greeting string personalized with the provided information
    """
    # Validate name is not empty
    if not name or not name.strip():
        return "Error: Name cannot be empty"

    # Build greeting
    greeting = "Hello"

    if title:
        greeting += f", {title}"

    greeting += f" {name.strip()}"

    if enthusiastic:
        greeting += "!"
    else:
        greeting += "."

    return greeting

# Test the tool locally
if __name__ == "__main__":
    # Test basic greeting
    print(greet("Alice"))  # Hello Alice.
    # Output: Hello Alice.

    # Test with title
    print(greet("Smith", title="Dr."))  # Hello, Dr. Smith.
    # Output: Hello, Dr. Smith.

    # Test enthusiastic
    print(greet("Bob", enthusiastic=True))  # Hello Bob!
    # Output: Hello Bob!

    # Test all parameters
    print(greet("Jane", title="Professor", enthusiastic=True))
    # Output: Hello, Professor Jane!

    # Test error case
    print(greet(""))  # Error message
    # Output: Error: Name cannot be empty
```

**Key points from this example:**

1. **Type hints**: `name: str`, `title: Optional[str]`, `enthusiastic: bool`
2. **Docstring**: Full description of what the tool does
3. **Parameter descriptions**: Args section explains each input
4. **Return type annotation**: `-> str` specifies output type
5. **Validation**: Empty names are caught with error message
6. **Testing**: Local calls verify behavior before server runs

## Understanding FastMCP Inspector

When you run an MCP server, you can use the Inspector to view the generated schema:

```bash
# Start your server
python my_server.py

# In another terminal, use the MCP Inspector
# (Tools like Claude Code's MCP extension inspect the server)
```

The Inspector shows:

- **Tool name**: `greet`
- **Description**: "Greet someone by name with optional title."
- **Input Schema**:
  ```json
  {
    "name": {"type": "string"},
    "title": {"type": "string", "default": null},
    "enthusiastic": {"type": "boolean", "default": false}
  }
  ```
- **Required fields**: Only `name` (others have defaults)

This is what agents see when discovering your tools. Your job is to ensure this schema accurately represents what your tool does.

## Practice

Now it's your turn. Write a simple tool for each scenario, implementing the six core concepts you've learned:

1. Type hints that specify parameter contracts
2. Docstrings that guide agents
3. JSON schema-compatible return types
4. Validation before processing
5. Clear error messages
6. Local testing before integration

Test each one locally before moving on. These exercises teach you what Claude Code sees when it generates tools—the pattern of thoughtful type annotations, clear descriptions, and defensive coding.

### Exercise 1: Temperature Converter

Write a tool that converts Celsius to Fahrenheit. It should:

- Accept a `celsius` parameter (float)
- Return a formatted string: "85.0°F" or similar
- Handle invalid inputs gracefully

Expected output for `celsius_to_fahrenheit(25)`:
```
77.0°F
```

<details>
<summary>Solution</summary>

```python
@mcp.tool()
def celsius_to_fahrenheit(celsius: float) -> str:
    """Convert Celsius temperature to Fahrenheit.

    Args:
        celsius: Temperature in degrees Celsius

    Returns:
        Temperature in Fahrenheit as formatted string (e.g., "77.0°F")
    """
    fahrenheit = (celsius * 9/5) + 32
    return f"{fahrenheit:.1f}°F"
```

**Test:**
```python
assert celsius_to_fahrenheit(0) == "32.0°F"
assert celsius_to_fahrenheit(25) == "77.0°F"
assert celsius_to_fahrenheit(100) == "212.0°F"
```

</details>

### Exercise 2: Word Counter with Options

Write a tool that counts words in a string. It should:

- Accept `text` parameter (string)
- Accept optional `exclude_numbers` parameter (boolean, default False)
- Return a dictionary with word count and longest word

Expected output for `count_words("Hello world from FastMCP", exclude_numbers=True)`:
```json
{"word_count": 4, "longest_word": "FastMCP"}
```

<details>
<summary>Solution</summary>

```python
from typing import Dict, Any

@mcp.tool()
def count_words(
    text: str,
    exclude_numbers: bool = False
) -> Dict[str, Any]:
    """Count words in text and find longest word.

    Args:
        text: The text to analyze
        exclude_numbers: If True, ignore numeric strings

    Returns:
        Dictionary with word_count and longest_word
    """
    words = text.split()

    if exclude_numbers:
        words = [w for w in words if not w.isdigit()]

    if not words:
        return {"word_count": 0, "longest_word": None}

    longest = max(words, key=len)
    return {
        "word_count": len(words),
        "longest_word": longest
    }
```

**Test:**
```python
result = count_words("Hello world")
assert result["word_count"] == 2
assert result["longest_word"] == "world"

result = count_words("Item1 Item2 Item3", exclude_numbers=True)
assert result["word_count"] == 0
```

</details>

### Exercise 3: Email Validator

Write a tool that validates email addresses. It should:

- Accept an `email` parameter
- Return a dictionary with `is_valid` (bool) and `reason` (string)
- Check for @ symbol, domain, and format basics

Expected output for `validate_email("john@example.com")`:
```json
{"is_valid": true, "reason": "Valid email format"}
```

Expected output for `validate_email("invalid.email")`:
```json
{"is_valid": false, "reason": "Missing @ symbol"}
```

<details>
<summary>Solution</summary>

```python
from typing import Dict

@mcp.tool()
def validate_email(email: str) -> Dict[str, any]:
    """Validate email address format.

    Args:
        email: The email address to validate

    Returns:
        Dictionary with is_valid (bool) and reason (string)
    """
    email = email.strip()

    if not email:
        return {"is_valid": False, "reason": "Email cannot be empty"}

    if "@" not in email:
        return {"is_valid": False, "reason": "Missing @ symbol"}

    parts = email.split("@")
    if len(parts) != 2:
        return {"is_valid": False, "reason": "Multiple @ symbols"}

    local, domain = parts

    if not local:
        return {"is_valid": False, "reason": "Missing local part (before @)"}

    if not domain or "." not in domain:
        return {"is_valid": False, "reason": "Invalid domain"}

    return {"is_valid": True, "reason": "Valid email format"}
```

**Test:**
```python
assert validate_email("john@example.com")["is_valid"] == True
assert validate_email("invalid.email")["is_valid"] == False
assert validate_email("@@@@")["is_valid"] == False
```

</details>

