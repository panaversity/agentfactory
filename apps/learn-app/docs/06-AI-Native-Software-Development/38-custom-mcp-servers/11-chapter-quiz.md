---
sidebar_position: 11
title: "Chapter 38 Assessment: Custom MCP Servers"
description: "Validate your understanding of building, designing, and deploying custom MCP servers through comprehensive assessment"
keywords: ["assessment", "quiz", "MCP servers", "validation", "comprehension", "design thinking"]
chapter: 38
lesson: 11
duration_minutes: 60

# HIDDEN SKILLS METADATA
skills:
  - name: "MCP Server Architecture Understanding"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can explain how MCP servers handle tools, resources, and prompts, understand lifecycle hooks, and describe request/response flow"

  - name: "MCP Implementation Quality Evaluation"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Analyze"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can identify bugs in MCP implementations, recognize security vulnerabilities, and spot design issues in code samples"

  - name: "MCP Server Design Decision-Making"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Evaluate"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can justify architectural choices when designing MCP servers, explain tool vs resource vs prompt decisions, and defend design patterns"

learning_objectives:
  - objective: "Demonstrate understanding of @mcp.tool decorator, type hint → JSON schema conversion, and tool invocation flow"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Multiple choice questions on core decorator and schema concepts"

  - objective: "Identify and fix bugs in MCP server code including type hints, logging, security issues, and resource configuration"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Code analysis scenarios with specific bugs to identify and correct"

  - objective: "Design an MCP server for a given domain by identifying appropriate tools, resources, and prompts with clear justification"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Open-ended design challenge with specification and design rationale"

cognitive_load:
  new_concepts: 0
  assessment: "This is assessment content—no new concepts. Validates mastery of all chapter concepts (tools, resources, prompts, security, testing, deployment)"

differentiation:
  extension_for_advanced: "For students exceeding expectations: Design a multi-resource server with OAuth authentication and async tool execution; justify architectural complexity tradeoffs"
  remedial_for_struggling: "For students struggling: Review specific lesson(s) where gaps appeared; use Part 1 multiple choice as diagnostic tool; pair with a peer for the design challenge"
---

# Chapter 38 Assessment: Custom MCP Servers

You've completed Chapter 38. You understand how to scaffold MCP servers, implement tools with proper type hints, expose resources through URI schemes, create prompt templates, handle authentication, test implementations, and package servers for distribution.

This assessment validates that understanding. You'll demonstrate mastery through three parts: **concept recognition**, **code analysis and debugging**, and **design thinking**.

**How to approach this assessment:**
- Take your time with Part 1—think through each option carefully
- In Part 2, identify the specific issue, explain why it's a problem, then write the fix
- For Part 3, treat this as a real project specification—justify your decisions

---

## Part 1: Concept Recognition (10 Questions)

Each question tests core concepts from the chapter. Select the best answer.

### Question 1: @mcp.tool() Decorator Purpose

What is the primary purpose of the `@mcp.tool()` decorator?

A) Registers a Python function as an MCP tool that AI clients can discover and invoke
B) Adds logging to a function for debugging
C) Creates an HTTP endpoint for external access
D) Packages a function for distribution to PyPI

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: A**

The `@mcp.tool()` decorator registers a Python function as a callable tool in the MCP server. When AI clients connect, they discover these tools through the MCP protocol and can invoke them with arguments. This is the foundational mechanism that makes tools available to agents.

**Why the others are wrong:**
- B) Logging is configured separately (or can be added in function body, but isn't the decorator's purpose)
- C) While MCP servers run over HTTP, the decorator itself doesn't create endpoints—the server infrastructure handles that
- D) Packaging for distribution is handled by pyproject.toml and build tools, not the decorator

</details>

---

### Question 2: Type Hints and JSON Schema Conversion

When you define a tool parameter with a type hint like `user_id: int`, what happens?

A) FastMCP automatically converts the type hint into a JSON Schema property with type "integer"
B) The type hint only affects Python type checking—it doesn't influence the MCP protocol
C) The parameter is treated as a string and must be validated manually
D) Type hints are ignored by MCP clients and parameters are always strings

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: A**

Type hints in Python are introspected by FastMCP to generate the JSON Schema that describes tool parameters. When you write `user_id: int`, FastMCP converts this to a JSON Schema property:
```json
{"type": "integer", "description": "..."}
```

This schema is sent to MCP clients, which validate inputs and inform users about required types.

**Why the others are wrong:**
- B) Type hints are used for schema generation—they directly influence the protocol
- C) Parameters are not automatically strings; they're validated against the schema type
- D) Type hints are critical to the MCP protocol conversation—not ignored

</details>

---

### Question 3: Pydantic Field Parameters

When you use `Field(description="...", min=1)` for a parameter, what aspect of the tool contract does this update?

A) Only the JSON Schema sent to clients—not the actual Python function behavior
B) Both the JSON Schema AND the Python function's type checking
C) Only the Python function signature—clients don't see Field constraints
D) Nothing—Field is cosmetic and doesn't affect schema or validation

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: A**

`Field()` updates the **JSON Schema** that Pydantic generates, which FastMCP sends to MCP clients. Clients see the description and constraints (like `min=1`). However, Field itself doesn't enforce validation on the Python side—Pydantic validation does that.

Specifically:
- The description appears in client UI
- Constraints like `min=1` inform clients about acceptable values
- But Python validation is handled by Pydantic models, not by Field alone

**Why the others are wrong:**
- B) Field affects schema, but Python type checking is separate (happens through Pydantic validation)
- C) Clients absolutely see Field constraints in the JSON Schema
- D) Field has real impact on the schema contract with clients

</details>

---

### Question 4: Static vs Templated Resources

When implementing resources, what's the key difference between a static resource and a templated resource?

A) Static resources provide a single fixed piece of data; templated resources can parameterize URIs to provide multiple related resources
B) Static resources are faster; templated resources are slower because they require URI parsing
C) Static resources use file:// URIs; templated resources use http:// URIs
D) There is no difference—"static" and "templated" are just naming conventions with no functional impact

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: A**

**Static Resources**: Single URI, fixed data
```python
@mcp.resource("user://profile")
def get_profile():
    return "..."
```

**Templated Resources**: URI pattern with parameters
```python
@mcp.resource("user://{user_id}/profile")
def get_user_profile(user_id: str):
    return f"Profile for {user_id}"
```

Templated resources allow one handler to serve multiple URIs by parameterizing the URI pattern.

**Why the others are wrong:**
- B) Performance difference is negligible; this isn't the key distinction
- C) URI schemes don't determine static vs templated; either can use any scheme
- D) This is a fundamental architectural difference, not just naming

</details>

---

### Question 5: @mcp.prompt Message Structure

When creating an MCP prompt with `@mcp.prompt()`, what is the purpose of the `messages` parameter?

A) Defines the conversation history that gets sent to the AI model as context
B) Only informational—it has no effect on how the prompt actually functions
C) Controls which AI models can access the prompt
D) Restricts who can call the prompt to specific user groups

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: A**

The `messages` parameter in a prompt definition specifies the conversation messages (system message + user message template) that will be sent to the AI model. This becomes the **context and instructions** the AI receives.

Example:
```python
@mcp.prompt()
def code_review(code: str):
    return {
        "messages": [
            {
                "role": "system",
                "content": "You are an expert code reviewer."
            },
            {
                "role": "user",
                "content": f"Review this code:\n{code}"
            }
        ]
    }
```

The AI model receives both the system context and the user prompt with inserted parameters.

**Why the others are wrong:**
- B) The messages directly control what the AI sees—very functional
- C) Model selection is determined by the client application, not the prompt definition
- D) Access control would be at the server level, not at individual prompts

</details>

---

### Question 6: Environment Variable Handling

When an MCP server needs to access a secret API key (e.g., for a backend API), what's the recommended approach?

A) Read from `os.environ` or a `.env` file; never hardcode secrets in source code
B) Hardcode the secret in the code since the code is private/internal
C) Store the secret in a visible comment so team members can reference it
D) Ask the AI to generate a temporary key when the tool is invoked

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: A**

**Best practice for secret handling:**
1. Read from environment variables: `api_key = os.environ.get("API_KEY")`
2. Or use `.env` files with `python-dotenv`: `load_dotenv()` then `os.environ["API_KEY"]`
3. Never hardcode secrets in source code (even in private repos—they can leak)

The server can validate that required secrets exist on startup:
```python
api_key = os.environ.get("API_KEY")
if not api_key:
    raise ValueError("API_KEY environment variable not set")
```

**Why the others are wrong:**
- B) "Private code" is not secure enough—repos get exposed, developers share code, CI logs leak
- C) Visible comments make secrets easily discoverable in version control and logs
- D) Generating temporary keys on demand defeats security; secrets should be provisioned at deployment time

</details>

---

### Question 7: Fail-Fast Validation Pattern

What is the "fail-fast validation" pattern and why is it important in MCP tools?

A) Validate all parameters immediately at the start of the tool function; return errors early if validation fails
B) Skip validation entirely and let downstream code handle errors
C) Validate parameters only after they've been used in calculations
D) Validate only for production servers, not during development

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: A**

**Fail-Fast Validation**: Check all inputs immediately at the function start. Return an error to the client as soon as validation fails.

Example:
```python
@mcp.tool()
def process_file(file_path: str, timeout: int) -> str:
    # Validate immediately
    if not os.path.exists(file_path):
        raise ValueError(f"File not found: {file_path}")
    if timeout < 1:
        raise ValueError("Timeout must be at least 1 second")

    # Only proceed if all validations pass
    return process(file_path, timeout)
```

**Why it matters:**
- Clients get error feedback immediately, not after wasted processing
- Prevents partial execution with invalid state
- Clear error messages help clients understand what went wrong
- Consistent error handling across all tools

**Why the others are wrong:**
- B) No validation leads to cryptic errors deep in the code
- C) Validating after use wastes computation and obscures error sources
- D) Validation is critical in all environments, not just production

</details>

---

### Question 8: Logging to stderr, Not stdout

Why is logging to `sys.stderr` important in MCP servers?

A) `stdout` is reserved for MCP protocol messages; logging to `stdout` corrupts the protocol
B) `stderr` is more secure than `stdout`
C) `stderr` is faster for I/O operations
D) There's no real difference; both work equally well

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: A**

In MCP servers, **stdout is the communication channel** between server and client. The MCP protocol messages (JSON-RPC requests/responses) are sent over stdout. If logging goes to stdout, it contaminates the protocol stream and breaks client communication.

Example of what goes wrong:
```python
# WRONG - breaks protocol
print("Debug: processing request")  # Goes to stdout → corrupts MCP protocol
```

Correct approach:
```python
# RIGHT - logging goes to stderr
import sys
print("Debug: processing request", file=sys.stderr)
```

Or use Python's logging module (configured to output to stderr):
```python
import logging
logging.basicConfig(stream=sys.stderr)
logger = logging.getLogger(__name__)
logger.debug("Processing request")
```

**Why the others are wrong:**
- B) Security isn't the primary reason—protocol integrity is
- C) I/O speed is negligible between stdout and stderr
- D) They're fundamentally different in the context of MCP protocol streams

</details>

---

### Question 9: pyproject.toml Entry Points

What purpose does the `[project.entry-points."mcp"]` section in `pyproject.toml` serve?

A) Declares how MCP clients can discover and launch the server (important for packaging and distribution)
B) Only needed for web servers, not MCP servers
C) Purely cosmetic—doesn't affect how the server runs
D) Controls which Python version is required

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: A**

The `[project.entry-points."mcp"]` section in pyproject.toml tells MCP clients (like Claude Desktop) how to **launch** your server:

```toml
[project.entry-points."mcp"]
my-custom-server = "my_package.server:main"
```

This entry point means:
- Server name: `my-custom-server`
- Server location: The `main` function in `my_package.server` module
- Clients use this to spawn and communicate with the server

**Why it matters for distribution:**
- Clients can auto-discover installed servers
- No manual configuration needed
- Standard way to ship MCP servers as installable packages

**Why the others are wrong:**
- B) Entry points apply to all server types, including MCP
- C) Entry points are critical—without them, clients can't find the server
- D) Python version is specified in `requires-python`, not entry-points

</details>

---

### Question 10: Tool vs Resource vs Prompt Decision

You're designing an MCP server to provide access to a company's internal database of projects. Projects have names, descriptions, team members, and budgets. What's the best architectural decision?

A) Create a tool for "GetProject" that accepts a project ID and returns the data
B) Create resources using URI patterns like `project://{project_id}` that expose project data
C) Create a prompt that helps users query the database
D) Use separate tools, resources, and prompts together—each serves a different purpose

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: D** (but B is also defensible depending on use case)

**The decision framework:**

**Tools** (callable functions): Use when you need to *perform actions* or *compute results*
- Examples: "Search projects by keyword", "Calculate project timeline", "Update project budget"
- Tools are procedures that do something

**Resources** (data access): Use when you're exposing *static or semi-static data* to be read
- Examples: "Get the full project document", "List all projects", "Fetch project members"
- Resources are nouns (things you read), not verbs (things you do)

**Prompts** (expert guidance): Use to *encode domain expertise* and *guide AI reasoning*
- Examples: "Project Risk Assessment" (guides AI through risk analysis), "Team Allocation" (helps AI think through team assignments)
- Prompts are templates that activate domain expertise

**For a project database:**
- **Resource**: `project://{project_id}` for reading project data
- **Tool**: "SearchProjects" for finding projects by criteria
- **Tool**: "UpdateProjectBudget" for modifying data
- **Prompt**: "ProjectRiskAssessment" to guide AI's analysis of a project

Answer D is most complete because real-world servers combine all three based on what the server needs to do.

**Why the others are limited:**
- A) Only using tools misses the benefits of resources for data access
- B) Only using resources can't handle actions (like search or updates)
- C) Only using prompts doesn't expose underlying data or capabilities

</details>

---

## Part 2: Code Analysis & Debugging (5 Scenarios)

Each scenario presents broken or problematic code. Identify the issue, explain why it's a problem, and provide the correct fix.

### Scenario 1: Missing Type Hint Causes Schema Issue

```python
from mcp import Server
from mcp.types import Tool

app = Server("my-server")

@app.tool()
def calculate_total(items):
    """Calculate total cost of items."""
    total = sum(items)
    return {"total": total}
```

**What's the problem?** Identify the issue, explain the impact, and provide the fixed code.

<details>
<summary>Answer & Explanation</summary>

**The Problem:**
The `items` parameter has no type hint. Without a type hint, FastMCP cannot generate a JSON Schema for the parameter. MCP clients will see this parameter in the schema with `type: "null"` or no type information, making it impossible for clients to validate inputs or help users understand what data to provide.

**Why it's a problem:**
- Clients don't know what type of data to send
- No validation happens at the MCP protocol level
- Users get cryptic errors if they send the wrong data type
- The tool contract is unclear

**The Fix:**
```python
from mcp import Server
from typing import List

app = Server("my-server")

@app.tool()
def calculate_total(items: List[float]) -> dict:
    """Calculate total cost of items.

    Args:
        items: List of item costs as floating point numbers

    Returns:
        Dictionary with 'total' key containing the sum
    """
    total = sum(items)
    return {"total": total}
```

**Key changes:**
1. Added type hint: `items: List[float]`
2. Added return type hint: `-> dict`
3. Added docstring explaining parameters and return value
4. Now FastMCP can generate schema: `{"type": "array", "items": {"type": "number"}}`

</details>

---

### Scenario 2: print() Instead of sys.stderr

```python
from mcp import Server
import json

app = Server("debug-server")

@app.tool()
def fetch_data(user_id: str) -> dict:
    """Fetch user data."""
    print(f"DEBUG: Fetching user {user_id}")  # ← Problem here
    data = {"user_id": user_id, "name": "Alice"}
    return data
```

**What's the problem?** Identify the issue and provide the fixed code.

<details>
<summary>Answer & Explanation</summary>

**The Problem:**
`print()` writes to stdout, which is the MCP protocol communication channel. This debug output corrupts the JSON-RPC messages being sent to the client, breaking the protocol.

**Why it's a problem:**
- The client receives JSON protocol messages mixed with human-readable debug output
- The JSON parser on the client side fails because the message is malformed
- The entire server stops responding
- Debugging output accidentally breaks production

**The Fix:**
```python
from mcp import Server
import sys

app = Server("debug-server")

@app.tool()
def fetch_data(user_id: str) -> dict:
    """Fetch user data."""
    # Option 1: Use sys.stderr for direct logging
    print(f"DEBUG: Fetching user {user_id}", file=sys.stderr)

    data = {"user_id": user_id, "name": "Alice"}
    return data
```

**Or better yet, use Python's logging module:**
```python
from mcp import Server
import logging

# Configure logging to stderr
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

app = Server("debug-server")

@app.tool()
def fetch_data(user_id: str) -> dict:
    """Fetch user data."""
    logger.debug(f"Fetching user {user_id}")  # Goes to stderr automatically

    data = {"user_id": user_id, "name": "Alice"}
    return data
```

**Key principle:**
- stdout = MCP protocol messages only
- stderr = logging and debugging output

</details>

---

### Scenario 3: API Key Exposed in Response

```python
@app.tool()
def query_api(query: str) -> dict:
    """Query the backend API."""
    import os

    api_key = os.environ["BACKEND_API_KEY"]
    response = {
        "query": query,
        "api_key": api_key,  # ← Security issue
        "result": "..."
    }
    return response
```

**What's the security problem?** Identify the issue and provide the fixed code.

<details>
<summary>Answer & Explanation</summary>

**The Problem:**
The API key is being returned in the tool's response. This exposes the secret to:
- Any client connected to the server
- Logs that capture API responses
- Network traffic if responses are logged or transmitted
- Version control if responses are stored in test data

**Why it's a problem:**
- The secret leaks to unauthorized parties
- Compromise of the server exposes the backend API
- Rotating the key becomes necessary and disruptive
- Regulatory compliance violations (if handling sensitive data)

**The Fix:**
```python
@app.tool()
def query_api(query: str) -> dict:
    """Query the backend API."""
    import os
    import requests

    api_key = os.environ.get("BACKEND_API_KEY")
    if not api_key:
        raise ValueError("BACKEND_API_KEY not configured")

    # Use the API key internally to authenticate with backend
    headers = {"Authorization": f"Bearer {api_key}"}
    backend_url = os.environ.get("BACKEND_URL", "https://api.example.com")

    response = requests.get(
        f"{backend_url}/query",
        params={"q": query},
        headers=headers
    )
    response.raise_for_status()

    # Return ONLY the result, never the API key
    return {
        "query": query,
        "result": response.json()
    }
```

**Key security fixes:**
1. Use API key internally only (don't return it)
2. Validate that API key is configured at server startup
3. Pass secrets through HTTP headers (Authorization), not response bodies
4. Never log secrets
5. Use environment variables and never commit secrets to version control

</details>

---

### Scenario 4: Resource URI Template Mismatch

```python
from mcp import Server

app = Server("resource-server")

@app.resource("document://{doc_id}/content")
def get_document_content(doc_id: str) -> str:
    """Get the full content of a document."""
    return f"Content of document {doc_id}"

@app.resource("document://{doc_id}/metadata")
def get_document_metadata(doc_id: str) -> dict:
    """Get metadata about a document."""
    # ← Issue: trying to read from same URI pattern as content
    # How does the server know whether to call get_document_content
    # or get_document_metadata when both match the same pattern?
```

**What's the problem?** Explain the issue and provide the corrected pattern.

<details>
<summary>Answer & Explanation</summary>

**The Problem:**
Two resources have **overlapping URI patterns**. Both handlers match `document://{doc_id}/...` patterns, creating ambiguity. The server won't know which handler to call for a given URI.

Actually, in this specific case, the patterns are different (`/content` vs `/metadata`), so this might work. But the **real issue** is if you had exact pattern matches:

```python
@app.resource("document://{doc_id}")
def get_full_document(doc_id: str):
    return {"id": doc_id, "content": "..."}

@app.resource("document://{doc_id}")  # ← Duplicate pattern!
def get_document_summary(doc_id: str):
    return {"id": doc_id, "summary": "..."}
```

**Why it's a problem:**
- Ambiguous routing: which handler should be called?
- Second handler silently overrides the first
- One functionality becomes unreachable
- Clients don't know which resource will be returned

**The Fix:**
Use **distinct URI paths** for different resources:

```python
from mcp import Server

app = Server("resource-server")

@app.resource("document://{doc_id}/content")
def get_document_content(doc_id: str) -> str:
    """Get the full content of a document."""
    return f"Content of document {doc_id}"

@app.resource("document://{doc_id}/metadata")
def get_document_metadata(doc_id: str) -> dict:
    """Get metadata about a document."""
    return {
        "id": doc_id,
        "created": "2025-01-01",
        "author": "Alice"
    }

@app.resource("document://{doc_id}/summary")  # ← Unique path
def get_document_summary(doc_id: str) -> str:
    """Get a brief summary of the document."""
    return f"Summary of document {doc_id}"
```

**Design principle:**
- Each resource should have a **unique URI pattern**
- URI path should indicate what data the resource contains
- Use hierarchical paths to organize related resources: `document://{id}/content`, `document://{id}/metadata`, etc.

</details>

---

### Scenario 5: Missing Startup Validation

```python
from mcp import Server
import os

app = Server("config-server")

@app.tool()
def process_with_api(data: str) -> dict:
    """Process data using external API."""
    # API key is expected to exist, but what if it doesn't?
    api_key = os.environ["EXTERNAL_API_KEY"]  # ← Will crash here if not set

    # ... call API with key ...
    return {"processed": True}
```

**What's the problem?** Explain the issue and provide a better pattern.

<details>
<summary>Answer & Explanation</summary>

**The Problem:**
If the `EXTERNAL_API_KEY` environment variable is not set when the server starts, the `KeyError` won't occur until the tool is actually invoked. This means:
1. Server starts successfully (false confidence)
2. Client calls the tool
3. Server crashes mid-execution
4. Client gets a cryptic error message
5. Debugging is harder because the error occurs at runtime, not startup

**Why it's a problem:**
- Silent failures: Server appears functional until tools are called
- Poor operational visibility: DevOps/developers don't know about misconfiguration until users report errors
- Wasted resources: Server runs without proper configuration
- Harder debugging: Error occurs in tool execution context, not at a clear startup point

**The Fix: Fail-Fast at Startup**

```python
from mcp import Server
import os
import logging
import sys

logger = logging.getLogger(__name__)

# Configure validation at module level or in a startup function
def validate_configuration():
    """Validate all required environment variables at startup."""
    required_vars = ["EXTERNAL_API_KEY", "DATABASE_URL", "LOG_LEVEL"]
    missing = []

    for var in required_vars:
        if var not in os.environ:
            missing.append(var)

    if missing:
        error_msg = f"Missing required environment variables: {', '.join(missing)}"
        logger.error(error_msg)
        sys.exit(1)

    logger.info("Configuration validation passed")

app = Server("config-server")

@app.tool()
def process_with_api(data: str) -> dict:
    """Process data using external API."""
    # By this point, we know EXTERNAL_API_KEY exists
    api_key = os.environ["EXTERNAL_API_KEY"]

    # ... call API with key ...
    return {"processed": True}

# Run validation when server initializes
validate_configuration()

# Or in a server startup hook if FastMCP supports it:
# @app.on_startup
# def startup():
#     validate_configuration()
```

**Or using a more Pythonic approach with Pydantic Settings:**

```python
from pydantic_settings import BaseSettings
import os

class ServerConfig(BaseSettings):
    """Server configuration from environment variables."""
    external_api_key: str  # Required, will error if not set
    database_url: str      # Required
    log_level: str = "INFO"  # Optional with default

    class Config:
        env_file = ".env"

# This validates at import time
config = ServerConfig()

from mcp import Server

app = Server("config-server")

@app.tool()
def process_with_api(data: str) -> dict:
    """Process data using external API."""
    # config.external_api_key is guaranteed to exist and be valid
    api_key = config.external_api_key
    return {"processed": True}
```

**Key principle:**
- **Validate configuration at server startup**, not at tool execution time
- Fail loud and early (crash on misconfiguration) rather than late (crash during execution)
- Provide clear error messages about what's missing or wrong
- Enable DevOps/developers to verify configuration before deploying

</details>

---

## Part 3: Design Challenge (Open-Ended)

Design an MCP server for the following domain. Your response should include a specification of what tools, resources, and prompts the server would provide, plus a brief justification of your architectural decisions.

### Challenge: Meeting Scheduler and Analytics MCP Server

**Domain Context:**

Your company uses a mix of calendar systems (Google Calendar, Outlook, internal scheduling). You need an MCP server that helps AI agents manage meetings and analyze meeting patterns.

**Requirements:**

1. AI agents should be able to:
   - Schedule new meetings (checking availability first)
   - Retrieve upcoming meetings for a specific user
   - Cancel or reschedule existing meetings
   - Get analytics on meeting patterns (e.g., total hours in meetings per week, meeting frequency)

2. The server should expose:
   - Calendar data so agents can see meeting details without calling tools
   - Meeting templates (templates for common meeting types with suggested agendas)
   - Analysis results (aggregated meeting statistics)

3. The server should guide AI agents to:
   - Consider participant availability before suggesting meeting times
   - Follow company meeting etiquette guidelines
   - Optimize meeting duration based on agenda complexity

**Your Task:**

Design the MCP server by specifying:

1. **Tools** (callable actions): List 3-5 tools with their purpose
2. **Resources** (data access): List 2-3 resources with their URI patterns
3. **Prompts** (domain expertise): List 1-2 prompts with their purpose
4. **Justification**: In 3-4 sentences, explain your architectural choices. Why did you choose tools vs resources vs prompts for each component?

**Example Format (you may extend this):**

```
## Tools

1. ScheduleMeeting(participants: List[str], duration_minutes: int, preferred_time: str) -> dict
   - Purpose: Create a new meeting after checking all participants' availability

2. CheckAvailability(user_id: str, date: str) -> dict
   - Purpose: Query calendar to find available time slots for a user on a specific date

## Resources

1. calendar://{user_id}/upcoming
   - Purpose: Retrieve all upcoming meetings for a user

## Prompts

1. MeetingEtiquetteGuide
   - Purpose: Guide AI through company meeting standards and best practices

## Justification

I chose ScheduleMeeting and CheckAvailability as tools because they require computation and state changes.
Resources expose meeting data for read-only access without executing complex logic.
The MeetingEtiquetteGuide prompt encodes domain expertise about company culture, enabling the AI
to make thoughtful decisions about meeting scheduling rather than just executing mechanical actions.
```

---

### Your Design Challenge Response:

<details>
<summary>Example Answer & Discussion (Read After You Attempt)</summary>

**Example Well-Designed Server:**

```
## Tools (Callable Actions)

1. ScheduleMeeting(participants: List[str], title: str, duration_minutes: int,
                    proposed_times: List[str]) -> dict
   - Purpose: Attempt to schedule a meeting by finding a time when all participants are available
   - Behavior: Returns confirmation with chosen time, or list of conflicts if no time works

2. RescheduleExistingMeeting(meeting_id: str, new_time: str) -> dict
   - Purpose: Move an existing meeting to a new time slot
   - Behavior: Updates all participants' calendars, sends notifications

3. CancelMeeting(meeting_id: str, reason: str) -> dict
   - Purpose: Cancel a meeting and notify all participants
   - Behavior: Frees up the time slot, sends cancellation notice with reason

4. GetMeetingAnalytics(user_id: str, time_period: str) -> dict
   - Purpose: Compute analytics for a user's meeting patterns
   - Behavior: Returns total hours, meeting count, frequency, peak meeting times

## Resources (Data Access)

1. calendar://{user_id}/meetings
   - Purpose: List all meetings for a user in a specific period (immutable view)
   - Returns: Array of meeting objects with time, participants, agenda

2. calendar://templates/standard
   - Purpose: Expose company meeting templates (1-on-1, All-Hands, Team Standup, etc.)
   - Returns: Template with suggested duration, agenda items, participant roles

3. analytics://{user_id}/patterns
   - Purpose: Static analytics view of meeting patterns over the past quarter
   - Returns: Statistics on meeting load, peak meeting times, most common participants

## Prompts (Domain Expertise)

1. MeetingSchedulingStrategy
   - Purpose: Guide AI through decision framework for scheduling decisions
   - Context: Help AI consider participant time zones, meeting fatigue, agenda complexity
   - Includes: Best practices for agenda length, suggested preparation time, avoiding back-to-back meetings

## Justification

I chose **scheduling actions (ScheduleMeeting, Reschedule, Cancel) as tools** because they require computation
(finding available slots, checking conflicts) and state changes (modifying calendars). These are procedures, not data.

I chose **calendar data, templates, and analytics as resources** because they're primarily for reading data and
making decisions. Agents need to see this information without triggering state changes. The URI patterns
reflect what data is being accessed (calendar for meetings, templates for standard formats, analytics for patterns).

I created a **MeetingSchedulingStrategy prompt** to encode the domain knowledge that scheduling isn't just about
finding open slots—it's about respecting human constraints (time zone differences, fatigue from back-to-back meetings,
appropriate meeting lengths based on agenda). This activates the AI's reasoning about the human context, not just mechanics.

## Design Reasoning Principles Demonstrated

1. **Tools are verbs** (Schedule, Reschedule, Cancel, Analyze)
2. **Resources are nouns** (Meetings, Templates, Analytics)
3. **Prompts are expertise** (How to think about scheduling decisions)
4. **Fail-fast validation**: Tools should check availability before committing changes
5. **Clear resource URIs**: Pattern hierarchy (calendar → user_id → data_type) reveals the data model
6. **Separation of concerns**: Analytics is a resource, not a tool, because it's a read-only computation

```

**Common Issues to Avoid:**

- **Over-tooling**: Don't create tools for every operation. "GetMeetingList" shouldn't be a tool—it's a resource.
- **Under-prompting**: Don't skip prompts. AI benefits from encoded domain expertise (meeting culture, time zone awareness, agenda complexity).
- **Unclear URIs**: Resource patterns should intuitively reflect what data they contain.
- **Missing startup validation**: Consider: what happens if the calendar system is unreachable? Validate configuration at server startup.

</details>

---

## Assessment Scoring & Feedback

### Part 1: Concept Recognition (10 Questions)

- **Score**: 1 point per correct answer
- **Passing threshold**: 7 out of 10 (70%)
- **What it measures**: Understanding of core MCP concepts and server mechanics
- **If you score below 7**: Review the lessons covering decorators, type hints, resource patterns, and protocol fundamentals

### Part 2: Code Analysis & Debugging (5 Scenarios)

- **Score**: 2 points per correct fix (identification + explanation + correct solution)
- **Passing threshold**: 7 out of 10 (70%)
- **What it measures**: Ability to recognize real-world bugs and apply fixes
- **If you score below 7**: Pay special attention to: type hints, stdout vs stderr, secret handling, URI patterns, fail-fast validation

### Part 3: Design Challenge (1 Open-Ended)

- **Scoring rubric**:
  - **Tools vs Resources vs Prompts decision**: Do your choices align with the principle (tools=actions, resources=data, prompts=expertise)? (0-3 points)
  - **Completeness**: Have you specified 3-5 tools, 2-3 resources, and 1-2 prompts? (0-2 points)
  - **Justification clarity**: Is your reasoning clear and does it reference the architectural principles from chapter? (0-3 points)
  - **Realism**: Would this server actually serve the stated requirements? (0-2 points)
  - **Total**: 0-10 points

- **Passing threshold**: 7 out of 10 (70%)
- **What it measures**: Ability to make sound architectural decisions, justify choices, and design servers for real-world domains
- **If you score below 7**: Review lessons on tool design, resource patterns, and when to use prompts for domain expertise

### Overall Assessment

- **Total possible points**: 10 + 10 + 10 = 30 points
- **Passing score**: 21 points (70%)
- **Scoring interpretation**:
  - **21-25 points (70-83%)**: Competent. You understand core concepts and can identify bugs. Work on the design challenge for deeper thinking.
  - **26-27 points (84-90%)**: Proficient. You demonstrate solid understanding across all three areas.
  - **28-30 points (94-100%)**: Excellent. You're ready for advanced MCP design work in production scenarios.

---

## Try With AI: Deepening Your Design Thinking

Use AI to extend your learning beyond the assessment.

### Prompt 1: Architectural Critique

If you completed the design challenge, share your server specification with AI:

"I designed an MCP server for [domain] with these tools, resources, and prompts: [paste your spec]. Critique this design. What architectural issues might appear in production? What did I miss?"

**What you're learning**: How to think critically about system design and anticipate production concerns.

### Prompt 2: Implementation Specification

Based on your design, ask AI for implementation details:

"For my [domain] MCP server, write the specification for the [ToolName] tool. Include: parameter types, expected errors, validation rules, and how it handles edge cases. Format as a spec.md file."

**What you're learning**: How to translate architectural designs into detailed implementation specifications that drive code generation.

### Prompt 3: Test Coverage Planning

Ask AI to help you think through testing:

"I have an MCP server with tools for [list tools] and resources for [list resources]. Design a comprehensive test plan. What scenarios should be tested for each tool? How do you test resource availability? What error conditions should be validated?"

**What you're learning**: How thorough testing protects production systems and catches bugs early.

---

## Next Steps After Assessment

**If you passed (score ≥ 21):**

Your Chapter 38 mastery is validated. You understand how to build, design, and deploy custom MCP servers. You're ready to:

1. **Build your own MCP servers** for your domain
2. **Contribute custom servers** to the open-source MCP ecosystem
3. **Advance to Chapter 39** (Agent Skills & MCP Code Execution) to explore more complex agent architectures
4. **Build Digital FTEs** that combine MCP servers with custom agents for production workflows

**If you didn't pass (score < 21):**

Review is recommended before moving forward:

1. **Identify weak areas** using your scoring breakdown
2. **Re-read the relevant lesson(s)** where gaps appeared
3. **Try the "Try With AI" prompts** to deepen understanding in weak areas
4. **Retake the assessment** after review

**Either way**: The investment in understanding MCP server design pays dividends. Every custom MCP server you build becomes reusable capability that multiplies the power of your AI agents.

---

## Reflecting on Chapter 38

As you complete this chapter, consider:

- What MCP server would *you* build for your domain or organization?
- What capabilities would transform your workflow if exposed through MCP?
- How does specification-driven server design (thinking about tool contracts before implementation) improve quality compared to iterative development?

These reflections prepare you for the next chapters, where you'll build increasingly sophisticated agent systems and Digital FTEs.

Good work on mastering MCP server development. You've learned a critical skill for building AI agent infrastructure.
