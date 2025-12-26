---
sidebar_position: 3
title: "Tools: The Model-Controlled Primitive"
chapter: 37
lesson: 3
duration_minutes: 14

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding MCP Tool Architecture"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can explain tool discovery (tools/list), tool execution (tools/call), and schema design patterns for MCP servers"

  - name: "Designing Tool Schemas for AI Agents"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can design tool schemas with appropriate names, descriptions, and JSON Schema input validation"

  - name: "Implementing MCP Tools with FastMCP"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can decorate Python functions as MCP tools using FastMCP decorators and Pydantic field descriptions"

learning_objectives:
  - objective: "Understand the model-controlled paradigm: AI decides when and how to invoke tools"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of why 'model-controlled' differs from function-call SDKs"

  - objective: "Implement tool discovery (tools/list) and understand JSON-RPC request/response flow"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Recognition of tools/list JSON-RPC structure and understanding of tool schema metadata"

  - objective: "Implement tool execution (tools/call) with input validation and result handling"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Ability to handle tool invocation with parameter validation and error handling"

  - objective: "Design MCP tool schemas following naming, description, and annotation standards"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Creation of well-designed tool schemas with semantic names and safety annotations"

  - objective: "Compare MCP tool standardization to Chapters 34-36 SDK tool patterns"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Articulation of why standardization matters and how MCP solves schema duplication"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (model-controlled, discovery, execution, schema, input validation, annotations, JSON-RPC, FastMCP) within B1 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Research idempotency and read-only hints; design a tool that requires both annotations; implement error handling with structured error responses"
  remedial_for_struggling: "Focus on single tool example (read_document); practice JSON-RPC structure separately; create tool with simple input validation first"
---

# Tools: The Model-Controlled Primitive

You've spent three chapters building agents with three different SDKs, each defining tools their own way. The OpenAI SDK uses one format, Claude SDK another, Google ADK a third. If you wanted to reuse a tool across SDKs, you'd write it three times—once for each framework.

MCP solves this fragmentation completely. **Tools in MCP are standardized, universal primitives.** Define them once, use them everywhere.

But there's something deeper here: The term "model-controlled" is the key insight. In the SDKs you learned, you controlled when tools were available. "Here are your tools, pick one." In MCP, the AI model decides when and how to use tools. That changes the relationship fundamentally.

## The Model-Controlled Paradigm

When you use the OpenAI SDK, you pass tools to the API:

```python
response = client.chat.completions.create(
    model="gpt-4",
    tools=[{
        "type": "function",
        "function": {
            "name": "get_weather",
            "description": "Get weather for a location",
            "parameters": {...}
        }
    }]
)
```

You decide what tools exist. You decide what the model can call. You're in control.

MCP inverts this: **The server defines what tools are available. The model discovers them, decides when to use them, and invokes them autonomously.**

```json
// Server broadcasts: "Here are my tools"
{
  "tools": [
    {
      "name": "read_document",
      "description": "Read a document by ID",
      "inputSchema": {...}
    }
  ]
}

// Model discovers tools → Decides to use read_document → Invokes it
// No need for human to pre-authorize which tools the model can access
```

**Why this matters**: In production systems, this means agents can use tools you define WITHOUT you having to update the agent code. Define a new tool on your server, broadcast it via tools/list, and Claude Code immediately gains that capability. This is why MCP works with any MCP-compatible client—the protocol handles discovery and invocation uniformly.

## Tool Discovery: tools/list

Before an AI agent can use a tool, it must know what tools exist. MCP handles this through the `tools/list` request.

### The Sequence

```
Client (Claude Code)        Server (Your MCP)
        |                           |
        |-----> tools/list -------->|
        |                           |
        |<--- Tool definitions -----|
        |                           |
```

### The Request

The client (Claude Code, Cursor, etc.) sends a request with no parameters:

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "tools/list",
  "params": {}
}
```

Simple. No filtering, no pagination. Just: "What tools do you have?"

### The Response

Your MCP server responds with the full tool inventory:

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "tools": [
      {
        "name": "read_document",
        "description": "Read the full contents of a document by its ID",
        "inputSchema": {
          "type": "object",
          "properties": {
            "doc_id": {
              "type": "string",
              "description": "The document ID (e.g., 'DOC-2024-001')"
            }
          },
          "required": ["doc_id"]
        }
      },
      {
        "name": "list_documents",
        "description": "List all available documents with metadata",
        "inputSchema": {
          "type": "object",
          "properties": {
            "filter": {
              "type": "string",
              "description": "Optional: filter by document type (contract, specification, etc.)"
            }
          }
        }
      }
    ]
  }
}
```

**Key observations:**

- Each tool has a `name` (how the model invokes it)
- Each has a `description` (what it does, why the model should use it)
- Each has `inputSchema` (JSON Schema defining required/optional parameters)
- No authentication tokens, no API keys—the server handles that internally

When Claude Code receives this, it knows: "I can read documents and list documents. The read_document tool requires a doc_id. Let me use this."

## Tool Execution: tools/call

Once the model decides to invoke a tool, it sends `tools/call`:

```
Client (Claude Code)        Server (Your MCP)
        |                           |
        |---- tools/call ----->|
        | (tool name + args)        |
        |                           |
        |<----- Result -------------|
        |                           |
```

### The Request

The client sends the tool name and arguments:

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "method": "tools/call",
  "params": {
    "name": "read_document",
    "arguments": {
      "doc_id": "DOC-2024-001"
    }
  }
}
```

The model decided to call `read_document` with `doc_id` = "DOC-2024-001". The server executes it.

### The Response

Your server reads the document and returns the result:

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "result": {
    "content": [
      {
        "type": "text",
        "text": "Document Title: Q4 2024 Revenue Report\n\nRevenue: $2.3M\nCosts: $1.1M\nProfit: $1.2M\n\nKey Findings:\n- 34% growth YoY\n- Cloud revenue up 45%\n- Support costs decreased 12%"
      }
    ]
  }
}
```

Or if the tool fails (document doesn't exist):

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "error": {
    "code": -32600,
    "message": "Document not found",
    "data": {
      "reason": "No document with ID DOC-2024-001 in system"
    }
  }
}
```

**Key observations:**

- The tool name and arguments come directly from the model's decision
- The server executes the tool in its own context (it has database access, file system access, etc.)
- Results return as structured `content` array (text, image, etc.)
- Errors are JSON-RPC errors with meaningful messages

## Tool Schema Design

Good tool schemas are the difference between agents using tools effectively and ignoring them. Poor schemas → models don't understand when to use the tool. Great schemas → models invoke them perfectly.

### Naming Convention

**Tool names should follow the pattern**: `[service]_[action]`

Examples:

- ✅ `github_create_issue` (service: GitHub, action: create issue)
- ✅ `database_query_users` (service: database, action: query users)
- ✅ `slack_send_message` (service: Slack, action: send message)
- ❌ `create_issue` (ambiguous—which service?)
- ❌ `createGitHubIssue` (camelCase—inconsistent with snake_case MCP convention)
- ❌ `GH_CI` (cryptic abbreviation—models don't understand)

### Descriptions

Descriptions answer: "Why would I use this tool?"

**Good description**:

```
"Create a new GitHub issue in a specific repository. Use this when the user wants to report a bug, request a feature, or document a task. Requires repo name and issue title."
```

**Poor description**:

```
"Create issue"  // Too vague—when should this be used?
```

### Input Schema: JSON Schema

MCP tools use JSON Schema to define what parameters are valid. This enables input validation before your tool is invoked.

Example schema for `github_create_issue`:

```json
{
  "type": "object",
  "properties": {
    "repo": {
      "type": "string",
      "description": "Repository name in format owner/repo (e.g., anthropic/claude-sdk)"
    },
    "title": {
      "type": "string",
      "description": "Issue title (50-200 characters recommended)"
    },
    "body": {
      "type": "string",
      "description": "Issue description using Markdown"
    },
    "labels": {
      "type": "array",
      "items": {"type": "string"},
      "description": "Optional labels (e.g., ['bug', 'urgent'])"
    },
    "assignee": {
      "type": "string",
      "description": "Optional GitHub username to assign"
    }
  },
  "required": ["repo", "title"]
}
```

**Critical fields**:

- `type: "object"` - Top level is always an object
- `properties` - Define each parameter
- `required` - Which parameters are mandatory
- `description` on each property - Why this parameter exists, what format it expects

### Tool Annotations

MCP allows annotations that communicate tool properties to the model. These help the model understand side effects.

**Common annotations**:

```json
{
  "name": "delete_file",
  "description": "Permanently delete a file from the system",
  "inputSchema": {...},
  "destructiveHint": true,  // Warning: This tool can't be undone
  "readOnlyHint": false     // This tool modifies state
}
```

**Annotation meanings**:

| Annotation | Meaning | Example |
|-----------|---------|---------|
| `readOnlyHint: true` | Tool only reads, doesn't modify | `read_document`, `list_files` |
| `destructiveHint: true` | Tool can't be undone (deletion) | `delete_file`, `drop_table` |
| `idempotentHint: true` | Safe to call multiple times with same args | `create_user` (with unique email check) |
| `openWorldHint: true` | Tool might work with external systems in unpredictable ways | `web_search`, `api_call_external` |

Annotations help models reason about risks. A model might ask for confirmation before calling a destructive tool, or refuse to call it without explicit user permission.

## Python Implementation with FastMCP

FastMCP is a minimal framework that turns Python functions into MCP tools. It handles JSON-RPC, serialization, and transport.

### Basic Tool

```python
from mcp.server.fastmcp import FastMCP
from pydantic import Field

mcp = FastMCP("DocumentServer")

@mcp.tool(
    name="read_document",
    description="Read the full contents of a document by its ID"
)
def read_document(
    doc_id: str = Field(description="Document ID (e.g., 'DOC-2024-001')")
) -> str:
    """
    Implementation: Load document from database, return content
    """
    # Validate doc_id format
    if not doc_id.startswith("DOC-"):
        raise ValueError(f"Invalid doc_id format: {doc_id}")

    # Load from database (pseudo-code)
    doc = database.find_document(doc_id)
    if not doc:
        raise ValueError(f"Document not found: {doc_id}")

    return doc.content
```

**How this works**:

1. Decorate function with `@mcp.tool()` - Registers it as MCP tool
2. Provide `name` and `description` - These appear in tools/list
3. Use type hints (`str`, `int`, etc.) - FastMCP converts to JSON Schema
4. Use Pydantic `Field()` for descriptions - These explain parameters to the model
5. Raise exceptions for validation failures - FastMCP converts to JSON-RPC errors

### Tool with Multiple Parameters

```python
@mcp.tool(
    name="create_github_issue",
    description="Create a new GitHub issue in a repository"
)
def create_github_issue(
    repo: str = Field(description="Repository in format owner/repo (e.g., anthropic/claude-sdk)"),
    title: str = Field(description="Issue title"),
    body: str = Field(description="Issue description (Markdown supported)"),
    labels: list[str] = Field(default=[], description="Optional labels to apply")
) -> dict:
    """
    Creates GitHub issue and returns issue number and URL
    """
    # Validate repo format
    if "/" not in repo:
        raise ValueError(f"Invalid repo format: {repo}. Expected owner/repo")

    owner, repo_name = repo.split("/", 1)

    # Call GitHub API
    issue = github.create_issue(
        owner=owner,
        repo=repo_name,
        title=title,
        body=body,
        labels=labels
    )

    return {
        "issue_number": issue.number,
        "issue_url": issue.html_url,
        "title": issue.title
    }
```

**How FastMCP generates schema**:

```json
{
  "name": "create_github_issue",
  "description": "Create a new GitHub issue in a repository",
  "inputSchema": {
    "type": "object",
    "properties": {
      "repo": {
        "type": "string",
        "description": "Repository in format owner/repo..."
      },
      "title": {
        "type": "string",
        "description": "Issue title"
      },
      "body": {
        "type": "string",
        "description": "Issue description (Markdown supported)"
      },
      "labels": {
        "type": "array",
        "items": {"type": "string"},
        "description": "Optional labels..."
      }
    },
    "required": ["repo", "title", "body"]
  }
}
```

FastMCP automatically:

- Converts Python type hints to JSON Schema types
- Marks parameters with defaults as optional
- Uses Field descriptions in the schema
- Validates inputs before calling your function
- Converts exceptions to JSON-RPC errors

### Tool with Annotations

```python
@mcp.tool(
    name="delete_file",
    description="Permanently delete a file from the system"
)
def delete_file(
    file_path: str = Field(description="Absolute path to file to delete")
) -> dict:
    """
    Deletes file from filesystem. This cannot be undone.
    """
    # Validate path (prevent deleting system files)
    if file_path.startswith("/usr/") or file_path.startswith("/etc/"):
        raise ValueError("Cannot delete system files")

    os.remove(file_path)
    return {"status": "deleted", "path": file_path}

# Register with destructive annotation
delete_file._destructive = True
delete_file._readonly = False
```

**Output**: MCP broadcasts this tool with annotations:

```json
{
  "name": "delete_file",
  "description": "Permanently delete a file from the system",
  "inputSchema": {...},
  "destructiveHint": true,
  "readOnlyHint": false
}
```

## Comparison: MCP vs SDK Tool Patterns

| Aspect | SDK Tools (Ch 34-36) | MCP Tools |
|--------|-------------------|-----------|
| **Tool schema format** | Different for each SDK | Standardized (JSON Schema) |
| **Where tools defined** | In your agent code | On server, discovered at runtime |
| **Model access control** | You pre-authorize tools | Model discovers & decides |
| **Reusability** | Tool code tied to SDK | Server works with any MCP client |
| **Setup complexity** | Configure in agent initialization | Configure client connection once |
| **Extensibility** | Add new tools → update agent code | Add new tools → server broadcasts |

**Example**: If you want both Claude Code AND Cursor to access a database query tool:

**SDK approach** (Chapters 34-36):
```
Claude SDK:     tool_schema_v1 = {...}  # Your format
OpenAI SDK:    tool_schema_v2 = {...}  # Different format
Cursor SDK:    tool_schema_v3 = {...}  # Yet different

Result: Write database tool THREE times
```

**MCP approach**:
```
MCP Server:    tools = [database_query_tool]  # One schema

Claude Code:   Reads tools/list → Uses tool
Cursor:        Reads tools/list → Uses tool
Zed:           Reads tools/list → Uses tool

Result: Write database tool ONCE
```

## How This Differs from Chapters 34-36

In Chapters 34-36, you learned tool use with three different SDKs. Each SDK required:

1. Defining tool schema in its specific format
2. Registering tool with the client
3. Handling tool invocations through SDK callbacks
4. Converting between SDK and your internal representation

MCP standardizes all three steps. You define one schema (JSON Schema) that works everywhere. You register once on the server. Tool invocations use the same JSON-RPC protocol across all clients.

**Key insight**: MCP doesn't replace SDKs. It sits **beside** them. Your agent (built with SDK from Ch 34-36) connects to MCP servers and gains their tools automatically. The agent handles reasoning and planning. MCP handles tool discovery and execution.

## Try With AI

Use Claude Code or Cursor (both MCP clients) for this exercise.

### Prompt 1: Design a Tool Schema

```
I want to create an MCP tool that allows AI agents to query a customer database.
The query should support filtering by:
- Customer name (optional, substring match)
- Registration date range (optional, from/to dates)
- Account status (optional: active, inactive, suspended)

Help me design the tool schema. What should the tool name be? What properties
should inputSchema have? What should be required vs optional? What description
would help an AI model understand when to use this tool?
```

**What you're learning**: Schema design requires clarity about what parameters make sense and what makes a tool discoverable to AI models.

### Prompt 2: Compare with SDK Tools

```
In Chapters 34-36, I defined tools for OpenAI SDK, Claude SDK, and Google ADK.
The tool definition was slightly different in each SDK. Now with MCP, I define
once and it works everywhere.

Show me an example: Take a simple "send_email" tool and show how I'd define it
in OpenAI SDK vs MCP. What's the key difference? Why is standardization valuable?
```

**What you're learning**: Understanding the problem MCP solves helps you recognize when standardization matters in systems design.

### Prompt 3: Implement a FastMCP Tool

```
I want to build an MCP server with a tool that reads Markdown files from a directory
and returns their contents. The tool should:
- Accept a filename parameter
- Only allow reading from a specific directory (security)
- Raise an error if file not found
- Return file content as a string

Write a Python function decorated with FastMCP that implements this. Include
validation, error handling, and a good description for the AI model.
```

**What you're learning**: Implementation patterns for real tools that handle validation, security, and error cases—the skills you'll need for Chapter 38 when you build your own MCP server.

### Safety Note

As you design tools for AI agents, remember: Tools are permissions. A tool that can delete files is a deletion permission. A tool that can send emails is an email-sending permission. Always validate inputs, check authorization, and fail safely. When building MCP tools for production, add rate limiting and audit logging.

