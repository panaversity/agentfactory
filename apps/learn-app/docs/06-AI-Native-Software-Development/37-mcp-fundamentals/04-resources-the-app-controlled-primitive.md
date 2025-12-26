---
sidebar_position: 4
title: "Resources: The App-Controlled Primitive"
chapter: 37
lesson: 4
duration_minutes: 12

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding MCP Resource Architecture"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can explain resource discovery (resources/list), resource reading (resources/read), URI patterns, and MIME type handling"

  - name: "Designing Resource Schemas for AI Agents"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can design resource URIs with direct and templated patterns, and choose appropriate MIME types for different content"

  - name: "Implementing MCP Resources with FastMCP"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can decorate Python functions as MCP resources using FastMCP decorators and handle URI templating"

  - name: "Distinguishing Resources from Tools in MCP Design"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can evaluate when to use resources (read-only data access) vs tools (actions) based on control and side-effect requirements"

learning_objectives:
  - objective: "Understand the app-controlled paradigm: Application decides when to expose resources to the model"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of why 'app-controlled' differs from 'model-controlled' (tools)"

  - objective: "Implement resource discovery (resources/list) and understand URI patterns"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Recognition of resources/list JSON-RPC structure and understanding of direct vs templated URIs"

  - objective: "Implement resource reading (resources/read) with MIME type handling"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Ability to handle resource requests with proper content-type encoding and error handling"

  - objective: "Design resource URIs and choose appropriate MIME types"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Creation of well-designed resource schemas with semantic URIs and correct MIME types"

  - objective: "Compare resources to tools and decide when each primitive is appropriate"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Evaluation of scenarios to determine resource vs tool usage based on control and side effects"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (app-controlled, resource discovery, resource reading, URI patterns, MIME types, direct resources, templated resources, resource vs tools) within B1 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Design a hybrid system using both templated resources and tools; implement access control based on resource URI parameters; research conditional resource exposure"
  remedial_for_struggling: "Focus on single resource example (document_library); practice URI templating separately; compare to tools side-by-side before integration"
---

# Resources: The App-Controlled Primitive

In the previous lesson, you learned about **tools**: the model-controlled primitive where Claude Code decides when and how to invoke them. Tools are perfect for actions—"Create an issue," "Send an email," "Delete a file."

But what about **data access**? When you want Claude Code to read documents, fetch configurations, or search through existing information without making decisions about *when* to expose that data—that's where resources come in.

Resources represent the inverse control model: **The application decides when to expose data. The model reads it.**

Think of the difference like this:

- **Tools** are like buttons your agent can press. Claude Code looks at available buttons and decides which one to push.
- **Resources** are like file cabinets your application unlocks. Claude Code can browse what's inside, but your application controls the key.

This distinction becomes critical when building Digital FTEs. You don't always want your agent making autonomous decisions about data access. Sometimes you want the agent to work with data *you've already decided to provide*.

## The App-Controlled Paradigm

Consider a document mention feature in Claude Code: You type `@quarterly_report.pdf` in your prompt, and Claude Code should fetch that document for context.

**With tools approach** (wrong for this use case):
```
You type: @quarterly_report.pdf
Claude Code: "I could call the fetch_document tool with this name"
Claude Code: "But should I? Do I have permission? Is this secure?"
Result: Friction, uncertainty, potential security issues
```

**With resources approach** (correct):
```
You type: @quarterly_report.pdf
Application: "This document is already mentioned by the user. I'll expose it as a resource."
Claude Code: "Perfect! I have access to this resource. Reading..."
Result: Seamless, safe, intentional
```

The key insight: **The application has already made the security decision.** Resources are pre-authorized data the application explicitly exposes. The model doesn't need to decide—it just reads what's available.

## Resource Discovery: resources/list

Before Claude Code can read a resource, it must know what resources exist. MCP handles this through the `resources/list` request.

### The Sequence

```
Client (Claude Code)        Server (Your MCP)
        |                           |
        |-----> resources/list ---->|
        |                           |
        |<--- Resource definitions -|
        |                           |
```

### The Request

The client sends a request with no parameters:

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "resources/list",
  "params": {}
}
```

Simple: "What resources do you have?"

### The Response

Your MCP server responds with the resource inventory:

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "resources": [
      {
        "uri": "docs://documents",
        "name": "All documents",
        "description": "Directory of all available documents",
        "mimeType": "application/json"
      },
      {
        "uri": "docs://documents/quarterly-report-2024",
        "name": "Q4 2024 Quarterly Report",
        "description": "Financial and operational report for Q4 2024",
        "mimeType": "text/plain"
      },
      {
        "uri": "docs://documents/architecture-guide",
        "name": "System Architecture Guide",
        "description": "Technical documentation for system design",
        "mimeType": "text/markdown"
      }
    ]
  }
}
```

**Key observations:**

- Each resource has a `uri` (unique identifier following scheme://path pattern)
- Each has a `name` (human-readable label)
- Each has a `description` (what it contains, why the model might want it)
- Each has a `mimeType` (how to interpret the content—text, JSON, PDF, etc.)
- Resources are static metadata—you're describing what's available, not fetching it yet

The URI pattern is critical. Notice:

- `docs://documents` - A direct resource (list all documents)
- `docs://documents/{doc_id}` - A templated resource (dynamic, matches patterns)

## Direct vs Templated Resources

MCP resources support two patterns:

### Direct Resources

Direct resources have **static URIs** that represent exactly one piece of data:

```json
{
  "uri": "docs://documents",
  "name": "Documents Index",
  "description": "Complete list of available documents",
  "mimeType": "application/json"
}
```

Example use: "Show me all available documents"

Claude Code knows this resource exists, can request it, and gets back the full list.

### Templated Resources

Templated resources use **URI patterns** with placeholders that Claude Code fills in dynamically:

```json
{
  "uri": "docs://documents/{doc_id}",
  "name": "Document by ID",
  "description": "Read full contents of a specific document",
  "mimeType": "text/plain"
}
```

Example use: Claude Code sees `@quarterly-report-2024` in your message, recognizes the pattern, and requests `docs://documents/quarterly-report-2024`.

**The flow**:

```
You type: @quarterly-report-2024
Claude Code sees this matches pattern docs://documents/{doc_id}
Claude Code: "I have a templated resource for this!"
Claude Code requests: resources/read with uri = docs://documents/quarterly-report-2024
Server: Reads the document, returns content
Claude Code: Includes it in context automatically
```

This is why document mentions work seamlessly. The application discovered the template, Claude Code matched it to your mention, and the resource was fetched without explicit tool calls.

## Resource Reading: resources/read

Once Claude Code identifies which resource to read, it sends `resources/read`:

```
Client (Claude Code)        Server (Your MCP)
        |                           |
        |---- resources/read ---->|
        | (uri)                     |
        |                           |
        |<----- Content ------------|
        |                           |
```

### The Request

The client requests a specific resource by URI:

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "method": "resources/read",
  "params": {
    "uri": "docs://documents/quarterly-report-2024"
  }
}
```

### The Response

Your server reads the resource and returns the content with appropriate MIME type:

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "result": {
    "contents": [
      {
        "uri": "docs://documents/quarterly-report-2024",
        "mimeType": "text/plain",
        "text": "QUARTERLY REPORT Q4 2024\n\nRevenue: $2.3M\nCosts: $1.1M\nProfit: $1.2M\n\nKey Highlights:\n- Cloud revenue up 45%\n- Customer retention: 94%\n- Geographic expansion: 3 new markets"
      }
    ]
  }
}
```

Or for JSON resources:

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "result": {
    "contents": [
      {
        "uri": "docs://documents",
        "mimeType": "application/json",
        "text": "[{\"id\": \"quarterly-report-2024\", \"title\": \"Q4 2024 Report\", \"type\": \"financial\", \"size\": 2048}, {\"id\": \"architecture-guide\", \"title\": \"System Architecture\", \"type\": \"technical\", \"size\": 5120}]"
      }
    ]
  }
}
```

Or if the resource doesn't exist:

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "error": {
    "code": -32600,
    "message": "Resource not found",
    "data": {
      "reason": "No resource at docs://documents/unknown-doc"
    }
  }
}
```

**Key observations:**

- The URI in the response matches the request (confirm which resource was fetched)
- The `mimeType` tells Claude Code how to interpret content (text/plain, application/json, text/markdown, etc.)
- For binary content (PDFs), return as base64-encoded text
- Errors use JSON-RPC error format with meaningful messages

## MIME Type Handling

MIME types tell Claude Code how to interpret resource content. Choosing the right type is critical:

| MIME Type | Use Case | Example |
|-----------|----------|---------|
| `text/plain` | Plain text files, unformatted content | .txt files, raw logs, configuration values |
| `text/markdown` | Markdown-formatted documentation | README.md, guides, formatted notes |
| `text/html` | HTML content | Web pages, HTML reports |
| `application/json` | Structured data (objects, arrays) | Configuration files, API responses, data indexes |
| `application/xml` | XML/structured markup | Configuration files, data exports |
| `application/pdf` | PDF documents (returned as base64) | Reports, forms, archived documents |
| `image/png`, `image/jpeg` | Images (returned as base64) | Screenshots, diagrams |
| `text/csv` | Comma-separated values | Data exports, spreadsheet data |

**Good MIME type usage**:
- `docs://config.json` → `application/json` (because it's structured data)
- `docs://readme.md` → `text/markdown` (because it needs formatting preserved)
- `docs://system.log` → `text/plain` (because it's raw text)

**Wrong MIME types**:
- `docs://config.json` → `text/plain` (loses structure information)
- `docs://readme.md` → `text/plain` (loses markdown semantics)

## Python Implementation with FastMCP

FastMCP makes resource implementation straightforward.

### Direct Resource

```python
from mcp.server.fastmcp import FastMCP

mcp = FastMCP("DocumentServer")

@mcp.resource(
    "docs://documents",
    name="Documents Index",
    description="Complete list of available documents",
    mime_type="application/json"
)
def list_documents() -> str:
    """
    Returns JSON array of all available documents with metadata
    """
    documents = [
        {
            "id": "quarterly-report-2024",
            "title": "Q4 2024 Quarterly Report",
            "type": "financial",
            "size": 2048
        },
        {
            "id": "architecture-guide",
            "title": "System Architecture Guide",
            "type": "technical",
            "size": 5120
        }
    ]

    import json
    return json.dumps(documents)
```

**How this works**:

1. Decorate function with `@mcp.resource()` - Registers it as MCP resource
2. Provide URI pattern (no placeholders for direct resources)
3. Provide `name` and `description` - These appear in resources/list
4. Specify `mime_type` - How to interpret returned content
5. Return string content - FastMCP handles JSON serialization

### Templated Resource

```python
@mcp.resource(
    "docs://documents/{doc_id}",
    name="Document by ID",
    description="Read full contents of a specific document",
    mime_type="text/plain"
)
def fetch_document(doc_id: str) -> str:
    """
    Returns full contents of document by ID
    """
    # Validate doc_id format
    if not doc_id or "/" in doc_id:
        raise ValueError(f"Invalid doc_id: {doc_id}")

    # Load document from database (pseudo-code)
    documents = {
        "quarterly-report-2024": "QUARTERLY REPORT Q4 2024\n\nRevenue: $2.3M\nCosts: $1.1M...",
        "architecture-guide": "SYSTEM ARCHITECTURE GUIDE\n\nThis document describes...",
    }

    if doc_id not in documents:
        raise ValueError(f"Document not found: {doc_id}")

    return documents[doc_id]
```

**How templating works**:

1. URI pattern includes `{doc_id}` placeholder
2. When Claude Code requests `docs://documents/quarterly-report-2024`, FastMCP extracts `doc_id = "quarterly-report-2024"`
3. Passes `doc_id` as parameter to your function
4. Your function validates and returns content
5. FastMCP returns with appropriate MIME type

### Multiple Templated Parameters

```python
@mcp.resource(
    "docs://documents/{doc_type}/{doc_id}",
    name="Filtered Document Access",
    description="Read documents filtered by type and ID",
    mime_type="application/json"
)
def fetch_filtered_document(doc_type: str, doc_id: str) -> str:
    """
    Returns document matching both type and ID filters
    """
    # Validate parameters
    valid_types = ["financial", "technical", "legal"]
    if doc_type not in valid_types:
        raise ValueError(f"Invalid doc_type: {doc_type}")

    if not doc_id or "/" in doc_id:
        raise ValueError(f"Invalid doc_id: {doc_id}")

    # Load from database
    doc = database.find_document(doc_type=doc_type, doc_id=doc_id)
    if not doc:
        raise ValueError(f"Document not found: {doc_type}/{doc_id}")

    import json
    return json.dumps(doc)
```

**Output**: MCP broadcasts this resource pattern:

```json
{
  "uri": "docs://documents/{doc_type}/{doc_id}",
  "name": "Filtered Document Access",
  "description": "Read documents filtered by type and ID",
  "mimeType": "application/json"
}
```

Claude Code can now request: `docs://documents/financial/quarterly-report-2024`

## Resources vs Tools: When to Use Each

This is the critical design decision. Both primitives provide data access, but they differ fundamentally:

| Aspect | Resources | Tools |
|--------|-----------|-------|
| **Control** | App-controlled (application decides what's exposed) | Model-controlled (model decides when to invoke) |
| **Purpose** | Read existing data | Perform actions or side effects |
| **Timing** | Pre-authorized access | On-demand invocation |
| **Side effects** | None (read-only) | Possible (mutations, side effects) |
| **Use case** | Document mention (@file), context injection, reference data | Creating issues, sending emails, updating records |
| **Security model** | Application pre-approves exposure | Model must request permission implicitly |
| **Example** | "Fetch this document I mentioned" | "Create a GitHub issue from this problem" |

**When to use Resources**:
1. **Pre-authorized data**: Application has already decided this data should be available
2. **Read-only access**: No side effects, just fetching information
3. **Document mentions**: @document features, context injection
4. **Reference data**: Configuration, schemas, lookup tables
5. **User-initiated context**: Data the user explicitly referenced

**When to use Tools**:
1. **Autonomous actions**: Model decides to create/modify/delete
2. **Side effects required**: Changes state in external systems
3. **Conditional execution**: "Should I do this?" requires model reasoning
4. **Permissions**: Model must request, user (or system) approves
5. **Error recovery**: Tool calls support structured error handling and retries

**Example scenarios**:

Scenario: "I want Claude Code to access my codebase"

- Resource: Expose project files as `code://files/{filename}` - Claude Code reads files you mention
- Tool: Provide `code_search(query)` - Claude Code decides when to search

**Which is better?** Both! Resources for browsing, tools for actions.

Scenario: "I want Claude Code to modify my codebase"

- Resource: WRONG - You don't want pre-authorized write access
- Tool: RIGHT - Model requests to modify, system validates safely

## Design Pattern: Document Mention Feature

Here's how to implement a robust document mention system combining resources and tools:

```python
@mcp.resource(
    "docs://documents",
    name="Documents Index",
    description="List of all available documents for mention",
    mime_type="application/json"
)
def list_documents() -> str:
    """Expose document list so Claude Code knows what's available"""
    documents = load_all_documents()
    return json.dumps([
        {
            "id": doc.id,
            "name": doc.name,
            "type": doc.type
        }
        for doc in documents
    ])

@mcp.resource(
    "docs://documents/{doc_id}",
    name="Document Content",
    description="Full content of a specific document",
    mime_type="text/plain"
)
def fetch_document(doc_id: str) -> str:
    """Fetch requested document - application controls exposure"""
    doc = database.find_document(doc_id)
    if not doc or not doc.is_accessible():
        raise ValueError(f"Document not accessible: {doc_id}")

    return doc.content

@mcp.tool(
    name="create_document_reference",
    description="Create a reference to a document for future context"
)
def create_reference(doc_id: str, context: str) -> dict:
    """Tool for creating document references - model-controlled action"""
    doc = database.find_document(doc_id)
    if not doc:
        raise ValueError(f"Document not found: {doc_id}")

    reference = database.create_reference(
        doc_id=doc_id,
        context=context,
        created_by="claude-code"
    )

    return {
        "reference_id": reference.id,
        "doc_id": doc_id,
        "status": "created"
    }
```

**How it flows**:

1. Claude Code requests `resources/list` → Sees `docs://documents` and pattern
2. User types `@quarterly-report-2024` in their message
3. Claude Code matches against pattern, requests `resources/read` with that doc
4. Application exposes document (pre-authorized)
5. Claude Code analyzes document in context
6. If Claude Code wants to reference this for future use, it calls `create_document_reference` tool
7. Tool is model-controlled—application validates that this action is safe

## Try With AI

Use Claude Code or your AI companion for these exercises.

### Prompt 1: Design a Resource Schema

```
I want to create an MCP resource system that exposes a knowledge base
to Claude Code. The knowledge base has:

- Articles (organized by category and ID)
- Configuration documents
- Frequently asked questions

Design a resource schema for me:
1. What direct resources should I expose? (list all articles, FAQ index)
2. What templated resources should I expose? (access specific articles)
3. What URIs and MIME types would work best?
4. How would a user mention an article in their prompt?

Show me the resources/list JSON response your system would return.
```

**What you're learning**: Resource schema design requires understanding both how to expose data and how users (and Claude Code) will discover and access it.

### Prompt 2: Resource vs Tool Decision

```
I'm building a project management MCP server. For each of these scenarios,
help me decide: Resource or Tool?

1. User wants Claude Code to read the current project plan
2. User wants Claude Code to create a new task
3. User wants Claude Code to check the status of in-progress tasks
4. User wants Claude Code to update task descriptions
5. User wants Claude Code to reference a specific document in discussions
6. User wants Claude Code to generate a report from task data

For each, explain:
- Why resource or tool is appropriate
- What the API would look like
- How the user would trigger it
```

**What you're learning**: Understanding the fundamental difference between app-controlled (resources) and model-controlled (tools) primitives helps you design secure, intuitive agent systems.

### Prompt 3: Implement a FastMCP Resource

```
I want to build an MCP resource that exposes configuration data from YAML files.

The resource should:
- List all available .yaml files in a config directory (direct resource)
- Allow fetching specific config files by name (templated resource)
- Return proper MIME types (application/json or text/plain)
- Validate that only .yaml files are accessible (security)
- Handle missing files gracefully

Write Python code using FastMCP that implements both the direct
and templated resources. Include error handling and validation.
```

**What you're learning**: Implementation patterns for real resources that handle validation, security, and dynamic content—the skills you'll need to build production MCP servers.

### Safety Note

Resources provide read-only access, but be careful: If a resource exposes sensitive data (API keys, passwords, private documents), you've given Claude Code access to that data. Always validate which resources are exposed and consider access controls based on the context or user. Resources are pre-authorized—make sure you intended to authorize them.

