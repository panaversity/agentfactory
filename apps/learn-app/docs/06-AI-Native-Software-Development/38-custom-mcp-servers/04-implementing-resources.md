---
sidebar_position: 4
title: "Implementing Resources (@mcp.resource)"
description: "Expose data and configuration through MCP's resource abstraction with URI templates, streaming, and content negotiation"
keywords:
  - MCP resources
  - @mcp.resource decorator
  - URI templates
  - Static resources
  - Templated resources
  - MIME types
  - Resource vs Tool decision
chapter: 38
lesson: 4
duration_minutes: 55

skills:
  - name: "MCP Resource Implementation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital literacy"
    measurable_at_this_level: "Students implement both static and templated resources with proper URI schemes and MIME types"

learning_objectives:
  - objective: "Understand when to use resources vs tools in MCP server design"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Decision framework application in Try With AI"
  - objective: "Implement static resources with fixed URIs"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Code implementation with MIME type specification"
  - objective: "Create templated resources with URI parameter extraction"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Working implementation with dynamic parameter handling"
  - objective: "Design resource URI schemes that communicate intent"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "URI design rationale in AI collaboration"

cognitive_load:
  new_concepts: 7
  assessment: "B1 tier - moderate scaffolding with decision frameworks, 7 concepts within working memory"

differentiation:
  extension_for_advanced: "Combine multiple resources with streaming content, implement resource caching strategies, design hierarchical resource namespaces"
  remedial_for_struggling: "Focus on static resources first, use explicit examples of URI structure before introducing parameters"
---

# Implementing Resources (@mcp.resource)

In Lesson 2, you learned that tools are for actions—operations that change state or compute results. Tools are how you **teach AI to do things**.

Resources are the opposite. Resources are how you **give AI information**—static or semi-static data that AI consumes without modification. Configuration settings, documentation, templates, policy files, historical data. Any information that AI reads but doesn't write.

This distinction matters because it changes how you structure your server. Tools require validation, error handling, and complex parameter definitions. Resources are simpler: they just expose data through URIs.

This lesson teaches you to implement resources using FastMCP's `@mcp.resource` decorator. You'll see when resources fit better than tools, how to design URI schemes that communicate intent, and how parameter extraction works.

## Understanding Resources vs Tools

Let's make the distinction concrete. Suppose you're building an MCP server for a project management system. You have two capabilities:

**Creating a task** (Tool)
- Parameter: task description, assigned to, priority, due date
- Side effect: task gets created, stored in database
- Return: confirmation, new task ID
- Validation: Description required, priority is enum, date must be future
- Error handling: Assignee invalid, description too long, database unavailable

**Reading a task** (Resource)
- Parameter: task ID (from URI)
- Side effect: none
- Return: task data (title, description, status, assigned to, due date)
- Validation: task must exist
- Error handling: task not found

The **Tool** requires validation logic, business rules, and error states. The **Resource** just fetches and returns data.

### Decision Framework: When to Use Resources

Ask yourself: "Is the client reading information or requesting an action?"

**Use Resources when:**
- Client needs to READ data without modification
- Data is static or semi-static (config, docs, settings)
- Access follows a predictable pattern (by ID, by name, by date)
- No complex business logic required
- You'd return the same data regardless of who's asking

**Use Tools when:**
- Operation changes state (create, update, delete)
- Complex validation or business rules apply
- Multiple parameters combine to determine behavior
- Error conditions require special handling
- Return value depends on context beyond the request parameters

**Real example**: A documentation server could use Resources for docs:

```python
@mcp.resource("docs://api/{version}/endpoints/{endpoint_name}")
async def get_endpoint_docs(version: str, endpoint_name: str) -> str:
    '''Return API documentation for specific endpoint.'''
    # Just read and return documentation
    return load_endpoint_docs(version, endpoint_name)
```

This works because:
- No modification happens
- Documentation is static or updated separately
- Access pattern is predictable (version → endpoint)
- Same docs returned for everyone

## Implementing Static Resources

A static resource has a fixed URI with no parameters. It represents a single piece of data.

### Example: Server Configuration as Resource

Imagine your MCP server is configured with settings—API keys, rate limits, debug mode. Instead of hardcoding these, expose them as a resource:

```python
from mcp.server.fastmcp import FastMCP
import json

mcp = FastMCP("project_mcp")

# In-memory configuration (in production, load from file or environment)
SERVER_CONFIG = {
    "version": "1.0.0",
    "debug": False,
    "rate_limit": 100,
    "max_payload_kb": 5000,
    "api_timeout_seconds": 30
}

@mcp.resource("config://server-settings")
async def get_server_config() -> str:
    '''Expose server configuration to AI clients.

    This resource provides read-only access to server settings including
    version, debug mode, rate limits, and timeout configurations. Used
    by clients to understand server capabilities and limitations.

    Returns:
        str: JSON-formatted configuration object
    '''
    return json.dumps(SERVER_CONFIG, indent=2)
```

**What's happening:**

1. `@mcp.resource("config://server-settings")` registers a resource at fixed URI
2. The URI has no parameters—it always points to the same data
3. The function takes no parameters (besides implicit context)
4. Return type is `str` (or can be JSON serializable)
5. No validation needed—configuration is always valid

**MIME type awareness**: By default, resources return `text/plain`. If you want to declare it's JSON, you can add MIME type information via the resource response format (handled automatically when returning JSON strings).

### Example: README as Resource

Another static use case—expose documentation:

```python
@mcp.resource("help://readme")
async def get_readme() -> str:
    '''Return markdown README explaining server capabilities.

    Clients use this to understand what operations are available,
    common workflows, and troubleshooting information.

    Returns:
        str: Markdown-formatted README
    '''
    with open("README.md", "r") as f:
        return f.read()
```

**Important**: The file is read at request time, not at startup. This means updates to `README.md` are reflected immediately without restarting the server.

## Implementing Templated Resources

A templated resource has **parameters in the URI** that extract values dynamically.

The syntax uses curly braces: `scheme://path/{parameter_name}`

Each parameter becomes a function argument.

### Example: Task by ID

```python
@mcp.resource("tasks://project/{project_id}/task/{task_id}")
async def get_task(project_id: str, task_id: str) -> str:
    '''Retrieve a specific task by project and task ID.

    Allows AI clients to access individual task details without
    listing all tasks. Useful when AI needs specific task data
    based on prior context.

    Args:
        project_id: The project containing the task (e.g., "proj-123")
        task_id: The specific task to retrieve (e.g., "task-456")

    Returns:
        str: JSON-formatted task details including title, description,
             status, assigned_to, due_date, and labels

    Raises:
        ValueError: If project or task doesn't exist
    '''
    # Fetch task from database or API
    task = await fetch_task(project_id, task_id)

    if not task:
        raise ValueError(f"Task {task_id} not found in project {project_id}")

    return json.dumps({
        "id": task["id"],
        "title": task["title"],
        "description": task["description"],
        "status": task["status"],
        "assigned_to": task["assigned_to"],
        "due_date": task["due_date"],
        "labels": task.get("labels", [])
    })
```

**Parameter extraction:**
- URI: `tasks://project/proj-123/task/task-456`
- `project_id` parameter extracts "proj-123"
- `task_id` parameter extracts "task-456"
- Function receives both as string arguments
- Function can validate/transform before using them

### Parameters Have Types (But Are Always Extracted as Strings)

When you define resource parameters in the function signature, use type hints. FastMCP validates the types:

```python
@mcp.resource("documents://doc/{doc_id}/version/{version_number}")
async def get_document_version(doc_id: str, version_number: int) -> str:
    '''Get a specific version of a document.

    URI parameters are extracted as strings from the URL, but
    FastMCP will attempt to convert them to declared types.

    Args:
        doc_id: Document identifier (extracted as string)
        version_number: Version number (extracted as int if possible)

    Returns:
        str: JSON-formatted document content with metadata
    '''
    # version_number will be int (or error if non-numeric)
    doc = await load_document(doc_id, version_number)

    return json.dumps({
        "id": doc_id,
        "version": version_number,
        "content": doc["content"],
        "created_at": doc["created_at"],
        "author": doc["author"]
    })
```

**Type coercion:**
- `str` parameters accept any value
- `int` parameters must be convertible to integers
- `bool` parameters expect "true"/"false"
- Invalid types result in 400 errors from FastMCP

## Designing URI Schemes

The URI structure communicates intent to AI clients. A well-designed scheme makes clear what data you're exposing and how to access it.

### Scheme Pattern: `scheme://namespace/path/{parameters}`

**Scheme** (before `://`)
- Describes the data category: `config`, `docs`, `tasks`, `files`, `settings`
- Helps AI client understand the domain
- Common patterns: `api`, `data`, `docs`, `config`

**Namespace** (after `://`, before first `/`)
- Organizes multiple resource types
- Optional—can go directly to path
- Examples: `api`, `v1`, `production`

**Path** (remaining parts before `{`)
- Hierarchical structure showing relationships
- Examples: `/project/{id}/task` shows tasks belong to projects

**Parameters** (in `{curly braces}`)
- Variable parts of the URI
- Usually entity IDs or filters
- Should have meaningful names (not just `id`, use `project_id`, `user_id`)

### Good URI Examples

```
config://database-connection          # Static config
docs://api/v1/{endpoint_name}         # API docs by endpoint
files://workspace/{workspace_id}/{file_path}  # Files in workspace
templates://email/{template_name}/language/{lang_code}  # Multilingual templates
logs://service/{service_name}/date/{date}  # Logs by service and date
```

### Anti-Patterns to Avoid

```
# Too vague
data://resource/{id}                  # What is "data"? What resource?

# Exposing implementation details
content://database-row/{table_name}/{pk}  # Clients shouldn't know table structure

# Unclear hierarchy
docs://endpoints-api-v1-get/{name}    # Hierarchy should be in path, not scheme
```

## Error Handling in Resources

Resources should validate parameters and handle missing data gracefully.

### Parameter Validation

```python
@mcp.resource("reports://generated/{report_id}")
async def get_report(report_id: str) -> str:
    '''Retrieve a generated report by ID.

    Args:
        report_id: Must be alphanumeric, 8-20 characters

    Returns:
        str: JSON report data

    Raises:
        ValueError: If report_id is invalid format or report doesn't exist
    '''
    # Validate format before querying
    if not (8 <= len(report_id) <= 20 and report_id.isalnum()):
        raise ValueError(f"Invalid report_id format: {report_id}")

    report = await database.get_report(report_id)

    if not report:
        raise ValueError(f"Report not found: {report_id}")

    return json.dumps(report)
```

**Error behavior:**
- Raise `ValueError` with descriptive message when data not found
- FastMCP converts exceptions to proper MCP error responses
- Message is returned to client explaining the issue

### Handling Missing Data

```python
@mcp.resource("users://profile/{user_id}")
async def get_user_profile(user_id: str) -> str:
    '''Get user profile information.'''
    user = await fetch_user(user_id)

    # Option 1: Return partial data if some fields missing
    if user:
        # Return what we have
        profile = {
            "id": user["id"],
            "email": user["email"],
            "name": user.get("name"),  # May be None
            "avatar": user.get("avatar")  # May be None
        }
        return json.dumps(profile)

    # Option 2: Raise error if required data missing
    raise ValueError(f"User profile not found: {user_id}")
```

## Resource vs Tool: Working Example

Let's see a complete scenario showing when to use Resources and when to use Tools.

**Scenario**: Build an AI-powered support system. Customers ask questions, AI consults knowledge base and company policies.

**What should be a Resource?**
- Company policy documents (static, read-only)
- Knowledge base articles (updated separately, read-only)
- Support ticket templates (static forms)
- Service status dashboard (read-only current state)

**What should be a Tool?**
- Create a support ticket (modifies state)
- Update ticket status (modifies state)
- Search knowledge base (complex query logic)
- Send email response (action with side effects)

**Implementation:**

```python
from mcp.server.fastmcp import FastMCP
import json

mcp = FastMCP("support_mcp")

# ============ RESOURCES (Read-Only Data) ============

@mcp.resource("policies://refund-policy")
async def get_refund_policy() -> str:
    '''Company refund policy document.

    Static resource—updated only when policy changes, which happens
    outside the MCP server lifecycle.
    '''
    return """
# Refund Policy

1. **30-Day Money Back Guarantee**: Full refund within 30 days of purchase
2. **No Questions Asked**: Refunds processed without justification
3. **Automatic Processing**: Refunds processed within 5 business days
4. **Original Payment Method**: Refunds return to original payment method
    """

@mcp.resource("kb://articles/{article_id}")
async def get_kb_article(article_id: str) -> str:
    '''Knowledge base article by ID.

    Resource because AI reads articles to answer questions.
    Articles are updated through separate content management system,
    not through MCP.
    '''
    article = await kb_store.get_article(article_id)

    if not article:
        raise ValueError(f"Article not found: {article_id}")

    return json.dumps({
        "id": article["id"],
        "title": article["title"],
        "content": article["content"],
        "updated_at": article["updated_at"],
        "category": article["category"]
    })

@mcp.resource("status://services/{service_name}")
async def get_service_status(service_name: str) -> str:
    '''Current status of a service.

    Resource because it's read-only current state that changes
    independently of user requests. (Status is updated by monitoring
    system, not through MCP operations.)
    '''
    status = await monitoring.get_status(service_name)

    return json.dumps({
        "service": service_name,
        "status": status["current"],  # "operational", "degraded", "down"
        "last_updated": status["timestamp"],
        "incident": status.get("incident_description")
    })

# ============ TOOLS (Actions with Validation & Side Effects) ============

@mcp.tool(name="create_support_ticket")
async def create_ticket(customer_email: str, subject: str, description: str) -> str:
    '''Create a support ticket.

    Tool because:
    1. Creates new entity in database (side effect)
    2. Requires validation (email format, subject length, etc.)
    3. Complex business logic (assign to queue, notify team)
    4. Returns identifier for future reference
    '''
    # Validation
    if not "@" in customer_email:
        raise ValueError("Invalid email address")
    if len(subject) < 5:
        raise ValueError("Subject must be at least 5 characters")

    # Create ticket (side effect)
    ticket_id = await ticketing_system.create_ticket(
        email=customer_email,
        subject=subject,
        description=description
    )

    return json.dumps({
        "status": "created",
        "ticket_id": ticket_id,
        "message": f"Support ticket {ticket_id} created. We'll respond within 24 hours."
    })

@mcp.tool(name="search_knowledge_base")
async def search_kb(query: str, limit: int = 5) -> str:
    '''Search knowledge base by keywords.

    Tool because:
    1. Complex search logic (ranking, filtering, relevance)
    2. Parameter combination determines behavior (query + limit affects results)
    3. Could fail in various ways (database error, empty results, invalid limit)
    4. Business logic (what fields to search, result ranking)

    Note: We expose articles as Resources (@mcp.resource),
    but expose search as Tool because search requires logic.
    '''
    if not query.strip():
        raise ValueError("Search query cannot be empty")
    if limit < 1 or limit > 50:
        raise ValueError("Limit must be between 1 and 50")

    results = await kb_store.search(query, limit=limit)

    return json.dumps({
        "query": query,
        "total_found": len(results),
        "articles": [
            {
                "id": r["id"],
                "title": r["title"],
                "relevance_score": r["score"]
            }
            for r in results
        ]
    })
```

**Notice the pattern:**
- **Resources**: `get_refund_policy`, `get_kb_article`, `get_service_status` — all read static/semi-static data
- **Tools**: `create_ticket`, `search_kb` — both perform operations with business logic

## Try With AI: Designing Resources for Your Domain

In this exercise, you'll work with AI to design resources for a specific domain. The collaboration will show how specification (clarifying your intent) leads to better design.

### Part 1: Initial Request

Ask AI to suggest resource designs for your domain:

```
I'm building an MCP server for [YOUR DOMAIN: e.g., "expense management", "email marketing", "project analytics"].

The server needs to expose:
1. [First type of data users need to read]
2. [Second type of data users need to read]
3. [Third type of data users need to read]

For each, suggest:
- Whether it should be a Resource or Tool
- The URI scheme and structure
- What parameters make sense
- Example URIs with real values

Be specific about why each should be a Resource vs Tool.
```

**Copy this prompt exactly as shown and submit to AI.**

### Part 2: Evaluating the Response

Review AI's suggestions. Ask yourself:

- Does each Resource represent read-only data?
- Are the URI schemes clear and hierarchical?
- Do parameter names communicate what they represent?
- Would you be able to explain each URI to a colleague?

If any suggestion seems unclear, ask AI for clarification.

### Part 3: Refinement Through Constraint

AI likely suggested a generic design. Now add your specific constraints:

```
Looking at these resource designs, I want to add constraints:

1. [Constraint about your domain: e.g., "all resources must be scoped by organization_id because we're multi-tenant"]
2. [Constraint about access: e.g., "some data should be rate-limited or require authentication"]
3. [Constraint about format: e.g., "responses must always include metadata about when data was last updated"]

How would you revise the resource designs given these constraints?

Specifically, show me:
- Revised URI schemes
- Changes to parameters
- Where additional validation would be needed
```

**This is where your domain expertise teaches AI.** You're showing constraints that exist in your specific context.

### Part 4: Final Design Proposal

Ask AI for a complete implementation skeleton:

```
Based on our refined design, generate Python code showing:

1. A static resource (one of the simpler ones)
2. A templated resource with one parameter
3. A templated resource with multiple parameters

For each, include:
- The @mcp.resource decorator with proper URI
- Function signature with parameter extraction
- Basic docstring
- Simple JSON return format

Don't implement the actual data fetching logic yet—just the structure.
```

### Part 5: Reflection

Compare your final design to the initial one. Ask yourself:

- What changed between initial and final design?
- What constraints from your domain shaped the design?
- How would an AI client understand your resource URIs?
- Which resources serve the most important use cases?

**What emerged**: Your initial request was generic ("expose data"). Your constraints made it specific ("expose data in a multi-tenant context with security"). The iteration shows how specification-driven design works—you move from generic to specific through constraints and refinement.

---

**Validation**: Resources implemented with @mcp.resource are the foundation of data exposure in MCP servers. Combined with Tools from Lessons 2-3, you now have the two primary building blocks: actions (Tools) and information (Resources). Lesson 5 will show Prompts—the third abstraction layer that encodes expertise into reusable guidance that AI applies.
