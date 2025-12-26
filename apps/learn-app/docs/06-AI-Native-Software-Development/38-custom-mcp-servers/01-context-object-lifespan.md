---
sidebar_position: 1
title: "Context Object & Server Lifespan"
description: "Master dependency injection patterns for MCP tools: Context objects for logging, progress notifications, session access, and async resource management in production servers"
keywords: ["MCP Context", "Dependency Injection", "Async Resource Management", "Server Lifecycle", "Progress Notifications", "Session Management", "FastMCP"]
chapter: 38
lesson: 1
duration_minutes: 16

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding MCP Context Dependency Injection"
    proficiency_level: "C2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain why Context is injected (not passed as argument) and how FastMCP wires dependencies automatically"

  - name: "Implementing Context-Aware Tool Functions"
    proficiency_level: "C2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write tool functions that use Context for logging (info/warning/error), progress reporting, and session access with proper type hints"

  - name: "Designing Async Context Managers for Server Resources"
    proficiency_level: "C2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement @mcp.server decorator with on_startup/on_shutdown for database connections, resource pools, and graceful shutdown"

  - name: "Integrating Session-Based Caching and Stateful Tool Behavior"
    proficiency_level: "C2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design session-aware tools that maintain cache across multiple tool invocations within single client connection"

learning_objectives:
  - objective: "Understand why Context is injected vs explicitly passed, and how this enables automatic wiring in framework patterns"
    proficiency_level: "C2"
    bloom_level: "Understand"
    assessment_method: "Explanation of dependency injection benefits in tool-based architecture"

  - objective: "Implement tools that use Context for logging, progress notifications, and structured output to clients"
    proficiency_level: "C2"
    bloom_level: "Apply"
    assessment_method: "Creation of tool with info/warning/error logging and report_progress() calls"

  - objective: "Design server lifecycle management using @mcp.server and async context managers for resource initialization and cleanup"
    proficiency_level: "C2"
    bloom_level: "Apply"
    assessment_method: "Implementation of on_startup/on_shutdown handlers with proper async/await patterns"

  - objective: "Build session-based tools that maintain state across multiple invocations, demonstrating context.session usage"
    proficiency_level: "C2"
    bloom_level: "Analyze"
    assessment_method: "Tool that uses session to cache results or maintain request-scoped state"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (dependency injection, Context typing, async context managers, logging, progress reporting, session lifecycle, resource management, graceful shutdown) within C2 limit (no artificial ceiling) ✓"

differentiation:
  extension_for_advanced: "Research lifecycle patterns in production systems; implement resource pooling with session-based connection management; design recovery strategies for interrupted operations"
  remedial_for_struggling: "Start with single Context usage (logging only); then add progress separately; practice async/await patterns independently before combining with Context"
---

# Context Object & Server Lifespan

In Chapter 37, you learned how decorators transform Python functions into MCP primitives. But basic decorators only get you so far. Production servers need **observability** (what's happening?), **feedback** (how's progress?), and **state management** (what am I tracking across requests?). That's where Context comes in.

Context isn't a parameter you pass. It's a **dependency the framework injects automatically**. This pattern—called **dependency injection**—is why FastMCP can wire up complex tools without requiring setup boilerplate.

Think of it like a restaurant kitchen: You don't manually tell each chef their tools, their timer, and communication channel. The restaurant infrastructure *provides* these automatically. Chefs focus on cooking, not logistics. Context works the same way: Your tools focus on domain logic, and FastMCP provides the infrastructure dependencies automatically.

## What Context Provides

Before you write code, understand what Context gives you:

| Capability | Purpose | Example |
|-----------|---------|---------|
| **logging** | Structured output to client + logs | `await context.info("Starting analysis...")` |
| **progress** | Real-time feedback for long operations | `await context.report_progress(25, 100)` |
| **session** | Access session ID and connection state | `context.session.session_id` |
| **create_message** | Call LLMs through connected client (Lesson 2) | `await context.session.create_message(...)` |

Context is **per-request**: Each tool invocation gets its own Context. Multiple tools running simultaneously don't interfere—each has isolated logging, progress, and session access.

## Dependency Injection: How Context Gets Into Your Tools

In traditional code, you'd pass Context as a parameter:

```python
# OLD PATTERN (NOT how MCP works):
def analyze(query: str, context: Context):  # ❌ Wrong
    await context.info("Starting...")
```

MCP uses **dependency injection** instead:

```python
# CORRECT: FastMCP injects Context automatically
@mcp.tool()
async def analyze(
    query: str = Field(description="What to analyze"),
    *,
    context: Context  # ← Special parameter (NOT in tool schema)
):
    await context.info("Starting analysis...")
```

**Critical detail**: The `context` parameter is **NOT in the tool schema**. The client never provides it. FastMCP intercepts tool invocation, creates a Context object, and passes it automatically.

### Why Dependency Injection?

1. **Clean tool schemas**: Clients see only real inputs (query, topic, etc.), not infrastructure params
2. **Automatic wiring**: No setup code needed; framework handles Context lifecycle
3. **Type safety**: Type hints (`: Context`) enable IDE autocomplete and validation
4. **Consistent patterns**: All tools access logging/progress the same way

## Logging and Progress: Observability for Long Operations

### Logging: Info, Warning, Error

```python
@mcp.tool(
    name="research",
    description="Research a complex topic"
)
async def research(
    topic: str = Field(description="Topic to research"),
    depth: int = Field(description="1-3, depth of research"),
    *,
    context: Context
):
    await context.info(f"Starting research on '{topic}' at depth {depth}")

    results = []
    try:
        sources = await fetch_sources(topic)
        await context.info(f"Found {len(sources)} sources")

        for i, source in enumerate(sources):
            content = await fetch_content(source)
            results.append(content)

            # Progress notification (covered next)
            await context.report_progress(i + 1, len(sources))

    except Exception as e:
        await context.error(f"Research failed: {str(e)}")
        raise

    await context.info(f"Research complete: {len(results)} sources processed")
    return {"sources": len(results), "data": results}
```

**What happens**:
- `context.info()` sends structured messages to the client
- Client displays these as real-time logs
- Messages persist in operation history
- Errors automatically trigger `context.error()` channel

**Key pattern**: Log BEFORE and AFTER significant operations. This helps clients (and you) understand what happened if something fails.

### Progress: Real-Time Feedback During Long Operations

For operations that take seconds or minutes, progress notifications prevent the client from timing out or thinking the server crashed.

```python
@mcp.tool(
    name="batch_analyze",
    description="Analyze multiple documents"
)
async def batch_analyze(
    document_ids: list[str] = Field(description="Document IDs to analyze"),
    *,
    context: Context
):
    total = len(document_ids)
    results = []

    for i, doc_id in enumerate(document_ids):
        analysis = await analyze_document(doc_id)
        results.append(analysis)

        # Progress: i documents done out of total
        await context.report_progress(i + 1, total)

    return {"analyzed": len(results), "results": results}
```

**Progress semantics**:
- `report_progress(completed, total)` → Percentage shown to client
- Typical use: Loop through items, increment progress each iteration
- No "progress(100)" needed; framework assumes tool completes when it returns

## Session: Per-Request State and LLM Integration

Every tool invocation gets a **session**—a connection handle for that specific request. Session lets you:

1. **Track request identity**: `context.session.session_id`
2. **Call LLMs back** (Lesson 2): `context.session.create_message()`
3. **Store request-scoped data**: Cache results for the duration of ONE request

### Session Basics: Understanding Request Isolation

```python
@mcp.tool(
    name="calculate",
    description="Perform calculation"
)
async def calculate(
    expression: str = Field(description="Math expression"),
    *,
    context: Context
):
    session_id = context.session.session_id
    await context.info(f"Session: {session_id}")

    # This session ID identifies this specific request
    # Different tool calls = different sessions
    result = eval_expression(expression)
    return {"result": result, "session": session_id}
```

**Important**: Session is request-scoped, not global. Two clients calling your tool simultaneously get different sessions. Each tool invocation within a session can see the same session ID.

## Server Lifecycle: on_startup and on_shutdown

Servers often need setup (connect to database, load credentials) and cleanup (close connections, flush buffers). That's what lifecycle handlers do.

### Basic Lifecycle Pattern

```python
mcp_server = fastmcp.FastMCP("research_server")

# Database connection (module-level)
db = None

@mcp_server.on_startup
async def startup():
    global db
    await context.info("Initializing database...")
    db = await asyncio.create_task(connect_to_postgres())
    await context.info("Database ready")

@mcp_server.on_shutdown
async def shutdown():
    global db
    if db:
        await context.info("Closing database...")
        await db.close()
        await context.info("Database closed")

@mcp.tool()
async def search(
    query: str = Field(description="Search query"),
    *,
    context: Context
):
    # db is available because startup() ran first
    results = await db.query(f"SELECT * FROM documents WHERE content ILIKE %{query}%")
    return {"results": results}

# In your main:
if __name__ == "__main__":
    asyncio.run(mcp_server.run(port=5000))
    # startup() runs before server accepts connections
    # shutdown() runs when server stops
```

**Lifecycle guarantees**:
1. `on_startup` runs BEFORE server accepts any tool calls
2. `on_shutdown` runs when server stops (Ctrl+C, deployment shutdown, etc.)
3. Resources initialized in startup are available to all tools
4. Cleanup in shutdown happens even if tools crash

### Advanced Pattern: Async Context Manager (Context Guard)

For sophisticated resource management, use Python's context manager pattern:

```python
class DatabasePool:
    def __init__(self, url: str, pool_size: int = 10):
        self.url = url
        self.pool_size = pool_size
        self.pool = None

    async def __aenter__(self):
        await context.info("Initializing connection pool...")
        self.pool = await create_pool(self.url, self.pool_size)
        return self.pool

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if self.pool:
            await context.info("Draining connection pool...")
            await self.pool.close()
        if exc_type:
            await context.error(f"Pool error: {exc_type.__name__}")

# Usage in server initialization:
mcp_server = fastmcp.FastMCP("advanced_server")
db_pool = None

@mcp_server.on_startup
async def startup():
    global db_pool
    async with DatabasePool("postgresql://localhost/mydb") as pool:
        db_pool = pool

@mcp.tool()
async def query(
    sql: str = Field(description="SQL query"),
    *,
    context: Context
):
    if not db_pool:
        raise RuntimeError("Database not initialized")

    async with db_pool.acquire() as conn:
        result = await conn.fetch(sql)
        await context.info(f"Query returned {len(result)} rows")
        return {"rows": result}
```

This pattern ensures resources are properly allocated and deallocated, even if tools crash.

## Putting It Together: A Production-Ready Tool

```python
import fastmcp
from fastmcp import Context
from pydantic import Field
import asyncio

mcp = fastmcp.FastMCP("document_processor")

# Server-level state (initialized on startup)
document_cache = {}

@mcp.on_startup
async def startup():
    # In production: Load cache from database, initialize ML models, etc.
    await asyncio.sleep(0.1)  # Simulate initialization

@mcp.on_shutdown
async def shutdown():
    # In production: Flush cache to database, clean up resources
    document_cache.clear()

@mcp.tool(
    name="process_document",
    description="Process a document and extract structured data"
)
async def process_document(
    document_id: str = Field(description="Document ID to process"),
    include_summary: bool = Field(
        default=True,
        description="Include AI summary of content"
    ),
    *,
    context: Context
):
    await context.info(f"Processing document {document_id}")

    # Check cache first
    if document_id in document_cache:
        await context.info("Found in cache, returning cached result")
        return document_cache[document_id]

    try:
        # Fetch document
        await context.report_progress(1, 3)
        doc = await fetch_document(document_id)
        await context.info(f"Fetched document: {len(doc['content'])} bytes")

        # Extract structure
        await context.report_progress(2, 3)
        extracted = await extract_structure(doc)
        await context.info(f"Extracted {len(extracted['sections'])} sections")

        # Optional: Summarize with AI
        if include_summary:
            await context.info("Generating AI summary...")
            summary = await context.session.create_message(
                model="claude-3-5-sonnet-20241022",
                max_tokens=300,
                messages=[{
                    "role": "user",
                    "content": f"Summarize this document:\n\n{doc['content'][:2000]}"
                }]
            )
            extracted["summary"] = summary.content[0].text

        await context.report_progress(3, 3)

        # Cache for future requests in this session
        document_cache[document_id] = extracted

        return extracted

    except Exception as e:
        await context.error(f"Failed to process document: {str(e)}")
        raise

if __name__ == "__main__":
    # Run with: mcp server run --name document_processor src/server.py
    mcp.run()
```

**What this demonstrates**:
- Startup initializes resources (cache)
- Each tool logs progress at key points
- Progress notifications for 3-step operation
- Session-based caching (results cached for duration of request)
- Optional LLM integration (Lesson 2 preview)
- Error handling with logging
- Shutdown cleanup

## Try With AI

**Setup**: Use Claude Code with the MCP Inspector to test Context usage. You'll implement a tool that uses logging, progress, and session.

**Prompt 1: Basic Context Implementation**

Ask Claude Code:

```
Create a FastMCP tool called "estimate_project" that:
- Takes project_name (string) and task_count (integer) as inputs
- Uses context.info() to log "Starting estimation for [project_name]"
- Uses context.report_progress() to loop through tasks
  - For each task: fetch estimate, increment progress
  - Report progress(current, task_count)
- Uses context.info() to log "Estimation complete"
- Returns dictionary with total_hours, tasks_processed

Show the complete implementation with proper type hints and Field descriptions.
```

**What you're learning**: How to wire Context as a parameter and use basic logging/progress patterns. Evaluate Claude's response:

- Does the context parameter come AFTER the `*` separator? (That's the Python pattern for keyword-only args)
- Are all Field descriptions present and clear?
- Does progress start at 1 and end at task_count?

**Prompt 2: Server Lifecycle**

Based on the tool from Prompt 1, ask:

```
Now add server lifecycle management:
- Create an @mcp.on_startup handler that:
  - Logs "Project estimation server starting"
  - Initializes a global dictionary called project_rates = {"small": 8, "medium": 20, "large": 40}

- Create an @mcp.on_shutdown handler that:
  - Logs "Project estimation server stopping"
  - Clears the project_rates dictionary

- Modify estimate_project to use the global project_rates for calculation
  - Log project_rates before calculation
  - Calculate total as: sum of (rate * 1 hour) for each task
```

**What you're learning**: Lifecycle patterns and how startup ensures resources exist before tools run. Evaluate:

- Does startup run before tools can be called? (No explicit guarantee in code, but that's the design)
- Is the shutdown handler cleanup reasonable?
- Does the tool correctly reference the global variable?

**Prompt 3: Session-Based State**

Ask:

```
You're implementing a "cache_aware_estimate" tool that:
- Caches project estimates in session for repeated queries
- First call: Calculates from scratch (slow)
- Second call: Returns cached result (instant)

Using context.session.session_id:
- On first call with project_name:
  - Calculate estimate (use context.report_progress)
  - Log "Caching for session {session_id}"
  - Store in session_cache[session_id] = result

- On repeated call with same project_name:
  - Check session_cache[session_id]
  - Log "Cache hit for session {session_id}"
  - Return immediately without recalculation

Show the implementation. Note: In real production, you'd use a proper session store.
What should session_cache be? (Hint: It needs to persist across tool calls but not across different sessions)
```

**What you're learning**: Why sessions exist (request isolation) and how to design stateful tools. Evaluate:

- What's Claude's answer to "what should session_cache be?"
  - Good: "A dict keyed by session_id" (session-scoped)
  - Less good: "A database table" (persists across sessions—violates design)
- Does the session_id lookup work correctly?
- Is the logging clear about cache hits vs misses?

---

✅ **Context Object & Server Lifespan enables production patterns.** You've seen how dependency injection provides logging, progress, session, and lifecycle management—all automatically. Next lesson: Using context.session.create_message() to have your server call LLMs back through connected clients.
