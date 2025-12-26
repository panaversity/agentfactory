---
sidebar_position: 8
title: "MCP Integration for External Tool Ecosystems"
description: "Connect agents to remote MCP servers using MCPServerStreamableHttp. Enable agents to access external documentation, APIs, and tool ecosystems. Pattern: TaskManager agent retrieving library documentation from Context7 MCP."
keywords: ["MCP", "Model Context Protocol", "MCPServerStreamableHttp", "tool ecosystems", "remote servers", "Context7", "documentation lookup", "async context managers", "mcp_servers", "resolve-library-id", "get-library-docs"]
chapter: 34
lesson: 8
duration_minutes: 90

# HIDDEN SKILLS METADATA
skills:
  - name: "MCPServerStreamableHttp Connection Management"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "System Integration"
    measurable_at_this_level: "Student can connect agents to remote MCP servers using async context managers and configure headers/authentication"

  - name: "MCP Tool Ecosystem Integration"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "System Design"
    measurable_at_this_level: "Student can leverage MCP tools (resolve-library-id, get-library-docs) within agent workflows"

  - name: "External Tool Composition for Agents"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can configure mcp_servers=[server] parameter and agents automatically receive MCP tools"

  - name: "Context7 Documentation Lookup"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Information Retrieval"
    measurable_at_this_level: "Student can use Context7 MCP to fetch up-to-date version-specific documentation for libraries"

  - name: "Async Context Manager Patterns for Resources"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can use `async with` patterns to manage MCP server lifecycle safely"

  - name: "Agent-Driven Tool Discovery and Composition"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "System Design"
    measurable_at_this_level: "Student can design agents that discover and use tools from multiple MCP servers"

learning_objectives:
  - objective: "Connect agents to remote MCP servers using MCPServerStreamableHttp with authentication"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Implement MCPServerStreamableHttp connection with proper async context manager"

  - objective: "Enable agents to automatically access MCP tools via mcp_servers parameter"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create agent that lists and uses tools from connected MCP server"

  - objective: "Integrate Context7 for dynamic documentation lookup in agent workflows"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Build agent that uses resolve-library-id and get-library-docs tools"

  - objective: "Design practical workflows that leverage external tool ecosystems"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Implement TaskManager agent that retrieves library documentation on demand"

cognitive_load:
  new_concepts: 6
  assessment: "6 core concepts (MCPServerStreamableHttp, async context managers, mcp_servers parameter, MCP tool discovery, Context7 integration, multi-server composition) appropriate for B2 proficiency with 90-minute duration. Builds on async/await knowledge from Part 5 and agent fundamentals from Lessons 1-7."

differentiation:
  extension_for_advanced: "Connect multiple MCP servers simultaneously; implement custom tool filtering; cache tool metadata; design multi-language documentation lookup system; integrate with private MCP servers requiring OAuth"
  remedial_for_struggling: "Start with simple MCPServerStreamableHttp connection without authentication; practice tool listing first before calling tools; defer Context7 integration until basic MCP patterns comfortable; use synchronous mock servers for testing"

generated_by: content-implementer
source_spec: specs/047-ch34-openai-agents-sdk
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# MCP Integration for External Tool Ecosystems

**Scenario**: Your TaskManager agent helps developers plan projects. When a developer asks "What parameters does the FastAPI `Depends` function accept?", your agent needs to fetch current documentation instantly. It's not enough to reason from training data—the API might have changed since the model was trained.

This is where the Model Context Protocol (MCP) becomes critical. MCP servers provide standardized access to external tools and data sources. Your agent can connect to these servers, discover available tools, and use them seamlessly within its workflow.

The OpenAI Agents SDK enables this through `MCPServerStreamableHttp`—a mechanism for connecting agents to remote MCP tool ecosystems. When configured, agents automatically gain access to all tools exported by the MCP server.

## Understanding MCP in the Agent Context

**The Core Problem**: Agents need to interact with external systems—documentation, databases, APIs—but each integration was custom code. MCP solves this by defining a universal protocol.

**Model Context Protocol** is a standard that defines:

1. **Tools**: Functions the MCP server exposes (e.g., "search documentation", "look up API reference")
2. **Prompts**: Templates for common queries the server handles
3. **Resources**: Data the server can access (e.g., knowledge bases, file systems)

An agent connected to an MCP server automatically gets access to its tools. The server handles the complexity of authentication, data retrieval, and formatting. The agent just calls the tools like any other function.

### Why This Matters for Production Agents

**Without MCP**: You hardcode integrations
```python
# Every API integration requires custom code
result = requests.get("https://api.github.com/repos/openai/...")
parsed = parse_github_response(result)
```

**With MCP**: Integration becomes configuration
```python
async with MCPServerStreamableHttp(url="https://mcp.github.com/") as server:
    agent = Agent(mcp_servers=[server])
    # Agent now has access to all GitHub tools automatically
```

This shifts from custom integration code to agent instruction. The agent decides *when* to use which tool based on the request.

## MCPServerStreamableHttp: Connecting to Remote Servers

`MCPServerStreamableHttp` establishes a connection to a remote MCP server over HTTP with streaming support. It's designed for servers you don't control—external APIs, third-party services, or remote documentation systems.

### Basic Connection Pattern

The fundamental pattern uses Python's async context manager to manage the server connection:

```python
from agents import Agent, Runner
from agents.mcp import MCPServerStreamableHttp
import asyncio
import os

async def main():
    # Connect to remote MCP server
    async with MCPServerStreamableHttp(
        name="GitHub MCP",
        url="https://mcp.example.com/mcp",
        headers={"Authorization": f"Bearer {os.getenv('MCP_API_KEY')}"}
    ) as server:
        # Create agent with MCP tools
        agent = Agent(
            name="Documentation Assistant",
            instructions="Help developers find information.",
            mcp_servers=[server]
        )

        # Agent can now call tools from this MCP server
        result = await Runner.run(agent, "Find the documentation for FastAPI")
        print(result.final_output)

# Run async function
asyncio.run(main())
```

**Output:**
```
Found documentation for FastAPI:
FastAPI is a modern, fast web framework for building APIs with Python...
[Additional context retrieved from MCP server]
```

Key points:
- `async with` manages connection lifecycle (opens on enter, closes on exit)
- Headers provide authentication (API keys, bearer tokens)
- `mcp_servers=[server]` parameter gives agent access to tools
- Tools are discovered automatically—no manual registration needed

### Understanding Async Context Managers

If async/await is new to you, the pattern is:

```python
# RESOURCE MANAGEMENT PATTERN
async with resource_manager() as resource:
    # Use resource (connected, initialized, ready)
    result = await resource.do_something()
# Resource automatically cleaned up (connection closed, memory freed)
```

For MCP servers:
- **Enter** (`async with`): Establish HTTP connection to remote server, authenticate, discover available tools
- **Use**: Agent calls tools provided by the server
- **Exit** (end of `with` block): Close connection, clean up resources

This prevents connection leaks where servers pile up without being closed.

## Practical Example: Context7 for Documentation Lookup

Context7 is a real-world MCP server (maintained by Upstash) that provides up-to-date documentation for thousands of libraries. Instead of agents relying on stale training data, they can fetch current documentation for exact library versions.

Context7 exposes two primary tools:

1. **resolve-library-id**: Given a package name, returns the Context7 library ID
   - Input: "fastapi", "react", "tensorflow"
   - Output: "/upstash/fastapi/0.104.1", "/upstash/react/18.2.0"

2. **get-library-docs**: Given a library ID and topic, returns current documentation
   - Input: library_id="/upstash/fastapi/0.104.1", topic="dependency_injection"
   - Output: Full documentation for that topic with examples

### Connecting to Context7

```python
from agents import Agent, Runner
from agents.mcp import MCPServerStreamableHttp
import asyncio
import os

async def main():
    # Context7 is a public MCP server at this URL
    # Get your API key from context7.com/dashboard
    context7_api_key = os.getenv("CONTEXT7_API_KEY")

    async with MCPServerStreamableHttp(
        name="Context7 Documentation",
        url="https://mcp.context7.com/mcp",
        headers={"Authorization": f"Bearer {context7_api_key}"}
    ) as context7_server:
        agent = Agent(
            name="API Reference Assistant",
            instructions="""You help developers understand API documentation.
When asked about a library or framework, use the Context7 documentation tools:
1. First call resolve-library-id to find the exact library
2. Then call get-library-docs to fetch current documentation
3. Provide the documentation with examples""",
            mcp_servers=[context7_server]
        )

        # Ask about a library
        result = await Runner.run(
            agent,
            "What's the signature for FastAPI's Depends function?"
        )
        print(result.final_output)

asyncio.run(main())
```

**Output:**
```
The FastAPI Depends function signature:

def Depends(
    dependency: Callable[..., Any],
    *,
    use_cache: bool = True,
) -> Any:

Used for dependency injection in FastAPI. When you use Depends() in a path
operation function parameter, FastAPI will execute the dependency function
and pass its return value to your operation.

Example:
async def get_current_user(token: str = Depends(oauth2_scheme)) -> User:
    return verify_token(token)

@app.get("/users/me")
async def read_user(current_user: User = Depends(get_current_user)):
    return current_user
```

The agent called:
1. `resolve-library-id("fastapi")` → Got "/upstash/fastapi/0.104.1"
2. `get-library-docs("/upstash/fastapi/0.104.1", topic="dependency_injection")` → Retrieved current docs
3. Synthesized the response with examples from the documentation

## Multiple MCP Servers: Composing Tool Ecosystems

Agents can connect to multiple MCP servers simultaneously. Each contributes tools that the agent can leverage:

```python
from agents import Agent, Runner
from agents.mcp import MCPServerStreamableHttp
import asyncio
import os

async def main():
    # Server 1: Documentation lookup
    async with MCPServerStreamableHttp(
        name="Context7",
        url="https://mcp.context7.com/mcp",
        headers={"Authorization": f"Bearer {os.getenv('CONTEXT7_API_KEY')}"}
    ) as context7_server:

        # Server 2: GitHub API access
        async with MCPServerStreamableHttp(
            name="GitHub",
            url="https://mcp.example.com/github-mcp",
            headers={"Authorization": f"Bearer {os.getenv('GITHUB_MCP_KEY')}"}
        ) as github_server:

            agent = Agent(
                name="Full-Stack Developer Assistant",
                instructions="""Help developers with code questions using:
1. Context7 for API documentation
2. GitHub for code examples and repository information
Choose the right tool based on what you need to find.""",
                mcp_servers=[context7_server, github_server]  # Both servers!
            )

            result = await Runner.run(
                agent,
                "Find the React hooks documentation and show me a GitHub example"
            )
            print(result.final_output)

asyncio.run(main())
```

**What happens inside**:
1. Agent receives the request
2. Agent considers available tools from BOTH MCP servers
3. Calls Context7 tools for React hooks documentation
4. Calls GitHub tools for code examples
5. Synthesizes both sources into a comprehensive response

This is the power of MCP: compose tool ecosystems without hardcoding integrations.

## TaskManager with Documentation Integration

Let's build a complete TaskManager that uses Context7 to help developers understand library documentation when planning projects:

```python
from agents import Agent, Runner, RunHooks, RunContextWrapper
from agents.mcp import MCPServerStreamableHttp
from pydantic import BaseModel
import asyncio
import os
from datetime import datetime

class TaskContext(BaseModel):
    user_id: str | None = None
    project_name: str | None = None
    current_library: str | None = None
    documentation_queries: list[str] = []

class TaskManagerWithDocs(RunHooks):
    """Tracks when agent uses documentation tools"""
    async def on_tool_start(
        self,
        context: RunContextWrapper,
        agent: Agent,
        tool
    ):
        tool_name = getattr(tool, 'name', str(tool))
        print(f"📚 Using tool: {tool_name}")

    async def on_agent_end(
        self,
        context: RunContextWrapper,
        agent: Agent,
        output: str
    ):
        if context.usage:
            print(f"✅ Completed with {context.usage.total_tokens} tokens")

async def run_task_manager_with_docs():
    """TaskManager that integrates documentation lookup"""

    context7_api_key = os.getenv("CONTEXT7_API_KEY")

    # Set up Context7 connection
    async with MCPServerStreamableHttp(
        name="Context7 Documentation",
        url="https://mcp.context7.com/mcp",
        headers={"Authorization": f"Bearer {context7_api_key}"}
    ) as context7_server:

        # Create TaskManager agent with documentation access
        task_manager = Agent(
            name="Technical Project Manager",
            instructions="""You help developers plan projects using current library documentation.

When developers mention a library:
1. Use resolve-library-id to find the exact library and version
2. Use get-library-docs to fetch current documentation
3. Explain relevant features and constraints based on actual documentation
4. Help plan tasks that account for the library's capabilities

Always ground recommendations in actual documentation, not assumptions.""",
            mcp_servers=[context7_server]
        )

        # Track usage
        hooks = TaskManagerWithDocs()

        # Create task context
        task_ctx = TaskContext(
            user_id="user-123",
            project_name="FastAPI E-commerce API",
            current_library="fastapi"
        )

        # Run with documentation integration
        result = await Runner.run(
            task_manager,
            """I'm building an e-commerce API with FastAPI.
            What dependency injection capabilities does FastAPI offer?
            How would I structure authentication with Depends?""",
            context=task_ctx,
            hooks=hooks
        )

        print("\n=== TASK MANAGER RESPONSE ===")
        print(result.final_output)
        print(f"\nDocumentation retrieved for: {task_ctx.project_name}")

# Run the example
asyncio.run(run_task_manager_with_docs())
```

**Output:**
```
📚 Using tool: resolve-library-id
📚 Using tool: get-library-docs
✅ Completed with 1247 tokens

=== TASK MANAGER RESPONSE ===

FastAPI provides powerful dependency injection through the `Depends()` function:

**How Dependency Injection Works:**
- Each parameter in a path operation can declare dependencies
- FastAPI executes dependencies and injects results
- Perfect for authentication, database access, validation

**For Authentication:**
1. Create a dependency function that extracts and validates credentials
2. Use `Depends(dependency_function)` in your path operation
3. FastAPI runs the dependency first, then passes result to your operation

Example authentication structure:
```python
async def verify_auth(token: str = Header(...)):
    user = await validate_token(token)
    return user

@app.post("/order")
async def create_order(current_user: User = Depends(verify_auth)):
    return {"order": "created", "user": current_user.id}
```

**For Your E-commerce API:**
Task suggestions based on FastAPI's capabilities:
1. Implement role-based access with dependency chain
2. Use dependencies for database session management
3. Create reusable dependency for product availability checks
4. Build authentication that leverages FastAPI's built-in validation

This approach ensures your architecture matches FastAPI's design patterns.
```

The agent automatically:
- Called `resolve-library-id("fastapi")` → Found correct version
- Called `get-library-docs` with relevant topics → Got current documentation
- Synthesized documentation with practical architectural advice
- Grounded all recommendations in actual library capabilities

## Designing Agents for Tool Discovery

When working with MCP servers, agents work best when you:

1. **Explain the available tools clearly**
   ```python
   instructions="""You have access to:
   - resolve-library-id: Find exact library IDs
   - get-library-docs: Fetch current documentation
   Use these to answer questions about libraries."""
   ```

2. **Let agents discover tools naturally**
   - Don't hardcode "call this tool first then that"
   - Let the agent reason about what it needs
   - Trust the agent to use tools effectively

3. **Provide context about tool limitations**
   ```python
   instructions="""Context7 documentation is accurate but limited to 5000 tokens.
   For deep dives, combine results and ask clarifying questions."""
   ```

## Comparison: When to Use MCP vs Custom Tools

| Scenario | MCP Server | Custom Tool | Tradeoff |
|----------|-----------|------------|----------|
| **Third-party service** (GitHub, Slack) | Best | Manual integration | MCP: standard interface; Custom: full control |
| **Internal proprietary API** | Not suitable | Required | MCP: open standard; Custom: proprietary |
| **Documentation lookup** | Excellent (Context7) | Slow to maintain | MCP: always current; Custom: outdated over time |
| **Custom business logic** | Not suitable | Required | MCP: external only; Custom: implement anything |
| **Quick prototyping** | Fast | Slower | MCP: one connection; Custom: more code |

For production agents, MCP servers handle external integrations. Custom tools handle internal logic.

## Try With AI

### Setup

You'll build a TaskManager agent that retrieves library documentation from Context7 MCP.

**Prerequisites**:
- Working knowledge of agents from previous lessons
- Context7 API key (free at context7.com/dashboard)

**Required imports**:
```python
from agents import Agent, Runner
from agents.mcp import MCPServerStreamableHttp
import asyncio
import os
```

### Prompt 1: Establish Your First MCP Connection

**What you're learning**: How to connect agents to remote MCP servers and manage lifecycle.

Ask AI:
```
I want to connect my agent to the Context7 MCP server for documentation lookup.

Build an async function that:
1. Creates MCPServerStreamableHttp for Context7 at https://mcp.context7.com/mcp
2. Uses an Authorization header with a Bearer token from CONTEXT7_API_KEY environment variable
3. Creates an agent with this MCP server in mcp_servers=[]
4. Runs the agent asking "What libraries are available in Context7?"
5. Prints the agent's response

Use async context manager pattern: async with MCPServerStreamableHttp(...) as server:
```

Review the response:

- Does it use `async with` for connection management?
- Does it pass `mcp_servers=[server]` to the Agent?
- Can you identify where authentication headers are configured?
- What would happen if the `with` block ended before agent execution?

### Prompt 2: Discover and Use Context7 Tools

**What you're learning**: How agents automatically access tools from connected MCP servers.

Ask AI:
```
Build on the previous example. Now I want the agent to:
1. Use resolve-library-id to find "fastapi" in Context7
2. Use get-library-docs to fetch documentation for FastAPI
3. Explain the documentation to the user

Update the agent's instructions to:
- First call resolve-library-id with the library name
- Pass the result to get-library-docs
- Provide a summary of what you learned

Show me the complete code with better instructions.
```

Review:

- Does the agent call resolve-library-id before get-library-docs?
- Is the library ID passed correctly between tools?
- Does the response include actual documentation content?
- Can you trace the tool calling sequence in the output?

### Prompt 3: Connect to Multiple MCP Servers

**What you're learning**: How to compose tool ecosystems with multiple MCP servers.

Ask AI:
```
I have two MCP servers:
1. Context7 at https://mcp.context7.com/mcp (documentation)
2. A custom server at https://internal-tools.example.com/mcp (internal APIs)

Both need Authorization headers with different API keys.

Build an agent that:
1. Connects to BOTH servers simultaneously
2. Uses resolve-library-id from Context7 when needed
3. Can also call internal tools from the custom server
4. Decides which server to use based on the question

Show the complete async code with both MCPServerStreamableHttp connections.
```

Review:

- Are both MCP servers in the mcp_servers list?
- Does the agent have access to tools from both servers?
- Can it intelligently choose which tool to use?
- How would you expand this to 5+ MCP servers?

### Prompt 4: Build TaskManager with Documentation Integration

**What you're learning**: How to architect production agents leveraging external MCP tool ecosystems.

Ask AI:
```
Build a complete TaskManager agent that:

1. Accepts a task context with project_name and current_library
2. Connects to Context7 MCP for documentation
3. When given a project description, uses Context7 to:
   - Look up the library specification
   - Retrieve documentation for relevant features
   - Suggest project tasks based on library capabilities
4. Returns structured output with:
   - Libraries used
   - Key capabilities identified from documentation
   - Recommended project tasks

Include:
- Pydantic context model
- MCPServerStreamableHttp connection
- Agent instructions that leverage Context7 tools
- Example execution with output

Show the complete, runnable code.
```

Review:

- Does the agent ground recommendations in actual documentation?
- Are multiple tools called in sequence (resolve → fetch)?
- How does documentation influence task suggestions?
- What happens if Context7 is unavailable?

### Reflection: MCP as Tool Composition

Ask yourself:

1. **Without MCP**: How would you hardcode Context7 documentation lookup?
   - Extra HTTP calls? Custom parsing? Versioning problems?

2. **With MCP**: How does standardization change your architecture?
   - Same pattern for any MCP server
   - Agents discover tools automatically
   - No custom integration code

3. **Scaling**: If you added 5 more MCP servers, what changes?
   - Just add more `async with MCPServerStreamableHttp(...) as server`
   - Agent tools grow automatically
   - No changes to agent logic

4. **Production readiness**: What would you add?
   - Connection pooling for performance
   - Error handling for unavailable servers
   - Tool caching to reduce latency
   - Authentication refresh strategies

Write your thoughts. You've now implemented agents that connect to external tool ecosystems—the foundation for production Digital FTEs that integrate with any MCP-compatible service.

**Expected outcome**: A working agent system where you can:
- Connect to remote MCP servers with authentication
- Automatically access tools from connected servers
- Use Context7 for dynamic documentation lookup
- Compose multiple MCP servers into unified tool ecosystems
- Design agents that leverage external tools effectively
