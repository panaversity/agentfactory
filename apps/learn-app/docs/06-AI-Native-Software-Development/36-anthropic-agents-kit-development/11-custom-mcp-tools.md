---
sidebar_position: 11
title: "Custom MCP Tools"
description: "Build domain-specific tools using the @tool decorator and create_sdk_mcp_server. Understand when to create custom tools vs using built-ins, how to design tool interfaces, and why proper tool design determines agent capability boundaries."
keywords: [MCP tools, "@tool decorator", create_sdk_mcp_server, tool design, input validation, error handling, tool naming, agent extensibility]
chapter: 36
lesson: 11
duration_minutes: 35

# HIDDEN SKILLS METADATA
skills:
  - name: "Creating Custom Tools with @tool Decorator"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can write async functions decorated with @tool, define tool schemas with type hints and descriptions, and return properly formatted MCP responses with content blocks"

  - name: "Building MCP Servers with create_sdk_mcp_server"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can instantiate MCP servers from tool sets, register tools in agent options, and invoke tools using the mcp__<server>__<toolname> naming convention"

  - name: "Tool Design Decision Framework"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can decide when custom tools are necessary vs when existing integrations suffice, design tool boundaries that match domain workflows, and evaluate tool input schemas for clarity and safety"

learning_objectives:
  - objective: "Write custom tools using @tool decorator with async functions and proper type-hinted schemas"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student implements 2-3 domain-specific tools with input validation and error handling"

  - objective: "Create and configure MCP servers with create_sdk_mcp_server and register in ClaudeAgentOptions"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student builds working MCP server and configures agent to access tools via mcp__ naming convention"

  - objective: "Analyze tool design tradeoffs including granularity, input validation, and error boundaries"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Student redesigns overly coarse or overly granular tool set based on reasoning about agent control and error recovery"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (@tool decorator, async functions, type hints, MCP servers, tool schemas, error handling, tool naming convention) within B2 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Design hierarchical tool systems where tools expose capabilities incrementally (basic read tools → advanced mutation tools); implement tool versioning and backwards compatibility; analyze how tool design prevents prompt injection and jailbreak attempts; create safety frameworks that validate inputs before passing to domain systems"
  remedial_for_struggling: "Start with single simple tool (e.g., 'get_time') before complex domain tools; use provided tool templates; focus on the @tool decorator syntax and basic MCP registration before designing tool schemas; practice with synchronous functions before async patterns"
---

# Custom MCP Tools

You've learned to configure built-in tools and load organizational intelligence through skills. Now you're going to cross a capability threshold: **creating custom tools that extend your agent's reach into specialized domains**.

Built-in tools (Read, Bash, WebSearch) are universal—they work for any agent, any workflow, any domain. But they're also generic. They don't understand your business logic, your API patterns, or your specialized knowledge systems.

Custom tools let you encode domain expertise directly into the agent's capabilities. When you build a financial analysis agent, custom tools connect directly to your data warehouse. When you build a customer support agent, custom tools integrate with your CRM and ticketing system. When you build a DevOps agent, custom tools orchestrate your infrastructure.

**This is how you build Digital FTEs that go beyond generic automation into specialized intelligence that generates revenue.**

## Why Built-in Tools Aren't Enough

Consider building a Digital FTE for stock research. The agent needs to:

1. Fetch real-time stock prices and historical data
2. Analyze company fundamentals (earnings, P/E ratios, debt-to-equity)
3. Calculate technical indicators (moving averages, RSI, MACD)
4. Compare sector performance and peer groups
5. Generate investment theses

Could you build this with built-in tools?

**Technically, yes.** Your agent could use Bash to call curl to your data APIs. Or use WebSearch to look up financial websites.

**Practically, no.** Here's why:

**The Control Problem**: If your agent uses Bash, it has full terminal access. You can't restrict it to just financial APIs. You'd have to allow it to run any command, then hope it doesn't accidentally run `rm -rf /`.

**The Context Problem**: If your agent uses WebSearch for stock data, it gets noisy web results mixed with irrelevant financial news. Your agent spends context reasoning about signal vs noise instead of analyzing investment potential.

**The Latency Problem**: Fetching through WebSearch adds 2-3 seconds per query. Real-time stock data through Bash with retries adds more. Your agent becomes slow.

**The Reliability Problem**: Web scraping breaks when websites change design. API calls through Bash require error handling per command. Your agent becomes fragile.

**The Authority Problem**: Your agent doesn't know which financial source is authoritative. Is Yahoo Finance more reliable than Google Finance? Your agent can't validate.

**With custom tools**, the problem disappears:

```python
@tool("get_stock_price", "Get current stock price for a symbol")
async def get_stock_price(args):
    """Fetch from your trusted data source."""
    symbol = args["symbol"]
    # Query YOUR financial data warehouse
    # Rate-limited, authenticated, reliable
    price = financial_db.get_price(symbol)
    return {"content": [{"type": "text", "text": f"${symbol}: {price}"}]}
```

Your agent calls your custom tool. It's fast, reliable, authoritative, and controlled. Your agent can't do anything except what your tool allows.

## The Anatomy of a Custom Tool

Custom tools have three parts:

1. **Function signature**: What the tool does and what inputs it accepts
2. **Implementation**: The actual logic (API calls, database queries, calculations)
3. **Return format**: How results come back to the agent

Let's build a concrete example: a tool for fetching GitHub repository statistics.

### Step 1: Write the Tool Function

```python
from claude_agent_sdk import tool, create_sdk_mcp_server
import httpx
import json

@tool("get_github_repo_stats",
      "Get GitHub repository statistics including stars, forks, and last update",
      {"owner": str, "repo": str})
async def get_github_repo_stats(args):
    """
    Fetch repository metadata from GitHub API.

    Args contain:
      - owner: Repository owner (username or organization)
      - repo: Repository name

    Returns:
      - MCP response with repository statistics
    """
    owner = args.get("owner")
    repo = args.get("repo")

    # Input validation
    if not owner or not repo:
        return {
            "content": [{"type": "text", "text": "Error: owner and repo are required"}],
            "isError": True
        }

    try:
        # Call GitHub API
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"https://api.github.com/repos/{owner}/{repo}",
                headers={"Accept": "application/vnd.github.v3+json"}
            )

            if response.status_code == 404:
                return {
                    "content": [{"type": "text", "text": f"Repository {owner}/{repo} not found"}],
                    "isError": True
                }

            response.raise_for_status()
            data = response.json()

        # Format response for agent
        stats = f"""
GitHub Repository: {data['full_name']}
Stars: {data['stargazers_count']}
Forks: {data['forks_count']}
Open Issues: {data['open_issues_count']}
Last Updated: {data['updated_at']}
Language: {data['language'] or 'Not specified'}
Description: {data['description'] or 'No description'}
"""

        return {
            "content": [{"type": "text", "text": stats}]
        }

    except httpx.HTTPError as e:
        return {
            "content": [{"type": "text", "text": f"API error: {str(e)}"}],
            "isError": True
        }
```

**Key patterns here:**

- **@tool decorator**: Takes tool name, description, and input schema
- **async def**: Tools are async functions for non-blocking calls
- **Type hints in schema**: `{"owner": str, "repo": str}` tells agent what inputs to provide
- **Error handling**: Validate inputs, catch exceptions, return error responses
- **MCP response format**: Dict with `"content"` list containing `{"type": "text", "text": "..."}` blocks
- **isError flag**: Set to True for failures so agent can handle errors gracefully

### Step 2: Create the MCP Server

```python
# Gather your tools into a server
github_server = create_sdk_mcp_server(
    name="github",
    tools=[
        get_github_repo_stats
        # You could add more GitHub tools here:
        # - get_github_user_repos(username)
        # - search_github_repos(query, language)
        # - get_github_file(owner, repo, path)
    ]
)
```

The server bundles tools under a namespace. This prevents naming collisions and groups related functionality.

### Step 3: Register in Agent Options

```python
from claude_agent_sdk import query, ClaudeAgentOptions

options = ClaudeAgentOptions(
    mcp_servers={"github": github_server},
    allowed_tools=[
        "mcp__github__get_github_repo_stats"
        # Add more tool names here as you add tools
    ]
)

async for message in query(
    prompt="Analyze the anthropic/anthropic-sdk repository. How popular is it?",
    options=options
):
    print(message)
```

**Output:**
```
Assistant: I'll fetch the GitHub repository statistics for you.

[Agent calls mcp__github__get_github_repo_stats with owner="anthropic", repo="anthropic-sdk"]

GitHub Repository: anthropic/anthropic-sdk
Stars: 15,243
Forks: 1,456
Open Issues: 89
Last Updated: 2025-12-20T14:32:15Z
Language: Python
Description: The official Anthropic SDK for Python

The repository is highly popular with over 15,000 stars and strong community engagement...
```

## Tool Naming Convention

When you create an MCP server and register tools, the naming convention matters: `mcp__<servername>__<toolname>`

**Why this matters:**

- `mcp__` prefix distinguishes custom tools from built-in tools (Read, Bash, etc.)
- `<servername>` prevents collisions (two different servers could have `get_data` tool)
- `<toolname>` is the specific tool within the server

**Example breakdown:**
- `mcp__github__get_github_repo_stats` → Server: github, Tool: get_github_repo_stats
- `mcp__database__query_analytics` → Server: database, Tool: query_analytics
- `mcp__weather__get_forecast` → Server: weather, Tool: get_forecast

When you add the tool to `allowed_tools`, use the full name:

```python
options = ClaudeAgentOptions(
    mcp_servers={"github": github_server},
    allowed_tools=[
        "mcp__github__get_github_repo_stats",
        "mcp__github__search_github_repos",  # If you add more tools
        "Read",  # Mix custom and built-in tools
        "Bash"
    ]
)
```

## Input Validation: The Security Boundary

Your custom tools are the security boundary between the agent and your systems. **Invalid input from the agent is your responsibility to handle.**

Design tool schemas to validate early:

```python
@tool("database_query",
      "Execute SQL query against analytics database",
      {
          "query": str,
          "timeout_seconds": int,
          "max_results": int
      })
async def database_query(args):
    query = args.get("query", "").strip()
    timeout = args.get("timeout_seconds", 30)
    max_results = args.get("max_results", 1000)

    # Validation 1: Query length prevents memory exhaustion
    if len(query) > 10000:
        return {
            "content": [{"type": "text", "text": "Error: Query too long (max 10000 chars)"}],
            "isError": True
        }

    # Validation 2: Timeout prevents runaway queries
    if timeout < 1 or timeout > 300:
        return {
            "content": [{"type": "text", "text": "Error: timeout_seconds must be 1-300"}],
            "isError": True
        }

    # Validation 3: Results limit prevents memory issues
    if max_results < 1 or max_results > 100000:
        return {
            "content": [{"type": "text", "text": "Error: max_results must be 1-100000"}],
            "isError": True
        }

    # Only AFTER validation, execute
    try:
        results = await execute_query(query, timeout_seconds=timeout)
        limited_results = results[:max_results]
        return {
            "content": [{"type": "text", "text": json.dumps(limited_results)}]
        }
    except Exception as e:
        return {
            "content": [{"type": "text", "text": f"Query error: {str(e)}"}],
            "isError": True
        }
```

## When to Create Custom Tools

Not every problem needs a custom tool. Use this decision framework:

| Question | Answer → Decision |
|----------|------------------|
| **Does this access external systems?** | YES → Custom tool (read-only) |
| **Does this transform data your agent can't transform?** | YES → Custom tool |
| **Is this called 3+ times per workflow?** | YES → Custom tool |
| **Does this require authentication or secrets?** | YES → Custom tool (keeps secrets server-side) |
| **Can built-in tools solve this?** | YES → Use built-in |
| **Is this one-off logic?** | YES → Use agent prompt instead |

**Examples that justify custom tools:**

- Fetch from internal databases (not accessible via Bash)
- Call proprietary APIs (require authentication)
- Transform domain data (calculating financial metrics)
- Rate-limited operations (prevent agent from exhausting limits)
- Real-time data (stock prices, weather, inventory)

**Examples that don't:**

- File reading → Use built-in Read
- File editing → Use built-in Edit
- Running commands → Use built-in Bash
- General web browsing → Use built-in WebSearch
- Ad-hoc calculations → Include in tool implementation, not custom tool

## Tool Design Patterns: Granularity Matters

Tools can be **too coarse** (trying to do everything) or **too fine** (single responsibility to a fault).

### Anti-Pattern: The God Tool (Too Coarse)

```python
@tool("do_everything", "Handle all operations")
async def do_everything(args):
    operation = args["operation"]  # "fetch_data", "analyze", "report"?

    if operation == "fetch_data":
        # 50 lines of data fetching
    elif operation == "analyze":
        # 100 lines of analysis
    elif operation == "report":
        # 75 lines of report generation
    # ... 10 more operations
```

**Problems:**
- Agent doesn't know what sub-operations exist
- Hard to debug failures (which operation failed?)
- Difficult to rate-limit or audit individual operations
- Tool becomes maintenance nightmare

### Pattern: Granular Tools (Better)

```python
@tool("fetch_user_data", "Get user profile and recent activity")
async def fetch_user_data(args):
    user_id = args["user_id"]
    # Just fetch, focused responsibility
    return {"content": [...]}

@tool("analyze_user_behavior", "Analyze user engagement patterns")
async def analyze_user_behavior(args):
    user_id = args["user_id"]
    # Just analyze, focused responsibility
    return {"content": [...]}

@tool("generate_user_report", "Create PDF report of user activity")
async def generate_user_report(args):
    user_id = args["user_id"]
    # Just generate, focused responsibility
    return {"content": [...]}
```

**Benefits:**
- Agent knows exactly what each tool does
- Failures are localized and easy to debug
- Each tool can be rate-limited independently
- Easy to add/remove/modify tools
- Clear responsibilities

### Sweet Spot: Related Operations (Best Practice)

```python
@tool("get_user_profile",
      "Get user profile including name, email, account_status",
      {"user_id": str})
async def get_user_profile(args):
    """Returns: name, email, account_status, created_at, subscription_tier"""
    pass

@tool("get_user_activity",
      "Get recent user activity: logins, purchases, support tickets",
      {"user_id": str, "days": int})
async def get_user_activity(args):
    """Returns: login history, purchase history, support tickets"""
    pass
```

**Balance**: Each tool has a clear purpose. Related queries grouped together (profile info, activity info). Not a god tool, not over-fragmented.

## Error Handling: The Recovery Path

Your tools will fail. Database connections timeout. APIs return errors. Users provide invalid input. Design tools to fail gracefully:

```python
@tool("send_email",
      "Send email message",
      {"to": str, "subject": str, "body": str})
async def send_email(args):
    to = args.get("to", "").strip()
    subject = args.get("subject", "").strip()
    body = args.get("body", "").strip()

    # Layer 1: Input validation (agent's responsibility to follow schema)
    if not to or not subject or not body:
        return {
            "content": [{"type": "text", "text": "Error: All fields required (to, subject, body)"}],
            "isError": True
        }

    # Layer 2: Schema validation (email format)
    if "@" not in to:
        return {
            "content": [{"type": "text", "text": f"Error: Invalid email format: {to}"}],
            "isError": True
        }

    # Layer 3: Length constraints
    if len(body) > 100000:
        return {
            "content": [{"type": "text", "text": "Error: Email body too long (max 100KB)"}],
            "isError": True
        }

    # Layer 4: Try to send, handle transient failures
    max_retries = 3
    for attempt in range(1, max_retries + 1):
        try:
            await email_service.send(to, subject, body)
            return {
                "content": [{"type": "text", "text": f"Email sent successfully to {to}"}]
            }
        except ConnectionError as e:
            if attempt < max_retries:
                await asyncio.sleep(2 ** attempt)  # Exponential backoff
                continue
            return {
                "content": [{"type": "text", "text": f"Error: Could not reach email service after {max_retries} attempts"}],
                "isError": True
            }
        except Exception as e:
            # Unexpected error - don't retry
            return {
                "content": [{"type": "text", "text": f"Error sending email: {type(e).__name__}"}],
                "isError": True
            }
```

**Layers of defense:**

1. **Input validation**: Reject invalid schemas early
2. **Format validation**: Check email formats, URLs, etc.
3. **Resource limits**: Prevent DOS (message size, query complexity)
4. **Retry logic**: Handle transient failures (network timeouts)
5. **Specific errors**: Return actionable error messages agent can learn from

When an error occurs, agent sees `isError: True` and knows to handle it gracefully rather than assuming success.

## Complete Example: Weather Tool System

Here's a complete MCP server with multiple tools, showing production patterns:

```python
from claude_agent_sdk import tool, create_sdk_mcp_server
from datetime import datetime
import httpx

# Tool 1: Get current weather
@tool("get_weather",
      "Get current weather conditions for a location",
      {"latitude": float, "longitude": float})
async def get_weather(args):
    lat = args.get("latitude")
    lon = args.get("longitude")

    # Validate coordinates
    if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
        return {
            "content": [{"type": "text", "text": "Error: Invalid coordinates"}],
            "isError": True
        }

    try:
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"https://api.open-meteo.com/v1/forecast",
                params={
                    "latitude": lat,
                    "longitude": lon,
                    "current": "temperature_2m,precipitation,weather_code",
                    "timezone": "auto"
                }
            )
            response.raise_for_status()
            data = response.json()

        current = data["current"]
        return {
            "content": [{
                "type": "text",
                "text": f"Temperature: {current['temperature_2m']}°C, Precipitation: {current['precipitation']}mm"
            }]
        }
    except Exception as e:
        return {
            "content": [{"type": "text", "text": f"Weather API error: {str(e)}"}],
            "isError": True
        }

# Tool 2: Get forecast
@tool("get_forecast",
      "Get 7-day weather forecast",
      {"latitude": float, "longitude": float})
async def get_forecast(args):
    lat = args.get("latitude")
    lon = args.get("longitude")

    if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
        return {
            "content": [{"type": "text", "text": "Error: Invalid coordinates"}],
            "isError": True
        }

    try:
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"https://api.open-meteo.com/v1/forecast",
                params={
                    "latitude": lat,
                    "longitude": lon,
                    "daily": "temperature_2m_max,temperature_2m_min,precipitation_sum",
                    "timezone": "auto",
                    "forecast_days": 7
                }
            )
            response.raise_for_status()
            data = response.json()

        forecast_text = "7-Day Forecast:\n"
        for i, (date, high, low, precip) in enumerate(zip(
            data["daily"]["time"],
            data["daily"]["temperature_2m_max"],
            data["daily"]["temperature_2m_min"],
            data["daily"]["precipitation_sum"]
        )):
            forecast_text += f"{date}: {high}°C / {low}°C, Precip: {precip}mm\n"

        return {"content": [{"type": "text", "text": forecast_text}]}
    except Exception as e:
        return {
            "content": [{"type": "text", "text": f"Forecast API error: {str(e)}"}],
            "isError": True
        }

# Create the server
weather_server = create_sdk_mcp_server(
    name="weather",
    tools=[get_weather, get_forecast]
)

# Use in agent
options = ClaudeAgentOptions(
    mcp_servers={"weather": weather_server},
    allowed_tools=[
        "mcp__weather__get_weather",
        "mcp__weather__get_forecast"
    ]
)
```

Notice the production patterns:

- **Consistent error handling**: Both tools validate inputs, catch exceptions
- **Clear schemas**: latitude/longitude with type hints and ranges
- **Specific error messages**: Agent knows what failed and why
- **Async implementation**: Non-blocking API calls
- **Grouped functionality**: Related weather tools in one server
- **Rate limiting via tool names**: You can track which tool is called via logs

## Try With AI

**Setup**: You're building a Digital FTE for a startup that manages freelance projects. The agent needs to track projects, estimate timelines, and update status. You don't have access to your Jira instance from the agent's restricted environment.

**Prompt 1: Analyze Tool Needs**

Ask your AI collaborator:
```
I'm building a project management agent. It needs to:
1. Get project details (name, description, status)
2. List all tasks in a project
3. Estimate task duration
4. Update task status

Which of these should be custom MCP tools vs built-in tools or prompting logic?
Why would custom tools be better than having the agent use Bash to curl my API?
```

Evaluate the response. Does the AI correctly identify that:
- Reading sensitive project data from internal systems requires custom tools (not Bash with credentials exposed)
- Status updates are mutation operations requiring controlled access (custom tool, not Bash)
- Tools should group related operations (get project, list tasks in same tool family)

**Prompt 2: Design Tool Schemas**

Based on the feedback, ask:
```
Design the MCP tool schemas for project management. For each tool, define:
1. Tool name and description
2. Input parameters with types
3. Return value format

Create tools for:
- get_project_details(project_id)
- list_project_tasks(project_id, status_filter)
- estimate_task_duration(task_id, complexity_level)
- update_task_status(task_id, new_status)

For each tool, what validation should happen BEFORE calling the backend system?
```

Check that the AI schemas include:
- Type hints (project_id: str, complexity_level: int)
- Reasonable constraints (status_filter limited to: "open", "in_progress", "done")
- Validation examples ("complexity_level must be 1-5")
- Error cases handled

**Prompt 3: Implement with Error Handling**

Ask:
```
Write the async Python implementation for get_project_details and update_task_status tools.
Include:
1. Input validation
2. Error handling with meaningful error messages
3. Retry logic for transient failures
4. Proper MCP response format

What happens if the project doesn't exist? If the status update fails? If the API times out?
```

Compare your implementation against the AI's. Ask yourself:
- Does the error handling protect your backend?
- Are error messages helpful to the agent?
- Are transient failures retried, permanent failures reported?
- Is the MCP response format correct?

---

**Safety Note**: When building custom tools that access your systems, always validate inputs strictly and log tool usage. Your tools are the security boundary—invalid input passed through could expose your databases or APIs. Design tools as if they'll be called by untrusted code.

