---
sidebar_position: 5
title: "Callbacks & Guardrails"
description: "Master Google ADK's callback system to implement safety guardrails, enforce policies, and control agent behavior at critical execution points"
keywords: ["Google ADK", "callbacks", "guardrails", "before_tool_callback", "after_tool_callback", "agent safety", "policy enforcement", "state management"]
chapter: 35
lesson: 5
duration_minutes: 60

# HIDDEN SKILLS METADATA
skills:
  - name: "ADK Callback System Architecture"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Software Architecture"
    measurable_at_this_level: "Student can identify the six callback types and explain when each executes in the agent lifecycle"

  - name: "Before-Tool Callback Implementation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can write before_tool_callback functions that validate, filter, or block tool execution"

  - name: "After-Tool Callback for Response Enhancement"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can implement after_tool_callback to transform tool responses or add transparency metadata"

  - name: "Policy Enforcement Patterns"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Security & Compliance"
    measurable_at_this_level: "Student can design domain filtering, rate limiting, and access control via callbacks"

  - name: "Callback-Aware Agent Instructions"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can update agent instructions to reference callback behavior and expected response formats"

  - name: "State Management in Callbacks"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can read and write to tool_context.state for logging, caching, and audit trails"

learning_objectives:
  - objective: "Understand the six callback types and their execution order in the agent lifecycle"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student maps callbacks to lifecycle phases; identifies which callback addresses given use case"

  - objective: "Implement before_tool_callback for domain filtering and policy enforcement"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates callback that blocks searches on prohibited domains"

  - objective: "Implement after_tool_callback for response enhancement and transparency"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates callback that extracts and logs sources from search results"

  - objective: "Design callback return behavior (None vs dict/LlmResponse) to control execution flow"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student articulates when to return None (allow) vs dict (block/override)"

  - objective: "Register callbacks with Agent class and understand execution order"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student configures agent with multiple callbacks; demonstrates correct execution order"

cognitive_load:
  new_concepts: 9
  assessment: "9 concepts (6 callback types, before_tool_callback, after_tool_callback, policy enforcement, state management, callback return behavior, execution order, domain filtering, response enhancement) within B1 limit of 10 concepts - PASS"

differentiation:
  extension_for_advanced: "Explore before_model_callback for LLM guardrails; implement caching patterns with callback state; build reusable callback library for common policies; compare ADK callbacks vs OpenAI input/output guardrails; investigate Plugins for global policy application"
  remedial_for_struggling: "Start with after_tool_callback first (simpler: observing only); use simple domain list instead of complex regex filtering; test callbacks individually before combining; use adk web UI Events tab to visualize callback execution"

generated_by: content-implementer
source_spec: Chapter 35 Lesson 5 specification (DeepLearning.AI L5 course + Google ADK official docs)
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# Callbacks & Guardrails

Your agent has been researching news, collecting financial data, and maintaining conversation state. But you're starting to notice problems: researchers use it to scrape competitor websites, and tool responses include raw HTML instead of clean summaries.

This is where **callbacks** come in. Callbacks are user-defined functions that hook into specific points in your agent's execution lifecycle—before and after each major operation. They give you precise control to enforce safety policies, validate data, enhance responses, and manage execution flow without modifying the core agent code.

In this lesson, you'll implement the two most powerful callbacks for building reliable agents: **before_tool_callback** for guardrails and **after_tool_callback** for response enhancement. You'll see how returning None allows normal execution while returning a dict or LlmResponse can block operations or override results entirely.

## The ADK Callback System

Google ADK provides six callback types that execute at different points in the agent's lifecycle. Understanding this architecture helps you choose the right callback for each use case.

### Six Callback Types

| Callback | Executes | Best For | Return Value |
|----------|----------|----------|--------------|
| **before_agent_callback** | Before agent processes request | Setup, request logging | None (usually) |
| **after_agent_callback** | After agent completes execution | Cleanup, response logging | None (usually) |
| **before_model_callback** | Before LLM is called | Input validation, prompt modification | None or `LlmResponse` |
| **after_model_callback** | After LLM responds | Response sanitization, output guardrails | None or `LlmResponse` |
| **before_tool_callback** | Before tool executes | Input validation, policy enforcement | None or dict (block) |
| **after_tool_callback** | After tool returns result | Response enhancement, logging, caching | dict with modified result |

**Key principle**: Returning `None` from a before-callback allows normal execution to proceed. Returning a value overrides the default behavior:
- `before_tool_callback` returns dict → Tool execution is skipped; dict becomes the result
- `after_tool_callback` returns dict → Modified dict replaces the tool's original response
- `before_model_callback` returns LlmResponse → LLM call is skipped; LlmResponse is processed

Let's focus on the two most important for building guardrails: before_tool_callback and after_tool_callback.

## Before-Tool Callback: Domain Filtering & Policy Enforcement

**Use case**: You want to block your agent from searching certain domains or enforce rate limits on API calls.

### Pattern: Domain Filtering

Here's how to prevent tool execution when search targets prohibited domains:

```python
BLOCKED_DOMAINS = [
    "wikipedia.org",
    "reddit.com",
    "youtube.com",
    "medium.com",
]

def filter_news_sources_callback(tool, args, tool_context):
    """
    Blocks search requests targeting certain domains.

    Args:
        tool: The tool object being invoked (has name, description)
        args: Dictionary of tool arguments {"query": "..."}
        tool_context: Provides access to state, session info

    Returns:
        None if tool should execute normally
        dict with "error" and "reason" if tool should be blocked
    """
    if tool.name == "google_search":
        query = args.get("query", "").lower()

        # Check if query targets a blocked domain
        for domain in BLOCKED_DOMAINS:
            if f"site:{domain}" in query:
                print(f"BLOCKED: {query}")
                return {
                    "error": "blocked_source",
                    "reason": f"Searches targeting {domain} are not allowed in this environment."
                }

        print(f"ALLOWED: {query}")

    # Return None to allow tool execution
    return None
```

**How it works:**
1. **Check condition**: Examine tool name and arguments
2. **Return None if allowed**: Tool executes normally with its original arguments
3. **Return dict if blocked**: Framework skips tool execution and uses your dict as the result

This callback prevents your agent from accidentally searching competitor sites, scraping Reddit discussions, or accessing YouTube transcripts.

### Agent Configuration

Register the callback with your agent:

```python
from google.adk.agents import Agent
from google.adk.tools import google_search

agent = Agent(
    name="guarded_research_agent",
    model="gemini-2.5-flash",
    instruction="""You are a news researcher with strict source guidelines.
You can search Google News and technical blogs.
You cannot search Wikipedia, Reddit, YouTube, or Medium.
If a query targets a blocked source, you'll receive an error message.
In that case, rephrase your search to target allowed sources only.""",
    tools=[google_search],
    before_tool_callback=[filter_news_sources_callback]
)
```

**Output Example:**

When your agent tries to search "site:wikipedia.org machine learning advances":

```
BLOCKED: site:wikipedia.org machine learning advances
{
  "error": "blocked_source",
  "reason": "Searches targeting wikipedia.org are not allowed in this environment."
}
```

The agent receives this error, realizes it violated a policy, and reformulates its search without the blocked domain.

## After-Tool Callback: Response Enhancement & Transparency

**Use case**: You want to add metadata to tool responses, extract sources for transparency, or transform raw results into cleaner formats.

### Pattern: Source Transparency Logging

Here's how to extract domains from search results and add a transparency log:

```python
import re
from urllib.parse import urlparse

def inject_process_log_after_search(tool, args, tool_context, tool_response):
    """
    Enhances search results by extracting and logging source domains.

    Args:
        tool: The tool object (has name, description)
        args: Tool arguments {"query": "..."}
        tool_context: Provides access to state, session info
        tool_response: The raw response from the tool (string)

    Returns:
        Enhanced dict with both results and source transparency log
    """
    if tool.name == "google_search" and isinstance(tool_response, str):
        # Extract all URLs from the response
        urls = re.findall(r'https?://[^\s/]+', tool_response)

        # Get unique domains
        unique_domains = sorted(set(urlparse(url).netloc for url in urls))

        if unique_domains:
            # Create a transparency log entry
            sourcing_log = f"Sources: {', '.join(unique_domains)}"

            # Read existing process log from state
            current_log = tool_context.state.get('process_log', [])

            # Update state with new entry
            tool_context.state['process_log'] = [sourcing_log] + current_log

        # Return enhanced response with both results and metadata
        return {
            "search_results": tool_response,
            "process_log": tool_context.state.get('process_log', [])
        }

    # Return tool_response unchanged for other tools
    return tool_response
```

**How it works:**
1. **Extract metadata**: Parse URLs from tool results
2. **Update state**: Store sourcing information in tool_context.state
3. **Return enhanced dict**: Include both original results and transparency metadata

### Agent Configuration with Enhanced Instructions

```python
agent = Agent(
    name="transparent_research_agent",
    model="gemini-2.5-flash",
    instruction="""You are a news researcher with transparency requirements.

When you search for information, the google_search tool returns results with a special format:
{
  "search_results": "...",
  "process_log": ["Sources: domain1.com, domain2.com", ...]
}

The process_log shows which domains your searches accessed. Include this transparency information in your final report:

"I researched this topic using: [sources from process_log]"

This builds user trust by showing your research methodology.""",
    tools=[google_search],
    after_tool_callback=[inject_process_log_after_search]
)
```

**Output Example:**

When agent searches "AI safety breakthroughs 2025":

```python
{
  "search_results": "[Full search result text...]",
  "process_log": [
    "Sources: news.google.com, techcrunch.com, arxiv.org",
    "Sources: github.com, huggingface.co"
  ]
}
```

Your agent sees this structured response, extracts the sources, and includes transparency in its report: "I researched this across TechCrunch, ArXiv, and GitHub to ensure current information."

## Combining Callbacks: Policy Enforcement + Transparency

Real-world agents often combine multiple callbacks. Here's a complete setup:

```python
# Combine both callbacks
agent = Agent(
    name="controlled_news_agent",
    model="gemini-2.5-flash",
    instruction="""You are an AI News Podcast Agent.

Your research tools have safety guardrails:
- Some domains are blocked (you'll receive error messages)
- All searches are logged for transparency
- You can see which sources you accessed via process_log

When research is blocked, reformulate your queries to access allowed sources.
Always cite your sources in the podcast script.""",
    tools=[google_search, get_financial_context],
    before_tool_callback=[filter_news_sources_callback],
    after_tool_callback=[inject_process_log_after_search]
)
```

**Execution flow:**

1. Agent decides to search "AI news this week"
2. **before_tool_callback**: Validates it doesn't target blocked domains → ALLOWED
3. Tool executes: google_search("AI news this week") → returns raw results
4. **after_tool_callback**: Extracts sources, updates process_log → returns enhanced dict
5. Agent receives: `{"search_results": "...", "process_log": [...]}`
6. Agent can see which domains it searched and include that transparency

## State Management in Callbacks

Callbacks have access to `tool_context.state`—a dictionary persistent across the agent's session. Use it for:

### Audit Trails

Track which tools were called with what arguments:

```python
def audit_trail_callback(tool, args, tool_context):
    """Log all tool calls for audit purposes."""
    if 'audit_log' not in tool_context.state:
        tool_context.state['audit_log'] = []

    tool_context.state['audit_log'].append({
        "tool": tool.name,
        "args": args,
        "timestamp": datetime.now().isoformat()
    })

    return None  # Allow execution to proceed
```

### Rate Limiting

Prevent excessive API calls:

```python
def rate_limit_callback(tool, args, tool_context):
    """Allow max 5 google_search calls per session."""
    if tool.name == "google_search":
        call_count = tool_context.state.get('search_count', 0)

        if call_count >= 5:
            return {
                "error": "rate_limit_exceeded",
                "reason": "Maximum 5 searches per session reached."
            }

        tool_context.state['search_count'] = call_count + 1

    return None
```

### Caching

Avoid redundant API calls:

```python
def caching_callback(tool, args, tool_context):
    """Cache search results by query."""
    if tool.name == "google_search":
        query = args.get("query", "")
        cache_key = f"search:{query}"

        # Check if we've searched this before
        if cache_key in tool_context.state:
            print(f"Using cached result for: {query}")
            return tool_context.state[cache_key]

    return None

def cache_store_callback(tool, args, tool_context, tool_response):
    """Store successful searches in cache."""
    if tool.name == "google_search":
        query = args.get("query", "")
        cache_key = f"search:{query}"
        tool_context.state[cache_key] = tool_response

    return tool_response
```

## Callback Return Behavior Reference

**This is the critical distinction between allowing and blocking:**

| Callback Type | Return None | Return Value |
|---------------|-------------|--------------|
| **before_tool_callback** | Tool executes with original args | Tool execution skipped; dict becomes result |
| **after_tool_callback** | Original response returned unchanged | Response replaced with returned dict |
| **before_model_callback** | LLM call proceeds normally | LLM call skipped; LlmResponse processed |

Example of blocking vs allowing:

```python
def example_callback(tool, args, tool_context):
    if condition_met:
        return {"error": "blocked"}  # Block execution
    else:
        return None  # Allow execution
```

## Callback Best Practices

### 1. Keep Callbacks Focused

Each callback should do one thing well:

```python
# Good: Single responsibility
def filter_domains_callback(tool, args, tool_context):
    """Only filters; doesn't log, cache, or modify."""
    if condition:
        return {"error": "blocked"}
    return None

# Avoid: Multiple concerns
def overloaded_callback(tool, args, tool_context):
    """Filters AND logs AND caches AND validates."""
    # Too much logic in one place
```

### 2. Handle Errors Gracefully

Don't let callback exceptions crash your agent:

```python
def safe_callback(tool, args, tool_context):
    try:
        # Your callback logic
        return None
    except Exception as e:
        print(f"Callback error: {e}")
        # Return None to allow tool execution rather than crashing
        return None
```

### 3. Update Agent Instructions

Always document callback behavior in agent instructions so the agent understands its constraints:

```python
agent = Agent(
    name="constrained_agent",
    instruction="""You have access to google_search with the following constraints:

Blocked domains: Wikipedia, Reddit, YouTube, Medium
Maximum searches: 5 per conversation
Rate limit: 1 search per second

All your searches are logged for transparency. Include sources in your final report.""",
    before_tool_callback=[filter_domains, rate_limit],
)
```

## Try With AI

**Setup**: You'll build a research agent with both guardrails and transparency.

### Prompt 1: Implement Domain Filtering

Copy and execute this code in your Python environment:

```python
from google.adk.agents import Agent
from google.adk.tools import google_search

BLOCKED_DOMAINS = ["reddit.com", "twitter.com", "youtube.com"]

def filter_social_media(tool, args, tool_context):
    """Block searches on social media platforms."""
    if tool.name == "google_search":
        query = args.get("query", "").lower()
        for domain in BLOCKED_DOMAINS:
            if f"site:{domain}" in query:
                return {
                    "error": "blocked_domain",
                    "reason": f"{domain} is not allowed for research in this context."
                }
    return None

agent = Agent(
    name="filtered_research_agent",
    model="gemini-2.5-flash",
    instruction="You are a researcher. You can search the web but not social media platforms.",
    tools=[google_search],
    before_tool_callback=[filter_social_media]
)

# Test: Agent tries to search Twitter
response = agent.run("Search site:twitter.com AI trends 2025")
print(response)
```

**What you're learning:** How returning a dict from before_tool_callback blocks tool execution and provides feedback to the agent.

### Prompt 2: Add Source Transparency

Enhance your agent with after_tool_callback:

```python
import re
from urllib.parse import urlparse

def add_source_transparency(tool, args, tool_context, tool_response):
    """Add source domains to every search result."""
    if tool.name == "google_search" and isinstance(tool_response, str):
        urls = re.findall(r'https?://[^\s/]+', tool_response)
        domains = sorted(set(urlparse(url).netloc for url in urls))

        if domains:
            if 'sources_accessed' not in tool_context.state:
                tool_context.state['sources_accessed'] = []
            tool_context.state['sources_accessed'].extend(domains)

        return {
            "results": tool_response,
            "sources": domains
        }
    return tool_response

# Update agent with both callbacks
agent = Agent(
    name="transparent_research_agent",
    model="gemini-2.5-flash",
    instruction="""You are a transparent researcher.

Your search results include a 'sources' list showing which domains you accessed.
In your final report, cite these sources: "I accessed information from: [sources]"

You cannot search social media (Reddit, Twitter, YouTube).""",
    tools=[google_search],
    before_tool_callback=[filter_social_media],
    after_tool_callback=[add_source_transparency]
)

# Test: Combined guardrails + transparency
response = agent.run("Find recent AI safety research and cite your sources")
print(response)
```

**What you're learning:** How combining before and after callbacks creates both safety and transparency in agent behavior.

### Prompt 3: State Management & Rate Limiting

Add rate limiting to prevent excessive API consumption:

```python
def enforce_search_limit(tool, args, tool_context):
    """Allow maximum 5 searches per session."""
    if tool.name == "google_search":
        search_count = tool_context.state.get('search_count', 0)

        if search_count >= 5:
            return {
                "error": "rate_limit_exceeded",
                "reason": f"Maximum 5 searches reached. Current usage: {search_count}/5"
            }

        tool_context.state['search_count'] = search_count + 1
        print(f"Search {search_count + 1}/5")

    return None

# Agent with three callbacks: filtering, rate limiting, transparency
agent = Agent(
    name="fully_controlled_agent",
    model="gemini-2.5-flash",
    instruction="""You are a controlled research agent with three constraints:

1. Domain filtering: Cannot search Reddit, Twitter, YouTube
2. Rate limiting: Maximum 5 searches per conversation
3. Transparency: All sources must be cited

When you hit rate limits or blocked domains, adapt your search strategy.""",
    tools=[google_search],
    before_tool_callback=[filter_social_media, enforce_search_limit],
    after_tool_callback=[add_source_transparency]
)

# Test: Multiple searches with rate limiting
response = agent.run("""
Find information about three topics:
1. Latest AI safety research
2. Emerging AI applications
3. AI ethics frameworks

Show me which sources you used for each.""")
print(response)
```

**What you're learning:** How to combine multiple before_tool_callbacks to create layered policy enforcement, and how to track state across callback executions.

**When you're done:**
- Run each prompt and observe how callbacks control behavior
- Modify BLOCKED_DOMAINS and SEARCH_LIMIT values; see how agent adapts
- Check tool_context.state to understand persistence across tool calls
- Write one additional callback (e.g., authentication, cost tracking) to solidify your understanding
