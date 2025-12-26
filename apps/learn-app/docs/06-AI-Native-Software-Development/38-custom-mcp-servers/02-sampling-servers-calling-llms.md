---
sidebar_position: 2
title: "Sampling — Servers Calling LLMs"
description: "Learn how MCP servers request LLM inference from clients using sampling, enabling powerful workflows where servers delegate reasoning to Claude or other frontier models without managing API keys."
keywords: ["sampling", "MCP servers", "LLM inference", "ctx.session.create_message", "client-side callbacks", "distributed reasoning"]
chapter: 38
lesson: 2
duration_minutes: 18

# HIDDEN SKILLS METADATA
skills:
  - name: "Implementing Sampling in MCP Servers"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can write an MCP server tool that uses ctx.session.create_message() to request Claude inference and handle the response"

  - name: "Understanding Server-Client LLM Delegation"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "System Architecture"
    measurable_at_this_level: "Student can explain why sampling is superior to servers managing their own API keys and when to use direct calls vs sampling"

  - name: "Implementing Sampling Callbacks"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can write a sampling_callback function that intercepts LLM requests and routes them through the client's model"

  - name: "Designing Sampling Workflows"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design server tools that combine deterministic operations with AI-powered reasoning through sampling"

learning_objectives:
  - objective: "Understand sampling as the mechanism for servers to request LLM inference without managing API keys"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of sampling pattern and why it's superior to server-side API key management"

  - objective: "Implement server-side sampling using ctx.session.create_message() to call Claude"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Write a working server tool that uses sampling to request Claude summarization"

  - objective: "Implement client-side sampling callback to handle LLM requests from servers"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Write a sampling callback that intercepts requests and routes through Claude"

  - objective: "Evaluate when to use sampling vs direct API calls vs deterministic operations"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Scenario analysis: given a tool requirement, decide which approach is most appropriate"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (sampling pattern, ctx.session.create_message, SamplingMessage, CreateMessageRequestParams, sampling callback, request interception, token management, workflow composition) within B1 limit (7-10 concepts) ✓ - Concepts chunk into sampling mechanics (4) and implementation patterns (4)"

differentiation:
  extension_for_advanced: "Implement streaming in sampling callbacks to reduce latency; analyze cost implications of routing all inference through client vs caching strategies"
  remedial_for_struggling: "Focus on single synchronous example first; separate sampling mechanics from async patterns; use concrete tool (research summarization) before abstract patterns"

---

# Sampling — Servers Calling LLMs

Your research tool needs to summarize documents. Your code analysis tool needs to explain complex patterns. Your customer support server needs to reason about edge cases. These tools require AI inference. But you face a problem:

**If the server manages its own API key**, it's tightly coupled to a specific model provider. It becomes expensive (running continuously), complex (error handling, rate limiting), and insecure (keys embedded everywhere).

**If the server asks the client to call Claude**, costs shift to the client, complexity vanishes, and the server stays provider-agnostic. This is sampling.

Sampling is the MCP mechanism that lets servers request LLM inference from clients without managing keys, credentials, or provider relationships. The server asks, "Hey client, can you run Claude on this?" The client responds, "Sure, here's what Claude says." Deterministic operations stay on the server. Reasoning operations route through the client's models.

This is how you build hybrid systems: fast operations on the server, reasoning on the frontier model, scaling without complexity.

## The Sampling Problem and Solution

Let's look at what you need vs what sampling gives you:

### The Naive Approach (Don't Do This)

```python
import anthropic
import os

@mcp.tool()
async def summarize(text_to_summarize: str):
    """Summarize research findings."""
    # PROBLEM: Server needs its own API key
    client = anthropic.Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))

    message = client.messages.create(
        model="claude-3-5-sonnet-20241022",
        max_tokens=1000,
        messages=[{
            "role": "user",
            "content": f"Summarize: {text_to_summarize}"
        }]
    )
    return message.content[0].text
```

**Problems**:
- Server has API key embedded (security risk)
- Server pays for inference (cost leakage)
- Server depends on Anthropic specifically (no provider flexibility)
- Server must handle Claude errors directly (complexity)
- Server can't use client's context window (isolated reasoning)

### The Sampling Approach (Do This Instead)

```python
@mcp.tool()
async def summarize(text_to_summarize: str, ctx: Context):
    """Summarize research findings using sampling."""
    # Client provides inference—server just specifies intent
    result = await ctx.session.create_message(
        messages=[
            SamplingMessage(
                role="user",
                content=TextContent(type="text", text=f"Summarize: {text_to_summarize}")
            )
        ],
        max_tokens=1000,
        system_prompt="You are a research expert summarizing academic findings"
    )

    if result.content.type == "text":
        return result.content.text
    else:
        raise ValueError("Sampling failed")
```

**Benefits**:
- No API keys on server (secure)
- Client pays for inference (aligned costs)
- Client determines model (flexible)
- Client handles errors (centralized)
- Server can leverage client's context (powerful)

## How Sampling Works: The Complete Flow

The pattern involves two sides working together:

### Server Side: Request Inference

```python
from mcp import Context
from mcp.types import SamplingMessage, TextContent

@mcp.tool()
async def analyze_code(code_snippet: str, ctx: Context):
    """Analyze Python code using Claude."""

    # Server specifies WHAT to do
    result = await ctx.session.create_message(
        messages=[
            SamplingMessage(
                role="user",
                content=TextContent(
                    type="text",
                    text=f"Identify bugs and suggest fixes:\n{code_snippet}"
                )
            )
        ],
        max_tokens=2000,
        system_prompt="You are an expert Python code reviewer. Focus on bugs, security issues, and performance."
    )

    # Unpack response
    if result.content and result.content[0].type == "text":
        analysis = result.content[0].text
        return {
            "analysis": analysis,
            "source": "claude-3-5-sonnet-20241022"
        }
    else:
        raise ValueError("Code analysis failed")
```

**What's happening**:
- Server creates `SamplingMessage` (not a regular message—signals "route to client")
- Calls `ctx.session.create_message()` (special method for sampling)
- Specifies `system_prompt` (instructions for Claude)
- Awaits response from client's Claude

### Client Side: Handle Sampling Callback

```python
from mcp.client.session import ClientSession
from mcp.types import CreateMessageRequestParams, CreateMessageResult

async def sampling_callback(
    request_id: str,
    params: CreateMessageRequestParams
):
    """Intercept server's sampling requests and route through Claude."""

    # Client has API key and model access—server doesn't
    client = anthropic.Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))

    # Convert sampling request to standard Claude API call
    response = client.messages.create(
        model="claude-3-5-sonnet-20241022",  # Client decides model
        max_tokens=params.max_tokens,
        system=params.system_prompt,
        messages=params.messages  # Already formatted by server
    )

    # Return result back to server
    return CreateMessageResult(
        model="claude-3-5-sonnet-20241022",
        content=[TextContent(
            type="text",
            text=response.content[0].text
        )],
        role="assistant"
    )

# Register callback with client
session = ClientSession(
    transport,
    init_options={
        "sampling_callback": sampling_callback
    }
)
```

**What's happening**:
- Client defines `sampling_callback` (intercepts `create_message` requests)
- Client owns API key (server never sees it)
- Client calls Claude (determines model, handles errors)
- Client returns structured response to server

## Comparison: Three Approaches

| Factor | Direct Server Call | Sampling | API Pass-Through |
|--------|-------------------|----------|------------------|
| **Server complexity** | High (API management) | Low (just request) | Medium |
| **Cost management** | Server pays | Client pays | Hidden |
| **Security** | Keys on server (risky) | Keys on client only (safe) | Keys duplicated |
| **Model flexibility** | Fixed to server's choice | Client controls model | Fixed upstream |
| **Error handling** | Server handles | Client handles | Mixed |
| **Context awareness** | Server context only | Client's full context | Limited |
| **Use case** | Never (anti-pattern) | Inference requests | Data transform |

**Key insight**: Sampling shifts both cost and responsibility to the client (where they belong). The server focuses on what it knows best; the client focuses on inference.

## Real-World Sampling Example: Research Assistant

Here's a complete example combining research operations (server) with AI reasoning (sampling):

**Server: Research Tool**

```python
import httpx
from mcp import Context
from mcp.types import SamplingMessage, TextContent

@mcp.tool()
async def research_topic(topic: str, ctx: Context):
    """Research a topic and synthesize findings."""

    # Step 1: Server fetches raw research (deterministic)
    async with httpx.AsyncClient() as client:
        response = await client.get(
            f"https://api.arxiv.org/query",
            params={"search_query": f"all:{topic}", "max_results": 5}
        )
        papers = response.json()

    # Step 2: Server extracts key information (deterministic)
    research_data = {
        "papers": [
            {
                "title": p["title"],
                "abstract": p["summary"][:500]  # First 500 chars
            }
            for p in papers[:5]
        ]
    }

    # Step 3: Server asks Claude to synthesize (sampling)
    synthesis = await ctx.session.create_message(
        messages=[
            SamplingMessage(
                role="user",
                content=TextContent(
                    type="text",
                    text=f"""Synthesize these research papers into a coherent summary.

Papers:
{json.dumps(research_data['papers'], indent=2)}

Create:
1. Key findings
2. Research gaps
3. Future directions"""
                )
            )
        ],
        max_tokens=1500,
        system_prompt="You are a research analyst synthesizing academic findings for executives."
    )

    # Step 4: Return synthesized findings
    if synthesis.content and synthesis.content[0].type == "text":
        return {
            "raw_papers": len(research_data['papers']),
            "synthesis": synthesis.content[0].text
        }
    else:
        raise ValueError("Synthesis failed")
```

**Client Usage**:

```python
# Client registers callback
async def handle_research_sampling(
    request_id: str,
    params: CreateMessageRequestParams
):
    """Route research synthesis through Claude."""
    response = client.messages.create(
        model="claude-3-5-sonnet-20241022",
        max_tokens=params.max_tokens,
        system=params.system_prompt,
        messages=params.messages
    )
    return CreateMessageResult(
        model="claude-3-5-sonnet-20241022",
        content=response.content,
        role="assistant"
    )

# When client calls the research tool
response = await session.call_tool(
    name="research_topic",
    arguments={"topic": "transformer architectures"}
)
# Output:
# {
#   "raw_papers": 5,
#   "synthesis": "Transformers have evolved from..."
# }
```

**What you see**:
- Server fetches data, formats it
- Server asks Claude to synthesize (sampling)
- Claude responds through client
- Server returns synthesized result to caller

The server never needed an API key. The client never needed to manage fetching. Clean separation.

## When to Use Sampling

**Use sampling when**:
- Server needs to reason about data it collected
- Tool requires natural language understanding
- You want client to bear inference costs
- Tool should work with any frontier model (not tied to one provider)
- You need the client's broader context

**Examples**:
- Summarizing documents (server retrieves, Claude synthesizes)
- Analyzing code (server parses, Claude explains)
- Content moderation (server checks patterns, Claude evaluates intent)
- Decision support (server fetches data, Claude recommends action)

**Don't use sampling when**:
- Operation is purely deterministic (pure computation, data lookup)
- Operation must complete in <500ms (sampling has latency)
- Server can't format data properly (client shouldn't guess intent)
- Cost must be server-controlled (sampling routes to client)

## Try With AI

**Part 1: Review the Code**

You have a document processing server that needs to classify documents. Read this code snippet:

```python
@mcp.tool()
async def classify_document(content: str, ctx: Context):
    """Classify document using sampling."""
    classification = await ctx.session.create_message(
        messages=[
            SamplingMessage(
                role="user",
                content=TextContent(
                    type="text",
                    text=f"Classify this document:\n{content}"
                )
            )
        ],
        max_tokens=200
    )
    return classification.content[0].text
```

Ask Claude: **"This server tool uses sampling to classify documents. What's missing from the implementation that would make it production-ready?"**

Pay attention to Claude's suggestions about:
- System prompts (does Claude have guidance?)
- Error handling (what if sampling fails?)
- Response validation (is the classification in the expected format?)

**Part 2: Implement Error Handling**

Claude will likely suggest handling sampling failures. Based on its suggestions, ask: **"Show me how to add retry logic and graceful fallback if sampling fails."**

Review the code Claude generates. Ask yourself:
- Does this handle the case where sampling times out?
- Does this prevent infinite retry loops?
- Does the fallback return sensible output?

**Part 3: Design a Sampling Workflow**

You're building a code review server. It needs to:
1. Parse Python code (deterministic)
2. Ask Claude for security issues (sampling)
3. Ask Claude for performance suggestions (sampling)
4. Return both analyses

Ask Claude: **"Design a tool that does code review using two separate sampling calls—one for security, one for performance. Should these be separate tools or one tool with two sampling requests?"**

Compare Claude's recommendation to your instinct. Ask: **"Why did you recommend this approach? What's the tradeoff?"**

This question forces you to think about sampling composition—when to use multiple samples vs when to combine them.

**Part 4: Evaluate Tradeoffs**

You're deciding: should your research server call Claude for each paper (5 sampling calls) or fetch all papers then call Claude once?

Ask Claude: **"Compare these two approaches: (A) sample Claude for each paper's summary individually, or (B) fetch all papers, then sample Claude once for synthesis. What are the latency, cost, and quality tradeoffs?"**

Notice what emerges: Claude will explain reasoning you might not have considered (parallelization, context window efficiency, response quality). Ask yourself: **"Which tradeoff matters most for my use case?"**

This is sampling in action—not just moving code around, but reasoning about when AI involvement creates value vs when it adds latency without benefit.
