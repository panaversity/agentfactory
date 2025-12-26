---
sidebar_position: 6
title: "Stateful vs Stateless Servers"
description: "Master horizontal scaling decisions: Choose stateful for real-time collaboration or stateless for simple integrations. Understand how load balancers affect MCP server architecture"
keywords: [mcp, scaling, stateless, stateful, load-balancer, horizontal-scaling, streamablehttp, json_response]
chapter: 38
lesson: 6
duration_minutes: 28

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding Stateless vs Stateful Server Tradeoffs"
    proficiency_level: "C2"
    category: "Technical"
    bloom_level: "Understand, Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can articulate architectural tradeoffs between stateful (SSE connections, sampling, progress) and stateless (simple integration) approaches"

  - name: "Designing Horizontal Scaling Architecture for MCP"
    proficiency_level: "C2"
    category: "Technical"
    bloom_level: "Analyze, Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can reason about when to scale stateless vs stateful, predict load balancer challenges, and justify configuration choices"

  - name: "Configuring stateless_http and json_response"
    proficiency_level: "C2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can enable stateless mode and json_response, understand feature loss, and document tradeoffs in production decisions"

  - name: "Evaluating Production Scaling Requirements"
    proficiency_level: "C2"
    category: "Applied"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can assess project requirements and recommend appropriate configuration (stateful, stateless, or hybrid)"

learning_objectives:
  - objective: "Explain the horizontal scaling problem caused by load balancers and how it affects MCP server architecture"
    proficiency_level: "C2"
    bloom_level: "Understand"
    assessment_method: "Lesson walkthrough and visual diagrams showing request routing"

  - objective: "Articulate tradeoffs between stateful servers (SSE, sampling, progress) and stateless servers (simplicity, horizontal scaling)"
    proficiency_level: "C2"
    bloom_level: "Analyze"
    assessment_method: "Decision framework exercise comparing approaches"

  - objective: "Implement stateless_http=True configuration and understand feature limitations"
    proficiency_level: "C2"
    bloom_level: "Apply"
    assessment_method: "Code implementation with feature matrix comparison"

  - objective: "Choose appropriate configuration (stateful vs stateless) based on project requirements"
    proficiency_level: "C2"
    bloom_level: "Evaluate"
    assessment_method: "Case-study analysis in Try With AI scenarios"

cognitive_load:
  new_concepts: 7
  assessment: "Load balancer problem, stateless mode, json_response mode, session management, feature limitations, decision criteria, production tradeoffs. Concepts build on StreamableHTTP foundation."

differentiation:
  extension_for_advanced: "Research session affinity (sticky sessions) solutions; explore hybrid approaches with client-side session reconstruction; analyze cost implications of stateless vs stateful at scale"
  remedial_for_struggling: "Focus on decision framework first (stateless if no sampling/progress/streams, stateful otherwise); skip implementation details; use Try With AI for guided exploration"
---

# Stateful vs Stateless Servers

Your MCP server is working beautifully. You've implemented sampling (Lesson 2), progress notifications (Lesson 3), secure file access (Lesson 4), and deployed it with StreamableHTTP (Lesson 5). But then success happens: Hundreds of clients want to use your server simultaneously.

You deploy multiple instances behind a load balancer. Then something strange occurs. A client's request sometimes works, sometimes fails mysteriously. Progress messages never arrive even though you see them in logs. Sampling calls hang with timeouts.

Welcome to the **horizontal scaling problem**. It's not your code—it's your architecture.

## The Horizontal Scaling Problem

### Single Instance: Everything Works

When your server runs as a single instance, everything is straightforward:

```
Client A ──────────────┐
                       ├─→ Server Instance (Single)
Client B ──────────────┘
```

When Client A calls a tool that needs sampling:

1. Client A connects via SSE and gets a **session ID**
2. Tool needs LLM inference → Calls `context.session.create_message()`
3. Server has the session → Can route response back through Client A's SSE connection
4. Client A receives response → Tool completes successfully

### Multiple Instances Behind Load Balancer: The Problem

Now you scale horizontally:

```
Client A ──────────────┐
                       ├─→ [Load Balancer] ──┐
Client B ──────────────┘                     ├─→ Server A
                                             │   Server B
                                             └─→ Server C
```

Here's the failure scenario:

**Timeline of requests from Client A:**

1. Client A connects → Load balancer routes to **Server A**
2. Client A gets session ID from Server A's connection
3. Client A calls a tool that needs sampling
4. Load balancer receives the request → Routes to **Server B** (round-robin)
5. Server B doesn't have Client A's session (different instance)
6. Server B's tool can't find the client's SSE connection
7. `context.session.create_message()` fails → Tool returns error
8. Client A frustrated: "Why did it work once and fail the next time?"

**The core issue:** Each server instance maintains sessions **in memory**. When a request goes to a different instance, that instance has no knowledge of the client's session.

### Feature Loss in Horizontal Scaling

With stateful servers and load balancers, you lose:

| Feature | Why It Breaks | Solution |
|---------|---------------|----------|
| **Sampling** (`create_message()`) | Different server can't find client's SSE | Sticky sessions OR stateless mode |
| **Progress Notifications** | Different server can't send via client's SSE | Sticky sessions OR stateless mode |
| **Subscription Streams** | SSE connection bound to one server | Sticky sessions OR stateless mode |
| **Session-Based State** | Cache/state lost when request goes to different server | Stateless design |

## Solution 1: Stateless Mode (`stateless_http=True`)

The simplest scaling solution: **Disable features that require server ↔ client communication.**

### What Stateless Mode Does

```python
from mcp.server.fastmcp import FastMCP

mcp = FastMCP(
    name="simple_integration",
    stateless_http=True  # ← Enable stateless mode
)
```

When `stateless_http=True`:

1. **No SSE connections** → Server can't initiate requests to clients
2. **No session IDs** → No server ↔ client state
3. **No sampling** → Tools can't call `context.session.create_message()`
4. **No progress** → `context.report_progress()` is silently ignored
5. **No subscriptions** → Resource subscriptions don't work
6. **Plain HTTP POST → JSON Response** → Only client-initiated requests work

### Trade-off Analysis: Stateless Mode

**What you gain:**
- Horizontal scaling works perfectly (any server instance handles any request)
- Load balancing becomes trivial (no sticky sessions needed)
- Infinite scale potential
- Stateless design is cloud-native and resilient

**What you lose:**
- No server calling client (`create_message()` unavailable)
- No progress feedback for long operations
- No real-time notifications
- No sampling (AI inside server, not through client)
- No subscription streams

### When to Use Stateless Mode

```
✓ Use stateless when:
  - Simple tool integrations (read file, transform data, call API)
  - No sampling needed (server doesn't need LLM from client)
  - No real-time feedback required
  - Horizontal scaling is critical
  - Integration just needs "request → response"

✗ Avoid stateless when:
  - Server needs to call LLM through client (sampling)
  - Users need progress feedback (long operations)
  - Real-time bidirectional communication required
  - Notifications must be sent to client
```

### Code Example: Stateless Tool

```python
from mcp.server.fastmcp import FastMCP
from pydantic import Field

mcp = FastMCP(
    name="file_processor",
    stateless_http=True
)

@mcp.tool()
async def process_file(
    file_path: str = Field(description="Path to file"),
    operation: str = Field(description="Operation: count-lines, reverse-words, etc")
) -> str:
    """Process a file with specified operation.

    NOTE: Stateless mode - no sampling, no progress.
    This tool works independently without client communication.
    """

    # Read file
    with open(file_path, 'r') as f:
        content = f.read()

    # Process
    if operation == "count-lines":
        return f"{len(content.splitlines())} lines"
    elif operation == "reverse-words":
        words = content.split()
        return " ".join(reversed(words))
    else:
        return f"Unknown operation: {operation}"


# ✗ This WILL NOT WORK in stateless mode:
@mcp.tool()
async def analyze_with_ai(
    text: str = Field(description="Text to analyze"),
    *,
    context: Context
) -> str:
    """This tool requires sampling - incompatible with stateless mode."""

    # ❌ This call will FAIL silently in stateless mode
    response = await context.session.create_message(
        model="claude-3-5-sonnet-20241022",
        max_tokens=1024,
        messages=[{"role": "user", "content": f"Analyze: {text}"}]
    )
    return response.content[0].text
```

**Output comparison:**

When request hits **Server A** in stateless mode:
```
Client: POST /tool/process_file
  {file_path: "/data/users.csv", operation: "count-lines"}

Server A: ✓ Processes file
          → Returns: "1,250 lines"
```

Works the same if request hits **Server B**—because no server state needed.

## Solution 2: Reducing Payload Size (`json_response=True`)

A complementary approach: **Disable streaming responses, use plain JSON instead.**

### What json_response Does

```python
mcp = FastMCP(
    name="simple_api",
    json_response=True  # ← Enable JSON-only responses
)
```

When `json_response=True`:

1. **POST responses return JSON** → `{"result": value}` instead of streaming
2. **No intermediate streaming** → Tool must complete fully before response
3. **No log statements during execution** → Only final output sent
4. **Simpler integration** → Clients expect plain JSON, not SSE/streams

### Trade-off Analysis: json_response

**What you gain:**
- Compatible with ANY HTTP client (cURL, JavaScript fetch, etc.)
- No streaming complexity
- Standard REST API semantics
- Easy to cache, load-balance, and CDN

**What you lose:**
- No real-time progress (nothing until tool finishes)
- No streaming data (results fully buffered)
- No log messages shown live
- Large results may timeout if tool takes too long

### When to Use json_response

```
✓ Use json_response when:
  - Tools complete quickly (< 5 seconds)
  - Results fit in memory
  - Simple HTTP clients expected
  - Standard JSON REST integration desired

✗ Avoid json_response when:
  - Tools run long (need progress feedback)
  - Streaming large results (memory pressure)
  - Clients need live logging
  - Real-time feedback essential
```

## Configuration Matrix: Which Combination?

Here's how these two settings combine:

| Config | Stateless | JSON Response | Use Case |
|--------|-----------|---------------|----------|
| **Default** | `False` | `False` | Powerful servers with sampling, progress, streams. Single instance or sticky sessions. |
| **Stateless** | `True` | `False` | Horizontal scaling, SSE available, but no sampling/progress. |
| **JSON Response** | `False` | `True` | Single instance, JSON responses, but still supports sampling/progress. |
| **Full Simple** | `True` | `True` | Maximum simplicity. Horizontal scaling, plain JSON. No advanced features. |

**Real-world decision tree:**

```
Does tool need LLM inference? (sampling)
├─ YES → Stateless=False
├─ NO  → Can use Stateless=True

Does tool output large/streaming data?
├─ YES → json_response=False (streaming)
├─ NO  → Can use json_response=True

Do you need real-time progress?
├─ YES → Stateless=False, json_response=False
├─ NO  → Can use either

Is horizontal scaling critical?
├─ YES → Stateless=True (with json_response=True for simplicity)
├─ NO  → Stateless=False (more features)
```

## Feature Limitations Matrix

**Complete comparison of what works where:**

| Capability | Stateful + Streaming | Stateless + Streaming | Stateless + JSON |
|-----------|----------------------|----------------------|------------------|
| Basic tools (text processing) | ✓ | ✓ | ✓ |
| File I/O tools | ✓ | ✓ | ✓ |
| API calls | ✓ | ✓ | ✓ |
| `context.info()` / `context.warning()` | ✓ (sent live) | ✓ (sent live) | ✗ (lost) |
| `context.report_progress()` | ✓ (sent live) | ✗ (ignored) | ✗ (ignored) |
| `context.session.create_message()` | ✓ (works) | ✗ (can't route) | ✗ (can't route) |
| Resources + subscriptions | ✓ (streaming) | ✗ (no SSE) | ✗ (no SSE) |
| Horizontal scaling | ✗ (needs sticky) | ✓ (trivial) | ✓ (trivial) |
| Requires load balancer config | ✗ | ✓ | ✓ |

**Key insight:** Each option removes features to enable simpler scaling. Choose based on what your tools actually need.

## Decision Framework: Choosing Your Configuration

**Decision question 1: Do you need sampling?**

```
Does tool call context.session.create_message()?
├─ YES → Must use stateless=False (stateful server)
└─ NO  → Can use stateless=True
```

**Decision question 2: Do you need real-time progress?**

```
Do users need progress updates while tool runs?
├─ YES → Must use json_response=False (streaming)
└─ NO  → Can use json_response=True
```

**Decision question 3: What's your scale requirement?**

```
Expected concurrent clients?
├─ < 100  → Stateful (default) works fine, simpler
├─ 100-1000 → Evaluate. Stateless saves ops complexity
└─ > 1000  → Stateless becomes essential
```

**Decision question 4: What's your operational complexity budget?**

```
Can your ops team manage:
- Sticky sessions (session affinity)?
- Client-side session reconstruction?
- Cross-instance caching?
├─ YES → Can use stateful with load balancer
└─ NO  → Must use stateless
```

## Production Architecture Patterns

### Pattern 1: Single Stateful Instance

**When:** < 100 concurrent clients, features matter

```
Clients → Reverse Proxy → MCP Server (stateful, streaming)
                         └─ Single instance
```

**Configuration:**
```python
mcp = FastMCP(name="my_server")  # Default: stateful, streaming
```

**Pros:**
- All features available (sampling, progress, streams)
- Simple operations
- No session coordination

**Cons:**
- Vertical scaling only
- Single point of failure

### Pattern 2: Horizontal Stateless with Load Balancer

**When:** 1000+ concurrent clients, simplicity important

```
Clients → Load Balancer → MCP Server A (stateless, JSON)
                       └─ MCP Server B (stateless, JSON)
                       └─ MCP Server C (stateless, JSON)
```

**Configuration:**
```python
mcp = FastMCP(
    name="my_server",
    stateless_http=True,
    json_response=True
)
```

**Pros:**
- Infinite horizontal scaling
- No sticky sessions needed
- Any server handles any request
- Cloud-native resilience

**Cons:**
- No sampling, progress, or streaming
- Limited to request-response tools

### Pattern 3: Hybrid with Session Affinity

**When:** Need features + scaling, can manage sticky sessions

```
Clients ──→ Load Balancer (sticky sessions)
            ├─→ MCP Server A (stateful, streaming)
            ├─→ MCP Server B (stateful, streaming)
            └─→ MCP Server C (stateful, streaming)
```

**Configuration:**
```python
mcp = FastMCP(name="my_server")  # Default: stateful
# Operations: Configure load balancer sticky sessions
# (different per load balancer: nginx, AWS ALB, etc.)
```

**Pros:**
- All features available
- Horizontal scaling possible
- More operationally complex

**Cons:**
- Requires load balancer sticky session support
- Session affinity reduces load distribution
- Must handle session failover

## Try With AI

### Scenario 1: Deciding Architecture for a Customer Service Agent

You're building an MCP server that helps customer service teams. The tool:
- Analyzes customer messages
- Calls AI through the client to suggest responses
- Shows real-time progress (30% analyzed, 60% drafted, etc.)
- Streams large help articles

Your product roadmap shows 1,000+ customers in 6 months.

**Setup:**

You're planning deployment. You have three options:

**Option A:** Use default stateful server with load balancer sticky sessions
- Pro: All features work perfectly
- Con: Ops complexity managing session affinity

**Option B:** Use stateless mode, remove sampling and progress
- Pro: Simple horizontal scaling, no ops overhead
- Con: Users lose real-time feedback and AI assistance

**Option C:** Split into two services:
- Stateful service for sampling/progress work (runs on dedicated infrastructure)
- Stateless simple tools behind load balancer
- Pro: Features where needed, scaling where needed
- Con: More complex service architecture

**Your decision:**

Ask AI which approach best matches your constraints:
```
"I'm building an MCP server for customer service agents.
Requirements:
- Tool needs sampling (AI through connected client)
- Needs progress notifications (users see real-time feedback)
- Streams help articles (large results)
- Expected 1,000+ concurrent users in 6 months
- Team of 3 engineers (limited ops capacity)

What configuration should I recommend: stateful with sticky sessions,
stateless with feature removal, or service split? Justify the choice
considering engineering complexity vs user experience."
```

**What you're learning:**

This scenario teaches architectural tradeoff analysis—a key skill for production MCP design. You'll consider user needs, team capacity, operational complexity, and scaling requirements simultaneously.

### Scenario 2: Evaluating Existing System Problems

Your team deployed a stateful MCP server last month. It's behind an AWS Application Load Balancer, but requests randomly fail. Sometimes tools work, sometimes they timeout with "session not found" errors. Logs show the errors happen inconsistently.

**Setup:**

You suspect the load balancer is routing requests to different instances. Your current configuration:

```python
mcp = FastMCP(
    name="analytics_engine",
    # Using defaults (stateful, streaming)
)
```

The server implements:
- Text analytics tool (no sampling needed)
- Report generation (large streaming results)
- The issue: Sometimes works, sometimes fails with session errors

**Your diagnosis:**

Ask AI to help you diagnose and recommend fixes:
```
"Our MCP server behind AWS ALB randomly fails with 'session not found' errors.
Current setup:
- FastMCP with defaults (stateful, streaming)
- AWS ALB without sticky sessions
- Tools: text analytics, report generation (large results)
- Error pattern: Random failures, not consistent per tool

Diagnosis:
1. What's causing the random failures?
2. Can we use stateless mode (analyze feature loss)?
3. Should we enable sticky sessions instead?
4. What's the simplest fix?

For each option, show pros/cons and implementation complexity."
```

**What you're learning:**

This teaches diagnostic reasoning—understanding how architecture decisions create production problems, and evaluating solutions based on tradeoffs rather than just "fix the error."

### Scenario 3: Feature Negotiation with Product Team

Your product lead wants to add sampling to an existing stateless server. "Users want AI-assisted responses," she says. "Add the feature."

But your server runs stateless across 10 instances with 5,000 concurrent users.

**Setup:**

Current server:
```python
mcp = FastMCP(
    name="document_processor",
    stateless_http=True,
    json_response=True
)
```

Tools process documents (no LLM needed). Adding sampling would require calling LLM through client—incompatible with stateless mode.

**Your challenge:**

Ask AI to help you prepare the business case for this decision:
```
"Product wants to add AI-assisted document analysis (sampling required).
Current system: stateless with 5,000 concurrent users, 10 server instances.

Scenario options:
1. Switch to stateful + sticky sessions (adds ops complexity)
2. Keep stateless, remove AI feature
3. Split services: stateless for basic tools, stateful for AI-assisted tools
4. Client-side workaround: Clients call AI separately, send to tool

For each option, create a summary showing:
- User impact (what changes?)
- Ops impact (what's harder?)
- Timeline (how long to implement?)
- Cost (engineering, infrastructure)

What would you recommend to the product team and why?"
```

**What you're learning:**

This teaches business reasoning—how architectural constraints affect product decisions. You're learning to communicate technical tradeoffs to non-technical stakeholders, and to negotiate solutions that balance engineering constraints with product goals.

**Key decisions to carry forward:**

1. **Stateful (default)** → More features, more operational complexity
2. **Stateless (`True`)** → Simpler scaling, fewer features
3. **JSON Response (`True`)** → Standard HTTP integration, no streaming
4. **Choose based on:** Sampling needs, progress requirements, scale target, ops capacity

Your choice here determines whether you build a system that scales elegantly or collapses under load. Make it thoughtfully, using the scenarios above as your guide.
