---
sidebar_position: 7
title: "Tracing & Observability"
description: "Instrument agents with lifecycle hooks and custom tracing to track execution, debug behavior, measure performance, and generate OpenAI Platform trace URLs for production monitoring."
keywords: ["tracing", "observability", "RunHooks", "on_agent_start", "on_agent_end", "on_tool_start", "on_tool_end", "trace", "custom_span", "group_id", "gen_trace_id", "OpenAI Platform", "lifecycle hooks"]
chapter: 34
lesson: 7
duration_minutes: 90

# HIDDEN SKILLS METADATA
skills:
  - name: "RunHooks Lifecycle Instrumentation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "System Design"
    measurable_at_this_level: "Student can implement RunHooks to capture agent lifecycle events (start, end, tool execution)"

  - name: "Agent Execution Lifecycle Management"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can track on_agent_start, on_agent_end, on_tool_start, on_tool_end events with context access"

  - name: "Custom Tracing with trace() and custom_span()"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can instrument custom code sections with trace contexts and spans"

  - name: "Trace Correlation and Group ID Management"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Data Management"
    measurable_at_this_level: "Student can use group_id and gen_trace_id to correlate multi-agent executions"

  - name: "Usage Tracking and Cost Attribution"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Resource Management"
    measurable_at_this_level: "Student can access RunContextWrapper.usage to track tokens, costs, and model decisions"

  - name: "OpenAI Platform Integration for Production Debugging"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "System Design"
    measurable_at_this_level: "Student can generate trace URLs and view execution traces in OpenAI Platform"

  - name: "Production Observability Architecture"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Understand"
    digcomp_area: "System Design"
    measurable_at_this_level: "Student can design observability infrastructure for production agent systems"

learning_objectives:
  - objective: "Implement RunHooks to capture agent lifecycle events and build observability infrastructure"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create RunHooks that log agent start/end and tool execution with context"

  - objective: "Use trace() and custom_span() to instrument workflows and correlate multi-agent executions"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Build traced workflow that shows how agents collaborate across lifecycle"

  - objective: "Track usage metrics and attribute costs to specific agents and tools"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Implement usage tracking that captures tokens and identifies expensive operations"

  - objective: "Generate and interpret OpenAI Platform trace URLs for debugging production agents"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create trace that can be viewed in OpenAI Platform for visual debugging"

cognitive_load:
  new_concepts: 7
  assessment: "7 core concepts (RunHooks, on_agent_start/end, on_tool_start/end, trace/custom_span, group_id, gen_trace_id, usage tracking) appropriate for B2 proficiency with 90-minute duration. Concepts build on existing agent knowledge from previous lessons."

differentiation:
  extension_for_advanced: "Design distributed tracing across multiple services; implement custom span processors; analyze trace data to optimize agent decision paths; correlate traces with external monitoring systems"
  remedial_for_struggling: "Start with basic RunHooks logging without context manipulation; practice on_agent_end with usage access first; defer custom_span instrumentation until trace context is comfortable"

generated_by: content-implementer
source_spec: specs/047-ch34-openai-agents-sdk
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# Tracing & Observability

**Scenario**: Your Customer Support Digital FTE has been running in production for a week. Suddenly, you notice a spike in API costs. One customer's conversation consumed $47 while typical interactions cost $2. What happened? Was the agent looping endlessly? Did it call the wrong tool multiple times? Did it hit a specific edge case?

Without visibility into agent execution, you're blind. You see results but not reasoning. You see costs but not their causes.

This is where observability becomes critical. You need to see inside the agent's mind—what it decided, when it decided it, what tools it called, how long things took, and what it cost.

The OpenAI Agents SDK provides two complementary capabilities:

1. **RunHooks**: Lifecycle callbacks that capture agent start/end and tool execution events
2. **Tracing**: Context managers that instrument workflows and generate trace URLs for visualization

Together, they transform your agent from a black box into a transparent system you can monitor, debug, and optimize.

## Understanding Observability Requirements

In production systems, you need visibility at three levels:

**Level 1: Lifecycle Events** — When did the agent start? When did it end? What tools did it call?

**Level 2: Execution Context** — What data does the agent have access to? How many tokens did this operation use? What model processed each decision?

**Level 3: Causal Chains** — How do multiple agents' executions relate to each other? Which tools contributed to the final cost?

Without Level 1, you can't debug failures. Without Level 2, you can't optimize costs. Without Level 3, you can't understand multi-agent workflows.

RunHooks handle Levels 1 and 2. Tracing handles Level 3.

## RunHooks: Capturing Lifecycle Events

A RunHook is a Python class that implements callback methods for agent lifecycle events. When you pass a RunHook to `Runner.run()`, the framework invokes your callbacks at each stage of execution.

### Defining a Basic RunHook

```python
from agents import RunHooks, RunContextWrapper, Agent

class LifecycleLogger(RunHooks):
    async def on_agent_start(
        self,
        context: RunContextWrapper,
        agent: Agent
    ):
        print(f"🚀 Agent '{agent.name}' starting")

    async def on_agent_end(
        self,
        context: RunContextWrapper,
        agent: Agent,
        output: str
    ):
        print(f"✅ Agent '{agent.name}' completed")
        if context.usage:
            print(f"   Tokens: {context.usage.total_tokens}")

hooks = LifecycleLogger()
result = Runner.run_sync(agent, "Hello", hooks=hooks)
```

**Output:**
```
🚀 Agent 'Customer Support' starting
✅ Agent 'Customer Support' completed
   Tokens: 342
```

This tells you when the agent started and stopped, and how many tokens the entire execution consumed. But you're still missing granularity—which specific operations were expensive?

### Tracking Tool Execution

RunHooks also capture tool start and end events:

```python
class DetailedTracker(RunHooks):
    async def on_tool_start(
        self,
        context: RunContextWrapper,
        agent: Agent,
        tool
    ):
        print(f"🔧 Tool '{tool.name}' starting")

    async def on_tool_end(
        self,
        context: RunContextWrapper,
        agent: Agent,
        tool,
        result: str
    ):
        print(f"   Completed: {result[:50]}...")

hooks = DetailedTracker()
result = Runner.run_sync(agent, "Find user details", hooks=hooks)
```

**Output:**
```
🚀 Agent 'Customer Support' starting
🔧 Tool 'database_lookup' starting
   Completed: User ID: 12345, Email: user@...
🔧 Tool 'knowledge_base_search' starting
   Completed: Similar issues: [Issue-4821, Issue-5102...
✅ Agent 'Customer Support' completed
   Tokens: 543
```

Now you can see the sequence of tool calls and identify bottlenecks. But you need more context about *why* tools were called and what the agent was thinking.

### Accessing RunContext for Rich Observability

The `RunContextWrapper` passed to every hook contains execution context:

```python
from agents import RunHooks, RunContextWrapper, Agent

class TaskManagerObserver(RunHooks):
    async def on_agent_end(
        self,
        context: RunContextWrapper,
        agent: Agent,
        output: str
    ):
        # Access the custom context object
        if hasattr(context, 'task_id'):
            task_id = context.task_id
            user_id = context.user_id
            print(f"Task {task_id} (User {user_id}): Completed")

        # Access usage metrics
        if context.usage:
            print(f"  Input tokens: {context.usage.input_tokens}")
            print(f"  Output tokens: {context.usage.output_tokens}")
            print(f"  Total: {context.usage.total_tokens}")

        # Access messages (full conversation history)
        print(f"  Turns in conversation: {len(context.messages)}")
```

The RunContextWrapper bridges your custom context (task_id, user_id, etc.) with SDK-provided context (usage, messages). This is where you connect agent observability to your business domain.

## Tracing: Correlating Multi-Agent Workflows

RunHooks give you visibility into a single agent's execution. But when you have multiple agents handing off to each other, you need to correlate their executions into a single causal chain.

Tracing does this by creating a context that spans multiple agents:

```python
from agents import trace, custom_span, gen_trace_id
from agents import Runner

trace_id = gen_trace_id()

# All agents in this workflow share the same trace context
with trace("customer_inquiry_workflow", group_id=trace_id) as workflow_trace:

    # Triage agent runs within trace
    with custom_span("triage_phase"):
        triage_result = Runner.run_sync(triage_agent, customer_input)

    # Specialist agent runs within same trace
    with custom_span("specialist_phase"):
        specialist_result = Runner.run_sync(specialist_agent, triage_result)

print(f"Trace ID: {trace_id}")
```

**Output:**
```
Trace ID: 550e8400-e29b-41d4-a716-446655440000

[OpenAI Platform will show:]
customer_inquiry_workflow
├── triage_phase
│   ├── triage_agent (0.3s, 122 tokens)
│   └── database_lookup tool (0.1s)
└── specialist_phase
    ├── specialist_agent (0.5s, 234 tokens)
    └── email_notification tool (0.2s)
```

The trace creates a visual timeline of the entire workflow. You can see:
- How long each phase took
- How many tokens each agent consumed
- Which tools were called in which order
- Where time was spent

This is what gets rendered in the OpenAI Platform's visual trace viewer.

### Custom Spans for Structured Workflows

`custom_span()` creates named sections within a trace. This is useful for breaking up workflows into logical phases:

```python
with trace("support_ticket_resolution", group_id=customer_id) as trace_ctx:

    # Phase 1: Intake
    with custom_span("intake"):
        ticket = Runner.run_sync(intake_agent, customer_message)

    # Phase 2: Analysis
    with custom_span("analysis"):
        analysis = Runner.run_sync(analysis_agent, ticket)

    # Phase 3: Solution
    with custom_span("solution"):
        solution = Runner.run_sync(solver_agent, analysis)

    # Phase 4: Communication
    with custom_span("communication"):
        response = Runner.run_sync(writer_agent, solution)
```

Each span is a time-bounded section. If the solution phase takes 5 minutes while others take 100ms, you'll see it immediately in the trace visualization.

## Cost Attribution and Usage Tracking

The real value of observability emerges when you tie it to costs. OpenAI's pricing model means expensive operations are often concentrated in a few agents or specific scenarios.

### Implementing Cost-Aware Hooks

```python
from agents import RunHooks

class CostTracker(RunHooks):
    def __init__(self):
        self.total_cost = 0.0
        self.costs_by_agent = {}

    async def on_agent_end(
        self,
        context: RunContextWrapper,
        agent: Agent,
        output: str
    ):
        if context.usage:
            # OpenAI pricing (as of 2025-12-26)
            input_cost = context.usage.input_tokens * 0.00003  # $0.03/1M tokens
            output_cost = context.usage.output_tokens * 0.00006  # $0.06/1M tokens
            agent_cost = input_cost + output_cost

            self.total_cost += agent_cost
            self.costs_by_agent[agent.name] = agent_cost

            print(f"💰 {agent.name}: ${agent_cost:.4f}")

# Run with cost tracking
tracker = CostTracker()
result = Runner.run_sync(support_agent, "Help me reset my password", hooks=tracker)

print(f"\nTotal cost: ${tracker.total_cost:.4f}")
for agent, cost in tracker.costs_by_agent.items():
    print(f"  {agent}: ${cost:.4f}")
```

**Output:**
```
💰 Customer Support Agent: $0.0187
💰 Database Lookup Tool: $0.0023

Total cost: $0.0210
  Customer Support Agent: $0.0187
  Database Lookup Tool: $0.0023
```

Now when you see a $47 bill for one conversation, you can trace it back to specific agents and tools. Was the triage agent looping? Was one tool call unexpectedly expensive? Which customer interaction triggered high costs?

### Building a TaskManager with Full Observability

Combining RunHooks with context objects creates a complete observability system:

```python
from agents import Agent, Runner, RunHooks
from pydantic import BaseModel
from datetime import datetime
import sqlite3

class TaskManagerContext(BaseModel):
    task_id: str
    user_id: str
    created_at: str

class ObservableRunner:
    def __init__(self, db_path="tasks.db"):
        self.db = sqlite3.connect(db_path)
        self._init_tables()

    def _init_tables(self):
        self.db.execute("""
            CREATE TABLE IF NOT EXISTS task_executions (
                task_id TEXT,
                user_id TEXT,
                agent_name TEXT,
                tokens_used INTEGER,
                cost REAL,
                started_at TEXT,
                ended_at TEXT,
                status TEXT
            )
        """)
        self.db.commit()

    class ExecutionLogger(RunHooks):
        def __init__(self, db, context):
            self.db = db
            self.context = context
            self.start_time = None

        async def on_agent_start(self, context, agent):
            self.start_time = datetime.now()

        async def on_agent_end(self, context, agent, output):
            if context.usage:
                cost = (
                    context.usage.input_tokens * 0.00003 +
                    context.usage.output_tokens * 0.00006
                )

                self.db.execute("""
                    INSERT INTO task_executions
                    (task_id, user_id, agent_name, tokens_used, cost, started_at, ended_at, status)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                """, (
                    self.context.task_id,
                    self.context.user_id,
                    agent.name,
                    context.usage.total_tokens,
                    cost,
                    self.start_time.isoformat(),
                    datetime.now().isoformat(),
                    "completed"
                ))
                self.db.commit()

    def run(self, agent: Agent, user_input: str, task_id: str, user_id: str):
        ctx = TaskManagerContext(
            task_id=task_id,
            user_id=user_id,
            created_at=datetime.now().isoformat()
        )

        logger = self.ExecutionLogger(self.db, ctx)
        return Runner.run_sync(agent, user_input, context=ctx, hooks=logger)

# Usage
runner = ObservableRunner()
result = runner.run(
    support_agent,
    "I can't log in",
    task_id="TASK-12345",
    user_id="USER-789"
)

# Query observability data
cursor = runner.db.execute("""
    SELECT agent_name, tokens_used, cost
    FROM task_executions
    WHERE task_id = ?
""", ("TASK-12345",))

for agent_name, tokens, cost in cursor:
    print(f"{agent_name}: {tokens} tokens (${cost:.4f})")
```

This creates a production-grade observability system where every agent execution is logged with cost attribution.

## OpenAI Platform Trace URLs

The most powerful feature: OpenAI Platform provides a visual trace viewer where you can see execution timelines, token consumption, and decision sequences.

When you use `trace()`, the SDK automatically generates a trace URL:

```python
from agents import trace

with trace("customer_interaction", group_id="user-123") as trace_context:
    result = Runner.run_sync(agent, input_text)

# Generate the platform URL
if trace_context.trace_url:
    print(f"View trace: {trace_context.trace_url}")
```

**The URL looks like:**
```
https://platform.openai.com/traces/550e8400-e29b-41d4-a716-446655440000
```

In the OpenAI Platform, you see:
- **Timeline**: Every API call, tool execution, and decision point on a horizontal timeline
- **Token breakdown**: Input vs output tokens for each LLM call
- **Message flow**: Full conversation history and model reasoning
- **Tool execution**: What each tool was called with and what it returned
- **Latency analysis**: Which operations were slow

This is invaluable for debugging agent behavior. When a customer says "Why did the agent do that?", you can show them the exact trace.

## Comparing Observability Strategies

Different observability needs call for different approaches:

| Scenario | Strategy | Example |
|----------|----------|---------|
| **Development debugging** | RunHooks + print logging | Understand why agent made a bad decision during development |
| **Cost optimization** | CostTracker hook + database | Identify which agents/tools are expensive |
| **Production monitoring** | Traces + group_id correlation | Track multi-agent workflows in production |
| **Customer support** | OpenAI Platform URLs | Show customers exactly what the agent did |
| **Performance analysis** | Custom spans + timing data | Identify bottlenecks (slow tools, long reasoning) |

## Try With AI

### Setup

You'll build a complete observability system for a TaskManager with cost tracking and trace URLs.

**Prerequisites**: You have a working agent from a previous lesson.

**Required imports**:
```python
from agents import Agent, Runner, RunHooks, trace, custom_span, gen_trace_id
from pydantic import BaseModel
```

### Prompt 1: Design Your First RunHook

**What you're learning**: How to capture lifecycle events and access agent context.

Ask AI:
```
I'm building a customer support agent. I want to log when the agent starts and ends,
and track how many tokens were used.

Create a RunHook class that:
1. Logs agent start with the agent name
2. Logs agent end with agent name and total tokens used
3. Access the RunContextWrapper to get usage.total_tokens

Show me the complete implementation.
```

Now review the response:

- Does the hook class inherit from `RunHooks`?
- Are the async methods `on_agent_start` and `on_agent_end` correctly defined?
- Does it correctly access `context.usage.total_tokens`?
- Compare: Could you have written this without AI's suggestion of the `async` pattern?

### Prompt 2: Extend Tracking to Tool Execution

**What you're learning**: How tool-level granularity reveals execution details.

Based on your first hook, ask AI:
```
Now extend the hook to also track when individual tools start and end.

Add:
1. on_tool_start - log which tool is being called
2. on_tool_end - log the tool result

Then show how to pass this extended hook to Runner.run_sync() when calling an agent
that uses tools.
```

Review the response:

- Can you see tool execution in the logging output?
- Does the output show agent events AND tool events in sequence?
- Which tools were expensive (how many tokens)?
- Compare: Did the tool tracking reveal information the agent-level tracking missed?

### Prompt 3: Add Cost Attribution

**What you're learning**: How to connect tokens to real dollars for business decisions.

Ask AI:
```
I need to track costs across agent executions. Modify the hook to calculate
approximate costs using these rates:
- Input tokens: $0.00003 per 1K tokens
- Output tokens: $0.00006 per 1K tokens

The hook should:
1. Calculate cost after each agent completes
2. Print agent name and cost
3. Keep a running total

Then show me how to access costs_by_agent after multiple executions.
```

Review:

- Does the cost calculation match the pricing model?
- Which agents were most expensive?
- Could you optimize by reducing tokens to certain agents?

### Prompt 4: Build a Traced Multi-Agent Workflow

**What you're learning**: How tracing correlates multiple agent executions into causal chains.

Ask AI:
```
I have two agents:
- triage_agent (categorizes customer requests)
- specialist_agent (handles the specific issue)

Build a workflow using trace() and custom_span() that:
1. Creates a trace with a unique group_id
2. Runs triage_agent within a "triage" span
3. Passes triage output to specialist_agent within a "specialist" span
4. Returns the trace URL at the end

Show the complete workflow code.
```

Review:

- Does the trace group_id allow you to correlate multiple agent runs?
- Are the custom_span names meaningful?
- Does the output include a trace URL you could visit in OpenAI Platform?

### Reflection: Observability as Design

Ask yourself these questions about your traced workflow:

1. **Visibility**: Without the trace, could you have understood what the agents were doing?
2. **Cost attribution**: Which agent (triage or specialist) consumed more tokens?
3. **Debugging**: If the specialist agent gave a bad answer, how would the trace help you understand why?
4. **Optimization**: What would you change to reduce cost or latency based on the trace data?

Write your answers in a comment. You've now built observability infrastructure that production systems depend on.

**Expected outcome**: A working observability system where you can:
- See agent lifecycle events (start, end, tool execution)
- Track costs per agent
- Correlate multi-agent workflows in traces
- Access OpenAI Platform URLs for visual debugging
