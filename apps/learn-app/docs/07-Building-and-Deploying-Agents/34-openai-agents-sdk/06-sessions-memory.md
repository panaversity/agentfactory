---
sidebar_position: 6
title: "Sessions & Conversation Memory"
description: "Build persistent conversation memory that remembers context across multiple agent interactions, enabling stateful AI experiences and conversation branching for complex workflows."
keywords: [sessions, memory, conversation history, SQLiteSession, AdvancedSQLiteSession, session persistence, branching]
chapter: 34
lesson: 6
duration_minutes: 90

# HIDDEN SKILLS METADATA
skills:
  - name: "Session-Based Conversation Management"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can implement persistent session memory for multi-turn agent conversations"

  - name: "Conversation Context Preservation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design sessions that maintain context across runs without manual history management"

  - name: "Usage Analytics & Token Tracking"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can measure and optimize token consumption across conversation turns"

  - name: "Conversation Branching Strategy"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design conversation branches to explore alternative paths without losing original context"

  - name: "Stateful Agent Architecture"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can architect multi-turn agent workflows with persistent state across customer interactions"

  - name: "Production Memory Systems"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Digital Content Creation"
    measurable_at_this_level: "Student can choose between SQLiteSession (simple) and AdvancedSQLiteSession (analytics) for production use"

learning_objectives:
  - objective: "Implement conversation memory that persists across multiple agent interactions without manual history management"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Build agent with multi-turn session and verify context is remembered"

  - objective: "Design conversation branches to explore alternative paths while maintaining referential integrity"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create branch from specific turn and verify conversation history integrity"

  - objective: "Track token usage and conversation metrics to optimize production agent costs"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Implement usage tracking and generate session statistics"

  - objective: "Distinguish between SQLiteSession and AdvancedSQLiteSession for different production scenarios"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Evaluate tradeoffs and choose appropriate session type for given requirements"

cognitive_load:
  new_concepts: 6
  assessment: "6 core concepts (SQLiteSession, session_id, get_items, AdvancedSQLiteSession, store_run_usage, create_branch_from_turn) appropriate for B2 proficiency level with 90-minute duration. Scaffolding decreases as students progress through hands-on examples."

differentiation:
  extension_for_advanced: "Design distributed session management across multiple services; implement custom session backends (PostgreSQL, Redis); analyze conversation metrics for agent optimization patterns"
  remedial_for_struggling: "Focus on basic SQLiteSession first; practice persistence across 2-3 runs before attempting branching; use visual diagrams of session state transitions"
---

# Sessions & Conversation Memory

Memory is what transforms a chatbot into a relationship. When your customer says "I mentioned this last week," a stateless system has to ask them to repeat themselves. But a system with memory remembers—and that transforms the interaction from transactional to genuine.

In production customer support, agents need to remember previous interactions. When a user returns after three days, the agent should know the context of their original issue, what was attempted, and what the current status is. Without memory, every conversation starts from zero. With memory, every interaction builds on understanding that deepens over time.

The OpenAI Agents SDK provides session management that handles this automatically. You don't manually append messages to a history list. You don't manually parse conversation state from databases. You create a session, pass it to `Runner.run()`, and the framework handles persistence—automatically maintaining context across calls.

This lesson teaches two paths: the simple `SQLiteSession` for basic memory, and the more powerful `AdvancedSQLiteSession` for production systems that need analytics, branching, and detailed usage tracking.

## Understanding Session Memory

A session is a container for conversation history. Think of it like a notebook that automatically saves every exchange between you and the agent.

### What a Session Does

**Without a session:**
```python
# Conversation 1
result1 = await Runner.run(agent, "My name is Alice")
print(result1.final_output)

# Conversation 2 - agent has NO memory
result2 = await Runner.run(agent, "What's my name?")
print(result2.final_output)
# Output: "I don't know your name. What is it?"
```

**Output:**
```
Nice to meet you, Alice!
I don't know your name. What is it?
```

**With a session:**
```python
from agents import Agent, Runner, SQLiteSession

session = SQLiteSession("user-alice-123")

# Conversation 1
result1 = await Runner.run(agent, "My name is Alice", session=session)
print(result1.final_output)

# Conversation 2 - agent remembers
result2 = await Runner.run(agent, "What's my name?", session=session)
print(result2.final_output)
```

**Output:**
```
Nice to meet you, Alice!
Your name is Alice. We discussed this earlier.
```

The session automatically includes all previous messages in the context window. The agent sees the full conversation history and responds with awareness of what came before.

### Session Architecture: Two Approaches

The SDK provides two session implementations with different tradeoffs:

| Feature | SQLiteSession | AdvancedSQLiteSession |
|---------|---|---|
| **Persistence** | In-memory or file-based SQLite | File-based SQLite with structured tables |
| **Memory tracking** | Yes (implicit) | Yes (detailed metrics per turn) |
| **Branching** | No | Yes (create alternative paths) |
| **Usage analytics** | No | Yes (tokens, requests, tool calls) |
| **Complexity** | Simple | Moderate |
| **Best for** | Development, simple apps | Production, customer support, analytics |

**Decision**: Use `SQLiteSession` when you need basic memory. Use `AdvancedSQLiteSession` when you need production reliability, analytics, or conversation branching.

## Building Conversation Memory with SQLiteSession

`SQLiteSession` is the simplest way to add memory. Create one, pass it to every `Runner.run()` call, and the framework handles the rest.

### Basic Session Setup

```python
from agents import Agent, Runner, SQLiteSession

# Create an agent
agent = Agent(
    name="Assistant",
    instructions="You are a helpful support agent. Remember context from previous messages."
)

# Create a session (in-memory, lost on restart)
session = SQLiteSession("customer-456")

# First message
print("Turn 1: User introduces issue")
result = await Runner.run(
    agent,
    "I'm having trouble with my billing account",
    session=session
)
print(f"Agent: {result.final_output}")

# Second message - agent remembers
print("\nTurn 2: User provides more context")
result = await Runner.run(
    agent,
    "It's been happening for 3 weeks",
    session=session
)
print(f"Agent: {result.final_output}")

# Third message - still in context
print("\nTurn 3: User asks for resolution")
result = await Runner.run(
    agent,
    "Can you fix this for me?",
    session=session
)
print(f"Agent: {result.final_output}")
```

**Output:**
```
Turn 1: User introduces issue
Agent: I understand you're experiencing billing issues. Let me help resolve this. Can you tell me the email address associated with your account?

Turn 2: User provides more context
Agent: Three weeks is concerning. Let me look into your account history. Based on the information you provided earlier (billing account issue), this is urgent.

Turn 3: User asks for resolution
Agent: I'll escalate this immediately since you've been waiting three weeks. I've reviewed your account from our previous conversation and can see the issue clearly.
```

Notice how the agent references previous messages without you explicitly providing them. The session automatically includes history.

### Persistent File-Based Sessions

For production use, store sessions in a database file that survives restarts:

```python
from agents import Agent, Runner, SQLiteSession

# File-based session (persists on disk)
session = SQLiteSession(
    session_id="customer-789",
    db_path="conversations.db"  # Stored on disk
)

# Later, even after the program restarts...
# Create new session with same ID and path
session = SQLiteSession(
    session_id="customer-789",
    db_path="conversations.db"
)

# Agent still has full history
result = await Runner.run(
    agent,
    "Hi! I'm back. Do you remember me?",
    session=session
)
print(result.final_output)
# Output: "Of course! You were working with us on your billing account issue three weeks ago. How can I help today?"
```

**Output:**
```
Of course! You were working with us on your billing account issue three weeks ago. How can I help today?
```

The session is persistent—the conversation history survives application restarts.

### Retrieving Conversation History

You can access the conversation history stored in a session:

```python
# Get last 3 messages
recent = await session.get_items(limit=3)
for item in recent:
    role = item.get("role", item.get("type"))
    content = item.get("content", "")
    print(f"{role}: {content}")

# Get all messages
all_history = await session.get_items()
print(f"Total turns in conversation: {len(all_history)}")
```

**Output:**
```
assistant: Of course! You were working with us on your billing account issue...
user: Hi! I'm back. Do you remember me?
assistant: I understand you're experiencing billing issues...
user: I'm having trouble with my billing account
Total turns in conversation: 6
```

This enables you to analyze conversations, audit agent behavior, or reconstruct what happened.

## Production Memory: AdvancedSQLiteSession

For customer support systems, you need more than basic memory. You need to track metrics, understand costs, and explore alternative conversation paths. That's what `AdvancedSQLiteSession` provides.

### Setting Up AdvancedSQLiteSession

```python
from agents import Agent, Runner, function_tool
from agents.extensions.memory import AdvancedSQLiteSession

# Create an agent with a tool
@function_tool
async def check_order_status(order_id: str) -> str:
    """Check the status of a customer order."""
    # Simulated order lookup
    if order_id == "ORD-123":
        return "Order shipped on Dec 15, arriving Dec 20"
    return "Order not found"

agent = Agent(
    name="Support Agent",
    instructions="Help customers with order inquiries. Use tools to check status.",
    tools=[check_order_status]
)

# Create advanced session with usage tracking
session = AdvancedSQLiteSession(
    session_id="customer-support-001",
    create_tables=True  # Create schema on first run
)

# First turn: customer asks about order
print("Turn 1: Check order status")
result = await Runner.run(
    agent,
    "What's the status of order ORD-123?",
    session=session
)
print(f"Agent: {result.final_output}")
print(f"Tokens used: {result.context_wrapper.usage.total_tokens}")

# Store usage metrics
await session.store_run_usage(result)

# Second turn: customer asks follow-up
print("\nTurn 2: Follow-up question")
result = await Runner.run(
    agent,
    "When exactly will it arrive?",
    session=session
)
print(f"Agent: {result.final_output}")
await session.store_run_usage(result)
```

**Output:**
```
Turn 1: Check order status
Agent: I'll check that for you. Order ORD-123 is on its way! It shipped on December 15 and is expected to arrive on December 20.
Tokens used: 287

Turn 2: Follow-up question
Agent: Based on the shipping information I reviewed earlier, December 20 is the expected delivery date. That's about 5 days from now.
Tokens used: 156
```

### Analyzing Conversation Metrics

`AdvancedSQLiteSession` tracks detailed usage statistics:

```python
# Get aggregated session statistics
session_stats = await session.get_session_usage()
if session_stats:
    print(f"Session Statistics:")
    print(f"  Total requests: {session_stats['requests']}")
    print(f"  Total tokens: {session_stats['total_tokens']}")
    print(f"  Input tokens: {session_stats['input_tokens']}")
    print(f"  Output tokens: {session_stats['output_tokens']}")
    print(f"  Total turns: {session_stats['total_turns']}")

# Get turn-by-turn breakdown
turns = await session.get_turn_usage()
print(f"\nTokens per turn:")
for turn_data in turns:
    turn_num = turn_data["user_turn_number"]
    tokens = turn_data["total_tokens"]
    print(f"  Turn {turn_num}: {tokens} tokens")

# Get tool usage statistics
tool_usage = await session.get_tool_usage()
print(f"\nTool usage:")
for tool_name, call_count, turn_num in tool_usage:
    print(f"  {tool_name}: called {call_count} times in turn {turn_num}")
```

**Output:**
```
Session Statistics:
  Total requests: 2
  Total tokens: 443
  Input tokens: 234
  Output tokens: 209
  Total turns: 2

Tokens per turn:
  Turn 1: 287 tokens
  Turn 2: 156 tokens

Tool usage:
  check_order_status: called 1 times in turn 1
```

This data helps you understand agent efficiency. Turn 1 cost more because the agent had to understand the request and call a tool. Turn 2 cost less because the agent could reference context from Turn 1.

### Conversation Branching: Exploring Alternative Paths

When a customer asks "What if I chose a different shipping option?" you want to explore that alternative without losing the original conversation. That's where branching comes in.

```python
print("=== Original Conversation Path ===")
print("Turn 1: Check order status")
result = await Runner.run(
    agent,
    "What's the status of order ORD-123?",
    session=session
)
print(f"Agent: {result.final_output}")
await session.store_run_usage(result)

print("\nTurn 2: Customer considers alternatives")
result = await Runner.run(
    agent,
    "If I had chosen standard shipping instead of express, what would the cost be?",
    session=session
)
print(f"Agent: {result.final_output}")
await session.store_run_usage(result)

print("\n=== Creating Alternative Path from Turn 1 ===")
# Show available turns for branching
turns = await session.get_conversation_turns()
print("Available turns to branch from:")
for turn in turns:
    print(f"  Turn {turn['turn']}: {turn['content'][:60]}...")

# Create branch from turn 1 (the initial status check)
branch_id = await session.create_branch_from_turn(1)
print(f"\nCreated new branch: {branch_id}")
print("This branch has Turn 1 but not Turn 2")

# Continue conversation in new branch
print("\nTurn 2 (new branch): Different question")
result = await Runner.run(
    agent,
    "Actually, I want to change the shipping address. Is that possible?",
    session=session
)
print(f"Agent: {result.final_output}")
await session.store_run_usage(result)

print("\n=== Switching Between Branches ===")
# Switch back to original branch
await session.switch_to_branch("main")
main_items = len(await session.get_items())
print(f"Main branch has {main_items} total items")

# Switch to alternative branch
await session.switch_to_branch(branch_id)
alt_items = len(await session.get_items())
print(f"Alternative branch has {alt_items} total items")

print("\nBranches are completely independent:")
print("- Main branch: Explored cost comparison")
print("- Alt branch: Addressed shipping address change")
print("- No interference between paths")
```

**Output:**
```
=== Original Conversation Path ===
Turn 1: Check order status
Agent: I'll check that for you. Order ORD-123 is on its way! It shipped on December 15 and is expected to arrive on December 20.

Turn 2: Customer considers alternatives
Agent: With standard shipping, your order would have cost $20 less and arrived around December 28. Express adds $25 to the base price.

=== Creating Alternative Path from Turn 1 ===
Available turns to branch from:
  Turn 1: What's the status of order ORD-123?

Created new branch: branch_1702858234
This branch has Turn 1 but not Turn 2

Turn 2 (new branch): Different question
Agent: Yes, absolutely! But since your order already shipped on December 15, we can't modify the address for this one. However, I can help you arrange a redirect with the carrier or update your address for future orders.

=== Switching Between Branches ===
Main branch has 4 items
Alternative branch has 3 items

Branches are completely independent:
- Main branch: Explored cost comparison
- Alt branch: Addressed shipping address change
- No interference between paths
```

Branching is powerful for customer support systems. When a customer says "Let me think about that option," you can create a branch to explore it without disturbing the original conversation flow. If they like the new path, you merge it. If they prefer the original, you switch back.

## Designing Stateful Agent Systems for Multiple Users

When you're building production systems, you need to manage sessions for many customers simultaneously.

### Session Organization Pattern

```python
from agents import Agent, Runner, SQLiteSession
from datetime import datetime

# Your application receives messages from different customers
customers = [
    {"id": "cust-alice-123", "name": "Alice"},
    {"id": "cust-bob-456", "name": "Bob"},
    {"id": "cust-carol-789", "name": "Carol"},
]

# Create an agent once (reuse across all customers)
agent = Agent(
    name="Support",
    instructions="Help customers with their issues. Be concise."
)

# Store sessions in a dictionary (in production: use a database)
sessions = {}

async def handle_customer_message(customer_id: str, message: str) -> str:
    """Process a message from a customer, maintaining their conversation history."""

    # Get or create session for this customer
    if customer_id not in sessions:
        sessions[customer_id] = SQLiteSession(
            session_id=customer_id,
            db_path=f"sessions/{customer_id}.db"
        )

    session = sessions[customer_id]

    # Run agent with customer's persistent session
    result = await Runner.run(agent, message, session=session)

    # Log the interaction
    print(f"[{datetime.now().isoformat()}] {customer_id}: {message}")
    print(f"Agent: {result.final_output}\n")

    return result.final_output

# Process messages from multiple customers
# Each customer gets their own conversation history
await handle_customer_message("cust-alice-123", "I can't log into my account")
await handle_customer_message("cust-bob-456", "What's my order status?")
await handle_customer_message("cust-alice-123", "I tried resetting my password but it didn't work")
# Alice's second message includes context from her first message
```

**Output:**
```
[2025-01-20T14:32:15] cust-alice-123: I can't log into my account
Agent: I'm sorry you're having trouble logging in. Let me help. Can you tell me what error message you're seeing?

[2025-01-20T14:32:45] cust-bob-456: What's my order status?
Agent: I'll help you find your order status. Can you provide your order number?

[2025-01-20T14:33:22] cust-alice-123: I tried resetting my password but it didn't work
Agent: Based on our earlier conversation, I see you're having login issues. Let me investigate why the password reset didn't work. This is unusual—let me escalate this to our account recovery team.
```

Key insight: One agent instance handles all customers. Each customer has their own session. The agent refers to the session when responding, so it maintains separate context for each person.

## Try With AI

Use your configured agent environment from Lesson 5. If you haven't set up LiteLLM with an alternative model, use the OpenAI setup.

### Prompt 1: Implement Basic Session Memory

**Setup**: Your TaskManager agent from previous lessons

```
I want my agent to remember across conversation turns.

Here's my current setup:
- I have a TaskManager agent with add_task, list_tasks, and complete_task tools
- Right now each Runner.run() call loses context from previous calls

Please show me how to add SQLiteSession so the agent remembers:
1. What tasks were created in previous turns
2. What the current task list is
3. What tasks were completed

Include a test that shows the agent remembering across 3 turns.
```

**Expected outcome**: Session that persists task context across multiple interactions, demonstrating memory through tool references ("I see we created 'Buy groceries' earlier")

### Prompt 2: Track Token Usage for Cost Optimization

**Setup**: Your TaskManager with AdvancedSQLiteSession now active

```
I want to understand the cost of my agent interactions.

Can you:
1. Add store_run_usage() calls to track tokens for each turn
2. Show me how to get session_usage() to see total tokens across all turns
3. Help me identify which turn used the most tokens and why
4. Create a simple cost calculation (assuming $0.01 per 1000 tokens)

I want to see: total tokens, tokens per turn, and the most expensive interaction.
```

**Expected outcome**: Usage analytics dashboard showing token consumption across conversation turns, enabling cost optimization decisions

### Prompt 3: Create a Conversation Branch and Compare Paths

**Setup**: Your TaskManager with a few completed conversation turns

```
I want to explore "what if" scenarios without losing my original conversation.

Starting from the point where we've added 3 tasks:
1. Create a branch from that turn
2. In the branch, ask the agent a different question (e.g., "What if we only mark high-priority tasks as important?")
3. Switch back to the main branch
4. Switch to the alternative branch
5. Show me that both branches exist independently

This will help me test different customer support scenarios without restarting.
```

**Expected outcome**: Demonstration that branching preserves referential integrity—the branch can reference the original context while pursuing alternative paths, enabling complex customer support workflows

