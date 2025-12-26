---
sidebar_position: 4
title: "Handoffs & Message Filtering"
description: "Master agent-to-agent handoffs with context passing, implement on_handoff callbacks for dynamic context injection, and filter conversation history before handing off to specialized agents."
keywords: ["handoffs", "agent orchestration", "message filtering", "context injection", "on_handoff callback", "HandoffInputData", "input_filter", "handoff_filters utility"]
chapter: 34
lesson: 4
duration_minutes: 90

skills:
  - name: "Multi-Agent Handoff Architecture"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "System Design"
    measurable_at_this_level: "Student can design agent relationships where control passes cleanly between specialized agents"

  - name: "Handoff Implementation with handoff()"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can use handoff() to register target agents and basic callbacks"

  - name: "on_handoff Callback for Context Injection"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can inject runtime data into context before control transfers to specialist agent"

  - name: "Message History Filtering with input_filter"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Data Management"
    measurable_at_this_level: "Student can remove tool calls, trim history, or selectively preserve context for specialized agents"

  - name: "HandoffInputData Structure"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Technical Design"
    measurable_at_this_level: "Student understands input_history, pre_handoff_items, and new_items components"

  - name: "handoff_filters Utilities"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can use provided utilities like remove_all_tools() to simplify filtering logic"

learning_objectives:
  - objective: "Design multi-agent systems where specialized agents hand off work to each other"
    proficiency_level: "B2"
    bloom_level: "Design"
    assessment_method: "Student creates agent network with 3+ agents handing off based on context"

  - objective: "Implement handoff callbacks that inject domain-specific context before transfer"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "on_handoff callback successfully injects data that specialist agent uses"

  - objective: "Filter message history strategically to reduce noise without losing critical context"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Input filter demonstrates understanding of what context specialist needs vs doesn't need"

  - objective: "Understand when handoffs are necessary vs when agents-as-tools provide better orchestration"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Student articulates tradeoffs between handoff vs agent-as-tool patterns"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (handoff list, handoff() function, on_handoff callback, HandoffInputData, input_filter, handoff_filters utilities) within B1-B2 limit of 10 concepts - PASS"

differentiation:
  extension_for_advanced: "Implement complex filtering logic that preserves context by theme; design cascading handoffs (agent A → B → C); combine handoffs with structured output validation"
  remedial_for_struggling: "Start with basic handoff list without callbacks; add on_handoff after handoff pattern is comfortable; skip advanced filtering initially"

generated_by: content-implementer
source_spec: specs/047-ch34-openai-agents-sdk
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# Handoffs & Message Filtering

**Scenario**: You've built a customer support system with three specialized agents: a triage agent who greets customers and understands their problem, a billing agent who handles payment questions, and a technical support agent who debugs issues.

When a customer says "I want to change my billing address," the triage agent *could* handle it. But the billing agent is the expert. So triage *hands off* control—passing the conversation history forward.

But here's the question: Should the billing agent see *everything* that was discussed? The tools the triage agent used? The failed attempts? Or should the handoff clean up the history, keeping only what matters?

This is where handoffs and message filtering become critical. They're how you scale agent systems from one agent to coordinated teams.

## The Problem Handoffs Solve

In Lesson 3, you learned about context objects that persist state within a single agent's lifecycle. But what happens when you need *multiple* specialized agents working together?

Consider an airline customer service system:

- **Triage Agent**: "Hello! How can I help? Are you asking about baggage, seating, or something else?"
- **Customer**: "I want to change my seat"
- **Triage Agent**: (recognizes this requires seat booking expertise, hands off)
- **Seat Booking Agent**: (takes over, can now focus on seat updates without distraction)

Without handoffs, you'd have one agent trying to do everything—triage, FAQ lookup, seat booking, refunds. The agent gets confused switching contexts. You'd need massive instructions explaining every scenario.

With handoffs, you have focused agents, each excellent at one job. When their expertise is no longer relevant, they pass control to the expert.

## Handoff Fundamentals

A handoff in the OpenAI Agents SDK has three components:

### 1. Register the Target Agent

```python
from agents import Agent

billing_agent = Agent(
    name="Billing Agent",
    instructions="Handle all billing-related questions."
)

triage_agent = Agent(
    name="Triage Agent",
    instructions="Route customers to the appropriate specialist.",
    handoffs=[billing_agent]  # Can hand off to billing agent
)
```

The `handoffs` list tells the triage agent: "You have the option to hand off to this agent."

**Output:**
```
# triage_agent now has the capability to recognize when a customer
# needs billing help and hand off control to billing_agent
```

### 2. The Agent Decides When to Handoff

You don't manually trigger handoffs. The agent reads its instructions and decides:

```python
instructions = """You are a triage agent. Route customers to:
- Billing Agent if they ask about payment, refunds, or billing address
- Technical Support if they ask about technical issues
- FAQ Agent if they ask general questions about our products
"""
```

When the agent's instructions say "use the Billing Agent for payment questions," and a customer asks about payment, the agent *chooses* to handoff.

**What happens during handoff**:
1. Agent recognizes: "This customer needs billing expertise"
2. Agent calls: `handoff_to_billing_agent()`
3. Control transfers: Triage agent stops, billing agent takes over
4. Conversation history: All previous messages pass to the new agent
5. Context object: The shared context (like customer name, account ID) transfers too

**Output**:
```
# Agent's reasoning process (visible in logs):
"User is asking about payment methods. This requires the Billing Agent's expertise.
Initiating handoff..."

# Handoff triggered → Control transfers to billing_agent
```

### 3. The New Agent Continues

The specialist agent receives the conversation and context:

```python
# Before handoff
Customer: "I'd like to change my billing address"
Triage Agent: "I'll connect you to our billing specialist"

# After handoff (billing agent sees this history)
Billing Agent: (sees previous messages) "I can help with that.
              What's your new address?"
```

**Output**:
```
# Conversation flows seamlessly across the handoff
Triage Agent: "I'll connect you to our billing specialist"
[HANDOFF: triage_agent → billing_agent]
Billing Agent: "I can help with that. What's your new address?"
Customer: "123 New Street"
Billing Agent: "Address updated to 123 New Street. Anything else?"
```

## Implementing Handoffs

### Basic Pattern

```python
from agents import Agent, Runner

# Specialist agents
spanish_agent = Agent(
    name="Spanish Assistant",
    instructions="You only speak Spanish.",
    handoff_description="A Spanish-speaking assistant."  # Important for routing
)

english_agent = Agent(
    name="English Assistant",
    instructions="Help customers in English. If they speak Spanish, handoff to Spanish assistant.",
    handoffs=[spanish_agent]
)

# Run
result = await Runner.run(english_agent, "Hola, necesito ayuda")
# english_agent recognizes Spanish → hands off to spanish_agent
```

**Output:**
```
spanish_agent: "Hola! En qué puedo ayudarte?"
```

**Key principle**: The agent decides when to handoff based on its instructions. You don't explicitly tell it "hand off now." Instead, you describe the specialist's expertise in `handoff_description`, and the agent uses that in its reasoning.

## Injecting Context at Handoff Time

Often, you need to inject fresh data *right at the moment of handoff*. For example, before an agent can process a seat booking, it needs to know the flight number.

This is what the `on_handoff` callback does:

### The on_handoff Callback

```python
from agents import handoff, RunContextWrapper

async def inject_flight_data(context: RunContextWrapper[AirlineContext]) -> None:
    """Called right before handing off to seat booking agent.

    Injects dynamic data that the specialist needs.
    """
    flight_number = f"FLT-{random.randint(100, 999)}"
    context.context.flight_number = flight_number
    print(f"Injected flight number: {flight_number}")

seat_booking_agent = Agent(
    name="Seat Booking Agent",
    instructions="Update the customer's seat using the flight_number in context."
)

triage_agent = Agent(
    name="Triage Agent",
    instructions="Route to seat booking specialist for seat changes.",
    handoffs=[
        handoff(
            agent=seat_booking_agent,
            on_handoff=inject_flight_data  # Called right before transfer
        )
    ]
)
```

**Output:**
```
Injected flight number: FLT-847
# seat_booking_agent receives context with flight_number populated
# seat_booking_agent.tools can now use context.context.flight_number
```

**How it works**:
1. Customer asks: "I want to change my seat"
2. Triage agent recognizes: "This needs seat booking agent"
3. Before handoff: `on_handoff` callback runs → injects flight number into context
4. Handoff: Control transfers to seat booking agent
5. Seat booking agent: Uses `context.context.flight_number` in its tools

This is powerful because:
- You can inject data based on the *current time* (not available when agent was initialized)
- You can add session-specific data (which flight are they on)
- You can fetch data from external sources right before specialist takes over
- The specialist agent never needs to ask for these details

## Message History Filtering

Here's a problem: After 10 exchanges between a customer and the triage agent, the conversation history is long. The triage agent used tools—searched the FAQ, looked up account info, checked system status. All those tool calls are in the history.

When handing off to the billing specialist, do they need to see:
- All the tool calls? (Probably not—they don't use those tools)
- All the preliminary back-and-forth? (Maybe not—they only care about the final decision)
- The customer's name and account history? (Yes—essential)

Message filtering lets you clean up the handoff to give the specialist only what they need.

### Understanding HandoffInputData

When a handoff occurs, the SDK passes a `HandoffInputData` object to your filter:

```python
class HandoffInputData:
    input_history: tuple[Message]  # All previous messages in the conversation
    pre_handoff_items: tuple[Item]  # Items from before the handoff decision
    new_items: tuple[Item]         # Items from the agent deciding to handoff
```

Think of it like:
- **input_history**: The entire conversation transcript
- **pre_handoff_items**: Actions the previous agent took (tools, reasoning)
- **new_items**: The final decision and handoff message

### Basic Filtering: Remove Tool Calls

```python
from agents import handoff, HandoffInputData
from agents.extensions import handoff_filters

def spanish_handoff_filter(data: HandoffInputData) -> HandoffInputData:
    """Remove tool calls from history before handing to Spanish agent."""

    # Use utility function to remove all tool-related messages
    filtered = handoff_filters.remove_all_tools(data)

    # Return cleaned data
    return HandoffInputData(
        input_history=filtered.input_history,
        pre_handoff_items=tuple(filtered.pre_handoff_items),
        new_items=tuple(filtered.new_items),
    )

spanish_agent = Agent(
    name="Spanish Assistant",
    instructions="You only speak Spanish."
)

english_agent = Agent(
    name="English Assistant",
    instructions="Route Spanish speakers to specialist.",
    handoffs=[
        handoff(spanish_agent, input_filter=spanish_handoff_filter)
    ]
)
```

**Output** (without filter):
```
[tool call] lookup_faq(query="What languages do you support?")
[tool response] "English, Spanish, French"
[message] "I'll connect you to our Spanish specialist"
```

**Output** (with filter applied):
```
[message] "I'll connect you to our Spanish specialist"
# Tool calls removed—cleaner context for Spanish agent
```

### Advanced Filtering: Trim History

Sometimes you want to keep just the most recent exchanges and discard older context:

```python
def trim_history_filter(data: HandoffInputData) -> HandoffInputData:
    """Keep only the last 5 messages in history."""

    # Get the last 5 messages
    trimmed_history = data.input_history[-5:] if len(data.input_history) > 5 else data.input_history

    return HandoffInputData(
        input_history=trimmed_history,
        pre_handoff_items=tuple(data.pre_handoff_items),
        new_items=tuple(data.new_items),
    )

triage_agent = Agent(
    ...
    handoffs=[
        handoff(specialist_agent, input_filter=trim_history_filter)
    ]
)
```

**Output**:
```
# Original history: 12 messages (customer's initial question + 11 followups)
# After filter: Last 5 messages only (most relevant context)
# Specialist sees only recent context, not full conversation history
```

### Combination: Remove Tools AND Trim

```python
def smart_filter(data: HandoffInputData) -> HandoffInputData:
    """Remove tools, then keep only last 10 messages."""

    # Step 1: Remove tool calls
    no_tools = handoff_filters.remove_all_tools(data)

    # Step 2: Trim history
    trimmed_history = no_tools.input_history[-10:] if len(no_tools.input_history) > 10 else no_tools.input_history

    return HandoffInputData(
        input_history=trimmed_history,
        pre_handoff_items=tuple(no_tools.pre_handoff_items),
        new_items=tuple(no_tools.new_items),
    )

agent = Agent(
    ...
    handoffs=[
        handoff(specialist, input_filter=smart_filter)
    ]
)
```

**Output**:
```
# Before filter: 15 messages with tool calls
# Step 1 (remove tools): 15 messages, no tool calls
# Step 2 (trim): Last 10 messages, no tool calls
# Final: Specialist receives clean, relevant context
```

## Complete Example: Airline Customer Service

Let's tie this together. A real airline system with triage routing to FAQ and seat booking specialists:

### Step 1: Define Context

```python
from pydantic import BaseModel

class AirlineContext(BaseModel):
    passenger_name: str | None = None
    confirmation_number: str | None = None
    seat_number: str | None = None
    flight_number: str | None = None
```

**Output:**
```
# Context structure ready to be populated during handoffs
```

### Step 2: Create Tools

```python
from agents import function_tool, RunContextWrapper

@function_tool
async def faq_lookup_tool(question: str) -> str:
    """Answer frequently asked questions about baggage, seating, wifi."""
    if "baggage" in question.lower():
        return "One checked bag allowed, max 50 lbs."
    elif "seat" in question.lower():
        return "Exit rows: 4 and 16. Economy Plus (5-8) has extra legroom."
    return "I don't know the answer."

@function_tool
async def update_seat(
    context: RunContextWrapper[AirlineContext],
    confirmation_number: str,
    new_seat: str
) -> str:
    """Update the passenger's seat."""
    context.context.confirmation_number = confirmation_number
    context.context.seat_number = new_seat
    assert context.context.flight_number is not None, "Flight number must be set"
    return f"Seat updated to {new_seat}"
```

**Output:**
```
# Tools ready. Note: update_seat requires flight_number to be in context.
# This will be injected via on_handoff callback before handoff occurs.
```

### Step 3: Create Agents with Handoffs

```python
from agents import Agent, handoff

# FAQ specialist
faq_agent = Agent[AirlineContext](
    name="FAQ Agent",
    instructions="Answer questions about baggage, seating, wifi using the FAQ tool.",
    tools=[faq_lookup_tool],
)

# Seat booking specialist
async def inject_flight_number(context: RunContextWrapper[AirlineContext]):
    """Inject flight number before seat booking."""
    import random
    context.context.flight_number = f"FLT-{random.randint(100, 999)}"

seat_booking_agent = Agent[AirlineContext](
    name="Seat Booking Agent",
    instructions="Help passengers update their seat using the update_seat tool.",
    tools=[update_seat],
)

# Triage agent orchestrates
triage_agent = Agent[AirlineContext](
    name="Triage Agent",
    instructions="""You're an airline customer service specialist.
    Route to FAQ Agent for questions about baggage, seating, wifi, policies.
    Route to Seat Booking Agent for seat changes (confirm number and new seat).
    """,
    handoffs=[
        faq_agent,
        handoff(seat_booking_agent, on_handoff=inject_flight_number)
    ],
)

# Allow agents to hand back to triage if needed
faq_agent.handoffs.append(triage_agent)
seat_booking_agent.handoffs.append(triage_agent)
```

**Output:**
```
# Three agents created with handoff routes:
# triage ← → faq
# triage ← → seat_booking (with flight number injection)
```

### Step 4: Run the System

```python
from agents import Runner

async def main():
    context = AirlineContext()
    input_items = []
    current_agent = triage_agent

    # Simulate customer conversation
    exchanges = [
        "Hi, I have questions about baggage",
        "What's the weight limit?",
        "Actually, I want to change my seat instead"
    ]

    for message in exchanges:
        input_items.append({"role": "user", "content": message})
        result = await Runner.run(current_agent, input_items, context=context)

        # Print response
        print(f"Agent: {result.last_agent.name}")
        print(f"Response: {result.final_output}\n")

        input_items = result.to_input_list()
        current_agent = result.last_agent

# Run
asyncio.run(main())
```

**Output:**
```
Agent: FAQ Agent
Response: One checked bag allowed, max 50 lbs.

Agent: Triage Agent
Response: I can help with that. What's your confirmation number?

Agent: Seat Booking Agent
Response: Seat updated to 12C
# (flight_number was injected by on_handoff callback)
```

## Key Design Patterns

### Handoff vs Agent-as-Tool

You have two ways to use multiple agents:

**Pattern 1: Handoff (Transfer Control)**
- Triage agent hands off to specialist
- Specialist has full control of conversation
- Best for: Customer support, sequential workflows

```python
triage_agent = Agent(
    handoffs=[specialist_agent]  # Transfer control
)
```

**Pattern 2: Agent-as-Tool (Orchestrated)**
- Manager agent calls specialist as a tool
- Manager retains control throughout
- Best for: Parallel workflows, research coordination

```python
manager_agent = Agent(
    tools=[
        specialist1.as_tool("research", "Research topic"),
        specialist2.as_tool("write", "Write content")
    ]
)
```

**Which to use:**
- Handoff: One conversation, one customer, sequential expertise needed
- Agent-as-Tool: Multiple tasks happening in parallel, manager coordinates

## When Message Filtering Matters

You should filter messages when:

1. **Tool calls are noise**: The specialist doesn't use the same tools
2. **History is long**: More than 10 exchanges, specialist only needs recent context
3. **Context is sensitive**: You want to redact certain information
4. **Agent confusion**: Specialist is getting distracted by irrelevant history

You should keep full history when:

1. **Specialist needs context**: They need to understand the whole problem
2. **History is short**: Less than 5 exchanges, no filtering needed
3. **Continuity matters**: Customer expects seamless conversation

---

## Try With AI

**Setup:** Use OpenAI API key or switch to LiteLLM for free models (Claude, Gemini).

### Prompt 1: Create a Basic Handoff System

```
Create a simple agent system with:
- A triage agent that routes between two specialists
- Triage asks: "Do you need technical support or billing help?"
- Technical agent: "I'll help debug your issue"
- Billing agent: "I'll help with your account"

Test it with user input: "I want to change my payment method"
What agent handles it? How does the handoff occur?
```

**What you're learning**: How agents recognize when to handoff based on their instructions and the specialist's description.

### Prompt 2: Inject Runtime Data at Handoff

```
Take the airline example from the lesson. Implement:
- An on_handoff callback that injects a flight_number into context
- A seat_booking_agent that uses flight_number from context
- Test with: "Change my seat to 12B"

Why is it important that flight_number is injected at handoff time,
not when the triage agent was created? (Hint: When would the flight
number be known vs unknown?)
```

**What you're learning**: How on_handoff callbacks solve the problem of "specialist needs data that only exists at the moment of handoff."

### Prompt 3: Filter Message History Before Handoff

```
Implement a message filter that:
1. Removes all tool calls (using handoff_filters.remove_all_tools)
2. Keeps only the last 8 messages

Test with a conversation that has:
- 12+ total messages
- Multiple tool calls (FAQ lookups, etc.)

Show the difference: What appears in history WITHOUT filtering vs WITH filtering?
```

**What you're learning**: How filtering reduces noise and helps specialists focus on what they actually need.

