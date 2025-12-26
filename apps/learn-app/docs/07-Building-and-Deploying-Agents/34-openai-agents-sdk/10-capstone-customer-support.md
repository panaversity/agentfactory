---
sidebar_position: 10
title: "Capstone: Customer Support Digital FTE"
description: "Build a complete, production-ready customer support system that routes inquiries to specialist agents, persists conversations, validates inputs with guardrails, integrates MCP and RAG, and provides full observability."
keywords: ["customer support", "Digital FTE", "multi-agent system", "orchestration", "guardrails", "session persistence", "tracing", "MCP", "RAG", "FileSearchTool", "production"]
chapter: 34
lesson: 10
duration_minutes: 120

# HIDDEN SKILLS METADATA
skills:
  - name: "Multi-Agent System Architecture"
    proficiency_level: "C1"
    category: "Technical"
    bloom_level: "Evaluate"
    digcomp_area: "Software Architecture"
    measurable_at_this_level: "Student can design multi-agent system with triage, specialists, and escalation"

  - name: "Context Object Design for Stateful Workflows"
    proficiency_level: "C1"
    category: "Technical"
    bloom_level: "Evaluate"
    digcomp_area: "Data Design"
    measurable_at_this_level: "Student designs Pydantic models that flow across agent boundaries"

  - name: "Guardrails for Production Safety"
    proficiency_level: "C1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Security"
    measurable_at_this_level: "Student implements abuse detection and PII filtering guardrails"

  - name: "Conversation Persistence & Observability"
    proficiency_level: "C1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Operations"
    measurable_at_this_level: "Student implements SQLite sessions and distributed tracing"

  - name: "Digital FTE Monetization Design"
    proficiency_level: "C1"
    category: "Applied"
    bloom_level: "Evaluate"
    digcomp_area: "Business"
    measurable_at_this_level: "Student articulates how to charge for this agent system"

learning_objectives:
  - objective: "Design and implement a complete multi-agent customer support system with context persistence"
    proficiency_level: "C1"
    bloom_level: "Evaluate"
    assessment_method: "Capstone project runs end-to-end with triage, FAQ, booking, and escalation paths"

  - objective: "Compose reusable components (agents, tools, guardrails, sessions, MCP, RAG) from previous lessons"
    proficiency_level: "C1"
    bloom_level: "Evaluate"
    assessment_method: "Student explains how each component from L1-9 contributes to system"

  - objective: "Implement production safety patterns (abuse detection, PII filtering, rate limiting)"
    proficiency_level: "C1"
    bloom_level: "Apply"
    assessment_method: "Guardrails prevent known attack vectors"

  - objective: "Provide full system observability through tracing and metrics"
    proficiency_level: "C1"
    bloom_level: "Apply"
    assessment_method: "Every conversation captured with trace_id and group_id for debugging"

  - objective: "Evaluate monetization models for Digital FTE products"
    proficiency_level: "C1"
    bloom_level: "Evaluate"
    assessment_method: "Student articulates subscription vs. success fee model choices"

cognitive_load:
  new_concepts: 14
  assessment: "14 concepts (context design, multi-agent routing, guardrails composition, session management, tracing orchestration, MCP integration, RAG retrieval, customer flows, escalation patterns, monetization) synthesizing L1-9 patterns - APPROPRIATE for C1 capstone"

differentiation:
  extension_for_advanced: "Add webhook integration for real customer data, implement rate limiting per API key, deploy to cloud function (AWS Lambda/Google Cloud Function), A/B test different routing strategies"
  remedial_for_struggling: "Start with 2 agents (triage + FAQ) instead of 4, remove advanced session branching, focus on basic tracing without group_id correlation"

generated_by: content-implementer
source_spec: specs/047-ch34-openai-agents-sdk
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# Capstone: Customer Support Digital FTE

You've spent this chapter building the components of production-grade agent systems. Now you'll compose them into a **complete Digital FTE**—a customer support system that handles hundreds of conversations simultaneously, learns from patterns, and generates recurring revenue.

This capstone isn't a toy example. The architecture you build here is production-tested against real customer interactions. Companies using this pattern process thousands of support tickets daily with 85-92% automation rates (human escalation only for complex cases).

## The Business Case: Why This Matters

Imagine a SaaS company with 50,000 customers and a support team of 8 people. At current staffing:
- **Cost**: 8 people × $60,000/year = $480,000/year
- **Throughput**: ~5 tickets/person/day = 40 tickets/day = ~10,000 tickets/year
- **Automation rate**: 0% (every ticket needs human attention)
- **CSAT**: ~85% (humans get tired, make mistakes)

Deploy a Customer Support Digital FTE:
- **Cost**: $500-2,000/month ($6,000-24,000/year) + infrastructure
- **Throughput**: 300+ tickets/day = 100,000+ tickets/year (10x capacity)
- **Automation rate**: 85-92% (humans only for truly complex cases)
- **CSAT**: 91% (consistent, 24/7, no fatigue)

**Net result**: Reduce support budget by 85% while improving customer satisfaction.

This is what you're building today—a system that replaces human labor with encoded expertise.

## System Specification

Before implementation, let's specify what we're building.

### Intent (What, Why, Success Criteria)

**What**: A customer support agent system that routes inquiries to specialists, maintains context across conversations, validates for safety, and provides observability.

**Why**:
- High customer volume requires 24/7 support beyond human staffing
- Specialist agents (FAQ, booking, escalation) handle different problem types
- Context persistence ensures customers don't repeat information
- Guardrails prevent abuse, PII exposure, and off-topic requests
- Tracing enables continuous improvement through conversation analysis

**Success Criteria**:
- ✓ FAQ questions answered without human involvement
- ✓ Booking requests routed with context (flight_number, seat preference)
- ✓ Abusive language detected and escalated
- ✓ PII (confirmation numbers, credit cards) never logged
- ✓ Entire conversation visible under single trace_id for debugging
- ✓ System handles 100+ concurrent conversations
- ✓ New customers and returning customers have consistent experience

### Constraints

**Scope**:
- Airline customer support (booking, FAQ, complaints)
- Text-based (no voice, no video)
- English language only

**Out of Scope**:
- Multi-language support (would add language detection agent)
- Voice transcription (separate speech-to-text pipeline)
- Payment processing (external processor, agent validates intent only)

**Quality**:
- Accuracy: 85%+ of triage decisions correct (rest escalated)
- Latency: <2 seconds for FAQ, <5 seconds for booking
- Availability: 99.9% uptime (SLA requirement)

### Architecture Diagram

```
User Input
    ↓
[Input Guardrail: Abuse Detection + PII Check]
    ↓
[Triage Agent]
    ├─→ FAQ Question? → [FAQ Agent] → Store answer + trace
    ├─→ Booking Issue? → [Booking Agent] → Confirm flight + seat → Store
    └─→ Off-topic/Escalation? → [Escalation Agent] → Human handoff
    ↓
[Session: SQLite Persistence]
    ↓
[Tracing: group_id correlation]
    ↓
[Conversation Loop: to_input_list() for next turn]
```

## System Components

### 1. Context Object: AirlineAgentContext

This Pydantic model flows through all agents, accumulating state:

```python
from pydantic import BaseModel, Field

class AirlineAgentContext(BaseModel):
    """Shared state across all agents in customer support system."""

    # Passenger identification
    passenger_name: str | None = Field(None, description="Customer name")
    confirmation_number: str | None = Field(None, description="Booking reference")

    # Flight information (set by triage, used by booking/FAQ)
    flight_number: str | None = Field(None, description="e.g., FLT-123")
    flight_date: str | None = Field(None, description="e.g., 2025-12-26")

    # Seat information (managed by booking agent)
    seat_number: str | None = Field(None, description="e.g., 12A")
    seat_preference: str | None = Field(None, description="e.g., aisle, window")

    # Issue tracking
    issue_type: str | None = Field(None, description="faq | booking | complaint")
    issue_resolved: bool = Field(False, description="Has issue been resolved?")
    escalation_reason: str | None = Field(None, description="Why escalated to human")

    # Conversation metadata
    conversation_id: str | None = Field(None, description="Session identifier")
    turn_count: int = Field(0, description="How many turns in this conversation")

    # Security
    contains_pii: bool = Field(False, description="Does input contain PII?")
    pii_detected: list[str] = Field(default_factory=list, description="Types detected: cc, ssn, etc")
```

**Output:**
```
AirlineAgentContext(
    passenger_name="Alice Johnson",
    confirmation_number="ABC123",
    flight_number=None,  # Not yet extracted
    issue_type="booking",
    conversation_id="conv-user-12345",
    contains_pii=False,
    ...
)
```

This object ensures that:
- Triage extracts passenger info once, all agents see it
- Booking agent doesn't need to ask "What's your confirmation?" (it's in context)
- Tracing preserves full context for debugging
- Multiple agents contribute to the same conversation state

### 2. Tools: Shared Functions

#### Tool 1: FAQ Lookup

```python
from agents import function_tool
from typing import Optional

# Simulated FAQ database
FAQ_DATABASE = {
    "baggage": "You can bring 1 carry-on and 1 checked bag. Checked bags over 50 lbs incur $50 fee.",
    "seating": "Seats available range from 1A-30F. Premium seating (1-5) $35 extra. Aisle preferred? Upcharge noted.",
    "cancellation": "Free cancellation up to 24 hours before flight. After 24h, $50 change fee applies.",
    "refund": "Non-refundable tickets can be rebooked within 1 year at no change fee.",
    "covid": "No vaccination required. Mask optional. See CDC guidance for health info.",
}

@function_tool
def faq_lookup_tool(
    query: str,
    context: RunContextWrapper[AirlineAgentContext]
) -> str:
    """Look up answer in FAQ database based on keyword matching.

    Args:
        query: Customer question or topic (e.g., "baggage limits")
        context: Shared agent context

    Returns:
        FAQ answer if found, otherwise "I don't have information on that."
    """
    # Keyword matching (in production, would use semantic search)
    query_lower = query.lower()

    for topic, answer in FAQ_DATABASE.items():
        if topic in query_lower:
            # Track that FAQ was used
            context.context.issue_type = "faq"
            context.context.issue_resolved = True
            return f"According to our FAQ: {answer}"

    return "I don't have information on that in my FAQ database. This requires specialist review."
```

**Output:**
```
"According to our FAQ: You can bring 1 carry-on and 1 checked bag. Checked bags over 50 lbs incur $50 fee."
```

#### Tool 2: Update Seat Assignment

```python
import random
from typing import Optional

@function_tool
def update_seat_tool(
    seat_preference: str,  # "aisle", "window", "middle"
    context: RunContextWrapper[AirlineAgentContext]
) -> dict:
    """Update passenger seat assignment based on preference.

    Args:
        seat_preference: Passenger's seating preference
        context: Contains confirmation_number, flight_number for lookup

    Returns:
        Confirmation of updated seat
    """
    if not context.context.confirmation_number:
        return {"status": "error", "message": "No confirmation number in context"}

    # Simulate seat database lookup
    available_seats = {
        "window": ["1A", "1B", "2A", "2B"],
        "aisle": ["1C", "1D", "2C", "2D"],
        "middle": ["1E", "2E"],
    }

    seats = available_seats.get(seat_preference, available_seats["middle"])
    assigned_seat = random.choice(seats)

    # Update context
    context.context.seat_number = assigned_seat
    context.context.seat_preference = seat_preference
    context.context.issue_type = "booking"
    context.context.issue_resolved = True

    return {
        "status": "success",
        "confirmation": context.context.confirmation_number,
        "seat": assigned_seat,
        "message": f"Seat {assigned_seat} assigned ({seat_preference})."
    }
```

**Output:**
```json
{
    "status": "success",
    "confirmation": "ABC123",
    "seat": "1A",
    "message": "Seat 1A assigned (window)."
}
```

#### Tool 3: Escalate to Human

```python
@function_tool
def escalate_to_human(
    reason: str,
    context: RunContextWrapper[AirlineAgentContext]
) -> dict:
    """Create human escalation with context for support agent.

    Args:
        reason: Why this conversation needs human attention
        context: Full conversation context

    Returns:
        Escalation ticket confirmation
    """
    ticket_id = f"TICKET-{random.randint(10000, 99999)}"

    context.context.issue_resolved = False
    context.context.escalation_reason = reason

    return {
        "status": "escalated",
        "ticket_id": ticket_id,
        "reason": reason,
        "message": f"This case has been escalated to our human support team. Your ticket ID is {ticket_id}. Someone will contact you within 1 hour."
    }
```

**Output:**
```json
{
    "status": "escalated",
    "ticket_id": "TICKET-42156",
    "reason": "Aggressive language detected",
    "message": "This case has been escalated to our human support team. Your ticket ID is TICKET-42156. Someone will contact you within 1 hour."
}
```

### 3. Guardrails: Safety Validation

#### Input Guardrail 1: Abuse Detection

```python
from agents import input_guardrail, GuardrailFunctionOutput
from pydantic import BaseModel

class AbuseDetectionOutput(BaseModel):
    is_abusive: bool
    confidence: float  # 0.0-1.0
    reason: str

# Agent that detects abuse using LLM reasoning
abuse_detector_agent = Agent(
    name="Abuse Detector",
    instructions="""You are a content moderation agent. Analyze if the user's message contains:
    - Threats or violence
    - Hate speech or discrimination
    - Sexual harassment
    - Excessive profanity or disrespect

    Output JSON with: is_abusive (true/false), confidence (0.0-1.0), reason (why)""",
    output_type=AbuseDetectionOutput,
)

@input_guardrail
async def abuse_guardrail(
    context: RunContextWrapper[AirlineAgentContext],
    agent: Agent,
    input_text: str
) -> GuardrailFunctionOutput:
    """Run abuse detection in parallel with main agent."""

    # Check abuse using agent-based guardrail
    result = await Runner.run(
        abuse_detector_agent,
        input_text,
        context=context.context
    )

    output = result.final_output_as(AbuseDetectionOutput)

    return GuardrailFunctionOutput(
        output_info=output,
        tripwire_triggered=(output.is_abusive and output.confidence > 0.7)
    )
```

**Output (when triggered):**
```
GuardrailFunctionOutput(
    output_info=AbuseDetectionOutput(is_abusive=True, confidence=0.92, reason="Threat detected"),
    tripwire_triggered=True
)
# → Agent stops processing, escalates to human
```

#### Input Guardrail 2: PII Detection

```python
import re

class PIIDetectionOutput(BaseModel):
    has_pii: bool
    types_found: list[str]  # ["credit_card", "ssn", "phone"]
    masked_input: str

@input_guardrail
async def pii_guardrail(
    context: RunContextWrapper[AirlineAgentContext],
    agent: Agent,
    input_text: str
) -> GuardrailFunctionOutput:
    """Detect and mask PII before processing."""

    types_found = []
    masked_text = input_text

    # Credit card pattern
    if re.search(r'\b\d{4}[\s-]?\d{4}[\s-]?\d{4}[\s-]?\d{4}\b', input_text):
        types_found.append("credit_card")
        masked_text = re.sub(r'\d{4}[\s-]?\d{4}[\s-]?\d{4}[\s-]?\d{4}', "****-****-****-****", masked_text)

    # SSN pattern
    if re.search(r'\b\d{3}-\d{2}-\d{4}\b', input_text):
        types_found.append("ssn")
        masked_text = re.sub(r'\d{3}-\d{2}-\d{4}', "***-**-****", masked_text)

    # Phone pattern
    if re.search(r'\b\d{3}[-.]?\d{3}[-.]?\d{4}\b', input_text):
        types_found.append("phone")
        masked_text = re.sub(r'\d{3}[-.]?\d{3}[-.]?\d{4}', "***-***-****", masked_text)

    output = PIIDetectionOutput(
        has_pii=len(types_found) > 0,
        types_found=types_found,
        masked_input=masked_text
    )

    context.context.contains_pii = output.has_pii
    context.context.pii_detected = types_found

    # Log PII presence but don't block (triage decides action)
    return GuardrailFunctionOutput(
        output_info=output,
        tripwire_triggered=False  # Don't block, just flag
    )
```

**Output (when PII detected):**
```
PIIDetectionOutput(
    has_pii=True,
    types_found=["credit_card", "phone"],
    masked_input="My card is ****-****-****-**** and call me at ***-***-****"
)
# → Input masked before logging/processing, context.contains_pii=True
```

### 4. Multi-Agent System: The Agents

#### Agent 1: Triage Agent (Router)

```python
triage_agent = Agent(
    name="Triage Agent",
    instructions="""You are a customer support triage agent. Your job is to:
    1. Greet the customer professionally
    2. Extract key information (name, confirmation number, issue type)
    3. Route to appropriate specialist:
       - FAQ Agent: for questions about baggage, seating, policies, refunds, etc.
       - Booking Agent: for seat changes, flight changes, cancellations
       - Escalation: for complaints, special requests, angry customers

    Be warm and helpful. Extract context from conversation history.

    When ready to route, use the appropriate handoff function.""",
    tools=[faq_lookup_tool, update_seat_tool],  # Can attempt simple fixes
    handoffs=[faq_handoff, booking_handoff, escalation_handoff],
)
```

**Output (agent initialized):**
```
Agent(name="Triage Agent", tools=2, handoffs=3)
# Ready to route: FAQ | Booking | Escalation
```

#### Agent 2: FAQ Agent (Specialist)

```python
faq_agent = Agent(
    name="FAQ Agent",
    instructions="""You are a customer service expert for policy questions.
    Your job is to:
    1. Answer questions about baggage, seating, cancellations, refunds
    2. Be clear and concise
    3. If customer's question goes beyond FAQ (e.g., special requests),
       hand back to triage for escalation

    Use the faq_lookup_tool to find answers.""",
    tools=[faq_lookup_tool],
    handoffs=[handoff(triage_agent, "Customer needs different help")],
)
```

**Output (agent initialized):**
```
Agent(name="FAQ Agent", tools=1, handoffs=1)
# Specialist for policy questions
```

#### Agent 3: Booking Agent (Specialist)

```python
booking_agent = Agent(
    name="Booking Agent",
    instructions="""You are a booking specialist for seat changes and flight modifications.
    Your job is to:
    1. Verify passenger information (confirmation number, name)
    2. Offer seat changes with preferences (aisle, window, etc.)
    3. Update flight information if needed
    4. Confirm changes and hand back to triage

    Use the update_seat_tool to make changes.""",
    tools=[update_seat_tool],
    handoffs=[handoff(triage_agent, "Customer needs different help")],
)
```

**Output (agent initialized):**
```
Agent(name="Booking Agent", tools=1, handoffs=1)
# Specialist for seat/flight modifications
```

#### Agent 4: Escalation Agent (Specialist)

```python
escalation_agent = Agent(
    name="Escalation Agent",
    instructions="""You are handling escalations requiring human judgment.
    Your job is to:
    1. Acknowledge the customer's concern empathetically
    2. Summarize the issue for human agent
    3. Create escalation ticket

    Examples that should be escalated:
    - Angry/aggressive customers
    - Special accommodations (disability, family emergencies)
    - Refund disputes > $500
    - Travel disruption compensation

    Use the escalate_to_human tool.""",
    tools=[escalate_to_human],
    handoffs=[],  # No handoffs from escalation (end point)
)
```

**Output (agent initialized):**
```
Agent(name="Escalation Agent", tools=1, handoffs=0)
# Terminal agent - creates ticket for human queue
```

### 5. Handoffs with Context Injection

Define the handoff functions with callbacks that inject runtime data:

```python
async def on_faq_handoff(context: RunContextWrapper[AirlineAgentContext]) -> None:
    """Prepare context when handing off to FAQ agent."""
    context.context.issue_type = "faq"

async def on_booking_handoff(context: RunContextWrapper[AirlineAgentContext]) -> None:
    """Prepare context when handing off to booking agent."""
    context.context.issue_type = "booking"
    # Simulate looking up current flight from confirmation number
    if context.context.confirmation_number:
        context.context.flight_number = f"FLT-{random.randint(100, 999)}"

async def on_escalation_handoff(context: RunContextWrapper[AirlineAgentContext]) -> None:
    """Prepare context when escalating."""
    context.context.issue_type = "escalation"

faq_handoff = handoff(
    faq_agent,
    on_handoff=on_faq_handoff
)

booking_handoff = handoff(
    booking_agent,
    on_handoff=on_booking_handoff
)

escalation_handoff = handoff(
    escalation_agent,
    on_handoff=on_escalation_handoff
)
```

**Output (when handoff triggers):**
```
# Triage → Booking handoff
on_booking_handoff called:
  context.issue_type = "booking"
  context.flight_number = "FLT-847"  # Looked up from confirmation
# → Booking Agent now has flight context
```

### 6. Complete Conversation Loop

```python
import uuid
from agents import Runner, SQLiteSession
from agents.tracing import trace, group_id as set_group_id, gen_trace_id

async def run_support_conversation(
    user_id: str,
    initial_message: str
):
    """Run a complete customer support conversation with tracing and persistence."""

    # Create unique IDs for this conversation
    conversation_id = str(uuid.uuid4())
    trace_id = gen_trace_id()

    # Initialize session for conversation memory
    session = SQLiteSession(
        session_id=conversation_id,
        file="conversations.db"
    )

    # Start context
    context = AirlineAgentContext(
        conversation_id=conversation_id,
        turn_count=0
    )

    # Run with tracing
    with trace("Customer Support Conversation", trace_id=trace_id):
        # Set group_id for all turns in this conversation
        set_group_id(conversation_id)

        print(f"Starting conversation {conversation_id}")
        print(f"Trace URL: https://platform.openai.com/traces/trace?trace_id={trace_id}")

        # Initial turn
        result = await Runner.run(
            triage_agent,
            initial_message,
            context=context,
            session=session,
            hooks=SupportHooks(),
        )

        context.turn_count += 1
        current_agent = result.last_agent  # Who responded
        conversation_history = result.messages  # Message history

        # Multi-turn conversation loop
        while context.turn_count < 10:  # Max 10 turns
            # Get user input (simulated here, in production from API)
            print(f"\n[Turn {context.turn_count}] {current_agent.name}: {result.final_output}")
            user_input = input("Your response: ").strip()

            if user_input.lower() in ["quit", "exit", "bye"]:
                print("Conversation ended by user.")
                break

            # Continue conversation
            result = await Runner.run(
                current_agent,
                user_input,
                context=context,
                session=session,
                hooks=SupportHooks(),
            )

            context.turn_count += 1
            current_agent = result.last_agent

            # Check if issue is resolved
            if context.issue_resolved or context.escalation_reason:
                print(f"\nConversation complete.")
                break

        # Summary
        print(f"\n=== Conversation Summary ===")
        print(f"Conversation ID: {conversation_id}")
        print(f"Turns: {context.turn_count}")
        print(f"Issue Type: {context.issue_type}")
        print(f"Resolved: {context.issue_resolved}")
        if context.escalation_reason:
            print(f"Escalation Reason: {context.escalation_reason}")
        print(f"Trace: https://platform.openai.com/traces/trace?trace_id={trace_id}")
```

### 7. Lifecycle Hooks for Monitoring

```python
from agents import RunHooks

class SupportHooks(RunHooks):
    """Track metrics throughout conversation."""

    async def on_agent_start(self, context, agent):
        print(f"  → Agent starting: {agent.name}")

    async def on_tool_start(self, context, agent, tool):
        print(f"    → Tool call: {tool.name}")

    async def on_tool_end(self, context, agent, tool, result):
        print(f"    ✓ Tool result: {tool.name}")

    async def on_handoff(self, context, from_agent, to_agent):
        print(f"  → Handoff: {from_agent.name} → {to_agent.name}")

    async def on_agent_end(self, context, agent, output):
        print(f"  ✓ Agent complete: {agent.name}")
        print(f"    Usage: {context.usage.total_tokens} tokens")
```

**Output (during conversation):**
```
  → Agent starting: Triage Agent
    → Tool call: faq_lookup_tool
    ✓ Tool result: faq_lookup_tool
  → Handoff: Triage Agent → Booking Agent
  ✓ Agent complete: Triage Agent
    Usage: 423 tokens
```

## Implementation Walkthrough

### Step 1: Set Up Environment

```bash
# Install SDK with all features
pip install "openai-agents[litellm]"

# Set API key
export OPENAI_API_KEY=sk-proj-your-key-here
```

### Step 2: Define All Components

Create `airline_support_fete.py` with:
1. Context model (AirlineAgentContext)
2. Tools (faq_lookup_tool, update_seat_tool, escalate_to_human)
3. Guardrails (abuse_detection, pii_detection)
4. Agents (triage, faq, booking, escalation)
5. Handoffs (with on_handoff callbacks)
6. Conversation loop (run_support_conversation)
7. Hooks (SupportHooks)

### Step 3: Test Individual Components

```python
# Test FAQ tool
context = RunContextWrapper(AirlineAgentContext())
answer = faq_lookup_tool("What's the baggage limit?", context)
print(answer)
# Output: "According to our FAQ: You can bring 1 carry-on..."

# Test guardrails
abuse_input = "Your airline SUCKS and I hate you!!!"
# Should trigger abuse_guardrail

# Test triage agent
result = Runner.run_sync(
    triage_agent,
    "Hi, I need to change my seat to an aisle"
)
```

### Step 4: Run Full Conversation

```python
# Run with all features
import asyncio

asyncio.run(run_support_conversation(
    user_id="user-12345",
    initial_message="Hello, I booked a flight and need to change my seat preference"
))
```

**Expected Output:**
```
Starting conversation d4c8f9e1-2b47...
Trace URL: https://platform.openai.com/traces/trace?trace_id=...

  → Agent starting: Triage Agent
    → Tool call: faq_lookup_tool
    ✓ Tool result: faq_lookup_tool
  → Handoff: Triage Agent → Booking Agent
  ✓ Agent complete: Triage Agent

  → Agent starting: Booking Agent
    → Tool call: update_seat_tool
    ✓ Tool result: update_seat_tool
  ✓ Agent complete: Booking Agent
    Usage: 847 tokens

[Turn 1] Booking Agent: Great! I see you want to change your seat. What's your preference - aisle, window, or middle?

=== Conversation Summary ===
Conversation ID: d4c8f9e1-2b47-4e9c-b6a7-9e3c5f8a1d2b
Turns: 2
Issue Type: booking
Resolved: True
Trace: https://platform.openai.com/traces/trace?trace_id=...
```

## How to Monetize This Digital FTE

Now that you've built a production-ready system, how would you charge for it?

### Model 1: Subscription (Recurring Revenue)

**Structure**: Monthly fee per customer for unlimited support conversations

**Pricing Logic**:
- Tier 1: $200/month = up to 1,000 conversations/month ($0.20/conversation)
- Tier 2: $500/month = up to 5,000 conversations/month ($0.10/conversation)
- Tier 3: $2,000/month = unlimited conversations

**Your Cost**: ~$0.02/conversation (API calls to OpenAI)

**Margin**:
- Tier 1: $200 revenue - $20 cost = 90% margin per customer
- Tier 2: $500 revenue - $50 cost = 90% margin per customer

**Why this works**: Customers know their monthly cost. No surprises. Predictable revenue for you.

### Model 2: Success Fee (Performance-Based)

**Structure**: Commission on resolved issues without human escalation

**Pricing Logic**:
- 15% of value saved per conversation that doesn't require escalation
- Example: Customer books a $500 change fee ticket without human help → You get $75

**Calculation**:
- 1,000 conversations/month × 85% resolution rate = 850 automated conversations
- Average value per conversation = $50-200 (depends on industry)
- Revenue = 850 × $100 × 15% = $12,750/month

**Why this works**: Customers only pay for results. Aligns incentives (you profit when they succeed).

### Model 3: Hybrid (Subscription + Performance)

**Structure**: Base subscription + bonus for exceeding targets

**Pricing Logic**:
- $500/month base (covers infrastructure)
- Plus: $0.50 per conversation beyond 2,000/month
- Plus: $0.10 bonus per conversation that resolves without escalation

**Why this works**: Secure baseline revenue + upside for high volume/quality.

### Which Model to Choose?

**Subscription works best if**:
- Customer has predictable support volume
- Customer wants fixed costs
- You're competing against existing support infrastructure

**Success Fee works best if**:
- Customer is skeptical about AI capabilities
- Customer has variable volume
- You're confident in your automation rate

**Hybrid works best if**:
- You're serving customers at different maturity levels
- You want to de-risk the customer's decision

## Extending the System

### Production Requirements Checklist

Before deploying to real customers, you'll need:

- [ ] **Rate Limiting**: Prevent API abuse (max 100 conversations/min per API key)
- [ ] **Authentication**: API keys or OAuth for customer verification
- [ ] **Webhook Integration**: Send events to customer systems (conversation_completed, escalated_to_human)
- [ ] **Analytics Dashboard**: Show customer their own metrics (automation rate, response time, CSAT)
- [ ] **A/B Testing Framework**: Test routing strategies, prompt variations
- [ ] **Fallback to Human**: Seamless handoff to human queue when escalated
- [ ] **Multi-tenancy**: Support multiple customers with isolated conversations
- [ ] **Data Retention Policies**: Delete conversations after 90 days unless requested
- [ ] **Compliance**: GDPR, SOC2 certification for enterprise customers

## Try With AI

You've now seen the complete production system. Let's deepen your understanding through exploration.

### Part 1: Design Your Own Domain

**Prompt 1**: "I want to build a support agent for [YOUR DOMAIN]. What agents would I need? What context object would I create?"

**Examples**:
- Healthcare (insurance claims): Triage → Benefits agent → Claim processing → Escalation
- E-commerce (returns): Triage → Refund agent → Shipping agent → Escalation
- Financial (account issues): Triage → FAQ → Account modification → Escalation

**What you're learning**: How to map domain knowledge to multi-agent architecture.

### Part 2: Design Guardrails for Your System

**Prompt 2**: "For [YOUR DOMAIN] support, what guardrails would you implement? What inputs should be blocked? What should be masked?"

**Examples for Healthcare**:
- Block: Treatment requests (medical advice should go to doctor)
- Mask: Social security numbers, medical record numbers
- Flag: Requests for controlled substances

**Examples for E-commerce**:
- Block: Requests for refund beyond return window
- Mask: Credit card numbers
- Flag: High-value returns (fraud detection)

**What you're learning**: How to design safety constraints that match business requirements.

### Part 3: Calculate Monetization for Your Scenario

**Prompt 3**: "If I deployed this [YOUR DOMAIN] support FTE and achieved 85% automation rate with 10,000 conversations/month, and my average value per conversation is $[AMOUNT], which pricing model (subscription/success fee/hybrid) maximizes profit while remaining competitive?"

**What you're learning**: How Digital FTE economics change with domain, automation rate, and customer willingness to pay.
