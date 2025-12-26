---
sidebar_position: 5
title: "Guardrails & Agent-Based Validation"
description: "Implement input and output guardrails to validate and control agent behavior. Learn simple guards, agent-based guardrails, and exception handling for production safety."
keywords: ["guardrails", "agent validation", "input guards", "output guards", "PII detection", "tripwire", "InputGuardrailTripwireTriggered", "production agents"]
chapter: 34
lesson: 5
duration_minutes: 90

# HIDDEN SKILLS METADATA
skills:
  - name: "Input Guardrail Implementation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can implement @input_guardrail functions that validate and block requests"

  - name: "Output Guardrail Implementation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can implement @output_guardrail functions that check agent responses for policy violations"

  - name: "Agent-Based Guardrails (Advanced)"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can create guardrail agents that use reasoning to detect policy violations"

  - name: "GuardrailFunctionOutput and Tripwire Pattern"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student understands tripwire_triggered flag and how guardrail decisions flow through the system"

  - name: "Exception Handling for Guardrails"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Error Handling"
    measurable_at_this_level: "Student can catch InputGuardrailTripwireTriggered and OutputGuardrailTripwireTriggered exceptions"

  - name: "Production Agent Safety Patterns"
    proficiency_level: "B2"
    category: "Soft"
    bloom_level: "Understand"
    digcomp_area: "Professional Practice"
    measurable_at_this_level: "Student understands when and how to apply guardrails in production agent workflows"

learning_objectives:
  - objective: "Implement input guardrails that block inappropriate requests before agent processing"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student creates functioning guardrail that prevents specific request types"

  - objective: "Implement output guardrails that catch policy violations in agent responses"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student creates guardrail that detects sensitive data in responses"

  - objective: "Build sophisticated guardrails using agent-based reasoning instead of simple pattern matching"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student creates and configures a guardrail agent for semantic validation"

  - objective: "Handle guardrail tripwire exceptions gracefully with user-friendly messages"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student implements try/except blocks that provide actionable feedback"

  - objective: "Understand when to use guardrails and design guardrail strategies for specific Digital FTE use cases"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Student articulates guardrail strategy for their custom agent project"

cognitive_load:
  new_concepts: 6
  assessment: "6 new concepts (@input_guardrail decorator, @output_guardrail decorator, GuardrailFunctionOutput, tripwire_triggered pattern, agent-based guardrails, exception handling patterns) within B2 limit of 10 concepts - PASS"

differentiation:
  extension_for_advanced: "Implement multi-level guardrails (fast keyword checks + expensive agent checks), combine guardrails with structured outputs for semantic validation, build guardrail pipelines that cascade checks"
  remedial_for_struggling: "Start with simple keyword-based input guardrails; skip agent-based guardrails initially. Practice exception handling separately before combining with guardrails."

generated_by: content-implementer
source_spec: specs/047-ch34-openai-agents-sdk
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# Guardrails & Agent-Based Validation

Your customer support agent is working beautifully—until a user asks it to help with homework assignments, or a malicious actor tries to extract your proprietary training data through the agent interface. You need barriers that stop inappropriate requests before they reach the agent, and catches dangerous outputs before they reach users.

This is what guardrails do. They're validation checkpoints running before and after agent execution—like security checkpoints at an airport. Quick checks filter most traffic; sophisticated checks catch edge cases. By the end of this lesson, you'll implement both.

## Understanding Guardrails: The Three Patterns

Production agents need guardrails at three levels:

1. **Input Guardrails** — Validate user requests before agent processing
2. **Output Guardrails** — Validate agent responses before returning to user
3. **Guardrail Exceptions** — Handle policy violations gracefully with user-friendly messages

### Why Guardrails Matter for Digital FTEs

A Digital FTE is a professional system representing your organization. It must:

- Reject requests outside its scope (a legal assistant shouldn't do math homework)
- Prevent PII leakage (never expose customer phone numbers in responses)
- Follow compliance rules (GDPR, HIPAA, company policy)
- Provide clear feedback when it can't help

Without guardrails, you have an agent. With guardrails, you have a **professional service**.

## Pattern 1: Simple Input Guardrails

The simplest guardrail checks user input against keywords or patterns before the agent sees it.

### Basic Input Guardrail Example

Create a file `guardrail_basic.py`:

```python
from agents import Agent, input_guardrail, GuardrailFunctionOutput
import asyncio

@input_guardrail
async def block_homework(context, agent, input_text: str) -> GuardrailFunctionOutput:
    """Block requests asking for homework help."""
    keywords = ["homework", "assignment", "exam help", "test answers"]

    # Check if input contains any blocked keywords
    is_blocked = any(keyword in input_text.lower() for keyword in keywords)

    return GuardrailFunctionOutput(
        output_info={"blocked_keywords": keywords if is_blocked else []},
        tripwire_triggered=is_blocked,
    )

# Create agent with guardrail
agent = Agent(
    name="Tutor",
    instructions="You are an educational tutor. Help students understand concepts.",
    input_guardrails=[block_homework],
)

async def main():
    # This will pass the guardrail
    result = await asyncio.run(Runner.run(agent, "Explain how photosynthesis works"))
    print("Request 1:", result.final_output)

    # This will trigger the guardrail
    try:
        result = await asyncio.run(Runner.run(agent, "Can you help me with my homework assignment?"))
        print("Request 2:", result.final_output)
    except InputGuardrailTripwireTriggered as e:
        print("Request blocked:", e.guardrail_result.output_info)
```

**Output:**
```
Request 1: Photosynthesis is the process plants use to convert light energy...

Request blocked: {'blocked_keywords': ['homework', 'assignment']}
```

### Breaking Down the Pattern

**The Decorator:**
```python
@input_guardrail
async def block_homework(context, agent, input_text: str) -> GuardrailFunctionOutput:
```

The `@input_guardrail` decorator registers this function as an input validation check. It receives:
- `context` — Request context (user info, session data)
- `agent` — The agent being protected
- `input_text` — What the user typed

**The Return:**
```python
return GuardrailFunctionOutput(
    output_info={"blocked_keywords": keywords if is_blocked else []},
    tripwire_triggered=is_blocked,  # If True, blocks execution
)
```

`GuardrailFunctionOutput` tells the SDK two things:
- `output_info` — Diagnostic data (what triggered, why, details)
- `tripwire_triggered` — Boolean flag (True = block, False = allow)

**Exception Handling:**
```python
except InputGuardrailTripwireTriggered as e:
    print("Request blocked:", e.guardrail_result.output_info)
```

When `tripwire_triggered=True`, the SDK raises an exception. Your code catches it and provides user feedback.

## Pattern 2: Agent-Based Guardrails (The Powerful Approach)

Simple keyword checks are fast but brittle. "Can you help with my learning assignment?" passes the keyword filter but violates the policy. **Agent-based guardrails use reasoning to detect nuanced policy violations.**

This is the production pattern. An AI agent evaluates user requests with semantic understanding, not just pattern matching.

### Building a Guardrail Agent

Create `guardrail_agent_based.py`:

```python
from agents import Agent, input_guardrail, GuardrailFunctionOutput, Runner, RunContextWrapper
from pydantic import BaseModel
import asyncio

# Define what the guardrail agent outputs
class RequestEvaluation(BaseModel):
    is_homework_request: bool
    reasoning: str
    confidence: float  # 0.0 to 1.0

# Create a specialized guardrail agent
guardrail_agent = Agent(
    name="RequestValidator",
    instructions="""You are a policy enforcement agent. Evaluate if user requests ask for homework help.

Guidelines:
- Direct homework requests: "solve this math problem", "help with essay"
- Indirect homework requests: "how would you approach this assignment?", "learn to solve this type of problem"
- Legitimate educational requests: "explain photosynthesis", "how do I study for a test?"

Distinguish between helping students learn vs. doing their homework for them.
Mark as homework request if user is asking you to DO the work rather than help them understand.""",
    output_type=RequestEvaluation,
)

# Now the guardrail uses the guardrail agent
@input_guardrail
async def smart_homework_check(
    context: RunContextWrapper, agent: Agent, input_text: str
) -> GuardrailFunctionOutput:
    """Use an agent to intelligently detect homework requests."""

    # Ask the guardrail agent to evaluate the request
    result = await Runner.run(guardrail_agent, input_text, context=context.context)
    evaluation = result.final_output_as(RequestEvaluation)

    # Block if confidence is high enough
    should_block = evaluation.is_homework_request and evaluation.confidence > 0.7

    return GuardrailFunctionOutput(
        output_info={
            "is_homework": evaluation.is_homework_request,
            "reasoning": evaluation.reasoning,
            "confidence": evaluation.confidence,
            "blocked": should_block,
        },
        tripwire_triggered=should_block,
    )

# Main agent with intelligent guardrail
main_agent = Agent(
    name="Tutor",
    instructions="Help students understand concepts through explanations and guidance.",
    input_guardrails=[smart_homework_check],
)

async def main():
    test_inputs = [
        "Explain the process of photosynthesis",  # Should pass
        "Can you solve these calculus problems for me?",  # Should block
        "How would I approach deriving this formula?",  # Should pass (asking for guidance)
        "Help me finish my English essay",  # Should block
    ]

    for test_input in test_inputs:
        try:
            result = await Runner.run(main_agent, test_input)
            print(f"✓ Allowed: {test_input[:50]}")
            print(f"  Response: {result.final_output[:100]}\n")
        except InputGuardrailTripwireTriggered as e:
            info = e.guardrail_result.output_info
            print(f"✗ Blocked: {test_input[:50]}")
            print(f"  Reason: {info['reasoning']}")
            print(f"  Confidence: {info['confidence']}\n")

asyncio.run(main())
```

**Output:**
```
✓ Allowed: Explain the process of photosynthesis
  Response: Photosynthesis is how plants convert light energy into chemical...

✗ Blocked: Can you solve these calculus problems for me?
  Reason: This is a direct request to solve homework problems (do the work)
  Confidence: 0.95

✓ Allowed: How would I approach deriving this formula?
  Reason: This asks for guidance on methodology, not to do homework

✗ Blocked: Help me finish my English essay
  Reason: Request to complete homework assignment work
  Confidence: 0.92
```

### Why This Pattern is Powerful

**Simple keyword filtering** catches "homework" and "assignment" but misses nuance.

**Agent-based validation** understands context:
- Distinguishes "solve this problem" (homework) vs "how would I solve this?" (learning)
- Catches indirect requests ("help me finish my essay")
- Adapts to policy changes without code modification (change agent instructions instead)

## Pattern 3: Output Guardrails (Catching Sensitive Data)

Input guardrails filter requests. Output guardrails filter responses—catching when an agent accidentally includes sensitive data.

### Building an Output Guardrail

Create `guardrail_output.py`:

```python
from agents import Agent, output_guardrail, GuardrailFunctionOutput, Runner
from pydantic import BaseModel
import re
import asyncio

# Define agent output structure
class SupportResponse(BaseModel):
    reasoning: str  # Internal thinking
    response: str   # What to show user

# Output guardrail that scans for PII
@output_guardrail
async def check_for_pii(
    context, agent: Agent, output: SupportResponse
) -> GuardrailFunctionOutput:
    """Scan output for personally identifiable information."""

    # Pattern to detect phone numbers (XXX-XXX-XXXX or similar)
    phone_pattern = r'\b\d{3}[-.]?\d{3}[-.]?\d{4}\b'
    # Pattern to detect email addresses
    email_pattern = r'\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,}\b'
    # Pattern to detect credit card numbers
    cc_pattern = r'\b\d{4}[\s-]?\d{4}[\s-]?\d{4}[\s-]?\d{4}\b'

    # Check both reasoning and response
    for text_field in [output.reasoning, output.response]:
        if text_field is None:
            continue

        phone_found = bool(re.search(phone_pattern, text_field))
        email_found = bool(re.search(email_pattern, text_field))
        cc_found = bool(re.search(cc_pattern, text_field))

        # Trip if any PII detected
        if phone_found or email_found or cc_found:
            return GuardrailFunctionOutput(
                output_info={
                    "pii_detected": True,
                    "phone_found": phone_found,
                    "email_found": email_found,
                    "credit_card_found": cc_found,
                    "field": "reasoning" if text_field == output.reasoning else "response",
                },
                tripwire_triggered=True,
            )

    return GuardrailFunctionOutput(
        output_info={"pii_detected": False},
        tripwire_triggered=False,
    )

# Customer support agent with output protection
support_agent = Agent(
    name="CustomerSupport",
    instructions="""You are a helpful customer support agent.
    Answer customer questions about orders, returns, and accounts.
    NEVER include customer phone numbers, emails, or credit card numbers in your response.
    Reference these details only by ID (Order #123, Account #456).""",
    output_type=SupportResponse,
    output_guardrails=[check_for_pii],
)

async def main():
    # Safe request
    safe_result = await Runner.run(
        support_agent,
        "What's the status of my order?"
    )
    print("Safe output:", safe_result.final_output.response)

    # Request that might leak PII
    try:
        risky_result = await Runner.run(
            support_agent,
            "Can you confirm my contact info? My number is 555-123-4567."
        )
        print("Response:", risky_result.final_output.response)
    except OutputGuardrailTripwireTriggered as e:
        print("PII Detected and blocked!")
        print("Issue:", e.guardrail_result.output_info)

asyncio.run(main())
```

**Output:**
```
Safe output: I'd be happy to help with your order status. Could you provide...

PII Detected and blocked!
Issue: {'pii_detected': True, 'phone_found': True, 'field': 'response'}
```

## Pattern 4: Handling Guardrail Exceptions Gracefully

When a guardrail trips, you need to provide the user with helpful feedback—not a confusing error message.

### Exception Handling Strategy

```python
from agents import (
    Agent,
    Runner,
    InputGuardrailTripwireTriggered,
    OutputGuardrailTripwireTriggered,
)
import asyncio

async def safe_agent_call(agent: Agent, user_input: str) -> str:
    """Call an agent with comprehensive error handling."""

    try:
        result = await Runner.run(agent, user_input)
        return result.final_output

    except InputGuardrailTripwireTriggered as e:
        # User input violated policy
        info = e.guardrail_result.output_info
        return f"""I can't help with that request.

Reason: {info.get('reasoning', 'Policy violation detected')}

I'm designed to help with [specific topics].
Would you like to ask something within my area of expertise?"""

    except OutputGuardrailTripwireTriggered as e:
        # Agent's response violated policy
        info = e.guardrail_result.output_info
        return f"""I found an issue with my response and couldn't complete your request safely.

Issue: Sensitive information was detected in my response
Please try rephrasing your question or contact support for help."""

    except Exception as e:
        # Unexpected error
        return f"An unexpected error occurred. Please try again later."

# Usage
async def main():
    agent = Agent(
        name="Helper",
        instructions="Help with questions",
        input_guardrails=[...],  # Your guardrails
        output_guardrails=[...],
    )

    response = await safe_agent_call(agent, "What's the capital of France?")
    print(response)

asyncio.run(main())
```

The key insight: **Don't expose guardrail mechanics to users.** Provide friendly, actionable feedback.

## Designing Guardrails for Your Digital FTE

Every agent needs different guardrails based on its purpose:

### Example: Legal Assistant Digital FTE

**Input Guardrails:**
- Block requests for personal legal advice (recommend "consult a lawyer")
- Block requests for medical/psychiatric help
- Block requests for illegal activities

**Output Guardrails:**
- Block responses containing specific client names/dates
- Block responses that create attorney-client privilege issues
- Block responses that could be misinterpreted as legal counsel

**Strategy:**
- Use fast keyword checks for obvious blocks
- Use agent-based reasoning for nuanced policy decisions
- Always provide users clear guidance on what you CAN help with

### Example: Customer Support Digital FTE

**Input Guardrails:**
- Block trolling/abusive language (optional based on policy)
- Accept all legitimate support questions

**Output Guardrails:**
- Block any PII (phone numbers, addresses, account numbers)
- Block internal system information
- Scrub references to other customers

**Strategy:**
- Input guardrails are light (you want to hear all customer issues)
- Output guardrails are strict (never leak data)
- Train agent to reference info by ID, never by name/number

## From TaskManager to Production Safety

Remember the TaskManager agent from Lesson 1? Let's add guardrails:

```python
from agents import Agent, input_guardrail, GuardrailFunctionOutput, Runner
from pydantic import BaseModel

@input_guardrail
async def task_scope_check(context, agent, input_text: str) -> GuardrailFunctionOutput:
    """Ensure requests are actually about tasks, not trying to misuse the agent."""
    # This agent only manages tasks—not project management, not time tracking
    blocking_keywords = ["time tracking", "payroll", "delete all", "hack", "vulnerability"]

    is_abuse = any(kw in input_text.lower() for kw in blocking_keywords)

    return GuardrailFunctionOutput(
        output_info={"task_scope_violation": is_abuse},
        tripwire_triggered=is_abuse,
    )

task_agent = Agent(
    name="TaskManager",
    instructions="""You help users manage their tasks.
    You can: add tasks, mark them complete, list them.
    You cannot: manage time tracking, change work hours, access payroll.""",
    tools=[add_task, list_tasks, complete_task],
    input_guardrails=[task_scope_check],
)
```

With this guardrail, someone can't trick your TaskManager into accessing systems it shouldn't.

## Try With AI

### Prompt 1: Simple Input Guardrail

```
Create a customer support agent with a simple input guardrail that blocks
requests outside the support domain (like requests for homework help or
hacking tutorials). Use keyword matching to detect violations.

Test your guardrail with 3 legitimate support questions and 2 off-topic requests.
For each test, show:
- The input
- Whether it was blocked
- What the guardrail detected
```

**What you're learning:** How to implement basic policy enforcement without slowing requests down

### Prompt 2: Agent-Based Guardrail with Nuance

```
Upgrade your guardrail from Prompt 1 to use agent-based reasoning instead of
keywords. The guardrail agent should:
- Understand context (distinguish between genuine questions and attempts to
  trick the system)
- Provide reasoning for why requests are blocked/allowed
- Return a confidence score

Compare:
- Keyword-based blocking (from Prompt 1)
- Agent-based blocking (this prompt)

Which catches more edge cases? Which is more expensive (slower)?
```

**What you're learning:** When to invest in sophisticated guardrails vs simple checks

### Prompt 3: Exception Handling and User Feedback

```
Implement a guardrail exception handler that:
1. Catches InputGuardrailTripwireTriggered exceptions
2. Catches OutputGuardrailTripwireTriggered exceptions
3. Provides different user-friendly messages for each

For a content moderation agent, show:
- What happens when input is blocked
- What happens when output is blocked
- What a user sees in each case (no technical error messages)

The user should understand why they were blocked and what to do next.
```

**What you're learning:** How to build professional error handling that users trust
