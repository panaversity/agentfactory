---
sidebar_position: 9
title: "Chapter 34 Quiz"
description: "Test your mastery of OpenAI Agents SDK production patterns"
chapter: 34
lesson: 9
duration_minutes: 45

# HIDDEN SKILLS METADATA
quiz_proficiency: "B1-B2 → C1"
bloom_levels: ["Remember", "Understand", "Apply", "Analyze", "Evaluate", "Create"]
assessment_type: "Summative"
passing_score: 70
total_questions: 15

---

# Chapter 34 Quiz: OpenAI Agents SDK Production Mastery

## Instructions

This quiz evaluates your mastery of OpenAI Agents SDK production patterns across all 8 lessons. You'll progress from foundational understanding to architectural design decisions.

**Format**: 15 questions with single correct answer
**Time limit**: 45 minutes
**Passing score**: 70% (11 of 15 correct)
**Retakes allowed**: 3 per 24 hours

Each question includes a learning reference to review relevant lesson content if you need clarification.

---

## Section 1: SDK Primitives & Execution (Lessons 1-2)

### Q1: Agent-Runner Relationship [Remember → Understand]

What is the primary responsibility of the `Runner` object in the OpenAI Agents SDK?

A) Define the agent's personality and instructions
B) Execute the agent loop, calling tools and handling handoffs until terminal output is produced
C) Store conversation history in the database
D) Validate user input before sending to the agent

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: B) Execute the agent loop, calling tools and handling handoffs until terminal output is produced**

The `Runner` is the orchestration engine. It:
- Invokes the LLM with current context
- Handles tool calls by running them and returning results
- Manages handoffs to other agents
- Continues looping until the agent produces `final_output`

The `Agent` defines *what* to do; the `Runner` implements *how* to do it. You use `Runner.run_sync()` for blocking execution or `Runner.run()` for async.

**Learning reference**: Lesson 1 - SDK Setup & First Agent, "Understanding Runner and Result Objects"

</details>

---

### Q2: Function Tool Decorators [Understand]

When you use the `@function_tool` decorator in an agent, what is its primary purpose?

A) It converts a Python function into an HTTP endpoint
B) It registers a callable as an available tool that the agent's LLM reasoning can invoke
C) It encrypts the function's parameters for security
D) It automatically caches function results for performance

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: B) It registers a callable as an available tool that the agent's LLM reasoning can invoke**

The `@function_tool` decorator makes your Python function available to the agent. When you decorate a function:

```python
@function_tool
async def search_database(query: str) -> str:
    """Find records matching query."""
    return results
```

The agent sees this tool in its tool list and can decide to call it. The LLM sees:
- Function name: `search_database`
- Description: Docstring
- Parameters: Type hints (str) and names

Type hints and docstrings are mandatory—they tell the LLM how to use your tool.

**Learning reference**: Lesson 2 - Function Tools & Context Objects

</details>

---

### Q3: Context Object Persistence [Understand]

You have this Pydantic context model in your agent:

```python
class TaskContext(BaseModel):
    user_id: str
    tasks_created: int = 0

context = TaskContext(user_id="user_123")
```

After a tool calls `context.tasks_created += 1`, what happens when the next tool executes in the same `Runner.run_sync()` call?

A) The modification is lost; each tool receives a fresh context copy
B) The modification persists; the next tool sees `tasks_created = 1`
C) The modification is cached but only visible after the run completes
D) The modification triggers a runtime error because context is immutable

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: B) The modification persists; the next tool sees `tasks_created = 1`**

Context objects in the SDK are mutable throughout a single `Runner.run_sync()` execution. When a tool accesses context via `RunContextWrapper[TaskContext]`, it gets a reference to the same object:

```python
@function_tool
async def increment_tasks(context: RunContextWrapper[TaskContext]) -> str:
    context.context.tasks_created += 1
    return f"Created {context.context.tasks_created} tasks"
```

Each tool in the same run sees all previous mutations. This enables state sharing across tools—critical for features like "remember the user's selected project" or "track how many items we've processed."

After the run completes, if you use a `Session`, the final context state is stored.

**Learning reference**: Lesson 2 - Function Tools & Context Objects, "Context Mutations Across Tool Calls"

</details>

---

## Section 2: Agent Composition & Orchestration (Lesson 3)

### Q4: Agents as Tools Use Case [Apply]

You're building a financial research system with three agents:
- `fundamentals_agent`: Analyzes quarterly earnings
- `risk_agent`: Evaluates market risks
- `writer_agent`: Synthesizes findings into a report

Your architecture uses `manager_agent` that calls all three to gather input. Which pattern should you use?

A) Handoffs: `manager_agent` hands off to each specialist in sequence
B) Agents as Tools: `manager_agent.as_tool()` on each specialist, adding as tools to manager
C) Agents as Tools: Call `specialist.as_tool()` for each specialist, adding as tools to manager
D) Orchestrator: Use Python to manually call each agent's Runner independently

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: C) Agents as Tools: Call `specialist.as_tool()` for each specialist, adding as tools to manager**

The distinction is critical:
- **Agents as Tools** (option C): The manager agent *owns* the conversation. It decides when to call specialists, orchestrates their outputs, and produces final synthesis.
- **Handoffs** (option A): Conversation *transfers* to specialist. Manager loses control. Used when specialist takes over from user.

In this case, the manager orchestrates research. Code looks like:

```python
fundamentals_tool = fundamentals_agent.as_tool(
    tool_name="get_fundamentals",
    tool_description="Analyze earnings and financial metrics"
)
# Similar for risk_tool, writer_tool

manager = Agent(
    name="Research Manager",
    instructions="Gather fundamentals and risk, then synthesize report",
    tools=[fundamentals_tool, risk_tool, writer_tool]
)
```

Option D (manual orchestration) bypasses the agent's decision-making—defeats the purpose of using agents.

**Learning reference**: Lesson 3 - Agents as Tools & Orchestration, "When to Use Agents as Tools vs. Handoffs"

</details>

---

### Q5: Custom Output Extractors [Apply]

You've built a `research_agent` that returns a detailed analysis. But your `orchestrator_agent` only needs the summary paragraph. How do you prevent the orchestrator from seeing irrelevant details?

A) Have the orchestrator extract the summary itself using string parsing
B) Return a stripped-down context from research_agent
C) Use `custom_output_extractor` when calling `research_agent.as_tool()`
D) Modify research_agent instructions to only output summaries

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: C) Use `custom_output_extractor` when calling `research_agent.as_tool()`**

The custom output extractor controls what the orchestrator sees without modifying the agent itself:

```python
research_tool = research_agent.as_tool(
    tool_name="research",
    tool_description="Detailed research report",
    custom_output_extractor=lambda result: str(result.final_output.summary_paragraph)
)
```

Benefits:
- The research agent still produces full output (useful for logging/audit)
- The orchestrator only sees the summary (keeps reasoning focused)
- No coupling—orchestrator and research agent evolve independently

Option A (string parsing) would be the orchestrator's job—messy and fragile.
Option D (modify instructions) loses information.

**Learning reference**: Lesson 3 - Agents as Tools & Orchestration, "Controlling Sub-Agent Output Visibility"

</details>

---

## Section 3: Handoffs & Context Control (Lesson 4)

### Q6: Handoff Callback Timing [Understand]

You have this handoff callback:

```python
async def on_booking_handoff(context: RunContextWrapper[AirlineContext]) -> None:
    context.context.flight_number = f"FLT-{random.randint(100, 999)}"

handoff(agent=booking_agent, on_handoff=on_booking_handoff)
```

When does `on_booking_handoff` execute?

A) Before the user types their booking request (pre-handoff)
B) Just before the booking agent receives control, after context injection
C) After the booking agent returns, before returning to the triage agent
D) During agent initialization, before any user interaction

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: B) Just before the booking agent receives control, after context injection**

The callback executes in the handoff pipeline:
1. Current agent (triage) finishes and decides to hand off
2. `on_handoff` callback fires → modifies context
3. Target agent (booking) starts and can access modified context

This enables runtime data injection. Real example:

```python
# Triage agent routes to booking, callback injects flight info
result = await Runner.run(triage_agent, user_input, context=context)
# Internally, when handoff triggers:
# → on_booking_handoff fires
# → context.flight_number = "FLT-549"
# → booking_agent starts, sees flight_number = "FLT-549"
```

The callback runs *synchronously* and must be fast (no 30-second API calls).

**Learning reference**: Lesson 4 - Handoffs & Message Filtering, "Handoff Callbacks for Context Injection"

</details>

---

### Q7: Handoff vs. Agents as Tools [Analyze]

Which difference between handoffs and agents-as-tools is MOST architecturally significant?

A) Handoffs are faster because they skip the LLM reasoning step
B) Agents as tools let the orchestrator decide what to call; handoffs transfer ownership to the receiving agent
C) Handoffs allow bidirectional communication; agents as tools don't
D) Agents as tools require explicit tool declarations; handoffs work dynamically

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: B) Agents as tools let the orchestrator decide what to call; handoffs transfer ownership to the receiving agent**

This is the architectural pivot:

**Agents as Tools**:
- Manager agent reasons: "I need fundamentals AND risk analysis"
- Calls both tools in sequence based on its reasoning
- Manager controls flow and synthesis

**Handoffs**:
- Current agent reasons: "This request is about seat booking"
- Decides to hand off (transfers conversation control)
- Receiving agent takes over; current agent steps back

Example:
```python
# Agents as Tools: Manager decides when to call
manager_tools = [fundamentals_tool, risk_tool]
manager.run(...)  # Manager calls tools in its reasoning loop

# Handoffs: Triage decides to hand off
triage_agent.run(...)  # Triage decides → hand off to specialist
# → Specialist takes over, triage waits
# → Specialist finishes or hands back
```

Option A is false (both use LLM reasoning).
Option C is false (both can be bidirectional, though less common).
Option D is false (both require declarations).

**Learning reference**: Lesson 3 vs. Lesson 4 comparison, "Architectural Ownership Model"

</details>

---

## Section 4: Guardrails & Validation (Lesson 5)

### Q8: Agent-Based Guardrails [Understand]

What is the primary advantage of an agent-based guardrail compared to a simple function-based guardrail?

A) It's faster because it skips the LLM
B) It can use full LLM reasoning to make nuanced validation decisions
C) It automatically scales to handle more requests
D) It requires no configuration

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: B) It can use full LLM reasoning to make nuanced validation decisions**

A simple function guardrail (regex, length checks) can only handle brittle rules:

```python
@input_guardrail
async def length_check(context, agent, input) -> GuardrailFunctionOutput:
    if len(input) > 1000:  # Brittle!
        return GuardrailFunctionOutput(tripwire_triggered=True)
```

An agent-based guardrail uses the LLM to reason:

```python
guardrail_agent = Agent(
    name="Policy Checker",
    instructions="Determine if request violates company policy"
)

@input_guardrail
async def policy_guardrail(context, agent, input) -> GuardrailFunctionOutput:
    result = await Runner.run(
        guardrail_agent,
        f"Does this violate policy: {input}?",
        context=context.context
    )
    output = result.final_output_as(PolicyCheckOutput)
    return GuardrailFunctionOutput(
        tripwire_triggered=output.violates_policy
    )
```

The guardrail agent can understand context ("Is this a homework help request?"), nuance ("Is this scientific homework or personal tutoring?"), and policy complexity.

**Learning reference**: Lesson 5 - Guardrails & Agent-Based Validation, "Agent-Based vs. Function-Based Guardrails"

</details>

---

### Q9: Input vs. Output Guardrails [Understand]

You want to prevent the agent from giving financial advice to unaccredited investors. Where should this validation occur?

A) Input guardrail: Block user requests asking for financial advice
B) Output guardrail: Prevent the agent from generating financial advice responses
C) Both: Input blocks requests, output blocks agent from responding
D) Neither: This should be in agent instructions, not guardrails

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: B) Output guardrail: Prevent the agent from generating financial advice responses**

The reasoning:
- **Input guardrails** stop user *input* from reaching the agent. Prevents users from asking for financial advice.
- **Output guardrails** stop agent *output* from reaching users. Prevents agent from generating financial advice, even if user asks indirectly.

For regulatory compliance (unaccredited investor protection), you need **both**:

```python
@input_guardrail
async def no_financial_requests(context, agent, input) -> GuardrailFunctionOutput:
    # Block: "Give me investment advice"
    ...

@output_guardrail
async def no_financial_responses(context, agent, output) -> GuardrailFunctionOutput:
    # Block agent response that mentions stocks, bonds, returns, etc.
    ...
```

Input alone doesn't prevent the user from asking indirectly ("What's a good way to grow my money?"). Output alone doesn't stop obvious violations. Defense in depth is required for compliance.

**Learning reference**: Lesson 5 - Guardrails & Agent-Based Validation, "Input vs. Output Guardrails"

</details>

---

## Section 5: Memory & Sessions (Lesson 6)

### Q10: SQLiteSession vs. AdvancedSQLiteSession [Analyze]

You're building a customer support agent that needs to:
1. Remember all conversation history across sessions
2. Track token usage per turn
3. Allow conversation branching ("What if the customer chose seat 12B instead of 12A?")

Which session backend should you use?

A) `SQLiteSession` — all three requirements are supported
B) `AdvancedSQLiteSession` — all three requirements are supported
C) You need to manually implement branching; use `AdvancedSQLiteSession` for the other two
D) `SQLiteSession` for simple history, `AdvancedSQLiteSession` if branching is required

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: B) `AdvancedSQLiteSession` — all three requirements are supported**

Capabilities:

**`SQLiteSession` (basic)**:
- ✅ Remembers history across sessions (automatic)
- ✅ File-based persistence
- ❌ No token usage tracking
- ❌ No branching

**`AdvancedSQLiteSession` (production)**:
- ✅ Remembers history across sessions
- ✅ Token usage tracking via `store_run_usage(result)`
- ✅ Conversation branching via `create_branch_from_turn(turn_id)`
- ✅ Branch switching via `switch_to_branch(branch_id)`
- ✅ Turn-by-turn metadata

Usage:

```python
session = AdvancedSQLiteSession(session_id="user_123")

# First turn
result1 = await Runner.run(agent, "Book seat 12A", session=session)
await session.store_run_usage(result1)  # Track tokens

# Branch: What if they chose 12B?
branch_id = await session.create_branch_from_turn(1)
result2 = await Runner.run(agent, "Actually, seat 12B", session=session)

# Original timeline: still has 12A
await session.switch_to_branch("main")
```

`AdvancedSQLiteSession` is the default for production customer support FTEs.

**Learning reference**: Lesson 6 - Sessions & Conversation Memory, "AdvancedSQLiteSession for Production"

</details>

---

### Q11: Conversation Branching Use Case [Apply]

You're debugging why your agent gave bad advice in turn 3. You want to replay the conversation from turn 2 with different user input. Which `AdvancedSQLiteSession` method do you use?

A) `create_branch_from_turn(2)` to branch at turn 2, then `Runner.run()` with new input
B) `get_conversation_by_turns()` to retrieve turn 2, then modify and re-run
C) `switch_to_branch("main")` to reset, then manually delete turns 3+
D) `reset_session()` to clear all history, then replay from the start

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: A) `create_branch_from_turn(2)` to branch at turn 2, then `Runner.run()` with new input**

The correct workflow:

```python
session = AdvancedSQLiteSession(session_id="debug_user_123")

# Original conversation (already stored)
# Turn 1: "What stocks should I buy?"
# Turn 2: "For retirement"
# Turn 3: [Agent gives bad advice]

# Now you want to debug:
# Branch at turn 2 (after "For retirement")
new_branch_id = await session.create_branch_from_turn(2)

# The session is now on the new branch with turns 1-2
# Run a new turn 3 with different input
result = await Runner.run(
    agent,
    "Actually, I want index funds",  # Different input
    session=session
)
# This creates turn 3 on the new branch (original turn 3 stays on "main")
```

Option B doesn't support direct modification.
Option C is destructive and loses the original branch.
Option D loses the entire conversation history.

**Learning reference**: Lesson 6 - Sessions & Conversation Memory, "Branching for Conversation Debugging"

</details>

---

## Section 6: Observability & Tracing (Lesson 7)

### Q12: RunHooks Lifecycle [Understand]

You're implementing `RunHooks` to monitor agent execution. Which hook fires when a tool is about to be invoked by the agent?

A) `on_llm_start()` — fires when the LLM is called
B) `on_tool_start()` — fires just before the tool executes
C) `on_agent_start()` — fires when the agent loop begins
D) `on_handoff()` — fires when the tool calls another agent

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: B) `on_tool_start()` — fires just before the tool executes**

The RunHooks lifecycle for a single turn:

```
on_agent_start()           # Agent loop begins
  ↓
on_llm_start()             # LLM is invoked with current context
  ↓
on_llm_end()               # LLM responds with tool calls
  ↓
for each tool call:
  → on_tool_start()        # Tool is about to execute ← YOUR ANSWER
  → [tool executes]
  → on_tool_end()          # Tool completed
  ↓
[Repeat: LLM, tools until final_output]
  ↓
on_agent_end()             # Agent loop terminates
```

You use `on_tool_start()` to:
- Log which tool is being called
- Measure execution time
- Validate tool parameters

```python
class MonitoringHooks(RunHooks):
    async def on_tool_start(self, context, agent, tool):
        print(f"Calling tool: {tool.name}")
        self.start_time = time.time()
```

**Learning reference**: Lesson 7 - Tracing, Hooks & Observability, "RunHooks Lifecycle Methods"

</details>

---

### Q13: Trace IDs vs. Custom Spans [Analyze]

When would you use `custom_span()` instead of just wrapping code in a `trace()` block?

A) `trace()` is for the entire agent run; `custom_span()` is for sub-operations within a trace
B) `custom_span()` is faster because it skips the tracing infrastructure
C) `trace()` generates dashboards; `custom_span()` only works in logs
D) They're interchangeable; use whichever you prefer

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: A) `trace()` is for the entire agent run; `custom_span()` is for sub-operations within a trace**

The hierarchy:

```python
trace_id = gen_trace_id()

with trace("User query processing", trace_id=trace_id):
    # Entire operation is traced

    with custom_span("Search the web"):
        results = await web_search(query)  # Sub-operation

    with custom_span("Process results"):
        filtered = filter_results(results)

    with custom_span("Generate response"):
        response = await agent.run(...)
```

In the OpenAI dashboard, you see:
- Top-level trace: "User query processing"
- Three child spans: "Search", "Process", "Generate"

This enables granular timing and diagnostics:

```python
# Without spans: You know the whole run took 5 seconds
# With spans: You know Search (1s) + Process (0.5s) + Generate (3.5s)
```

You use `custom_span()` for:
- Parallel operations (orchestrator calls 3 sub-agents)
- Sequential phases (search → process → generate)
- Performance debugging

**Learning reference**: Lesson 7 - Tracing, Hooks & Observability, "Custom Spans for Operation Granularity"

</details>

---

### Q14: Group ID for Correlation [Apply]

You're building a customer support system where each conversation has multiple turns. You want all turns from the same conversation to appear together in the tracing dashboard. How do you link them?

A) Use the same `trace_id` for all turns
B) Use the same `group_id` parameter across all turns
C) Create a single trace that spans the entire conversation
D) Manually tag each turn with the conversation UUID in logs

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: B) Use the same `group_id` parameter across all turns**

Implementation:

```python
conversation_id = "conv_12345"

# Turn 1
result1 = await Runner.run(
    agent,
    "I need help with my booking",
    group_id=conversation_id  # ← Group all turns
)

# Turn 2
result2 = await Runner.run(
    agent,
    "Can I change my seat?",
    group_id=conversation_id  # ← Same group
)

# Turn 3
result3 = await Runner.run(
    agent,
    "Thanks, that's all",
    group_id=conversation_id  # ← Same group
)

# In OpenAI dashboard: All three turns appear under group "conv_12345"
```

The `group_id` is distinct from `trace_id`:
- **`trace_id`**: Unique per operation, links to dashboard URL
- **`group_id`**: Shared across multiple traces, groups related work

Option A (same trace_id) would merge operations into one trace—doesn't work for multi-turn conversations.
Option C doesn't align with multi-turn architecture.
Option D is manual and scales poorly.

**Learning reference**: Lesson 7 - Tracing, Hooks & Observability, "Group IDs for Multi-Turn Correlation"

</details>

---

## Section 7: System Design & Capstone (Lesson 8)

### Q15: Customer Support FTE Architecture [Create]

You're building a customer support FTE for an airline with three specialist agents:
- `triage_agent`: Routes to FAQ or booking
- `faq_agent`: Answers common questions
- `booking_agent`: Changes seats and flight times

A customer asks: "Can I change my seat?" and then "What's your baggage policy?"

Using the patterns from the Customer Support FTE lesson, describe the architectural decision:
1. Should the triage agent own the conversation?
2. Should you use handoffs or agents-as-tools?
3. Should you use `on_handoff` callbacks? If yes, what for?

Which answer best reflects the production pattern?

A) Triage owns conversation; use handoffs; no callbacks needed (specialists have enough context)
B) Triage owns conversation; use agents-as-tools; callbacks inject flight_number before seat_booking_agent
C) Triage owns conversation; use handoffs; on_handoff callback injects flight_number before seat_booking_agent
D) FAQ owns conversation; use agents-as-tools; callbacks transfer control back to triage

<details>
<summary>Answer & Explanation</summary>

**Correct Answer: C) Triage owns conversation; use handoffs; on_handoff callback injects flight_number before seat_booking_agent**

Architectural reasoning:

**Why Triage owns conversation**:
- Triage decides routing based on user input ("change seat" → booking, "baggage policy" → FAQ)
- User conversation stays with one entity (better for context and user experience)
- Specialists hand back after completion ("Anything else?" → back to triage)

**Why handoffs (not agents-as-tools)**:
- Each turn, the user is talking to ONE agent (triage initially)
- When user asks about seating, conversation *transfers* to booking agent
- When user pivots to policy, booking agent hands back to triage
- This is conversational handoff, not tool calling

**Why `on_handoff` callbacks**:
- When triage hands to booking, the callback injects `flight_number = "FLT-549"`
- Booking agent accesses context via tools: `context.context.flight_number`
- Without callback, booking agent wouldn't know which flight (user never said it in this turn)

```python
async def on_booking_handoff(context: RunContextWrapper[AirlineContext]) -> None:
    # Booking agent needs flight_number, extracted from context
    # (context was populated by earlier FAQ interaction or database lookup)
    pass

handoff(agent=booking_agent, on_handoff=on_booking_handoff)
```

Option A (no callbacks): Booking agent lacks flight info; can't complete request.
Option B (agents-as-tools): Triage would call booking as a tool → wrong conversation model.
Option D (FAQ owns): Triage wouldn't get called again; can't re-enter for routing.

**Learning reference**: Lesson 8 - Capstone: Customer Support FTE, Complete Implementation

</details>

---

## Summary & Next Steps

Congratulations on completing the Chapter 34 quiz!

**If you scored 11+ (70%+)**:
You're ready to apply these patterns to production systems. Consider:
- Implementing the Customer Support FTE capstone with your own domain
- Adding tracing and observability to your agent system
- Deploying with SQLite sessions for persistent memory

**If you scored below 70%**:
Review these high-impact lessons:
- **Lesson 1-2** (Foundational): Agent, Runner, tools, context
- **Lesson 3-4** (Critical distinction): Agents as tools vs. handoffs
- **Lesson 5-6** (Production): Guardrails and persistent sessions
- **Lesson 7-8** (Mastery): Tracing and complete system design

**What comes next**:
- Part 7: Deployment & Operations (agent monitoring, A/B testing, cost optimization)
- Advanced multi-modal agents (voice, vision integration)
- Extending the SDK with custom hooks and session backends

---

## Scoring Rubric

| Score | Proficiency | Assessment |
|-------|-------------|-----------|
| 90-100% | C1 (Advanced) | Mastery—you understand architectural tradeoffs and can design new systems |
| 80-89% | C1 (Intermediate) | Strong understanding—you can implement patterns correctly with guidance |
| 70-79% | B2 (Proficient) | Competent—you can use SDK patterns in supervised environments |
| Below 70% | B1 (Developing) | Review lessons and retake; patterns need reinforcement |

