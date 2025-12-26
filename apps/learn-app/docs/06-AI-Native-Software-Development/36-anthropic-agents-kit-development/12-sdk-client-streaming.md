---
sidebar_position: 12
title: "ClaudeSDKClient and Streaming"
description: "Master multi-turn conversations with ClaudeSDKClient and implement streaming input patterns with message generators—the foundation for long-running agents and real-time collaboration workflows."
keywords: [ClaudeSDKClient, streaming input, message generators, multi-turn conversations, image handling, interrupts, session continuity]
chapter: 36
lesson: 12
duration_minutes: 30

# HIDDEN SKILLS METADATA
skills:
  - name: "Multi-Turn Conversation Management"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can construct ClaudeSDKClient sessions where context persists across multiple query() calls and implement bidirectional conversation patterns"

  - name: "Streaming Input Patterns"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can implement message generators that stream multi-part input to agents, handling complex prompts with dynamic content generation"

  - name: "Session State and Continuity"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Analyze"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can reason about when to use ClaudeSDKClient (multi-turn) vs query() (single-turn), and evaluate architectural tradeoffs for long-running conversational agents"

learning_objectives:
  - objective: "Implement multi-turn conversations using ClaudeSDKClient where context persists across sequential queries"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student creates conversation that references earlier messages and builds on previous context"

  - objective: "Design and implement message generator functions that stream complex input to agents"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student implements generator producing multi-part message sequences and validates streaming behavior"

  - objective: "Analyze when to use streaming input for long-running conversations vs single-turn queries"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Student evaluates architectural patterns for chatbots, code analysis tools, and iterative refinement workflows"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (ClaudeSDKClient, streaming input, message generators, image handling, interrupts, session state, query vs SDKClient) within B2 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Implement long-running agents that manage conversation state across resume sessions; design interrupt handlers that gracefully pause and resume streaming; build adaptive conversation systems that adjust context size based on token usage; evaluate cost optimization strategies for multi-turn conversations"
  remedial_for_struggling: "Focus on simple two-message pattern (query + follow-up) before complex generators; use concrete examples comparing query() output vs ClaudeSDKClient conversation continuity; visualize context accumulation across turns; start with text-only conversations before adding images"
---

# ClaudeSDKClient and Streaming

You've mastered the basic `query()` pattern. It solves one-shot problems well: "Find and fix the bug." "Explain this code." Done.

But what if you need **ongoing collaboration**? Not one question. Not a fixed sequence. A genuine conversation where Claude remembers what you said, builds on prior context, and refines outputs iteratively.

That's what ClaudeSDKClient provides: **multi-turn conversation with session persistence**.

And beyond that lies an even more powerful pattern: **streaming input**. Instead of sending a complete prompt at once, you feed messages into the agent as a generator—enabling dynamic composition, image insertion, and real-time feedback loops.

## From Query to Conversation

Let's clarify the fundamental difference between what you've learned and what you're learning now.

### Query Pattern (Single-Turn)

```python
async for msg in query(
    prompt="What's the capital of France?",
    options=ClaudeAgentOptions(...)
):
    if hasattr(msg, "result"):
        print(msg.result)  # "Paris"
```

Query is:
- **Stateless**: Each call is independent
- **Simple**: One prompt in, one response out
- **Fast**: Minimal overhead

This pattern works for autonomous tasks: "Find bugs in this file." "Refactor this function." The agent doesn't need to remember previous conversations because there is no previous conversation.

### ClaudeSDKClient Pattern (Multi-Turn)

```python
async with ClaudeSDKClient(options) as client:
    # First turn
    await client.query("What's the capital of France?")
    async for msg in client.receive_response():
        print(msg)  # "Paris"

    # Second turn - Claude remembers context
    await client.query("What's the population of that city?")
    async for msg in client.receive_response():
        print(msg)  # "About 2.1 million..."

    # Third turn - Claude knows "that city" = Paris
    await client.query("When was it founded?")
    async for msg in client.receive_response():
        print(msg)  # "Dating back to around 250 BC..."
```

ClaudeSDKClient is:
- **Stateful**: Conversation context persists across turns
- **Conversational**: Back-and-forth dialogue with context accumulation
- **Interactive**: Ideal for iterative refinement and collaborative workflows

### Decision Framework: When to Use Each

| Pattern | Use When | Example |
|---------|----------|---------|
| **query()** | Autonomous one-shot tasks | "Find bugs in auth.py" |
| **query()** | Predictable, repeatable workflows | Batch processing, pipeline stages |
| **ClaudeSDKClient** | Conversational refinement | User iterating on requirements |
| **ClaudeSDKClient** | Long-running interactive sessions | Chatbots, code review collaboration |
| **ClaudeSDKClient** | Context-dependent follow-ups | "What's the risk of that approach?" |

## Architecture: How ClaudeSDKClient Differs

When you use `query()`, each invocation is isolated:

```
┌─────────────────────────────────────────┐
│ Query 1: "Analyze auth.py"              │
│ - Fresh agent instance                  │
│ - No memory of previous work            │
│ - Agent completes, session ends         │
└─────────────────────────────────────────┘

┌─────────────────────────────────────────┐
│ Query 2: "Review database schema"       │
│ - Completely new session                │
│ - Agent cannot reference Query 1        │
│ - No continuity between tasks           │
└─────────────────────────────────────────┘
```

ClaudeSDKClient maintains a **persistent conversation context**:

```
┌──────────────────────────────────────────────────┐
│  ClaudeSDKClient Session (Persistent)            │
│                                                  │
│  Turn 1: "Analyze auth.py"                      │
│  ↓ Response + Context Stored                    │
│  Turn 2: "What security risks did you find?"    │
│  ↓ Claude reads Turn 1 response, continues      │
│  Turn 3: "Fix those risks"                      │
│  ↓ Claude knows exact risks from Turn 2         │
│                                                  │
│  All turns share accumulated context →          │
│  No information loss between messages           │
└──────────────────────────────────────────────────┘
```

This is critical for:
- **Iterative refinement**: Each turn improves on prior context
- **Constraint accumulation**: Adding requirements progressively
- **Error recovery**: "That approach won't work because [context]. Try this instead."

## Building Multi-Turn Conversations

### Basic Pattern: The Conversation Loop

```python
from claude_agent_sdk import ClaudeSDKClient, ClaudeAgentOptions

async def build_specification_collaboratively():
    """Agent and human iterate to refine requirements."""

    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Bash"],
        permission_mode="acceptEdits"
    )

    async with ClaudeSDKClient(options) as client:
        # Turn 1: Initial requirements
        await client.query("""
        I'm building a user authentication system.
        Initial requirements:
        - Email/password login
        - Session management
        - Password reset via email

        What security patterns should I follow?
        """)

        async for msg in client.receive_response():
            print(msg)

        # Turn 2: Claude's response is stored. Add constraint.
        await client.query("""
        Good suggestions. We also need to support OAuth2 (Google, GitHub).
        How does OAuth2 integration change the architecture?
        """)

        async for msg in client.receive_response():
            print(msg)

        # Turn 3: Claude remembers both prior requirements AND OAuth2
        await client.query("""
        Understood. Now walk me through the database schema for this system.
        What tables do we need? What are foreign key relationships?
        """)

        async for msg in client.receive_response():
            print(msg)
```

**Key observation**: Turn 3 query never mentions email, passwords, or OAuth2. Claude knows these from prior context. You only specify the NEW question.

This is **specification accumulation**—each turn adds one constraint, and Claude maintains the growing specification across all turns.

### Pattern: Interrupts for Long-Running Operations

What if Claude's analysis is taking too long, or you need to stop and resume later?

```python
async with ClaudeSDKClient(options) as client:
    await client.query("Analyze this large codebase for performance issues...")

    try:
        async for msg in client.receive_response():
            # Check for timeout or user interrupt
            if should_interrupt():  # Your interrupt logic
                await client.interrupt()  # Stop gracefully
                break
            print(msg)
    except InterruptedError:
        print("Agent interrupted by user")
        # Session persists - you can resume later
```

The session remains open. You can:
1. Pause analysis with `interrupt()`
2. Make different query without losing context
3. Resume from where you left off

## Streaming Input: Dynamic Message Composition

So far, you've passed complete prompts to `query()` or `ClaudeSDKClient.query()`. But what if your input is **dynamic**? What if you're:
- Reading files and building prompts on-the-fly
- Inserting images at specific points
- Streaming user feedback in real-time

That's where **message generators** come in.

### The Generator Pattern

Instead of:
```python
prompt = "Static string"
await client.query(prompt)
```

You can:
```python
async def message_generator():
    # Dynamically produce message parts
    yield first_part
    yield second_part
    yield third_part

await client.query(prompt=message_generator())
```

Each `yield` sends a message part to Claude as streaming input.

### Example: Building Complex Analysis Requests

Imagine you're analyzing a codebase for refactoring opportunities. You want to:
1. Send the specification
2. Send the current code
3. Send performance metrics
4. Ask Claude to synthesize recommendations

```python
async def analyze_refactoring_request():
    """Generate multi-part analysis request by reading files."""

    # Part 1: Specification
    spec_content = open("spec.md").read()
    yield {
        "type": "user",
        "message": {
            "role": "user",
            "content": f"## Current Specification\n\n{spec_content}"
        }
    }

    # Part 2: Source code
    code_content = open("src/main.py").read()
    yield {
        "type": "user",
        "message": {
            "role": "user",
            "content": f"## Source Code\n\n```python\n{code_content}\n```"
        }
    }

    # Part 3: Performance metrics
    metrics = open("metrics.json").read()
    yield {
        "type": "user",
        "message": {
            "role": "user",
            "content": f"## Performance Profile\n\n{metrics}"
        }
    }

    # Part 4: The actual question
    yield {
        "type": "user",
        "message": {
            "role": "user",
            "content": """
            Based on the specification, code, and performance metrics:
            1. What refactorings would most improve performance?
            2. How would each refactoring change the code structure?
            3. What risks should I be aware of?
            """
        }
    }

# Execute the streaming request
async with ClaudeSDKClient(options) as client:
    async for msg in client.query(prompt=analyze_refactoring_request()):
        print(msg)
```

**Key insight**: Each `yield` is a separate message part. Claude reads them all before responding, but YOU control the composition dynamically. If a file is too large, you can truncate it. If a metric changes, you regenerate it. The generator pattern gives you programmatic control over request assembly.

### Streaming Input with Images

One of the most powerful uses of streaming input: inserting images at specific points in your analysis.

```python
async def analyze_ui_screenshot():
    """Analyze screenshot with contextual code."""

    yield {
        "type": "user",
        "message": {
            "role": "user",
            "content": "## Current UI Implementation\n\nHere's the screenshot:"
        }
    }

    # Insert image from file
    with open("screenshot.png", "rb") as f:
        image_data = base64.b64encode(f.read()).decode()

    yield {
        "type": "user",
        "message": {
            "role": "user",
            "content": [
                {
                    "type": "image",
                    "source": {
                        "type": "base64",
                        "media_type": "image/png",
                        "data": image_data
                    }
                },
                {
                    "type": "text",
                    "text": "And here's the React component:"
                }
            ]
        }
    }

    # Add the code
    code = open("Button.tsx").read()
    yield {
        "type": "user",
        "message": {
            "role": "user",
            "content": f"```tsx\n{code}\n```\n\nAnalyze: Does the UI match the implementation? What mismatches do you see?"
        }
    }

async with ClaudeSDKClient(options) as client:
    async for msg in client.query(prompt=analyze_ui_screenshot()):
        print(msg)
```

This is how you build **visual code review agents** that compare screenshots to implementations, or **UI testing agents** that analyze pixel-level changes.

## Comparing the Three Patterns

| Pattern | Structure | Use Case | Example |
|---------|-----------|----------|---------|
| **query(prompt="text")** | Static string → Single response | One-shot autonomous work | "Find bugs in auth.py" |
| **ClaudeSDKClient.query("text")** | Multi-turn dialogue within session | Iterative refinement | "Analyze... What risks?... Fix those." |
| **query(prompt=generator())** | Dynamically composed streaming input | Complex multi-part requests | Read files → build request → analyze |

**Decision Matrix**:
- Do you need **multiple back-and-forth turns**? → `ClaudeSDKClient`
- Is your **input dynamically generated** (reading files, inserting images)? → Generator pattern
- Is this a **simple one-shot task**? → Basic `query(prompt="text")`

## Practical Patterns for Production Agents

### Pattern 1: Iterative Code Review Collaboration

```python
async def code_review_session(repo_path):
    """Ongoing code review where reviewer and developer iterate."""

    options = ClaudeAgentOptions(
        allowed_tools=["Read", "Bash", "Grep"],
        permission_mode="default"
    )

    async with ClaudeSDKClient(options) as client:
        # Initial review
        code = open(f"{repo_path}/api.py").read()
        await client.query(f"Review this API code for security and performance:\n\n```python\n{code}\n```")

        async for msg in client.receive_response():
            review_feedback = str(msg)

        # Developer addresses feedback
        await client.query("I've updated the code with parameterized queries. Here's the new version: [updated code]. Did this address your security concerns?")

        async for msg in client.receive_response():
            pass

        # Discussion continues with full context
        await client.query("One more question: should we add request rate limiting?")

        async for msg in client.receive_response():
            print(msg)
```

### Pattern 2: Requirements Refinement Conversation

```python
async def refine_requirements():
    """Specification evolves through dialogue."""

    async with ClaudeSDKClient(options) as client:
        # Turn 1: Initial idea
        await client.query("""
        I'm building a Digital FTE that moderates user comments.
        Requirements:
        - Detect offensive language
        - Flag misinformation
        - Consider context (is it satire? criticism vs attack?)
        - Escalate to human if uncertain

        What's missing from my requirements?
        """)

        async for msg in client.receive_response():
            print("Claude's feedback:", msg)

        # Turn 2: Claude identified gaps, now add specifics
        await client.query("""
        Good points about cultural context. We're starting with English-only, US-focused.
        We'll add multi-language support later.

        How should we structure the escalation system? What data should we attach to escalated comments?
        """)

        async for msg in client.receive_response():
            print(msg)

        # Turn 3: Claude understands scope, now architectural depth
        await client.query("""
        We're using PostgreSQL, FastAPI backend, React frontend.
        Walk me through the data schema for this system.
        """)

        async for msg in client.receive_response():
            print(msg)
```

### Pattern 3: Streaming Analysis with Images and Code

```python
async def analyze_ui_bug():
    """Visual bug analysis: screenshot → code → discussion."""

    async def visual_analysis_request():
        # Show the broken UI
        yield {
            "type": "user",
            "message": {
                "role": "user",
                "content": "## Reported Bug\n\nThe button appears shifted on mobile. Here's the screenshot:"
            }
        }

        # Insert screenshot (base64 or file)
        with open("bug-screenshot.png", "rb") as f:
            screenshot_data = base64.b64encode(f.read()).decode()

        yield {
            "type": "user",
            "message": {
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "source": {
                            "type": "base64",
                            "media_type": "image/png",
                            "data": screenshot_data
                        }
                    }
                ]
            }
        }

        # Show the component code
        css = open("styles.css").read()
        yield {
            "type": "user",
            "message": {
                "role": "user",
                "content": f"## CSS\n\n```css\n{css}\n```\n\nWhat's causing the shift? How would you fix it?"
            }
        }

    async with ClaudeSDKClient(options) as client:
        async for msg in client.query(prompt=visual_analysis_request()):
            if hasattr(msg, "content"):
                print("Claude:", msg.content)
```

## When to Use Streaming vs ClaudeSDKClient vs Query

**Choose query() if:**
- Task is autonomous (agent doesn't need to ask questions)
- Input is fixed and complete at call time
- No conversation needed

**Choose ClaudeSDKClient if:**
- You need back-and-forth dialogue
- Context from earlier turns informs later turns
- Workflow is inherently interactive

**Choose streaming generator if:**
- Input is dynamically composed (reading files)
- You're inserting images at specific points
- You need programmatic control over request assembly

**Combine streaming + ClaudeSDKClient if:**
- First turn uses streaming input (complex multi-part request)
- Following turns use context from first turn
- Interactive refinement of initial complex request

Example:
```python
async with ClaudeSDKClient(options) as client:
    # Turn 1: Streaming input (multiple files + question)
    async for msg in client.query(prompt=dynamic_analysis_generator()):
        print(msg)

    # Turn 2: Follow-up using context from Turn 1
    await client.query("Those are good suggestions. Can you estimate the refactoring effort?")
    async for msg in client.receive_response():
        print(msg)
```

This combines **complexity of input composition** (Turn 1) with **context persistence** (Turn 2).

## Try With AI

Setup: You're building a specification assistant—a Digital FTE that helps engineers clarify vague requirements into executable specifications. The conversation should demonstrate iterative refinement where each turn reveals constraints the previous turn didn't address.

**Part 1: Initial Specification Request**

Ask Claude:

```
I'm building a file storage system for a startup.
Users upload files, store them, download them later.
What are all the things I need to think about in the specification?
```

As you read Claude's response, notice what it identifies as gaps in your initial idea: concurrency, permission, retention, file size limits, deletion, etc.

**Part 2: Constraint Addition**

Now tell Claude a constraint based on your domain:

```
We're enterprise SaaS, so security is critical. We also need to support versioning—
users can upload a new version of a file without deleting the old one.

How does versioning change the architecture?
```

Claude will now reference its prior analysis AND adjust for versioning. It doesn't repeat "you need permissions" because it already said that—it builds on it.

**Part 3: Schema Design**

Continue the conversation:

```
Okay, now design the database schema for this system.
```

Claude has all prior context: enterprise-grade security, versioning, permissions. It won't propose naive designs because it remembers the constraints.

**Part 4: Architectural Validation**

Ask Claude to validate the emerging design:

```
In this schema, how would you handle concurrent uploads of the same file?
What race conditions might occur?
```

Claude sees the schema from Part 3, remembers constraints from Parts 1-2, and analyzes concurrency from that context.

**Reflection Questions**

As you work through this conversation:

- In Part 2, did Claude reduce verbosity because it already covered permissions? Why matters: ClaudeSDKClient isn't just about back-and-forth—it's about **progressive complexity without redundancy**.
- In Part 3, was the schema different from what you'd get if you asked for it in isolation? Compare: Run this same schema question in a new `query()` call without prior context. Notice the difference?
- In Part 4, did Claude's analysis reference the schema from Part 3 explicitly? That's context persistence. That's the power of multi-turn.

This entire conversation is an example of **specification accumulation**: each turn adds one requirement, and the agent builds increasingly sophisticated output that respects all prior constraints.

Now build your own: A different domain (content moderation, image processing, financial analysis). Follow this pattern: Vague requirement → Add constraint → Ask for artifact (schema/API/architecture) → Validate design. Watch how context accumulation eliminates repetition and improves output quality.
