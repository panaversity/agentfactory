---
sidebar_position: 5
title: "Implementing Prompts (@mcp.prompt)"
description: "Create reusable prompt templates that encode domain expertise and guide AI agent behavior"
keywords: ["MCP", "prompts", "@mcp.prompt", "prompt templates", "UserMessage", "AssistantMessage", "prompt parameters", "expertise encoding"]
chapter: 38
lesson: 5
duration_minutes: 55

# HIDDEN SKILLS METADATA
skills:
  - name: "MCP Prompt Decorator Implementation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can implement @mcp.prompt decorators with parameters and message structures"

  - name: "Prompt Message Structures (UserMessage/AssistantMessage)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student understands message role semantics and when to use each type"

  - name: "Parameterized Prompt Templates"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can create prompts with dynamic parameters using Pydantic Field annotations"

  - name: "Domain Expertise Encoding in Prompts"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Evaluate"
    digcomp_area: "AI & Agents"
    measurable_at_this_level: "Student can identify which domain knowledge belongs in prompt templates vs tool documentation"

  - name: "Prompt vs Tool Design Decisions"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "AI & Agents"
    measurable_at_this_level: "Student can distinguish when to expose functionality via tools vs prompts"

  - name: "Prompt Composition and Chaining"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "AI & Agents"
    measurable_at_this_level: "Student can combine multiple prompts or nest prompts in sequences"

  - name: "Prompt Testing and Validation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Testing & Validation"
    measurable_at_this_level: "Student can test prompt behavior with MCP Inspector and verify output quality"

learning_objectives:
  - objective: "Implement @mcp.prompt decorator with proper message structures and parameter handling"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates working prompt with UserMessage/AssistantMessage and validates via MCP Inspector"

  - objective: "Encode domain expertise into parameterized prompt templates that guide AI behavior"
    proficiency_level: "B1"
    bloom_level: "Evaluate"
    assessment_method: "Student compares multiple prompt approaches and justifies expertise encoding choices"

  - objective: "Distinguish when to use prompts vs tools for different MCP server capabilities"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Student categorizes 5+ capabilities as prompt-appropriate or tool-appropriate with reasoning"

  - objective: "Test prompts using MCP Inspector and identify prompt quality issues"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student uses MCP Inspector to test prompt and adjusts based on output quality"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (@mcp.prompt decorator, UserMessage/AssistantMessage types, Pydantic Field parameters, prompt arguments, message roles, expertise encoding, prompt vs tool tradeoffs) within B1 range (7-10)"

differentiation:
  extension_for_advanced: "Create multi-turn prompts with assistant messages; implement dynamic prompt composition based on context; design prompt templates that adapt to different user expertise levels"
  remedial_for_struggling: "Start with single-argument UserMessage-only prompts before adding parameters; use MCP Inspector to inspect message structure; compare to provided examples before implementing original prompts"
---

# Implementing Prompts (@mcp.prompt)

You've built tools that expose *operations*—functions agents can call to execute tasks. Tools are how agents *do* things. But agents also need *guidance*—structured instructions that help them reason about complex workflows without requiring human intervention every time.

This is where prompts come in.

In Chapter 37, you learned that MCP prompts are pre-crafted templates that encode domain expertise. They're user-controlled (users decide when to apply them), and they can include dynamic parameters for flexibility. Now you'll implement them.

The key insight separates prompts from tools: **Tools expose operations. Prompts encode expertise.** A tool does something specific. A prompt teaches an agent how to do something well.

## The Prompt Design Philosophy

Before implementing, understand the philosophy underlying MCP prompts.

### Why Prompts Matter

Consider two approaches:

**Without prompts**, a user asks Claude directly: "Help me review this Python code for security issues."

Claude will attempt a code review, but the quality depends on:
- Claude's default code review knowledge (trained on many examples, but no specialized guidance)
- The user's ability to explain what "good" looks like
- Whether edge cases are covered

**With a prompt**, you—as the MCP server author—have already spent time crafting a thorough code review template. You've tested it. You know it handles Python-specific security patterns, identifies common vulnerabilities, and asks the right follow-up questions.

Users benefit from your expertise without becoming prompt engineering experts themselves.

### Prompts vs Tools: A Clear Distinction

This distinction is critical:

| Aspect | Tool | Prompt |
|--------|------|--------|
| **Purpose** | Execute a specific operation | Provide guidance for reasoning |
| **Control** | Agent decides when to invoke | User decides when to apply |
| **Invocation** | Automatic in agent workflow | Explicit user request |
| **State Changes** | Modifies system state (creates ticket, updates record) | No side effects |
| **Parameters** | Define operation inputs | Customize guidance context |
| **Examples** | Create issue, search database, send email | Code review framework, writing guide, architectural decision framework |

**Decision framework**: If the capability changes system state → Tool. If it provides guidance without changing anything → Prompt.

## Core Concepts: Message Structures

Prompts return *message structures*, not raw text. This matters because it enables multi-turn conversations and role-based reasoning.

### UserMessage and AssistantMessage Types

MCP defines two message types that together form a conversation context:

**UserMessage**: Represents user instructions or requests. This is where you put the actual prompt instructions.

```python
from mcp.types import UserMessage

message = UserMessage(
    content="Review this code for security vulnerabilities: [code here]"
)
```

**AssistantMessage**: Represents assistant responses or system instructions that guide the model. Use these to prime the assistant with a specific reasoning approach.

```python
from mcp.types import AssistantMessage

message = AssistantMessage(
    content="I'll analyze this code systematically: First, I'll identify potential vulnerabilities..."
)
```

Why both? Because conversations are contextual. A UserMessage alone says "here's a task." A UserMessage followed by an AssistantMessage says "here's a task, and here's how I'll approach it." The assistant message primes reasoning without being user input.

**Output:**

```python
# Messages returned by prompt:
[
    UserMessage(content="Code to review:\n..."),
    AssistantMessage(content="I'll check for: 1) SQL injection, 2) Path traversal, 3) Insecure deserialization...")
]
```

## Implementing Your First Prompt

### The @mcp.prompt Decorator

The `@mcp.prompt()` decorator registers a function as an MCP prompt. It mirrors the `@mcp.tool()` syntax:

```python
from mcp.server.fastmcp import FastMCP
from mcp.types import UserMessage, AssistantMessage

mcp = FastMCP("my_server")

@mcp.prompt()
def code_review() -> list:
    """Provide a code review framework."""
    return [
        UserMessage(content="Review the provided code for issues."),
        AssistantMessage(content="I'll analyze for functionality, performance, and security.")
    ]
```

**What happens**:

1. FastMCP extracts the prompt name from the function name (`code_review`)
2. Extracts the description from the docstring
3. Registers the prompt so clients can discover it via `prompts/list`
4. When a user requests the prompt, FastMCP calls your function and returns the messages

**Output from MCP Inspector:**

```
Prompt: code_review
Description: Provide a code review framework.
Messages: 2 (UserMessage + AssistantMessage)
```

### Adding Parameters

Prompts become powerful when they accept parameters. This lets users customize the template for their specific context.

```python
from pydantic import Field

@mcp.prompt()
def code_review(language: str = Field(description="Programming language of the code")) -> list:
    """Review code in the specified language for quality issues."""
    return [
        UserMessage(
            content=f"Review this {language} code for issues:\n\n[CODE HERE]"
        ),
        AssistantMessage(
            content=f"I'll analyze this {language} code for: 1) Language-specific best practices, 2) Common {language} pitfalls, 3) Performance patterns."
        )
    ]
```

**Now users can customize**: The same prompt adapts to Python, Rust, Go, etc. by passing the language parameter.

**Parameter rules**:
- Use Pydantic `Field()` for descriptions
- Mark required parameters with `...` (ellipsis)
- Mark optional parameters with `default=` values
- Include type hints so FastMCP can validate inputs

## Encoding Domain Expertise in Prompts

This is where prompts become genuinely valuable—they let you encode knowledge that might take users hours to discover.

### Example: Legal Document Analysis

Imagine you're building an MCP server for legal professionals. You could expose a tool that "analyzes documents." But lawyers have very specific frameworks for analyzing different document types. A contract review is different from a patent analysis, which differs from regulatory compliance review.

Without prompts, lawyers would write the same detailed instructions every time. With prompts, you encode the expertise:

```python
@mcp.prompt()
def contract_analysis(
    contract_type: str = Field(description="Type: NDA, SLA, Employment, License, etc."),
    focus_areas: str = Field(description="Specific concerns: liability, payment terms, duration, etc.")
) -> list:
    """Analyze a contract using legal best practices."""

    expertise = {
        "NDA": "Check: definition of confidential info, term length, permitted disclosures, remedies for breach",
        "SLA": "Check: service levels, uptime guarantees, penalties, change management, escalation procedures",
        "Employment": "Check: at-will provisions, non-compete scope, IP ownership, confidentiality obligations",
    }

    focus_guidance = f"Priority areas: {focus_areas}"
    analysis_framework = expertise.get(contract_type, "General contract analysis")

    return [
        UserMessage(
            content=f"Analyze this {contract_type.upper()} contract.\n\n{focus_guidance}"
        ),
        AssistantMessage(
            content=f"""I'll review this {contract_type} systematically:

1. **Framework**: {analysis_framework}
2. **Standard practice**: Compare against typical market terms
3. **Risk areas**: Identify unusual or unfavorable provisions
4. **Recommendations**: Suggest modifications to protect your interests

Let me start with the analysis."""
        )
    ]
```

**What's happening**:
- You're encoding legal expertise (what matters in each contract type)
- You're structuring the reasoning approach (systematic framework)
- You're providing templates that lawyers can reuse instead of typing detailed instructions

This is how prompts add value—domain expertise packaged as reusable templates.

### When Expertise Belongs in Prompts

Not all expertise belongs in prompts. Some belongs in tools, documentation, or configuration. Ask:

1. **Is this a decision framework or a specific operation?**
   - Framework (how to analyze) → Prompt
   - Operation (extract email from contact list) → Tool

2. **Will users want to customize the approach?**
   - Yes → Prompt with parameters
   - No → Tool with fixed behavior

3. **Does this provide ongoing guidance or one-time execution?**
   - Ongoing (code review framework for many reviews) → Prompt
   - One-time (get user by ID) → Tool

## Advanced Prompt Patterns

### Multi-Turn Conversations

Prompts can include multiple UserMessages and AssistantMessages to set up a conversation context:

```python
@mcp.prompt()
def architectural_decision() -> list:
    """Guide architectural decision-making for a system design."""
    return [
        UserMessage(
            content="Help me think through this architectural decision."
        ),
        AssistantMessage(
            content="I'll help you work through this systematically. Let's start by clarifying the constraints and tradeoffs."
        ),
        UserMessage(
            content="Here's what I'm trying to build: [ARCHITECTURE CONTEXT]"
        ),
        AssistantMessage(
            content="""Now I'll analyze the options:

1. **Option A advantages**: [...]
2. **Option A tradeoffs**: [...]
3. **Option B advantages**: [...]
4. **Option B tradeoffs**: [...]

What constraints matter most to your situation?"""
        )
    ]
```

This creates a conversation context where the assistant has already acknowledged the task and begun structured analysis. When the user applies this prompt, the assistant picks up that reasoning thread.

### Dynamic Content with Conditionals

Prompts can adapt based on parameters:

```python
@mcp.prompt()
def code_quality_gate(
    framework: str = Field(description="Testing framework: pytest, unittest, jest, etc."),
    strictness: str = Field(description="Level: permissive, standard, strict")
) -> list:
    """Establish code quality criteria for review."""

    strictness_levels = {
        "permissive": "coverage >= 60%, cyclomatic complexity <= 15",
        "standard": "coverage >= 75%, cyclomatic complexity <= 10, no deprecated patterns",
        "strict": "coverage >= 90%, cyclomatic complexity <= 7, full type hints required"
    }

    criteria = strictness_levels.get(strictness, strictness_levels["standard"])

    return [
        UserMessage(
            content=f"""Evaluate this code against our {framework} quality gate:

**Criteria ({strictness} level):**
{criteria}

**Check**:
1. Test coverage
2. Complexity metrics
3. Pattern adherence
4. Type safety
5. Documentation quality"""
        ),
        AssistantMessage(
            content=f"I'll check this {framework} code against {strictness}-level criteria systematically."
        )
    ]
```

Now the same prompt adapts: A "strict" code review is different from a "permissive" one, and the criteria adjust based on the testing framework.

## Try With AI: Prompts in Practice

Work through three prompts that move from simple to domain-specific:

### Part 1: Simple Prompt

Ask AI to design a basic code review prompt:

"Create an @mcp.prompt that provides a code review framework. The prompt should:
- Accept a programming language parameter
- Return UserMessage with the code and a request to review it
- Return AssistantMessage that explains the review approach for that language
- Include specific areas to check (style, performance, security)

Write the complete @mcp.prompt implementation."

**What you're learning**: How to structure basic prompts with parameters and message roles.

### Part 2: Refine with Your Constraints

Now add your expertise:

"Looking at the code review prompt, I want to specialize it for our team. We:
- Work primarily with Python and Go
- Care most about security vulnerabilities
- Want the prompt to prioritize common mistakes in our domain (database queries, API responses)

Update the prompt to encode this expertise. Add specific checks for our patterns."

**What emerges**: The prompt now reflects your team's actual priorities, not generic advice. You've taught AI your constraints, and AI refined the template accordingly.

### Part 3: Create a Domain-Specific Prompt

Design an original prompt for your domain:

"I need a prompt for [your domain/profession]. The prompt should:
- Guide users through [your specific workflow]
- Encode 3-5 critical checks or decision points
- Accept 2-3 parameters that customize the approach
- Include both UserMessage and AssistantMessage components

Create the full @mcp.prompt implementation."

**What you're learning**: How to identify what knowledge belongs in a prompt, and how to structure that knowledge for reuse.

---

## Testing Prompts with MCP Inspector

After implementing prompts, validate them with MCP Inspector:

**Verification Checklist**:

1. Prompt appears in `prompts/list` ✓
2. Parameters show correct types and descriptions ✓
3. When you request the prompt, messages return correctly ✓
4. Message content is clear and actionable ✓
5. Parameters are actually used in message content ✓

**Common issues**:
- Missing docstring → No description appears
- Parameters not used in messages → Parameters seem pointless
- Message content too generic → Prompt adds no value over direct user request
- Missing parameter descriptions → Users can't understand what to pass

## Summary of Concepts

| Concept | Purpose |
|---------|---------|
| **@mcp.prompt** | Decorator that registers a function as a prompt template |
| **UserMessage** | Message type representing user/system instructions |
| **AssistantMessage** | Message type priming assistant reasoning approach |
| **Prompt Parameters** | Dynamic values (Pydantic Field) that customize templates |
| **Expertise Encoding** | Domain knowledge embedded in prompt text and structure |
| **Prompt vs Tool** | Tools = operations, Prompts = guidance frameworks |
| **Message Composition** | Multiple messages creating conversation context |

## Try With AI

**Setup**: Your MCP server from previous lessons, with at least one tool implemented.

**Prompt 1: Learn Prompt Structure**

"I have an MCP server that [describe your server's purpose]. Design a @mcp.prompt that:
- Guides users through a common workflow with this server
- Takes 2 parameters that customize the approach
- Includes both UserMessage (user instructions) and AssistantMessage (AI reasoning guidance)
- Encodes domain expertise for [your domain]

Write the complete implementation. Explain why this expertise belongs in a prompt vs a tool."

**Expected**: Working prompt code that you can add to your server.

**Prompt 2: Refine Based on Your Knowledge**

Review the prompt AI generated. Now: "This prompt is good, but our users actually care about [specific concern]. Update it to prioritize [your actual requirement]. Show me the updated prompt code."

**What emerges**: Through your feedback, the prompt becomes more specialized to your actual needs. You're teaching AI domain constraints, and it adapts the template.

**Prompt 3: Validate and Test**

"I've added this prompt to my MCP server. How do I test it with MCP Inspector? Walk me through the steps to verify:
1. The prompt appears in the prompt list
2. Parameters work correctly
3. Messages are formatted properly
4. The prompt is actually useful

What would indicate that the prompt needs improvement?"

**What you're learning**: How to validate that prompts work as intended and identify quality issues before users rely on them.

---

## Safety Consideration

Prompts contain instructions for AI reasoning. If prompts include sensitive information (API keys, customer data, authentication tokens), anyone with access to your MCP server can see them. Keep sensitive data out of prompt text—use separate secure tools or environment variables instead.

