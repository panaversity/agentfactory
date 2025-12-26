---
sidebar_position: 5
title: "Prompts: The User-Controlled Primitive"
chapter: 37
lesson: 5
duration_minutes: 12

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding MCP Prompt Architecture"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can explain prompt discovery (prompts/list), prompt retrieval (prompts/get), and how prompts encode domain expertise into reusable templates"

  - name: "Designing Domain-Expert Prompts for MCP"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can design prompts with clear arguments, understand static vs dynamic templates, and articulate how prompts capture domain knowledge"

  - name: "Implementing MCP Prompts with FastMCP"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can implement prompts using FastMCP @mcp.prompt() decorator with Field descriptions for arguments"

  - name: "Comparing the Three MCP Control Models"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can distinguish tools (LLM-controlled), resources (app-controlled), and prompts (user-controlled) with appropriate use cases for each"

learning_objectives:
  - objective: "Understand the user-controlled paradigm: Users select when to apply pre-crafted instruction templates"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of how prompts differ from tools and resources in control flow"

  - objective: "Implement prompt discovery (prompts/list) and understand JSON-RPC request/response flow"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Recognition of prompts/list structure and understanding of prompt metadata"

  - objective: "Implement prompt retrieval (prompts/get) with dynamic argument substitution"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Ability to retrieve prompts with arguments and understand template variable substitution"

  - objective: "Design domain-expert prompts that encode tacit knowledge into reusable patterns"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Creation of well-designed prompts with clear purpose, arguments, and educational value"

  - objective: "Compare the three control models (tools, resources, prompts) and select appropriate primitive for use case"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Analysis table matching control models to scenarios"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (user-controlled, discovery, retrieval, static/dynamic templates, argument substitution, domain expertise encoding, control model comparison) within B1 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Implement multi-argument prompts with conditional logic. Design a prompt library for your domain (legal, medical, engineering) that captures expert decision-making patterns. Research how slash commands in chat interfaces use this pattern."
  remedial_for_struggling: "Focus on single static prompt (no arguments) first. Understand prompt/list discovery separately from prompts/get retrieval. Build one simple argument prompt after understanding both operations."
---

# Prompts: The User-Controlled Primitive

You know how Claude Code has slash commands? `/summarize`, `/review`, `/test`—quick prompts that appear when you need them, encoding expertise into one-click workflows.

That's exactly what MCP prompts are. Pre-crafted instruction templates that domain experts create once and users apply whenever needed. Unlike tools (where the AI decides when to use them) and resources (where the app fetches them automatically), **prompts put the user in control: "I want to apply the legal contract review prompt now."**

The insight is subtle but profound: Some of your most valuable expertise isn't code. It's the way you ask questions, structure analysis, or guide thinking. Prompts let you package that expertise and distribute it.

## The User-Controlled Paradigm

In the three MCP primitives you've learned, control flows different directions:

**Tools**: The LLM decides. You provide tools, AI chooses when to call them.

```
You: "What's the weather?"
AI: [decides] → Calls get_weather tool → Returns result
```

**Resources**: The app decides. You fetch data when needed.

```
You: "Show me document X"
App: [decides] → Fetches resource:///documents/X → Returns data
```

**Prompts**: The user decides. You select when to apply an expert-crafted template.

```
You: [consciously chooses] → Applies contract_review_prompt → AI receives structured instruction
```

This isn't subtle. Compare these workflows:

| Primitive | Who Controls? | When Applied? | Use Case |
|-----------|--------------|---------------|----------|
| Tools | LLM decides autonomously | During conversation | Actions (file ops, API calls, calculations) |
| Resources | App fetches automatically | Before/during conversation | Data access (files, databases, APIs) |
| Prompts | User selects explicitly | Before conversation | Domain expertise templates |

Prompts are the way domain experts encode their judgment into systems. A contract lawyer designs the prompt once; clients use it thousands of times, getting expert-quality review every time.

## Prompt Discovery: prompts/list

Before a user can apply a prompt, they need to know what prompts exist. MCP handles this through `prompts/list`.

### The Sequence

```
Client (Claude Code)        Server (Your MCP)
        |                           |
        |-----> prompts/list ------>|
        |                           |
        |<--- Prompt definitions ----|
        |                           |
```

### The Request

The client sends a simple discovery request:

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "prompts/list",
  "params": {}
}
```

No pagination. No filtering. Just: "What prompts do you have?"

### The Response

Your server responds with available prompts:

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "prompts": [
      {
        "name": "contract_review",
        "description": "Legal contract analysis with risk assessment",
        "arguments": [
          {
            "name": "contract_type",
            "description": "Type of contract (NDA, employment, licensing, etc.)",
            "required": true
          }
        ]
      },
      {
        "name": "code_security_audit",
        "description": "Security-focused code review using OWASP principles",
        "arguments": [
          {
            "name": "language",
            "description": "Programming language (python, javascript, go, etc.)",
            "required": true
          },
          {
            "name": "framework",
            "description": "Framework context (fastapi, express, gin, optional)",
            "required": false
          }
        ]
      }
    ]
  }
}
```

Each prompt includes:
- **name**: Unique identifier (snake_case)
- **description**: Human-readable summary
- **arguments**: List of parameters the prompt accepts (optional)

Notice: Unlike tools, prompts don't have input schemas. Arguments are simpler—just name and description. This is intentional. Prompts are instruction templates, not computational functions.

## Prompt Retrieval: prompts/get

Once the user selects a prompt, the client requests the actual prompt text with arguments:

### The Request

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "method": "prompts/get",
  "params": {
    "name": "contract_review",
    "arguments": {
      "contract_type": "NDA"
    }
  }
}
```

The client passes:
- **name**: Which prompt to retrieve
- **arguments**: Values to substitute into the template

### The Response

```json
{
  "jsonrpc": "2.0",
  "id": 2,
  "result": {
    "messages": [
      {
        "role": "user",
        "content": "You are a contract review specialist focusing on NDA agreements.\n\nAnalyze the following NDA for:\n\n1. **Confidentiality Scope**: What information is protected? How broad is the definition?\n\n2. **Term & Termination**: How long does the NDA last? Can it be terminated early?\n\n3. **Return/Destruction**: What happens to information after termination?\n\n4. **Allowed Disclosures**: What exceptions exist (legal requirement, public domain)?\n\n5. **Remedies**: What's the penalty for breach?\n\nProvide risk rating (LOW/MEDIUM/HIGH) for each section."
      }
    ]
  }
}
```

The response is a `messages` array—the exact instructions to give the AI. The server has:

1. Loaded the template
2. Substituted arguments (contract_type = "NDA")
3. Returned the ready-to-use instruction

## Implementing Prompts with FastMCP

Here's how to implement the contract review prompt:

```python
from mcp.server.fastmcp import FastMCP
from pydantic import Field

mcp = FastMCP("legal-assistant")

@mcp.prompt()
def contract_review(
    contract_type: str = Field(
        description="Type of contract (NDA, employment, licensing, etc.)"
    )
) -> str:
    """Legal contract analysis with risk assessment"""

    return f"""You are a contract review specialist focusing on {contract_type} agreements.

Analyze the following {contract_type} for:

1. **Confidentiality Scope**: What information is protected? How broad is the definition?

2. **Term & Termination**: How long does the {contract_type} last? Can it be terminated early?

3. **Return/Destruction**: What happens to information after termination?

4. **Allowed Disclosures**: What exceptions exist (legal requirement, public domain)?

5. **Remedies**: What's the penalty for breach?

Provide risk rating (LOW/MEDIUM/HIGH) for each section."""


@mcp.prompt()
def code_security_audit(
    language: str = Field(description="Programming language (python, javascript, go)"),
    framework: str = Field(
        description="Framework context (fastapi, express, gin, optional)",
        default=None
    )
) -> str:
    """Security-focused code review using OWASP principles"""

    framework_context = f" in {framework}" if framework else ""

    return f"""You are a security engineer reviewing {language} code{framework_context}.

Analyze for OWASP Top 10 vulnerabilities:

1. **Injection**: SQL injection, command injection, template injection
2. **Authentication**: Weak credential handling, session management flaws
3. **Sensitive Data**: Exposed secrets, unencrypted transmission
4. **XML External Entities**: XXE attacks in XML parsing
5. **Broken Access Control**: Authorization bypasses
6. **Misconfiguration**: Security headers, debug flags, unnecessary services
7. **XSS**: Cross-site scripting (reflected, stored, DOM)
8. **Insecure Deserialization**: Object injection risks
9. **Components**: Known vulnerabilities in dependencies
10. **Insufficient Logging**: Missing security event tracking

For each finding, provide:
- Vulnerability class
- Risk level (CRITICAL/HIGH/MEDIUM/LOW)
- Proof of concept
- Remediation steps"""
```

The `@mcp.prompt()` decorator transforms a Python function into a discoverable prompt. Arguments become `Field()` descriptions.

## Static vs Dynamic Prompts

**Static prompts** have no arguments—they're the same every time:

```python
@mcp.prompt()
def daily_standup() -> str:
    """Morning standup meeting template"""
    return """Generate a standup summary covering:

    1. What did you complete yesterday?
    2. What will you complete today?
    3. What blockers exist?"""
```

**Dynamic prompts** accept arguments that customize the template:

```python
@mcp.prompt()
def api_design_review(
    framework: str = Field(description="API framework (REST, GraphQL, gRPC)"),
    context: str = Field(description="Domain context (ecommerce, healthcare, finance)")
) -> str:
    """API design review for specified framework and domain"""
    return f"""Review this {framework} API design for {context} applications.

    Evaluate:
    - Endpoint design patterns
    - Error handling standards
    - Security controls for {context}
    - Versioning strategy
    - Rate limiting approach"""
```

Dynamic prompts are more powerful. They let one template serve thousands of use cases.

## The Three Control Models Summary

Here's how the three MCP primitives compare:

| **Primitive** | **Controller** | **Timing** | **Best For** | **Example** |
|---|---|---|---|---|
| **Tools** | LLM decides | During conversation | Autonomous actions | `save_file()`, `run_test()` |
| **Resources** | App fetches | Before/during conversation | Data access | Database queries, file reads |
| **Prompts** | User selects | Before conversation | Domain expertise | Contract review, security audit |

**How to choose**:

- Something the AI should decide autonomously? → **Tool**
- Data the app should fetch when needed? → **Resource**
- Expert-crafted instruction the user applies? → **Prompt**

A domain expert (lawyer, architect, engineer) creates prompts once. Users apply them thousands of times, getting consistent, expert-quality guidance without needing the expert present.

## Why Prompts Matter

Three reasons prompts are revolutionary:

**1. Expertise Distribution**

A legal expert spends 10 hours designing the contract review prompt. 10,000 non-lawyers use it and get expert-quality reviews. The bottleneck is eliminated.

**2. Consistency**

When the expert is a person, they're inconsistent (tired, busy, ill, on vacation). When the expert is a prompt, every application is identical. Quality is predictable.

**3. Knowledge Preservation**

An expert leaves the company. If their knowledge was in their head, it's gone. If it's in a prompt, it lives on. The organization retains the expertise.

This is why MCP prompts are part of the Agent Factory vision. You build agents that distribute your expertise globally.

## Safety Note

When designing prompts, remember: **The user sees the prompt, but they don't see the implementation.** A security-focused prompt that makes a dangerous recommendation can cause harm. Test prompts thoroughly with edge cases and adversarial inputs before distributing them.

## Try With AI

**Setup**: You'll use Claude Code to explore prompts in your existing MCP servers.

**Prompt 1: Understanding Prompt Discovery**

Ask Claude: "In the learning context, explain how `prompts/list` discovers available prompts in an MCP server. What information does the response include and why?"

**What you're learning**: You're examining the protocol structure for prompt discovery—the request that populates the user interface with available prompts.

**Prompt 2: Designing a Domain-Expert Prompt**

Ask Claude: "I'm a data scientist who wants to create an MCP prompt that guides users through feature engineering decisions. The prompt should accept arguments for data type (categorical, numerical, time-series) and problem context (classification, regression, clustering). Design this prompt with clear questions that guide systematic feature selection."

Record Claude's suggestion. Notice:
- How arguments customize the template
- How the prompt guides thinking (questions vs directives)
- Whether the prompt encodes expert decision-making

**Prompt 3: Comparing Control Models**

Ask Claude: "In MCP, when would you use each primitive? I want to:
1. Let Claude automatically save analysis results (which primitive?)
2. Have Claude analyze a file I select (which primitive?)
3. Apply a legal-review checklist I designed (which primitive?)

For each, explain why that primitive matches the control flow."

**What emerged from iteration**: Through these three interactions, you've seen:
- How prompts differ from tools and resources in control flow
- How to design prompts that encode expertise through questions
- How to select the right primitive for your use case

This is the foundation for designing MCP systems that distribute domain expertise globally.
