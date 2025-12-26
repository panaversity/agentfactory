---
sidebar_position: 1
title: "MCP Architecture Overview"
chapter: 37
lesson: 1
duration_minutes: 14

# HIDDEN SKILLS METADATA
skills:
  - name: "Understanding MCP Protocol Architecture"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Digital Problem-Solving"
    measurable_at_this_level: "Student can explain the three-tier MCP architecture (Host, Client, Server) and how JSON-RPC 2.0 communication flows between components"

  - name: "Recognizing Tool Fragmentation Problem"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "System Thinking"
    measurable_at_this_level: "Student can articulate why MCP solves the SDK fragmentation problem and when standardization creates value"

  - name: "Evaluating MCP vs. Custom Integration Tradeoffs"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can compare MCP integration vs. custom tool schemas and decide when each approach is appropriate"

  - name: "Identifying the Three Primitives"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Remember"
    digcomp_area: "Digital Literacy"
    measurable_at_this_level: "Student can recognize tools, resources, and prompts as the three capability categories MCP provides"

learning_objectives:
  - objective: "Understand MCP as a universal protocol addressing SDK tool fragmentation (why it exists and what problem it solves)"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of tool schema fragmentation and how MCP standardization solves it"

  - objective: "Grasp the Host-Client-Server architecture and JSON-RPC 2.0 communication model"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Diagram labeling exercise identifying Host, Client, Server roles and message flow"

  - objective: "Recognize the three MCP primitives (tools, resources, prompts) as distinct capabilities"
    proficiency_level: "B1"
    bloom_level: "Remember"
    assessment_method: "Classification exercise: given examples, identify which primitive each represents"

  - objective: "Analyze tradeoffs between MCP integration and custom tool implementations"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Scenario evaluation: given a tool requirement, decide MCP vs. custom implementation"

cognitive_load:
  new_concepts: 7
  assessment: "7 concepts (MCP protocol, Host role, Client role, Server role, JSON-RPC transport, tools primitive, resources+prompts primitives) within B1 limit (7-10 concepts) ✓ - Related concepts chunk together (Host/Client/Server architecture = 1 chunk; three primitives = 1 chunk)"

differentiation:
  extension_for_advanced: "Research AAIF governance changes (December 2025); analyze how moving MCP to Linux Foundation affects adoption vs. vendor-specific standards"
  remedial_for_struggling: "Focus on Host-Client-Server relationships through concrete example: Your IDE (Host) runs MCP Client that talks to GitHub MCP Server. Repeat this mental model for each example"
---

# MCP Architecture Overview

You've built agents with Claude SDK, OpenAI SDK, and Google SDK. Each one forced you to learn different tool schemas, different request patterns, different response handling. You've probably thought: "Why can't there be a standard?"

There is now—and it solves something deeper than just SDK compatibility.

Model Context Protocol (MCP) is the universal standard for connecting AI systems to tools, data, and services. Think of it as USB-C for AI agents. Just as USB-C works the same way across phones, laptops, and peripherals, MCP works the same way across Claude, ChatGPT, Cursor, VS Code, and hundreds of other tools. You define a tool once in MCP format, and every MCP-compatible agent can use it instantly.

Released by Anthropic in November 2024 and adopted by OpenAI in March 2025, MCP is rapidly becoming the de facto standard for agentic AI. When you understand MCP, you're not learning one framework—you're learning the protocol that's unifying the entire ecosystem.

## Why MCP Exists: The Tool Fragmentation Problem

Before MCP, integrating tools into AI systems meant learning three incompatible patterns:

**OpenAI SDK Tool Schema** (from Chapter 34):
```python
{
  "type": "function",
  "function": {
    "name": "get_weather",
    "parameters": {
      "type": "object",
      "properties": {
        "location": {"type": "string"}
      }
    }
  }
}
```

**Anthropic SDK Tool Schema** (from Chapter 35):
```python
{
  "name": "get_weather",
  "description": "Get weather for a location",
  "input_schema": {
    "type": "object",
    "properties": {
      "location": {"type": "string"}
    },
    "required": ["location"]
  }
}
```

**Google SDK Tool Schema** (from Chapter 36):
```python
{
  "name": "get_weather",
  "description": "Get weather for a location",
  "input": {
    "type": "object",
    "properties": {
      "location": {"type": "string"}
    }
  }
}
```

Same tool. Three incompatible schemas. If you want your weather tool to work across all platforms, you're implementing it three times—maintaining three separate definitions, debugging three different behaviors, dealing with three incompatible SDKs.

MCP solves this by providing ONE standardized schema that works everywhere:

```json
{
  "name": "get_weather",
  "description": "Get weather for a location",
  "inputSchema": {
    "type": "object",
    "properties": {
      "location": {"type": "string"}
    },
    "required": ["location"]
  }
}
```

Define once. Use everywhere.

| Aspect | Before MCP | With MCP |
|--------|-----------|---------|
| **Tool Definition** | 1 tool = 3 SDK-specific schemas | 1 tool = 1 MCP schema |
| **Agent Support** | Add support per SDK per tool | Add MCP client once, all tools work |
| **Maintenance** | Update 3 schemas when logic changes | Update 1 schema |
| **Time to New Platform** | Learn new SDK, implement new tool schema | Add MCP client (1 day) |
| **Ecosystem Fragmentation** | High (every SDK reinvents the wheel) | Low (one standard protocol) |

## The Host-Client-Server Architecture

MCP uses a clean three-tier architecture that you need to visualize clearly:

```
┌──────────────────────────────────────────────────────────┐
│                     MCP Host                             │
│  (Your application: Claude Code, IDE, API service)      │
│                                                          │
│  ┌─────────────────────────────────────────────────┐   │
│  │         MCP Client                              │   │
│  │  (Manages server connections, tool routing)     │   │
│  └────────────┬────────────────────────────────────┘   │
│               │ JSON-RPC 2.0 (stdio or HTTP/SSE)       │
│               │                                         │
└───────────────┼─────────────────────────────────────────┘
                │
                │ (Network or local pipe)
                │
        ┌───────▼────────┐
        │  MCP Server    │
        │  (Git, Files,  │
        │   Database)    │
        └────────────────┘
```

### Host: Where Humans Work

The **MCP Host** is your application—Claude Code, Cursor IDE, VS Code, or your custom API service. It's where humans (or other agents) initiate requests.

- Claude Code hosting MCP Client → Users request "connect GitHub"
- VS Code hosting MCP Client → Developers want file search across codebase
- Your API service hosting MCP Client → Backend needs database access

The Host doesn't directly implement tools. It delegates to the Client.

### Client: The Connection Manager

The **MCP Client** is a component INSIDE the Host that manages connections to servers. One Host can run one or more Clients, and each Client manages exactly one Server connection.

**Important distinction**: Not every Agent SDK needs multiple Clients. But every Agent that wants to use MCP needs at least one MCP Client—the lightweight component that speaks JSON-RPC 2.0 to MCP Servers.

The Client:
- Discovers what a server offers (tools, resources, prompts)
- Routes requests from Host to appropriate Server
- Translates Host requests into JSON-RPC format
- Deserializes Server responses back to Host format

### Server: The Tool Provider

The **MCP Server** is a standalone process (Python, Node.js, etc.) that exposes capabilities:

- **Tools**: Executable functions with inputs and outputs
- **Resources**: Files, database records, API data
- **Prompts**: Template prompts that encode domain expertise

MCP Servers are typically simple services. A GitHub MCP Server provides tools for creating issues, fetching PRs, and managing workflows. A Database MCP Server provides tools for querying and updating records. A Filesystem Server provides tools for reading files, searching directories, and listing folders.

Each Server is stateless and decoupled from the Host.

## Communication: JSON-RPC 2.0

All MCP communication uses **JSON-RPC 2.0**—the same lightweight request-response protocol used by Ethereum, Discord, and thousands of other systems.

Example: Host wants to list files in a directory

**Request** (Host → Client → Server):
```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "resources/list",
  "params": {}
}
```

**Response** (Server → Client → Host):
```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "result": {
    "resources": [
      {
        "uri": "file:///path/to/file.py",
        "name": "file.py",
        "mimeType": "text/x-python"
      }
    ]
  }
}
```

JSON-RPC is simple because it has to work across incompatible systems. It's universal—whether your MCP Server runs locally (stdio transport) or remotely (HTTP/SSE), the protocol is identical.

## The Three Primitives: What Servers Provide

Every MCP Server exposes capabilities through three distinct types:

### 1. Tools: Actions the Server Can Execute

Tools are **functions** the Server implements. The Host sends a request; the Server executes and returns results.

Tool schema defines:
- **name**: Unique identifier
- **description**: What it does
- **inputSchema**: JSON Schema for parameters

Example: GitHub Server provides a `create_issue` tool
```json
{
  "name": "create_issue",
  "description": "Create a new GitHub issue",
  "inputSchema": {
    "type": "object",
    "properties": {
      "repo": {"type": "string"},
      "title": {"type": "string"},
      "body": {"type": "string"}
    }
  }
}
```

### 2. Resources: Data the Server Can Access

Resources are **read-only data** exposed by the Server. The Host can list them and fetch their contents.

Resource examples:
- A Filesystem Server provides resources for every file in a directory
- A Database Server provides resources representing tables or query results
- A GitHub Server provides resources for issues, PRs, code comments

You can't modify resources through MCP (that's what tools are for), but you can read them and search them.

### 3. Prompts: Templates Encoding Domain Expertise

Prompts are **pre-written instruction templates** the Server provides. Instead of the Host writing prompts from scratch, it can request a Prompt template from the Server.

Example: A Code Review Server might provide a "security_review" prompt:
```
Review this code for security vulnerabilities.

Code:
[inserted by client]

Checklist:
- SQL injection points
- Authentication bypasses
- Data exposure risks
```

Rather than hardcoding this prompt in your Client, the Server provides it. If the Server's maintainers discover better security questions, they update the prompt once—and every Client using that Server immediately gets the improvement.

## MCP in the Agent Stack: Where It Fits

**Chapter 34-36**: You used SDKs to call tools directly

```
Your Code → SDK Client → Model.chat() → Tool Call → SDK Tool Handler → Response
```

**Chapter 37+ (with MCP)**: You use SDKs to call MCP Servers

```
Your Code → SDK Client → Model.chat() → Tool Call → MCP Client → MCP Server → Response
```

MCP sits **between the Agent and external services**. The Agent doesn't know (or care) that it's using MCP—it just calls tools. The MCP Client translates those calls into MCP format, routes to the appropriate Server, and handles responses.

This is why MCP is powerful: **Your agent code doesn't change.** You add an MCP Client, point it to MCP Servers, and suddenly your agent has access to any service that provides an MCP Server.

## Adoption: The Convergence Signal

**November 2024**: Anthropic releases MCP publicly
**December 2024**: Multiple MCP servers emerge (GitHub, filesystem, databases, Slack)
**March 2025**: OpenAI officially adopts MCP in ChatGPT and Agents SDK
**June 2025**: MCP 2025-06-18 release adds OAuth authorization and structured outputs
**September 2025**: MCP Registry preview launched for server discovery
**December 2025**: Anthropic donates MCP to Agentic AI Foundation under Linux Foundation governance

When your direct competitor (OpenAI) adopts your standard and contributes it to a neutral foundation, you know the protocol isn't niche—it's becoming infrastructure.

## Why This Matters for Digital FTE Production

As you build Digital FTEs (Chapter 40+), you'll need agents that interact with real systems:

- **Accessing your customer's codebase** → MCP Filesystem Server
- **Querying customer databases** → MCP Database Server
- **Creating tickets in their project management** → MCP GitHub/Jira Server
- **Pulling documents from knowledge base** → MCP Resource Server

MCP solves a critical problem: Your Digital FTE can't require customers to learn three different tool integration patterns. It needs to work the same way everywhere. MCP gives you that standardization.

By Chapter 38, you'll build your own MCP Servers. By Chapter 40, you'll compose multiple servers to create integrated workflows. Understanding MCP architecture now is the foundation for that capability.

## Try With AI

Use Claude Code or your AI companion (ChatGPT, Gemini) for these exercises.

### Prompt 1: Visualize the Architecture

**Setup**: You're explaining MCP to a developer friend who's used only OpenAI SDK

**What you're learning**: Distinguishing between architectural layers (Host/Client/Server) and understanding how scaling to multiple capabilities requires multiple Servers, not multiple Clients per Server.

```
I've been using OpenAI's SDK for months, but I keep hearing about
MCP. Help me understand the architecture by answering these questions
about the system below:

System: I want my agent to create GitHub issues using MCP

- Where does the Host fit in?
- Where does the Client fit in?
- Where does the Server fit in?
- If I wanted to also add a filesystem MCP Server, would I need
  multiple Clients or multiple Servers?

Use a simple ASCII diagram showing how requests flow.
```

### Prompt 2: Compare Integration Patterns

**Setup**: Deciding whether to use MCP or implement custom integration

**What you're learning**: Recognizing when standardization creates value (reusability, multi-platform support) versus when custom integration is justified (one-off need, deep framework coupling).

```
I need my agent to access my company's Postgres database.

I could:
1. Implement direct database tools in my agent using the Anthropic SDK
2. Use an MCP Database Server

For each option, walk me through:
- How the agent requests data
- What happens if I want the same agent to work in ChatGPT
- What happens if another developer wants to reuse my database access
- Maintenance burden when the schema changes

Which approach scales better? When would you pick custom integration over MCP?
```

### Prompt 3: Connect to Your Future Work

**Setup**: Looking ahead to when you'll build MCP Servers

**What you're learning**: Understanding why MCP Servers are designed for reusability across multiple clients, not just your one agent. This prepares you for the implementation work ahead.

```
I'm learning MCP now, and I know Chapter 38 teaches me to build my own
MCP Servers. Imagine I'm building a "code analysis" MCP Server that:

- Provides tools for analyzing code quality
- Provides resources showing analysis results
- Provides prompts for common review scenarios

For this server, help me:
1. Define the JSON schema for 2-3 tools (code quality analysis, style checking)
2. Describe what resources this server would expose
3. Explain what domain expertise a prompt from this server would encode

How is this different from just writing these tools directly in my agent?
```

### Safety Note

As you work with MCP, remember: MCP Servers can expose sensitive data (database credentials, private files, API keys). Always review Server implementations carefully before connecting them to production agents. MCP provides the protocol—security depends on correct implementation.

