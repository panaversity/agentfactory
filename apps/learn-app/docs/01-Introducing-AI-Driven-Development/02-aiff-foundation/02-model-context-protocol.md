---
sidebar_position: 2
title: "Lesson 2: MCP - Connecting Agents to Tools"
description: "Understanding the Model Context Protocol - how AI agents connect to external tools and data"
keywords: [MCP, Model Context Protocol, Resources, Tools, Prompts, agent connectivity]
---

# Lesson 2: MCP - Connecting Agents to Tools

## Learning Objectives

By the end of this lesson, you will be able to:

- Explain what problem MCP solves
- Identify the three MCP primitives: Resources, Tools, and Prompts
- Describe MCP architecture: Host → Client → Server
- Connect MCP to Digital FTE "Act" power

---

## The Problem: Fragmented Connections

Before MCP, connecting AI agents to external tools was chaos.

Want your Claude agent to access your company's CRM? Write custom integration code. Want the same for ChatGPT? Write different code. For Gemini? Start over again.

Every AI platform had its own approach. Every tool connection required platform-specific work. If you switched AI providers, you'd rewrite everything.

```
Before MCP:

Agent A ──── Custom Code A ──── Salesforce
Agent B ──── Custom Code B ──── Salesforce
Agent C ──── Custom Code C ──── Salesforce
```

This is the fragmentation problem. And it's exactly what MCP solves.

---

## The USB Analogy

Think about USB again (we used this analogy in Lesson 1, and it's worth extending).

Before USB:
- Every device had proprietary connectors
- Cables weren't interchangeable
- Switching devices meant buying new cables

After USB:
- One standard connector
- Any device works with any port
- Buy once, use everywhere

**MCP is USB for AI agents.**

```
After MCP:

Agent A ─┐
Agent B ─┼── MCP ──── Any Tool
Agent C ─┘
```

One protocol. Universal connectivity. Write an MCP server once, and any MCP-compatible agent can use it.

---

## Three Primitives: The Building Blocks

MCP defines three types of capabilities that servers can expose to agents:

### 1. Resources — Data to Read

Resources are things your agent can **read**. They provide context and information.

Examples:
- File contents from a document repository
- Database records from your CRM
- API responses from external services
- Configuration settings

Think of Resources as the agent's **eyes**—what it can see and read.

### 2. Tools — Functions to Call

Tools are things your agent can **do**. They execute actions and produce results.

Examples:
- Search for information
- Create a new record
- Send an email
- Run a calculation

Think of Tools as the agent's **hands**—what it can act on.

### 3. Prompts — Templates for Interaction

Prompts are reusable message templates that guide common interactions.

Examples:
- A template for summarizing documents
- A workflow for analyzing code
- A structured format for generating reports

Think of Prompts as the agent's **playbook**—pre-configured interaction patterns.

---

## The Architecture: Host → Client → Server

MCP uses a three-tier architecture:

```
┌──────────────────┐
│      Host        │  ← The LLM application (Claude Desktop, ChatGPT)
│  ┌────────────┐  │
│  │   Client   │  │  ← Connector that manages MCP connections
│  └────────────┘  │
└────────┬─────────┘
         │
         │ MCP Protocol
         │
┌────────▼─────────┐
│     Server       │  ← External service providing capabilities
│  (MCP Server)    │
└──────────────────┘
```

**Host**: The LLM application you're using. Claude Desktop is a Host. ChatGPT is a Host. Your custom agent application would be a Host.

**Client**: Lives inside the Host. Manages connections to MCP servers, handles the protocol communication, routes requests and responses.

**Server**: Provides capabilities (Resources, Tools, Prompts) to the agent. This is what you or others build to expose functionality.

When you hear "MCP server," think: "A service that exposes capabilities to AI agents."

---

## Transport: How Messages Travel

MCP supports two transport mechanisms:

### stdio — For Local Tools

Used when the MCP server runs on your own machine. Communication happens through standard input/output streams.

Example: A file system MCP server that lets your agent read local files.

### HTTP with SSE — For Remote Services

Used when the MCP server runs on a remote system. Uses HTTP for requests and Server-Sent Events for streaming responses.

Example: A cloud-based database MCP server your team shares.

You don't need to implement these yourself—the MCP SDKs handle transport. But understanding the distinction helps: **local = stdio, remote = HTTP/SSE**.

---

## A Brief History

- **November 2024**: Anthropic releases MCP as open source
- **March 2025**: OpenAI officially adopts MCP across its products
- **November 2025**: MCP specification 2025-11-25 released with significant updates
- **December 2025**: MCP donated to AAIF under Linux Foundation governance

As Mike Krieger, Chief Product Officer at Anthropic, stated:

> "When we open sourced it in November 2024, we hoped other developers would find it as useful as we did. A year later, it's become the industry standard for connecting AI systems to data and tools."

MCP went from one company's internal tool to industry standard in about a year. That's remarkably fast adoption.

---

## Connection to Digital FTEs

Remember the Five Powers from Chapter 1? Every Digital FTE combines:

1. **See**: Visual understanding
2. **Hear**: Audio processing
3. **Reason**: Multi-step planning
4. **Act**: Execute code, call APIs, operate tools
5. **Remember**: Maintain context

**MCP enables the "Act" power.**

Without MCP, your Digital FTE can reason about what to do but can't actually do it. With MCP, your FTE connects to:

- CRM systems to update customer records
- Databases to query and modify data
- APIs to integrate with any service
- File systems to read and write documents

The more MCP servers you connect, the more capable your Digital FTE becomes.

---

## What You'll See in Later Chapters

In Part 2 (AI Tool Landscape), you'll encounter MCP commands like:

```bash
# Adding an MCP server to Gemini CLI
gemini mcp add filesystem

# Listing connected MCP servers
claude mcp list
```

When you see these commands, you'll understand: **you're connecting your agent to external capabilities via MCP**.

---

## Quick Knowledge Check

Before moving on, make sure you can answer:

1. What problem does MCP solve?
2. What are the three MCP primitives and what does each one do?
3. What's the difference between a Host, Client, and Server in MCP?
4. How does MCP enable the "Act" power in Digital FTEs?

---

## Try With AI

Ask your AI assistant:

> "Explain MCP like I'm 10 years old"

Then ask:

> "What's the difference between MCP Resources and Tools?"

Evaluate whether the AI correctly distinguishes between data you can read (Resources) and actions you can take (Tools).

---

## Summary

- **MCP** (Model Context Protocol) is the universal standard for connecting AI agents to tools
- **Three primitives**: Resources (data to read), Tools (functions to call), Prompts (templates)
- **Architecture**: Host (LLM app) → Client (connector) → Server (capability provider)
- **Transport**: stdio for local, HTTP/SSE for remote
- **History**: Anthropic created it (2024), industry adopted it, Linux Foundation governs it (2025)
- **Digital FTE connection**: MCP enables the "Act" power—without it, agents can only think, not do

---

## Next Steps

Now you understand how agents connect to tools. But how do agents know the specific conventions of a project they're working on? That's where AGENTS.md comes in. [Continue to Lesson 3: AGENTS.md - Project Context for Agents](./03-agents-md-project-context.md).
