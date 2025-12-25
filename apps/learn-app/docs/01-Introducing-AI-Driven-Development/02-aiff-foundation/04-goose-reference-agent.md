---
sidebar_position: 4
title: "Lesson 4: goose - A Reference Agent"
description: "Understanding goose - Block's open source MCP-native agent framework"
keywords: [goose, Block, MCP, agent framework, reference implementation, open source]
---

# Lesson 4: goose - A Reference Agent

## Learning Objectives

By the end of this lesson, you will be able to:

- Explain what goose is and who created it
- Identify goose's key features
- Understand goose as a reference implementation of MCP
- Connect goose to the General Agents path from the Two Paths Framework

---

## From Abstract to Concrete

We've covered two standards so far:
- **MCP**: The protocol for agent-to-tool connectivity
- **AGENTS.md**: The format for project-specific agent guidance

These are protocols and formats—abstract specifications. But what does a working agent that implements them actually look like?

**goose answers this question.**

goose is Block's open source AI agent framework. It's a complete, production-tested implementation that proves these standards work at enterprise scale.

---

## What is goose?

goose is described as "your local AI agent, automating engineering tasks seamlessly."

Created by Block (the company behind Square, Cash App, and other financial products), goose is:

- **Open source**: Released under Apache License 2.0
- **Local-first**: Runs on your machine, keeping control in your hands
- **MCP-native**: Built on Model Context Protocol from the ground up
- **Extensible**: Connect to any MCP server or API
- **LLM-agnostic**: Works with Claude, GPT, Gemini, or local models

Think of goose as a reference implementation—proof that the AAIF standards work together in a real agent.

---

## Key Features

### Local Execution

goose runs on your machine. This matters for:

- **Privacy**: Your code and data stay local
- **Control**: You decide what the agent can access
- **Speed**: No round-trips to remote servers for local operations

### MCP Integration

goose was one of the first agents to implement MCP. It can connect to any MCP server, giving it access to:

- File systems
- Databases
- APIs
- Custom tools you build

Adding a new capability is as simple as connecting an MCP server.

### Multi-Model Support

goose doesn't lock you into one AI provider. Configure it to use:

- Claude (Anthropic)
- GPT-4/GPT-5 (OpenAI)
- Gemini (Google)
- Local models (Ollama, etc.)

You can even configure different models for different tasks—optimize for cost and performance.

### Autonomous Operation

goose handles complex tasks end-to-end:

- Build entire projects from scratch
- Debug failures across multiple files
- Execute and test code
- Interact with external APIs

It's not just suggesting code—it's executing workflows.

---

## Enterprise Adoption at Block

goose isn't an experiment. It's production software used at scale:

- **Thousands of Block employees** use goose daily
- Available as both **desktop app and CLI**
- Comes with curated, approved MCP servers for enterprise use

The results speak for themselves:

> "Most employees report saving 50–75% of their time on common tasks, and several have shared that work which once took days can now be completed in just a few hours."

This is Digital FTE-level impact—significant time savings on real work.

---

## goose vs Claude Code

In Chapter 1, you learned about the Two Paths Framework:

- **Path A: General Agents** (Claude Code, Gemini CLI, goose)
- **Path B: Custom Agents** (OpenAI SDK, Claude SDK)

Both goose and Claude Code are **General Agents**. They're not competitors—they're validation of the same approach.

| Aspect | Claude Code | goose |
|--------|-------------|-------|
| **Creator** | Anthropic | Block |
| **License** | Proprietary | Open Source (Apache 2.0) |
| **MCP Support** | Yes | Yes |
| **AGENTS.md Support** | Yes | Yes |
| **Default LLM** | Claude | Configurable |

The fact that both exist—and both use MCP and AGENTS.md—proves the standards work. You could use either. You could use both for different purposes.

---

## Why goose Matters for You

goose demonstrates several things important for your Agent Factory journey:

### 1. The Standards Are Real

MCP and AGENTS.md aren't theoretical. Enterprise companies run them in production.

### 2. Open Source Agents Exist

If you want to understand how agents work internally, goose's source code is available. Study it. Learn from it.

### 3. Enterprise-Scale Validation

The patterns you're learning work at Block scale. What works for thousands of developers will work for you.

### 4. Reference Architecture

When you build Custom Agents later (Path B), goose shows what good General Agent architecture looks like.

### 5. Your Agent Factory Blueprint

As you progress through this book, you'll transition from using General Agents (Path A) to building Custom Agents (Path B). goose demonstrates the architectural patterns you'll apply:

- **MCP integration**: How production agents connect to external tools
- **Local-first execution**: Why privacy and control matter for client deployments
- **Multi-model support**: How to avoid vendor lock-in in your products
- **Extensibility**: How to design agents that customers can customize

Block's engineers use goose daily. Your clients will use YOUR agents daily if you apply the same architectural principles. goose is the blueprint.

---

## goose in the AAIF Context

goose is an AAIF founding project alongside MCP and AGENTS.md. This matters because:

- Block contributed goose to neutral governance
- The project will evolve with community input
- Standards integration will deepen over time

goose isn't just Block's tool anymore—it's part of the open agentic AI ecosystem.

---

## Quick Knowledge Check

Before moving on, make sure you can answer:

1. What is goose and who created it?
2. What are goose's key features?
3. How does goose relate to Claude Code in the Two Paths Framework?
4. Why is goose being an AAIF project significant?

---

## Try With AI

Ask your AI assistant:

> "Compare goose and Claude Code as AI agents"

Evaluate the response:
- Does it correctly identify both as General Agents?
- Does it mention they both use MCP?
- Does it note goose is open source while Claude Code is Anthropic's product?

---

## Summary

- **goose** is Block's open source AI agent framework
- **Key features**: Local-first, MCP-native, extensible, any LLM
- **Enterprise tested**: Thousands of Block employees, 50-75% time savings
- **General Agent**: Same path as Claude Code in the Two Paths Framework
- **AAIF founding project**: Contributed to neutral governance under Linux Foundation
- **Reference implementation**: Proves AAIF standards work at scale

---

## Next Steps

You've seen what a real agent looks like. Now let's learn how to package domain expertise so your agents—and Digital FTEs—can use it. [Continue to Lesson 5: Agent Skills - Packaging Expertise](./05-agent-skills-packaging-expertise.md).
