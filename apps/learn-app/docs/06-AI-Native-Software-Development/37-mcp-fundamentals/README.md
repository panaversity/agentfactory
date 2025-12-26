---
sidebar_position: 37
title: "Chapter 37: Model Context Protocol (MCP) Fundamentals"
description: "Master the universal protocol for connecting AI agents to tools, data, and services"
---

# Chapter 37: Model Context Protocol (MCP) Fundamentals

You've built agents with three major SDKs. Each had its own way of defining tools—different schemas, different conventions, different limitations. MCP (Model Context Protocol) solves this fragmentation. It's the USB-C of AI agents: one protocol that works everywhere.

Introduced by Anthropic in November 2024 and rapidly adopted across the industry (OpenAI adopted MCP in March 2025), MCP provides a standard way for AI agents to discover and use tools, access resources, and receive contextual prompts. Claude Code, Cursor, Zed, VS Code, and dozens of other tools already speak MCP. When you add an MCP server to your environment, every MCP-compatible agent gains those capabilities instantly.

This chapter teaches MCP from first principles. You'll understand the protocol architecture, learn to use existing MCP servers effectively, and prepare for Chapter 38 where you'll build your own.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Understand MCP architecture**: Grasp the Host-Client-Server model, transport layers (stdio, Streamable HTTP), and the three primitives (tools, resources, prompts)
- **Configure MCP servers**: Set up MCP servers in Claude Code, Claude Desktop, Cursor, and other clients using JSON configuration
- **Use tools effectively**: Understand tool schemas, invoke tools correctly, and handle tool results
- **Access resources**: Read files, database records, and API data through MCP's resource abstraction
- **Leverage prompts**: Use server-provided prompt templates that encode domain expertise
- **Debug MCP connections**: Diagnose connection issues, trace message flow, and resolve common problems

## Chapter Structure

| # | Lesson | Duration | Description |
|---|--------|----------|-------------|
| 1 | [MCP Architecture Overview](./01-mcp-architecture-overview.md) | 14 min | Protocol design, Host-Client-Server model, and why standardization matters |
| 2 | [Transport Layers](./02-transport-layers.md) | 12 min | stdio for local servers, Streamable HTTP for remote, and when to use each |
| 3 | [Tools: The Model-Controlled Primitive](./03-tools-the-model-controlled-primitive.md) | 14 min | Executable functions that LLMs invoke to perform actions |
| 4 | [Resources: The App-Controlled Primitive](./04-resources-the-app-controlled-primitive.md) | 12 min | Read-only data sources that provide context to AI |
| 5 | [Prompts: The User-Controlled Primitive](./05-prompts-the-user-controlled-primitive.md) | 12 min | Pre-crafted instruction templates encoding domain expertise |
| 6 | [Configuring MCP Clients](./06-configuring-mcp-clients.md) | 14 min | Setup in Claude Code, Claude Desktop, Cursor, VS Code, and programmatic clients |
| 7 | [Using Community MCP Servers](./07-using-community-mcp-servers.md) | 15 min | Filesystem, GitHub, databases, and other popular servers |
| 8 | [Debugging and Troubleshooting](./08-debugging-and-troubleshooting.md) | 12 min | MCP Inspector, connection diagnostics, and common error patterns |
| 9 | [Chapter Quiz](./09-chapter-quiz.md) | 15 min | Test your understanding of MCP fundamentals |

**Total Chapter Duration**: ~2 hours

## Prerequisites

- **Chapters 34-36**: Agent SDK experience (understanding of tool use in OpenAI, Claude, and Google SDKs)
- **Chapter 5**: Claude Code mastery (you've used MCP without knowing it)
- **Part 5**: Python Fundamentals (for understanding server implementations)

## Key Concepts

### The Three Primitives

| Primitive | Controller | Purpose | Example |
|-----------|------------|---------|---------|
| **Tools** | Model-controlled | Perform actions | `github_create_issue`, `read_file` |
| **Resources** | App-controlled | Read data | `docs://documents/{id}`, `db://users` |
| **Prompts** | User-controlled | Instruction templates | `summarize_document`, `code_review` |

### Transport Options

| Transport | Best For | Clients | Complexity |
|-----------|----------|---------|------------|
| **stdio** | Local development, desktop apps | Single | Low |
| **Streamable HTTP** | Production, cloud deployment | Multiple | Medium |

## What's Next

After completing this chapter, you'll be ready for:

- **Chapter 38: MCP Server Development** — Build your own MCP servers to expose your tools and data
- **Chapter 39: Code Execution with MCP** — Execute code safely within MCP servers

## Resources

- [Official MCP Specification](https://modelcontextprotocol.io/specification/2025-06-18)
- [MCP Python SDK](https://github.com/modelcontextprotocol/python-sdk)
- [MCP TypeScript SDK](https://github.com/modelcontextprotocol/typescript-sdk)
- [Community MCP Servers](https://github.com/modelcontextprotocol/servers)
