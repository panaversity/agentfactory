---
sidebar_position: 37
title: "Chapter 37: Model Context Protocol (MCP) Fundamentals"
description: "Master the universal protocol for connecting AI agents to tools, data, and services"
---

# Chapter 37: Model Context Protocol (MCP) Fundamentals

You've built agents with three major SDKs. Each had its own way of defining tools—different schemas, different conventions, different limitations. MCP (Model Context Protocol) solves this fragmentation. It's the USB-C of AI agents: one protocol that works everywhere.

Introduced by Anthropic in late 2024 and rapidly adopted across the industry, MCP provides a standard way for AI agents to discover and use tools, access resources, and receive contextual prompts. Claude Code, Cursor, Zed, and dozens of other tools already speak MCP. When you add an MCP server to your environment, every MCP-compatible agent gains those capabilities instantly.

This chapter teaches MCP from first principles. You'll understand the protocol architecture, learn to use existing MCP servers effectively, and prepare for Chapter 38 where you'll build your own.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Understand MCP architecture**: Grasp the client-server model, transport layers (stdio, HTTP/SSE), and the three capability types (tools, resources, prompts)
- **Configure MCP servers**: Set up MCP servers in Claude Code, Cursor, and other clients using JSON configuration
- **Use tools effectively**: Understand tool schemas, invoke tools correctly, and handle tool results
- **Access resources**: Read files, database records, and API data through MCP's resource abstraction
- **Leverage prompts**: Use server-provided prompt templates that encode domain expertise
- **Debug MCP connections**: Diagnose connection issues, trace message flow, and resolve common problems

## Chapter Structure

1. **MCP Architecture Overview** — Protocol design, client-server model, and why standardization matters
2. **Transport Layers** — stdio for local servers, HTTP/SSE for remote, and when to use each
3. **The Three Capabilities** — Tools (actions), Resources (data), and Prompts (templates) in depth
4. **Configuring MCP Clients** — Setup in Claude Code, Cursor, Zed, and programmatic clients
5. **Using Community MCP Servers** — Filesystem, GitHub, databases, and other popular servers
6. **Debugging & Troubleshooting** — Connection diagnostics, message tracing, and common pitfalls
7. **Capstone: MCP-Powered Workflow** — Configure a complete development environment with multiple MCP servers working together

## Prerequisites

- Chapters 34-36: Agent SDK experience (understanding of tool use)
- Chapter 5: Claude Code mastery (you've used MCP without knowing it)
- Part 5: Python Fundamentals (for understanding server implementations)
