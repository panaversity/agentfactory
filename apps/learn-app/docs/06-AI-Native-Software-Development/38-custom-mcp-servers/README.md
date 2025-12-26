---
sidebar_position: 38
title: "Chapter 38: Advanced MCP Server Development"
description: "Master production patterns for MCP servers: Context, Sampling, Progress Notifications, Roots, StreamableHTTP, and horizontal scaling"
---

# Chapter 38: Advanced MCP Server Development

Chapter 37 taught you MCP fundamentals: architecture, primitives, and basic server implementation with @mcp.tool, @mcp.resource, and @mcp.prompt decorators. Now you'll master the **advanced patterns** that separate hobby servers from production-ready systems.

This chapter covers the features that make MCP servers truly powerful: servers that call LLMs through clients (sampling), provide real-time feedback (progress notifications), control file access securely (roots), and scale horizontally for production workloads (StreamableHTTP with stateful/stateless configurations).

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Use the Context object**: Inject Context into tools for logging, progress, and session access
- **Implement sampling**: Have your server call LLMs through connected clients
- **Send notifications**: Provide real-time progress and logging feedback during operations
- **Configure roots**: Control file system access with secure permission boundaries
- **Deploy with StreamableHTTP**: Run production servers with SSE and session management
- **Scale horizontally**: Choose stateful vs stateless configurations for load balancing
- **Handle errors gracefully**: Return structured JSON-RPC errors without crashing
- **Package for distribution**: Create installable servers with pyproject.toml

## Chapter Structure (10 Lessons)

### Layer 2: AI Collaboration (Building with AI)
1. **Context Object & Server Lifespan** — Injection, session access, async resource management
2. **Sampling: Servers Calling LLMs** — ctx.session.create_message(), client callbacks, cost shifting
3. **Progress & Logging Notifications** — report_progress(), info/warning/error, client handling
4. **Roots: File System Permissions** — list_roots, is_path_allowed(), secure path validation
5. **StreamableHTTP Transport** — SSE connections, session IDs, production deployment
6. **Stateful vs Stateless Servers** — stateless_http, json_response, horizontal scaling tradeoffs
7. **Error Handling & Recovery** — JSON-RPC errors, graceful degradation, retry-safe design
8. **Packaging & Distribution** — pyproject.toml, entry points, installation testing

### Layer 4: Spec-Driven Integration
9. **Capstone: Production MCP Server** — Full server with sampling, progress, roots, StreamableHTTP

### Assessment
10. **Chapter Quiz** — Advanced patterns, transport configuration, scaling decisions

## Key Distinction: Chapter 37 vs Chapter 38

| Topic | Chapter 37 (Fundamentals) | Chapter 38 (Advanced) |
|-------|--------------------------|----------------------|
| @mcp.tool basics | ✓ Decorators, schemas | Skip |
| @mcp.resource basics | ✓ URIs, MIME types | Skip |
| @mcp.prompt basics | ✓ Arguments, messages | Skip |
| **Context Object** | — | ✓ Injection, lifespan |
| **Sampling** | — | ✓ Server → Client LLM calls |
| **Notifications** | — | ✓ Progress, logging |
| **Roots** | — | ✓ File permissions |
| **StreamableHTTP** | — | ✓ SSE, sessions |
| **Scaling** | — | ✓ Stateful vs stateless |

## Prerequisites

- **Chapter 37**: MCP Fundamentals (decorators, primitives, basic servers)
- **Part 5**: Python Fundamentals (async/await, decorators, Pydantic)

## Tools Required

- Python 3.11+
- uv package manager
- Node.js + npm (for MCP Inspector)
- Claude Desktop or Claude Code (for testing)

## Chapter Outcome

Students completing this chapter will have:

1. **Production Skills**: Ability to build servers with sampling, progress, and proper error handling
2. **Deployment Knowledge**: Understanding of StreamableHTTP and scaling configurations
3. **Security Awareness**: Roots-based file access control
4. **Digital FTE Component**: A complete, production-ready MCP server with advanced features
