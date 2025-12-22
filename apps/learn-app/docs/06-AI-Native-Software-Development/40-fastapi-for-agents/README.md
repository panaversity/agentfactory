---
sidebar_position: 40
title: "Chapter 40: FastAPI for Agents"
description: "Expose agent capabilities as production-ready REST APIs with FastAPI"
---

# Chapter 40: FastAPI for Agents

You've built agents with SDKs, connected them via MCP, and added skills with code execution. Now you'll expose them as HTTP services. FastAPI is the natural choice: Python-native, async-first, automatic OpenAPI documentation, and battle-tested in production.

This chapter consolidates what you learned building MCP servers—HTTP transports, request/response patterns, streaming—into a formal API layer. MCP taught you the concepts; FastAPI gives you the industry-standard framework for production services.

By the end, your agents are callable via REST endpoints—ready for integration into any application, or as the foundation for ChatKit Server in Chapter 41.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Structure agent APIs**: Design RESTful endpoints that expose agent capabilities cleanly
- **Handle agent requests**: Process incoming requests, validate inputs with Pydantic, invoke agents appropriately
- **Stream agent responses**: Implement Server-Sent Events (SSE) for real-time agent output
- **Manage conversation state**: Handle sessions, context persistence, and stateful interactions
- **Process long-running tasks**: Use background tasks for operations that exceed HTTP timeouts
- **Document agent APIs**: Generate OpenAPI specs that describe agent capabilities for consumers
- **Secure agent endpoints**: Implement authentication, rate limiting, and input validation

## Chapter Structure

1. **FastAPI Fundamentals** — Project structure, async handlers, dependency injection, and Pydantic models
2. **Agent Endpoint Design** — RESTful patterns for agent invocation, conversation management, and tool access
3. **Streaming Responses** — SSE implementation, chunked responses, and real-time agent output
4. **State & Session Management** — Conversation context, session stores, and maintaining agent memory across requests
5. **Background Processing** — Long-running agents, task queues, webhooks, and polling patterns
6. **API Documentation** — OpenAPI generation, schema customization, and describing agent capabilities
7. **Security Patterns** — API key authentication, rate limiting, input sanitization, and error handling
8. **Capstone: Agent API Service** — Spec-driven implementation of a complete agent API with streaming, sessions, and documentation

## Prerequisites

- Chapters 34-36: Agent SDK experience (agents to expose)
- Chapters 37-38: MCP experience (HTTP/SSE patterns already familiar)
- Part 5: Python Fundamentals (async/await, type hints, Pydantic)

## Looking Ahead

This chapter gives you a REST API. Chapter 41 (ChatKit Server) builds on this foundation to add conversational infrastructure—the streaming, session management, and turn-taking that chat interfaces require.
