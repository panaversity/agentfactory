---
sidebar_position: 40
title: "Chapter 40: FastAPI for Agents"
description: "Expose agent capabilities as production-ready REST APIs with FastAPI"
---

# Chapter 40: FastAPI for Agents

You've built agents with SDKs (Chapter 34), connected them via MCP (Chapters 37-38), and added reusable skills (Chapter 39). Now you'll expose them as HTTP services. FastAPI is the natural choice: Python-native, async-first, automatic OpenAPI documentation, and battle-tested in production.

This chapter consolidates what you learned building MCP servers—HTTP transports, request/response patterns, streaming—into a formal API layer. MCP taught you the concepts; FastAPI gives you the industry-standard framework for production services.

By the end, your multi-agent systems are callable via REST endpoints—triage routing, tool execution, streaming responses—all accessible through a production API.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Create FastAPI applications**: Build HTTP APIs with automatic documentation and validation
- **Implement CRUD operations**: Design RESTful endpoints for Create, Read, Update, Delete
- **Handle errors properly**: Return appropriate HTTP status codes (200, 201, 400, 404, 422)
- **Use dependency injection**: Organize code with FastAPI's Depends() pattern
- **Stream responses**: Implement Server-Sent Events (SSE) for real-time updates
- **Expose multi-agent systems**: Wrap OpenAI Agents SDK agents in endpoints with handoffs
- **Stream agent responses**: SSE with tool calls, handoffs, and completion events

## Chapter Structure

| # | Lesson | Focus |
|---|--------|-------|
| 1 | Hello FastAPI | First app, uvicorn, Swagger UI |
| 2 | POST and Pydantic Models | Request validation, models |
| 3 | Full CRUD Operations | GET, PUT, DELETE patterns |
| 4 | Error Handling | HTTPException, status codes |
| 5 | Dependency Injection | Depends(), repository pattern |
| 6 | Streaming with SSE | Server-Sent Events foundation |
| 7 | Agent Integration | Multi-agent handoffs, triage routing |
| 8 | Capstone: Agent-Powered Task Service | Complete multi-agent API |

## Prerequisites

- **Chapter 34**: OpenAI Agents SDK (Agent, Runner, function_tool, handoffs)
- **Chapters 37-38**: MCP experience (HTTP/SSE patterns familiar)
- **Part 5**: Python Fundamentals (async/await, type hints, Pydantic)

## Looking Ahead

This chapter gives you a REST API exposing full agent capabilities. Chapter 41 (ChatKit Server) builds on this foundation to add conversational infrastructure—the session management, turn-taking, and chat-specific patterns that conversational interfaces require.
